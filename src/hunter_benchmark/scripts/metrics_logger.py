#!/usr/bin/env python3
"""Live metrics_logger node.

Subscribes to /trajectory/reference (for total_length_m/is_closed_loop),
/tracking/diagnostics (for lateral/heading/speed error and steering
stats), and /ackermann_like_controller/odometry (for distance traveled),
using the shared `stats` module so a bag reprocessed later via
summarize_run.py reproduces the exact same summary.csv. Bag recording
itself is handled by a separate `ros2 bag record` process launched
alongside this node (see benchmark.launch.py) rather than programmatic
rosbag2_py writing -- this was already the assumption baked into
docker-compose.yml's `:rw` mount comment on src/.
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from hunter_msgs.msg import Trajectory, TrackingDiagnostics
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stats  # noqa: E402  (installed alongside this script -- see CMakeLists.txt)


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('track_name', 'unknown_track')
        self.declare_parameter('controller_name', 'unknown_controller')
        self.declare_parameter('results_dir', os.path.expanduser('~/hunter_benchmark_results'))
        self.declare_parameter('reversal_deadband_rad', 0.02)
        self.declare_parameter('summary_write_period_s', 5.0)
        self.declare_parameter('pose_source', 'unknown')
        # Rigid transform taking a ground-truth pose into the reference path's
        # frame. Gazebo's OdometryPublisher reports the model's WORLD pose,
        # while trajectory_server may have transformed the path into a
        # spawn-relative frame (odom/map) -- these params undo that so the two
        # are comparable. Set from benchmark.launch.py; identity means ground
        # truth and the path already share a frame.
        self.declare_parameter('gt_origin_x_m', 0.0)
        self.declare_parameter('gt_origin_y_m', 0.0)
        self.declare_parameter('gt_origin_yaw_rad', 0.0)

        self._track_name = self.get_parameter('track_name').value
        self._controller_name = self.get_parameter('controller_name').value
        self._results_dir = self.get_parameter('results_dir').value
        self._pose_source = self.get_parameter('pose_source').value
        self._gt_origin = (
            self.get_parameter('gt_origin_x_m').value,
            self.get_parameter('gt_origin_y_m').value,
            self.get_parameter('gt_origin_yaw_rad').value,
        )
        self._summary_path = os.path.join(self._results_dir, 'summary.csv')
        self._summary = None  # built once Trajectory arrives (needs total_length_m)

        # Reference path retained so true cross-track error can be recomputed
        # from ground truth. /tracking/diagnostics only ever reports error
        # against the pose the CONTROLLER was given, which on the Phase 3
        # wheel-odometry runs read 1 mm while the robot was 2.29 m off-line.
        self._waypoints = []
        self._gt_prev_index = 0
        self._last_est_xy = None

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(Trajectory, '/trajectory/reference', self._on_trajectory, latched_qos)
        self.create_subscription(TrackingDiagnostics, '/tracking/diagnostics', self._on_diagnostics, 20)
        # Remapped by benchmark.launch.py to whichever estimate the tracker is
        # using, so estimator_error compares like with like.
        self.create_subscription(
            Odometry, '/ackermann_like_controller/odometry', self._on_odometry, 10)
        self.create_subscription(Odometry, '/ground_truth/odometry', self._on_ground_truth, 10)

        period = self.get_parameter('summary_write_period_s').value
        self.create_timer(period, self.write_summary)

        self.get_logger().info(
            f"metrics_logger ready: track='{self._track_name}' controller='{self._controller_name}' "
            f"results_dir='{self._results_dir}' (waiting for /trajectory/reference)")

    def _on_trajectory(self, msg: Trajectory) -> None:
        if self._summary is not None:
            return  # published once (latched); ignore any repeats
        deadband = self.get_parameter('reversal_deadband_rad').value
        self._summary = stats.RunSummary(
            self._track_name, self._controller_name, msg.total_length_m, msg.is_closed_loop,
            reversal_deadband_rad=deadband, pose_source=self._pose_source)
        self._waypoints = [(w.x_m, w.y_m) for w in msg.waypoints]
        self.get_logger().info(
            f"Trajectory received: {len(msg.waypoints)} waypoints, "
            f"{'closed loop' if msg.is_closed_loop else 'open path'}, {msg.total_length_m:.2f} m")

    def _on_diagnostics(self, msg: TrackingDiagnostics) -> None:
        if self._summary is None:
            return
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._summary.add_sample(
            stamp_s, msg.lateral_error_m, msg.heading_error_rad, msg.speed_error_mps,
            msg.commanded_steering_rad, msg.path_s_m)

    def _on_odometry(self, msg: Odometry) -> None:
        if self._summary is None:
            return
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self._last_est_xy = (x, y)
        self._summary.add_odometry(x, y)

    def _on_ground_truth(self, msg: Odometry) -> None:
        """True cross-track error, plus how far the estimator has wandered.

        This is the only channel that can tell 'the controller tracked badly'
        apart from 'the controller tracked a phantom perfectly'.
        """
        if self._summary is None or not self._waypoints:
            return

        ox, oy, oyaw = self._gt_origin
        gx, gy = msg.pose.pose.position.x, msg.pose.pose.position.y
        if ox or oy or oyaw:
            c, s = math.cos(oyaw), math.sin(oyaw)
            dx, dy = gx - ox, gy - oy
            gx, gy = c * dx + s * dy, -s * dx + c * dy

        lateral = self._signed_cross_track(gx, gy)
        if lateral is None:
            return

        est_err = None
        if self._last_est_xy is not None and self._pose_source != 'ground_truth':
            est_err = math.hypot(self._last_est_xy[0] - gx, self._last_est_xy[1] - gy)
        self._summary.add_ground_truth_sample(lateral, est_err)

    def _signed_cross_track(self, x: float, y: float):
        """Signed perpendicular distance to the reference path.

        Windowed search around the previous match, matching the semantics of
        hunter_control_common::computeTrackingError so the ground-truth number
        is directly comparable to the controller's own. Sign convention is the
        same 2D cross product: positive = left of the path tangent.
        """
        n = len(self._waypoints)
        if n < 2:
            return None

        window = 200
        best_i, best_d2 = None, float('inf')
        for k in range(-window, window + 1):
            i = (self._gt_prev_index + k) % n
            wx, wy = self._waypoints[i]
            d2 = (wx - x) ** 2 + (wy - y) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        self._gt_prev_index = best_i

        ax, ay = self._waypoints[best_i]
        bx, by = self._waypoints[(best_i + 1) % n]
        tx, ty = bx - ax, by - ay
        norm = math.hypot(tx, ty)
        if norm < 1e-9:
            return None
        return (tx * (y - ay) - ty * (x - ax)) / norm

    def write_summary(self) -> None:
        if self._summary is None:
            return
        stats.write_summary_csv(self._summary_path, self._summary.to_row())
        if getattr(self._summary, 'laps_completed', 0) >= 1 and getattr(self._summary, 'distance_traveled_m', 0) > getattr(self._summary, 'total_length_m', 100) * 0.8:
            self.get_logger().info('Lap completed! Exiting metrics_logger...')
            sys.exit(0)


def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_summary()
        node.get_logger().info(f"Final summary written to {node._summary_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
