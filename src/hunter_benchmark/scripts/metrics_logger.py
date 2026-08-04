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

        self._track_name = self.get_parameter('track_name').value
        self._controller_name = self.get_parameter('controller_name').value
        self._results_dir = self.get_parameter('results_dir').value
        self._summary_path = os.path.join(self._results_dir, 'summary.csv')
        self._summary = None  # built once Trajectory arrives (needs total_length_m)

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(Trajectory, '/trajectory/reference', self._on_trajectory, latched_qos)
        self.create_subscription(TrackingDiagnostics, '/tracking/diagnostics', self._on_diagnostics, 20)
        self.create_subscription(
            Odometry, '/ackermann_like_controller/odometry', self._on_odometry, 10)

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
            reversal_deadband_rad=deadband)
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
        self._summary.add_odometry(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def write_summary(self) -> None:
        if self._summary is None:
            return
        stats.write_summary_csv(self._summary_path, self._summary.to_row())


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
