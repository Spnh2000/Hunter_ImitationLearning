"""
waypoint_recorder — drives the Hunter manually (teleop) and records the path.

Subscribes: /ackermann_like_controller/odometry  (nav_msgs/Odometry)
Output:     CSV file with columns  x, y, yaw, speed

Parameters:
  output_file    str    path to write the CSV (default: ~/recorded_path.csv)
  record_rate    float  Hz — maximum rate at which points are logged (default 10)
  min_distance   float  m  — minimum distance between consecutive recorded points
                              (avoids dense clusters when the robot is stationary)

Usage:
  ros2 run hunter_controllers waypoint_recorder
  (drive the Hunter with teleop)
  Ctrl+C  →  CSV saved
"""

import math
import os
import signal
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from hunter_controllers.controller_base import quat_to_yaw


class WaypointRecorder(Node):

    def __init__(self):
        super().__init__('waypoint_recorder')

        self.declare_parameter(
            'output_file',
            os.path.expanduser('~/recorded_path.csv'),
        )
        self.declare_parameter('record_rate',  10.0)
        self.declare_parameter('min_distance', 0.1)

        self._out_file    = self.get_parameter('output_file').value
        rate              = self.get_parameter('record_rate').value
        self._min_dist    = self.get_parameter('min_distance').value

        self._points: list[tuple[float, float, float, float]] = []
        self._last_pos: tuple[float, float] | None = None
        self._record = False  # enabled by timer below

        self._odom_sub = self.create_subscription(
            Odometry,
            '/ackermann_like_controller/odometry',
            self._odom_cb,
            10,
        )
        self._timer = self.create_timer(1.0 / rate, self._enable_record)

        # Flush on Ctrl+C
        signal.signal(signal.SIGINT, self._shutdown)

        self.get_logger().info(
            f'WaypointRecorder ready — output: {self._out_file}\n'
            f'Drive the Hunter then press Ctrl+C to save.'
        )

    def _enable_record(self):
        self._record = True

    def _odom_cb(self, msg: Odometry) -> None:
        if not self._record:
            return
        self._record = False  # reset until next timer tick (rate-limits logging)

        x     = msg.pose.pose.position.x
        y     = msg.pose.pose.position.y
        yaw   = quat_to_yaw(msg.pose.pose.orientation)
        speed = msg.twist.twist.linear.x

        # Distance filter
        if self._last_pos is not None:
            dx = x - self._last_pos[0]
            dy = y - self._last_pos[1]
            if math.hypot(dx, dy) < self._min_dist:
                return

        self._last_pos = (x, y)
        self._points.append((x, y, yaw, speed))

    def _shutdown(self, *_):
        self._save()
        self.get_logger().info(f'Saved {len(self._points)} waypoints to {self._out_file}')
        rclpy.try_shutdown()
        sys.exit(0)

    def _save(self):
        os.makedirs(os.path.dirname(os.path.abspath(self._out_file)), exist_ok=True)
        with open(self._out_file, 'w') as f:
            f.write('x,y,yaw,speed\n')
            for x, y, yaw, speed in self._points:
                f.write(f'{x:.6f},{y:.6f},{yaw:.6f},{speed:.4f}\n')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # handled by signal
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
