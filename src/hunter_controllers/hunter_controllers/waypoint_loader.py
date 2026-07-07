"""
waypoint_loader — loads a CSV waypoint file and publishes the path for RViz2.

Topics published:
  /reference_path  (nav_msgs/msg/Path, latched QoS)

Parameters:
  waypoint_file  str   filename inside package waypoints/ dir, or absolute path
  frame_id       str   frame for the Path header (default: 'odom')
  publish_rate   float Hz (default 1.0 — low; path rarely changes)
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from ament_index_python.packages import get_package_share_directory

from hunter_controllers.controller_base import quat_to_yaw


def yaw_to_quat(yaw: float):
    """Return (x,y,z,w) quaternion for a pure-yaw rotation."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class WaypointLoader(Node):

    def __init__(self):
        super().__init__('waypoint_loader')

        self.declare_parameter('waypoint_file', 'example_track.csv')
        self.declare_parameter('frame_id',      'odom')
        self.declare_parameter('publish_rate',  1.0)

        wf         = self.get_parameter('waypoint_file').value
        self._fid  = self.get_parameter('frame_id').value
        rate       = self.get_parameter('publish_rate').value

        self._waypoints = self._load(wf)
        self._path_msg  = self._build_path()

        # Latched QoS so late subscribers receive the path immediately
        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(Path, '/reference_path', latched)

        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'WaypointLoader: {len(self._waypoints)} waypoints — '
            f'publishing /reference_path @ {rate} Hz'
        )

    # ------------------------------------------------------------------

    def _load(self, waypoint_file: str) -> list[tuple[float, float, float, float]]:
        if os.path.isabs(waypoint_file):
            path = waypoint_file
        else:
            try:
                pkg = get_package_share_directory('hunter_controllers')
                path = os.path.join(pkg, 'waypoints', waypoint_file)
            except Exception:
                path = waypoint_file

        rows: list[tuple[float, float, float, float]] = []
        try:
            with open(path) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split(',')
                    if len(parts) < 4:
                        continue
                    try:
                        rows.append(
                            (float(parts[0]), float(parts[1]),
                             float(parts[2]), float(parts[3]))
                        )
                    except ValueError:
                        continue
        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {path}')
        return rows

    def _build_path(self) -> Path:
        path = Path()
        path.header.frame_id = self._fid
        for x, y, yaw, _ in self._waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self._fid
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            qx, qy, qz, qw    = yaw_to_quat(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        return path

    def _publish(self) -> None:
        self._path_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
