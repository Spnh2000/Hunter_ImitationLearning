"""
Shared helpers for the Hunter V2 f1tenth_planning wrapper nodes.
"""

import math
import os

import numpy as np
import rclpy
import rclpy.time
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped

MAX_STEER = 0.524   # Hunter V2 physical limit ~30 deg


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def quat_to_yaw(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion (no tf2 dependency)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ---------------------------------------------------------------------------
# obs dict helpers
# ---------------------------------------------------------------------------

def build_obs() -> dict:
    """Return a zero-initialised obs dict with all keys used by planner calls."""
    return {
        'pose_x':       0.0,
        'pose_y':       0.0,
        'pose_theta':   0.0,
        'linear_vel_x': 0.0,
        'scan':         np.zeros(1080, dtype=np.float32),
    }


# ---------------------------------------------------------------------------
# Waypoint loading
# ---------------------------------------------------------------------------

def load_waypoints_f1tenth(waypoint_file: str, max_speed: float = 1.0) -> np.ndarray:
    """
    Load a CSV stored in our format ``x, y, yaw, speed``
    and return it in the f1tenth_planning format::

        columns: [x, y, speed, yaw, curvature]   shape (N, 5)

    f1tenth_planning waypoint conventions:
      - PurePursuit / Stanley  →  first 4 cols  [x, y, speed, yaw]
      - LQR                    →  all 5 cols     [x, y, speed, yaw, curvature]

    Speed values are clamped to *max_speed*.
    """
    if not waypoint_file:
        return np.empty((0, 5))

    if os.path.isabs(waypoint_file):
        path = waypoint_file
    else:
        try:
            pkg = get_package_share_directory('hunter_controllers')
            path = os.path.join(pkg, 'waypoints', waypoint_file)
        except Exception:
            path = waypoint_file

    rows: list[list[float]] = []
    try:
        with open(path, newline='') as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) < 4:
                    continue
                try:
                    rows.append([float(p) for p in parts[:4]])
                except ValueError:
                    continue
    except FileNotFoundError:
        return np.empty((0, 5))

    if not rows:
        return np.empty((0, 5))

    arr = np.array(rows, dtype=float)       # (N, 4)  [x, y, yaw, speed]

    # Swap cols 2↔3 to match library convention  [x, y, speed, yaw]
    lib4 = arr[:, [0, 1, 3, 2]]
    lib4[:, 2] = np.clip(lib4[:, 2], 0.0, max_speed)

    # Compute curvature from x, y for LQR's 5th column
    xy = lib4[:, :2]
    n = len(xy)
    kappas = np.zeros(n)
    for i in range(n):
        prv = xy[(i - 1) % n]
        cur = xy[i]
        nxt = xy[(i + 1) % n]
        dx1 = cur[0] - prv[0];  dy1 = cur[1] - prv[1]
        dx2 = nxt[0] - cur[0];  dy2 = nxt[1] - cur[1]
        cross = abs(dx1 * dy2 - dy1 * dx2)
        seg = (math.hypot(dx1, dy1) + math.hypot(dx2, dy2)) / 2 + 1e-9
        kappas[i] = cross / seg ** 2

    return np.column_stack([lib4, kappas])  # (N, 5)  [x, y, speed, yaw, curvature]


# ---------------------------------------------------------------------------
# Command conversion  (single authoritative place for angular.z computation)
# ---------------------------------------------------------------------------

def make_cmd_vel(speed: float, steering_angle: float, wheelbase: float = 0.648) -> TwistStamped:
    """
    Convert f1tenth_planning controller output to the TwistStamped message
    required by AckermannSteeringController on ``/ackermann_like_controller/reference``.

    Note: despite the name ``make_cmd_vel``, the Hunter V2 AckermannSteeringController
    subscribes on ``~/reference`` and expects ``TwistStamped``, NOT a plain ``Twist``
    on ``~/cmd_vel``.  This is the only place ``angular.z`` is computed.

    Args:
        speed:          longitudinal speed [m/s] — from planner.plan()
        steering_angle: front-wheel angle [rad]  — from planner.plan()
        wheelbase:      Hunter V2 wheelbase = 0.648 m

    Conversion: ``ω = v·tan(δ)/L``
    Stamp = 0 so the controller's reference_timeout uses its own clock,
    avoiding rejections from ROS ↔ Gazebo clock-sync jitter.
    """
    delta = float(np.clip(steering_angle, -MAX_STEER, MAX_STEER))
    omega = speed * math.tan(delta) / wheelbase if abs(speed) > 0.01 else 0.0

    ts = TwistStamped()
    ts.header.stamp    = rclpy.time.Time().to_msg()
    ts.header.frame_id = 'base_link'
    ts.twist.linear.x  = float(np.clip(speed, 0.0, 2.0))
    ts.twist.angular.z = float(omega)
    return ts
