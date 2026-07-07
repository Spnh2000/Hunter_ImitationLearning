"""
Unified Hunter V2 path-tracking controller node.

Uses f1tenth_planning controllers directly (PurePursuit, Stanley, LQR, KinMPC).
Controller selected at runtime via the ``controller_type`` ROS parameter.

Actual f1tenth_planning API (different from the project-prompt description):
  - No Track class, no obs-dict — individual pose args.
  - Waypoints: [x, y, speed, yaw]  (N×4) for PP/Stanley
               [x, y, speed, yaw, curvature]  (N×5) for LQR
  - plan() returns (steering_angle, speed) for all controllers.

Topic:  /ackermann_like_controller/reference  (TwistStamped, NOT cmd_vel/Twist)
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from hunter_controllers.hunter_utils import (
    load_waypoints_f1tenth,
    make_cmd_vel,
    quat_to_yaw,
)


class HunterControllerNode(Node):

    def __init__(self):
        super().__init__('hunter_controller_node')

        # ── Parameters ───────────────────────────────────────────────────────
        ctrl  = self.declare_parameter('controller_type',   'pure_pursuit').value
        wf    = self.declare_parameter('waypoint_file',     '').value
        self.L        = self.declare_parameter('wheelbase',         0.648).value
        self.max_spd  = self.declare_parameter('max_speed',          1.0).value
        freq          = self.declare_parameter('control_freq',       20.0).value

        # controller-specific
        self.lookahead = self.declare_parameter('lookahead_distance', 1.0).value
        self.k_path    = self.declare_parameter('k_path',             5.0).value
        self.lqr_ts    = self.declare_parameter('lqr_timestep',       0.05).value

        # ── Load waypoints in f1tenth format ────────────────────────────────
        wps = load_waypoints_f1tenth(wf, max_speed=self.max_spd)
        if len(wps) == 0:
            self.get_logger().error(f'No waypoints loaded from {wf!r} — check path/format')
        else:
            self.get_logger().info(f'Loaded {len(wps)} waypoints from {wf!r}')

        # ── Instantiate planner ──────────────────────────────────────────────
        self.ctrl_type = ctrl
        self.planner   = self._build_planner(ctrl, wps)

        # ── ROS I/O ──────────────────────────────────────────────────────────
        self._state: np.ndarray | None = None

        self._cmd_pub = self.create_publisher(
            TwistStamped, '/ackermann_like_controller/reference', 10)
        self._dbg_pub = self.create_publisher(
            Float64MultiArray, '/controller_debug', 10)

        self.create_subscription(
            Odometry, '/ackermann_like_controller/odometry',
            self._odom_cb, 10)

        self.create_timer(1.0 / freq, self._timer_cb)
        self.get_logger().info(
            f'[hunter_controller_node] ready — type={ctrl}, waypoints={wf!r}')

    # ------------------------------------------------------------------ build

    def _build_planner(self, ctrl: str, wps: np.ndarray):
        """Instantiate the selected f1tenth planner and return it."""
        wps4 = wps[:, :4]   # [x, y, speed, yaw]   for PP/Stanley
        wps5 = wps           # [x, y, speed, yaw, κ] for LQR

        if ctrl == 'pure_pursuit':
            from f1tenth_planning.control.pure_pursuit.pure_pursuit import PurePursuitPlanner
            return PurePursuitPlanner(wheelbase=self.L, waypoints=wps4)

        elif ctrl == 'stanley':
            from f1tenth_planning.control.stanley.stanley import StanleyPlanner
            return StanleyPlanner(wheelbase=self.L, waypoints=wps4)

        elif ctrl == 'lqr':
            from f1tenth_planning.control.lqr.lqr import LQRPlanner
            p = LQRPlanner(wheelbase=self.L, waypoints=wps5)
            # Library bug: __init__ ignores the wheelbase arg — fix inline
            p.wheelbase = self.L
            return p

        elif ctrl == 'mpc':
            try:
                from f1tenth_planning.control.kinematic_mpc.kinematic_mpc import KinMPCPlanner
                return KinMPCPlanner(wheelbase=self.L, waypoints=wps4)
            except ImportError as e:
                self.get_logger().warn(
                    f'KinMPCPlanner not available ({e}); falling back to pure_pursuit')
                from f1tenth_planning.control.pure_pursuit.pure_pursuit import PurePursuitPlanner
                self.ctrl_type = 'pure_pursuit'
                return PurePursuitPlanner(wheelbase=self.L, waypoints=wps4)

        else:
            self.get_logger().warn(f'Unknown controller {ctrl!r}; using pure_pursuit')
            from f1tenth_planning.control.pure_pursuit.pure_pursuit import PurePursuitPlanner
            self.ctrl_type = 'pure_pursuit'
            return PurePursuitPlanner(wheelbase=self.L, waypoints=wps4)

    # ------------------------------------------------------------------ odom

    def _odom_cb(self, msg: Odometry) -> None:
        self._state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_yaw(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
        ])

    # ------------------------------------------------------------------ timer

    def _timer_cb(self) -> None:
        if self._state is None or self.planner is None:
            return

        x, y, yaw, v = self._state

        try:
            steering_angle, ref_speed = self._call_planner(x, y, yaw, v)
        except Exception as e:
            self.get_logger().warn(f'Planner error: {e}')
            return

        ref_speed = float(np.clip(ref_speed, 0.0, self.max_spd))
        self._cmd_pub.publish(make_cmd_vel(ref_speed, steering_angle, self.L))

        dbg = Float64MultiArray()
        dbg.data = [float(steering_angle), float(ref_speed), float(v)]
        self._dbg_pub.publish(dbg)

    def _call_planner(self, x: float, y: float, yaw: float, v: float):
        """Dispatch to the selected planner's plan() call."""
        if self.ctrl_type == 'pure_pursuit':
            return self.planner.plan(x, y, yaw, self.lookahead)

        elif self.ctrl_type == 'stanley':
            return self.planner.plan(x, y, yaw, v, k_path=self.k_path)

        elif self.ctrl_type == 'lqr':
            return self.planner.plan(x, y, yaw, v, timestep=self.lqr_ts)

        else:
            # mpc / fallback — uses pure_pursuit interface
            return self.planner.plan(x, y, yaw, self.lookahead)


def main(args=None):
    rclpy.init(args=args)
    node = HunterControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
