#!/usr/bin/env python3
"""
Lateral Path-Following PID Controller Node.
Implements a ROS 2 node that tracks a hardcoded path using a PID controller
acting on the heading error towards a look-ahead target waypoint.
"""
from setuptools import namespaces
import math
import rclpy, os
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
class LateralPIDController(Node):
    """
    ROS 2 Node implementing a lateral path-following PID controller.
    """
    
    # =========================================================================
    # Configurable placeholder variables (easy to tune)
    # =========================================================================
    KP = 1.5
    KI = 0.05
    KD = 0.2
    
    TARGET_SPEED = 1.0      # Target linear speed in m/s
    LOOKAHEAD_DISTANCE = 1.8  # Lookahead distance in meters
    MAX_STEER = 0.785      # Max steering angle/angular velocity in rad (~26.4 deg)
    MAX_INTEGRAL = 0.5         # Anti-windup limit for the integral term

    def __init__(self):
        super().__init__('lateral_pid_controller')
        self.get_logger().info("Initializing Lateral PID Controller Node...")

        self.WAYPOINTS = []

        # workspace_src_dir = os.path.expanduser('~/RPTU/lab_pj/hunter-v2/Hunter_RL/src')
        workspace_src_dir = os.path.expanduser('~/Project_Lab/Hunter_RL/src')
        csv_file_path = os.path.join(workspace_src_dir, 'IMS', 'IMS_centerline.csv')
        self.get_logger().info(f"Attempting to load track data from: {csv_file_path}")

        try:
            # Centerline uses commas and has X and Y in the first two columns
            raw_waypoints = np.loadtxt(csv_file_path, delimiter=',')
            self.WAYPOINTS = raw_waypoints[:, 0:2]
            
            self.get_logger().info(f"Successfully loaded {len(self.WAYPOINTS)} Centerline waypoints!")
            
        except Exception as e:
            self.get_logger().error(f"CRITICAL: Error reading the CSV file: {e}")
            self.WAYPOINTS = np.array([])
    
        # Declare parameters (default to class constants)
        self.declare_parameter('kp', self.KP)
        self.declare_parameter('ki', self.KI)
        self.declare_parameter('kd', self.KD)
        self.declare_parameter('target_speed', self.TARGET_SPEED)
        self.declare_parameter('lookahead_distance', self.LOOKAHEAD_DISTANCE)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', -1.57)
        # Retrieve parameter values
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.target_speed = self.get_parameter('target_speed').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.spawn_yaw = self.get_parameter('spawn_yaw').value
        # Controller states
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.last_time = None
        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ackermann_like_controller/odometry',
            self.odom_callback,
            10
        )
        # Publisher
        self.ref_pub = self.create_publisher(
            TwistStamped,
            '/ackermann_like_controller/reference',
            10
        )
        self.get_logger().info(f"Subscribed to '/ackermann_like_controller/odometry'")
        self.get_logger().info(f"Publishing to '/ackermann_like_controller/reference'")
        self.get_logger().info(f"PID Parameters -> Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}")
        self.get_logger().info(f"Lookahead distance: {self.lookahead_distance} meters")
    def odom_callback(self, msg: Odometry):
        """
        Callback triggered upon receiving a new Odometry message.
        Processes states, computes PID tracking, and publishes TwistStamped command.
        """
        # 1. Parse current robot position (X, Y) from odom
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y
        # 2. Convert quaternion orientation into a yaw angle (theta_odom) in odom frame
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_odom = math.atan2(siny_cosp, cosy_cosp)
        # 2.5 Transform odom pose to world frame to align with WAYPOINTS
        cos_yaw = math.cos(self.spawn_yaw)
        sin_yaw = math.sin(self.spawn_yaw)
        x = x_odom * cos_yaw - y_odom * sin_yaw + self.spawn_x
        y = x_odom * sin_yaw + y_odom * cos_yaw + self.spawn_y
        theta = theta_odom + self.spawn_yaw
        theta = math.atan2(math.sin(theta), math.cos(theta))
        # 3. Find the closest waypoint on the path
        min_dist = float('inf')
        closest_idx = 0
        for i, waypoint in enumerate(self.WAYPOINTS):
            dist = math.hypot(waypoint[0] - x, waypoint[1] - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        # Calculate Cross-Track Error (CTE) relative to closest waypoint for logging
        closest_waypoint = self.WAYPOINTS[closest_idx]
        next_idx = (closest_idx + 1) % len(self.WAYPOINTS)
        next_waypoint = self.WAYPOINTS[next_idx]
        
        path_yaw = math.atan2(next_waypoint[1] - closest_waypoint[1], next_waypoint[0] - closest_waypoint[0])
        dx = x - closest_waypoint[0]
        dy = y - closest_waypoint[1]
        
        # Left of path is positive, right of path is negative
        cte = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        # 3.5 Speed logic: slow down on tight corners or high lateral error to maintain traction
        speed = self.target_speed
        if abs(cte) > 0.15 or abs(self.prev_error) > 0.10:
            speed = max(1.2, speed * 0.6)
        # Calculate dynamic lookahead distance based on current target speed
        dynamic_lookahead = max(1.0, self.lookahead_distance * (speed / self.target_speed))
        # 4. Find look-ahead target waypoint based on dynamic_lookahead (in meters)
        accum_dist = 0.0
        lookahead_idx = closest_idx
        n_wpts = len(self.WAYPOINTS)
        
        while accum_dist < dynamic_lookahead:
            n_idx = (lookahead_idx + 1) % n_wpts
            p1 = self.WAYPOINTS[lookahead_idx]
            p2 = self.WAYPOINTS[n_idx]
            step_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            accum_dist += step_dist
            lookahead_idx = n_idx
            if lookahead_idx == closest_idx:
                break  # Prevent infinite loop if lookahead exceeds path length
        target_waypoint = self.WAYPOINTS[lookahead_idx]
        # 5. Determine the target yaw required to head towards the look-ahead point
        target_yaw = math.atan2(target_waypoint[1] - y, target_waypoint[0] - x)
        # Calculate heading error relative to target waypoint
        heading_error = target_yaw - theta
        # 6. Normalize heading error cleanly between -pi and pi using math.atan2
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        # 7. Compute Time Step (dt)
        curr_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = curr_time
            return
        dt = (curr_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 0.01  # Fallback to prevent division by zero
        # 8. Calculate continuous PID loop equations
        self.integral_error += heading_error * dt
        # Anti-windup clamping
        self.integral_error = max(min(self.integral_error, self.MAX_INTEGRAL), -self.MAX_INTEGRAL)
        derivative_error = (heading_error - self.prev_error) / dt
        # Continuous PID logic: Output = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)
        steering_output = (self.kp * heading_error) + (self.ki * self.integral_error) + (self.kd * derivative_error)
        # 9. Clamp/limit the maximum absolute angular output to prevent over-steering
        steering_output = max(min(steering_output, self.MAX_STEER), -self.MAX_STEER)
        # Log tracking state details periodically
        self.get_logger().info(
            f"CTE: {cte:.3f}m | Heading Err: {heading_error:.3f}rad | "
            f"Steer: {steering_output:.3f}rad | Speed: {speed:.2f}m/s | Wpt: #{lookahead_idx}",
            throttle_duration_sec=0.5
        )
        # 10. Populate and publish TwistStamped message
        msg_out = TwistStamped()
        msg_out.header.stamp = curr_time.to_msg()
        msg_out.header.frame_id = 'base_link'
        msg_out.twist.linear.x = speed
        msg_out.twist.angular.z = steering_output
        self.ref_pub.publish(msg_out)
        # Save states for next loop
        self.prev_error = heading_error
        self.last_time = curr_time
def main(args=None):
    rclpy.init(args=args)
    node = LateralPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down PID controller node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()