#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import numpy as np
import math
import os
import json
import sys

class PerformanceMetricsNode(Node):
    def __init__(self):
        super().__init__('performance_metrics')
        self.get_logger().info('Initializing Performance Metrics Node...')
        
        # Load Track
        # workspace_src_dir = os.path.expanduser('~/RPTU/lab_pj/hunter-v2/Hunter_RL/src')
        workspace_src_dir = os.path.expanduser('~/Project_Lab/Hunter_RL/src')
        csv_file_path = os.path.join(workspace_src_dir, 'IMS', 'IMS_centerline.csv')
        
        self.WAYPOINTS = []
        try:
            raw_waypoints = np.loadtxt(csv_file_path, delimiter=',')
            self.WAYPOINTS = raw_waypoints[:, 0:2]
            self.get_logger().info(f'Loaded {len(self.WAYPOINTS)} centerline waypoints.')
        except Exception as e:
            self.get_logger().error(f'Failed to load track CSV: {e}')
            
        # Metrics state
        self.total_squared_lateral_error = 0.0
        self.total_squared_heading_error = 0.0
        self.max_abs_lateral_error = 0.0
        self.max_abs_heading_error = 0.0
        self.odom_count = 0
        
        self.total_steering_oscillation = 0.0
        self.prev_steering = None
        
        self.max_waypoint_idx_reached = 0
        
        self.spawn_x = self.declare_parameter('spawn_x', 0.0).value
        self.spawn_y = self.declare_parameter('spawn_y', 0.0).value
        self.spawn_yaw = self.declare_parameter('spawn_yaw', -1.57).value

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/ackermann_like_controller/odometry', 
            self.odom_callback, 
            10
        )
        self.ref_sub = self.create_subscription(
            TwistStamped, 
            '/ackermann_like_controller/reference', 
            self.ref_callback, 
            10
        )

    def ref_callback(self, msg):
        current_steering = msg.twist.angular.z
        if self.prev_steering is not None:
            diff = abs(current_steering - self.prev_steering)
            self.total_steering_oscillation += diff
        self.prev_steering = current_steering

    def odom_callback(self, msg):
        if len(self.WAYPOINTS) == 0:
            return
            
        # Extract robot pose
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        # Convert to world frame based on spawn
        cos_yaw = math.cos(self.spawn_yaw)
        sin_yaw = math.sin(self.spawn_yaw)
        x = x_odom * cos_yaw - y_odom * sin_yaw + self.spawn_x
        y = x_odom * sin_yaw + y_odom * cos_yaw + self.spawn_y
        
        # Yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta_odom = math.atan2(siny_cosp, cosy_cosp)
        theta = theta_odom + self.spawn_yaw
        
        # Find closest waypoint
        min_dist = float('inf')
        closest_idx = 0
        for i, wp in enumerate(self.WAYPOINTS):
            dist = math.hypot(wp[0] - x, wp[1] - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        # Track lap completion
        if closest_idx > self.max_waypoint_idx_reached:
            # Simple assumption: monotonic increase without skipping full track
            # To handle lap wrap-around, we cap it by half the track size
            if closest_idx - self.max_waypoint_idx_reached < len(self.WAYPOINTS) / 2:
                self.max_waypoint_idx_reached = closest_idx
                
        if self.max_waypoint_idx_reached > len(self.WAYPOINTS) * 0.95 and closest_idx < len(self.WAYPOINTS) * 0.05:
            self.get_logger().info('1 lap completed! Shutting down...')
            sys.exit(0)
        
        # CTE & Heading Error
        closest_wp = self.WAYPOINTS[closest_idx]
        next_idx = (closest_idx + 1) % len(self.WAYPOINTS)
        next_wp = self.WAYPOINTS[next_idx]
        
        path_yaw = math.atan2(next_wp[1] - closest_wp[1], next_wp[0] - closest_wp[0])
        dx = x - closest_wp[0]
        dy = y - closest_wp[1]
        
        cte = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        abs_cte = abs(cte)
        
        heading_error = path_yaw - theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        abs_he = abs(heading_error)
        
        # Accumulate metrics
        self.total_squared_lateral_error += cte ** 2
        self.total_squared_heading_error += heading_error ** 2
        if abs_cte > self.max_abs_lateral_error:
            self.max_abs_lateral_error = abs_cte
        if abs_he > self.max_abs_heading_error:
            self.max_abs_heading_error = abs_he
            
        self.odom_count += 1

    def save_metrics(self):
        if self.odom_count == 0:
            self.get_logger().info('No odometry data received. No metrics saved.')
            return
            
        rmse_cte = math.sqrt(self.total_squared_lateral_error / self.odom_count)
        rmse_he = math.sqrt(self.total_squared_heading_error / self.odom_count)
        
        completion_pct = (self.max_waypoint_idx_reached / float(len(self.WAYPOINTS) - 1)) * 100.0 if len(self.WAYPOINTS) > 1 else 0.0
        # Cap at 100%
        completion_pct = min(100.0, completion_pct)
        
        report = {
            'Lap Completion (%)': round(completion_pct, 2),
            'Lateral Error RMSE (m)': round(rmse_cte, 4),
            'Max Lateral Error (m)': round(self.max_abs_lateral_error, 4),
            'Heading Error RMSE (rad)': round(rmse_he, 4),
            'Max Heading Error (rad)': round(self.max_abs_heading_error, 4),
            'Total Steering Oscillation (rad)': round(self.total_steering_oscillation, 4)
        }
        
        try:
            with open('controller_benchmark.json', 'w') as f:
                json.dump(report, f, indent=4)
            self.get_logger().info(f'Metrics saved to controller_benchmark.json:\n{json.dumps(report, indent=4)}')
        except Exception as e:
            self.get_logger().error(f'Failed to save metrics: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Performance Metrics Node...')
    finally:
        node.save_metrics()
        node.destroy_node()
        # Shutdown cleanly without raising if already shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
