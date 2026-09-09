#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import torch
import torch.nn as nn
import numpy as np
import os

class ImitationPolicy(nn.Module):
    def __init__(self):
        super(ImitationPolicy, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(1080, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 2)
        )
    def forward(self, x):
        return self.network(x)

from rclpy.qos import qos_profile_sensor_data

class ImitationTracker(Node):
    def __init__(self):
        super().__init__('imitation_tracker')

        self.declare_parameter('model_path', 'src/hunter_control/models/imitation_policy.pth')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.get_logger().info(f"Loading Imitation Policy from: {model_path}")
        
        self.model = ImitationPolicy()
        try:
            # Depending on how the model was saved, it could be a state_dict or the full model
            # Here we assume state_dict based on typical PyTorch workflows
            self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
            self.model.eval()
            self.get_logger().info("Model successfully loaded.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model from {model_path}: {e}")

        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/ackermann_like_controller/reference',
            10
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Imitation Tracker Node started.")

    def scan_callback(self, msg):
        # Data Preprocessing (CRITICAL)
        ranges = np.array(msg.ranges, dtype=np.float32)
        
        # Replace inf and NaN with 30.0
        np.nan_to_num(ranges, copy=False, nan=30.0, posinf=30.0, neginf=30.0)
        
        # Normalize to [0, 1] by dividing by 30.0
        ranges = ranges / 30.0
        
        # Convert to torch tensor and add batch dimension
        input_tensor = torch.tensor(ranges, dtype=torch.float32).unsqueeze(0)

        # Forward pass
        with torch.no_grad():
            output = self.model(input_tensor)
            
        # The model outputs a tensor of shape [1, 2]: [steering, speed]
        steering = output[0, 0].item()
        speed = output[0, 1].item()

        # Action Publishing
        twist_msg = TwistStamped()
        
        # Zero Timestamp Rule
        twist_msg.header.stamp.sec = 0
        twist_msg.header.stamp.nanosec = 0
        twist_msg.header.frame_id = '' # Usually not strictly required unless specified
        
        twist_msg.twist.linear.x = speed
        twist_msg.twist.angular.z = steering
        
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    tracker = ImitationTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
