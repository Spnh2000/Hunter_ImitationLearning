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
    def __init__(self, cnn_channels, fc_hidden_sizes, kernel_size=5, dropout_rate=0.0, use_batch_norm=False):
        super(ImitationPolicy, self).__init__()
        
        # 1. Build 1D CNN feature extractor
        cnn_layers = []
        in_channels = 1
        
        for out_channels in cnn_channels:
            cnn_layers.append(nn.Conv1d(in_channels, out_channels, kernel_size=kernel_size, padding=kernel_size//2))
            if use_batch_norm:
                cnn_layers.append(nn.BatchNorm1d(out_channels))
            cnn_layers.append(nn.ReLU())
            cnn_layers.append(nn.MaxPool1d(kernel_size=2))
            in_channels = out_channels
            
        self.cnn = nn.Sequential(*cnn_layers)
        
        # Calculate flattened size
        length = 1080
        for _ in cnn_channels:
            length = length // 2
            
        flattened_size = cnn_channels[-1] * length if cnn_channels else 1080
        
        # 2. Build Fully Connected head
        fc_layers = []
        in_features = flattened_size
        
        for h in fc_hidden_sizes:
            fc_layers.append(nn.Linear(in_features, h))
            if use_batch_norm:
                fc_layers.append(nn.BatchNorm1d(h))
            fc_layers.append(nn.ReLU())
            if dropout_rate > 0.0:
                fc_layers.append(nn.Dropout(dropout_rate))
            in_features = h
            
        fc_layers.append(nn.Linear(in_features, 2)) # Output: [steering, speed]
        self.fc = nn.Sequential(*fc_layers)

    def forward(self, x):
        # x shape: [batch, 1080] -> Reshape for CNN: [batch, 1 channel, 1080]
        x = x.unsqueeze(1)
        x = self.cnn(x)
        x = x.view(x.size(0), -1) # Flatten
        return self.fc(x)

from rclpy.qos import qos_profile_sensor_data

class ImitationTracker(Node):
    def __init__(self):
        super().__init__('imitation_tracker')

        self.declare_parameter('model_path', 'src/hunter_control/models/imitation_policy.pth')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.get_logger().info(f"Loading Imitation Policy from: {model_path}")
        
        try:
            checkpoint = torch.load(model_path, map_location=torch.device('cpu'))
            
            # Initialize model dynamically from saved hyperparameters
            hparams = checkpoint['hyperparameters']
            self.model = ImitationPolicy(
                cnn_channels=hparams['cnn_channels'],
                fc_hidden_sizes=hparams['fc_hidden_sizes'],
                kernel_size=hparams['kernel_size'],
                dropout_rate=hparams['dropout_rate'],
                use_batch_norm=hparams['use_batch_norm']
            )
            
            # Load weights
            self.model.load_state_dict(checkpoint['state_dict'])
            self.model.eval()
            
            # Load action normalization stats
            self.action_mean = np.array(checkpoint['action_mean'])
            self.action_std = np.array(checkpoint['action_std'])
            
            self.get_logger().info("Model and normalization stats successfully loaded.")
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
            
        # The model outputs a normalized tensor of shape [1, 2]: [steering, speed]
        normalized_actions = output.numpy()[0]
        
        # Un-normalize actions using saved statistics
        real_actions = normalized_actions * self.action_std + self.action_mean
        
        steering = float(real_actions[0])
        speed = float(real_actions[1])

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
