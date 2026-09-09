#!/usr/bin/env python3

import argparse
import os
import math
import numpy as np

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def main():
    parser = argparse.ArgumentParser(description="Build offline RL dataset from ROS bags.")
    parser.add_argument('--bags', nargs='+', required=True, help="Path to input MCAP bags (directory containing .mcap and metadata.yaml).")
    parser.add_argument('--out', required=True, help="Path to output NPZ dataset.")
    args = parser.parse_args()

    # Pre-fetch message types
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from hunter_msgs.msg import TrackingDiagnostics

    topic_types = {
        '/scan': LaserScan,
        '/ground_truth/odometry': Odometry,
        '/ackermann_like_controller/reference': TwistStamped,
        '/tracking/diagnostics': TrackingDiagnostics
    }

    obs_list = []
    act_list = []
    rew_list = []
    term_list = []
    timeout_list = []
    pose_list = []

    for bag_path in args.bags:
        print(f"Processing {bag_path}...")
        
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except Exception as e:
            print(f"Failed to open {bag_path}: {e}")
            continue
        
        topic_types_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
        
        skip = False
        for req_topic in topic_types.keys():
            if req_topic not in topic_types_map:
                print(f"Warning: {req_topic} missing in {bag_path}. Skipping bag.")
                skip = True
                break
        if skip:
            continue

        latest_msgs = {}
        bag_term_list = []
        
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            
            if topic in topic_types:
                msg_type = topic_types[topic]
                msg = deserialize_message(data, msg_type)
                latest_msgs[topic] = msg
                
                # We use /scan as the trigger for a new step
                if topic == '/scan':
                    if all(k in latest_msgs for k in ['/ground_truth/odometry', '/ackermann_like_controller/reference', '/tracking/diagnostics']):
                        # Process Observation
                        scan = latest_msgs['/scan']
                        ranges = np.array(scan.ranges, dtype=np.float32)
                        ranges[np.isnan(ranges)] = 30.0
                        ranges[np.isinf(ranges)] = 30.0
                        ranges = np.clip(ranges, 0.0, 30.0)
                        
                        # Process Action [steering, speed]
                        twist = latest_msgs['/ackermann_like_controller/reference']
                        action = np.array([twist.twist.angular.z, twist.twist.linear.x], dtype=np.float32)
                        
                        # Process Poses [x, y, yaw]
                        odom = latest_msgs['/ground_truth/odometry']
                        x = odom.pose.pose.position.x
                        y = odom.pose.pose.position.y
                        yaw = quaternion_to_yaw(odom.pose.pose.orientation)
                        pose = np.array([x, y, yaw], dtype=np.float32)
                        
                        # Process Reward
                        diag = latest_msgs['/tracking/diagnostics']
                        reward = diag.actual_speed_mps * math.cos(diag.heading_error_rad) - 0.5 * abs(diag.lateral_error_m)
                        
                        obs_list.append(ranges)
                        act_list.append(action)
                        rew_list.append(reward)
                        pose_list.append(pose)
                        
                        # Assuming the episode hasn't ended yet
                        bag_term_list.append(False)
                        timeout_list.append(False)

        # Mark the last step of the bag as timeout or terminal
        if len(bag_term_list) > 0:
            bag_term_list[-1] = True
            timeout_list[-1] = True
            term_list.extend(bag_term_list)

    if len(obs_list) == 0:
        print("No data extracted. Exiting.")
        return

    # Save to NPZ
    print(f"Saving to {args.out}...")
    np.savez_compressed(args.out,
        observations=np.array(obs_list),
        actions=np.array(act_list),
        rewards=np.array(rew_list, dtype=np.float32),
        terminals=np.array(term_list, dtype=bool),
        timeouts=np.array(timeout_list, dtype=bool),
        expert_poses=np.array(pose_list)
    )
        
    print(f"Done! Dataset contains {len(obs_list)} steps.")

if __name__ == '__main__':
    main()
