"""Standalone trajectory_server launch, for testing a track file in isolation
(e.g. against RViz) before bringing up the rest of the benchmark stack.

Usage:
  ros2 launch hunter_trajectory trajectory_server.launch.py \\
      track_file:=/ros_ws/src/Monza/Monza_raceline.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('track_file', description='Path to a centerline/raceline CSV'),
        DeclareLaunchArgument('path_format', default_value='auto'),
        DeclareLaunchArgument('frame_id', default_value='odom'),
        DeclareLaunchArgument('speed_source', default_value='curvature_limited'),
        DeclareLaunchArgument('max_speed_mps', default_value='1.0'),
        DeclareLaunchArgument('min_speed_mps', default_value='0.2'),
        DeclareLaunchArgument('lateral_accel_limit_mps2', default_value='1.2'),
        DeclareLaunchArgument('constant_speed_mps', default_value='0.5'),
        DeclareLaunchArgument('close_loop_gap_threshold_m', default_value='0.05'),
        DeclareLaunchArgument('visualize', default_value='true'),
    ]

    trajectory_server = Node(
        package='hunter_trajectory',
        executable='trajectory_server',
        name='trajectory_server',
        output='screen',
        parameters=[{
            'track_file': LaunchConfiguration('track_file'),
            'path_format': LaunchConfiguration('path_format'),
            'frame_id': LaunchConfiguration('frame_id'),
            'speed_source': LaunchConfiguration('speed_source'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'min_speed_mps': LaunchConfiguration('min_speed_mps'),
            'lateral_accel_limit_mps2': LaunchConfiguration('lateral_accel_limit_mps2'),
            'constant_speed_mps': LaunchConfiguration('constant_speed_mps'),
            'close_loop_gap_threshold_m': LaunchConfiguration('close_loop_gap_threshold_m'),
            'visualize': LaunchConfiguration('visualize'),
        }],
    )

    return LaunchDescription([*declared_arguments, trajectory_server])
