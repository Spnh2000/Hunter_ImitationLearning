"""
Hunter V2 controller launcher.

Usage:
  ros2 launch hunter_controllers controller_launch.py
  ros2 launch hunter_controllers controller_launch.py controller_type:=stanley
  ros2 launch hunter_controllers controller_launch.py controller_type:=lqr
  ros2 launch hunter_controllers controller_launch.py controller_type:=mpc
  ros2 launch hunter_controllers controller_launch.py \\
      controller_type:=pure_pursuit waypoint_file:=monza_track.csv
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('hunter_controllers')

    declared = [
        DeclareLaunchArgument(
            'controller_type',
            default_value='pure_pursuit',
            description='Controller: pure_pursuit | stanley | lqr | mpc',
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='Monza_baseline.csv',
            description='CSV filename in package waypoints/ dir, or absolute path',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
    ]

    waypoint_loader = Node(
        package='hunter_controllers',
        executable='waypoint_loader',
        name='waypoint_loader',
        parameters=[{
            'use_sim_time':  LaunchConfiguration('use_sim_time'),
            'waypoint_file': LaunchConfiguration('waypoint_file'),
        }],
        output='screen',
    )

    controller = Node(
        package='hunter_controllers',
        executable='hunter_controller_node',
        name='hunter_controller_node',
        parameters=[
            os.path.join(pkg, 'config', 'params.yaml'),
            {
                'use_sim_time':    LaunchConfiguration('use_sim_time'),
                'controller_type': LaunchConfiguration('controller_type'),
                'waypoint_file':   LaunchConfiguration('waypoint_file'),
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        *declared,
        waypoint_loader,
        controller,
    ])
