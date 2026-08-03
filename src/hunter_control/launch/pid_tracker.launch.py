import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'target_speed',
            default_value='0.5',
            description='Target forward velocity in m/s'
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='1.0',
            description='Lookahead distance in meters'
        ),
        DeclareLaunchArgument(
            'kp',
            default_value='2.0',
            description='Proportional gain'
        ),
        DeclareLaunchArgument(
            'ki',
            default_value='0.05',
            description='Integral gain'
        ),
        DeclareLaunchArgument(
            'kd',
            default_value='0.2',
            description='Derivative gain'
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='Robot spawn X coordinate'
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Robot spawn Y coordinate'
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='-1.57',
            description='Robot spawn yaw angle'
        )
    ]

    # Tracker node configuration
    tracker_node = Node(
        package='hunter_control',
        executable='pid_tracker',
        name='pid_tracker',
        output='screen',
        parameters=[{
            'target_speed': LaunchConfiguration('target_speed'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'use_sim_time': True
        }]
    )

    import launch

    # Metrics node configuration
    metrics_node = Node(
        package='hunter_control',
        executable='performance_metrics',
        name='performance_metrics',
        output='screen',
        parameters=[{
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'use_sim_time': True
        }]
    )
    
    shutdown_on_exit = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=metrics_node,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
        )
    )

    return LaunchDescription(declared_arguments + [tracker_node, metrics_node, shutdown_on_exit])
