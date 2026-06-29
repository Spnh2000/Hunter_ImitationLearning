import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Resolve paths dynamically
    try:
        share_dir = get_package_share_directory('hunter_control')
        workspace_root = share_dir
        for _ in range(4):
            workspace_root = os.path.dirname(workspace_root)
        default_csv_path = os.path.join(workspace_root, 'src', 'Monza', 'monza_track.csv')
        default_metrics_path = os.path.join(workspace_root, 'metrics_baseline.json')
    except Exception:
        default_csv_path = 'src/Monza/monza_track.csv'
        default_metrics_path = 'metrics_baseline.json'

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'csv_path',
            default_value=default_csv_path,
            description='Full path to the reference centerline or raceline CSV file'
        ),
        DeclareLaunchArgument(
            'target_speed',
            default_value='2.5',
            description='Target linear velocity in m/s (negative values follow speed profile in raceline CSV)'
        ),
        DeclareLaunchArgument(
            'kp_lateral',
            default_value='1.6',
            description='Proportional gain for lateral (cross-track) error'
        ),
        DeclareLaunchArgument(
            'ki_lateral',
            default_value='0.04',
            description='Integral gain for lateral error'
        ),
        DeclareLaunchArgument(
            'kd_lateral',
            default_value='0.18',
            description='Derivative gain for lateral error'
        ),
        DeclareLaunchArgument(
            'kp_heading',
            default_value='1.2',
            description='Proportional gain for heading error'
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='1.5',
            description='Lookahead distance in meters for reference tracking (0.0 to track closest point directly)'
        ),
        DeclareLaunchArgument(
            'max_laps',
            default_value='1',
            description='Number of laps to complete before stopping and saving metrics'
        ),
        DeclareLaunchArgument(
            'metrics_output_path',
            default_value=default_metrics_path,
            description='Full path to save the final JSON metrics report'
        )
    ]

    # Tracker node configuration
    tracker_node = Node(
        package='hunter_control',
        executable='pid_tracker',
        name='pid_tracker',
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('csv_path'),
            'target_speed': LaunchConfiguration('target_speed'),
            'kp_lateral': LaunchConfiguration('kp_lateral'),
            'ki_lateral': LaunchConfiguration('ki_lateral'),
            'kd_lateral': LaunchConfiguration('kd_lateral'),
            'kp_heading': LaunchConfiguration('kp_heading'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'max_laps': LaunchConfiguration('max_laps'),
            'metrics_output_path': LaunchConfiguration('metrics_output_path'),
            'use_sim_time': True
        }]
    )

    return LaunchDescription(declared_arguments + [tracker_node])
