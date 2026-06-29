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
        default_metrics_path = os.path.join(workspace_root, 'metrics_lqr_baseline.json')
    except Exception:
        default_csv_path = 'src/Monza/monza_track.csv'
        default_metrics_path = 'metrics_lqr_baseline.json'

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'csv_path',
            default_value=default_csv_path,
            description='Full path to the reference centerline or raceline CSV file'
        ),
        DeclareLaunchArgument(
            'target_speed',
            default_value='1.5',
            description='Target linear velocity in m/s'
        ),
        DeclareLaunchArgument(
            'q_lateral',
            default_value='2.0',
            description='Weight penalty for cross-track lateral error'
        ),
        DeclareLaunchArgument(
            'q_lateral_dot',
            default_value='0.2',
            description='Weight penalty for lateral error change rate'
        ),
        DeclareLaunchArgument(
            'q_heading',
            default_value='1.5',
            description='Weight penalty for heading error'
        ),
        DeclareLaunchArgument(
            'q_heading_dot',
            default_value='0.1',
            description='Weight penalty for heading error change rate'
        ),
        DeclareLaunchArgument(
            'r_steering',
            default_value='1.0',
            description='Control effort weight penalty for steering angle'
        ),
        DeclareLaunchArgument(
            'max_laps',
            default_value='1',
            description='Number of laps to complete before stopping'
        ),
        DeclareLaunchArgument(
            'metrics_output_path',
            default_value=default_metrics_path,
            description='Full path to save final JSON metrics report'
        )
    ]

    # Tracker node configuration
    tracker_node = Node(
        package='hunter_control',
        executable='lqr_tracker',
        name='lqr_tracker',
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('csv_path'),
            'target_speed': LaunchConfiguration('target_speed'),
            'q_lateral': LaunchConfiguration('q_lateral'),
            'q_lateral_dot': LaunchConfiguration('q_lateral_dot'),
            'q_heading': LaunchConfiguration('q_heading'),
            'q_heading_dot': LaunchConfiguration('q_heading_dot'),
            'r_steering': LaunchConfiguration('r_steering'),
            'max_laps': LaunchConfiguration('max_laps'),
            'metrics_output_path': LaunchConfiguration('metrics_output_path'),
            'use_sim_time': True
        }]
    )

    return LaunchDescription(declared_arguments + [tracker_node])
