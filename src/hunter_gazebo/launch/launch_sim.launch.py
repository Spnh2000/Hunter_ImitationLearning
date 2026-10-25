import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r -v 4 empty.sdf',
            description='Arguments forwarded to gz sim (world SDF, verbosity, etc.)',
        ),
        # Spawn position — set these when using a track world (e.g. IMS)
        DeclareLaunchArgument('spawn_x',   default_value='-0.656', description='Robot spawn X [m]'),
        DeclareLaunchArgument('spawn_y',   default_value='0.142',  description='Robot spawn Y [m]'),
        DeclareLaunchArgument('spawn_z',   default_value='0.15',   description='Robot spawn Z [m]'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.503',  description='Robot spawn yaw [rad]'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Force gz-transport onto loopback so server/GUI/create all find each other
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    # ── Gazebo Harmonic ──────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

    # ── Robot description (Harmonic gz_ros2_control plugin) ──────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('hunter_description'), 'description', 'robot.urdf.xacro']
        ),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ── Spawn robot in Gazebo Harmonic ───────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'hunter',
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Clock bridge (gz → ROS) ──────────────────────────────────────────────
    # All other topics (odom, joint_states, tf, cmd_vel) are handled by
    # gz_ros2_control + tricycle_controller directly on the ROS 2 side.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── Controllers (spawned after robot is in Gazebo) ───────────────────────
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_like_controller'],
        output='screen',
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    hunter_description_path = get_package_share_directory('hunter_description')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(hunter_description_path, 'rviz/robot_view.rviz')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        set_gz_ip,
        *declared_arguments,
        # Chain: spawn → joint_state_broadcaster → ackermann_controller
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_ackermann_controller],
            )
        ),
        gazebo,
        bridge,
        node_robot_state_publisher,
        spawn_entity,
        rviz,
    ])
