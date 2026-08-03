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
            # default_value='-r -s -v 4 empty.sdf',
            default_value='-r -v 4 empty.sdf',
            description='Arguments forwarded to gz sim (world SDF, verbosity, etc.)',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Force gz-transport onto loopback so server/GUI/create all find each other
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    # Add Conda env library directory to Gazebo search path for macOS compatibility
    # conda_prefix = os.environ.get('CONDA_PREFIX', '')
    # current_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    # if conda_prefix:
    #     lib_path = os.path.join(conda_prefix, 'lib')
    #     gz_plugin_path = f"{current_plugin_path}:{lib_path}" if current_plugin_path else lib_path
    # else:
    #     gz_plugin_path = current_plugin_path
    # set_gz_plugin_path = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_plugin_path)

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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    load_ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_like_controller'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
        # set_gz_plugin_path,
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
