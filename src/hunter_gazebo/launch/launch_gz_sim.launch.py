import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_hunter_gazebo = get_package_share_directory('hunter_gazebo')
    worlds_dir = os.path.join(pkg_hunter_gazebo, 'worlds')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='IMS.sdf',
            description='World SDF filename inside hunter_gazebo/worlds/',
        ),
        DeclareLaunchArgument(
            'spawn_x', default_value='0',
            description='Robot spawn X (metres) — midpoint of outer/inner wall pair on top straight',
        ),
        DeclareLaunchArgument(
            'spawn_y', default_value='0',
            description='Robot spawn Y (metres)',
        ),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.3',
            description='Robot spawn Z (metres) — keep ≥0.3 to clear the ground',
        ),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='-1.57',
            description='Robot spawn yaw (radians) — aligned with the top straight direction',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Force gz-transport onto loopback so server/GUI/create all find each other
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    # Add Conda env library directory to Gazebo search path for macOS compatibility
    conda_prefix = os.environ.get('CONDA_PREFIX', '')
    current_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if conda_prefix:
        lib_path = os.path.join(conda_prefix, 'lib')
        gz_plugin_path = f"{current_plugin_path}:{lib_path}" if current_plugin_path else lib_path
    else:
        gz_plugin_path = current_plugin_path
    set_gz_plugin_path = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_plugin_path)

    # Pass the full absolute path so Gazebo doesn't need GZ_SIM_RESOURCE_PATH.
    # Concatenate worlds_dir (resolved at description-generation time) with the
    # world arg (resolved at launch time).
    gz_args = [f'-r -s -v 4 {worlds_dir}/', LaunchConfiguration('world')]

    # ── Gazebo Harmonic ──────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # ── Robot description ────────────────────────────────────────────────────
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

    # ── Spawn entity in Gazebo Harmonic ──────────────────────────────────────
    # Delayed so Gazebo has time to finish loading the Monza world before the
    # entity-factory service is called. 5 s is usually enough; increase if
    # Gazebo is slow on first run.
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

    spawn_after_gazebo = TimerAction(period=5.0, actions=[spawn_entity])

    # ── Clock bridge (gz → ROS) ──────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── Controllers ──────────────────────────────────────────────────────────
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
        set_gz_plugin_path,
        *declared_arguments,
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
        spawn_after_gazebo,
        rviz,
    ])
