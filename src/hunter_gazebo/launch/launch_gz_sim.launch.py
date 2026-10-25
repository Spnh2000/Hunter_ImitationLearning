import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
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
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI). Use for benchmark/CI runs.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz. Set false for headless benchmark runs.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='IMS.sdf',
            description='World SDF filename inside hunter_gazebo/worlds/',
        ),
        # IMS track start: first centerline point of IMS_centerline_2x.csv (0, 0, yaw=-1.5506 rad)
        DeclareLaunchArgument(
            'spawn_delay', default_value='',
            description='Seconds to wait before spawning the robot. Empty = 5 s headless, '
                        '20 s with the GUI, because Gazebo takes much longer to be ready '
                        'when it has a window to draw.'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Robot spawn X [m]',
        ),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Robot spawn Y [m]',
        ),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.3',
            description='Robot spawn Z [m] — keep ≥ 0.3 to clear the ground',
        ),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='-1.5506',
            description='Robot spawn yaw [rad] — -1.5506 ≈ south, aligned with IMS start',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Force gz-transport onto loopback so server/GUI/create all find each other
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    # Pass the full absolute path so Gazebo doesn't need GZ_SIM_RESOURCE_PATH.
    # Concatenate worlds_dir (resolved at description-generation time) with the
    # world arg (resolved at launch time).
    # '-s' (server only) when headless: without it Gazebo starts its full GUI,
    # which on a machine with no display wastes CPU at best and takes the
    # launch down at worst. Benchmark runs should always pass headless:=true.
    gz_args = [
        PythonExpression(["'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]),
        f'-r -v 4 {worlds_dir}/', LaunchConfiguration('world'),
    ]

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
    # Delayed so Gazebo has time to finish loading the IMS world before the
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

    # 5 s is enough for a server-only start; the full GUI needs far longer to
    # load the world, render it and bring up the entity-factory service. Spawning
    # too early fails with no robot in the world and no obvious error -- which is
    # exactly what a GUI run looked like. Overridable with spawn_delay.
    spawn_delay = PythonExpression([
        "'", LaunchConfiguration('spawn_delay'), "' if '", LaunchConfiguration('spawn_delay'),
        "' != '' else ('5.0' if '", LaunchConfiguration('headless'), "' == 'true' else '20.0')"])
    spawn_after_gazebo = TimerAction(period=spawn_delay, actions=[spawn_entity])

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
    # Skippable via use_rviz:=false. Headless benchmark runs have no display,
    # and RViz failing to open a window can take the whole launch down with it.
    hunter_description_path = get_package_share_directory('hunter_description')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(hunter_description_path, 'rviz/robot_view.rviz')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        set_gz_ip,
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
        # RViz starts only once the robot is actually in the world. Started
        # concurrently it comes up before robot_description and TF exist, so the
        # robot model is missing and the displays complain about topics that do
        # not have publishers yet.
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=3.0, actions=[rviz])])
        ),
        gazebo,
        bridge,
        node_robot_state_publisher,
        spawn_after_gazebo,
    ])
