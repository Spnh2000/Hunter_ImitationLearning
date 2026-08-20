"""Localization stack: sensor bridges + slam_toolbox + dual EKF.

Frame ownership, which is the part that breaks silently if it is wrong:

    map --(slam_toolbox)--> odom --(local EKF)--> base_link --(URDF)--> laser

  * map -> odom        slam_toolbox
  * odom -> base_link  ekf_filter_node_odom     (NOT the wheel controller;
                       ackermann_like_controller.yaml sets enable_odom_tf: false)
  * base_link -> laser robot_state_publisher    (static, from sensors.xacro)

The global EKF publishes NO TF. slam_toolbox cannot emit /slam_toolbox/pose
without also emitting the map -> odom TF -- one timer drives both, and its own
config notes "if 0 never publishes odometry". Since that pose is the global
EKF's only absolute measurement, slam_toolbox has to keep its TF, so the two
split roles instead:

  * TF map -> odom          raw SLAM, carries discrete correction steps
  * /odometry/filtered_map  50 Hz, corrections absorbed by the filter

The tracker consumes the latter as nav_msgs/Odometry and never looks up TF
(Phase 3 reuse callout #6), so it gets SLAM's accuracy with the EKF's
smoothness, while TF consumers such as RViz still see what SLAM actually
believes. Exactly one publisher of map -> odom either way.

Usage:
  # stand-alone, alongside an already-running sim
  ros2 launch hunter_localization localization.launch.py slam_mode:=mapping

  # normally included by hunter_benchmark/launch/benchmark.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _bridges(use_sim_time):
    """ros_gz_bridge entries for the sensors and the ground-truth reference.

    /scan and /imu carry gz_frame_id set in sensors.xacro; without those the
    bridged messages arrive with an empty frame_id and every TF-dependent
    consumer (slam_toolbox first) fails in a way that looks like a TF bug.
    """
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sensor_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
        ),
        # Ground-truth pose from the OdometryPublisher system in sensors.xacro.
        # Remapped to /ground_truth/odometry so its role is unmistakable: it is
        # the A/B reference and the yardstick for true cross-track error, never
        # an input to the deployed estimator.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ground_truth_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['/model/hunter/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
            remappings=[('/model/hunter/odometry', '/ground_truth/odometry')],
        ),
    ]


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('hunter_localization')
    slam_mode = LaunchConfiguration('slam_mode').perform(context)
    map_file = LaunchConfiguration('map_file').perform(context)
    backend = LaunchConfiguration('localization_backend').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    init_x = LaunchConfiguration('initial_pose_x').perform(context)
    init_y = LaunchConfiguration('initial_pose_y').perform(context)
    init_yaw = LaunchConfiguration('initial_pose_yaw').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    if slam_mode not in ('mapping', 'localization', 'disabled'):
        raise RuntimeError(
            f"slam_mode must be 'mapping', 'localization' or 'disabled', got '{slam_mode}'")

    actions = list(_bridges(use_sim_time))

    # Local EKF always runs: it owns odom -> base_link, which slam_toolbox
    # needs as its motion prior. Starting SLAM without it means SLAM has no
    # odom frame to work against.
    actions.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf_local.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/filtered')],
    ))

    # AMCL path: localize against the track's occupancy grid instead of a SLAM
    # pose graph. That grid is the SAME file my_convert_track.py consumed to
    # build the Gazebo world, so no mapping run is needed and the map frame
    # coincides with the world frame by construction.
    use_amcl = (slam_mode == 'localization' and backend == 'amcl')

    if use_amcl:
        if not map_yaml:
            raise RuntimeError(
                'localization_backend:=amcl requires map_yaml (the track occupancy '
                'grid, e.g. src/Monza/Monza_map_2x.yaml)')
        amcl_cfg = os.path.join(pkg, 'config', 'amcl.yaml')
        actions.append(Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[amcl_cfg, {'use_sim_time': use_sim_time,
                                   'yaml_filename': os.path.abspath(map_yaml)}],
        ))
        actions.append(Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[amcl_cfg, {
                'use_sim_time': use_sim_time,
                # Seed the true start: AMCL is used for TRACKING here, not
                # global localization. A racetrack is a long corridor of
                # repetitive geometry where kidnapped-robot recovery is
                # unreliable, so never make it guess.
                'set_initial_pose': True,
                'initial_pose.x': float(init_x),
                'initial_pose.y': float(init_y),
                'initial_pose.yaw': float(init_yaw),
            }],
        ))
        # map_server and amcl are lifecycle nodes; nothing happens until they
        # are configured and activated. Same trap slam_toolbox set.
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['map_server', 'amcl'], 'bond_timeout': 0.0}],
        ))
        actions.append(Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node_map', output='screen',
            parameters=[os.path.join(pkg, 'config', 'ekf_global.yaml'),
                        {'use_sim_time': use_sim_time,
                         # AMCL's output replaces slam_toolbox's as the global
                         # filter's only absolute measurement.
                         'pose0': '/amcl_pose'}],
            remappings=[('odometry/filtered', 'odometry/filtered_map')],
        ))
        return actions

    if slam_mode != 'disabled':
        cfg = 'slam_mapping.yaml' if slam_mode == 'mapping' else 'slam_localization.yaml'
        slam_params = [os.path.join(pkg, 'config', cfg), {'use_sim_time': use_sim_time}]
        if slam_mode == 'localization':
            # slam_toolbox wants the path WITHOUT the .posegraph suffix.
            # No IMS fallback: benchmark.launch.py resolves this from track_name.
            # A hardcoded default here silently loads the wrong track's map.
            if not map_file:
                raise RuntimeError(
                    'slam_mode:=localization requires map_file (pose graph path, no extension)')
            resolved = map_file
            slam_params.append({'map_file_name': resolved.replace('.posegraph', '')})

        # slam_toolbox is a LIFECYCLE node in Jazzy. Started as a plain Node it
        # comes up UNCONFIGURED and stays there silently: the process runs, the
        # topics appear (created by subscribers), but /slam_toolbox/pose has
        # zero publishers and /map is never produced. Downstream that looks
        # like healthy dead reckoning, because the global EKF simply never
        # receives an absolute measurement -- the exact failure this phase
        # exists to eliminate, wearing a disguise.
        #
        # Transition sequence mirrors slam_toolbox's own online_async_launch.py:
        #   unconfigured --CONFIGURE--> inactive --ACTIVATE--> active
        # The executable differs per mode, and picking the wrong one fails
        # silently. Only localization_slam_toolbox_node publishes
        # /slam_toolbox/pose (async_slam_toolbox_node has no such code path at
        # all), and that pose is the global EKF's ONLY absolute measurement.
        # Running async in localization mode leaves the EKF propagating on
        # wheel odometry and gyro forever, so pose_source:=slam_ekf would
        # quietly be dead reckoning wearing a SLAM label.
        slam_executable = ('async_slam_toolbox_node' if slam_mode == 'mapping'
                           else 'localization_slam_toolbox_node')
        slam_node = LifecycleNode(
            package='slam_toolbox',
            executable=slam_executable,
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=slam_params + [{'use_lifecycle_manager': False}],
        )
        actions.append(slam_node)
        actions.append(EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )))
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[hunter_localization] slam_toolbox configured; activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )))

        # Global EKF only makes sense with SLAM feeding it: with slam_mode
        # disabled there is no absolute reference, so map -> odom would be a
        # fixed identity and running it would only add confusion.
        actions.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'ekf_global.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/filtered_map')],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_mode', default_value='localization',
            description="'mapping' to build a new map, 'localization' to run against a "
                        "saved pose graph (the Phase 6 default), 'disabled' for local EKF "
                        "and bridges only",
        ),
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Serialized pose graph for localization mode, without the '
                        '.posegraph extension. Empty = <pkg>/maps/IMS',
        ),
        DeclareLaunchArgument(
            'localization_backend', default_value='amcl',
            description="Which localizer runs when slam_mode:=localization. 'amcl' "
                        "(default) localizes against the track occupancy grid and needs "
                        "no mapping run. 'slam_toolbox' uses a serialized pose graph and "
                        "is currently BROKEN -- it activates but never publishes "
                        "/slam_toolbox/pose, leaving the global EKF on dead reckoning.",
        ),
        DeclareLaunchArgument(
            'map_yaml', default_value='',
            description='Occupancy grid YAML for localization_backend:=amcl, e.g. '
                        'src/Monza/Monza_map_2x.yaml. Must be the same map the world '
                        'SDF was generated from.',
        ),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
