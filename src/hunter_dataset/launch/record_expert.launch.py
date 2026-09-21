"""Phase 6 expert data collection: drive a lap and bag everything.

Wraps benchmark.launch.py rather than reimplementing the stack. That is not
tidiness -- benchmark.launch.py holds the frame logic that decides whether the
reference path is published in world or spawn-relative coordinates, and a
hand-rolled equivalent got that wrong during Step 2 testing (the robot drove
into a wall in seconds because the path was rotated 88 degrees). Anything that
drives the robot goes through the launch file that owns that decision.

Topology, which matters for the contact stream:

    gz sensor --bridge--> /bumper_contacts --> episode_monitor  (full rate, detection)
                                           \\-> topic_tools drop
                                                --> /bumper_contacts_bagged (~20 Hz, recorder)

The throttle sits between the bridge and the recorder, never between the
sensor and the monitor: detection needs every contact, the bag does not.
Contact information above the dataset rate can never reach a dataset row, so
throttling to 20 Hz discards nothing recoverable -- while untrottled it is
~326 Hz (Gazebo ignores a contact sensor's update_rate) and ~33 MB per lap.

A run_manifest.yaml is written BEFORE the run starts, so a crashed lap still
tells you what configuration produced its partial bag.
"""
import hashlib
import os
import subprocess
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Bagged raw. The dataset exposes a subset; the bag keeps what the schema did
# not anticipate, because that is the whole point of the two-stage design.
BAG_TOPICS = [
    '/scan',                                  # student observation, full 1080
    '/tracking/diagnostics',                  # expert action (post-clamp) + progress
    '/ground_truth/odometry',                 # teacher-side privilege
    '/ackermann_like_controller/reference',   # executed command, cross-check
    '/ackermann_like_controller/odometry_bagged',  # raw wheel odom, 1 kHz -> 50 Hz
    '/odometry/filtered',                     # local EKF -- what a deployable
                                              #   estimator would have said
    '/episode/event',                         # terminals, RECORDED not re-derived
    '/bumper_contacts_bagged',                # throttled contact detail
    '/imu',
    '/trajectory/reference',                  # the path itself (latched, one msg).
                                              #   Without it no path-relative
                                              #   feature can be recomputed offline.
    '/tf_static',                             # sensor extrinsics
    '/robot_description',                     # URDF: wheelbase, steering limits
]


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def _git(args, cwd):
    try:
        return subprocess.run(['git'] + args, cwd=cwd, capture_output=True,
                              text=True, timeout=5).stdout.strip()
    except Exception:
        return ''


def _setup(context, *args, **kwargs):
    track = LaunchConfiguration('track_name').perform(context)
    track_file = LaunchConfiguration('track_file').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    out_root = LaunchConfiguration('output_dir').perform(context)
    run_id = LaunchConfiguration('run_id').perform(context) or time.strftime('%Y%m%d_%H%M%S')

    max_speed = LaunchConfiguration('max_speed_mps').perform(context)
    run_dir = os.path.join(out_root, f'{track}_{max_speed}ms_{run_id}')
    os.makedirs(run_dir, exist_ok=True)

    # --- manifest FIRST -----------------------------------------------------
    # Written before anything starts. A run that dies mid-lap still leaves a
    # complete record of the configuration that produced its partial bag.
    repo = os.path.abspath(os.path.join(
        get_package_share_directory('hunter_dataset'), '..', '..', '..', '..'))
    cfg_path = os.path.join(
        get_package_share_directory('hunter_benchmark'), 'config', params_file)
    effective = {}
    if os.path.exists(cfg_path):
        with open(cfg_path) as f:
            effective = yaml.safe_load(f) or {}

    # --- max_speed_mps must be a SETTING, not a label ------------------------
    # The track params file says 2.7 for every track. Overriding it here, in a
    # file that benchmark.launch.py then loads, is the only way a run named
    # "<track>_1.5ms_<id>" is guaranteed to have driven at 1.5. Before this,
    # the argument reached the manifest and the directory name and nothing
    # else -- every speed setting would have driven at 2.7 and been labelled
    # otherwise, which no inspection of the dataset alone can detect.
    for node in ('trajectory_server', 'pid_tracker'):
        effective.setdefault(node, {}).setdefault('ros__parameters', {})['max_speed_mps'] = float(max_speed)
    effective_path = os.path.join(run_dir, 'effective_params.yaml')
    with open(effective_path, 'w') as f:
        yaml.safe_dump(effective, f, sort_keys=False)

    dirty = _git(['status', '--porcelain'], repo)
    manifest = {
        'run_id': run_id,
        'track_id': track,
        'track_file': track_file,
        # A track_id string is an assertion; the hash is verifiable against the
        # geometry actually driven.
        'track_sha256': _sha256(track_file) if os.path.exists(track_file) else None,
        'max_speed_mps': float(max_speed),
        'spawn_s_frac': float(LaunchConfiguration('spawn_s_frac').perform(context)),
        'controller_label': LaunchConfiguration('controller').perform(context),
        'pose_source': 'ground_truth',
        'world': LaunchConfiguration('world').perform(context),
        'params_file': params_file,
        'effective_params': effective,
        'git_rev': _git(['rev-parse', 'HEAD'], repo),
        # Non-empty working tree means git_rev does NOT fully describe the code
        # that produced this data.
        'git_dirty': bool(dirty),
        'git_dirty_files': dirty.splitlines()[:50],
        'bag_topics': BAG_TOPICS,
        'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    }
    with open(os.path.join(run_dir, 'run_manifest.yaml'), 'w') as f:
        yaml.safe_dump(manifest, f, sort_keys=False)

    return [
        SetLaunchConfiguration('run_dir', run_dir),
        SetLaunchConfiguration('bag_path', os.path.join(run_dir, 'bag')),
        # Absolute path: benchmark's PathJoinSubstitution yields it unchanged.
        SetLaunchConfiguration('effective_params_file', effective_path),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('track_name', default_value='Monza'),
        DeclareLaunchArgument('track_file', default_value='src/Monza/Monza_centerline_2x.csv'),
        DeclareLaunchArgument('params_file', default_value='monza_defaults.yaml'),
        DeclareLaunchArgument('world', default_value='Monza.sdf'),
        DeclareLaunchArgument('controller', default_value='pid'),
        DeclareLaunchArgument('max_speed_mps', default_value='2.7'),
        DeclareLaunchArgument('spawn_s_frac', default_value='0.0'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('output_dir', default_value='datasets/raw'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
    ]

    benchmark = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hunter_benchmark'),
                                  'launch', 'benchmark.launch.py'])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'track_file': LaunchConfiguration('track_file'),
            # The per-run copy with max_speed_mps applied, NOT the track file.
            'params_file': LaunchConfiguration('effective_params_file'),
            'track_name': LaunchConfiguration('track_name'),
            'controller': LaunchConfiguration('controller'),
            # The expert cheats: ground truth is a teacher-side privilege and
            # never enters the student's observation space.
            'pose_source': 'ground_truth',
            'slam_mode': 'disabled',
            'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'spawn_s_frac': LaunchConfiguration('spawn_s_frac'),
            # This launch owns the bag; benchmark's 4-topic version would
            # otherwise write a second, smaller one.
            'record_bag': 'false',
        }.items(),
    )

    contact_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='contact_bridge',
        output='screen', parameters=[{'use_sim_time': True}],
        arguments=['/world/track_world/model/hunter/link/base_link/sensor/bumper/contact'
                   '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
        remappings=[('/world/track_world/model/hunter/link/base_link/sensor/bumper/contact',
                     '/bumper_contacts')],
    )

    # /scan and /imu are bridged by benchmark.launch.py (slam_mode:=disabled
    # branch). A second bridge here would deliver every scan twice: N = 2 x
    # 20 x duration, duplicate stamps, and an exact-match join that still
    # passes. Exactly one bridge per topic, always.

    # 1 of every 16 at ~326 Hz lands near the 20 Hz dataset rate.
    contact_throttle = Node(
        package='topic_tools', executable='drop', name='contact_throttle',
        output='screen', parameters=[{'use_sim_time': True}],
        arguments=['/bumper_contacts', '15', '16', '/bumper_contacts_bagged'],
    )

    # Wheel odometry is published at the 1 kHz ros2_control update rate and
    # was 75 % of every bag by message count (~280 of 400 MB per lap). The EKF
    # consumes the raw stream live; the bagged copy is 50 Hz like the other
    # odometry streams. Same pattern as the contact throttle: downstream of the
    # producer, so live consumers are untouched.
    wheel_odom_throttle = Node(
        package='topic_tools', executable='drop', name='wheel_odom_throttle',
        output='screen', parameters=[{'use_sim_time': True}],
        arguments=['/ackermann_like_controller/odometry', '19', '20',
                   '/ackermann_like_controller/odometry_bagged'],
    )

    episode_monitor = Node(
        package='hunter_control', executable='episode_monitor', name='episode_monitor',
        output='screen',
        parameters=[{'use_sim_time': True, 'scan_contact_m': 0.4, 'latch_until_reset': True}],
    )

    # Local EKF only -- no SLAM. Gives /odometry/filtered so a later analysis can
    # ask what a deployable estimator WOULD have reported during expert driving.
    local_ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('hunter_localization'),
                                          'config', 'ekf_local.yaml']),
                    {'use_sim_time': True}],
    )

    bag = ExecuteProcess(
        # mcap chunk compression (Zstd) is native to the format: any mcap
        # reader, rosbag2_py included, decompresses transparently, unlike
        # rosbag2's file-level --compression-mode which needs its own plugin
        # on read and renames the file. Measured 6x on an uncompressed lap.
        cmd=['ros2', 'bag', 'record', '--storage', 'mcap',
             '--storage-config-file',
             os.path.join(get_package_share_directory('hunter_dataset'),
                          'config', 'mcap_zstd.yaml'),
             '-o', LaunchConfiguration('bag_path')] + BAG_TOPICS,
        output='screen',
    )

    return LaunchDescription([
        *args,
        OpaqueFunction(function=_setup),
        benchmark,
        contact_bridge,
        contact_throttle,
        wheel_odom_throttle,
        episode_monitor,
        local_ekf,
        bag,
    ])
