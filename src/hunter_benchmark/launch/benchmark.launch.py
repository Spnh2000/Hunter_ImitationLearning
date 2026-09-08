"""Full Phase 3 benchmark pass: Gazebo + trajectory_server + a tracker +
metrics_logger (+ optional rosbag2 recording), as one `ros2 launch` command.

Usage (run from the workspace root, matching this repo's documented
convention of `cd ~/sp_codes/hunter_robot` before `ros2 launch`):
  ros2 launch hunter_benchmark benchmark.launch.py \\
      track_file:=src/IMS/IMS_centerline_2x.csv

  ros2 launch hunter_benchmark benchmark.launch.py \\
      track_file:=src/IMS/IMS_centerline_2x.csv controller:=stanley

Use the *_2x* track files. IMS.sdf is generated at 2x scale (see
src/scale_track.py): F1TENTH circuits are drawn for ~0.3 m-long 1/10-scale
cars, and at 1x the Hunter had only 0.22 m of tracking margin at the tightest
corner -- less than its own odometry drift, so wall contact was arithmetic
rather than a tuning failure. At 2x that margin is 1.37 m. Pairing a 1x CSV
with the 2x world (or vice versa) puts the reference path nowhere near the
walls, so keep the suffixes matched.

The CENTERLINE is the reference line, not the raceline: the F1TENTH raceline
was optimized for a ~0.3 m-wide car and passes within ~0.05 m of the walls in
corners. trajectory_server synthesizes heading/curvature/speed for centerline
input, so nothing else changes.

  # Optional: spawn the robot from a different file than the one tracked
  # (e.g. spawn centered while evaluating a merge-on transient).
  ros2 launch hunter_benchmark benchmark.launch.py \\
      track_file:=src/IMS/IMS_raceline.csv \\
      spawn_reference_file:=src/IMS/IMS_centerline.csv

Phase 4+ trackers plug in via the `controller` argument alone -- this
launch file selects the tracker executable via the fixed
'<controller>_tracker' naming convention (all living in the hunter_control
package) and should not need to change (Phase 3 plan, reuse-risk callout #7).
"""
import math
import os
import re
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                             OpaqueFunction, SetLaunchConfiguration, TimerAction,
                             EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _read_first_waypoint(track_file: str):
    """Read just enough of a centerline/raceline CSV to get a spawn pose
    (x, y, yaw) aligned with the track's first point.

    Deliberately duplicates a small slice of trajectory_server's C++
    parsing logic -- this was the explicit, confirmed tradeoff in the
    Phase 3 plan's 'spawn bug fix' decision (launch-time Python parse,
    chosen over a more complex runtime-teleport-after-spawn approach).
    """
    is_raceline = 'raceline' in os.path.basename(track_file).lower()
    rows = []
    with open(track_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            rows.append(line)
            if len(rows) >= 2:
                break

    if is_raceline:
        fields = [v.strip() for v in rows[0].split(';')]
        return float(fields[1]), float(fields[2]), float(fields[3])

    fields0 = [v.strip() for v in rows[0].split(',')]
    x, y = float(fields0[0]), float(fields0[1])
    if len(rows) >= 2:
        fields1 = [v.strip() for v in rows[1].split(',')]
        yaw = math.atan2(float(fields1[1]) - y, float(fields1[0]) - x)
    else:
        yaw = 0.0
    return x, y, yaw


def _extent(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return max(max(xs) - min(xs), max(ys) - min(ys))


def _track_file_extent(track_file):
    """Bounding-box size of the reference path, in metres."""
    is_raceline = 'raceline' in os.path.basename(track_file).lower()
    pts = []
    with open(track_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if is_raceline:
                v = [float(t) for t in line.split(';')]
                pts.append((v[1], v[2]))
            else:
                v = [float(t) for t in line.split(',')]
                pts.append((v[0], v[1]))
    return _extent(pts) if pts else None


def _world_extent(world_path):
    """Bounding-box size of the world's wall models, in metres."""
    with open(world_path) as f:
        sdf = f.read()
    poses = re.findall(
        r'<model name="wall_[^"]*">.*?<pose>([-\d. e]+)</pose>', sdf, re.S)
    pts = []
    for p in poses:
        vals = p.split()
        pts.append((float(vals[0]), float(vals[1])))
    return _extent(pts) if pts else None


def _assert_scales_match(track_file, world_path):
    """Fail loudly if the reference CSV and the world SDF were built at
    different scales.

    This mismatch is otherwise silent and looks exactly like a controller
    bug: the robot tracks its (wrongly scaled) path perfectly while drifting
    steadily into a wall. Since src/scale_track.py introduced *_2x files
    alongside the 1x originals, launching with the wrong one is an easy
    mistake, so catch it before the robot moves.
    """
    track = _track_file_extent(track_file)
    world = _world_extent(world_path)
    if not track or not world:
        return
    ratio = track / world
    if not 0.8 < ratio < 1.25:
        raise RuntimeError(
            f'\n*** TRACK / WORLD SCALE MISMATCH ***\n'
            f'  track_file {track_file}\n'
            f'      spans {track:.1f} m\n'
            f'  world      {world_path}\n'
            f'      spans {world:.1f} m  (ratio {ratio:.2f}x)\n'
            f'  These must be generated at the same scale. If the world is 2x, use the\n'
            f'  *_2x.csv reference (see src/scale_track.py); if you regenerated the world\n'
            f'  at 1x, use the original CSV.\n')


# Which Odometry topic the tracker treats as its pose, and which frame the
# reference path is published in. Switching pose source is a pure remap --
# pid_tracker.cpp consumes nav_msgs/Odometry and never looks up TF (Phase 3
# reuse callout #6), so nothing here requires a code change.
POSE_SOURCES = {
    # Global EKF output: map-frame pose at the EKF's 50 Hz, with SLAM's
    # discrete corrections already absorbed. This is the deployable one.
    'slam_ekf': ('/odometry/filtered_map', 'map'),
    # Gazebo OdometryPublisher. Exact, has no hardware analogue; the A/B
    # reference and the yardstick for true cross-track error.
    'ground_truth': ('/ground_truth/odometry', 'odom'),
    # Phase 3 behaviour: raw dead reckoning. Drifts without bound. Kept as the
    # control condition -- it should FAIL the acceptance targets, which is how
    # we know the benchmark can detect the problem at all.
    'wheel_odom': ('/ackermann_like_controller/odometry', 'odom'),
}


def _configure_run(context, *args, **kwargs):
    """Resolves the spawn pose and the per-run results directory once,
    at launch time, from the other launch arguments' actual values."""
    track_file = LaunchConfiguration('track_file').perform(context)
    world_path = os.path.join(
        FindPackageShare('hunter_gazebo').perform(context), 'worlds',
        LaunchConfiguration('world').perform(context))
    _assert_scales_match(track_file, world_path)

    # slam_ekf without a SLAM stack silently degrades to dead reckoning, which
    # looks like a working run. Refuse it rather than report a fake result.
    pose_source = LaunchConfiguration('pose_source').perform(context)
    slam_mode = LaunchConfiguration('slam_mode').perform(context)
    if pose_source == 'slam_ekf' and slam_mode == 'disabled':
        raise RuntimeError(
            '\npose_source:=slam_ekf needs slam_mode:=localization (or mapping).\n'
            'With slam_mode:=disabled the global EKF gets no absolute measurement and\n'
            'propagates on wheel odometry alone -- a SLAM run in name only.\n')

    # Default the pose graph to THIS track, not whichever one was hardcoded.
    # Loading IMS's graph while driving Monza is silent: SLAM localizes against
    # geometry that is not there, and RViz shows the wrong map over the right world.
    # AMCL's map: the <Track>_map_2x.yaml sitting beside the track CSV. That is
    # the exact file my_convert_track.py turned into the world SDF, so the
    # occupancy grid and the Gazebo walls are the same geometry by construction.
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    if not map_yaml:
        track_dir = os.path.dirname(track_file)
        track_stem = os.path.basename(track_dir)
        candidate = os.path.join(track_dir, f'{track_stem}_map_2x.yaml')
        if os.path.exists(candidate):
            map_yaml = candidate

    map_file = LaunchConfiguration('map_file').perform(context)
    if not map_file:
        map_file = os.path.join(
            FindPackageShare('hunter_localization').perform(context),
            'maps', LaunchConfiguration('track_name').perform(context))
    spawn_reference_file = LaunchConfiguration('spawn_reference_file').perform(context)
    # A racing line is not centered between the walls by design (it cuts
    # toward apexes); spawning directly on it can start the robot close to
    # a wall. spawn_reference_file lets you spawn from a separate (typically
    # centerline) file while trajectory_server/the tracker still follow
    # track_file unchanged.
    x, y, yaw = _read_first_waypoint(spawn_reference_file or track_file)

    results_dir = LaunchConfiguration('results_dir').perform(context)
    track_name = LaunchConfiguration('track_name').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    pose_source = LaunchConfiguration('pose_source').perform(context)
    if pose_source not in POSE_SOURCES:
        raise RuntimeError(
            f"pose_source must be one of {sorted(POSE_SOURCES)}, got '{pose_source}'")
    pose_topic, path_frame = POSE_SOURCES[pose_source]

    # Tag the run directory with the pose source so an A/B comparison cannot
    # silently overwrite itself or be mixed up after the fact.
    run_dir = os.path.join(
        results_dir,
        f"{track_name}_{controller}_{pose_source}_{time.strftime('%Y%m%d_%H%M%S')}")

    # Where the reference path's frame origin sits, which differs per source:
    #   wheel_odom  -- the controller's odometry starts integrating at
    #                  (0,0,yaw=0) on activation, so the path must be shifted
    #                  by the spawn pose to meet it.
    #   slam_ekf    -- slam_toolbox initialises `map` at the robot's start
    #                  pose, the same convention, so the shift is identical.
    #   ground_truth-- Gazebo reports WORLD pose, which is already the CSV's
    #                  own frame, so no shift at all.
    path_origin = (0.0, 0.0, 0.0) if pose_source == 'ground_truth' else (x, y, yaw)

    return [
        SetLaunchConfiguration('spawn_x', str(x)),
        SetLaunchConfiguration('spawn_y', str(y)),
        SetLaunchConfiguration('spawn_yaw', str(yaw)),
        SetLaunchConfiguration('run_dir', run_dir),
        SetLaunchConfiguration('map_file', map_file),
        SetLaunchConfiguration('map_yaml', map_yaml),
        SetLaunchConfiguration('pose_topic', pose_topic),
        SetLaunchConfiguration('path_frame', path_frame),
        SetLaunchConfiguration('path_origin_x', str(path_origin[0])),
        SetLaunchConfiguration('path_origin_y', str(path_origin[1])),
        SetLaunchConfiguration('path_origin_yaw', str(path_origin[2])),
        # Ground truth must be moved by the SAME shift applied to the path, so
        # that the metrics logger compares them in one frame.
        SetLaunchConfiguration('gt_origin_x', str(path_origin[0])),
        SetLaunchConfiguration('gt_origin_y', str(path_origin[1])),
        SetLaunchConfiguration('gt_origin_yaw', str(path_origin[2])),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'track_file', default_value='src/IMS/IMS_centerline_2x.csv',
            description='Path to a centerline/raceline CSV. MUST match the scale the world SDF '
                        'was generated at -- the launch aborts if it does not. The 1x originals '
                        '(IMS_centerline.csv) are upstream F1TENTH reference data and do not '
                        'match the current 2x world.',
        ),
        DeclareLaunchArgument(
            'spawn_reference_file', default_value='',
            description='Optional: compute the spawn pose from this file instead of track_file '
                        '(e.g. the matching *_centerline.csv, which is centered between the walls '
                        'by definition, unlike a raceline). Empty = spawn from track_file itself.',
        ),
        DeclareLaunchArgument(
            'track_name', default_value='IMS',
            description='Label recorded in summary.csv (independent of track_file path)',
        ),
        DeclareLaunchArgument(
            'controller', default_value='pid',
            description="Tracker to run; selects the hunter_control '<controller>_tracker' executable",
        ),
        DeclareLaunchArgument('world', default_value='IMS.sdf'),
        DeclareLaunchArgument(
            'params_file', default_value='ims_defaults.yaml',
            description='Per-track params inside hunter_benchmark/config/ '
                        '(ims_defaults.yaml | monza_defaults.yaml). Must match track_file/world.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Set false for headless benchmark runs (no display).',
        ),
        DeclareLaunchArgument(
            # ground_truth by default: it is the only source that works with no
            # prerequisites (slam_ekf needs a pose graph built for THIS track first),
            # and it is what the Phase 6 expert drives on. Pair with the
            # slam_mode:=disabled default -- the two defaults must stay coherent.
            'pose_source', default_value='ground_truth',
            description="Which pose estimate the tracker consumes: 'slam_ekf' (LiDAR SLAM "
                        "+ dual EKF, the deployable stack), 'ground_truth' (exact Gazebo "
                        "pose, the A/B reference), or 'wheel_odom' (Phase 3 dead reckoning, "
                        "the control condition expected to fail). Pure remap -- no tracker "
                        "code changes between them.",
        ),
        DeclareLaunchArgument(
            'slam_mode', default_value='disabled',
            description="'disabled' (default) skips the whole localization stack -- most runs "
                        "do not need it. 'mapping' builds a new map; 'localization' runs "
                        "against a saved pose graph. Required non-disabled for "
                        "pose_source:=slam_ekf.",
        ),
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Serialized pose graph for slam_mode:=localization, without the '
                        '.posegraph extension. Empty = hunter_localization/maps/<track_name>.',
        ),
        DeclareLaunchArgument(
            'localization_backend', default_value='amcl',
            description="Localizer for slam_mode:=localization. 'amcl' (default) needs no "
                        "mapping run -- it localizes against the track occupancy grid that "
                        "generated the world. 'slam_toolbox' is currently broken.",
        ),
        DeclareLaunchArgument(
            'map_yaml', default_value='',
            description='Occupancy grid for AMCL. Empty = derived from track_file, i.e. '
                        'the <Track>_map_2x.yaml beside it.',
        ),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument(
            'record_il_dataset', default_value='false',
            description='If true, also records /scan and /ground_truth/odometry to build Offline RL datasets.'
        ),
        DeclareLaunchArgument(
            'results_dir', default_value='src/hunter_benchmark/results',
            description='Relative to the directory ros2 launch is invoked from (workspace root, '
                        'per this repo convention) -- must resolve under src/ so results survive '
                        'a container restart; build/install/log are container-only volumes.',
        ),
    ]

    # Per-track canonical params. Selectable so a multi-track collection run
    # (Phase 6) can switch tracks without editing this file: IMS and Monza need
    # different speed-profile settings because their curvature differs by an
    # order of magnitude.
    track_defaults = PathJoinSubstitution(
        [FindPackageShare('hunter_benchmark'), 'config',
         LaunchConfiguration('params_file')])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hunter_gazebo'), 'launch', 'launch_gz_sim.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': '0.3',
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    trajectory_server = Node(
        package='hunter_trajectory',
        executable='trajectory_server',
        name='trajectory_server',
        output='screen',
        parameters=[track_defaults, {
            'track_file': LaunchConfiguration('track_file'),
            # Path frame follows the pose source: 'map' when localizing (the
            # global EKF's output frame), 'odom' for ground truth and raw
            # wheel odometry. Overrides the track params file's frame_id.
            'frame_id': LaunchConfiguration('path_frame'),
            # The wheel odometry (the trackers' only pose source) starts
            # integrating from (0,0,yaw=0) at controller activation -- it cannot
            # know the world spawn pose. trajectory_server must therefore
            # transform the world-frame track CSV into the odom frame using the
            # same spawn pose this launch file just computed; without this the
            # tracker chases a path rotated by the spawn yaw and drives straight
            # into the wall.
            # Zero for ground_truth (Gazebo already reports world pose, the
            # CSV's own frame); the spawn pose otherwise. See _configure_run.
            'odom_origin_x_m': ParameterValue(
                LaunchConfiguration('path_origin_x'), value_type=float),
            'odom_origin_y_m': ParameterValue(
                LaunchConfiguration('path_origin_y'), value_type=float),
            'odom_origin_yaw_rad': ParameterValue(
                LaunchConfiguration('path_origin_yaw'), value_type=float),
        }],
    )

    tracker = Node(
        package='hunter_control',
        executable=[LaunchConfiguration('controller'), '_tracker'],
        name=[LaunchConfiguration('controller'), '_tracker'],
        output='screen',
        parameters=[track_defaults, {'controller_name': LaunchConfiguration('controller')}],
        # The entire A/B switch. The tracker subscribes to
        # /ackermann_like_controller/odometry by name in pid_tracker.cpp; this
        # remap points that subscription at whichever estimator is under test.
        remappings=[('/ackermann_like_controller/odometry',
                     LaunchConfiguration('pose_topic'))],
    )

    # Gated on slam_mode, NOT pose_source, so a map can be built while driving
    # on a pose source other than SLAM. That matters for map quality: building
    # the map while tracking ground truth gives clean, wall-free motion,
    # whereas mapping while driving on the drifting wheel odometry that SLAM
    # is meant to replace risks a collision corrupting the very map every
    # later run depends on.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hunter_localization'),
                                  'launch', 'localization.launch.py'])
        ),
        launch_arguments={
            'slam_mode': LaunchConfiguration('slam_mode'),
            'map_file': LaunchConfiguration('map_file'),
            'localization_backend': LaunchConfiguration('localization_backend'),
            'map_yaml': LaunchConfiguration('map_yaml'),
            # AMCL is seeded with the SAME spawn pose Gazebo used, so it starts
            # localized instead of trying global localization on a racetrack --
            # a long corridor of repetitive geometry where that is unreliable.
            'initial_pose_x': LaunchConfiguration('spawn_x'),
            'initial_pose_y': LaunchConfiguration('spawn_y'),
            'initial_pose_yaw': LaunchConfiguration('spawn_yaw'),
            'use_sim_time': 'true',
        }.items(),
        condition=LaunchConfigurationNotEquals('slam_mode', 'disabled'),
    )

    # Ground-truth bridge for runs where the localization stack is not started
    # (slam_mode:=disabled) -- otherwise localization.launch.py provides it.
    # metrics_logger ALWAYS needs /ground_truth/odometry to measure true
    # cross-track error; without it the benchmark can only report the
    # estimator's opinion of itself, which is precisely the number that read
    # 1 mm while the robot was 2.29 m off-line.
    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/model/hunter/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/hunter/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            ('/model/hunter/odometry', '/ground_truth/odometry'),
            ('/model/hunter/pose', '/tf')
        ],
        condition=LaunchConfigurationEquals('slam_mode', 'disabled'),
    )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge_for_il',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        condition=IfCondition(LaunchConfiguration('record_il_dataset')),
    )

    metrics_logger = Node(
        package='hunter_benchmark',
        executable='metrics_logger.py',
        name='metrics_logger',
        output='screen',
        parameters=[{
            'track_name': LaunchConfiguration('track_name'),
            'controller_name': LaunchConfiguration('controller'),
            'results_dir': LaunchConfiguration('run_dir'),
            'pose_source': LaunchConfiguration('pose_source'),
            # Gazebo's OdometryPublisher reports the model's WORLD pose, while
            # trajectory_server shifts the path into a spawn-relative frame.
            # Applying the same spawn pose here brings ground truth into the
            # path's frame so the two are comparable.
            'gt_origin_x_m': ParameterValue(LaunchConfiguration('gt_origin_x'), value_type=float),
            'gt_origin_y_m': ParameterValue(LaunchConfiguration('gt_origin_y'), value_type=float),
            'gt_origin_yaw_rad': ParameterValue(
                LaunchConfiguration('gt_origin_yaw'), value_type=float),
        }],
        # See the tracker remap: the logger must measure the same estimate the
        # controller consumed, or estimator_error compares unlike things.
        remappings=[('/ackermann_like_controller/odometry',
                     LaunchConfiguration('pose_topic'))],
    )

    def _build_bag_record(context):
        cmd = [
            'ros2', 'bag', 'record', '--storage', 'mcap',
            '-o', PathJoinSubstitution([LaunchConfiguration('run_dir'), 'bag']).perform(context),
            '/trajectory/reference', '/tracking/diagnostics',
            '/ackermann_like_controller/odometry', '/ackermann_like_controller/reference',
        ]
        
        if LaunchConfiguration('record_il_dataset').perform(context).lower() == 'true':
            cmd.extend(['/scan', '/ground_truth/odometry'])
            
        return [ExecuteProcess(
            cmd=cmd,
            output='screen',
            condition=IfCondition(LaunchConfiguration('record_bag')),
        )]

    bag_record = OpaqueFunction(function=_build_bag_record)

    # Fixed delay rather than cross-launch-file event wiring into
    # launch_gz_sim.launch.py's internal OnProcessExit chain (spawn ->
    # joint_state_broadcaster -> ackermann_like_controller): simpler, and
    # this launch file is run manually per benchmark pass, so a few extra
    # seconds of margin matters less than not touching the included file.
    benchmark_stack = TimerAction(
        period=10.0, actions=[trajectory_server, tracker, metrics_logger, bag_record])

    # Localization starts earlier than the tracker: the EKFs need odometry and
    # IMU flowing, and slam_toolbox needs odom -> base_link to exist, before
    # the tracker's first control tick asks for a pose. The 5 s / 10 s split
    # gives the estimators a head start rather than racing them.
    localization_stack = TimerAction(
        period=5.0, actions=[localization, ground_truth_bridge, sensor_bridge])

    shutdown_on_logger_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=metrics_logger,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=_configure_run),
        gazebo,
        localization_stack,
        benchmark_stack,
        shutdown_on_logger_exit,
    ])
