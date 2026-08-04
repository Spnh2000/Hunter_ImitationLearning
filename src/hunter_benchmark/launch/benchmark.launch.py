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
                             OpaqueFunction, SetLaunchConfiguration, TimerAction)
from launch.conditions import IfCondition
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


def _configure_run(context, *args, **kwargs):
    """Resolves the spawn pose and the per-run results directory once,
    at launch time, from the other launch arguments' actual values."""
    track_file = LaunchConfiguration('track_file').perform(context)
    world_path = os.path.join(
        FindPackageShare('hunter_gazebo').perform(context), 'worlds',
        LaunchConfiguration('world').perform(context))
    _assert_scales_match(track_file, world_path)
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
    run_dir = os.path.join(
        results_dir, f"{track_name}_{controller}_{time.strftime('%Y%m%d_%H%M%S')}")

    return [
        SetLaunchConfiguration('spawn_x', str(x)),
        SetLaunchConfiguration('spawn_y', str(y)),
        SetLaunchConfiguration('spawn_yaw', str(yaw)),
        SetLaunchConfiguration('run_dir', run_dir),
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
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument(
            'results_dir', default_value='src/hunter_benchmark/results',
            description='Relative to the directory ros2 launch is invoked from (workspace root, '
                        'per this repo convention) -- must resolve under src/ so results survive '
                        'a container restart; build/install/log are container-only volumes.',
        ),
    ]

    ims_defaults = PathJoinSubstitution(
        [FindPackageShare('hunter_benchmark'), 'config', 'ims_defaults.yaml'])

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
        }.items(),
    )

    trajectory_server = Node(
        package='hunter_trajectory',
        executable='trajectory_server',
        name='trajectory_server',
        output='screen',
        parameters=[ims_defaults, {
            'track_file': LaunchConfiguration('track_file'),
            # The wheel odometry (the trackers' only pose source) starts
            # integrating from (0,0,yaw=0) at controller activation -- it cannot
            # know the world spawn pose. trajectory_server must therefore
            # transform the world-frame track CSV into the odom frame using the
            # same spawn pose this launch file just computed; without this the
            # tracker chases a path rotated by the spawn yaw and drives straight
            # into the wall.
            'odom_origin_x_m': ParameterValue(LaunchConfiguration('spawn_x'), value_type=float),
            'odom_origin_y_m': ParameterValue(LaunchConfiguration('spawn_y'), value_type=float),
            'odom_origin_yaw_rad': ParameterValue(LaunchConfiguration('spawn_yaw'), value_type=float),
        }],
    )

    tracker = Node(
        package='hunter_control',
        executable=[LaunchConfiguration('controller'), '_tracker'],
        name=[LaunchConfiguration('controller'), '_tracker'],
        output='screen',
        parameters=[ims_defaults, {'controller_name': LaunchConfiguration('controller')}],
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
        }],
    )

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--storage', 'mcap',
            '-o', PathJoinSubstitution([LaunchConfiguration('run_dir'), 'bag']),
            '/trajectory/reference', '/tracking/diagnostics',
            '/ackermann_like_controller/odometry', '/ackermann_like_controller/reference',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag')),
    )

    # Fixed delay rather than cross-launch-file event wiring into
    # launch_gz_sim.launch.py's internal OnProcessExit chain (spawn ->
    # joint_state_broadcaster -> ackermann_like_controller): simpler, and
    # this launch file is run manually per benchmark pass, so a few extra
    # seconds of margin matters less than not touching the included file.
    benchmark_stack = TimerAction(
        period=10.0, actions=[trajectory_server, tracker, metrics_logger, bag_record])

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=_configure_run),
        gazebo,
        benchmark_stack,
    ])
