# Phase 1 Validation: Hunter V2 ROS 2, RViz, Gazebo, Manual Control

Date: 2026-07-20
Branch: `dev-saqar`
Head commit: `f68c6fc added README`

## Verdict

Phase 1 is **PASS WITH WARNINGS** for the Hunter V2 simulation/manual-control baseline.

The robot description parses, the Monza Gazebo Sim launch starts, the Hunter model spawns once, `gz_ros2_control` loads the expected interfaces, both ros2_control controllers activate, `/clock`, odometry, joint states and TF are available, and measured manual `TwistStamped` commands move the robot with plausible Ackermann behavior.

Phase 3 remains **gated** because the controller stack depends on `f1tenth_planning`, but `src/f1tenth_planning` is currently a missing gitlink with no `.gitmodules` mapping. The default controller node fails at runtime with `ModuleNotFoundError`.

## Environment And Repository

Executed repository inspection commands:

```bash
pwd
git status --short --branch
git branch --show-current
git log --oneline -10
find . -maxdepth 3 -type f | sort
```

Observed stack:

| Item | Evidence |
|---|---|
| Workspace | `/home/thesp/sp_codes/hunter_robot` |
| OS | Ubuntu 24.04.4 LTS Noble |
| ROS 2 | Jazzy, `ROS_DISTRO=jazzy`, `ros2` at `/opt/ros/jazzy/bin/ros2` |
| Gazebo | Gazebo Sim 8.11.0 / Harmonic (`gz sim --version`) |
| Control plugin | `gz_ros2_control` present; `gazebo_ros2_control` not found |
| Python | 3.12.3 |
| Compiler | GCC/G++ 13.3.0 |
| RMW | `rmw_fastrtps_cpp` from `ros2 doctor --report` |
| ROS packages | `hunter_description`, `hunter_gazebo`, `hunter_controllers` |

`src/f1tenth_planning` is recorded in Git as a gitlink at commit `70f0b6ccd09a975ee8711e132e75800da6cc61e0`, but the path is deleted in the worktree and `git submodule status` fails with:

```text
fatal: no submodule mapping found in .gitmodules for path 'src/f1tenth_planning'
```

## Fixes Applied During Validation

1. Fixed Docker workspace paths in [docker/Dockerfile](/home/thesp/sp_codes/hunter_robot/docker/Dockerfile:107):
   `WORKDIR` and `.bashrc` overlay/resource paths now use `/ros_ws`, matching Compose and the entrypoint.
2. Added `liburdfdom-tools` to [docker/Dockerfile](/home/thesp/sp_codes/hunter_robot/docker/Dockerfile:57), satisfying the package manifest test dependency used by `check_urdf`.
3. Replaced Compose bind mounts for `build/`, `install/`, and `log/` with Docker named volumes in [docker-compose.yml](/home/thesp/sp_codes/hunter_robot/docker-compose.yml:43), avoiding host/container CMake cache and symlink-install conflicts.
4. Updated the Docker/host build note in [README.md](/home/thesp/sp_codes/hunter_robot/README.md:34).
5. Repaired stale generated artifacts by removing old `build/`, `install/`, and `log/`, then reran a clean host build.
6. Added utility smoke tests in [test_hunter_utils.py](/home/thesp/sp_codes/hunter_robot/src/hunter_controllers/test/test_hunter_utils.py:1) so host and Docker test runs execute real checks.
7. Removed deprecated `tests_require` metadata from [setup.py](/home/thesp/sp_codes/hunter_robot/src/hunter_controllers/setup.py:1); test dependencies remain declared in `package.xml`.

## A1 Build And Dependency Validation

Commands executed:

```bash
ros2 doctor --report
docker compose build
docker compose run --rm dev rosdep check --from-paths src --ignore-src
docker compose run --rm dev colcon build --symlink-install --event-handlers console_direct+
docker compose run --rm dev colcon test --event-handlers console_direct+
colcon build --symlink-install --event-handlers console_direct+
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Results:

| Check | Result | Evidence |
|---|---|---|
| ROS environment | PASS WITH WARNING | `ros2 doctor --report` completed, but sandbox network-interface inspection reported permission warnings. |
| Docker image build | PASS | `docker compose build` completed for `hunter_robot:jazzy`. |
| Docker dependency validation | PASS | `All system dependencies have been satisfied`. |
| Clean host build | PASS | `Summary: 3 packages finished [6.08s]`. |
| Host tests | PASS | `test/test_hunter_utils.py ....`; `Summary: 4 tests, 0 errors, 0 failures, 0 skipped`. |
| Docker workspace build/tests | PASS WITH WARNING | Docker build and `colcon test` pass with 4 unittest checks. Docker `colcon test-result` reports `0 tests` because the setuptools/unittest path does not emit colcon result XML, even though `colcon test` ran 4 tests and returned 0. |

Initial host `rosdep` was unavailable (`rosdep: command not found`), so dependency validation was performed in the project Docker environment. The first default host build also failed because generated artifacts were owned by the container and then had `/ros_ws` CMake cache paths; cleaning generated artifacts resolved it.

## A2 Robot Description Validation

Commands executed:

```bash
source install/setup.bash
xacro -o /tmp/hunter_default.urdf src/hunter_description/description/robot.urdf.xacro
check_urdf /tmp/hunter_default.urdf
```

Result: **PASS**

Evidence:

```text
robot name is: hunter
Successfully Parsed XML
root Link: base_link
  chassis
    front_left_steering_link -> front_left_wheel
    front_right_steering_link -> front_right_wheel
    rear_left_wheel
    rear_right_wheel
```

Key source findings:

| Item | Evidence |
|---|---|
| Front steering joints | `front_left_steering_joint`, `front_right_steering_joint`, revolute, limit `+-0.461` rad |
| Rear traction joints | `rear_left_wheel_joint`, `rear_right_wheel_joint`, continuous velocity-command joints |
| Wheel radius | `0.165` m |
| Wheelbase | Controller YAML uses `0.65` m; controller node parameter default is `0.648` m |
| Track width | `0.585` m |
| ros2_control hardware | `gz_ros2_control/GazeboSimSystem` |
| Gazebo plugin | `libgz_ros2_control-system.so` |

The steering joint axis is physically vertical after the joint-origin rotation, and wheel axes resolve to the lateral axle direction.

## A3 RViz Validation

Command executed:

```bash
source install/setup.bash
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```

Result: **PASS WITH WARNING**

The first launch failed because GUI processes inherited Snap/VS Code runtime variables and resolved `libpthread` from `/snap/core20`. Re-running the same launch with Snap/GTK variables stripped allowed RViz to start:

```text
[rviz2]: Stereo is NOT SUPPORTED
[rviz2]: OpenGl version: 4.6 (GLSL 4.6)
```

The RViz config fixed frame is `base_link`, which is appropriate for robot-model inspection. Runtime TF showed `base_link -> chassis` and the wheel/steering links. Warning: the RViz config still contains stale display entries for `virtual_front_wheel` and `virtual_rear_wheel`, which are not in the generated URDF.

## A4 Gazebo Validation

Command executed:

```bash
source install/setup.bash
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```

Result: **PASS**

Runtime evidence from the sanitized launch:

```text
Gazebo Sim Server v8.11.0
Loading SDF world file[.../hunter_gazebo/worlds/Monza.sdf]
World [track_world] initialized
Entity creation successful
Created entity [198] named [hunter]
```

`gz sdf -k src/hunter_gazebo/worlds/Monza.sdf` returned:

```text
Valid.
```

`/clock` was available and active; `ros2 topic hz /clock` reported roughly `750-830 Hz` during the runtime sample. The robot did not fall, explode, or drift during the stop command.

## A5 ros2_control Validation

Commands executed:

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces
```

Result: **PASS**

Controller state:

```text
ackermann_like_controller ackermann_steering_controller/AckermannSteeringController  active
joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster              active
```

Claimed command interfaces:

```text
front_left_steering_joint/position [claimed]
front_right_steering_joint/position [claimed]
rear_left_wheel_joint/velocity [claimed]
rear_right_wheel_joint/velocity [claimed]
```

State interfaces were available for front steering positions and all wheel positions/velocities. Controller update rate is configured as `1000` Hz in `ackermann_like_controller.yaml`.

## A6 Topic And Message Validation

Commands executed:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /joint_states -v
ros2 topic hz /ackermann_like_controller/odometry
ros2 topic hz /joint_states
```

Result: **PASS WITH WARNING**

Observed nodes included:

```text
/ackermann_like_controller
/controller_manager
/gz_ros_control
/joint_state_broadcaster
/ros_gz_bridge
/rviz
```

Key topics:

| Purpose | Topic | Type |
|---|---|---|
| Vehicle command | `/ackermann_like_controller/reference` | `geometry_msgs/msg/TwistStamped` |
| Odometry | `/ackermann_like_controller/odometry` | `nav_msgs/msg/Odometry` |
| Joint states | `/joint_states` | `sensor_msgs/msg/JointState` |
| Simulation clock | `/clock` | `rosgraph_msgs/msg/Clock` |
| Robot description | `/robot_description` | `std_msgs/msg/String` |
| TF | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` |

Topic rates sampled:

```text
/ackermann_like_controller/odometry: ~296-394 Hz
/joint_states: ~475 Hz
```

Warning: `ros2 topic info /ackermann_like_controller/reference -v` reports `Unknown topic` when no publisher is alive, even though `ros2 topic list -t` shows the topic and the runtime probe measured `reference_subscribers=1`. The controller accepted and acted on `TwistStamped` commands. The configured watchdog/reference timeout is `0.5` s.

No explicit emergency-stop interface was found.

## A7 Manual Driving Validation

Result: **PASS**

Runtime probe published `geometry_msgs/msg/TwistStamped` to `/ackermann_like_controller/reference` and sampled `/ackermann_like_controller/odometry` plus `/joint_states`.

```text
reference_subscribers=1
label,cmd_linear_x,cmd_angular_z,dx,dy,dist,dyaw,odom_vx,odom_wz,front_left_steer,front_right_steer
stop,0.000,0.000,-0.0000,-0.0000,0.0000,0.0000,-0.0000,0.0000,-0.0000,-0.0000
forward,0.700,0.000,1.0998,0.0000,1.0998,0.0000,0.7000,0.0000,0.0000,0.0000
reverse,-0.400,0.000,-0.6275,-0.0000,0.6275,0.0000,-0.4000,0.0000,-0.0000,0.0000
left_arc,0.450,0.450,0.6630,0.1588,0.6818,0.5061,0.4500,0.3581,0.4610,0.4610
right_arc,0.450,-0.450,0.6583,0.2440,0.7021,-0.4737,0.4500,-0.3581,-0.4610,-0.4610
combined_left,0.800,0.700,1.1070,0.4816,1.2072,0.8215,0.8000,0.5966,0.4610,0.4252
```

Interpretation:

| Behavior | Result |
|---|---|
| Stop | No measurable drift |
| Forward | Positive displacement, positive odometry velocity |
| Reverse | Negative displacement, negative odometry velocity |
| Left steering | Positive yaw change |
| Right steering | Negative yaw change |
| Combined steering/motion | Plausible curved motion |
| Steering limits | Commands saturate at approximately `+-0.461` rad |

## A8 TF And Odometry Validation

Commands executed:

```bash
ros2 topic echo --once /tf_static
ros2 topic echo --once /tf
ros2 topic echo --once /ackermann_like_controller/odometry
```

Result: **PASS WITH WARNING**

Evidence:

```text
/tf_static: base_link -> chassis, z = 0.37
/tf: chassis -> front_left_steering_link -> front_left_wheel
/tf: chassis -> front_right_steering_link -> front_right_wheel
/tf: chassis -> rear_left_wheel
/tf: chassis -> rear_right_wheel
```

Odometry sample after manual driving:

```text
header.frame_id: odom
child_frame_id: base_link
pose.position.x: 3.0881
pose.position.y: 1.0078
twist.linear.x: approximately 0 after stop
twist.angular.z: approximately 0 after stop
```

Warning: `enable_odom_tf` is configured `false`, so there is no `odom -> base_link` TF in `/tf`. This is acceptable for the current RViz fixed frame (`base_link`) and for controllers consuming the odometry topic directly, but Phase 3 benchmarks should either keep using the odometry message as pose source or intentionally enable/publish odometry TF. No false `map -> odom` transform was observed.

## A9 Acceptance Criteria

| Criterion | Classification | Notes |
|---|---|---|
| Clean workspace build succeeds | PASS | Clean host build: 3 packages finished. Docker workspace build also passes with named volumes. |
| Tests complete without blocking failures | PASS WITH WARNING | 4 utility smoke tests pass on host and Docker; launch/controller integration coverage is still missing. |
| Robot description parses | PASS | `xacro` and `check_urdf` succeeded. |
| Robot renders in RViz | PASS WITH WARNING | RViz starts after stripping Snap GUI environment leakage. |
| Robot spawns correctly in Gazebo | PASS | `Entity creation successful`, one `hunter` entity. |
| Controllers activate correctly | PASS | Ackermann controller and joint-state broadcaster active. |
| Manual command interface is confirmed | PASS | `/ackermann_like_controller/reference`, `TwistStamped`, one subscriber. |
| Manual driving works | PASS | Measured stop/forward/reverse/left/right/combined behavior. |
| Steering direction is correct | PASS | Positive `angular.z` produced positive yaw; negative produced negative yaw. |
| Odometry is available and plausible | PASS | Odom pose and velocity match commanded motion. |
| TF is valid | PASS WITH WARNING | Robot tree valid; odom TF intentionally disabled. |
| Simulation time is consistent | PASS | `/clock` available; runtime timestamps use sim time. |
| One documented launch starts Phase 1 | PASS WITH WARNING | `ros2 launch hunter_gazebo launch_gz_sim.launch.py`; may need Snap GUI variables stripped when launched from VS Code Snap. |

## Remaining Warnings And Blockers

1. **Phase 3 blocker:** `f1tenth_planning` is missing. `python3 -c "import f1tenth_planning"` fails, and `ros2 run hunter_controllers hunter_controller_node ... controller_type:=pure_pursuit` exits with `ModuleNotFoundError`.
2. **Test gap:** smoke tests now cover utility-level waypoint loading and command conversion, but no automated tests currently exercise URDF parsing, launch, controllers, topic contracts, or manual command behavior.
3. **RViz config warning:** stale `virtual_front_wheel` and `virtual_rear_wheel` display entries should be removed or the frames should be intentionally added.
4. **GUI environment warning:** when launched from the VS Code Snap environment, Gazebo/RViz initially crashed due `/snap/core20` library leakage. Stripping Snap/GTK variables resolved the launch.
5. **No explicit e-stop topic found:** the Ackermann controller reference timeout provides stale-command stop behavior, but there is no separate emergency-stop interface.
