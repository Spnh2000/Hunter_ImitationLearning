# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current state warning

This repo is **mid-refactor** as of this writing. Two things are broken/missing in the worktree and this is intentional, not a bug to "fix" reflexively:

- **`src/hunter_controllers/` is an empty directory.** It used to be a working, tested `ament_python` ROS package (unified controller node, waypoint loader/recorder, `f1tenth_planning`-backed Pure Pursuit/Stanley/LQR/MPC). It was deliberately deleted and is being rewritten. `git log`/`git show HEAD:src/hunter_controllers/...` still has the old version if you need historical reference, but do not assume any of its files currently exist — check first.
- **`src/f1tenth_planning` is a broken git submodule reference** (a gitlink in the git tree pointing at commit `70f0b6c`, but there is no `.gitmodules` entry mapping it to a URL). It resolves to nothing on disk. Any work that depends on the F1TENTH planning library needs this resolved first (re-add as a proper submodule, vendor it, or replace it).

Only `hunter_description` and `hunter_gazebo` are currently real, buildable ROS packages. `colcon build` will only build those two until `hunter_controllers` is rebuilt.

`docs/validation/phase_1_validation.md` and `phase_2_validation.md` are dated validation reports from when `hunter_controllers` still existed — useful for understanding prior working behavior (topics, controller state, waypoint stats) but do not describe the current tree.

## What this project is

Autonomous driving stack for the AgileX Hunter V2 (Ackermann-steered 4-wheel rover) on ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8). The intended design (per `README.md` and the pre-refactor `hunter_controllers`) runs F1TENTH-style path-tracking controllers (Pure Pursuit, Stanley, LQR, MPC) against a pre-recorded waypoint CSV to drive the robot around a Monza-circuit Gazebo world.

## Development environment

Docker is the primary supported environment (`docker/Dockerfile`, `docker-compose.yml`). It targets Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic; `src/` is bind-mounted read-write, `build/`/`install/`/`log/` are named Docker volumes (kept separate from any host-native build to avoid CMake cache path conflicts). Host-native builds are also supported if ROS Jazzy + Gazebo Harmonic are installed locally.

```bash
# Build the dev image
docker compose build

# Once per host login (GUI access for Gazebo/RViz)
xhost +local:docker

# Interactive shell in the container
docker compose run --rm dev

# Inside the container — build the workspace
colcon build --symlink-install
source install/setup.bash

# One-shot non-interactive sim launch (no controller)
docker compose run --rm sim
```

`--symlink-install` matters: it makes host-side Python edits (e.g. anything in `hunter_controllers` once rebuilt) take effect without rebuilding.

Host build (no Docker), from repo root:
```bash
colcon build --symlink-install
source install/setup.bash
```
Don't mix Docker-built and host-built `build/`/`install/`/`log/` directories — delete and rebuild clean if switching between them.

### Running the simulation

```bash
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```
This starts Gazebo Harmonic with the Monza world (`src/hunter_gazebo/worlds/Monza.sdf`), spawns the Hunter V2 from `hunter_description`'s xacro, brings up `joint_state_broadcaster` and `ackermann_like_controller` (in that order via `OnProcessExit` event handlers, since the controller manager isn't ready until the entity spawns), and opens RViz. Key launch args: `spawn_x`/`spawn_y`/`spawn_z`/`spawn_yaw`, `world`, `use_sim_time`.

Once `hunter_controllers` exists again, the controller is launched separately (was `ros2 launch hunter_controllers controller_launch.py`, param `controller_type:={pure_pursuit,stanley,lqr,mpc}`).

### Tests

Only `ament_cmake`-declared test deps exist right now (`hunter_description`: `ament_cmake_pytest`, `launch_testing_ros`, urdf checks; `hunter_gazebo`: `ament_lint_auto`/`ament_lint_common`). Run via:
```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```
Note from the last validation pass: the Python-package (`ament_python`/setuptools) test path does not emit colcon result XML even when `colcon test` runs real unittest checks — `colcon test-result` can show `0 tests` while the console output shows tests passing. Trust the `colcon test` console output over `colcon test-result` for Python packages.

## Architecture

### `hunter_description` (ament_cmake)
URDF/xacro robot model + ros2_control config + RViz layout. No compiled code — `CMakeLists.txt` just installs `description/`, `launch/`, `config/`, `rviz/`, `meshes/`.

- `description/robot.urdf.xacro` — top-level, includes the others.
- `description/robot_core.xacro` — links/joints. Front `front_{left,right}_steering_joint` (revolute, ±0.461 rad) → `front_{left,right}_wheel` (continuous, mimic-driven); rear `rear_{left,right}_wheel_joint` (continuous, velocity-commanded). Wheel radius 0.165 m, track width 0.585 m. Collision geometry is simple primitives (box/cylinder) — **not** the `.dae` meshes (DARTsim in Harmonic silently drops `.dae` collision submeshes with 0 normals; keep `.dae` visual-only).
- `description/ros2_control.xacro` — hardware interface is `gz_ros2_control/GazeboSimSystem`; Gazebo plugin is `libgz_ros2_control-system.so`. Keep URDF `<mimic>` tags even though DARTsim logs a non-fatal "mimic constraint not supported" — `gz_ros2_control`'s own P-controller for mimic joints reads `hardware_info.mimic_joints`, which is populated only from URDF `<mimic>`, not from xacro `<param name="mimic">`.
- `config/ackermann_like_controller.yaml` — the `AckermannSteeringController` (from `steering_controllers_library`) config: 1000 Hz update rate (matches Gazebo's 1 ms physics step), `wheelbase: 0.65`, `reference_timeout: 0.5`s, `enable_odom_tf: false` (odometry is published on the topic only, no `odom→base_link` TF).
- Driven-joint friction: keep `friction="0.0"` on traction joints. DARTsim implements velocity control as a servo force capped at the joint's `effort` limit; if Coulomb friction exceeds that limit the wheels can't turn (this doesn't happen on ODE/Classic, which used kinematic `SetVelocity()`).

### `hunter_gazebo` (ament_cmake)
Gazebo Harmonic world + full sim launch. `worlds/Monza.sdf` is the track (also duplicated at `src/Monza.sdf` at the repo root — validated identical; `hunter_gazebo/worlds/` is the one actually loaded). `launch/launch_gz_sim.launch.py` is the maintained launch file; `launch_sim.launch.py` is an older variant.

`SetEnvironmentVariable('GZ_IP', '127.0.0.1')` is set first in the launch file and again in `docker-compose.yml`'s `dev` service — required on this class of machine, otherwise gz-transport binds to a non-loopback interface and `ros_gz_sim create` hangs "Requesting list of world names" forever because the Gazebo server/GUI/spawner can't discover each other.

### `hunter_controllers` (currently empty — see warning above)
When present, this is an `ament_python` package. Historical design (from git history / README, not current code):
- Single unified node (`hunter_controller_node`) that selects the tracking algorithm at launch via a `controller_type` param (`pure_pursuit` default, `stanley`, `lqr`, `mpc`) rather than one node per algorithm.
- Publishes `geometry_msgs/TwistStamped` to `/ackermann_like_controller/reference` — **not** plain `Twist` on `cmd_vel`. Conversion: `angular.z = v · tan(δ) / L` where `δ` is the steering angle from the planner and `L` is the wheelbase. Uses a zero ROS timestamp (`rclpy.time.Time().to_msg()`) on published commands so `reference_timeout` is judged against the controller's own clock rather than ROS↔Gazebo clock-sync jitter.
- Subscribes to `/ackermann_like_controller/odometry` (`nav_msgs/Odometry`) for pose.
- Waypoint CSVs (`x, y, yaw, speed`) live under `waypoints/`; format differs from the `f1tenth_planning` library's own expected array shapes (`[x, y, speed, yaw]` for Pure Pursuit/Stanley, 5-column with curvature for LQR) — a conversion function bridges the two.
- Depends on `f1tenth_planning` (currently the broken submodule — see warning above). If restored, its `control/` subpackages import as e.g. `from f1tenth_planning.control.pure_pursuit.pure_pursuit import PurePursuitPlanner`, constructed as `Planner(wheelbase=..., waypoints=ndarray)` and stepped via `.plan(pose_x, pose_y, pose_theta, ...)`. The library's `setup.py` only declares the top-level package, so colcon's install only gets egg-info — the container entrypoint (`docker/entrypoint.sh`) prepends `src/f1tenth_planning` to `PYTHONPATH` to make subpackages importable, mirroring an editable pip install.

### Track/world data (`src/Monza/`)
Reference assets for the Monza circuit: `Monza_map.{png,yaml}` (occupancy-grid style map, resolution 0.09585), `Monza_centerline.csv`, `Monza_raceline.csv`, `Monza_DonkeySim_waypoints.txt` — these are the **unscaled upstream F1TENTH data, kept as-is for reference**. The files actually driven are the `*_2x` derivatives generated by `src/scale_track.py` (see the 2× note under "Known environment gotchas"). `my_convert_track.py` converts a map PNG + YAML into the Gazebo world SDF; regenerate with `--smoothness 0.0003 --thickness 0.06` (the 0.002 default chords ~0.9 m across curved corners and eats track width).

## Known environment gotchas

- **Xacro substitutions:** use `$(find pkg_name)` inside `.xacro` files, not `$(find-pkg-share pkg_name)` — the latter is a launch-file-only substitution and crashes xacro processing.
- **SDF friction syntax:** sdformat 14 (Harmonic) only reads `<mu>value</mu>` as text content in `<gazebo reference>` blocks. The Gazebo-Classic-era `<mu1 value="...">` attribute form is silently ignored (wrong friction, not an error).
- **GUI env leakage under VS Code / Snap:** Gazebo/RViz can crash on launch if Snap/GTK env vars leak into the process (resolves `/snap/core20` libpthread instead of the system one). Strip Snap/GTK env vars from the shell launching `ros2 launch` if this happens.
- **Check for zombie tracker nodes before trusting any run.** `ros2 node list | grep pid_tracker` — if it appears more than once, a previous run's tracker survived and is publishing to `/ackermann_like_controller/reference` alongside the new one at 50 Hz each. Nothing in ROS prevents this. The robot then receives alternating contradictory commands, drifts sideways even on a straight, and wedges against a wall *with its wheels still turning*. `pkill -9 -f pid_tracker; pkill -9 -f trajectory_server` between runs.
- **No e-stop topic:** stopping relies entirely on `reference_timeout` (0.5s) on the Ackermann controller — there's no dedicated emergency-stop interface.
- **Monza is used at 2× scale, and the CSV scale must match the world SDF.** F1TENTH circuits are drawn for ~0.3 m-long 1/10-scale cars. At 1× the Hunter had only 0.216 m of margin at the tightest corner — less than its own odometry drift — so wall contact was arithmetic, not a tuning failure. `src/scale_track.py` generates `Monza_map_2x.yaml` + `Monza_{centerline,raceline}_2x.csv` (the PNG is unchanged; only the YAML `resolution`/`origin` scale), and `Monza.sdf` is regenerated from the 2× YAML. Margin at 2× is 1.372 m. **Pairing a 1× CSV with the 2× world puts the reference path nowhere near the walls** — always keep the suffixes matched.
- **The Monza raceline CSV is physically unusable for the Hunter at any scale.** It was optimized for a ~0.3 m-wide F1TENTH car and hugs the walls in corners by design. The benchmark reference is the **centerline**, and `trajectory_server` must smooth it (`centerline_smoothing_m`, default 2.8) because even the 2×-scaled raw centerline reaches ~1.97 rad/m, past the Hunter's full-lock limit (tan(0.461)/0.65 ≈ 0.764 rad/m, i.e. a 1.31 m minimum turning radius).
- **Wall SDF generation needs `--smoothness ≤ 0.0005` in `my_convert_track.py`.** The old default (0.002) allowed ~0.9 m chord deviation, so straight wall boxes cut across curved corners and encroached up to ~1 m into the track (47 segments). The current `Monza.sdf` was regenerated at 0.0003 (127 segments, centerline wall clearance ≥ 0.80 m).
- **Wheel odometry starts at (0,0,yaw=0), not the world spawn pose.** `AckermannSteeringController`'s dead-reckoned odometry has no way to know where Gazebo spawned the robot. Track CSVs are in world coordinates, so any node comparing odometry poses against them must first transform the path into the odom frame using the spawn pose (`trajectory_server`'s `odom_origin_{x_m,y_m,yaw_rad}` params; `benchmark.launch.py` wires its computed spawn pose in automatically). Skipping this rotates the reference path by the spawn yaw and steers the robot straight into a wall — while dead-reckoned diagnostics *still look perfect*, because slipping wheels keep integrating a fictional trajectory. Corollary: never validate physical tracking from odometry-derived logs alone; check the Gazebo GUI or ground-truth world pose.
