# Hunter V2 — Autonomous Path Tracking on ROS 2 Jazzy + Gazebo Harmonic

Path-tracking, perception and benchmarking stack for the [AgileX Hunter V2](https://www.agilex.ai/chassis/4) (Ackermann-steered 4-wheel rover), running on ROS 2 Jazzy and Gazebo Harmonic (gz-sim 8). The robot follows a reference line derived from [F1TENTH racetracks](https://github.com/f1tenth/f1tenth_racetracks) rebuilt as Gazebo worlds, while a metrics pipeline records per-run tracking accuracy against **ground truth** for comparing controllers.

**Project status:** Phase 5 of 9 complete.

| Phase | Contents | State |
|---|---|---|
| 1–2 | Robot bring-up, track conversion pipeline | done |
| 3 | Trajectory server + PID tracker + metrics logger | done |
| 5 | 2D LiDAR + IMU, dual EKF, SLAM mapping, A/B pose-source switch | done |
| 4, 7–8 | Pure Pursuit / Stanley / LQR / MPC, then offline RL | pending |
| 6 | Imitation dataset collection (expert = PID on ground truth, student sees `/scan` only) | next |
| 9 | Sim-to-sim policy transfer to the Hunter | pending |

`slam_toolbox` *localization* mode is deferred — see [Known limitations](#known-limitations).

---

## Architecture

```
        <Track>_centerline_2x.csv          /scan   /imu   wheel odom
                    │                         │      │        │
          ┌─────────▼──────────┐         ┌────▼──────▼────────▼────┐
          │ trajectory_server  │         │   hunter_localization   │
          │ (hunter_trajectory)│         │  slam_toolbox + 2x EKF  │
          └─────────┬──────────┘         └────────────┬────────────┘
                    │ /trajectory/reference            │ pose
                    │              ┌───────────────────┘
          ┌─────────▼──────────────▼─┐   pose_source: ground_truth
          │       pid_tracker        │                | slam_ekf
          │     (hunter_control)     │                | wheel_odom
          └─────────┬────────────────┘
                    ├──▶ /ackermann_like_controller/reference  (TwistStamped)
                    └──▶ /tracking/diagnostics
                              │
                    ┌─────────▼──────────┐
                    │   metrics_logger   │  → summary.csv + rosbag2 (mcap)
                    │ (hunter_benchmark) │    estimator error AND true error
                    └────────────────────┘
```

Adding a controller in a later phase means adding one executable to `hunter_control` named `<controller>_tracker`; `trajectory_server`, the message contracts and the benchmark pipeline stay untouched.

### Packages

| Package | Build type | Role |
|---|---|---|
| `hunter_description` | ament_cmake | URDF/xacro, ros2_control config, RViz layout |
| `hunter_gazebo` | ament_cmake | IMS + Monza world SDFs, full simulation launch |
| `hunter_msgs` | ament_cmake | `Waypoint`, `Trajectory`, `TrackingDiagnostics` messages |
| `hunter_trajectory` | ament_cmake (C++) | `trajectory_server` — CSV → reference trajectory |
| `hunter_control` | ament_cmake (C++) | `pid_tracker` + shared `hunter_control_common` geometry |
| `hunter_benchmark` | ament_cmake | `metrics_logger`, `summarize_run.py`, benchmark launch, per-track params |
| `hunter_localization` | ament_cmake | Dual-EKF + slam_toolbox configs and launch (Phase 5) |

---

## Requirements

| Item | Version |
|---|---|
| OS | Ubuntu 24.04 Noble |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| Python | 3.12 |
| Extra ROS packages | `control_toolbox`, `ros2bag`, `rosbag2_storage_mcap`, `robot_localization`, `slam_toolbox` |

---

## Quick start (Docker — recommended)

```bash
docker compose build          # ~5-10 min on first run
xhost +local:docker           # once per host login, for the Gazebo/RViz GUI
docker compose run --rm dev   # interactive shell in the container
```

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash

# Full benchmark run: Gazebo + trajectory server + PID tracker + metrics
ros2 launch hunter_benchmark benchmark.launch.py

# Monza instead of the default IMS -- all three must agree
ros2 launch hunter_benchmark benchmark.launch.py \
    world:=Monza.sdf \
    track_file:=src/Monza/Monza_centerline_2x.csv \
    params_file:=monza_defaults.yaml track_name:=Monza

# No display (CI, batch collection): Gazebo server-only, no RViz
ros2 launch hunter_benchmark benchmark.launch.py headless:=true use_rviz:=false
```

That single command spawns the robot on the reference line, brings up the controllers, starts tracking, and records a rosbag plus `summary.csv` under `src/hunter_benchmark/results/`.

To run just the simulation with no controller:

```bash
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```

### Host build (no Docker)

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch hunter_benchmark benchmark.launch.py
```

> Docker and host builds must not share `build/`, `install/` or `log/` — the CMake caches store absolute paths. Delete them when switching.

---

## Quick start with Docker on macOS (Apple Silicon or Intel)

Standard X11 forwarding via XQuartz has OpenGL driver compatibility issues with modern Gazebo Harmonic and RViz on macOS. Instead, use the macOS-specific configuration which runs a virtual display server (VNC/noVNC) inside the container and streams the graphical desktop directly to your web browser.

### 1. Build the macOS image

```bash
docker compose -f docker-compose.mac.yml build
```

### 2. Open a shell in the container
You **must** pass the `--service-ports` flag to map the web viewer port back to your Mac:

```bash
docker compose -f docker-compose.mac.yml run --service-ports --rm dev
```

### 3. Open the Web GUI in your browser
Open your web browser (Safari, Chrome, Firefox) and go to:
```
http://localhost:6080/vnc.html
```
Click the **Connect** button to see the virtual desktop.

### 4. Build the ROS workspace (inside the container)

```bash
colcon build --symlink-install
```

### 5. Launch the simulation

In the container terminal:
```bash
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```
Look at your browser tab (`http://localhost:6080/vnc.html`) to see the Gazebo Sim and RViz windows.

### 6. Launch the controller (second terminal / second container shell)

Open a second terminal window on your Mac host and run:
```bash
# Open a second shell in the same container:
docker compose -f docker-compose.mac.yml exec dev bash

# Then run the benchmark:
ros2 launch hunter_benchmark benchmark.launch.py
```

---

## Tracks and the 2× scale

Two circuits from [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks), both used at **2× scale**. Adding another is two commands:

```bash
python3 src/scale_track.py <Track> 2.0
python3 src/my_convert_track.py <Track>/<Track>_map.png <Track>/<Track>_map_2x.yaml \
    -o hunter_gazebo/worlds/<Track>.sdf --smoothness 0.0003 --thickness 0.06
```

The map PNG is never modified — scaling is expressed as a larger `resolution` in the map YAML, with `origin` scaled to match. Lengths (x, y, arc length, track widths) scale by the factor, curvature scales inversely, angles are untouched.

**Why scale at all?** F1TENTH circuits are drawn for ~0.3 m-long 1/10-scale cars, while the Hunter V2 is 0.98 m long with a **1.31 m minimum turning radius**. Two independent reasons:

- *Kinematic* — corners tighter than 1.31 m are undrivable at any speed.
- *Margin* — wheel odometry drifts without bound, so the corridor must absorb pose error the controller cannot see.

| | IMS 2× | Monza 2× |
|---|---|---|
| Track length | 586 m | 892 m |
| Raw minimum radius | 7.8 m (fine at 1×) | **0.74 m at 1× — undrivable** |
| Minimum radius after smoothing | 28.1 m | 2.21 m |
| Wall clearance | 1.87 m | 1.75 m |
| Margin after robot half-width | 1.50 m | 1.37 m |
| Curvature speed limit binds | **never** | **10.8% of track**, down to 0.94 m/s |

IMS is an oval and is kinematically fine at 1×; it is scaled purely for margin. **Monza genuinely requires it** — its 0.74 m chicanes are about half the radius the Hunter can physically turn.

That last row matters for Phase 6. On IMS the curvature-limited profile never engages, so commanded speed is a flat 2.7 m/s for the whole lap — an imitation dataset collected there has *zero* speed variance and would train a policy that only ever outputs one number, while showing excellent loss. Monza's chicanes force real slowdowns.

| File | Role |
|---|---|
| `src/<Track>/<Track>_centerline.csv`, `_raceline.csv` | Upstream F1TENTH data, unmodified |
| `src/<Track>/<Track>_centerline_2x.csv` | **The reference line actually driven** |
| `src/<Track>/<Track>_map_2x.yaml` | Scaled map metadata used to build the world |

The **centerline** is the reference, not the raceline: the F1TENTH raceline hugs the walls by design, which a 0.745 m-wide robot cannot follow at any scale.

> The launch **aborts** if `track_file` and the world SDF were generated at different scales — a mismatch otherwise looks exactly like a controller bug, with the robot tracking its wrongly-scaled path perfectly while drifting into a wall.

---

## Sensing and localization (Phase 5)

| Sensor | Spec | Topic |
|---|---|---|
| 2D LiDAR | 270° FOV, **1080 beams**, 30 m, 20 Hz, 0.01 m noise | `/scan` |
| IMU | 100 Hz, with bias | `/imu` |

The LiDAR deliberately matches the **f1tenth_gym** sensor exactly (270°/1080/30 m) so a gym-trained policy's observation vector transfers without reshaping.

Frame ownership — the part that fails silently when wrong:

```
map --(slam_toolbox)--> odom --(local EKF)--> base_link --(URDF)--> laser
```

The **local EKF** fuses wheel odometry + IMU into a continuous `odom→base_link` at 50 Hz — no jumps, which is what a controller needs. The **global EKF** consumes SLAM's absolute pose and publishes the smoothed `/odometry/filtered_map`; it deliberately does **not** publish TF, because slam_toolbox cannot emit its pose without also emitting `map→odom`, and two publishers of one transform is the classic dual-EKF failure.

Build a map (drives the track with PID while mapping, then serialise):

```bash
ros2 launch hunter_benchmark benchmark.launch.py slam_mode:=mapping pose_source:=ground_truth
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: 'src/hunter_localization/maps/IMS'}"
```

> Pose graphs are **not in git** — `IMS.posegraph` is over 100 MB, past GitHub's per-file limit, and is regenerable by the command above.

### Pose source A/B switch

`pose_source` swaps what the tracker believes, with no code change — the tracker consumes `nav_msgs/Odometry` and never looks up TF:

| `pose_source` | Topic | Use |
|---|---|---|
| `ground_truth` | `/ground_truth/odometry` | Expert driving, benchmark reference |
| `slam_ekf` | `/odometry/filtered_map` | Deployable stack |
| `wheel_odom` | `/ackermann_like_controller/odometry` | Phase 3 behaviour, drifts |

---

## How the controller works

**Principle: feed forward whatever the reference geometry already tells you; let the PID correct only the residual error.**

Steering combines a cross-track PID with two feedforward terms:

```
steering = clamp( PID(−lateral_error)
                  − heading_gain · heading_error        ← Stanley-style heading term
                  + atan(wheelbase · curvature),        ← geometric corner anticipation
                  ±max_steering )
```

Heading error is a proportional feedforward rather than a second PID: on a curving path the heading error is a persistent geometric fact, not a disturbance, so an integral term on it would wind up and overshoot on corner exit.

Speed feeds forward the profile speed with a PID trim, where the profile itself comes from curvature: `v = sqrt(a_lat_limit / |κ|)`, clamped.

Two **separate** lookaheads matter, because they need opposite lead distances:

- `curvature_preview_m` (0.3 m) — steering feedforward, must be **short**; it should describe the curve the front wheels are entering *now*. Too long and the robot steers for a corner it hasn't reached.
- `speed_preview_m` (2.5 m) — corner braking, must be **long** enough to shed speed before the corner.

### Key parameters (`src/hunter_benchmark/config/monza_defaults.yaml`)

One canonical params file is shared by every phase, so cross-controller comparisons stay meaningful.

```yaml
trajectory_server:
  max_speed_mps: 2.7             # straights
  lateral_accel_limit_mps2: 0.4  # sets corner speed via v = sqrt(a_lat/κ)
  centerline_smoothing_m: 2.8    # rounds corners within the vehicle's steering limit

pid_tracker:
  wheelbase_m: 0.65
  max_steering_rad: 0.461        # ⇒ 1.31 m minimum turning radius
  curvature_preview_m: 0.3       # steering feedforward (short)
  speed_preview_m: 2.5           # corner braking (long)
  steering_pid: {p: 1.3, i: 0.04, d: 0.25}
  speed_pid:    {p: 0.5, i: 0.3,  d: 0.0}
```

Gains are live-tunable: `ros2 param set /pid_tracker steering_pid.p 1.5`.

> **Speed vs. real hardware.** The stock Hunter 2.0 is rated 6 km/h (1.67 m/s); AgileX rates the upgraded drivetrain at 10 km/h (2.78 m/s). The URDF's traction joint limit (17 rad/s × 0.165 m wheel = 2.81 m/s) reflects the upgraded figure. Results above ~1.67 m/s assume the upgraded drivetrain.

---

## ROS interface

| Topic | Type | Direction |
|---|---|---|
| `/trajectory/reference` | `hunter_msgs/Trajectory` | trajectory_server → tracker (latched) |
| `/trajectory/path_viz` | `nav_msgs/Path` | RViz visualisation (latched) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR, 270°/1080 beams @ 20 Hz |
| `/imu` | `sensor_msgs/Imu` | IMU @ 100 Hz |
| `/ackermann_like_controller/odometry` | `nav_msgs/Odometry` | raw wheel odometry (drifts) |
| `/odometry/filtered` | `nav_msgs/Odometry` | local EKF, continuous, 50 Hz |
| `/odometry/filtered_map` | `nav_msgs/Odometry` | global EKF, map frame |
| `/ground_truth/odometry` | `nav_msgs/Odometry` | Gazebo truth — sim only, never on hardware |
| `/ackermann_like_controller/reference` | `geometry_msgs/TwistStamped` | tracker → robot |
| `/tracking/diagnostics` | `hunter_msgs/TrackingDiagnostics` | tracker → metrics |

Two non-obvious details:

- `enable_odom_tf: false` on the wheel controller — the **local EKF** owns `odom→base_link`. Trackers consume `Odometry` directly and never look up TF, which is what makes the `pose_source` A/B a pure remap.
- Commands carry a **zero timestamp**, so the controller's 0.5 s `reference_timeout` is judged against its own clock rather than ROS↔Gazebo sync jitter. That timeout is also the de facto e-stop; there is no dedicated emergency-stop topic.

---

## Benchmarking

`metrics_logger` records to an mcap rosbag and writes `summary.csv` with frozen columns: mean/RMS/max lateral and heading error, laps completed, distance travelled, steering-command variance and reversal rate, speed error.

`summarize_run.py` recomputes the identical statistics offline from a recorded bag via a shared stats module, so later phases can reuse the pipeline without a live ROS graph.

Because `pose_source` also selects what the tracker believes, `summary.csv` reports **both** the estimator's own error and the true error against ground truth. That distinction is not cosmetic: in one run the controller reported 0.2 mm RMS while sitting 0.57 m off the line.

```bash
ros2 run hunter_benchmark summarize_run.py src/hunter_benchmark/results/<run>/bag
```

---

## Known limitations

**Wheel odometry drifts, and the controller cannot see it.** Dead-reckoned encoders accumulate slip every corner with no correction. Measured against ground truth at 2.7 m/s, the robot reached **2.29 m** of physical deviation while its own diagnostics reported **0.001 m** median lateral error. The controller is accurate; its idea of where it is, is not.

This is why `metrics_logger` reports true error separately, and why the expert in Phase 6 drives on ground truth.

**`slam_toolbox` localization mode is deferred.** Mapping works and produces a full-coverage pose graph. Localization mode loads that graph, activates, and receives scans — but never publishes `/slam_toolbox/pose` or `map→odom`, so the global EKF gets no absolute measurement and silently degrades to IMU-aided dead reckoning. Untried next steps: seed `/initialpose` rather than `map_start_at_dock`, or try `map_and_localization_slam_toolbox_node`. **AMCL is the likelier fix** — it publishes `/amcl_pose` as `PoseWithCovarianceStamped`, exactly what the global EKF's `pose0` already expects, so it is a config swap rather than a rewrite.

Not on the critical path: Phase 6 uses privileged expert distillation, where the expert drives on ground truth and the student observes `/scan` only, so no pose estimate is needed at deployment.

**Everything silently-failing found so far shares one signature** — every node reports healthy while the answer is wrong. Validate against Gazebo ground truth or the GUI, never from diagnostics alone.

---

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Robot drives into a wall and stops with wheels still turning | Usually a **duplicate tracker**. Check `ros2 node list \| grep pid_tracker`; if it appears more than once, a previous run survived and both are publishing commands at 50 Hz. `pkill -9 -f pid_tracker; pkill -9 -f trajectory_server` |
| Launch aborts with `TRACK / WORLD SCALE MISMATCH` | `track_file` and the world SDF were built at different scales — use the `_2x` CSV, or regenerate the world |
| `ros_gz_sim create` hangs on "Requesting list of world names" | `GZ_IP=127.0.0.1` not set; gz-transport bound to a non-loopback interface |
| Gazebo GUI doesn't appear | `xhost +local:docker` on the host |
| Gazebo crashes at startup | Set `LIBGL_ALWAYS_SOFTWARE=1`, or strip Snap/GTK env vars if launching from VS Code |
| `CMakeCache.txt` path mismatch | Mixed Docker and host builds — delete `build/`, `install/`, `log/` and rebuild |
| Controller stops after 0.5 s | Nothing is publishing to `/ackermann_like_controller/reference` (`reference_timeout`) |
| `PermissionError` writing `summary.csv` | `results/` is root-owned from a container run: `sudo chown -R $USER:$USER src/hunter_benchmark/results` |
| Branch switch blocked by untracked `summary.csv` | Same root-ownership problem. Rename them (`mv` inside `results/` needs no sudo) or chown as above |
| `slam_toolbox` runs but publishes nothing | It is a **lifecycle** node — it must be CONFIGUREd then ACTIVATEd. A `/slam_toolbox/transition_event` topic with 0 publishers elsewhere is the tell |
| Push rejected, `GH001: Large files detected` | A pose graph got committed. They are gitignored and regenerable — see [Sensing and localization](#sensing-and-localization-phase-5) |

---

## Repository layout

```
src/
├── hunter_description/   # URDF/xacro, ros2_control, RViz
├── hunter_gazebo/        # IMS.sdf + Monza.sdf (2x scale), simulation launch
├── hunter_msgs/          # Trajectory / Waypoint / TrackingDiagnostics
├── hunter_trajectory/    # trajectory_server node
├── hunter_control/       # pid_tracker + shared geometry library
├── hunter_benchmark/     # metrics, summarise script, benchmark launch, per-track params
├── hunter_localization/  # dual-EKF + slam_toolbox configs, launch, maps (gitignored)
├── IMS/                  # oval circuit: CSVs, map, 2x derivatives
├── Monza/                # twisty circuit: CSVs, map, 2x derivatives
├── scale_track.py        # uniform track scaling tool
└── my_convert_track.py   # occupancy map → Gazebo world SDF
docs/validation/          # per-phase validation reports
docker/                   # Dockerfile + entrypoint
```
