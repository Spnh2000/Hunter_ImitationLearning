# Hunter V2 — Autonomous Path Tracking on ROS 2 Jazzy + Gazebo Harmonic

Path-tracking and benchmarking stack for the [AgileX Hunter V2](https://www.agilex.ai/chassis/4) (Ackermann-steered 4-wheel rover), running on ROS 2 Jazzy and Gazebo Harmonic (gz-sim 8). The robot follows a reference line derived from an [F1TENTH racetrack](https://github.com/f1tenth/f1tenth_racetracks) around a Gazebo reconstruction of the Indianapolis Motor Speedway, while a metrics pipeline records per-run tracking accuracy for comparing controllers.

**Project status:** Phase 3 of 9 — a PID baseline controller plus the trajectory/benchmark infrastructure that later phases reuse unchanged. Phases 4–8 add Pure Pursuit, Stanley, LQR and MPC as drop-in tracker nodes; Phase 9 covers offline RL evaluation.

---

## Architecture

```
                     IMS_centerline_2x.csv
                              │
                    ┌─────────▼──────────┐
                    │ trajectory_server  │  parse → smooth → speed profile →
                    │ (hunter_trajectory)│  transform into odom frame
                    └─────────┬──────────┘
                              │ /trajectory/reference  (latched)
                    ┌─────────▼──────────┐
   /…/odometry ────▶│    pid_tracker     │──▶ /…/reference  (TwistStamped)
                    │  (hunter_control)  │──▶ /tracking/diagnostics
                    └─────────┬──────────┘
                              │
                    ┌─────────▼──────────┐
                    │   metrics_logger   │  → summary.csv + rosbag2 (mcap)
                    │ (hunter_benchmark) │
                    └────────────────────┘
```

Adding a controller in a later phase means adding one executable to `hunter_control` named `<controller>_tracker`; `trajectory_server`, the message contracts and the benchmark pipeline stay untouched.

### Packages

| Package | Build type | Role |
|---|---|---|
| `hunter_description` | ament_cmake | URDF/xacro, ros2_control config, RViz layout |
| `hunter_gazebo` | ament_cmake | IMS world SDF, full simulation launch |
| `hunter_msgs` | ament_cmake | `Waypoint`, `Trajectory`, `TrackingDiagnostics` messages |
| `hunter_trajectory` | ament_cmake (C++) | `trajectory_server` — CSV → reference trajectory |
| `hunter_control` | ament_cmake (C++) | `pid_tracker` + shared `hunter_control_common` geometry |
| `hunter_benchmark` | ament_cmake | `metrics_logger`, `summarize_run.py`, benchmark launch, canonical params |

---

## Requirements

| Item | Version |
|---|---|
| OS | Ubuntu 24.04 Noble |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| Python | 3.12 |
| Extra ROS packages | `control_toolbox`, `ros2bag`, `rosbag2_storage_mcap` |

---

## Quick start (Docker — recommended)

```bash
cd ~/sp_codes/hunter_robot
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

## Track data and the 2× scale

The circuit is the **Indianapolis Motor Speedway (IMS)** from [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks), used at **2× scale** and generated by `src/scale_track.py`:

```bash
python3 src/scale_track.py IMS 2.0
python3 src/my_convert_track.py IMS/IMS_map.png IMS/IMS_map_2x.yaml \
    -o hunter_gazebo/worlds/IMS.sdf --smoothness 0.0003 --thickness 0.06
```

The map PNG is never modified — scaling is expressed as a larger `resolution` in the map YAML, with `origin` scaled to match. Lengths (x, y, arc length, track widths) scale by the factor, curvature scales inversely, angles are untouched.

**Why scale at all?** F1TENTH circuits are drawn for ~0.3 m-long 1/10-scale cars, while the Hunter V2 is 0.98 m long with a **1.31 m minimum turning radius**. Two separate reasons a track may need scaling:

- *Kinematic* — corners tighter than 1.31 m radius are undrivable at any speed. This bites on twisty circuits; Monza's chicanes reach 0.47 m, about 3× tighter than the vehicle can turn.
- *Margin* — wheel odometry drifts without bound, so the corridor has to absorb pose error the controller cannot see.

IMS is an oval and is **kinematically fine at 1×** (7.8 m raw minimum radius). It is scaled 2× purely for the second reason.

| | 1× | 2× (current) |
|---|---|---|
| Track length | 293 m | 586 m |
| Reference-path wall clearance | 0.89 m | **1.87 m** |
| Margin after robot half-width | 0.51 m | **1.50 m** |
| Minimum radius after smoothing | 14.8 m | **28.1 m** |

With a 28 m minimum radius — 21× the vehicle's requirement — the curvature-limited speed profile never binds, so the robot holds `max_speed_mps` for the entire lap.

| File | Role |
|---|---|
| `src/IMS/IMS_centerline.csv`, `IMS_raceline.csv` | Upstream F1TENTH reference data, kept unmodified |
| `src/IMS/IMS_centerline_2x.csv` | **The reference line actually driven** |
| `src/IMS/IMS_map_2x.yaml` | Scaled map metadata used to generate the world |

The **centerline** is the reference, not the raceline: the F1TENTH raceline hugs the walls by design, which a 0.745 m-wide robot cannot follow at any scale.

> The launch file **aborts** if `track_file` and the world SDF were generated at different scales — a mismatch otherwise looks exactly like a controller bug, with the robot tracking its wrongly-scaled path perfectly while drifting into a wall.

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
| `/ackermann_like_controller/odometry` | `nav_msgs/Odometry` | **the only pose source** |
| `/ackermann_like_controller/reference` | `geometry_msgs/TwistStamped` | tracker → robot |
| `/tracking/diagnostics` | `hunter_msgs/TrackingDiagnostics` | tracker → metrics |

Two non-obvious details:

- `enable_odom_tf: false` — there is **no `odom→base_link` TF**. Every tracker must consume `Odometry` directly; TF lookups silently fail. Most external path-tracking examples assume TF is available.
- Commands carry a **zero timestamp**, so the controller's 0.5 s `reference_timeout` is judged against its own clock rather than ROS↔Gazebo sync jitter. That timeout is also the de facto e-stop; there is no dedicated emergency-stop topic.

---

## Benchmarking

`metrics_logger` records to an mcap rosbag and writes `summary.csv` with frozen columns: mean/RMS/max lateral and heading error, laps completed, distance travelled, steering-command variance and reversal rate, speed error.

`summarize_run.py` recomputes the identical statistics offline from a recorded bag via a shared stats module, so Phase 9's offline RL evaluation can reuse the pipeline without a live ROS graph.

```bash
ros2 run hunter_benchmark summarize_run.py src/hunter_benchmark/results/<run>/bag
```

---

## Known limitations

**Wheel odometry drifts, and the controller cannot see it.** Pose comes from dead-reckoned wheel encoders. Every corner adds a little wheel slip that is never corrected, so error accumulates without bound. Measured against Gazebo ground truth at 2.7 m/s, the robot reached a peak physical deviation of **2.29 m** while its own diagnostics reported a median lateral error of **0.001 m**. The controller is accurate; its map of where it is, is not.

Consequences worth understanding:

- The robot can be pinned against a wall while `/tracking/diagnostics` shows a perfect run — the wheels keep turning and the odometry keeps integrating a fiction. **Always validate physical behaviour against the Gazebo view or ground-truth pose, never from diagnostics alone.**
- Benchmarking Phase 4–9 controllers on this pose source partly measures odometry drift rather than controller quality. Bridging Gazebo's ground-truth pose for benchmark runs (keeping wheel odometry for realism runs) is the intended fix.

**Validated behaviour at 2.7 m/s:** zero steering saturation, 0.001 m median tracking error, continuous driving with no stalls. Wall contact still occurs in some corners purely from the drift above.

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

---

## Repository layout

```
src/
├── hunter_description/   # URDF/xacro, ros2_control, RViz
├── hunter_gazebo/        # IMS.sdf (2x scale), simulation launch
├── hunter_msgs/          # Trajectory / Waypoint / TrackingDiagnostics
├── hunter_trajectory/    # trajectory_server node
├── hunter_control/       # pid_tracker + shared geometry library
├── hunter_benchmark/     # metrics, summarise script, benchmark launch, params
├── IMS/                  # track CSVs, map, and 2x-scaled derivatives
├── scale_track.py        # uniform track scaling tool
└── my_convert_track.py   # occupancy map → Gazebo world SDF
docs/validation/          # per-phase validation reports
docker/                   # Dockerfile + entrypoint
```
