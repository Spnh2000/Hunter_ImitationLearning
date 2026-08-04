# Hunter V2 — ROS 2 Jazzy + Gazebo Harmonic

Autonomous driving stack for the [AgileX Hunter V2](https://www.agilex.ai/chassis/4) using ROS 2 Jazzy and Gazebo Harmonic (gz-sim 8). The stack runs F1TENTH-style path-tracking controllers (Pure Pursuit, Stanley, LQR, MPC) on a pre-recorded or hand-crafted waypoint file and drives the robot around the Monza circuit world.

---

## Package overview

```
src/
├── hunter_description/    # URDF/xacro, ros2_control config, RViz config
├── hunter_gazebo/         # Gazebo world (Monza.sdf), simulation launch file
├── hunter_controllers/    # Path-tracking controller node, waypoint tools
│   ├── waypoints/         # Monza_baseline.csv, monza_track.csv
│   └── config/params.yaml
└── f1tenth_planning/      # F1TENTH planning library (Pure Pursuit, Stanley, LQR, MPC)
```

| Package | Build type | Role |
|---|---|---|
| `hunter_description` | ament_cmake | URDF, ros2_control hardware interface, RViz layout |
| `hunter_gazebo` | ament_cmake | Gazebo world, full simulation launch |
| `hunter_controllers` | ament_python | Controller node, waypoint loader/recorder |
| `f1tenth_planning` | pure Python | Planning algorithms (vendored, patched for Python 3.12) |

---

## Hardware / software requirements

| Item | Version |
|---|---|
| OS | Ubuntu 24.04 Noble |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| Python | 3.12 |
| GPU | Any OpenGL-capable GPU; Intel integrated supported via Mesa/DRI |

---

## Quick start with Docker (recommended)

Docker is the supported way to run the stack. The image bundles all system dependencies; `src/` is volume-mounted so you edit on the host and run in the container. Docker build artifacts live in named volumes so container builds do not conflict with native host builds.

### 1. Build the image

```bash
cd ~/sp_codes/hunter_robot
docker compose build
```

First build takes ~5–10 minutes (downloads ~1 GB of ROS + Gazebo packages).

### 2. Allow GUI access

Run once per host login session (needed for Gazebo and RViz):

```bash
xhost +local:docker
```

### 3. Open a shell in the container

```bash
docker compose run --rm dev
```

### 4. Build the ROS workspace (inside the container)

```bash
colcon build --symlink-install
```

`--symlink-install` means Python source edits on the host take effect immediately without rebuilding.

### 5. Launch the simulation

In the container:

```bash
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```

This starts Gazebo Harmonic with the Monza world, spawns the Hunter V2, brings up the `AckermannSteeringController` and `JointStateBroadcaster`, and opens RViz.

### 6. Launch the controller (second terminal / second container shell)

```bash
# Open a second shell in the same container:
docker compose exec dev bash

# Then run the controller:
ros2 launch hunter_controllers controller_launch.py
```

To select a different controller or waypoint file:

```bash
ros2 launch hunter_controllers controller_launch.py \
    controller_type:=stanley \
    waypoint_file:=monza_track.csv
```

---

## Running on the host (without Docker)

Requires ROS 2 Jazzy, Gazebo Harmonic, and all ROS packages installed natively.

```bash
cd ~/sp_codes/hunter_robot
colcon build --symlink-install
source install/setup.bash

# Terminal 1 — simulation
ros2 launch hunter_gazebo launch_gz_sim.launch.py

# Terminal 2 — controller
ros2 launch hunter_controllers controller_launch.py
```

> **Note:** Docker and host builds use separate artifact locations. If you still have old container-generated `build/`, `install/` or `log/` directories from an earlier Compose layout, remove them before building on the host.

---

## Path-tracking controllers

The unified `hunter_controller_node` selects the algorithm at launch time via the `controller_type` parameter. All algorithms come from the bundled `f1tenth_planning` library.

| `controller_type` | Algorithm | Notes |
|---|---|---|
| `pure_pursuit` (default) | Pure Pursuit | Lookahead distance configurable; adaptive at speed |
| `stanley` | Stanley | Cross-track + heading error; works well on smooth paths |
| `lqr` | Linear Quadratic Regulator | Uses 5-column waypoints with curvature |
| `mpc` | Kinematic MPC | Falls back to Pure Pursuit if cvxpy is unavailable |

### Controller parameters (`config/params.yaml`)

```yaml
hunter_controller_node:
  ros__parameters:
    controller_type: "pure_pursuit"
    wheelbase: 0.648          # Hunter V2 front-to-rear axle [m]
    max_speed: 1.0            # hard cap on waypoint speeds [m/s]
    control_freq: 20.0        # Hz
    lookahead_distance: 1.0   # Pure Pursuit floor lookahead [m]
    k_path: 5.0               # Stanley cross-track gain
    lqr_timestep: 0.05        # LQR discretisation step [s]
```

### ROS interface

| Topic | Type | Direction |
|---|---|---|
| `/ackermann_like_controller/reference` | `geometry_msgs/TwistStamped` | **publish** — speed + angular velocity commands |
| `/ackermann_like_controller/odometry` | `nav_msgs/Odometry` | **subscribe** — pose in `odom` frame |
| `/controller_debug` | `std_msgs/Float64MultiArray` | **publish** — `[steering_angle, ref_speed, current_speed]` |

> The AckermannSteeringController subscribes on `~/reference` with **TwistStamped** — not `~/cmd_vel` with plain Twist. `angular.z` carries `v·tan(δ)/L`.

---

## Waypoints

Waypoint CSVs live in `src/hunter_controllers/waypoints/`. Format: `x, y, yaw, speed`.

| File | Points | Path | Speed |
|---|---|---|---|
| `Monza_baseline.csv` | 1 159 | Centerline, starts at (0, 0) | 0.3–0.8 m/s curvature-aware |
| `monza_track.csv` | 2 197 | Optimised raceline, starts at (−0.656, 0.142) | 0.3–0.8 m/s curvature-aware |

Speed is assigned by curvature: 0.8 m/s on straights, 0.5 m/s on gentle curves, 0.3 m/s on tight corners.

### Recording new waypoints

Drive the robot manually with the teleop node and record the path:

```bash
# Terminal 1 — teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/ackermann_like_controller/reference

# Terminal 2 — recorder
ros2 run hunter_controllers waypoint_recorder
```

The recorder writes a CSV to the current directory when you press Ctrl+C.

---

## Simulation spawn position

The robot spawns at the Monza start/finish line by default:

| Parameter | Value | Meaning |
|---|---|---|
| `spawn_x` | `0` | Track origin X |
| `spawn_y` | `0.142` | Track origin Y |
| `spawn_z` | `0.3` | Height above ground (clears collision mesh) |
| `spawn_yaw` | `1.503` | ~86° — pointing north, aligned with the straight |

Override at launch:

```bash
ros2 launch hunter_gazebo launch_gz_sim.launch.py \
    spawn_x:=-0.656 spawn_y:=0.142 spawn_yaw:=1.503
```

---

## ros2_control setup

The `AckermannSteeringController` from `steering_controllers_library` drives the Hunter V2:

- **Traction joints:** `rear_right_wheel_joint`, `rear_left_wheel_joint` (velocity command)
- **Steering joints:** `front_right_steering_joint`, `front_left_steering_joint` (position command)
- **Odometry frame:** `odom` (coincides with the Gazebo world frame at t = 0 in simulation)
- **Reference timeout:** 500 ms — the controller stops if no command arrives within this window
- **Stamp trick:** commands use a zero ROS timestamp (`rclpy.time.Time().to_msg()`) so the timeout is measured against the controller's own clock, avoiding false timeouts from ROS ↔ Gazebo clock-sync jitter

---

## Docker details

```
docker/
├── Dockerfile         # Ubuntu 24.04 + ROS Jazzy + Gazebo Harmonic + all deps
└── entrypoint.sh      # sources ROS base → workspace overlay → f1tenth PYTHONPATH
docker-compose.yml     # dev (interactive) + sim (one-shot launch) services
.dockerignore          # excludes build/, install/, src/ from build context
```

### Key compose flags

| Setting | Value | Reason |
|---|---|---|
| `network_mode: host` | host | ROS 2 DDS multicast and Gazebo transport require host networking |
| `devices: /dev/dri` | host DRI | Intel integrated GPU hardware-accelerated Mesa rendering |
| `group_add: video` | video group | DRI device read access without running as root |
| `DISPLAY` | from host | X11 forwarding for Gazebo GUI and RViz |
| `GZ_IP=127.0.0.1` | loopback | Forces Gazebo transport onto loopback; avoids multi-NIC discovery issues |

### Why `PYTHONPATH` for f1tenth_planning

`f1tenth_planning/setup.py` declares `packages=['f1tenth_planning']` (top-level only). Colcon's install prefix therefore only receives the egg-info — not the `control/`, `planning/`, or `utils/` sub-packages. The entrypoint prepends `src/f1tenth_planning` to `PYTHONPATH` to replicate the editable install that exists on the host.

### Troubleshooting

| Symptom | Fix |
|---|---|
| `CMakeCache.txt` path mismatch on first container build | Delete `build/` and `install/` on the host, then rebuild inside the container |
| Gazebo GUI doesn't appear | Run `xhost +local:docker` on the host before starting the container |
| Gazebo crashes at startup | Set `LIBGL_ALWAYS_SOFTWARE=1` in `docker-compose.yml` to force Mesa software rendering |
| `ros2 topic list` shows nothing | Check `ROS_DOMAIN_ID` matches between terminals |
| Controller stops after 0.5 s | Check `/ackermann_like_controller/reference` is being published; `reference_timeout: 0.5` in controller YAML |

---

## f1tenth_planning patches

Two bugs in the vendored library were fixed in-tree:

1. **Python 3.12 / numba incompatibility** — `numba.njit` decorator fails on Python 3.12 due to a `coverage.types.Tracer` attribute error. Patched in `f1tenth_planning/utils/utils.py` with a no-op fallback decorator.

2. **LQR wheelbase hardcoded** — `LQRPlanner.__init__` set `self.wheelbase = 0.33` regardless of the constructor argument. Patched in `f1tenth_planning/control/lqr/lqr.py`. The controller node also applies `p.wheelbase = self.L` after instantiation as a belt-and-braces fix.
