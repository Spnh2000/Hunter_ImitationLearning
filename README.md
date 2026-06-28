# Hunter Robot Simulation — ROS 2 Jazzy + Gazebo Harmonic

URDF description and Gazebo Harmonic simulation for the **AgileX Hunter V2** — an Ackermann-steered
mobile robot — running on **ROS 2 Jazzy Jalisco** with `ros2_control` and `ackermann_steering_controller`.

> **Note:** This repository is a migration of the original
> [LCAS/hunter_robot](https://github.com/LCAS/hunter_robot) (ROS 2 Humble + Gazebo Classic)
> to **ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8, DARTsim)**.
> See [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) for the full list of changes.

Official robot: [AgileX Hunter V2](https://global.agilex.ai/products/hunter-2-0)

---

## Features

- True Ackermann steering via `ackermann_steering_controller/AckermannSteeringController`
- Full `ros2_control` integration with `gz_ros2_control/GazeboSimSystem` hardware plugin
- All four actuated joints independently commanded (no mimic joints)
- URDF with simple-primitive collision geometry (DARTsim-compatible)
- Gazebo Harmonic Ogre2 materials (explicit `<gazebo reference>` colour blocks)
- RViz2 visualisation launch

---

## Prerequisites

| Component | Version |
|-----------|---------|
| OS | Ubuntu 24.04 LTS (Noble) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz-sim 8) |
| Kernel tested | 6.x |

### System dependencies

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ackermann-steering-controller \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro
```

---

## Installation

```bash
# 1. Create (or navigate to) your colcon workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Clone
git clone https://github.com/Spnh2000/hunter_robot.git

# 3. Install remaining rosdep dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install
source install/setup.bash
```

---

## Packages

| Package | Contents |
|---------|----------|
| `hunter_description` | URDF/XACRO, ros2_control config, RViz2 launch |
| `hunter_gazebo` | Gazebo Harmonic launch files, clock bridge |

---

## Usage

### Visualise in RViz2 (no simulation)

```bash
ros2 launch hunter_description robot_view.launch.py
```

### Simulate in Gazebo Harmonic

Both launch files are equivalent and start Gazebo Harmonic + RViz2:

```bash
ros2 launch hunter_gazebo launch_gz_sim.launch.py
# or
ros2 launch hunter_gazebo launch_sim.launch.py
```

#### macOS Specific Run Instructions
On macOS, due to Gazebo rendering and network interface limitations, you must run the server headless and launch the GUI client manually in separate terminals. 

Ensure you set the loopback IP (`GZ_IP=127.0.0.1`) and localhost DDS discovery (`ROS_LOCALHOST_ONLY=1`) in both terminals.

**Terminal 1 (Server & RViz):**
```bash
conda activate ros_env
export ROS_LOCALHOST_ONLY=1
export GZ_IP=127.0.0.1
source install/setup.zsh
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```
> [!NOTE]
> Because the `Monza` racetrack world is large, the launch file has a **15-second delay** before spawning the robot entity. Wait at least 15 seconds after launching Terminal 1 for the car to appear.

**Terminal 2 (Gazebo GUI Client):**
```bash
conda activate ros_env
export ROS_LOCALHOST_ONLY=1
export GZ_IP=127.0.0.1
gz sim -g
```
**Teleop control**

```bash
export ROS_LOCALHOST_ONLY=1
export GZ_IP=127.0.0.1
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p use_sim_time:=true \
  --remap cmd_vel:=/ackermann_like_controller/reference
```

Optional launch arguments:

```bash
# Custom world SDF or verbosity level
ros2 launch hunter_gazebo launch_gz_sim.launch.py gz_args:="-r -v 4 my_world.sdf"
```

After launch, verify controllers are active:

```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]                        active
ackermann_like_controller[ackermann_steering_controller/AckermannSteeringController]           active
```

### Teleop control

> **Important:** `ackermann_steering_controller` subscribes to `TwistStamped` on the
> `~/reference` topic (not `~/cmd_vel`). Pass `-p stamped:=true` and `-p use_sim_time:=true`.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p use_sim_time:=true \
  --remap cmd_vel:=/ackermann_like_controller/reference
```

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | Stop |
| `u/o/m/.` | Diagonal movement |

---

## Controller Configuration

[`hunter_description/config/ackermann_like_controller.yaml`](hunter_description/config/ackermann_like_controller.yaml)

Key parameters:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `update_rate` | 1000 Hz | Matches Gazebo Harmonic default physics step |
| `traction_joints_names` | `[rear_right_wheel_joint, rear_left_wheel_joint]` | Both rear wheels independently commanded |
| `steering_joints_names` | `[front_right_steering_joint, front_left_steering_joint]` | Both front wheels independently commanded |
| `traction_wheels_radius` | 0.165 m | |
| `wheelbase` | 0.65 m | |
| `traction_track_width` | 0.585 m | Rear axle centre-to-centre |
| `reference_timeout` | 0.5 s | Robot stops if no command received |

---

## Known Log Warnings (non-fatal)

| Warning | Cause | Impact |
|---------|-------|--------|
| `QML: Binding loop detected` | Qt/QML GUI cosmetic issue | None |

---

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Type | Ackermann-steered 4-wheel |
| Drive | Rear-wheel drive (both rear wheels independently velocity-controlled) |
| Steering | Front-wheel Ackermann (both front joints independently position-controlled) |
| Wheel radius | 0.165 m |
| Wheelbase | 0.65 m |
| Track width | 0.585 m |
| Max steer angle | ±0.461 rad (~26.4°) |

---

## References

- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [ros2_controllers / ackermann_steering_controller](https://github.com/ros-controls/ros2_controllers/tree/master/ackermann_steering_controller)
- [ros_gz (ros_gz_bridge, ros_gz_sim)](https://github.com/gazebosim/ros_gz)
- [ros2_control Getting Started — Jazzy](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [AgileX Hunter V2 reference URDF](https://github.com/agilexrobotics/ugv_gazebo_sim/blob/master/hunter/hunter2_base/urdf/hunter2_base_gazebo.xacro)
- Original Humble repository: [LCAS/hunter_robot](https://github.com/LCAS/hunter_robot)

---

## License

Apache License 2.0 — see [LICENSE](LICENSE).

Original authors: Ibrahim Hroob (LCAS, University of Lincoln), Hariharan Arunachalam.
