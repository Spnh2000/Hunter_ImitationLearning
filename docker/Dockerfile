# ─────────────────────────────────────────────────────────────────────────────
# Hunter V2 — ROS 2 Jazzy + Gazebo Harmonic development image
# OS:  Ubuntu 24.04 (Noble)
# Sim: Gazebo Harmonic (gz-sim 8, via OSRF apt repo)
# GPU: Intel integrated (Mesa / DRI — no NVIDIA required)
# ─────────────────────────────────────────────────────────────────────────────
FROM ros:jazzy-ros-base

# ---------------------------------------------------------------------------
# 1. Add Gazebo Harmonic apt repository (OSRF)
#    ros:jazzy-ros-base ships only the ROS apt key; Gazebo needs a second one.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
       signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
       http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. ROS / Gazebo / controller packages
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    # Gazebo Harmonic CLI + libraries (gz sim, gz topic, gz model …)
    gz-harmonic \
    \
    # ROS ↔ Gazebo bridge (ros_gz_sim, ros_gz_bridge, ros_gz_interfaces)
    ros-jazzy-ros-gz \
    # gz_ros2_control plugin (GazeboSimSystem hardware interface)
    ros-jazzy-gz-ros2-control \
    \
    # ros2_control stack
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-ackermann-steering-controller \
    ros-jazzy-steering-controllers-library \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    \
    # Robot description / URDF pipeline
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    \
    # Visualization
    ros-jazzy-rviz2 \
    \
    # Teleop (useful for manual overrides)
    ros-jazzy-teleop-twist-keyboard \
    \
    # Build tooling
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    \
    # Python scientific stack (numpy/scipy come via ROS debs where available)
    python3-numpy \
    python3-scipy \
    \
    # Mesa / DRI drivers for Intel integrated GPU (hardware-accelerated OpenGL)
    libgl1-mesa-dri \
    libgl1 \
    mesa-utils \
    \
    # Shell quality-of-life
    bash-completion \
    less \
    vim \
    \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. Python pip packages
#    f1tenth_planning/requirements.txt pins pre-3.12 versions — incompatible.
#    Only matplotlib (not in ROS debs) is needed at runtime.
#    numba is stubbed in our code; cvxpy/osqp are unused by active nodes.
# ---------------------------------------------------------------------------
RUN pip3 install --no-cache-dir --break-system-packages \
    matplotlib

# ---------------------------------------------------------------------------
# 4. Workspace skeleton
#    src/ is volume-mounted at runtime (edit on host, build in container).
#    build/ and install/ are also mounted so colcon artifacts persist.
# ---------------------------------------------------------------------------
WORKDIR /ros_ws

# ---------------------------------------------------------------------------
# 5. Entrypoint: sources ROS base + workspace overlay on every invocation
# ---------------------------------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
