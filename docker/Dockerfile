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
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
\
    # Dual-EKF localization (hunter_localization/config/ekf_*.yaml)
    ros-jazzy-robot-localization \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-rqt\
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-ackermann-steering-controller \
    ros-jazzy-steering-controllers-library \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    # PID primitive for pid_tracker (control_toolbox::PidROS) -- see hunter_control
    ros-jazzy-control-toolbox \
    \
    # rosbag2 recording (metrics_logger/summarize_run.py, benchmark.launch.py)
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-rosbag2-py \
    \
    # Robot description / URDF pipeline
    liburdfdom-tools \
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
    git \
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


# Install Python dependencies for control package
RUN pip3 install --no-cache-dir --break-system-packages \
    transforms3d
    
# Install the highly optimized, lightweight PyTorch CPU wheel
RUN pip3 install --no-cache-dir --break-system-packages torch --index-url https://download.pytorch.org/whl/cpu

# Setup bashrc for interactive shells
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo 'export GZ_SIM_RESOURCE_PATH="/ros_ws/external/ackermann_sim:${GZ_SIM_RESOURCE_PATH}"' >> /root/.bashrc

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
