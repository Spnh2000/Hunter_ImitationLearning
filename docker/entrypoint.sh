#!/bin/bash
# Hunter V2 container entrypoint.
# Executed before every `docker compose run` / `docker compose exec` command.
set -e

# 1. ROS 2 Jazzy base
source /opt/ros/jazzy/setup.bash

# 2. Workspace overlay (present after `colcon build` inside the container)
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

# 3. f1tenth_planning sub-packages (control/, planning/, utils/)
#    The package's setup.py only lists the top-level 'f1tenth_planning'
#    namespace, so colcon's install/ prefix only gets the egg-info.
#    Adding src/f1tenth_planning to PYTHONPATH mirrors the editable install
#    used on the host and makes all sub-packages importable.
export PYTHONPATH="/ros_ws/src/f1tenth_planning${PYTHONPATH:+:${PYTHONPATH}}"

exec "$@"
