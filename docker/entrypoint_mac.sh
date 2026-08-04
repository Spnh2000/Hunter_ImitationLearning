#!/bin/bash
set -e

# 1. Start Xvfb (Virtual Framebuffer) on Display :1
Xvfb :1 -screen 0 1280x1024x24 &
sleep 1

# 2. Start Fluxbox window manager
DISPLAY=:1 fluxbox &
sleep 1

# 3. Start VNC server (x11vnc)
x11vnc -display :1 -forever -shared -nopw -rfbport 5900 -bg
sleep 1

# 4. Start noVNC proxy on port 6080 pointing to VNC port 5900
websockify --web /usr/share/novnc/ 0.0.0.0:6080 127.0.0.1:5900 &

# 5. Set DISPLAY environment variable for any child processes
export DISPLAY=:1

# 6. Source ROS 2 Jazzy base
source /opt/ros/jazzy/setup.bash

# 7. Source workspace overlay if it exists
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

# 8. Set f1tenth_planning PYTHONPATH
export PYTHONPATH="/ros_ws/src/f1tenth_planning${PYTHONPATH:+:${PYTHONPATH}}"

# 9. Execute the user command
exec "$@"
