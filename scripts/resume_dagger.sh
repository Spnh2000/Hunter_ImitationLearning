#!/bin/bash
# Resume the Phase 8 pipeline after scripts/pause_dagger.sh.
#
#   scripts/resume_dagger.sh
#
# Finished rollouts are skipped (each wrote "status": "ok"). --skip-train makes
# the current iteration keep the student it started with instead of retraining
# it on the rollouts it has already produced -- otherwise half an iteration
# would be collected by one policy and half by another.
set -e
cd "$(dirname "$0")/.."
pgrep -f 'bash scripts/phase8_pipeline.sh' > /dev/null && { echo "already running"; exit 1; }
source /opt/ros/jazzy/setup.bash; source install/setup.bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
export HUNTER_REPO="$PWD"
OK=$(grep -l '"status": "ok"' datasets/raw/*dagger_it*/collection_report.json 2>/dev/null | wc -l)
echo "resuming with $OK/75 rollouts already banked"
LEFT=$(timeout 20 ros2 node list --no-daemon --spin-time 2 2>/dev/null | wc -l)
[ "$LEFT" != "0" ] && { echo "ABORT: $LEFT ROS nodes still up -- clean those first"; exit 1; }
setsid nohup scripts/phase8_pipeline.sh --skip-train >> results/phase8_pipeline2.log 2>&1 < /dev/null &
disown
sleep 5
echo "resumed. tail -f results/phase8_pipeline2.log"
