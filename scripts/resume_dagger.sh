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
# systemd-inhibit: a closed laptop lid suspended the machine mid-rollout on
# 2026-09-23, and it did not survive the resume -- the box rebooted and the
# whole pipeline died. Worse, the runner could not even detect the freeze: its
# stuck-watchdog counts SIMULATED seconds, and a suspended simulator publishes
# no clock, so sim time stops advancing and the watchdog can never fire. The
# inhibitor is the fix; the watchdog gap is noted in scripts/phase8_pipeline.sh.
INHIBIT=""
command -v systemd-inhibit >/dev/null && INHIBIT="systemd-inhibit --what=sleep:idle:handle-lid-switch --who=hunter_phase8 --why=DAgger_collection --mode=block"
setsid nohup $INHIBIT scripts/phase8_pipeline.sh --skip-train >> results/phase8_pipeline2.log 2>&1 < /dev/null &
disown
sleep 5
echo "resumed. tail -f results/phase8_pipeline2.log"
