#!/bin/bash
# Pause the Phase 8 pipeline. Resume with scripts/resume_dagger.sh.
#
#   scripts/pause_dagger.sh          wait for the rollout in progress to finish,
#                                    then stop  (loses nothing)
#   scripts/pause_dagger.sh --now    stop immediately (the rollout in progress
#                                    is discarded and redone on resume)
#
# Kills by PROCESS GROUP, never by name: a Python ROS node's comm is "python3",
# so a name-based kill leaves orphans running -- that is how an abandoned
# training process once pinned this machine at load 9.8 for hours.
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash 2>/dev/null || true

PID=$(pgrep -f 'bash scripts/phase8_pipeline.sh' | head -1)
[ -z "$PID" ] && { echo "pipeline is not running"; exit 0; }
PGID=$(ps -o pgid= -p "$PID" | tr -d ' ')
echo "pipeline pid $PID, process group $PGID"

if [ "$1" != "--now" ]; then
  BEFORE=$(ls -d datasets/raw/*dagger_it* 2>/dev/null | wc -l)
  echo "waiting for the rollout in progress to finish (Ctrl-C then --now to stop sooner)..."
  while kill -0 "$PID" 2>/dev/null; do
    DONE=$(grep -l '"status": "ok"' datasets/raw/*dagger_it*/collection_report.json 2>/dev/null | wc -l)
    CUR=$(ls -d datasets/raw/*dagger_it* 2>/dev/null | wc -l)
    # the report appears when the rollout has been converted and inspected
    [ "$DONE" -ge "$BEFORE" ] && [ "$CUR" -ge "$BEFORE" ] && \
      [ -f "$(ls -dt datasets/raw/*dagger_it* 2>/dev/null | head -1)/collection_report.json" ] && break
    sleep 20
  done
  echo "rollout finished; stopping"
fi

kill -INT -"$PGID" 2>/dev/null || true
for i in $(seq 1 20); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
kill -9 -"$PGID" 2>/dev/null || true
sleep 2
# anything the pipeline started, by process group
ps -eo pid,pgid,comm | awk -v g="$PGID" '$2==g {print $1}' | xargs -r kill -9 2>/dev/null || true
sleep 2
LEFT=$(timeout 20 ros2 node list --no-daemon --spin-time 2 2>/dev/null | wc -l)
echo "ROS nodes still up: $LEFT (must be 0)"
[ "$LEFT" != "0" ] && timeout 20 ros2 node list --no-daemon --spin-time 2 2>/dev/null
OK=$(grep -l '"status": "ok"' datasets/raw/*dagger_it*/collection_report.json 2>/dev/null | wc -l)
echo "PAUSED. $OK/75 rollouts banked. Resume with scripts/resume_dagger.sh"
