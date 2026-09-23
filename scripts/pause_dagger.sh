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

banked() { grep -l '"status": "ok"' datasets/raw/*dagger_it*/collection_report.json 2>/dev/null | wc -l; }

if [ "$1" != "--now" ]; then
  # Wait for the count of COMPLETED rollouts to rise by one. The first version
  # compared that count against the number of run DIRECTORIES -- which also
  # counts the in-progress one and any failed one -- so the condition could
  # never become true and the script waited indefinitely while the pipeline
  # carried on. Six more rollouts ran during one such wait.
  BEFORE=$(banked)
  echo "waiting for the rollout in progress to finish ($BEFORE banked; --now to stop sooner)..."
  while kill -0 "$PID" 2>/dev/null; do
    [ "$(banked)" -gt "$BEFORE" ] && break
    sleep 20
  done
  echo "rollout finished ($(banked) banked); stopping"
fi

kill -INT -"$PGID" 2>/dev/null || true
for i in $(seq 1 20); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
kill -9 -"$PGID" 2>/dev/null || true
sleep 2
# anything the pipeline started, by process group
ps -eo pid,pgid,comm | awk -v g="$PGID" '$2==g {print $1}' | xargs -r kill -9 2>/dev/null || true

# ...and the simulator launches, which are NOT in that group. The runners start
# each `ros2 launch` with start_new_session=True so they can own it by its own
# pgid -- which means killing the pipeline's group leaves the launch orphaned,
# still running Gazebo, the policy, the shadow and a recorder. The pause then
# reports success over a live simulator and the next resume aborts on a dirty
# preflight. SIGINT each launch's own group so the mcap still finalises.
for lp in $(pgrep -f 'ros2 launch hunter_dataset' 2>/dev/null); do
  lg=$(ps -o pgid= -p "$lp" 2>/dev/null | tr -d ' ')
  [ -n "$lg" ] && { echo "  stopping orphaned launch pid $lp (group $lg)"; kill -INT -"$lg" 2>/dev/null; }
done
for i in $(seq 1 30); do pgrep -f 'ros2 launch hunter_dataset' >/dev/null || break; sleep 1; done
for lp in $(pgrep -f 'ros2 launch hunter_dataset' 2>/dev/null); do
  lg=$(ps -o pgid= -p "$lp" 2>/dev/null | tr -d ' ')
  [ -n "$lg" ] && kill -9 -"$lg" 2>/dev/null
done
sleep 3
LEFT=$(timeout 25 ros2 node list --no-daemon --spin-time 3 2>/dev/null | wc -l)
if [ "$LEFT" != "0" ]; then
  echo "NOT PAUSED CLEANLY -- $LEFT ROS nodes still up:"
  timeout 25 ros2 node list --no-daemon --spin-time 3 2>/dev/null | sed 's/^/    /'
  echo "resume will refuse to start until these are gone."
  exit 1
fi
echo "ROS graph empty.
OK=$(grep -l '"status": "ok"' datasets/raw/*dagger_it*/collection_report.json 2>/dev/null | wc -l)
echo "PAUSED. $OK/75 rollouts banked. Resume with scripts/resume_dagger.sh"
