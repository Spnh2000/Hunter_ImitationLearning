#!/bin/bash
# Watch one trained policy drive, with the Gazebo GUI and RViz.
#
#   scripts/watch_policy.sh <checkpoint|pid> [track] [speed_cap]
#
#   scripts/watch_policy.sh checkpoints/bc_mlp_s0.pt                  # SaoPaulo @ 2.1 -- a track it has NEVER seen
#   scripts/watch_policy.sh checkpoints/bc_mlp_s0.pt Nuerburgring 2.7 # a track it trained on
#   scripts/watch_policy.sh checkpoints/bc_discrete_s0.pt Montreal 1.5
#   scripts/watch_policy.sh pid SaoPaulo 2.1                          # the expert, for comparison
#
# Goes through record_expert.launch.py -- the same path the scored evaluation
# uses -- so what you watch is what would be measured. That launch is also what
# applies max_speed_mps to the policy node.
#
# Runs on an isolated ROS domain and Gazebo partition, so it cannot disturb a
# collection or DAgger run happening at the same time. Ctrl-C to stop.
#
# Works unchanged inside the container:
#   docker compose run --rm dev scripts/watch_policy.sh \
#       checkpoints/final/dagger_mlp_s0.pt SaoPaulo 2.1
# docker-compose.yml mounts scripts/ and checkpoints/ for exactly this, and
# `cd $(dirname $0)/..` lands on /ros_ws either way.
set -e
cd "$(dirname "$0")/.."
CKPT="${1:?usage: watch_policy.sh <checkpoint|pid> [track] [speed_cap]}"
TRACK="${2:-SaoPaulo}"
CAP="${3:-2.1}"
# Default under the repo so it works identically on the host and in the
# container (where /tmp is ephemeral and host scratch paths do not exist).
OUT="${WATCH_OUT:-$PWD/results/watch_runs}"

source /opt/ros/jazzy/setup.bash; source install/setup.bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
for v in $(env | grep -oE '^(SNAP[A-Z_]*|GTK_[A-Z_]*|GDK_[A-Z_]*)=' | tr -d '='); do unset "$v"; done
export PATH="/usr/bin:$PATH" GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$LD_LIBRARY_PATH"
export ROS_DOMAIN_ID=9 GZ_PARTITION=watch          # isolated from any running job
mkdir -p "$OUT"

ARGS=(world:="$TRACK".sdf
      track_file:=src/race_tracks/"$TRACK"/"$TRACK"_centerline_2x.csv
      params_file:="$(echo "$TRACK" | tr 'A-Z' 'a-z')"_defaults.yaml
      track_name:="$TRACK"
      max_speed_mps:="$CAP"
      headless:=false use_rviz:=true 
      output_dir:="$OUT" run_id:="watch_$(date +%H%M%S)")

if [ "$CKPT" = "pid" ]; then
  echo ">>> the EXPERT (PID driving on ground truth) -- $TRACK @ $CAP m/s"
else
  ARCH=$(/usr/bin/python3 -c "import torch;print(torch.load('$CKPT',map_location='cpu',weights_only=False)['arch'])")
  echo ">>> POLICY $(basename "$CKPT") [$ARCH] -- $TRACK @ $CAP m/s -- sees /scan only, no pose"
  ARGS+=(controller:=policy policy_checkpoint:="$PWD/$CKPT" policy_arch:="$ARCH")
fi
echo ">>> startup order: Gazebo loads the world (~20 s), then the robot is"
echo ">>>   spawned, then RViz opens once the robot actually exists."
echo ">>> RViz will warn about the /map topic having no publisher -- that is"
echo ">>>   expected: this run has no localization stack (slam_mode:=disabled),"
echo ">>>   because the policy drives on /scan and needs no map. Ignore it, or"
echo ">>>   untick the Map display."
echo ">>> Ctrl-C to stop."
echo ">>> the run is written to $OUT ; summary.csv lands in src/hunter_benchmark/results/"
exec ros2 launch hunter_dataset record_expert.launch.py "${ARGS[@]}"
