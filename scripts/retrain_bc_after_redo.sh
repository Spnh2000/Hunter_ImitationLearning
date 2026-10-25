#!/bin/bash
# One-off: wait for the p1 redo (Silverstone 1.5 / Montreal 2.7) to finish,
# then retrain the BC checkpoints under the 2-laps-per-track split rule so
# bc_* and dagger_* differ only in their data, not in their split.
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
while pgrep -f 'collect_dataset --plan .* --tracks Silverstone,Montreal' > /dev/null; do sleep 60; done
echo "[$(date '+%F %T')] redo finished; retraining BC under heldout_lap_caps [2.1, 2.7]"
mkdir -p checkpoints/old_split && mv checkpoints/bc_*_s?.pt* checkpoints/old_split/ 2>/dev/null || true
ros2 run hunter_il il_train_all --data-dir datasets/raw --config src/hunter_il/config/train_default.yaml \
    --datasets bc --seeds 0,1,2 --out checkpoints
echo "[$(date '+%F %T')] BC retrained"
