#!/bin/bash
# Retrain the discrete arm with the fixed bins and re-evaluate it closed-loop.
# The MLP and stacked arms are untouched -- only discrete used ActionBins.
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
export HUNTER_REPO="$PWD"
CFG=src/hunter_il/config/train_default.yaml
PLAN=src/hunter_dataset/config/collection_plan.yaml
say() { echo "[$(date '+%F %T')] $*"; }

say "archiving the broken-bin discrete checkpoints and their eval runs"
mkdir -p checkpoints/broken_bins results/eval_v1_broken_bins
mv checkpoints/bc_discrete_s?.pt* checkpoints/dagger_discrete_s?.pt* checkpoints/broken_bins/ 2>/dev/null || true
mv results/eval_v1/*eval_bc_discrete_s? results/eval_v1/*eval_dagger_discrete_s? results/eval_v1_broken_bins/ 2>/dev/null || true

say "retraining discrete: BC (3 seeds) then DAgger (3 seeds)"
ros2 run hunter_il il_train_all --data-dir datasets/raw --config $CFG \
    --datasets bc --archs discrete --seeds 0,1,2 --out checkpoints
ros2 run hunter_il il_train_all --data-dir datasets/raw --config $CFG \
    --datasets dagger --archs discrete --seeds 0,1,2 --out checkpoints

say "re-evaluating the discrete arm closed-loop on the three held-out tracks"
ros2 run hunter_dataset evaluate_policies --plan $PLAN --out results/eval_v1 --laps 1.5 \
    checkpoints/bc_discrete_s?.pt checkpoints/dagger_discrete_s?.pt

say "offline table for the rebuilt arm"
ros2 run hunter_il il_eval_offline --data-dir datasets/raw --config $CFG \
    --out results/eval_v1/offline_discrete.json \
    checkpoints/bc_discrete_s?.pt checkpoints/dagger_discrete_s?.pt
say "DISCRETE REDO DONE"
