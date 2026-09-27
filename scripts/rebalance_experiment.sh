#!/bin/bash
# Does DAgger's offline degradation come from the DATA or from the MIX?
#
# Unbalanced, DAgger rows are 75 % of the training set and the model's
# agreement with the expert on ordinary states falls apart: held-out-track
# steering RMSE 19.4 mrad (BC only) -> 108.9 mrad (DAgger), even though the
# DAgger policy is the one that actually completes laps. This trains the same
# architecture on the same rows with a 50/50 BC:DAgger batch mix and evaluates
# it the same way, so the three-way comparison is like-for-like:
#
#   bc_mlp         expert demonstrations only
#   dagger_mlp     + DAgger, natural 23/77 mix
#   balanced_mlp   + DAgger, forced 50/50 mix
#
# MLP only, 3 seeds: the architecture axis is already settled (no effect), so
# spending simulator time on stacked/discrete here would buy nothing.
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
export HUNTER_REPO="$PWD"
CFG=src/hunter_il/config/train_default.yaml
PLAN=src/hunter_dataset/config/collection_plan.yaml
say() { echo "[$(date '+%F %T')] $*"; }

say "waiting for the discrete redo to finish (they would fight for cores)"
while pgrep -f 'bash scripts/redo_discrete.sh' > /dev/null; do sleep 120; done
say "discrete redo done"

for seed in 0 1 2; do
  out=checkpoints/balanced_mlp_s${seed}.pt
  if [ -f "$out.json" ]; then say "skip $out (exists)"; continue; fi
  say "training $out (50/50 BC:DAgger)"
  ros2 run hunter_il il_train --config $CFG --arch mlp --data-dir datasets/raw \
      --out "$out" --seed "$seed" --tag balanced --balance
done

say "closed-loop on the three held-out tracks"
ros2 run hunter_dataset evaluate_policies --plan $PLAN --out results/eval_v1 --laps 1.5 \
    checkpoints/balanced_mlp_s?.pt

say "offline table for the balanced arm"
ros2 run hunter_il il_eval_offline --data-dir datasets/raw --config $CFG \
    --out results/eval_v1/offline_balanced.json checkpoints/balanced_mlp_s?.pt
say "REBALANCE EXPERIMENT DONE"
