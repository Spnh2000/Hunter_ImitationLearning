#!/bin/bash
# Phase 8 from the DAgger stage onward. p1 collection, Part D and BC training
# are already done (see results/phase8_pipeline.log and checkpoints/bc_*).
#
#   nohup scripts/phase8_pipeline.sh > results/phase8_pipeline2.log 2>&1 &
#
# Every step skips finished work, so re-running the script resumes it.
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
export HUNTER_REPO="$PWD"
PLAN=src/hunter_dataset/config/collection_plan.yaml
CFG=src/hunter_il/config/train_default.yaml
say() { echo "[$(date '+%F %T')] $*"; }

say "=== DAgger: 5 iterations, MLP student, 1 lap per training track x speed (IMS 0.5) ==="
# "$@" carries --skip-train through from resume_dagger.sh, so a resumed
# iteration keeps the student it started with rather than retraining on its own
# half-finished rollouts.
ros2 run hunter_dataset dagger_runner --plan $PLAN --train-config $CFG --arch mlp \
    --iterations 5 --laps 1.0 --data-dir datasets/raw --ckpt-dir checkpoints "$@"

say "=== Part D again, now with DAgger coverage ==="
ros2 run hunter_il il_diagnostics --data-dir datasets/raw --out diagnostics/p1_dagger

say "=== DAgger-data training: 3 archs x 3 seeds on everything ==="
ros2 run hunter_il il_train_all --data-dir datasets/raw --config $CFG --datasets dagger --seeds 0,1,2 --out checkpoints

say "=== Part E: closed-loop on the held-out tracks, PID reference + 18 policies ==="
ros2 run hunter_dataset evaluate_policies --plan $PLAN --out results/eval_v1 --laps 1.5 \
    pid checkpoints/bc_*_s?.pt checkpoints/dagger_*_s?.pt

say "=== offline validation table, beside the closed-loop one ==="
ros2 run hunter_il il_eval_offline --data-dir datasets/raw --config $CFG --out results/eval_v1/offline.json \
    checkpoints/bc_*_s?.pt checkpoints/dagger_*_s?.pt
say "PIPELINE DONE -> results/eval_v1/eval.csv, results/eval_v1/offline.json, diagnostics/"
