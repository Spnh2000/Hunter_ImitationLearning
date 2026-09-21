#!/bin/bash
# Phase 8 end to end, after collection pass p1. Every step is resumable on its
# own (each skips finished work), so a failed step is re-run by re-running
# this script; set -e stops the chain at the first failure instead of
# feeding a broken artefact to the next step.
#
#   nohup scripts/phase8_pipeline.sh > results/phase8_pipeline.log 2>&1 &
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
export HUNTER_REPO="$PWD"
PLAN=src/hunter_dataset/config/collection_plan.yaml
CFG=src/hunter_il/config/train_default.yaml
say() { echo "[$(date '+%F %T')] $*"; }

say "waiting for collect_dataset (pass p1) to finish"
while pgrep -f 'collect_dataset --plan' > /dev/null; do sleep 120; done
say "p1 done. run statuses:"
for r in datasets/raw/*_p1/collection_report.json; do
  printf '   %-40s %s\n' "$(basename "$(dirname "$r")")" "$(grep -o '"status": "[^"]*"' "$r" | head -1)"
done
BAD=$(grep -L '"status": "ok"' datasets/raw/*_p1/collection_report.json | wc -l)
say "$BAD p1 runs not ok (continuing; il_train uses whatever converted)"

say "=== Part D: diagnostics on p1 ==="
ros2 run hunter_il il_diagnostics --data-dir datasets/raw --out diagnostics/p1

say "=== BC training: 3 archs x 3 seeds on expert data ==="
ros2 run hunter_il il_train_all --data-dir datasets/raw --config $CFG --datasets bc --seeds 0,1,2 --out checkpoints

say "=== DAgger: 5 iterations, MLP student, 1 lap per training track x speed ==="
ros2 run hunter_dataset dagger_runner --plan $PLAN --train-config $CFG --arch mlp \
    --iterations 5 --laps 1.0 --data-dir datasets/raw --ckpt-dir checkpoints

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
