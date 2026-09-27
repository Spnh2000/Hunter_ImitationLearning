# Quickstart — watch a trained policy drive

Three commands from a fresh clone to a robot driving a racetrack on laser data
alone. No dataset download, no training.

## 1. Build

```bash
git clone https://github.com/Spnh2000/Hunter_RL.git && cd Hunter_RL
docker compose build && xhost +local:docker && docker compose run --rm dev
# inside the container:
colcon build --symlink-install && source install/setup.bash
```

Host build works too if you have ROS 2 Jazzy and Gazebo Harmonic — see the
[README](README.md#requirements).

## 2. Watch the trained policy

```bash
scripts/watch_policy.sh checkpoints/final/dagger_mlp_s0.pt SaoPaulo 2.1
```

Gazebo and RViz open. The robot drives São Paulo — **a circuit this policy has
never seen** — using nothing but its laser scanner. No map, no GPS, no
localization. It should complete the lap holding about 0.17 m from the
reference line.

## 3. Compare against the expert, and against plain copying

```bash
# the teacher: a PID controller reading its exact position from the simulator
scripts/watch_policy.sh pid SaoPaulo 2.1

# the same network trained WITHOUT DAgger -- this one usually crashes
scripts/watch_policy.sh checkpoints/final/bc_mlp_s0.pt Montreal 2.7
```

That third command is the point of the project. Same architecture, same amount
of training, same everything except how the training data was collected — and
it crashes on 67 % of runs where the DAgger policy crashes on none.

## What is in `checkpoints/final/`

| file | trained on |
|---|---|
| `bc_mlp_s*.pt`, `bc_stacked_s*.pt` | expert demonstrations only (behaviour cloning) |
| `dagger_mlp_s*.pt`, `dagger_stacked_s*.pt` | + expert corrections of the student's own mistakes (DAgger) |

`s0/s1/s2` are training seeds. Each `.pt.json` holds that model's scores and
full training history.

Held-out circuits — never trained on, so the honest test — are **Silverstone,
SaoPaulo and Montreal**. Montreal is the hardest: 0.66 m of margin, less than
half of every other track.

## Reproducing the numbers

The dataset (3.7 GB of HDF5) is not in the repo. Without it you can still watch
any policy drive and re-run the closed-loop evaluation:

```bash
ros2 run hunter_dataset evaluate_policies \
    --plan src/hunter_dataset/config/collection_plan.yaml \
    --out results/my_eval --laps 1.5 pid checkpoints/final/*.pt
```

~30 h for all policies; a single `<policy> <track>` pair is about 30 minutes.

To rebuild the dataset from scratch (~20 h of simulation):

```bash
ros2 run hunter_dataset collect_dataset \
    --plan src/hunter_dataset/config/collection_plan.yaml --pass p1
```

## Where to read next

- [`docs/phases_6_8_simple.md`](docs/phases_6_8_simple.md) — the whole project
  in plain language, no machine-learning background assumed
- [`docs/phases_6_8_explained.md`](docs/phases_6_8_explained.md) — the
  technical reference
- [README](README.md#results) — the results table
