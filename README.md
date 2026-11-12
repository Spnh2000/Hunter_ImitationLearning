# Learning to Race from Laser Data Alone

**Imitation learning for the AgileX Hunter V2 — behaviour cloning vs. DAgger, on ROS 2 Jazzy and Gazebo Harmonic.**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)
![PyTorch](https://img.shields.io/badge/PyTorch-CPU-EE4C2C?logo=pytorch&logoColor=white)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

Can a neural network drive a robot around a racetrack using **only a 2D laser scan**? This project trains one by imitating a PID controller that is allowed to see the robot's exact position in simulation (*privileged expert distillation*, Chen et al., [Learning by Cheating](https://arxiv.org/abs/1912.12294), CoRL 2019).

- **A laser-only policy drives three circuits it has never seen without a single crash**, 0.17 m from the racing line, against the expert's 0.02 m.
- **The same network trained by plain behaviour cloning (BC) crashes in 67 % of runs.** DAgger — letting the student drive while the expert labels its mistakes — makes the difference.
- **Offline validation error ranked the policies backwards.** Judged on recorded data, the crashing policy looks **5.6× better**. Only closed-loop evaluation showed which one works.

<p align="center">
  <img src="media/fig_headline.png" width="640" alt="Offline error favours behaviour cloning; in the simulator behaviour cloning crashes in most runs and DAgger in none">
</p>

**Contents:** [Results](#results) · [How it works](#how-it-works) · [Setup](#setup) · [Watching each policy](#watching-each-policy) · [Running the scripts](#running-the-scripts) · [Tracks](#tracks) · [Troubleshooting](#troubleshooting) · [Layout](#repository-layout) · [Further reading](#further-reading)

---

## Results

Each policy drove the three circuits it never saw in training — Silverstone, São Paulo, Montreal — once per training seed (3 seeds), at a 2.7 m/s speed cap, graded against Gazebo ground truth.

| policy | crash rate | laps completed | true lateral RMS | *offline steer error* |
|---|---:|---:|---:|---:|
| **PID expert** (drives on ground-truth pose) | 0 % | 1.00 | **0.019 m** | — |
| BC + MLP | **67 %** | 0.33 | 0.332 m | *19.4 mrad* |
| BC + stacked K=3 | **56 %** | 0.33 | 0.420 m | *19.7 mrad* |
| BC + discretised | **67 %** | 0.33 | 0.407 m | *21.5 mrad* |
| **DAgger + MLP** | **0 %** | **1.00** | **0.168 m** | *108.9 mrad* |
| DAgger + stacked K=3 | 0 % | 1.00 | 0.195 m | *160.4 mrad* |
| DAgger + discretised | 0 % | 1.00 | 0.249 m | *281.9 mrad* |
| **DAgger, rebalanced 50/50** | **0 %** | **1.00** | 0.175 m | *68.1 mrad* |

Print this table yourself with `scripts/show_results.sh`; every run is in [`results/eval_v1/eval_all.csv`](results/eval_v1/eval_all.csv).

- **Validation loss ranked the policies backwards.** Offline error averages over the expert's own states, almost all on the racing line; driving is decided by the states a policy reaches after its own mistakes — which only DAgger trains on.
- **Architecture made no difference**, as predicted before training: the current scan already determines the expert's steering, so a history of scans adds nothing.
- **DAgger fixed the failures it saw.** All 7 round-1 crashes happened at Shanghai's 1.44 m hairpin; after the expert labelled those states, rounds 3 and 4 ran 30 rollouts without a crash.
- **Rebalancing to a 50/50 BC:DAgger mix** cut the offline error from 108.9 to 68.1 mrad with an unchanged crash record.

<p align="center">
  <img src="media/fig_dagger.png" width="820" alt="DAgger rollouts per round, and the Shanghai hairpin where every round-1 crash happened">
</p>

**Limits.** Simulation only — transfer to the real robot is untested. One expert, five training circuits, three test circuits. The speed cap is given to the policy as one extra input.

---

## How it works

1. **The expert** — a PID cross-track controller with curvature and heading feedforward — drives on Gazebo's ground-truth pose and stays within 15–25 mm RMS of the line.
2. **The dataset** records, at 20 Hz, what the laser saw (720 ranges) and what the expert commanded (steering, speed): 8 circuits × 3 speed caps, ~13 h of driving. Five circuits train; **Silverstone, São Paulo and Montreal are held out**.
3. **The student** (`policy_tracker`) subscribes to `/scan` and nothing else, so it cannot read the pose even by mistake.
4. **DAgger** lets the student drive while a shadow expert with no command publisher records what it would have done. Five rounds, 75 rollouts.
5. **Evaluation** drives every policy on the held-out circuits and grades it against ground truth.

<p align="center">
  <img src="media/fig_tracks.png" width="820" alt="The eight circuits at twice F1TENTH scale; three are held out for testing">
</p>

---

## Setup

Requires Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Python 3.12 — or just Docker, which provides all of it (recommended).

### Docker (Linux)

```bash
docker compose build          # ~5-10 min the first time
xhost +local:docker           # once per login, lets Gazebo/RViz open windows
docker compose run --rm dev   # shell inside the container

# inside the container (the build persists between container runs):
colcon build --symlink-install
source install/setup.bash
```

### Docker (macOS)

X11 forwarding does not work well with Gazebo on macOS, so the Mac image streams a virtual desktop to your browser:

```bash
docker compose -f docker-compose.mac.yml build
docker compose -f docker-compose.mac.yml run --service-ports --rm dev   # --service-ports is required
```

Open <http://localhost:6080/vnc.html> and click **Connect** — Gazebo and RViz windows appear there. Inside the container, build as above. For a second shell: `docker compose -f docker-compose.mac.yml exec dev bash`.

### Host build (no Docker)

Install the ROS packages `control_toolbox`, `ros2bag`, `rosbag2_storage_mcap`, `robot_localization`, `slam_toolbox`, `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`, `topic_tools`, and the Python packages `numpy`, `h5py`, `pyyaml`, `matplotlib` and `torch` (CPU is enough), then:

```bash
colcon build --symlink-install
source install/setup.bash
```

> Docker and host builds must not share `build/`, `install/` or `log/` — delete them when switching.

---

## Watching each policy

Trained policies ship in [`checkpoints/final/`](checkpoints/final/), so no dataset or training is needed. One script launches Gazebo + RViz and lets the chosen policy drive:

```bash
scripts/watch_policy.sh <checkpoint|pid> [track] [speed_cap]
```

| argument | default | values |
|---|---|---|
| `checkpoint` | — (required) | a `.pt` file, or `pid` for the expert |
| `track` | `SaoPaulo` | `Silverstone`, `SaoPaulo`, `Montreal` (held out — never seen in training) · `Shanghai`, `Nuerburgring`, `Melbourne`, `Monza`, `IMS` (training tracks) |
| `speed_cap` | `2.1` | m/s; training used `1.5`, `2.1` and `2.7` |

From the host with Docker, prefix the command: `docker compose run --rm dev scripts/watch_policy.sh …` (build once first, as in [Setup](#setup)). Gazebo takes ~20 s to load before the robot spawns. Press **Ctrl-C** to stop. The run's bag and summary go to `results/watch_runs/` (gitignored). An RViz warning about `/map` having no publisher is expected — the policy needs no map.

### The expert

```bash
scripts/watch_policy.sh pid SaoPaulo 2.1
```

The PID teacher, reading its exact pose from the simulator. Holds about 2 cm from the line on every track.

### DAgger policies — drive cleanly

```bash
scripts/watch_policy.sh checkpoints/final/dagger_mlp_s0.pt     SaoPaulo    2.1
scripts/watch_policy.sh checkpoints/final/dagger_mlp_s1.pt     Montreal    2.7   # narrowest track
scripts/watch_policy.sh checkpoints/final/dagger_stacked_s0.pt Silverstone 2.7
```

Laser only, no pose. Expect a full lap with no crash, about 0.11–0.28 m from the line on the held-out tracks.

### BC policies — usually crash

```bash
scripts/watch_policy.sh checkpoints/final/bc_mlp_s0.pt     Montreal    2.7
scripts/watch_policy.sh checkpoints/final/bc_stacked_s1.pt Silverstone 2.7
```

Same networks, trained only on expert demonstrations. In the 2.7 m/s evaluation every shipped BC checkpoint crashed on Montreal and 5 of 6 crashed on Silverstone, while all of them finished São Paulo — so pick Montreal or Silverstone at 2.7 to see the failure.

### All shipped checkpoints

| file | data | architecture |
|---|---|---|
| `bc_mlp_s{0,1,2}.pt` | expert demonstrations only | MLP on the current scan |
| `bc_stacked_s{0,1,2}.pt` | expert demonstrations only | MLP on the last 3 scans |
| `dagger_mlp_s{0,1,2}.pt` | + DAgger corrections | MLP on the current scan |
| `dagger_stacked_s{0,1,2}.pt` | + DAgger corrections | MLP on the last 3 scans |

`s0/s1/s2` are training seeds; each `.pt.json` next to a checkpoint holds its scores and training history. The architecture is read from the checkpoint automatically.

**Not shipped:** the discretised (`bc_discrete_*`, `dagger_discrete_*`), rebalanced (`balanced_mlp_*`) and per-round DAgger (`dagger_mlp_it*`) weights — only their `.pt.json` metrics are in the repo. To watch them, train them first (needs the dataset, see [below](#training-and-evaluation-pipeline)), then pass the new file, e.g. `scripts/watch_policy.sh checkpoints/balanced_mlp_s0.pt Montreal 2.7`.

---

## Running the scripts

All scripts are run from the repository root (they `cd` there themselves), on the host or inside the container.

| script | needs | what it does |
|---|---|---|
| `scripts/watch_policy.sh` | build | Watch one policy or the expert drive, with GUI ([above](#watching-each-policy)) |
| `scripts/show_results.sh` | nothing | Print the results table from `results/eval_v1/`; `--raw` prints every run |
| `python3 scripts/make_figures.py` | numpy, matplotlib | Regenerate the figures in `media/` from tracked results; `--out DIR --format pdf` for vector output |
| `scripts/check_scripts.sh` | nothing | Syntax-check every shell script in `scripts/` |
| `scripts/phase8_pipeline.sh` | build + dataset | DAgger (5 rounds) → diagnostics → train all policies → closed-loop + offline evaluation. Resumable |
| `scripts/rebalance_experiment.sh` | build + dataset | Train `balanced_mlp_s{0,1,2}` (50/50 BC:DAgger) and evaluate them |

```bash
scripts/show_results.sh                    # the results table
scripts/show_results.sh --raw              # one line per evaluation run
python3 scripts/make_figures.py            # -> media/*.png
scripts/check_scripts.sh                   # "all scripts parse"
```

The two pipeline scripts run for many hours. Run them in the background and keep the machine from sleeping (a suspended host freezes simulation time and the run stalls):

```bash
nohup systemd-inhibit --what=sleep:idle scripts/phase8_pipeline.sh > results/phase8_pipeline.log 2>&1 &
nohup scripts/rebalance_experiment.sh > results/rebalance.log 2>&1 &
```

Every step skips work that already exists, so re-running a script resumes it. Arguments to `phase8_pipeline.sh` are passed to `dagger_runner` (e.g. `--skip-train` when resuming a DAgger round).

### Training and evaluation pipeline

The training data (~2.3 GB of HDF5) is not in the repository; the first step recreates it. Each step is a `ros2 run` command, in this order:

```bash
PLAN=src/hunter_dataset/config/collection_plan.yaml
CFG=src/hunter_il/config/train_default.yaml

# 1. Collect expert demonstrations (~20 h of simulation, resumable)
ros2 run hunter_dataset collect_dataset --plan $PLAN --pass p1

# 2. Pre-training diagnostics (information asymmetry, speed and coverage plots)
ros2 run hunter_il il_diagnostics --data-dir datasets/raw --out diagnostics/

# 3. Train the BC policies (3 architectures x 3 seeds)
ros2 run hunter_il il_train_all --data-dir datasets/raw --config $CFG --datasets bc --seeds 0,1,2 --out checkpoints

# 4. DAgger: 5 rounds of student rollouts labelled by the shadow expert
ros2 run hunter_dataset dagger_runner --plan $PLAN --train-config $CFG --arch mlp \
    --iterations 5 --laps 1.0 --data-dir datasets/raw --ckpt-dir checkpoints

# 5. Train the DAgger policies on everything collected
ros2 run hunter_il il_train_all --data-dir datasets/raw --config $CFG --datasets dagger --seeds 0,1,2 --out checkpoints

# 6. Closed-loop evaluation on the held-out tracks (~20 min per run)
ros2 run hunter_dataset evaluate_policies --plan $PLAN --out results/my_eval --laps 1.5 \
    pid checkpoints/bc_*_s?.pt checkpoints/dagger_*_s?.pt

# 7. Offline validation error, reported beside the closed-loop table
ros2 run hunter_il il_eval_offline --data-dir datasets/raw --config $CFG \
    --out results/my_eval/offline.json checkpoints/bc_*_s?.pt checkpoints/dagger_*_s?.pt
```

Steps 4–7 are what `scripts/phase8_pipeline.sh` runs. To train one policy only:

```bash
ros2 run hunter_il il_train --config $CFG --arch discrete --data-dir datasets/raw \
    --out checkpoints/dagger_discrete_s0.pt --seed 0 --tag dagger
```

`--arch` is `mlp`, `stacked` or `discrete`; add `--balance` for the 50/50 mix. Without the dataset you can still re-run the evaluation on the shipped checkpoints (step 6 with `checkpoints/final/*.pt`, about 12 h).

### Running the expert without the learning stack

```bash
# Expert benchmark on the default IMS track: Gazebo + trajectory server + PID + metrics
ros2 launch hunter_benchmark benchmark.launch.py

# Another track -- world, track_file, params_file and track_name must all match
ros2 launch hunter_benchmark benchmark.launch.py \
    world:=Nuerburgring.sdf \
    track_file:=src/race_tracks/Nuerburgring/Nuerburgring_centerline_2x.csv \
    params_file:=nuerburgring_defaults.yaml track_name:=Nuerburgring

# No display
ros2 launch hunter_benchmark benchmark.launch.py headless:=true use_rviz:=false

# Simulation only, no controller
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```

Results (bag + `summary.csv`) are written under `src/hunter_benchmark/results/`.

---

## Tracks

Eight [F1TENTH](https://github.com/f1tenth/f1tenth_racetracks) circuits, scaled 2× so the 0.98 m-long Hunter (1.31 m minimum turning radius) fits through every corner.

| split | tracks |
|---|---|
| training | Shanghai, Nuerburgring, Melbourne, Monza, IMS |
| held out (testing only) | **Silverstone, SaoPaulo, Montreal** |

Montreal is the hardest: 0.66 m of margin to the walls, less than half of any other track. Track geometry, the scaling rationale and how to add a new track are in the [technical reference](docs/TECHNICAL_REFERENCE.md#tracks-and-the-2-scale).

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Gazebo/RViz window doesn't appear | Run `xhost +local:docker` on the host |
| Gazebo crashes at startup | Set `LIBGL_ALWAYS_SOFTWARE=1`, or launch from a plain terminal rather than VS Code's |
| `ros_gz_sim create` hangs on "Requesting list of world names" | `export GZ_IP=127.0.0.1` |
| Robot drives into a wall with wheels still turning | A previous run is still alive. Check `ros2 node list`; stop leftovers with `pkill -9 -x pid_tracker; pkill -9 -x trajectory_server` |
| Launch aborts with `TRACK / WORLD SCALE MISMATCH` | Use the `_2x` centerline CSV that matches the world |
| `CMakeCache.txt` path mismatch | Mixed Docker and host builds — delete `build/`, `install/`, `log/` and rebuild |
| `PermissionError` writing under `results/` or `src/` | Files are root-owned from a container run: `docker run --rm --entrypoint chown -v "$PWD:/w" hunter_robot:jazzy -R "$(id -u):$(id -g)" /w/results /w/src /w/datasets` |
| `watch_policy.sh`: `ModuleNotFoundError: torch` on the host | Install PyTorch for the system Python (`/usr/bin/python3`), or run the script in Docker |

More cases in the [technical reference](docs/TECHNICAL_REFERENCE.md#troubleshooting).

---

## Repository layout

```
src/
├── hunter_description/   # robot URDF (LiDAR, IMU, contact sensor), ros2_control, RViz
├── hunter_gazebo/        # eight world SDFs (2x scale), simulation launch
├── hunter_msgs/          # custom messages
├── hunter_trajectory/    # trajectory_server node
├── hunter_control/       # pid_tracker (expert), pid_shadow (DAgger labeller), policy_tracker (student)
├── hunter_benchmark/     # benchmark launch, per-track params, metrics
├── hunter_localization/  # EKF + AMCL + slam_toolbox configs
├── hunter_dataset/       # recorder, converter, collection + DAgger runners, evaluator
├── hunter_il/            # training: models, data loading, diagnostics (PyTorch, no ROS)
├── race_tracks/          # the eight circuits and their 2x versions
├── scale_track.py        # track scaling tool
└── my_convert_track.py   # occupancy map -> Gazebo world
checkpoints/final/        # the trained policies the results come from
results/eval_v1/          # every closed-loop evaluation run
datasets/raw/             # run manifests and reports (HDF5 data not in git)
diagnostics/              # information-asymmetry and coverage analyses
scripts/                  # see "Running the scripts"
docs/                     # report, explainers, technical reference
media/                    # README figures
docker/                   # Dockerfiles + entrypoints
```

---

## Further reading

| Document | For |
|---|---|
| [QUICKSTART.md](QUICKSTART.md) | Clone to a policy driving an unseen circuit, in three commands |
| [docs/PROJECT_REPORT.md](docs/PROJECT_REPORT.md) | The whole project as a standalone report |
| [docs/phases_6_8_simple.md](docs/phases_6_8_simple.md) | The learning phases in plain language |
| [docs/phases_6_8_explained.md](docs/phases_6_8_explained.md) | The learning phases in technical detail |
| [docs/TECHNICAL_REFERENCE.md](docs/TECHNICAL_REFERENCE.md) | Tracks, sensing, dataset schema, controller, ROS topics, known limitations |

## Citation

If you use this code or its results, please cite it — GitHub's **"Cite this repository"** button reads [`CITATION.cff`](CITATION.cff).

## License

[Apache License 2.0](LICENSE). Copyright 2026 Spnh2000 and simaes.

`src/hunter_description` derives from the University of Lincoln's Hunter description package (Ibrahim Hroob, Hariharan Arunachalam), also Apache-2.0.

## Acknowledgements

- Track layouts: [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks).
- Robot model: University of Lincoln Hunter description package.
- Method: D. Chen *et al.*, "Learning by Cheating", CoRL 2019; S. Ross *et al.*, "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning" (DAgger), AISTATS 2011.
