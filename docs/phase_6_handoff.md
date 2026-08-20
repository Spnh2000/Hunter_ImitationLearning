# Phase 6 Handoff

**Hunter V2 · ROS 2 Jazzy · Gazebo Harmonic**

Where the project stands after Phase 5, and exactly what has to be true before
imitation-learning data collection begins.

| | |
|---|---|
| Branch | `dev-saqar` @ `5d9b7d4` |
| Packages | 8 |
| Tracks driveable | 2 of 8 available |

---

## 1. Project state

| Phase | Contents | State |
|---|---|---|
| 1–2 | Robot bring-up, track conversion pipeline | done |
| 3 | Trajectory server, PID tracker, metrics logger | done |
| 5 | LiDAR + IMU, dual EKF, SLAM mapping, AMCL, pose-source A/B | done |
| **6** | **Imitation dataset collection** | **next** |
| 4, 7–8 | Pure Pursuit / Stanley / LQR / MPC, then offline RL | pending |
| 9 | Sim-to-sim policy transfer | pending |

### Packages

| Package | Role |
|---|---|
| `hunter_description` | URDF/xacro, sensors, ros2_control config, RViz |
| `hunter_gazebo` | IMS + Monza world SDFs, simulation launch |
| `hunter_msgs` | `Waypoint`, `Trajectory`, `TrackingDiagnostics` |
| `hunter_trajectory` | `trajectory_server` — CSV → reference trajectory |
| `hunter_control` | `pid_tracker` + shared geometry library |
| `hunter_benchmark` | `metrics_logger`, `summarize_run.py`, benchmark launch, per-track params |
| `hunter_localization` | Dual-EKF, slam_toolbox and AMCL configs + launch |

---

## 2. What the controller actually does

All figures measured against **Gazebo ground truth**, not the robot's own estimate.
That distinction matters more than anything else in this document.

| Pose source | True RMS error | Estimator error | Verdict |
|---|---:|---:|---|
| Ground truth | 0.003 m | 0 | reference |
| AMCL + dual EKF | 0.003 m | 0.66 m * | works |
| Wheel odometry | 1.38 m | 24.2 m | drifts |

\* Flat, not growing (max 0.72 m, ratio 1.09) — consistent with a constant map-to-world
frame offset rather than drift. **Unverified**; worth chasing before Phase 8, harmless
for Phase 6.

> **The number that justified the whole phase.** In one run the controller reported
> **0.2 mm** RMS tracking error while sitting **0.57 m** off the line. Without a
> ground-truth channel, that broken run would have looked like the best result of the three.

At 2.7 m/s the PID holds zero steering saturation and completes laps without stalling.
It is a smooth expert — steering reversal rate `0.0`, command variance `7e-05` — which is
what matters for distillation, since jitter in the teacher becomes jitter in the student.

---

## 3. The two driveable tracks

Both scaled 2×. F1TENTH circuits are drawn for 0.3 m-long cars; the Hunter is 0.98 m long
with a **1.31 m minimum turning radius**.

| | IMS | Monza |
|---|---:|---:|
| Track length | 586 m | 892 m |
| Raw minimum radius | 7.8 m | 0.74 m |
| Minimum radius, smoothed | 28.1 m | 2.21 m |
| Margin after robot half-width | 1.50 m | 1.37 m |
| Speed profile binds | never | 10.8 % |
| Minimum commanded speed | 2.7 m/s | 0.94 m/s |

> **Why Monza had to exist.** On IMS the curvature speed limit never engages, so commanded
> speed is a flat 2.7 m/s for the entire lap. A dataset collected there has **zero speed
> variance**: the student's speed output would learn to emit one constant and show excellent
> loss while having learned nothing. You would not discover it until Phase 9.

Monza's raw 0.74 m corners are roughly half what the vehicle can turn, so it is undrivable
at 1× — the scaling is a hard requirement there, not a comfort margin.

---

## 4. Sensing, and why it transfers

| | f1tenth_gym | Hunter |
|---|---|---|
| Field of view | 270° | 270° (±2.35619 rad) |
| Beams | 1080 | 1080 |
| Max range | 30 m | 30 m |
| Rate | — | 20 Hz, 0.01 m noise |

A gym-trained policy's observation vector transfers without reshaping. The 2× track scaling
also puts the Hunter within **1.5 %** of an F1TENTH car's effective wheelbase in training
units (0.325 m vs 0.330 m) — an accident of the drift-margin decision, but a useful one
for Phase 9.

---

## 5. Six silent failures found

Every one reported healthy while producing wrong answers. Recorded because the pattern
will recur.

1. **slam_toolbox is a lifecycle node.** Launched as a plain `Node` it sits unconfigured
   forever — process alive, topics listed, publisher count zero.
2. **`transform_publish_period: 0.0`** disables the pose output, not just the TF. The global
   EKF then has no absolute measurement and quietly becomes dead reckoning.
3. **Wrong executable per mode.** `async_slam_toolbox_node` has no pose-publishing code path
   at all; only `localization_slam_toolbox_node` does.
4. **`map_file` defaulted to IMS** regardless of track, so launching Monza loaded IMS's map
   and localized against geometry that wasn't there.
5. **Duplicate trackers.** Two `pid_tracker` processes both publish at 50 Hz; the robot
   drifts sideways and wedges with its wheels still turning.
6. **Root-owned `results/`** from container runs crashes the metrics logger and blocks
   branch switches.

> **Operating rule.** Validate physical behaviour against Gazebo ground truth or the GUI.
> Never conclude a run succeeded from `/tracking/diagnostics` alone — that topic reports what
> the robot *believes*, which is exactly what fails first.

---

## 6. Before Phase 6 starts

### Gate 1 — Fix the results directory

Root-owned from Docker runs, which crashes `metrics_logger` mid-collection.

```bash
sudo chown -R $USER:$USER src/hunter_benchmark/results
```

### Gate 2 — Commit the working tree

46 files uncommitted, including `amcl.yaml` and the Monza track. The branch is pushed,
but this work is not.

### Gate 3 — Convert six more tracks

Melbourne, Montreal, Nuerburgring, SaoPaulo, Shanghai and Silverstone are already in
`src/f1tenth_racetracks/` with all three required files.

Two tracks cannot support a held-out-track validation split, and held-out *laps* are
near-duplicates that measure memorization instead of generalization.

```bash
python3 src/scale_track.py Silverstone 2.0
python3 src/my_convert_track.py \
    f1tenth_racetracks/Silverstone/Silverstone_map.png \
    f1tenth_racetracks/Silverstone/Silverstone_map_2x.yaml \
    -o hunter_gazebo/worlds/Silverstone.sdf \
    --smoothness 0.0003 --thickness 0.06
```

Then check the track is driveable at all — smoothed minimum radius must exceed 1.31 m —
and that its speed profile binds somewhere.

### Gate 4 — Confirm speed variance exists ✔

Done for Monza: the profile binds on 10.8 % of the track, down to 0.94 m/s. Check each
newly converted track the same way — an all-oval set reintroduces the constant-speed problem.

---

## 7. Phase 6, step by step

Privileged expert distillation: the expert drives on ground truth, the student sees only
`/scan`. No pose estimate is needed at deployment, which is why the deferred SLAM
localization does not block anything here.

### Step 1 — Settle the schema decisions first

Three choices are expensive to reverse once data exists:

- the 1080 → 720 downsampling method
- whether the dataset is stored at scan rate (20 Hz) or control rate (50 Hz)
- whether actions are physical units or normalized

Wrong answers mean re-driving every lap.

### Step 2 — Build the recorder

Record rosbag2 raw, then derive the HDF5 dataset with a versioned converter. This still
gives one canonical downsampling, but a schema change becomes a re-run of a script instead
of a re-collection.

### Step 3 — Collect the base dataset

Expert on ground truth, 10–15 laps per track per speed setting, three speed settings.
Seven tracks train, three held out entirely.

### Step 4 — Run the quality checks before training anything

Action distribution sanity, state coverage, and the information-asymmetry measurement —
k-nearest-neighbour action variance in observation space. If near-identical scans carry
different expert actions, the student learns their average and drives badly.

### Step 5 — Add the DAgger loop

Iteration 0 is plain behaviour cloning. From iteration 1, roll out the student and let the
expert relabel visited states. Verify it works by watching the lateral-error distribution
widen across iterations — if it doesn't, you are doing expensive cloning.

> **Expert choice.** Use the PID now rather than waiting for Phase 4's controllers.
> Smoothness matters more than lap time in distillation, and the PID already measures
> smooth: zero steering reversals, `7e-05` command variance. Record a `controller_label`
> so a Stanley dataset can be compared or mixed in later.

---

## 8. Running the stack today

```bash
# PID on Monza, ground truth, no SLAM -- the Phase 6 expert configuration
ros2 launch hunter_benchmark benchmark.launch.py \
    world:=Monza.sdf \
    track_file:=src/Monza/Monza_centerline_2x.csv \
    params_file:=monza_defaults.yaml \
    track_name:=Monza

# headless, for batch collection
    ... headless:=true use_rviz:=false

# watch the drift for yourself
    ... pose_source:=wheel_odom

# AMCL localization (needs a complete map for the track)
    ... pose_source:=slam_ekf slam_mode:=localization
```

The four track arguments move together. The launch aborts on a world/CSV scale mismatch,
but **cannot** detect wrong *params* — IMS params on Monza would silently apply IMS's speed
profile to Monza's corners.

---

*Hunter V2 autonomous racing stack · Phases 1–5 complete · Prepared as a Phase 6 starting point.*
