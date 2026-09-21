# Phase 6 Handoff

**Hunter V2 · ROS 2 Jazzy · Gazebo Harmonic**

Where the project stands after Phase 5, and exactly what has to be true before
imitation-learning data collection begins.

| | |
|---|---|
| Branch | `dev-saqar` @ `5d9b7d4` |
| Packages | 8 |
| Tracks driveable | **8** |

---

## 0. Update, 2026-09-19 — Steps 1–3 done, gate PASSED

Scope changed since this document was written: **Phase 4 is dropped** (PID is
the permanent expert) and **Phases 7–8 are pure imitation learning**, not
offline RL — {BC, DAgger} × {MLP, frame-stacked MLP, discretised} = 6 policies
evaluated closed-loop against PID. The design for that is in
`docs/phase_8_design.md`. The eight-track plan is now 5 train / 3 held out,
IMS capped at 1–2 laps.

What exists now, all under `src/hunter_dataset/`:

| piece | state |
|---|---|
| `dataset_spec.py` — schema, zero ROS imports, test-enforced | done |
| `record_expert.launch.py` — bridges, `episode_monitor`, contact throttle, 12-topic bag, `run_manifest.yaml` written before driving | done |
| `convert_dataset` — searchsorted join, exact-stamp episode events, converter report | done |
| `inspect_dataset` — PASS/FAIL per check, loud constant-column banner, nonzero exit | done |
| **Step 3 gate on Nuerburgring** | **PASS** — 7,544 rows, N = 20 × 377 s exactly, join Δt median 9.5 / p95 10 / max 40 ms, 2 rejects, 7,544/7,544 exact event matches, speed 1.10–2.70 m/s (std 0.46), steering std 0.05 |

The gate caught one real bug. `pid_tracker` ran its control loop on a
**wall-clock** timer with no `use_sim_time`: 170 ticks per simulated second
at this host's RTF instead of 50, a fixed `dt = 1/50` scaling the effective
integral gain up and derivative gain down by 1/RTF, and diagnostics stamped
on a different clock from `/scan` (0 of 7,549 rows joined). Fixed with
`create_timer` on the node clock plus `use_sim_time` on `trajectory_server`,
the tracker and `metrics_logger`. Tracking quality before/after on the same
lap: |lateral| median 5.8 → 4.8 mm, p95 37 → 33 mm, max 63 → 72 mm — the
expert is unchanged to within noise, and now independent of host load.

Two things must be true before Step 4 (full collection):

1. **Commit.** The gate dataset carries `git_dirty=True`; every collected
   file's `git_rev` must describe the code that produced it.
2. **The runner owns the launch PID.** Three `metrics_logger` zombies from
   three runs were alive at once during the gate — Python nodes survive
   name-based kill loops. SIGINT the launch, assert `ros2 node list` empty.

Sections 6–7 below predate this update; Gates 3–4 and Steps 1–3 are done.

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
| `hunter_gazebo` | Eight track world SDFs, simulation launch |
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

## 3. The eight driveable tracks

All scaled 2x. F1TENTH circuits are drawn for 0.3 m-long cars; the Hunter is 0.98 m long
with a **1.31 m minimum turning radius**.

| Track | Length | R_min | Clearance | Margin | Speed binds | Min speed |
|---|---:|---:|---:|---:|---:|---:|
| IMS | 586 m | 28.07 m | 1.87 m | 1.50 m | **0 %** | 2.70 m/s |
| Monza | 892 m | 2.21 m | 1.75 m | 1.37 m | 10.8 % | 0.87 m/s |
| Melbourne | 949 m | 2.51 m | 2.10 m | 1.73 m | 12.4 % | 0.94 m/s |
| Montreal | 570 m | 2.20 m | 1.03 m | **0.66 m** | 18.6 % | 0.87 m/s |
| Nuerburgring | 892 m | 2.74 m | 2.08 m | 1.71 m | 21.9 % | 1.00 m/s |
| SaoPaulo | 689 m | 3.17 m | 2.11 m | 1.73 m | 21.3 % | 1.09 m/s |
| Shanghai | 995 m | **1.44 m** | 2.22 m | 1.84 m | 22.6 % | 0.63 m/s |
| Silverstone | 916 m | 2.61 m | 1.85 m | 1.48 m | 21.0 % | 0.96 m/s |

All eight clear the 1.31 m kinematic gate. Two deserve care:

- **Montreal** — margin 0.66 m, less than half the others, because its corridor is narrow
  (1.03 m clearance). Cornering drift reached 2.29 m in Phase 5 testing, so expect wall
  contact and a lower clean-lap rate.
- **Shanghai** — R_min 1.44 m against the vehicle's 1.31 m limit, only 10 % headroom. The
  robot will be at near-full steering lock in its tightest corner.

Both would be comfortable at 3x if they prove troublesome.

### Why 2x, specifically

Three independent reasons converge on the same factor.

**Kinematics.** Monza's raw 0.74 m corners are roughly half what the vehicle can turn, so
it is undrivable at 1x. The scaling is a hard requirement there, not a comfort margin.

**Drift margin.** Wheel odometry drifts without bound, so the corridor must absorb pose
error the controller cannot see.

**Proportion.** At 2x the robot occupies the same fraction of the circuit that a real F1
car does:

| | Lap length / car length | Corridor / car width |
|---|---:|---:|
| Real F1 at Monza | 1029 | 6.0 |
| **Hunter at Monza 2x** | **910** | **5.9** |
| Hunter at Monza 1x | 455 | 3.0 |
| Hunter at Monza 3x | 1365 | — |
| F1TENTH at Monza 1x | 1487 | — |

88 % of real F1's lengthwise proportion, within 2 % widthwise. At 1x the car is more than
twice too big for the track; at 3x it would overshoot toward the F1TENTH end of the scale.

A related consequence for Phase 9: at 2x the Hunter's effective wheelbase in training units
is 0.325 m against an F1TENTH's 0.330 m -- a **1.5 % match**. The tracks are
F1-proportioned *and* the vehicle behaves like the car gym policies were trained on.

### Speed variance -- confirmed, not assumed

Measured from the trajectory `trajectory_server` actually publishes, not a re-derivation:

| Track | Waypoints | Min | Max | Std dev | Below max |
|---|---:|---:|---:|---:|---:|
| IMS | 2931 | 2.700 | 2.700 | **0.0000** | **0.0 %** |
| Monza | 4461 | 0.865 | 2.700 | 0.2742 | 10.8 % |
| Melbourne | 4743 | 0.943 | 2.700 | 0.3201 | 12.4 % |
| Montreal | 2851 | 0.867 | 2.700 | 0.4366 | 18.6 % |
| Nuerburgring | 4462 | 0.999 | 2.700 | 0.3599 | 21.9 % |
| SaoPaulo | 3447 | 1.087 | 2.700 | 0.3634 | 21.3 % |
| Shanghai | 4977 | 0.633 | 2.700 | 0.3541 | 22.6 % |
| Silverstone | 4580 | 0.964 | 2.700 | 0.3227 | 21.0 % |

**Seven of eight carry genuine speed variation.** IMS is confirmed degenerate: standard
deviation exactly zero, every waypoint at 2.70 m/s.

> **Do not train the speed head on IMS alone.** With zero variance the student learns to
> emit one constant and reports excellent loss while having learned nothing about speed
> control. You would not discover it until Phase 9.

> **Verification note.** Two earlier attempts at this table produced plausible but wrong
> numbers: `pkill -f trajectory_server` matched its own invoking shell, and `ros2 run` is a
> wrapper whose death leaves the real node alive, so each probe received the *previous*
> track's latched `transient_local` message. Both were caught only by cross-checking
> waypoint counts against `length / 0.2 m`. Always validate the count.

## 4. Sensing, and why it transfers

| | f1tenth_gym | Hunter |
|---|---|---|
| Field of view | 270° | 270° (±2.35619 rad) |
| Beams | 1080 | 1080 |
| Max range | 30 m | 30 m |
| Rate | — | 20 Hz, 0.01 m noise |

A gym-trained policy's observation vector transfers without reshaping (see section 3 for the
proportion analysis, including the 1.5 % effective-wheelbase match to an F1TENTH car).

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

### Gate 3 — Convert six more tracks ✔

Done. Melbourne, Montreal, Nuerburgring, SaoPaulo, Shanghai and Silverstone are converted,
worlds generated, and per-track `<track>_defaults.yaml` written. Eight tracks total, enough
for a held-out-track split (held-out *laps* are near-duplicates that measure memorization
rather than generalization).

Converting another track:

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

Done for all eight (see section 3). Seven carry real variation, binding on 10.8–22.6 % of
the lap down to 0.63–1.09 m/s. **IMS is degenerate** — std dev exactly zero — and must not
be the only track behind the speed head.

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
