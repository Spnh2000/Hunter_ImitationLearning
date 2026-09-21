# Phase 8 Design — policy_tracker, shadow PID, training harness

Proposed before code, per the brief. Everything below follows the README's
existing conventions: the `<controller>_tracker` slot in `hunter_control`, the
frozen `TrackingDiagnostics` contract, one canonical params file per track,
zero-timestamp commands, `results/` layout.

The comparison is **PID → 6 learned policies** (3 fitting methods × 2 collection
strategies). Phase 4 is dropped; Phase 8 is pure imitation, not offline RL.

---

## 1. `policy_tracker` — the inference node

### Decision: rclpy, installed into `hunter_control`

`benchmark.launch.py` hardcodes `package='hunter_control'` and resolves the
executable as `<controller>_tracker`. Rather than add a `tracker_package` launch
argument (a parallel structure), the policy node is a **Python program installed
by `hunter_control`'s CMakeLists via `install(PROGRAMS ...)`** — exactly how
`hunter_benchmark` already ships `metrics_logger.py`. Result:

    ros2 launch hunter_benchmark benchmark.launch.py controller:=policy ...

works with **no launch-file change** beyond two pass-through params.

**Why rclpy over C++/TorchScript.** The 20 Hz budget is 50 ms per tick.
Measured on this host (batch 1, eager PyTorch 2.12, CPU, pinned to 2 threads
so the simulator keeps its cores, 500 timed calls after warm-up):

| architecture | params | median | p99 | max |
|---|---:|---:|---:|---:|
| MLP 720→256→128→64→2 | 226k | 0.18 ms | 1.8 ms | 6.5 ms |
| stacked K=3, 2160→… | 594k | 0.26 ms | 1.8 ms | 7.2 ms |
| discrete 720→…→21+21 | 228k | 0.17 ms | 1.2 ms | 2.6 ms |

Worst observed tick is 7 ms against a 50 ms budget, with rclpy callback
overhead on the order of 1 ms on top. That is 7× headroom at the tail. C++/
TorchScript would buy latency we cannot spend, at the cost of an export step
between every training run and every closed-loop test. Iteration speed wins.
Revisit only if a future architecture (a CNN over the scan, say) pushes the
tail past ~20 ms — and the node logs its own tick time so that would be
visible, not inferred.

### Interface

```
policy_tracker
  params
    checkpoint_path   string   REQUIRED. Contains arch, bins, spec version.
    arch              string   mlp | stacked | discrete  (checked against ckpt)
    controller_name   string   from launch, as every tracker
    wheelbase_m, max_steering_rad, max_speed_mps   from the track params file
  subscribes
    /scan                    sensor_msgs/LaserScan      the ONLY observation
  publishes
    /ackermann_like_controller/reference   TwistStamped   ZERO stamp, like pid_tracker
    /tracking/diagnostics    hunter_msgs/TrackingDiagnostics
```

`/tracking/diagnostics` keeps its type and its role, so `summary.csv` and
`summarize_run.py` stay comparable across PID and policy rows.
`commanded_steering_rad` / `commanded_speed_mps` are the policy's post-clamp
outputs. The pose-derived fields — `lateral_error_m`, `heading_error_rad`,
`path_s_m`, `speed_error_mps` — are published as **NaN**: the student has no
pose, and that must hold in the node's *subscriptions*, not just its model
input. `policy_tracker` subscribes to `/scan` and nothing else.

**One small change to `metrics_logger` is required**, and I am flagging it
because the brief said "unchanged": `stats.py:136` counts `laps_completed`
from the tracker's own `path_s_m`, which a policy cannot report. The fix is
to derive progress from ground truth when `path_s_m` is NaN — the logger
already projects the ground-truth pose onto the path for
`gt_lateral_error_m` (`_signed_cross_track`), so the arc length falls out of
the same projection. The `summary.csv` columns do not change; for PID on
ground truth the two sources agree by construction, and for every other
tracker the lap count becomes ground-truth-referenced, which is what a
benchmark row should have been anyway. Roughly fifteen lines, unit-tested
against the existing PID bags.

Observation pipeline in the node is **`dataset_spec.scan_to_observation`** —
the same function the converter used, imported from the same module. If a
future downsample change reaches the converter, it reaches deployment
automatically; there is no second implementation to drift.

One checkpoint format for all three architectures; `arch` selects the model
class, and a mismatch between the param and the checkpoint **raises at
startup** rather than loading the wrong head.

---

## 2. Shadow-mode PID — the DAgger relabel

### The safety constraint, met structurally

The README's first troubleshooting entry is two trackers publishing commands
at once. DAgger runs `policy_tracker` and a PID **by design**, so the PID must
be *incapable* of publishing a command — not configured not to.

**Refactor:** the control law in `pid_tracker.cpp` moves into
`hunter_control_common` as a `PidCore` class — pure computation,
`(pose, trajectory, dt) → (steering, speed)`, **no ROS publishers inside it**.
Two thin executables compose it:

| executable    | constructs `cmd_pub_`? | publishes                                   |
|---------------|:----------------------:|---------------------------------------------|
| `pid_tracker` | **yes**                | `/ackermann_like_controller/reference` + `/tracking/diagnostics` |
| `pid_shadow`  | **no — no such member** | `/expert/relabel` only                     |

`pid_shadow` has no command publisher *field*. It cannot publish a command
because there is nothing to call `publish()` on. This is a constructor-level
separation: the wrong behaviour is unrepresentable, not merely disabled.

`/expert/relabel` carries `hunter_msgs/TrackingDiagnostics` — the same type
the converter already reads — so during a DAgger run:

    /tracking/diagnostics   ← policy_tracker   (what the STUDENT executed)
    /expert/relabel         ← pid_shadow       (what the EXPERT would have done)

The converter joins both to each scan; `actions` = relabel,
`action_source = 1`, `dagger_beta = 0`. The executed student action is kept in
`infos/executed_action` for coverage analysis.

`pid_shadow` drives on `/ground_truth/odometry` (the teacher's privilege) via
the same remap the launch already applies.

### Process ownership — a hard requirement for the run controller

Every collection or DAgger run is one `ros2 launch` per episode batch, and
the runner **holds that launch's PID and terminates it with SIGINT**, then
asserts `ros2 node list` is empty before starting the next. Name-based kill
loops are banned: a Python node's `comm` is `python3`, and three
`metrics_logger` zombies from three separate runs were found alive at once
during the Step 3 gate, each still subscribed to the live graph. They only
subscribed, so no data was corrupted — a leftover *tracker* would have been
a different story, and the README's first troubleshooting entry.

### Episode handling during rollout

`episode_monitor` fires → the run controller (a small Python `dagger_runner`)
records the terminal, calls `/episode/reset`, and **respawns on the reference
line at `s − 10 m`** via the Gazebo set-pose service. The approach trajectory
is kept in full — those rows are the recovery states the expert never visits
and are the most valuable in the dataset. Episodes cap at 1.5 laps by
`path_s_m`.

### Verifying DAgger works

Per-iteration histogram of `lateral_error_m` from the bag. Iteration 0 is a
spike at ~1 mm. If iterations 1+ do not visibly widen, the student is not
leaving the line and DAgger is producing expensive behaviour cloning. This is
the first plot in the Phase 8 validation report.

---

## 3. Training harness — `hunter_il` (new, ament_python)

Layered so nothing above the schema is duplicated:

    hunter_dataset   schema + collection + conversion       (no torch)
    hunter_il        models + training + diagnostics        (torch, no ROS)
    hunter_control   policy_tracker  →  imports hunter_il.models   (thin ROS wrapper)

```
src/hunter_il/hunter_il/
  data.py          HDF5 loader; train/val split by track then by lap;
                   episode-aware frame stacking (below)
  models.py        MlpPolicy, StackedMlpPolicy, DiscretePolicy — one trunk,
                   three input/head configurations
  bins.py          discretisation edges: computed from training data,
                   STORED IN THE CHECKPOINT, loaded for decoding
  train.py         one loop, config-driven; seeds; logs val loss per epoch
  checkpoint.py    save/load {state_dict, arch, bins, spec_version,
                   action_bounds, normalisation stats}
  diagnostics.py   information asymmetry (PCA→32, k-d tree, k=20,
                   median(local_std)/global_std, spatial map);
                   aggregate speed histogram; lateral-offset coverage BC vs DAgger
  eval_offline.py  validation loss table, so it can sit beside closed-loop
```

### Frame stacking — the boundary rule, stated

Stack index `t` uses rows `[t−K+1 … t]` **restricted to the same
`infos/episode_id`**. Missing history at an episode start is **padded by
repeating the episode's first scan** — the same zero-order-hold-backward
convention `prev_action` uses. Implemented as an explicit gather with a
boundary mask, never as slice arithmetic that could run into the previous
episode. Unit test: every stack at the first row of episode *k* contains only
rows whose `episode_id == k`.

Getting this wrong trains on scans from a different track with no error and a
validation loss that looks fine. The test exists because the failure is silent.

### Discretisation — bins are a stored design decision

- **Steering: 21 bins**, edges from **training-set quantiles** so they are
  fine near zero (where almost all driving happens) and coarse at the limits.
  21 gives ~0.02 rad resolution near centre, finer than the expert's own
  steering std (~0.05 rad), so bin-centre error is below teacher noise.
- **Speed: 21 bins**, quantile edges over `[0, ACTION_BOUNDS]`. The three
  `max_speed_mps` settings make the distribution trimodal; quantiles place
  bins where mass is.

Edges live in the checkpoint. A model whose predictions cannot be decoded
without a separate file is a model that will be decoded wrong.

### Splits

5 tracks train, **3 held out entirely** (Silverstone, SaoPaulo, Montreal —
one flowing, one technical, one narrow), plus 2 laps per training track held
out for a seen-track validation loss. Held-out laps measure memorisation;
held-out tracks measure generalisation. Report both.

---

## 4. Evaluation — closed loop, in Gazebo

Validation MSE averages over states; driving is decided by the rare hard ones.
So the result is the **closed-loop table** on the 3 held-out tracks,
ground-truth referenced, via the unchanged `metrics_logger → summary.csv`
path, with the PID expert as the reference row. **N ≥ 3 seeds per
configuration**, variance reported. Validation loss printed alongside — the
gap between the two rankings is a finding, not an inconvenience.

---

## 5. Painful to change after collection — flagged

| decision | why it is irreversible |
|---|---|
| `max_speed_mps ∈ {1.5, 2.1, 2.7}` per run | baked into every action label; the speed bins are derived from it |
| `SCAN_CONTACT_M = 0.4` | defines what `terminals` means; relabelling requires the bag AND re-running the monitor |
| Chassis-only contact sensor | already measured to miss wheel-first crashes; adding wheel sensors later changes `trigger` semantics across datasets |
| IMS cap at ~1–2 laps | the aggregate speed histogram is the check; trimming later is a filter, adding later is a drive |
| 2× track scale | every range value encodes it; `track_scale` attr exists for this reason |

The raw bags retain full 1080 beams, all twelve topics, and the manifest, so
everything *downstream* of the bag is a converter re-run. Only the five above
require re-driving.
