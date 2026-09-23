# Phases 6–8 explained

**Hunter V2 · ROS 2 Jazzy · Gazebo Harmonic · branch `dev-saqar`**
Status as of 2026-09-23. Written to be readable without the surrounding
conversation: what these phases are, why each decision was made, what has
actually been measured, and what is still running.

---

## 1. The one-paragraph version

A PID controller drives the Hunter around eight racetracks using **ground-truth
pose** — information a real robot does not have. Every 50 ms it records what it
saw on the LiDAR and what it did. A neural network is then trained to reproduce
those commands **from the LiDAR alone**. The question the project answers is:
how close can a scan-only policy get to a controller that cheats, and which
combination of training data and network architecture gets closest? That is
**privileged expert distillation** (Chen et al., *Learning by Cheating*,
CoRL 2019). Phase 6 builds the dataset; Phases 7–8 train and evaluate the
policies.

---

## 2. Why this is set up the way it is

### 2.1 The expert cheats on purpose

The PID drives on `/ground_truth/odometry` — Gazebo's exact world pose. It is
a *teacher*, not a deployable controller. Letting it cheat makes it an almost
perfect demonstrator (3–5 mm median tracking error), and a good teacher is
what imitation learning needs. The student never sees pose: `policy_tracker`
subscribes to `/scan` and nothing else, which is enforced by the node's
*subscription list*, not by masking a tensor.

### 2.2 Phase 4 was dropped, and that has a consequence

Phase 4 would have added Pure Pursuit, Stanley, LQR and MPC. It is gone: the
PID is now the **permanent expert**. This means the PID's tracking quality is
no longer just a benchmark row — it is a property of the dataset. Every label
the student learns is a PID output, so a flaw in the PID becomes a flaw in
every policy. This is why so much attention went into the expert being
correct (see §5.1, the clock bug).

### 2.3 Phases 7–8 are imitation, not reinforcement learning

Originally these were offline RL. They are now pure imitation learning. Two
practical consequences:

- **No reward function is needed.** The `rewards` field exists in the file
  format (it is D4RL-shaped so standard loaders work) and is filled with
  zeros.
- **No deliberately bad data.** For offline RL you want to see failures. For
  imitation, a demonstration of bad driving is a lesson in bad driving. So:
  no detuned gains, no noise injected on expert actions. Crash-*approach*
  states do belong, because the expert relabels them with the correct
  recovery — that is the DAgger mechanism in §6.

---

## 3. The eight tracks, and why they are 2× scale

F1TENTH circuits are drawn for 0.3 m-long 1/10-scale cars. The Hunter is
0.98 m long with a **1.31 m minimum turning radius**. At 1× scale, Monza's
chicanes have a 0.74 m radius — physically undrivable. Scaling everything by
2× fixes that and, as a bonus, makes the tracks *proportionally* like F1 for
this car (a Hunter lap of Nuerburgring 2× is 910 car-lengths; an F1 lap of the
real circuit is 1,029).

| Track | Length | R_min | Margin | Speed limit binds | Role |
|---|---:|---:|---:|---:|---|
| Shanghai | 995 m | 1.44 m | 1.84 m | 22.6 % | train |
| Nuerburgring | 892 m | 2.74 m | 1.71 m | 21.9 % | train |
| Melbourne | 949 m | 2.51 m | 1.73 m | 12.4 % | train |
| Monza | 892 m | 2.21 m | 1.37 m | 10.8 % | train |
| IMS | 586 m | 28.07 m | 1.50 m | **0 %** | train (capped) |
| Silverstone | 916 m | 2.61 m | 1.48 m | 21.0 % | **held out** |
| SaoPaulo | 689 m | 3.17 m | 1.73 m | 21.3 % | **held out** |
| Montreal | 570 m | 2.20 m | **0.66 m** | 18.6 % | **held out** |

"Speed limit binds" is the fraction of the lap where the curvature-limited
profile `v = sqrt(a_lat / κ)` falls below the speed cap — i.e. where the expert
must actually slow down. It is computed from the CSV alone, no simulator.

**Why the split is 5 train / 3 held out.** The three held-out tracks are never
trained on. They are the generalisation test: a policy that has memorised five
circuits will look excellent on them and fail on a sixth. The three chosen span
the range — Silverstone flowing, SaoPaulo technical, Montreal narrow (its
0.66 m margin is less than half the others, so it is the hardest).
Additionally, **two laps per training track** are held out to measure
memorisation of *seen* geometry. Held-out laps and held-out tracks answer
different questions and are reported separately.

**Why IMS is included but capped.** IMS is an oval; the speed profile never
engages, so commanded speed is a flat 2.7 m/s for the whole lap. A dataset
collected only there has *zero* speed variance and would train a policy that
emits one number while showing excellent loss. It is kept at 1 lap per speed
because it teaches one real lesson — "open geometry means full speed" — and
nothing else.

---

## 4. Phase 6 — building the dataset

### 4.1 Three schema decisions, fixed before any data existed

These are expensive to reverse once hundreds of thousands of rows exist, so
they live in one file with **zero ROS imports** (`hunter_dataset/dataset_spec.py`),
enforced by a test that scans the AST *and* imports it in a bare interpreter.

| Decision | Value | Why |
|---|---|---|
| Observation | 1080 → **720** beams, dropping every beam where `i % 3 == 1` | Selection, not interpolation: every value stays a real measurement. inf/NaN → 30.0 **then** clip to [0.15, 30.0] — in that order, because `np.clip` propagates NaN |
| Rate | **20 Hz** (the scan rate); the expert command is paired nearest-in-time within **15 ms** | The student runs at scan rate. The 50 Hz controller is oversampled relative to it. Rows outside 15 ms are rejected and counted — never interpolated |
| Units | Physical, **never normalised** (rad, m/s, m) | Normalisation belongs in the checkpoint, not the data, so the file stays interpretable |

`actions` are `[steering_rad, speed_mps]`. `infos/` carries the ground-truth
pose, progress, lap index, episode id, previous action, and the DAgger fields.

### 4.2 What counts as a crash

`episode_monitor` (C++) fires a terminal on **chassis contact** *or* **any scan
beam under 0.40 m**. Both exist because the contact sensor alone is not enough:
it only watches the chassis collision, and at full steering lock the *wheels*
reach the wall first — `contact_count` stayed 0 in every realistic crash
tested while the robot was visibly pinned. Scan proximity is the trigger that
actually fires. 0.40 m is 4.4× below the measured free-driving minimum of
1.77 m, so it cannot fire in normal driving.

The event is stamped with **the scan's own timestamp**, so the converter joins
it by exact match; a miss raises rather than silently dropping a row.

### 4.3 The pipeline

```
record_expert.launch.py   drive + record 12 topics to an mcap bag
        ↓                 (+ run_manifest.yaml written BEFORE the robot moves)
convert_dataset           bag → HDF5: join by timestamp, report Δt, reject beyond slop
        ↓
inspect_dataset           PASS/FAIL per check, nonzero exit on any FAIL
        ↓
collect_dataset           drives the whole plan, resumable, owns its processes
```

The inspector is deliberately loud. It prints per-action-dimension
min/max/std/unique and puts a `!!!!` banner on any near-constant column,
because a speed column that never varies trains fine and drives badly.

### 4.4 The gate, and what it caught

Before collecting 45 laps, **one** lap was collected and inspected. It
converted to **zero rows** — and that was the single most valuable result of
Phase 6. See §5.1.

### 4.5 What was actually collected (pass p1, complete)

**24 runs, 391,644 rows, 45 laps, 5.4 hours of driving, 2.3 GB of bags.**
8 tracks × 3 speed caps (1.5 / 2.1 / 2.7 m/s) × 2 laps (IMS: 1). Zero crashes;
every run passed inspection.

Three speed caps exist so the speed head has something to learn. At cap 1.5 the
expert is at the cap 94 % of the time; at 2.7 only 72 %, because corners force
it below. Without multiple caps the speed output would be nearly constant.

---

## 5. The four bugs that mattered

Each of these was silent — every node reported healthy while the answer was
wrong. That is the recurring signature in this project, and the reason for the
loud inspector, the assertions, and the gate.

### 5.1 The expert's control loop ran on the wrong clock

**Symptom:** the gate lap joined 0 of 7,549 rows.

**Cause:** `pid_tracker` used `create_wall_timer` and was never given
`use_sim_time`. Gazebo runs at ~0.3× real time here, so the loop ticked **170
times per simulated second instead of 50**, while its `dt` stayed hardcoded at
1/50. The effective integral gain was 3.4× the configured value and the
derivative gain 0.29×. Diagnostics were stamped in wall-clock time while scans
were stamped in sim time, which is why nothing joined.

**Why it mattered beyond the join:** the expert's behaviour depended on how
busy the computer was. On real hardware (RTF = 1) it would have driven with
different gains than it was tuned with.

**Fix:** `create_timer` on the node clock, `use_sim_time` on the tracker, the
trajectory server and the metrics logger. Tracking before/after on the same
lap: |lateral| median 5.8 → 4.8 mm — unchanged within noise, but now
reproducible.

### 5.2 `max_speed_mps` was a label, not a setting

The recorder put the requested cap in the manifest and the directory name but
never in a file the controller read. Every run would have driven at the params
file's 2.7 m/s and been *labelled* 1.5 or 2.1. No inspection of the dataset
alone could have detected this. Fixed by writing a per-run
`effective_params.yaml` that the launch actually loads.

### 5.3 The same bug again, one level deeper — and it cost 20 hours

After 5.2 was fixed, the first full DAgger run still produced garbage. The
params were written under a `pid_tracker:` key, but rclcpp matches parameters
**by node name**, and with `controller:=policy` the node is called
`policy_tracker`. It matched nothing and fell back to its compiled default of
2.7 m/s. Every DAgger rollout drove at 2.43–2.46 m/s whatever its cap, while
the shadow expert — which *did* get its section — labelled those states with
the correct cap. Two thirds of the data paired cap-2.7 states with cap-1.5 or
2.1 actions.

The damage was measurable: the resulting model predicted steering with a
**−0.252 rad bias** on held-out expert states, where the BC model predicts
+0.001 against a truth of +0.005.

**Fix:** write the section under the tracker's real node name, *and* make
`dagger_runner` abort the run if the student ever commands above its cap. The
second half matters more than the first: the class of failure is now
unrepresentable rather than merely fixed.

### 5.4 Zombie processes

Python ROS nodes have `comm` = `python3`, so name-based kill loops never touch
them. Three `metrics_logger` processes from three separate runs were once alive
simultaneously. All runners now own the launch **by PID and process group**,
terminate recorder-first (SIGINT, wait, SIGTERM) so the mcap finalises, and
assert `ros2 node list` is empty before and after every run.

---

## 6. Phases 7–8 — the experiment

### 6.1 Six policies: two axes

|  | **MLP** (720→256→128→64→2) | **Frame-stacked** (K=3) | **Discretised** (21+21 bins) |
|---|---|---|---|
| **BC data** | policy 1 | policy 2 | policy 3 |
| **DAgger data** | policy 4 | policy 5 | policy 6 |

Each is trained with 3 seeds, so 18 checkpoints, plus PID as the reference row.

**Axis 1 — architecture.** Framed as a hypothesis test against the measured
information asymmetry (§6.3), not a bake-off:

- *Frame stacking* asks whether the current scan is enough, or whether
  velocity/history information is needed. Stacks are gathered through an
  explicit index table clamped to each row's own episode start, with a unit
  test asserting no stack ever crosses an episode boundary — that failure is
  silent and would train on scans from a different track.
- *Discretisation* asks whether predicting a bin is easier than regressing a
  number. Bin edges come from training-set quantiles, so they are fine near
  zero (where almost all driving happens) and coarse at the limits, with the
  outer edges pinned to the physical bounds. **The edges live in the
  checkpoint** — a model whose outputs cannot be decoded without a separate
  file is a model that will be decoded wrong.

**Axis 2 — the data.** Behaviour cloning versus DAgger. This is §7.

### 6.2 The speed cap is an input (a deviation worth flagging)

Part D measured that the expert's speed is a function of the scan **within**
one cap (local/global action std = 0.000 on every track) and **not** across
caps (0.74–1.02). The reason is visible on the spatial map: on straights the
speed equals the cap, and nothing in the LiDAR says whether this run is capped
at 1.5, 2.1 or 2.7; in corners the speed equals the curvature limit, which is
cap-independent.

A scan-only student would learn the average and drive the wrong speed at every
cap. So the cap enters every policy as **one normalised scalar** beside the 720
ranges.

This is a deviation from "the student observes only `/scan`", and it is
deliberate: the cap is the *operator's setpoint* — the number a human hands the
driver — not privileged state about where the robot is. Pose stays hidden. It
is a config switch (`cap_input`), so the pure scan-only version remains
available as an ablation.

### 6.3 Information asymmetry, and why it is measured first

The expert acts on pose; the student sees only the scan. If two states look the
same to the scan but the expert acts differently in them, **no** scan-only
policy can fit both — the ceiling is set by the data, not the network. The
measurement: project observations to 32 dimensions (PCA), find each row's 20
nearest neighbours in that space, and take the std of the expert's action among
those neighbours divided by its global std.

Result on all 391k rows: **steering 0.003–0.040** (healthy is < 0.2). The scan
determines the expert's steering almost perfectly. Speed is discussed in §6.2.

This is the number that predicts the frame-stacking result: if the current scan
already determines the action, history has nothing left to add.

### 6.4 Evaluation is closed-loop, not validation loss

Validation loss averages over states; driving is decided by the rare hard ones,
and a policy that is 99 % accurate can still leave the track. So the result is
the **closed-loop table**: each policy drives the three held-out tracks in
Gazebo, measured against ground truth through the unchanged
`metrics_logger → summary.csv` path, with PID as the reference row and three
seeds per configuration.

Validation loss is reported **beside** it. Where the two rankings disagree is a
finding, not an inconvenience.

---

## 7. DAgger — and the question of five iterations

### 7.1 The problem DAgger solves

Behaviour cloning trains on the *expert's* states: a narrow ribbon along the
racing line, |lateral error| median 3 mm. The moment the student makes a small
error it is somewhere the expert never was — and there it has no training
signal, so it makes a bigger error. Errors compound. This is **covariate
shift**, and it is why a policy with excellent validation loss can drive into a
wall.

Measured here exactly as predicted: a BC student with 7.4 mrad steering error
offline sat **0.09–0.44 m** off the line when it actually drove.

### 7.2 The mechanism

Per iteration: the **student drives** (β = 0 — the expert never touches the
controls), while `pid_shadow` watches with ground-truth pose and publishes, at
every timestep, *what the expert would have done in this state*. Those
(student's state, expert's action) pairs are added to the dataset and the
student is retrained on everything so far.

The safety property is structural: `pid_shadow` and `pid_tracker` share the
same control law (`PidCore`) but `pid_shadow` **has no command-publisher
member**. A shadow that steers the robot is not a state the type can express.
This matters because the README's very first troubleshooting entry is two
controllers commanding the robot at once, and DAgger runs an expert beside the
student by design.

On a crash: the approach trajectory is **kept** — those recovery states are the
most valuable rows in the dataset — and the robot is teleported back to the
reference line 10 m before the crash point, the monitor is re-armed, the
student's frame stack is cleared, and the run continues.

### 7.3 Why iterate more than once

**Each iteration only fixes the mistakes the current student makes.** Fixing
them produces a *different* student, which makes different (usually smaller)
mistakes, in states the dataset still does not cover. One round of correction
cannot converge for the same reason one Newton step rarely finds a root.

More concretely, the iterations are not interchangeable:

- **Iteration 1's data comes from the BC student** — a poor driver that wanders
  0.1–0.4 m off the line. Its states teach *large* recoveries.
- **Iteration 5's data comes from a good driver** — small deviations, which is
  where the deployed policy will actually spend its time.

You need the later iterations precisely because the final policy operates in
the distribution the *later* students induce. Training only on iteration-1 data
would teach a policy optimised for recovering from errors it no longer makes.
DAgger's convergence guarantee (Ross, Gordon & Bagnell, AISTATS 2011) is
exactly the statement that this loop reaches a fixed point where the state
distribution the student induces matches the one it was trained on.

**The empirical evidence from the first (later invalidated) run** — the
per-iteration coverage of the student's lateral offset:

| iteration | \|lat\| median | \|lat\| p95 | crashes |
|---|---:|---:|---:|
| 1 | 0.103 m | 0.417 m | 6 |
| 2 | 0.163 m | 0.721 m | 54 |
| 3 | 0.135 m | 0.496 m | 0 |
| 4 | 0.091 m | 0.475 m | 20 |
| 5 | **0.074 m** | **0.228 m** | **0** |

Read it as: the distribution first *widened* (iteration 2 explored badly and
crashed 54 times — those crashes are training data, not failures), then
tightened as the student learned to recover, and by iteration 5 it was driving
within 7 cm of the line with no crashes at all. **It was still improving at
iteration 5**, which is the honest argument that 5 is not too many. The brief
asked for 5–8; 5 is the low end.

### 7.4 The counter-argument, and what I would do about it

Five iterations cost **~20 hours of simulation**, and the table above suggests
the curve is flattening. There is a defensible case for stopping at 3 or 4.

The principled criterion is not "how many hours" but **has the state
distribution stopped moving** — if iteration *n*'s coverage statistics match
iteration *n−1*'s, further iterations are adding rows from a distribution the
model already covers, and the loop has converged.

I have **not** added that early stop, because with one invalidated run there is
only one noisy curve to calibrate a threshold against, and stopping early on a
bad threshold is worse than 4 wasted hours. If you would rather cap the cost,
say so and I will add: stop when the median and p95 both move less than 10 %
between consecutive iterations. Under the numbers above that would have stopped
after iteration 4 and saved about 4 hours.

---

## 8. Where things stand

**Done:**

- Eight tracks converted, scaled and validated
- Phase 6 dataset tooling, gate passed, **collection pass p1 complete** (24/24 runs)
- Part D diagnostics on the full dataset
- 9 BC checkpoints (3 architectures × 3 seeds), trained and validated offline
- `policy_tracker`, `pid_shadow`, `dagger_runner`, `evaluate_policies`, the
  `hunter_il` training harness — all built and smoke-tested closed-loop

**Running now:** DAgger, 5 iterations, MLP student (~20 h).

**Then, automatically:** diagnostics with DAgger coverage → 9 DAgger
checkpoints → closed-loop evaluation of PID + 18 policies on the three held-out
tracks (~20 h) → the offline table beside it.

**Early offline results (BC only, held-out tracks):**

| architecture | held-out laps | held-out tracks |
|---|---:|---:|
| MLP | 7.4 mrad | 19.4 mrad |
| frame-stacked K=3 | 7.4 mrad | 19.7 mrad |
| discretised | 24.9 mrad | 25.8 mrad |

MLP and frame-stacking are indistinguishable — the stacking hypothesis is
answered, and it agrees with the asymmetry measurement in §6.3: there is no
hidden state for history to recover. Discretisation is ~3× worse offline;
whether that survives closed-loop is an open question, because bin-centre error
is a *bounded* error and regression error is not.

**Open items:**

- Per-wheel contact sensors (the chassis sensor misses wheel-first crashes).
  Adding them later changes what `trigger` means across datasets, so it should
  happen between collections, not during one.
- Collection passes p2/p3 exist in the plan but will only run if training on p1
  proves data-limited.

---

## 9. Reproducing any of it

```bash
# one expert run
ros2 launch hunter_dataset record_expert.launch.py \
    track_name:=Nuerburgring track_file:=src/Nuerburgring/Nuerburgring_centerline_2x.csv \
    params_file:=nuerburgring_defaults.yaml world:=Nuerburgring.sdf \
    max_speed_mps:=2.7 run_id:=001 output_dir:=datasets/raw

# bag → HDF5 → inspection
ros2 run hunter_dataset convert_dataset datasets/raw/<run>/bag out.hdf5 --manifest datasets/raw/<run>/run_manifest.yaml
ros2 run hunter_dataset inspect_dataset out.hdf5

# a whole collection pass, resumable
ros2 run hunter_dataset collect_dataset --plan src/hunter_dataset/config/collection_plan.yaml --pass p1

# diagnostics, training, DAgger, evaluation
ros2 run hunter_il il_diagnostics --data-dir datasets/raw --out diagnostics/p1
ros2 run hunter_il il_train_all --data-dir datasets/raw --seeds 0,1,2 --out checkpoints
ros2 run hunter_dataset dagger_runner --plan src/hunter_dataset/config/collection_plan.yaml \
    --train-config src/hunter_il/config/train_default.yaml --arch mlp --iterations 5
ros2 run hunter_dataset evaluate_policies --plan src/hunter_dataset/config/collection_plan.yaml \
    --out results/eval_v1 --laps 1.5 pid checkpoints/*_s?.pt

# or the whole thing
scripts/phase8_pipeline.sh
```

Related documents: [`phase_6_handoff.md`](phase_6_handoff.md) (the pre-Phase-6
state and the step-by-step plan), [`phase_8_design.md`](phase_8_design.md) (the
design proposal and, in §6, what changed between proposal and implementation).
