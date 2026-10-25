# Teaching a robot to race on laser data alone

**A ROS 2 / Gazebo study of privileged expert distillation on an Ackermann rover**

AgileX Hunter V2 · ROS 2 Jazzy · Gazebo Harmonic · July–September 2026
Repository: `Hunter_RL`, branch `dev-saqar`

---

## Summary

A robot cannot follow a racing line without knowing where it is, and a real
robot does not know. Its wheel encoders drift — in this project, by **2.29 m**
while the controller's own diagnostics reported an error of **1 mm**.

This study asks whether a neural network can learn to drive from a laser
scanner alone by imitating a controller that *is* allowed to know its position.
The answer is yes, with one large caveat about how you measure it.

**Result.** A policy reading only a 2D laser scan completes laps of three
racetracks it has never seen, holding **0.168 m** from the reference line,
where the privileged controller it learned from holds 0.019 m. An identical
network trained by ordinary imitation — copying the expert's demonstrations —
**crashes on 67 % of the same runs.**

**The finding.** Standard offline evaluation ranked those two policies in
exactly the wrong order. Measured by agreement with the expert on held-out
data, the crashing policy scores **5.6× better**. Selecting on that number
would have shipped the policy that drives into walls.

Everything here is reproducible from the repository: 9 ROS 2 packages,
~7,400 lines, 33 commits, 13.3 hours of recorded driving across 8 circuits,
and roughly 70 wall-clock hours of simulation.

---

## 1. The problem

### 1.1 Why not just use the position?

In simulation you can ask Gazebo for the robot's exact pose, and a PID
controller given that pose tracks a racing line to within 3–5 mm. On hardware
that pose does not exist. Wheel odometry integrates slip and drifts without
bound; the failure is silent, because the controller is tracking its own
belief perfectly while the robot is somewhere else. We measured exactly that:
**2.29 m of physical deviation reported as 1 mm of error.**

SLAM and AMCL can supply an absolute pose — both were built here in Phase 5,
and AMCL tracks to ~3 mm against the track's own occupancy grid. But that
requires a prior map, and a localizer that can fail.

### 1.2 The alternative

A 2D LiDAR measures distance in 1080 directions, 20 times a second. Inside a
walled corridor those distances implicitly encode where you are. The question
is whether a policy can extract enough of that to drive — and how much it
loses relative to a controller that simply knows.

### 1.3 Method: privileged expert distillation

Following Chen et al., *Learning by Cheating* (CoRL 2019):

1. Let a controller **cheat** — give it ground-truth pose so it demonstrates
   near-perfectly.
2. Record what the laser saw and what that controller did, 20 times a second.
3. Train a network to reproduce those commands **from the laser alone**.
4. Take the pose away and measure what the network can actually do.

The teacher's privilege is the point: a good demonstrator makes a good
dataset. The student's *lack* of privilege has to be enforced structurally —
in this implementation the policy node subscribes to `/scan` and to no pose
topic at all, so the constraint lives in the process, not in a comment.

---

## 2. The platform

### 2.1 Robot

AgileX Hunter V2: Ackermann-steered, 0.98 m long, 0.745 m wide, 0.65 m
wheelbase, **1.31 m minimum turning radius**, ~2.8 m/s on the upgraded
drivetrain. Simulated in Gazebo Harmonic via `ros2_control`, with a 270° /
1080-beam / 30 m LiDAR, a 100 Hz IMU, and a chassis contact sensor.

### 2.2 Tracks, and why everything is 2× scale

The circuits come from the F1TENTH dataset — real Formula 1 layouts drawn for
0.3 m-long 1/10-scale cars. At that size Monza's chicanes have a **0.74 m**
radius, barely half what the Hunter can physically turn. Every track is
therefore rebuilt at **2× scale**, which fixes the kinematics and, as a
by-product, restores F1-like proportions for this vehicle: a Hunter lap of
Nürburgring 2× is 910 car-lengths, an F1 lap of the real circuit is 1,029.

| Circuit | Length | R_min | Wall margin | Speed limit binds | Role |
|---|---:|---:|---:|---:|---|
| Shanghai | 995 m | **1.44 m** | 1.84 m | 22.6 % | train |
| Nürburgring | 892 m | 2.74 m | 1.71 m | 21.9 % | train |
| Melbourne | 949 m | 2.51 m | 1.73 m | 12.4 % | train |
| Monza | 892 m | 2.21 m | 1.37 m | 10.8 % | train |
| IMS | 586 m | 28.07 m | 1.50 m | **0 %** | train (capped) |
| Silverstone | 916 m | 2.61 m | 1.48 m | 21.0 % | **held out** |
| São Paulo | 689 m | 3.17 m | 1.73 m | 21.3 % | **held out** |
| Montreal | 570 m | 2.20 m | **0.66 m** | 18.6 % | **held out** |

*Speed limit binds* is the fraction of the lap where the curvature-limited
profile `v = √(a_lat / κ)` falls below the speed cap — where the expert must
genuinely brake. It is computed from the reference CSV alone.

**Three circuits are never trained on.** They are the only honest test: a
policy that has memorised five layouts will look excellent on those five and
fail on a sixth. Montreal is the hardest, with less than half the corridor
margin of any other track.

**IMS is included but capped at one lap per speed.** Its speed profile never
engages, so commanded speed is a flat 2.7 m/s for the whole lap. A dataset
built only from it would have *zero* speed variance and would train a policy
that emits a single number while reporting excellent loss. It earns its place
by teaching one thing — open geometry means full speed — and is deliberately
prevented from teaching anything else.

### 2.3 The expert

A PID cross-track controller with two feedforward terms:

```
steering = clamp( PID(−lateral_error)
                  − k_heading · heading_error        ← Stanley-style
                  + atan(wheelbase · curvature),     ← geometric anticipation
                  ±0.461 rad )
```

with a curvature-limited speed profile. Two separate lookaheads matter and
pull in opposite directions: steering feedforward needs a **short** lead
(0.3 m — describe the curve the front wheels are entering *now*), corner
braking needs a **long** one (2.5 m — the vehicle needs metres to shed speed).
Sharing a single lookahead produced a slow drift to >1.5 m of deviation on
long sweepers.

Driving on ground truth, it holds **3–5 mm** median error. That is the ceiling
the students are measured against.

---

## 3. The dataset

### 3.1 Three decisions frozen before any data existed

These are expensive to reverse once hundreds of thousands of rows exist, so
they live in one file (`dataset_spec.py`) with **zero ROS imports**, enforced
by a test that scans the AST *and* re-imports the module in a bare interpreter.

| Decision | Value | Reasoning |
|---|---|---|
| Observation | 1080 → **720** beams, dropping every beam where `i % 3 == 1` | Selection, not interpolation — every number stays a real measurement. Non-finite → 30.0 **then** clip to [0.15, 30.0]; in that order, because `np.clip` propagates NaN |
| Rate | **20 Hz**, expert command paired nearest-in-time within **15 ms** | The student runs at scan rate; the 50 Hz controller is oversampled relative to it. Rows outside the window are rejected and counted, never interpolated |
| Units | Physical, **never normalised** | Normalisation belongs in the model checkpoint, not in the data, so the file stays interpretable a year later |

Storage is D4RL-shaped HDF5 (`observations / actions / rewards / terminals /
timeouts / infos/*`) so any standard imitation loader reads it.

### 3.2 Episode termination

`episode_monitor` fires on chassis contact **or** any laser beam under 0.40 m.
Both exist because the contact sensor alone is insufficient: it watches only
the chassis collision, and at full steering lock the *wheels* reach the wall
first. `contact_count` stayed 0 in every realistic crash tested while the robot
was visibly pinned. Scan proximity is the trigger that actually fires; 0.40 m
sits 4.4× below the measured free-driving minimum of 1.77 m, so it cannot fire
during normal driving.

### 3.3 What was collected

| | |
|---|---|
| Expert demonstrations | 24 runs, **391,644 rows**, 5.4 h of driving |
| DAgger corrections | 75 rollouts (17 ended early), **566,831 rows**, 7.9 h |
| **Total** | **958,475 rows ≈ 13.3 h** |
| Conditions | 8 circuits × 3 speed caps (1.5 / 2.1 / 2.7 m/s) |

Three speed caps exist so the speed output has something to learn: at 1.5 m/s
the expert sits at the cap 94 % of the time, at 2.7 m/s only 72 %, because
corners force it lower.

---

## 4. The experiment

Two axes, six policies, three training seeds each, plus the expert as the
reference row.

|  | plain MLP | frame-stacked K=3 | discretised |
|---|---|---|---|
| **BC data** (expert only) | ✓ | ✓ | ✓ |
| **DAgger data** | ✓ | ✓ | ✓ |

### 4.1 Axis 1 — architecture, framed as a hypothesis test

- **MLP**: 720 → 256 → 128 → 64 → 2, ~226k parameters.
- **Frame stacking (K=3)**: the current scan plus the two before it. Tests
  whether the policy needs to perceive *motion*, not just position.
- **Discretisation**: 21 preset steering and 21 speed options instead of
  continuous outputs. Tests whether classification is an easier problem than
  regression.

These were not a bake-off. **Before any training**, the dataset was measured
for *information asymmetry*: project the observations to 32 dimensions by PCA,
take each row's 20 nearest neighbours, and compute the standard deviation of
the expert's action among those neighbours divided by its global standard
deviation. If scans that look alike get actions that differ wildly, no
scan-only policy can fit them and the ceiling is set by the data.

**Steering scored 0.003–0.040** (healthy is < 0.2) on all eight circuits. The
current scan already determines the expert's steering — which *predicts* that
frame stacking will add nothing.

### 4.2 Axis 2 — behaviour cloning versus DAgger

**Behaviour cloning** trains on the expert's demonstrations. The problem is
that the expert is *good*: every example shows the robot within 3 mm of the
line. When the student is 2 cm off, it is somewhere the dataset barely covers,
so its next command is a worse guess, and errors compound. This is **covariate
shift**, and it is why a policy with excellent validation loss can leave the
track.

**DAgger** (Ross, Gordon & Bagnell, 2011) fixes it: let the student drive
while the expert watches with its privileged pose and records, at every
timestep, what it *would* have done. Add those pairs to the dataset, retrain,
repeat. The student generates the states; the expert labels them.

Five iterations were run. The safety property is structural: the watching
expert (`pid_shadow`) shares the control law with the driving expert but **has
no command-publisher member at all** — a shadow that steers the robot is not a
state the type can express. Two controllers commanding one robot is the single
most common failure in this codebase, and DAgger deliberately runs two at once.

### 4.3 One deviation, stated plainly

The diagnostics found that the expert's *speed* is determined by the scan
within one speed cap (asymmetry 0.000) but not across caps (0.74–1.02). The
same straight is driven at 1.5, 2.1 and 2.7 m/s and nothing in the laser says
which. A scan-only policy would learn the average and be wrong at every
setting.

So the speed cap is supplied as **one extra scalar** alongside the 720 ranges.
This is a departure from "the student sees only `/scan`", and the justification
is that the cap is the *operator's setpoint* — a number a human dials in — not
privileged information about where the robot is. Pose remains hidden. It is a
configuration switch, so the strict scan-only variant remains available as an
ablation.

### 4.4 Evaluation

**Closed-loop, in the simulator, on the three held-out circuits.** Each policy
drives Silverstone, São Paulo and Montreal, three times each (one per seed),
measured against ground truth through the same metrics pipeline the expert
uses. Validation loss is reported *beside* the result, never used to select.

---

## 5. Results

Nine runs per row (3 seeds × 3 unseen circuits).

| policy | crash rate | laps completed | true lateral RMS | *offline steer error* |
|---|---:|---:|---:|---:|
| **PID expert** (privileged) | 0 % | 1.00 | **0.019 m** | — |
| BC + MLP | **67 %** | 0.33 | 0.332 m | *19.4 mrad* |
| BC + stacked K=3 | **56 %** | 0.33 | 0.420 m | *19.7 mrad* |
| BC + discretised | **67 %** | 0.33 | 0.407 m | *21.5 mrad* |
| **DAgger + MLP** | **0 %** | **1.00** | **0.168 m** | *108.9 mrad* |
| DAgger + stacked K=3 | 0 % | 1.00 | 0.195 m | *160.4 mrad* |
| DAgger + discretised | 0 % | 1.00 | 0.249 m | *281.9 mrad* |
| **DAgger, rebalanced 50/50** | **0 %** | **1.00** | 0.175 m | *68.1 mrad* |

### 5.1 The headline: the offline metric ranked the policies backwards

Read the last column against the first.

**Offline, behaviour cloning beats DAgger by 5.6×** — 19.4 against 108.9
milliradians of disagreement with the expert on held-out data.

**In the simulator, behaviour cloning crashes on 67 % of runs and DAgger on
none.**

The measurement everyone reaches for first ranked the policies in exactly the
wrong order. This is not a narrow margin: 67 % versus 0 %, consistent across
three seeds and three circuits, with per-row standard deviation on the DAgger
results of 0.003–0.028 m.

The reason is the whole of §4.2. Validation loss averages over states; driving
is decided by the rare hard ones, and a policy is judged on the states it
*puts itself into*. Copying a flawless demonstrator teaches nothing about
those.

### 5.2 Axis 1 — architecture made no difference

MLP 0.168 m versus stacked 0.195 m, both with zero crashes, and 19.4 versus
19.7 mrad offline: indistinguishable within seed noise.

This was **predicted before training** by the information-asymmetry
measurement. The current scan already determines the expert's steering, so a
history of scans has no hidden state left to recover. A prediction that
survives contact with the experiment is worth considerably more than a horse
race that happens to come out level.

### 5.3 Axis 2 — DAgger was decisive, and its mechanism was visible

67 % → 0 % crash rate; 0.332 → 0.168 m tracking error.

The mechanism could be watched as it happened. Iteration 1 produced **7
crashes, every one of them at Shanghai's 1.44 m hairpin** — at s = 872–874 m,
repeatable to within one metre across all three speed caps. The student drove
870 m cleanly and lost it at the same corner every time. After the shadow
expert relabelled those states, **iterations 3 and 4 ran all 30
rollouts across five circuits without a crash.** Iteration 5 had two failures,
both on Monza at the same point (s ≈ 147 m).

### 5.4 The follow-up: how much of the offline collapse was the mix?

The DAgger policies' poor offline scores raised a question. DAgger rows had
grown to **75 %** of the training set and are dominated by off-line states
carrying large corrective labels — so was the network simply swamped?

Retraining the same architecture on the same rows with a forced **50/50**
BC:DAgger composition per batch **halved the offline error to 68.1 mrad while
keeping the perfect crash record** — 0 %, every lap, 0.175 m against the
unbalanced 0.168 m, which is inside seed noise.

So the collapse was substantially an artefact of composition. **Not entirely**:
68.1 mrad is still 3.5× BC's 19.4. A genuine component of the trade remains — a
policy trained to recover is measurably less precise at imitating a
demonstrator that never needs to.

### 5.5 How good is the result, in absolute terms?

The best policy drives every unseen circuit without crashing, holding 0.168 m
from the racing line, against the cheating expert's 0.019 m. Roughly **nine
times less accurate than a controller that knows exactly where it is, using
nothing but a laser scanner, on tracks it has never seen.**

Whether that suffices depends on the corridor. On Montreal — 0.66 m of margin,
less than half of any other circuit — a 0.17 m RMS error is comfortable but
not generous.

---

## 6. Engineering: eight silent defects

Every defect found in this project shared one signature: **every program
reported that it was healthy while producing wrong answers.** Nothing crashed;
no error message appeared. That is the most transferable lesson here, so each
is recorded with how it was caught.

| # | Defect | Caught by | Cost |
|---|---|---|---|
| 1 | Expert's control loop ran on wall-clock, not sim time — 170 ticks per simulated second instead of 50, scaling its effective I and D gains by 1/RTF | A deliberate one-lap gate before collecting 45: it produced **zero** usable rows | Would have invalidated the entire dataset *and* made the expert host-load dependent |
| 2 | `max_speed_mps` was a label, not a setting — it reached the filename and manifest but never the controller | Code inspection while writing the collection runner | Every run would have driven at 2.7 m/s while labelled 1.5 or 2.1; **undetectable from the dataset** |
| 3 | The same bug one level deeper: params filed under `pid_tracker`, but ROS matches by node name and the student node is `policy_tracker` | The resulting policies had a 0.25 rad steering bias — the signature of contradictory labels, not undertraining | **20 hours of simulation discarded** |
| 4 | Zombie processes — Python ROS nodes all appear as `python3`, so name-based cleanup never finds them | Three `metrics_logger` instances from three separate runs found alive simultaneously | Contaminated runs; one orphan pinned the machine at load 9.8 for hours |
| 5 | Discrete bin edges collapsed — pure quantile spacing on a peaked distribution gave a **9,931×** width ratio, eight bins finer than the actuator, and two end bins covering 84 % / 89 % of the range | Dumping a checkpoint to document its file format | One third of the architecture comparison was measuring the defect |
| 6 | A stale run directory silently disabled the bag recorder (`ros2 bag record` refuses an existing output dir) | A rollout drove a flawless 890 m lap and produced zero rows | One rollout, and would have recurred after every crash |
| 7 | The pause script never waited (compared completed runs against directory count) and left the simulator running (each launch is in its own session) | Using it | Reported "PAUSED" over a live simulator |
| 8 | A committed shell script that would not parse | `bash -n` after the fact | Trivial, but shipped |

Three patterns worth extracting:

**Gates pay for themselves.** Defect 1 was caught because one lap was recorded
and inspected before forty-five were. That single decision saved the project.

**Tests can pass on broken artefacts.** Defect 5's test suite asserted
round-trip error within one bin width and monotonic edges pinned to the
physical bounds. **Both are true of a completely collapsed partition.** The
fix was to assert the shape of the partition itself — no bin below actuator
resolution, width ratio under 40, no single bin covering half the range — and
to verify the new tests fail against the old behaviour.

**Prefer making failures unrepresentable over fixing instances.** Defect 3 was
fixed *and* the runner now aborts if the student ever commands above its cap.
The shadow expert has no command publisher, rather than a disabled one. Defect
2's value is now checked by the dataset inspector. The bug class becomes hard
to have silently, not merely absent today.

---

## 7. Limitations

- **Simulation only.** Sim-to-real transfer is Phase 9 and untouched. The
  LiDAR is noise-modelled but idealised; the contact model is approximate.
- **One expert.** The PID is the permanent teacher, so every policy inherits
  its behaviour and its flaws. A different expert would produce different
  students, and the comparison says nothing about which expert is best.
- **Five circuits of training data.** Generalisation is demonstrated on three
  held-out tracks, which is honest but narrow.
- **The chassis contact sensor misses wheel-first crashes.** Scan proximity
  carries the load. Per-wheel sensors are the fix; adding them changes what
  `trigger` means across datasets, so it belongs between collections.
- **DAgger iteration 2 failed outright** — all 15 rollouts stuck or crashing,
  though its rows were still trained on; iterations 3–4 recovered unaided. It happened in
  **both** runs of the experiment, so it is reproducible and unexplained.
- **58 of 75 rollouts completed.** The 17 failures (iteration 2, plus two Monza
  rollouts in iteration 5) were still kept in the training set.
- **The speed cap is an input**, as argued in §4.3. The strict scan-only
  ablation was implemented but not run.

---

## 8. What would come next

1. **Sim-to-real (Phase 9).** The policies need no map and no localization,
   which is exactly what makes them plausible on hardware — the headline
   practical argument for this whole approach.
2. **Sweep the BC:DAgger ratio.** 50/50 halved the offline gap at no cost to
   driving. The optimum is somewhere and has not been located.
3. **Explain iteration 2.** Reproducible failure, unexplained.
4. **The scan-only ablation**, to quantify precisely what the cap input buys.
5. **Per-wheel contact sensors**, for a cleaner crash definition.

---

## Appendix A — Reproducing this

The trained policies ship with the repository. Seeing the result takes three
commands and no dataset download:

```bash
git clone https://github.com/Spnh2000/Hunter_RL.git && cd Hunter_RL
docker compose build && docker compose run --rm dev
colcon build --symlink-install && source install/setup.bash

# a policy driving a circuit it has never seen, on laser data alone
scripts/watch_policy.sh checkpoints/final/dagger_mlp_s0.pt SaoPaulo 2.1

# the same network trained without DAgger — this one usually crashes
scripts/watch_policy.sh checkpoints/final/bc_mlp_s0.pt Montreal 2.7
```

Rebuilding the whole pipeline:

```bash
ros2 run hunter_dataset collect_dataset --plan <plan> --pass p1        # ~20 h
ros2 run hunter_il il_diagnostics --data-dir datasets/raw --out diagnostics/
ros2 run hunter_il il_train_all --data-dir datasets/raw --seeds 0,1,2
ros2 run hunter_dataset dagger_runner --plan <plan> --arch mlp --iterations 5
ros2 run hunter_dataset evaluate_policies --plan <plan> --out results/eval --laps 1.5 pid checkpoints/final/*.pt
```

## Appendix B — Where things live

| Package | Contents |
|---|---|
| `hunter_description` | URDF/xacro, sensors, `ros2_control` |
| `hunter_gazebo` | 8 world SDFs at 2× scale |
| `hunter_msgs` | `Trajectory`, `TrackingDiagnostics`, `EpisodeEvent` |
| `hunter_trajectory` | CSV → reference trajectory with curvature-limited speed |
| `hunter_control` | `PidCore` → `pid_tracker` + `pid_shadow`; `episode_monitor`; `policy_tracker` |
| `hunter_benchmark` | Metrics, per-track params, benchmark launch |
| `hunter_localization` | Dual EKF, AMCL, SLAM configs (Phase 5) |
| `hunter_dataset` | Schema, recorder, converter, inspector, collection runner, DAgger runner, evaluator |
| `hunter_il` | Models, episode-aware data loading, discretisation, training, diagnostics (PyTorch, no ROS) |

| Document | Purpose |
|---|---|
| [`../README.md`](../README.md) | Overview, architecture, results, troubleshooting |
| [`../QUICKSTART.md`](../QUICKSTART.md) | Clone to driving in three commands |
| [`phases_6_8_simple.md`](phases_6_8_simple.md) | The learning phase in plain language — to learn from and teach from |
| [`phases_6_8_explained.md`](phases_6_8_explained.md) | Technical reference with exact figures |
| `results/eval_v1/eval_all.csv` | Every evaluation run, 84 rows |

## Appendix C — Glossary

| Term | Meaning |
|---|---|
| **Privileged expert distillation** | Train a teacher with information the student will not have, then distil its behaviour |
| **Behaviour cloning (BC)** | Supervised learning on expert demonstrations alone |
| **Covariate shift** | The student drifts into states its training data never covered; errors compound |
| **DAgger** | Student acts, expert labels the resulting states, data is aggregated, repeat |
| **Closed-loop evaluation** | Running the policy on the robot and measuring the outcome, as opposed to scoring its predictions |
| **Held-out track** | A circuit never used in training, reserved to test generalisation |
| **Information asymmetry** | A measure of whether the student's observation contains enough to reproduce the teacher's action |
| **Ground truth** | The simulator's exact pose — used to grade and to teach, never given to the student |
