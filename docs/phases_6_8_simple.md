# Phases 6–8 in plain language

A guide to learn from, and to teach from. No prior machine-learning knowledge
assumed. Every technical term is introduced in **bold** the first time with a
plain-English meaning, because those are the words you will need when you
explain this to someone else.

The dense reference version, with exact numbers and commands, is
[`phases_6_8_explained.md`](phases_6_8_explained.md).
For the whole project as a standalone report — problem, method, results,
limitations — see [`PROJECT_REPORT.md`](PROJECT_REPORT.md).

---

## Part 1 — The idea, in one page

**We are teaching a robot to drive around a racetrack using only its laser
scanner.**

The robot is an AgileX Hunter V2 — a car-like rover that steers with its front
wheels. The tracks are Formula 1 circuits, shrunk down and rebuilt as
simulated worlds.

Here is the problem. To follow a racing line, you need to know where you are.
In simulation we can ask the simulator for the robot's exact position — but a
real robot cannot do that. Its wheel encoders drift: we measured the robot
physically **2.29 m** away from where it believed it was, while its own
diagnostics cheerfully reported an error of **1 mm**. So "just use the position"
is not a solution on real hardware.

What the robot *does* have, cheaply and reliably, is a **LiDAR** — a laser
scanner that measures the distance to the nearest obstacle in 1080 directions
around it, 20 times a second. On a walled racetrack, those distances implicitly
describe where you are in the corridor.

So the plan is:

1. Let a controller that **can** see the exact position drive the track
   perfectly. Call it the **teacher**.
2. While it drives, record two things 20 times a second: what the laser saw,
   and what the teacher did (how much it steered, how fast it went).
3. Train a neural network — the **student** — to look at a laser scan and
   produce the same steering and speed the teacher would have.
4. Take the position away, let the student drive on the laser alone, and see
   how close it gets to the teacher.

That is the whole project. The technical name is **privileged expert
distillation**: the teacher has *privileged* information (the exact position),
and we *distil* its behaviour into a student that does not.

> **If you explain nothing else, explain this:** we let a controller cheat so
> it can be a perfect demonstrator, then we train a network to copy it without
> the cheat.

---

## Part 2 — The three pieces

### The teacher: a PID controller

A **PID controller** is a classic bit of control engineering, not machine
learning. It is about fifteen lines of arithmetic. Every 20 milliseconds it
asks: *how far am I from the line I am supposed to be on?* — and steers in
proportion to that error.

- **P** (proportional): the further off you are, the harder you steer back.
- **I** (integral): if you have been slightly off for a long time, push a bit
  harder.
- **D** (derivative): if you are closing on the line fast, ease off so you do
  not overshoot.

Ours also looks a little way ahead on the track and slows down for corners
before it reaches them.

Because it can see its exact position, it drives extremely well — typically
within about **2 centimetres** of the intended line. That is what makes it a good
teacher. A sloppy teacher would produce a sloppy student.

### The student: a small neural network

Do not overthink this. The student is a function:

```
   720 numbers in                           2 numbers out
   (distances, in metres)   →   network  →  (steering angle, speed)
   + the speed limit
```

The 720 numbers are the laser scan. (The scanner produces 1080 beams; we keep
every beam except each third one, which is enough detail and a third less data.
We *select* beams rather than averaging them, so every number stays a real
measurement.)

The network itself is four layers of multiplication and addition —
721 → 256 → 128 → 64 → 2 (the 720 laser distances plus the speed limit) — with about 226,000 adjustable numbers inside, called
**weights**. Training means nudging those weights until the network's output
matches the teacher's on the recorded examples.

It is small by modern standards. It has to be: it must produce an answer in
under 50 milliseconds on the robot's own CPU. Measured: **1.5 ms typical,
7 ms in the slowest 1 % of cases**. Plenty of room.

### The lessons: a dataset of (scan, action) pairs

Every row of the dataset is one moment in time:

| what the laser saw | what the teacher did |
|---|---|
| 720 distances | steer −0.03 rad, go 2.4 m/s |

We have **391,644 such rows** — about five and a half hours of driving across
eight tracks. That is the dataset the students learn from. (DAgger, in Part 3,
later adds 566,831 more.)

---

## Part 3 — Why copying is harder than it sounds

This is the most important idea in the whole project, and it is the one people
usually miss. Take your time with it.

### The problem

The teacher is *good*. It stays within about 2 cm of the line. So every single
example in the dataset shows the robot almost exactly on the line.

Now the student drives. It is a copy, not a clone, so it is a tiny bit off —
say 2 cm. But **2 cm off the line is a situation the dataset barely contains**.
The student has essentially never been shown what to do from there. So its next
command is a slightly worse guess, and now it is 5 cm off. Which it has *never*
seen. Then 15 cm. Then it is in the wall.

The errors compound. The name for this is **covariate shift** — the states the
student actually visits drift away from the states it was trained on.

> **The analogy that works:** imagine learning to ride a bicycle purely by
> watching videos of an expert riding perfectly. Every video shows a bike
> upright and balanced. Then you get on, and you wobble — and nothing you
> watched has ever shown you a wobble, let alone how to recover from one.

We measured this exactly. Students whose *test score* was excellent — steering
within 7 milliradians of the teacher on held-out examples — sat **33
centimetres** off the line on tracks they had never seen, and crashed in two
runs out of three. Good marks, bad driving.

### The fix: DAgger

**DAgger** (short for "Dataset Aggregation") is a neat trick, and once you see
it you cannot unsee it.

> **Let the student drive. Have the teacher watch and, at every single moment,
> say out loud what it *would* have done. Write those down. Add them to the
> lessons. Retrain. Repeat.**

That is it. The student drives, so we get examples of the situations the
*student* actually gets itself into — including wobbles, bad corner entries,
and near-misses with walls. And because the teacher is watching with its
privileged position information, we get the correct answer for each of those
situations, without the teacher ever touching the controls.

Back to the bicycle: instead of only watching videos, you ride, you wobble, and
an instructor beside you says "right now, lean left". You get labelled examples
of *your own* mistakes.

**A safety detail worth mentioning when you explain this**, because it shows
how the idea is implemented and not just described: the watching teacher and
the driving teacher are two separate programs. The watching one (`pid_shadow`)
was *built without the ability to send commands to the robot* — not disabled by
a setting, but literally missing the piece of code that would publish a
command. Two controllers fighting over one robot is the single most common
failure in this codebase, and DAgger deliberately runs two controllers at once.
So we made the dangerous configuration impossible to express rather than
merely switched off.

### Why we repeat it five times

This is the question people ask, so it is worth having a crisp answer.

**One round of correction only fixes the mistakes the student is making
right now.** Fix those, and you have a *different* student — a better one, which
makes different and smaller mistakes, in situations the dataset still does not
cover. So you go round again.

The rounds are not interchangeable, and this is the key point:

- **Round 1's lessons come from a bad driver** that wanders 5–40 cm off the
  line. They teach big recoveries.
- **Round 5's lessons come from a good driver** making small deviations — which
  is exactly where the finished policy will spend its life.

If you trained only on round-1 data, you would get a policy expert at
recovering from mistakes it no longer makes. You need the later rounds because
the final policy lives in the situations the *later* students create.

Here is what we actually measured (how far off the line the student sat, and
how often it hit a wall):

| round | typical distance off the line | worst 5 % | crashes |
|---|---:|---:|---:|
| 1 | 8.6 cm | 27.6 cm | 7 |
| 2 | 16.1 cm | 129 cm | 71 |
| 3 | 7.5 cm | 21.9 cm | 0 |
| 4 | **6.8 cm** | **18.7 cm** | **0** |
| 5 | 11.2 cm | 26.7 cm | 10 |

Read it as a story: in round 2 the student got adventurous, explored badly and
crashed 71 times — **and those crash approaches are valuable lessons**, because
each one is a "here is how you get into trouble, and here is what the teacher
would have done about it". Rounds 3 and 4 then drove without a single crash,
within 7 cm of the line.

Be honest about round 5 when you present this: it was *worse* than round 4 (two
rollouts on Monza failed). The table shows why more than one round is needed,
but it does not show that a fifth round was. (The theory — Ross, Gordon &
Bagnell 2011 — says the loop converges when the situations the student creates
stop changing.)

---

## Part 4 — The experiment

We are not just building one student. We are answering two questions at once,
which gives **six students**:

|  | plain network | network that sees 3 scans | network that picks from a menu |
|---|---|---|---|
| **lessons from the teacher only** | student 1 | student 2 | student 3 |
| **lessons from DAgger** | student 4 | student 5 | student 6 |

Each is trained three times from different random starting points (**seeds**),
so we can tell a real difference from luck. That is 18 trained networks, plus
the teacher itself as the benchmark to beat.

### Question 1 (across the columns): does the network design matter?

- **Plain network** — looks at the current laser scan, outputs two numbers.
- **Three scans stacked** — looks at the current scan *and the two before it*.
  The idea: maybe the robot needs to sense how it is *moving*, not just where
  it is, and you cannot see motion in a single snapshot.
- **Menu instead of numbers** (**discretisation**) — instead of outputting
  "steer 0.037 radians", it picks one of 21 pre-set steering options. The idea:
  choosing from a menu can be an easier problem than producing an exact number.
  The menu is finer near straight-ahead, where almost all driving happens.
  (The first version of the menu was broken — see defect 5 in Part 9 — and was
  fixed before the final results.)

We did not just guess at these. Before training anything, we **measured**
whether the current scan is enough (Part 8 below), and the measurement
predicted that stacking would add nothing. That is the difference between a
bake-off and a hypothesis test — and it is a good thing to say when presenting
this work.

### Question 2 (down the rows): does DAgger beat plain copying?

Row 1 is **behaviour cloning** (**BC**) — the plain "copy the teacher" approach
of Part 2. Row 2 adds the DAgger rounds of Part 3. This is the comparison that
shows whether fixing covariate shift is worth the effort.

### One honest wrinkle: the speed limit is an input

We ran each track at three different speed limits — 1.5, 2.1 and 2.7 m/s — so
the student has something to learn about speed rather than always outputting
one number.

Then we measured something that forced a design change. **Nothing in the laser
scan tells you which speed limit is in force.** The same straight looks
identical whether the teacher is doing 1.5 or 2.7 m/s. A scan-only student
would learn the average and be wrong at every setting.

So the speed limit is fed to the network as one extra number alongside the 720
distances.

Be upfront about this when you explain it, because it looks like cheating. The
argument is: the speed limit is the **operator's setting** — the number a human
dials in before the robot sets off — not secret information about where the
robot is. Position stays hidden, which is the thing that actually matters. And
it is a switch we can turn off to test the pure version.

---

## Part 5 — How a policy is actually made, and how to run one

Parts 1–4 described the idea. This part is the mechanics: what training does,
what file comes out, and how that file makes the robot drive. If you are
explaining this to an engineer, this is the part they will ask about.

### Step 1 — what goes in

The 24 recorded runs, sitting in `datasets/raw/` as HDF5 files. 391,644 rows.
Each row is one moment: 720 laser distances, and the two numbers the expert
produced.

Those rows get split three ways, and the split matters:

| group | rows | used for |
|---|---:|---|
| **training** | 185,527 | the only rows the network is allowed to learn from |
| **held-out laps** | 67,510 | two laps of each *training* track — "did it memorise this track?" |
| **held-out tracks** | 138,607 | three tracks it never sees — "did it learn to drive?" |

### Step 2 — the loop

One **epoch** is one pass over all 185,527 training rows. Training runs 60 of
them. Inside an epoch:

```
shuffle the rows                   (so the order never teaches anything)
take them 256 at a time — a "batch":

   forward    feed 720 laser numbers + 1 speed-limit number into the network,
              out come 2 numbers: steering and speed
   loss       measure how far those are from what the expert actually did
   backward   work out, for each of the 226,119 weights, which direction would
              have made that gap smaller
   step       nudge every weight a little in that direction
```

That last pair is all that "learning" means here: **measure the error, nudge
the weights to shrink it, repeat a few hundred thousand times.** Nothing more
mysterious is happening.

A detail worth understanding: steering is in radians (roughly ±0.46) and speed
in m/s (0 to 2.8). If you measured the error in raw units, speed would dominate
simply because its numbers are bigger, and the network would optimise speed at
steering's expense. So both are rescaled to the range −1…1 before the error is
measured — that way a 10 % mistake in steering counts the same as a 10 %
mistake in speed.

### Step 3 — checking honestly, once per epoch

After each epoch, the two held-out groups are run through the network **with
learning switched off**. That is the score that means something: the network
cannot have memorised those rows, because it has never been allowed to adjust
itself on them.

This is also how you see trouble. If the training error keeps falling while the
held-out error starts *rising*, the network has begun memorising instead of
generalising — **overfitting**. Ours did not: both were still improving at
epoch 59.

### Step 4 — keeping the best version, not the last

The weights are saved from whichever epoch scored best on the **held-out laps**
— not from the final epoch, which is not necessarily the best one.

Note *which* group is used for that choice. The held-out **tracks** are the
final exam, and they are never allowed to influence any decision during
training — not the stopping point, not the architecture, nothing. The moment
you use your test set to pick something, it stops being a test set and starts
being part of the training. That is one of the easiest ways to fool yourself in
machine learning, and it is easy to do by accident.

About 20 minutes per model on the CPU. For behaviour cloning that is nine
models: three architectures × three random starting points.

### What comes out: the checkpoint

One file: `checkpoints/final/bc_mlp_s0.pt`, 919 KB. Read the name as
*BC data, MLP architecture, seed 0*.

It is a dictionary containing:

| what | why it is in there |
|---|---|
| `state_dict` | **the weights** — 226,119 learned numbers. This is the model |
| `arch` = `mlp` | which network shape to rebuild before loading the weights into it |
| `K` = `1` | how many consecutive scans it expects (3 for the stacked version) |
| `cap_input` = `True` | it takes the speed limit as an extra input |
| `schema_version` | which data format it was trained against |
| `bins` | the 21+21 menu, for the discrete policy only (`None` for the others) |
| `metrics`, `train_config` | the scores, the full 60-epoch history, the seed, the config |

The weights themselves are just four tables of numbers:

```
trunk.0.weight  (256, 721)  184,576   ← 721 inputs: 720 laser + 1 speed limit
trunk.2.weight  (128, 256)   32,768
trunk.4.weight  (64, 128)     8,192
head.weight     (2, 64)         128   ← 2 outputs: steering, speed
```

**The design principle worth stating out loud: the file describes itself.** It
carries its own architecture, input width, bins and data-format version. You
cannot load it into the wrong network shape, because the file says which shape
it needs. Loading refuses outright if the data format has changed
incompatibly, and the driving node refuses to start if you tell it an
architecture that disagrees with the file.

This sounds like bureaucracy until you picture the alternative: a model saved
as bare weights, loaded a month later into a network someone has since edited.
It loads without complaint and drives subtly wrongly forever. **A model that
cannot be decoded without a second file is a model that will eventually be
decoded wrong.**

There is also a `.pt.json` beside each checkpoint with the same scores in
readable text, so you can compare models without loading PyTorch at all:

```bash
cat checkpoints/bc_mlp_s0.pt.json | python3 -m json.tool | head -20
```

### How the checkpoint makes the robot drive

Three lines, in `policy_tracker.py`:

```python
self.model, meta = checkpoint.load(ckpt)   # rebuild the network, load the weights
obs = scan_to_observation(msg.ranges)      # 1080 laser beams -> the 720 the model expects
a   = self.model(obs, cap)                 # -> steering, speed -> publish to the robot
```

The middle line is the one to point at. `scan_to_observation` is **the same
function that built the training data**, imported from the same file. Not a
reimplementation, not a copy — the same code. If that conversion ever changes,
it changes in one place and both training and driving follow it together.

The classic version of this bug is a model trained on data processed one way
and deployed with data processed slightly differently — beams in a different
order, a different unit, a different treatment of "no return". The model still
runs. It just quietly produces nonsense. Sharing the function makes that
impossible rather than unlikely.

### Running one yourself

```bash
# a policy on a track it has NEVER seen, with the GUI
scripts/watch_policy.sh checkpoints/final/bc_mlp_s0.pt SaoPaulo 2.1

# the expert on the same track, for comparison
scripts/watch_policy.sh pid SaoPaulo 2.1
```

A good order to learn from:

1. `... bc_mlp_s0.pt Nuerburgring 1.5` — a track it trained on, slowest speed.
   Does it drive at all?
2. `... bc_mlp_s0.pt SaoPaulo 2.1` — never seen. Does it generalise?
3. `... bc_mlp_s0.pt Silverstone 2.7` — never seen, fastest. In the evaluation
   this policy crashed here in all three seeds, at the same corner. Montreal
   (0.66 m of margin, the narrowest corridor) is the other hard one.

Watch the gap between the robot and the reference line in RViz. Expect visible
wobble where the expert has none — and if it leaves the track, **that is a
result, not a malfunction**. It is Part 3's compounding-error problem happening
in front of you, and it is exactly what the DAgger rounds exist to fix.

To score them all instead of watching one:

```bash
ros2 run hunter_dataset evaluate_policies \
    --plan src/hunter_dataset/config/collection_plan.yaml \
    --out results/eval_bc --laps 1.5 pid checkpoints/final/bc_*_s?.pt
```

That drives every policy on all three unseen tracks and writes `eval.csv`:
crashes, laps completed, and true distance from the line, averaged over the
three seeds.

---

## Part 6 — How we judge it

The obvious way to score a neural network is its **validation loss**: show it
examples it has never seen, and measure how far its answers are from the
teacher's.

**We do not decide anything with that number.** Here is why.

Validation loss is an *average over all situations*. But driving is decided by
the rare hard ones. A policy can be right 99 % of the time and still leave the
track, because the 1 % it gets wrong is the corner entry at Montreal. A student
can have an excellent average and be a menace.

So the real test is **closed-loop evaluation**: put each of the 18 policies in
the simulator and make it actually drive. We measure how far it really was from
the line (using the simulator's ground truth), whether it crashed, and how many
laps it finished.

Crucially, we test on **three tracks the students have never seen** —
Silverstone, São Paulo and Montreal. A policy that has memorised five circuits
will look brilliant on those five and fall apart on a sixth. The three held-out
tracks are the only honest test of whether it learned *driving* or just
*this track*.

We report the validation loss anyway, side by side. **If the two rankings
disagree — if the network with the best test score is not the best driver —
that is a finding worth writing up**, not a problem to hide. It is the whole
point of insisting on closed-loop evaluation.

---

## Part 7 — The results

The experiment finished on 2026-09-25. Every policy drove the three tracks it
had never seen — Silverstone, São Paulo and Montreal — three times each, once
per training seed. Nine runs per row.

| policy | crashed | laps finished | true error | *offline score* |
|---|---:|---:|---:|---:|
| **the teacher** (PID, sees its position) | 0 % | 1.00 | **0.019 m** | — |
| copying + plain network | **67 %** | 0.33 | 0.332 m | *19.4 mrad* |
| copying + 3 stacked scans | **56 %** | 0.33 | 0.420 m | *19.7 mrad* |
| copying + menu | **67 %** | 0.33 | 0.407 m | *21.5 mrad* |
| **DAgger + plain network** | **0 %** | **1.00** | **0.168 m** | *108.9 mrad* |
| DAgger + 3 stacked scans | 0 % | 1.00 | 0.195 m | *160.4 mrad* |
| DAgger + menu | 0 % | 1.00 | 0.249 m | *281.9 mrad* |
| **DAgger, rebalanced 50/50** | **0 %** | **1.00** | 0.175 m | *68.1 mrad* |

### The finding: the test scores were backwards

Read the last column against the first. **Offline, the copying policies score
5.6× better than the DAgger ones** — 19.4 against 108.9 milliradians of
disagreement with the teacher.

**In the simulator, the copying policies crash on 67 % of runs and the DAgger
policies on none.**

The measurement everyone reaches for first — how closely does the network
reproduce the teacher's answers on held-out examples — ranked the policies in
exactly the wrong order. Had we picked the winner by test score, we would have
shipped the one that drives into walls.

This is not a subtle margin or a statistical artefact. It is 67 % versus 0 %,
consistent across three seeds and three circuits. And the reason is the whole
of Part 3: a policy is judged on the states it *puts itself into*, and copying
a perfect driver teaches nothing about those.

> **If you present one slide, present this one.** It is the argument for
> closed-loop evaluation, and it cost about 50 hours of simulation (DAgger plus
> evaluation) to make concretely rather than rhetorically.

### The two questions, answered

**Does the network architecture matter? No.** Plain and stacked are
indistinguishable — 0.168 m against 0.195 m, both with zero crashes, and
within seed noise offline too. **This was predicted before any training ran**,
by the information-asymmetry measurement in Part 8: the current laser scan
already determines the teacher's steering, so a history of scans has nothing
left to contribute. A prediction that survives contact with the result is
worth more than a horse race.

**Does DAgger beat plain copying? Decisively.** 67 % → 0 % crash rate and 2×
better tracking. The mechanism was visible while it happened: in round 1 the
student crashed seven times, every one of them at Shanghai's tightest hairpin.
After the teacher relabelled those states, **rounds 3 and 4 ran all 30
rollouts on five circuits without a single crash.** (Round 2 failed, and round 5
had two failures on Monza — see below.)

### How good is "good"?

The best student drives every unseen circuit without crashing, holding 0.168 m
from the racing line. The teacher holds 0.019 m — but the teacher is reading
its exact position from the simulator, which no real robot can do.

So: roughly **nine times less accurate than a controller that cheats, using
nothing but a laser scanner, on tracks it has never seen.** Whether that is
good enough depends on the corridor: on Montreal, whose margin is 0.66 m, a
0.17 m RMS error is comfortable but not generous.

### What we would still like to know

- **Why are the DAgger policies worse against the teacher (108.9 vs 19.4 mrad)
  while driving so much better?** We tested this. DAgger data had grown to
  75 % of the training set, so we retrained forcing an equal mix of the two
  kinds of data in every batch. That **halved the offline error to 68.1 mrad
  and kept the perfect crash record** — same zero crashes, same 0.17 m
  accuracy. So most of the apparent collapse was the mix, not the data. But
  68.1 is still 3.5× the copying policies' 19.4, so part of the trade is
  genuine: a policy that can recover from mistakes is measurably less precise
  at imitating a driver who never makes any.
- Round 2 failed outright — all 15 rollouts ended stuck or crashing. Their
  rows were still used for training: up to each crash they are the student's
  own states labelled by the teacher, which is what DAgger collects. Rounds
  3–4 then ran all 30 rollouts without a crash; in round 5 two Monza rollouts
  failed at the same point (s ≈ 147 m). Round 2's failure happened in both
  runs of the experiment, so it is reproducible and unexplained.

---

## Part 8 — Measuring the ceiling before training anything

Here is a question worth asking before spending days on training: **is this
even possible?**

The teacher decides what to do based on its exact position. The student only
sees the laser. What if two different situations look *identical* to the laser
but need *different* actions? Then no network, however large, can get both
right. The limit would be set by the data, not by the design.

The name for this is **information asymmetry** — the teacher knows something
the student cannot see.

You can measure it without training anything:

1. Take all 391,644 laser scans.
2. For each one, find the 20 scans that look most similar to it.
3. Ask: did the teacher do the *same thing* in all 20, or wildly different
   things?

If similar-looking scans get similar actions, the laser contains enough
information and the problem is learnable. If they get scattered actions, it is
not.

**Result: steering scored 0.003–0.040, where anything under 0.2 is healthy.**
The laser determines the teacher's steering almost perfectly. The ceiling is
high; the problem is learnable.

That same measurement is what caught the speed-limit issue in Part 4: speed
scored 0.74–1.02 (bad) when we pooled all three limits together, but **0.000**
when we looked at each limit separately. That pattern — fine within a group,
terrible across groups — is precisely the signature of a missing input, and it
is what told us to feed the speed limit in.

> This is a nice thing to show people: a measurement that told us the answer
> *before* we trained anything, and then a second measurement from the same
> tool that found a design flaw.

---

## Part 9 — What went wrong (the most useful part)

Five things went wrong. All five were **silent**: every program reported that it
was healthy while producing wrong answers. If you take one professional lesson
from this project, that is it.

### 1. The teacher's clock was wrong

The teacher is supposed to think 50 times per simulated second. But the
simulator runs at about a third of real speed, and the teacher was counting in
*real* seconds — so it actually thought **170 times per simulated second**,
while still assuming it was 50.

Two consequences. Its steering behaved as if its tuning knobs were set three
times too high on one term and a third as high on another — so **the teacher's
driving depended on how busy the computer was**. And its recordings were
timestamped on a different clock from the laser's, so nothing lined up.

**How we caught it:** we deliberately recorded *one* lap and inspected it
before recording forty-five. That one lap produced **zero usable rows**. If we
had skipped the check, we would have collected five hours of unusable data —
and worse, might never have noticed the teacher was tuning-dependent.

### 2. "Go 1.5 m/s" was only a label

The speed limit was written into the file name and the run's record sheet — but
never into the file the controller actually reads. Every run would have driven
at 2.7 m/s while being *labelled* 1.5 or 2.1.

Nothing in the dataset could reveal this. The data would be perfectly
self-consistent and completely wrong.

### 3. The same bug again, one level down — and it cost 20 hours

We fixed #2 for the teacher. Then DAgger ran, and the student still ignored its
speed limit.

The settings were filed under the heading `pid_tracker`. But ROS matches
settings **by program name**, and the student's program is called
`policy_tracker`. It matched nothing, and silently fell back to its built-in
default of 2.7 m/s.

So for twenty hours the student drove at 2.7 m/s on every run, while the
watching teacher — which *did* get its settings correctly — was labelling those
moments with what it would do at 1.5 m/s. Two thirds of the data paired
"situations at high speed" with "actions for low speed".

**How we caught it:** the resulting policies were terrible in a very specific
way — they wanted to steer hard left all the time (a bias of 0.25 radians)
where the correct answer was roughly zero. That is not the signature of a
network that needs more training; it is the signature of contradictory
lessons. Twenty hours of simulation had to be thrown away.

**The fix is the interesting bit.** We corrected the filing, *and* we made the
student's runner **refuse to continue** if the student is ever seen commanding
a speed above its limit. The bug class is now impossible to have silently,
which is a stronger property than having fixed this instance of it.

### 4. Ghost programs

Python programs in ROS all show up in the process list under the same name
(`python3`), so a cleanup routine that kills programs *by name* never finds
them. We once had three leftover monitoring programs from three different runs
alive at the same time, quietly listening to the live system.

Now every runner tracks the programs it starts **by process ID**, shuts them
down in the right order, and refuses to start the next run until it has
confirmed the system is empty.

### 5. The menu of steering options collapsed (a design flaw, not a typo)

This one is different from the other four: nothing was miswired. The code did
exactly what it was told to. What it was told to do was wrong, and nobody
looked at the result.

**Background.** One of the three architectures does not output a steering
*number*. It picks from a menu of 21 preset steering options — see Part 4. The
question is where to put those 21 options along the range of ±0.461 radians.

**What we chose.** Not evenly. The expert steers near zero almost all the time,
so spreading the options evenly would waste most of them on hard-lock angles
the robot rarely uses. Instead we spaced them by **quantiles**: put the options
where the data actually is, so they bunch up near zero and spread out toward
the limits. That reasoning is standard and it is correct.

**What actually came out.** The menu was dumped while documenting the file
format, and it looked like this:

```
option  0:  covers  -0.461 .. -0.072   ← 84 % of the entire left steering range
option  7:  covers  -0.0008 .. -0.0004 ← a range no vehicle could express
option 10:  covers  -0.00005 .. 0.0    ← likewise
option 20:  covers  +0.050 .. +0.461   ← 89 % of the entire right steering range
```

The widest option is **9,931 times** wider than the narrowest. Eight of the 21
are spent distinguishing steering angles within ±0.0006 rad — differences far
below anything the steering can physically produce — while the two options at
the ends each swallow most of the usable range. Ask this policy for a hard
corner needing 0.4 rad and the best it can say is 0.106 rad. Speed is worse:
**14 of its 21 options are narrower than 0.001 m/s.**

**Why.** The steering distribution is not merely peaked, it is *extremely*
peaked — the expert holds almost exactly straight for most of every lap. Push
quantiles through a distribution that concentrated and they pile up on top of
each other. The idea was right; there was just no floor under how narrow an
option was allowed to get, and — the actual mistake — **nobody printed the
menu and looked at it.**

**What it cost.** The discrete policy scored 25 mrad against the plain
network's 19. That was being read as "discretisation is worse", which was not a
fair reading: that arm of the experiment was measuring *"discretisation with a
broken menu"*, which answers no question anyone asked.

It also explained something puzzling: the discrete policy barely gets
worse on unseen tracks (24.9 → 25.8) where the plain network degrades by 2.6×.
That is not robustness. It is the menu error being so large that it drowns out
whether the track is hard or easy.

**The fix** was small: mix even spacing with quantile spacing and put a floor
under the option width, then retrain the discrete models and re-run their
evaluation. After the fix, the copying + menu policy crashed in 67 % of runs
instead of 89 %, and DAgger + menu in 0 % instead of 11 %, in line with the
other designs. That is how we know the broken menu, not the idea of a menu, had
been driving the old numbers.

**The lesson**, and it is the same one as the other four: *the code was
correct, the output was never inspected.* There was a tool that prints a loud
warning when a column of data never varies (Part 4), and nothing equivalent for
the menu. Every check you write protects the thing you were
already worried about. This one existed for the data and not for the model.

### The pattern

Write this on the whiteboard when you present:

> **Every failure in this project looked healthy from the inside.** Nothing
> crashed. No error message appeared. The only way any of them was caught was
> by checking the *output* against something external — ground truth, a
> deliberately loud inspection, an assertion that says "if this is ever true,
> stop", or simply printing the thing and reading it.

That is why the dataset inspector prints a giant `!!!!` banner when a column of
numbers never changes, why the runner refuses to start on a dirty system, and
why we recorded one lap before recording forty-five.

---

## Part 10 — Explaining it to someone else

### In 30 seconds

> We let a controller cheat — it can see the robot's exact position, so it
> drives the racing line almost perfectly. We record what the laser scanner saw
> and what that controller did, hundreds of thousands of times. Then we train a
> neural network to produce the same commands from the laser alone, and we test
> it on tracks it has never seen.

### In two minutes, add

> The hard part is that copying a perfect driver teaches you nothing about
> recovering from mistakes, because a perfect driver never makes any. So the
> student drives while the teacher watches and says what it *would* have done,
> and we add those corrections to the lessons. Repeat five times, and the
> student learns to recover from its own errors. We compare six versions —
> three network designs × with and without those corrections — and we judge
> them by actually driving, not by test scores, because a policy can score well
> on average and still crash in the one corner that matters.

### The four ideas that carry the whole thing

1. **Privileged expert distillation** — let the teacher cheat so it is a good
   demonstrator, then distil it into a student that cannot.
2. **Covariate shift** — copying a perfect driver leaves you helpless the
   moment you are not perfect. (The bicycle-video analogy.)
3. **DAgger** — the student drives, the teacher narrates, the corrections go
   back into the lessons, repeat.
4. **Closed-loop evaluation** — test scores average over easy situations;
   driving is decided by hard ones. Judge it by driving, on tracks it has never
   seen.

### Questions you will probably get

**"Why not just train it on the real robot?"** Because it would crash a few
hundred times while learning, and the simulator gives us exact ground truth to
measure against. Transferring to real hardware is Phase 9.

**"Why is the teacher not just the final answer?"** Because it needs to know
exactly where it is, and a real robot does not. That is the entire reason the
student exists.

**"Isn't feeding it the speed limit cheating?"** The speed limit is a setting a
human chooses, not information about the world that the robot would have to
sense. Position — the thing the student genuinely must do without — stays
hidden. And we can switch the extra input off to test the strict version.

**"How do you know it hasn't just memorised the tracks?"** Three of the eight
tracks are never used in training at all, and that is where the results come
from. We also hold back two laps of each training track, which answers a
different question: memorising a track it *has* seen.

**"What if the laser simply isn't enough?"** We measured that before training
anything — similar-looking scans get similar teacher actions (score 0.003–0.040
where under 0.2 is healthy), so the information is there. See Part 8.

---

## Glossary

| Term | Plain meaning |
|---|---|
| **LiDAR / laser scan** | A sensor giving the distance to the nearest obstacle in many directions |
| **PID controller** | Classic control maths: steer in proportion to how far off you are (plus two correction terms) |
| **Expert / teacher** | The PID controller, allowed to see the robot's exact position |
| **Student / policy** | The neural network that drives using only the laser |
| **Privileged information** | What the teacher can see but the student cannot — here, exact position |
| **Distillation** | Training a student to reproduce a teacher's behaviour |
| **Behaviour cloning (BC)** | Plain copying: train only on recordings of the teacher driving |
| **Covariate shift** | The student drifts into situations its lessons never covered, and errors compound |
| **DAgger** | Student drives, teacher labels what it would have done, corrections are added to the lessons, repeat |
| **Ground truth** | The simulator's exact answer, used to grade — never given to the student |
| **Validation loss** | How far the network's answers are from the teacher's, on examples it did not train on |
| **Closed-loop evaluation** | Actually driving the robot and measuring the result — as opposed to marking its homework |
| **Held-out track** | A track never used in training, kept purely to test generalisation |
| **Seed** | The random starting point for training; several are used to tell real effects from luck |
| **Information asymmetry** | A measure of whether the student's sensor contains enough to reproduce the teacher |
| **Discretisation** | Picking from a menu of preset values instead of outputting an exact number |
| **Frame stacking** | Feeding the network several consecutive scans so it can perceive motion |
| **Episode** | One continuous stretch of driving, ending in a crash or when recording stops |
