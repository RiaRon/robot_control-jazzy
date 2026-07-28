# Direct-torque identification: stiffness and friction without the gravity model

## The problem

`r2s identify` excites a joint by feeding forward a fraction of its *modelled*
gravity torque and reading the standing position error that impedance control
leaves behind. Measured on the OpenArm's right arm on 2026-07-28:

| joint | kp (N·m/rad) | alpha | rounds used / dropped |
|-------|-------------:|------:|----------------------:|
| r_aj_1 | 67.2 | 0.80 | 27 / 0 |
| r_aj_2 | 63.7 | 0.80 | 27 / 0 |
| r_aj_3 | 89.9 | 0.60 | 24 / 3 |
| r_aj_4 | 70.3 | 0.73 | 24 / 3 |
| r_aj_5 | 15.1 | **−0.075** | 10 / 17 |
| r_aj_6 | 11.3 | 0.61 | 10 / 17 |
| r_aj_7 | 13.4 | 0.36 | 14 / 13 |

The four proximal joints are sound. The three wrist joints are not, for two
reasons that the method cannot fix from inside itself.

**Gravity does not load a wrist.** r_aj_5 is a forearm roll: when the hand sits
on its axis, gravity makes no moment about it at any joint angle. The designed
pose set left j6 and j7 within 0.21 rad of zero at every pose, so j5's modelled
torque spanned 0.112 N·m across eighteen rounds against 3–11 N·m at the
shoulder. Feeding forward 0, 50 and 100% of 0.03 N·m moved nothing: the three
rounds of a pose returned tracking errors equal to five decimal places.

**The model can be wrong in sign.** The URDF hangs a 0.42 kg two-finger gripper
off *both* wrists; the right arm carries none. The wrist joints' modelled torque
is almost entirely that phantom tool, so it is not merely too large — its
direction is uncorrelated with the real load. `alpha` is a scalar correction and
absorbs magnitude error (the proximal joints' 0.60–0.83 is exactly the 20–40%
overestimate an extra 0.42 kg produces). It cannot absorb a sign, which is what
r_aj_5's negative alpha reports.

Both are instances of one root cause: **the excitation is the gravity model**,
so a joint gravity does not reach cannot be measured, and a joint whose model is
wrong is measured wrongly.

## What we are building

**A — measurement plumbing.** Publish an absolute feedforward torque, chosen by
us rather than derived from the model, and sweep it. At a fixed pose the
equilibrium is

```
error = (tau_gravity − tau_applied) / kp + c
```

so sweeping `tau_applied` gives a line of slope −1/kp. No gravity model, no sign
convention, no dependence on the pose loading the joint. Every joint becomes
measurable.

**B — friction model.** A staircase that runs up and back down traces a
hysteresis loop whose two branches are separated by the stiction band. Fitting
both branches yields kp from their slope, the Coulomb level `Fc` from their
separation, and the bias `Fo` from the midline — from data the current fit
throws away. The dynamic model then gains the terms to hold those numbers:
`sign(v)` becomes `tanh(k·v)` and a constant bias column is added, matching the
friction model OpenArm's own bilateral-control stack uses.

Explicitly **not** in scope: publishing friction compensation on the real robot.
That needs Fc, Fv and Fo measured first, and it is a separate deliverable.

## A — measurement plumbing

### A1. `robotctl pose torque`

A new pose stage, symmetric with `pose gravity`: at the arm's current pose it
drives each joint in turn through a torque staircase and writes one sweep file.

```
robotctl pose torque --group openarm_right_arm --execute \
    --deflection 0.05 --steps 7 --output ~/r2s/sweeps/torque0.json
```

- `--deflection` (rad, default 0.05): how far each joint should be pushed at the
  ends of its staircase. Sizing by deflection rather than by torque is what
  makes one flag work across joints whose stiffness differs sixfold, and it is
  what keeps the experiment inside the arm's own limits.
- `--steps` (default 7): staircase points from −tau_max to +tau_max. The run
  visits them ascending then descending, so it publishes `2*steps − 1` torques
  and the file holds `2*steps − 2` rounds per joint. The first torque is where
  the joint arrives from the probe: it is reached travelling *downward*, so it
  sits on the descending equilibrium, a full `2 Fc/kp` off the ascending line
  the fit would read it on. It is held like any other round and not recorded,
  which leaves every recorded round reached moving the way its branch says.
  `--steps 3` is therefore the smallest run that gives each branch the two
  rounds the fit needs.
- `--joint` (repeatable, default every joint in the group): which joints to
  excite.

Only the joint under test receives a feedforward torque; the rest receive zero.
The vendor hardware runs MIT mode — `tau = kp(q_des − q) + kd(qd_des − qd) +
tau_ff` in one packet — so the position loop continues to hold the whole arm
while one joint is pushed against it. Nothing is commanded to a new position.

**Sizing the torque.** kp is the unknown, so `tau_max` cannot be computed up
front; it is probed. Every deflection in the probe is a *difference between two
readings*, never a reading on its own: a tracking error is
`(tau_gravity − tau_applied)/kp + c`, so most of it is the load the joint is
carrying and the stiction holding it, and only what changed when a torque
arrived says anything about kp. For each joint:

- read the tracking error at zero torque, held as long as any other round;
- publish the seed torque and read again. A seed inside the joint's stiction
  band moves it not at all, and `SEED_TORQUE_NM` (0.05) is below every Fc on
  this arm, so the seed doubles until the joint moves — bounded by
  `MAX_PROBE_FRACTION` of the rating and by `MAX_SEED_DOUBLINGS`, refusing at
  either. The smallest seed that moved the joint is reported: it stood still at
  half that torque, so its Fc is bracketed between the two;
- double that seed once more and difference the two readings. Both were taken
  moving the same way, so the Coulomb offset cancels and the difference is the
  elastic `seed/kp`. Extrapolating from the breaking-loose reading alone would
  divide by `(seed − Fc)/kp` and run away;
- extrapolate to `--deflection`, publish that torque, and re-check once against
  the zero-torque baseline. If the deflection achieved misses what was asked by
  more than `RECHECK_TOLERANCE`, extrapolate once more from that
  better-conditioned measurement and use it. Once, not until it converges.

Three bounds refuse rather than clamp, because a torque large enough to trip
one is not a measurement anyone asked for:

- the profile's effort limit for that joint;
- `MAX_PROBE_FRACTION` (0.25) of that limit, so a runaway extrapolation from a
  near-zero seed deflection cannot ask for the joint's full rating;
- the deflection must not carry the joint within `MARGIN_RAD` (0.20) of a
  position limit, since a joint pressed into its stop reads as stiction. This is
  checked against the deflection asked for, and again against the one the probe
  measured — the ceiling above only bounds torque, not travel.

The probe runs once per joint at the first pose and the resulting torques are
reused at later poses. kp is a property of the drive, not of the pose, and
re-probing at every pose triples the wall-clock for a number that does not move.

### A2. Sweep schema v2

`GravitySweep` gains one field:

```python
#: (rounds, joints) the torque actually published, in N.m.
applied_torque: np.ndarray
```

This is not new information for gravity sweeps — `_frozen_rounds` already
computes `scales * modelled_torque` inline — it is that quantity given a name
and a place, so a sweep whose applied torque is *not* a multiple of the model
can be recorded at all.

`SWEEP_SCHEMA_VERSION` goes to 2. Each round writes `applied_torque` alongside
`scale`. `read_sweep` accepts both versions: a v1 file's applied torque is
reconstructed as `scale * modelled_torque`, which is what it always meant. The
nine sweep files already collected therefore keep working unchanged.

For a torque sweep `scale` is written as zeros: the field records what fraction
of the *model* was published, and the answer is none of it.

### A3. Regression generalisation

`fit_static_gravity` builds rows `(tau_model, −scale * tau_model, 1.0)` against
the measured error. The middle column is the applied torque wearing a disguise.
It becomes `(tau_model, −applied_torque, 1.0)`, and `_frozen_rounds` reads
`sweep.applied_torque` instead of recomputing the product.

For gravity sweeps this is an identity — the same numbers reach the same least
squares — so the existing estimates must reproduce exactly. For torque sweeps
the first column is the real gravity torque at that pose, which the sweep still
records, so a mixed set of gravity and torque sweeps fits jointly: gravity
sweeps vary column one and pin `alpha`, torque sweeps vary column two and pin
`kp`.

## B — friction model

### B1. Fitting Fc and Fo from the staircase

Static equilibrium with friction is not an equation but an inequality: the joint
holds wherever `|tau_gravity − tau_applied + kp·error| ≤ Fc`. A staircase
ascending then descending therefore traces two parallel branches:

```
deflection
    |            ___/          ascending branch
    |        ___/
    |    ___/                  slope = −1/kp on both
    |___/  ___/
    |  ___/                    descending branch
    |_/                        vertical gap = 2·Fc/kp
```

Fitting is two ordinary least squares, not a piecewise search:

1. Split the rounds by direction of travel.
2. Fit a line to each branch. Their slopes both estimate −1/kp; disagreement
   beyond `BRANCH_SLOPE_TOLERANCE` (0.25 of the mean) means the joint was not
   behaving like a spring with dry friction, and the joint is reported
   unidentifiable rather than fitted.
3. `kp` = −1 / mean slope.
4. `Fc` = (intercept difference) · kp / 2.
5. The midline intercept gives `tau_gravity + Fo` for that pose. Across poses,
   where `tau_gravity` varies and `Fo` does not, the two separate — the same
   argument that makes several poses necessary today.

The frozen-round exclusion does not apply to this fit. Rounds inside the band
are the measurement, not noise, which is the reversal that makes B worth doing:
seventeen of the wrist's twenty-seven discarded rounds become signal.

`StaticEstimate` gains `coulomb_nm` and `bias_nm` arrays, NaN where a joint was
measured only by gravity sweeps.

**Two measurements of the same quantity.** The dynamic fit already estimates a
Coulomb term, as a regression coefficient over a moving track. The staircase
measures it directly: a known torque, at rest, at the moment the joint breaks
free. The static value is the measurement of record and is what `bundle` carries
and `export` writes, wherever the staircase covered a joint; the dynamic
coefficient stays in the dynamic estimate because that model needs its own
internally consistent parameters. `validate` reports the two side by side, since
a large disagreement is evidence about the model rather than about the arm, and
silently preferring one would hide it.

### B2. `tanh` and a bias in the dynamic model

`identification.py` builds dynamic design rows
`[error, −velocity, −sign(velocity)]` and simulates

```
accel = kp·error − damping·velocity − friction·sign(velocity) − gravity/inertia
```

`sign` is discontinuous at zero velocity — precisely where a held pose lives —
so it cannot represent the band the arm actually has. The columns become
`[error, −velocity, −tanh(k·velocity), −1]` and the simulation follows.

`k` is fixed at `FRICTION_SHARPNESS = 100.0` (rad/s)⁻¹, so the term reaches 76%
of Fc at 0.01 rad/s. It is a constant rather than a fitted parameter because no
experiment in this pipeline observes it: the static staircase holds the arm
still, and identifying a transition sharpness needs a low-speed velocity sweep
we are not building. The constant is named and documented so that the day
someone measures it, there is one place to put it.

### B3. What reaches hdgp

`export --hdgp` substitutes `stiffness` and `damping` into hdgp's PD controller
and passes `friction` through as an additive Coulomb term. `Fc` maps onto that
existing field.

`Fo` has no counterpart in hdgp's actuator configuration. It is written to the
export's audit block, where the other unmapped measurement (inertia) already
lives, and it is applied inside robot_control's own simulation so that
`r2s validate` scores against a model that includes it. The spec records this as
a known gap rather than inventing a field hdgp would ignore.

## Testing

Unit, against the existing pytest suite (381 passing today):

- **A3 is an identity.** Re-fit the nine collected sweeps through the
  generalised regression and assert the estimate matches the committed
  `static_right.json` to within floating-point tolerance. This is the test that
  makes the refactor safe.
- **v1 sweeps still read.** A fixture written at schema 1 loads with
  `applied_torque == scale * modelled_torque`.
- **Torque sizing refuses rather than clamps** at each of the three bounds,
  naming the joint and the bound, following the gate's existing convention.
- **A synthetic spring with known kp, Fc and Fo** run through the staircase
  fitter recovers all three within tolerance, and a joint whose two branches
  disagree in slope is reported unidentifiable.
- **`tanh` model reduces to the old one** far from zero velocity: at
  |v| ≫ 1/k the new simulation and the old agree.

Hardware, on the right arm, in order:

1. `pose torque` at one pose, `--steps 3`, one wrist joint, dry run first.
2. The same with `--execute`, confirming the probe converges and the arm returns
   to its start.
3. A full run at three poses, fitted jointly with the nine gravity sweeps, with
   every joint identified and r_aj_5's alpha no longer negative.

## Risks

**A phantom-gripper URDF still corrupts `alpha`.** A and B measure kp, Fc and Fo
without the model, but `alpha` is by definition a statement about the model, and
the model describes a tool the right arm does not carry. Fixing it means
regenerating the description without the gripper, which the bimanual launch
cannot express per-arm today. Out of scope here; recorded so the wrist's alpha
is not read as trustworthy when the rest of the estimate is.

**Torque applied against a stuck joint winds up no further than commanded.**
Unlike a position command in an impedance loop, a feedforward torque does not
accumulate: the staircase publishes a fixed value per round and releases it at
the end. The failure mode to watch is a joint that has already been driven into
its stop by gravity, which `MARGIN_RAD` is there to prevent.

**Time.** Seven joints × 13 rounds × 2 s hold ≈ 3 minutes per pose, plus the
probe. Three poses is roughly ten minutes of arm time.
