# Real2Sim parameter identification

## Why

The gravity work left the arm with a compensation scale tuned per joint, and the
sweep that tuned it is already an identification experiment nobody is reading as
one. At each scale a known feedforward torque is published and the joint's
standing error is measured, which is exactly the observation a stiffness estimate
is made of.

Two things make this worth building out rather than leaving as a tuning aid.

**The existing `r2s fit` has no gravity term.** It fits

```
qdd = k (q_cmd - q) - d qd - f sign(qd)
```

per joint. On a real arm under gravity the true equation carries `- tau_g(q)/J`,
and with that term missing the regression has nowhere to put a standing load but
into `k`. It has only ever run on synthetic tracks, so this has never shown.

**The two experiments identify complementary halves of the same parameters.**
Writing the physical equation out:

```
J qdd = kp (q_cmd - q) - b qd - tau_f sign(qd) - tau_g(q) + tau_ff
```

`fit_second_order` divides through by `J`, so what it returns is
`k = kp/J`, `d = b/J`, `f = tau_f/J` — every parameter scaled by an inertia it
cannot separate. The gravity sweep works at `qd = qdd = 0`, where the same
equation collapses to

```
kp (q_cmd - q) = tau_g(q) - tau_ff,   so   error = (tau_g - s tau_model) / kp
```

which yields `kp` in N·m/rad on its own, with no inertia in it. Put the two
together and the inertia falls out as `J = kp / k`, and with it `b = d J` and
`tau_f = f J`. Neither experiment gives that alone.

That is the deliverable: an inertia, damping, friction and stiffness set per
joint, measured rather than taken from the URDF, which is what a simulator needs
to behave like this arm.

## What the data already says

From the two real sweeps on the right arm, at one pose. Slope of error against
scale is `-tau_model/kp`, so `kp` follows from a quantity we published and one we
measured:

| joint | `tau_model` (N·m) | `kp` from slope | `DEFAULT_KP` in the vendor header |
| --- | ---: | ---: | ---: |
| `r_aj_1` | +1.27 | ≈ 7.5 | 20 |
| `r_aj_2` | +5.14 | ≈ 15.4 | 20 |
| `r_aj_3` | +2.49 | ≈ 28.4 | 20 |

All three disagree with the header, and they scatter in both directions, which is
the signature of an under-determined fit rather than a surprising robot: at a
single pose, `kp`, the Coulomb friction, and any error in the modelled torque are
one equation in three unknowns. `r_aj_4` holding at exactly +0.0075 rad across
six consecutive scales says the same thing from the other side — inside the
stiction band the joint simply does not move, so those samples carry no
information about `kp` at all.

Separating them needs the load varied while friction stays put, which means
**sweeping at several poses**. The same joint at a different arm configuration
carries a different `tau_model`, and the pair of unknowns separates.

## Global constraints

Carried forward and still binding:

- Work only on the long-lived `jazzy` branch.
- Fake hardware is the default; reaching hardware needs an explicit flag, and
  nothing publishes without `--execute`.
- Profile limits are authoritative; every commanded torque passes
  `CommandGate.authorize_effort`.
- `import robot_control` must not require `rclpy`.
- Vendor tree changes need a declared patch and a `post_patch_sha256` update.
- Artifacts stay schema-versioned, checksummed, and tied to a profile and asset
  manifest hash.

One new constraint this plan introduces:

- **An identification run moves the arm through a designed set of poses on its
  own.** Every pose is checked against the profile before the run starts, not as
  it is reached, so a run that would leave the envelope is refused before
  anything moves rather than stopping somewhere unplanned.

## Design

### Where it goes in the pipeline

`r2s collect / normalize / fit / validate / export` is one axis: excite the arm
along a track, fit its dynamics, check on held-out data, export a bundle. The
static sweep is a second axis feeding the same bundle, not a replacement.

- `pose gravity` gains `--output`, writing what it already measures.
- New `r2s identify` consumes one or more of those files and fits `kp`,
  `tau_f`, and a per-joint torque-model correction across poses.
- `r2s fit` gains the gravity term it is missing, and combines a static estimate
  when given one, to report `J`, `b`, `tau_f` in physical units.

Keeping `identify` separate from `fit` rather than widening `fit` keeps the two
experiments' failure modes apart: a bad excitation track and a bad pose set fail
for unrelated reasons, and a combined stage would report one as the other.

### The static model

Per joint, over rounds indexed by pose `p` and scale `s`:

```
error(p, s) = ( alpha_j tau_model(p) - s tau_model(p) ) / kp_j  +  c_j
```

`alpha_j` corrects the modelled torque for that joint — URDF masses that do not
match the built arm, cabling, anything bolted on. `c_j` absorbs the
scale-independent offset, which is where stiction lives. Linear in
`1/kp_j`, `alpha_j/kp_j` and `c_j`, so it is a least-squares fit once there are
at least three distinct `tau_model` values, which is what several poses buys.

Samples inside the stiction band carry no gradient and must be excluded rather
than fitted: a joint that did not move between two scales tells you nothing about
its stiffness. Detected as consecutive scales whose error differs by less than
the measurement noise, and reported, not silently dropped.

### Pose design

Poses must vary the load on each joint independently enough to condition the fit.
A set that swings the shoulder while leaving the wrist in the same orientation
identifies the shoulder and tells you nothing new about the wrist. The generator
scores a candidate set by the conditioning of each joint's regression matrix and
reports the worst-conditioned joint, so an under-designed set is visible before
the run rather than after the fit produces a confident wrong number.

### Rejected alternatives

- **Trusting `DEFAULT_KP`.** It is a vendor constant that the hardware applies
  and the configuration does not, and all three fitted values disagree with it.
  Sim built on it would be wrong in a way nothing downstream could see.
- **Identifying from the dynamic track alone.** It cannot separate inertia from
  stiffness, which is the parameter a simulator most needs.
- **Adding gravity as a free per-joint bias in `fit_second_order`.** It would
  absorb the load without measuring it, and at a different pose the same bias
  would be wrong.

## Tasks

### Task 1 — persist what the sweep already measures

`pose gravity --output PATH` writes the pose, the modelled torque, the scales,
and the measured per-joint errors, checksummed and tied to the profile and asset
manifest hash like every other artifact.

**Done when** a sweep on fake hardware round-trips through the file, and a file
written against one profile is refused when loaded against another.

### Task 2 — `r2s identify`: fit `kp`, stiction and torque correction

Consume several sweep files, fit the static model per joint, exclude and report
stiction-band samples, and report a per-joint confidence from the residual and
the conditioning.

**Done when** the fit recovers known parameters from synthetic sweeps to within a
few percent, refuses a single-pose input as under-determined naming the joints it
cannot separate, and reports rather than hides excluded samples.

### Task 3 — the pose set, and a run that executes it

A generator producing a conditioned set of poses within the profile limits, and
`r2s identify --collect` driving the arm through them, sweeping at each, and
writing one file per pose. Every pose validated before the first move.

**Done when** a set is refused up front if any pose is outside the limits or the
worst joint conditioning is below threshold, and a real run produces files for
every pose without an intervening manual step.

### Task 4 — gravity in `r2s fit`, and the combined parameters

Subtract the modelled torque, scaled by the `alpha` from Task 2, before fitting;
then report `J = kp/k`, `b = d J`, `tau_f = f J` per joint.

**Done when** the same synthetic robot is recovered by both paths to agreement,
and the gravity term measurably reduces the fit residual on real data.

### Task 5 — bundle and export

Carry the identified parameters into the schema v2 bundle, so `validate` and
`export` cover them the way they cover the existing estimate.

**Done when** an exported bundle carries the parameters with provenance, and
validation fails a bundle whose parameters were fitted against a different
profile or manifest hash.

### Task 6 — documentation and measured record

README (Korean, operator order), `docs/cli.md` reference, and the real fitted
parameters with their residuals in `docs/jazzy-verification.md`.

## Risks

- **The arm moves itself through a pose set.** Largest new hazard here. Poses are
  validated before the run, torque is released on any exit, and the run refuses
  to start without `--execute` like everything else.
- **A confident wrong fit.** Least squares always returns something. Conditioning
  and residual are reported per joint, and a joint that cannot be identified is
  named rather than given a number.
- **Stiction is not a clean Coulomb term.** The model treats it as a constant
  offset, which is a first approximation; the residual is what says whether that
  was good enough, and it is reported per joint rather than averaged away.
- **`alpha` may absorb real modelling error.** A per-joint correction can hide a
  wrong centre of mass rather than reveal it. Cross-checking against the dynamic
  fit in Task 4 is what catches that, since an inertia inconsistent with the
  geometry shows up there.
