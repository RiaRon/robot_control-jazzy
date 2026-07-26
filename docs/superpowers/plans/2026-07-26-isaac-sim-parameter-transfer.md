# Putting the identified parameters into Isaac Sim

## Why

The identification pipeline now produces `J`, `b`, `tau_f` and `kp` per joint,
measured rather than taken from the URDF. Nothing turns them into a simulator
that behaves like the arm.

This is the missing half of the reference recipe. That paper's real-to-sim
module (§III-A) closes its loop **on the simulator**: it samples parameter
combinations, runs the same joint-target sequence in parallel sim environments
and once on the real robot, and keeps whichever parameters minimise the
sim-versus-real tracking error. Everything built here so far is the measurement
side of that comparison — the real run and an analytic model of it. There is no
simulator in the loop at all.

Two things follow. The obvious one is that the parameters need to reach Isaac
Sim. The less obvious one is that a white-box identification changes what the
autotune search has to do: instead of sampling inertia across orders of
magnitude, it can search a bracket around a measured value, and the two
independent routes to that value already say whether it is trustworthy.

## What the asset already gives

`hdgp/assets/robot/openarm_tesollo_sensor_rl/` carries the USD beside the URDF,
and the USD is **layered**:

```
configuration/
  openarm_tesollo_sensor_rl_base.usd
  openarm_tesollo_sensor_rl_physics.usd     <- drive parameters live here
  openarm_tesollo_sensor_rl_robot.usd
  openarm_tesollo_sensor_rl_sensor.usd
```

That separation is the design hook. Identified parameters belong in a **physics
layer override**, never in the robot or base layers — the same discipline as the
vendor-patch rule, for the same reason: a measured number and a piece of
geometry have different provenance and different lifetimes, and mixing them
means neither can be re-derived.

The manifest's `control_joint_order` (34 joints) and `source_to_canonical_joints`
already map the asset's names onto the profile's canonical ones, and the
manifest hash `b89a946e…` matches what the profile pins. So the naming problem
is solved before this starts.

**The USD files are Git LFS pointers and are not fetched in this checkout.**
Nothing below runs until `git lfs pull` in `hdgp`.

## The mapping, and the double-counting trap in it

Isaac Sim drives a joint with an implicit PD actuator:

```
tau = stiffness * (q_des - q) + damping * (qd_des - qd)
```

alongside `armature`, `friction` and `max_effort` on the joint itself.

| identified | Isaac Sim | |
| --- | --- | --- |
| `kp` [N·m/rad] | drive `stiffness` | direct |
| `b` [N·m·s/rad] | drive `damping` | direct |
| `tau_f` [N·m] | joint `friction` | Coulomb |
| `J` [kg·m²] | **`armature`, less what the geometry already contributes** | not direct |

The trap is the last row. `J` as identified is the **total** inertia the joint
sees. The USD already contributes link inertia from the geometry, so:

```
armature = J_identified - J_from_geometry
```

Setting link inertia to `J` double-counts. Setting `armature` to `J` also
double-counts. Either way the simulated arm is heavier than the real one, every
policy trained on it learns to push harder than it needs to, and nothing
downstream can see the error — it looks like a robot, just not this one.

`armature` is the right home for the remainder because it is exactly what the
URDF cannot express: rotor and gearbox inertia reflected through the reduction.
On a harmonic-drive joint that term is not small.

`J_from_geometry` is computable from the chain `kinematics.py` already builds
for gravity, so this needs no new physics — the composite mass and centre of
mass are already lumped there.

**A negative armature is a finding, not a rounding error.** It says the measured
total inertia is less than the geometry alone implies, which means either the
identification or the URDF masses are wrong. It should be refused and named,
the way an unidentifiable joint already is.

## The symmetry that gives a verifier for free

`predict_second_order` **is a simulator** — a white-box one, with no geometry, no
contact and no cross-joint coupling. Isaac Sim is a black-box one with all
three.

The same held-out run and the same metric score both:

- the analytic model, replaying the holdout's commands → RMSE
- Isaac Sim, replaying the same commands → RMSE

Isaac Sim has strictly more information than the analytic model. So if it scores
**worse**, its parameters are wrong — that is not a threshold anyone has to
choose, it is an ordering that has to hold. **The analytic model is the control
group**, and it costs nothing because it already exists.

This is also the honest reading of the reference recipe's Table I, which reports
that lower autotune MSE goes with higher sim-to-real transfer. The metric there
is sim-versus-real tracking error on a shared command sequence, which is exactly
what `r2s collect` records and `score_holdout` computes.

## Isaac Sim's Python is not ROS's Python

Isaac Sim ships its own interpreter, a version behind ROS 2 Jazzy's 3.12. So the
sim side cannot import anything that needs `rclpy`.

It does not have to. The rclpy-free constraint and `requires-python >= 3.10`
mean `robot_control`'s core imports cleanly under Isaac Sim's Python, and the
boundary between the two worlds is already made of artifacts rather than
processes:

- the calibration bundle, schema v2 with its `identified` block
- the recording `.npz` from `r2s collect`

Both are JSON and numpy. **Nothing on the sim side talks to ROS.** Third
consumer of the same neutral core, after the two ROS branches — which is what
the Humble parity plan means when it says doing that port properly makes this a
consumer rather than a copy.

## Global constraints

Carried forward:

- Fake hardware is the default; reaching hardware needs an explicit flag.
- Profile limits are authoritative.
- `import robot_control` must not require `rclpy` — load-bearing here, not
  incidental.
- Artifacts stay schema-versioned, checksummed, and tied to a profile and asset
  manifest hash.

New:

- **Identified parameters are written as a physics layer override, never into
  the robot or base layers.** The asset is generated by
  `tools/generate_rl_urdf.py` and regenerating it must not silently discard a
  measurement, nor must a measurement silently become part of the geometry.
- **A simulator's parameters cite the identification they came from**, the same
  way the bundle's `identified` block cites its sweeps and its track. A physics
  layer with no provenance is a set of numbers somebody typed.

## Design

### Where the boundary sits

```
r2s collect  ->  run.npz          -.
r2s identify ->  static.json       |
r2s fit      ->  estimate.json     +->  r2s sim-params  ->  physics layer .usda
r2s bundle   ->  identified.json  -'                        + provenance

                 run.npz (holdout) ->  isaac replay  ->  sim_run.npz
                                                          |
                       score both against the same holdout, analytic as control
```

`r2s sim-params` is offline and needs neither ROS nor Isaac Sim: it reads a
bundle and the asset, and writes a layer. The replay needs Isaac Sim and nothing
else. Keeping those apart means the mapping is testable on any machine, and only
the replay needs the simulator installed.

### Why not drive Isaac Sim from ROS

`isaac_ros2_bridge` would let the existing `ros_adapter` command the sim as
though it were the robot, which is tempting because `r2s collect` would then work
unchanged.

It answers the wrong question. The point is to compare the simulator against a
recording of the real arm under the same commands; putting a ROS control stack
between them adds that stack's own delay and jitter to the sim side only, and
`delay_sec` is a parameter being measured. The replay should command the sim
directly at the recorded timestamps.

### Rejected alternatives

- **Sampling parameters from scratch, as the reference does.** It works, and it
  spends its search budget rediscovering an inertia that has already been
  measured twice by independent routes. Search the bracket instead.
- **Writing parameters into the robot layer.** Regenerating the asset would
  silently drop them, or worse, keep them and make the geometry unre-derivable.
- **Trusting the mapping without the replay.** The reference is explicit that
  identical physical constants do not imply identical kinematic and dynamic
  behaviour between a simulator and the world. The mapping is a hypothesis; the
  replay is the test.
- **Scoring the sim on its own, without the analytic control.** Then a bad RMSE
  has no interpretation — it could be the parameters, the contact model, or a
  hard task. Against the analytic model it has exactly one.

## Tasks

### Task 1 — map identified parameters onto drive parameters

`kp -> stiffness`, `b -> damping`, `tau_f -> friction`, and
`armature = J - J_from_geometry` with the geometry term computed from the chain
`kinematics.py` builds.

**Done when** a synthetic bundle round-trips to drive parameters and back, a
negative armature is refused naming the joint, and the geometry term matches an
independently computed composite inertia for a known chain.

### Task 2 — write the physics layer, with provenance

`r2s sim-params --bundle identified.json --output physics_identified.usda`,
carrying the profile, the asset manifest hash, and the sweeps and track the
numbers came from.

**Done when** the layer loads over the asset without touching the robot or base
layers, a layer written against one asset manifest is refused against another,
and re-running the asset generator leaves it intact.

### Task 3 — replay a recording in Isaac Sim

Drive the simulated arm with a recording's commands at its recorded timestamps
and write the response in the same `.npz` shape, so it is the same kind of object
as a real recording.

**Done when** a sim replay normalizes and scores through the existing
`normalize`/`score_holdout` path with no special-casing, and the sim's own
`/joint_states` equivalent carries stamps on one clock the way the real
recorder's do.

### Task 4 — score the simulator, with the analytic model as control

Report both RMSEs against the same holdout and refuse a parameter set where the
simulator does worse than the analytic model.

**Done when** a deliberately mis-scaled inertia makes the simulator lose to the
analytic model and is refused, and a correct one wins.

### Task 5 — autotune, searching the bracket rather than the space

The reference's §III-A loop, bounded by the identification: sample around the
measured values rather than across orders of magnitude, and use the
two-inertia disagreement as the bracket width.

**Done when** the search improves on the analytic mapping on real data, and
reports how much of the improvement came from parameters the white-box model
does not have — contact, coupling, geometry — because that is the only part
worth the compute.

### Task 6 — documentation and measured record

README (Korean, operator order), `docs/cli.md` reference, and the measured
sim-versus-real numbers in `docs/jazzy-verification.md` beside the real-arm
identification they came from.

## Risks

- **The USD assets are LFS pointers here.** Nothing runs until they are fetched;
  a plan that silently assumed otherwise would fail at Task 2.
- **Which actuator model the asset uses changes the mapping.** Implicit PD and
  explicit torque control map `kp` differently, and the physics layer has to be
  read before anything is written to it.
- **The identified `kp` is the motor firmware's, not the controller's.** It is
  `DEFAULT_KP` inside `v10_simple_hardware.cpp`, applied below anything
  `ros2_control` configures. The simulator has no firmware, so its drive
  stiffness has to reproduce the loop the firmware closes, not the command chain
  above it. Getting this backwards would produce a sim that tracks far better
  than the arm and a policy that never learns to fight droop.
- **No real data yet.** Every number this consumes comes from an identification
  that has not run on hardware. This plan is blocked behind that in a way the
  earlier ones were not: they could be verified against a synthetic robot,
  and here the synthetic robot is the thing being checked.
- **Contact is out of scope and is where the reference spends its effort.** This
  plan matches a free-moving arm's dynamics. Object modelling, contact
  parameters and the sparse/dense perception mix are separate work, and matching
  the arm is a precondition for them rather than a substitute.
