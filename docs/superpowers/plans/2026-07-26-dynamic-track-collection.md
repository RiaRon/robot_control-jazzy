# Collecting a dynamic track from the real arm

## Why

`r2s fit --static` now produces an inertia, a damping and a friction in physical
units, verified against a synthetic robot. It has never seen a real track,
because there is no way to make one:

```python
# cli.py, r2s collect
if args.execute:
    print("ROS publisher backend is required; no command was published")
    return 2
```

That leaves the second half of the previous plan's Task 4 unmet — "the gravity
term measurably reduces the fit residual on real data" cannot be checked against
data that does not exist — and it is the only thing between the static
identification, which does reach hardware, and a bundle whose numbers came from
this arm.

## What is actually missing

Audited rather than assumed. Five gaps in `collect` itself:

1. **No publisher.** The stage computes the excitation, prints the sample count,
   and exits. It never constructs a `RosAdapter`.
2. **No recorder.** A fit needs the measured response. Nothing subscribes.
3. **No producer for `normalize`'s input.** `r2s normalize --input` wants an
   `.npz` carrying `command_time_ns`, `command`, `measured_time_ns`, `measured`,
   `joint_names`. Those field names appear in exactly three places: `normalize`
   itself, `track.py`, and one test fixture. **Nothing writes that file**, so the
   chain breaks at its first link.
4. **No group scoping.** The excitation is built over `profile.joints` — all 43,
   both arms plus the whole DG5F hand — with no `--group`. Every other command
   that reaches hardware is group-scoped, because a controller claims one group's
   interfaces.
5. **No gate.** `neutral` is the midpoint of each joint's range and `amplitude`
   is 5% of it; neither passes `CommandGate`. There is also no move to the start
   pose, so the first sample would be a jump from wherever the arm happens to be.

And three more found while reading the code around it:

6. **The adapter throws the timestamp away.**

   ```python
   def _record(self, message):
       self._latest = dict(zip(message.name, message.position))
   ```

   `header.stamp` is never stored. `joint_states()` then sets `self._latest =
   None` before waiting, deliberately, so `read_state` never returns a pose from
   before the motion it just commanded. Both are right for reading a pose and
   both are fatal for recording a stream: a recorder needs every message, kept,
   with the time the hardware read it.

7. **`/joint_states` is subscribed with `qos_profile_sensor_data`**, which is
   best-effort — messages can be dropped. `normalize_track` interpolates without
   looking at spacing, so a dropped run becomes a smooth line through data that
   was never measured. Nothing currently reports it.

8. **The three-repetition contract has no producer.** `split_repetitions` exists
   and is tested, the v2 bundle's `source` block carries `fit_runs` and
   `holdout_runs`, and `validate_holdout` wants holdout metrics — but nothing
   runs the excitation three times, and `r2s validate --metrics` therefore takes
   a hand-written JSON file. The holdout half of validation has never run on
   anything measured.

## The clock, which is where this goes wrong quietly

Both streams must be stamped **from the node clock**, and this is the one mistake
that would be invisible.

`normalize_track` computes `start = max(command_time_ns[0], measured_time_ns[0])`
and `stop = min(command_time_ns[-1], measured_time_ns[-1])`. If commands are
stamped with `time.monotonic()` — seconds since boot — and measurements with
`header.stamp` — seconds since the epoch, or since the simulation started under
`use_sim_time` — the two are on different epochs. The overlap is then either
empty, which raises `TrackError`, or worse, spuriously non-empty and the
interpolation is nonsense that fits without complaint.

So: `node.get_clock().now()` for the command, `message.header.stamp` for the
measurement. Never wall time, never monotonic, and correct under simulation time
for free.

### Why command and measurement are not paired into one row

A loop that publishes command X and writes `(X, whatever was just read)` asserts
that the measurement is the response to X. It is not: the state just read
responds to a command from some cycles earlier, and the message in hand was
stamped earlier still.

That delay is a **parameter being measured**, not an inconvenience —
`ControllerCalibration` carries `delay_sec` and `delay_steps`, and
`validate_holdout` checks `delay_residual_sec` against the command period.
Pairing at log time bakes in `delay = 0` and destroys the ability to recover it.
Interpolating onto a common grid afterwards, which is exactly what
`normalize_track` is for, keeps the alignment a decision that can still be
revised.

One writer appending to one file, two kinds of record, each with its own stamp.
Separate *files* or separate *threads* would buy nothing: at 100 Hz over seven
joints this is about 5.6 kB/s, so there is no throughput problem to solve, and
two writers would only add lock contention and partial writes.

## Global constraints

Carried forward and still binding:

- Work only on the long-lived `jazzy` branch.
- Fake hardware is the default; reaching hardware needs an explicit flag, and
  nothing publishes without `--execute`.
- Profile limits are authoritative; every commanded position passes `CommandGate`.
- `import robot_control` must not require `rclpy`.
- Vendor tree changes need a declared patch and a `post_patch_sha256` update.
- Artifacts stay schema-versioned, checksummed, and tied to a profile and asset
  manifest hash.

New for this plan:

- **A collection run streams thousands of commands, so the whole excitation is
  authorized before the first one is published.** Same rule as the pose-set
  itinerary: a run that stopped partway would leave the arm mid-excitation at a
  velocity nobody chose, and refusing up front costs nothing. The excitation is
  deterministic, so validating it whole is possible.
- **A recorded gap is reported, never interpolated over silently.** Best-effort
  QoS means dropped samples are normal; a smooth line through a hole that was
  never measured is not.

## Design

### Where the excitation starts

`neutral` becomes the arm's **current measured pose** by default, not the
midpoint of its range. Two reasons: the midpoint of the arms' symmetric limits is
all-zeros, which is the pose the arm is straight out and most loaded, and
starting where the arm already is removes the move-to-start jump entirely rather
than gating it. `--named` remains available for a known SRDF start when a run
needs to be comparable to an earlier one.

The excitation is then built around that pose, and `authorize_trajectory` over
the whole thing checks every sample against the position limits from where the
arm actually is — which is the check that matters, since `neutral + amplitude`
near a hard stop is exactly what the midpoint calculation could not see.

### What a recording is

One `.npz` per repetition, carrying what `normalize_track` already expects plus
what is needed to trust it:

- `command_time_ns`, `command` — stamped when published, from the node clock.
- `measured_time_ns`, `measured` — one row per `/joint_states` message actually
  received, stamped with `header.stamp`. Not resampled, not deduplicated: the
  arrival pattern is data about the pipeline being identified.
- `joint_names` — canonical, the group's, in the group's order.
- The profile, asset id and manifest hash, so a recording cannot be fitted
  against another robot.

Deliberately **not** a `CanonicalTrack`: that type is the *output* of
`normalize`, on a uniform grid. Writing one here would mean resampling at record
time, which is the pairing mistake in another costume.

### Three repetitions

`collect` runs the excitation three times and writes three recordings with a run
manifest naming them, which is what `split_repetitions` has always expected: two
to fit, one held out. The manifest is what lets `fit` and `validate` cite
`fit_runs` and `holdout_runs` in the bundle instead of a human filling them in.

### Rejected alternatives

- **Pairing command and measurement per sample at record time.** Discussed
  above; it destroys the delay, which is a parameter.
- **Recording with `read_state()` in the publish loop.** It nulls `_latest` and
  blocks until a fresh message arrives, so the publish rate would be hostage to
  the state rate, and every sample would still be unstamped.
- **Reliable QoS on `/joint_states` to avoid drops.** The publisher's QoS is not
  ours to choose, and a reliable subscription to a best-effort publisher simply
  does not connect. Drops have to be measured instead.
- **`CanonicalTrack` as the recording format.** Resampling at record time throws
  away the arrival pattern that says whether the recording is trustworthy.
- **Reusing `pose follow`'s streaming loop.** It clamps and reports, because an
  operator dragging faster than the arm can move is normal. A designed excitation
  that violates a limit is a bug in the design, and should refuse.

## Tasks

### Task 1 — record `/joint_states` with its timestamps

A recording path in the adapter that subscribes once, appends every message with
`header.stamp`, and never discards. Separate from `read_state`, whose
discard-and-wait behaviour is correct for reading a pose and wrong for a stream.
Reports how many messages arrived and the largest gap between them.

**Done when** a fake-hardware run records a monotonic stamped stream, the
existing `read_state` behaviour is unchanged, and the recorder's stamps come from
the node clock so they are on the same epoch as a published command's.

### Task 2 — `r2s collect --execute` publishes and records

Group-scoped, seeded from the current pose, whole excitation authorized before
the first sample, streamed at the profile's command rate, recorded to one `.npz`,
checksummed and tied to the profile and asset manifest hash.

**Done when** a run against fake hardware produces a recording that
`r2s normalize` accepts without an intervening manual step, an excitation that
would leave the envelope is refused before anything is published, and the arm is
left holding its last commanded pose on any exit including interruption.

### Task 3 — three repetitions and a run manifest

Three recordings per run, with a manifest naming which are for fitting and which
is held out, consumed by `split_repetitions`.

**Done when** a run writes three recordings and a manifest, `fit` names its
`fit_runs` from it, and a manifest whose recordings disagree about the profile,
the group or the joint order is refused.

### Task 4 — make a recorded gap visible

`normalize` reports the largest gap it interpolated across and refuses one longer
than a threshold, rather than drawing a smooth line through data that was never
measured.

**Done when** a recording with a deliberate hole in it is refused naming the gap
and where it is, a recording with ordinary jitter passes, and the reported drop
count matches what the recorder counted.

### Task 5 — compute holdout metrics instead of hand-writing them

`validate --metrics` takes a file nobody produces. With a manifest and a fitted
model, the held-out repetition's RMSE, delay residual and tracking improvement
are computed rather than asserted.

**Done when** `validate` runs from a manifest and a fit with no hand-written
metrics file, a model that does not predict the holdout fails, and the bundle's
`source` block cites the runs it was actually fitted on.

### Task 6 — documentation and measured record

README (Korean, operator order) for the collect step, `docs/cli.md` reference,
and the real fitted parameters with their residuals in
`docs/jazzy-verification.md` — including whether the gravity term reduces the
residual on real data, which is the half of the previous plan's Task 4 that this
work exists to settle.

## Risks

- **The arm is commanded through a designed excitation at the controller rate.**
  Larger than the pose-set risk: it is continuous motion rather than a move and a
  hold. Mitigated the same way — the whole excitation is authorized before the
  first sample, and refusing costs nothing. `--amplitude-scale` defaults low, and
  starting from the current pose means the first sample is a small step rather
  than a jump.
- **Dropped samples make a recording look better than it is.** Blind
  interpolation is smooth by construction. Task 4 exists because the fit would
  otherwise be quietly fitted to invented data.
- **A wrong clock is invisible.** Mixing epochs either raises `TrackError` — the
  good case — or produces an overlap that interpolates to nonsense and fits
  without complaint. Both streams take the node clock, and a test pins that they
  are comparable rather than merely present.
- **Excitation may not excite.** `normalize_track` already refuses a track whose
  commanded range is under `minimum_range_rad`, which catches a joint that never
  moved. It does not catch one that moved too slowly to show inertia; the
  residual and the two-inertia cross-check in `fit --static` are what say so.
- **Three repetitions triple the time on hardware.** Real, and not worth avoiding:
  a holdout that shares data with the fit validates nothing.
