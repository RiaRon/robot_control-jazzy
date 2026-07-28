# Direct-Torque Identification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure joint stiffness and Coulomb friction by publishing a torque we
choose, so a joint gravity never loads — the three wrist joints — can be
identified at all.

**Architecture:** The static regression already multiplies a scale by a modelled
torque to get the torque it actually published; the change is to record that
product as a field and let it come from somewhere other than the model. A new
`pose torque` stage drives one joint at a time through a torque staircase, up
and back down, and the hysteresis between the two branches yields Coulomb
friction and bias alongside stiffness. The dynamic model then gains the terms
those numbers need: `tanh` in place of `sign`, and a bias column.

**Tech Stack:** Python 3.10, numpy, pytest, ROS 2 Humble (`rclpy` only inside
`ros_adapter`), `robotctl` console script.

## Global Constraints

- Repo: `/home/user/rl_ws/robot_control`, branch `humble`.
- The suite passes at **381 tests, 2 skipped** before this work. It must pass at
  every commit. Run `python3 -m pytest -q` from the repo root.
- **Nine sweep files already collected** at `~/r2s/sweeps/pose0.json` …
  `pose8.json` on the 5070ti (`usr@100.106.38.98`) must keep loading and must
  keep producing the same static estimate. They are schema version 1.
- TDD: every task writes a failing test first, runs it to see it fail, then
  implements. No implementation commit without a test in the same commit.
- Style: comments state constraints the code cannot show. Never restate what the
  next line does. Immutable/`frozen=True` dataclasses stay frozen.
- No new runtime dependency. numpy and the standard library only.
- Torque bounds **refuse** (raise), never clamp. This matches `authorize_effort`,
  which already refuses: "too much does not put the arm in the wrong place, it
  accelerates it out of the place it was holding."

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `src/robot_control/identification.py` (884 lines) | sweep dataclass, static fit, dynamic fit | `applied_torque` field; generalised regression; new staircase fitter; `StaticEstimate` fields; `tanh`+bias in dynamic design and simulation |
| `src/robot_control/artifacts.py` (307) | signed JSON read/write | sweep schema v2 with v1 back-compat |
| `src/robot_control/cli.py` (2236) | CLI stages | new `pose torque` stage, torque probe, staircase measurement loop |
| `tests/test_identification.py` | fit unit tests | staircase fitter, regression identity |
| `tests/test_sweep_artifacts.py` | artifact round-trips | v1/v2 sweep compatibility |
| `tests/test_cli_torque.py` (new) | `pose torque` behaviour | probe sizing, refusals, sweep shape |

`cli.py` is already 2236 lines. The torque-sweep measurement loop and probe are
~120 lines that belong together and have one job, so they go in a new module
`src/robot_control/excitation.py` and `cli.py` calls into it. This keeps
`cli.py` from growing and gives the probe a home that does not need argparse to
test.

---

### Task 1: Record the torque a sweep actually published

**Files:**
- Modify: `src/robot_control/identification.py:85-145` (`GravitySweep`)
- Test: `tests/test_identification.py`

**Interfaces:**
- Produces: `GravitySweep.applied_torque: np.ndarray` — (rounds, joints) N·m,
  a required constructor argument.

- [ ] **Step 1: Write the failing test**

Add to `tests/test_identification.py`:

```python
def test_a_sweep_records_the_torque_it_published():
    """The regression's second column is the applied torque, and it has to be
    able to come from somewhere other than a multiple of the model — that is
    the whole point of driving a joint gravity does not reach."""
    sweep = GravitySweep(
        group="openarm_right_arm",
        joint_names=("r_aj_1", "r_aj_2"),
        poses=np.zeros((2, 2)),
        modelled_torque=np.array([[1.0, 2.0], [1.0, 2.0]]),
        scales=np.zeros((2, 2)),
        applied_torque=np.array([[0.3, -0.3], [-0.3, 0.3]]),
        errors=np.zeros((2, 2)),
    )

    assert sweep.applied_torque.tolist() == [[0.3, -0.3], [-0.3, 0.3]]


def test_a_sweep_refuses_an_applied_torque_of_the_wrong_shape():
    with pytest.raises(FitError, match="applied_torque"):
        GravitySweep(
            group="openarm_right_arm",
            joint_names=("r_aj_1", "r_aj_2"),
            poses=np.zeros((2, 2)),
            modelled_torque=np.zeros((2, 2)),
            scales=np.zeros((2, 2)),
            applied_torque=np.zeros((2, 3)),
            errors=np.zeros((2, 2)),
        )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_identification.py -k applied_torque -v`
Expected: FAIL with `TypeError: __init__() got an unexpected keyword argument 'applied_torque'`

- [ ] **Step 3: Write minimal implementation**

In `identification.py`, add the field after `scales` and include it in the grid
validation tuple:

```python
    #: (rounds, joints) the fraction of it actually published.
    scales: np.ndarray
    #: (rounds, joints) the torque actually published, in N.m. Not derivable
    #: from scales once an excitation publishes a torque of its own choosing
    #: rather than a multiple of the model.
    applied_torque: np.ndarray
    #: (rounds, joints) the controller's own tracking error after the hold.
    errors: np.ndarray
```

and:

```python
    _GRIDS = ("poses", "modelled_torque", "scales", "applied_torque", "errors")
```

Update the `__post_init__` count message to name the new grid:

```python
            raise FitError(
                "every round needs a pose, a modelled torque, a scale, an "
                f"applied torque and an error, but the counts differ: {sorted(counts)}"
            )
```

- [ ] **Step 4: Run the whole suite**

Run: `python3 -m pytest -q`
Expected: FAIL — every existing `GravitySweep(...)` construction is now missing
an argument. That is the list of call sites Task 2 fixes. Note them.

- [ ] **Step 5: Fix every construction site**

Run `grep -rn "GravitySweep(" src tests` and give each one an
`applied_torque=` argument equal to `scales * modelled_torque`, which is what
that sweep published. In `cli.py:_measure_sweep` the value is the `effort` the
gate authorised — pass that through rather than recomputing it:

```python
            effort = gate.authorize_effort(torque * scales)
            _publish_for(adapter, effort, hold_sec)
            error = adapter.read_tracking_error()
            poses.append(state)
            torques.append(torque)
            applied.append(scales)
            published.append(effort)
```

and in the returned sweep:

```python
        scales=np.asarray(applied, dtype=float),
        applied_torque=np.asarray(published, dtype=float),
```

- [ ] **Step 6: Run the whole suite**

Run: `python3 -m pytest -q`
Expected: 383 passed, 2 skipped

- [ ] **Step 7: Commit**

```bash
git add src/robot_control/identification.py src/robot_control/cli.py tests/test_identification.py
git commit -F - <<'EOF'
feat: 스윕이 실제로 발행한 토크를 기록한다

정적 회귀의 2열은 이미 "실제 가한 토크"인데 scale x modelled로 매번 다시
계산하고 있었다. 모델의 배수가 아닌 토크를 발행하려면 그 값이 이름과 자리를
가져야 한다.
EOF
```

---

### Task 2: Read and write sweep schema v2

**Files:**
- Modify: `src/robot_control/artifacts.py:23` (`SWEEP_SCHEMA_VERSION`), `:139-178`
- Test: `tests/test_sweep_artifacts.py`

**Interfaces:**
- Consumes: `GravitySweep.applied_torque` from Task 1.
- Produces: `read_sweep` accepts schema 1 and 2; `write_sweep` emits 2.

- [ ] **Step 1: Write the failing test**

Add to `tests/test_sweep_artifacts.py`, which already owns sweep round-trips
and carries `profile`, `_sweep(profile, ...)` and `_resign` fixtures:

```python
def test_a_version_1_sweep_still_loads(tmp_path, profile):
    """Nine sweeps were collected before applied_torque existed. In a v1 file
    the applied torque is the scale times the model, because that is the only
    thing the old excitation could publish."""
    import json
    from robot_control.artifacts import read_sweep, write_sweep

    path = tmp_path / "v1.json"
    write_sweep(path, _sweep(), profile)
    payload = json.loads(path.read_text())
    payload["schema_version"] = 1
    for entry in payload["rounds"]:
        entry.pop("applied_torque")
    payload.pop("checksum_sha256", None)
    path.write_text(json.dumps(_resigned(payload)))

    loaded = read_sweep(path, profile)

    assert loaded.applied_torque == pytest.approx(
        loaded.scales * loaded.modelled_torque
    )


def test_a_version_2_sweep_round_trips_an_applied_torque_the_model_cannot_explain(
    tmp_path, profile
):
    from robot_control.artifacts import read_sweep, write_sweep

    sweep = _sweep(applied=np.full((2, 7), 0.4), modelled=np.zeros((2, 7)))
    path = tmp_path / "v2.json"
    write_sweep(path, sweep, profile)

    assert read_sweep(path, profile).applied_torque == pytest.approx(0.4)
```

Write `_sweep()` and `_resigned()` helpers in that file following the module's
existing fixture style; `_resigned` recomputes the artifact checksum the same
way `_sign_and_write` does, so the downgraded payload still verifies.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_sweep_artifacts.py -k applied -v`
Expected: FAIL — `write_sweep` emits no `applied_torque` key.

- [ ] **Step 3: Write minimal implementation**

```python
SWEEP_SCHEMA_VERSION = 2
```

In `write_sweep`'s round dict add:

```python
                    "applied_torque": sweep.applied_torque[index].tolist(),
```

In `read_sweep`, accept either version and reconstruct when the field is absent:

```python
    payload = _verify(path, profile, SWEEP_KIND, SWEEP_SCHEMA_VERSION, minimum=1)
    ...
        scales = np.array([entry["scale"] for entry in rounds], dtype=float)
        modelled = np.array(
            [entry["modelled_torque"] for entry in rounds], dtype=float
        )
        # A version 1 file predates the field. Its excitation could only ever
        # publish a multiple of the model, so that product is not a guess.
        applied = np.array(
            [entry["applied_torque"] for entry in rounds], dtype=float
        ) if "applied_torque" in rounds[0] else scales * modelled
```

`_verify` currently demands an exact version. Give it a `minimum` keyword
defaulting to the exact version so no other caller changes:

```python
def _verify(path, profile, kind, version, *, minimum=None):
    ...
    floor = version if minimum is None else minimum
    if not floor <= found <= version:
        raise ArtifactError(
            f"{kind} schema version {found} is not between {floor} and {version}"
        )
```

- [ ] **Step 4: Run the whole suite**

Run: `python3 -m pytest -q`
Expected: 385 passed, 2 skipped

- [ ] **Step 5: Verify against the real collected files**

```bash
scp usr@100.106.38.98:'~/r2s/sweeps/pose*.json' /tmp/realsweeps/
python3 - <<'PY'
from pathlib import Path
from robot_control.artifacts import read_sweep
from robot_control.profile import load_builtin_profile
profile = load_builtin_profile("openarm_tesollo")
for path in sorted(Path("/tmp/realsweeps").glob("pose*.json")):
    sweep = read_sweep(path, profile)
    print(path.name, sweep.rounds, "rounds, applied span",
          float(sweep.applied_torque.ptp()))
PY
```
Expected: all nine load, each reporting a non-zero applied span.

- [ ] **Step 6: Commit**

```bash
git add src/robot_control/artifacts.py tests/test_sweep_artifacts.py
git commit -F - <<'EOF'
feat: 스윕 스키마 v2 — applied_torque, v1 하위호환

v1 파일의 여진은 모델의 배수만 발행할 수 있었으므로 scale x modelled가
그 파일의 applied_torque다. 추측이 아니라 정의다. 수집해 둔 9개 파일이
그대로 읽힌다.
EOF
```

---

### Task 3: Fit against the applied torque instead of the scale

**Files:**
- Modify: `src/robot_control/identification.py:297-320` (`_frozen_rounds`), `:360-400` (`fit_static_gravity` row build)
- Test: `tests/test_identification.py`

**Interfaces:**
- Consumes: `GravitySweep.applied_torque`.
- Produces: no signature change. `fit_static_gravity(sweeps, *, noise_rad, max_condition) -> StaticEstimate` is unchanged.

- [ ] **Step 1: Write the failing test**

```python
def test_the_fit_reads_the_applied_torque_rather_than_recomputing_it():
    """A sweep whose applied torque is not a multiple of its model — which is
    what a direct-torque excitation produces — must still fit. Recomputing
    scale times model would read this sweep as having published nothing."""
    rounds = 5
    applied = np.linspace(-0.6, 0.6, rounds).reshape(rounds, 1)
    kp, gravity, offset = 20.0, 0.15, 0.001
    errors = (gravity - applied) / kp + offset

    sweep = GravitySweep(
        group="openarm_right_arm",
        joint_names=("r_aj_5",),
        poses=np.zeros((rounds, 1)),
        # The model says this joint carries nothing, and it is wrong.
        modelled_torque=np.zeros((rounds, 1)),
        scales=np.zeros((rounds, 1)),
        applied_torque=applied,
        errors=errors,
    )

    estimate = fit_static_gravity([sweep])

    assert estimate.stiffness[0] == pytest.approx(kp, rel=1e-6)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_identification.py -k applied_torque_rather -v`
Expected: FAIL — the joint is reported unidentifiable, because with
`modelled_torque` zero the recomputed applied torque is zero and the design
matrix is singular.

- [ ] **Step 3: Write minimal implementation**

In `_frozen_rounds`:

```python
    applied = sweep.applied_torque[:, joint]
```

In `fit_static_gravity`'s row loop:

```python
                torque = float(sweep.modelled_torque[index, joint])
                published = float(sweep.applied_torque[index, joint])
                rows.append((torque, -published, 1.0))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_identification.py -k applied_torque_rather -v`
Expected: PASS

- [ ] **Step 5: Prove the change is an identity on real data**

This is the test that makes the refactor safe. Copy the nine real sweeps into
`tests/data/sweeps/` (they are signed artifacts of a real run and belong in the
repo as fixtures) and add:

```python
REAL_SWEEPS = Path(__file__).parent / "data" / "sweeps"
#: Measured on the right arm, 2026-07-28, and recorded here because a
#: generalised regression that changes them is a regression, not a refactor.
EXPECTED_STIFFNESS = {
    "r_aj_1": 67.23, "r_aj_2": 63.73, "r_aj_3": 89.86, "r_aj_4": 70.26,
    "r_aj_5": 15.09, "r_aj_6": 11.30, "r_aj_7": 13.40,
}


def test_the_generalised_regression_reproduces_the_measured_estimate(profile):
    from robot_control.artifacts import read_sweep

    sweeps = [read_sweep(path, profile) for path in sorted(REAL_SWEEPS.glob("pose*.json"))]

    estimate = fit_static_gravity(sweeps)

    for index, name in enumerate(estimate.joint_names):
        assert estimate.stiffness[index] == pytest.approx(
            EXPECTED_STIFFNESS[name], rel=1e-3
        ), f"{name} moved; the regression change was not an identity"
```

- [ ] **Step 6: Run the whole suite**

Run: `python3 -m pytest -q`
Expected: 387 passed, 2 skipped

- [ ] **Step 7: Commit**

```bash
git add src/robot_control/identification.py tests/test_identification.py tests/data/sweeps
git commit -F - <<'EOF'
feat: 정적 회귀가 발행된 토크를 직접 읽는다

2열이 -scale*tau_model에서 -applied_torque로 일반화된다. 중력 스윕에는
항등이라 실측 9개 스윕의 추정치가 그대로 재현되어야 하고, 그걸 픽스처로
못박았다. 모델 토크가 0인 관절도 이제 적합된다.
EOF
```

---

### Task 4: Size a probe torque from a target deflection

**Files:**
- Create: `src/robot_control/excitation.py`
- Test: `tests/test_excitation.py` (new)

**Interfaces:**
- Produces:
  ```python
  MAX_PROBE_FRACTION = 0.25
  MARGIN_RAD = 0.20
  SEED_TORQUE_NM = 0.05

  class ExcitationRefused(ValueError): ...

  def probe_torque(
      *, deflection_rad: float, seed_torque_nm: float, seed_deflection_rad: float,
      effort_limit_nm: float, position_rad: float, lower_rad: float,
      upper_rad: float, joint: str,
  ) -> float
  ```
  Returns the torque that should produce `deflection_rad`. Raises
  `ExcitationRefused` naming *joint* and the bound it broke.

- [ ] **Step 1: Write the failing test**

```python
"""Sizing the excitation torque, which cannot be computed in advance.

kp is the unknown the experiment is for, so the torque that produces a wanted
deflection is found by pushing a little and extrapolating. Everything here is
arithmetic on a measured response — no robot, no ROS — so it is tested directly
rather than through the CLI.
"""

import pytest

from robot_control.excitation import ExcitationRefused, probe_torque

BOUNDS = dict(
    effort_limit_nm=7.0, position_rad=0.0, lower_rad=-1.57, upper_rad=1.57,
    joint="r_aj_5",
)


def test_it_extrapolates_linearly_from_the_seed_response():
    # 0.05 N.m moved it 0.004 rad, so kp is 12.5 and 0.05 rad wants 0.625.
    torque = probe_torque(
        deflection_rad=0.05, seed_torque_nm=0.05, seed_deflection_rad=0.004, **BOUNDS
    )

    assert torque == pytest.approx(0.625)


def test_it_refuses_rather_than_clamps_at_the_effort_limit():
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*effort"):
        probe_torque(
            deflection_rad=0.05, seed_torque_nm=0.05,
            seed_deflection_rad=1e-5, **BOUNDS
        )


def test_it_refuses_a_torque_over_the_probe_fraction_of_the_rating():
    """A near-zero seed deflection extrapolates to a huge torque. The joint is
    rated for it; the experiment is not, and asking for a joint's full rating
    is not a measurement anyone requested."""
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*25%"):
        probe_torque(
            deflection_rad=0.05, seed_torque_nm=0.05,
            seed_deflection_rad=0.001, **BOUNDS
        )


def test_it_refuses_a_deflection_that_would_reach_a_stop():
    """A joint pressed into its stop reads as stiction, so the measurement
    would be of the stop rather than of the drive."""
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*limit"):
        probe_torque(
            deflection_rad=0.05, seed_torque_nm=0.05, seed_deflection_rad=0.004,
            effort_limit_nm=7.0, position_rad=1.54, lower_rad=-1.57,
            upper_rad=1.57, joint="r_aj_5",
        )


def test_it_refuses_a_seed_the_joint_did_not_respond_to():
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*did not move"):
        probe_torque(
            deflection_rad=0.05, seed_torque_nm=0.05,
            seed_deflection_rad=0.0, **BOUNDS
        )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_excitation.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'robot_control.excitation'`

- [ ] **Step 3: Write minimal implementation**

Create `src/robot_control/excitation.py`:

```python
"""Choosing the torque a stiffness measurement should publish.

The gravity excitation had nothing to choose: it published a fraction of a
torque the model handed it. Publishing a torque of our own means deciding how
big, and the joint's stiffness — the thing being measured — is what sets that.
So it is probed: push a little, read the response, extrapolate.
"""

from __future__ import annotations


#: Ceiling on the probe, as a fraction of the joint's rating. A seed that
#: barely moves extrapolates to an enormous torque; the joint is rated for it
#: and the experiment is not.
MAX_PROBE_FRACTION = 0.25
#: How close to a position limit the deflected joint may come. A joint pressed
#: into its stop is held by the stop, and reads as stiction.
MARGIN_RAD = 0.20
#: The first push, small enough to be safe on the stiffest joint in the arm.
SEED_TORQUE_NM = 0.05


class ExcitationRefused(ValueError):
    """A torque the experiment will not publish, naming the bound it broke."""


def probe_torque(
    *,
    deflection_rad: float,
    seed_torque_nm: float,
    seed_deflection_rad: float,
    effort_limit_nm: float,
    position_rad: float,
    lower_rad: float,
    upper_rad: float,
    joint: str,
) -> float:
    if abs(seed_deflection_rad) < 1e-6:
        raise ExcitationRefused(
            f"{joint} did not move under {seed_torque_nm:g} N.m, so its "
            "stiffness cannot be extrapolated; raise the seed torque"
        )
    wanted = abs(deflection_rad * seed_torque_nm / seed_deflection_rad)

    room = min(position_rad - lower_rad, upper_rad - position_rad) - MARGIN_RAD
    if deflection_rad > room:
        raise ExcitationRefused(
            f"{joint} has {room:.3f} rad to a position limit before its "
            f"{MARGIN_RAD:g} rad margin, less than the {deflection_rad:g} rad "
            "deflection asked for; move to a pose with more room"
        )
    ceiling = MAX_PROBE_FRACTION * effort_limit_nm
    if wanted > ceiling:
        raise ExcitationRefused(
            f"{joint} would need {wanted:.3f} N.m for {deflection_rad:g} rad, "
            f"over {MAX_PROBE_FRACTION:.0%} of its {effort_limit_nm:g} N.m "
            "rating; ask for a smaller deflection"
        )
    if wanted > effort_limit_nm:
        raise ExcitationRefused(f"{joint} would need {wanted:.3f} N.m, over its effort limit")
    return wanted
```

Order matters: the position check runs before the ceiling check so a pose with
no room is reported as a pose problem rather than as a torque problem.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_excitation.py -v`
Expected: 5 passed. Note `test_it_refuses_rather_than_clamps_at_the_effort_limit`
passes via the 25% ceiling — that is correct; the raw limit is the backstop.

- [ ] **Step 5: Commit**

```bash
git add src/robot_control/excitation.py tests/test_excitation.py
git commit -F - <<'EOF'
feat: 목표 변위에서 여진 토크를 탐색한다

kp가 미지수라 토크를 미리 계산할 수 없다. 작은 씨앗 토크로 밀어 응답을 보고
외삽한다. 세 상한(정격의 25%, 정격 자체, 위치 한계까지의 여유)은 자르지 않고
거부하며 어느 관절이 어느 상한을 넘었는지 이름을 댄다.
EOF
```

---

### Task 5: Build the staircase a torque sweep publishes

**Files:**
- Modify: `src/robot_control/excitation.py`
- Test: `tests/test_excitation.py`

**Interfaces:**
- Produces:
  ```python
  def staircase(peak_nm: float, steps: int) -> list[float]
  ```
  Ascending from `-peak_nm` to `+peak_nm` in `steps` points, then descending
  back, without repeating the peak. Length `2 * steps - 1`.

- [ ] **Step 1: Write the failing test**

```python
def test_the_staircase_climbs_and_comes_back_without_repeating_the_peak():
    """Both directions of travel, because the gap between the ascending and
    descending branches is what measures Coulomb friction. A single direction
    traces one branch and measures nothing about it."""
    from robot_control.excitation import staircase

    values = staircase(peak_nm=0.6, steps=4)

    assert values == pytest.approx([-0.6, -0.2, 0.2, 0.6, 0.2, -0.2, -0.6])
    assert len(values) == 2 * 4 - 1


def test_the_staircase_needs_at_least_two_points_per_branch():
    from robot_control.excitation import ExcitationRefused, staircase

    with pytest.raises(ExcitationRefused, match="steps"):
        staircase(peak_nm=0.6, steps=1)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_excitation.py -k staircase -v`
Expected: FAIL with `ImportError: cannot import name 'staircase'`

- [ ] **Step 3: Write minimal implementation**

```python
def staircase(peak_nm: float, steps: int) -> list[float]:
    """Torques from -peak to +peak and back, one list, in visiting order.

    Both directions because a joint held by dry friction sits at a different
    equilibrium depending on which way it last moved, and the distance between
    those two equilibria is the measurement.
    """
    if steps < 2:
        raise ExcitationRefused(f"a staircase needs at least 2 steps, got {steps}")
    span = 2.0 * peak_nm
    up = [-peak_nm + span * index / (steps - 1) for index in range(steps)]
    return up + up[-2::-1]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_excitation.py -k staircase -v`
Expected: 2 passed

- [ ] **Step 5: Commit**

```bash
git add src/robot_control/excitation.py tests/test_excitation.py
git commit -F - <<'EOF'
feat: 계단 여진 — 올라갔다 내려온다

마른 마찰로 잡힌 관절은 마지막으로 움직인 방향에 따라 다른 평형에 앉는다.
그 두 평형 사이의 거리가 곧 측정값이므로 한 방향만 훑으면 잴 것이 없다.
EOF
```

---

### Task 6: `robotctl pose torque`

**Files:**
- Modify: `src/robot_control/cli.py` (parser in `_add_pose`, dispatch in `_pose`, new `_pose_torque`)
- Modify: `src/robot_control/excitation.py` (measurement loop)
- Test: `tests/test_cli_torque.py` (new)

**Interfaces:**
- Consumes: `probe_torque`, `staircase`, `SEED_TORQUE_NM` from Tasks 4–5;
  `GravitySweep(..., applied_torque=...)` from Task 1.
- Produces: `robotctl pose torque --group G [--deflection R] [--steps N]
  [--joint C]... [--hold-sec S] [--output P] [--execute]`, writing a sweep
  artifact readable by `r2s identify --sweep`.

- [ ] **Step 1: Write the failing test**

```python
"""`pose torque`: pushing a joint with a torque we chose, not one the model gave.

The stub arm is a linear spring with dry friction, so the sweep this writes
should be exactly what the staircase fitter in Task 7 can invert. The point of
testing at this level is the plumbing — one joint driven at a time, the arm
released at the end, the artifact shaped right — not the arithmetic, which
tests/test_excitation.py covers.
"""

import numpy as np
import pytest

from robot_control.cli import main

GROUP = "openarm_right_arm"
KP = np.array([67.0, 64.0, 90.0, 70.0, 15.0, 11.0, 13.0])


class SpringArm:
    """Deflects by torque/kp, and records every effort it was given."""

    def __init__(self):
        self.joints = np.zeros(7)
        self.published = []
        self.effort = np.zeros(7)

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def read_robot_description(self):
        return "<robot name='stub'/>"

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def send_effort(self, effort):
        self.effort = np.asarray(effort, dtype=float).copy()
        self.published.append(self.effort)

    def read_tracking_error(self):
        return self.effort / KP


@pytest.fixture
def arm(monkeypatch):
    from robot_control import ros_adapter

    stub = SpringArm()
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: stub)
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain", lambda *a: _flat_chain()
    )
    return stub


def test_it_drives_one_joint_at_a_time(arm, tmp_path):
    code = main(["pose", "torque", "--group", GROUP, "--execute",
                 "--steps", "3", "--hold-sec", "0.01",
                 "--output", str(tmp_path / "t.json")])

    assert code == 0
    for effort in arm.published:
        assert np.count_nonzero(effort) <= 1, (
            "only the joint under test may be pushed; the rest are held by "
            "the position loop"
        )


def test_it_releases_the_torque_when_it_finishes(arm, tmp_path):
    main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "3",
          "--hold-sec", "0.01", "--output", str(tmp_path / "t.json")])

    assert arm.published[-1].tolist() == [0.0] * 7


def test_the_sweep_it_writes_records_the_torque_and_not_a_scale(arm, tmp_path, profile):
    from robot_control.artifacts import read_sweep

    output = tmp_path / "t.json"
    main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "3",
          "--hold-sec", "0.01", "--joint", "r_aj_5", "--output", str(output)])

    sweep = read_sweep(output, profile)
    assert sweep.rounds == 2 * 3 - 1
    assert np.count_nonzero(sweep.scales) == 0
    assert np.ptp(sweep.applied_torque[:, 4]) > 0


def test_a_dry_run_publishes_nothing(arm, tmp_path):
    assert main(["pose", "torque", "--group", GROUP, "--steps", "3"]) == 0
    assert arm.published == []
```

Reuse the flat-chain helper from `tests/test_cli_collect.py` for `_flat_chain`.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_cli_torque.py -v`
Expected: FAIL — argparse exits 2, `invalid choice: 'torque'`.

- [ ] **Step 3: Add the measurement loop to `excitation.py`**

```python
def measure_staircase(
    adapter, gate, group, *, joints, deflection_rad, steps, hold_sec, publish
):
    """Drive each joint through its staircase and collect the rounds.

    *publish* is a callable taking (effort, seconds); the caller owns how a
    torque is held, so this stays free of ROS and of wall-clock policy.

    Every joint but the one under test receives zero. The vendor hardware runs
    MIT mode, so the position loop is still holding the whole arm while one
    joint is pushed against it; nothing here commands a position.
    """
    width = len(group.joints)
    poses, applied, errors = [], [], []
    try:
        for name in joints:
            index = group.joints.index(name)
            limit = group.limits[index]
            seed = np.zeros(width)
            seed[index] = SEED_TORQUE_NM
            publish(gate.authorize_effort(seed), hold_sec)
            response = float(adapter.read_tracking_error()[index])

            peak = probe_torque(
                deflection_rad=deflection_rad,
                seed_torque_nm=SEED_TORQUE_NM,
                seed_deflection_rad=response,
                effort_limit_nm=limit.effort,
                position_rad=float(adapter.read_state()[index]),
                lower_rad=limit.lower,
                upper_rad=limit.upper,
                joint=name,
            )
            for value in staircase(peak, steps):
                effort = np.zeros(width)
                effort[index] = value
                effort = gate.authorize_effort(effort)
                publish(effort, hold_sec)
                poses.append(adapter.read_state())
                applied.append(effort)
                errors.append(adapter.read_tracking_error())
    finally:
        publish(np.zeros(width), 0.0)
    return np.asarray(poses), np.asarray(applied), np.asarray(errors)
```

Add `import numpy as np` and pass the group's per-joint limit objects in as
`group.limits`; if the profile group does not carry them, build the list in
`cli.py` with the existing `_joint_limits(profile, group)` helper and pass it as
a `limits=` argument rather than reaching through `group`.

- [ ] **Step 4: Add the CLI stage**

In `_add_pose`, after the `gravity` parser:

```python
    torque = stages.add_parser(
        "torque",
        help="push each joint with a torque of our own and measure the response",
    )
    torque.add_argument("--profile", default="openarm_tesollo")
    torque.add_argument("--group", required=True)
    torque.add_argument(
        "--deflection",
        type=float,
        default=DEFAULT_DEFLECTION_RAD,
        help="radians to push each joint at the ends of its staircase; the "
        "torque that produces it is probed, since stiffness is the unknown",
    )
    torque.add_argument("--steps", type=int, default=7)
    torque.add_argument(
        "--joint",
        action="append",
        help="canonical joint to excite; repeatable. Default: every joint",
    )
    torque.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    torque.add_argument("--output", type=Path)
    torque.add_argument("--execute", action="store_true")
```

with `DEFAULT_DEFLECTION_RAD = 0.05` beside the other CLI constants, and in
`_pose`'s dispatch:

```python
        if args.stage == "torque":
            return _pose_torque(args, profile)
```

and the stage itself, following `_pose_gravity`'s shape:

```python
def _pose_torque(args, profile) -> int:
    """Measure stiffness with a torque we chose rather than one gravity gave.

    `pose gravity` can only publish a multiple of the modelled torque, so a
    joint the model says carries nothing — a wrist whose tool sits on its own
    axis — cannot be excited at all, whatever the scale.
    """
    from .ros_adapter import RosAdapter

    group = _group(profile, args.group)
    if not group.compensable:
        raise ValueError(
            f"group {group.name!r} declares no effort_controller, so torque "
            "cannot be published for it"
        )
    joints = args.joint or list(group.joints)
    for name in joints:
        if name not in group.joints:
            raise ValueError(
                f"{name!r} is not a joint of {group.name!r}; it has "
                f"{list(group.joints)}"
            )
    if args.output is not None and not args.execute:
        raise ValueError(
            "--output records what the arm measured, so it needs --execute"
        )

    limits = _joint_limits(profile, group)
    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        gate = _gate(profile, group, seed=None)
        print(
            f"{group.name}: {len(joints)} joints, {2 * args.steps - 1} rounds "
            f"each, {args.deflection:g} rad target deflection"
        )
        if not args.execute:
            print("DRY RUN: nothing published; pass --execute")
            return 0

        poses, applied, errors = measure_staircase(
            adapter, gate, group,
            joints=joints, limits=limits, deflection_rad=args.deflection,
            steps=args.steps, hold_sec=args.hold_sec,
            publish=lambda effort, seconds: _publish_for(adapter, effort, seconds),
        )
        sweep = GravitySweep(
            group=group.name,
            joint_names=tuple(group.joints),
            poses=poses,
            # No gravity model was consulted, so there is no modelled torque to
            # report and no fraction of one that was published.
            modelled_torque=np.zeros_like(applied),
            scales=np.zeros_like(applied),
            applied_torque=applied,
            errors=errors,
        )
        if args.output is not None:
            write_sweep(args.output, sweep, profile)
            print(f"wrote {args.output}")
    return 0
```

- [ ] **Step 5: Run the whole suite**

Run: `python3 -m pytest -q`
Expected: 392 passed, 2 skipped

- [ ] **Step 6: Commit**

```bash
git add src/robot_control/cli.py src/robot_control/excitation.py tests/test_cli_torque.py
git commit -F - <<'EOF'
feat: robotctl pose torque — 우리가 고른 토크로 관절을 민다

pose gravity는 모델 토크의 배수만 발행할 수 있어, 모델이 "아무것도 안 든다"고
하는 관절(공구가 자기 축 위에 얹힌 손목)은 scale을 무엇으로 두든 여진이 0이다.
한 번에 한 관절만 밀고 나머지는 0을 받는다 — MIT 모드라 위치 루프가 팔 전체를
계속 잡고 있다.
EOF
```

---

### Task 7: Fit stiffness, Coulomb friction and bias from the staircase

**Files:**
- Modify: `src/robot_control/identification.py` (`StaticEstimate`, new `fit_staircase`)
- Test: `tests/test_identification.py`

**Interfaces:**
- Produces:
  ```python
  BRANCH_SLOPE_TOLERANCE = 0.25

  def fit_staircase(sweeps, *, joint_names=None) -> StaticEstimate
  ```
  and on `StaticEstimate`: `coulomb_nm: np.ndarray`, `bias_nm: np.ndarray`,
  both NaN for joints no staircase covered.

- [ ] **Step 1: Write the failing test**

```python
def _staircase_sweep(kp, coulomb, gravity, peak=0.6, steps=5):
    """A joint that behaves exactly like a spring with dry friction."""
    from robot_control.excitation import staircase

    values = staircase(peak, steps)
    errors, held = [], None
    for index, applied in enumerate(values):
        target = (gravity - applied) / kp
        if held is None or abs(target - held) > coulomb / kp:
            direction = 1.0 if index < steps else -1.0
            held = target + direction * coulomb / kp
        errors.append(held)
    return GravitySweep(
        group="openarm_right_arm",
        joint_names=("r_aj_5",),
        poses=np.zeros((len(values), 1)),
        modelled_torque=np.zeros((len(values), 1)),
        scales=np.zeros((len(values), 1)),
        applied_torque=np.array(values).reshape(-1, 1),
        errors=np.array(errors).reshape(-1, 1),
    )


def test_the_staircase_recovers_stiffness_friction_and_bias():
    """Rounds inside the stiction band are the measurement here, not noise.
    The gravity fit drops them; seventeen of the wrist's twenty-seven rounds
    were dropped that way, and they are exactly the ones that carry Fc."""
    estimate = fit_staircase([_staircase_sweep(kp=15.0, coulomb=0.08, gravity=0.2)])

    assert estimate.stiffness[0] == pytest.approx(15.0, rel=0.05)
    assert estimate.coulomb_nm[0] == pytest.approx(0.08, rel=0.15)
    assert estimate.bias_nm[0] == pytest.approx(0.2, rel=0.15)


def test_a_joint_whose_branches_disagree_in_slope_is_not_identified():
    """Two branches that are not parallel did not come from a spring with dry
    friction, and a number fitted to them would describe nothing."""
    sweep = _staircase_sweep(kp=15.0, coulomb=0.08, gravity=0.2)
    bent = sweep.errors.copy()
    bent[len(bent) // 2:] *= 3.0
    sweep = replace(sweep, errors=bent)

    estimate = fit_staircase([sweep])

    assert np.isnan(estimate.stiffness[0])
    assert any("r_aj_5" == name for name, _ in estimate.unidentifiable)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_identification.py -k staircase -v`
Expected: FAIL with `ImportError: cannot import name 'fit_staircase'`

- [ ] **Step 3: Write minimal implementation**

Add the two fields to `StaticEstimate` with defaults so existing constructions
keep working:

```python
    #: Fc, N.m: the torque the joint holds without any position error at all,
    #: measured as the gap between the staircase's two branches. NaN where only
    #: gravity sweeps covered the joint.
    coulomb_nm: np.ndarray | None = None
    #: Fo + tau_gravity, N.m: where the staircase's midline crosses zero
    #: torque. NaN where only gravity sweeps covered the joint.
    bias_nm: np.ndarray | None = None
```

and the fitter:

```python
BRANCH_SLOPE_TOLERANCE = 0.25


def fit_staircase(sweeps: Sequence[GravitySweep]) -> StaticEstimate:
    """Fit kp, Coulomb friction and bias from a torque staircase.

    Static equilibrium under dry friction is an inequality, not an equation:
    the joint holds anywhere within Fc of balance. Running the torque up and
    back down therefore traces two parallel lines whose slope is -1/kp and
    whose separation is 2 Fc / kp, so both come out of two ordinary least
    squares rather than a search over breakpoints.
    """
    sweeps = list(sweeps)
    if not sweeps:
        raise FitError("a staircase fit needs at least one sweep")
    names = sweeps[0].joint_names
    width = len(names)
    stiffness = np.full(width, np.nan)
    coulomb = np.full(width, np.nan)
    bias = np.full(width, np.nan)
    residual = np.full(width, np.nan)
    used = np.zeros(width, dtype=int)
    unidentifiable: list[tuple[str, str]] = []

    for joint, name in enumerate(names):
        rising, falling = [], []
        for sweep in sweeps:
            applied = sweep.applied_torque[:, joint]
            errors = sweep.errors[:, joint]
            if float(np.ptp(applied)) <= 0.0:
                continue
            peak = int(np.argmax(applied))
            rising.extend(zip(applied[: peak + 1], errors[: peak + 1]))
            falling.extend(zip(applied[peak:], errors[peak:]))
        used[joint] = len(rising) + len(falling)
        if len(rising) < 2 or len(falling) < 2:
            unidentifiable.append(
                (name, f"{len(rising)} rising and {len(falling)} falling rounds; "
                       "each branch needs two")
            )
            continue

        fits = []
        for branch in (rising, falling):
            torques = np.array([value for value, _ in branch])
            deflections = np.array([value for _, value in branch])
            fits.append(np.polyfit(torques, deflections, 1))
        slopes = np.array([fit[0] for fit in fits])
        if np.any(slopes >= 0):
            unidentifiable.append(
                (name, "deflection grew with the torque opposing it, so the "
                       "joint did not respond the way a spring does")
            )
            continue
        mean_slope = float(np.mean(slopes))
        if abs(slopes[0] - slopes[1]) > BRANCH_SLOPE_TOLERANCE * abs(mean_slope):
            unidentifiable.append(
                (name, f"the two branches differ in slope by "
                       f"{abs(slopes[0] - slopes[1]) / abs(mean_slope):.0%}, over "
                       f"{BRANCH_SLOPE_TOLERANCE:.0%}: not one spring with dry friction")
            )
            continue

        stiffness[joint] = -1.0 / mean_slope
        intercepts = np.array([fit[1] for fit in fits])
        coulomb[joint] = abs(intercepts[0] - intercepts[1]) * stiffness[joint] / 2.0
        bias[joint] = float(np.mean(intercepts)) * stiffness[joint]
        predicted = np.concatenate([
            np.polyval(fit, np.array([value for value, _ in branch]))
            for fit, branch in zip(fits, (rising, falling))
        ])
        measured = np.concatenate([
            np.array([value for _, value in branch]) for branch in (rising, falling)
        ])
        residual[joint] = float(np.sqrt(np.mean((predicted - measured) ** 2)))

    return StaticEstimate(
        joint_names=names,
        stiffness=stiffness,
        torque_scale=np.full(width, np.nan),
        offset=np.full(width, np.nan),
        residual_rmse=residual,
        condition=np.full(width, np.nan),
        used=used,
        excluded=np.zeros(width, dtype=int),
        unidentifiable=tuple(unidentifiable),
        coulomb_nm=coulomb,
        bias_nm=bias,
    )
```

`torque_scale` is NaN here on purpose: alpha is a statement about the gravity
model, and this fit never consulted one.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_identification.py -k staircase -v`
Expected: 2 passed

- [ ] **Step 5: Run the whole suite**

Run: `python3 -m pytest -q`
Expected: 394 passed, 2 skipped

- [ ] **Step 6: Commit**

```bash
git add src/robot_control/identification.py tests/test_identification.py
git commit -F - <<'EOF'
feat: 계단 히스테리시스에서 kp, Fc, Fo를 적합한다

마른 마찰 아래의 정적 평형은 방정식이 아니라 부등식이다. 토크를 올렸다
내리면 평행한 두 직선이 생기고, 기울기가 -1/kp, 간격이 2Fc/kp다. 구간 탐색
없이 최소제곱 두 번으로 끝난다. 지금 frozen으로 버려지는 라운드가 여기서는
Fc를 나르는 신호다. torque_scale은 의도적으로 NaN — alpha는 중력 모델에 대한
진술인데 이 적합은 모델을 보지 않았다.
EOF
```

---

### Task 8: `tanh` and a bias in the dynamic model

**Files:**
- Modify: `src/robot_control/identification.py:600-625` (`_design`), `:786-812` (simulation), `DynamicEstimate`
- Test: `tests/test_identification.py`

**Interfaces:**
- Produces: `FRICTION_SHARPNESS = 100.0`; `DynamicEstimate.bias: np.ndarray`.

- [ ] **Step 1: Write the failing test**

```python
def test_the_friction_term_is_smooth_through_zero_velocity():
    """sign() is discontinuous exactly where a held pose lives, so it cannot
    represent the band the arm has. The static fit works around that by
    discarding rounds inside the band; the dynamic model should not need to."""
    from robot_control.identification import FRICTION_SHARPNESS, _design

    designs, _ = _design(TIME, COMMAND, MEASURED)

    column = designs[0][:, 2]
    assert np.all(np.abs(column) <= 1.0)
    assert np.any((np.abs(column) > 0.01) & (np.abs(column) < 0.99)), (
        "a sign column is only ever -1, 0 or 1; a tanh column has values "
        "between, and those are the samples near zero velocity"
    )


def test_the_dynamic_design_carries_a_bias_column():
    """Fo has nowhere to go without it, and at the wrist — where the real
    gravity torque is about 0.2 N.m — a bias of similar size dominates."""
    from robot_control.identification import _design

    designs, _ = _design(TIME, COMMAND, MEASURED)

    assert designs[0].shape[1] == 4
    assert np.allclose(designs[0][:, 3], -1.0)


def test_far_from_zero_velocity_tanh_matches_the_old_sign_model():
    from robot_control.identification import FRICTION_SHARPNESS, _design

    designs, _ = _design(TIME, COMMAND, FAST_MEASURED)

    velocity = np.diff(FAST_MEASURED, axis=0)[:-1, 0]
    assert np.allclose(designs[0][:, 2], -np.sign(velocity), atol=1e-3)
```

Define `TIME`, `COMMAND`, `MEASURED` and `FAST_MEASURED` at module scope: 200
uniformly sampled steps, `MEASURED` a slow sinusoid crossing zero velocity
(amplitude 0.01 rad, one period), `FAST_MEASURED` the same shape scaled so
|v| ≫ 1/`FRICTION_SHARPNESS` throughout (amplitude 5.0 rad).

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_identification.py -k "tanh or bias_column" -v`
Expected: FAIL — the column is `-sign(v)` and the design has three columns.

- [ ] **Step 3: Write minimal implementation**

```python
#: (rad/s)^-1. The friction term reaches 76% of Fc at 0.01 rad/s. A constant
#: rather than a fitted parameter because nothing in this pipeline observes it:
#: the static staircase holds the arm still, and identifying a transition
#: sharpness needs a low-speed velocity sweep that does not exist yet. Named so
#: that when someone measures it there is one place to put it.
FRICTION_SHARPNESS = 100.0
```

In `_design`:

```python
        columns = [
            error,
            -prior_velocity,
            -np.tanh(FRICTION_SHARPNESS * prior_velocity),
            -np.ones_like(error),
        ]
```

Add `bias` to `DynamicEstimate` and unpack it where the fit reads
`params[:3]`, which becomes `params[:4]`. In the simulation:

```python
        acceleration = (
            estimate.stiffness * (command[index] - position)
            - estimate.damping * velocity
            - estimate.friction * np.tanh(FRICTION_SHARPNESS * velocity)
            - estimate.bias
        )
```

Every place that scales a dynamic parameter by inertia must scale `bias` too —
`friction=dynamic.friction * inertia` gains `bias=dynamic.bias * inertia`.

- [ ] **Step 4: Run the whole suite**

Run: `python3 -m pytest -q`
Expected: some combined-fit tests fail on the new column count. Fix each by
adding the bias term to its expectation; do not weaken an assertion to pass.

- [ ] **Step 5: Run the whole suite again**

Run: `python3 -m pytest -q`
Expected: 397 passed, 2 skipped

- [ ] **Step 6: Commit**

```bash
git add src/robot_control/identification.py tests/
git commit -F - <<'EOF'
fix: 동적 마찰을 tanh로, 그리고 바이어스 항을 더한다

sign()은 dq=0에서 불연속인데 정적으로 잡힌 자세가 사는 곳이 정확히 거기다.
지금은 마찰대에 걸린 라운드를 버려서 그 문제를 피하고 있었다. tanh는 0을
매끄럽게 통과해 마찰대 자체를 표현한다. Fo 열도 더한다 — 손목의 실제 중력
토크가 0.2 N.m 수준이라 비슷한 크기의 바이어스가 지배한다.

OpenArm bilateral control 문서의 tau_f = Fc*tanh(k*dq) + Fv*dq + Fo와 형태가
맞는다. k는 상수 — 정적 계단은 팔을 세워두고 재므로 전이 날카로움을 관측하지
못한다.
EOF
```

---

### Task 9: Carry Fc and Fo through bundle, validate and export

**Files:**
- Modify: `src/robot_control/cli.py` (`_fit`, `_bundle`, `_validate`), `src/robot_control/hdgp_export.py:85-95`
- Test: `tests/test_hdgp_export.py`, `tests/test_cli_bundle.py`

**Interfaces:**
- Consumes: `StaticEstimate.coulomb_nm`, `.bias_nm` (Task 7);
  `DynamicEstimate.bias` (Task 8).

- [ ] **Step 1: Write the failing test**

```python
def test_the_static_coulomb_measurement_is_what_export_writes():
    """Two measurements of one quantity. The staircase measures it directly —
    a known torque, at rest, at the moment the joint breaks free — and the
    dynamic fit infers it as a regression coefficient over a moving track."""
    identified = _identified(
        coulomb_nm=np.full(7, 0.08), friction=np.full(7, 0.31)
    )

    body, audit = hdgp_group_payload(identified, "openarm_right_arm", ...)

    assert body["friction"] == pytest.approx(0.08)
    assert audit["dynamic_friction_nm"] == pytest.approx([0.31] * 7)


def test_a_joint_no_staircase_covered_falls_back_to_the_dynamic_coefficient():
    identified = _identified(
        coulomb_nm=np.full(7, np.nan), friction=np.full(7, 0.31)
    )

    body, _ = hdgp_group_payload(identified, "openarm_right_arm", ...)

    assert body["friction"] == pytest.approx(0.31)


def test_validate_reports_both_friction_measurements(...):
    """A large disagreement is evidence about the model rather than about the
    arm, and preferring one silently would hide it."""
    ...
    assert "coulomb" in report and "dynamic_friction" in report
```

Fill the `...` from the existing fixtures in each test module.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_hdgp_export.py -k friction -v`
Expected: FAIL — `hdgp_group_payload` has no notion of a static Coulomb value.

- [ ] **Step 3: Write minimal implementation**

In `_fit`, carry `static.coulomb_nm` and `static.bias_nm` into the combined
estimate and into the JSON it writes, beside `torque_scale`.

In `hdgp_export.py`, prefer the static measurement where it exists:

```python
    # Two measurements of one quantity, and they are not equally good. The
    # staircase reads it directly at rest under a torque we chose; the dynamic
    # coefficient is a regression over a track where it competes with damping.
    coulomb = np.where(
        np.isnan(identified.coulomb_nm), identified.friction, identified.coulomb_nm
    )
```

and use `coulomb` where `identified.friction` fed `gains["friction"]`, keeping
the dynamic value in the audit block as `dynamic_friction_nm`. Add
`bias_nm` to the audit block beside `inertia_kg_m2`, with a comment recording
that hdgp has no field for it.

- [ ] **Step 4: Run the whole suite**

Run: `python3 -m pytest -q`
Expected: 400 passed, 2 skipped

- [ ] **Step 5: Commit**

```bash
git add src/robot_control/cli.py src/robot_control/hdgp_export.py tests/
git commit -F - <<'EOF'
feat: 정적 Fc가 기록의 근거, 동적 계수는 감사 블록으로

같은 쿨롱 마찰을 두 번 재게 된다. 계단은 정지 상태에서 우리가 고른 토크로
직접 재고, 동적 계수는 감쇠와 경쟁하는 회귀 계수다. 계단이 덮은 관절은
정적 값을, 아닌 관절은 동적 값을 쓴다. validate가 둘을 나란히 보고한다.

Fo는 hdgp에 대응 필드가 없어 감사 블록으로 간다(inertia와 같은 처지).
EOF
```

---

### Task 10: Hardware validation on the right arm

**Files:** none — this is a run, and its artifacts.

**Interfaces:** consumes everything above.

- [ ] **Step 1: Deploy**

```bash
git push origin humble
ssh usr@100.106.38.98 'cd ~/rl_ws/robot_control && git pull -q && python3 -m pytest -q | tail -2'
```
Expected: 400 passed, 2 skipped

- [ ] **Step 2: Dry run, one joint, on the arm**

Bring up with `use_fake_hardware:=false`, then `robotctl pose ready --execute`,
then:

```bash
robotctl pose torque --group openarm_right_arm --joint r_aj_5 --steps 3
```
Expected: prints the plan and "DRY RUN: nothing published". The arm does not move.

- [ ] **Step 3: One joint, executed**

```bash
./ros_ws/load_effort_controllers.sh right
robotctl pose torque --group openarm_right_arm --joint r_aj_5 --steps 3 \
    --execute --output ~/r2s/sweeps/torque0.json
```
Expected: the probe converges, five rounds run, "torque released" prints, and
the arm ends where it started. Watch the joint: it should rock a few degrees
either side, not lunge.

- [ ] **Step 4: All joints, three poses**

Repeat step 3 with no `--joint`, at three poses reached with `pose joints`,
writing `torque0.json`, `torque1.json`, `torque2.json`. Unload the effort
controllers afterwards.

- [ ] **Step 5: Fit everything together**

```bash
robotctl r2s identify \
    $(for i in 0 1 2 3 4 5 6 7 8; do printf -- "--sweep ~/r2s/sweeps/pose$i.json "; done) \
    $(for i in 0 1 2; do printf -- "--sweep ~/r2s/sweeps/torque$i.json "; done) \
    --output ~/r2s/static_right.json
```
Expected: all seven joints identified; `r_aj_5`'s stiffness near the 15.5 the
hand calculation gave; Fc and Fo reported for every joint the staircase covered.

- [ ] **Step 6: Record the result**

Append the measured table to the Notion page under DRL → Robot_Control →
control 실험, section 4, replacing the 2026-07-28 table with the new one and
noting which joints the staircase covered.

---

## Self-Review

**Spec coverage:** A1 → Tasks 4, 5, 6. A2 → Tasks 1, 2. A3 → Task 3. B1 →
Task 7. B2 → Task 8. B3 → Task 9. Testing section → tests inside each task plus
Task 10. The spec's "two measurements of the same quantity" resolution → Task 9.

**Known gap, deliberate:** the spec's risk about the phantom-gripper URDF has no
task. It is out of scope by the spec's own statement and needs a launch change
that cannot be made per-arm; it stays recorded in the spec and in Notion §8.

**Type consistency:** `applied_torque` (Tasks 1, 2, 3, 6, 7), `coulomb_nm` /
`bias_nm` (Tasks 7, 9), `FRICTION_SHARPNESS` (Task 8), `probe_torque` /
`staircase` / `ExcitationRefused` / `measure_staircase` (Tasks 4, 5, 6) are
spelled identically at every use.
