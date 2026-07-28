from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


class FitError(ValueError):
    pass


#: One count of a 14-bit absolute encoder over a full turn is 2*pi/16384, about
#: 3.8e-4 rad. Below that a joint has not measurably moved, whatever the torque
#: did, so the pair of samples says nothing about its stiffness.
DEFAULT_NOISE_RAD = 4e-4

#: Two samples count as distinct torque levels when they differ by this fraction
#: of the sweep's own torque span. Scale-free on purpose: the absolute torques
#: differ by an order of magnitude between a shoulder and a wrist.
DISTINCT_TORQUE_FRACTION = 0.05

#: Condition number of a joint's normalised regression above which its
#: parameters are not considered separated. A single pose sits above 1e5 however
#: many scales it holds, because the modelled torque is then a constant column
#: and cannot be told from the offset; two poses whose modelled torque differs
#: by 10% sit near 50.
MAX_CONDITION = 200.0

#: How far the two routes to a joint's inertia may differ before the pair is
#: refused. kp/k and 1/g come from different columns of different experiments, so
#: a gap this size means one of them is measuring something else.
MAX_INERTIA_DISAGREEMENT = 0.25


@dataclass(frozen=True)
class SecondOrderEstimate:
    """The dynamic fit, with every parameter divided by an inertia.

    ``qdd = k (q_cmd - q) - d qd - f sign(qd) - g tau_g(q)`` gives
    ``k = kp/J``, ``d = b/J``, ``f = tau_f/J`` and ``g = 1/J``. Only the last
    carries the inertia on its own, and only when a gravity column was supplied.
    """

    stiffness: np.ndarray
    damping: np.ndarray
    friction: np.ndarray
    residual_rmse: np.ndarray
    #: 1/J, or None when the fit ran without a gravity column.
    inverse_inertia: np.ndarray | None = None


@dataclass(frozen=True)
class CombinedEstimate:
    """What the two experiments say together, in physical units.

    ``inertia`` comes from ``kp/k``: the static fit's stiffness, which has no
    inertia in it, over the dynamic fit's, which is that same stiffness divided
    by one. ``inertia_from_gravity`` is the dynamic fit's own ``1/g``, from a
    different column of a different experiment, so the two agreeing is evidence
    rather than arithmetic.
    """

    joint_names: tuple[str, ...]
    #: J, kg m^2.
    inertia: np.ndarray
    #: b, N.m.s/rad.
    damping: np.ndarray
    #: tau_f, N.m.
    friction: np.ndarray
    #: kp, N.m/rad, carried through from the static fit.
    stiffness: np.ndarray
    inertia_from_gravity: np.ndarray
    #: Relative gap between the two inertias, or nan with no gravity column.
    disagreement: np.ndarray


@dataclass(frozen=True)
class ValidationResult:
    status: str
    failures: tuple[str, ...]


@dataclass(frozen=True)
class GravitySweep:
    """One arm pose held at several gravity feedforward scales, and what it did.

    At each round a known torque is published and the joint's standing error is
    read, which is the observation a stiffness estimate is made of:

    ``kp (q_cmd - q) = tau_g(q) - s tau_model(q)``

    Every round carries its own pose and its own modelled torque rather than one
    for the whole sweep. Compensation moves the arm, so the load a joint holds
    at scale 1.0 is not the load it was holding at 0.0, and pairing an error
    with the torque from a different pose is the one way to get a confident
    wrong number out of the fit.
    """

    group: str
    joint_names: tuple[str, ...]
    #: (rounds, joints) canonical joint values measured before each round's read.
    poses: np.ndarray
    #: (rounds, joints) modelled gravity torque at that round's pose, unscaled.
    modelled_torque: np.ndarray
    #: (rounds, joints) the fraction of it actually published.
    scales: np.ndarray
    #: (rounds, joints) the torque actually published, in N.m. Not derivable
    #: from scales once an excitation publishes a torque of its own choosing
    #: rather than a multiple of the model.
    applied_torque: np.ndarray
    #: (rounds, joints) the controller's own tracking error after the hold.
    errors: np.ndarray
    #: The joint whose scale varied, when the sweep varied only one.
    sweep_joint: str | None = None

    _GRIDS = ("poses", "modelled_torque", "scales", "applied_torque", "errors")

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.joint_names)
        if not names:
            raise FitError("a sweep needs at least one joint")
        object.__setattr__(self, "joint_names", names)
        for field_name in self._GRIDS:
            grid = np.asarray(getattr(self, field_name), dtype=float)
            if grid.ndim != 2 or grid.shape[1] != len(names):
                raise FitError(
                    f"{field_name} must be (rounds, {len(names)}), got {grid.shape}"
                )
            if not np.isfinite(grid).all():
                raise FitError(f"{field_name} carries a non-finite value")
            object.__setattr__(self, field_name, grid)
        counts = {getattr(self, name).shape[0] for name in self._GRIDS}
        if len(counts) != 1:
            raise FitError(
                "every round needs a pose, a modelled torque, a scale, an "
                f"applied torque and an error, but the counts differ: {sorted(counts)}"
            )
        if self.rounds == 0:
            raise FitError("a sweep needs at least one round")
        if self.sweep_joint is not None and self.sweep_joint not in names:
            raise FitError(
                f"sweep_joint {self.sweep_joint!r} is not one of {list(names)}"
            )

    @property
    def rounds(self) -> int:
        return int(self.poses.shape[0])


@dataclass(frozen=True)
class StaticEstimate:
    """Per joint, what the sweeps say about how it holds a load.

    ``stiffness`` is in N.m/rad and carries no inertia, which is what makes it
    worth measuring: the dynamic fit returns every parameter divided by an
    inertia it cannot separate, and this is the second equation that separates
    it.

    A joint the sweeps could not identify is ``nan`` here and named in
    ``unidentifiable`` with the reason. Least squares always returns something;
    a number for a joint whose load never varied would be that something.
    """

    joint_names: tuple[str, ...]
    #: kp, N.m/rad.
    stiffness: np.ndarray
    #: alpha: the factor the modelled gravity torque was wrong by. ``nan`` for
    #: a joint whose model was zero at every used round even when stiffness
    #: was still identified from the applied torque: alpha multiplies that
    #: zero model, so no value of it is distinguishable from any other.
    torque_scale: np.ndarray
    #: c, rad: the scale-independent standing error, which is where stiction is.
    offset: np.ndarray
    residual_rmse: np.ndarray
    #: Conditioning of each joint's regression, so a marginal fit is visible.
    condition: np.ndarray
    #: Rounds that entered each joint's fit, and rounds dropped as frozen.
    used: np.ndarray
    excluded: np.ndarray
    unidentifiable: tuple[tuple[str, str], ...]
    #: Fc, N.m: the torque the joint holds without any position error at all,
    #: measured as the gap between the staircase's two branches. NaN where only
    #: gravity sweeps covered the joint.
    coulomb_nm: np.ndarray | None = None
    #: Fo + tau_gravity, N.m: where the staircase's midline crosses zero
    #: torque. NaN where only gravity sweeps covered the joint.
    bias_nm: np.ndarray | None = None


@dataclass(frozen=True)
class PoseSet:
    """Poses to sweep at, and how well they condition each joint's fit."""

    poses: np.ndarray
    scales: tuple[float, ...]
    condition: np.ndarray

    @property
    def worst_joint(self) -> int:
        return int(np.argmax(self.condition))

    @property
    def worst_condition(self) -> float:
        return float(np.max(self.condition))


def _static_design(torques: np.ndarray, scales: Sequence[float]) -> np.ndarray:
    """Rows of the static regression for one joint, over poses and scales."""
    return np.array(
        [(torque, -scale * torque, 1.0) for torque in torques for scale in scales],
        dtype=float,
    )


def _condition(design: np.ndarray) -> float:
    """Conditioning of a regression with its columns scaled to comparable size.

    Without the normalisation the number would mostly report that a torque in
    N.m and a column of ones are different sizes, which says nothing about
    whether the parameters can be separated.
    """
    norms = np.linalg.norm(design, axis=0)
    if not np.all(norms > 0):
        return float("inf")
    return float(np.linalg.cond(design / norms))


def _set_condition(torques: np.ndarray, scales: Sequence[float]) -> np.ndarray:
    """Per joint, how well a candidate set of poses separates its parameters."""
    torques = np.atleast_2d(torques)
    return np.array(
        [
            _condition(_static_design(torques[:, joint], scales))
            for joint in range(torques.shape[1])
        ]
    )


def design_pose_set(
    torque_at,
    lower: np.ndarray,
    upper: np.ndarray,
    *,
    scales: Sequence[float],
    poses: int = 4,
    candidates: int = 256,
    seed: int = 0,
    reach: float = 0.5,
) -> PoseSet:
    """Choose poses that let each joint's stiffness be told from its torque model.

    Greedy: start at the middle of the envelope, then repeatedly add whichever
    candidate leaves the worst-conditioned joint best off. Greedy rather than a
    search because the payoff is nearly all in the second pose — one pose is one
    equation in three unknowns and later ones only add redundancy — so the
    achieved number is reported to the caller to judge rather than promised.

    *reach* bounds sampling to a fraction of each joint's range about its middle.
    A pose against a hard stop cannot droop, and a joint that cannot droop looks
    exactly like one held by stiction.

    Deterministic in *seed*, which is what makes a dry run a review: `--execute`
    with the same seed visits the poses that were printed. Nothing here checks
    for self-collision — the profile bounds each joint, not the arm against
    itself — so the printed itinerary is the review, not a formality.
    """
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    scales = tuple(float(scale) for scale in scales)
    if lower.shape != upper.shape or lower.ndim != 1 or not lower.size:
        raise FitError("limits must be one matching value per joint")
    if np.any(upper <= lower):
        raise FitError("every joint's upper limit must exceed its lower limit")
    if poses < 1:
        raise FitError("a pose set needs at least one pose")
    if len(set(scales)) < 2:
        raise FitError(
            "a sweep needs at least two distinct scales, or the joint is never "
            "seen responding to a change in torque"
        )
    if not 0.0 < reach <= 1.0:
        raise FitError("reach must be a fraction of the range, above 0 and at most 1")
    if candidates < 1:
        raise FitError("candidates must be positive")

    middle = (lower + upper) / 2
    half = (upper - lower) / 2 * reach
    rng = np.random.default_rng(seed)
    pool = middle + rng.uniform(-1.0, 1.0, (candidates, lower.size)) * half
    pool_torque = np.array([np.asarray(torque_at(pose), dtype=float) for pose in pool])

    chosen = [middle.copy()]
    torques = [np.asarray(torque_at(middle), dtype=float)]
    while len(chosen) < poses:
        scores = [
            np.max(_set_condition(np.vstack([*torques, candidate]), scales))
            for candidate in pool_torque
        ]
        best = int(np.argmin(scores))
        chosen.append(pool[best].copy())
        torques.append(pool_torque[best].copy())

    return PoseSet(
        poses=np.array(chosen),
        scales=scales,
        condition=_set_condition(np.array(torques), scales),
    )


def _frozen_rounds(sweep: GravitySweep, joint: int, noise_rad: float) -> set[int]:
    """Rounds where the applied torque changed and the joint did not move.

    Inside its stiction band a joint is held by friction rather than by the
    position error, so its standing error stops responding to torque. Those
    samples do not merely add noise, they pull the fitted stiffness towards
    infinity, and they have to go. Both members of such a pair are dropped: the
    first one is where the joint stopped, and nothing says whether that was its
    equilibrium or the edge of the band.
    """
    applied = sweep.applied_torque[:, joint]
    span = float(np.max(applied) - np.min(applied))
    distinct = span * DISTINCT_TORQUE_FRACTION
    order = np.argsort(applied)
    frozen: set[int] = set()
    for first, second in zip(order, order[1:]):
        commanded = abs(float(applied[second] - applied[first]))
        moved = abs(float(sweep.errors[second, joint] - sweep.errors[first, joint]))
        if commanded > distinct and moved <= noise_rad:
            frozen.update((int(first), int(second)))
    return frozen


def fit_static_gravity(
    sweeps: Sequence[GravitySweep],
    *,
    noise_rad: float = DEFAULT_NOISE_RAD,
    max_condition: float = MAX_CONDITION,
) -> StaticEstimate:
    """Fit stiffness, a torque-model correction and an offset from held poses.

    At equilibrium the impedance controller balances the load with a standing
    position error, so with a fraction *s* of the modelled torque fed forward:

        kp (q_cmd - q) = alpha tau_model - s tau_model

    which, with an offset for what friction holds without any error at all, is

        error = (alpha - s) tau_model / kp + c

    Linear in ``alpha/kp``, ``1/kp`` and ``c``, so it is least squares — but
    only once ``tau_model`` itself varies. At a single pose it is a constant
    column, indistinguishable from the offset, and the fit is under-determined
    however many scales the sweep held. Varying it is what several poses buys.
    """
    sweeps = list(sweeps)
    if not sweeps:
        raise FitError("a static fit needs at least one gravity sweep")
    names = sweeps[0].joint_names
    group = sweeps[0].group
    for sweep in sweeps[1:]:
        # Group first: it is the identity the operator chose, and a group
        # mismatch always drags a joint mismatch along behind it.
        if sweep.group != group:
            raise FitError(
                f"sweeps cover different groups: {group!r} and {sweep.group!r}"
            )
        if sweep.joint_names != names:
            raise FitError(
                f"sweeps cover different joints: {list(names)} against "
                f"{list(sweep.joint_names)}"
            )

    width = len(names)
    stiffness = np.full(width, np.nan)
    torque_scale = np.full(width, np.nan)
    offset = np.full(width, np.nan)
    residual = np.full(width, np.nan)
    condition = np.full(width, np.inf)
    used = np.zeros(width, dtype=int)
    excluded = np.zeros(width, dtype=int)
    unidentifiable: list[tuple[str, str]] = []

    for joint, name in enumerate(names):
        rows: list[tuple[float, float, float]] = []
        targets: list[float] = []
        for sweep in sweeps:
            frozen = _frozen_rounds(sweep, joint, noise_rad)
            excluded[joint] += len(frozen)
            for index in range(sweep.rounds):
                if index in frozen:
                    continue
                torque = float(sweep.modelled_torque[index, joint])
                published = float(sweep.applied_torque[index, joint])
                rows.append((torque, -published, 1.0))
                targets.append(float(sweep.errors[index, joint]))
        used[joint] = len(rows)

        if len(rows) < 3:
            unidentifiable.append(
                (
                    name,
                    f"{len(rows)} usable rounds against three parameters"
                    + (f", {excluded[joint]} dropped as frozen" if excluded[joint] else ""),
                )
            )
            continue
        design = np.asarray(rows, dtype=float)
        # A joint the model says carries no gravity torque at any used round
        # has a by_stiffness column of exact zeros: alpha is not merely hard
        # to pin down, it multiplies nothing, so every value fits the data
        # equally well and none of them is "the" answer. Left in, that column
        # turns a fittable two-parameter design into one `_condition` can only
        # ever report as an exact singularity. Fit what the applied torque
        # actually resolves — stiffness and offset — and leave the
        # torque-model correction at its unidentified default, `nan`, same as
        # every other parameter this function cannot pin down.
        #
        # This only catches the column being bit-exact zero. A model that is
        # merely small — the realistic case for, say, a wrist near-aligned
        # with gravity — still takes the three-column path below and lives or
        # dies by `max_condition`, unchanged from before this function read
        # applied_torque directly.
        has_model = bool(np.any(design[:, 0]))
        fit_design = design if has_model else design[:, 1:]
        condition[joint] = _condition(fit_design)
        if not np.isfinite(condition[joint]):
            unidentifiable.append(
                (name, "no torque was ever fed forward to this joint")
            )
            continue
        if condition[joint] > max_condition:
            unidentifiable.append(
                (
                    name,
                    f"condition {condition[joint]:.3g} over {max_condition:g}: "
                    "the poses do not vary this joint's load enough to tell its "
                    "stiffness from its torque model",
                )
            )
            continue

        target = np.asarray(targets, dtype=float)
        params, _, _, _ = np.linalg.lstsq(fit_design, target, rcond=None)
        if has_model:
            by_stiffness, inverse_stiffness, constant = (float(value) for value in params)
        else:
            by_stiffness = float("nan")
            inverse_stiffness, constant = (float(value) for value in params)
        if inverse_stiffness <= 0:
            unidentifiable.append(
                (
                    name,
                    f"fitted stiffness {1.0 / inverse_stiffness:.3g} N.m/rad is "
                    "not positive, so the joint did not respond to torque the "
                    "way a spring does",
                )
            )
            continue
        stiffness[joint] = 1.0 / inverse_stiffness
        torque_scale[joint] = by_stiffness / inverse_stiffness
        offset[joint] = constant
        residual[joint] = float(np.sqrt(np.mean((fit_design @ params - target) ** 2)))

    return StaticEstimate(
        joint_names=names,
        stiffness=stiffness,
        torque_scale=torque_scale,
        offset=offset,
        residual_rmse=residual,
        condition=condition,
        used=used,
        excluded=excluded,
        unidentifiable=tuple(unidentifiable),
    )


#: How far a staircase's two branch slopes may differ, as a fraction of their
#: mean, before the pair is refused rather than averaged. Two branches from one
#: spring-with-friction joint share a slope; a bigger gap means something else
#: moved between them.
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
    group = sweeps[0].group
    for sweep in sweeps[1:]:
        # Group first: it is the identity the operator chose, and a group
        # mismatch always drags a joint mismatch along behind it.
        if sweep.group != group:
            raise FitError(
                f"sweeps cover different groups: {group!r} and {sweep.group!r}"
            )
        if sweep.joint_names != names:
            raise FitError(
                f"sweeps cover different joints: {list(names)} against "
                f"{list(sweep.joint_names)}"
            )

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
            # The peak round belongs to the rising branch only: its deflection
            # was reached travelling upward, so putting it in the falling
            # branch too would put a point a full 2 Fc/kp off that branch's
            # line and drag the fit.
            rising.extend(zip(applied[: peak + 1], errors[: peak + 1]))
            falling.extend(zip(applied[peak + 1 :], errors[peak + 1 :]))
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
        # alpha is a statement about the gravity model; this fit never
        # consulted one, so nothing distinguishes one value of it from another.
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


MULTISINE_FREQUENCIES = (0.7, 1.3, 2.1, 3.7)

#: Peak slew of the unit multisine, in rad/s per rad of amplitude. Every
#: component is at its steepest when its sine crosses zero, and they can line up,
#: so this is the bound the design has to be checked against.
MULTISINE_SLEW = 2 * np.pi * sum(MULTISINE_FREQUENCIES) / len(MULTISINE_FREQUENCIES)


def _bridge(start: np.ndarray, target: np.ndarray, max_step: np.ndarray) -> np.ndarray:
    """Samples slewing from *start* towards *target*, none longer than max_step.

    The target itself is not included: the phase that follows provides it.
    """
    travel = target - start
    reach = np.abs(travel) / max_step
    needed = int(np.ceil(float(np.max(reach)))) if np.any(reach > 0) else 0
    if needed <= 1:
        return np.zeros((0, len(start)))
    alpha = (np.arange(1, needed) / needed)[:, None]
    return start + alpha * travel


def build_excitation(
    neutral: np.ndarray,
    amplitude: np.ndarray,
    rate_hz: float,
    max_step: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a deterministic step/ramp/hold/multisine identification track.

    The phases are shapes, and the joins between them are discontinuities. That
    is harmless on paper and not harmless as a stream of position commands: on
    the real arm at 100 Hz, the join into the multisine asks for seven times the
    profile's velocity limit, so the gate refuses the whole track.

    *max_step* is how far each joint may move per sample. Given it, the joins are
    bridged at that rate rather than jumped. The alternative — shrinking the
    amplitude until every discontinuity fits in one sample — would shrink it by
    the same factor of seven, which is most of the excitation; bridging keeps the
    amplitude and costs a few samples.

    A multisine too fast to slew is refused rather than bridged. Its peak slew is
    its frequencies times its amplitude, and no amount of extra time changes
    either.
    """
    neutral = np.asarray(neutral, dtype=float)
    amplitude = np.asarray(amplitude, dtype=float)
    if neutral.shape != amplitude.shape or neutral.ndim != 1 or rate_hz <= 0:
        raise FitError("invalid excitation arguments")
    if max_step is not None:
        max_step = np.asarray(max_step, dtype=float)
        if max_step.shape != neutral.shape or np.any(max_step <= 0):
            raise FitError(
                "max_step must be one positive value per joint, "
                f"{len(neutral)} of them"
            )
        peak = MULTISINE_SLEW * amplitude / rate_hz
        over = peak > max_step + 1e-12
        if np.any(over):
            worst = int(np.argmax(peak / max_step))
            fits = float(np.min(max_step / peak))
            raise FitError(
                f"the multisine cannot be slewed: joint {worst} would move "
                f"{peak[worst]:.4g} rad per sample against a budget of "
                f"{max_step[worst]:.4g}. Extra time does not help — the peak "
                "slew is frequency times amplitude. Scale the amplitude by "
                f"{fits:.3g} or less."
            )

    definitions = (
        ("hold", 0.5),
        ("step", 0.5),
        ("hold", 0.5),
        ("ramp", 1.0),
        ("multisine", 3.0),
        ("hold", 0.5),
    )
    blocks: list[np.ndarray] = []
    labels: list[str] = []
    phase_start = neutral.copy()
    elapsed = 0.0
    for name, duration in definitions:
        count = max(2, int(round(duration * rate_hz)))
        local = np.arange(count) / rate_hz
        if name == "step":
            values = np.tile(neutral + amplitude, (count, 1))
        elif name == "ramp":
            alpha = np.linspace(1.0, -1.0, count)[:, None]
            values = neutral + alpha * amplitude
        elif name == "multisine":
            wave = sum(
                np.sin(2 * np.pi * f * (elapsed + local))
                for f in MULTISINE_FREQUENCIES
            )
            wave /= len(MULTISINE_FREQUENCIES)
            values = neutral + wave[:, None] * amplitude
        else:
            values = np.tile(phase_start, (count, 1))
        if max_step is not None:
            bridge = _bridge(phase_start, values[0], max_step)
            if len(bridge):
                blocks.append(bridge)
                labels.extend(["bridge"] * len(bridge))
        phase_start = values[-1]
        blocks.append(values)
        labels.extend([name] * count)
        elapsed += duration
    command = np.vstack(blocks)
    time = np.arange(len(command), dtype=float) / rate_hz
    return time, command, np.asarray(labels)


def split_repetitions(run_ids: Sequence[str]) -> tuple[tuple[str, ...], tuple[str, ...]]:
    if len(run_ids) != 3 or len(set(run_ids)) != 3:
        raise FitError("identification requires exactly three unique repetitions")
    return tuple(run_ids[:2]), (run_ids[2],)


def fit_second_order(
    time_sec: np.ndarray,
    command: np.ndarray,
    measured: np.ndarray,
    *,
    gravity_torque: np.ndarray | None = None,
) -> SecondOrderEstimate:
    """Fit ``qdd = k(q_cmd-q) - d qd - f sign(qd) - g tau_g(q)`` per joint.

    *gravity_torque* is the load acting against each joint at each sample, in
    N.m, already corrected by the ``alpha`` a static fit measured. Supplying it
    does two things: it stops the standing load being absorbed into ``k``, which
    is the only column the regression could otherwise put it in, and its
    coefficient is ``1/J``, which is the inertia on its own.

    Omitting it keeps the older three-parameter fit, which is right for a track
    with no standing load in it — a horizontal joint, or synthetic data.
    """
    return fit_second_order_runs(
        [(time_sec, command, measured, gravity_torque)]
    )


def _second_order_rows(run) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Design rows and targets, one pair per joint, for one uniform run."""
    time_sec, command, measured, gravity_torque = run
    time_sec = np.asarray(time_sec, dtype=float)
    command = np.asarray(command, dtype=float)
    measured = np.asarray(measured, dtype=float)
    if (
        time_sec.ndim != 1
        or command.ndim != 2
        or measured.shape != command.shape
        or len(time_sec) != len(command)
        or len(time_sec) < 5
        or np.any(np.diff(time_sec) <= 0)
    ):
        raise FitError("invalid fit track")
    if gravity_torque is not None:
        gravity_torque = np.asarray(gravity_torque, dtype=float)
        if gravity_torque.shape != command.shape or not np.isfinite(gravity_torque).all():
            raise FitError(
                "gravity torque must carry one finite value per sample and joint: "
                f"expected {command.shape}, got {gravity_torque.shape}"
            )
    dt = np.diff(time_sec)
    if np.max(dt) - np.min(dt) > np.mean(dt) * 1e-4:
        raise FitError("fit track must be uniformly sampled")
    step = float(np.mean(dt))
    velocity = np.diff(measured, axis=0) / step
    acceleration = np.diff(velocity, axis=0) / step

    designs, targets = [], []
    for joint in range(command.shape[1]):
        error = command[1:-1, joint] - measured[1:-1, joint]
        prior_velocity = velocity[:-1, joint]
        columns = [error, -prior_velocity, -np.sign(prior_velocity)]
        if gravity_torque is not None:
            # Aligned with `error`, which reads the same window of samples.
            columns.append(-gravity_torque[1:-1, joint])
        designs.append(np.column_stack(columns))
        targets.append(acceleration[:, joint])
    return designs, targets


def fit_second_order_runs(runs) -> SecondOrderEstimate:
    """Fit one model across several runs of the same excitation.

    Stacked as rows of one regression rather than fitted separately and
    averaged: the runs are repeats of one experiment, so the parameters are
    shared and every sample is evidence about the same numbers. Averaging
    per-run fits would instead weight a run that happened to be short as
    heavily as a long one.

    The runs are not concatenated into a single track, because the joins
    between them are not motion — the arm was driven back to the start in
    between — and differentiating across a join would invent an acceleration
    that never happened.
    """
    runs = list(runs)
    if not runs:
        raise FitError("a fit needs at least one run")
    per_run = [_second_order_rows(run) for run in runs]
    widths = {len(designs) for designs, _targets in per_run}
    if len(widths) != 1:
        raise FitError(f"the runs cover different joint counts: {sorted(widths)}")
    columns = {designs[0].shape[1] for designs, _targets in per_run}
    if len(columns) != 1:
        raise FitError(
            "some runs carry a gravity column and some do not, so they cannot "
            "be fitted together"
        )

    width = widths.pop()
    stiffness = np.empty(width)
    damping = np.empty(width)
    friction = np.empty(width)
    residual = np.empty(width)
    with_gravity = columns.pop() == 4
    inverse_inertia = np.empty(width) if with_gravity else None
    for joint in range(width):
        design = np.vstack([designs[joint] for designs, _targets in per_run])
        target = np.concatenate([targets[joint] for _designs, targets in per_run])
        params, _, rank, _ = np.linalg.lstsq(design, target, rcond=None)
        if rank < 2 or params[0] <= 0 or params[1] < 0:
            raise FitError(f"unidentifiable dynamics for joint {joint}")
        if with_gravity:
            if params[3] <= 0:
                raise FitError(
                    f"joint {joint} accelerates towards its load rather than away "
                    "from it, so the gravity column has the wrong sign or the "
                    "wrong chain"
                )
            inverse_inertia[joint] = params[3]
        prediction = design @ params
        stiffness[joint], damping[joint], friction[joint] = params[:3]
        residual[joint] = float(np.sqrt(np.mean((prediction - target) ** 2)))
    return SecondOrderEstimate(
        stiffness, damping, friction, residual, inverse_inertia
    )


def combine(
    static: StaticEstimate,
    dynamic: SecondOrderEstimate,
    joint_names: Sequence[str],
) -> CombinedEstimate:
    """Put the two fits together, and get the inertia the simulator needs.

    ``fit_second_order`` returns ``k = kp/J``, and the static fit returns ``kp``
    with no inertia in it, so ``J = kp/k`` and with it ``b = d J`` and
    ``tau_f = f J``. Neither experiment gives that alone: the dynamic track
    cannot separate inertia from stiffness, and a held pose has no inertia in it
    to separate.
    """
    names = tuple(str(name) for name in joint_names)
    if static.joint_names != names or dynamic.stiffness.shape != (len(names),):
        raise FitError(
            f"the two fits cover different joints: static {list(static.joint_names)}, "
            f"dynamic {dynamic.stiffness.shape[0]} of them, asked for {list(names)}"
        )
    if static.unidentifiable:
        raise FitError(
            "the static fit left joints unidentified, so their stiffness cannot "
            "carry an inertia: "
            + ", ".join(name for name, _reason in static.unidentifiable)
        )
    if not np.all(dynamic.stiffness > 0) or not np.all(static.stiffness > 0):
        raise FitError("both fits must give a positive stiffness for every joint")

    inertia = static.stiffness / dynamic.stiffness
    if dynamic.inverse_inertia is None:
        from_gravity = np.full(len(names), np.nan)
    else:
        from_gravity = 1.0 / dynamic.inverse_inertia
    return CombinedEstimate(
        joint_names=names,
        inertia=inertia,
        damping=dynamic.damping * inertia,
        friction=dynamic.friction * inertia,
        stiffness=static.stiffness.copy(),
        inertia_from_gravity=from_gravity,
        disagreement=np.abs(from_gravity - inertia) / inertia,
    )


@dataclass(frozen=True)
class HoldoutMetrics:
    """How a fitted model did on a run it was not fitted on."""

    #: Open-loop prediction error against the measured positions.
    rmse_rad: float
    #: Error of the model-free assumption that the arm reached its command.
    baseline_rmse_rad: float
    #: Lag the model did not account for, found by shifting the prediction.
    delay_residual_sec: float
    #: How much of the baseline error the model removed.
    improvement_fraction: float


def _fit_step(time_sec: np.ndarray) -> float:
    time_sec = np.asarray(time_sec, dtype=float)
    if time_sec.ndim != 1 or len(time_sec) < 5 or np.any(np.diff(time_sec) <= 0):
        raise FitError("invalid holdout track")
    dt = np.diff(time_sec)
    if np.max(dt) - np.min(dt) > np.mean(dt) * 1e-4:
        raise FitError("holdout track must be uniformly sampled")
    return float(np.mean(dt))


def predict_second_order(
    estimate: SecondOrderEstimate,
    time_sec: np.ndarray,
    command: np.ndarray,
    measured: np.ndarray,
    *,
    gravity_torque: np.ndarray | None = None,
) -> np.ndarray:
    """Simulate the fitted model open loop along *command*.

    Open loop on purpose: the measurement is used only to start the integration,
    never to correct it. Feeding the measurement back each step would score how
    well the model interpolates between samples it was already given, which every
    model does well; a simulator has to run without them.

    Integrated the same way ``fit_second_order`` differentiates — semi-implicit
    Euler — so a model fitted from a track reproduces that track exactly and any
    error is the model's rather than the arithmetic's.
    """
    step = _fit_step(time_sec)
    command = np.asarray(command, dtype=float)
    measured = np.asarray(measured, dtype=float)
    width = len(estimate.stiffness)
    if command.ndim != 2 or command.shape[1] != width or measured.shape != command.shape:
        raise FitError(
            f"the model covers {width} joints, the track "
            f"{command.shape[-1] if command.ndim == 2 else '?'}"
        )
    if (estimate.inverse_inertia is None) != (gravity_torque is None):
        raise FitError(
            "a model fitted with a gravity column must be scored with one, and "
            "one fitted without must not be: otherwise a different model is "
            "being scored than the one that was fitted"
        )
    if gravity_torque is not None:
        gravity_torque = np.asarray(gravity_torque, dtype=float)
        if gravity_torque.shape != command.shape:
            raise FitError("gravity torque must match the track's shape")

    predicted = np.empty_like(command)
    # Started at the second sample, because that is where a velocity can be
    # observed. Under semi-implicit Euler (q[i+1] = q[i] + dt*qd[i+1]) the first
    # difference is qd[1], not qd[0], so seeding position q[0] with it would
    # pair a position with the velocity from one step later — a small error that
    # then integrates for the whole run.
    predicted[0] = measured[0]
    position = measured[1].copy()
    velocity = (measured[1] - measured[0]) / step
    for index in range(1, len(command)):
        predicted[index] = position
        acceleration = (
            estimate.stiffness * (command[index] - position)
            - estimate.damping * velocity
            - estimate.friction * np.sign(velocity)
        )
        if gravity_torque is not None:
            acceleration = acceleration - estimate.inverse_inertia * gravity_torque[index]
        velocity = velocity + step * acceleration
        position = position + step * velocity
    return predicted


def score_holdout(
    estimate: SecondOrderEstimate,
    time_sec: np.ndarray,
    command: np.ndarray,
    measured: np.ndarray,
    *,
    gravity_torque: np.ndarray | None = None,
    max_lag_samples: int = 50,
) -> HoldoutMetrics:
    """Measure a fitted model against a run it was not fitted on.

    The baseline is the model-free assumption that the arm reached its command.
    That is what somebody with no identification at all would believe, so it is
    what a model has to beat to be worth carrying.
    """
    predicted = predict_second_order(
        estimate, time_sec, command, measured, gravity_torque=gravity_torque
    )
    step = _fit_step(time_sec)
    measured = np.asarray(measured, dtype=float)
    command = np.asarray(command, dtype=float)

    rmse = float(np.sqrt(np.mean((predicted - measured) ** 2)))
    baseline = float(np.sqrt(np.mean((command - measured) ** 2)))

    # How far the prediction would have to slide to line up best. A model that
    # captured the loop's delay needs no sliding; one that did not shows it here
    # rather than spreading it through the position error.
    lag = min(max_lag_samples, len(measured) // 4)
    best_shift, best_error = 0, rmse
    for shift in range(-lag, lag + 1):
        if shift == 0:
            continue
        if shift > 0:
            error = predicted[:-shift] - measured[shift:]
        else:
            error = predicted[-shift:] - measured[:shift]
        value = float(np.sqrt(np.mean(error**2)))
        if value < best_error:
            best_shift, best_error = shift, value

    return HoldoutMetrics(
        rmse_rad=rmse,
        baseline_rmse_rad=baseline,
        delay_residual_sec=abs(best_shift) * step,
        improvement_fraction=(
            0.0 if baseline <= 0 else max(0.0, 1.0 - rmse / baseline)
        ),
    )


def validate_holdout(
    *,
    openarm_rmse_rad: float,
    tesollo_rmse_rad: float,
    delay_residual_sec: float,
    command_period_sec: float,
    improvement_fraction: float,
) -> ValidationResult:
    failures = []
    if openarm_rmse_rad > 0.03:
        failures.append("openarm_rmse")
    if tesollo_rmse_rad > 0.05:
        failures.append("tesollo_rmse")
    if delay_residual_sec > command_period_sec:
        failures.append("delay_residual")
    if improvement_fraction < 0.30:
        failures.append("tracking_improvement")
    return ValidationResult(
        status="validated" if not failures else "model_inadequate",
        failures=tuple(failures),
    )
