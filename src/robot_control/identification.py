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
    #: (rounds, joints) the controller's own tracking error after the hold.
    errors: np.ndarray
    #: The joint whose scale varied, when the sweep varied only one.
    sweep_joint: str | None = None

    _GRIDS = ("poses", "modelled_torque", "scales", "errors")

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
                "every round needs a pose, a modelled torque, a scale and an "
                f"error, but the counts differ: {sorted(counts)}"
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
    #: alpha: the factor the modelled gravity torque was wrong by.
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
    applied = sweep.scales[:, joint] * sweep.modelled_torque[:, joint]
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
                scale = float(sweep.scales[index, joint])
                rows.append((torque, -scale * torque, 1.0))
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
        condition[joint] = _condition(design)
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
        params, _, _, _ = np.linalg.lstsq(design, target, rcond=None)
        by_stiffness, inverse_stiffness, constant = (float(value) for value in params)
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
        residual[joint] = float(np.sqrt(np.mean((design @ params - target) ** 2)))

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
    width = command.shape[1]
    stiffness = np.empty(width)
    damping = np.empty(width)
    friction = np.empty(width)
    residual = np.empty(width)
    inverse_inertia = None if gravity_torque is None else np.empty(width)
    for joint in range(width):
        error = command[1:-1, joint] - measured[1:-1, joint]
        prior_velocity = velocity[:-1, joint]
        columns = [error, -prior_velocity, -np.sign(prior_velocity)]
        if gravity_torque is not None:
            # Aligned with `error`, which reads the same window of samples.
            columns.append(-gravity_torque[1:-1, joint])
        design = np.column_stack(columns)
        params, _, rank, _ = np.linalg.lstsq(design, acceleration[:, joint], rcond=None)
        if rank < 2 or params[0] <= 0 or params[1] < 0:
            raise FitError(f"unidentifiable dynamics for joint {joint}")
        if gravity_torque is not None:
            if params[3] <= 0:
                raise FitError(
                    f"joint {joint} accelerates towards its load rather than away "
                    "from it, so the gravity column has the wrong sign or the "
                    "wrong chain"
                )
            inverse_inertia[joint] = params[3]
        prediction = design @ params
        stiffness[joint], damping[joint], friction[joint] = params[:3]
        residual[joint] = float(np.sqrt(np.mean((prediction - acceleration[:, joint]) ** 2)))
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
