"""Turn a calibration bundle into the file hdgp's training env loads.

The two schemas disagree about more than spelling. A bundle records what was
measured — one value per joint, with the sweeps and track it came from. hdgp's
``real2sim_actuator_cfg`` takes one scalar per actuator group and nothing else,
because that is what an ``ImplicitActuatorCfg`` holds. Converting therefore
throws information away, and every guard here exists because hdgp's
``get_actuator_params`` answers a name it does not recognise with the env's own
default and says nothing: a bad export does not fail, it trains.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .artifacts import nan_to_null
from .calibration import CalibrationBundle, IdentifiedParameters
from .profile import RobotProfile

#: The schema hdgp's loader accepts. It refuses anything else outright, so this
#: is a contract with that file rather than a version of our own to bump.
HDGP_SCHEMA_VERSION = 1

#: How far a group's joints may disagree, as (max - min) / mean, before one
#: scalar stops describing them. The arm is the case that motivates a limit at
#: all: its real gains run kp 70 / 60 / 10 across the seven joints, so the mean
#: describes no joint in the group. The hand groups are five copies of one
#: finger and sit far below this.
DEFAULT_MAX_SPREAD = 0.5


class HdgpExportError(ValueError):
    pass


def _collapse(values: np.ndarray) -> tuple[float, float]:
    """One number for the group, and how badly it misrepresents the joints."""
    mean = float(np.mean(values))
    if mean <= 0.0:
        # Friction is legitimately all-zero on a well-behaved joint; a relative
        # spread is undefined there rather than large.
        return mean, 0.0
    return mean, float((values.max() - values.min()) / mean)


def _group_payload(
    identified: IdentifiedParameters,
    source: str,
    *,
    delay_steps: int,
    max_spread: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    # Two measurements of one quantity, and they are not equally good. The
    # staircase reads it directly at rest under a torque we chose; the dynamic
    # coefficient is a regression over a track where it competes with damping.
    coulomb = np.where(
        np.isnan(identified.coulomb_nm), identified.friction, identified.coulomb_nm
    )
    gains = {
        "stiffness": identified.stiffness,
        "damping": identified.damping,
        "joint_friction": coulomb,
    }
    collapsed: dict[str, float] = {}
    spread: dict[str, float] = {}
    for key, values in gains.items():
        collapsed[key], spread[key] = _collapse(values)

    for key in ("stiffness", "damping"):
        # Only the two the env substitutes into its PD controller. Friction is
        # an additive Coulomb term, not a gain, and varies across an arm for
        # reasons that averaging does not distort in the same way.
        if spread[key] <= max_spread:
            continue
        worst = dict(zip(identified.joint_names, gains[key].tolist()))
        raise HdgpExportError(
            f"group {source!r}: measured {key} spans {spread[key]:.2f} of its "
            f"mean across {worst}, so one scalar describes no joint in it. "
            f"Split the group in hdgp's env_cfg and give each part its own "
            f"hdgp_group, or pass a wider max_spread to accept the average."
        )

    body = {key: float(value) for key, value in collapsed.items()}
    # hdgp reads this field into its dataclass but never applies it; it is
    # written because it is true, not because it does anything there yet.
    body["delay_steps"] = int(delay_steps)

    audit = {
        "source_group": source,
        "joint_names": list(identified.joint_names),
        "stiffness_nm_per_rad": identified.stiffness.tolist(),
        "damping_nm_s_per_rad": identified.damping.tolist(),
        # The per-joint source of joint_friction above: measured where the
        # staircase reached the joint, the dynamic coefficient elsewhere.
        "friction_nm": coulomb.tolist(),
        # The dynamic regression's own coefficient, kept alongside the
        # measurement of record rather than only feeding into it, so a large
        # gap between the two is visible here rather than only inferable.
        "dynamic_friction_nm": identified.friction.tolist(),
        # No hdgp entry point exists for inertia, so this block is the only
        # place the measurement survives the conversion at all.
        "inertia_kg_m2": identified.inertia.tolist(),
        # Fo: the dynamic fit's own bias term, scaled by its inertia. Also has
        # no hdgp entry point — an ImplicitActuatorCfg has no constant torque
        # offset field — so this is the only place it survives either.
        "bias_nm": identified.bias.tolist(),
        # Fo + tau_gravity(pose): the static fit's own bias, still carrying
        # the gravity torque at the pose it was measured at. Not the same
        # number as bias_nm above, and not averaged with it — a gap between
        # them is evidence about the gravity model, not noise to smooth over.
        "static_bias_nm": identified.static_bias_nm.tolist(),
        "spread": {key: float(value) for key, value in spread.items()},
        "provenance": {
            "sweep_sha256": list(identified.sweep_sha256),
            "track_sha256": identified.track_sha256,
        },
    }
    return body, audit


def hdgp_calibration(
    bundle: CalibrationBundle,
    profile: RobotProfile,
    *,
    max_spread: float = DEFAULT_MAX_SPREAD,
) -> dict[str, Any]:
    """Build hdgp's schema v1 payload from a bundle's identified parameters.

    Only measured groups appear. A group with no measurement is left out so
    hdgp keeps the env's own gain, which is the honest answer; writing the
    bundle's nominal guess there would be indistinguishable downstream from a
    measurement.
    """
    if not bundle.identified:
        raise HdgpExportError(
            "this bundle carries no identified parameters, so the export would "
            "apply nothing while reporting success; run r2s identify and r2s "
            "bundle first"
        )

    groups: dict[str, dict[str, Any]] = {}
    measured: dict[str, dict[str, Any]] = {}
    claimed: dict[str, str] = {}
    for source in sorted(bundle.identified):
        target = profile.groups[source].hdgp_group
        if target is None:
            raise HdgpExportError(
                f"group {source!r} was measured but the profile does not say "
                "what hdgp calls it; declare hdgp_group on it, since an "
                "unnamed group is dropped here and defaulted there in silence"
            )
        if target in claimed:
            raise HdgpExportError(
                f"groups {claimed[target]!r} and {source!r} both export as "
                f"{target!r}; one would overwrite the other"
            )
        claimed[target] = source
        groups[target], measured[target] = _group_payload(
            bundle.identified[source],
            source,
            delay_steps=bundle.controller.delay_steps,
            max_spread=max_spread,
        )

    return {
        "schema_version": HDGP_SCHEMA_VERSION,
        # hdgp's loader ignores both of these. They are what lets a calibration
        # found in a log directory be traced back to the robot it came off.
        "robot_asset": profile.asset_id,
        "source_dataset": _source_dataset(bundle),
        "groups": groups,
        "measured": measured,
    }


def _source_dataset(bundle: CalibrationBundle) -> str:
    checksum = bundle.payload.get("checksum_sha256")
    if not checksum:
        return f"robot_control:{bundle.profile}:unsigned"
    return f"robot_control:{bundle.profile}:{checksum}"


def write_hdgp_calibration(
    path: str | Path,
    bundle: CalibrationBundle,
    profile: RobotProfile,
    *,
    max_spread: float = DEFAULT_MAX_SPREAD,
) -> Mapping[str, Any]:
    payload = hdgp_calibration(bundle, profile, max_spread=max_spread)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # nan (bias_nm/static_bias_nm for a joint no staircase, or dynamic fit,
    # covered) means "not measured", written here as JSON's own null rather
    # than the non-standard `NaN` token — never a refusal, since an
    # unmeasured joint is expected. Converted on a copy, on the way to disk
    # only: the dict this function returns keeps nan, for a caller that reads
    # it in-process rather than off the file.
    path.write_text(
        json.dumps(nan_to_null(payload), indent=2, sort_keys=True, allow_nan=False)
        + "\n"
    )
    return payload
