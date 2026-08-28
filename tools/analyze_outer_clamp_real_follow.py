#!/usr/bin/env python3
"""Compare real Follow runs before and after the outer crossing clamp.

The input archives and JSON files are opened read-only.  Old traces are used
only for one-cycle structural replay; they are not treated as a prediction of
the changed closed loop.
"""

from __future__ import annotations

import argparse
import json
import math
import tarfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

import matplotlib.pyplot as plt
import numpy as np

from replay_follow_outer import summarize_outer_replay


PROFILE_STAGES = {
    "profile_ramp": "ramp",
    "profile_hold": "hold",
    "profile_return": "return",
    "profile_origin_hold": "origin-hold",
}
JOINT_LAYERS = (
    "ik_target_to_command",
    "command_to_measured",
    "ik_target_to_measured",
)
TCP_PAIRS = {
    "accepted_marker_to_measured": ("accepted_marker", "measured"),
    "accepted_marker_to_ik": ("accepted_marker", "ik_target"),
    "ik_to_command": ("ik_target", "command"),
    "command_to_measured": ("command", "measured"),
    "ik_to_measured": ("ik_target", "measured"),
}


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def _read_tar_json(archive: Path, member_name: str) -> dict[str, Any]:
    with tarfile.open(archive, "r:gz") as handle:
        member = next(
            item for item in handle.getmembers() if Path(item.name).name == member_name
        )
        stream = handle.extractfile(member)
        if stream is None:
            raise ValueError(f"cannot read {member_name} from {archive}")
        return json.load(stream)


def _stats(values: Iterable[float]) -> dict[str, float | int | None]:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {"samples": 0, "mean": None, "rms": None, "max": None, "final": None}
    return {
        "samples": int(array.size),
        "mean": float(np.mean(array)),
        "rms": float(np.sqrt(np.mean(array * array))),
        "max": float(np.max(array)),
        "final": float(array[-1]),
    }


def _vector(sample: dict[str, Any], section: str, key: str) -> np.ndarray | None:
    values = sample.get(section, {}).get(key)
    array = np.asarray(values, dtype=float)
    if array.ndim != 1 or array.size == 0 or not np.all(np.isfinite(array)):
        return None
    return array


def _quat_distance(left: np.ndarray, right: np.ndarray) -> float:
    left = left / np.linalg.norm(left)
    right = right / np.linalg.norm(right)
    return 2.0 * math.acos(float(np.clip(abs(np.dot(left, right)), 0.0, 1.0)))


def _window(payload: dict[str, Any], phase: str | None = None) -> list[dict[str, Any]]:
    samples = [sample for sample in payload.get("trace", ()) if sample.get("stage") in PROFILE_STAGES]
    if phase is None:
        return samples
    return [sample for sample in samples if PROFILE_STAGES[sample["stage"]] == phase]


def _tcp_metrics(samples: list[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for label, (left_key, right_key) in TCP_PAIRS.items():
        position: list[float] = []
        orientation: list[float] = []
        signed_x: list[float] = []
        for sample in samples:
            left_p = _vector(sample, "tcp_positions_m", left_key)
            right_p = _vector(sample, "tcp_positions_m", right_key)
            left_q = _vector(sample, "tcp_orientations_xyzw", left_key)
            right_q = _vector(sample, "tcp_orientations_xyzw", right_key)
            if left_p is not None and right_p is not None:
                delta = left_p - right_p
                position.append(float(np.linalg.norm(delta)))
                signed_x.append(float(delta[0]))
            if left_q is not None and right_q is not None:
                orientation.append(_quat_distance(left_q, right_q))
        output[label] = {
            "position_m": _stats(position),
            "orientation_rad": _stats(orientation),
            "world_x_signed_m": _stats(signed_x),
        }
    return output


def _joint_metrics(samples: list[dict[str, Any]], joint_names: list[str]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for layer in JOINT_LAYERS:
        rows: list[np.ndarray] = []
        for sample in samples:
            values = _vector(sample, "joint_error_rad", layer)
            if values is not None and values.size == len(joint_names):
                rows.append(values)
        matrix = np.asarray(rows, dtype=float)
        if matrix.size == 0:
            continue
        output[layer] = {
            "vector_norm_rad": _stats(np.linalg.norm(matrix, axis=1)),
            "per_joint": {
                name: {
                    "abs_rad": _stats(np.abs(matrix[:, index])),
                    "signed_mean_rad": float(np.mean(matrix[:, index])),
                    "signed_final_rad": float(matrix[-1, index]),
                }
                for index, name in enumerate(joint_names)
            },
        }
    return output


def _limiter_metrics(samples: list[dict[str, Any]]) -> dict[str, Any]:
    counts = Counter()
    for sample in samples:
        limits = sample.get("limits", {})
        counts["cartesian_linear"] += bool(limits.get("cartesian_speed"))
        counts["cartesian_angular"] += bool(limits.get("cartesian_angular_speed"))
        joint = limits.get("joint", {})
        for name in ("velocity", "lead", "position"):
            counts[f"joint_{name}"] += bool(joint.get(name))
    total = len(samples)
    return {
        name: {"count": int(counts[name]), "rate": counts[name] / total if total else None}
        for name in (
            "cartesian_linear",
            "cartesian_angular",
            "joint_velocity",
            "joint_lead",
            "joint_position",
        )
    }


def _actual_clamp_metrics(
    samples: list[dict[str, Any]], joint_names: list[str], epsilon: float
) -> dict[str, Any]:
    joint = {name: Counter() for name in joint_names}
    phase = {name: Counter() for name in PROFILE_STAGES.values()}
    active_samples = 0
    raw_corrections: list[float] = []
    raw_penetrations: list[float] = []
    bounded_penetrations: list[float] = []
    final_penetrations: list[float] = []
    unclamped_candidate_mismatch = 0
    unclamped_joint_updates = 0
    for sample in samples:
        outer = sample.get("outer_target_crossing_clamp")
        if not isinstance(outer, dict):
            continue
        masks = {
            "clamp": outer.get("clamp_mask", ()),
            "strict": outer.get("crossing_mask", ()),
            "hold": outer.get("target_hold_mask", ()),
            "outward": outer.get("outward_accumulation_blocked_mask", ()),
            "reversal": outer.get("target_reversal_outward_blocked_mask", ()),
            "stalled": outer.get("stalled_recovery_mask", ()),
        }
        phase_name = PROFILE_STAGES[sample["stage"]]
        active_samples += bool(outer.get("active"))
        command = _vector(sample, "joint_positions_rad", "command")
        final = _vector(sample, "joint_positions_rad", "next_command")
        target = np.asarray(outer.get("ik_target_rad"), dtype=float)
        raw = np.asarray(outer.get("raw_candidate_rad"), dtype=float)
        bounded = np.asarray(outer.get("bounded_candidate_rad"), dtype=float)
        if any(array.shape != (len(joint_names),) for array in (target, raw, bounded)):
            continue
        raw_corrections.append(float(np.linalg.norm(raw - bounded)))
        for index, name in enumerate(joint_names):
            for label, mask in masks.items():
                value = bool(mask[index]) if len(mask) == len(joint_names) else False
                joint[name][label] += value
                phase[phase_name][label] += value
            if command is None:
                continue
            before = target[index] - command[index]
            raw_after = target[index] - raw[index]
            bounded_after = target[index] - bounded[index]
            if before * raw_after < -(epsilon * epsilon):
                raw_penetrations.append(abs(raw_after))
            if before * bounded_after < -(epsilon * epsilon):
                bounded_penetrations.append(abs(bounded_after))
            if final is not None and before * (target[index] - final[index]) < -(epsilon * epsilon):
                final_penetrations.append(abs(target[index] - final[index]))
            if not bool(masks["clamp"][index]):
                unclamped_joint_updates += 1
                unclamped_candidate_mismatch += abs(raw[index] - bounded[index]) > epsilon
    return {
        "active_samples": active_samples,
        "sample_count": len(samples),
        "joint_events": {name: dict(counts) for name, counts in joint.items()},
        "phase_joint_events": {name: dict(counts) for name, counts in phase.items()},
        "raw_to_bounded_norm_rad": _stats(raw_corrections),
        "raw_strict_crossing_max_penetration_rad": max(raw_penetrations, default=0.0),
        "bounded_crossing_max_penetration_rad": max(bounded_penetrations, default=0.0),
        "final_command_recrossing_joint_events": len(final_penetrations),
        "final_command_recrossing_max_penetration_rad": max(final_penetrations, default=0.0),
        "unclamped_joint_updates": unclamped_joint_updates,
        "unclamped_raw_bounded_mismatch_joint_events": unclamped_candidate_mismatch,
    }


def _stale_resolution(samples: list[dict[str, Any]], joint_names: list[str]) -> dict[str, Any]:
    """Measure stale command dwell using IK motion direction, never joint-angle zero."""
    if not samples:
        return {"return": {}, "origin-hold": {}}
    target = np.asarray(
        [_vector(sample, "joint_positions_rad", "ik_target") for sample in samples],
        dtype=float,
    )
    command = np.asarray(
        [_vector(sample, "joint_positions_rad", "command") for sample in samples],
        dtype=float,
    )
    time = np.asarray([float(sample.get("timestamp_sec", np.nan)) for sample in samples])
    stages = [PROFILE_STAGES[sample["stage"]] for sample in samples]
    output: dict[str, Any] = {}
    for phase_name in ("return", "origin-hold"):
        indices = [index for index, name in enumerate(stages) if name == phase_name]
        per_joint: dict[str, Any] = {}
        for joint_index, joint_name in enumerate(joint_names):
            ramp_indices = [index for index, name in enumerate(stages) if name == "ramp"]
            hold_indices = [index for index, name in enumerate(stages) if name == "hold"]
            if not ramp_indices or not hold_indices or not indices:
                continue
            outbound_delta = target[hold_indices[-1], joint_index] - target[ramp_indices[0], joint_index]
            if abs(outbound_delta) <= 1e-5:
                per_joint[joint_name] = {"applicable": False}
                continue
            direction = -math.copysign(1.0, outbound_delta)
            start = indices[0]
            if phase_name == "return":
                hold_target = target[hold_indices[-1], joint_index]
                moving = [
                    index
                    for index in indices
                    if (target[index, joint_index] - hold_target) * direction > 1e-6
                ]
                if moving:
                    start = moving[0]
            stale = (command[indices, joint_index] - target[indices, joint_index]) * direction < -1e-6
            phase_positions = np.asarray(indices)
            eligible = phase_positions >= start
            strict_stale = stale & eligible
            offset = np.abs(command[indices, joint_index] - target[indices, joint_index])
            significant_stale = strict_stale & (offset > 1e-3)

            def _dwell(mask: np.ndarray) -> float:
                selected = phase_positions[mask]
                if selected.size == 0:
                    return 0.0
                first = int(selected[0])
                later = phase_positions[(phase_positions > first) & ~mask]
                resolved = int(later[0]) if later.size else indices[-1]
                return max(0.0, float(time[resolved] - time[first]))

            strict_dwell = _dwell(strict_stale)
            significant_dwell = _dwell(significant_stale)
            strict_indices = phase_positions[strict_stale]
            max_offset = (
                float(np.max(np.abs(command[strict_indices, joint_index] - target[strict_indices, joint_index])))
                if strict_indices.size
                else 0.0
            )
            per_joint[joint_name] = {
                "applicable": True,
                "strict_sign_stale_dwell_sec": strict_dwell,
                "stale_above_1mrad_dwell_sec": significant_dwell,
                "max_stale_offset_rad": max_offset,
            }
        applicable = [value for value in per_joint.values() if value.get("applicable")]
        worst = max(
            applicable,
            key=lambda value: value["stale_above_1mrad_dwell_sec"],
            default=None,
        )
        worst_name = next((name for name, value in per_joint.items() if value is worst), None)
        output[phase_name] = {
            "per_joint": per_joint,
            "worst_joint": worst_name,
            "max_strict_sign_stale_dwell_sec": max(
                (value["strict_sign_stale_dwell_sec"] for value in applicable), default=None
            ),
            "max_stale_above_1mrad_dwell_sec": (
                worst["stale_above_1mrad_dwell_sec"] if worst else None
            ),
            "max_stale_offset_rad": max(
                (value["max_stale_offset_rad"] for value in applicable), default=None
            ),
        }
    return output


def _quality(payload: dict[str, Any]) -> dict[str, Any]:
    result = payload.get("result", {})
    execution = result.get("diagnostic_execution", {})
    stages = [sample.get("stage") for sample in payload.get("trace", ())]
    profile = _window(payload)
    finite = True
    for sample in profile:
        for section in ("joint_positions_rad", "tcp_positions_m", "tcp_orientations_xyzw"):
            for values in sample.get(section, {}).values():
                finite = finite and bool(np.all(np.isfinite(np.asarray(values, dtype=float))))
    return {
        "schema_version": payload.get("schema_version"),
        "termination": result.get("termination"),
        "partial": bool(result.get("is_partial")),
        "profile_started": bool(execution.get("started")),
        "profile_completed": bool(execution.get("completed")),
        "trace_samples": len(payload.get("trace", ())),
        "profile_samples": len(profile),
        "profile_publish_count": execution.get("position_publish_count"),
        "stages": sorted(set(stages)),
        "complete_phase_set": set(PROFILE_STAGES).issubset(stages),
        "starts_with_startup": bool(stages and stages[0] == "startup_alignment"),
        "ends_with_origin_hold": bool(stages and stages[-1] == "profile_origin_hold"),
        "finite_profile_trace": finite,
    }


def _safety(payload: dict[str, Any], samples: list[dict[str, Any]]) -> dict[str, Any]:
    result = payload.get("result", {})
    ik = result.get("ik", {})
    outcomes = Counter(event.get("outcome") for event in ik.get("events", ()))
    jumps = result.get("ik_target_jumps", ())
    return {
        "ik_accepted": outcomes["accepted"],
        "ik_failed": int(ik.get("failed", outcomes["failed"])),
        "ik_superseded": int(ik.get("superseded", outcomes["superseded"])),
        "continuity_rejected": int(ik.get("continuity_rejected", 0)),
        "continuity_retries": int(ik.get("continuity_retries", 0)),
        "continuity_exhausted": int(ik.get("continuity_exhausted", 0)),
        "target_jump_events": (
            len(jumps.get("events", ()))
            if isinstance(jumps, dict)
            else len(jumps) if isinstance(jumps, list) else int(jumps or 0)
        ),
        "limiters": _limiter_metrics(samples),
    }


def _analyze_run(name: str, period: str, profile: str, payload: dict[str, Any]) -> dict[str, Any]:
    samples = _window(payload)
    phases = {
        phase: {
            "tcp": _tcp_metrics(_window(payload, phase)),
            "joints": _joint_metrics(_window(payload, phase), payload["joint_names"]),
            "limiters": _limiter_metrics(_window(payload, phase)),
        }
        for phase in PROFILE_STAGES.values()
    }
    filtered = dict(payload)
    filtered["trace"] = samples
    crossing: dict[str, Any]
    if period == "before":
        crossing = summarize_outer_replay(filtered)
    else:
        crossing = _actual_clamp_metrics(
            samples,
            payload["joint_names"],
            float(payload.get("settings", {}).get("outer_target_sign_epsilon_rad", 1e-12)),
        )
    return {
        "name": name,
        "period": period,
        "profile_kind": profile,
        "quality": _quality(payload),
        "configuration": {
            "group": payload.get("group"),
            "joint_names": payload.get("joint_names"),
            "profile": payload.get("profile"),
            "diagnostic_profile": payload.get("settings", {}).get("diagnostic_profile"),
            "command_rate_hz": payload.get("settings", {}).get("command_rate_hz"),
            "kp_per_sec": payload.get("settings", {}).get("kp_per_sec"),
            "gravity_scale": payload.get("settings", {}).get("gravity_scale"),
            "max_tcp_speed_m_s": payload.get("settings", {}).get("max_tcp_speed_m_s"),
            "max_tcp_angular_speed_rad_s": payload.get("settings", {}).get("max_tcp_angular_speed_rad_s"),
            "max_joint_lead_sec": payload.get("settings", {}).get("max_joint_lead_sec"),
            "max_ik_step_m": payload.get("settings", {}).get("max_ik_step_m"),
            "max_ik_angular_step_rad": payload.get("settings", {}).get("max_ik_angular_step_rad"),
        },
        "actual_control_rate_hz": payload.get("result", {}).get("actual_control_rate_hz"),
        "profile_only": {
            "tcp": _tcp_metrics(samples),
            "joints": _joint_metrics(samples, payload["joint_names"]),
            "limiters": _limiter_metrics(samples),
        },
        "phases": phases,
        "crossing": crossing,
        "stale_resolution": _stale_resolution(samples, payload["joint_names"]),
        "safety": _safety(payload, samples),
    }


def _json_default(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"cannot serialize {type(value).__name__}")


def _range(values: list[float]) -> dict[str, float]:
    return {"mean": float(np.mean(values)), "min": float(np.min(values)), "max": float(np.max(values))}


def _aggregate(runs: list[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for profile in ("translation", "rotation", "translation-rotation"):
        output[profile] = {}
        for period in ("before", "after"):
            selected = [run for run in runs if run["profile_kind"] == profile and run["period"] == period]
            metrics: dict[str, list[float]] = defaultdict(list)
            for run in selected:
                for layer in TCP_PAIRS:
                    tcp = run["profile_only"]["tcp"][layer]
                    for domain, scale in (("position_m", 1000.0), ("orientation_rad", 180.0 / math.pi)):
                        for stat in ("mean", "rms", "max", "final"):
                            metrics[f"{layer}_{domain}_{stat}"].append(tcp[domain][stat] * scale)
                for joint_layer in JOINT_LAYERS:
                    joint = run["profile_only"]["joints"][joint_layer]
                    for stat in ("mean", "rms", "max", "final"):
                        metrics[f"{joint_layer}_joint_norm_mrad_{stat}"].append(
                            joint["vector_norm_rad"][stat] * 1000.0
                        )
                for phase in ("return", "origin-hold"):
                    phase_accepted = run["phases"][phase]["tcp"]["accepted_marker_to_measured"]
                    metrics[f"{phase}_position_max_mm"].append(phase_accepted["position_m"]["max"] * 1000.0)
                    metrics[f"{phase}_position_final_mm"].append(phase_accepted["position_m"]["final"] * 1000.0)
                    metrics[f"{phase}_orientation_max_deg"].append(phase_accepted["orientation_rad"]["max"] * 180.0 / math.pi)
                    metrics[f"{phase}_orientation_final_deg"].append(phase_accepted["orientation_rad"]["final"] * 180.0 / math.pi)
                    dwell = run["stale_resolution"][phase]["max_stale_above_1mrad_dwell_sec"]
                    offset = run["stale_resolution"][phase]["max_stale_offset_rad"]
                    if dwell is not None:
                        metrics[f"{phase}_max_stale_above_1mrad_dwell_sec"].append(dwell)
                    if offset is not None:
                        metrics[f"{phase}_max_stale_offset_mrad"].append(offset * 1000.0)
                if period == "before":
                    metrics["crossing_max_mrad"].append(run["crossing"]["old_max_crossing_rad"] * 1000.0)
                    metrics["crossing_events"].append(run["crossing"]["old_strict_crossing_joint_events"])
                else:
                    metrics["crossing_max_mrad"].append(run["crossing"]["bounded_crossing_max_penetration_rad"] * 1000.0)
                    strict = sum(item.get("strict", 0) for item in run["crossing"]["joint_events"].values())
                    metrics["raw_crossing_events"].append(strict)
                    metrics["raw_crossing_attempt_max_mrad"].append(
                        run["crossing"]["raw_strict_crossing_max_penetration_rad"] * 1000.0
                    )
                    metrics["final_recrossing_max_mrad"].append(
                        run["crossing"]["final_command_recrossing_max_penetration_rad"] * 1000.0
                    )
                    metrics["clamp_active_rate"].append(
                        run["crossing"]["active_samples"] / run["crossing"]["sample_count"]
                    )
                    metrics["clamp_joint_events"].append(
                        sum(item.get("clamp", 0) for item in run["crossing"]["joint_events"].values())
                    )
                    metrics["outward_blocked_joint_events"].append(
                        sum(item.get("outward", 0) for item in run["crossing"]["joint_events"].values())
                    )
            output[profile][period] = {
                "run_count": len(selected),
                **{name: _range(values) for name, values in metrics.items() if values},
            }
    return output


def _plots(report: dict[str, Any], output_dir: Path) -> None:
    labels = ["Translation", "Rotation", "Combined"]
    keys = ["translation", "rotation", "translation-rotation"]
    x = np.arange(3)
    width = 0.34

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.1), constrained_layout=True)
    before = [report["aggregates"][key]["before"]["crossing_max_mrad"]["mean"] for key in keys]
    after = [report["aggregates"][key]["after"]["crossing_max_mrad"]["mean"] for key in keys]
    axes[0].bar(x - width / 2, before, width, label="before: offline raw")
    axes[0].bar(x + width / 2, after, width, label="after: bounded/final")
    axes[0].set_ylabel("maximum crossing penetration (mrad)")
    axes[0].set_xticks(x, labels)
    axes[0].legend(fontsize=8)
    before = [report["aggregates"][key]["before"]["return_max_stale_above_1mrad_dwell_sec"]["mean"] for key in keys]
    after = [report["aggregates"][key]["after"]["return_max_stale_above_1mrad_dwell_sec"]["mean"] for key in keys]
    axes[1].bar(x - width / 2, before, width, label="before")
    axes[1].bar(x + width / 2, after, width, label="after")
    axes[1].set_ylabel("worst return stale dwell > 1 mrad (s)")
    axes[1].set_xticks(x, labels)
    axes[1].legend(fontsize=8)
    fig.suptitle("Outer target crossing and reversal recovery (profile_only)")
    fig.savefig(output_dir / "outer-clamp-crossing-recovery.svg")
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.0), constrained_layout=True)
    for column, phase in enumerate(("return", "origin-hold")):
        for row, (domain, unit) in enumerate((("position", "mm"), ("orientation", "deg"))):
            metric = f"{phase}_{domain}_max_{unit}"
            before = [report["aggregates"][key]["before"][metric]["mean"] for key in keys]
            after = [report["aggregates"][key]["after"][metric]["mean"] for key in keys]
            ax = axes[row, column]
            ax.bar(x - width / 2, before, width, label="before")
            ax.bar(x + width / 2, after, width, label="after")
            ax.set_xticks(x, labels)
            ax.set_ylabel(f"maximum error ({unit})")
            ax.set_title(phase)
            if row == 0:
                ax.legend(fontsize=8)
    fig.suptitle("Return and origin-hold tracking error (profile_only)")
    fig.savefig(output_dir / "outer-clamp-return-origin-hold.svg")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=Path("/home/cbj4/openarm_follow_data"))
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    root = args.data_root
    pre25 = root / "openarm-handoff-v2-pre-timeout-fix-2026-08-25"
    archive26 = root / "openarm-baseline-translation-2026-08-26.tar.gz"
    archive28 = root / "2026-08-28-outer-clamp-real.tar.gz"
    definitions: list[tuple[str, str, str, dict[str, Any]]] = []
    for index in (1, 2):
        name = f"T{index}-before"
        payload = _read_tar_json(archive26, f"right-follow-baseline-translation-run{index}.json")
        definitions.append((name, "before", "translation", payload))
    for profile, filenames in {
        "rotation": ("right-follow-rotation-run2.json", "right-follow-rotation-run3.json"),
        "translation-rotation": ("right-follow-combined-run1.json", "right-follow-combined-run2.json"),
    }.items():
        for index, filename in enumerate(filenames, 1):
            definitions.append((f"{'R' if profile == 'rotation' else 'C'}{index}-before", "before", profile, _read_json(pre25 / filename)))
    for profile, prefix in (("translation", "t"), ("rotation", "r"), ("translation-rotation", "c")):
        for index in (1, 2, 3):
            definitions.append((f"{prefix.upper()}{index}-after", "after", profile, _read_tar_json(archive28, f"{prefix}{index}-{'combined' if prefix == 'c' else profile}.json")))

    runs = [_analyze_run(*definition) for definition in definitions]
    supplementary = _analyze_run(
        "R4-before-supplementary",
        "before",
        "rotation",
        _read_json(pre25 / "right-follow-rotation-run4.json"),
    )
    report = {
        "method": {
            "comparison_window": "profile_only = profile_ramp + profile_hold + profile_return + profile_origin_hold",
            "before_crossing": "one-cycle structural replay on recorded old closed-loop trace; not a new closed-loop result",
            "stale_definition": "command opposite the latest IK target motion direction; strict-sign and above-1-mrad dwell are both recorded; joint-angle zero is not used",
        },
        "runs": runs,
        "aggregates": _aggregate(runs),
        "supplementary_rotation_run4": supplementary,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "analysis.json").write_text(
        json.dumps(report, default=_json_default, indent=2, sort_keys=True) + "\n"
    )
    _plots(report, args.output_dir)
    print(json.dumps(report["aggregates"], default=_json_default, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
