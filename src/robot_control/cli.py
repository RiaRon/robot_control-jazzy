from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .artifacts import read_hdf5, track_sha256, write_hdf5
from .calibration import load_bundle
from .identification import build_excitation, fit_second_order, validate_holdout
from .profile import load_builtin_profile
from .track import normalize_track


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robotctl")
    commands = parser.add_subparsers(dest="command", required=True)
    r2s = commands.add_parser("r2s")
    stages = r2s.add_subparsers(dest="stage", required=True)
    for stage in ("preflight", "collect", "normalize", "fit", "validate", "export"):
        item = stages.add_parser(stage)
        item.add_argument("--profile", default="openarm_tesollo")
        if stage == "collect":
            mode = item.add_mutually_exclusive_group()
            mode.add_argument("--dry-run", action="store_true")
            mode.add_argument("--execute", action="store_true")
            item.add_argument("--amplitude-scale", type=float, default=0.3)
        if stage == "fit":
            item.add_argument("--population", type=int, default=128)
            item.add_argument("--track", type=Path)
            item.add_argument("--output", type=Path)
        if stage == "normalize":
            item.add_argument("--input", type=Path)
            item.add_argument("--output", type=Path)
        if stage in {"validate", "export"}:
            item.add_argument("--bundle", type=Path)
        if stage == "validate":
            item.add_argument("--metrics", type=Path)
            item.add_argument("--output", type=Path)
        if stage == "export":
            item.add_argument("--validation", type=Path)
            item.add_argument("--output", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    profile = load_builtin_profile(args.profile)
    if args.stage == "preflight":
        print(f"profile: {profile.name}")
        print(f"asset: {profile.asset_id}")
        print(f"joints: {len(profile.joints)}")
        print("publish_enabled: false")
    elif args.stage == "collect":
        if args.amplitude_scale <= 0 or args.amplitude_scale > 1:
            raise SystemExit("--amplitude-scale must be in (0, 1]")
        mode = "EXECUTE" if args.execute else "DRY RUN"
        neutral = np.array([(joint.lower + joint.upper) / 2 for joint in profile.joints])
        amplitude = np.array(
            [(joint.upper - joint.lower) * 0.05 * args.amplitude_scale for joint in profile.joints]
        )
        time, command, phases = build_excitation(neutral, amplitude, profile.ros["jazzy"].command_rate_hz)
        print(
            f"{mode}: profile={profile.name} amplitude_scale={args.amplitude_scale} "
            f"samples={len(time)} phases={','.join(dict.fromkeys(phases))}"
        )
        if args.execute:
            print("ROS publisher backend is required; no command was published")
            return 2
    elif args.stage == "normalize":
        if not args.input or not args.output:
            raise SystemExit("--input and --output are required")
        raw = np.load(args.input, allow_pickle=False)
        track = normalize_track(
            raw["command_time_ns"],
            raw["command"],
            raw["measured_time_ns"],
            raw["measured"],
            list(raw["joint_names"]),
            profile.ros["jazzy"].command_rate_hz,
        )
        write_hdf5(args.output, track)
        print(f"normalize: {args.output} sha256={track_sha256(track)}")
    elif args.stage == "fit":
        if not args.track or not args.output:
            raise SystemExit("--track and --output are required")
        if args.population <= 0:
            raise SystemExit("--population must be positive")
        track = read_hdf5(args.track)
        estimate = fit_second_order(track.timestamps_ns * 1e-9, track.command, track.measured)
        args.output.write_text(
            json.dumps(
                {
                    "population": args.population,
                    "joint_names": track.joint_names,
                    "stiffness": estimate.stiffness.tolist(),
                    "damping": estimate.damping.tolist(),
                    "friction": estimate.friction.tolist(),
                    "residual_rmse": estimate.residual_rmse.tolist(),
                    "track_sha256": track_sha256(track),
                },
                indent=2,
            )
            + "\n"
        )
        print(f"fit: {args.output}")
    elif args.stage == "validate":
        if not args.bundle:
            raise SystemExit("--bundle is required")
        bundle = load_bundle(args.bundle, profile)
        if not args.metrics or not args.output:
            raise SystemExit("--metrics and --output are required")
        metrics = json.loads(args.metrics.read_text())
        result = validate_holdout(**metrics)
        args.output.write_text(
            json.dumps({"status": result.status, "failures": result.failures}, indent=2) + "\n"
        )
        print(f"validate: schema v{bundle.schema_version}, status={result.status}")
        return 0 if result.status == "validated" else 3
    elif args.stage == "export":
        if not args.bundle or not args.validation or not args.output:
            raise SystemExit("--bundle, --validation, and --output are required")
        load_bundle(args.bundle, profile)
        validation = json.loads(args.validation.read_text())
        if validation.get("status") != "validated":
            print("export blocked: model_inadequate")
            return 3
        args.output.write_bytes(args.bundle.read_bytes())
        print(f"export: {args.output}")
    else:
        print(f"{args.stage}: profile={profile.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
