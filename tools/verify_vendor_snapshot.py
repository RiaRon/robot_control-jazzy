#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import shutil
import subprocess
from tempfile import TemporaryDirectory
from pathlib import Path

import yaml


def _excluded(relative: str, patterns: list[str]) -> bool:
    return any(
        fnmatch.fnmatch(relative, pattern)
        or relative == pattern.removesuffix("/**")
        for pattern in patterns
    )


def _files(root: Path, excluded: list[str]) -> dict[str, Path]:
    result = {}
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        if not _excluded(relative, excluded):
            result[relative] = path
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class _PatchApplicationError(Exception):
    def __init__(self, relative_path: str):
        self.relative_path = relative_path


def materialize_expected_tree(
    metadata_path: Path, source_checkout: Path, destination: Path
) -> None:
    metadata = yaml.safe_load(metadata_path.read_text()) or {}
    excluded = [str(pattern) for pattern in metadata.get("excluded", [])]
    for relative, source_path in _files(source_checkout, excluded).items():
        destination_path = destination / relative
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, destination_path)

    metadata_directory = metadata_path.resolve().parent
    for patch_reference in metadata.get("patches", []):
        relative_path = str(patch_reference)
        if Path(relative_path).is_absolute():
            raise _PatchApplicationError(relative_path)
        patch_path = (metadata_directory / relative_path).resolve()
        try:
            patch_path.relative_to(metadata_directory)
        except ValueError:
            raise _PatchApplicationError(relative_path)
        if not patch_path.is_file():
            raise _PatchApplicationError(relative_path)
        try:
            subprocess.run(
                ["patch", "--batch", "--forward", "-p1", "-i", str(patch_path)],
                cwd=destination,
                text=True,
                capture_output=True,
                check=True,
            )
        except subprocess.CalledProcessError:
            raise _PatchApplicationError(relative_path)


def verify_snapshot(
    metadata_path: Path, source_checkout: Path, snapshot: Path
) -> list[str]:
    metadata = yaml.safe_load(metadata_path.read_text()) or {}
    excluded = [str(pattern) for pattern in metadata.get("excluded", [])]
    try:
        with TemporaryDirectory() as directory:
            expected_tree = Path(directory)
            materialize_expected_tree(metadata_path, source_checkout, expected_tree)
            source_files = _files(expected_tree, excluded)
            snapshot_files = _files(snapshot, excluded)
            source_names = set(source_files)
            snapshot_names = set(snapshot_files)
            errors = [
                f"missing: {name}" for name in sorted(source_names - snapshot_names)
            ]
            errors.extend(
                f"unexpected: {name}" for name in sorted(snapshot_names - source_names)
            )
            errors.extend(
                f"content mismatch: {name}"
                for name in sorted(source_names & snapshot_names)
                if _sha256(source_files[name]) != _sha256(snapshot_files[name])
            )
            return errors
    except _PatchApplicationError as error:
        return [f"patch failed: {error.relative_path}"]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify a vendored source snapshot.")
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    args = parser.parse_args(argv)
    errors = verify_snapshot(args.metadata, args.source, args.snapshot)
    if errors:
        print("\n".join(errors))
        return 1
    print("snapshot verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
