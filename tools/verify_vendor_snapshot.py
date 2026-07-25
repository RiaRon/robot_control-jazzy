#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import re
import shutil
import subprocess
from pathlib import Path, PurePosixPath
from tempfile import TemporaryDirectory

import yaml


_SHA256 = re.compile(r"[0-9a-f]{64}")


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


class _MetadataFormatError(Exception):
    pass


def _read_metadata(metadata_path: Path) -> dict:
    document = yaml.safe_load(metadata_path.read_text())
    if document is None:
        return {}
    if not isinstance(document, dict):
        raise _MetadataFormatError
    return document


def _post_patch_inventory_errors(
    metadata: object,
    source_checkout: Path,
    expected_tree: Path,
    excluded: list[str],
) -> list[str]:
    if not isinstance(metadata, dict):
        return ["post-patch inventory invalid: metadata must be a mapping"]

    if "post_patch_sha256" not in metadata:
        raw_inventory = {}
    else:
        raw_inventory = metadata["post_patch_sha256"]
    if not isinstance(raw_inventory, dict):
        return [
            "post-patch inventory invalid: expected a path-to-sha256 mapping"
        ]

    errors = []
    inventory_paths = set()
    valid_hashes = {}
    for raw_path, raw_digest in raw_inventory.items():
        if not isinstance(raw_path, str):
            errors.append(f"post-patch inventory invalid path: {raw_path!r}")
            continue
        path = PurePosixPath(raw_path)
        if (
            not raw_path
            or path.is_absolute()
            or ".." in path.parts
            or "\\" in raw_path
            or path.as_posix() != raw_path
        ):
            errors.append(f"post-patch inventory invalid path: {raw_path}")
            continue
        inventory_paths.add(raw_path)
        if not isinstance(raw_digest, str) or _SHA256.fullmatch(raw_digest) is None:
            errors.append(f"post-patch inventory invalid hash: {raw_path}")
            continue
        valid_hashes[raw_path] = raw_digest

    source_files = _files(source_checkout, excluded)
    expected_files = _files(expected_tree, excluded)
    changed_paths = {
        name
        for name in source_files.keys() | expected_files.keys()
        if name not in source_files
        or name not in expected_files
        or _sha256(source_files[name]) != _sha256(expected_files[name])
    }

    errors.extend(
        f"post-patch inventory missing: {name}"
        for name in sorted(changed_paths - inventory_paths)
    )
    errors.extend(
        f"post-patch inventory stale: {name}"
        for name in sorted(inventory_paths - changed_paths)
    )
    for name in sorted(changed_paths & valid_hashes.keys()):
        expected_path = expected_files.get(name)
        if expected_path is None or valid_hashes[name] != _sha256(expected_path):
            errors.append(f"post-patch hash mismatch: {name}")
    return errors


def materialize_expected_tree(
    metadata_path: Path, source_checkout: Path, destination: Path
) -> None:
    metadata = _read_metadata(metadata_path)
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
        except (OSError, subprocess.CalledProcessError):
            raise _PatchApplicationError(relative_path)


def verify_snapshot(
    metadata_path: Path, source_checkout: Path, snapshot: Path
) -> list[str]:
    missing_roots = []
    if not source_checkout.is_dir():
        missing_roots.append(f"source directory missing: {source_checkout}")
    if not snapshot.is_dir():
        missing_roots.append(f"snapshot directory missing: {snapshot}")
    if missing_roots:
        return missing_roots

    try:
        metadata = _read_metadata(metadata_path)
    except _MetadataFormatError:
        return ["metadata invalid: expected a mapping"]
    excluded = [str(pattern) for pattern in metadata.get("excluded", [])]
    try:
        with TemporaryDirectory() as directory:
            expected_tree = Path(directory)
            materialize_expected_tree(metadata_path, source_checkout, expected_tree)
            inventory_errors = _post_patch_inventory_errors(
                metadata,
                source_checkout,
                expected_tree,
                excluded,
            )
            source_files = _files(expected_tree, excluded)
            snapshot_files = _files(snapshot, excluded)
            source_names = set(source_files)
            snapshot_names = set(snapshot_files)
            errors = inventory_errors + [
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
    except _MetadataFormatError:
        return ["metadata invalid: expected a mapping"]


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
