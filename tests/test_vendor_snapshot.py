from pathlib import Path

import pytest
import yaml

from tools.verify_vendor_snapshot import verify_snapshot


ROOT = Path(__file__).parents[1]
AFTER_SHA256 = "7b9a72466d3960eb2aacccfc848939453490db0678bd4725def3f789b891c919"
UNCHANGED_SHA256 = (
    "1cd263f1102656dd6b6cf1d626d1a96f9eba0406af3cb2a52560d473d4801052"
)


def test_metadata_pins_validated_commits():
    openarm = yaml.safe_load(
        (ROOT / "vendor_metadata/openarm/UPSTREAM.yaml").read_text()
    )
    tesollo = yaml.safe_load(
        (ROOT / "vendor_metadata/tesollo/UPSTREAM.yaml").read_text()
    )
    openarm_can = yaml.safe_load(
        (ROOT / "vendor_metadata/openarm_can/UPSTREAM.yaml").read_text()
    )
    description = yaml.safe_load(
        (ROOT / "vendor_metadata/openarm_description/UPSTREAM.yaml").read_text()
    )
    assert openarm["branch"] == "jazzy"
    assert openarm["commit"] == "8087bbc2b37c0b2b2652c0134a9b2b369c57567e"
    assert tesollo["branch"] == "jazzy-dev"
    assert tesollo["commit"] == "3926c2eab8d011046f64874d6252213b2cf18f48"
    assert openarm_can["commit"] == "c32ecd31da267967f0c913c2118c843177d88b91"
    assert description["commit"] == "c8696ebfd64ea08ee0a212a9bae21055b6f381bc"
    assert description["source_subpath"] == "src/openarm_description"


def test_snapshot_verifier_detects_changed_file(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    (source / "LICENSE").write_text("license")
    (snapshot / "LICENSE").write_text("modified")
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: [.git, .github]\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        "content mismatch: LICENSE"
    ]


def test_snapshot_verifier_rejects_missing_source_directory(tmp_path):
    source = tmp_path / "missing-source"
    snapshot = tmp_path / "snapshot"
    snapshot.mkdir()
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        f"source directory missing: {source}"
    ]


def test_snapshot_verifier_rejects_missing_snapshot_directory(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "missing-snapshot"
    source.mkdir()
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        f"snapshot directory missing: {snapshot}"
    ]


def test_snapshot_verifier_reports_missing_and_unexpected_in_sorted_order(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    (source / "b.txt").write_text("b")
    (snapshot / "a.txt").write_text("a")
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        "missing: b.txt",
        "unexpected: a.txt",
    ]


def test_snapshot_verifier_applies_declared_patches(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    patches = tmp_path / "patches"
    source.mkdir()
    snapshot.mkdir()
    patches.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("after\n")
    (patches / "0001-change.patch").write_text(
        "--- a/example.txt\n"
        "+++ b/example.txt\n"
        "@@ -1 +1 @@\n"
        "-before\n"
        "+after\n"
    )
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text(
        "excluded: []\n"
        "patches:\n"
        "  - patches/0001-change.patch\n"
        "post_patch_sha256:\n"
        f"  example.txt: {AFTER_SHA256}\n"
    )

    assert verify_snapshot(metadata, source, snapshot) == []

    (snapshot / "example.txt").write_text("undeclared edit\n")
    assert verify_snapshot(metadata, source, snapshot) == [
        "content mismatch: example.txt"
    ]


def test_snapshot_verifier_requires_every_post_patch_hash(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    patches = tmp_path / "patches"
    source.mkdir()
    snapshot.mkdir()
    patches.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("after\n")
    (patches / "0001-change.patch").write_text(
        "--- a/example.txt\n"
        "+++ b/example.txt\n"
        "@@ -1 +1 @@\n"
        "-before\n"
        "+after\n"
    )
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\npatches:\n  - patches/0001-change.patch\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        "post-patch inventory missing: example.txt"
    ]


def test_snapshot_verifier_rejects_stale_post_patch_inventory_entry(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    patches = tmp_path / "patches"
    source.mkdir()
    snapshot.mkdir()
    patches.mkdir()
    (source / "example.txt").write_text("before\n")
    (source / "unchanged.txt").write_text("unchanged\n")
    (snapshot / "example.txt").write_text("after\n")
    (snapshot / "unchanged.txt").write_text("unchanged\n")
    (patches / "0001-change.patch").write_text(
        "--- a/example.txt\n"
        "+++ b/example.txt\n"
        "@@ -1 +1 @@\n"
        "-before\n"
        "+after\n"
    )
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text(
        "excluded: []\n"
        "patches:\n"
        "  - patches/0001-change.patch\n"
        "post_patch_sha256:\n"
        f"  example.txt: {AFTER_SHA256}\n"
        f"  unchanged.txt: {UNCHANGED_SHA256}\n"
    )

    assert verify_snapshot(metadata, source, snapshot) == [
        "post-patch inventory stale: unchanged.txt"
    ]


def test_snapshot_verifier_rejects_mismatched_post_patch_hash(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    patches = tmp_path / "patches"
    source.mkdir()
    snapshot.mkdir()
    patches.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("after\n")
    (patches / "0001-change.patch").write_text(
        "--- a/example.txt\n"
        "+++ b/example.txt\n"
        "@@ -1 +1 @@\n"
        "-before\n"
        "+after\n"
    )
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text(
        "excluded: []\n"
        "patches:\n"
        "  - patches/0001-change.patch\n"
        "post_patch_sha256:\n"
        f"  example.txt: {UNCHANGED_SHA256}\n"
    )

    assert verify_snapshot(metadata, source, snapshot) == [
        "post-patch hash mismatch: example.txt"
    ]


def test_snapshot_verifier_rejects_invalid_post_patch_inventory_schema(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\npost_patch_sha256: []\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        "post-patch inventory invalid: expected a path-to-sha256 mapping"
    ]


@pytest.mark.parametrize(
    "document",
    [
        "- not-a-mapping\n",
        "[]\n",
        "0\n",
        "false\n",
    ],
)
def test_snapshot_verifier_rejects_non_mapping_metadata(tmp_path, document):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text(document)

    assert verify_snapshot(metadata, source, snapshot) == [
        "metadata invalid: expected a mapping"
    ]


def test_snapshot_verifier_rejects_missing_patch(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("before\n")
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\npatches:\n  - patches/0001-change.patch\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        "patch failed: patches/0001-change.patch"
    ]


def test_snapshot_verifier_rejects_absolute_patch_path(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    patches = tmp_path / "patches"
    source.mkdir()
    snapshot.mkdir()
    patches.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("after\n")
    patch = patches / "0001-change.patch"
    patch.write_text(
        "--- a/example.txt\n"
        "+++ b/example.txt\n"
        "@@ -1 +1 @@\n"
        "-before\n"
        "+after\n"
    )
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text(f"excluded: []\npatches:\n  - {patch}\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        f"patch failed: {patch}"
    ]


def test_snapshot_verifier_rejects_unavailable_patch_command(tmp_path, monkeypatch):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    patches = tmp_path / "patches"
    source.mkdir()
    snapshot.mkdir()
    patches.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("after\n")
    (patches / "0001-change.patch").write_text(
        "--- a/example.txt\n"
        "+++ b/example.txt\n"
        "@@ -1 +1 @@\n"
        "-before\n"
        "+after\n"
    )
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\npatches:\n  - patches/0001-change.patch\n")

    def unavailable_patch_command(*args, **kwargs):
        raise OSError("patch command unavailable")

    monkeypatch.setattr(
        "tools.verify_vendor_snapshot.subprocess.run", unavailable_patch_command
    )

    assert verify_snapshot(metadata, source, snapshot) == [
        "patch failed: patches/0001-change.patch"
    ]
