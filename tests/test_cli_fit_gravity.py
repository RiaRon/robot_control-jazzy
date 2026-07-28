"""`robotctl r2s fit --static`: the gravity term, and physical parameters out.

The HDF5 layer is reached through its reader rather than a real file: h5py is an
optional extra and is not installed here, and what these tests are about is what
the fit does with a track, not how it was stored.
"""

import json

import numpy as np
import pytest

from robot_control.artifacts import write_static_estimate
from robot_control.cli import main
from robot_control.identification import StaticEstimate
from robot_control.kinematics import chain_from_urdf
from robot_control.profile import load_builtin_profile
from robot_control.track import CanonicalTrack


GROUP = "openarm_right_arm"
TIP = "openarm_right_hand_tcp"
STEP = 1e-3
SAMPLES = 3000


def _urdf():
    """A seven-joint arm about +y, each link a little further out along +x."""
    parts = ['<robot name="stub">', '<link name="base"/>']
    parent = "base"
    for index in range(1, 8):
        link = f"openarm_right_link{index}"
        origin = '0 0 0' if index == 1 else '0.12 0 0'
        parts.append(
            f'<joint name="openarm_right_joint{index}" type="revolute">'
            f'<parent link="{parent}"/><child link="{link}"/>'
            f'<origin xyz="{origin}" rpy="0 0 0"/><axis xyz="0 1 0"/>'
            f'<limit lower="-3.2" upper="3.2" effort="50" velocity="2"/></joint>'
        )
        parts.append(
            f'<link name="{link}"><inertial>'
            f'<origin xyz="0.06 0 0" rpy="0 0 0"/><mass value="0.35"/>'
            f'<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>'
            f"</inertial></link>"
        )
        parent = link
    parts.append(
        f'<joint name="openarm_right_tcp" type="fixed">'
        f'<parent link="{parent}"/><child link="{TIP}"/>'
        f'<origin xyz="0.08 0 0" rpy="0 0 0"/></joint>'
    )
    parts.append(f'<link name="{TIP}"/>')
    parts.append("</robot>")
    return "".join(parts)


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


@pytest.fixture
def urdf(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_text(_urdf())
    return path


KP = np.linspace(20.0, 8.0, 7)
DAMPING = np.linspace(1.5, 0.4, 7)
FRICTION = np.linspace(0.4, 0.1, 7)
INERTIA = np.linspace(0.35, 0.05, 7)


def _track(chain, loaded=True):
    """Simulate the arm the URDF describes, with or without its gravity load."""
    command = np.zeros((SAMPLES, 7))
    clock = np.arange(SAMPLES) * STEP
    for joint in range(7):
        command[:, joint] = 0.3 * np.sin(2 * np.pi * (0.6 + 0.2 * joint) * clock)

    measured = np.zeros((SAMPLES, 7))
    position = np.zeros(7)
    velocity = np.zeros(7)
    for index in range(SAMPLES):
        measured[index] = position
        load = chain.gravity_torque(position) if loaded else np.zeros(7)
        acceleration = (
            KP * (command[index] - position)
            - DAMPING * velocity
            - FRICTION * np.sign(velocity)
            - load
        ) / INERTIA
        velocity = velocity + STEP * acceleration
        position = position + STEP * velocity
    return CanonicalTrack(
        (clock * 1e9).astype(np.int64),
        command,
        measured,
        tuple(load_builtin_profile("openarm_tesollo").groups[GROUP].joints),
    )


def _chain():
    return chain_from_urdf(
        _urdf(),
        [f"openarm_right_joint{index}" for index in range(1, 8)],
        TIP,
    )


def _install(monkeypatch, canonical):
    monkeypatch.setattr("robot_control.cli.read_hdf5", lambda _path: canonical)
    monkeypatch.setattr("robot_control.cli.track_sha256", lambda _track: "f" * 64)
    return canonical


@pytest.fixture
def track(monkeypatch, profile):
    return _install(monkeypatch, _track(_chain()))


@pytest.fixture
def level_track(monkeypatch, profile):
    """The same arm with no standing load, which is what the older fit assumes."""
    return _install(monkeypatch, _track(_chain(), loaded=False))


def _static(path, profile, stiffness=KP):
    write_static_estimate(
        path,
        StaticEstimate(
            joint_names=profile.groups[GROUP].joints,
            stiffness=np.asarray(stiffness, dtype=float),
            torque_scale=np.ones(7),
            offset=np.zeros(7),
            residual_rmse=np.full(7, 1e-4),
            condition=np.full(7, 5.0),
            used=np.full(7, 12, dtype=int),
            excluded=np.zeros(7, dtype=int),
            unidentifiable=(),
        ),
        profile,
        group=GROUP,
        noise_rad=4e-4,
        sources=["a" * 64],
    )
    return path


def test_fit_with_a_static_estimate_reports_physical_parameters(
    track, urdf, profile, tmp_path, capsys
):
    static = _static(tmp_path / "static.json", profile)
    output = tmp_path / "estimate.json"

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(output),
         "--static", str(static), "--urdf", str(urdf)]
    )

    out = capsys.readouterr().out
    assert code == 0, out
    payload = json.loads(output.read_text())
    np.testing.assert_allclose(payload["inertia_kg_m2"], INERTIA, rtol=0.05)
    np.testing.assert_allclose(payload["damping_nm_s_per_rad"], DAMPING, rtol=0.05)
    np.testing.assert_allclose(payload["friction_nm"], FRICTION, rtol=0.1)
    np.testing.assert_allclose(payload["stiffness_nm_per_rad"], KP, rtol=1e-9)
    # The second, independent route to the same inertia.
    np.testing.assert_allclose(payload["inertia_from_gravity_kg_m2"], INERTIA, rtol=0.05)
    assert max(payload["inertia_disagreement"]) < 0.05
    # Fo, computed by `combine` alongside everything above; the track's own
    # simulation carries no bias term, so it should come back near zero.
    assert "bias_nm" in payload
    np.testing.assert_allclose(payload["bias_nm"], 0.0, atol=0.05)
    # The static estimate here is a gravity sweep, which never touches a
    # staircase — its own Fc and Fo are unmeasured, not zero. json.loads
    # reads the on-disk null back as None; `np.asarray(..., dtype=float)` —
    # what every real reader in this codebase does with these keys — turns
    # it into nan from there.
    assert payload["coulomb_nm"] == [None] * 7
    assert np.all(np.isnan(np.asarray(payload["coulomb_nm"], dtype=float)))
    assert np.all(np.isnan(np.asarray(payload["static_bias_nm"], dtype=float)))
    assert "Fo (N.m)" in out
    assert "landed in bias" not in out
    # Every joint unmeasured here — read the file back as text, not just
    # through json.loads, since Python parses both `null` and the
    # non-standard `NaN` token the same way.
    text = output.read_text()
    assert "null" in text
    assert "NaN" not in text


def test_fit_carries_a_static_estimates_own_coulomb_and_bias_through(
    monkeypatch, track, urdf, profile, tmp_path, capsys
):
    """Plumbing only: the file round trip for a staircase-backed static
    estimate does not exist yet (`write_static_estimate` refuses one), so this
    exercises `_fit` with a `StaticEstimate` built in-process instead."""
    import dataclasses

    import robot_control.artifacts as artifacts_module

    static = _static(tmp_path / "static.json", profile)
    artifact = artifacts_module.read_static_estimate(static, profile)
    seeing = dataclasses.replace(
        artifact,
        estimate=dataclasses.replace(
            artifact.estimate,
            coulomb_nm=np.full(7, 0.08),
            bias_nm=np.full(7, 0.3),
        ),
    )
    monkeypatch.setattr(
        "robot_control.cli.read_static_estimate", lambda _path, _profile: seeing
    )
    output = tmp_path / "estimate.json"

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(output),
         "--static", str(static), "--urdf", str(urdf)]
    )

    assert code == 0, capsys.readouterr().out
    payload = json.loads(output.read_text())
    np.testing.assert_allclose(payload["coulomb_nm"], 0.08)
    np.testing.assert_allclose(payload["static_bias_nm"], 0.3)


def test_fit_writes_null_for_a_partially_covered_staircase(
    monkeypatch, track, urdf, profile, tmp_path
):
    """Reproduces the reported case exactly: one joint measured, six not."""
    import dataclasses

    import robot_control.artifacts as artifacts_module

    static = _static(tmp_path / "static.json", profile)
    artifact = artifacts_module.read_static_estimate(static, profile)
    coulomb = np.full(7, np.nan)
    coulomb[0] = 0.08
    seeing = dataclasses.replace(
        artifact,
        estimate=dataclasses.replace(artifact.estimate, coulomb_nm=coulomb),
    )
    monkeypatch.setattr(
        "robot_control.cli.read_static_estimate", lambda _path, _profile: seeing
    )
    output = tmp_path / "estimate.json"

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(output),
         "--static", str(static), "--urdf", str(urdf)]
    )
    assert code == 0

    text = output.read_text()
    assert "null" in text
    assert "NaN" not in text
    payload = json.loads(output.read_text())
    assert payload["coulomb_nm"][0] == pytest.approx(0.08)
    assert payload["coulomb_nm"][1:] == [None] * 6
    assert np.all(np.isnan(np.asarray(payload["coulomb_nm"][1:], dtype=float)))


def test_fit_without_static_warns_that_any_standing_load_landed_in_bias(
    level_track, tmp_path, capsys
):
    output = tmp_path / "estimate.json"

    code = main(["r2s", "fit", "--track", "t.h5", "--output", str(output)])

    assert code == 0
    out = capsys.readouterr().out
    assert "landed in bias" in out
    assert "--static" in out


def test_fit_without_a_static_estimate_writes_what_it_always_did(
    level_track, tmp_path, capsys
):
    output = tmp_path / "estimate.json"

    assert main(["r2s", "fit", "--track", "t.h5", "--output", str(output)]) == 0

    payload = json.loads(output.read_text())
    assert set(payload) == {
        "population", "joint_names", "stiffness", "damping", "friction",
        "residual_rmse", "track_sha256",
    }


def test_a_loaded_track_without_the_gravity_term_gives_a_badly_wrong_fit(
    track, tmp_path, capsys
):
    """Used to refuse outright: the regression had nowhere to put a standing
    load but the stiffness and damping columns, and drove one of them out of
    the range the fit accepts. The bias column is a new, better sink for it,
    so the fit no longer refuses — but a per-joint constant cannot reproduce a
    pose-dependent gravity torque either, and what it cannot absorb still
    lands in stiffness, badly.
    """
    output = tmp_path / "estimate.json"

    code = main(["r2s", "fit", "--track", "t.h5", "--output", str(output)])

    assert code == 0, capsys.readouterr().out
    payload = json.loads(output.read_text())
    relative_error = np.abs(np.array(payload["stiffness"]) - KP) / KP
    assert np.all(relative_error > 0.05), (
        "the load still has to distort something; if this passes, the track "
        "no longer carries a standing load"
    )


def test_fit_static_needs_a_urdf(track, profile, tmp_path, capsys):
    """The modelled torque along the track has to come from somewhere."""
    static = _static(tmp_path / "static.json", profile)

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(tmp_path / "out.json"),
         "--static", str(static)]
    )

    assert code == 2
    assert "--urdf" in capsys.readouterr().out


def test_fit_refuses_a_track_of_other_joints(monkeypatch, urdf, profile, tmp_path, capsys):
    other = CanonicalTrack(
        (np.arange(100) * 1_000_000).astype(np.int64),
        np.zeros((100, 7)),
        np.zeros((100, 7)),
        tuple(profile.groups["openarm_left_arm"].joints),
    )
    monkeypatch.setattr("robot_control.cli.read_hdf5", lambda _path: other)
    static = _static(tmp_path / "static.json", profile)

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(tmp_path / "out.json"),
         "--static", str(static), "--urdf", str(urdf)]
    )

    assert code == 2
    out = capsys.readouterr().out
    assert "l_aj_1" in out and "r_aj_1" in out


def test_fit_refuses_two_inertias_that_disagree(track, urdf, profile, tmp_path, capsys):
    """A stiffness measured on another robot would otherwise become an inertia."""
    static = _static(tmp_path / "static.json", profile, stiffness=KP * 2.0)
    output = tmp_path / "estimate.json"

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(output),
         "--static", str(static), "--urdf", str(urdf)]
    )

    assert code == 3
    assert "disagree" in capsys.readouterr().out
    assert not output.exists()


def test_fit_refuses_a_static_estimate_from_another_asset(
    track, urdf, profile, tmp_path, capsys
):
    static = _static(tmp_path / "static.json", profile)
    payload = json.loads(static.read_text())
    payload["asset"]["id"] = "some-other-arm"
    static.write_text(json.dumps(payload))

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(tmp_path / "out.json"),
         "--static", str(static), "--urdf", str(urdf)]
    )

    assert code == 2
    assert "checksum" in capsys.readouterr().out
