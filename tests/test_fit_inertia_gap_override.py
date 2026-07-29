"""`--accept-inertia-gap`: writing a fit whose two inertia routes disagree.

The gap gate is right by default — kp/k and 1/g measuring different numbers
usually means one of them is measuring the wrong arm. But on a real arm whose
URDF mass distribution is only approximately right, the gravity route is the
inaccurate one and kp/k (staircase kp, measured without any model) is not.
Refusing then throws away a calibration that is good where it matters, so the
gate becomes something the operator can accept on the record.
"""

import json

import numpy as np
import pytest

from robot_control.artifacts import write_static_estimate
from robot_control.cli import main
from robot_control.identification import StaticEstimate
from robot_control.profile import load_builtin_profile
from robot_control.track import CanonicalTrack


GROUP = "openarm_right_arm"
TIP = "openarm_right_hand_tcp"
STEP = 1e-3
SAMPLES = 3000
KP = np.linspace(20.0, 8.0, 7)
DAMPING = np.linspace(1.5, 0.4, 7)
FRICTION = np.linspace(0.4, 0.1, 7)
INERTIA = np.linspace(0.35, 0.05, 7)


def _urdf():
    parts = ['<robot name="stub">', '<link name="base"/>']
    parent = "base"
    for index in range(1, 8):
        link = f"openarm_right_link{index}"
        origin = "0 0 0" if index == 1 else "0.12 0 0"
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
    parts.append(f'<link name="{TIP}"/></robot>')
    return "".join(parts)


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


@pytest.fixture
def urdf(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_text(_urdf())
    return path


@pytest.fixture
def track(monkeypatch, profile):
    """A track whose standing load is 40% of what the URDF describes.

    That is the real failure: a hand the model gets wrong, so the gravity
    route to the inertia lands somewhere the stiffness route does not.
    """
    from robot_control.kinematics import chain_from_urdf

    chain = chain_from_urdf(
        _urdf(), [f"openarm_right_joint{i}" for i in range(1, 8)], TIP
    )
    clock = np.arange(SAMPLES) * STEP
    command = np.column_stack(
        [0.3 * np.sin(2 * np.pi * (0.6 + 0.2 * j) * clock) for j in range(7)]
    )
    measured = np.zeros((SAMPLES, 7))
    position, velocity = np.zeros(7), np.zeros(7)
    for index in range(SAMPLES):
        measured[index] = position
        acceleration = (
            KP * (command[index] - position)
            - DAMPING * velocity
            - FRICTION * np.sign(velocity)
            - 0.4 * chain.gravity_torque(position)
        ) / INERTIA
        velocity = velocity + STEP * acceleration
        position = position + STEP * velocity
    canonical = CanonicalTrack(
        (clock * 1e9).astype(np.int64),
        command,
        measured,
        tuple(profile.groups[GROUP].joints),
    )
    monkeypatch.setattr("robot_control.cli.read_hdf5", lambda _p: canonical)
    monkeypatch.setattr("robot_control.cli.track_sha256", lambda _t: "f" * 64)
    return canonical


def _static(path, profile):
    write_static_estimate(
        path,
        StaticEstimate(
            joint_names=profile.groups[GROUP].joints,
            stiffness=KP.copy(),
            torque_scale=np.ones(7),
            offset=np.zeros(7),
            residual_rmse=np.full(7, 1e-4),
            condition=np.full(7, 5.0),
            used=np.full(7, 12, dtype=int),
            excluded=np.zeros(7, dtype=int),
            unidentifiable=(),
            coulomb_nm=FRICTION * INERTIA,
            bias_nm=np.zeros(7),
        ),
        profile,
        group=GROUP,
        noise_rad=4e-4,
        sources=["a" * 64],
    )
    return path


def test_a_disagreeing_fit_is_refused_by_default(track, urdf, profile, tmp_path, capsys):
    output = tmp_path / "estimate.json"

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(output),
         "--static", str(_static(tmp_path / "s.json", profile)), "--urdf", str(urdf)]
    )

    assert code == 3, capsys.readouterr().out
    assert "refused" in capsys.readouterr().out
    assert not output.exists()


def test_accepting_the_gap_writes_the_fit_and_records_the_gap(
    track, urdf, profile, tmp_path, capsys
):
    output = tmp_path / "estimate.json"

    code = main(
        ["r2s", "fit", "--track", "t.h5", "--output", str(output),
         "--static", str(_static(tmp_path / "s.json", profile)), "--urdf", str(urdf),
         "--accept-inertia-gap"]
    )

    out = capsys.readouterr().out
    assert code == 0, out
    assert "accepted" in out
    payload = json.loads(output.read_text())
    # The stiffness route is the one that survives, and it is the measured one.
    np.testing.assert_allclose(payload["stiffness_nm_per_rad"], KP, rtol=1e-9)
    np.testing.assert_allclose(payload["inertia_kg_m2"], INERTIA, rtol=0.1)
    # The gap goes on the record rather than being smoothed away.
    assert max(payload["inertia_disagreement"]) > 0.25
    assert payload["inertia_gap_accepted"] is True
