"""Feed the exported file to hdgp's own loader, not to our idea of it.

Everything else here tests the writer against a description of the reader. This
loads `real2sim_actuator_cfg.py` out of the hdgp checkout and runs it, so a
change on that side that our description has not caught shows up as a failure
rather than as a training run on default gains.
"""

import importlib.util
import os
import sys
from pathlib import Path

import numpy as np
import pytest

from robot_control.calibration import identified_block, write_bundle
from robot_control.hdgp_export import write_hdgp_calibration
from robot_control.identification import CombinedEstimate
from robot_control.profile import load_builtin_profile


GROUP = "tesollo_curl"
HDGP_GROUP = "tesollo_hand_curl"
LOADER = Path(
    "source/openarm/openarm/tesollo/right/grasp_adapt/real2sim_actuator_cfg.py"
)


def _hdgp_root() -> Path | None:
    explicit = os.environ.get("HDGP_ROOT")
    if explicit:
        return Path(explicit).expanduser()
    for ancestor in Path(__file__).resolve().parents:
        candidate = ancestor / "hdgp"
        if (candidate / LOADER).is_file():
            return candidate
    return None


@pytest.fixture(scope="module")
def hdgp_loader():
    root = _hdgp_root()
    if root is None or not (root / LOADER).is_file():
        pytest.skip("hdgp checkout not found beside this repository")
    spec = importlib.util.spec_from_file_location("hdgp_r2s", root / LOADER)
    module = importlib.util.module_from_spec(spec)
    # `@dataclass` resolves annotations through sys.modules; without this the
    # module's own dataclasses fail to build.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def exported(tmp_path):
    profile = load_builtin_profile("openarm_tesollo")
    joints = profile.groups[GROUP].joints
    width = len(joints)
    combined = CombinedEstimate(
        joint_names=joints,
        inertia=np.full(width, 0.02),
        damping=np.full(width, 4.5),
        friction=np.full(width, 0.15),
        stiffness=np.full(width, 26.0),
        inertia_from_gravity=np.full(width, 0.02),
        disagreement=np.zeros(width),
    )
    groups = {
        name: {"nominal": {"stiffness": 30.0, "damping": 5.0, "friction": 0.1}}
        for name in profile.groups
    }
    groups[GROUP]["identified"] = identified_block(
        combined,
        profile,
        torque_scale=np.ones(width).tolist(),
        sweep_sha256=["a" * 64],
        track_sha256="c" * 64,
    )
    bundle = write_bundle(
        tmp_path / "bundle.json",
        {
            "schema_version": 2,
            "profile": profile.name,
            "asset": {
                "id": profile.asset_id,
                "manifest_sha256": profile.manifest_sha256,
            },
            "controller": {
                "command_period_sec": 0.01,
                "delay_sec": 0.02,
                "interpolation": "linear",
                "filter": {"type": "none"},
            },
            "groups": groups,
        },
        profile,
    )
    path = tmp_path / "real2sim.json"
    write_hdgp_calibration(path, bundle, profile)
    return path


def test_hdgp_reads_the_export_and_applies_the_measurement(hdgp_loader, exported):
    calibration = hdgp_loader.load_real2sim_calibration(exported)

    params = hdgp_loader.get_actuator_params(HDGP_GROUP, calibration, 30.0, 5.0)

    # Not the 30.0 / 5.0 the env would have used on its own.
    assert params["stiffness"] == pytest.approx(26.0)
    assert params["damping"] == pytest.approx(4.5)
    assert params["friction"] == pytest.approx(0.15)


def test_an_unmeasured_group_keeps_the_envs_own_gain(hdgp_loader, exported):
    calibration = hdgp_loader.load_real2sim_calibration(exported)

    params = hdgp_loader.get_actuator_params(
        "tesollo_hand_abduction", calibration, 30.0, 5.0
    )

    assert params == {"stiffness": 30.0, "damping": 5.0}


def test_the_name_we_export_under_is_the_name_hdgps_env_declares(hdgp_loader):
    """The mapping's other half: what hdgp's env_cfg actually calls its groups.

    hdgp resolves an unknown group to the default silently, so a rename there
    would otherwise surface as a training run that ignored the calibration.
    """
    import ast

    root = _hdgp_root()
    env_cfg = root / LOADER.parent / "grasp_right_env_cfg.py"
    if not env_cfg.is_file():
        pytest.skip(f"env_cfg not found: {env_cfg}")
    declared = {
        ast.literal_eval(key)
        for node in ast.walk(ast.parse(env_cfg.read_text()))
        if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values)
        if isinstance(key, ast.Constant)
        and isinstance(value, ast.Call)
        and isinstance(value.func, ast.Name)
        and value.func.id == "ImplicitActuatorCfg"
    }
    if not declared:
        pytest.skip("no ImplicitActuatorCfg groups parsed from env_cfg")

    profile = load_builtin_profile("openarm_tesollo")
    exported_names = {
        group.hdgp_group for group in profile.groups.values() if group.hdgp_group
    }

    assert exported_names <= declared, sorted(exported_names - declared)
