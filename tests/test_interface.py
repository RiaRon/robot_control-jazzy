import numpy as np
import pytest

from robot_control.interface import CanonicalInterface, InterfaceError
from robot_control.profile import load_builtin_profile


def test_fake_hardware_canonical_source_round_trip():
    profile = load_builtin_profile("openarm_tesollo")
    interface = CanonicalInterface(profile)
    canonical = np.linspace(-0.01, 0.01, len(profile.joints))

    source = interface.command_to_source(canonical)
    restored = interface.state_to_canonical(source)

    assert list(source) == [joint.source for joint in profile.joints]
    np.testing.assert_allclose(restored, canonical)


def test_state_requires_exact_joint_coverage():
    interface = CanonicalInterface(load_builtin_profile("openarm_tesollo"))
    with pytest.raises(InterfaceError, match="coverage"):
        interface.state_to_canonical({"openarm_right_joint1": 0.0})
