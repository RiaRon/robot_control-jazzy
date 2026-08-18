import math

import numpy as np
import pytest

from robot_control.diagnostic_profile import DiagnosticProfile


ORIGIN_POSITION = np.array([0.1, -0.2, 0.3])
ORIGIN_ORIENTATION = (0.0, 0.0, 0.0, 1.0)


def test_translation_profile_is_a_deterministic_round_trip():
    profile = DiagnosticProfile(
        kind="translation",
        distance_m=0.01,
        linear_speed_m_s=0.005,
        hold_sec=1.0,
        translation_axis="y",
    )

    assert profile.duration_sec == pytest.approx(6.0)
    halfway_out = profile.sample(
        1.0, ORIGIN_POSITION, ORIGIN_ORIENTATION
    )
    np.testing.assert_allclose(
        halfway_out.position, [0.1, -0.195, 0.3]
    )
    assert halfway_out.phase == "translation_ramp_out"

    held = profile.sample(2.5, ORIGIN_POSITION, ORIGIN_ORIENTATION)
    np.testing.assert_allclose(held.position, [0.1, -0.19, 0.3])
    assert held.phase == "translation_hold"

    complete = profile.sample(6.1, ORIGIN_POSITION, ORIGIN_ORIENTATION)
    np.testing.assert_allclose(complete.position, ORIGIN_POSITION)
    assert complete.orientation == pytest.approx(ORIGIN_ORIENTATION)
    assert complete.complete


def test_rotation_profile_uses_the_startup_tcp_local_axis():
    profile = DiagnosticProfile(
        kind="rotation",
        angle_rad=math.radians(10.0),
        angular_speed_rad_s=0.1,
        hold_sec=1.0,
        rotation_axis="z",
    )

    peak = profile.sample(
        profile.angle_rad / profile.angular_speed_rad_s,
        ORIGIN_POSITION,
        ORIGIN_ORIENTATION,
    )
    assert peak.orientation[2] == pytest.approx(
        math.sin(math.radians(5.0))
    )
    assert peak.orientation[3] == pytest.approx(
        math.cos(math.radians(5.0))
    )
    np.testing.assert_allclose(peak.position, ORIGIN_POSITION)


def test_combined_profile_repeats_without_accumulating_offset():
    profile = DiagnosticProfile(
        kind="translation-rotation",
        distance_m=0.003,
        angle_rad=0.03,
        linear_speed_m_s=0.01,
        angular_speed_rad_s=0.1,
        hold_sec=0.1,
        repetitions=2,
    )

    complete = profile.sample(
        profile.duration_sec + 0.01,
        ORIGIN_POSITION,
        ORIGIN_ORIENTATION,
    )
    np.testing.assert_allclose(complete.position, ORIGIN_POSITION)
    assert complete.orientation == pytest.approx(ORIGIN_ORIENTATION)
    assert complete.repetition == 2
    assert complete.complete


@pytest.mark.parametrize(
    "kwargs",
    [
        {"distance_m": 0.031},
        {"angle_rad": math.radians(10.1)},
        {"linear_speed_m_s": 0.021},
        {"angular_speed_rad_s": 0.101},
        {"repetitions": 4},
    ],
)
def test_profile_refuses_values_outside_the_conservative_envelope(kwargs):
    with pytest.raises(ValueError):
        DiagnosticProfile(**kwargs)
