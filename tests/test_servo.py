import numpy as np
import pytest

from robot_control.servo import CartesianPI, ServoError


@pytest.mark.parametrize(
    "kwargs",
    [
        {"kp": 0.0},
        {"kp": float("nan")},
        {"ki": -1.0},
        {"tolerance": 0.0},
        {"max_speed": float("inf")},
    ],
)
def test_cartesian_pi_rejects_invalid_settings(kwargs):
    settings = dict(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)
    settings.update(kwargs)
    with pytest.raises(ServoError):
        CartesianPI(**settings)


def test_cartesian_pi_limits_velocity_norm_to_fifty_mm_per_second():
    servo = CartesianPI(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)

    step = servo.update(np.zeros(3), np.array([1.0, 0.0, 0.0]), 0.01)

    assert np.linalg.norm(step.velocity) == pytest.approx(0.05)
    assert step.speed_limited


def test_integral_eliminates_a_constant_position_droop():
    servo = CartesianPI(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)
    measured = np.zeros(3)
    command = measured.copy()
    target = np.array([0.04, 0.0, 0.0])
    droop = np.array([0.01, 0.0, 0.0])

    for _ in range(1000):
        step = servo.update(measured, target, 0.01)
        command = command + step.velocity * 0.01
        measured = command - droop
        servo.commit(joint_limited=False)

    assert abs(target[0] - measured[0]) <= 0.002


def test_large_target_change_resets_integral():
    servo = CartesianPI(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)
    for _ in range(10):
        servo.update(np.zeros(3), np.array([0.004, 0.0, 0.0]), 0.01)
        servo.commit(joint_limited=False)

    step = servo.update(np.zeros(3), np.array([0.020, 0.0, 0.0]), 0.01)
    servo.commit(joint_limited=False)

    assert step.target_reset
    assert not step.integral_frozen
    np.testing.assert_allclose(servo.integral, np.zeros(3))


def test_joint_limit_rolls_back_pending_integral():
    servo = CartesianPI(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)
    first = servo.update(np.zeros(3), np.array([0.004, 0.0, 0.0]), 0.01)
    servo.commit(joint_limited=True)

    second = servo.update(np.zeros(3), np.array([0.004, 0.0, 0.0]), 0.01)

    np.testing.assert_allclose(second.velocity, first.velocity)


def test_deadband_holds_zero_and_external_error_restarts_control():
    servo = CartesianPI(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)

    inside = servo.update(np.zeros(3), np.array([0.001, 0.0, 0.0]), 0.01)

    assert inside.within_tolerance
    assert not inside.integral_frozen
    np.testing.assert_allclose(inside.velocity, np.zeros(3))

    outside = servo.update(np.zeros(3), np.array([0.003, 0.0, 0.0]), 0.01)

    assert not outside.within_tolerance
    assert outside.velocity[0] > 0.0


@pytest.mark.parametrize(
    ("measured", "target", "dt"),
    [
        ([0.0, 0.0], [0.0, 0.0, 0.0], 0.01),
        ([0.0, 0.0, 0.0], [0.0, 0.0, float("nan")], 0.01),
        ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0),
    ],
)
def test_cartesian_pi_rejects_invalid_runtime_inputs(measured, target, dt):
    servo = CartesianPI(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)

    with pytest.raises(ServoError):
        servo.update(measured, target, dt)


def test_zero_integral_gain_never_accumulates_hidden_state():
    servo = CartesianPI(kp=2.0, ki=0.0, tolerance=0.0005, max_speed=0.05)

    for _ in range(10_000):
        servo.update(np.zeros(3), np.array([0.001, 0.0, 0.0]), 0.01)
        servo.commit(joint_limited=False)

    np.testing.assert_allclose(servo.integral, np.zeros(3))


def test_integral_is_bounded_per_axis_during_a_long_hold():
    servo = CartesianPI(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)

    for _ in range(10_000):
        servo.update(np.zeros(3), np.array([0.003, -0.003, 0.003]), 0.01)
        servo.commit(joint_limited=False)

    assert np.all(np.abs(servo.integral) <= 0.05)


def test_saturated_controller_can_unwind_an_integral_against_the_error():
    servo = CartesianPI(kp=2.0, ki=1.0, tolerance=0.002, max_speed=0.05)
    for _ in range(100):
        servo.update(np.zeros(3), np.array([0.004, 0.0, 0.0]), 0.01)
        servo.commit(joint_limited=False)
    before = servo.integral[0]

    servo.update(np.array([0.1, 0.0, 0.0]), np.array([0.004, 0.0, 0.0]), 0.01)
    servo.commit(joint_limited=False)

    assert servo.integral[0] < before
