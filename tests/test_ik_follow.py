import threading
import time

import numpy as np
import pytest

from robot_control.ros_adapter import Pose


def _wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.001)
    raise AssertionError("condition did not become true")


def _pose(x):
    return Pose((x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "world")


def test_latest_ik_worker_latches_a_successful_solution():
    from robot_control.ik_follow import LatestIkWorker

    worker = LatestIkWorker(
        lambda pose, seed: np.asarray(seed) + pose.position[0]
    )
    try:
        worker.submit(_pose(0.25), np.array([1.0, 2.0]))
        status = _wait_until(lambda: worker.snapshot().succeeded == 1 and worker.snapshot())

        np.testing.assert_allclose(status.target, [1.25, 2.25])
        assert status.target_sequence == 1
        assert status.submitted == 1
        assert status.failed == 0
        assert status.superseded == 0
        assert len(status.timings) == 1
        timing = status.timings[0]
        assert timing.sequence == 1
        assert timing.outcome == "accepted"
        assert timing.started_at_sec >= timing.requested_at_sec
        assert timing.completed_at_sec >= timing.started_at_sec
        assert timing.accepted_at_sec == timing.completed_at_sec
    finally:
        worker.close()


def test_latest_ik_worker_discards_pending_and_inflight_stale_results():
    from robot_control.ik_follow import LatestIkWorker

    entered = threading.Event()
    release = threading.Event()
    solved = []

    def solve(pose, seed):
        solved.append(pose.position[0])
        if len(solved) == 1:
            entered.set()
            assert release.wait(1.0)
        return np.array([pose.position[0]])

    worker = LatestIkWorker(solve)
    try:
        worker.submit(_pose(1.0), np.array([0.0]))
        assert entered.wait(1.0)
        worker.submit(_pose(2.0), np.array([0.0]))
        worker.submit(_pose(3.0), np.array([0.0]))
        release.set()

        status = _wait_until(
            lambda: worker.snapshot().succeeded == 1 and worker.snapshot()
        )
        np.testing.assert_allclose(status.target, [3.0])
        assert status.target_sequence == 3
        assert solved == [1.0, 3.0]
        assert status.submitted == 3
        assert status.superseded == 2
        assert [timing.outcome for timing in status.timings] == [
            "superseded_after_complete",
            "superseded_before_start",
            "accepted",
        ]
        assert status.timings[1].completed_at_sec is None
    finally:
        worker.close()


def test_latest_ik_worker_keeps_last_target_when_a_new_solve_fails():
    from robot_control.ik_follow import LatestIkWorker

    def solve(pose, seed):
        if pose.position[0] == 2.0:
            raise RuntimeError("no solution")
        return np.array([pose.position[0]])

    worker = LatestIkWorker(solve)
    try:
        worker.submit(_pose(1.0), np.array([0.0]))
        _wait_until(lambda: worker.snapshot().succeeded == 1)
        worker.submit(_pose(2.0), np.array([0.0]))
        status = _wait_until(lambda: worker.snapshot().failed == 1 and worker.snapshot())

        np.testing.assert_allclose(status.target, [1.0])
        assert status.target_sequence == 1
        assert status.succeeded == 1
    finally:
        worker.close()


def test_latest_ik_worker_copies_submitted_seed_and_returned_target():
    from robot_control.ik_follow import LatestIkWorker

    entered = threading.Event()
    release = threading.Event()

    def solve(_pose, seed):
        entered.set()
        assert release.wait(1.0)
        return seed

    worker = LatestIkWorker(solve)
    try:
        seed = np.array([0.1, 0.2])
        worker.submit(_pose(0.0), seed)
        assert entered.wait(1.0)
        seed[:] = 9.0
        release.set()
        status = _wait_until(lambda: worker.snapshot().succeeded == 1 and worker.snapshot())
        target = status.target
        target[:] = 8.0

        np.testing.assert_allclose(worker.snapshot().target, [0.1, 0.2])
    finally:
        worker.close()


def test_latest_ik_worker_retries_a_branch_jump_from_the_same_seed():
    from robot_control.ik_follow import LatestIkWorker

    unsafe = np.array([3.1559, 3.1269, 1.5527, 0.0, 1.5889, 0.0, 0.0])
    continuous = np.array([0.02, -0.01, 0.01, 0.0, -0.02, 0.0, 0.0])
    returned = [unsafe, continuous]
    seen_seeds = []

    def solve(_pose, seed):
        seen_seeds.append(np.asarray(seed, dtype=float).copy())
        return returned.pop(0)

    worker = LatestIkWorker(
        solve,
        max_target_jump_rad=0.30,
        max_continuity_attempts=4,
    )
    try:
        worker.submit(_pose(0.01), np.zeros(7))
        status = _wait_until(
            lambda: worker.snapshot().succeeded == 1
            and worker.snapshot()
        )

        np.testing.assert_allclose(status.target, continuous)
        assert status.target_sequence == 1
        assert status.solve_attempts == 4
        assert status.continuity_rejected == 1
        assert status.continuity_retries == 3
        assert status.continuity_exhausted == 0
        assert status.continuity_refusal is None
        assert len(status.continuity_rejections) == 1
        assert not status.continuity_rejections[0].exhausted
        assert len(seen_seeds) == 4
        for seed in seen_seeds:
            np.testing.assert_array_equal(seed, np.zeros(7))
    finally:
        worker.close()


def test_latest_ik_worker_exhausts_retries_and_keeps_previous_target():
    from robot_control.ik_follow import LatestIkWorker

    jump = np.array([3.1559, 3.1269, 1.5527, 0.0, 1.5889, 0.0, 0.0])
    seen_jump_seeds = []

    def solve(pose, seed):
        seed = np.asarray(seed, dtype=float)
        if pose.position[0] == 1.0:
            return seed + 0.01
        seen_jump_seeds.append(seed.copy())
        return seed + jump

    worker = LatestIkWorker(
        solve,
        max_target_jump_rad=0.30,
        max_continuity_attempts=4,
    )
    try:
        worker.submit(_pose(1.0), np.zeros(7))
        first = _wait_until(
            lambda: worker.snapshot().succeeded == 1
            and worker.snapshot()
        )
        np.testing.assert_allclose(first.target, np.full(7, 0.01))

        # The caller supplies a different measured state, but continuity mode
        # must seed every retry from the previous accepted joint solution.
        worker.submit(_pose(2.0), np.full(7, -0.2))
        status = _wait_until(
            lambda: worker.snapshot().continuity_exhausted == 1
            and worker.snapshot()
        )

        np.testing.assert_allclose(status.target, np.full(7, 0.01))
        assert status.target_sequence == 1
        assert status.succeeded == 1
        assert status.failed == 0
        assert status.solve_attempts == 8
        assert status.continuity_rejected == 4
        assert status.continuity_retries == 3
        assert status.continuity_refusal.sequence == 2
        assert status.continuity_refusal.attempt == 4
        assert status.timings[1].outcome == "continuity_refused"
        assert len(seen_jump_seeds) == 4
        for seed in seen_jump_seeds:
            np.testing.assert_allclose(seed, np.full(7, 0.01))
    finally:
        worker.close()


def test_continuity_retry_solver_errors_still_end_in_safe_exhaustion():
    from robot_control.ik_follow import LatestIkWorker

    jump = np.array([3.1559, 3.1269, 1.5527, 0.0, 1.5889, 0.0, 0.0])
    calls = 0

    def solve(_pose, seed):
        nonlocal calls
        calls += 1
        if calls == 1:
            return np.asarray(seed, dtype=float) + jump
        raise RuntimeError("retry solver failure")

    worker = LatestIkWorker(
        solve,
        max_target_jump_rad=0.30,
        max_continuity_attempts=4,
    )
    try:
        worker.submit(_pose(0.01), np.zeros(7))
        status = _wait_until(
            lambda: worker.snapshot().continuity_exhausted == 1
            and worker.snapshot()
        )

        assert status.target is None
        assert status.succeeded == 0
        assert status.failed == 0
        assert status.solve_attempts == 4
        assert status.continuity_rejected == 1
        assert status.continuity_retries == 3
        assert status.continuity_refusal.attempt == 4
        np.testing.assert_allclose(
            status.continuity_refusal.joint_delta_rad, jump
        )
    finally:
        worker.close()



def test_closest_candidate_wins_among_multiple_continuous_solutions():
    from robot_control.ik_follow import LatestIkWorker

    returned = iter(
        [
            np.array([0.12, 0.0]),
            np.array([0.02, 0.01]),
            np.array([-0.05, 0.0]),
            np.array([0.08, 0.0]),
        ]
    )
    worker = LatestIkWorker(
        lambda _pose, _seed: next(returned),
        max_target_jump_rad=0.30,
        max_continuity_attempts=4,
        joint_weights=[1.0, 4.0],
    )
    try:
        worker.submit(_pose(0.0), np.zeros(2))
        status = _wait_until(lambda: worker.snapshot().succeeded == 1 and worker.snapshot())
        np.testing.assert_allclose(status.target, [0.02, 0.01])
        selection = status.selections[0]
        assert selection.selected_candidate == 2
        assert selection.selected_cost == pytest.approx(np.sqrt(0.0008))
        assert len(selection.candidates) == 4
    finally:
        worker.close()


def test_equal_cost_uses_lexicographic_deterministic_tie_break():
    from robot_control.ik_follow import LatestIkWorker

    returned = iter(
        [np.array([0.1]), np.array([-0.1]), np.array([0.2]), np.array([-0.2])]
    )
    worker = LatestIkWorker(
        lambda _pose, _seed: next(returned),
        max_target_jump_rad=0.30,
        max_continuity_attempts=4,
    )
    try:
        worker.submit(_pose(0.0), np.zeros(1))
        status = _wait_until(lambda: worker.snapshot().succeeded == 1 and worker.snapshot())
        np.testing.assert_allclose(status.target, [-0.1])
        assert status.selections[0].selected_candidate == 2
    finally:
        worker.close()


def test_continuous_joint_uses_wrapped_shortest_delta():
    from robot_control.ik_follow import joint_delta

    delta = joint_delta(
        np.array([-np.pi + 0.01, 0.2]),
        np.array([np.pi - 0.01, 0.1]),
        [True, False],
    )
    np.testing.assert_allclose(delta, [0.02, 0.1], atol=1e-12)


def test_closest_selection_is_reproducible_for_50_independent_runs():
    from robot_control.ik_follow import LatestIkWorker

    selected = []
    for _ in range(50):
        returned = iter(
            [np.array([0.08]), np.array([-0.03]), np.array([0.01]), np.array([0.04])]
        )
        worker = LatestIkWorker(
            lambda _pose, _seed: next(returned),
            max_target_jump_rad=0.30,
            max_continuity_attempts=4,
        )
        try:
            worker.submit(_pose(0.0), np.zeros(1))
            status = _wait_until(lambda: worker.snapshot().succeeded == 1 and worker.snapshot())
            selected.append(float(status.target[0]))
        finally:
            worker.close()
    np.testing.assert_array_equal(selected, np.full(50, 0.01))


def test_candidate_batch_stops_at_latency_cap_between_solves():
    from robot_control.ik_follow import LatestIkWorker

    now = [0.0]

    def clock():
        return now[0]

    def solve(_pose, seed):
        now[0] += 0.06
        return np.asarray(seed, dtype=float) + 0.01

    worker = LatestIkWorker(
        solve,
        clock=clock,
        max_target_jump_rad=0.30,
        max_continuity_attempts=4,
        max_batch_latency_sec=0.05,
    )
    try:
        worker.submit(_pose(0.0), np.zeros(2))
        status = _wait_until(
            lambda: worker.snapshot().succeeded == 1 and worker.snapshot()
        )
        assert status.solve_attempts == 1
        assert status.candidate_count == 1
        assert status.selections[0].batch_latency_sec == pytest.approx(0.06)
    finally:
        worker.close()
