import threading
import time

import numpy as np

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
