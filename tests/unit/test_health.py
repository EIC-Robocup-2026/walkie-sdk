"""
Offline unit tests for the web health monitor (walkie_sdk.web.health).

These drive ``HealthMonitor.poll_once()`` directly with fake session/robot
objects -- no FastAPI, no background thread, no real robot. The focus is the
camera freeze-detection state machine and the tri-state per-subsystem logic.
"""

import numpy as np

from walkie_sdk.web.health import FAIL, OK, UNKNOWN, HealthMonitor


# ── Fakes ───────────────────────────────────────────────────────────────
class FakeCameras:
    """Yields a scripted sequence of frames; clamps to the last once exhausted."""

    camera_names = ["head"]
    is_streaming = True

    def __init__(self, frames):
        self._frames = list(frames)
        self._i = 0

    def get_frame(self, name="head"):
        if not self._frames:
            return None
        frame = self._frames[min(self._i, len(self._frames) - 1)]
        self._i += 1
        return frame


class FakeStatus:
    def get_position(self):
        return {"x": 0.0, "y": 0.0, "heading": 0.0}


class FakeLift:
    def get(self, norm_pos=True):
        return 0.5


class FakeHead:
    def get_angle(self):
        return 0.0


class FakeArm:
    def get_joint_states(self):
        return {"left_arm": {}}


class FakeRobot:
    def __init__(self, frames):
        self.status = FakeStatus()
        self.lift = FakeLift()
        self.head = FakeHead()
        self.arm = FakeArm()
        self.cameras = FakeCameras(frames)
        self.camera = None


class FakeSession:
    def __init__(self, robot):
        self._robot = robot

    @property
    def robot(self):
        return self._robot

    @property
    def is_connected(self):
        return self._robot is not None


def _check(report, key):
    return next(c for c in report["checks"] if c["key"] == key)


_ZEROS = np.zeros((4, 4, 3), dtype=np.uint8)
_ONES = np.ones((4, 4, 3), dtype=np.uint8)


# ── Camera freeze detection ─────────────────────────────────────────────
def test_camera_ok_on_first_poll():
    # A single identical frame is not enough history to call it frozen.
    mon = HealthMonitor(FakeSession(FakeRobot([_ZEROS])), freeze_threshold=2)
    assert _check(mon.poll_once(), "camera")["status"] == OK


def test_camera_frozen_after_threshold():
    mon = HealthMonitor(FakeSession(FakeRobot([_ZEROS])), freeze_threshold=2)
    mon.poll_once()  # first sighting -> ok
    report = mon.poll_once()  # same frame again -> frozen
    cam = _check(report, "camera")
    assert cam["status"] == FAIL
    assert "frozen" in cam["detail"]
    assert report["any_failed"] is True


def test_camera_recovers_when_frame_changes():
    robot = FakeRobot([_ZEROS, _ZEROS, _ONES])
    mon = HealthMonitor(FakeSession(robot), freeze_threshold=2)
    mon.poll_once()  # zeros (ok)
    assert _check(mon.poll_once(), "camera")["status"] == FAIL  # zeros again -> frozen
    assert _check(mon.poll_once(), "camera")["status"] == OK    # ones -> recovered


def test_camera_no_frame_is_fail():
    mon = HealthMonitor(FakeSession(FakeRobot([None])))
    cam = _check(mon.poll_once(), "camera")
    assert cam["status"] == FAIL
    assert "no frame" in cam["detail"]


# ── Tri-state subsystem logic ───────────────────────────────────────────
def test_disconnected_all_unknown():
    report = HealthMonitor(FakeSession(None)).poll_once()
    assert all(c["status"] == UNKNOWN for c in report["checks"])
    assert report["any_failed"] is False


def test_getter_none_or_raises_is_fail():
    class NoneStatus:
        def get_position(self):
            return None

    class RaisingLift:
        def get(self, norm_pos=True):
            raise RuntimeError("boom")

    robot = FakeRobot([_ZEROS])
    robot.status = NoneStatus()
    robot.lift = RaisingLift()
    report = HealthMonitor(FakeSession(robot)).poll_once()
    st = {c["key"]: c["status"] for c in report["checks"]}
    assert st["nav"] == FAIL   # getter returned None
    assert st["lift"] == FAIL  # getter raised -> caught -> fail (not unknown)
    assert st["head"] == OK
    assert st["arm"] == OK


def test_disconnect_resets_freeze_state():
    same = _ZEROS
    session = FakeSession(FakeRobot([same]))
    mon = HealthMonitor(session, freeze_threshold=2)
    mon.poll_once()
    assert _check(mon.poll_once(), "camera")["status"] == FAIL  # frozen

    session._robot = None
    mon.poll_once()  # disconnected -> freeze state cleared

    # Reconnect to a fresh (still static) feed: first poll must read OK again,
    # proving the per-camera counter was reset rather than carried over.
    session._robot = FakeRobot([same])
    assert _check(mon.poll_once(), "camera")["status"] == OK
