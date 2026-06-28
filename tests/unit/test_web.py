"""
Offline unit tests for the web interface (walkie_sdk.web).

These never touch a real robot: the FastAPI app is driven through a fake
``RobotSession`` and a fake ``WalkieRobot`` that record calls. Skipped cleanly
if the optional web extras (fastapi/httpx) are not installed.
"""

import numpy as np
import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")

from fastapi.testclient import TestClient  # noqa: E402

from walkie_sdk.web.app import create_app  # noqa: E402
from walkie_sdk.web.state import RobotNotConnected, RobotSession  # noqa: E402


# ── Fakes ───────────────────────────────────────────────────────────────
class FakeNav:
    def __init__(self):
        self.calls = []
        self.status = None

    def go_to(self, **kw):
        self.calls.append(("go_to", kw))
        return "IN_PROGRESS"

    def cancel(self):
        self.calls.append(("cancel", {}))
        return True

    def stop(self):
        self.calls.append(("stop", {}))
        return True


class FakeStatus:
    def get_position(self):
        return {"x": 1.0, "y": 2.0, "heading": 0.5}

    def get_velocity(self):
        return {"linear": 0.1, "angular": 0.0}


class FakeLift:
    def __init__(self):
        self.calls = []
        self.status = "SUCCEEDED"

    def set(self, **kw):
        self.calls.append(kw)
        return "IN_PROGRESS"

    def get(self, norm_pos=True):
        return 0.5 if norm_pos else 37.0


class FakeArm:
    def __init__(self):
        self.calls = []

    def go_to_pose(self, **kw):
        self.calls.append(("pose", kw))
        return "SUCCEEDED"

    def go_to_home(self, group_name):
        self.calls.append(("home", group_name))
        return "SUCCEEDED"

    def control_gripper(self, **kw):
        self.calls.append(("gripper", kw))
        return "SUCCEEDED"

    def set_joint_positions(self, **kw):
        self.calls.append(("joints", kw))
        return True

    def get_joint_states(self):
        return {"left_gripper": 0.0}


class FakeHead:
    def get_angle(self):
        return 0.0


class FakeCameras:
    camera_names = ["head", "left"]
    is_streaming = True

    # Constant frame: this *is* the frozen signature, so any health check that
    # polls this more than `freeze_threshold` times will (correctly) go RED.
    def get_frame(self, name="head"):
        return np.zeros((4, 4, 3), dtype=np.uint8)


class FakeRobot:
    def __init__(self):
        self.is_connected = True
        self.ip = "10.0.0.1"
        self.namespace = ""
        self.ros_protocol = "rosbridge"
        self.camera_protocol = "zenoh"
        self.nav = FakeNav()
        self.status = FakeStatus()
        self.lift = FakeLift()
        self.arm = FakeArm()
        self.head = FakeHead()
        self.cameras = FakeCameras()
        self.camera = object()

    def disconnect(self):
        self.is_connected = False


class FakeSession:
    """Mimics RobotSession's surface used by app.py."""

    def __init__(self, robot=None):
        self._robot = robot

    @property
    def robot(self):
        return self._robot

    @property
    def is_connected(self):
        return bool(self._robot and self._robot.is_connected)

    def require(self):
        if not self.is_connected:
            raise RobotNotConnected()
        return self._robot

    def connect(self, **params):
        self._robot = FakeRobot()
        return self._robot

    def disconnect(self):
        self._robot = None

    def snapshot(self):
        if not self._robot:
            return {"connected": False}
        return {
            "connected": True,
            "ip": self._robot.ip,
            "ros_protocol": "rosbridge",
            "position": self._robot.status.get_position(),
            "velocity": self._robot.status.get_velocity(),
            "lift": 0.5,
            "lift_cm": 37.0,
            "cameras": ["head", "left"],
        }


@pytest.fixture
def connected_client():
    session = FakeSession(robot=FakeRobot())
    return TestClient(create_app(session)), session


@pytest.fixture
def offline_client():
    session = FakeSession(robot=None)
    return TestClient(create_app(session)), session


# ── Status / connection ─────────────────────────────────────────────────
def test_status_disconnected(offline_client):
    client, _ = offline_client
    r = client.get("/api/status")
    assert r.status_code == 200
    assert r.json() == {"connected": False}


def test_connect_then_status(offline_client):
    client, _ = offline_client
    r = client.post("/api/connect", json={"ip": "10.0.0.1"})
    assert r.status_code == 200
    assert r.json()["ok"] is True
    assert client.get("/api/status").json()["connected"] is True


def test_action_when_disconnected_returns_409(offline_client):
    client, _ = offline_client
    r = client.post("/api/nav/goto", json={"x": 1, "y": 2, "heading": 0})
    assert r.status_code == 409
    assert r.json()["error"] == "not_connected"


# ── Health ──────────────────────────────────────────────────────────────
# The route returns the monitor's *cached* report; the background poll thread
# isn't started in these tests (no lifespan), so we drive poll_once() directly.
def test_health_disconnected(offline_client):
    client, _ = offline_client
    client.app.state.health_monitor.poll_once()
    body = client.get("/api/health").json()
    assert all(c["status"] == "unknown" for c in body["checks"])
    assert body["any_failed"] is False


def test_health_connected_single_poll_ok(connected_client):
    client, _ = connected_client
    client.app.state.health_monitor.poll_once()  # one poll can't detect freeze
    body = client.get("/api/health").json()
    st = {c["key"]: c["status"] for c in body["checks"]}
    assert st["nav"] == "ok"
    assert st["lift"] == "ok"
    assert st["head"] == "ok"
    assert st["arm"] == "ok"
    assert st["camera"] == "ok"
    assert body["any_failed"] is False


def test_health_camera_freeze(connected_client):
    client, _ = connected_client
    monitor = client.app.state.health_monitor
    for _ in range(5):  # constant FakeCameras frame -> frozen
        monitor.poll_once()
    body = client.get("/api/health").json()
    cam = next(c for c in body["checks"] if c["key"] == "camera")
    assert cam["status"] == "fail"
    assert "frozen" in cam["detail"]
    assert body["any_failed"] is True


# ── Navigation ──────────────────────────────────────────────────────────
def test_nav_goto_forwards_args(connected_client):
    client, session = connected_client
    r = client.post("/api/nav/goto", json={"x": 1.5, "y": -2.0, "heading": 0.3})
    assert r.json() == {"ok": True, "result": "IN_PROGRESS"}
    name, kw = session._robot.nav.calls[-1]
    assert name == "go_to"
    assert kw == {"x": 1.5, "y": -2.0, "heading": 0.3, "blocking": False}


def test_nav_stop(connected_client):
    client, session = connected_client
    assert client.post("/api/nav/stop").json()["result"] is True
    assert ("stop", {}) in session._robot.nav.calls


# ── Lift ────────────────────────────────────────────────────────────────
def test_lift_set(connected_client):
    client, session = connected_client
    r = client.post("/api/lift/set", json={"pos": 0.8})
    assert r.json()["result"] == "IN_PROGRESS"
    assert session._robot.lift.calls[-1]["pos"] == 0.8
    assert session._robot.lift.calls[-1]["norm_pos"] is True


def test_lift_get(connected_client):
    client, _ = connected_client
    assert client.get("/api/lift").json()["norm"] == 0.5


# ── Arm + Gripper ───────────────────────────────────────────────────────
def test_arm_pose(connected_client):
    client, session = connected_client
    r = client.post(
        "/api/arm/pose",
        json={"x": 0.3, "y": 0.0, "z": 0.2, "group_name": "right_arm"},
    )
    assert r.json()["result"] == "SUCCEEDED"
    name, kw = session._robot.arm.calls[-1]
    assert name == "pose"
    assert kw["group_name"] == "right_arm"


def test_gripper(connected_client):
    client, session = connected_client
    # Default norm=True maps 0.7 → 0.028 m (70% of 0.04 m).
    r = client.post("/api/arm/gripper", json={"position": 0.7})
    assert r.json()["result"] == "SUCCEEDED"
    assert session._robot.arm.calls[-1][1]["position"] == pytest.approx(0.028)


def test_gripper_raw_meters(connected_client):
    client, session = connected_client
    r = client.post(
        "/api/arm/gripper", json={"position": 0.03, "norm": False}
    )
    assert r.json()["result"] == "SUCCEEDED"
    assert session._robot.arm.calls[-1][1]["position"] == 0.03


# ── Camera ──────────────────────────────────────────────────────────────
def test_camera_snapshot_returns_jpeg(connected_client):
    client, _ = connected_client
    r = client.get("/api/camera/snapshot?camera=head")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"
    assert r.content[:2] == b"\xff\xd8"  # JPEG magic


# ── Frontend ────────────────────────────────────────────────────────────
def test_index_served(offline_client):
    client, _ = offline_client
    r = client.get("/")
    assert r.status_code == 200
    assert "Walkie" in r.text


# ── Real RobotSession (no network) ──────────────────────────────────────
def test_session_connect_disconnect(monkeypatch):
    import walkie_sdk.web.state as state

    monkeypatch.setattr(state, "WalkieRobot", lambda **kw: FakeRobot())
    s = RobotSession()
    assert s.is_connected is False
    with pytest.raises(RobotNotConnected):
        s.require()

    s.connect(ip="1.2.3.4", namespace="r1")
    assert s.is_connected is True
    assert s.require().ip == "10.0.0.1"
    snap = s.snapshot()
    assert snap["connected"] is True and snap["namespace"] == "r1"

    s.disconnect()
    assert s.is_connected is False
