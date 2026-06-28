"""
Background system-health monitor for the web dashboard.

A single :class:`HealthMonitor` polls the connected robot's subsystems on a
fixed cadence and caches a small status report that ``GET /api/health`` returns.
Running checks in a background thread (instead of on each request) gives camera
freeze detection a stable cadence and a remembered previous frame, and keeps the
HTTP path free of robot calls so concurrent browser tabs never race.

Camera freeze detection compares consecutive captured frames byte-for-byte: a
live sensor always injects per-pixel noise, so byte-identical frames mean the
feed is stuck (the recurring ZED green-frame fault). This only holds if the poll
interval exceeds the camera's healthy inter-frame time -- the default zenoh
camera streams well above 1 Hz, so ``_POLL_INTERVAL_S`` has large margin.
"""

from __future__ import annotations

import hashlib
import threading
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional

if TYPE_CHECKING:
    from walkie_sdk.web.state import RobotSession

# How often the background thread runs the checks. Must stay comfortably above
# the camera's healthy frame period or a healthy feed reads as "frozen".
_POLL_INTERVAL_S = 1.5

# Declare the camera frozen once its frame hash is unchanged across this many
# consecutive polls (so ~ (N-1) * _POLL_INTERVAL_S seconds of no change). Never
# fires on the first sighting of a frame.
_FREEZE_THRESHOLD = 3

# Status strings shared with the frontend.
OK = "ok"
FAIL = "fail"
UNKNOWN = "unknown"

# (key, label) for every monitored subsystem, in display order.
_SUBSYSTEMS = [
    ("nav", "Navigation"),
    ("lift", "Lift"),
    ("head", "Head Tilt"),
    ("arm", "Arm"),
    ("camera", "Camera"),
]


class HealthMonitor:
    """
    Owns cross-poll state for subsystem health (notably camera freeze state).

    One instance per :class:`~walkie_sdk.web.state.RobotSession`. ``poll_once``
    is pure and directly unit-testable; the background thread is just a loop
    around it.
    """

    def __init__(
        self,
        session: "RobotSession",
        *,
        interval: float = _POLL_INTERVAL_S,
        freeze_threshold: int = _FREEZE_THRESHOLD,
    ) -> None:
        self._session = session
        self._interval = interval
        self._freeze_threshold = freeze_threshold
        self._lock = threading.Lock()
        self._report: Dict[str, Any] = _disconnected_report()
        # Per-camera freeze state: name -> {"last_hash": str|None, "repeat": int}
        self._cam_state: Dict[str, Dict[str, Any]] = {}
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # ── lifecycle ───────────────────────────────────────────────────────
    def start(self) -> None:
        """Spawn the background poll thread (idempotent)."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="HealthMonitor", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the background poll thread (idempotent)."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.poll_once()
            except Exception:
                pass  # a transient robot error must never kill the monitor
            self._stop.wait(self._interval)

    # ── checks ──────────────────────────────────────────────────────────
    def poll_once(self) -> Dict[str, Any]:
        """Run every subsystem check, refresh freeze state, cache + return the report."""
        robot = self._session.robot
        if robot is None or not _safe(lambda: self._session.is_connected):
            self._cam_state.clear()
            report = _disconnected_report()
        else:
            checks = [
                self._check_nav(robot),
                self._check_lift(robot),
                self._check_head(robot),
                self._check_arm(robot),
                self._check_camera(robot),
            ]
            report = {
                "connected": True,
                "checks": checks,
                "any_failed": any(c["status"] == FAIL for c in checks),
            }
        with self._lock:
            self._report = report
        return report

    def latest(self) -> Dict[str, Any]:
        """Return the most recently computed report."""
        with self._lock:
            return self._report

    # ── per-subsystem checks ────────────────────────────────────────────
    # Each getter is a non-blocking cached read; None (or any exception, caught
    # by _safe) means the subsystem has no live data → FAIL.
    def _check_nav(self, robot: Any) -> Dict[str, Any]:
        ok = _safe(lambda: robot.status.get_position()) is not None
        return _row("nav", "Navigation", ok, "" if ok else "No odometry received")

    def _check_lift(self, robot: Any) -> Dict[str, Any]:
        ok = _safe(lambda: robot.lift.get()) is not None
        return _row("lift", "Lift", ok, "" if ok else "No lift position")

    def _check_head(self, robot: Any) -> Dict[str, Any]:
        ok = _safe(lambda: robot.head.get_angle()) is not None
        return _row("head", "Head Tilt", ok, "" if ok else "No head angle")

    def _check_arm(self, robot: Any) -> Dict[str, Any]:
        # get_joint_states() is the non-blocking hub read; get_ee_pose() is a
        # 5 s blocking service call and unsuitable for polling.
        ok = _safe(lambda: robot.arm.get_joint_states()) is not None
        return _row("arm", "Arm", ok, "" if ok else "No joint states")

    def _check_camera(self, robot: Any) -> Dict[str, Any]:
        names = _camera_names(robot)
        if not names:
            self._cam_state.clear()
            return _row("camera", "Camera", False, "No camera configured")

        # Fast path: whole transport reports nothing streaming.
        cam_obj = robot.cameras if robot.cameras is not None else robot.camera
        if _safe(lambda: cam_obj.is_streaming) is False:
            self._cam_state.clear()
            return _row("camera", "Camera", False, "not streaming")

        missing: List[str] = []
        frozen: List[str] = []
        for name in names:
            frame = _safe(lambda n=name: _grab_frame(robot, n))
            if frame is None:
                missing.append(name)
                self._cam_state[name] = {"last_hash": None, "repeat": 0}
                continue
            digest = hashlib.sha1(frame.tobytes()).hexdigest()
            state = self._cam_state.setdefault(name, {"last_hash": None, "repeat": 0})
            if state["last_hash"] == digest:
                state["repeat"] += 1
            else:
                state["repeat"] = 0
            state["last_hash"] = digest
            # repeat counts *additional* identical sightings, so +1 for the first.
            if state["repeat"] + 1 >= self._freeze_threshold:
                frozen.append(name)

        if missing or frozen:
            bits = []
            if missing:
                bits.append("no frame: " + ", ".join(missing))
            if frozen:
                bits.append("frozen: " + ", ".join(frozen))
            return _row("camera", "Camera", False, "; ".join(bits))
        return _row("camera", "Camera", True, "")


# ── module helpers ──────────────────────────────────────────────────────
def _camera_names(robot: Any) -> List[str]:
    """Names of the robot's cameras, or [] if none are configured."""
    if getattr(robot, "cameras", None) is not None:
        return list(robot.cameras.camera_names)
    if getattr(robot, "camera", None) is not None:
        return ["__single__"]
    return []


def _grab_frame(robot: Any, name: str) -> Any:
    """Return a BGR frame from the named camera, or None (mirrors app._grab_frame)."""
    if robot.cameras is not None:
        return robot.cameras.get_frame(name)
    if robot.camera is not None:
        return robot.camera.get_frame()
    return None


def _row(key: str, label: str, ok: bool, detail: str) -> Dict[str, Any]:
    return {"key": key, "label": label, "status": OK if ok else FAIL, "detail": detail}


def _disconnected_report() -> Dict[str, Any]:
    """All subsystems unknown (grey, no alarm) -- the not-connected state."""
    return {
        "connected": False,
        "checks": [
            {"key": key, "label": label, "status": UNKNOWN, "detail": ""}
            for key, label in _SUBSYSTEMS
        ],
        "any_failed": False,
    }


def _safe(fn: Callable[[], Any]) -> Any:
    """Call ``fn`` and swallow any error, returning None instead."""
    try:
        return fn()
    except Exception:
        return None
