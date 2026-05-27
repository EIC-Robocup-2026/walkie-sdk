"""
Server-side connection state for the web interface.

A single ``RobotSession`` owns at most one live ``WalkieRobot`` connection.
All HTTP requests share it, so connect/disconnect is guarded by a lock and
every robot call is funnelled through :meth:`RobotSession.require`.
"""

from __future__ import annotations

import threading
from typing import Any, Dict, Optional

from walkie_sdk.robot import WalkieRobot


class RobotNotConnected(Exception):
    """Raised when a robot action is attempted with no live connection."""


class RobotSession:
    """
    Holds the single server-side robot connection.

    The web app is a thin remote control: the browser never talks to the robot
    directly, it asks this session which owns the underlying ``WalkieRobot``.
    Connect/disconnect mutate shared state, so they take ``_lock``; read-only
    accessors (:meth:`require`, :meth:`snapshot`) rely on the GIL and the
    modules' own internal locks.
    """

    def __init__(self) -> None:
        self._robot: Optional[WalkieRobot] = None
        self._lock = threading.Lock()
        self._params: Dict[str, Any] = {}

    @property
    def robot(self) -> Optional[WalkieRobot]:
        """The current robot, or None if not connected."""
        return self._robot

    @property
    def is_connected(self) -> bool:
        """True if a robot is connected and its transport is live."""
        robot = self._robot
        return bool(robot and robot.is_connected)

    def require(self) -> WalkieRobot:
        """Return the live robot or raise :class:`RobotNotConnected`."""
        robot = self._robot
        if robot is None or not robot.is_connected:
            raise RobotNotConnected("Robot is not connected")
        return robot

    def connect(self, **params: Any) -> WalkieRobot:
        """
        Connect to a robot, replacing any existing connection.

        ``params`` are passed straight to ``WalkieRobot(...)`` (ip,
        ros_protocol, ros_port, camera_protocol, camera_port, namespace,
        arm_mode, timeout). Raises whatever ``WalkieRobot`` raises on failure
        (ConnectionError / ValueError); on failure no connection is retained.
        """
        with self._lock:
            self._teardown_locked()
            robot = WalkieRobot(**params)
            self._robot = robot
            self._params = dict(params)
            return robot

    def disconnect(self) -> None:
        """Disconnect and drop the current robot, if any."""
        with self._lock:
            self._teardown_locked()

    def _teardown_locked(self) -> None:
        """Disconnect the current robot. Caller must hold ``_lock``."""
        if self._robot is not None:
            try:
                self._robot.disconnect()
            except Exception:
                pass
            self._robot = None

    def snapshot(self) -> Dict[str, Any]:
        """
        Best-effort status snapshot for the dashboard.

        Every field is wrapped so a half-initialised or mid-disconnect robot
        still yields a serialisable dict instead of a 500.
        """
        robot = self._robot
        if robot is None:
            return {"connected": False}

        out: Dict[str, Any] = {
            "connected": robot.is_connected,
            "ip": self._params.get("ip"),
            "namespace": self._params.get("namespace", ""),
            "ros_protocol": self._params.get("ros_protocol", "rosbridge"),
            "camera_protocol": self._params.get("camera_protocol", "zenoh"),
        }

        out["position"] = _safe(lambda: robot.status.get_position())
        out["velocity"] = _safe(lambda: robot.status.get_velocity())
        out["nav_status"] = _safe(lambda: robot.nav.status)
        out["lift"] = _safe(lambda: robot.lift.get(norm_pos=True))
        out["lift_cm"] = _safe(lambda: robot.lift.get(norm_pos=False))
        out["lift_status"] = _safe(lambda: robot.lift.status)
        out["head_angle"] = _safe(lambda: robot.head.get_angle())
        out["arm_states"] = _safe(lambda: robot.arm.get_joint_states())
        out["joints_count"] = _safe(lambda: len(robot.joints.get_all()))
        out["camera_available"] = robot.camera is not None
        out["cameras"] = _safe(
            lambda: robot.cameras.camera_names if robot.cameras else []
        ) or []
        return out


def _safe(fn: Any) -> Any:
    """Call ``fn`` and swallow any error, returning None instead."""
    try:
        return fn()
    except Exception:
        return None


# Module-level singleton shared by the FastAPI app (and overridable in tests).
session = RobotSession()
