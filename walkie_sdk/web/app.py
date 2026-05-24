"""
FastAPI application for the Walkie web interface.

The app is a thin HTTP shell over a :class:`~walkie_sdk.web.state.RobotSession`.
Every ``/api/*`` route either reads the session snapshot or forwards a request
body to the matching SDK method. Robot calls are intentionally defensive: the
SDK already turns most failures into ``"FAILED"``/``None`` return values rather
than exceptions, and anything that does raise is surfaced as a JSON error
instead of a bare 500 so the dashboard can show it.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Callable, Dict

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from walkie_sdk.web import models
from walkie_sdk.web.state import RobotNotConnected, RobotSession
from walkie_sdk.web.state import session as default_session

STATIC_DIR = Path(__file__).parent / "static"


def create_app(session: RobotSession | None = None) -> FastAPI:
    """
    Build the FastAPI app.

    Args:
        session: Robot session to drive. Defaults to the module-level singleton;
            tests inject a fake here.
    """
    session = session or default_session
    app = FastAPI(title="Walkie SDK Web Interface", version="0.2.0")

    # ── Connection lifecycle ────────────────────────────────────────────
    @app.post("/api/connect")
    def connect(req: models.ConnectRequest) -> Dict[str, Any]:
        try:
            session.connect(**req.model_dump())
        except Exception as e:  # ConnectionError, ValueError, ...
            return JSONResponse(
                {"ok": False, "error": str(e)}, status_code=502
            )
        return {"ok": True, "status": session.snapshot()}

    @app.post("/api/disconnect")
    def disconnect() -> Dict[str, Any]:
        session.disconnect()
        return {"ok": True}

    @app.get("/api/status")
    def status() -> Dict[str, Any]:
        return session.snapshot()

    @app.post("/api/namespace")
    def set_namespace(req: models.NamespaceRequest) -> Dict[str, Any]:
        robot = session.require()
        robot.namespace = req.namespace
        return {"ok": True, "namespace": robot.namespace}

    # ── Navigation ──────────────────────────────────────────────────────
    @app.post("/api/nav/goto")
    def nav_goto(req: models.GotoRequest) -> Dict[str, Any]:
        robot = session.require()
        result = robot.nav.go_to(
            x=req.x, y=req.y, heading=req.heading, blocking=req.blocking
        )
        return {"ok": True, "result": result}

    @app.post("/api/nav/cancel")
    def nav_cancel() -> Dict[str, Any]:
        robot = session.require()
        return {"ok": True, "result": robot.nav.cancel()}

    @app.post("/api/nav/stop")
    def nav_stop() -> Dict[str, Any]:
        robot = session.require()
        return {"ok": True, "result": robot.nav.stop()}

    # ── Lift ────────────────────────────────────────────────────────────
    @app.post("/api/lift/set")
    def lift_set(req: models.LiftRequest) -> Dict[str, Any]:
        robot = session.require()
        result = robot.lift.set(
            pos=req.pos,
            speed=req.speed,
            accel=req.accel,
            norm_pos=req.norm_pos,
            blocking=req.blocking,
        )
        return {"ok": True, "result": result}

    @app.get("/api/lift")
    def lift_get() -> Dict[str, Any]:
        robot = session.require()
        return {
            "ok": True,
            "norm": robot.lift.get(norm_pos=True),
            "cm": robot.lift.get(norm_pos=False),
            "status": robot.lift.status,
        }

    # ── Arm + Gripper ───────────────────────────────────────────────────
    @app.post("/api/arm/pose")
    def arm_pose(req: models.ArmPoseRequest) -> Dict[str, Any]:
        robot = session.require()
        result = robot.arm.go_to_pose(
            x=req.x,
            y=req.y,
            z=req.z,
            roll=req.roll,
            pitch=req.pitch,
            yaw=req.yaw,
            group_name=req.group_name,
            cartesian_path=req.cartesian_path,
            blocking=req.blocking,
            mode=req.mode,
        )
        return {"ok": True, "result": result}

    @app.post("/api/arm/home")
    def arm_home(req: models.HomeRequest) -> Dict[str, Any]:
        robot = session.require()
        return {"ok": True, "result": robot.arm.go_to_home(req.group_name)}

    @app.post("/api/arm/gripper")
    def arm_gripper(req: models.GripperRequest) -> Dict[str, Any]:
        robot = session.require()
        result = robot.arm.control_gripper(
            group_name=req.group_name,
            position=req.position,
            blocking=req.blocking,
        )
        return {"ok": True, "result": result}

    @app.post("/api/arm/joints")
    def arm_joints(req: models.ArmJointsRequest) -> Dict[str, Any]:
        robot = session.require()
        ok = robot.arm.set_joint_positions(
            left_arm=req.left_arm,
            right_arm=req.right_arm,
            left_gripper=req.left_gripper,
            right_gripper=req.right_gripper,
            blocking=req.blocking,
        )
        return {"ok": True, "result": ok}

    @app.get("/api/arm/states")
    def arm_states() -> Dict[str, Any]:
        robot = session.require()
        return {"ok": True, "states": robot.arm.get_joint_states()}

    # ── Camera ──────────────────────────────────────────────────────────
    @app.get("/api/camera/snapshot")
    def camera_snapshot(camera: str = "head") -> Response:
        robot = session.require()
        frame = _grab_frame(robot, camera)
        if frame is None:
            raise HTTPException(503, "No camera frame available")
        return Response(content=_encode_jpeg(frame), media_type="image/jpeg")

    @app.get("/api/camera/stream")
    def camera_stream(camera: str = "head") -> StreamingResponse:
        robot = session.require()

        def gen():
            boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
            while session.is_connected:
                frame = _grab_frame(robot, camera)
                if frame is None:
                    time.sleep(0.05)
                    continue
                yield boundary + _encode_jpeg(frame) + b"\r\n"
                time.sleep(0.04)  # ~25 fps cap

        return StreamingResponse(
            gen(), media_type="multipart/x-mixed-replace; boundary=frame"
        )

    # ── Exception handling ──────────────────────────────────────────────
    @app.exception_handler(RobotNotConnected)
    def _not_connected(_request, _exc):  # noqa: ANN001
        return JSONResponse({"ok": False, "error": "not_connected"}, status_code=409)

    # ── Static frontend ─────────────────────────────────────────────────
    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(STATIC_DIR / "index.html")

    if STATIC_DIR.is_dir():
        app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

    return app


def _grab_frame(robot: Any, camera: str):
    """Return a BGR frame from the named camera, or None if unavailable."""
    if robot.cameras is not None:
        return robot.cameras.get_frame(camera)
    if robot.camera is not None:
        return robot.camera.get_frame()
    return None


def _encode_jpeg(frame) -> bytes:
    """Encode a BGR numpy frame as JPEG bytes."""
    import cv2  # local import: only needed when a camera is actually used

    ok, buf = cv2.imencode(".jpg", frame)
    if not ok:
        raise HTTPException(500, "Failed to encode frame")
    return buf.tobytes()
