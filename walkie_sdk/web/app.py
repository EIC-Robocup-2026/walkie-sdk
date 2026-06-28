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
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from walkie_sdk.web import models
from walkie_sdk.web.health import HealthMonitor
from walkie_sdk.web.state import RobotNotConnected, RobotSession
from walkie_sdk.web.state import session as default_session

STATIC_DIR = Path(__file__).parent / "static"
SOUNDS_DIR = Path(__file__).parent / "sounds"

_GRIPPER_MAX_M = 0.04  # mirror walkie_sdk.modules.arm._GRIPPER_MAX_M


def create_app(session: RobotSession | None = None) -> FastAPI:
    """
    Build the FastAPI app.

    Args:
        session: Robot session to drive. Defaults to the module-level singleton;
            tests inject a fake here.
    """
    session = session or default_session

    # Background subsystem-health monitor; cached report served at /api/health.
    monitor = HealthMonitor(session)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        monitor.start()
        try:
            yield
        finally:
            monitor.stop()

    app = FastAPI(
        title="Walkie SDK Web Interface", version="0.3.0", lifespan=lifespan
    )
    app.state.health_monitor = monitor

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

    @app.get("/api/health")
    def health() -> Dict[str, Any]:
        return monitor.latest()

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

    # ── Arm ─────────────────────────────────────────────────────────────
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
            frame_id=req.frame_id,
            cartesian_path=req.cartesian_path,
            blocking=req.blocking,
        )
        return {"ok": True, "result": result}

    @app.post("/api/arm/pose_quat")
    def arm_pose_quat(req: models.ArmPoseQuatRequest) -> Dict[str, Any]:
        robot = session.require()
        result = robot.arm.go_to_pose_quat(
            x=req.x,
            y=req.y,
            z=req.z,
            qx=req.qx,
            qy=req.qy,
            qz=req.qz,
            qw=req.qw,
            group_name=req.group_name,
            frame_id=req.frame_id,
            cartesian_path=req.cartesian_path,
            blocking=req.blocking,
        )
        return {"ok": True, "result": result}

    @app.post("/api/arm/pose_relative")
    def arm_pose_relative(req: models.ArmPoseRelativeRequest) -> Dict[str, Any]:
        robot = session.require()
        result = robot.arm.go_to_pose_relative(
            x=req.x,
            y=req.y,
            z=req.z,
            roll=req.roll,
            pitch=req.pitch,
            yaw=req.yaw,
            group_name=req.group_name,
            frame_id=req.frame_id,
            cartesian_path=req.cartesian_path,
            ee_frame=req.ee_frame,
            blocking=req.blocking,
        )
        return {"ok": True, "result": result}

    @app.post("/api/arm/home")
    def arm_home(req: models.HomeRequest) -> Dict[str, Any]:
        robot = session.require()
        return {
            "ok": True,
            "result": robot.arm.go_to_home(
                group_name=req.group_name,
                pose_name=req.pose_name,
                blocking=req.blocking,
            ),
        }

    @app.post("/api/arm/clear_objects")
    def arm_clear_objects() -> Dict[str, Any]:
        robot = session.require()
        ok = robot.arm.clear_collision_objects()
        out: Dict[str, Any] = {"ok": ok}
        if not ok:
            out["error"] = "Failed to clear collision objects (is move_group up?)"
        return out

    @app.post("/api/arm/clear_octomap")
    def arm_clear_octomap() -> Dict[str, Any]:
        robot = session.require()
        ok = robot.arm.clear_octomap()
        out: Dict[str, Any] = {"ok": ok}
        if not ok:
            out["error"] = "Failed to clear octomap (is move_group up?)"
        return out

    @app.post("/api/arm/toggle_collision")
    def arm_toggle_collision(req: models.ToggleCollisionRequest) -> Dict[str, Any]:
        robot = session.require()
        ok = robot.arm.toggle_gripper_collision(req.group_name, req.enable)
        out: Dict[str, Any] = {"ok": ok, "group_name": req.group_name, "enable": req.enable}
        if not ok:
            out["error"] = (
                f"Failed to {'enable' if req.enable else 'disable'} collision "
                f"for {req.group_name} (is move_group up?)"
            )
        return out

    @app.post("/api/arm/param/set")
    def arm_param_set(req: models.ParamSetRequest) -> Dict[str, Any]:
        robot = session.require()
        res = robot.arm.set_param_result(req.name, req.value)
        out: Dict[str, Any] = {"ok": res["ok"], "name": req.name, "value": req.value}
        if not res["ok"]:
            out["error"] = (
                f"Commander rejected '{req.name}' = {req.value!r}: "
                f"{res['reason'] or 'unknown reason'}"
            )
        return out

    @app.post("/api/arm/param/get")
    def arm_param_get(req: models.ParamGetRequest) -> Dict[str, Any]:
        robot = session.require()
        value = robot.arm.get_param(req.name)
        if value is None:
            return {
                "ok": False,
                "name": req.name,
                "value": None,
                "error": f"Param '{req.name}' not found or not set on commander",
            }
        return {"ok": True, "name": req.name, "value": value}

    @app.post("/api/arm/gripper")
    def arm_gripper(req: models.GripperRequest) -> Dict[str, Any]:
        robot = session.require()
        position_m = (
            max(0.0, min(1.0, req.position)) * _GRIPPER_MAX_M
            if req.norm
            else req.position
        )
        result = robot.arm.control_gripper(
            group_name=req.group_name,
            position=position_m,
            blocking=req.blocking,
        )
        return {"ok": True, "result": result, "position_m": position_m}

    @app.post("/api/arm/grasp")
    def arm_grasp(req: models.GraspRequest) -> Dict[str, Any]:
        robot = session.require()
        # Returns {"status", "success", "grasped", "gripper_gap"}.
        # Judge success by "grasped" (a real grasp stalls, so "success" is often False).
        result = robot.arm.grasp(group_name=req.group_name, position=req.position)
        return {"ok": True, **result}

    @app.post("/api/arm/joints")
    def arm_joints(req: models.ArmJointPositionRequest) -> Dict[str, Any]:
        robot = session.require()
        result = robot.arm.set_joint_position(
            group_name=req.group_name,
            joint_positions=req.joint_positions,
            mode=req.mode,
            duration=req.duration,
            blocking=req.blocking,
        )
        return {"ok": True, "result": result}

    @app.get("/api/arm/states")
    def arm_states() -> Dict[str, Any]:
        robot = session.require()
        return {"ok": True, "states": robot.arm.get_joint_states()}

    @app.post("/api/arm/ee_pose")
    def arm_ee_pose(req: models.EePoseRequest) -> Dict[str, Any]:
        robot = session.require()
        pose = robot.arm.get_ee_pose(
            group_name=req.group_name,
            frame_id=req.frame_id,
            timeout=req.timeout,
        )
        return {"ok": True, "pose": pose}

    # ── Head ────────────────────────────────────────────────────────────
    @app.post("/api/head/tilt")
    def head_tilt(req: models.HeadTiltRequest) -> Dict[str, Any]:
        robot = session.require()
        try:
            robot.head.tilt(req.angle_rad)
        except ValueError as e:
            return JSONResponse(
                {"ok": False, "error": str(e)}, status_code=400
            )
        return {"ok": True, "angle_rad": req.angle_rad}

    @app.get("/api/head")
    def head_get() -> Dict[str, Any]:
        robot = session.require()
        return {"ok": True, "angle_rad": robot.head.get_angle()}

    # ── Joints (shared hub) ─────────────────────────────────────────────
    @app.get("/api/joints")
    def joints_get() -> Dict[str, Any]:
        robot = session.require()
        return {"ok": True, "joints": robot.joints.get_all()}

    @app.get("/api/joints/{name}")
    def joints_get_one(name: str) -> Dict[str, Any]:
        robot = session.require()
        return {
            "ok": True,
            "position": robot.joints.get(name),
            "velocity": robot.joints.get_velocity(name),
            "effort": robot.joints.get_effort(name),
        }

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

    if SOUNDS_DIR.is_dir():
        app.mount("/sounds", StaticFiles(directory=str(SOUNDS_DIR)), name="sounds")

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
