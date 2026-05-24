"""
Request body schemas for the web API.

These mirror the keyword arguments of the corresponding SDK methods so the
route handlers can forward them with ``model.dict()`` and minimal glue.
"""

from __future__ import annotations

from typing import List, Optional

from pydantic import BaseModel, Field


class ConnectRequest(BaseModel):
    """Arguments for ``WalkieRobot(...)`` (see robot.py)."""

    ip: str
    ros_protocol: str = "rosbridge"
    ros_port: int = 9090
    camera_protocol: str = "zenoh"
    camera_port: int = 7447
    namespace: str = ""
    arm_mode: str = "custom_ik"
    timeout: float = 10.0


class GotoRequest(BaseModel):
    """Arguments for ``nav.go_to(...)``."""

    x: float
    y: float
    heading: float = 0.0
    blocking: bool = False


class LiftRequest(BaseModel):
    """Arguments for ``lift.set(...)``."""

    pos: float
    speed: float = 2.0
    accel: float = 1.0
    norm_pos: bool = True
    blocking: bool = False


class ArmPoseRequest(BaseModel):
    """Arguments for ``arm.go_to_pose(...)`` (Euler orientation)."""

    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    group_name: str = "left_arm"
    cartesian_path: bool = False
    blocking: bool = False
    mode: Optional[str] = None  # "moveit" | "custom_ik" | None (use default)


class GripperRequest(BaseModel):
    """Arguments for ``arm.control_gripper(...)``."""

    group_name: str = "left_gripper"
    position: float
    blocking: bool = False


class HomeRequest(BaseModel):
    """Arguments for ``arm.go_to_home(...)``."""

    group_name: str = "left_arm"


class ArmJointsRequest(BaseModel):
    """Arguments for ``arm.set_joint_positions(...)``."""

    left_arm: Optional[List[float]] = None
    right_arm: Optional[List[float]] = None
    left_gripper: Optional[float] = None
    right_gripper: Optional[float] = None
    blocking: bool = False


class NamespaceRequest(BaseModel):
    """Set the active ROS namespace."""

    namespace: str = Field(default="")
