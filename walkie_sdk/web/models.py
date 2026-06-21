"""
Request body schemas for the web API.

These mirror the keyword arguments of the corresponding SDK methods so the
route handlers can forward them with ``model.dict()`` and minimal glue.
"""

from __future__ import annotations

from typing import Any, List, Optional

from pydantic import BaseModel, Field


class ConnectRequest(BaseModel):
    """Arguments for ``WalkieRobot(...)`` (see robot.py)."""

    ip: str
    ros_protocol: str = "hybrid"
    ros_port: int = 9090
    camera_protocol: str = "zenoh"
    camera_port: int = 7447
    namespace: str = ""
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
    """Arguments for ``arm.go_to_pose(...)`` (Euler RPY orientation)."""

    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    group_name: str = "left_arm_lift"
    frame_id: str = "base_footprint"
    cartesian_path: bool = False
    blocking: bool = False


class ArmPoseQuatRequest(BaseModel):
    """Arguments for ``arm.go_to_pose_quat(...)``."""

    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    group_name: str = "left_arm_lift"
    frame_id: str = "base_footprint"
    cartesian_path: bool = False
    blocking: bool = False


class ArmPoseRelativeRequest(BaseModel):
    """Arguments for ``arm.go_to_pose_relative(...)``."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    group_name: str = "left_arm_lift"
    frame_id: str = "base_footprint"
    cartesian_path: bool = False
    ee_frame: bool = False
    blocking: bool = False


class GripperRequest(BaseModel):
    """
    Arguments for ``arm.control_gripper(...)``.

    ``position`` is in meters in [0.0, 0.04] when ``norm=False``,
    or normalized [0.0, 1.0] when ``norm=True`` (default). 0 = closed, 1 = open.
    """

    group_name: str = "left_gripper"
    position: float
    norm: bool = True
    blocking: bool = False


class GraspRequest(BaseModel):
    """Arguments for ``arm.grasp(...)`` — close the gripper and report whether an
    object is actually held (the ``grasped`` field of the response)."""

    group_name: str = "left_gripper"
    position: float = 0.0   # meters, close target (0 = fully closed)


class HomeRequest(BaseModel):
    """Arguments for ``arm.go_to_home(...)``."""

    group_name: str = "left_arm_lift"
    pose_name: str = ""
    blocking: bool = False


class ParamSetRequest(BaseModel):
    """Arguments for ``arm.set_param(...)``. ``value`` is JSON-typed (number,
    bool, string, or list) — the client sends it already coerced."""

    name: str
    value: Any


class ParamGetRequest(BaseModel):
    """Arguments for ``arm.get_param(...)``."""

    name: str


class ToggleCollisionRequest(BaseModel):
    """Arguments for ``arm.toggle_gripper_collision(...)``."""

    group_name: str = "left_gripper"
    enable: bool


class ArmJointPositionRequest(BaseModel):
    """Arguments for ``arm.set_joint_position(...)``."""

    group_name: str = "left_arm"
    joint_positions: List[float]
    mode: str = "commander"  # "commander" | "jtc"
    duration: float = 1.0
    blocking: bool = False


class EePoseRequest(BaseModel):
    """Arguments for ``arm.get_ee_pose(...)``."""

    group_name: str = "left_arm"
    frame_id: str = "base_footprint"
    timeout: float = 5.0


class HeadTiltRequest(BaseModel):
    """Arguments for ``head.tilt(...)``.

    ``angle_rad`` must be within ±π/4 (±45°). Positive tilts the camera
    downward.
    """

    angle_rad: float


class DrawPoseRequest(BaseModel):
    """Arguments for ``viz.draw_pose(...)`` — preview an arm target pose in RViz.

    Supply either ``roll/pitch/yaw`` (Euler RPY) **or** ``qx/qy/qz/qw`` (quaternion).
    When ``qw`` is provided the quaternion fields take priority.
    """

    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    qx: Optional[float] = None
    qy: Optional[float] = None
    qz: Optional[float] = None
    qw: Optional[float] = None
    frame_id: str = "base_footprint"
    topic: str = "walkie/arm_target_pose"


class NamespaceRequest(BaseModel):
    """Set the active ROS namespace."""

    namespace: str = Field(default="")
