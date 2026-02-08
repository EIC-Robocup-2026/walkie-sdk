"""
Walkie SDK - Python SDK for controlling Walkie robots.

A pure Python library that wraps ROS2 communication into simple function calls.
Supports multiple protocols for flexibility:

- rosbridge: WebSocket via roslibpy (default, no ROS2 required on client)
- zenoh: Zenoh DDS bridge (no ROS2 required on client)

Example:
    from walkie_sdk import WalkieRobot

    # Default: WebSocket + WebRTC
    bot = WalkieRobot(ip="192.168.1.100")

    print(f"Pose: {bot.status.get_pose()}")
    bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
"""

from walkie_sdk.core.factory import (
    CameraProtocol,
    ROSProtocol,
    TransportFactory,
)
from walkie_sdk.modules.arm import ArmControlMode
from walkie_sdk.modules.visualization import (
    ARROW,
    CUBE,
    CUBE_LIST,
    CYLINDER,
    DEFAULT_AXIS_TOPIC,
    LINE_LIST,
    LINE_STRIP,
    MESH_RESOURCE,
    POINTS,
    SPHERE,
    SPHERE_LIST,
    TEXT_VIEW_FACING,
    TRIANGLE_LIST,
    Visualization,
)
from walkie_sdk.robot import WalkieRobot
from walkie_sdk.modules.multi_camera import MultiCamera

__version__ = "0.3.0"

__all__ = [
    # Main class
    "WalkieRobot",
    # Multi-camera support
    "MultiCamera",
    # Protocol enums (for advanced usage)
    "ROSProtocol",
    "CameraProtocol",
    "TransportFactory",
    # Arm control mode enum
    "ArmControlMode",
    # Visualization
    "Visualization",
    "ARROW",
    "CUBE",
    "SPHERE",
    "CYLINDER",
    "LINE_STRIP",
    "LINE_LIST",
    "CUBE_LIST",
    "SPHERE_LIST",
    "POINTS",
    "TEXT_VIEW_FACING",
    "MESH_RESOURCE",
    "TRIANGLE_LIST",
    "DEFAULT_AXIS_TOPIC",
    # Version
    "__version__",
]
