"""
Robot functionality modules.

- Navigation: go_to, cancel, stop
- Telemetry: get_pose, get_velocity
- Camera: get_frame
"""

from walkie_sdk.modules.navigation import Navigation
from walkie_sdk.modules.telemetry import Telemetry
from walkie_sdk.modules.camera import Camera

__all__ = ["Navigation", "Telemetry", "Camera"]
