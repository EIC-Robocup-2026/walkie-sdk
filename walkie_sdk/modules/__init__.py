"""
Walkie SDK - Modules

High-level robot control modules that provide user-friendly APIs
for navigation, telemetry, and camera access.

These modules are protocol-agnostic and work with any transport
implementation (rosbridge, zenoh) via abstract interfaces.
"""

from walkie_sdk.modules.camera import Camera
from walkie_sdk.modules.navigation import Navigation
from walkie_sdk.modules.telemetry import Telemetry

__all__ = [
    "Navigation",
    "Telemetry",
    "Camera",
]
