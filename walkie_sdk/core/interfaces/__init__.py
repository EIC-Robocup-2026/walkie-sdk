"""
Walkie SDK - Core Interfaces

Abstract base classes defining the contracts for transport implementations.
These interfaces allow the SDK to work with different protocols
(rosbridge, zenoh) without changing the module code.
"""

from walkie_sdk.core.interfaces.camera_transport import CameraTransportInterface
from walkie_sdk.core.interfaces.ros_transport import ROSTransportInterface

__all__ = [
    "ROSTransportInterface",
    "CameraTransportInterface",
]
