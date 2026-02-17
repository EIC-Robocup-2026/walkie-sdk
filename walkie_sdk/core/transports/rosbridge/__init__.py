"""
Walkie SDK - ROSBridge Transport

WebSocket-based transport implementation using roslibpy.
This transport connects to a rosbridge_server running on the robot
and does not require ROS2 to be installed on the client machine.

Components:
- ROSBridgeTransport: ROS communication via WebSocket
"""

from walkie_sdk.core.transports.rosbridge.transport import ROSBridgeTransport

__all__ = [
    "ROSBridgeTransport",
]
