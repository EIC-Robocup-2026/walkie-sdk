"""
Walkie SDK - Transport Implementations

This package contains concrete implementations of the transport interfaces.
Each subpackage provides a different protocol for communicating with ROS2:

- rosbridge: WebSocket via roslibpy (no ROS2 required on client)
- zenoh: Zenoh DDS bridge (no ROS2 required on client)

Transports are loaded lazily by the TransportFactory to avoid
importing unnecessary dependencies.
"""

# Note: Transports are imported lazily by the factory to avoid
# requiring all dependencies to be installed. Do not import
# specific transports here.

__all__ = [
    "rosbridge",
    "zenoh",
]
