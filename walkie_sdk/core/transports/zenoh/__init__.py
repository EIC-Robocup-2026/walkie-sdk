"""
Walkie SDK - Zenoh Transport

Zenoh-based transport implementation for ROS2 communication.
This transport uses the Zenoh DDS bridge for low-latency communication
without requiring ROS2 to be installed on the client machine.

Status: Not yet implemented (stub)
"""

from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

from walkie_sdk.core.interfaces import CameraTransportInterface, ROSTransportInterface


class ZenohTransport(ROSTransportInterface):
    """
    ROS transport implementation using Zenoh DDS bridge.

    This transport provides an alternative to rosbridge with potentially
    better performance characteristics for edge computing scenarios.

    Status: Not yet implemented.

    Args:
        host: Robot IP address or hostname
        port: Zenoh router port (default: 7447)
        timeout: Connection timeout in seconds (default: 10.0)
    """

    def __init__(
        self,
        host: str,
        port: int = 7447,
        timeout: float = 10.0,
        **kwargs,
    ):
        raise NotImplementedError(
            "ZenohTransport is not yet implemented. "
            "Please use 'rosbridge' or 'rclpy' protocol instead. "
            "Zenoh support is planned for a future release."
        )

    @property
    def is_connected(self) -> bool:
        return False

    def connect(self) -> None:
        raise NotImplementedError("ZenohTransport is not yet implemented")

    def disconnect(self) -> None:
        raise NotImplementedError("ZenohTransport is not yet implemented")

    def subscribe(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[Dict[str, Any]], None],
        throttle_rate: int = 0,
        queue_size: int = 1,
    ) -> Any:
        raise NotImplementedError("ZenohTransport is not yet implemented")

    def unsubscribe(self, handle: Any) -> None:
        raise NotImplementedError("ZenohTransport is not yet implemented")

    def publish(
        self,
        topic: str,
        message_type: str,
        message: Dict[str, Any],
    ) -> None:
        raise NotImplementedError("ZenohTransport is not yet implemented")

    def call_action(
        self,
        action_name: str,
        action_type: str,
        goal: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        raise NotImplementedError("ZenohTransport is not yet implemented")

    def cancel_action(self) -> None:
        raise NotImplementedError("ZenohTransport is not yet implemented")

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request: Dict[str, Any],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        raise NotImplementedError("ZenohTransport is not yet implemented")


class ZenohCamera(CameraTransportInterface):
    """
    Camera transport implementation using Zenoh.

    Status: Not yet implemented.

    Args:
        host: Robot IP address or hostname
        port: Zenoh port for video stream
        topic: Zenoh topic for camera frames
    """

    def __init__(
        self,
        host: str,
        port: int = 7447,
        topic: str = "camera/image",
        **kwargs,
    ):
        raise NotImplementedError(
            "ZenohCamera is not yet implemented. "
            "Please use 'webrtc' or 'ros_image' camera protocol instead. "
            "Zenoh camera support is planned for a future release."
        )

    @property
    def is_streaming(self) -> bool:
        return False

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        return None

    def connect(self) -> None:
        raise NotImplementedError("ZenohCamera is not yet implemented")

    def disconnect(self) -> None:
        raise NotImplementedError("ZenohCamera is not yet implemented")

    def get_frame(self) -> Optional[np.ndarray]:
        raise NotImplementedError("ZenohCamera is not yet implemented")


__all__ = ["ZenohTransport", "ZenohCamera"]
