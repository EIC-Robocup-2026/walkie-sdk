"""
Walkie SDK - rclpy Transport (Stub)

Native ROS2 Python transport implementation.
This transport requires ROS2 to be installed on the client machine.

Status: NOT YET IMPLEMENTED
"""

from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

from walkie_sdk.core.interfaces import CameraTransportInterface, ROSTransportInterface


class RclpyTransport(ROSTransportInterface):
    """
    ROS transport implementation using native rclpy.

    This transport uses the ROS2 Python client library directly,
    providing the best performance but requiring ROS2 to be installed.

    Status: NOT YET IMPLEMENTED

    Args:
        node_name: Name for the ROS2 node (default: "walkie_sdk")
        timeout: Connection timeout in seconds (default: 10.0)

    Raises:
        NotImplementedError: This transport is not yet implemented
    """

    def __init__(
        self,
        node_name: str = "walkie_sdk",
        timeout: float = 10.0,
        **kwargs,
    ):
        raise NotImplementedError(
            "RclpyTransport is not yet implemented. "
            "Please use 'rosbridge' protocol instead, or contribute an implementation! "
            "See: https://github.com/walkie-team/walkie-sdk"
        )

    def connect(self) -> None:
        raise NotImplementedError("RclpyTransport is not yet implemented")

    def disconnect(self) -> None:
        raise NotImplementedError("RclpyTransport is not yet implemented")

    @property
    def is_connected(self) -> bool:
        return False

    def subscribe(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[Dict[str, Any]], None],
        throttle_rate: int = 0,
        queue_size: int = 1,
    ) -> Any:
        raise NotImplementedError("RclpyTransport is not yet implemented")

    def unsubscribe(self, handle: Any) -> None:
        raise NotImplementedError("RclpyTransport is not yet implemented")

    def publish(
        self,
        topic: str,
        message_type: str,
        message: Dict[str, Any],
    ) -> None:
        raise NotImplementedError("RclpyTransport is not yet implemented")

    def call_action(
        self,
        action_name: str,
        action_type: str,
        goal: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        raise NotImplementedError("RclpyTransport is not yet implemented")

    def cancel_action(self) -> None:
        raise NotImplementedError("RclpyTransport is not yet implemented")

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request: Dict[str, Any],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        raise NotImplementedError("RclpyTransport is not yet implemented")


class ROSImageCamera(CameraTransportInterface):
    """
    Camera transport using ROS sensor_msgs/Image topic.

    Subscribes to a ROS2 image topic and provides frames via the
    standard camera interface. Pairs with RclpyTransport.

    Status: NOT YET IMPLEMENTED

    Args:
        ros_transport: RclpyTransport instance for ROS communication
        topic: Image topic name (default: "/camera/image_raw")
        **kwargs: Additional options

    Raises:
        NotImplementedError: This camera transport is not yet implemented
    """

    def __init__(
        self,
        ros_transport: ROSTransportInterface,
        topic: str = "/camera/image_raw",
        **kwargs,
    ):
        raise NotImplementedError(
            "ROSImageCamera is not yet implemented. "
            "Please use 'webrtc' camera protocol instead, or contribute an implementation! "
            "See: https://github.com/walkie-team/walkie-sdk"
        )

    def connect(self) -> None:
        raise NotImplementedError("ROSImageCamera is not yet implemented")

    def disconnect(self) -> None:
        raise NotImplementedError("ROSImageCamera is not yet implemented")

    @property
    def is_streaming(self) -> bool:
        return False

    def get_frame(self) -> Optional[np.ndarray]:
        raise NotImplementedError("ROSImageCamera is not yet implemented")

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        return None


__all__ = ["RclpyTransport", "ROSImageCamera"]
