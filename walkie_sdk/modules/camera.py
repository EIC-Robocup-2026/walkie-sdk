"""
Camera - Robot camera/video stream module.

Provides get_frame() function and streaming status by wrapping
a camera transport implementation.

This module uses the CameraTransportInterface abstraction, allowing it
to work with any camera transport implementation (WebRTC, ROS Image, Zenoh).
"""

from typing import TYPE_CHECKING, Optional, Tuple

import numpy as np

if TYPE_CHECKING:
    from walkie_sdk.core.interfaces import CameraTransportInterface


class Camera:
    """
    Robot camera interface.

    Wraps a camera transport to provide a consistent API for accessing
    video frames regardless of the underlying protocol.

    This class works with any transport that implements CameraTransportInterface,
    making it protocol-agnostic (works with WebRTC, ROS Image topics, Zenoh, etc.).

    Args:
        transport: Camera transport instance implementing CameraTransportInterface

    Example:
        >>> frame = bot.camera.get_frame()
        >>> if frame is not None:
        ...     cv2.imshow("Robot Camera", frame)
        ...     cv2.waitKey(1)
    """

    def __init__(self, transport: "CameraTransportInterface"):
        self._transport = transport

    @property
    def is_streaming(self) -> bool:
        """
        Check if camera is actively streaming.

        Returns:
            True if connected and receiving frames, False otherwise
        """
        return self._transport.is_streaming

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        """
        Get the frame dimensions.

        Returns:
            Tuple of (height, width, channels) or None if unknown/not streaming
        """
        return self._transport.frame_shape

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest camera frame.

        Returns the most recent frame received from the camera.
        Does not block waiting for a new frame - returns the cached
        frame or None if no frame is available yet.

        Returns:
            BGR image as numpy array (HxWx3, uint8), or None if no frame available

        Example:
            >>> frame = bot.camera.get_frame()
            >>> if frame is not None:
            ...     # Frame is OpenCV-compatible BGR numpy array
            ...     height, width, channels = frame.shape
            ...     cv2.imwrite("snapshot.jpg", frame)
        """
        return self._transport.get_frame()

    def start(self) -> None:
        """
        Start the camera stream.

        Called automatically when WalkieRobot connects with camera enabled.
        Can be called manually to restart a stopped stream.

        Raises:
            ConnectionError: If unable to establish camera stream
        """
        if not self._transport.is_streaming:
            self._transport.connect()

    def stop(self) -> None:
        """
        Stop the camera stream.

        Releases resources and stops frame reception.
        The stream can be restarted by calling start().
        """
        if self._transport.is_streaming:
            self._transport.disconnect()

    def __repr__(self) -> str:
        status = "streaming" if self.is_streaming else "stopped"
        shape = self.frame_shape
        if shape:
            return f"Camera(status={status}, resolution={shape[1]}x{shape[0]})"
        return f"Camera(status={status})"
