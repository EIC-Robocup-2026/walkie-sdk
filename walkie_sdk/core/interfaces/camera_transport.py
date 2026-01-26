"""
CameraTransportInterface - Abstract interface for camera/video streams.

Defines the contract that any camera transport implementation must fulfill,
allowing the SDK to work with different camera protocols (WebRTC, Zenoh, etc.)
"""

from abc import ABC, abstractmethod
from typing import Optional, Tuple

import numpy as np


class CameraTransportInterface(ABC):
    """
    Abstract interface for camera/video streams.

    Implementations:
    - WebRTCCamera: WebRTC stream (for rosbridge setup)
    - ZenohCamera: Zenoh video stream
    - SharedMemoryCamera: Direct memory access (same-host)

    All implementations must provide thread-safe frame access.
    """

    @abstractmethod
    def connect(self) -> None:
        """
        Start the camera stream.

        Establishes connection and begins receiving frames.
        Blocks until the stream is ready or raises an error.

        Raises:
            ConnectionError: If unable to establish camera stream
        """
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """
        Stop the camera stream.

        Releases all resources and stops frame reception.
        Safe to call multiple times.
        """
        pass

    @property
    @abstractmethod
    def is_streaming(self) -> bool:
        """
        Check if camera is actively streaming.

        Returns:
            True if connected and receiving frames, False otherwise
        """
        pass

    @abstractmethod
    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest camera frame.

        Returns the most recent frame received. Does not block waiting
        for a new frame - returns the cached frame or None.

        Returns:
            BGR image as numpy array (HxWx3, uint8), or None if no frame available

        Note:
            The returned array should be treated as read-only or copied
            before modification to avoid race conditions.
        """
        pass

    @property
    @abstractmethod
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        """
        Get the frame dimensions.

        Returns:
            Tuple of (height, width, channels) or None if unknown/not streaming
        """
        pass

    def __enter__(self) -> "CameraTransportInterface":
        """Context manager entry - connect to camera."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit - disconnect from camera."""
        self.disconnect()
