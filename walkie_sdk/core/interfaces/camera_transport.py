"""
CameraTransportInterface - Abstract interface for camera/video streams.

Defines the contract that any camera transport implementation must fulfill,
allowing the SDK to work with different camera protocols (Zenoh, USB, etc.)
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

import numpy as np


class CameraTransportInterface(ABC):
    """
    Abstract interface for camera/video streams.

    Implementations:
    - ZenohCamera: Zenoh video stream
    - USBCamera: Local USB camera via OpenCV

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


class MultiCameraTransportInterface(CameraTransportInterface):
    """
    Abstract interface for multi-camera transports.

    Extends CameraTransportInterface with named camera access.
    Implementations provide access to multiple cameras (e.g. head, left, right)
    through a unified interface.

    Implementations:
    - ZenohCamera (with multi_camera=True)
    """

    @abstractmethod
    def get_frame(self, camera_name: Optional[str] = None) -> Optional[np.ndarray]:
        """
        Get the latest frame from a specific camera.

        Args:
            camera_name: Name of camera (e.g. "head", "left", "right").
                         If None, returns the default camera frame.

        Returns:
            BGR image as numpy array (HxWx3, uint8), or None if not available
        """
        pass

    @abstractmethod
    def get_all_frames(self) -> Dict[str, np.ndarray]:
        """
        Get the latest frames from all available cameras.

        Returns:
            Dictionary mapping camera name to BGR numpy array.
            Only includes cameras that have frames available.
        """
        pass

    @property
    @abstractmethod
    def camera_names(self) -> List[str]:
        """
        Get list of available camera names.

        Returns:
            List of camera name strings (e.g. ["head", "left", "right"])
        """
        pass
