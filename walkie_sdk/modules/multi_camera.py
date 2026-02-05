"""
MultiCamera - Multi-camera robot interface module.

Provides access to multiple cameras (head, left wrist, right wrist)
on the robot. Supports both shared memory and Zenoh transports.
"""

from typing import TYPE_CHECKING, Dict, List, Optional, Tuple, Union

import numpy as np

if TYPE_CHECKING:
    from walkie_sdk.core.interfaces import CameraTransportInterface


class MultiCamera:
    """
    Multi-camera robot interface.

    Provides convenient access to multiple cameras on the robot:
    - Head/front camera: Main forward-facing camera
    - Left wrist camera: Camera on left arm gripper
    - Right wrist camera: Camera on right arm gripper

    This class can wrap either:
    - A multi-camera transport (SharedMemoryMultiCamera, ZenohCamera with multi_camera=True)
    - A dictionary of single-camera transports

    Args:
        transport: Multi-camera transport or dict of camera transports

    Example:
        >>> # Get all frames
        >>> frames = bot.cameras.get_all_frames()
        >>> for name, frame in frames.items():
        ...     cv2.imshow(f"Camera: {name}", frame)
        
        >>> # Get specific camera
        >>> head_frame = bot.cameras.get_head_frame()
    """

    CAMERA_NAMES = ["head", "left", "right"]

    def __init__(
        self,
        transport: Union["CameraTransportInterface", Dict[str, "CameraTransportInterface"]],
    ):
        self._transport = transport
        self._is_dict = isinstance(transport, dict)

    @property
    def is_streaming(self) -> bool:
        """
        Check if any camera is actively streaming.

        Returns:
            True if at least one camera is streaming
        """
        if self._is_dict:
            return any(t.is_streaming for t in self._transport.values())
        return self._transport.is_streaming

    @property
    def camera_names(self) -> List[str]:
        """
        Get list of available camera names.

        Returns:
            List of camera names
        """
        if self._is_dict:
            return list(self._transport.keys())
        if hasattr(self._transport, 'camera_names'):
            return self._transport.camera_names
        return self.CAMERA_NAMES

    def get_frame(self, camera_name: str = "head") -> Optional[np.ndarray]:
        """
        Get the latest frame from a specific camera.

        Args:
            camera_name: Name of camera ("head", "left", or "right")

        Returns:
            BGR image as numpy array (HxWx3, uint8), or None if not available
        """
        if self._is_dict:
            if camera_name in self._transport:
                return self._transport[camera_name].get_frame()
            return None
        
        # Multi-camera transport
        if hasattr(self._transport, 'get_frame'):
            # Check if transport's get_frame accepts camera_name
            try:
                return self._transport.get_frame(camera_name)
            except TypeError:
                # Single camera transport
                return self._transport.get_frame()
        return None

    def get_head_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest head/front camera frame.

        Returns:
            BGR image as numpy array, or None if not available
        """
        if self._is_dict:
            return self.get_frame("head")
        if hasattr(self._transport, 'get_head_frame'):
            return self._transport.get_head_frame()
        return self.get_frame("head")

    def get_left_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest left wrist camera frame.

        Returns:
            BGR image as numpy array, or None if not available
        """
        if self._is_dict:
            return self.get_frame("left")
        if hasattr(self._transport, 'get_left_frame'):
            return self._transport.get_left_frame()
        return self.get_frame("left")

    def get_right_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest right wrist camera frame.

        Returns:
            BGR image as numpy array, or None if not available
        """
        if self._is_dict:
            return self.get_frame("right")
        if hasattr(self._transport, 'get_right_frame'):
            return self._transport.get_right_frame()
        return self.get_frame("right")

    def get_all_frames(self) -> Dict[str, np.ndarray]:
        """
        Get the latest frames from all available cameras.

        Returns:
            Dictionary mapping camera name to frame

        Example:
            >>> frames = bot.cameras.get_all_frames()
            >>> if "head" in frames:
            ...     cv2.imshow("Head Camera", frames["head"])
        """
        if self._is_dict:
            frames = {}
            for name, transport in self._transport.items():
                frame = transport.get_frame()
                if frame is not None:
                    frames[name] = frame
            return frames
        
        # Multi-camera transport
        if hasattr(self._transport, 'get_all_frames'):
            return self._transport.get_all_frames()
        
        # Fallback: try each camera individually
        frames = {}
        for name in self.camera_names:
            frame = self.get_frame(name)
            if frame is not None:
                frames[name] = frame
        return frames

    def get_frame_shape(self, camera_name: str = "head") -> Optional[Tuple[int, int, int]]:
        """
        Get the frame dimensions for a specific camera.

        Args:
            camera_name: Name of camera

        Returns:
            Tuple of (height, width, channels) or None if unknown
        """
        if self._is_dict:
            if camera_name in self._transport:
                return self._transport[camera_name].frame_shape
            return None
        return self._transport.frame_shape

    def start(self) -> None:
        """
        Start all camera streams.

        Raises:
            ConnectionError: If unable to establish camera streams
        """
        if self._is_dict:
            for transport in self._transport.values():
                if not transport.is_streaming:
                    transport.connect()
        else:
            if not self._transport.is_streaming:
                self._transport.connect()

    def stop(self) -> None:
        """
        Stop all camera streams.
        """
        if self._is_dict:
            for transport in self._transport.values():
                if transport.is_streaming:
                    transport.disconnect()
        else:
            if self._transport.is_streaming:
                self._transport.disconnect()

    def __repr__(self) -> str:
        status = "streaming" if self.is_streaming else "stopped"
        cameras = self.camera_names
        return f"MultiCamera(status={status}, cameras={cameras})"
