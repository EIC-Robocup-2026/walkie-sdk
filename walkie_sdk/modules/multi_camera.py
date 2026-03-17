"""
MultiCamera - Multi-camera robot interface module.

Provides access to multiple cameras (head, left wrist, right wrist)
on the robot. Supports Zenoh, USB, and mixed transports.
"""

from typing import TYPE_CHECKING, Dict, List, Optional, Tuple, Union

import numpy as np

if TYPE_CHECKING:
    from walkie_sdk.core.interfaces import CameraTransportInterface
    from walkie_sdk.core.interfaces.camera_transport import (
        MultiCameraTransportInterface,
    )


class MultiCamera:
    """
    Multi-camera robot interface.

    Provides convenient access to multiple cameras on the robot:
    - Head/front camera: Main forward-facing camera
    - Left wrist camera: Camera on left arm gripper
    - Right wrist camera: Camera on right arm gripper

    This class can wrap either:
    - A multi-camera transport implementing MultiCameraTransportInterface
      (e.g. ZenohCamera with multi_camera=True)
    - A single-camera transport implementing CameraTransportInterface
      (e.g. ZenohCamera, USBCamera)
    - A dictionary of single-camera transports

    Args:
        transport: Camera transport or dict of camera transports

    Example:
        ```python
        # Get all frames
        frames = bot.cameras.get_all_frames()
        for name, frame in frames.items():
            cv2.imshow(f"Camera: {name}", frame)

        # Get specific camera
        head_frame = bot.cameras.get_frame("head")
        ```
    """

    CAMERA_NAMES = ["head", "left", "right"]

    def __init__(
        self,
        transport: Union[
            "CameraTransportInterface",
            "MultiCameraTransportInterface",
            Dict[str, "CameraTransportInterface"],
        ],
    ):
        self._transport = transport
        self._is_dict = isinstance(transport, dict)

        # Detect multi-camera transport at init time (avoids repeated isinstance)
        from walkie_sdk.core.interfaces.camera_transport import (
            MultiCameraTransportInterface,
        )

        self._is_multi = not self._is_dict and isinstance(
            transport, MultiCameraTransportInterface
        )

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
        if self._is_multi:
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

        if self._is_multi:
            return self._transport.get_frame(camera_name)

        # Single-camera transport -- ignore camera_name
        return self._transport.get_frame()

    def get_all_frames(self) -> Dict[str, np.ndarray]:
        """
        Get the latest frames from all available cameras.

        Returns:
            Dictionary mapping camera name to frame

        Example:
            ```python
            frames = bot.cameras.get_all_frames()
            if "head" in frames:
                cv2.imshow("Head Camera", frames["head"])
            ```
        """
        if self._is_dict:
            frames = {}
            for name, transport in self._transport.items():
                frame = transport.get_frame()
                if frame is not None:
                    frames[name] = frame
            return frames

        if self._is_multi:
            return self._transport.get_all_frames()

        # Single-camera transport -- return under default name
        frame = self._transport.get_frame()
        return {"head": frame} if frame is not None else {}

    def get_frame_shape(
        self, camera_name: str = "head"
    ) -> Optional[Tuple[int, int, int]]:
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
