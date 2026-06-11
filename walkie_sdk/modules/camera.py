"""
Camera - Robot camera/video stream module.

Provides get_frame() function and streaming status by wrapping
a camera transport implementation.

This module uses the CameraTransportInterface abstraction, allowing it
to work with any camera transport implementation (Zenoh, USB, etc.).
"""

import threading
from typing import TYPE_CHECKING, Any, Dict, Optional, Tuple

import numpy as np

from walkie_sdk.config.ros_topics import CAMERA_INFO_TOPICS

if TYPE_CHECKING:
    from walkie_sdk.core.interfaces import (
        CameraTransportInterface,
        ROSTransportInterface,
    )


class Camera:
    """
    Robot camera interface.

    Wraps a camera transport to provide a consistent API for accessing
    video frames regardless of the underlying protocol.

    This class works with any transport that implements CameraTransportInterface,
    making it protocol-agnostic (works with Zenoh, USB, etc.).

    Args:
        transport: Camera transport instance implementing CameraTransportInterface
        ros_transport: Optional ROS transport used to fetch CameraInfo intrinsics.
            Without it, get_camera_info()/get_intrinsics() return None.
        camera_name: Key into CAMERA_INFO_TOPICS for this camera (default: "head")

    Example:
        ```python
        frame = bot.camera.get_frame()
        if frame is not None:
            cv2.imshow("Robot Camera", frame)
            cv2.waitKey(1)
        ```
    """

    def __init__(
        self,
        transport: "CameraTransportInterface",
        ros_transport: Optional["ROSTransportInterface"] = None,
        camera_name: str = "head",
    ):
        self._transport = transport
        self._ros_transport = ros_transport
        self._camera_name = camera_name
        self._camera_info: Optional[Dict[str, Any]] = None

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
            ```python
            frame = bot.camera.get_frame()
            if frame is not None:
                # Frame is OpenCV-compatible BGR numpy array
                height, width, channels = frame.shape
                cv2.imwrite("snapshot.jpg", frame)
            ```
        """
        return self._transport.get_frame()

    def get_depth(self) -> Optional[np.ndarray]:
        """
        Get the latest depth frame.

        Available with the zenoh camera transport, which streams depth
        automatically. Returns the most recent depth image; does not block
        waiting for a new one.

        Returns:
            Single-channel depth array in the topic's native units, or None if
            depth is disabled/unsupported or no frame is available yet.
            For the ZED head (``32FC1``) this is ``HxW`` float32 **metres**,
            where invalid pixels are ``NaN``.

        Example:
            ```python
            depth = bot.camera.get_depth()
            if depth is not None:
                # distance to the pixel at the image centre, in metres
                h, w = depth.shape
                print(depth[h // 2, w // 2])
            ```
        """
        return self._transport.get_depth()

    def get_camera_info(self, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        """
        Get the camera calibration (sensor_msgs/msg/CameraInfo) as a dict.

        Fetched once from the camera_info topic next to the image topic
        (CAMERA_INFO_TOPICS) and cached — intrinsics are static, so later
        calls return immediately without touching the network.

        The ZED head's depth is registered to the left rectified image, so
        these intrinsics apply to both the colour frame and get_depth().
        Because the image is rectified, distortion coefficients are zero and
        the 'k' matrix can be used directly.

        Args:
            timeout: Seconds to wait for the first CameraInfo message

        Returns:
            CameraInfo as a dict (keys: 'k', 'd', 'p', 'width', 'height', ...)
            or None if no ROS transport is available, the topic is not
            configured, or no message arrived within the timeout.

        Example:
            ```python
            info = bot.camera.get_camera_info()
            fx, fy = info["k"][0], info["k"][4]
            cx, cy = info["k"][2], info["k"][5]
            ```
        """
        if self._camera_info is not None:
            return self._camera_info

        if self._ros_transport is None:
            return None

        topic = CAMERA_INFO_TOPICS.get(self._camera_name)
        if not topic:
            return None

        result: Dict[str, Any] = {}
        received = threading.Event()

        def _on_info(msg: Dict[str, Any]) -> None:
            if not received.is_set():
                result.update(msg)
                received.set()

        try:
            handle = self._ros_transport.subscribe(
                topic, "sensor_msgs/msg/CameraInfo", _on_info
            )
        except Exception as e:
            print(f"[Camera] Failed to subscribe to '{topic}': {e}")
            return None

        try:
            if not received.wait(timeout):
                print(f"[Camera] No CameraInfo on '{topic}' within {timeout}s")
                return None
            self._camera_info = result
            return self._camera_info
        finally:
            try:
                self._ros_transport.unsubscribe(handle)
            except Exception:
                pass

    def get_intrinsics(self, timeout: float = 5.0) -> Optional[Dict[str, float]]:
        """
        Get pinhole intrinsics for projecting depth pixels to 3D.

        Convenience wrapper around get_camera_info() that extracts the
        focal lengths and principal point from the 'k' matrix.

        Args:
            timeout: Seconds to wait for the first CameraInfo message

        Returns:
            Dict with 'fx', 'fy', 'cx', 'cy', 'width', 'height',
            or None if the camera info is unavailable.

        Example:
            ```python
            intr = bot.camera.get_intrinsics()
            depth = bot.camera.get_depth()
            z = depth[v, u]
            x = (u - intr["cx"]) * z / intr["fx"]
            y = (v - intr["cy"]) * z / intr["fy"]
            # (x, y, z) is in the camera's *optical* frame
            ```
        """
        info = self.get_camera_info(timeout=timeout)
        if not info:
            return None

        k = info.get("k") or info.get("K")
        if not k or len(k) < 9:
            return None

        return {
            "fx": k[0],
            "fy": k[4],
            "cx": k[2],
            "cy": k[5],
            "width": info.get("width"),
            "height": info.get("height"),
        }

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
