"""
Walkie SDK - Shared Memory Transport

Shared memory camera transport for direct access to camera frames
from Isaac Sim on the same host. Provides zero-copy frame access
for maximum performance.
"""

from __future__ import annotations

import ctypes
import threading
from multiprocessing import shared_memory
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from walkie_sdk.core.interfaces import CameraTransportInterface

# Constants (must match simulation side)
SHM_SIZE_PER_IMAGE = 4 * 1024 * 1024  # 4MB per image
SHM_PREFIX = "walkie_cam_"


class SimpleImageHeader(ctypes.Structure):
    """Header structure for shared memory image data.

    Must match the structure in tools/shared_memory_utils.py
    """

    _fields_ = [
        ("timestamp", ctypes.c_uint64),
        ("height", ctypes.c_uint32),
        ("width", ctypes.c_uint32),
        ("channels", ctypes.c_uint32),
        ("encoding", ctypes.c_uint32),  # 0=raw, 1=JPEG
        ("quality", ctypes.c_uint32),
        ("data_size", ctypes.c_uint32),
        ("image_name", ctypes.c_char * 16),
    ]


def get_shm_name(image_name: str) -> str:
    """Get shared memory name for a camera image."""
    return f"{SHM_PREFIX}{image_name}"


class SharedMemoryCamera(CameraTransportInterface):
    """
    Camera transport using shared memory for same-host access.

    Provides direct access to camera frames from Isaac Sim via
    Linux shared memory. This is the fastest transport option
    for same-host scenarios (simulation + SDK on same machine).

    Args:
        camera_name: Name of the camera ("head", "left", or "right")

    Example:
        camera = SharedMemoryCamera("head")
        camera.connect()
        frame = camera.get_frame()
        camera.disconnect()
    """

    def __init__(
        self,
        camera_name: str = "head",
        **kwargs,
    ):
        self._camera_name = camera_name
        self._shm: Optional[shared_memory.SharedMemory] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._last_timestamp: int = 0
        self._frame_lock = threading.Lock()
        self._streaming = False

    @property
    def camera_name(self) -> str:
        """Name of the camera."""
        return self._camera_name

    @property
    def is_streaming(self) -> bool:
        """Check if connected to shared memory."""
        return self._streaming

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        """Get frame dimensions."""
        with self._frame_lock:
            if self._latest_frame is not None:
                return self._latest_frame.shape
        return None

    def connect(self) -> None:
        """Connect to shared memory."""
        try:
            shm_name = get_shm_name(self._camera_name)
            self._shm = shared_memory.SharedMemory(name=shm_name)
            self._streaming = True
            print(f"  ✓ Connected to shared memory camera: {shm_name}")
        except FileNotFoundError:
            raise ConnectionError(
                f"Shared memory '{get_shm_name(self._camera_name)}' not found. "
                f"Is the simulation running with cameras enabled?"
            )
        except Exception as e:
            raise ConnectionError(f"Failed to connect to shared memory: {e}")

    def disconnect(self) -> None:
        """Disconnect from shared memory."""
        if self._shm:
            # try:
            # self._shm.close()
            # except Exception:
            # pass
            self._shm = None
        self._streaming = False

    def get_frame(self) -> Optional[np.ndarray]:
        """Get the latest camera frame from shared memory.

        Returns:
            BGR image as numpy array, or None if no frame available
        """
        if not self._shm:
            return None

        try:
            header_size = ctypes.sizeof(SimpleImageHeader)

            # Read header
            header_data = bytes(self._shm.buf[:header_size])
            header = SimpleImageHeader.from_buffer_copy(header_data)

            # Check for new data
            if header.timestamp <= self._last_timestamp:
                # Return cached frame
                with self._frame_lock:
                    return (
                        self._latest_frame.copy()
                        if self._latest_frame is not None
                        else None
                    )

            # Read payload
            data_start = header_size
            data_end = data_start + header.data_size
            payload = bytes(self._shm.buf[data_start:data_end])

            # Decode image
            if header.encoding == 1:  # JPEG
                if not HAS_CV2:
                    print("[SharedMemoryCamera] cv2 not available, cannot decode JPEG")
                    return None
                encoded = np.frombuffer(payload, dtype=np.uint8)
                frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if frame is None:
                    return None
            else:  # RAW
                frame = np.frombuffer(payload, dtype=np.uint8)
                expected_size = header.height * header.width * header.channels
                if frame.size != expected_size:
                    return None
                frame = frame.reshape(header.height, header.width, header.channels)

            # Update cache
            with self._frame_lock:
                self._latest_frame = frame
                self._last_timestamp = header.timestamp

            return frame.copy()

        except Exception as e:
            print(f"[SharedMemoryCamera] Error reading frame: {e}")
            return None

    def get_timestamp(self) -> Optional[int]:
        """Get the timestamp of the last frame.

        Returns:
            Timestamp in milliseconds, or None if no frame received
        """
        return self._last_timestamp if self._last_timestamp > 0 else None


class MultiSharedMemoryCamera(CameraTransportInterface):
    """
    Multi-camera transport using shared memory.

    Provides access to multiple cameras (head, left, right) via
    shared memory. Each camera has its own shared memory segment.

    Args:
        camera_names: List of camera names to access

    Example:
        cameras = MultiSharedMemoryCamera(["head", "left", "right"])
        cameras.connect()
        frames = cameras.get_all_frames()
        cameras.disconnect()
    """

    def __init__(
        self,
        camera_names: Optional[List[str]] = None,
        **kwargs,
    ):
        self._camera_names = camera_names or ["head", "left", "right"]
        self._cameras: Dict[str, SharedMemoryCamera] = {}
        self._streaming = False

    @property
    def camera_names(self) -> List[str]:
        """List of camera names."""
        return self._camera_names

    @property
    def is_streaming(self) -> bool:
        """Check if at least one camera is connected."""
        return any(cam.is_streaming for cam in self._cameras.values())

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        """Get frame dimensions of first available camera."""
        for camera in self._cameras.values():
            shape = camera.frame_shape
            if shape is not None:
                return shape
        return None

    def connect(self) -> None:
        """Connect to all camera shared memories."""
        connected = []
        for name in self._camera_names:
            try:
                camera = SharedMemoryCamera(name)
                camera.connect()
                self._cameras[name] = camera
                connected.append(name)
            except ConnectionError:
                print(f"  ! Camera '{name}' not available")

        if not connected:
            raise ConnectionError(
                "No cameras available. Is the simulation running with cameras enabled?"
            )

        self._streaming = True
        print(f"  ✓ Connected to cameras: {connected}")

    def disconnect(self) -> None:
        """Disconnect from all cameras."""
        for camera in self._cameras.values():
            camera.disconnect()
        self._cameras.clear()
        self._streaming = False

    def get_frame(self, camera_name: str = "head") -> Optional[np.ndarray]:
        """Get the latest frame from a specific camera.

        Args:
            camera_name: Name of the camera ("head", "left", or "right")

        Returns:
            BGR image as numpy array, or None if not available
        """
        if camera_name in self._cameras:
            return self._cameras[camera_name].get_frame()
        return None

    def get_head_frame(self) -> Optional[np.ndarray]:
        """Get the latest head/front camera frame."""
        return self.get_frame("head")

    def get_left_frame(self) -> Optional[np.ndarray]:
        """Get the latest left wrist camera frame."""
        return self.get_frame("left")

    def get_right_frame(self) -> Optional[np.ndarray]:
        """Get the latest right wrist camera frame."""
        return self.get_frame("right")

    def get_all_frames(self) -> Dict[str, np.ndarray]:
        """Get the latest frames from all connected cameras.

        Returns:
            Dictionary mapping camera name to frame
        """
        frames = {}
        for name, camera in self._cameras.items():
            frame = camera.get_frame()
            if frame is not None:
                frames[name] = frame
        return frames


__all__ = [
    "SharedMemoryCamera",
    "MultiSharedMemoryCamera",
    "get_shm_name",
]
