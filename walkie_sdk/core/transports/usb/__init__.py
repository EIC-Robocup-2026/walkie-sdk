"""
Walkie SDK - USB Camera Transport

Local USB camera transport using OpenCV VideoCapture.
Supports auto-retry on device disconnect with exponential backoff.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple, Union

import numpy as np

try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    CV2_AVAILABLE = False

from walkie_sdk.core.interfaces import CameraTransportInterface


class USBCamera(CameraTransportInterface):
    """
    Camera transport for local USB/V4L2 cameras via OpenCV VideoCapture.

    Reads frames in a background thread and caches the latest frame
    for non-blocking retrieval. Automatically retries on device
    disconnect with exponential backoff.

    Args:
        device: Camera device identifier. Accepts:
            - int: Device index (e.g. 0). Simple but fragile -- index can
              change between reboots or when devices are plugged/unplugged.
            - str: Device path. Recommended stable options:
              - ``"/dev/v4l/by-id/usb-Logitech_C920-video-index0"``
                Stable per device; survives reboot and port changes.
              - ``"/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0"``
                Stable per physical USB port.
            Use ``ls /dev/v4l/by-id/`` to discover available cameras.
        width: Desired frame width (None = camera default)
        height: Desired frame height (None = camera default)
        fps: Desired capture FPS (None = camera default)
        max_retries: Maximum reconnection attempts before giving up (default: 5)
        retry_base_delay: Base delay in seconds for exponential backoff (default: 1.0)
    """

    def __init__(
        self,
        device: Union[int, str] = 0,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
        max_retries: int = 5,
        retry_base_delay: float = 1.0,
        **kwargs,
    ):
        if not CV2_AVAILABLE:
            raise ImportError(
                "opencv-python is required for USBCamera. "
                "Install it with: pip install opencv-python"
            )

        self._device = device
        self._width = width
        self._height = height
        self._fps = fps
        self._max_retries = max_retries
        self._retry_base_delay = retry_base_delay

        self._cap: Optional[cv2.VideoCapture] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._streaming = False
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def is_streaming(self) -> bool:
        return self._streaming

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        with self._frame_lock:
            if self._latest_frame is not None:
                return self._latest_frame.shape
        return None

    def connect(self) -> None:
        """Open the USB camera device and start the background read thread."""
        if self._streaming:
            return

        print(f"  -> Opening USB camera device: {self._device}...")

        self._cap = self._open_capture()
        if self._cap is None or not self._cap.isOpened():
            raise ConnectionError(f"Failed to open USB camera device: {self._device}")

        self._log_capture_info()

        # Start background frame reader
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._read_loop,
            name=f"USBCamera-{self._device}",
            daemon=True,
        )
        self._thread.start()
        self._streaming = True
        print(f"  -> USB Camera Stream Started (device={self._device})")

    def disconnect(self) -> None:
        """Stop reading and release the camera device."""
        self._streaming = False
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

        if self._cap is not None:
            self._cap.release()
            self._cap = None

        with self._frame_lock:
            self._latest_frame = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._frame_lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
        return None

    # -- Internal methods --

    def _open_capture(self) -> Optional[cv2.VideoCapture]:
        """Create and configure a VideoCapture instance."""
        cap = cv2.VideoCapture(self._device)
        if not cap.isOpened():
            return None

        if self._width is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        if self._height is not None:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        if self._fps is not None:
            cap.set(cv2.CAP_PROP_FPS, self._fps)

        return cap

    def _log_capture_info(self) -> None:
        """Log the actual resolution and FPS negotiated by the device."""
        if self._cap is None:
            return
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self._cap.get(cv2.CAP_PROP_FPS)
        print(f"  -> USB Camera: {w}x{h} @ {fps:.1f} FPS")

    def _read_loop(self) -> None:
        """
        Background thread: continuously read frames from the capture device.
        On failure, attempts to reconnect with exponential backoff.
        """
        consecutive_failures = 0

        while not self._stop_event.is_set():
            if self._cap is None or not self._cap.isOpened():
                # Attempt reconnection
                if not self._try_reconnect():
                    # All retries exhausted
                    print(
                        f"[USBCamera] Device {self._device}: "
                        f"reconnection failed after {self._max_retries} attempts. Giving up."
                    )
                    self._streaming = False
                    return
                consecutive_failures = 0
                continue

            ret, frame = self._cap.read()

            if not ret or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= 10:
                    print(
                        f"[USBCamera] Device {self._device}: "
                        f"{consecutive_failures} consecutive read failures, attempting reconnect..."
                    )
                    # Release and let the reconnect logic handle it
                    self._cap.release()
                    self._cap = None
                    consecutive_failures = 0
                continue

            consecutive_failures = 0
            with self._frame_lock:
                self._latest_frame = frame

    def _try_reconnect(self) -> bool:
        """
        Attempt to reopen the camera device with exponential backoff.

        Returns:
            True if reconnected successfully, False if all retries exhausted.
        """
        for attempt in range(1, self._max_retries + 1):
            if self._stop_event.is_set():
                return False

            delay = self._retry_base_delay * (2 ** (attempt - 1))
            print(
                f"[USBCamera] Device {self._device}: "
                f"retry {attempt}/{self._max_retries} in {delay:.1f}s..."
            )

            # Wait with early exit if stop is requested
            if self._stop_event.wait(timeout=delay):
                return False

            # Release any existing capture
            if self._cap is not None:
                self._cap.release()
                self._cap = None

            self._cap = self._open_capture()
            if self._cap is not None and self._cap.isOpened():
                print(
                    f"[USBCamera] Device {self._device}: reconnected on attempt {attempt}"
                )
                self._log_capture_info()
                return True

        return False


__all__ = ["USBCamera"]
