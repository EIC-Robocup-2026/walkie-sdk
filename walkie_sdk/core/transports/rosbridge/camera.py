"""
WebRTCCamera - WebRTC-based camera transport.

This camera transport receives video frames via WebRTC, typically paired
with the ROSBridgeTransport for remote robot communication.
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import TYPE_CHECKING, Any, Optional, Tuple

import numpy as np

from walkie_sdk.core.interfaces import CameraTransportInterface

# WebRTC dependencies - optional, check at runtime
WEBRTC_AVAILABLE = False

if TYPE_CHECKING:
    # For type checking only - these may not be installed
    from aiohttp import ClientSession
    from aiortc import RTCPeerConnection, RTCSessionDescription

try:
    import aiohttp
    import aiortc

    WEBRTC_AVAILABLE = True
except ImportError:
    aiohttp = None  # type: ignore
    aiortc = None  # type: ignore


class WebRTCCamera(CameraTransportInterface):
    """
    Camera transport implementation using WebRTC.

    Connects to a WebRTC signaling server on the robot and receives
    video frames with low latency. Typically used with ROSBridgeTransport.

    Args:
        host: Robot IP address or hostname
        port: WebRTC signaling server port (default: 8554)
        stun_server: STUN server URL for NAT traversal (optional)
        timeout: Connection timeout in seconds (default: 10.0)

    Example:
        camera = WebRTCCamera(host="192.168.1.100", port=8554)
        camera.connect()

        frame = camera.get_frame()
        if frame is not None:
            cv2.imshow("Robot Camera", frame)

        camera.disconnect()

    Note:
        Requires aiortc and aiohttp packages to be installed.
    """

    def __init__(
        self,
        host: str,
        port: int = 8554,
        stun_server: Optional[str] = None,
        timeout: float = 10.0,
    ):
        if not WEBRTC_AVAILABLE:
            raise ImportError(
                "WebRTC dependencies not installed. "
                "Install with: pip install aiortc aiohttp av"
            )

        self._host = host
        self._port = port
        self._stun_server = stun_server
        self._timeout = timeout

        # State
        self._pc: Any = None  # RTCPeerConnection when connected
        self._streaming = False
        self._lock = threading.Lock()

        # Frame buffer
        self._current_frame: Optional[np.ndarray] = None
        self._frame_shape: Optional[Tuple[int, int, int]] = None

        # Async event loop for WebRTC
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    @property
    def host(self) -> str:
        """Robot IP address or hostname."""
        return self._host

    @property
    def port(self) -> int:
        """WebRTC signaling server port."""
        return self._port

    @property
    def is_streaming(self) -> bool:
        """Check if camera is actively streaming."""
        return bool(self._streaming)

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        """Get frame dimensions (height, width, channels)."""
        with self._lock:
            return self._frame_shape

    def connect(self) -> None:
        """
        Start the WebRTC camera stream.

        Raises:
            ConnectionError: If unable to establish WebRTC connection
        """
        if self._streaming:
            return

        print(f"  → Connecting to WebRTC camera at {self._host}:{self._port}...")

        self._stop_event.clear()

        # Start async event loop in background thread
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()

        # Wait for connection with timeout
        deadline = time.time() + self._timeout
        while not self._streaming and time.time() < deadline:
            if self._stop_event.is_set():
                raise ConnectionError("WebRTC connection failed")
            time.sleep(0.1)

        if not self._streaming:
            self.disconnect()
            raise ConnectionError(
                f"WebRTC connection timeout after {self._timeout}s. "
                f"Is the WebRTC server running at {self._host}:{self._port}?"
            )

        print("  ✓ WebRTC camera connected")

    def disconnect(self) -> None:
        """Stop the WebRTC camera stream."""
        self._stop_event.set()

        if self._loop is not None:
            # Schedule cleanup in the async loop
            asyncio.run_coroutine_threadsafe(self._cleanup_async(), self._loop)

        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        self._streaming = False
        self._loop = None
        self._pc = None

        print("  ✓ WebRTC camera disconnected")

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest camera frame.

        Returns:
            BGR image as numpy array (HxWx3, uint8), or None if no frame available
        """
        with self._lock:
            if self._current_frame is not None:
                return self._current_frame.copy()
            return None

    def _run_async_loop(self) -> None:
        """Run the async event loop in a background thread."""
        try:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(self._connect_webrtc())
        except Exception as e:
            print(f"  ⚠ WebRTC error: {e}")
            self._stop_event.set()
        finally:
            if self._loop is not None:
                self._loop.close()

    async def _connect_webrtc(self) -> None:
        """Establish WebRTC connection and receive video frames."""
        # Import at runtime to avoid issues when not installed
        import aiohttp as aio
        from aiortc import RTCPeerConnection, RTCSessionDescription

        # Create peer connection
        if self._stun_server:
            from aiortc import RTCConfiguration, RTCIceServer

            config = RTCConfiguration(iceServers=[RTCIceServer(urls=self._stun_server)])
            self._pc = RTCPeerConnection(configuration=config)
        else:
            self._pc = RTCPeerConnection()

        pc = self._pc  # Local reference for closures

        # Handle incoming video track
        @pc.on("track")
        def on_track(track: Any) -> None:
            if track.kind == "video":
                asyncio.ensure_future(self._receive_frames(track))

        @pc.on("connectionstatechange")
        async def on_connection_state_change() -> None:
            state = pc.connectionState
            if state == "connected":
                self._streaming = True
            elif state in ("failed", "closed", "disconnected"):
                self._streaming = False
                self._stop_event.set()

        # Get offer from signaling server
        signaling_url = f"http://{self._host}:{self._port}/offer"

        try:
            async with aio.ClientSession() as session:
                # Create and send offer
                pc.addTransceiver("video", direction="recvonly")
                offer = await pc.createOffer()
                await pc.setLocalDescription(offer)

                async with session.post(
                    signaling_url,
                    json={
                        "sdp": pc.localDescription.sdp,
                        "type": pc.localDescription.type,
                    },
                    timeout=aio.ClientTimeout(total=self._timeout),
                ) as response:
                    if response.status != 200:
                        raise ConnectionError(
                            f"Signaling server returned {response.status}"
                        )

                    answer_data = await response.json()
                    answer = RTCSessionDescription(
                        sdp=answer_data["sdp"], type=answer_data["type"]
                    )
                    await pc.setRemoteDescription(answer)

        except aio.ClientError as e:
            raise ConnectionError(f"Failed to connect to signaling server: {e}")

        # Keep running until stop requested
        while not self._stop_event.is_set():
            await asyncio.sleep(0.1)

    async def _receive_frames(self, track) -> None:
        """Receive and decode video frames from the track."""
        while not self._stop_event.is_set():
            try:
                frame = await asyncio.wait_for(track.recv(), timeout=1.0)

                # Convert to numpy array (BGR format for OpenCV)
                img = frame.to_ndarray(format="bgr24")

                with self._lock:
                    self._current_frame = img
                    self._frame_shape = img.shape

            except asyncio.TimeoutError:
                continue
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"  ⚠ Frame receive error: {e}")
                break

    async def _cleanup_async(self) -> None:
        """Clean up async resources."""
        if self._pc is not None:
            await self._pc.close()

    def __repr__(self) -> str:
        status = "streaming" if self._streaming else "stopped"
        return f"WebRTCCamera(host='{self._host}', port={self._port}, status={status})"
