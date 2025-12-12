"""
WebRTCClient - Video streaming client for robot camera.

Connects to the robot's WebRTC server and decodes video frames
in a background thread, providing synchronous access to the latest frame.
"""

import asyncio
import threading
import time
from typing import Optional
import json

import numpy as np
import cv2

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
    from aiortc.contrib.media import MediaRecorder
    from av import VideoFrame
    AIORTC_AVAILABLE = True
except ImportError:
    AIORTC_AVAILABLE = False

try:
    import aiohttp
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False


class WebRTCClient:
    """
    WebRTC video streaming client.
    
    Connects to the robot's WebRTC server and provides access to video frames.
    Runs frame decoding in a background thread with an async event loop.
    
    Args:
        host: Robot IP address or hostname
        port: WebRTC signaling port (default: 8554)
        timeout: Connection timeout in seconds (default: 10.0)
    
    Raises:
        ConnectionError: If connection fails or times out
        ImportError: If aiortc is not installed
    """
    
    def __init__(self, host: str, port: int = 8554, timeout: float = 10.0):
        if not AIORTC_AVAILABLE:
            raise ImportError(
                "aiortc is required for WebRTC support. "
                "Install with: pip install aiortc"
            )
        
        self._host = host
        self._port = port
        self._timeout = timeout
        
        self._pc: Optional[RTCPeerConnection] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._connected = False
        self._running = False
        
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
    
    @property
    def host(self) -> str:
        """Robot IP address or hostname."""
        return self._host
    
    @property
    def port(self) -> int:
        """WebRTC signaling port."""
        return self._port
    
    @property
    def is_connected(self) -> bool:
        """Check if connected to WebRTC stream."""
        return self._connected
    
    def connect(self) -> None:
        """
        Connect to the WebRTC server and start receiving frames.
        
        Blocks until connected or timeout. Prints status feedback.
        
        Raises:
            ConnectionError: If connection fails or times out
        """
        print(f"  → Connecting to WebRTC at {self._host}:{self._port}...")
        
        self._running = True
        self._connection_error: Optional[Exception] = None
        self._connection_event = threading.Event()
        
        # Start background thread with async event loop
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        
        # Wait for connection with timeout
        if not self._connection_event.wait(timeout=self._timeout):
            self._running = False
            raise ConnectionError(
                f"WebRTC connection timeout after {self._timeout}s. "
                f"Is WebRTC server running at {self._host}:{self._port}?"
            )
        
        if self._connection_error:
            self._running = False
            raise ConnectionError(
                f"Failed to connect to WebRTC: {self._connection_error}"
            )
        
        print(f"  ✓ WebRTC stream connected")
    
    def _run_async_loop(self) -> None:
        """Run the async event loop in background thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        try:
            self._loop.run_until_complete(self._connect_and_receive())
        except Exception as e:
            self._connection_error = e
            self._connection_event.set()
        finally:
            self._loop.close()
    
    async def _connect_and_receive(self) -> None:
        """Async coroutine to connect and receive video frames."""
        try:
            self._pc = RTCPeerConnection()
            
            @self._pc.on("track")
            async def on_track(track: MediaStreamTrack):
                if track.kind == "video":
                    self._connected = True
                    self._connection_event.set()
                    
                    while self._running:
                        try:
                            frame = await asyncio.wait_for(
                                track.recv(),
                                timeout=1.0
                            )
                            # Convert to numpy BGR array
                            img = frame.to_ndarray(format="bgr24")
                            with self._frame_lock:
                                self._latest_frame = img
                        except asyncio.TimeoutError:
                            continue
                        except Exception:
                            break
            
            # Create offer and connect to signaling server
            await self._perform_signaling()
            
            # Keep running while connected
            while self._running and self._connected:
                await asyncio.sleep(0.1)
                
        except Exception as e:
            self._connection_error = e
            self._connection_event.set()
            raise
    
    async def _perform_signaling(self) -> None:
        """
        Perform WebRTC signaling with the server.
        
        Sends offer to HTTP signaling endpoint and receives answer.
        """
        # Add transceiver to receive video
        self._pc.addTransceiver("video", direction="recvonly")
        
        # Create offer
        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)
        
        # Send offer to signaling server via HTTP POST
        signaling_url = f"http://{self._host}:{self._port}/offer"
        
        try:
            import aiohttp
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    signaling_url,
                    json={
                        "sdp": self._pc.localDescription.sdp,
                        "type": self._pc.localDescription.type
                    },
                    timeout=aiohttp.ClientTimeout(total=self._timeout)
                ) as response:
                    if response.status != 200:
                        raise ConnectionError(
                            f"Signaling server returned status {response.status}"
                        )
                    answer_data = await response.json()
                    
            # Set remote description from answer
            answer = RTCSessionDescription(
                sdp=answer_data["sdp"],
                type=answer_data["type"]
            )
            await self._pc.setRemoteDescription(answer)
            
        except ImportError:
            # Fallback using urllib if aiohttp not available
            import urllib.request
            import urllib.error
            
            request_data = json.dumps({
                "sdp": self._pc.localDescription.sdp,
                "type": self._pc.localDescription.type
            }).encode("utf-8")
            
            req = urllib.request.Request(
                signaling_url,
                data=request_data,
                headers={"Content-Type": "application/json"}
            )
            
            try:
                with urllib.request.urlopen(req, timeout=self._timeout) as response:
                    answer_data = json.loads(response.read().decode("utf-8"))
            except urllib.error.URLError as e:
                raise ConnectionError(f"Failed to connect to signaling server: {e}")
            
            answer = RTCSessionDescription(
                sdp=answer_data["sdp"],
                type=answer_data["type"]
            )
            await self._pc.setRemoteDescription(answer)
    
    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest video frame.
        
        Returns:
            numpy array (BGR format, shape HxWx3) or None if no frame available
        """
        with self._frame_lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None
    
    def disconnect(self) -> None:
        """Disconnect from the WebRTC server."""
        self._running = False
        self._connected = False
        
        if self._pc is not None and self._loop is not None:
            # Close peer connection in the async loop
            future = asyncio.run_coroutine_threadsafe(
                self._pc.close(),
                self._loop
            )
            try:
                future.result(timeout=2.0)
            except Exception:
                pass
            self._pc = None
        
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        
        self._latest_frame = None
        print(f"  ✓ WebRTC disconnected")
    
    def __enter__(self) -> "WebRTCClient":
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.disconnect()
