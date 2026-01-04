#!/usr/bin/env python3
"""
WebRTC Test Server

A simple WebRTC server that streams webcam video or a test pattern.
Used for testing the Walkie SDK WebRTC camera client.

Usage:
    uv run python tests/webrtc_test_server.py [--port PORT] [--source SOURCE]

Options:
    --port PORT       Server port (default: 8554)
    --source SOURCE   Video source: 'webcam', 'test', or device index (default: webcam)

Example:
    # Stream webcam on default port
    uv run python tests/webrtc_test_server.py

    # Stream test pattern on port 8080
    uv run python tests/webrtc_test_server.py --port 8080 --source test

    # Stream from specific camera device
    uv run python tests/webrtc_test_server.py --source 0

Requirements:
    - aiortc
    - aiohttp
    - opencv-python
    - av
"""

import argparse
import asyncio
import json
import time
from typing import Optional

import cv2
import numpy as np
from aiohttp import web

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
    from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRelay
    from av import VideoFrame
except ImportError:
    print("Error: Required packages not installed.")
    print("Install with: pip install aiortc aiohttp av opencv-python")
    exit(1)


class WebcamVideoTrack(VideoStreamTrack):
    """
    Video track that captures from a webcam using OpenCV.
    """

    kind = "video"

    def __init__(self, device: int = 0):
        super().__init__()
        self.device = device
        self.cap: Optional[cv2.VideoCapture] = None
        self.frame_count = 0
        self._start_time = time.time()

    async def recv(self):
        """Receive the next video frame."""
        # Initialize capture on first frame
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.device)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open camera device {self.device}")
            print(f"Opened camera device {self.device}")

        # Calculate timestamp
        pts, time_base = await self.next_timestamp()

        # Read frame from webcam
        ret, frame = self.cap.read()
        if not ret:
            # If read fails, return a black frame
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Convert BGR to RGB for av
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create VideoFrame
        video_frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        self.frame_count += 1
        return video_frame

    def stop(self):
        """Release the camera."""
        super().stop()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            print("Camera released")


class TestPatternTrack(VideoStreamTrack):
    """
    Video track that generates a test pattern.
    """

    kind = "video"

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        super().__init__()
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_count = 0
        self._start_time = time.time()

    async def recv(self):
        """Generate the next test pattern frame."""
        pts, time_base = await self.next_timestamp()

        # Create test pattern
        frame = self._create_test_pattern()

        # Convert to VideoFrame
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        self.frame_count += 1
        return video_frame

    def _create_test_pattern(self) -> np.ndarray:
        """Create a colorful test pattern with moving elements."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Color bars (top third)
        bar_width = self.width // 8
        colors = [
            (255, 255, 255),  # White
            (255, 255, 0),  # Yellow
            (0, 255, 255),  # Cyan
            (0, 255, 0),  # Green
            (255, 0, 255),  # Magenta
            (255, 0, 0),  # Red
            (0, 0, 255),  # Blue
            (0, 0, 0),  # Black
        ]
        for i, color in enumerate(colors):
            x1 = i * bar_width
            x2 = (i + 1) * bar_width
            frame[0 : self.height // 3, x1:x2] = color

        # Gradient (middle third)
        for x in range(self.width):
            gray = int(255 * x / self.width)
            frame[self.height // 3 : 2 * self.height // 3, x] = (gray, gray, gray)

        # Moving circle (bottom third)
        elapsed = time.time() - self._start_time
        circle_x = int((self.width / 2) + (self.width / 3) * np.sin(elapsed * 2))
        circle_y = int(self.height * 5 / 6)
        cv2.circle(frame, (circle_x, circle_y), 30, (0, 255, 0), -1)

        # Frame counter text
        text = f"Frame: {self.frame_count}"
        cv2.putText(
            frame,
            text,
            (10, self.height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        # Timestamp
        timestamp = f"Time: {elapsed:.1f}s"
        cv2.putText(
            frame,
            timestamp,
            (10, self.height - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        # Title
        title = "WebRTC Test Pattern"
        cv2.putText(
            frame,
            title,
            (self.width // 2 - 120, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        return frame


class WebRTCServer:
    """Simple WebRTC signaling server."""

    def __init__(self, video_source: str = "webcam", port: int = 8554):
        self.port = port
        self.video_source = video_source
        self.pcs: set[RTCPeerConnection] = set()
        self.video_track: Optional[VideoStreamTrack] = None

    def _create_video_track(self) -> VideoStreamTrack:
        """Create video track based on source configuration."""
        if self.video_source == "test":
            print("Using test pattern source")
            return TestPatternTrack()
        elif self.video_source == "webcam":
            print("Using webcam source (device 0)")
            return WebcamVideoTrack(device=0)
        else:
            # Assume it's a device index
            try:
                device = int(self.video_source)
                print(f"Using webcam source (device {device})")
                return WebcamVideoTrack(device=device)
            except ValueError:
                print(f"Unknown source '{self.video_source}', using test pattern")
                return TestPatternTrack()

    async def handle_offer(self, request: web.Request) -> web.Response:
        """Handle incoming WebRTC offer."""
        try:
            params = await request.json()
            offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        except Exception as e:
            return web.Response(
                status=400,
                content_type="application/json",
                text=json.dumps({"error": str(e)}),
            )

        pc = RTCPeerConnection()
        self.pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"Connection state: {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)
            elif pc.connectionState == "closed":
                self.pcs.discard(pc)

        # Create and add video track
        video_track = self._create_video_track()
        pc.addTrack(video_track)

        # Set remote description (offer)
        await pc.setRemoteDescription(offer)

        # Create answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        print(f"Sending answer to client (total connections: {len(self.pcs)})")

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )

    async def handle_index(self, request: web.Request) -> web.Response:
        """Serve a simple status page."""
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>WebRTC Test Server</title>
            <style>
                body {{ font-family: Arial, sans-serif; margin: 40px; }}
                .status {{ color: green; }}
                code {{ background: #f0f0f0; padding: 2px 6px; }}
            </style>
        </head>
        <body>
            <h1>WebRTC Test Server</h1>
            <p class="status">✓ Server is running</p>
            <ul>
                <li>Port: <code>{self.port}</code></li>
                <li>Video Source: <code>{self.video_source}</code></li>
                <li>Active Connections: <code>{len(self.pcs)}</code></li>
            </ul>
            <h2>Endpoints</h2>
            <ul>
                <li><code>POST /offer</code> - WebRTC signaling endpoint</li>
                <li><code>GET /</code> - This status page</li>
            </ul>
            <h2>Test Client</h2>
            <p>Run the test client:</p>
            <pre>uv run python tests/test_webrtc_camera.py localhost {self.port}</pre>
        </body>
        </html>
        """
        return web.Response(content_type="text/html", text=html)

    async def on_shutdown(self, app: web.Application):
        """Cleanup on server shutdown."""
        print("Shutting down, closing peer connections...")
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()
        print("All connections closed")

    def run(self):
        """Start the server."""
        app = web.Application()
        app.router.add_get("/", self.handle_index)
        app.router.add_post("/offer", self.handle_offer)
        app.on_shutdown.append(self.on_shutdown)

        print("=" * 50)
        print("WebRTC Test Server")
        print("=" * 50)
        print(f"Video Source: {self.video_source}")
        print(f"Listening on: http://0.0.0.0:{self.port}")
        print(f"Signaling endpoint: http://localhost:{self.port}/offer")
        print()
        print("To test, run:")
        print(f"  uv run python tests/test_webrtc_camera.py localhost {self.port}")
        print()
        print("Press Ctrl+C to stop")
        print("=" * 50)

        web.run_app(app, host="0.0.0.0", port=self.port, print=None)


def main():
    parser = argparse.ArgumentParser(
        description="WebRTC Test Server - streams webcam or test pattern",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Video Sources:
  webcam    Use default webcam (device 0)
  test      Generate a test pattern
  0, 1, ... Use specific camera device index

Examples:
  python webrtc_test_server.py
  python webrtc_test_server.py --port 8080
  python webrtc_test_server.py --source test
  python webrtc_test_server.py --source 1
        """,
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8554,
        help="Server port (default: 8554)",
    )
    parser.add_argument(
        "--source",
        default="webcam",
        help="Video source: 'webcam', 'test', or device index (default: webcam)",
    )

    args = parser.parse_args()

    server = WebRTCServer(video_source=args.source, port=args.port)
    server.run()


if __name__ == "__main__":
    main()
