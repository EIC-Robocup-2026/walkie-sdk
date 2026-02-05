#!/usr/bin/env python3
"""
Simple WebRTC Camera Test Client

A standalone test script to verify WebRTC camera connectivity
without using the full Walkie SDK.

Usage:
    uv run python tests/test_webrtc_camera.py [host] [port]

Example:
    uv run python tests/test_webrtc_camera.py 192.168.1.100 8554

Requirements:
    - aiortc
    - aiohttp
    - opencv-python
    - av
"""

import argparse
import asyncio
import sys
import time
from typing import Optional

import cv2
import numpy as np

try:
    import aiohttp
    from aiortc import RTCPeerConnection, RTCSessionDescription
except ImportError:
    print("Error: Required packages not installed.")
    print("Install with: pip install aiortc aiohttp av")
    sys.exit(1)


class SimpleWebRTCCamera:
    """Simple WebRTC camera client for testing."""

    def __init__(self, host: str, port: int = 8554):
        self.host = host
        self.port = port
        self.signaling_url = f"http://{host}:{port}/offer"

        self.pc: Optional[RTCPeerConnection] = None
        self.current_frame: Optional[np.ndarray] = None
        self.frame_count = 0
        self.connected = False
        self._stop_event = asyncio.Event()
        self._frame_task: Optional[asyncio.Task] = None

    async def connect(self) -> bool:
        """Establish WebRTC connection."""
        print(f"Connecting to WebRTC server at {self.host}:{self.port}...")

        try:
            self.pc = RTCPeerConnection()
            self._stop_event.clear()

            # Handle incoming video track
            @self.pc.on("track")
            def on_track(track):
                print(f"Received track: {track.kind}")
                if track.kind == "video":
                    # Start frame receiving task
                    self._frame_task = asyncio.create_task(self._receive_frames(track))

            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                state = self.pc.connectionState
                print(f"Connection state: {state}")
                if state == "connected":
                    self.connected = True
                elif state in ("failed", "closed", "disconnected"):
                    self.connected = False
                    self._stop_event.set()

            # Create offer
            self.pc.addTransceiver("video", direction="recvonly")
            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)

            # Send offer to signaling server
            async with aiohttp.ClientSession() as session:
                print(f"Sending offer to {self.signaling_url}...")
                async with session.post(
                    self.signaling_url,
                    json={
                        "sdp": self.pc.localDescription.sdp,
                        "type": self.pc.localDescription.type,
                    },
                    timeout=aiohttp.ClientTimeout(total=10.0),
                ) as response:
                    if response.status != 200:
                        print(f"Error: Server returned {response.status}")
                        return False

                    answer_data = await response.json()
                    print("Received answer from server")

                    answer = RTCSessionDescription(
                        sdp=answer_data["sdp"], type=answer_data["type"]
                    )
                    await self.pc.setRemoteDescription(answer)

            print("WebRTC signaling complete!")
            return True

        except aiohttp.ClientError as e:
            print(f"Connection error: {e}")
            return False
        except Exception as e:
            print(f"Error: {e}")
            import traceback

            traceback.print_exc()
            return False

    async def _receive_frames(self, track):
        """Receive and store video frames."""
        print("Starting to receive video frames...")

        try:
            while not self._stop_event.is_set():
                try:
                    frame = await asyncio.wait_for(track.recv(), timeout=2.0)

                    # Convert to numpy array (BGR for OpenCV)
                    img = frame.to_ndarray(format="bgr24")
                    self.current_frame = img
                    self.frame_count += 1

                except asyncio.TimeoutError:
                    # No frame received, but keep trying
                    continue
                except Exception as e:
                    if not self._stop_event.is_set():
                        print(f"Frame receive error: {e}")
                    break
        finally:
            print("Stopped receiving frames")

    def get_frame(self) -> Optional[np.ndarray]:
        """Get the latest frame."""
        if self.current_frame is not None:
            return self.current_frame.copy()
        return None

    @property
    def is_running(self) -> bool:
        """Check if the camera is running."""
        return not self._stop_event.is_set()

    async def disconnect(self):
        """Close the WebRTC connection."""
        self._stop_event.set()

        # Wait for frame task to finish
        if self._frame_task is not None:
            try:
                await asyncio.wait_for(self._frame_task, timeout=2.0)
            except asyncio.TimeoutError:
                self._frame_task.cancel()
            self._frame_task = None

        if self.pc is not None:
            await self.pc.close()
            self.pc = None

        self.connected = False
        print("Disconnected")


async def run_camera_test(host: str, port: int):
    """Run the camera test with OpenCV display."""
    camera = SimpleWebRTCCamera(host, port)

    # Connect
    if not await camera.connect():
        print("Failed to connect!")
        return

    # Wait for first frame
    print("Waiting for video stream...")
    start_time = time.time()
    while camera.current_frame is None:
        if time.time() - start_time > 10.0:
            print("Timeout waiting for video stream")
            await camera.disconnect()
            return
        if not camera.is_running:
            print("Connection closed while waiting")
            await camera.disconnect()
            return
        await asyncio.sleep(0.1)

    print(f"First frame received! Shape: {camera.current_frame.shape}")

    # Display video
    print("\nDisplaying video feed...")
    print("Press 'q' to quit, 's' to save snapshot")

    window_name = "WebRTC Camera Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    fps_time = time.time()
    fps_count = 0
    fps = 0.0
    snapshot_num = 0

    try:
        while camera.is_running:
            frame = camera.get_frame()

            if frame is not None:
                fps_count += 1

                # Calculate FPS
                elapsed = time.time() - fps_time
                if elapsed >= 1.0:
                    fps = fps_count / elapsed
                    fps_count = 0
                    fps_time = time.time()

                # Add overlay
                display = frame.copy()
                h, w = display.shape[:2]

                # FPS
                cv2.putText(
                    display,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )

                # Resolution
                cv2.putText(
                    display,
                    f"Resolution: {w}x{h}",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )

                # Frame count
                cv2.putText(
                    display,
                    f"Frames: {camera.frame_count}",
                    (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )

                cv2.imshow(window_name, display)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("Quitting...")
                break
            elif key == ord("s"):
                if frame is not None:
                    snapshot_num += 1
                    filename = f"webrtc_snapshot_{snapshot_num:03d}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"Saved: {filename}")

            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        cv2.destroyAllWindows()
        await camera.disconnect()

    print(f"\nTotal frames received: {camera.frame_count}")


def main():
    parser = argparse.ArgumentParser(
        description="Simple WebRTC Camera Test Client",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python test_webrtc_camera.py 192.168.1.100
  python test_webrtc_camera.py 192.168.1.100 8554
  python test_webrtc_camera.py localhost 8554
        """,
    )
    parser.add_argument(
        "host",
        nargs="?",
        default="127.0.0.1",
        help="WebRTC server host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "port",
        nargs="?",
        type=int,
        default=8554,
        help="WebRTC server port (default: 8554)",
    )

    args = parser.parse_args()

    print("=" * 50)
    print("Simple WebRTC Camera Test Client")
    print("=" * 50)
    print(f"Host: {args.host}")
    print(f"Port: {args.port}")
    print()

    # Run async event loop
    asyncio.run(run_camera_test(args.host, args.port))


if __name__ == "__main__":
    main()
