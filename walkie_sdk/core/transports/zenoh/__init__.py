"""
Walkie SDK - Zenoh Transport

Zenoh-based transport implementation for ROS2 communication.
This transport uses Zenoh for low-latency communication
without requiring ROS2 to be installed on the client machine.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

try:
    import zenoh

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None

from walkie_sdk.core.interfaces import CameraTransportInterface, ROSTransportInterface


class ZenohTransport(ROSTransportInterface[Any]):
    """
    ROS transport implementation using Zenoh.

    This transport provides an alternative to rosbridge with potentially
    better performance characteristics for edge computing scenarios.

    Args:
        host: Robot IP address or hostname (or Zenoh router address)
        port: Zenoh router port (default: 7447)
        timeout: Connection timeout in seconds (default: 10.0)
    """

    def __init__(
        self,
        host: str,
        port: int = 7447,
        timeout: float = 10.0,
        **kwargs,
    ):
        if not ZENOH_AVAILABLE:
            raise ImportError(
                "zenoh-python is not installed. Install with: pip install zenoh-python"
            )

        self._host = host
        self._port = port
        self._timeout = timeout
        self._session: Optional[Any] = None
        self._lock = threading.Lock()
        self._subscribers: Dict[str, Any] = {}
        self._publishers: Dict[str, Any] = {}

        # Track current action for cancellation
        self._current_goal: Optional[Any] = None
        self._current_action_client: Optional[Any] = None

    @property
    def host(self) -> str:
        """Robot IP address or hostname."""
        return self._host

    @property
    def port(self) -> int:
        """Zenoh router port."""
        return self._port

    @property
    def is_connected(self) -> bool:
        """Check if connected to Zenoh."""
        return self._session is not None

    def connect(self) -> None:
        """Connect to Zenoh router."""
        print(f"  → Connecting to Zenoh at {self._host}:{self._port}...")

        try:
            # Configure Zenoh session using string-based keys (zenoh>=1.0 API)
            conf = zenoh.Config()
            # Enable shared memory transport
            conf.insert_json5("transport/shared_memory/enabled", "true")
            if self._host != "localhost" and self._host != "127.0.0.1":
                conf.insert_json5("mode", json.dumps("client"))
                conf.insert_json5(
                    "connect/endpoints", json.dumps([f"tcp/{self._host}:{self._port}"])
                )
            else:
                # Peer mode for local transport (no router required)
                conf.insert_json5("mode", json.dumps("peer"))

            self._session = zenoh.open(conf)

            # Wait for connection
            start_time = time.time()
            while not self._session:
                if time.time() - start_time > self._timeout:
                    raise ConnectionError(
                        f"Connection timeout after {self._timeout}s. "
                        f"Is Zenoh router running at {self._host}:{self._port}?"
                    )
                time.sleep(0.1)

            print(f"  ✓ Connected to Zenoh")
        except Exception as e:
            self._session = None
            raise ConnectionError(f"Failed to connect to Zenoh: {e}")

    def disconnect(self) -> None:
        """Close Zenoh connection."""
        with self._lock:
            # Close all subscribers and publishers
            for sub in self._subscribers.values():
                try:
                    sub.close()
                except Exception:
                    pass
            for pub in self._publishers.values():
                try:
                    pub.close()
                except Exception:
                    pass

            self._subscribers.clear()
            self._publishers.clear()

            if self._session:
                try:
                    self._session.close()
                except Exception:
                    pass
                self._session = None

    def subscribe(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[Dict[str, Any]], None],
        throttle_rate: int = 0,
        queue_size: int = 1,
    ) -> Any:
        """Subscribe to a ROS topic via Zenoh."""
        if not self._session:
            raise ConnectionError("Not connected to Zenoh")

        # Convert ROS topic to Zenoh key (remove leading / if present)
        zenoh_key = topic.lstrip("/")

        print(
            f"[ZenohTransport] Subscribing to zenoh key: '{zenoh_key}' (from topic: '{topic}')"
        )

        # Debug: track message count
        msg_count = [0]

        def zenoh_callback(sample):
            msg_count[0] += 1
            try:
                payload = sample.payload
                if payload:
                    # Decode JSON message
                    data_str = bytes(payload).decode("utf-8")
                    msg_dict = json.loads(data_str)
                    if msg_count[0] == 1:
                        print(
                            f"[ZenohTransport] First message on '{zenoh_key}': keys={list(msg_dict.keys())}"
                        )
                    callback(msg_dict)
            except Exception as e:
                print(f"[ZenohTransport] Error processing message on {topic}: {e}")

        with self._lock:
            subscriber = self._session.declare_subscriber(zenoh_key, zenoh_callback)
            self._subscribers[topic] = subscriber

        print(f"[ZenohTransport] Successfully subscribed to '{zenoh_key}'")
        return subscriber

    def unsubscribe(self, handle: Any) -> None:
        """Unsubscribe from a topic."""
        with self._lock:
            # Find and remove subscriber
            for topic, sub in list(self._subscribers.items()):
                if sub == handle:
                    try:
                        sub.close()
                    except Exception:
                        pass
                    del self._subscribers[topic]
                    break

    def publish(
        self,
        topic: str,
        message_type: str,
        message: Dict[str, Any],
    ) -> None:
        """Publish a message to a ROS topic via Zenoh."""
        if not self._session:
            raise ConnectionError("Not connected to Zenoh")

        # Convert ROS topic to Zenoh key
        zenoh_key = topic.lstrip("/")

        # Get or create publisher
        with self._lock:
            if topic not in self._publishers:
                publisher = self._session.declare_publisher(zenoh_key)
                self._publishers[topic] = publisher
            else:
                publisher = self._publishers[topic]

        # Serialize message to JSON and publish
        try:
            message_json = json.dumps(message)
            publisher.put(message_json.encode("utf-8"))
        except Exception as e:
            print(f"[ZenohTransport] Error publishing to {topic}: {e}")
            raise

    def call_action(
        self,
        action_name: str,
        action_type: str,
        goal: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Call a ROS2 action via Zenoh (using queryable pattern)."""
        if not self._session:
            raise ConnectionError("Not connected to Zenoh")

        # For actions, we use Zenoh's queryable pattern
        # This is a simplified implementation
        action_key = action_name.lstrip("/")

        # Publish goal and wait for result
        goal_json = json.dumps(goal)

        # TODO: Implement proper action call pattern with Zenoh queryables
        # For now, return a placeholder
        return {"result": None, "status": "UNKNOWN"}

    def cancel_action(self) -> None:
        """Cancel the current action goal."""
        # TODO: Implement action cancellation
        pass

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request: Dict[str, Any],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Call a ROS service via Zenoh (using queryable pattern)."""
        if not self._session:
            raise ConnectionError("Not connected to Zenoh")

        # Use Zenoh queryable for service calls
        service_key = service_name.lstrip("/")

        try:
            request_json = json.dumps(request)
            # Query Zenoh for service response
            replies = self._session.get(
                service_key, request_json.encode("utf-8"), zenoh.Queue()
            )

            # Wait for response with timeout
            start_time = time.time()
            for reply in replies:
                if time.time() - start_time > timeout:
                    raise TimeoutError(f"Service call to {service_name} timed out")

                if reply.is_ok:
                    payload = reply.payload
                    if payload:
                        response_str = bytes(payload).decode("utf-8")
                        return json.loads(response_str)

            raise TimeoutError(f"Service call to {service_name} timed out")
        except Exception as e:
            print(f"[ZenohTransport] Error calling service {service_name}: {e}")
            raise


class ZenohCamera(CameraTransportInterface):
    """
    Camera transport implementation using Zenoh.

    Supports both single-camera and multi-camera modes. For multi-camera,
    subscribes to multiple topics (head, left, right).

    Args:
        host: Robot IP address or hostname
        port: Zenoh port for video stream
        topic: Zenoh topic for camera frames (single camera mode)
        camera_name: Camera name for multi-camera mode ("head", "left", "right")
    """

    CAMERA_TOPICS = {
        "head": "walkie/camera/head",
        "left": "walkie/camera/left",
        "right": "walkie/camera/right",
    }

    def __init__(
        self,
        host: str,
        port: int = 7447,
        topic: str = "walkie/camera/image",
        camera_name: str = "head",
        multi_camera: bool = False,
        **kwargs,
    ):
        if not ZENOH_AVAILABLE:
            raise ImportError(
                "zenoh-python is not installed. Install with: pip install zenoh-python"
            )

        self._host = host
        self._port = port
        self._topic = topic
        self._camera_name = camera_name
        self._session: Optional[Any] = None
        self._subscribers: Dict[str, Any] = {}
        self._latest_frames: Dict[str, np.ndarray] = {}
        self._frame_lock = threading.Lock()
        self._streaming = False

    @property
    def is_streaming(self) -> bool:
        """Check if camera stream is active."""
        return self._streaming

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        """Get frame dimensions."""
        with self._frame_lock:
            # Return shape of first available frame
            for frame in self._latest_frames.values():
                if frame is not None:
                    return frame.shape
        return None

    def connect(self) -> None:
        """Connect to Zenoh and start camera stream."""
        try:
            # Configure Zenoh session using string-based keys (zenoh>=1.0 API)
            conf = zenoh.Config()
            # Enable shared memory transport
            conf.insert_json5("transport/shared_memory/enabled", "true")
            if self._host != "localhost" and self._host != "127.0.0.1":
                conf.insert_json5("mode", json.dumps("client"))
                conf.insert_json5(
                    "connect/endpoints", json.dumps([f"tcp/{self._host}:{self._port}"])
                )
            else:
                conf.insert_json5("mode", json.dumps("peer"))

            self._session = zenoh.open(conf)

            # Create frame callback
            def make_frame_callback(camera_name: str):
                def frame_callback(sample):
                    try:
                        import cv2

                        payload = sample.payload
                        if payload:
                            img_bytes = bytes(payload)

                            # Check for JPEG (0xFF 0xD8)
                            if img_bytes.startswith(b"\xff\xd8"):
                                nparr = np.frombuffer(img_bytes, np.uint8)
                                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                            else:
                                # Try to decode as raw with 12-byte header (h, w, c)
                                if len(img_bytes) > 12:
                                    header = np.frombuffer(
                                        img_bytes[:12], dtype=np.uint32
                                    )
                                    h, w, c = header
                                    if h * w * c == len(img_bytes) - 12:
                                        frame = np.frombuffer(
                                            img_bytes[12:], dtype=np.uint8
                                        ).reshape(h, w, c)
                                    else:
                                        frame = None
                                else:
                                    frame = None

                            if frame is not None:
                                with self._frame_lock:
                                    self._latest_frames[camera_name] = frame
                    except Exception as e:
                        print(
                            f"[ZenohCamera] Error processing frame for {camera_name}: {e}"
                        )

                return frame_callback

            # Subscribe to camera topics
            for cam_name, topic in self.CAMERA_TOPICS.items():
                subscriber = self._session.declare_subscriber(
                    topic, make_frame_callback(cam_name)
                )
                self._subscribers[cam_name] = subscriber
            print(f"  ✓ Connected to Zenoh multi-camera stream")

            self._streaming = True
        except Exception as e:
            raise ConnectionError(f"Failed to connect to Zenoh camera: {e}")

    def disconnect(self) -> None:
        """Disconnect from camera stream."""
        for subscriber in self._subscribers.values():
            try:
                subscriber.close()
            except Exception:
                pass
        self._subscribers.clear()

        if self._session:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None

        self._streaming = False
        with self._frame_lock:
            self._latest_frames.clear()

    def get_frame(self, camera_name: Optional[str] = None) -> Optional[np.ndarray]:
        """Get the latest camera frame.

        Args:
            camera_name: Camera name for multi-camera mode. If None, uses default camera.

        Returns:
            BGR image as numpy array, or None if no frame available
        """
        cam = camera_name or self._camera_name
        with self._frame_lock:
            frame = self._latest_frames.get(cam)
            return frame.copy() if frame is not None else None

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
        """Get the latest frames from all cameras.

        Returns:
            Dictionary mapping camera name to frame
        """
        with self._frame_lock:
            return {
                name: frame.copy()
                for name, frame in self._latest_frames.items()
                if frame is not None
            }


__all__ = ["ZenohTransport", "ZenohCamera"]
