"""
Walkie SDK - Zenoh Transport

Zenoh-based transport implementation for ROS2 communication using the zenoh_ros2_sdk.
This handles CDR serialization, discovery, and type support automatically.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple, List

import numpy as np

# Import Zenoh and the SDK components
try:
    import zenoh
    from zenoh_ros2_sdk import (
        ZenohSession, 
        ROS2Publisher, 
        ROS2Subscriber, 
        ROS2ServiceClient
    )
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None

try:
    import cv2
except ImportError:
    cv2 = None

from walkie_sdk.core.interfaces import CameraTransportInterface, ROSTransportInterface

ROS_DOMAIN_ID = 23


def _msg_to_dict(msg: Any) -> Dict[str, Any]:
    """
    Recursively convert a rosbags/SDK message object to a dictionary.
    This bridges the gap between the SDK's object-oriented returns and the
    Walkie SDK's dictionary-based interface.
    """
    if hasattr(msg, "__dataclass_fields__"):
        result = {}
        for field in msg.__dataclass_fields__:
            value = getattr(msg, field)
            result[field] = _msg_to_dict(value)
        return result
    elif isinstance(msg, list):
        return [_msg_to_dict(x) for x in msg]
    elif isinstance(msg, (bytes, bytearray)):
        return msg
    elif hasattr(msg, "tolist"):  # numpy arrays
        return msg.tolist()
    else:
        return msg


class ZenohTransport(ROSTransportInterface[Any]):
    """
    ROS transport implementation using Zenoh SDK.
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
                "zenoh_ros2_sdk is not installed or accessible. Please install zenoh and the SDK."
            )

        self._host = host
        self._port = port
        self._timeout = timeout
        
        self._session_mgr = None
        self._lock = threading.Lock()
        
        # Cache SDK entities
        self._subscribers: Dict[str, ROS2Subscriber] = {}
        self._publishers: Dict[str, ROS2Publisher] = {}
        self._service_clients: Dict[str, ROS2ServiceClient] = {}

    @property
    def host(self) -> str:
        return self._host

    @property
    def port(self) -> int:
        return self._port

    @property
    def is_connected(self) -> bool:
        return self._session_mgr is not None

    def connect(self) -> None:
        """Connect to Zenoh router via SDK Session."""
        print(f"  → Connecting to Zenoh Router at {self._host}:{self._port}...")

        try:
            # Initialize the Singleton Session
            self._session_mgr = ZenohSession.get_instance(
                router_ip=self._host, 
                router_port=self._port
            )
            print(f"  ✓ Connected to Zenoh (Session ID: {self._session_mgr.session_id})")
        except Exception as e:
            self._session_mgr = None
            raise ConnectionError(f"Failed to connect to Zenoh: {e}")

    def disconnect(self) -> None:
        """Close all entities and session."""
        with self._lock:
            for sub in self._subscribers.values():
                sub.close()
            for pub in self._publishers.values():
                pub.close()
            for client in self._service_clients.values():
                client.close()

            self._subscribers.clear()
            self._publishers.clear()
            self._service_clients.clear()
            
            # Reset session reference but don't close singleton (Camera might use it)
            self._session_mgr = None

    def subscribe(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[Dict[str, Any]], None],
        throttle_rate: int = 0,
        queue_size: int = 1,
    ) -> Any:
        """Subscribe using ROS2Subscriber."""
        if not self._session_mgr:
            raise ConnectionError("Not connected to Zenoh")

        with self._lock:
            if topic in self._subscribers:
                return self._subscribers[topic]

            print(f"[ZenohTransport] Subscribing to: {topic} ({message_type})")

            # Wrapper to convert SDK Object -> Dict
            def callback_wrapper(msg_obj):
                try:
                    msg_dict = _msg_to_dict(msg_obj)
                    callback(msg_dict)
                except Exception as e:
                    print(f"[ZenohTransport] Error in callback for {topic}: {e}")

            # Create Subscriber using standard ROS 2 types
            sub = ROS2Subscriber(
                topic=topic,
                msg_type=message_type,
                callback=callback_wrapper,
                router_ip=self._host,
                router_port=self._port
            )
            self._subscribers[topic] = sub
            return sub

    def unsubscribe(self, handle: Any) -> None:
        """Unsubscribe and close the SDK subscriber."""
        with self._lock:
            # Handle is the ROS2Subscriber instance
            if hasattr(handle, 'topic') and handle.topic in self._subscribers:
                handle.close()
                del self._subscribers[handle.topic]
            elif hasattr(handle, 'close'):
                handle.close()

    def publish(
        self,
        topic: str,
        message_type: str,
        message: Dict[str, Any],
    ) -> None:
        """Publish using ROS2Publisher."""
        if not self._session_mgr:
            raise ConnectionError("Not connected to Zenoh")

        with self._lock:
            if topic not in self._publishers:
                # Create Publisher
                self._publishers[topic] = ROS2Publisher(
                    topic=topic,
                    msg_type=message_type,
                    router_ip=self._host,
                    router_port=self._port
                )
            
            pub = self._publishers[topic]

        # Publish using kwargs
        try:
            pub.publish(**message)
        except Exception as e:
            print(f"[ZenohTransport] Error publishing to {topic}: {e}")
            raise

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request: Dict[str, Any],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Call service using ROS2ServiceClient."""
        if not self._session_mgr:
            raise ConnectionError("Not connected to Zenoh")

        with self._lock:
            if service_name not in self._service_clients:
                self._service_clients[service_name] = ROS2ServiceClient(
                    service_name=service_name,
                    srv_type=service_type,
                    timeout=timeout,
                    router_ip=self._host,
                    router_port=self._port
                )
            client = self._service_clients[service_name]

        try:
            # SDK call returns an object
            response_obj = client.call(**request)
            
            if response_obj is None:
                raise TimeoutError(f"Service call to {service_name} timed out or failed")
                
            return _msg_to_dict(response_obj)
        except Exception as e:
            print(f"[ZenohTransport] Error calling service {service_name}: {e}")
            raise

    def call_action(
        self,
        action_name: str,
        action_type: str,
        goal: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Call a ROS2 action.
        Note: The current zenoh_ros2_sdk does not explicitly expose a high-level ActionClient yet.
        """
        print(f"[ZenohTransport] WARNING: Action {action_name} call not fully supported in this SDK version.")
        return {"result": None, "status": "NOT_IMPLEMENTED"}

    def cancel_action(self) -> None:
        pass


class ZenohCamera(CameraTransportInterface):
    """
    Camera transport implementation using Zenoh SDK.
    Subscribes to standard 'sensor_msgs/msg/CompressedImage'.
    """
    
    # Configure your ROS_DOMAIN_ID here if needed

    # Define standard ROS 2 topics for camera streams
    # Adjust these topics to match your robot's actual output
    CAMERA_TOPICS = {
        "head":  f"/zed/zed_node/rgb/color/rect/image/compressed",
        "left":  f"/walkie/camera/left/compressed",
        "right": f"/walkie/camera/right/compressed",
    }
    
    # Example ZED Topic if using standard ZED wrapper
    # "head": f"/zed/zed_node/rgb/image_rect_color/compressed"

    def __init__(
        self,
        host: str,
        port: int = 7447,
        topic: str = "walkie/camera/image/compressed",
        camera_name: str = "head",
        multi_camera: bool = False,
        **kwargs,
    ):
        if not ZENOH_AVAILABLE:
            raise ImportError("zenoh_ros2_sdk is not installed.")
        if cv2 is None:
            raise ImportError("opencv-python is required for ZenohCamera.")

        self._host = host
        self._port = port
        self._default_topic = topic 
        self._camera_name = camera_name
        self._multi_camera = multi_camera
        
        self._session_mgr = None
        self._subscribers: Dict[str, ROS2Subscriber] = {}
        
        self._latest_frames: Dict[str, np.ndarray] = {}
        self._frame_lock = threading.Lock()
        self._streaming = False

    @property
    def is_streaming(self) -> bool:
        return self._streaming

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        with self._frame_lock:
            for frame in self._latest_frames.values():
                if frame is not None:
                    return frame.shape
        return None

    def connect(self) -> None:
        if self._streaming:
            return

        print(f"  → Connecting Camera to Zenoh at {self._host}:{self._port}...")
        # Reuse existing session singleton if available
        self._session_mgr = ZenohSession.get_instance(self._host, self._port)

        # Determine which st to subscribe to
        topics = {}
        if self._multi_camera:
            topics = self.CAMERA_TOPICS
        else:
            # Use specific topic for requested camera
            if self._camera_name in self.CAMERA_TOPICS:
                topics[self._camera_name] = self.CAMERA_TOPICS[self._camera_name]
            else:
                topics[self._camera_name] = self._default_topic

        # Create subscribers for CompressedImage
        for name, topic in topics.items():
            print(f"  → Subscribing to camera topic: {topic}")
            
            # Closure to capture camera name for the callback
            def make_cb(cam_id):
                return lambda msg: self._on_frame(cam_id, msg)

            self._subscribers[name] = ROS2Subscriber(
                topic=topic,
                msg_type="sensor_msgs/msg/CompressedImage",
                domain_id=ROS_DOMAIN_ID,
                callback=make_cb(name),
                router_ip=self._host,
                router_port=self._port
            )
        
        self._streaming = True
        print("  ✓ Camera Stream Started")

    def disconnect(self) -> None:
        self._streaming = False
        for sub in self._subscribers.values():
            sub.close()
        self._subscribers.clear()
        
        with self._frame_lock:
            self._latest_frames.clear()

    def _on_frame(self, name: str, msg: Any) -> None:
        """Handle CompressedImage message."""
        try:
            # ROS 2 CompressedImage: 'data' field contains the bytes
            data = msg.data
            # print(f"Received frame for camera '{name}', size: {len(data)} bytes")
            
            # Handle different byte representations (rosbags vs standard)
            if hasattr(data, 'tobytes'):
                data = data.tobytes()
            elif isinstance(data, list):
                data = bytes(data)
            elif not isinstance(data, (bytes, bytearray)):
                # Fallback
                data = bytes(data)
            
            # Decode JPEG/PNG to OpenCV image
            np_arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                with self._frame_lock:
                    self._latest_frames[name] = frame
        except Exception as e:
            # print(f"Frame decode error: {e}")
            pass

    def get_frame(self, camera_name: Optional[str] = None) -> Optional[np.ndarray]:
        target = camera_name or self._camera_name
        with self._frame_lock:
            frame = self._latest_frames.get(target)
            return frame.copy() if frame is not None else None

    def get_head_frame(self) -> Optional[np.ndarray]:
        return self.get_frame("head")

    def get_left_frame(self) -> Optional[np.ndarray]:
        return self.get_frame("left")

    def get_right_frame(self) -> Optional[np.ndarray]:
        return self.get_frame("right")

    def get_all_frames(self) -> Dict[str, np.ndarray]:
        with self._frame_lock:
            return {
                k: v.copy() for k, v in self._latest_frames.items() if v is not None
            }


__all__ = ["ZenohTransport", "ZenohCamera"]