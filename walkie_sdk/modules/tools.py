"""
Tools - Utility module for Object Detection and Coordinate Transformation.

Provides functions to send 2D detections and retrieve corresponding 3D coordinates.
"""

import threading
import time
from typing import Any, Dict, Optional, List

# Adjust import based on your project structure
from walkie_sdk.utils import converters
from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace

# -----------------------------------------------------------------------------
# Constants (Topics, Services, Types)
# -----------------------------------------------------------------------------
# For Publishing (Request)
OBJECT_POSE_TOPIC = "/yolo/detections_2d"
DETECTION_2D_TYPE = "vision_msgs/msg/Detection2DArray"

# For Subscribing (Response)
DETECT_3D_TOPIC = "/ob_detection/poses"
DETECTION_3D_TYPE = "geometry_msgs/msg/PoseArray"  # Fixed typo: Detextion -> Detection


class Tools:
    """
    Controller for auxiliary robot tools and vision processing.

    This module works with any transport implementation (rosbridge, zenoh).
    """

    def __init__(
        self,
        transport: ROSTransportInterface,
        namespace: str = "",
    ):
        self._transport = transport
        self._namespace = namespace

        # Thread safety for local state
        self._lock = threading.Lock()
        self._latest_detection: Optional[Dict[str, Any]] = None

        # Synchronization Event: Used to wait for the reply
        self._response_event = threading.Event()

        self._subscription = None
        self._subscribed = False

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------
    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """Set ROS namespace and re-initialize subscriptions if necessary."""
        self._namespace = value
        # Reset subscription on namespace change
        if self._subscribed:
            self._unsubscribe()
            self._subscribed = False

    # -------------------------------------------------------------------------
    # Lifecycle Management
    # -------------------------------------------------------------------------

    def start(self) -> None:
        """
        Start the module (subscribe to topics).
        Called automatically by WalkieRobot.
        """
        self._setup_subscription()

    def stop(self) -> None:
        """
        Stop the module (unsubscribe).
        Called automatically by WalkieRobot on disconnect.
        """
        self._unsubscribe()
        self._subscribed = False

    # -------------------------------------------------------------------------
    # Internal Helpers
    # -------------------------------------------------------------------------
    def _setup_subscription(self):
        """Setup subscription to 3D detection topic."""
        if self._subscribed:
            return

        topic = apply_namespace(DETECT_3D_TOPIC, self._namespace)

        def callback(msg: Dict):
            # 1. Store the data
            with self._lock:
                self._latest_detection = msg
                # Optional: Debug print
                print(f"[Tools] Received 3D detection response")

            # 2. Signal that data has arrived (Unblocks the wait)
            self._response_event.set()

        try:
            print(f"[Tools] Subscribing to: {topic}")
            self._subscription = self._transport.subscribe(
                topic, DETECTION_3D_TYPE, callback
            )
            self._subscribed = True
        except Exception as e:
            print(f"[Tools] Failed to subscribe: {e}")

    def _unsubscribe(self):
        """Clean up subscription."""
        if self._subscription and hasattr(self._transport, "unsubscribe"):
            try:
                self._transport.unsubscribe(self._subscription)
            except Exception:
                pass
        self._subscription = None

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def bboxes_to_positions(
        self, coords: List[List[float]], timeout: float = 5.0
    ) -> Optional[Dict[str, Any]]:
        """
        Publishes 2D bounding boxes and waits for the corresponding 3D detections.

        Args:
            coords: A list of 2D bounding boxes [cx, cy, w, h].
                    Example: [[320, 240, 50, 50], ...]
            timeout: Maximum time to wait for a response in seconds.

        Returns:
            Dictionary containing 'vision_msgs/msg/Detection3DArray' data,
            or None if the request timed out.
        """
        # 1. Ensure we are listening for the reply
        if not self._subscribed:
            self._setup_subscription()

        # 2. Clear the 'Data Received' flag
        # We do this BEFORE publishing to ensure we don't read old data
        self._response_event.clear()

        # Optional: Clear old data cache to be safe
        with self._lock:
            self._latest_detection = None

        try:
            # 3. Convert and Publish the Request
            # Using your converter logic
            msg = converters.convert_bboxes_to_detection_array(coords)

            # Publish to ROS 2
            # print("publishing to topic")
            # print(f"publish to topic: {OBJECT_POSE_TOPIC} ,with msg_type: {DETECTION_2D_TYPE}")
            # print(msg)
            topic = apply_namespace(OBJECT_POSE_TOPIC, self._namespace)
            self._transport.publish(topic, DETECTION_2D_TYPE, msg)

            # 4. Wait for the response
            # This blocks until the callback runs OR timeout expires
            data_received = self._response_event.wait(timeout=timeout)

            if data_received:
                with self._lock:
                    # Return the fresh data
                    detections = (
                        converters.convert_poses_to_array(self._latest_detection.copy())
                        if self._latest_detection
                        else None
                    )
                    return detections
            else:
                print(f"[Tools] Timeout ({timeout}s) waiting for 3D detections.")
                return None

        except Exception as e:
            print(f"[Tools] Error in bboxes_to_positions: {e}")
            return None
