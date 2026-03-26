"""
Telemetry - Robot status and sensor data module.

Provides get_position() and get_velocity() functions by subscribing
to robot's odometry topic.

This module uses ROSTransportInterface abstraction, allowing it
to work with any transport implementation (rosbridge, zenoh).
"""

import threading
import base64
import struct
from typing import Any, Dict, List, Optional


from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.converters import quaternion_to_euler
from walkie_sdk.utils.namespace import apply_namespace
from walkie_sdk.config.ros_topics import TELEMETRY_TOPICS

# Default ROS topic names and types (without namespace)
DEFAULT_ODOM_TOPIC = TELEMETRY_TOPICS["odom"]
ODOM_TYPE = TELEMETRY_TOPICS["odom_type"]

DEFAULT_ZED_POINT_CLOUD_TOPIC = TELEMETRY_TOPICS["zed_point_cloud"]
ZED_POINT_CLOUD_TYPE = TELEMETRY_TOPICS["point_cloud_type"]



class Telemetry:
    """
    Robot telemetry/status provider.

    Subscribes to odometry and provides current position and velocity.
    Data is cached and updated in background via ROS subscription.

    This class works with any transport that implements ROSTransportInterface,
    making it protocol-agnostic (works with rosbridge, zenoh, etc.).

    Args:
        transport: Transport instance implementing ROSTransportInterface
        namespace: ROS namespace prefix for topics (default: "" = no namespace)
    """

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace
        self._lock = threading.Lock()

        # Cached odometry data
        self._pose: Optional[Dict[str, float]] = None
        self._velocity: Optional[Dict[str, float]] = None
        self._raw_odom: Optional[Dict[str, Any]] = None

        # Cached point cloud data
        self._zed_point_cloud: Optional[Dict[str, Any]] = None

        # Subscription handle (type varies by transport)
        self._odom_subscription: Optional[Any] = None
        self._subscribed = False

    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """Set ROS namespace for topics."""
        self._namespace = value

    @property
    def odom_topic(self) -> str:
        """Get the full odom topic name with namespace."""
        return apply_namespace(DEFAULT_ODOM_TOPIC, self._namespace)
    

    @property
    def zed_point_cloud_topic(self) -> str:
        """Get the full ZED point cloud topic name with namespace."""
        return apply_namespace(DEFAULT_ZED_POINT_CLOUD_TOPIC, self._namespace)

    def start(self) -> None:
        """
        Start subscribing to telemetry topics.

        Called automatically when WalkieRobot connects.
        """
        if self._subscribed:
            return

        if not self._transport.is_connected:
            return

        try:
            self._odom_subscription = self._transport.subscribe(
                topic=self.odom_topic,
                message_type=ODOM_TYPE,
                callback=self._on_odom,
                throttle_rate=100,  # 10 Hz max
                queue_size=1,
            )
            self._subscribed = True
        except Exception as e:
            print(f"  ⚠ Failed to subscribe to odometry: {e}")

        try:
            self._zed_point_cloud_subscription = self._transport.subscribe(
                topic=self.zed_point_cloud_topic,
                message_type=ZED_POINT_CLOUD_TYPE,
                callback=self._on_zed_point_cloud,
                throttle_rate=100,  # 10 Hz max
                queue_size=1,
            )
            self._subscribed = True
        except Exception as e:
            print(f"  ⚠ Failed to subscribe to ZED point cloud: {e}")

    def stop(self) -> None:
        """Stop telemetry subscriptions."""
        if self._odom_subscription is not None:
            try:
                self._transport.unsubscribe(self._odom_subscription)
            except Exception:
                pass
            self._odom_subscription = None

        if self._zed_point_cloud_subscription is not None:
            try:
                self._transport.unsubscribe(self._zed_point_cloud_subscription)
            except Exception:
                pass
            self._zed_point_cloud_subscription = None

        self._subscribed = False

    def _on_odom(self, msg: Dict[str, Any]) -> None:
        """Callback for odometry messages."""
        with self._lock:
            self._raw_odom = msg

            # Extract pose
            try:
                pose = msg["pose"]["pose"]
                position = pose["position"]
                orientation = pose["orientation"]

                # Convert quaternion to yaw (heading)
                _, _, yaw = quaternion_to_euler(
                    orientation["x"],
                    orientation["y"],
                    orientation["z"],
                    orientation["w"],
                )

                self._pose = {"x": position["x"], "y": position["y"], "heading": yaw}
            except (KeyError, TypeError):
                pass

            # Extract velocity
            try:
                twist = msg["twist"]["twist"]
                self._velocity = {
                    "linear": twist["linear"]["x"],
                    "angular": twist["angular"]["z"],
                }
            except (KeyError, TypeError):
                pass
    def _on_zed_point_cloud(self, msg: Dict[str, Any]) -> None:
        """Callback for ZED point cloud messages."""
        with self._lock:
            self._zed_point_cloud = msg
            #PointCloud2 Message Keys: ['header', 'height', 'width', 'fields', 'is_bigendian', 'point_step', 'row_step', 'data', 'is_dense']

            try:
                # 1. Extract basic metadata
                width = msg.get("width", 0)
                height = msg.get("height", 0)
                point_step = msg.get("point_step", 0)
                is_dense = msg.get("is_dense", False)
                
                # 2. Extract field names (e.g., ['x', 'y', 'z', 'rgb'])
                fields = msg.get("fields", [])
                field_names = [f.get("name") for f in fields]
                
                # 3. Store parsed metadata in a clean dictionary
                self._point_cloud_metadata = {
                    "width": width,
                    "height": height,
                    "num_points": width * height,
                    "fields": field_names,
                    "point_step": point_step,
                    "is_dense": is_dense
                }

                # Optional: Extract the very first point as a sanity check/sample
                # Note: PointCloud2 data from rosbridge is usually a base64 encoded string.
                raw_data = msg.get("data")
                if raw_data and point_step >= 12: # At least enough bytes for 3 floats (x,y,z)
                    byte_data = base64.b64decode(raw_data) if isinstance(raw_data, str) else bytes(raw_data)
                    
                    # Unpack the first 12 bytes as 3 little-endian floats (X, Y, Z)
                    # '<fff' stands for little-endian, float, float, float
                    first_point = struct.unpack_from('<fff', byte_data, offset=0)
                    self._point_cloud_metadata["sample_point_xyz"] = {
                        "x": first_point[0],
                        "y": first_point[1],
                        "z": first_point[2]
                    }

            except Exception as e:
                # Fail gracefully if the message format is unexpected
                print(f"  ⚠ Failed to parse ZED point cloud info: {e}")

    def get_position(self) -> Optional[Dict[str, float]]:
        """
        Get the current robot position.

        Returns:
            Dictionary with 'x', 'y' (meters), and 'heading' (radians),
            or None if no odometry data is available yet.

        Example:
            ```python
            pos = bot.status.get_position()
            # {'x': 1.2, 'y': 3.5, 'heading': 0.5}
            ```
        """
        with self._lock:
            if self._pose is not None:
                return self._pose.copy()
            return None

    def get_velocity(self) -> Optional[Dict[str, float]]:
        """
        Get the current robot velocity.

        Returns:
            Dictionary with 'linear' (m/s) and 'angular' (rad/s),
            or None if no odometry data is available yet.

        Example:
            ```python
            vel = bot.status.get_velocity()
            # {'linear': 0.2, 'angular': 0.0}
            ```
        """
        with self._lock:
            if self._velocity is not None:
                return self._velocity.copy()
            return None

    def get_raw_odom(self) -> Optional[Dict[str, Any]]:
        """
        Get the raw odometry message.

        Returns:
            Full ROS Odometry message as dictionary, or None if not available.
        """
        with self._lock:
            if self._raw_odom is not None:
                return self._raw_odom.copy()
            return None
        
    def get_point_cloud_info(self) -> Optional[Dict[str, Any]]:
        """
        Get the parsed metadata and structural info of the latest point cloud.

        Returns:
            Dictionary with width, height, field names, and a sample point.

        Example:
            ```python
            pc_info = bot.status.get_point_cloud_info()
            if pc_info:
                print(
                    f"{pc_info['width']}x{pc_info['height']} "
                    f"fields={pc_info.get('fields')}"
                )
                # Example keys: width, height, num_points, fields, is_dense,
                # point_step, and sample_point_xyz
            ```
        """
        with self._lock:
            if hasattr(self, '_point_cloud_metadata'):
                return self._point_cloud_metadata.copy()
            return None
        
    def get_full_point_cloud(self) -> Optional[List[tuple[float, float, float]]]:
        """
        Retrieves and decodes the entire point cloud into a list of (X, Y, Z) coordinates.
        
        Returns:
            A list of tuples containing (x, y, z) floats, or None if no data is available.

        Example:
            ```python
            full_cloud = bot.status.get_full_point_cloud()
            if full_cloud:
                print(f"Extracted {len(full_cloud)} points")
                print("First 3:", full_cloud[:3])
                print("Last 3:", full_cloud[-3:])
            ```
        """
        with self._lock:
            msg = self._zed_point_cloud
            if not msg:
                return None

            try:
                raw_data = msg.get("data")
                point_step = msg.get("point_step", 0)
                
                if not raw_data or point_step < 12:
                    return []

                # Decode base64 if coming from rosbridge JSON, otherwise treat as bytes
                byte_data = base64.b64decode(raw_data) if isinstance(raw_data, str) else bytes(raw_data)
                
                points = []
                
                # Loop through the byte array, jumping forward by 'point_step' bytes each time
                for i in range(0, len(byte_data), point_step):
                    # Make sure we have at least 12 bytes left to unpack 3 floats
                    if i + 12 <= len(byte_data):
                        # '<fff' = Little-endian, float, float, float (X, Y, Z)
                        x, y, z = struct.unpack_from('<fff', byte_data, offset=i)
                        
                        # Filter out invalid points (NaNs or zeros) if needed. 
                        # Often ZED outputs NaNs for points it can't calculate depth for.
                        # if x == x and y == y and z == z: # Fast NaN check
                        points.append((x, y, z))
                        
                return points

            except Exception as e:
                print(f"  ⚠ Failed to unpack full ZED point cloud: {e}")
                return None

    @property
    def has_data(self) -> bool:
        """Check if telemetry data is available."""
        with self._lock:
            return self._pose is not None
