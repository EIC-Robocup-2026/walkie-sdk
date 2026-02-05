"""
Telemetry - Robot status and sensor data module.

Provides get_pose() and get_velocity() functions by subscribing
to robot's odometry topic.

This module uses ROSTransportInterface abstraction, allowing it
to work with any transport implementation (rosbridge, zenoh).
"""

import threading
from typing import Any, Dict, Optional

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.converters import quaternion_to_euler
from walkie_sdk.utils.namespace import apply_namespace

# Default ROS topic names and types (without namespace)
DEFAULT_ODOM_TOPIC = "odom"
DEFAULT_ODOM_TOPIC = "omni_wheel_drive_controller/odom"
ODOM_TYPE = "nav_msgs/msg/Odometry"


class Telemetry:
    """
    Robot telemetry/status provider.

    Subscribes to odometry and provides current pose and velocity.
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

    def stop(self) -> None:
        """Stop telemetry subscriptions."""
        if self._odom_subscription is not None:
            try:
                self._transport.unsubscribe(self._odom_subscription)
            except Exception:
                pass
            self._odom_subscription = None
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

    def get_pose(self) -> Optional[Dict[str, float]]:
        """
        Get the current robot pose.

        Returns:
            Dictionary with 'x', 'y' (meters), and 'heading' (radians),
            or None if no odometry data is available yet.

        Example:
            >>> bot.status.get_pose()
            {'x': 1.2, 'y': 3.5, 'heading': 0.5}
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
            >>> bot.status.get_velocity()
            {'linear': 0.2, 'angular': 0.0}
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

    @property
    def has_data(self) -> bool:
        """Check if telemetry data is available."""
        with self._lock:
            return self._pose is not None
