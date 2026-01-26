"""
Navigation - Robot navigation control module.

Provides go_to(), cancel(), and stop() functions for controlling
robot's navigation via Nav2 action server.

This module uses ROSTransportInterface abstraction, allowing it
to work with any transport implementation (rosbridge, zenoh).
"""

import threading
from typing import Any, Callable, Dict, Optional

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.converters import euler_to_quaternion
from walkie_sdk.utils.namespace import apply_namespace

# Default Nav2 action and topic names (without namespace)
DEFAULT_NAV2_ACTION_NAME = "navigate_to_pose"
NAV2_ACTION_TYPE = "nav2_msgs/action/NavigateToPose"
DEFAULT_CMD_VEL_TOPIC = "cmd_vel"
CMD_VEL_TYPE = "geometry_msgs/msg/Twist"


class Navigation:
    """
    Robot navigation controller.

    Provides methods to send navigation goals, cancel navigation,
    and perform emergency stops.

    This class works with any transport that implements ROSTransportInterface,
    making it protocol-agnostic (works with rosbridge, zenoh, etc.).

    Args:
        transport: Transport instance implementing ROSTransportInterface
        namespace: ROS namespace prefix for topics/actions (default: "" = no namespace)
    """

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace
        self._current_goal_id: Optional[str] = None
        self._goal_lock = threading.Lock()
        self._navigation_status: Optional[str] = None

    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """Set ROS namespace for topics/actions."""
        self._namespace = value

    @property
    def nav2_action_name(self) -> str:
        """Get the full Nav2 action name with namespace."""
        return apply_namespace(DEFAULT_NAV2_ACTION_NAME, self._namespace)

    @property
    def cmd_vel_topic(self) -> str:
        """Get the full cmd_vel topic name with namespace."""
        return apply_namespace(DEFAULT_CMD_VEL_TOPIC, self._namespace)

    def go_to(
        self,
        x: float,
        y: float,
        heading: float,
        blocking: bool = True,
        timeout: Optional[float] = None,
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> str:
        """
        Navigate to a target pose.

        Sends a navigation goal to the Nav2 action server.

        Args:
            x: Target X coordinate in meters (map frame)
            y: Target Y coordinate in meters (map frame)
            heading: Target heading in radians (0 = +X, π/2 = +Y)
            blocking: If True, wait for navigation to complete (default: True)
            timeout: Optional timeout in seconds (None = wait forever)
            feedback_callback: Optional callback for navigation feedback

        Returns:
            Status string: "SUCCEEDED", "FAILED", "CANCELED", or "IN_PROGRESS"

        Raises:
            ConnectionError: If not connected to ROS
            TimeoutError: If blocking and navigation times out

        Example:
            >>> bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
            'SUCCEEDED'
            >>> bot.nav.go_to(x=5.0, y=3.0, heading=1.57, blocking=False)
            'IN_PROGRESS'
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        # Convert heading to quaternion (roll=0, pitch=0, yaw=heading)
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, heading)

        # Build NavigateToPose goal message
        goal_msg = {
            "pose": {
                "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
                "pose": {
                    "position": {"x": float(x), "y": float(y), "z": 0.0},
                    "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                },
            }
        }

        if not blocking:
            # Non-blocking: send goal and return immediately
            self._navigation_status = "IN_PROGRESS"
            threading.Thread(
                target=self._send_goal_async,
                args=(goal_msg, feedback_callback),
                daemon=True,
            ).start()
            return "IN_PROGRESS"

        # Blocking: send goal and wait for result
        return self._send_goal_blocking(goal_msg, timeout, feedback_callback)

    def _send_goal_blocking(
        self,
        goal_msg: Dict[str, Any],
        timeout: Optional[float],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]],
    ) -> str:
        """Send goal and block until complete."""
        try:
            result = self._transport.call_action(
                action_name=self.nav2_action_name,
                action_type=NAV2_ACTION_TYPE,
                goal=goal_msg,
                feedback_callback=feedback_callback,
                timeout=timeout,
            )

            # Check result status
            if result.get("status") == "SUCCEEDED":
                self._navigation_status = "SUCCEEDED"
                return "SUCCEEDED"
            else:
                self._navigation_status = "FAILED"
                return "FAILED"

        except TimeoutError:
            self._navigation_status = "FAILED"
            raise
        except Exception as e:
            self._navigation_status = "FAILED"
            print(f"Navigation failed: {e}")
            return "FAILED"

    def _send_goal_async(
        self,
        goal_msg: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]],
    ) -> None:
        """Send goal in background thread."""
        try:
            result = self._transport.call_action(
                action_name=self.nav2_action_name,
                action_type=NAV2_ACTION_TYPE,
                goal=goal_msg,
                feedback_callback=feedback_callback,
                timeout=None,
            )

            if result.get("status") == "SUCCEEDED":
                self._navigation_status = "SUCCEEDED"
            else:
                self._navigation_status = "FAILED"

        except Exception:
            self._navigation_status = "FAILED"

    def cancel(self) -> bool:
        """
        Cancel the current navigation goal.

        Returns:
            True if cancellation was sent successfully, False otherwise

        Example:
            >>> bot.nav.go_to(x=10.0, y=5.0, heading=0.0, blocking=False)
            >>> time.sleep(2)
            >>> bot.nav.cancel()
            True
        """
        if not self._transport.is_connected:
            return False

        try:
            self._transport.cancel_action()
            self._navigation_status = "CANCELED"
            return True
        except Exception:
            return False

    def stop(self) -> bool:
        """
        Emergency stop - immediately halt robot motion.

        Publishes zero velocity to /cmd_vel topic.
        This is more immediate than cancel() which waits for Nav2 response.

        Returns:
            True if stop command was sent successfully, False otherwise

        Example:
            >>> bot.nav.stop()
            True
        """
        if not self._transport.is_connected:
            return False

        # Zero velocity Twist message
        zero_twist = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }

        try:
            self._transport.publish(self.cmd_vel_topic, CMD_VEL_TYPE, zero_twist)
            # Also cancel any ongoing navigation
            self._transport.cancel_action()
            self._navigation_status = "STOPPED"
            return True
        except Exception:
            return False

    @property
    def status(self) -> Optional[str]:
        """
        Get the current navigation status.

        Returns:
            Status string or None if no navigation has been started
        """
        return self._navigation_status

    @property
    def is_navigating(self) -> bool:
        """
        Check if robot is currently navigating.

        Returns:
            True if navigation is in progress
        """
        return self._navigation_status == "IN_PROGRESS"
