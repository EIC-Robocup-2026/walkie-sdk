"""
Navigation - Robot navigation control module.

Provides go_to(), cancel(), and stop() functions for controlling
robot's navigation via Nav2 action server.

This module uses ROSTransportInterface abstraction, allowing it
to work with any transport implementation (rosbridge, zenoh).
"""

import threading
import time
from typing import TYPE_CHECKING, Any, Callable, Dict, Optional, Tuple

import numpy as np

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.converters import euler_to_quaternion
from walkie_sdk.utils.namespace import apply_namespace
from walkie_sdk.config.ros_topics import NAV_TOPICS, NAV_ACTIONS, MAP_TOPICS, MAP_SERVICES

if TYPE_CHECKING:
    from walkie_sdk.modules.head import Head

HEAD_DEFAULT_TILT_MIN: float = -0.1
HEAD_DEFAULT_TILT_MAX: float = 0.3


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

    def __init__(
        self,
        transport: ROSTransportInterface,
        namespace: str = "",
        head: "Optional[Head]" = None,
    ):
        self._transport = transport
        self._namespace = namespace
        self._head = head
        self._current_goal_id: Optional[str] = None
        self._goal_lock = threading.Lock()
        self._navigation_status: Optional[str] = None
        self._distance_remaining: Optional[float] = None
        self._number_of_recoveries: int = 0
        self._current_pose: Optional[Dict[str, Any]] = None
        self._final_distance_remaining: Optional[float] = None
        self._nav_error_code: Optional[int] = None
        self._nav_error_msg: Optional[str] = None
        self._map_metadata: Optional[dict] = None

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
        return apply_namespace(NAV_ACTIONS["navigate_to_pose"], self._namespace)

    @property
    def navigate_to_object_action_name(self) -> str:
        """Get the full navigate_to_object action name with namespace."""
        return apply_namespace(NAV_ACTIONS["navigate_to_object"], self._namespace)

    @property
    def cmd_vel_topic(self) -> str:
        """Get the full cmd_vel topic name with namespace."""
        return apply_namespace(NAV_TOPICS["cmd_vel"], self._namespace)

    @property
    def map_topic(self) -> str:
        """Get the full map topic name with namespace."""
        return apply_namespace(MAP_TOPICS["map"], self._namespace)

    @property
    def map_metadata(self) -> Optional[dict]:
        """Metadata (resolution, size, origin) of the last fetched map, or None."""
        return self._map_metadata

    def go_to(
        self,
        x: float,
        y: float,
        heading: Optional[float] = None,
        blocking: bool = True,
        timeout: Optional[float] = None,
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        goal_tolerance: Optional[float] = None,
        standoff: float = 0.0,
        align_method: str = "",
    ) -> str:
        """
        Navigate to a target pose or object position.

        When ``heading`` is provided, sends a ``NavigateToPose`` goal to Nav2
        (original behaviour). When ``heading`` is ``None``, sends a
        ``NavigateToObject`` goal to nav_commander, which computes the
        approach pose via PCA edge-fit alignment.

        Args:
            x: Target X coordinate in meters (map frame)
            y: Target Y coordinate in meters (map frame)
            heading: Target heading in radians (0 = +X, π/2 = +Y).
                     ``None`` → use NavigateToObject (no heading required).
            blocking: If True, wait for navigation to complete (default: True)
            timeout: Optional timeout in seconds (None = wait forever)
            feedback_callback: Optional callback for navigation feedback
            goal_tolerance: If set (metres), a FAILED result is promoted to
                "CLOSE_ENOUGH" when final distance_remaining ≤ this value.
            standoff: Per-goal standoff override in metres (NavigateToObject
                      only; 0.0 = use nav_commander's configured default).
            align_method: Alignment method for NavigateToObject (``""`` =
                          default ``nearest_edge``, ``"face_target"`` skips
                          edge alignment). Ignored when heading is provided.

        Returns:
            Status string: "SUCCEEDED", "FAILED", "CANCELED", "CLOSE_ENOUGH", or "IN_PROGRESS"

        Raises:
            ConnectionError: If not connected to ROS
            TimeoutError: If blocking and navigation times out

        Example:
            ```python
            # With heading — Nav2 navigate_to_pose
            result = bot.nav.go_to(x=2.0, y=1.0, heading=0.0)

            # Without heading — nav_commander navigate_to_object
            result = bot.nav.go_to(x=2.0, y=1.0)
            result = bot.nav.go_to(x=2.0, y=1.0, standoff=0.5)
            ```
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        # Reset feedback state for new goal
        self._distance_remaining = None
        self._number_of_recoveries = 0
        self._current_pose = None
        self._final_distance_remaining = None
        self._nav_error_code = None
        self._nav_error_msg = None

        if heading is None:
            # navigate_to_object path — nav_commander computes the approach pose
            goal_msg = {
                "obj_x": float(x),
                "obj_y": float(y),
                "align_method": align_method,
                "standoff": float(standoff),
            }
            action_name = self.navigate_to_object_action_name
            action_type = NAV_ACTIONS["navigate_to_object_type"]
        else:
            # navigate_to_pose path — standard Nav2
            qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, heading)
            goal_msg = {
                "pose": {
                    "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
                    "pose": {
                        "position": {"x": float(x), "y": float(y), "z": 0.0},
                        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                    },
                }
            }
            action_name = self.nav2_action_name
            action_type = NAV_ACTIONS["navigate_to_pose_type"]

        if not blocking:
            self._navigation_status = "IN_PROGRESS"
            threading.Thread(
                target=self._send_goal_async,
                args=(action_name, action_type, goal_msg, self._make_feedback_handler(feedback_callback), goal_tolerance),
                daemon=True,
            ).start()
            return "IN_PROGRESS"

        return self._send_goal_blocking(action_name, action_type, goal_msg, timeout, self._make_feedback_handler(feedback_callback), goal_tolerance)

    def _make_feedback_handler(
        self,
        user_callback: Optional[Callable[[Dict[str, Any]], None]],
    ) -> Callable[[Dict[str, Any]], None]:
        """Wrap user callback to also update internal feedback state."""
        def handler(feedback: Dict[str, Any]) -> None:
            fb = feedback.get("feedback", feedback)  # unwrap rosbridge envelope
            self._distance_remaining = fb.get("distance_remaining")
            self._number_of_recoveries = fb.get("number_of_recoveries", 0)
            self._current_pose = fb.get("current_pose")
            if user_callback:
                user_callback(feedback)
        return handler

    def _wait_for_camera_default(self, wait_timeout: float = 5.0) -> None:
        """Block until head tilt is back in the default forward range, or timeout."""
        if self._head is None:
            return
        deadline = time.monotonic() + wait_timeout
        while time.monotonic() < deadline:
            angle = self._head.get_angle()
            if angle is not None and HEAD_DEFAULT_TILT_MIN <= angle <= HEAD_DEFAULT_TILT_MAX:
                return
            time.sleep(0.05)

    def _send_goal_blocking(
        self,
        action_name: str,
        action_type: str,
        goal_msg: Dict[str, Any],
        timeout: Optional[float],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]],
        goal_tolerance: Optional[float] = None,
    ) -> str:
        """Send goal and block until complete."""
        try:
            result = self._transport.call_action(
                action_name=action_name,
                action_type=action_type,
                goal=goal_msg,
                feedback_callback=feedback_callback,
                timeout=timeout,
            )

            nav_result = result.get("result") or {}
            self._nav_error_code = nav_result.get("error_code")
            self._nav_error_msg = nav_result.get("error_msg") or nav_result.get("message")
            self._final_distance_remaining = self._distance_remaining

            status = result.get("status")
            if status == "SUCCEEDED":
                self._wait_for_camera_default()
                self._navigation_status = "SUCCEEDED"
                return "SUCCEEDED"
            elif status == "CANCELED":
                self._navigation_status = "CANCELED"
                return "CANCELED"
            else:
                if (
                    goal_tolerance is not None
                    and self._final_distance_remaining is not None
                    and self._final_distance_remaining <= goal_tolerance
                ):
                    self._navigation_status = "CLOSE_ENOUGH"
                    return "CLOSE_ENOUGH"
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
        action_name: str,
        action_type: str,
        goal_msg: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]],
        goal_tolerance: Optional[float] = None,
    ) -> None:
        """Send goal in background thread."""
        try:
            result = self._transport.call_action(
                action_name=action_name,
                action_type=action_type,
                goal=goal_msg,
                feedback_callback=feedback_callback,
                timeout=None,
            )

            nav_result = result.get("result") or {}
            self._nav_error_code = nav_result.get("error_code")
            self._nav_error_msg = nav_result.get("error_msg") or nav_result.get("message")
            self._final_distance_remaining = self._distance_remaining

            status = result.get("status")
            if status == "SUCCEEDED":
                self._navigation_status = "SUCCEEDED"
            elif status == "CANCELED":
                self._navigation_status = "CANCELED"
            else:
                if (
                    goal_tolerance is not None
                    and self._final_distance_remaining is not None
                    and self._final_distance_remaining <= goal_tolerance
                ):
                    self._navigation_status = "CLOSE_ENOUGH"
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
            ```python
            bot.nav.go_to(x=10.0, y=5.0, heading=0.0, blocking=False)
            time.sleep(2)
            bot.nav.cancel()  # returns True
            ```
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
            ```python
            bot.nav.stop()  # returns True
            ```
        """
        if not self._transport.is_connected:
            return False

        # Zero velocity Twist message
        zero_twist = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }

        try:
            self._transport.publish(self.cmd_vel_topic, NAV_TOPICS["cmd_vel_type"], zero_twist)
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

    @property
    def distance_remaining(self) -> Optional[float]:
        """Distance remaining to the navigation goal in metres (from latest Nav2 feedback)."""
        return self._distance_remaining

    @property
    def number_of_recoveries(self) -> int:
        """Number of recovery behaviors triggered so far in the current navigation goal."""
        return self._number_of_recoveries

    @property
    def current_pose(self) -> Optional[Dict[str, Any]]:
        """Most recent robot pose reported by Nav2 feedback (PoseStamped as dict)."""
        return self._current_pose

    @property
    def final_distance_remaining(self) -> Optional[float]:
        """Distance to goal (metres) captured at the moment the last action ended."""
        return self._final_distance_remaining

    @property
    def nav_error_code(self) -> Optional[int]:
        """Nav2 error_code from the action result (None if not provided by the Nav2 version)."""
        return self._nav_error_code

    @property
    def nav_error_msg(self) -> Optional[str]:
        """Nav2 error_msg from the action result (None if not provided by the Nav2 version)."""
        return self._nav_error_msg

    @property
    def last_result(self) -> Dict[str, Any]:
        """Rich result dict from the last navigation action."""
        return {
            "status": self._navigation_status,
            "final_distance": self._final_distance_remaining,
            "error_code": self._nav_error_code,
            "error_msg": self._nav_error_msg,
            "recoveries": self._number_of_recoveries,
        }

    # ------------------------------------------------------------------
    # Occupancy grid map
    # ------------------------------------------------------------------

    @staticmethod
    def _occupancy_grid_to_image(grid: dict) -> np.ndarray:
        """
        Convert a nav_msgs/OccupancyGrid (as a dict) into a grayscale image.

        Cell values map to: -1 (unknown) → 127, 0 (free) → 255, occupied → 0.
        The grid is stored row-major starting at the bottom-left in ROS, so the
        result is flipped vertically to match image (top-left origin) convention.
        """
        info = grid["info"]
        w, h = info["width"], info["height"]
        data = np.array(grid["data"], dtype=np.int16).reshape(h, w)
        img = np.where(data == -1, 127, np.where(data == 0, 255, 0)).astype(np.uint8)
        return np.flipud(img)  # ROS bottom-left → image top-left

    def get_map_image(self, timeout: float = 5.0) -> Optional[np.ndarray]:
        """
        Fetch the robot's occupancy grid map and return it as a grayscale image.

        Tries the map service first (nav_msgs/srv/GetMap), then falls back to
        subscribing to the map topic. On success, ``map_metadata`` is populated
        with the map's resolution, size, and origin for pixel conversions.

        Args:
            timeout: Per-source timeout in seconds (applies to both the service
                     call and the topic subscription wait).

        Returns:
            A ``(height, width)`` uint8 numpy array, or None if the map could
            not be retrieved.

        Raises:
            ConnectionError: If not connected to the robot.
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        grid = self._fetch_grid_via_service(timeout)
        if grid is None:
            grid = self._fetch_grid_via_topic(timeout)
        if grid is None:
            return None

        info = grid["info"]
        origin = info["origin"]["position"]
        self._map_metadata = {
            "resolution": info["resolution"],
            "width":      info["width"],
            "height":     info["height"],
            "origin_x":   origin["x"],
            "origin_y":   origin["y"],
        }
        return self._occupancy_grid_to_image(grid)

    def _fetch_grid_via_service(self, timeout: float) -> Optional[dict]:
        """Try to fetch the occupancy grid via the GetMap service."""
        try:
            response = self._transport.call_service(
                service_name=apply_namespace(MAP_SERVICES["get_map"], self._namespace),
                service_type=MAP_SERVICES["get_map_type"],
                request={},
                timeout=timeout,
            )
            return response.get("map")
        except Exception:
            return None

    def _fetch_grid_via_topic(self, timeout: float) -> Optional[dict]:
        """Fall back to fetching the occupancy grid by subscribing to the map topic."""
        received: list = [None]
        lock = threading.Lock()

        def _cb(msg: dict) -> None:
            with lock:
                received[0] = msg

        handle = None
        try:
            handle = self._transport.subscribe(
                topic=self.map_topic,
                message_type=MAP_TOPICS["map_type"],
                callback=_cb,
                queue_size=1,
            )
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                with lock:
                    if received[0] is not None:
                        return received[0]
                time.sleep(0.05)
            return None
        except Exception:
            return None
        finally:
            if handle is not None:
                try:
                    self._transport.unsubscribe(handle)
                except Exception:
                    pass

    def map_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert a map-frame coordinate (metres) to a pixel coordinate.

        The pixel coordinate matches the image returned by ``get_map_image()``
        (top-left origin, y increasing downward).

        Args:
            x: Map-frame X coordinate in metres.
            y: Map-frame Y coordinate in metres.

        Returns:
            (px, py) pixel coordinates.

        Raises:
            RuntimeError: If no map has been fetched yet (call get_map_image()).
        """
        if self._map_metadata is None:
            raise RuntimeError("No map metadata — call get_map_image() first")
        m = self._map_metadata
        px = int((x - m["origin_x"]) / m["resolution"])
        py = int(m["height"] - 1 - (y - m["origin_y"]) / m["resolution"])
        return px, py
