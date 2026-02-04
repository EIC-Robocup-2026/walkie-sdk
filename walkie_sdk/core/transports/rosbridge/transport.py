"""
ROSBridgeTransport - WebSocket-based ROS transport using roslibpy.

This transport communicates with ROS2 via the rosbridge_suite WebSocket server.
It does not require ROS2 to be installed on the client machine.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Optional

import roslibpy

from walkie_sdk.core.interfaces import ROSTransportInterface


class ROSBridgeTransport(ROSTransportInterface[roslibpy.Topic]):
    """
    ROS transport implementation using WebSocket via roslibpy.

    Connects to a rosbridge_server running on the robot and provides
    topic subscription/publishing, action calls, and service calls
    over WebSocket.

    Args:
        host: Robot IP address or hostname
        port: ROSBridge WebSocket port (default: 9090)
        timeout: Connection timeout in seconds (default: 10.0)

    Example:
        transport = ROSBridgeTransport(host="192.168.1.100", port=9090)
        transport.connect()

        # Subscribe to odometry
        def on_odom(msg):
            print(f"Position: {msg['pose']['pose']['position']}")

        handle = transport.subscribe("/odom", "nav_msgs/msg/Odometry", on_odom)

        # Publish velocity
        transport.publish("/cmd_vel", "geometry_msgs/msg/Twist", {
            "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
        })

        transport.disconnect()
    """

    def __init__(self, host: str, port: int = 9090, timeout: float = 10.0):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._ros: Optional[roslibpy.Ros] = None
        self._lock = threading.Lock()

        # Track current action for cancellation
        self._current_goal: Optional[Any] = None
        self._current_action_client: Optional[Any] = None

    @property
    def host(self) -> str:
        """Robot IP address or hostname."""
        return self._host

    @property
    def port(self) -> int:
        """ROSBridge WebSocket port."""
        return self._port

    @property
    def is_connected(self) -> bool:
        """Check if connected to ROSBridge."""
        if self._ros is None:
            return False
        return bool(self._ros.is_connected)

    def connect(self) -> None:
        """
        Connect to the ROSBridge server.

        Blocks until connected or timeout.

        Raises:
            ConnectionError: If connection fails or times out
        """
        print(f"  → Connecting to ROSBridge at {self._host}:{self._port}...")

        try:
            self._ros = roslibpy.Ros(host=self._host, port=self._port)
            self._ros.run()

            # Wait for connection with timeout
            start_time = time.time()
            while not self._ros.is_connected:
                if time.time() - start_time > self._timeout:
                    self._ros.terminate()
                    self._ros = None
                    raise ConnectionError(
                        f"Connection timeout after {self._timeout}s. "
                        f"Is ROSBridge running at {self._host}:{self._port}?"
                    )
                time.sleep(0.1)

            print("  ✓ ROSBridge connected")

        except Exception as e:
            self._ros = None
            if isinstance(e, ConnectionError):
                raise
            raise ConnectionError(
                f"Failed to connect to ROSBridge at {self._host}:{self._port}: {e}"
            ) from e

    def disconnect(self) -> None:
        """Disconnect from the ROSBridge server."""
        if self._ros is not None:
            try:
                self._ros.terminate()
            except Exception:
                pass
            self._ros = None
            print("  ✓ ROSBridge disconnected")

    def _ensure_connected(self) -> roslibpy.Ros:
        """Get the ROS client, raising if not connected."""
        if self._ros is None or not self._ros.is_connected:
            raise ConnectionError("Not connected to ROSBridge")
        return self._ros

    def subscribe(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[Dict[str, Any]], None],
        throttle_rate: int = 0,
        queue_size: int = 1,
    ) -> roslibpy.Topic:
        """
        Subscribe to a ROS topic.

        Args:
            topic: Topic name (e.g., "/odom")
            message_type: ROS message type (e.g., "nav_msgs/msg/Odometry")
            callback: Function to call with each message
            throttle_rate: Minimum interval between messages in ms (0 = no throttle)
            queue_size: Message queue size

        Returns:
            roslibpy.Topic instance (subscription handle)

        Raises:
            ConnectionError: If not connected
        """
        ros = self._ensure_connected()

        topic_obj = roslibpy.Topic(
            ros,
            topic,
            message_type,
            throttle_rate=throttle_rate,
            queue_size=queue_size,
        )
        topic_obj.subscribe(callback)
        return topic_obj

    def unsubscribe(self, handle: roslibpy.Topic) -> None:
        """
        Unsubscribe from a topic.

        Args:
            handle: The topic handle returned by subscribe()
        """
        try:
            handle.unsubscribe()
        except Exception:
            pass

    def publish(
        self,
        topic: str,
        message_type: str,
        message: Dict[str, Any],
    ) -> None:
        """
        Publish a message to a ROS topic.

        Args:
            topic: Topic name (e.g., "/cmd_vel")
            message_type: ROS message type (e.g., "geometry_msgs/msg/Twist")
            message: Message data as dictionary

        Raises:
            ConnectionError: If not connected
        """
        ros = self._ensure_connected()

        topic_obj = roslibpy.Topic(ros, topic, message_type)
        topic_obj.publish(roslibpy.Message(message))

    def call_action(
        self,
        action_name: str,
        action_type: str,
        goal: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Call a ROS2 action and wait for result.

        Args:
            action_name: Action server name (e.g., "/navigate_to_pose")
            action_type: Action type (e.g., "nav2_msgs/action/NavigateToPose")
            goal: Goal message as dictionary
            feedback_callback: Optional callback for feedback
            timeout: Optional timeout in seconds (None = wait forever)

        Returns:
            Dict with 'result' and 'status' keys

        Raises:
            ConnectionError: If not connected
            TimeoutError: If action times out
        """
        ros = self._ensure_connected()

        result_event = threading.Event()
        result_data: Dict[str, Any] = {"result": None, "status": None}

        def on_result(result: Dict[str, Any]) -> None:
            result_data["result"] = result
            result_data["status"] = "SUCCEEDED"
            result_event.set()

        def on_feedback(feedback: Dict[str, Any]) -> None:
            if feedback_callback:
                feedback_callback(feedback)

        def on_error(error: Exception) -> None:
            result_data["status"] = "FAILED"
            result_data["error"] = str(error)
            result_event.set()

        action_client = roslibpy.ActionClient(ros, action_name, action_type)
        goal_message = roslibpy.Message(goal)

        # Send goal and get handle
        goal_handle = action_client.send_goal(
            goal_message, on_result, feedback=on_feedback, errback=on_error
        )

        # Store for cancellation
        with self._lock:
            self._current_goal = goal_handle
            self._current_action_client = action_client

        # Wait for result
        if result_event.wait(timeout=timeout):
            with self._lock:
                self._current_goal = None
                self._current_action_client = None
                #action_client.destroy()
            return result_data
        else:
            # Timeout - cancel the goal
            try:
                action_client.cancel_goal(goal_handle)
            except Exception:
                pass

            with self._lock:
                self._current_goal = None
                self._current_action_client = None

            raise TimeoutError(f"Action {action_name} timed out after {timeout}s")
        
    def cancel_action(self) -> None:
        """Cancel the current action goal if any."""
        with self._lock:
            if (
                self._current_goal is not None
                and self._current_action_client is not None
            ):
                try:
                    self._current_action_client.cancel_goal(self._current_goal)
                except Exception:
                    pass
                self._current_goal = None
                self._current_action_client = None

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request: Dict[str, Any],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Call a ROS service.

        Args:
            service_name: Service name
            service_type: Service type
            request: Request message as dictionary
            timeout: Timeout in seconds

        Returns:
            Service response as dictionary

        Raises:
            ConnectionError: If not connected
            TimeoutError: If service call times out
        """
        ros = self._ensure_connected()

        service = roslibpy.Service(ros, service_name, service_type)
        request_msg = roslibpy.ServiceRequest(request)

        result_event = threading.Event()
        result_data: Dict[str, Any] = {}

        def callback(response: Dict[str, Any]) -> None:
            result_data.update(response)
            result_event.set()

        service.call(request_msg, callback)

        if result_event.wait(timeout=timeout):
            return result_data
        else:
            raise TimeoutError(f"Service {service_name} timed out after {timeout}s")

    def __enter__(self) -> "ROSBridgeTransport":
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.disconnect()

    def __repr__(self) -> str:
        status = "connected" if self.is_connected else "disconnected"
        return f"ROSBridgeTransport(host='{self._host}', port={self._port}, status={status})"
