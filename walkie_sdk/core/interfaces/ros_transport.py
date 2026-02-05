"""
ROSTransportInterface - Abstract base class for ROS communication transports.

This interface defines the contract that any ROS transport implementation must fulfill.
Implementations include:
- ROSBridgeTransport: WebSocket via roslibpy (no ROS2 required on client)
- ZenohTransport: Zenoh DDS bridge
"""

from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, Generic, Optional, TypeVar

# Generic type for subscription handles (each transport may use different types)
SubscriptionHandle = TypeVar("SubscriptionHandle")


class ROSTransportInterface(ABC, Generic[SubscriptionHandle]):
    """
    Abstract interface for ROS communication transports.

    This interface abstracts away the underlying protocol used to communicate
    with ROS2, allowing the SDK to work with different backends:
    - WebSocket (rosbridge/roslibpy)
    - Zenoh DDS bridge

    All implementations must provide the same API for:
    - Topic subscription and publishing
    - Action client calls
    - Service calls
    """

    @abstractmethod
    def connect(self) -> None:
        """
        Establish connection to ROS.

        Raises:
            ConnectionError: If connection fails or times out
        """
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """
        Close connection and cleanup resources.

        Safe to call multiple times.
        """
        pass

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check if transport is connected and ready.

        Returns:
            True if connected and operational
        """
        pass

    @abstractmethod
    def subscribe(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[Dict[str, Any]], None],
        throttle_rate: int = 0,
        queue_size: int = 1,
    ) -> SubscriptionHandle:
        """
        Subscribe to a ROS topic.

        Args:
            topic: Full topic name (e.g., "/odom", "/robot1/cmd_vel")
            message_type: ROS message type (e.g., "nav_msgs/msg/Odometry")
            callback: Function called with message dict on each message received
            throttle_rate: Minimum interval between callbacks in milliseconds (0 = no throttle)
            queue_size: Message queue size for buffering

        Returns:
            Handle that can be passed to unsubscribe()

        Raises:
            ConnectionError: If not connected
        """
        pass

    @abstractmethod
    def unsubscribe(self, handle: SubscriptionHandle) -> None:
        """
        Unsubscribe from a topic.

        Args:
            handle: Subscription handle returned by subscribe()
        """
        pass

    @abstractmethod
    def publish(
        self,
        topic: str,
        message_type: str,
        message: Dict[str, Any],
    ) -> None:
        """
        Publish a message to a ROS topic.

        Args:
            topic: Full topic name (e.g., "/cmd_vel")
            message_type: ROS message type (e.g., "geometry_msgs/msg/Twist")
            message: Message data as dictionary matching the ROS message structure

        Raises:
            ConnectionError: If not connected
        """
        pass

    @abstractmethod
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
            feedback_callback: Optional callback for progress feedback
            timeout: Optional timeout in seconds (None = wait forever)

        Returns:
            Dictionary with:
                - 'result': Action result data (or None if failed)
                - 'status': Status string ("SUCCEEDED", "FAILED", "CANCELED")

        Raises:
            ConnectionError: If not connected
            TimeoutError: If action times out
        """
        pass

    @abstractmethod
    def cancel_action(self) -> None:
        """
        Cancel the current action goal if any.

        Safe to call even if no action is in progress.
        """
        pass

    @abstractmethod
    def call_service(
        self,
        service_name: str,
        service_type: str,
        request: Dict[str, Any],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Call a ROS service and wait for response.

        Args:
            service_name: Service name (e.g., "/get_map")
            service_type: Service type (e.g., "nav_msgs/srv/GetMap")
            request: Request message as dictionary
            timeout: Timeout in seconds

        Returns:
            Service response as dictionary

        Raises:
            ConnectionError: If not connected
            TimeoutError: If service call times out
        """
        pass
