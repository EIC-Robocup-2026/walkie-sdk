"""
Lidar - 2D laser scanner stream module.

Subscribes to a sensor_msgs/msg/LaserScan topic (default ``/scan``) via the
active ROS transport (rosbridge or zenoh) and caches the latest message.
Exposes get_scan() (non-blocking) and get_once() (blocking --once equivalent).
"""

import threading
import time
from typing import Any, Dict, List, Optional

from walkie_sdk.config.ros_topics import LIDAR_TOPICS
from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace


class Lidar:
    """
    2D lidar interface.

    Subscribes to the ``scan`` topic defined in ``LIDAR_TOPICS`` via the active
    ROS transport (rosbridge or zenoh) and caches the latest LaserScan message.
    Subscription is started during ``WalkieRobot._connect()`` via
    ``_setup_subscription()``.

    Args:
        transport: ROS transport interface (rosbridge or zenoh).
        namespace: ROS namespace prefix (default: "" = no namespace).

    Example:
        ```python
        # Blocking fetch — equivalent to ros2 topic echo /scan --once
        scan = bot.lidar.get_once(timeout=5.0)
        if scan is not None:
            print(scan["range_min"], scan["range_max"])
            print(len(scan["ranges"]))   # number of beams
        ```
    """

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace
        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None
        self._handle: Any = None
        self._subscribed = False

    # ------------------------------------------------------------------
    # Namespaced topic
    # ------------------------------------------------------------------

    @property
    def scan_topic(self) -> str:
        """Namespaced LaserScan topic name."""
        return apply_namespace(LIDAR_TOPICS["scan"], self._namespace)

    @property
    def namespace(self) -> str:
        """Current ROS namespace prefix."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        self._namespace = value

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def _setup_subscription(self) -> None:
        """Subscribe to the scan topic. Called by WalkieRobot._connect()."""
        if self._subscribed:
            return
        if not self._transport.is_connected:
            return

        try:
            self._handle = self._transport.subscribe(
                topic=self.scan_topic,
                message_type=LIDAR_TOPICS["scan_type"],
                callback=self._on_scan,
                queue_size=1,
            )
            self._subscribed = True
        except Exception as e:
            print(f"  ⚠ Failed to subscribe to lidar scan topic: {e}")

    def _on_scan(self, msg: Dict[str, Any]) -> None:
        with self._lock:
            self._latest = msg

    def stop(self) -> None:
        """Unsubscribe from the scan topic."""
        if self._handle is not None:
            try:
                self._transport.unsubscribe(self._handle)
            except Exception:
                pass
        self._handle = None
        self._subscribed = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_subscribed(self) -> bool:
        """True if the subscription is active."""
        return self._subscribed

    def get_scan(self) -> Optional[Dict[str, Any]]:
        """
        Return the latest cached LaserScan message (non-blocking).

        Returns:
            ROS LaserScan message as a dict, or None if no message yet.

        Example:
            ```python
            scan = bot.lidar.get_scan()
            if scan is not None:
                print(len(scan["ranges"]))   # number of beams
            ```
        """
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def get_ranges(self) -> Optional[List[float]]:
        """
        Return just the ``ranges`` array from the latest scan (non-blocking).

        Returns:
            List of range measurements in metres, or None if no message yet.
        """
        scan = self.get_scan()
        return list(scan["ranges"]) if scan is not None else None

    def get_once(self, timeout: float = 10.0) -> Optional[Dict[str, Any]]:
        """
        Block until one scan arrives and return it.

        Equivalent to ``ros2 topic echo <scan_topic> --once``.
        Polls the cached message until it becomes non-None or the timeout expires.

        Args:
            timeout: Maximum seconds to wait (default: 10.0).

        Returns:
            ROS LaserScan message as a dict, or None if timeout reached.

        Example:
            ```python
            scan = bot.lidar.get_once(timeout=5.0)
            if scan:
                print(scan["header"]["frame_id"])
            ```
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.get_scan()
            if msg is not None:
                return msg
            time.sleep(0.05)
        return None

    def __repr__(self) -> str:
        status = "subscribed" if self._subscribed else "stopped"
        return f"Lidar(status={status}, topic='{self.scan_topic}')"
