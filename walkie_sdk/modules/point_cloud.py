"""
PointCloud - Robot point cloud stream module.

Subscribes to PointCloud2 topics via the active ROS transport (rosbridge or zenoh)
and caches the latest message per named source. Exposes get_cloud(), get_all_clouds(),
and get_once() (blocking --once equivalent).
"""

import threading
import time
from typing import Any, Dict, List, Optional

from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
from walkie_sdk.core.interfaces import ROSTransportInterface


_PC_MSG_TYPE = "sensor_msgs/msg/PointCloud2"


class PointCloud:
    """
    Robot point cloud interface.

    Subscribes to all sources defined in ``POINT_CLOUD_TOPICS`` via the
    active ROS transport (rosbridge or zenoh). Sources are named by their
    config key (e.g. "head"). Subscription is started during
    ``WalkieRobot._connect()`` via ``_setup_subscription()``.

    Args:
        transport: ROS transport interface (rosbridge or zenoh).
        namespace: ROS namespace prefix (default: "" = no namespace).

    Example:
        ```python
        # Blocking fetch — equivalent to ros2 topic echo <topic> --once
        cloud = bot.point_cloud.get_once("head", timeout=10.0)
        if cloud is not None:
            print(cloud["header"]["frame_id"])
            print(cloud["width"])   # number of points
        ```
    """

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace
        self._lock = threading.Lock()
        self._latest: Dict[str, Dict[str, Any]] = {}
        self._handles: List[Any] = []
        self._subscribed = False

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def _setup_subscription(self) -> None:
        """Subscribe to all POINT_CLOUD_TOPICS sources. Called by WalkieRobot._connect()."""
        if self._subscribed:
            return
        if not self._transport.is_connected:
            return

        try:
            for name, topic in POINT_CLOUD_TOPICS.items():
                handle = self._transport.subscribe(
                    topic=topic,
                    message_type=_PC_MSG_TYPE,
                    callback=self._make_cb(name),
                    queue_size=1,
                )
                self._handles.append(handle)
            self._subscribed = True
        except Exception as e:
            print(f"  ⚠ Failed to subscribe to point cloud topics: {e}")

    def _make_cb(self, source_name: str):
        def _cb(msg: Dict[str, Any]) -> None:
            with self._lock:
                self._latest[source_name] = msg
        return _cb

    def stop(self) -> None:
        """Unsubscribe from all point cloud topics."""
        for handle in self._handles:
            try:
                self._transport.unsubscribe(handle)
            except Exception:
                pass
        self._handles.clear()
        self._subscribed = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_subscribed(self) -> bool:
        """True if subscriptions are active."""
        return self._subscribed

    @property
    def source_names(self) -> List[str]:
        """Names of subscribed point cloud sources (e.g. ['head'])."""
        return list(POINT_CLOUD_TOPICS.keys())

    def get_cloud(self, source_name: str = "head") -> Optional[Dict[str, Any]]:
        """
        Return the latest cached PointCloud2 message (non-blocking).

        Args:
            source_name: Key from POINT_CLOUD_TOPICS (default: "head").

        Returns:
            ROS PointCloud2 message as a dict, or None if no message yet.

        Example:
            ```python
            cloud = bot.point_cloud.get_cloud()
            if cloud is not None:
                print(cloud["width"])   # number of points
            ```
        """
        with self._lock:
            msg = self._latest.get(source_name)
            return dict(msg) if msg is not None else None

    def get_all_clouds(self) -> Dict[str, Dict[str, Any]]:
        """
        Return the latest cached PointCloud2 messages for all sources.

        Returns:
            Dict mapping source name to PointCloud2 message dict.
            Only includes sources that have received at least one message.
        """
        with self._lock:
            return {k: dict(v) for k, v in self._latest.items()}

    def get_once(self, source_name: str = "head", timeout: float = 10.0) -> Optional[Dict[str, Any]]:
        """
        Block until one message arrives and return it.

        Equivalent to ``ros2 topic echo <topic> --once``.
        Polls the cached message until it becomes non-None or the timeout expires.

        Args:
            source_name: Key from POINT_CLOUD_TOPICS (default: "head").
            timeout: Maximum seconds to wait (default: 10.0).

        Returns:
            ROS PointCloud2 message as a dict, or None if timeout reached.

        Example:
            ```python
            cloud = bot.point_cloud.get_once(timeout=10.0)
            if cloud:
                print(cloud["header"]["frame_id"])
            ```
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.get_cloud(source_name)
            if msg is not None:
                return msg
            time.sleep(0.05)
        return None

    def __repr__(self) -> str:
        status = "subscribed" if self._subscribed else "stopped"
        sources = ", ".join(self.source_names)
        return f"PointCloud(status={status}, sources=[{sources}])"
