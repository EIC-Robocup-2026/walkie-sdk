"""
PointCloudTransportInterface - Abstract interface for point cloud streams.

Defines the contract for Zenoh-backed point cloud transports,
allowing named multi-source access (e.g. "head").
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional


class PointCloudTransportInterface(ABC):
    """
    Abstract interface for point cloud transports.

    Implementations subscribe to one or more named PointCloud2 topics,
    cache the latest message per source, and expose them via get_cloud().

    Implementations:
    - ZenohPointCloud: Zenoh-based subscriber
    """

    @abstractmethod
    def connect(self) -> None:
        """
        Start subscribing to point cloud topics.

        Raises:
            ConnectionError: If unable to establish subscriptions
        """
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """
        Stop subscriptions and release resources.

        Safe to call multiple times.
        """
        pass

    @property
    @abstractmethod
    def is_streaming(self) -> bool:
        """True if subscriptions are active."""
        pass

    @property
    @abstractmethod
    def source_names(self) -> List[str]:
        """Names of subscribed point cloud sources (e.g. ['head'])."""
        pass

    @abstractmethod
    def get_cloud(self, source_name: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        Return the latest cached PointCloud2 message for the named source.

        Args:
            source_name: Source key from POINT_CLOUD_TOPICS (e.g. "head").
                         Defaults to "head" when None.

        Returns:
            ROS PointCloud2 message as a dict, or None if no message yet.
        """
        pass

    @abstractmethod
    def get_all_clouds(self) -> Dict[str, Dict[str, Any]]:
        """
        Return the latest cached clouds for all subscribed sources.

        Returns:
            Dict mapping source name to PointCloud2 message dict.
            Only includes sources that have received at least one message.
        """
        pass

    def __enter__(self) -> "PointCloudTransportInterface":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()
