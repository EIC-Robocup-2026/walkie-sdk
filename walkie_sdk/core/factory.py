"""
TransportFactory - Factory for creating transport instances.

Provides a centralized way to create ROS and camera transports
based on the selected protocol, with lazy loading of implementations.
"""

from enum import Enum
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from walkie_sdk.core.interfaces import (
        CameraTransportInterface,
        ROSTransportInterface,
    )


class ROSProtocol(Enum):
    """
    Available ROS communication protocols.

    Each protocol has different requirements and trade-offs:
    - ROSBRIDGE: WebSocket via roslibpy. No ROS2 needed on client. Higher latency.
    - ZENOH: Zenoh DDS bridge. Good performance. No ROS2 needed on client.
    - AUTO: Auto-detect best available protocol.
    """

    ROSBRIDGE = "rosbridge"
    ZENOH = "zenoh"
    AUTO = "auto"


class CameraProtocol(Enum):
    """
    Available camera stream protocols.

    - WEBRTC: WebRTC stream. Pairs with rosbridge. Low latency video.
    - ZENOH: Zenoh video stream. Pairs with zenoh transport.
    - SHM: Shared memory. Direct access for same-host scenarios.
    - NONE: Disable camera functionality.
    """

    WEBRTC = "webrtc"
    ZENOH = "zenoh"
    SHM = "shm"
    NONE = "none"


class TransportFactory:
    """
    Factory for creating transport instances based on protocol selection.

    Uses lazy loading to only import the required transport implementation,
    avoiding unnecessary dependencies when using a specific protocol.

    Example:
        from walkie_sdk.core.factory import TransportFactory, ROSProtocol

        # Create a rosbridge transport
        transport = TransportFactory.create_ros_transport(
            protocol=ROSProtocol.ROSBRIDGE,
            host="192.168.1.100",
            port=9090,
        )

        # Create with auto-detection
        transport = TransportFactory.create_ros_transport(
            protocol=ROSProtocol.AUTO,
            host="192.168.1.100",
        )
    """

    @staticmethod
    def create_ros_transport(
        protocol: ROSProtocol,
        host: str,
        port: int = 9090,
        timeout: float = 10.0,
        **kwargs,
    ) -> "ROSTransportInterface":
        """
        Create a ROS transport for the specified protocol.

        Args:
            protocol: The ROS protocol to use (ROSProtocol enum)
            host: Robot IP address or hostname
            port: Port for the transport (default: 9090 for rosbridge)
            timeout: Connection timeout in seconds
            **kwargs: Additional protocol-specific arguments

        Returns:
            A transport instance implementing ROSTransportInterface

        Raises:
            ValueError: If protocol is unknown
            ImportError: If required dependencies are not installed
            ConnectionError: If AUTO protocol fails to find a working transport
        """
        if protocol == ROSProtocol.ROSBRIDGE:
            from walkie_sdk.core.transports.rosbridge import ROSBridgeTransport

            return ROSBridgeTransport(host=host, port=port, timeout=timeout)

        elif protocol == ROSProtocol.ZENOH:
            from walkie_sdk.core.transports.zenoh import ZenohTransport

            return ZenohTransport(host=host, port=port, timeout=timeout, **kwargs)

        elif protocol == ROSProtocol.AUTO:
            return TransportFactory._auto_detect_ros(host, port, timeout, **kwargs)

        else:
            raise ValueError(f"Unknown ROS protocol: {protocol}")

    @staticmethod
    def create_camera_transport(
        protocol: CameraProtocol,
        host: str,
        port: int = 8554,
        ros_transport: Optional["ROSTransportInterface"] = None,
        topic: str = "/camera/image_raw",
        **kwargs,
    ) -> Optional["CameraTransportInterface"]:
        """
        Create a camera transport for the specified protocol.

        Args:
            protocol: The camera protocol to use (CameraProtocol enum)
            host: Robot IP address or hostname
            port: Port for the camera stream (default: 8554 for WebRTC)
            ros_transport: ROS transport instance (required for ROS_IMAGE protocol)
            topic: Camera topic name (for ROS_IMAGE protocol)
            **kwargs: Additional protocol-specific arguments

        Returns:
            A camera transport instance, or None if protocol is NONE

        Raises:
            ValueError: If protocol is unknown or requirements not met
            ImportError: If required dependencies are not installed
        """
        if protocol == CameraProtocol.NONE:
            return None

        if protocol == CameraProtocol.WEBRTC:
            from walkie_sdk.core.transports.rosbridge import WebRTCCamera

            return WebRTCCamera(host=host, port=port, **kwargs)

        elif protocol == CameraProtocol.ZENOH:
            from walkie_sdk.core.transports.zenoh import ZenohCamera

            # Check for multi_camera flag in kwargs
            multi_camera = kwargs.pop("multi_camera", False)
            camera_name = kwargs.pop("camera_name", "head")
            return ZenohCamera(
                host=host,
                port=port,
                multi_camera=multi_camera,
                camera_name=camera_name,
                **kwargs,
            )

        elif protocol == CameraProtocol.SHM:
            from walkie_sdk.core.transports.shm import MultiSharedMemoryCamera

            # Use multi-camera SHM transport
            camera_names = kwargs.pop("camera_names", ["head", "left", "right"])
            return MultiSharedMemoryCamera(camera_names=camera_names, **kwargs)

        else:
            raise ValueError(f"Unknown camera protocol: {protocol}")

    @staticmethod
    def _auto_detect_ros(
        host: str,
        port: int,
        timeout: float,
        **kwargs,
    ) -> "ROSTransportInterface":
        """
        Auto-detect the best available ROS transport.

        Tries protocols in order of preference:
        1. zenoh (good performance, no ROS2 needed)
        2. rosbridge (fallback, always available)

        Args:
            host: Robot IP address
            port: Port number
            timeout: Connection timeout
            **kwargs: Additional arguments

        Returns:
            A connected transport instance

        Raises:
            ConnectionError: If no transport could connect
        """
        errors = []

        # Try zenoh first
        try:
            from walkie_sdk.core.transports.zenoh import ZenohTransport

            transport = ZenohTransport(host=host, port=port, timeout=timeout, **kwargs)
            transport.connect()
            print(f"  ✓ Auto-detected: zenoh")
            return transport
        except ImportError:
            errors.append("zenoh: not installed")
        except Exception as e:
            errors.append(f"zenoh: {e}")

        # Fall back to rosbridge (should always be available)
        try:
            from walkie_sdk.core.transports.rosbridge import ROSBridgeTransport

            transport = ROSBridgeTransport(host=host, port=port, timeout=timeout)
            transport.connect()
            print(f"  ✓ Auto-detected: rosbridge (WebSocket)")
            return transport
        except ImportError:
            errors.append("rosbridge: roslibpy not installed")
        except Exception as e:
            errors.append(f"rosbridge: {e}")

        # All protocols failed
        error_details = "\n  ".join(errors)
        raise ConnectionError(
            f"Auto-detection failed. No transport could connect:\n  {error_details}"
        )

    @staticmethod
    def get_default_camera_protocol(ros_protocol: ROSProtocol) -> CameraProtocol:
        """
        Get the default camera protocol for a given ROS protocol.

        Args:
            ros_protocol: The ROS protocol being used

        Returns:
            The recommended camera protocol to pair with it
        """
        protocol_map = {
            ROSProtocol.ROSBRIDGE: CameraProtocol.WEBRTC,
            ROSProtocol.ZENOH: CameraProtocol.ZENOH,
            ROSProtocol.AUTO: CameraProtocol.WEBRTC,  # Safe default
        }
        return protocol_map.get(ros_protocol, CameraProtocol.NONE)
