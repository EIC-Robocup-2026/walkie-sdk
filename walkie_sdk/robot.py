"""
WalkieRobot - Main entry point for the Walkie SDK.

Provides a unified interface to control the robot through
.nav, .status, and .camera submodules.
"""

from typing import Optional

from walkie_sdk.core.bridge_client import BridgeClient
from walkie_sdk.modules.navigation import Navigation
from walkie_sdk.modules.telemetry import Telemetry


class WalkieRobot:
    """
    Main SDK class for controlling a Walkie robot.

    Auto-connects to the robot on initialization and provides access to:
    - .nav: Navigation controls (go_to, cancel, stop)
    - .status: Telemetry data (get_pose, get_velocity)

    Args:
        ip: Robot IP address or hostname
        ws_port: ROSBridge WebSocket port (default: 9090)
        timeout: Connection timeout in seconds (default: 10.0)
        enable_camera: Enable WebRTC camera stream (default: True)
        namespace: ROS namespace for topics/actions (default: "" = no namespace)

    Raises:
        ConnectionError: If connection to robot fails

    Example:
        >>> from walkie_sdk import WalkieRobot
        >>> bot = WalkieRobot(ip="192.168.1.100")
        Connecting to Walkie robot at 192.168.1.100...
          → Connecting to ROSBridge at 192.168.1.100:9090...
          ✓ ROSBridge connected
        ✓ Robot connected!

        >>> bot.status.get_pose()
        {'x': 0.0, 'y': 0.0, 'heading': 0.0}

        >>> bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
        'SUCCEEDED'

        >>> bot.disconnect()

        # With namespace:
        >>> bot = WalkieRobot(ip="192.168.1.100", namespace="robot1")
        # Topics will be /robot1/odom, /robot1/cmd_vel, etc.
    """

    def __init__(
        self,
        ip: str,
        ws_port: int = 9090,
        timeout: float = 10.0,
        enable_camera: bool = True,
        namespace: str = "",
    ):
        self._ip = ip
        self._ws_port = ws_port
        self._timeout = timeout
        self._namespace = namespace
        self._connected = False

        # Initialize clients
        self._bridge = BridgeClient(host=ip, port=ws_port, timeout=timeout)

        # Initialize modules with namespace
        self._nav = Navigation(self._bridge, namespace=namespace)
        self._status = Telemetry(self._bridge, namespace=namespace)

        # Auto-connect
        self._connect()

    def _connect(self) -> None:
        """Connect to robot and start modules."""
        print(f"Connecting to Walkie robot at {self._ip}...")

        # Connect ROSBridge
        try:
            self._bridge.connect()
        except ConnectionError as e:
            raise ConnectionError(f"Failed to connect to robot: {e}") from e

        # Start telemetry subscription
        self._status.start()

        self._connected = True
        print(f"✓ Robot connected!")

    @property
    def nav(self) -> Navigation:
        """
        Navigation controller.

        Provides:
        - go_to(x, y, heading, blocking=True): Navigate to pose
        - cancel(): Cancel current navigation
        - stop(): Emergency stop
        """
        return self._nav

    @property
    def status(self) -> Telemetry:
        """
        Telemetry/status provider.

        Provides:
        - get_pose(): Get current pose {x, y, heading}
        - get_velocity(): Get current velocity {linear, angular}
        """
        return self._status

    @property
    def ip(self) -> str:
        """Robot IP address."""
        return self._ip

    @property
    def namespace(self) -> str:
        """Current ROS namespace for topics/actions."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """
        Set ROS namespace for topics/actions.

        Updates namespace for both navigation and telemetry modules.
        Note: Telemetry subscription will use old namespace until restart.
        """
        self._namespace = value
        self._nav.namespace = value
        self._status.namespace = value

    @property
    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._connected and self._bridge.is_connected

    def disconnect(self) -> None:
        """
        Disconnect from the robot.

        Stops all subscriptions, closes WebRTC stream, and terminates
        ROSBridge connection. Safe to call multiple times.
        """
        if not self._connected:
            return

        print(f"Disconnecting from robot...")

        # Stop telemetry
        self._status.stop()

        # Disconnect ROSBridge
        self._bridge.disconnect()

        self._connected = False
        print(f"✓ Robot disconnected")

    def __enter__(self) -> "WalkieRobot":
        """Context manager entry (already connected)."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit - disconnect."""
        self.disconnect()

    def __del__(self) -> None:
        """Destructor - ensure clean disconnect."""
        try:
            self.disconnect()
        except Exception:
            pass

    def __repr__(self) -> str:
        status = "connected" if self.is_connected else "disconnected"
        return f"WalkieRobot(ip='{self._ip}', status={status})"
