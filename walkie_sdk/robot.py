"""
WalkieRobot - Main entry point for Walkie SDK.

Provides a unified interface to control the robot through
.nav, .status, and .camera submodules.

Supports multiple communication protocols via the transport abstraction layer:
- rosbridge: WebSocket via roslibpy (default, no ROS2 required on client)
- zenoh: Zenoh DDS bridge (good performance, no ROS2 required)
"""

from typing import Optional

from walkie_sdk.core.factory import (
    CameraProtocol,
    ROSProtocol,
    TransportFactory,
)
from walkie_sdk.core.interfaces import (
    CameraTransportInterface,
    ROSTransportInterface,
)
from walkie_sdk.modules.arm import Arm
from walkie_sdk.modules.camera import Camera
from walkie_sdk.modules.multi_camera import MultiCamera
from walkie_sdk.modules.navigation import Navigation
from walkie_sdk.modules.telemetry import Telemetry


class WalkieRobot:
    """
    Main SDK class for controlling a Walkie robot.

    Auto-connects to the robot on initialization and provides access to:
    - .nav: Navigation controls (go_to, cancel, stop)
    - .status: Telemetry data (get_pose, get_velocity)
    - .camera: Camera frames (get_frame) - if enabled

    Args:
        ip: Robot IP address or hostname
        ros_protocol: ROS communication protocol to use:
            - "rosbridge": WebSocket via roslibpy (default, no ROS2 required)
            - "zenoh": Zenoh DDS bridge (no ROS2 required)
            - "auto": Auto-detect best available protocol
        ros_port: Port for ROS transport (default: 9090 for rosbridge)
        camera_protocol: Camera stream protocol to use:
            - "webrtc": WebRTC stream (default, pairs with rosbridge)
            - "zenoh": Zenoh video stream (pairs with zenoh)
            - "shm": Shared memory (same-host only)
            - "none": Disable camera functionality
        camera_port: Port for camera stream (default: 8554 for WebRTC)
        timeout: Connection timeout in seconds (default: 10.0)
        namespace: ROS namespace for topics/actions (default: "" = no namespace)

    Raises:
        ConnectionError: If connection to robot fails
        ValueError: If invalid protocol specified

    Example:
        >>> from walkie_sdk import WalkieRobot
        >>>
        >>> # Default: WebSocket + WebRTC (no ROS2 needed on client)
        >>> bot = WalkieRobot(ip="192.168.1.100")
        >>>
        >>> bot.status.get_pose()
        {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        >>>
        >>> bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
        'SUCCEEDED'
        >>>
        >>> bot.disconnect()

        # With namespace:
        >>> bot = WalkieRobot(ip="192.168.1.100", namespace="robot1")
        # Topics will be /robot1/odom, /robot1/cmd_vel, etc.
    """

    def __init__(
        self,
        ip: str,
        ros_protocol: str = "rosbridge",
        ros_port: int = 9090,
        camera_protocol: str = "webrtc",
        camera_port: int = 8554,
        timeout: float = 10.0,
        namespace: str = "",
        # Legacy parameters for backward compatibility
        ws_port: Optional[int] = None,
        webrtc_port: Optional[int] = None,
        enable_camera: bool = True,
    ):
        # Handle legacy parameter names for backward compatibility
        if ws_port is not None:
            ros_port = ws_port
        if webrtc_port is not None:
            camera_port = webrtc_port
        if not enable_camera:
            camera_protocol = "none"

        self._ip = ip
        self._ros_port = ros_port
        self._camera_port = camera_port
        self._timeout = timeout
        self._namespace = namespace
        self._connected = False

        # Parse protocol enums
        try:
            self._ros_protocol = ROSProtocol(ros_protocol)
        except ValueError:
            valid = [p.value for p in ROSProtocol]
            raise ValueError(
                f"Invalid ros_protocol '{ros_protocol}'. Valid options: {valid}"
            )

        try:
            self._camera_protocol = CameraProtocol(camera_protocol)
        except ValueError:
            valid = [p.value for p in CameraProtocol]
            raise ValueError(
                f"Invalid camera_protocol '{camera_protocol}'. Valid options: {valid}"
            )

        # Create ROS transport via factory
        self._transport: ROSTransportInterface = TransportFactory.create_ros_transport(
            protocol=self._ros_protocol,
            host=ip,
            port=ros_port,
            timeout=timeout,
        )

        # Create camera transport via factory (may be None)
        self._camera_transport: Optional[CameraTransportInterface] = (
            TransportFactory.create_camera_transport(
                protocol=self._camera_protocol,
                host=ip,
                port=camera_port,
                ros_transport=self._transport,
            )
        )

        # Initialize modules with transport interface (not specific implementation)
        self._nav = Navigation(self._transport, namespace=namespace)
        self._status = Telemetry(self._transport, namespace=namespace)
        self._arm = Arm(self._transport, namespace=namespace)
        self._camera: Optional[Camera] = (
            Camera(self._camera_transport) if self._camera_transport else None
        )

        # Multi-camera interface (wraps camera transport for multi-cam access)
        self._multi_camera: Optional[MultiCamera] = (
            MultiCamera(self._camera_transport) if self._camera_transport else None
        )

        # Auto-connect
        self._connect()

    def _connect(self) -> None:
        """Connect to robot and start modules."""
        print(f"Connecting to Walkie robot at {self._ip}...")
        print(f"  Protocol: {self._ros_protocol.value}")

        # Connect ROS transport
        try:
            self._transport.connect()
        except ConnectionError as e:
            raise ConnectionError(f"Failed to connect to robot: {e}") from e

        # Start telemetry subscription
        self._status.start()

        # Setup arm subscription (must be done after transport is connected)
        self._arm._setup_state_subscription()

        # Connect camera if enabled
        if self._camera_transport is not None:
            try:
                self._camera_transport.connect()
            except Exception as e:
                print(f"  ⚠ Camera connection failed: {e}")
                print(f"    Camera will not be available.")
                self._camera = None

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
    def arm(self) -> Arm:
        """
        Arm controller.

        Provides:
        - set_joint_positions(left_arm, right_arm, ...): Set joint positions
        - set_joint_velocities(left_arm, right_arm, ...): Set joint velocities
        - set_joint_torques(left_arm, right_arm, ...): Set joint torques
        - get_joint_states(): Get current joint states
        """
        return self._arm

    @property
    def camera(self) -> Optional[Camera]:
        """
        Camera interface (if enabled).

        Provides:
        - get_frame(): Get latest camera frame as numpy array
        - is_streaming: Check if camera is active

        Returns None if camera was disabled or failed to connect.
        """
        return self._camera

    @property
    def cameras(self) -> Optional[MultiCamera]:
        """
        Multi-camera interface (if enabled).

        Provides access to multiple cameras on the robot:
        - get_head_frame(): Get head/front camera frame
        - get_left_frame(): Get left wrist camera frame
        - get_right_frame(): Get right wrist camera frame
        - get_all_frames(): Get all camera frames as dict
        - get_frame(camera_name): Get frame from specific camera

        Returns None if camera was disabled or failed to connect.

        Example:
            >>> frames = bot.cameras.get_all_frames()
            >>> head = bot.cameras.get_head_frame()
        """
        return self._multi_camera

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
        self._arm.namespace = value

    @property
    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._connected and self._transport.is_connected

    @property
    def ros_protocol(self) -> str:
        """Get the ROS protocol being used."""
        return self._ros_protocol.value

    @property
    def camera_protocol(self) -> str:
        """Get the camera protocol being used."""
        return self._camera_protocol.value

    def disconnect(self) -> None:
        """
        Disconnect from the robot.

        Stops all subscriptions, closes camera stream, and terminates
        ROS transport connection. Safe to call multiple times.
        """
        if not self._connected:
            return

        print(f"Disconnecting from robot...")

        # Stop telemetry
        self._status.stop()

        # Stop camera
        if self._camera is not None:
            try:
                self._camera.stop()
            except Exception:
                pass

        # Disconnect camera transport
        if self._camera_transport is not None:
            try:
                self._camera_transport.disconnect()
            except Exception:
                pass

        # Disconnect ROS transport
        self._transport.disconnect()

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
        return (
            f"WalkieRobot(ip='{self._ip}', "
            f"ros_protocol='{self._ros_protocol.value}', "
            f"status={status})"
        )
