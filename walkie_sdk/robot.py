"""
WalkieRobot - Main entry point for the Walkie SDK.

Provides a unified interface to control the robot through
.nav, .status, and .camera submodules.
"""

from typing import Optional

from walkie_sdk.core.bridge_client import BridgeClient
from walkie_sdk.core.webrtc_client import WebRTCClient
from walkie_sdk.modules.navigation import Navigation
from walkie_sdk.modules.telemetry import Telemetry
from walkie_sdk.modules.camera import Camera


class WalkieRobot:
    """
    Main SDK class for controlling a Walkie robot.
    
    Auto-connects to the robot on initialization and provides access to:
    - .nav: Navigation controls (go_to, cancel, stop)
    - .status: Telemetry data (get_pose, get_velocity)
    - .camera: Video stream (get_frame)
    
    Args:
        ip: Robot IP address or hostname
        ws_port: ROSBridge WebSocket port (default: 9090)
        webrtc_port: WebRTC signaling port (default: 8554)
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
          → Connecting to WebRTC at 192.168.1.100:8554...
          ✓ WebRTC stream connected
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
        webrtc_port: int = 8554,
        timeout: float = 10.0,
        enable_camera: bool = True,
        namespace: str = ""
    ):
        self._ip = ip
        self._ws_port = ws_port
        self._webrtc_port = webrtc_port
        self._timeout = timeout
        self._enable_camera = enable_camera
        self._namespace = namespace
        self._connected = False
        
        # Initialize clients
        self._bridge = BridgeClient(host=ip, port=ws_port, timeout=timeout)
        self._webrtc: Optional[WebRTCClient] = None
        if enable_camera:
            self._webrtc = WebRTCClient(host=ip, port=webrtc_port, timeout=timeout)
        
        # Initialize modules with namespace
        self._nav = Navigation(self._bridge, namespace=namespace)
        self._status = Telemetry(self._bridge, namespace=namespace)
        self._camera: Optional[Camera] = None
        if self._webrtc:
            self._camera = Camera(self._webrtc)
        
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
        
        # Connect WebRTC (optional, don't fail if unavailable)
        if self._webrtc:
            try:
                self._webrtc.connect()
            except ConnectionError as e:
                print(f"  ⚠ WebRTC connection failed: {e}")
                print(f"    Camera will not be available.")
                self._camera = None
        
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
    def camera(self) -> Optional[Camera]:
        """
        Camera interface.
        
        Provides:
        - get_frame(): Get latest camera frame as numpy array
        
        Returns None if camera is not enabled or failed to connect.
        """
        return self._camera
    
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
        
        # Disconnect WebRTC
        if self._webrtc:
            self._webrtc.disconnect()
        
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
