"""
WalkieRobot - Main entry point for Walkie SDK.

Provides a unified interface to control the robot through
.nav, .status, and .camera submodules.

Supports multiple communication protocols via the transport abstraction layer:
- hybrid: Zenoh for topics/services + rosbridge for actions (default, fast)
- rosbridge: WebSocket via roslibpy (no ROS2 required on client)
- zenoh: Zenoh DDS bridge (good performance, topics/services only, no actions)
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
from walkie_sdk.modules.button import Button
from walkie_sdk.modules.camera import Camera
from walkie_sdk.modules.head import Head
from walkie_sdk.modules.joint_state_hub import JointStateHub
from walkie_sdk.modules.lift import Lift
from walkie_sdk.modules.multi_camera import MultiCamera
from walkie_sdk.modules.navigation import Navigation
from walkie_sdk.modules.telemetry import Telemetry
from walkie_sdk.modules.visualization import Visualization
from walkie_sdk.modules.point_cloud import PointCloud
from walkie_sdk.modules.tools import Tools
from walkie_sdk.modules.transform import Transform
from walkie_sdk.modules.grasp import Grasp

from walkie_sdk.config.ros_topics import load_config


class WalkieRobot:
    """
    Main SDK class for controlling a Walkie robot.

    Auto-connects to the robot on initialization and provides access to:
    - .nav: Navigation controls (go_to, cancel, stop)
    - .status: Telemetry data (get_position, get_velocity)
    - .camera: Camera frames (get_frame) - if enabled
    - .viz: Visualization markers for RViz2 (draw_marker, clear_markers)

    Args:
        ip: Robot IP address or hostname
        ros_protocol: ROS communication protocol to use:
            - "hybrid": zenoh for topics/services + rosbridge for actions
              (default; needs both a zenoh router and rosbridge_server on the robot)
            - "rosbridge": WebSocket via roslibpy (no ROS2 required)
            - "zenoh": Zenoh DDS bridge (topics/services only, no action support)
            - "auto": Auto-detect best available protocol
        ros_port: rosbridge WebSocket port (default: 9090); used for actions
            under "hybrid" and for everything under "rosbridge"
        zenoh_port: Zenoh router port (default: 7447); used by "hybrid"/"zenoh"
        camera_protocol: Camera stream protocol to use:
            - "zenoh": Zenoh video stream (default)
            - "usb": Local USB camera via OpenCV
            - "none": Disable camera functionality
        camera_port: Port for camera stream
        timeout: Connection timeout in seconds (default: 10.0)
        namespace: ROS namespace for topics/actions (default: "" = no namespace)
        arm_mode: Default arm control mode:
            - "moveit": MoveIt motion planning (default)
            - "custom_ik": Publish Pose to custom IK solver for teleop
        arm_target_pose_topic: Topic for custom IK mode (default: "/target_pose")
        config_path: Path to custom ROS topics configuration file (optional)

    Raises:
        ConnectionError: If connection to robot fails
        ValueError: If invalid protocol specified

    Example:
        ```python
        from walkie_sdk import WalkieRobot

        # Default: WebSocket + Zenoh camera (no ROS2 needed on client)
        bot = WalkieRobot(ip="192.168.1.100")

        bot.status.get_position()
        # {'x': 0.0, 'y': 0.0, 'heading': 0.0}

        bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
        # 'SUCCEEDED'

        bot.disconnect()

        # With namespace:
        bot = WalkieRobot(ip="192.168.1.100", namespace="robot1")
        # Topics will be /robot1/odom, /robot1/cmd_vel, etc.
        ```
    """

    def __init__(
        self,
        ip: str,
        ros_protocol: str = "hybrid",
        ros_port: int = 9090,
        zenoh_port: int = 7447,
        camera_protocol: str = "zenoh",
        camera_port: int = 7447,
        timeout: float = 10.0,
        namespace: str = "",
        config_path: str = None,
        button_key: str = "0x1008ff47",
        # Legacy parameters for backward compatibility
        ws_port: Optional[int] = None,
        enable_camera: bool = True,
    ):
        # Load custom config if provided
        if config_path:
            load_config(config_path)

        # Handle legacy parameter names for backward compatibility
        if ws_port is not None:
            ros_port = ws_port
        if not enable_camera:
            camera_protocol = "none"

        self._ip = ip
        self._ros_port = ros_port
        self._zenoh_port = zenoh_port
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
            zenoh_port=zenoh_port,
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

        # Shared joint state hub — one subscription for all modules
        self._joints = JointStateHub(self._transport, namespace=namespace)

        # Head tilt module (created early so Navigation can reference it)
        self._head = Head(self._transport, namespace=namespace, joint_state_hub=self._joints)

        # Initialize modules with transport interface (not specific implementation)
        self._nav = Navigation(self._transport, namespace=namespace, head=self._head)
        self._status = Telemetry(self._transport, namespace=namespace)
        self._arm = Arm(self._transport, namespace=namespace, joint_state_hub=self._joints)
        self._camera: Optional[Camera] = (
            Camera(self._camera_transport, ros_transport=self._transport)
            if self._camera_transport
            else None
        )

        # Multi-camera interface (wraps camera transport for multi-cam access)
        self._multi_camera: Optional[MultiCamera] = (
            MultiCamera(self._camera_transport) if self._camera_transport else None
        )

        # Tools module
        self._tools = Tools(self._transport, namespace=namespace)

        # Grasp module (GraspNet service client)
        self._grasp = Grasp(self._transport, namespace=namespace)

        # Transform lookup module
        self._transform = Transform(self._transport, namespace=namespace)

        # Visualization module (marker publishing for RViz2)
        self._viz = Visualization(self._transport, namespace=namespace)

        # Lift module
        self._lift = Lift(self._transport, namespace=namespace, joint_state_hub=self._joints)

        # Point cloud module (subscribes via the active ROS transport)
        self._point_cloud = PointCloud(self._transport, namespace=namespace)

        # Button module (Pi Pico USB HID keyboard input — no ROS transport)
        self._button = Button(key=button_key)

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

        # Single joint_states subscription shared by Arm, Lift, and Head
        self._joints._setup_subscription()

        # Connect camera if enabled
        if self._camera_transport is not None:
            try:
                self._camera_transport.connect()
            except Exception as e:
                print(f"  ⚠ Camera connection failed: {e}")
                print(f"    Camera will not be available.")
                self._camera = None

        # Start point cloud subscriptions
        self._point_cloud._setup_subscription()

        # Start button (Pi Pico HID keyboard) listener
        self._button._start()
        # Always enable auto-tilt on startup (short timeout: service is rosbridge-only,
        # so we don't want to stall 5 s waiting for the Zenoh fallback)
        self._head.set_auto_tilt(True, timeout=1.0)

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
        - get_position(): Get current pose {x, y, heading}
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
            ```python
            frames = bot.cameras.get_all_frames()
            head = bot.cameras.get_frame("head")
            ```
        """
        return self._multi_camera

    @property
    def viz(self) -> Visualization:
        """
            Visualization marker controller for RViz2.

            Provides:
            - draw_marker(position, quaternion, frame_id, ...): Publish a single marker
            - draw_markers(markers): Publish multiple markers as MarkerArray
            - update_marker(marker_id, ...): Update an existing marker
            - delete_marker(marker_id): Delete a specific marker
            - clear_markers(): Remove all markers
            - draw_pose(position, quaternion, frame_id, topic): Publish a PoseStamped
            - update_pose(position, quaternion, topic): Update an existing PoseStamped

        Example:
            ```python
            bot.viz.draw_marker([1.0, 2.0, 0.0])  # Red arrow
            bot.viz.draw_pose([1.0, 2.0, 0.0])    # Pose triad
            ```
        """
        return self._viz

    @property
    def lift(self) -> Lift:
        """
        Lift controller.

        Provides:
        - set(pos, speed, accel, norm_pos=True): Send position command
        - get(norm_pos=True): Read current position

        Positions are normalized 0.0–1.0 by default (0.0 = bottom, 1.0 = top).
        Pass norm_pos=False to use real centimeters (0.0–74.35 cm).

        Example:
            ```python
            bot.lift.set(0.5)              # move to midpoint (normalized)
            bot.lift.get()                 # e.g. 0.5
            bot.lift.set(37.0, norm_pos=False)  # 37 cm
            bot.lift.get(norm_pos=False)   # e.g. 37.0
            ```
        """
        return self._lift

    @property
    def head(self) -> Head:
        """
        Head tilt controller.

        Provides:
        - tilt(angle_rad): Set tilt angle in radians (positive = camera down)
        - get_angle(): Get last commanded tilt angle, or None if not yet set

        Safe range: ±π/4 rad (±45°). Values outside this range raise ValueError.

        Example:
            ```python
            bot.head.tilt(0.5)      # tilt camera 0.5 rad downward
            bot.head.tilt(0.0)      # look forward
            bot.head.get_angle()    # returns 0.0
            ```
        """
        return self._head

    @property
    def joints(self) -> JointStateHub:
        """
        Shared joint state hub.

        Single source of truth for all joint positions, velocities, and efforts.
        Backed by one subscription to the joint_states topic.

        Provides:
        - get(joint_name): Current position of a joint (rad or m)
        - get_velocity(joint_name): Current velocity
        - get_effort(joint_name): Current effort
        - get_all(): Full snapshot as {name: {position, velocity, effort}}

        Example:
            ```python
            bot.joints.get("head_servo_joint")   # head tilt in rad
            bot.joints.get("lift_joint")         # lift height in m
            bot.joints.get_all()                 # all joints
            ```
        """
        return self._joints

    @property
    def tools(self) -> Tools:
        """Tools module for utility functions."""
        return self._tools

    @property
    def grasp(self) -> Grasp:
        """
        Grasp controller (unified GraspNet server).

        Provides:
        - from_mask(mask, bbox, ...): grasp from a YOLO mask over the live view
        - from_cloud(cloud, ...): grasp from a segmented PointCloud2 in the request
        - from_pos(object_cloud, ...): live crop + GraspNet + antipodal validation
        - set_standby(load): load/unload the GPU model
        - status(): server state + VRAM

        Each grasp call returns a dict with a best-first ``grasps`` list
        (position/orientation/score/width) in the planning frame, or None.

        Example:
            ```python
            cloud = bot.point_cloud.get_once(timeout=10.0)
            res = bot.grasp.from_cloud(cloud)
            if res and res["grasps"]:
                print(res["grasps"][0]["position"])
            ```
        """
        return self._grasp

    @property
    def transform(self) -> Transform:
        """Transform module for coordinate frame lookups."""
        return self._transform

    @property
    def point_cloud(self) -> "PointCloud":
        """
        Point cloud interface.

        Provides:
        - get_cloud(source_name="head"): Latest cached PointCloud2 dict (non-blocking)
        - get_all_clouds(): Latest clouds for all subscribed sources
        - get_once(source_name="head", timeout=10.0): Block until first message (--once)

        Example:
            ```python
            cloud = bot.point_cloud.get_once(timeout=10.0)
            if cloud:
                print(cloud["header"]["frame_id"])
                print(cloud["width"])   # number of points
            ```
        """
        return self._point_cloud

    @property
    def button(self) -> Button:
        """
        Button state from Pi Pico USB HID keyboard input.

        Provides:
        - is_pressed: True while the physical button is held, False otherwise
        - key: The function key name being listened for (e.g. ``"f1"``)

        The Pi Pico must be programmed to send the matching key (default F1)
        as a USB HID keyboard event when the button is pressed/released.

        Example:
            ```python
            while True:
                if bot.button.is_pressed:
                    print("Button held!")
                time.sleep(0.05)
            ```
        """
        return self._button

    def draw_marker(
        self,
        position,
        quaternion=None,
        frame_id: str = "base_link",
        **kwargs,
    ) -> int:
        """
        Convenience method to publish a visualization marker to RViz2.

        Shortcut for bot.viz.draw_marker(). See Visualization.draw_marker()
        for full parameter documentation.

        Args:
            position: [x, y, z] position in the reference frame.
            quaternion: [x, y, z, w] orientation quaternion. Defaults to identity.
            frame_id: TF reference frame (default: "base_link").
            **kwargs: Additional marker options (marker_type, scale, color, etc.)

        Returns:
            The marker ID that was used.

        Example:
            ```python
            bot.draw_marker([1.0, 2.0, 0.0])
            ```
        """
        return self._viz.draw_marker(
            position=position, quaternion=quaternion, frame_id=frame_id, **kwargs
        )

    def update_marker(
        self,
        marker_id: int,
        position=None,
        quaternion=None,
        **kwargs,
    ) -> None:
        """
        Convenience method to update an existing visualization marker.

        Only pass the fields you want to change. Everything else is kept
        from the original draw_marker() call.

        Shortcut for bot.viz.update_marker(). See Visualization.update_marker()
        for full parameter documentation.

        Args:
            marker_id: ID of the marker to update.
            position: New [x, y, z] position (or None to keep current).
            quaternion: New [x, y, z, w] orientation (or None to keep current).
            **kwargs: Additional fields to update (frame_id, scale, color, etc.)

        Example:
            ```python
            mid = bot.draw_marker([0, 0, 0])
            bot.update_marker(mid, position=[1.0, 2.0, 0.0])
            ```
        """
        self._viz.update_marker(
            marker_id, position=position, quaternion=quaternion, **kwargs
        )

    def draw_pose(
        self,
        position,
        quaternion=None,
        frame_id: str = "base_link",
        **kwargs,
    ) -> str:
        """
        Convenience method to publish a PoseStamped to RViz2.

        Shortcut for bot.viz.draw_pose(). See Visualization.draw_pose()
        for full parameter documentation.

        Args:
            position: [x, y, z] position in the reference frame.
            quaternion: [x, y, z, w] orientation quaternion. Defaults to identity.
            frame_id: TF reference frame (default: "base_link").
            **kwargs: Additional options (topic, etc.)

        Returns:
            The topic string that was used.

        Example:
            ```python
            bot.draw_pose([1.0, 2.0, 0.0])
            # returns 'walkie/target_pose'
            ```
        """
        return self._viz.draw_pose(
            position=position, quaternion=quaternion, frame_id=frame_id, **kwargs
        )

    def update_pose(
        self,
        position=None,
        quaternion=None,
        **kwargs,
    ) -> None:
        """
        Convenience method to update an existing PoseStamped.

        Only pass the fields you want to change. Everything else is kept
        from the original draw_pose() call.

        Shortcut for bot.viz.update_pose(). See Visualization.update_pose()
        for full parameter documentation.

        Args:
            position: New [x, y, z] position (or None to keep current).
            quaternion: New [x, y, z, w] orientation (or None to keep current).
            **kwargs: Additional fields to update (frame_id, topic, etc.)

        Example:
            ```python
            bot.draw_pose([0, 0, 0], topic="my_pose")
            bot.update_pose(position=[1.0, 2.0, 0.0], topic="my_pose")
            ```
        """
        self._viz.update_pose(position=position, quaternion=quaternion, **kwargs)

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

        Updates namespace for navigation, telemetry, arm, lift, and visualization modules.
        Telemetry subscription keeps using the previous namespace until restart.
        Arm and lift subscriptions re-subscribe immediately using the new namespace.
        """
        self._namespace = value
        self._nav.namespace = value
        self._status.namespace = value
        self._joints.namespace = value
        self._arm.namespace = value
        self._lift.namespace = value
        self._viz.namespace = value
        self._head.namespace = value
        self._transform.namespace = value
        self._grasp.namespace = value

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

        # Stop button listener
        self._button._stop()

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
