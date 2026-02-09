"""
Arm - Robot arm control module.

Provides set_joint_positions(), set_joint_velocities(), set_joint_torques(),
and get_joint_states() functions for controlling robot's arms.

Supports two control modes:
- MOVEIT: Motion planning via MoveIt action servers (default)
- CUSTOM_IK: Teleop via publishing geometry_msgs/Pose to a custom IK solver node

This module uses ROSTransportInterface abstraction, allowing it
to work with any transport implementation (rosbridge, zenoh).
"""

import threading
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.converters import euler_to_quaternion
from walkie_sdk.utils.namespace import apply_namespace

# Default arm topic names (without namespace)
DEFAULT_ARM_COMMANDS_TOPIC = "walkie/arm/commands"
DEFAULT_ARM_STATES_TOPIC = "/joint_states"
ARM_COMMANDS_TYPE = "sensor_msgs/msg/JointState"
ARM_STATES_TYPE = "sensor_msgs/msg/JointState"

MOVEIT_ACTION_INTERFACE = "my_robot_interfaces/action"

# Custom IK defaults
DEFAULT_TARGET_POSE_TOPIC = "/target_pose"
TARGET_POSE_TYPE = "geometry_msgs/msg/PoseStamped"


class ArmControlMode(Enum):
    """Control mode for arm pose commands.

    Attributes:
        MOVEIT: Use MoveIt action servers for motion planning (default).
        CUSTOM_IK: Publish geometry_msgs/Pose to a custom IK solver node for teleop.
    """

    MOVEIT = "moveit"
    CUSTOM_IK = "custom_ik"


class Arm:
    """
    Robot arm controller.

    Provides methods to control dual-arm robot joints via position, velocity,
    or torque commands, and to read current joint states.

    This class works with any transport that implements ROSTransportInterface,
    making it protocol-agnostic (works with rosbridge, zenoh, etc.).

    Args:
        transport: Transport instance implementing ROSTransportInterface
        namespace: ROS namespace prefix for topics (default: "" = no namespace)
    """

    # Debug settings
    DEBUG_SUBSCRIBE = False  # Set to True to enable debug prints
    DEBUG_INTERVAL = 100  # Print every N messages

    def __init__(
        self,
        transport: ROSTransportInterface,
        namespace: str = "",
        default_mode: ArmControlMode = ArmControlMode.MOVEIT,
        target_pose_topic: str = DEFAULT_TARGET_POSE_TOPIC,
    ):
        self._transport = transport
        self._namespace = namespace
        self._default_mode = (
            ArmControlMode(default_mode)
            if isinstance(default_mode, str)
            else default_mode
        )
        self._target_pose_topic = target_pose_topic
        self._states_lock = threading.Lock()
        self._latest_states: Optional[Dict[str, any]] = None
        self._subscribed = False
        self._msg_count = 0  # Debug counter

        # Try to subscribe to joint states (may fail if transport not connected yet)
        self._setup_state_subscription()

    def _setup_state_subscription(self):
        """Setup subscription to joint states."""
        # Skip if already subscribed
        if self._subscribed:
            return

        def state_callback(msg: Dict):
            self._msg_count += 1
            if self.DEBUG_SUBSCRIBE and self._msg_count % self.DEBUG_INTERVAL == 0:
                names = msg.get("name", [])
                positions = msg.get("position", [])
                print(
                    f"[Arm] Received #{self._msg_count} | joints={len(names)} | pos[0:3]={positions[:3] if positions else 'N/A'}"
                )
            elif self.DEBUG_SUBSCRIBE and self._msg_count == 1:
                print(f"[Arm] First message received! Keys: {list(msg.keys())}")
            with self._states_lock:
                self._latest_states = msg

        try:
            states_topic = apply_namespace(DEFAULT_ARM_STATES_TOPIC, self._namespace)
            print(f"[Arm] Subscribing to topic: '{states_topic}'")
            self._transport.subscribe(states_topic, ARM_STATES_TYPE, state_callback)
            self._subscribed = True
            print(f"[Arm] Successfully subscribed to '{states_topic}'")
        except Exception as e:
            print(f"[Arm] Failed to subscribe to joint states: {e}")

    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """Set ROS namespace for topics."""
        self._namespace = value
        # Reset subscription state and re-subscribe with new namespace
        self._subscribed = False
        self._setup_state_subscription()

    @property
    def arm_commands_topic(self) -> str:
        """Get the full arm commands topic name with namespace."""
        return apply_namespace(DEFAULT_ARM_COMMANDS_TOPIC, self._namespace)

    @property
    def arm_states_topic(self) -> str:
        """Get the full arm states topic name with namespace."""
        return apply_namespace(DEFAULT_ARM_STATES_TOPIC, self._namespace)

    @property
    def default_mode(self) -> ArmControlMode:
        """Current default control mode for go_to_pose() and go_to_pose_quaternion()."""
        return self._default_mode

    @default_mode.setter
    def default_mode(self, value: Union[str, ArmControlMode]) -> None:
        """Set default control mode.

        Args:
            value: ArmControlMode enum or string ("moveit", "custom_ik").
        """
        self._default_mode = ArmControlMode(value) if isinstance(value, str) else value
        print(f"[Arm] Default control mode set to: {self._default_mode.value}")

    @property
    def target_pose_topic(self) -> str:
        """Get the target pose topic for custom IK mode."""
        return self._target_pose_topic

    @target_pose_topic.setter
    def target_pose_topic(self, value: str) -> None:
        """Set the target pose topic for custom IK mode."""
        self._target_pose_topic = value
        print(f"[Arm] Target pose topic set to: {self._target_pose_topic}")

    def set_joint_positions(
        self,
        left_arm: Optional[List[float]] = None,
        right_arm: Optional[List[float]] = None,
        left_gripper: Optional[float] = None,
        right_gripper: Optional[float] = None,
        blocking: bool = True,
    ) -> bool:
        """
        Set joint positions for the robot arms.

        Args:
            left_arm: List of 7 joint positions for left arm (radians)
            right_arm: List of 7 joint positions for right arm (radians)
            left_gripper: Gripper position for left gripper (0.0 to 1.0)
            right_gripper: Gripper position for right gripper (0.0 to 1.0)
            blocking: If True, wait for command to be sent (default: True)

        Returns:
            True if command was sent successfully
        """
        try:
            # Build joint state message
            positions = []
            names = []

            # Add left arm joints
            if left_arm:
                for i in range(1, 8):
                    names.append(f"left_joint{i}")
                    positions.append(left_arm[i - 1] if i - 1 < len(left_arm) else 0.0)

            # Add right arm joints
            if right_arm:
                for i in range(1, 8):
                    names.append(f"right_joint{i}")
                    positions.append(
                        right_arm[i - 1] if i - 1 < len(right_arm) else 0.0
                    )

            # Add grippers
            if left_gripper is not None:
                names.append("left_gripper_controller")
                positions.append(left_gripper)

            if right_gripper is not None:
                names.append("right_gripper_controller")
                positions.append(right_gripper)

            # Create JointState message
            msg = {
                "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
                "name": names,
                "position": positions,
                "velocity": [0.0] * len(positions),
                "effort": [0.0] * len(positions),
            }

            # Publish command
            self._transport.publish(self.arm_commands_topic, ARM_COMMANDS_TYPE, msg)

            return True
        except Exception as e:
            print(f"[Arm] Error setting joint positions: {e}")
            return False

    def set_joint_velocities(
        self,
        left_arm: Optional[List[float]] = None,
        right_arm: Optional[List[float]] = None,
        blocking: bool = True,
    ) -> bool:
        """
        Set joint velocities for the robot arms.

        Args:
            left_arm: List of 7 joint velocities for left arm (rad/s)
            right_arm: List of 7 joint velocities for right arm (rad/s)
            blocking: If True, wait for command to be sent

        Returns:
            True if command was sent successfully
        """
        # Similar to set_joint_positions but with velocities
        # TODO: Implement velocity control
        print("[Arm] Velocity control not yet implemented")
        return False

    def set_joint_torques(
        self,
        left_arm: Optional[List[float]] = None,
        right_arm: Optional[List[float]] = None,
        blocking: bool = True,
    ) -> bool:
        """
        Set joint torques for the robot arms.

        Args:
            left_arm: List of 7 joint torques for left arm (Nm)
            right_arm: List of 7 joint torques for right arm (Nm)
            blocking: If True, wait for command to be sent

        Returns:
            True if command was sent successfully
        """
        # Similar to set_joint_positions but with torques
        # TODO: Implement torque control
        print("[Arm] Torque control not yet implemented")
        return False

    # tested
    def get_joint_states(self) -> Optional[Dict[str, any]]:
        """
        Get current joint states.

        Returns:
            Dictionary with joint states, or None if not available.
            Format:
            {
                "left_arm": {
                    "positions": [7 floats],
                    "velocities": [7 floats],
                    "torques": [7 floats]
                },
                "right_arm": {
                    "positions": [7 floats],
                    "velocities": [7 floats],
                    "torques": [7 floats]
                },
                "left_gripper": float,
                "right_gripper": float
            }
        """
        with self._states_lock:
            if self._latest_states is None:
                return None

            # Parse joint state message
            try:
                msg = self._latest_states
                names = msg.get("name", [])
                positions = msg.get("position", [])
                velocities = msg.get("velocity", [])
                efforts = msg.get("effort", [])

                # Extract left arm joints
                left_arm_pos = []
                left_arm_vel = []
                left_arm_torque = []

                right_arm_pos = []
                right_arm_vel = []
                right_arm_torque = []

                left_gripper = None
                right_gripper = None

                for i, name in enumerate(names):
                    if name.startswith("left_joint"):
                        idx = int(name.replace("left_joint", "")) - 1
                        if idx < 7:
                            while len(left_arm_pos) <= idx:
                                left_arm_pos.append(0.0)
                                left_arm_vel.append(0.0)
                                left_arm_torque.append(0.0)
                            if i < len(positions):
                                left_arm_pos[idx] = positions[i]
                            if i < len(velocities):
                                left_arm_vel[idx] = velocities[i]
                            if i < len(efforts):
                                left_arm_torque[idx] = efforts[i]
                    elif name.startswith("right_joint"):
                        idx = int(name.replace("right_joint", "")) - 1
                        if idx < 7:
                            while len(right_arm_pos) <= idx:
                                right_arm_pos.append(0.0)
                                right_arm_vel.append(0.0)
                                right_arm_torque.append(0.0)
                            if i < len(positions):
                                right_arm_pos[idx] = positions[i]
                            if i < len(velocities):
                                right_arm_vel[idx] = velocities[i]
                            if i < len(efforts):
                                right_arm_torque[idx] = efforts[i]
                    elif name.startswith("left_gripper"):
                        if i < len(positions):
                            left_gripper = positions[i]
                    elif name.startswith("right_gripper_controller"):
                        if i < len(positions):
                            right_gripper = positions[i]

                return {
                    "left_arm": {
                        "positions": left_arm_pos[:7],
                        "velocities": left_arm_vel[:7],
                        "torques": left_arm_torque[:7],
                    },
                    "right_arm": {
                        "positions": right_arm_pos[:7],
                        "velocities": right_arm_vel[:7],
                        "torques": right_arm_torque[:7],
                    },
                    "left_gripper": left_gripper,
                    "right_gripper": right_gripper,
                }
            except Exception as e:
                print(f"[Arm] Error parsing joint states: {e}")
                return None

    # ── Custom IK helpers ──────────────────────────────────────────────

    def _resolve_mode(
        self, mode: Optional[Union[str, ArmControlMode]]
    ) -> ArmControlMode:
        """Resolve effective control mode from per-call override or instance default."""
        if mode is None:
            return self._default_mode
        return ArmControlMode(mode) if isinstance(mode, str) else mode

    def _publish_target_pose(
        self,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0,
    ) -> bool:
        """Publish a geometry_msgs/PoseStamped to the custom IK target topic.

        Args:
            x, y, z: Target position in meters.
            qx, qy, qz, qw: Target orientation as quaternion (default: identity).

        Returns:
            True if published successfully, False on error.
        """
        try:
            msg = {
                "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
                "pose": {
                    "position": {
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                    },
                    "orientation": {
                        "x": float(qx),
                        "y": float(qy),
                        "z": float(qz),
                        "w": float(qw),
                    },
                },
            }
            self._transport.publish(self._target_pose_topic, TARGET_POSE_TYPE, msg)
            return True
        except Exception as e:
            print(f"[Arm] Error publishing target pose: {e}")
            return False

    # ── MoveIt action helpers ──────────────────────────────────────────

    # added ros2 action arm manipulator
    def _send_action_goal(
        self,
        action_name: str,
        action_type: str,
        goal_msg: Dict[str, Any],
        blocking: bool,
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> str:
        """
        Helper to handle blocking vs non-blocking action calls.
        """

        # 1. Define the function that performs the actual call
        def perform_call():
            try:
                result = self._transport.call_action(
                    action_name=action_name,
                    action_type=action_type,
                    goal=goal_msg,
                    feedback_callback=feedback_callback,
                    timeout=None,  # Or set a timeout like 10.0
                )
                status = result.get("status", "UNKNOWN")
                return status
            except Exception as e:
                print(f"[Arm] Action {action_name} failed: {e}")
                return "FAILED"

        # 2. Handle Blocking Mode
        if blocking:
            return perform_call()

        # 3. Handle Non-Blocking Mode (Async)
        else:
            thread = threading.Thread(target=perform_call, daemon=True)
            thread.start()
            return "IN_PROGRESS"

    # tested
    def go_to_home(self, group_name: str) -> None:
        """Move the arm to its defined home position."""
        try:
            result = self._transport.call_action(
                action_name="go_to_home",
                action_type=f"{MOVEIT_ACTION_INTERFACE}/GoToHome",
                goal={"group_name": group_name},
            )

            # Check result status
            if result.get("status") == "SUCCEEDED":
                return "SUCCEEDED"
            else:
                return "FAILED"

        except TimeoutError:
            print("timeout error:")
        except Exception as e:
            print(f"Go Home failed: {e}")
            return "FAILED"

    # tested
    """    
    Open: -15.71 rad
    Close: 0.7 rad
    """

    def control_gripper(
        self,
        group_name: str,
        position: float,
        blocking: bool = True,  # NEW ARGUMENT
        feedback_callback: Optional[
            Callable[[Dict[str, Any]], None]
        ] = None,  # NEW ARGUMENT
    ) -> str:
        """
        Open or close the gripper.

        Args:
            open: True to open, False to close.
            callback: Optional callback for action completion.
        """
        goal_msg = {"group_name": group_name, "position": position}

        return self._send_action_goal(
            action_name="control_gripper",
            action_type=f"{MOVEIT_ACTION_INTERFACE}/ControlGripper",
            goal_msg=goal_msg,
            blocking=blocking,
            feedback_callback=feedback_callback,
        )

    def go_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        group_name: str,
        cartesian_path: bool = False,
        blocking: bool = True,
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        mode: Optional[Union[str, ArmControlMode]] = None,
    ) -> str:
        """
        Move the arm to a specific Cartesian pose.

        Routes through MoveIt action or custom IK topic based on the active mode.

        Args:
            x, y, z: Target position in meters.
            roll, pitch, yaw: Target orientation in radians (converted to quaternion for custom IK).
            group_name: MoveIt planning group name (used in MOVEIT mode only).
            cartesian_path: Use Cartesian path planning (MOVEIT mode only).
            blocking: Wait for action to complete (MOVEIT mode only).
            feedback_callback: Action feedback callback (MOVEIT mode only).
            mode: Override control mode for this call. None uses default_mode.
                  Accepts ArmControlMode enum or string ("moveit", "custom_ik").

        Returns:
            Status string: "SUCCEEDED", "FAILED", or "IN_PROGRESS".
        """
        effective_mode = self._resolve_mode(mode)

        if effective_mode == ArmControlMode.CUSTOM_IK:
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
            success = self._publish_target_pose(x, y, z, qx, qy, qz, qw)
            return "SUCCEEDED" if success else "FAILED"

        # MoveIt mode (default)
        goal_msg = {
            "group_name": group_name,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "cartesian_path": cartesian_path,
        }

        return self._send_action_goal(
            action_name="go_to_pose",
            action_type=f"{MOVEIT_ACTION_INTERFACE}/GoToPose",
            goal_msg=goal_msg,
            blocking=blocking,
            feedback_callback=feedback_callback,
        )

    # tested
    def go_to_pose_relative(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        group_name: str,
        cartesian_path: bool = False,
        blocking: bool = True,  # NEW ARGUMENT
        feedback_callback: Optional[
            Callable[[Dict[str, Any]], None]
        ] = None,  # NEW ARGUMENT
    ) -> None:
        """
        Move the arm to a specific Cartesian pose using an absolute coordinate action.
        """
        goal_msg = {
            "group_name": group_name,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "cartesian_path": cartesian_path,
        }

        # Assuming the action name is 'go_to_pose'
        return self._send_action_goal(
            action_name="go_to_pose_relative",
            action_type=f"{MOVEIT_ACTION_INTERFACE}/GoToPoseRelative",
            goal_msg=goal_msg,
            blocking=blocking,
            feedback_callback=feedback_callback,
        )
    def go_to_pose_quaternion(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        group_name: str,
        cartesian_path: bool = False,
        blocking: bool = True,
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        mode: Optional[Union[str, ArmControlMode]] = None,
    ) -> str:
        """
        Move the arm to a specific Cartesian pose using quaternion orientation data.

        Args:
            x, y, z: Target position in meters.
            qx, qy, qz, qw: Target orientation as a quaternion.
            group_name: MoveIt planning group name (e.g., "left_arm").
            cartesian_path: If true, compute a linear path in Cartesian space.
            blocking: If true, wait for the action to finish before returning.
            feedback_callback: Optional callback for action feedback.
            mode: Control mode override ("moveit" or "custom_ik").

        Returns:
            Status string: "SUCCEEDED", "FAILED", or "IN_PROGRESS".
        """
        effective_mode = self._resolve_mode(mode)

        if effective_mode == ArmControlMode.CUSTOM_IK:
            # For custom IK topic, we just publish the raw quaternion directly
            success = self._publish_target_pose(x, y, z, qx, qy, qz, qw)
            return "SUCCEEDED" if success else "FAILED"

        # MoveIt mode (Action Server)
        # The keys here must match your .action file Goal fields exactly
        goal_msg = {
            "group_name": group_name,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "qx": float(qx),
            "qy": float(qy),
            "qz": float(qz),
            "qw": float(qw),
            "cartesian_path": cartesian_path,
        }

        # Uses the GoToPoseQuaternion action type defined in your robot interfaces
        return self._send_action_goal(
            action_name="go_to_pose_quat",
            action_type=f"{MOVEIT_ACTION_INTERFACE}/GoToPoseQuaternion",
            goal_msg=goal_msg,
            blocking=blocking,
            feedback_callback=feedback_callback,
        )

    def go_to_pose_quaternion_move_action(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        group_name: str,
        link_name: str = "left_link7",
        frame_id: str = "base_footprint",
        allowed_planning_time: float = 10.0,
        blocking: bool = True,
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        mode: Optional[Union[str, ArmControlMode]] = None,
    ) -> str:
        """
        Move the arm to a specific Cartesian pose using Quaternion orientation.

        Routes through MoveGroup action server or custom IK topic based on mode.

        Args:
            x, y, z: Target position in meters.
            qx, qy, qz, qw: Target orientation as quaternion.
            group_name: MoveIt planning group name (MOVEIT mode only).
            link_name: End-effector link name (MOVEIT mode only).
            frame_id: Reference frame (MOVEIT mode only).
            allowed_planning_time: Max planning time in seconds (MOVEIT mode only).
            blocking: Wait for action to complete (MOVEIT mode only).
            feedback_callback: Action feedback callback (MOVEIT mode only).
            mode: Override control mode for this call. None uses default_mode.
                  Accepts ArmControlMode enum or string ("moveit", "custom_ik").

        Returns:
            Status string: "SUCCEEDED", "FAILED", or "IN_PROGRESS".
        """
        effective_mode = self._resolve_mode(mode)

        if effective_mode == ArmControlMode.CUSTOM_IK:
            success = self._publish_target_pose(x, y, z, qx, qy, qz, qw)
            return "SUCCEEDED" if success else "FAILED"

        # MoveIt mode (default) - MoveGroup action server
        goal_msg = {
            "request": {
                "group_name": group_name,
                "goal_constraints": [
                    {
                        "position_constraints": [
                            {
                                "header": {"frame_id": frame_id},
                                "link_name": link_name,
                                "constraint_region": {
                                    "primitives": [
                                        {"type": 1, "dimensions": [0.1, 0.1, 0.1]}
                                    ],
                                    "primitive_poses": [
                                        {
                                            "position": {
                                                "x": float(x),
                                                "y": float(y),
                                                "z": float(z),
                                            }
                                        }
                                    ],
                                },
                                "weight": 1.0,
                            }
                        ],
                        "orientation_constraints": [
                            {
                                "header": {"frame_id": frame_id},
                                "link_name": link_name,
                                "orientation": {
                                    "x": float(qx),
                                    "y": float(qy),
                                    "z": float(qz),
                                    "w": float(qw),
                                },
                                "absolute_x_axis_tolerance": 0.1,
                                "absolute_y_axis_tolerance": 0.1,
                                "absolute_z_axis_tolerance": 0.1,
                                "weight": 1.0,
                            }
                        ],
                    }
                ],
                "allowed_planning_time": float(allowed_planning_time),
                "max_velocity_scaling_factor": 0.1,
                "max_acceleration_scaling_factor": 0.1,
            }
        }

        return self._send_action_goal(
            action_name="move_action",
            action_type="moveit_msgs/action/MoveGroup",
            goal_msg=goal_msg,
            blocking=blocking,
            feedback_callback=feedback_callback,
        )

    """
    got o pose qurtanion

    ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{

    request: {

        group_name: 'left_arm',

        goal_constraints: [{

        position_constraints: [{

            header: {frame_id: 'base_footprint'},

            link_name: 'left_link7',

            constraint_region: {

            primitives: [{type: 1, dimensions: [0.1, 0.1, 0.1]}],

            primitive_poses: [{position: {x: 0.38, y: 0.19, z: 0.58}}]

            },

            weight: 1.0

        }],

        orientation_constraints: [{

            header: {frame_id: 'base_footprint'},

            link_name: 'left_link7',

            orientation: {x: -0.5, y: -0.5, z: 0.5, w: 0.5},

            absolute_x_axis_tolerance: 0.1,

            absolute_y_axis_tolerance: 0.1,

            absolute_z_axis_tolerance: 0.1,

            weight: 1.0

        }]

        }],

        allowed_planning_time: 10.0,

        max_velocity_scaling_factor: 0.1,

        max_acceleration_scaling_factor: 0.1

    }

    }"
    """
