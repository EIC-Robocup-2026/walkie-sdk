"""
Arm - Robot arm control module.

Provides set_joint_positions(), set_joint_velocities(), set_joint_torques(),
and get_joint_states() functions for controlling robot's arms.

This module uses ROSTransportInterface abstraction, allowing it
to work with any transport implementation (rosbridge, zenoh).
"""

import threading
from typing import Any, Callable, Dict, List, Optional, Tuple

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace

# Default arm topic names (without namespace)
DEFAULT_ARM_COMMANDS_TOPIC = "walkie/arm/commands"
DEFAULT_ARM_STATES_TOPIC = "/joint_states"
ARM_COMMANDS_TYPE = "sensor_msgs/msg/JointState"
ARM_STATES_TYPE = "sensor_msgs/msg/JointState"

MOVEIT_ACTION_INTERFACE = "my_robot_interfaces/action"


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

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace
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

    #tested
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
            

    
    #added ros2 action arm manipulator
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
                    timeout=None # Or set a timeout like 10.0
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
        
    #tested
    def go_to_home(self,group_name: str) -> None:
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
    #tested
    """    
    Open: -15.71 rad
    Close: 0.7 rad
    """
    def control_gripper(
            self,group_name: str,
            position: float,
            blocking: bool = True, # NEW ARGUMENT
            feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None # NEW ARGUMENT
              ) -> None:
        """
        Open or close the gripper.
        
        Args:
            open: True to open, False to close.
            callback: Optional callback for action completion.
        """
        goal_msg = {
            "group_name": group_name, 
            "position": position
        }

        return self._send_action_goal(
            action_name="control_gripper",
            action_type=f"{MOVEIT_ACTION_INTERFACE}/ControlGripper",
            goal_msg=goal_msg,
            blocking=blocking,
            feedback_callback=feedback_callback
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
        blocking: bool = True, # NEW ARGUMENT
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None # NEW ARGUMENT
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
            "cartesian_path": cartesian_path
        }
        
        # Assuming the action name is 'go_to_pose'
        return self._send_action_goal(
            action_name="go_to_pose",
            action_type=f"{MOVEIT_ACTION_INTERFACE}/GoToPose",
            goal_msg=goal_msg,
            blocking=blocking,
            feedback_callback=feedback_callback
        )
        

    #tested
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
        blocking: bool = True, # NEW ARGUMENT
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None # NEW ARGUMENT
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
            "cartesian_path": cartesian_path
        }
        
        # Assuming the action name is 'go_to_pose'
        return self._send_action_goal(
            action_name="go_to_pose_relative",
            action_type=f"{MOVEIT_ACTION_INTERFACE}/GoToPoseRelative",
            goal_msg=goal_msg,
            blocking=blocking,
            feedback_callback=feedback_callback
        )