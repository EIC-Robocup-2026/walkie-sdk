"""
Arm - Robot arm control module.

Maps onto the openarm_bimanual_commander_cpp action/service interfaces:

Actions (via transport.call_action):
  go_to_pose, go_to_pose_quat, go_to_pose_relative,
  go_to_home, control_gripper, set_joint_position

Services (via transport.call_service):
  get_ee_pose, get_joint_states  (→ get_joint_states_service)

Direct JTC streaming (via transport.publish):
  set_joint_position(..., mode="jtc") publishes trajectory_msgs/msg/JointTrajectory
"""

import threading
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional

from walkie_sdk.config.ros_topics import ARM_ACTIONS, ARM_SERVICES, ARM_TOPICS
from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace

if TYPE_CHECKING:
    from walkie_sdk.modules.joint_state_hub import JointStateHub

# Joint name mapping for JTC direct streaming.
# Lengths must match the group DOF expected by set_joint_position.
_LEFT_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
_RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]

_JTC_JOINT_NAMES: Dict[str, List[str]] = {
    "left_arm": _LEFT_JOINTS,
    "right_arm": _RIGHT_JOINTS,
    "left_arm_lift": _LEFT_JOINTS + ["lift_joint"],
    "right_arm_lift": _RIGHT_JOINTS + ["lift_joint"],
    "both_arms": _LEFT_JOINTS + _RIGHT_JOINTS,
    "both_arms_lift": _LEFT_JOINTS + _RIGHT_JOINTS + ["lift_joint"],
}

_RIGHT_GROUPS = {"right_arm", "right_arm_lift"}
_BOTH_GROUPS = {"both_arms", "both_arms_lift"}

_GRIPPER_MAX_M = 0.04  # fully open, metres


class ArmGroup:
    """Pre-selects a MoveIt group so callers don't repeat group_name on every call."""

    def __init__(self, arm: "Arm", group_name: str, gripper_name: str = None):
        self._arm = arm
        self._group_name = group_name
        self._gripper_name = gripper_name

    @property
    def group_name(self) -> str:
        return self._group_name

    def go_to_pose(self, pos, rot, **kwargs) -> str:
        x, y, z = pos
        roll, pitch, yaw = rot
        return self._arm.go_to_pose(
            x, y, z, roll, pitch, yaw, group_name=self._group_name, **kwargs
        )

    def go_to_pose_quat(self, pos, rot, **kwargs) -> str:
        x, y, z = pos
        qx, qy, qz, qw = rot
        return self._arm.go_to_pose_quat(
            x, y, z, qx, qy, qz, qw, group_name=self._group_name, **kwargs
        )

    def go_to_pose_relative(self, pos, rot, **kwargs) -> str:
        x, y, z = pos
        roll, pitch, yaw = rot
        return self._arm.go_to_pose_relative(
            x, y, z, roll, pitch, yaw, group_name=self._group_name, **kwargs
        )

    def go_to_home(self, **kwargs) -> str:
        return self._arm.go_to_home(group_name=self._group_name, **kwargs)

    def set_joint_position(self, joint_positions, **kwargs) -> str:
        return self._arm.set_joint_position(self._group_name, joint_positions, **kwargs)

    def get_ee_pose(self, **kwargs):
        return self._arm.get_ee_pose(self._group_name, **kwargs)

    def get_joint_states_service(self, **kwargs):
        return self._arm.get_joint_states_service(self._group_name, **kwargs)

    def gripper(self, value: float, norm: bool = True, **kwargs) -> str:
        """
        Control the gripper.

        Args:
            value: 0.0–1.0 normalized (default) or raw meters when norm=False.
                   0.0 = fully closed, 1.0 = fully open (0.04 m).
            norm: If True (default), treat value as normalized [0, 1].
        """
        if self._gripper_name is None:
            raise ValueError(f"Group '{self._group_name}' has no associated gripper")
        meters = float(value) * _GRIPPER_MAX_M if norm else float(value)
        return self._arm.control_gripper(self._gripper_name, meters, **kwargs)


class Arm:
    """
    Robot arm controller wrapping openarm_bimanual_commander_cpp interfaces.

    Args:
        transport: Transport instance implementing ROSTransportInterface
        namespace: ROS namespace prefix for topics/actions (default: "" = no namespace)
        joint_state_hub: Shared hub that owns the joint_states subscription.
    """

    def __init__(
        self,
        transport: ROSTransportInterface,
        namespace: str = "",
        joint_state_hub: "JointStateHub" = None,
    ):
        self._transport = transport
        self._namespace = namespace
        self._joint_state_hub = joint_state_hub

        self.left = ArmGroup(self, "left_arm_lift", gripper_name="left_gripper")
        self.right = ArmGroup(self, "right_arm_lift", gripper_name="right_gripper")

    # ── Properties ────────────────────────────────────────────────────────

    @property
    def namespace(self) -> str:
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        self._namespace = value

    # ── Internal helpers ──────────────────────────────────────────────────

    def _send_action_goal(
        self,
        action_name: str,
        action_type: str,
        goal_msg: Dict[str, Any],
        blocking: bool,
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> str:
        def _call() -> str:
            try:
                result = self._transport.call_action(
                    action_name=apply_namespace(action_name, self._namespace),
                    action_type=action_type,
                    goal=goal_msg,
                    feedback_callback=feedback_callback,
                    timeout=None,
                )
                return result.get("status", "UNKNOWN")
            except Exception as e:
                print(f"[Arm] Action '{action_name}' failed: {e}")
                return "FAILED"

        if blocking:
            return _call()
        threading.Thread(target=_call, daemon=True).start()
        return "IN_PROGRESS"

    def _jtc_publish(
        self, group_name: str, joint_positions: List[float], duration: float
    ) -> str:
        """Publish a single-point JointTrajectory to the arm JTC controller(s).

        Always sends exactly 7 joints per arm — lift_joint is never included
        because left/right_joint_trajectory_controller only know 7 joints and
        have allow_partial_joints_goal=false.
        """
        sec = int(duration)
        nanosec = int((duration - sec) * 1e9)

        def _publish(topic_key: str, names: List[str], positions: List[float]) -> None:
            self._transport.publish(
                apply_namespace(ARM_TOPICS[topic_key], self._namespace),
                ARM_TOPICS["jtc_type"],
                {
                    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
                    "joint_names": names,
                    "points": [{
                        "positions": positions,
                        "velocities": [],
                        "accelerations": [],
                        "time_from_start": {"sec": sec, "nanosec": nanosec},
                    }],
                },
            )

        positions = [float(p) for p in joint_positions]

        if group_name in _BOTH_GROUPS:
            _publish("jtc_left",  _LEFT_JOINTS, positions[:len(_LEFT_JOINTS)])
            _publish("jtc_right", _RIGHT_JOINTS, positions[len(_LEFT_JOINTS):len(_LEFT_JOINTS) + len(_RIGHT_JOINTS)])
        elif group_name in ("left_arm", "left_arm_lift"):
            _publish("jtc_left", _LEFT_JOINTS, positions[:len(_LEFT_JOINTS)])
        elif group_name in ("right_arm", "right_arm_lift"):
            _publish("jtc_right", _RIGHT_JOINTS, positions[:len(_RIGHT_JOINTS)])
        else:
            print(f"[Arm] Unknown group '{group_name}' for JTC mode.")
            return "FAILED"

        return "SUCCEEDED"


    # ── Action methods ────────────────────────────────────────────────────

    def go_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        group_name: str,
        frame_id: str = "base_footprint",
        cartesian_path: bool = False,
        blocking: bool = True,
        feedback_callback: Optional[Callable] = None,
    ) -> str:
        """
        Move arm to an absolute Cartesian pose using Euler RPY orientation.

        Args:
            x, y, z: Target position in meters.
            roll, pitch, yaw: Target orientation in radians (RPY).
            group_name: MoveIt group (e.g. "left_arm", "right_arm").
            frame_id: Reference frame (default: "base_footprint").
            cartesian_path: Plan a straight-line Cartesian path.
            blocking: Wait for action to complete.
            feedback_callback: Optional callback for action feedback.

        Returns:
            "SUCCEEDED" | "FAILED" | "IN_PROGRESS"
        """
        goal = {
            "group_name": group_name,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "frame_id": frame_id,
            "cartesian_path": cartesian_path,
        }
        return self._send_action_goal(
            ARM_ACTIONS["go_to_pose"],
            f"{ARM_ACTIONS['interface']}/GoToPose",
            goal,
            blocking,
            feedback_callback,
        )

    def go_to_pose_quat(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        group_name: str,
        frame_id: str = "base_footprint",
        cartesian_path: bool = False,
        blocking: bool = True,
        feedback_callback: Optional[Callable] = None,
    ) -> str:
        """
        Move arm to an absolute Cartesian pose using quaternion orientation.

        Args:
            x, y, z: Target position in meters.
            qx, qy, qz, qw: Target orientation as quaternion.
            group_name: MoveIt group.
            frame_id: Reference frame (default: "base_footprint").
            cartesian_path: Plan a straight-line Cartesian path.
            blocking: Wait for action to complete.
            feedback_callback: Optional callback for action feedback.

        Returns:
            "SUCCEEDED" | "FAILED" | "IN_PROGRESS"
        """
        goal = {
            "group_name": group_name,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "qx": float(qx),
            "qy": float(qy),
            "qz": float(qz),
            "qw": float(qw),
            "frame_id": frame_id,
            "cartesian_path": cartesian_path,
        }
        return self._send_action_goal(
            ARM_ACTIONS["go_to_pose_quat"],
            f"{ARM_ACTIONS['interface']}/GoToPoseQuaternion",
            goal,
            blocking,
            feedback_callback,
        )

    def go_to_pose_relative(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        group_name: str,
        frame_id: str = "base_footprint",
        cartesian_path: bool = False,
        ee_frame: bool = False,
        blocking: bool = True,
        feedback_callback: Optional[Callable] = None,
    ) -> str:
        """
        Move arm by a relative Cartesian displacement.

        Args:
            x, y, z: Positional offset in meters.
            roll, pitch, yaw: Orientation delta in radians.
            group_name: MoveIt group.
            frame_id: Reference frame for the offset (default: "base_footprint").
            cartesian_path: Plan a straight-line Cartesian path.
            ee_frame: If True, offset is in EEF-local axes; if False, in frame_id world axes.
            blocking: Wait for action to complete.
            feedback_callback: Optional callback for action feedback.

        Returns:
            "SUCCEEDED" | "FAILED" | "IN_PROGRESS"
        """
        goal = {
            "group_name": group_name,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "frame_id": frame_id,
            "cartesian_path": cartesian_path,
            "ee_frame": ee_frame,
        }
        return self._send_action_goal(
            ARM_ACTIONS["go_to_pose_relative"],
            f"{ARM_ACTIONS['interface']}/GoToPoseRelative",
            goal,
            blocking,
            feedback_callback,
        )

    def go_to_home(
        self,
        group_name: str,
        blocking: bool = True,
        feedback_callback: Optional[Callable] = None,
    ) -> str:
        """
        Move arm to its named home position.

        Args:
            group_name: MoveIt group (arm groups only, not gripper).
            blocking: Wait for action to complete.
            feedback_callback: Optional callback for action feedback.

        Returns:
            "SUCCEEDED" | "FAILED" | "IN_PROGRESS"
        """
        goal = {"group_name": group_name}
        return self._send_action_goal(
            ARM_ACTIONS["go_to_home"],
            f"{ARM_ACTIONS['interface']}/GoToHome",
            goal,
            blocking,
            feedback_callback,
        )

    def control_gripper(
        self,
        group_name: str,
        position: float,
        blocking: bool = True,
        feedback_callback: Optional[Callable] = None,
    ) -> str:
        """
        Open or close a gripper.

        Args:
            group_name: "left_gripper" or "right_gripper".
            position: Gripper position in meters (0.0 = closed, 0.04 = fully open).
            blocking: Wait for action to complete.
            feedback_callback: Optional callback for action feedback.

        Returns:
            "SUCCEEDED" | "FAILED" | "IN_PROGRESS"
        """
        goal = {"group_name": group_name, "position": float(position)}
        return self._send_action_goal(
            ARM_ACTIONS["control_gripper"],
            f"{ARM_ACTIONS['interface']}/ControlGripper",
            goal,
            blocking,
            feedback_callback,
        )

    def set_joint_position(
        self,
        group_name: str,
        joint_positions: List[float],
        mode: str = "commander",
        duration: float = 1.0,
        blocking: bool = True,
        feedback_callback: Optional[Callable] = None,
    ) -> str:
        """
        Move arm joints to target positions.

        Args:
            group_name: MoveIt group name (e.g. "left_arm", "right_arm", "both_arms").
            joint_positions: Target joint angles in radians. Length must match group DOF.
            mode: "commander" (MoveIt action, collision-checked) or
                  "jtc" (direct JointTrajectory publish, no collision check, high-rate capable).
            duration: Trajectory duration in seconds — JTC mode only.
            blocking: Wait for action to complete — commander mode only.
            feedback_callback: Action feedback callback — commander mode only.

        Returns:
            "SUCCEEDED" | "FAILED" | "IN_PROGRESS"

        Note:
            JTC mode always returns "SUCCEEDED" immediately (fire-and-forget).
            Commander mode is collision-checked but limited to ~0.4 Hz.
            For high-rate streaming (VLA, servo), use JTC mode.
        """
        if mode == "jtc":
            return self._jtc_publish(group_name, joint_positions, duration)

        goal = {
            "group_name": group_name,
            "joint_positions": [float(p) for p in joint_positions],
        }
        return self._send_action_goal(
            ARM_ACTIONS["set_joint_position"],
            f"{ARM_ACTIONS['interface']}/SetJointPosition",
            goal,
            blocking,
            feedback_callback,
        )

    # ── Service methods ───────────────────────────────────────────────────

    def get_ee_pose(
        self,
        group_name: str,
        frame_id: str = "base_footprint",
        timeout: float = 5.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Query the current end-effector pose from the commander.

        Args:
            group_name: MoveIt group (e.g. "left_arm", "right_arm").
            frame_id: Reference frame for the returned pose (default: "base_footprint").
            timeout: Service call timeout in seconds.

        Returns:
            Dict with keys "x", "y", "z", "qx", "qy", "qz", "qw", "frame_id", or None on failure.
        """
        try:
            response = self._transport.call_service(
                service_name=apply_namespace(
                    ARM_SERVICES["get_ee_pose"], self._namespace
                ),
                service_type=ARM_SERVICES["get_ee_pose_type"],
                request={"group_name": group_name, "frame_id": frame_id},
                timeout=timeout,
            )
            if not response.get("success", False):
                print(f"[Arm] get_ee_pose failed: {response.get('status', 'unknown')}")
                return None
            return {
                "x": response["x"],
                "y": response["y"],
                "z": response["z"],
                "qx": response["qx"],
                "qy": response["qy"],
                "qz": response["qz"],
                "qw": response["qw"],
                "frame_id": response.get("frame_id", frame_id),
            }
        except TimeoutError:
            print(f"[Arm] get_ee_pose timed out after {timeout}s")
            return None
        except Exception as e:
            print(f"[Arm] get_ee_pose error: {e}")
            return None

    def get_joint_states_service(
        self,
        group_name: str,
        timeout: float = 5.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Query current joint positions for a group via service (one-shot RPC).

        Unlike get_joint_states() which reads from the continuous topic subscription,
        this makes a synchronous service call and returns the current snapshot for
        the specified group.

        Args:
            group_name: MoveIt group (e.g. "left_arm", "right_arm").
            timeout: Service call timeout in seconds.

        Returns:
            Dict with "joint_names" and "joint_positions" lists, or None on failure.
        """
        try:
            response = self._transport.call_service(
                service_name=apply_namespace(
                    ARM_SERVICES["get_joint_states"], self._namespace
                ),
                service_type=ARM_SERVICES["get_joint_states_type"],
                request={"group_name": group_name},
                timeout=timeout,
            )
            if not response.get("success", False):
                print(
                    f"[Arm] get_joint_states_service failed: {response.get('status', 'unknown')}"
                )
                return None
            return {
                "joint_names": response.get("joint_names", []),
                "joint_positions": response.get("joint_positions", []),
            }
        except TimeoutError:
            print(f"[Arm] get_joint_states_service timed out after {timeout}s")
            return None
        except Exception as e:
            print(f"[Arm] get_joint_states_service error: {e}")
            return None

    # ── Topic-based state read (continuous subscription) ──────────────────

    def get_joint_states(self) -> Optional[Dict[str, Any]]:
        """
        Get latest joint states from the shared JointStateHub.

        Returns the most recently received data parsed into left_arm, right_arm,
        and gripper fields. Returns None if the hub has no data yet.

        For a one-shot per-group query, use get_joint_states_service() instead.

        Returns:
            Dict with "left_arm", "right_arm", "left_gripper", "right_gripper", or None.
        """
        if self._joint_state_hub is None:
            return None

        all_joints = self._joint_state_hub.get_all()
        if not all_joints:
            return None

        try:
            left_pos, left_vel, left_torque = [], [], []
            right_pos, right_vel, right_torque = [], [], []
            left_gripper = None
            right_gripper = None

            for name, data in all_joints.items():
                pos    = data["position"]
                vel    = data["velocity"]
                effort = data["effort"]

                if name.startswith("openarm_left_joint") or name.startswith("left_joint"):
                    idx = int(name.split("joint")[-1]) - 1
                    if idx < 7:
                        while len(left_pos) <= idx:
                            left_pos.append(0.0)
                            left_vel.append(0.0)
                            left_torque.append(0.0)
                        left_pos[idx]    = pos
                        left_vel[idx]    = vel
                        left_torque[idx] = effort
                elif name.startswith("openarm_right_joint") or name.startswith("right_joint"):
                    idx = int(name.split("joint")[-1]) - 1
                    if idx < 7:
                        while len(right_pos) <= idx:
                            right_pos.append(0.0)
                            right_vel.append(0.0)
                            right_torque.append(0.0)
                        right_pos[idx]    = pos
                        right_vel[idx]    = vel
                        right_torque[idx] = effort
                elif "left_gripper" in name or "left_finger" in name:
                    left_gripper = pos
                elif "right_gripper" in name or "right_finger" in name:
                    right_gripper = pos

            return {
                "left_arm": {
                    "positions":  left_pos[:7],
                    "velocities": left_vel[:7],
                    "torques":    left_torque[:7],
                },
                "right_arm": {
                    "positions":  right_pos[:7],
                    "velocities": right_vel[:7],
                    "torques":    right_torque[:7],
                },
                "left_gripper":  left_gripper,
                "right_gripper": right_gripper,
            }
        except Exception as e:
            print(f"[Arm] Error parsing joint states: {e}")
            return None
