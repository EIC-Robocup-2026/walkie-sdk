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

from walkie_sdk.config.ros_topics import (
    ARM_ACTIONS,
    ARM_PARAMS,
    ARM_SERVICES,
    ARM_TOPICS,
)
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


# rcl_interfaces/msg/ParameterType enum values.
_PT_NOT_SET = 0
_PT_BOOL = 1
_PT_INTEGER = 2
_PT_DOUBLE = 3
_PT_STRING = 4
_PT_BOOL_ARRAY = 6
_PT_INTEGER_ARRAY = 7
_PT_DOUBLE_ARRAY = 8
_PT_STRING_ARRAY = 9


def _empty_param_value() -> Dict[str, Any]:
    """A fully-populated rcl_interfaces/msg/ParameterValue with all fields at
    their defaults — rosbridge/zenoh expect every field present."""
    return {
        "type": _PT_NOT_SET,
        "bool_value": False,
        "integer_value": 0,
        "double_value": 0.0,
        "string_value": "",
        "byte_array_value": [],
        "bool_array_value": [],
        "integer_array_value": [],
        "double_array_value": [],
        "string_array_value": [],
    }


def _to_param_value(value: Any) -> Dict[str, Any]:
    """Build a rcl_interfaces/msg/ParameterValue dict from a Python value,
    inferring the ROS parameter type. bool is checked before int (bool is an
    int subclass in Python)."""
    pv = _empty_param_value()
    if isinstance(value, bool):
        pv["type"] = _PT_BOOL
        pv["bool_value"] = value
    elif isinstance(value, int):
        pv["type"] = _PT_INTEGER
        pv["integer_value"] = value
    elif isinstance(value, float):
        pv["type"] = _PT_DOUBLE
        pv["double_value"] = value
    elif isinstance(value, str):
        pv["type"] = _PT_STRING
        pv["string_value"] = value
    elif isinstance(value, (list, tuple)):
        seq = list(value)
        if all(isinstance(v, bool) for v in seq):
            pv["type"] = _PT_BOOL_ARRAY
            pv["bool_array_value"] = seq
        elif all(isinstance(v, int) and not isinstance(v, bool) for v in seq):
            pv["type"] = _PT_INTEGER_ARRAY
            pv["integer_array_value"] = seq
        elif all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in seq):
            pv["type"] = _PT_DOUBLE_ARRAY
            pv["double_array_value"] = [float(v) for v in seq]
        else:
            pv["type"] = _PT_STRING_ARRAY
            pv["string_array_value"] = [str(v) for v in seq]
    else:
        raise TypeError(f"Unsupported parameter value type: {type(value)}")
    return pv


def _to_param_value_typed(value: Any, declared_type: int) -> Dict[str, Any]:
    """Build a ParameterValue coerced to the param's already-declared ROS type.

    ROS 2 rejects a set when the value's type doesn't match the declared type
    (e.g. an int literal for a double param), so when we know the declared type
    we coerce instead of inferring. declared_type == PARAMETER_NOT_SET (the param
    doesn't exist yet) falls back to inference."""
    if declared_type == _PT_NOT_SET:
        return _to_param_value(value)
    pv = _empty_param_value()
    pv["type"] = declared_type
    try:
        if declared_type == _PT_BOOL:
            pv["bool_value"] = bool(value)
        elif declared_type == _PT_INTEGER:
            pv["integer_value"] = int(value)
        elif declared_type == _PT_DOUBLE:
            pv["double_value"] = float(value)
        elif declared_type == _PT_STRING:
            pv["string_value"] = str(value)
        elif declared_type == _PT_BOOL_ARRAY:
            pv["bool_array_value"] = [bool(v) for v in value]
        elif declared_type == _PT_INTEGER_ARRAY:
            pv["integer_array_value"] = [int(v) for v in value]
        elif declared_type == _PT_DOUBLE_ARRAY:
            pv["double_array_value"] = [float(v) for v in value]
        elif declared_type == _PT_STRING_ARRAY:
            pv["string_array_value"] = [str(v) for v in value]
        else:
            return _to_param_value(value)
    except (TypeError, ValueError):
        # Value can't be coerced to the declared type — let the inferred type go
        # through so the commander returns a clear type-mismatch reason.
        return _to_param_value(value)
    return pv


def _from_param_value(pv: Dict[str, Any]) -> Any:
    """Extract the Python value from a rcl_interfaces/msg/ParameterValue dict.
    Returns None if the parameter is not set (i.e. doesn't exist)."""
    t = pv.get("type", _PT_NOT_SET)
    return {
        _PT_BOOL: pv.get("bool_value"),
        _PT_INTEGER: pv.get("integer_value"),
        _PT_DOUBLE: pv.get("double_value"),
        _PT_STRING: pv.get("string_value"),
        _PT_BOOL_ARRAY: pv.get("bool_array_value"),
        _PT_INTEGER_ARRAY: pv.get("integer_array_value"),
        _PT_DOUBLE_ARRAY: pv.get("double_array_value"),
        _PT_STRING_ARRAY: pv.get("string_array_value"),
    }.get(t, None)


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

    def clear_collision_objects(self, timeout: float = 5.0) -> bool:
        """
        Detach anything held by the grippers and remove all world collision
        objects from the MoveIt planning scene (the octomap is left untouched).

        Returns True if the commander reported success.
        """
        try:
            response = self._transport.call_service(
                service_name=apply_namespace(
                    ARM_SERVICES["clear_objects"], self._namespace
                ),
                service_type=ARM_SERVICES["clear_objects_type"],
                request={},
                timeout=timeout,
            )
            ok = bool(response.get("success", False))
            msg = response.get("message", "")
            if ok:
                print(f"[Arm] clear_collision_objects: {msg}")
            else:
                print(f"[Arm] clear_collision_objects failed: {msg}")
            return ok
        except Exception as e:
            print(f"[Arm] clear_collision_objects error: {e}")
            return False

    def clear_octomap(self, timeout: float = 5.0) -> bool:
        """
        Clear the MoveIt octomap (the sensed point-cloud/depth-camera obstacles
        that block planning) via move_group's /clear_octomap service. Leaves
        collision objects and robot state untouched.

        Returns True if the call succeeded.
        """
        try:
            self._transport.call_service(
                service_name=apply_namespace(
                    ARM_SERVICES["clear_octomap"], self._namespace
                ),
                service_type=ARM_SERVICES["clear_octomap_type"],
                request={},
                timeout=timeout,
            )
            print("[Arm] clear_octomap: cleared")
            return True
        except Exception as e:
            print(f"[Arm] clear_octomap error: {e}")
            return False

    def toggle_gripper_collision(
        self, group_name: str, enable: bool, timeout: float = 5.0
    ) -> bool:
        """
        Enable or disable collision checking for a gripper's links against the
        rest of the world (octomap, scene objects, other robot links).

        Args:
            group_name: "left_gripper" or "right_gripper".
            enable: True  = collision checked (normal);
                    False = gripper links ignored vs the world.
            timeout: Service call timeout in seconds.

        Returns True if the commander accepted the change.
        """
        try:
            response = self._transport.call_service(
                service_name=apply_namespace(
                    ARM_SERVICES["toggle_collision"], self._namespace
                ),
                service_type=ARM_SERVICES["toggle_collision_type"],
                request={"group_name": group_name, "enable": bool(enable)},
                timeout=timeout,
            )
            ok = bool(response.get("success", False))
            print(f"[Arm] toggle_gripper_collision: {response.get('status', '')}")
            return ok
        except Exception as e:
            print(f"[Arm] toggle_gripper_collision error: {e}")
            return False

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

    # ── Commander parameters (live tuning via ROS 2 param services) ───────

    def _param_service(self, key: str) -> str:
        """Absolute name of one of the commander node's parameter services,
        e.g. '/bimanual_commander/get_parameters'.

        ROS 2 advertises the per-node parameter services under the node's
        FULLY-QUALIFIED name, so this is built with a leading slash — unlike the
        commander's custom services (get_ee_pose, ...) which live at the root.
        A namespace, if set, is inserted ahead of the node name."""
        node = str(ARM_PARAMS["node"]).strip("/")
        ns = self._namespace.strip("/")
        parts = [p for p in (ns, node, ARM_PARAMS[key]) if p]
        return "/" + "/".join(parts)

    def _declared_types(
        self, names: List[str], timeout: float = 5.0
    ) -> Dict[str, int]:
        """Map each name to its already-declared ROS ParameterType (0 =
        PARAMETER_NOT_SET / unknown). Used to coerce set values to the right
        type so the commander doesn't reject e.g. an int for a double param."""
        try:
            resp = self._transport.call_service(
                service_name=self._param_service("get"),
                service_type=ARM_PARAMS["get_type"],
                request={"names": list(names)},
                timeout=timeout,
            )
            values = resp.get("values", [])
            return {n: int(v.get("type", 0)) for n, v in zip(names, values)}
        except Exception:
            return {n: 0 for n in names}

    def set_param_result(
        self, name: str, value: Any, timeout: float = 5.0
    ) -> Dict[str, Any]:
        """Set one commander param and return {"ok": bool, "reason": str}.

        The value is coerced to the param's already-declared type, so passing a
        bare int (e.g. 1) for a double param like gripper_speed works instead of
        being rejected for a type mismatch."""
        declared = self._declared_types([name], timeout).get(name, 0)
        pv = _to_param_value_typed(value, declared)
        try:
            response = self._transport.call_service(
                service_name=self._param_service("set"),
                service_type=ARM_PARAMS["set_type"],
                request={"parameters": [{"name": name, "value": pv}]},
                timeout=timeout,
            )
            results = response.get("results", [])
            if not results:
                return {"ok": False, "reason": "no result from set_parameters"}
            ok = bool(results[0].get("successful", False))
            reason = results[0].get("reason", "") or ("" if ok else "rejected")
            if not ok:
                print(f"[Arm] set_param('{name}') rejected: {reason}")
            return {"ok": ok, "reason": reason}
        except Exception as e:
            print(f"[Arm] set_param('{name}') error: {e}")
            return {"ok": False, "reason": str(e)}

    def set_param(self, name: str, value: Any, timeout: float = 5.0) -> bool:
        """
        Set a commander ROS parameter live (e.g. gripper_speed, planner_id,
        arm_planning_time, grasp_object_size, finger_padding, ...).

        The value is coerced to the param's already-declared type, so a bare int
        works for a double param. Returns True if the commander accepted it; use
        set_param_result() to also get the rejection reason.

        Args:
            name: Parameter name as declared by the commander.
            value: New value.
            timeout: Service call timeout in seconds.

        Returns:
            True if the commander accepted the change.
        """
        return self.set_param_result(name, value, timeout)["ok"]

    def set_params(self, params: Dict[str, Any], timeout: float = 5.0) -> bool:
        """Set several commander parameters in one atomic call. Returns True
        only if every parameter was accepted. Each value is coerced to the
        param's already-declared type."""
        declared = self._declared_types(list(params.keys()), timeout)
        try:
            response = self._transport.call_service(
                service_name=self._param_service("set"),
                service_type=ARM_PARAMS["set_type"],
                request={
                    "parameters": [
                        {"name": n,
                         "value": _to_param_value_typed(v, declared.get(n, 0))}
                        for n, v in params.items()
                    ]
                },
                timeout=timeout,
            )
            results = response.get("results", [])
            ok = bool(results) and all(r.get("successful", False) for r in results)
            if not ok:
                print(f"[Arm] set_params partial/failed: {results}")
            return ok
        except Exception as e:
            print(f"[Arm] set_params error: {e}")
            return False

    # Valid values for the commander's grasp_scene_action param. See the
    # commander: grasp = attach the box, place = detach + remove after the next
    # motion, none = leave scene alone.
    GRASP_SCENE_ACTIONS = ("grasp", "place", "none")

    def set_grasp_scene_action(self, action: str, timeout: float = 5.0) -> bool:
        """
        Set the commander's ``grasp_scene_action`` param, read fresh on the next
        gripper command. Use before sending an open/close to control what the
        planning-scene grasp box does:

            grasp - attach the box (picking up)
            place - detach + remove after the next motion (placing/releasing)
            none  - leave the planning scene untouched

        Returns True if the commander accepted the change.
        """
        if action not in self.GRASP_SCENE_ACTIONS:
            raise ValueError(
                f"grasp_scene_action must be one of {self.GRASP_SCENE_ACTIONS}, "
                f"got {action!r}"
            )
        return self.set_param("grasp_scene_action", action, timeout=timeout)

    def get_param(self, name: str, timeout: float = 5.0) -> Optional[Any]:
        """Read a single commander parameter. Returns None if it doesn't exist
        or the call fails."""
        values = self.get_params([name], timeout=timeout)
        if values is None:
            return None
        return values.get(name)

    def get_params(
        self, names: List[str], timeout: float = 5.0
    ) -> Optional[Dict[str, Any]]:
        """Read several commander parameters at once. Returns a name→value dict
        (value None for params that aren't set), or None on call failure."""
        try:
            response = self._transport.call_service(
                service_name=self._param_service("get"),
                service_type=ARM_PARAMS["get_type"],
                request={"names": list(names)},
                timeout=timeout,
            )
            values = response.get("values", [])
            return {n: _from_param_value(v) for n, v in zip(names, values)}
        except Exception as e:
            print(f"[Arm] get_params error: {e}")
            return None

    def list_params(
        self, prefixes: Optional[List[str]] = None, timeout: float = 5.0
    ) -> Optional[List[str]]:
        """List the commander's parameter names. Optionally filter by prefix.
        Returns None on call failure."""
        try:
            response = self._transport.call_service(
                service_name=self._param_service("list"),
                service_type=ARM_PARAMS["list_type"],
                request={"prefixes": list(prefixes or []), "depth": 0},
                timeout=timeout,
            )
            return response.get("result", {}).get("names", [])
        except Exception as e:
            print(f"[Arm] list_params error: {e}")
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
