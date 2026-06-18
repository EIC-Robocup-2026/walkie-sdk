"""Head module — controls the robot's head servo tilt angle."""

import math
from typing import TYPE_CHECKING, Optional

from walkie_sdk.config.ros_topics import HEAD_TOPICS
from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace

if TYPE_CHECKING:
    from walkie_sdk.modules.joint_state_hub import JointStateHub

HEAD_TILT_MIN: float = -math.pi / 4  # -0.785 rad  (45° upward)
HEAD_TILT_MAX: float = math.pi / 3   #  1.047 rad  (60° downward)


class Head:
    """
    Head tilt controller.

    Publishes Float64MultiArray to the head servo controller topic.
    Positive angles tilt the camera downward toward the floor; negative
    angles tilt it upward.

    Args:
        transport: ROS transport interface.
        namespace: ROS namespace prefix (default: no namespace).

    Example:
        ```python
        bot.head.tilt(0.5)      # tilt 0.5 rad downward
        bot.head.tilt(0.0)      # look forward (center)
        bot.head.get_angle()    # returns 0.0
        ```
    """

    def __init__(
        self,
        transport: ROSTransportInterface,
        namespace: str = "",
        joint_state_hub: "JointStateHub" = None,
    ) -> None:
        self._transport = transport
        self._namespace = namespace
        self._joint_state_hub = joint_state_hub
        self._angle: Optional[float] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def tilt(self, angle_rad: float) -> None:
        """
        Set the head tilt angle.

        Args:
            angle_rad: Tilt angle in radians. Positive values angle the
                camera downward; negative values angle it upward.
                Must be within [HEAD_TILT_MIN, HEAD_TILT_MAX].

        Raises:
            ValueError: If angle_rad is outside the safe range.
        """
        if not (HEAD_TILT_MIN <= angle_rad <= HEAD_TILT_MAX):
            raise ValueError(
                f"Head tilt angle {angle_rad:.3f} rad is out of range "
                f"[{HEAD_TILT_MIN:.3f}, {HEAD_TILT_MAX:.3f}]. "
                "Positive values tilt the camera downward."
            )
        self._transport.publish(
            self.cmd_topic,
            HEAD_TOPICS["cmd_type"],
            {"data": [float(angle_rad)]},
        )
        self._angle = angle_rad

    def get_angle(self) -> Optional[float]:
        """
        Return the current tilt angle in radians from the robot's joint state.

        Reads from the shared JointStateHub (joint name: ``head_servo_joint``).
        Falls back to the last commanded angle if the hub has no data yet,
        or None if the joint has never been commanded.
        """
        if self._joint_state_hub is not None:
            hub_val = self._joint_state_hub.get(HEAD_TOPICS["state_joint"])
            if hub_val is not None:
                return hub_val
        return self._angle

    def set_auto_tilt(self, enable: bool, timeout: float = 5.0) -> bool:
        """
        Enable or disable the head_tilt_near_goal auto-tilt node.

        Args:
            enable: True to enable auto tilt, False to disable.
            timeout: Service call timeout in seconds.

        Returns:
            True if the service reported success, False otherwise.
        """
        response = self._transport.call_service(
            service_name=apply_namespace(
                HEAD_TOPICS["auto_tilt_enable_service"], self._namespace
            ),
            service_type=HEAD_TOPICS["auto_tilt_enable_service_type"],
            request={"data": bool(enable)},
            timeout=timeout,
        )
        return bool(response.get("success", False))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @property
    def cmd_topic(self) -> str:
        return apply_namespace(HEAD_TOPICS["cmd"], self._namespace)

    @property
    def namespace(self) -> str:
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        self._namespace = value
