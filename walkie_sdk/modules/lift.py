"""
Lift - Robot lift/elevator control module.

Provides set() and get() for controlling the linear lift actuator.

Command topic: lift/cmd  (std_msgs/msg/Float64MultiArray)
               data: [target_pos_cm, vel_cm_s, accel_cm_s2]

Position feedback is read from the shared JointStateHub via the
``lift_joint`` entry in joint_states (position in meters).

Travel range: 0.0 cm (bottom) → 74.35 cm (top)
Normalized:   0.0              → 1.0
"""

import threading
import time
from typing import TYPE_CHECKING, Optional

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace
from walkie_sdk.config.ros_topics import LIFT_TOPICS

if TYPE_CHECKING:
    from walkie_sdk.modules.joint_state_hub import JointStateHub

LIFT_MAX_CM = 74.35
LIFT_DEFAULT_SPEED = 2.0
LIFT_DEFAULT_ACCEL = 1.0
LIFT_DEFAULT_TIMEOUT = 30.0
LIFT_DEFAULT_TOLERANCE = 0.02  # normalized (≈ 1.5 cm)


class Lift:
    """
    Robot lift controller.

    Publishes position commands and reads back joint state feedback.
    Positions can be expressed in normalized form (0.0–1.0) or real
    centimeters (0.0–74.35 cm).

    Args:
        transport: Transport instance implementing ROSTransportInterface
        namespace: ROS namespace prefix for topics (default: "" = no namespace)
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
        self._status: Optional[str] = None

    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        self._namespace = value

    @property
    def cmd_topic(self) -> str:
        """Full lift command topic name with namespace."""
        return apply_namespace(LIFT_TOPICS["cmd"], self._namespace)

    @property
    def status(self) -> Optional[str]:
        """Current lift status: ``'SUCCEEDED'``, ``'TIMEOUT'``, ``'IN_PROGRESS'``, or None."""
        return self._status

    @property
    def is_moving(self) -> bool:
        """True if a non-blocking move is currently in progress."""
        return self._status == "IN_PROGRESS"

    def _publish_command(self, pos_cm: float, speed: float, accel: float) -> None:
        msg = {"data": [float(pos_cm), float(speed), float(accel)]}
        self._transport.publish(self.cmd_topic, LIFT_TOPICS["cmd_type"], msg)

    def _wait_until_reached(self, target_norm: float, tolerance: float, timeout: float) -> str:
        """Poll position until within tolerance or timeout. Returns 'SUCCEEDED' or 'TIMEOUT'."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = self.get(norm_pos=True)
            if current is not None and abs(current - target_norm) <= tolerance:
                return "SUCCEEDED"
            time.sleep(0.05)
        return "TIMEOUT"

    def _poll_and_update_status(self, target_norm: float, tolerance: float, timeout: float) -> None:
        """Background thread: poll position and update status when reached or timed out."""
        self._status = self._wait_until_reached(target_norm, tolerance, timeout)

    def set(
        self,
        pos: float,
        speed: float = LIFT_DEFAULT_SPEED,
        accel: float = LIFT_DEFAULT_ACCEL,
        norm_pos: bool = True,
        blocking: bool = True,
        timeout: float = LIFT_DEFAULT_TIMEOUT,
        tolerance: float = LIFT_DEFAULT_TOLERANCE,
    ) -> str:
        """
        Send a position command to the lift.

        Args:
            pos: Target position. Normalized 0.0–1.0 when norm_pos=True,
                 or real centimeters 0.0–74.35 when norm_pos=False.
            speed: Travel speed in cm/s (default: 2.0).
            accel: Acceleration in cm/s² (default: 1.0).
            norm_pos: If True (default), treat pos as normalized 0.0–1.0.
                      If False, treat pos as real position in cm.
            blocking: If True (default), wait until the lift reaches the target
                      position before returning. If False, return immediately.
            timeout: Maximum seconds to wait when blocking=True (default: 30.0).
            tolerance: Acceptable error in normalized units to consider the
                       target reached (default: 0.02 ≈ 1.5 cm).

        Returns:
            ``"SUCCEEDED"`` when position reached, ``"TIMEOUT"`` if timed out,
            or ``"IN_PROGRESS"`` when blocking=False.

        Example:
            ```python
            bot.lift.set(0.5)                        # midpoint, wait until reached
            bot.lift.set(1.0, speed=5.0, accel=2.0) # top, fast, blocking
            bot.lift.set(0.0, blocking=False)        # fire-and-forget
            bot.lift.set(37.0, norm_pos=False)       # 37 cm, blocking
            ```
        """
        if norm_pos:
            if pos < 0.0 or pos > 1.0:
                print(f"[Lift] pos {pos} out of normalized range [0,1], clamping.")
            target_norm = max(0.0, min(1.0, pos))
            pos_cm = target_norm * LIFT_MAX_CM
        else:
            if pos < 0.0 or pos > LIFT_MAX_CM:
                print(f"[Lift] pos {pos} out of range [0, {LIFT_MAX_CM}] cm, clamping.")
            pos_cm = max(0.0, min(LIFT_MAX_CM, pos))
            target_norm = pos_cm / LIFT_MAX_CM

        self._publish_command(pos_cm, speed, accel)

        if not blocking:
            self._status = "IN_PROGRESS"
            threading.Thread(
                target=self._poll_and_update_status,
                args=(target_norm, tolerance, timeout),
                daemon=True,
            ).start()
            return "IN_PROGRESS"

        result = self._wait_until_reached(target_norm, tolerance, timeout)
        self._status = result
        return result

    def get(self, norm_pos: bool = True) -> Optional[float]:
        """
        Get the current lift position.

        Args:
            norm_pos: If True (default), return normalized value 0.0–1.0.
                      If False, return real position in centimeters.

        Returns:
            Current position, or None if no data has been received yet.

        Example:
            ```python
            bot.lift.get()               # e.g. 0.5  (normalized)
            bot.lift.get(norm_pos=False) # e.g. 37.175  (cm)
            ```
        """
        if self._joint_state_hub is None:
            return None

        pos_m = self._joint_state_hub.get("lift_joint")
        if pos_m is None:
            return None

        pos_cm = pos_m * 100.0
        return pos_cm / LIFT_MAX_CM if norm_pos else pos_cm
