"""
Lift - Robot lift/elevator control module.

Provides set() and get() for controlling the linear lift actuator.

Command topic:  lift/cmd          (std_msgs/msg/Float64MultiArray)
                data: [target_pos_cm, vel_cm_s, accel_cm_s2]

Feedback topic: lift/joint_states (sensor_msgs/msg/JointState)
                position[0] in meters

Travel range: 0.0 cm (bottom) → 74.35 cm (top)
Normalized:   0.0              → 1.0
"""

import threading
from typing import Optional

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace
from walkie_sdk.config.ros_topics import LIFT_TOPICS

LIFT_MAX_CM = 74.35
LIFT_DEFAULT_SPEED = 2.0   # cm/s
LIFT_DEFAULT_ACCEL = 1.0   # cm/s²


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

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace
        self._lock = threading.Lock()
        self._latest_pos_m: Optional[float] = None
        self._subscribed = False

    def _setup_state_subscription(self) -> None:
        """Subscribe to lift joint states. Called after transport connects."""
        if self._subscribed:
            return

        def _cb(msg: dict) -> None:
            positions = msg.get("position", [])
            if positions:
                with self._lock:
                    self._latest_pos_m = float(positions[0])

        try:
            topic = apply_namespace(LIFT_TOPICS["states"], self._namespace)
            print(f"[Lift] Subscribing to topic: '{topic}'")
            self._transport.subscribe(topic, LIFT_TOPICS["states_type"], _cb)
            self._subscribed = True
            print(f"[Lift] Successfully subscribed to '{topic}'")
        except Exception as e:
            print(f"[Lift] Failed to subscribe to lift joint states: {e}")

    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """Set ROS namespace and re-subscribe with the new prefix."""
        self._namespace = value
        self._subscribed = False
        self._setup_state_subscription()

    @property
    def cmd_topic(self) -> str:
        """Full lift command topic name with namespace."""
        return apply_namespace(LIFT_TOPICS["cmd"], self._namespace)

    @property
    def states_topic(self) -> str:
        """Full lift joint states topic name with namespace."""
        return apply_namespace(LIFT_TOPICS["states"], self._namespace)

    def set(
        self,
        pos: float,
        speed: float = LIFT_DEFAULT_SPEED,
        accel: float = LIFT_DEFAULT_ACCEL,
        norm_pos: bool = True,
    ) -> None:
        """
        Send a position command to the lift.

        Args:
            pos: Target position. Normalized 0.0–1.0 when norm_pos=True,
                 or real centimeters 0.0–74.35 when norm_pos=False.
            speed: Travel speed in cm/s (default: 2.0).
            accel: Acceleration in cm/s² (default: 1.0).
            norm_pos: If True (default), treat pos as normalized 0.0–1.0.
                      If False, treat pos as real position in cm.

        Example:
            ```python
            bot.lift.set(0.5)                          # midpoint (normalized)
            bot.lift.set(37.175, norm_pos=False)       # midpoint in cm
            bot.lift.set(1.0, speed=5.0, accel=2.0)   # top, fast
            ```
        """
        if norm_pos:
            if pos < 0.0 or pos > 1.0:
                print(f"[Lift] pos {pos} out of normalized range [0,1], clamping.")
            pos_cm = max(0.0, min(1.0, pos)) * LIFT_MAX_CM
        else:
            if pos < 0.0 or pos > LIFT_MAX_CM:
                print(f"[Lift] pos {pos} out of range [0, {LIFT_MAX_CM}] cm, clamping.")
            pos_cm = max(0.0, min(LIFT_MAX_CM, pos))

        msg = {"data": [float(pos_cm), float(speed), float(accel)]}
        self._transport.publish(self.cmd_topic, LIFT_TOPICS["cmd_type"], msg)

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
            bot.lift.get()              # e.g. 0.5  (normalized)
            bot.lift.get(norm_pos=False) # e.g. 37.175  (cm)
            ```
        """
        with self._lock:
            pos_m = self._latest_pos_m

        if pos_m is None:
            return None

        pos_cm = pos_m * 100.0
        return pos_cm / LIFT_MAX_CM if norm_pos else pos_cm
