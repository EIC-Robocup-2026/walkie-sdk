"""Head module — controls the robot's head servo tilt angle."""

import math
from typing import Optional

from walkie_sdk.config.ros_topics import HEAD_TOPICS
from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace

HEAD_TILT_MIN: float = -math.pi / 4  # -0.785 rad  (45° upward)
HEAD_TILT_MAX: float = math.pi / 4   #  0.785 rad  (45° downward)


class Head:
    """
    Head tilt controller.

    Publishes Float64MultiArray to the head servo controller topic.
    Positive angles tilt the camera downward toward the floor; negative
    angles tilt it upward. The safe operating range is ±45° (±π/4 rad).

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

    def __init__(self, transport: ROSTransportInterface, namespace: str = "") -> None:
        self._transport = transport
        self._namespace = namespace
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
                Must be within [HEAD_TILT_MIN, HEAD_TILT_MAX] (±π/4 ≈ ±0.785 rad).

        Raises:
            ValueError: If angle_rad is outside the ±45° safe range.
        """
        if not (HEAD_TILT_MIN <= angle_rad <= HEAD_TILT_MAX):
            raise ValueError(
                f"Head tilt angle {angle_rad:.3f} rad is out of range "
                f"[{HEAD_TILT_MIN:.3f}, {HEAD_TILT_MAX:.3f}] (±45°). "
                "Positive values tilt the camera downward."
            )
        self._transport.publish(
            self.cmd_topic,
            HEAD_TOPICS["cmd_type"],
            {"data": [float(angle_rad)]},
        )
        self._angle = angle_rad

    def get_angle(self) -> Optional[float]:
        """Return the last commanded tilt angle in radians, or None if never set."""
        return self._angle

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
