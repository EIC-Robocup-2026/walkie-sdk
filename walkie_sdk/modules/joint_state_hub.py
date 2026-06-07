"""JointStateHub — single subscription to joint_states shared by all modules."""

import threading
from typing import Dict, Optional

from walkie_sdk.config.ros_topics import JOINT_STATE_TOPICS
from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace


class JointStateHub:
    """
    Subscribes once to the joint_states topic and caches all joint data.

    All modules that need joint feedback (Arm, Lift, Head) read from this
    instead of holding their own subscriptions. One transport subscription
    covers every joint on the robot.

    Args:
        transport: ROS transport interface.
        namespace: ROS namespace prefix (default: no namespace).

    Example:
        ```python
        bot.joints.get("head_servo_joint")   # current position in radians
        bot.joints.get("lift_joint")         # current position in meters
        bot.joints.get_all()                 # full snapshot of all joints
        ```
    """

    def __init__(self, transport: ROSTransportInterface, namespace: str = "") -> None:
        self._transport = transport
        self._namespace = namespace
        self._lock = threading.Lock()
        self._data: Dict[str, Dict[str, float]] = {}
        self._subscribed = False

    # ── Subscription setup ─────────────────────────────────────────────────

    def _setup_subscription(self) -> None:
        """Subscribe to joint_states. Called by WalkieRobot after connecting."""
        if self._subscribed:
            return

        def _safe(v) -> float:
            """Convert to float, returning 0.0 for None or NaN."""
            if v is None:
                return 0.0
            f = float(v)
            return f if f == f else 0.0  # NaN check

        def _cb(msg: dict) -> None:
            names      = msg.get("name", [])
            positions  = msg.get("position", [])
            velocities = msg.get("velocity", [])
            efforts    = msg.get("effort", [])
            data: Dict[str, Dict[str, float]] = {}
            for i, name in enumerate(names):
                data[name] = {
                    "position": _safe(positions[i] if i < len(positions) else None),
                    "velocity": _safe(velocities[i] if i < len(velocities) else None),
                    "effort":   _safe(efforts[i]   if i < len(efforts)    else None),
                }
            with self._lock:
                # Merge per-joint rather than replacing the whole cache: the
                # robot publishes joints (e.g. lift_joint, head_servo_joint)
                # from separate publishers whose messages interleave, each
                # carrying only its own joint. A full replacement would wipe
                # out the other publisher's joint on every message.
                self._data.update(data)

        try:
            topic = apply_namespace(JOINT_STATE_TOPICS["states"], self._namespace)
            print(f"[JointStateHub] Subscribing to '{topic}'")
            self._transport.subscribe(topic, JOINT_STATE_TOPICS["states_type"], _cb)
            self._subscribed = True
            print(f"[JointStateHub] Subscribed to '{topic}'")
        except Exception as e:
            print(f"[JointStateHub] Failed to subscribe: {e}")

    # ── Public API ─────────────────────────────────────────────────────────

    def get(self, joint_name: str) -> Optional[float]:
        """Return the current position of a joint (rad or m), or None if no data yet."""
        with self._lock:
            entry = self._data.get(joint_name)
        return entry["position"] if entry is not None else None

    def get_velocity(self, joint_name: str) -> Optional[float]:
        """Return the current velocity of a joint, or None if no data yet."""
        with self._lock:
            entry = self._data.get(joint_name)
        return entry["velocity"] if entry is not None else None

    def get_effort(self, joint_name: str) -> Optional[float]:
        """Return the current effort of a joint, or None if no data yet."""
        with self._lock:
            entry = self._data.get(joint_name)
        return entry["effort"] if entry is not None else None

    def get_all(self) -> Dict[str, Dict[str, float]]:
        """Return a full snapshot: ``{joint_name: {position, velocity, effort}}``."""
        with self._lock:
            return {name: dict(data) for name, data in self._data.items()}

    # ── Namespace ──────────────────────────────────────────────────────────

    @property
    def namespace(self) -> str:
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        self._namespace = value
        self._subscribed = False
        self._setup_subscription()
