"""
Walkie SDK - Hybrid Transport

Combines two transports behind the single :class:`ROSTransportInterface`:

- **Zenoh** carries topics and services (subscribe / publish / call_service).
  This is the fast, native-latency path (~1-2 ms service calls).
- **Rosbridge** carries actions (call_action / cancel_action), because
  ``zenoh_ros2_sdk`` exposes no action client, whereas roslibpy's
  ``ActionClient`` works. Actions (nav ``go_to``, arm moves) are infrequent and
  latency-tolerant, so rosbridge's per-call overhead is irrelevant there.

Modules stay protocol-agnostic — they call the same interface methods and never
know two transports are in play. Requires the robot to run *both* a zenoh router
and rosbridge_server (the Walkie robot already does).
"""

from __future__ import annotations

from typing import Any, Callable, Dict, Optional

from walkie_sdk.core.interfaces.ros_transport import ROSTransportInterface

# The hybrid transport inherently needs both backends, so importing them here
# (rather than lazily) is fine — this module is only imported when the hybrid
# protocol is explicitly selected by the factory.
from walkie_sdk.core.transports.rosbridge import ROSBridgeTransport
from walkie_sdk.core.transports.zenoh import ZenohTransport


# rcl_interfaces parameter services consistently time out on zenoh_ros2_sdk because
# their nested message types are mis-resolved by the type hash. Route them directly
# to rosbridge (the rosbridge path always worked for these).
_ROSBRIDGE_ONLY_SERVICE_TYPES = frozenset({
    "rcl_interfaces/srv/SetParameters",
    "rcl_interfaces/srv/SetParametersAtomically",
    "rcl_interfaces/srv/GetParameters",
    "rcl_interfaces/srv/ListParameters",
    "rcl_interfaces/srv/DescribeParameters",
})


class HybridTransport(ROSTransportInterface[Any]):
    """Zenoh for topics/services, rosbridge for actions.

    Args:
        host: Robot IP address or hostname (shared by both backends).
        zenoh_port: Zenoh router port (default 7447).
        rosbridge_port: Rosbridge WebSocket port (default 9090).
        timeout: Connection timeout in seconds.
    """

    def __init__(
        self,
        host: str,
        zenoh_port: int = 7447,
        rosbridge_port: int = 9090,
        timeout: float = 10.0,
        **kwargs,
    ):
        self._host = host
        self._zenoh = ZenohTransport(host=host, port=zenoh_port, timeout=timeout)
        self._rosbridge = ROSBridgeTransport(host=host, port=rosbridge_port, timeout=timeout)

    # ── Sub-transport access (for debugging/advanced use) ───────────────────

    @property
    def zenoh(self) -> ZenohTransport:
        """The zenoh backend (topics/services)."""
        return self._zenoh

    @property
    def rosbridge(self) -> ROSBridgeTransport:
        """The rosbridge backend (actions)."""
        return self._rosbridge

    # ── Lifecycle ───────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Connect both backends.

        Zenoh (the data path used at startup by telemetry/joints/services) is
        required. Rosbridge (actions) is best-effort: if it fails, topics and
        services still work and only actions are unavailable — consistent with
        the SDK's graceful-degradation convention.
        """
        self._zenoh.connect()
        try:
            self._rosbridge.connect()
        except Exception as e:
            print(
                f"  ⚠ Hybrid: rosbridge connect failed ({e}); "
                f"actions (nav go_to, arm moves) will be unavailable."
            )

    def disconnect(self) -> None:
        """Disconnect both backends. Safe to call multiple times."""
        for name, t in (("zenoh", self._zenoh), ("rosbridge", self._rosbridge)):
            try:
                t.disconnect()
            except Exception as e:
                print(f"  ⚠ Hybrid: {name} disconnect error: {e}")

    @property
    def is_connected(self) -> bool:
        """Connected once the zenoh data path is up (the primary channel)."""
        return self._zenoh.is_connected

    # ── Topics + services → zenoh ───────────────────────────────────────────

    def subscribe(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[Dict[str, Any]], None],
        throttle_rate: int = 0,
        queue_size: int = 1,
    ) -> Any:
        return self._zenoh.subscribe(
            topic, message_type, callback, throttle_rate=throttle_rate, queue_size=queue_size
        )

    def unsubscribe(self, handle: Any) -> None:
        self._zenoh.unsubscribe(handle)

    def publish(self, topic: str, message_type: str, message: Dict[str, Any]) -> None:
        self._zenoh.publish(topic, message_type, message)

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request: Dict[str, Any],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Call a service over zenoh, falling back to rosbridge on failure.

        zenoh is the fast path and handles most services (e.g. get_transform).
        But some fail over ``zenoh_ros2_sdk``: responses containing nested
        messages mis-resolve their type (e.g. ``nav_msgs/srv/GetMap`` ->
        ``KeyError 'nav_msgs/srv/msg/OccupancyGrid'``), and the SDK's computed
        type hash can mismatch the robot's, causing a timeout. rosbridge handles
        these — it's the same path that worked before the zenoh migration — so we
        fall back to it rather than failing the call.
        """
        # Route services that are known to fail on zenoh directly to rosbridge.
        if service_type in _ROSBRIDGE_ONLY_SERVICE_TYPES:
            if self._rosbridge.is_connected:
                return self._rosbridge.call_service(
                    service_name, service_type, request, timeout=timeout
                )
        try:
            return self._zenoh.call_service(service_name, service_type, request, timeout=timeout)
        except Exception as e:
            if self._rosbridge.is_connected:
                print(
                    f"[Hybrid] zenoh service '{service_name}' failed ({e}); "
                    f"falling back to rosbridge."
                )
                return self._rosbridge.call_service(
                    service_name, service_type, request, timeout=timeout
                )
            raise

    # ── Actions → rosbridge ─────────────────────────────────────────────────

    def call_action(
        self,
        action_name: str,
        action_type: str,
        goal: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        if not self._rosbridge.is_connected:
            print(
                f"[Hybrid] rosbridge not connected; cannot call action '{action_name}'."
            )
            return {"result": None, "status": "FAILED"}
        return self._rosbridge.call_action(
            action_name, action_type, goal, feedback_callback=feedback_callback, timeout=timeout
        )

    def cancel_action(self) -> None:
        self._rosbridge.cancel_action()

    def __repr__(self) -> str:
        z = "up" if self._zenoh.is_connected else "down"
        r = "up" if self._rosbridge.is_connected else "down"
        return f"HybridTransport(host='{self._host}', zenoh={z}, rosbridge={r})"
