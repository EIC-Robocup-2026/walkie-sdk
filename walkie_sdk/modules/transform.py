from __future__ import annotations

from typing import Optional

from walkie_sdk.config.ros_topics import TF_SERVICE
from walkie_sdk.core.interfaces.ros_transport import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace


class Transform:
    """Coordinate frame transform lookup via the walkie_tf service node."""

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace

    @property
    def namespace(self) -> str:
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        self._namespace = value

    def lookup(
        self,
        source_frame: str,
        target_frame: str,
        timeout: float = 5.0,
    ) -> Optional[dict]:
        """Return the transform from source_frame to target_frame.

        Returns:
            {"position": {"x": ..., "y": ..., "z": ...},
             "quaternion": {"x": ..., "y": ..., "z": ..., "w": ...}}
            or None on failure.
        """
        try:
            response = self._transport.call_service(
                service_name=apply_namespace(TF_SERVICE["service_name"], self._namespace),
                service_type=TF_SERVICE["service_type"],
                request={
                    "source_frame": source_frame,
                    "target_frame": target_frame,
                    "timeout_sec": timeout,
                },
                timeout=timeout + 1.0,
            )
            if not response.get("success", False):
                print(f"[Transform] lookup failed: {response.get('message', 'unknown error')}")
                return None
            return {
                "position": {
                    "x": response["x"],
                    "y": response["y"],
                    "z": response["z"],
                },
                "quaternion": {
                    "x": response["qx"],
                    "y": response["qy"],
                    "z": response["qz"],
                    "w": response["qw"],
                },
            }
        except TimeoutError:
            print(f"[Transform] lookup({source_frame!r} -> {target_frame!r}) timed out after {timeout}s")
            return None
        except Exception as e:
            print(f"[Transform] lookup error: {e}")
            return None
