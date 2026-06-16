from __future__ import annotations

import threading
import time
from typing import Optional

from walkie_sdk.config.ros_topics import TF_SERVICE, TF_TOPICS
from walkie_sdk.core.interfaces.ros_transport import ROSTransportInterface
from walkie_sdk.utils.converters import resolve_tf_chain
from walkie_sdk.utils.namespace import apply_namespace


class Transform:
    """Coordinate frame transform lookup.

    Two modes of operation:
    - **Buffered** (default when connected): subscribes to /tf and /tf_static,
      caches the full transform tree locally, and resolves lookups in < 1 ms
      with no network round-trip.
    - **Service fallback**: calls the walkie_tf get_transform service for frames
      not yet in the local buffer (e.g., during the brief warmup window after
      start_buffering() is called).
    """

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace

        # Local TF buffer
        self._tf_cache: dict[tuple[str, str], tuple] = {}      # dynamic (/tf)
        self._static_cache: dict[tuple[str, str], tuple] = {}  # static (/tf_static)
        self._lock = threading.Lock()
        self._buffering = False
        self._has_received_tf = False   # True once the first /tf or /tf_static message arrives
        self._tf_handle = None
        self._tf_static_handle = None

    # ── Namespace ───────────────────────────────────────────────────────────

    @property
    def namespace(self) -> str:
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        self._namespace = value
        if self._buffering:
            self.stop_buffering()
            self.start_buffering()

    # ── Buffer lifecycle ────────────────────────────────────────────────────

    def start_buffering(self) -> None:
        """Subscribe to /tf and /tf_static and start caching transforms locally.

        Called automatically by WalkieRobot._connect(). After a brief warmup
        (~50–100 ms for the first messages to arrive), lookup() will resolve
        entirely from the local cache with no service call.
        """
        self._tf_static_handle = self._transport.subscribe(
            apply_namespace(TF_TOPICS["tf_static"], self._namespace),
            TF_TOPICS["tf_static_type"],
            self._on_tf_static,
        )
        self._tf_handle = self._transport.subscribe(
            apply_namespace(TF_TOPICS["tf"], self._namespace),
            TF_TOPICS["tf_type"],
            self._on_tf,
        )
        self._buffering = True

    def stop_buffering(self) -> None:
        """Unsubscribe from TF topics and clear the local cache."""
        self._buffering = False
        self._has_received_tf = False
        if self._tf_handle is not None:
            try:
                self._transport.unsubscribe(self._tf_handle)
            except Exception:
                pass
            self._tf_handle = None
        if self._tf_static_handle is not None:
            try:
                self._transport.unsubscribe(self._tf_static_handle)
            except Exception:
                pass
            self._tf_static_handle = None
        with self._lock:
            self._tf_cache.clear()
            self._static_cache.clear()

    @property
    def is_buffering(self) -> bool:
        return self._buffering

    @property
    def buffered_edge_count(self) -> int:
        """Number of transform edges currently held in the local cache."""
        with self._lock:
            return len(self._tf_cache) + len(self._static_cache)

    # ── TF callbacks ────────────────────────────────────────────────────────

    def _on_tf(self, msg: dict) -> None:
        try:
            with self._lock:
                self._has_received_tf = True
                for t in msg.get("transforms", []):
                    key = (t["header"]["frame_id"], t["child_frame_id"])
                    tr = t["transform"]["translation"]
                    ro = t["transform"]["rotation"]
                    self._tf_cache[key] = (
                        tr["x"], tr["y"], tr["z"],
                        ro["x"], ro["y"], ro["z"], ro["w"],
                    )
        except Exception:
            pass

    def _on_tf_static(self, msg: dict) -> None:
        try:
            with self._lock:
                self._has_received_tf = True
                for t in msg.get("transforms", []):
                    key = (t["header"]["frame_id"], t["child_frame_id"])
                    tr = t["transform"]["translation"]
                    ro = t["transform"]["rotation"]
                    self._static_cache[key] = (
                        tr["x"], tr["y"], tr["z"],
                        ro["x"], ro["y"], ro["z"], ro["w"],
                    )
        except Exception:
            pass

    # ── Public API ──────────────────────────────────────────────────────────

    def lookup(
        self,
        source_frame: str,
        target_frame: str,
        timeout: float = 5.0,
    ) -> Optional[dict]:
        """Return the transform from source_frame to target_frame.

        Tries the local TF buffer first (< 1 ms); falls back to the
        walkie_tf service if the frame pair is not yet in the buffer.

        Returns:
            {"position": {"x": ..., "y": ..., "z": ...},
             "quaternion": {"x": ..., "y": ..., "z": ..., "w": ...}}
            or None on failure.
        """
        # ── Fast path: local buffer ─────────────────────────────────────
        if self._buffering:
            # If the buffer hasn't received any messages yet (warmup window right
            # after connect), poll briefly before falling through to the service.
            if not self._has_received_tf:
                deadline = time.monotonic() + min(timeout, 0.5)
                while not self._has_received_tf and time.monotonic() < deadline:
                    time.sleep(0.02)

            with self._lock:
                combined = {**self._static_cache, **self._tf_cache}
            result = resolve_tf_chain(combined, source_frame, target_frame)
            if result is not None:
                tx, ty, tz, qx, qy, qz, qw = result
                return {
                    "position": {"x": tx, "y": ty, "z": tz},
                    "quaternion": {"x": qx, "y": qy, "z": qz, "w": qw},
                }

        # ── Fallback: service call ──────────────────────────────────────
        try:
            response = self._transport.call_service(
                service_name=apply_namespace(TF_SERVICE["service_name"], self._namespace),
                service_type=TF_SERVICE["service_type"],
                request={
                    "source_frame": source_frame,
                    "target_frame": target_frame,
                    "timeout_sec": timeout,
                },
                timeout=timeout + 5.0,
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
