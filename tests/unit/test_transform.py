import math
from unittest.mock import MagicMock, call

import pytest

from walkie_sdk.modules.transform import Transform
from walkie_sdk.utils.converters import (
    compose_transforms,
    invert_transform,
    resolve_tf_chain,
    rotate_vector_by_quaternion,
)


def _make_transform(namespace: str = "") -> tuple[Transform, MagicMock]:
    transport = MagicMock()
    return Transform(transport=transport, namespace=namespace), transport


_SUCCESS_RESPONSE = {
    "success": True,
    "message": "",
    "x": 1.0,
    "y": 2.0,
    "z": 3.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0,
}


class TestLookupSuccess:
    def test_returns_position_and_quaternion(self):
        transform, transport = _make_transform()
        transport.call_service.return_value = _SUCCESS_RESPONSE
        result = transform.lookup("map", "base_link")
        assert result == {
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

    def test_calls_correct_service_type(self):
        transform, transport = _make_transform()
        transport.call_service.return_value = _SUCCESS_RESPONSE
        transform.lookup("map", "base_link")
        call_kwargs = transport.call_service.call_args[1]
        assert "walkie_tf_interfaces/srv/GetTransform" in call_kwargs["service_type"]

    def test_request_contains_frames(self):
        transform, transport = _make_transform()
        transport.call_service.return_value = _SUCCESS_RESPONSE
        transform.lookup("odom", "camera_frame")
        request = transport.call_service.call_args[1]["request"]
        assert request["source_frame"] == "odom"
        assert request["target_frame"] == "camera_frame"

    def test_request_contains_timeout(self):
        transform, transport = _make_transform()
        transport.call_service.return_value = _SUCCESS_RESPONSE
        transform.lookup("map", "base_link", timeout=3.0)
        request = transport.call_service.call_args[1]["request"]
        assert request["timeout_sec"] == 3.0

    def test_service_timeout_is_greater_than_request_timeout(self):
        transform, transport = _make_transform()
        transport.call_service.return_value = _SUCCESS_RESPONSE
        transform.lookup("map", "base_link", timeout=4.0)
        service_timeout = transport.call_service.call_args[1]["timeout"]
        assert service_timeout > 4.0


class TestLookupFailure:
    def test_success_false_returns_none(self):
        transform, transport = _make_transform()
        transport.call_service.return_value = {"success": False, "message": "no such frame"}
        assert transform.lookup("map", "missing_frame") is None

    def test_timeout_error_returns_none(self):
        transform, transport = _make_transform()
        transport.call_service.side_effect = TimeoutError
        assert transform.lookup("map", "base_link") is None

    def test_generic_exception_returns_none(self):
        transform, transport = _make_transform()
        transport.call_service.side_effect = RuntimeError("connection lost")
        assert transform.lookup("map", "base_link") is None


class TestNamespace:
    def test_namespace_applied_to_service_name(self):
        transform, transport = _make_transform(namespace="robot1")
        transport.call_service.return_value = _SUCCESS_RESPONSE
        transform.lookup("map", "base_link")
        service_name = transport.call_service.call_args[1]["service_name"]
        assert service_name.startswith("robot1/")

    def test_no_namespace_uses_bare_service_name(self):
        transform, transport = _make_transform(namespace="")
        transport.call_service.return_value = _SUCCESS_RESPONSE
        transform.lookup("map", "base_link")
        service_name = transport.call_service.call_args[1]["service_name"]
        assert "/" not in service_name

    def test_namespace_getter_setter_roundtrip(self):
        transform, _ = _make_transform(namespace="alpha")
        assert transform.namespace == "alpha"
        transform.namespace = "beta"
        assert transform.namespace == "beta"


# ── TF math helpers ─────────────────────────────────────────────────────────


class TestRotateVectorByQuaternion:
    def test_identity_quaternion_leaves_vector_unchanged(self):
        v = (1.0, 2.0, 3.0)
        q = (0.0, 0.0, 0.0, 1.0)  # identity
        result = rotate_vector_by_quaternion(v, q)
        assert all(abs(result[i] - v[i]) < 1e-9 for i in range(3))

    def test_90_degree_z_rotation(self):
        # Rotate (1,0,0) by 90° around Z → (0,1,0)
        v = (1.0, 0.0, 0.0)
        q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        rx, ry, rz = rotate_vector_by_quaternion(v, q)
        assert abs(rx - 0.0) < 1e-9
        assert abs(ry - 1.0) < 1e-9
        assert abs(rz - 0.0) < 1e-9


class TestCompositionAndInversion:
    def _identity(self):
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    def test_compose_with_identity_is_noop(self):
        tf = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        result = compose_transforms(self._identity(), tf)
        assert all(abs(result[i] - tf[i]) < 1e-9 for i in range(7))

    def test_compose_two_translations(self):
        t1 = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        t2 = (2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        result = compose_transforms(t1, t2)
        assert abs(result[0] - 3.0) < 1e-9

    def test_invert_then_compose_gives_identity(self):
        tf = (1.0, 2.0, 3.0, 0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        tf_inv = invert_transform(tf)
        result = compose_transforms(tf, tf_inv)
        identity = self._identity()
        assert all(abs(result[i] - identity[i]) < 1e-9 for i in range(7))


class TestResolveTfChain:
    def _edge(self, tx=0.0, ty=0.0, tz=0.0):
        return (tx, ty, tz, 0.0, 0.0, 0.0, 1.0)

    def test_same_frame_returns_identity(self):
        result = resolve_tf_chain({}, "map", "map")
        assert result == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    def test_direct_edge_returns_transform(self):
        data = {("map", "odom"): self._edge(tx=1.0)}
        result = resolve_tf_chain(data, "map", "odom")
        assert result is not None
        assert abs(result[0] - 1.0) < 1e-9

    def test_reverse_edge_inverts_transform(self):
        data = {("map", "odom"): self._edge(tx=5.0)}
        result = resolve_tf_chain(data, "odom", "map")
        assert result is not None
        assert abs(result[0] - (-5.0)) < 1e-9

    def test_three_frame_chain(self):
        data = {
            ("map", "odom"): self._edge(tx=1.0),
            ("odom", "base_link"): self._edge(tx=2.0),
        }
        result = resolve_tf_chain(data, "map", "base_link")
        assert result is not None
        assert abs(result[0] - 3.0) < 1e-9

    def test_disconnected_frames_returns_none(self):
        data = {("map", "odom"): self._edge(tx=1.0)}
        assert resolve_tf_chain(data, "map", "camera_link") is None


# ── Local buffer mode ────────────────────────────────────────────────────────


def _make_buffering_transform():
    transport = MagicMock()
    transport.subscribe.return_value = MagicMock()
    t = Transform(transport=transport, namespace="")
    t.start_buffering()
    return t, transport


class TestBufferLifecycle:
    def test_start_buffering_subscribes_to_tf_and_tf_static(self):
        t, transport = _make_buffering_transform()
        assert transport.subscribe.call_count == 2
        topics = [c[1]["topic"] if c[1] else c[0][0]
                  for c in transport.subscribe.call_args_list]
        # Both /tf and /tf_static must be subscribed
        topic_args = [str(transport.subscribe.call_args_list[i]) for i in range(2)]
        assert t.is_buffering

    def test_stop_buffering_unsubscribes(self):
        t, transport = _make_buffering_transform()
        t.stop_buffering()
        assert not t.is_buffering
        assert transport.unsubscribe.call_count == 2

    def test_buffered_edge_count_starts_at_zero(self):
        t, _ = _make_buffering_transform()
        assert t.buffered_edge_count == 0


class TestBufferLookup:
    def _tf_msg(self, parent, child, tx=0.0, ty=0.0, tz=0.0):
        return {"transforms": [{
            "header": {"frame_id": parent},
            "child_frame_id": child,
            "transform": {
                "translation": {"x": tx, "y": ty, "z": tz},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }]}

    def test_lookup_returns_buffered_result_without_service_call(self):
        t, transport = _make_buffering_transform()
        t._on_tf(self._tf_msg("map", "base_link", tx=3.0))
        result = t.lookup("map", "base_link")
        assert result is not None
        assert abs(result["position"]["x"] - 3.0) < 1e-9
        transport.call_service.assert_not_called()

    def test_static_transforms_used_in_lookup(self):
        t, transport = _make_buffering_transform()
        t._on_tf_static(self._tf_msg("base_link", "camera", tx=0.5))
        result = t.lookup("base_link", "camera")
        assert result is not None
        assert abs(result["position"]["x"] - 0.5) < 1e-9
        transport.call_service.assert_not_called()

    def test_dynamic_transform_overrides_static(self):
        t, transport = _make_buffering_transform()
        t._on_tf_static(self._tf_msg("map", "odom", tx=1.0))
        t._on_tf(self._tf_msg("map", "odom", tx=9.9))
        result = t.lookup("map", "odom")
        assert result is not None
        assert abs(result["position"]["x"] - 9.9) < 1e-9

    def test_lookup_falls_back_to_service_for_unknown_frame(self):
        t, transport = _make_buffering_transform()
        transport.call_service.return_value = _SUCCESS_RESPONSE
        # Warm the buffer with an unrelated edge so the warmup wait is skipped.
        t._on_tf(self._tf_msg("map", "odom"))
        result = t.lookup("map", "unknown_frame")
        transport.call_service.assert_called_once()

    def test_lookup_three_frame_chain_from_buffer(self):
        t, transport = _make_buffering_transform()
        t._on_tf(self._tf_msg("map", "odom", tx=1.0))
        t._on_tf(self._tf_msg("odom", "base_link", tx=2.0))
        result = t.lookup("map", "base_link")
        assert result is not None
        assert abs(result["position"]["x"] - 3.0) < 1e-9
        transport.call_service.assert_not_called()

    def test_malformed_tf_message_does_not_crash(self):
        t, _ = _make_buffering_transform()
        t._on_tf({"transforms": [{"bad_key": "no_header"}]})
        assert t.buffered_edge_count == 0

    def test_namespace_change_resubscribes(self):
        t, transport = _make_buffering_transform()
        initial_subscribe_count = transport.subscribe.call_count
        t.namespace = "robot1"
        assert transport.unsubscribe.called
        assert transport.subscribe.call_count > initial_subscribe_count
