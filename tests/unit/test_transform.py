from unittest.mock import MagicMock

import pytest

from walkie_sdk.modules.transform import Transform


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
