"""Unit tests for walkie_sdk.modules.tools.Tools.bboxes_to_positions."""

from unittest.mock import MagicMock
import pytest

from walkie_sdk.modules.tools import Tools
from walkie_sdk.config.ros_topics import OB_POSE_SERVICE


def _make_tools(namespace: str = "") -> tuple[Tools, MagicMock]:
    transport = MagicMock()
    return Tools(transport=transport, namespace=namespace), transport


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


class TestBboxesToPositionsSuccess:
    def test_returns_positions_list(self):
        tools, transport = _make_tools()
        transport.call_service.return_value = {
            "success": True,
            "poses": {"poses": [
                {"position": {"x": 1.0, "y": 2.0, "z": 3.0}},
            ]},
        }

        result = tools.bboxes_to_positions([[320, 240, 50, 50]])

        assert result == [[1.0, 2.0, 3.0]]

    def test_multiple_bboxes(self):
        tools, transport = _make_tools()
        transport.call_service.return_value = {
            "success": True,
            "poses": {"poses": [
                {"position": {"x": 1.0, "y": 2.0, "z": 3.0}},
                {"position": {"x": 4.0, "y": 5.0, "z": 6.0}},
            ]},
        }

        result = tools.bboxes_to_positions([[100, 200, 30, 40], [300, 400, 60, 80]])

        assert result == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]

    def test_service_called_with_correct_args(self):
        tools, transport = _make_tools()
        transport.call_service.return_value = {
            "success": True,
            "poses": {"poses": []},
        }

        tools.bboxes_to_positions([[10, 20, 5, 5]], timeout=3.0)

        call_kwargs = transport.call_service.call_args[1]
        assert call_kwargs["service_name"] == OB_POSE_SERVICE["service_name"]
        assert call_kwargs["service_type"] == OB_POSE_SERVICE["service_type"]
        assert call_kwargs["timeout"] == 3.0
        assert "detections" in call_kwargs["request"]

    def test_namespace_prefixed_service_name(self):
        tools, transport = _make_tools(namespace="robot1")
        transport.call_service.return_value = {
            "success": True,
            "poses": {"poses": []},
        }

        tools.bboxes_to_positions([[0, 0, 1, 1]])

        call_kwargs = transport.call_service.call_args[1]
        expected = f"robot1/{OB_POSE_SERVICE['service_name']}"
        assert call_kwargs["service_name"] == expected


# ---------------------------------------------------------------------------
# Failure paths
# ---------------------------------------------------------------------------


class TestBboxesToPositionsFailure:
    def test_success_false_returns_none(self):
        tools, transport = _make_tools()
        transport.call_service.return_value = {"success": False}

        assert tools.bboxes_to_positions([[1, 2, 3, 4]]) is None

    def test_timeout_returns_none(self):
        tools, transport = _make_tools()
        transport.call_service.side_effect = TimeoutError

        assert tools.bboxes_to_positions([[1, 2, 3, 4]]) is None

    def test_generic_exception_returns_none(self):
        tools, transport = _make_tools()
        transport.call_service.side_effect = RuntimeError("connection lost")

        assert tools.bboxes_to_positions([[1, 2, 3, 4]]) is None

    def test_missing_success_key_returns_none(self):
        """Response without 'success' key defaults to False."""
        tools, transport = _make_tools()
        transport.call_service.return_value = {}

        assert tools.bboxes_to_positions([[1, 2, 3, 4]]) is None
