"""Unit tests for walkie_sdk.modules.head.Head."""

import math
from unittest.mock import MagicMock

import pytest

from walkie_sdk.modules.head import Head, HEAD_TILT_MIN, HEAD_TILT_MAX
from walkie_sdk.config.ros_topics import HEAD_TOPICS


def _make_hub(angle: float = None) -> MagicMock:
    hub = MagicMock()
    hub.get.return_value = angle
    return hub


def _make_head(namespace: str = "", hub_angle: float = None) -> tuple[Head, MagicMock, MagicMock]:
    transport = MagicMock()
    hub = _make_hub(hub_angle)
    return Head(transport=transport, namespace=namespace, joint_state_hub=hub), transport, hub


class TestHeadTilt:
    def test_valid_angle_publishes_correct_payload(self):
        head, transport, _ = _make_head()
        head.tilt(0.5)
        topic, msg_type, msg = transport.publish.call_args[0]
        assert msg == {"data": [0.5]}
        assert msg_type == HEAD_TOPICS["cmd_type"]

    def test_zero_angle_publishes_zero(self):
        head, transport, _ = _make_head()
        head.tilt(0.0)
        msg = transport.publish.call_args[0][2]
        assert msg == {"data": [0.0]}

    def test_min_boundary_is_accepted(self):
        head, transport, _ = _make_head()
        head.tilt(HEAD_TILT_MIN)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == pytest.approx(HEAD_TILT_MIN)

    def test_max_boundary_is_accepted(self):
        head, transport, _ = _make_head()
        head.tilt(HEAD_TILT_MAX)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == pytest.approx(HEAD_TILT_MAX)

    def test_above_max_raises_value_error(self):
        head, _, _ = _make_head()
        with pytest.raises(ValueError, match="out of range"):
            head.tilt(HEAD_TILT_MAX + 0.001)

    def test_below_min_raises_value_error(self):
        head, _, _ = _make_head()
        with pytest.raises(ValueError, match="out of range"):
            head.tilt(HEAD_TILT_MIN - 0.001)

    def test_out_of_range_does_not_publish(self):
        head, transport, _ = _make_head()
        with pytest.raises(ValueError):
            head.tilt(1.0)
        transport.publish.assert_not_called()

    def test_topic_uses_namespace(self):
        head, transport, _ = _make_head(namespace="robot1")
        head.tilt(0.3)
        topic = transport.publish.call_args[0][0]
        assert topic.startswith("robot1/")


class TestHeadGetAngle:
    def test_returns_none_before_any_data_and_no_command(self):
        head, _, _ = _make_head(hub_angle=None)
        assert head.get_angle() is None

    def test_returns_hub_value_when_available(self):
        head, _, _ = _make_head(hub_angle=0.42)
        assert head.get_angle() == pytest.approx(0.42)

    def test_hub_value_takes_priority_over_last_command(self):
        head, _, hub = _make_head(hub_angle=0.42)
        head.tilt(0.1)
        # hub says 0.42 — trust the sensor
        assert head.get_angle() == pytest.approx(0.42)

    def test_falls_back_to_last_command_when_hub_has_no_data(self):
        head, _, hub = _make_head(hub_angle=None)
        head.tilt(0.3)
        hub.get.return_value = None  # hub still has no data
        assert head.get_angle() == pytest.approx(0.3)

    def test_returns_none_when_hub_empty_and_never_commanded(self):
        head, _, hub = _make_head(hub_angle=None)
        assert head.get_angle() is None

    def test_hub_updates_reflected_immediately(self):
        head, _, hub = _make_head(hub_angle=0.1)
        assert head.get_angle() == pytest.approx(0.1)
        hub.get.return_value = 0.5
        assert head.get_angle() == pytest.approx(0.5)

    def test_not_updated_after_failed_tilt(self):
        head, _, hub = _make_head(hub_angle=None)
        head.tilt(0.3)
        with pytest.raises(ValueError):
            head.tilt(1.0)
        hub.get.return_value = None
        # _angle should still be 0.3 from the successful tilt
        assert head.get_angle() == pytest.approx(0.3)


class TestHeadNamespace:
    def test_default_topic_has_no_prefix(self):
        head, transport, _ = _make_head()
        head.tilt(0.0)
        topic = transport.publish.call_args[0][0]
        assert topic == HEAD_TOPICS["cmd"]

    def test_namespace_setter_updates_topic(self):
        head, transport, _ = _make_head()
        head.namespace = "mybot"
        head.tilt(0.0)
        topic = transport.publish.call_args[0][0]
        assert topic == f"mybot/{HEAD_TOPICS['cmd']}"
