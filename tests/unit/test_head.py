"""Unit tests for walkie_sdk.modules.head.Head."""

import math
from unittest.mock import MagicMock

import pytest

from walkie_sdk.modules.head import Head, HEAD_TILT_MIN, HEAD_TILT_MAX
from walkie_sdk.config.ros_topics import HEAD_TOPICS


def _make_head(namespace: str = "") -> tuple[Head, MagicMock]:
    transport = MagicMock()
    return Head(transport=transport, namespace=namespace), transport


class TestHeadTilt:
    def test_valid_angle_publishes_correct_payload(self):
        head, transport = _make_head()
        head.tilt(0.5)
        topic, msg_type, msg = transport.publish.call_args[0]
        assert msg == {"data": [0.5]}
        assert msg_type == HEAD_TOPICS["cmd_type"]

    def test_zero_angle_publishes_zero(self):
        head, transport = _make_head()
        head.tilt(0.0)
        msg = transport.publish.call_args[0][2]
        assert msg == {"data": [0.0]}

    def test_min_boundary_is_accepted(self):
        head, transport = _make_head()
        head.tilt(HEAD_TILT_MIN)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == pytest.approx(HEAD_TILT_MIN)

    def test_max_boundary_is_accepted(self):
        head, transport = _make_head()
        head.tilt(HEAD_TILT_MAX)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == pytest.approx(HEAD_TILT_MAX)

    def test_above_max_raises_value_error(self):
        head, _ = _make_head()
        with pytest.raises(ValueError, match="out of range"):
            head.tilt(HEAD_TILT_MAX + 0.001)

    def test_below_min_raises_value_error(self):
        head, _ = _make_head()
        with pytest.raises(ValueError, match="out of range"):
            head.tilt(HEAD_TILT_MIN - 0.001)

    def test_out_of_range_does_not_publish(self):
        head, transport = _make_head()
        with pytest.raises(ValueError):
            head.tilt(1.0)
        transport.publish.assert_not_called()

    def test_topic_uses_namespace(self):
        head, transport = _make_head(namespace="robot1")
        head.tilt(0.3)
        topic = transport.publish.call_args[0][0]
        assert topic.startswith("robot1/")


class TestHeadGetAngle:
    def test_returns_none_before_first_tilt(self):
        head, _ = _make_head()
        assert head.get_angle() is None

    def test_returns_last_commanded_angle(self):
        head, _ = _make_head()
        head.tilt(0.3)
        assert head.get_angle() == pytest.approx(0.3)

    def test_updates_after_second_tilt(self):
        head, _ = _make_head()
        head.tilt(0.1)
        head.tilt(-0.2)
        assert head.get_angle() == pytest.approx(-0.2)

    def test_not_updated_after_failed_tilt(self):
        head, _ = _make_head()
        head.tilt(0.3)
        with pytest.raises(ValueError):
            head.tilt(1.0)
        assert head.get_angle() == pytest.approx(0.3)


class TestHeadNamespace:
    def test_default_topic_has_no_prefix(self):
        head, transport = _make_head()
        head.tilt(0.0)
        topic = transport.publish.call_args[0][0]
        assert topic == HEAD_TOPICS["cmd"]

    def test_namespace_setter_updates_topic(self):
        head, transport = _make_head()
        head.namespace = "mybot"
        head.tilt(0.0)
        topic = transport.publish.call_args[0][0]
        assert topic == f"mybot/{HEAD_TOPICS['cmd']}"
