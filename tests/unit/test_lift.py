"""Unit tests for walkie_sdk.modules.lift.Lift."""

from unittest.mock import MagicMock, call
import pytest

from walkie_sdk.modules.lift import Lift, LIFT_MAX_CM, LIFT_DEFAULT_SPEED, LIFT_DEFAULT_ACCEL
from walkie_sdk.config.ros_topics import LIFT_TOPICS


def _approx(value: float, rel: float = 1e-6) -> pytest.approx:
    return pytest.approx(value, rel=rel)


def _make_lift(namespace: str = "") -> tuple[Lift, MagicMock]:
    transport = MagicMock()
    return Lift(transport=transport, namespace=namespace), transport


# ---------------------------------------------------------------------------
# set() — normalized mode (default)
# ---------------------------------------------------------------------------


class TestLiftSetNormalized:
    def test_midpoint_sends_half_max_cm(self):
        lift, transport = _make_lift()
        lift.set(0.5)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM * 0.5)

    def test_bottom_sends_zero(self):
        lift, transport = _make_lift()
        lift.set(0.0)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_top_sends_max_cm(self):
        lift, transport = _make_lift()
        lift.set(1.0)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_over_one_is_clamped_to_max(self):
        lift, transport = _make_lift()
        lift.set(1.5)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_negative_is_clamped_to_zero(self):
        lift, transport = _make_lift()
        lift.set(-0.5)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_default_speed_and_accel(self):
        lift, transport = _make_lift()
        lift.set(0.5)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][1] == _approx(LIFT_DEFAULT_SPEED)
        assert msg["data"][2] == _approx(LIFT_DEFAULT_ACCEL)

    def test_custom_speed_and_accel(self):
        lift, transport = _make_lift()
        lift.set(0.5, speed=5.0, accel=3.0)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][1] == _approx(5.0)
        assert msg["data"][2] == _approx(3.0)

    def test_publishes_to_correct_topic(self):
        lift, transport = _make_lift()
        lift.set(0.5)
        topic = transport.publish.call_args[0][0]
        assert topic == LIFT_TOPICS["cmd"]

    def test_publishes_correct_message_type(self):
        lift, transport = _make_lift()
        lift.set(0.5)
        msg_type = transport.publish.call_args[0][1]
        assert msg_type == LIFT_TOPICS["cmd_type"]


# ---------------------------------------------------------------------------
# set() — real-position mode
# ---------------------------------------------------------------------------


class TestLiftSetRealPos:
    def test_sends_raw_cm(self):
        lift, transport = _make_lift()
        lift.set(37.0, norm_pos=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(37.0)

    def test_zero_cm(self):
        lift, transport = _make_lift()
        lift.set(0.0, norm_pos=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_max_cm(self):
        lift, transport = _make_lift()
        lift.set(LIFT_MAX_CM, norm_pos=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_over_max_is_clamped(self):
        lift, transport = _make_lift()
        lift.set(100.0, norm_pos=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_negative_is_clamped_to_zero(self):
        lift, transport = _make_lift()
        lift.set(-10.0, norm_pos=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)


# ---------------------------------------------------------------------------
# get() — before and after subscription data
# ---------------------------------------------------------------------------


class TestLiftGet:
    def test_returns_none_before_any_data(self):
        lift, _ = _make_lift()
        assert lift.get() is None
        assert lift.get(norm_pos=False) is None

    def _inject_position_m(self, lift: Lift, pos_m: float) -> None:
        """Simulate receiving a joint state message at pos_m meters."""
        subscribe_call = lift._transport.subscribe.call_args
        if subscribe_call is None:
            # Manually trigger subscription setup
            lift._setup_state_subscription()
            subscribe_call = lift._transport.subscribe.call_args
        callback = subscribe_call[0][2]
        callback({"name": ["lift_joint"], "position": [pos_m], "velocity": [0.0], "effort": [0.0]})

    def test_get_normalized_after_data(self):
        lift, _ = _make_lift()
        lift._setup_state_subscription()
        self._inject_position_m(lift, LIFT_MAX_CM / 2 / 100)  # exact midpoint in meters
        result = lift.get(norm_pos=True)
        assert result == _approx(0.5, rel=1e-6)

    def test_get_real_pos_after_data(self):
        lift, _ = _make_lift()
        lift._setup_state_subscription()
        self._inject_position_m(lift, LIFT_MAX_CM / 2 / 100)  # exact midpoint in meters
        result = lift.get(norm_pos=False)
        assert result == _approx(LIFT_MAX_CM / 2, rel=1e-6)

    def test_get_bottom_normalized(self):
        lift, _ = _make_lift()
        lift._setup_state_subscription()
        self._inject_position_m(lift, 0.0)
        assert lift.get() == _approx(0.0)

    def test_get_top_normalized(self):
        lift, _ = _make_lift()
        lift._setup_state_subscription()
        self._inject_position_m(lift, LIFT_MAX_CM / 100.0)
        assert lift.get() == _approx(1.0)

    def test_get_default_is_normalized(self):
        lift, _ = _make_lift()
        lift._setup_state_subscription()
        self._inject_position_m(lift, 0.0)
        assert lift.get() == lift.get(norm_pos=True)


# ---------------------------------------------------------------------------
# Subscription setup
# ---------------------------------------------------------------------------


class TestLiftSubscription:
    def test_subscribes_to_correct_topic(self):
        lift, transport = _make_lift()
        lift._setup_state_subscription()
        topic = transport.subscribe.call_args[0][0]
        assert topic == LIFT_TOPICS["states"]

    def test_subscribes_with_correct_message_type(self):
        lift, transport = _make_lift()
        lift._setup_state_subscription()
        msg_type = transport.subscribe.call_args[0][1]
        assert msg_type == LIFT_TOPICS["states_type"]

    def test_subscribes_only_once(self):
        lift, transport = _make_lift()
        lift._setup_state_subscription()
        lift._setup_state_subscription()
        assert transport.subscribe.call_count == 1

    def test_namespace_prefixed_topics(self):
        lift, transport = _make_lift(namespace="robot1")
        lift._setup_state_subscription()
        lift.set(0.5)

        sub_topic = transport.subscribe.call_args[0][0]
        pub_topic = transport.publish.call_args[0][0]

        assert sub_topic == f"robot1/{LIFT_TOPICS['states']}"
        assert pub_topic == f"robot1/{LIFT_TOPICS['cmd']}"

    def test_namespace_setter_resubscribes(self):
        lift, transport = _make_lift()
        lift._setup_state_subscription()
        assert transport.subscribe.call_count == 1

        lift.namespace = "robot2"
        assert transport.subscribe.call_count == 2
        new_topic = transport.subscribe.call_args[0][0]
        assert new_topic == f"robot2/{LIFT_TOPICS['states']}"
