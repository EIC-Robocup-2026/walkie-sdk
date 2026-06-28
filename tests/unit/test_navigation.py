"""Unit tests for walkie_sdk.modules.navigation.Navigation."""

import math
from unittest.mock import MagicMock, call
import pytest

from walkie_sdk.modules.navigation import Navigation
from walkie_sdk.config.ros_topics import NAV_ACTIONS, NAV_TOPICS


def _make_nav(namespace: str = "") -> tuple[Navigation, MagicMock]:
    transport = MagicMock()
    transport.is_connected = True
    transport.call_action.return_value = {"result": {}, "status": "SUCCEEDED"}
    return Navigation(transport=transport, namespace=namespace), transport


# ---------------------------------------------------------------------------
# go_to with heading — Nav2 navigate_to_pose path
# ---------------------------------------------------------------------------


class TestGoToWithHeading:
    def test_calls_navigate_to_pose_action(self):
        nav, transport = _make_nav()
        nav.go_to(1.0, 2.0, heading=0.0)
        action_name = transport.call_action.call_args[1]["action_name"]
        action_type = transport.call_action.call_args[1]["action_type"]
        assert action_name == "navigate_to_pose"
        assert action_type == NAV_ACTIONS["navigate_to_pose_type"]

    def test_goal_contains_pose_structure(self):
        nav, transport = _make_nav()
        nav.go_to(3.0, -1.5, heading=math.pi / 2)
        goal = transport.call_action.call_args[1]["goal"]
        pos = goal["pose"]["pose"]["position"]
        assert pos["x"] == pytest.approx(3.0)
        assert pos["y"] == pytest.approx(-1.5)
        assert pos["z"] == pytest.approx(0.0)

    def test_heading_encoded_as_quaternion(self):
        nav, transport = _make_nav()
        nav.go_to(0.0, 0.0, heading=0.0)
        goal = transport.call_action.call_args[1]["goal"]
        orient = goal["pose"]["pose"]["orientation"]
        # heading=0 → qw=1, qz=0
        assert orient["w"] == pytest.approx(1.0, abs=1e-6)
        assert orient["z"] == pytest.approx(0.0, abs=1e-6)

    def test_namespace_prefixes_action_name(self):
        nav, transport = _make_nav(namespace="robot1")
        nav.go_to(0.0, 0.0, heading=0.0)
        action_name = transport.call_action.call_args[1]["action_name"]
        assert action_name == "robot1/navigate_to_pose"

    def test_returns_succeeded(self):
        nav, transport = _make_nav()
        result = nav.go_to(1.0, 1.0, heading=0.0)
        assert result == "SUCCEEDED"

    def test_returns_failed_on_bad_status(self):
        nav, transport = _make_nav()
        transport.call_action.return_value = {"result": {}, "status": "FAILED"}
        result = nav.go_to(1.0, 1.0, heading=0.0)
        assert result == "FAILED"


# ---------------------------------------------------------------------------
# go_to without heading — navigate_to_object path
# ---------------------------------------------------------------------------


class TestGoToWithoutHeading:
    def test_calls_navigate_to_object_action(self):
        nav, transport = _make_nav()
        nav.go_to(1.0, 2.0)
        action_name = transport.call_action.call_args[1]["action_name"]
        action_type = transport.call_action.call_args[1]["action_type"]
        assert action_name == "navigate_to_object"
        assert action_type == NAV_ACTIONS["navigate_to_object_type"]

    def test_goal_keys_are_obj_x_obj_y(self):
        nav, transport = _make_nav()
        nav.go_to(3.5, -2.0)
        goal = transport.call_action.call_args[1]["goal"]
        assert goal["obj_x"] == pytest.approx(3.5)
        assert goal["obj_y"] == pytest.approx(-2.0)

    def test_default_standoff_and_align_method(self):
        nav, transport = _make_nav()
        nav.go_to(1.0, 2.0)
        goal = transport.call_action.call_args[1]["goal"]
        assert goal["standoff"] == pytest.approx(0.0)
        assert goal["align_method"] == ""

    def test_standoff_forwarded(self):
        nav, transport = _make_nav()
        nav.go_to(1.0, 2.0, standoff=0.5)
        goal = transport.call_action.call_args[1]["goal"]
        assert goal["standoff"] == pytest.approx(0.5)

    def test_align_method_forwarded(self):
        nav, transport = _make_nav()
        nav.go_to(1.0, 2.0, align_method="face_target")
        goal = transport.call_action.call_args[1]["goal"]
        assert goal["align_method"] == "face_target"

    def test_namespace_prefixes_action_name(self):
        nav, transport = _make_nav(namespace="robot1")
        nav.go_to(0.0, 0.0)
        action_name = transport.call_action.call_args[1]["action_name"]
        assert action_name == "robot1/navigate_to_object"

    def test_returns_succeeded(self):
        nav, transport = _make_nav()
        transport.call_action.return_value = {
            "result": {"success": True, "message": "nearest_edge"},
            "status": "SUCCEEDED",
        }
        result = nav.go_to(1.0, 2.0)
        assert result == "SUCCEEDED"

    def test_message_stored_in_nav_error_msg(self):
        nav, transport = _make_nav()
        transport.call_action.return_value = {
            "result": {"success": True, "message": "face_target"},
            "status": "SUCCEEDED",
        }
        nav.go_to(1.0, 2.0)
        assert nav.nav_error_msg == "face_target"

    def test_standoff_and_align_method_ignored_when_heading_given(self):
        """standoff/align_method must not appear in the Nav2 goal."""
        nav, transport = _make_nav()
        nav.go_to(1.0, 2.0, heading=0.0, standoff=0.5, align_method="face_target")
        goal = transport.call_action.call_args[1]["goal"]
        assert "standoff" not in goal
        assert "align_method" not in goal


# ---------------------------------------------------------------------------
# Non-blocking path
# ---------------------------------------------------------------------------


class TestGoToNonBlocking:
    def test_returns_in_progress_immediately(self):
        nav, transport = _make_nav()
        # make call_action block forever — should not matter for non-blocking
        import threading
        barrier = threading.Barrier(2)

        def _slow_action(**_kwargs):
            barrier.wait(timeout=2)
            return {"result": {}, "status": "SUCCEEDED"}

        transport.call_action.side_effect = _slow_action
        result = nav.go_to(1.0, 2.0, blocking=False)
        assert result == "IN_PROGRESS"
        barrier.wait(timeout=2)  # unblock the background thread


# ---------------------------------------------------------------------------
# Connection guard
# ---------------------------------------------------------------------------


class TestConnectionGuard:
    def test_raises_when_not_connected(self):
        nav, transport = _make_nav()
        transport.is_connected = False
        with pytest.raises(ConnectionError):
            nav.go_to(1.0, 2.0)


# ---------------------------------------------------------------------------
# set_velocity / stop — direct cmd_vel publish (TwistStamped)
# ---------------------------------------------------------------------------


def _published(transport) -> tuple[str, str, dict]:
    """Return (topic, message_type, message) of the last transport.publish call."""
    args = transport.publish.call_args
    # publish(topic, message_type, message) — positional in Navigation
    return args[0][0], args[0][1], args[0][2]


class TestSetVelocity:
    def test_publishes_twiststamped(self):
        nav, transport = _make_nav()
        result = nav.set_velocity(0.5, -0.2, 0.3)
        assert result is True

        topic, msg_type, msg = _published(transport)
        assert topic == "cmd_vel"
        assert msg_type == NAV_TOPICS["cmd_vel_type"] == "geometry_msgs/msg/TwistStamped"
        assert msg["header"]["frame_id"] == "base_link"
        twist = msg["twist"]
        assert twist["linear"]["x"] == pytest.approx(0.5)
        assert twist["linear"]["y"] == pytest.approx(-0.2)
        assert twist["linear"]["z"] == pytest.approx(0.0)
        assert twist["angular"]["x"] == pytest.approx(0.0)
        assert twist["angular"]["y"] == pytest.approx(0.0)
        assert twist["angular"]["z"] == pytest.approx(0.3)

    def test_respects_namespace(self):
        nav, transport = _make_nav(namespace="robot1")
        nav.set_velocity(0.1, 0.0, 0.0)
        topic, _, _ = _published(transport)
        assert topic == "robot1/cmd_vel"

    def test_not_connected_returns_false(self):
        nav, transport = _make_nav()
        transport.is_connected = False
        result = nav.set_velocity(0.5, 0.0, 0.0)
        assert result is False
        transport.publish.assert_not_called()


class TestStop:
    def test_publishes_zero_twiststamped(self):
        nav, transport = _make_nav()
        result = nav.stop()
        assert result is True

        topic, msg_type, msg = _published(transport)
        assert topic == "cmd_vel"
        assert msg_type == "geometry_msgs/msg/TwistStamped"
        assert msg["header"]["frame_id"] == "base_link"
        twist = msg["twist"]
        assert twist["linear"] == {"x": 0.0, "y": 0.0, "z": 0.0}
        assert twist["angular"] == {"x": 0.0, "y": 0.0, "z": 0.0}
        transport.cancel_action.assert_called_once()
