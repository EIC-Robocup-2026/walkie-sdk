"""Unit tests for ArmGroup — the ergonomic sub-object on bot.arm.left / bot.arm.right."""

import pytest
from unittest.mock import MagicMock

from walkie_sdk.modules.arm import Arm, ArmGroup, _GRIPPER_MAX_M


@pytest.fixture
def arm():
    t = MagicMock()
    t.call_action.return_value = {"status": "SUCCEEDED"}
    t.call_service.return_value = {
        "success": True,
        "x": 0.0, "y": 0.0, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
        "joint_names": [], "joint_positions": [],
    }
    return Arm(t)


# ── Group attributes ──────────────────────────────────────────────────────

class TestArmGroupAttributes:
    def test_left_group_name(self, arm):
        assert arm.left.group_name == "left_arm_lift"

    def test_right_group_name(self, arm):
        assert arm.right.group_name == "right_arm_lift"

    def test_left_and_right_are_arm_group(self, arm):
        assert isinstance(arm.left, ArmGroup)
        assert isinstance(arm.right, ArmGroup)


# ── Pose delegation ───────────────────────────────────────────────────────

class TestArmGroupPoseDelegation:
    def test_go_to_pose_passes_group(self, arm):
        arm.left.go_to_pose(pos=[0.4, 0.0, 0.5], rot=[0.0, 0.0, 0.0], blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["group_name"] == "left_arm_lift"

    def test_go_to_pose_unpacks_pos(self, arm):
        arm.left.go_to_pose(pos=[0.4, 0.1, 0.5], rot=[0.0, 0.0, 0.0], blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["x"] == pytest.approx(0.4)
        assert goal["y"] == pytest.approx(0.1)
        assert goal["z"] == pytest.approx(0.5)

    def test_go_to_pose_unpacks_rot(self, arm):
        arm.left.go_to_pose(pos=[0.0, 0.0, 0.0], rot=[1.0, 2.0, 3.0], blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["roll"]  == pytest.approx(1.0)
        assert goal["pitch"] == pytest.approx(2.0)
        assert goal["yaw"]   == pytest.approx(3.0)

    def test_go_to_pose_quat_passes_group(self, arm):
        arm.right.go_to_pose_quat(pos=[0.4, 0.0, 0.5], rot=[0.0, 0.0, 0.0, 1.0], blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["group_name"] == "right_arm_lift"

    def test_go_to_pose_quat_unpacks_rot(self, arm):
        arm.left.go_to_pose_quat(pos=[0.0, 0.0, 0.0], rot=[0.1, 0.2, 0.3, 0.9], blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["qx"] == pytest.approx(0.1)
        assert goal["qy"] == pytest.approx(0.2)
        assert goal["qz"] == pytest.approx(0.3)
        assert goal["qw"] == pytest.approx(0.9)

    def test_go_to_pose_relative_passes_group_and_ee_frame(self, arm):
        arm.left.go_to_pose_relative(pos=[0, 0, 0.05], rot=[0, 0, 0], ee_frame=True, blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["group_name"] == "left_arm_lift"
        assert goal["ee_frame"] is True
        assert goal["z"] == pytest.approx(0.05)

    def test_go_to_home_passes_group(self, arm):
        arm.right.go_to_home(blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["group_name"] == "right_arm_lift"

    def test_set_joint_position_passes_group(self, arm):
        arm.left.set_joint_position([0.0] * 8, blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["group_name"] == "left_arm_lift"

    def test_kwargs_forwarded(self, arm):
        arm.left.go_to_pose(pos=[0.4, 0, 0.5], rot=[0, 0, 0], frame_id="map", blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["frame_id"] == "map"


# ── Gripper ───────────────────────────────────────────────────────────────

class TestArmGroupGripper:
    def test_normalized_open_sends_max_m(self, arm):
        arm.left.gripper(1.0, blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["group_name"] == "left_gripper"
        assert goal["position"] == pytest.approx(_GRIPPER_MAX_M)

    def test_normalized_closed_sends_zero(self, arm):
        arm.left.gripper(0.0, blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["position"] == pytest.approx(0.0)

    def test_normalized_half(self, arm):
        arm.left.gripper(0.5, blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["position"] == pytest.approx(0.5 * _GRIPPER_MAX_M)

    def test_raw_meters_bypasses_scale(self, arm):
        arm.left.gripper(0.02, norm=False, blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["position"] == pytest.approx(0.02)

    def test_right_gripper_uses_right_group(self, arm):
        arm.right.gripper(1.0, blocking=False)
        goal = arm._transport.call_action.call_args[1]["goal"]
        assert goal["group_name"] == "right_gripper"

    def test_raises_when_no_gripper_name(self, arm):
        g = ArmGroup(arm, "some_group", gripper_name=None)
        with pytest.raises(ValueError, match="no associated gripper"):
            g.gripper(0.5)


# ── Services ──────────────────────────────────────────────────────────────

class TestArmGroupServices:
    def test_get_ee_pose_passes_group(self, arm):
        arm.left.get_ee_pose()
        req = arm._transport.call_service.call_args[1]["request"]
        assert req["group_name"] == "left_arm_lift"

    def test_get_joint_states_service_passes_group(self, arm):
        arm.right.get_joint_states_service()
        req = arm._transport.call_service.call_args[1]["request"]
        assert req["group_name"] == "right_arm_lift"


# ── Plan-only & execute_stored_plan ───────────────────────────────────────

def _param_side_effect(**kwargs):
    """Return realistic responses for get/set_parameters and execute_stored_plan."""
    stype = kwargs.get("service_type", "")
    if "GetParameters" in stype:
        # Declare all params as PARAMETER_NOT_SET so _to_param_value infers type.
        empty_pv = {
            "type": 0, "bool_value": False, "integer_value": 0,
            "double_value": 0.0, "string_value": "", "byte_array_value": [],
            "bool_array_value": [], "integer_array_value": [],
            "double_array_value": [], "string_array_value": [],
        }
        names = kwargs.get("request", {}).get("names", [])
        return {"values": [empty_pv for _ in names]}
    if "SetParameters" in stype:
        return {"results": [{"successful": True, "reason": ""}]}
    # execute_stored_plan service
    return {"success": True, "message": "Executed stored plan for left_arm_lift"}


@pytest.fixture
def arm_param():
    """Arm with a transport that handles param services and execute_stored_plan."""
    t = MagicMock()
    t.call_action.return_value = {"status": "SUCCEEDED"}
    t.call_service.side_effect = lambda **kw: _param_side_effect(**kw)
    return Arm(t)


class TestSetPlanOnly:
    def test_set_plan_only_true_calls_set_parameters(self, arm_param):
        ok = arm_param.set_plan_only(True)
        assert ok is True
        calls = arm_param._transport.call_service.call_args_list
        set_calls = [c for c in calls if "SetParameters" in c[1].get("service_type", "")]
        assert len(set_calls) == 1
        params = set_calls[0][1]["request"]["parameters"]
        assert params[0]["name"] == "plan_only"
        assert params[0]["value"]["bool_value"] is True

    def test_set_plan_only_false(self, arm_param):
        ok = arm_param.set_plan_only(False)
        assert ok is True
        calls = arm_param._transport.call_service.call_args_list
        set_calls = [c for c in calls if "SetParameters" in c[1].get("service_type", "")]
        params = set_calls[0][1]["request"]["parameters"]
        assert params[0]["value"]["bool_value"] is False

    def test_arm_group_plan_only_delegates(self, arm_param):
        ok = arm_param.left.plan_only(True)
        assert ok is True
        calls = arm_param._transport.call_service.call_args_list
        set_calls = [c for c in calls if "SetParameters" in c[1].get("service_type", "")]
        params = set_calls[0][1]["request"]["parameters"]
        assert params[0]["name"] == "plan_only"


class TestExecuteStoredPlan:
    def test_execute_stored_plan_sets_group_param(self, arm_param):
        arm_param.execute_stored_plan("left_arm")
        calls = arm_param._transport.call_service.call_args_list
        set_calls = [c for c in calls if "SetParameters" in c[1].get("service_type", "")]
        assert any(
            p["name"] == "execute_plan_group" and p["value"]["string_value"] == "left_arm"
            for call in set_calls
            for p in call[1]["request"]["parameters"]
        )

    def test_execute_stored_plan_calls_trigger_service(self, arm_param):
        ok = arm_param.execute_stored_plan("left_arm")
        assert ok is True
        calls = arm_param._transport.call_service.call_args_list
        trigger_calls = [
            c for c in calls
            if "execute_stored_plan" in c[1].get("service_name", "")
            and "Trigger" in c[1].get("service_type", "")
        ]
        assert len(trigger_calls) == 1
        assert trigger_calls[0][1]["request"] == {}

    def test_arm_group_execute_stored_plan_passes_group(self, arm_param):
        arm_param.left.execute_stored_plan()
        calls = arm_param._transport.call_service.call_args_list
        set_calls = [c for c in calls if "SetParameters" in c[1].get("service_type", "")]
        assert any(
            p["name"] == "execute_plan_group"
            and p["value"]["string_value"] == arm_param.left.group_name
            for call in set_calls
            for p in call[1]["request"]["parameters"]
        )

    def test_execute_stored_plan_returns_false_on_service_failure(self, arm_param):
        def fail_trigger(**kwargs):
            if "execute_stored_plan" in kwargs.get("service_name", "") and "Trigger" in kwargs.get("service_type", ""):
                return {"success": False, "message": "No stored plan"}
            return _param_side_effect(**kwargs)

        arm_param._transport.call_service.side_effect = lambda **kw: fail_trigger(**kw)
        ok = arm_param.execute_stored_plan("left_arm")
        assert ok is False

    def test_execute_stored_plan_returns_false_when_param_set_fails(self, arm_param):
        def fail_set(**kwargs):
            if "SetParameters" in kwargs.get("service_type", ""):
                return {"results": [{"successful": False, "reason": "rejected"}]}
            return _param_side_effect(**kwargs)

        arm_param._transport.call_service.side_effect = lambda **kw: fail_set(**kw)
        ok = arm_param.execute_stored_plan("left_arm")
        assert ok is False
