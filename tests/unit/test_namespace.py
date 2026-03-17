"""Unit tests for walkie_sdk.utils.namespace."""

from walkie_sdk.utils.namespace import apply_namespace


class TestApplyNamespace:
    def test_no_namespace(self):
        assert apply_namespace("odom", "") == "odom"

    def test_with_namespace(self):
        assert apply_namespace("odom", "robot1") == "robot1/odom"

    def test_namespace_trailing_slash_stripped(self):
        assert apply_namespace("cmd_vel", "robot1/") == "robot1/cmd_vel"

    def test_namespace_leading_slash_stripped(self):
        assert apply_namespace("cmd_vel", "/robot1") == "robot1/cmd_vel"

    def test_namespace_both_slashes_stripped(self):
        assert apply_namespace("cmd_vel", "/robot1/") == "robot1/cmd_vel"

    def test_topic_without_leading_slash(self):
        """Topics should NOT have leading slashes (fix #10 verified)."""
        result = apply_namespace("joint_states", "walkie")
        assert "//" not in result
        assert result == "walkie/joint_states"

    def test_empty_name_with_namespace(self):
        assert apply_namespace("", "robot1") == "robot1/"

    def test_empty_name_no_namespace(self):
        assert apply_namespace("", "") == ""

    def test_nested_namespace(self):
        assert apply_namespace("odom", "floor2/robot1") == "floor2/robot1/odom"
