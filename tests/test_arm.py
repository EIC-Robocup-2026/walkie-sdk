"""
Consolidated hardware integration test for the Arm module (bot.arm).

⚠️  MOVES THE ARMS. Run with the workspace clear and an e-stop within reach.
Every motion test asks for confirmation first (type 'y'); pass --yes to skip
the prompts once you trust the setup.

Covers both control modes:
  - moveit    : commands route to MoveIt/robot action servers (my_robot_interfaces)
  - custom_ik : pose commands publish geometry_msgs/PoseStamped to an IK node topic

Mode routing of the methods under test:
  set_joint_positions ............. publishes JointState (mode-independent)
  set_joint_velocities / _torques . NOT IMPLEMENTED — must return False (no motion)
  get_joint_states ................ read-only
  control_gripper ................. action only  -> skipped in custom_ik
  go_to_home ...................... action only  -> skipped in custom_ik
  go_to_pose_relative ............. action only  -> skipped in custom_ik
  go_to_pose ...................... routes by mode
  go_to_pose_quaternion ........... routes by mode
  go_to_pose_quaternion_move_action routes by mode (MoveGroup in moveit)

Requires: rosbridge server + (moveit mode) the robot action servers, or
(custom_ik mode) an IK node subscribed to the target-pose topic, plus the
joint_states publisher.

Usage:
    python tests/test_arm.py --ip 192.168.1.100                       # moveit (default)
    python tests/test_arm.py --ip 192.168.1.100 --mode custom_ik
    python tests/test_arm.py --ip 192.168.1.100 --mode both --yes
    python tests/test_arm.py --ip 192.168.1.100 --group right_arm --gripper-group right_gripper
"""

import argparse
import sys
import time

from walkie_sdk import WalkieRobot
from walkie_sdk.modules.arm import ArmControlMode


AUTO_YES = False  # set from --yes


# ── Helpers ────────────────────────────────────────────────────────────────


def _section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _pass(msg: str) -> str:
    print(f"  [PASS] {msg}")
    return "PASS"


def _fail(msg: str) -> str:
    print(f"  [FAIL] {msg}")
    return "FAIL"


def _skip(msg: str) -> str:
    print(f"  [SKIP] {msg}")
    return "SKIP"


def _confirm(action: str) -> bool:
    if AUTO_YES:
        print(f"  [auto-yes] {action}")
        return True
    try:
        ans = input(f"  >> About to {action}. Proceed? [y/N] ").strip().lower()
    except EOFError:
        return False
    return ans in ("y", "yes")


def _ok_status(s: str) -> bool:
    return s in ("SUCCEEDED", "IN_PROGRESS")


# ── Read-only / no-motion tests ──────────────────────────────────────────────


def test_get_joint_states(bot: WalkieRobot, timeout: float) -> str:
    _section("TEST: get_joint_states() (read-only)")
    print(f"  Subscribed to '{bot.arm.arm_states_topic}', waiting up to {timeout}s ...")
    deadline = time.time() + timeout
    states = None
    while time.time() < deadline:
        states = bot.arm.get_joint_states()
        if states is not None:
            break
        time.sleep(0.1)
    if states is None:
        return _fail(f"no joint states within {timeout}s — is '{bot.arm.arm_states_topic}' publishing?")
    la = states["left_arm"]["positions"]
    ra = states["right_arm"]["positions"]
    print(f"    left_arm pos:  {[round(p, 3) for p in la]}")
    print(f"    right_arm pos: {[round(p, 3) for p in ra]}")
    print(f"    grippers: left={states['left_gripper']}  right={states['right_gripper']}")
    return _pass("joint states parsed")


def test_unimplemented_stubs(bot: WalkieRobot) -> str:
    _section("TEST: set_joint_velocities()/set_joint_torques() are no-op stubs")
    v = bot.arm.set_joint_velocities(left_arm=[0.0] * 7)
    t = bot.arm.set_joint_torques(left_arm=[0.0] * 7)
    if v is not False or t is not False:
        return _fail(f"expected both to return False, got velocities={v}, torques={t}")
    return _pass("both returned False without commanding motion")


# ── Motion tests ─────────────────────────────────────────────────────────────


def test_set_joint_positions(bot: WalkieRobot, group: str) -> str:
    _section("TEST: set_joint_positions() — small nudge of joint 1")
    states = bot.arm.get_joint_states()
    side = "left" if group.startswith("left") else "right"
    base = list(states[f"{side}_arm"]["positions"]) if states else [0.0] * 7
    target = list(base)
    target[0] = base[0] + 0.1  # nudge first joint by ~0.1 rad
    print(f"    {side}_arm joint1: {base[0]:.3f} -> {target[0]:.3f} rad")
    if not _confirm(f"nudge {side}_arm joint1 by +0.1 rad"):
        return _skip("user declined motion")
    kwargs = {f"{side}_arm": target}
    ok = bot.arm.set_joint_positions(**kwargs)
    if not ok:
        return _fail("set_joint_positions returned False")
    return _pass("joint position command published")


def test_control_gripper(bot: WalkieRobot, gripper_group: str, mode: ArmControlMode) -> str:
    _section("TEST: control_gripper() — open then close")
    if mode == ArmControlMode.CUSTOM_IK:
        return _skip("control_gripper is action-only; not used in custom_ik mode")
    if not _confirm(f"open then close gripper '{gripper_group}'"):
        return _skip("user declined motion")
    open_status = bot.arm.control_gripper(group_name=gripper_group, position=-15.71)
    print(f"    open  -> {open_status}")
    time.sleep(1.0)
    close_status = bot.arm.control_gripper(group_name=gripper_group, position=0.7)
    print(f"    close -> {close_status}")
    if not (_ok_status(open_status) and _ok_status(close_status)):
        return _fail(f"gripper actions failed: open={open_status}, close={close_status}")
    return _pass("gripper open + close succeeded")


def test_go_to_home(bot: WalkieRobot, group: str, mode: ArmControlMode) -> str:
    _section("TEST: go_to_home()")
    if mode == ArmControlMode.CUSTOM_IK:
        return _skip("go_to_home is action-only; not used in custom_ik mode")
    if not _confirm(f"move '{group}' to home"):
        return _skip("user declined motion")
    status = bot.arm.go_to_home(group_name=group)
    print(f"    go_to_home -> {status}")
    if status != "SUCCEEDED":
        return _fail(f"expected SUCCEEDED, got {status}")
    return _pass("arm reached home")


def test_go_to_pose(bot: WalkieRobot, group: str, pose, mode: ArmControlMode) -> str:
    _section(f"TEST: go_to_pose() [mode={mode.value}]")
    x, y, z, roll, pitch, yaw = pose
    print(f"    target: pos=({x:.2f},{y:.2f},{z:.2f})  rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})")
    if not _confirm(f"move '{group}' to pose ({x:.2f},{y:.2f},{z:.2f})"):
        return _skip("user declined motion")
    status = bot.arm.go_to_pose(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, group_name=group)
    print(f"    go_to_pose -> {status}")
    if not _ok_status(status):
        return _fail(f"expected SUCCEEDED/IN_PROGRESS, got {status}")
    return _pass(f"go_to_pose returned {status}")


def test_go_to_pose_relative(bot: WalkieRobot, group: str, mode: ArmControlMode) -> str:
    _section("TEST: go_to_pose_relative() — small relative move")
    if mode == ArmControlMode.CUSTOM_IK:
        return _skip("go_to_pose_relative is action-only; not used in custom_ik mode")
    if not _confirm(f"move '{group}' by relative +2cm in Z"):
        return _skip("user declined motion")
    status = bot.arm.go_to_pose_relative(x=0.0, y=0.0, z=0.02, roll=0.0, pitch=0.0, yaw=0.0,
                                         group_name=group)
    print(f"    go_to_pose_relative -> {status}")
    if not _ok_status(status):
        return _fail(f"expected SUCCEEDED/IN_PROGRESS, got {status}")
    return _pass(f"go_to_pose_relative returned {status}")


def test_go_to_pose_quaternion(bot: WalkieRobot, group: str, pose, mode: ArmControlMode) -> str:
    _section(f"TEST: go_to_pose_quaternion() [mode={mode.value}]")
    x, y, z = pose[0], pose[1], pose[2]
    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0  # identity orientation
    if not _confirm(f"move '{group}' to quat pose ({x:.2f},{y:.2f},{z:.2f})"):
        return _skip("user declined motion")
    status = bot.arm.go_to_pose_quaternion(x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw,
                                           group_name=group)
    print(f"    go_to_pose_quaternion -> {status}")
    if not _ok_status(status):
        return _fail(f"expected SUCCEEDED/IN_PROGRESS, got {status}")
    return _pass(f"go_to_pose_quaternion returned {status}")


def test_go_to_pose_quaternion_move_action(bot: WalkieRobot, group: str, pose, mode: ArmControlMode) -> str:
    _section(f"TEST: go_to_pose_quaternion_move_action() [mode={mode.value}]")
    x, y, z = pose[0], pose[1], pose[2]
    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    if not _confirm(f"move '{group}' via MoveGroup to ({x:.2f},{y:.2f},{z:.2f})"):
        return _skip("user declined motion")
    status = bot.arm.go_to_pose_quaternion_move_action(
        x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw, group_name=group)
    print(f"    move_action -> {status}")
    if not _ok_status(status):
        return _fail(f"expected SUCCEEDED/IN_PROGRESS, got {status}")
    return _pass(f"move_action returned {status}")


# ── Per-mode runner ──────────────────────────────────────────────────────────


def run_mode(bot: WalkieRobot, mode: ArmControlMode, args) -> list:
    print(f"\n{'#' * 60}")
    print(f"#  ARM MODE: {mode.value}")
    print(f"{'#' * 60}")
    bot.arm.default_mode = mode
    if mode == ArmControlMode.CUSTOM_IK:
        bot.arm.target_pose_topic = args.target_pose_topic

    pose = (args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
    results = []
    results.append(test_set_joint_positions(bot, args.group))
    results.append(test_control_gripper(bot, args.gripper_group, mode))
    results.append(test_go_to_home(bot, args.group, mode))
    results.append(test_go_to_pose(bot, args.group, pose, mode))
    results.append(test_go_to_pose_relative(bot, args.group, mode))
    results.append(test_go_to_pose_quaternion(bot, args.group, pose, mode))
    results.append(test_go_to_pose_quaternion_move_action(bot, args.group, pose, mode))
    return results


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    global AUTO_YES
    parser = argparse.ArgumentParser(description="Hardware test for Arm (bot.arm) — MOVES ARMS")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--mode", choices=["moveit", "custom_ik", "both"], default="moveit",
                        help="Arm control mode to test (default: moveit)")
    parser.add_argument("--group", default="left_arm", help="MoveIt planning group (default: left_arm)")
    parser.add_argument("--gripper-group", default="left_gripper", help="Gripper group (default: left_gripper)")
    parser.add_argument("--target-pose-topic", default="/target_pose",
                        help="Custom-IK target pose topic (default: /target_pose)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for joint states (default: 10.0)")
    # Safe-ish default pose (matches existing tests/test_call_action.py for left_arm)
    parser.add_argument("--x", type=float, default=0.38)
    parser.add_argument("--y", type=float, default=0.19)
    parser.add_argument("--z", type=float, default=0.58)
    parser.add_argument("--roll", type=float, default=-1.57)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--yes", "-y", action="store_true", help="Skip confirmation prompts")
    args = parser.parse_args()
    AUTO_YES = args.yes

    if args.mode == "both":
        modes = [ArmControlMode.MOVEIT, ArmControlMode.CUSTOM_IK]
    else:
        modes = [ArmControlMode(args.mode)]

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="none",
        namespace=args.namespace,
        arm_mode=modes[0].value,
        arm_target_pose_topic=args.target_pose_topic,
    )
    print("Connected.\n")
    print("  ⚠️  This test moves the arms. Keep the workspace clear.")

    results = []
    try:
        # Read-only checks once, up front.
        results.append(test_get_joint_states(bot, args.timeout))
        results.append(test_unimplemented_stubs(bot))
        for mode in modes:
            results.extend(run_mode(bot, mode, args))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        bot.disconnect()
        print("\nDisconnected.")

    passed = results.count("PASS")
    failed = results.count("FAIL")
    skipped = results.count("SKIP")
    total = len(results)
    print(f"\n{'=' * 60}")
    print(f"  Results: {passed} passed, {failed} failed, {skipped} skipped  (of {total})")
    print("=" * 60)
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
