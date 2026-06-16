"""
Hardware integration test for the Navigation module (bot.nav) against a live robot.

⚠️  MOVES THE ROBOT BASE. Run only with a clear floor and an e-stop within reach.
Every test that commands motion asks for confirmation first (type 'y'); pass
--yes to skip the prompts (e.g. for repeat runs once you trust the setup).

By default the goal is the robot's *current* pose plus an optional small offset
(--dx/--dy, both default 0.0), so the base barely moves while still exercising
the full Nav2 action handshake. Pass explicit --goal-x/--goal-y/--goal-heading
to send an absolute map-frame goal instead.

NOTE: go_to() sends the goal in the "map" frame, while bot.status reads odometry
(frame configurable, often "odom"). If your map and odom frames differ, prefer
explicit --goal-* coordinates.

Requires: rosbridge server + Nav2 'navigate_to_pose' action. Telemetry
(current_pose) is used to derive the default goal.

Usage:
    python tests/test_navigation.py --ip 192.168.1.100
    python tests/test_navigation.py --ip 192.168.1.100 --dx 0.3                 # nudge 30cm
    python tests/test_navigation.py --goal-x 2.0 --goal-y 1.0 --goal-heading 0.0
    python tests/test_navigation.py --goal-obj-x 1.5 --goal-obj-y -3.0          # navigate_to_object
    python tests/test_navigation.py --ip 192.168.1.100 --yes                    # no prompts
"""

import argparse
import math
import sys
import time

from walkie_sdk import WalkieRobot


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
    """Ask before any motion. Returns True to proceed."""
    if AUTO_YES:
        print(f"  [auto-yes] {action}")
        return True
    try:
        ans = input(f"  >> About to {action}. Proceed? [y/N] ").strip().lower()
    except EOFError:
        return False
    return ans in ("y", "yes")


def _wait_for_pose(bot: WalkieRobot, timeout: float):
    print(f"  Waiting up to {timeout:.0f}s for telemetry...", end="", flush=True)
    deadline = time.time() + timeout
    while time.time() < deadline:
        pose = bot.status.get_position()
        if pose is not None:
            print(" got it.")
            return pose
        time.sleep(0.1)
    print(" timed out.")
    return None


def _resolve_goal(bot: WalkieRobot, args):
    """Return (x, y, heading) goal, or None if it cannot be determined."""
    if args.goal_x is not None and args.goal_y is not None:
        heading = args.goal_heading if args.goal_heading is not None else 0.0
        return (args.goal_x, args.goal_y, heading)
    pose = _wait_for_pose(bot, args.timeout)
    if pose is None:
        return None
    heading = args.goal_heading if args.goal_heading is not None else pose["heading"]
    return (pose["x"] + args.dx, pose["y"] + args.dy, heading)


# ── Test cases ─────────────────────────────────────────────────────────────


def test_initial_status(bot: WalkieRobot) -> str:
    _section("TEST 1: initial status (no motion)")
    status = bot.nav.status
    nav = bot.nav.is_navigating
    if status is not None:
        return _fail(f"expected status None before any goal, got '{status}'")
    if nav:
        return _fail("is_navigating should be False before any goal")
    return _pass("status is None and is_navigating is False")


def test_blocking_go_to(bot: WalkieRobot, goal, timeout: float) -> str:
    _section("TEST 2: blocking go_to()")
    x, y, h = goal
    print(f"  Goal (map frame): x={x:.3f}  y={y:.3f}  heading={h:.3f} rad")
    if not _confirm(f"navigate (blocking) to x={x:.2f}, y={y:.2f}"):
        return _skip("user declined motion")
    result = bot.nav.go_to(x=x, y=y, heading=h, blocking=True, timeout=timeout)
    print(f"  Result: {result}  |  final status: {bot.nav.status}")
    if result != "SUCCEEDED":
        return _fail(f"expected SUCCEEDED, got {result}")
    return _pass(f"navigation SUCCEEDED")


def test_nonblocking_then_cancel(bot: WalkieRobot, goal, timeout: float) -> str:
    _section("TEST 3: non-blocking go_to() then cancel()")
    x, y, h = goal
    if not _confirm(f"navigate (non-blocking) toward x={x:.2f}, y={y:.2f}, then cancel"):
        return _skip("user declined motion")
    result = bot.nav.go_to(x=x, y=y, heading=h, blocking=False)
    print(f"  Immediate return: {result}")
    if result != "IN_PROGRESS":
        return _fail(f"expected IN_PROGRESS, got {result}")
    if not bot.nav.is_navigating:
        return _fail("is_navigating should be True right after non-blocking go_to")
    time.sleep(2.0)
    ok = bot.nav.cancel()
    print(f"  cancel() -> {ok}  |  status: {bot.nav.status}")
    if not ok:
        return _fail("cancel() returned False")
    return _pass("non-blocking goal accepted and cancelled")


def test_emergency_stop(bot: WalkieRobot, goal) -> str:
    _section("TEST 4: emergency stop()")
    x, y, h = goal
    if not _confirm(f"navigate (non-blocking) toward x={x:.2f}, y={y:.2f}, then STOP"):
        return _skip("user declined motion")
    bot.nav.go_to(x=x, y=y, heading=h, blocking=False)
    time.sleep(1.5)
    ok = bot.nav.stop()
    print(f"  stop() -> {ok}  |  status: {bot.nav.status}")
    if not ok:
        return _fail("stop() returned False")
    return _pass("emergency stop sent (zero cmd_vel + cancel)")


def test_navigate_to_object(bot: WalkieRobot, obj_x: float, obj_y: float, standoff: float, timeout: float) -> str:
    _section("TEST 6: navigate_to_object (no heading)")
    print(f"  Object position (map frame): x={obj_x:.3f}  y={obj_y:.3f}  standoff={standoff:.2f} m")
    if not _confirm(f"navigate to object at x={obj_x:.2f}, y={obj_y:.2f} via nav_commander"):
        return _skip("user declined motion")
    result = bot.nav.go_to(x=obj_x, y=obj_y, standoff=standoff, blocking=True, timeout=timeout)
    print(f"  Result: {result}  |  final status: {bot.nav.status}  |  message: {bot.nav.nav_error_msg}")
    if result not in ("SUCCEEDED", "CLOSE_ENOUGH"):
        return _fail(f"expected SUCCEEDED/CLOSE_ENOUGH, got {result}")
    return _pass(f"{result} — method: {bot.nav.nav_error_msg}")


def test_feedback_callback(bot: WalkieRobot, goal, timeout: float) -> str:
    _section("TEST 5: blocking go_to() with feedback_callback")
    x, y, h = goal
    if not _confirm(f"navigate (blocking) to x={x:.2f}, y={y:.2f} with feedback"):
        return _skip("user declined motion")

    count = {"n": 0}

    def on_feedback(fb: dict) -> None:
        count["n"] += 1
        if count["n"] <= 3:
            print(f"    [feedback #{count['n']}] keys={list(fb.keys())}")

    result = bot.nav.go_to(x=x, y=y, heading=h, blocking=True,
                           timeout=timeout, feedback_callback=on_feedback)
    print(f"  Result: {result}  |  feedback messages: {count['n']}")
    if result != "SUCCEEDED":
        return _fail(f"navigation did not succeed: {result}")
    # Feedback count may be 0 for a near-zero move; plumbing is what we verify.
    return _pass(f"SUCCEEDED with {count['n']} feedback message(s)")


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    global AUTO_YES
    parser = argparse.ArgumentParser(description="Hardware test for Navigation (bot.nav) — MOVES BASE")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Max seconds per blocking move (default: 60.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--dx", type=float, default=0.0, help="X offset from current pose for default goal (m)")
    parser.add_argument("--dy", type=float, default=0.0, help="Y offset from current pose for default goal (m)")
    parser.add_argument("--goal-x", type=float, default=None, help="Absolute map-frame goal X (overrides --dx)")
    parser.add_argument("--goal-y", type=float, default=None, help="Absolute map-frame goal Y (overrides --dy)")
    parser.add_argument("--goal-heading", type=float, default=None, help="Goal heading in radians")
    parser.add_argument("--goal-obj-x", type=float, default=None, help="Object X for navigate_to_object test (no heading)")
    parser.add_argument("--goal-obj-y", type=float, default=None, help="Object Y for navigate_to_object test (no heading)")
    parser.add_argument("--standoff", type=float, default=0.0, help="Standoff override for navigate_to_object (m, 0=default)")
    parser.add_argument("--yes", "-y", action="store_true", help="Skip confirmation prompts")
    args = parser.parse_args()
    AUTO_YES = args.yes

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ip=args.ip,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.\n")
    print("  ⚠️  This test moves the robot base. Keep the area clear.")

    results = []
    try:
        results.append(test_initial_status(bot))

        # navigate_to_object test (no heading required)
        if args.goal_obj_x is not None and args.goal_obj_y is not None:
            results.append(test_navigate_to_object(bot, args.goal_obj_x, args.goal_obj_y, args.standoff, args.timeout))

        # navigate_to_pose tests (heading required; derive goal from telemetry or explicit coords)
        goal = _resolve_goal(bot, args)
        if goal is None:
            print("\n  Could not determine a nav2 goal: no telemetry and no --goal-x/--goal-y given.")
            print("  Tip: pass --goal-obj-x/--goal-obj-y to test navigate_to_object without telemetry.")
            results.append(_fail("goal resolution failed — navigate_to_pose tests skipped"))
        else:
            results.append(test_blocking_go_to(bot, goal, args.timeout))
            results.append(test_nonblocking_then_cancel(bot, goal, args.timeout))
            results.append(test_emergency_stop(bot, goal))
            results.append(test_feedback_callback(bot, goal, args.timeout))
    except KeyboardInterrupt:
        print("\nInterrupted by user — sending stop().")
        try:
            bot.nav.stop()
        except Exception:
            pass
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
