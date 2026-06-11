"""
Hardware integration test for navigate_to_object (bot.nav.go_to without heading).

Exercises the /navigate_to_object action on nav_commander, which computes a
PCA edge-fit approach pose so the robot faces the object directly without the
caller needing to specify a heading.

⚠️  MOVES THE ROBOT BASE. Run only with a clear floor and an e-stop within reach.
Every motion test asks for confirmation; pass --yes to skip prompts.

Requires: rosbridge + Nav2 + nav_commander with the /navigate_to_object action.

Usage:
    python tests/test_navigation_interactive.py --obj-x 1.5 --obj-y -3.0
    python tests/test_navigation_interactive.py --obj-x 1.5 --obj-y -3.0 --standoff 0.5
    python tests/test_navigation_interactive.py --obj-x 1.5 --obj-y -3.0 --yes
    python tests/test_navigation_interactive.py --ip 192.168.1.100 --obj-x 1.5 --obj-y -3.0
"""

import argparse
import sys
import time

from walkie_sdk import WalkieRobot


AUTO_YES = False


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


# ── Test cases ─────────────────────────────────────────────────────────────


def test_blocking_nav_to_object(bot: WalkieRobot, obj_x: float, obj_y: float, standoff: float, timeout: float) -> str:
    _section("TEST 1: blocking go_to() without heading → navigate_to_object")
    print(f"  Object (map frame): x={obj_x:.3f}  y={obj_y:.3f}  standoff={standoff:.2f} m")
    if not _confirm(f"navigate to object at x={obj_x:.2f}, y={obj_y:.2f}"):
        return _skip("user declined motion")

    result = bot.nav.go_to(x=obj_x, y=obj_y, standoff=standoff, blocking=True, timeout=timeout)
    print(f"  Result: {result}")
    print(f"  Status: {bot.nav.status}")
    print(f"  Method: {bot.nav.nav_error_msg}")
    if result not in ("SUCCEEDED", "CLOSE_ENOUGH"):
        return _fail(f"expected SUCCEEDED/CLOSE_ENOUGH, got {result}")
    return _pass(f"{result} — align method used: {bot.nav.nav_error_msg}")


def test_face_target_align(bot: WalkieRobot, obj_x: float, obj_y: float, standoff: float, timeout: float) -> str:
    _section("TEST 2: navigate_to_object with align_method='face_target' (skip edge fit)")
    print(f"  Object (map frame): x={obj_x:.3f}  y={obj_y:.3f}")
    if not _confirm(f"navigate to object at x={obj_x:.2f}, y={obj_y:.2f} with face_target"):
        return _skip("user declined motion")

    result = bot.nav.go_to(x=obj_x, y=obj_y, standoff=standoff, align_method="face_target", blocking=True, timeout=timeout)
    print(f"  Result: {result}  |  Method: {bot.nav.nav_error_msg}")
    if result not in ("SUCCEEDED", "CLOSE_ENOUGH"):
        return _fail(f"expected SUCCEEDED/CLOSE_ENOUGH, got {result}")
    return _pass(f"{result}")


def test_nonblocking_then_cancel(bot: WalkieRobot, obj_x: float, obj_y: float, timeout: float) -> str:
    _section("TEST 3: non-blocking navigate_to_object then cancel()")
    if not _confirm(f"navigate (non-blocking) toward x={obj_x:.2f}, y={obj_y:.2f}, then cancel"):
        return _skip("user declined motion")

    result = bot.nav.go_to(x=obj_x, y=obj_y, blocking=False)
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


def test_feedback_callback(bot: WalkieRobot, obj_x: float, obj_y: float, standoff: float, timeout: float) -> str:
    _section("TEST 4: navigate_to_object with feedback_callback")
    if not _confirm(f"navigate to object at x={obj_x:.2f}, y={obj_y:.2f} with feedback"):
        return _skip("user declined motion")

    count = {"n": 0}

    def on_feedback(fb: dict) -> None:
        count["n"] += 1
        if count["n"] <= 3:
            dist = fb.get("feedback", fb).get("distance_remaining", "?")
            print(f"    [feedback #{count['n']}] distance_remaining={dist}")

    result = bot.nav.go_to(x=obj_x, y=obj_y, standoff=standoff, blocking=True,
                           timeout=timeout, feedback_callback=on_feedback)
    print(f"  Result: {result}  |  feedback messages: {count['n']}")
    if result not in ("SUCCEEDED", "CLOSE_ENOUGH"):
        return _fail(f"navigation did not succeed: {result}")
    return _pass(f"{result} with {count['n']} feedback message(s)")


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    global AUTO_YES
    parser = argparse.ArgumentParser(
        description="Hardware test for navigate_to_object (go_to without heading) — MOVES BASE"
    )
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Max seconds per blocking move (default: 60.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--obj-x", type=float, required=True, help="Object X position in map frame")
    parser.add_argument("--obj-y", type=float, required=True, help="Object Y position in map frame")
    parser.add_argument("--standoff", type=float, default=0.0, help="Standoff override in metres (0=nav_commander default)")
    parser.add_argument("--yes", "-y", action="store_true", help="Skip confirmation prompts")
    args = parser.parse_args()
    AUTO_YES = args.yes

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.\n")
    print("  ⚠️  This test moves the robot base. Keep the area clear.")
    print(f"  Object position: x={args.obj_x}  y={args.obj_y}  standoff={args.standoff} m")

    results = []
    try:
        results.append(test_blocking_nav_to_object(bot, args.obj_x, args.obj_y, args.standoff, args.timeout))
        results.append(test_face_target_align(bot, args.obj_x, args.obj_y, args.standoff, args.timeout))
        results.append(test_nonblocking_then_cancel(bot, args.obj_x, args.obj_y, args.timeout))
        results.append(test_feedback_callback(bot, args.obj_x, args.obj_y, args.standoff, args.timeout))
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
