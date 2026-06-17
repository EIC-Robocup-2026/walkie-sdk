"""
Hardware integration test for the Telemetry module (bot.status) against a live robot.

Read-only — never commands motion. Verifies odometry is flowing and that
get_position()/get_velocity()/get_raw_odom() return well-formed data.

Requires a running rosbridge server publishing the odom topic:
  current_pose   nav_msgs/msg/Odometry   (configurable via TELEMETRY_TOPICS / WALKIE_TELEMETRY_ODOM)

Usage:
    python tests/test_telemetry.py
    python tests/test_telemetry.py --ip 192.168.1.100
    python tests/test_telemetry.py --ip 192.168.1.100 --port 9090 --timeout 10.0
    python tests/test_telemetry.py --namespace robot1 --samples 20
"""

import argparse
import sys
import time

from walkie_sdk import WalkieRobot


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


def _wait_for_data(bot: WalkieRobot, timeout: float) -> bool:
    """Block until the first odom message arrives or timeout expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if bot.status.has_data:
            return True
        time.sleep(0.1)
    return False


# ── Test cases ─────────────────────────────────────────────────────────────


def test_wait_for_first_message(bot: WalkieRobot, timeout: float) -> str:
    _section("TEST 1: receive first odometry message")
    print(f"  Subscribed to '{bot.status.odom_topic}', waiting up to {timeout}s ...")
    if not _wait_for_data(bot, timeout):
        return _fail(f"no odom within {timeout}s — is '{bot.status.odom_topic}' publishing?")
    return _pass("odometry received (has_data == True)")


def test_get_position(bot: WalkieRobot) -> str:
    _section("TEST 2: get_position()")
    pose = bot.status.get_position()
    if pose is None:
        return _fail("get_position() returned None")
    missing = [k for k in ("x", "y", "heading") if k not in pose]
    if missing:
        return _fail(f"missing keys {missing} in {pose}")
    return _pass(f"x={pose['x']:.3f}  y={pose['y']:.3f}  heading={pose['heading']:.3f} rad")


def test_get_velocity(bot: WalkieRobot) -> str:
    _section("TEST 3: get_velocity()")
    vel = bot.status.get_velocity()
    if vel is None:
        return _fail("get_velocity() returned None (does odom include twist?)")
    missing = [k for k in ("linear", "angular") if k not in vel]
    if missing:
        return _fail(f"missing keys {missing} in {vel}")
    return _pass(f"linear={vel['linear']:.3f} m/s  angular={vel['angular']:.3f} rad/s")


def test_get_raw_odom(bot: WalkieRobot) -> str:
    _section("TEST 4: get_raw_odom()")
    raw = bot.status.get_raw_odom()
    if not isinstance(raw, dict) or "pose" not in raw:
        return _fail(f"raw odom missing 'pose': {type(raw)}")
    return _pass(f"raw odom keys: {sorted(raw.keys())}")


def test_streaming(bot: WalkieRobot, samples: int) -> str:
    _section(f"TEST 5: stream {samples} live samples (~10 Hz)")
    seen = 0
    for _ in range(samples):
        pose = bot.status.get_position()
        vel = bot.status.get_velocity()
        if pose is not None:
            seen += 1
            v = vel["linear"] if vel else float("nan")
            print(f"    x={pose['x']:+.3f}  y={pose['y']:+.3f}  "
                  f"heading={pose['heading']:+.3f}  v={v:+.3f}", end="\r")
        time.sleep(0.1)
    print()
    if seen == 0:
        return _fail("no samples observed during streaming window")
    return _pass(f"{seen}/{samples} samples had position data")


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="Hardware test for Telemetry (bot.status)")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for first odom (default: 10.0)")
    parser.add_argument("--samples", type=int, default=20, help="Live samples to stream in test 5 (default: 20)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ip=args.ip,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.\n")

    results = []
    try:
        first = test_wait_for_first_message(bot, args.timeout)
        results.append(first)
        if first == "PASS":
            results.append(test_get_position(bot))
            results.append(test_get_velocity(bot))
            results.append(test_get_raw_odom(bot))
            results.append(test_streaming(bot, args.samples))
        else:
            print("\n  Skipping remaining tests — no telemetry data.")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        bot.disconnect()
        print("\nDisconnected.")

    passed = results.count("PASS")
    total = len(results)
    print(f"\n{'=' * 60}")
    print(f"  Results: {passed}/{total} passed")
    print("=" * 60)
    sys.exit(0 if passed == total and total > 0 else 1)


if __name__ == "__main__":
    main()
