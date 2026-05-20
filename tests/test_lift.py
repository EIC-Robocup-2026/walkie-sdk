"""
Hardware integration test for Lift module against a live robot or simulator.

Requires a running rosbridge server with the lift topics active:
  /lift/cmd           std_msgs/msg/Float64MultiArray
  /lift/joint_states  sensor_msgs/msg/JointState

Usage:
    python tests/test_lift.py
    python tests/test_lift.py --ip 192.168.1.100
    python tests/test_lift.py --ip 192.168.1.100 --port 9090 --timeout 60.0
    python tests/test_lift.py --namespace robot1 --tolerance 0.02
"""

import argparse
import sys
import time

from walkie_sdk import WalkieRobot
from walkie_sdk.modules.lift import LIFT_MAX_CM


# ── Helpers ────────────────────────────────────────────────────────────────


def _section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _pass(msg: str) -> None:
    print(f"  [PASS] {msg}")


def _fail(msg: str) -> None:
    print(f"  [FAIL] {msg}")


def _wait_for_data(bot: WalkieRobot, timeout: float) -> bool:
    """Block until the first joint state arrives or timeout expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if bot.lift.get() is not None:
            return True
        time.sleep(0.1)
    return False


def _wait_for_status(bot: WalkieRobot, expected: str, timeout: float) -> bool:
    """Poll bot.lift.status until it matches expected, printing live position."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        pos = bot.lift.get(norm_pos=True)
        status = bot.lift.status
        elapsed = timeout - (deadline - time.time())
        pos_str = f"{pos:.4f}" if pos is not None else "N/A"
        print(f"    status={status}  pos={pos_str}  t={elapsed:.1f}s", end="\r")
        if status == expected:
            print()
            return True
        time.sleep(0.05)
    print()
    return False


# ── Test cases ─────────────────────────────────────────────────────────────


def test_read_initial_position(bot: WalkieRobot, timeout: float) -> bool:
    _section("TEST 1: read initial lift position")
    print(f"  Waiting up to {timeout}s for first joint state ...")
    if not _wait_for_data(bot, timeout):
        _fail(f"no data received within {timeout}s — is lift/joint_states publishing?")
        return False
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    _pass(f"norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    return True


def test_blocking_move_to_bottom(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 2: blocking — move to bottom (norm=0.0)")
    print(f"  Calling set(0.0, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(0.0, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Result: {result}  |  pos norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result} (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"SUCCEEDED — norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    return True


def test_blocking_move_to_top(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 3: blocking — move to top (norm=1.0)")
    print(f"  Calling set(1.0, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(1.0, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Result: {result}  |  pos norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result} (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"SUCCEEDED — norm={pos_norm:.4f}  real={pos_cm:.2f} cm  (max={LIFT_MAX_CM} cm)")
    return True


def test_blocking_move_to_midpoint(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 4: blocking — move to midpoint (norm=0.5)")
    print(f"  Calling set(0.5, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(0.5, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Result: {result}  |  pos norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result} (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"SUCCEEDED — norm={pos_norm:.4f}  real={pos_cm:.2f} cm  (target={LIFT_MAX_CM / 2:.2f} cm)")
    return True


def test_blocking_real_position_mode(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 5: blocking — real-position mode (target=37.0 cm)")
    target_cm = 37.0
    print(f"  Calling set({target_cm}, norm_pos=False, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(target_cm, norm_pos=False, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_cm = bot.lift.get(norm_pos=False)
    pos_norm = bot.lift.get(norm_pos=True)
    print(f"  Result: {result}  |  pos real={pos_cm:.2f} cm  norm={pos_norm:.4f}")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result} (pos={pos_cm:.2f} cm, tolerance={tolerance * LIFT_MAX_CM:.2f} cm)")
        return False
    _pass(f"SUCCEEDED — real={pos_cm:.2f} cm  norm={pos_norm:.4f}  (target={target_cm} cm)")
    return True


def test_non_blocking_move(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 6: non-blocking — set(0.0, blocking=False) then poll status")
    print("  Calling set(0.0, blocking=False) ...")
    result = bot.lift.set(0.0, blocking=False, timeout=timeout, tolerance=tolerance)
    print(f"  Immediate return: {result}")
    if result != "IN_PROGRESS":
        _fail(f"expected IN_PROGRESS immediately, got {result}")
        return False
    if not bot.lift.is_moving:
        _fail("is_moving should be True right after non-blocking set()")
        return False

    print(f"  Polling bot.lift.status until SUCCEEDED (max {timeout}s) ...")
    reached = _wait_for_status(bot, "SUCCEEDED", timeout)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Final status: {bot.lift.status}  |  pos norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if not reached:
        _fail(f"status never became SUCCEEDED within {timeout}s (last status={bot.lift.status})")
        return False
    _pass(f"status=SUCCEEDED — norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    return True


def test_custom_speed_accel(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 7: blocking — custom speed and accel (set(0.5, speed=5.0, accel=2.0))")
    print(f"  Calling set(0.5, speed=5.0, accel=2.0, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(0.5, speed=5.0, accel=2.0, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_norm = bot.lift.get(norm_pos=True)
    print(f"  Result: {result}  |  pos norm={pos_norm:.4f}")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result}")
        return False
    _pass(f"SUCCEEDED — norm={pos_norm:.4f}")
    return True


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Hardware integration test for Lift module"
    )
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Max seconds to wait for each move (default: 60.0)")
    parser.add_argument("--tolerance", type=float, default=0.02, help="Normalized position tolerance (default: 0.02 ≈ 1.5 cm)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.\n")
    print(f"  timeout   = {args.timeout}s")
    print(f"  tolerance = {args.tolerance} norm ({args.tolerance * LIFT_MAX_CM:.2f} cm)")

    results = []
    try:
        results.append(test_read_initial_position(bot, args.timeout))
        results.append(test_blocking_move_to_bottom(bot, args.timeout, args.tolerance))
        results.append(test_blocking_move_to_top(bot, args.timeout, args.tolerance))
        results.append(test_blocking_move_to_midpoint(bot, args.timeout, args.tolerance))
        results.append(test_blocking_real_position_mode(bot, args.timeout, args.tolerance))
        results.append(test_non_blocking_move(bot, args.timeout, args.tolerance))
        results.append(test_custom_speed_accel(bot, args.timeout, args.tolerance))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        bot.disconnect()
        print("\nDisconnected.")

    passed = sum(results)
    total = len(results)
    print(f"\n{'=' * 60}")
    print(f"  Results: {passed}/{total} passed")
    print("=" * 60)
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
