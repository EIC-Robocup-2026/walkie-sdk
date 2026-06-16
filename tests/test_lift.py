"""
Hardware integration test for Lift module against a live robot or simulator.

Position convention: 0.0 = bottom, 1.0 = top.

⚠️  SAFETY: this robot currently cannot travel below the midpoint, so this test
NEVER commands a position lower than --min-pos (default 0.5). Every target is
clamped to [min_pos, 1.0] before being sent, and the lift is expected to start
near the top (~1.0). When the hardware limit is lifted, pass --min-pos 0.0 to
exercise the full range again.

Requires a running rosbridge server with the lift topics active:
  /lift/cmd           std_msgs/msg/Float64MultiArray
  /lift/joint_states  sensor_msgs/msg/JointState

Usage:
    python tests/test_lift.py
    python tests/test_lift.py --ip 192.168.1.100
    python tests/test_lift.py --ip 192.168.1.100 --port 9090 --timeout 60.0
    python tests/test_lift.py --namespace robot1 --tolerance 0.02
    python tests/test_lift.py --ip 192.168.1.100 --min-pos 0.6     # raise the floor
    python tests/test_lift.py --ip 192.168.1.100 --min-pos 0.0     # full range (limit removed)
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


def _guard_norm(pos: float, min_pos: float) -> float:
    """Clamp a normalized target to [min_pos, 1.0]. Safety net against driving too low."""
    safe = min(1.0, max(min_pos, pos))
    if safe != pos:
        print(f"    [guard] requested norm {pos:.3f} clamped to {safe:.3f} (floor={min_pos})")
    return safe


def _guard_cm(cm: float, min_pos: float) -> float:
    """Clamp a real-cm target to [min_pos*MAX, MAX]. Safety net against driving too low."""
    min_cm = min_pos * LIFT_MAX_CM
    safe = min(LIFT_MAX_CM, max(min_cm, cm))
    if safe != cm:
        print(f"    [guard] requested {cm:.2f} cm clamped to {safe:.2f} cm (floor={min_cm:.2f} cm)")
    return safe


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
    _section("TEST 1: read initial lift position (expected near top ~1.0)")
    print(f"  Waiting up to {timeout}s for first joint state ...")
    if not _wait_for_data(bot, timeout):
        _fail(f"no data received within {timeout}s — is lift/joint_states publishing?")
        return False
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    _pass(f"norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if pos_norm < 0.9:
        print(f"  [note] initial position {pos_norm:.4f} is not near the top (1.0) as expected.")
    return True


def test_blocking_move_to_mid(bot: WalkieRobot, timeout: float, tolerance: float, min_pos: float) -> bool:
    target = _guard_norm(round((min_pos + 1.0) / 2, 4), min_pos)
    _section(f"TEST 2: blocking — move down to mid of allowed range (norm={target})")
    print(f"  Calling set({target}, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(target, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Result: {result}  |  pos norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result} (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"SUCCEEDED — norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    return True


def test_blocking_move_to_floor(bot: WalkieRobot, timeout: float, tolerance: float, min_pos: float) -> bool:
    target = _guard_norm(min_pos, min_pos)
    _section(f"TEST 3: blocking — move to lowest allowed position (norm={target})")
    print(f"  Calling set({target}, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(target, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Result: {result}  |  pos norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result} (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"SUCCEEDED — norm={pos_norm:.4f}  real={pos_cm:.2f} cm  (floor={min_pos})")
    return True


def test_blocking_move_to_top(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 4: blocking — move to top (norm=1.0)")
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


def test_blocking_real_position_mode(bot: WalkieRobot, timeout: float, tolerance: float, min_pos: float) -> bool:
    # Pick a real-cm target inside the allowed band (~0.6 of full travel, clamped to floor).
    target_cm = _guard_cm(round(max(0.6, min_pos) * LIFT_MAX_CM, 1), min_pos)
    _section(f"TEST 5: blocking — real-position mode (target={target_cm} cm)")
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


def test_non_blocking_move(bot: WalkieRobot, timeout: float, tolerance: float, min_pos: float) -> bool:
    # Move back up to the top so the motion is clearly visible after the cm-mode move.
    target = _guard_norm(1.0, min_pos)
    _section(f"TEST 6: non-blocking — set({target}, blocking=False) then poll status")
    print(f"  Calling set({target}, blocking=False) ...")
    result = bot.lift.set(target, blocking=False, timeout=timeout, tolerance=tolerance)
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


def test_custom_speed_accel(bot: WalkieRobot, timeout: float, tolerance: float, min_pos: float) -> bool:
    target = _guard_norm(round((min_pos + 1.0) / 2, 4), min_pos)
    _section(f"TEST 7: blocking — custom speed and accel (set({target}, speed=5.0, accel=2.0))")
    print(f"  Calling set({target}, speed=5.0, accel=2.0, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(target, speed=5.0, accel=2.0, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_norm = bot.lift.get(norm_pos=True)
    print(f"  Result: {result}  |  pos norm={pos_norm:.4f}")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result}")
        return False
    _pass(f"SUCCEEDED — norm={pos_norm:.4f}")
    return True


def test_park_at_top(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 8: park — return to top (norm=1.0) as the final state")
    print(f"  Calling set(1.0, blocking=True, timeout={timeout}s) ...")
    result = bot.lift.set(1.0, blocking=True, timeout=timeout, tolerance=tolerance)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Result: {result}  |  pos norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if result != "SUCCEEDED":
        _fail(f"expected SUCCEEDED, got {result} (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"parked at top — norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    return True


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Hardware integration test for Lift module (never commands below --min-pos)"
    )
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Max seconds to wait for each move (default: 60.0)")
    parser.add_argument("--tolerance", type=float, default=0.02, help="Normalized position tolerance (default: 0.02 ≈ 1.5 cm)")
    parser.add_argument("--min-pos", type=float, default=0.5, dest="min_pos",
                        help="Lowest normalized position the test is allowed to command (default: 0.5)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    args = parser.parse_args()

    # Keep the floor itself sane.
    args.min_pos = min(1.0, max(0.0, args.min_pos))

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ip=args.ip,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.\n")
    print(f"  timeout   = {args.timeout}s")
    print(f"  tolerance = {args.tolerance} norm ({args.tolerance * LIFT_MAX_CM:.2f} cm)")
    print(f"  min_pos   = {args.min_pos} norm ({args.min_pos * LIFT_MAX_CM:.2f} cm)  ← lift is never sent below this")

    results = []
    try:
        results.append(test_read_initial_position(bot, args.timeout))
        results.append(test_blocking_move_to_mid(bot, args.timeout, args.tolerance, args.min_pos))
        results.append(test_blocking_move_to_floor(bot, args.timeout, args.tolerance, args.min_pos))
        results.append(test_blocking_move_to_top(bot, args.timeout, args.tolerance))
        results.append(test_blocking_real_position_mode(bot, args.timeout, args.tolerance, args.min_pos))
        results.append(test_non_blocking_move(bot, args.timeout, args.tolerance, args.min_pos))
        results.append(test_custom_speed_accel(bot, args.timeout, args.tolerance, args.min_pos))
        results.append(test_park_at_top(bot, args.timeout, args.tolerance))
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
