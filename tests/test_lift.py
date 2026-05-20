"""
Hardware integration test for Lift module against a live robot or simulator.

Requires a running rosbridge server with the lift topics active:
  /lift/cmd           std_msgs/msg/Float64MultiArray
  /lift/joint_states  sensor_msgs/msg/JointState

Usage:
    python tests/test_lift.py
    python tests/test_lift.py --ip 192.168.1.100
    python tests/test_lift.py --ip 192.168.1.100 --port 9090 --timeout 15.0
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


def _wait_until_reached(
    bot: WalkieRobot,
    target_norm: float,
    tolerance: float,
    timeout: float,
) -> bool:
    """
    Poll lift position until it is within `tolerance` of `target_norm`
    (normalized 0–1). Returns True if reached, False on timeout.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        pos = bot.lift.get(norm_pos=True)
        if pos is not None:
            elapsed = timeout - (deadline - time.time())
            print(f"    pos={pos:.4f}  target={target_norm:.4f}  err={abs(pos - target_norm):.4f}  t={elapsed:.1f}s", end="\r")
            if abs(pos - target_norm) <= tolerance:
                print()  # newline after \r progress
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


def test_move_to_bottom(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 2: move to bottom (norm=0.0)")
    print(f"  Sending set(0.0), waiting up to {timeout}s to reach target ...")
    bot.lift.set(0.0)
    reached = _wait_until_reached(bot, 0.0, tolerance, timeout)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Final position: norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if not reached:
        _fail(f"did not reach bottom within {timeout}s (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"reached bottom — norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    return True


def test_move_to_top(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 3: move to top (norm=1.0)")
    print(f"  Sending set(1.0), waiting up to {timeout}s to reach target ...")
    bot.lift.set(1.0)
    reached = _wait_until_reached(bot, 1.0, tolerance, timeout)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Final position: norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if not reached:
        _fail(f"did not reach top within {timeout}s (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"reached top — norm={pos_norm:.4f}  real={pos_cm:.2f} cm  (max={LIFT_MAX_CM} cm)")
    return True


def test_move_to_midpoint(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 4: move to midpoint (norm=0.5)")
    print(f"  Sending set(0.5), waiting up to {timeout}s to reach target ...")
    bot.lift.set(0.5)
    reached = _wait_until_reached(bot, 0.5, tolerance, timeout)
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm = bot.lift.get(norm_pos=False)
    print(f"  Final position: norm={pos_norm:.4f}  real={pos_cm:.2f} cm")
    if not reached:
        _fail(f"did not reach midpoint within {timeout}s (pos={pos_norm:.4f}, tolerance={tolerance})")
        return False
    _pass(f"reached midpoint — norm={pos_norm:.4f}  real={pos_cm:.2f} cm  (target={LIFT_MAX_CM / 2:.2f} cm)")
    return True


def test_real_position_mode(bot: WalkieRobot, timeout: float, tolerance: float) -> bool:
    _section("TEST 5: real-position mode (norm_pos=False, target=37.0 cm)")
    target_cm = 37.0
    target_norm = target_cm / LIFT_MAX_CM
    print(f"  Sending set({target_cm}, norm_pos=False), waiting up to {timeout}s ...")
    bot.lift.set(target_cm, norm_pos=False)
    reached = _wait_until_reached(bot, target_norm, tolerance, timeout)
    pos_cm = bot.lift.get(norm_pos=False)
    pos_norm = bot.lift.get(norm_pos=True)
    print(f"  Final position: real={pos_cm:.2f} cm  norm={pos_norm:.4f}")
    if not reached:
        _fail(f"did not reach {target_cm} cm within {timeout}s (pos={pos_cm:.2f} cm, tolerance={tolerance * LIFT_MAX_CM:.2f} cm)")
        return False
    _pass(f"reached target — real={pos_cm:.2f} cm  norm={pos_norm:.4f}  (target={target_cm} cm)")
    return True


def test_custom_speed_accel(bot: WalkieRobot) -> bool:
    _section("TEST 6: custom speed and accel (set(0.5, speed=5.0, accel=2.0))")
    try:
        bot.lift.set(0.5, speed=5.0, accel=2.0)
        _pass("command sent without error")
        return True
    except Exception as e:
        _fail(f"exception raised: {e}")
        return False


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Hardware integration test for Lift module"
    )
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Max seconds to wait for each move (default: 60.0)")
    parser.add_argument("--tolerance", type=float, default=0.02, help="Normalized position tolerance for 'reached' check (default: 0.02 ≈ 1.5 cm)")
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
    print(f"  tolerance = {args.tolerance} norm ({args.tolerance * LIFT_MAX_CM:.2f} cm)")

    results = []
    try:
        results.append(test_read_initial_position(bot, args.timeout))
        results.append(test_move_to_bottom(bot, args.timeout, args.tolerance))
        results.append(test_move_to_top(bot, args.timeout, args.tolerance))
        results.append(test_move_to_midpoint(bot, args.timeout, args.tolerance))
        results.append(test_real_position_mode(bot, args.timeout, args.tolerance))
        results.append(test_custom_speed_accel(bot))
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
