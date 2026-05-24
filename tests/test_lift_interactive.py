"""
Interactive hardware test for Lift module against a live robot or simulator.

Lets you type target positions at runtime — useful for real-life testing where
you want to manually drive the lift to arbitrary positions and observe behaviour.

Requires a running rosbridge server with the lift topics active:
  /lift/cmd           std_msgs/msg/Float64MultiArray
  /lift/joint_states  sensor_msgs/msg/JointState

Usage:
    python tests/test_lift_interactive.py
    python tests/test_lift_interactive.py --ip 192.168.1.100
    python tests/test_lift_interactive.py --ip 192.168.1.100 --port 9090 --timeout 60.0
    python tests/test_lift_interactive.py --namespace robot1 --tolerance 0.02

At the prompt type:
    0.5          → move to norm 0.5  (halfway)
    1.0          → move to norm 1.0  (top)
    37.0 cm      → move to 37.0 cm  (real position)
    0.8 norm     → move to norm 0.8  (explicit unit)
    status       → print current position and status
    q / quit     → exit
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


def _print_status(bot: WalkieRobot) -> None:
    pos_norm = bot.lift.get(norm_pos=True)
    pos_cm   = bot.lift.get(norm_pos=False)
    status   = bot.lift.status
    norm_str = f"{pos_norm:.4f}" if pos_norm is not None else "N/A"
    cm_str   = f"{pos_cm:.2f} cm" if pos_cm is not None else "N/A"
    print(f"  status={status}  norm={norm_str}  real={cm_str}")


def _parse_target(raw: str) -> tuple[float, bool] | None:
    """
    Parse a user input string into (value, norm_pos).
    Returns None if the input is invalid.
    """
    parts = raw.split()
    if not parts:
        return None

    try:
        value = float(parts[0])
    except ValueError:
        return None

    norm_pos = True  # default unit
    if len(parts) >= 2:
        unit = parts[1].lower()
        if unit in ("cm", "real"):
            norm_pos = False
        elif unit in ("norm", "normalized"):
            norm_pos = True
        else:
            print(f"  Unknown unit {unit!r}. Use 'norm' or 'cm'.")
            return None

    # Range validation
    if norm_pos and not (0.0 <= value <= 1.0):
        print(f"  Norm position must be in [0.0, 1.0], got {value}.")
        return None
    if not norm_pos and not (0.0 <= value <= LIFT_MAX_CM):
        print(f"  Real position must be in [0.0, {LIFT_MAX_CM}] cm, got {value}.")
        return None

    return value, norm_pos


# ── Interactive session ────────────────────────────────────────────────────


def run_interactive_session(bot: WalkieRobot, timeout: float, tolerance: float) -> tuple[int, int]:
    """
    Prompt the user for lift targets until they quit.
    Returns (passed_moves, attempted_moves).
    """
    _section("Interactive Lift Control")
    print(f"  LIFT_MAX_CM = {LIFT_MAX_CM} cm   |   tolerance = {tolerance} norm ({tolerance * LIFT_MAX_CM:.2f} cm)")
    print()
    print("  Commands:")
    print("    <value>              move to norm position  (0.0 – 1.0)")
    print("    <value> norm         move to norm position  (0.0 – 1.0)")
    print("    <value> cm           move to real position  (0.0 – {:.1f} cm)".format(LIFT_MAX_CM))
    print("    status               print current position and status")
    print("    q / quit             exit interactive session")
    print()

    passed = 0
    attempted = 0

    while True:
        try:
            raw = input("  lift> ").strip()
        except (KeyboardInterrupt, EOFError):
            print()
            break

        if not raw:
            continue

        if raw.lower() in ("q", "quit", "exit"):
            break

        if raw.lower() in ("status", "s"):
            _print_status(bot)
            continue

        parsed = _parse_target(raw)
        if parsed is None:
            print(f"  Invalid input: {raw!r}")
            print("  Try: 0.5   |   37.0 cm   |   0.8 norm   |   status   |   q")
            continue

        value, norm_pos = parsed
        unit_str = "norm" if norm_pos else "cm"
        print(f"  → Moving to {value} {unit_str}  (blocking=True, timeout={timeout}s) ...")
        attempted += 1

        result = bot.lift.set(
            value,
            norm_pos=norm_pos,
            blocking=True,
            timeout=timeout,
            tolerance=tolerance,
        )

        pos_norm = bot.lift.get(norm_pos=True)
        pos_cm   = bot.lift.get(norm_pos=False)
        norm_str = f"{pos_norm:.4f}" if pos_norm is not None else "N/A"
        cm_str   = f"{pos_cm:.2f}" if pos_cm is not None else "N/A"
        print(f"  Result: {result}  |  pos norm={norm_str}  real={cm_str} cm")

        if result == "SUCCEEDED":
            _pass(f"SUCCEEDED — norm={norm_str}  real={cm_str} cm")
            passed += 1
        else:
            _fail(f"Expected SUCCEEDED, got {result}")

    return passed, attempted


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Interactive hardware test for Lift module"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int,   default=9090,  help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout",   type=float, default=60.0,  help="Max seconds to wait per move (default: 60.0)")
    parser.add_argument("--tolerance", type=float, default=0.02,  help="Normalized position tolerance (default: 0.02 ≈ 1.5 cm)")
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
    print("Connected.")
    print(f"  timeout   = {args.timeout}s")
    print(f"  tolerance = {args.tolerance} norm ({args.tolerance * LIFT_MAX_CM:.2f} cm)")

    passed = attempted = 0
    try:
        print("\nWaiting for first joint state ...")
        if not _wait_for_data(bot, timeout=10.0):
            print("  [WARN] No joint state received within 10 s — is lift/joint_states publishing?")
        else:
            _print_status(bot)

        passed, attempted = run_interactive_session(bot, args.timeout, args.tolerance)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        bot.disconnect()
        print("\nDisconnected.")

    print(f"\n{'=' * 60}")
    if attempted == 0:
        print("  No moves attempted.")
    else:
        print(f"  Moves: {passed}/{attempted} SUCCEEDED")
    print("=" * 60)
    sys.exit(0 if (attempted == 0 or passed == attempted) else 1)


if __name__ == "__main__":
    main()
