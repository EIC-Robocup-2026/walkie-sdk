"""
Interactive hardware test for JointStateHub against a live robot.

Verifies that the centralized joint_states subscription works end-to-end:
all modules (Arm, Lift, Head) read real sensor data through bot.joints
instead of maintaining their own subscriptions.

Menu:
  1  Live monitor        — continuously print all joints (Ctrl+C to stop)
  2  Query joint         — look up one joint by name
  3  Sanity: head tilt   — command a tilt angle, verify readback via hub
  4  Sanity: lift        — compare bot.lift.get() vs bot.joints.get("lift_joint")
  5  Sanity: arm         — compare bot.arm.get_joint_states() vs hub raw values
  6  Wait for first data — block until hub receives its first message
  q  Quit

Usage:
    python tests/test_joint_state_hub_interactive.py
    python tests/test_joint_state_hub_interactive.py --ip 192.168.1.100
    python tests/test_joint_state_hub_interactive.py --ip 192.168.1.100 --port 9090
    python tests/test_joint_state_hub_interactive.py --ip 192.168.1.100 --namespace robot1
"""

import argparse
import math
import os
import time

from walkie_sdk import WalkieRobot
from walkie_sdk.modules.lift import LIFT_MAX_CM


# ── Terminal helpers ───────────────────────────────────────────────────────

def _clear():
    os.system("clear" if os.name == "posix" else "cls")


def _section(title: str) -> None:
    print(f"\n{'=' * 62}")
    print(f"  {title}")
    print("=" * 62)


def _pass(msg: str) -> None:
    print(f"  [PASS] {msg}")


def _fail(msg: str) -> None:
    print(f"  [FAIL] {msg}")


def _warn(msg: str) -> None:
    print(f"  [WARN] {msg}")


def _prompt_float(prompt: str, default: float) -> float:
    raw = input(f"  {prompt} [{default}]: ").strip()
    return float(raw) if raw else default


# ── Joint table formatter ──────────────────────────────────────────────────

_COL_NAME = 32
_COL_VAL  = 10


def _joint_table(all_joints: dict) -> str:
    if not all_joints:
        return "  (no data yet)"
    header = (
        f"  {'Joint':<{_COL_NAME}} {'pos (rad/m)':>{_COL_VAL}}  "
        f"{'vel':>{_COL_VAL}}  {'effort':>{_COL_VAL}}"
    )
    sep = "  " + "-" * (_COL_NAME + _COL_VAL * 3 + 6)
    rows = [header, sep]
    for name in sorted(all_joints):
        d = all_joints[name]
        rows.append(
            f"  {name:<{_COL_NAME}} {d['position']:>{_COL_VAL}.4f}  "
            f"{d['velocity']:>{_COL_VAL}.4f}  {d['effort']:>{_COL_VAL}.4f}"
        )
    return "\n".join(rows)


# ── Menu actions ───────────────────────────────────────────────────────────

def do_live_monitor(bot: WalkieRobot) -> None:
    """Continuously refresh joint table until Ctrl+C."""
    print("\n  Live monitor — press Ctrl+C to stop\n")
    try:
        while True:
            all_joints = bot.joints.get_all()
            _clear()
            print(f"  Joint State Hub — Live Monitor   ({time.strftime('%H:%M:%S')})")
            print(f"  Joints in hub: {len(all_joints)}")
            print()
            print(_joint_table(all_joints))
            print()
            print("  Ctrl+C to return to menu")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n  (stopped)")


def do_query_joint(bot: WalkieRobot) -> None:
    """Look up a single joint by name."""
    _section("Query Joint")
    name = input("  Joint name: ").strip()
    if not name:
        print("  (cancelled)")
        return

    pos    = bot.joints.get(name)
    vel    = bot.joints.get_velocity(name)
    effort = bot.joints.get_effort(name)

    if pos is None:
        _fail(f"'{name}' not found in hub (no data or wrong name)")
        all_joints = bot.joints.get_all()
        if all_joints:
            print(f"  Available joints: {', '.join(sorted(all_joints))}")
    else:
        print(f"  {name}:")
        print(f"    position : {pos:.6f} rad/m  ({math.degrees(pos):.2f} deg)")
        print(f"    velocity : {vel:.6f}")
        print(f"    effort   : {effort:.6f}")


def do_sanity_head(bot: WalkieRobot) -> None:
    """Command a head tilt and verify the hub reports it back."""
    _section("Sanity Check: Head Tilt")

    joint_name = "head_servo_joint"

    before = bot.joints.get(joint_name)
    print(f"  Hub value before command: {before}")

    angle = _prompt_float("Tilt angle in radians (+ve = down, range ±0.785)", 0.3)
    try:
        bot.head.tilt(angle)
    except ValueError as e:
        _fail(str(e))
        return

    print(f"  Command sent: {angle:.3f} rad — waiting 0.5 s for servo to respond ...")
    time.sleep(0.5)

    hub_val  = bot.joints.get(joint_name)
    head_val = bot.head.get_angle()

    print(f"  bot.joints.get('{joint_name}') = {hub_val}")
    print(f"  bot.head.get_angle()           = {head_val}")

    if hub_val is None:
        _fail("Hub has no data for head_servo_joint — is joint_states publishing?")
        return

    tolerance = 0.05  # rad
    if abs(hub_val - angle) <= tolerance:
        _pass(f"Hub position {hub_val:.3f} rad matches command {angle:.3f} rad (±{tolerance})")
    else:
        _warn(
            f"Hub position {hub_val:.3f} rad differs from command {angle:.3f} rad "
            f"by {abs(hub_val - angle):.3f} rad — servo may still be moving"
        )

    if hub_val == head_val:
        _pass("bot.head.get_angle() returns the hub value (not the commanded value)")
    else:
        _warn(
            f"bot.head.get_angle() = {head_val:.3f} — expected {hub_val:.3f} "
            f"(may be the fallback commanded value if hub data is stale)"
        )

    # Reset to forward
    reset = input("\n  Reset to 0.0 rad? [Y/n]: ").strip().lower()
    if reset != "n":
        bot.head.tilt(0.0)
        print("  Reset command sent.")


def do_sanity_lift(bot: WalkieRobot) -> None:
    """Compare bot.lift.get() with the raw hub value for lift_joint."""
    _section("Sanity Check: Lift")

    hub_m    = bot.joints.get("lift_joint")
    lift_pos = bot.lift.get(norm_pos=True)
    lift_cm  = bot.lift.get(norm_pos=False)

    print(f"  bot.joints.get('lift_joint') = {hub_m} m (raw)")
    print(f"  bot.lift.get(norm_pos=False) = {lift_cm} cm")
    print(f"  bot.lift.get(norm_pos=True)  = {lift_pos} (normalized)")

    if hub_m is None:
        _fail("Hub has no data for lift_joint — is joint_states publishing?")
        return

    hub_cm = hub_m * 100.0
    if lift_cm is None:
        _fail("bot.lift.get() returned None — hub data not reaching Lift module")
        return

    if abs(hub_cm - lift_cm) < 0.01:
        _pass(f"Lift.get() ({lift_cm:.4f} cm) matches hub ({hub_cm:.4f} cm)")
    else:
        _fail(
            f"Mismatch: Lift.get() = {lift_cm:.4f} cm, "
            f"hub = {hub_cm:.4f} cm (diff {abs(hub_cm - lift_cm):.4f} cm)"
        )


def do_sanity_arm(bot: WalkieRobot) -> None:
    """Compare bot.arm.get_joint_states() with hub raw values for arm joints."""
    _section("Sanity Check: Arm")

    arm_data = bot.arm.get_joint_states()
    all_joints = bot.joints.get_all()

    if not all_joints:
        _fail("Hub has no data yet")
        return

    if arm_data is None:
        _fail("bot.arm.get_joint_states() returned None")
        return

    passed = 0
    checked = 0

    for side, prefix in [("left_arm", "openarm_left_joint"), ("right_arm", "openarm_right_joint")]:
        positions = arm_data[side]["positions"]
        print(f"\n  {side} ({len(positions)} joints):")
        print(f"    {'Joint':<30} {'arm.get()':>10}  {'hub.get()':>10}  {'match':>6}")
        print(f"    {'-'*58}")
        for i, pos in enumerate(positions, 1):
            joint_name = f"{prefix}{i}"
            hub_val = all_joints.get(joint_name, {}).get("position")
            match = hub_val is not None and abs(pos - hub_val) < 1e-4
            checked += 1
            if match:
                passed += 1
            mark = "OK" if match else "DIFF"
            hub_str = f"{hub_val:.4f}" if hub_val is not None else "N/A"
            print(f"    {joint_name:<30} {pos:>10.4f}  {hub_str:>10}  {mark:>6}")

    print()
    if passed == checked:
        _pass(f"All {checked} arm joints match between Arm module and hub")
    else:
        _fail(f"{checked - passed}/{checked} arm joints differ — check parsing logic")


def do_wait_for_data(bot: WalkieRobot) -> None:
    """Block until the hub receives its first message."""
    _section("Wait for First Data")
    timeout = _prompt_float("Timeout (s)", 10.0)

    print(f"  Waiting up to {timeout:.0f} s for joint_states ...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if bot.joints.get_all():
            elapsed = timeout - (deadline - time.time())
            _pass(f"First data arrived in {elapsed:.2f} s")
            joints = bot.joints.get_all()
            print(f"  Joints received: {len(joints)}")
            print(f"  Names: {', '.join(sorted(joints))}")
            return
        time.sleep(0.05)

    _fail(f"No data after {timeout:.0f} s — is joint_states topic active?")


# ── Main menu ──────────────────────────────────────────────────────────────

_MENU = """
┌─────────────────────────────────────────────────────────┐
│         Walkie Joint State Hub — Hardware Test          │
├──────┬──────────────────────────────────────────────────┤
│  1   │ Live monitor        (all joints, live refresh)   │
│  2   │ Query joint         (look up one joint by name)  │
│  3   │ Sanity: head tilt   (command + hub readback)     │
│  4   │ Sanity: lift        (Lift.get() vs hub)          │
│  5   │ Sanity: arm         (Arm.get_joint_states() vs hub)│
│  6   │ Wait for first data                              │
│  q   │ Quit                                             │
└──────┴──────────────────────────────────────────────────┘
"""

_ACTIONS = {
    "1": do_live_monitor,
    "2": do_query_joint,
    "3": do_sanity_head,
    "4": do_sanity_lift,
    "5": do_sanity_arm,
    "6": do_wait_for_data,
}


def main():
    parser = argparse.ArgumentParser(
        description="Interactive hardware test for JointStateHub"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int, default=9090,      help="Rosbridge port (default: 9090)")
    parser.add_argument("--protocol",  default="rosbridge",         help="ROS protocol (default: rosbridge)")
    parser.add_argument("--namespace", default="",                  help="ROS namespace (default: none)")
    parser.add_argument("--no-camera", action="store_true",         help="Disable camera (faster connect)")
    args = parser.parse_args()

    print(f"\nConnecting to {args.ip}:{args.port} (protocol={args.protocol}) ...")
    bot = WalkieRobot(
        ip=args.ip,
        ros_protocol=args.protocol,
        ros_port=args.port,
        camera_protocol="none" if args.no_camera else "zenoh",
        namespace=args.namespace,
    )
    print("Connected.")
    print(f"  Namespace : '{args.namespace or '(none)'}'")

    # Quick initial data wait
    print("  Waiting up to 5 s for first joint_states message ...")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if bot.joints.get_all():
            n = len(bot.joints.get_all())
            print(f"  Hub ready — {n} joints received.")
            break
        time.sleep(0.05)
    else:
        print("  [WARN] No joint_states received yet — hub may be empty.")

    try:
        while True:
            print(_MENU)
            choice = input("Choice: ").strip().lower()

            if choice in ("q", "quit", "exit"):
                break

            action = _ACTIONS.get(choice)
            if action is None:
                print(f"  Unknown choice '{choice}'")
                continue

            try:
                action(bot)
            except KeyboardInterrupt:
                print("\n  (interrupted — back to menu)")
            except Exception as e:
                print(f"  Error: {e}")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        bot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
