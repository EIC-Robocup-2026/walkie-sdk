"""
Interactive arm tester using the ergonomic sub-object API (bot.arm.left / bot.arm.right).

Controls (main menu):
  l  switch to LEFT arm  (bot.arm.left)
  r  switch to RIGHT arm (bot.arm.right)
  1  go_to_pose          — absolute EEF pose (Euler RPY)
  2  go_to_pose_relative — relative EEF displacement
  3  go_to_pose_quat     — absolute EEF pose (quaternion)
  4  go_to_home          — named home position
  5  gripper             — open / close (normalised 0.0–1.0)
  6  set_joint_position  — direct joint angles (commander or JTC)
  7  get_ee_pose         — query current EEF pose (service)
  8  get_joint_states    — query joint positions (service)
  9  get_joint_states    — read from topic subscription (all joints)
  q  quit

Usage:
    python tests/test_arm_larm_interactive.py
    python tests/test_arm_larm_interactive.py --ip 192.168.1.100
    python tests/test_arm_larm_interactive.py --ip 192.168.1.100 --port 9090
    python tests/test_arm_larm_interactive.py --ip 192.168.1.100 --namespace robot1
"""

import argparse
import math

from walkie_sdk import WalkieRobot
from walkie_sdk.modules.arm import ArmGroup

# ── helpers ────────────────────────────────────────────────────────────────


def _prompt_float(prompt: str, default: float = 0.0) -> float:
    raw = input(f"  {prompt} [{default}]: ").strip()
    return float(raw) if raw else default


def _prompt_bool(prompt: str, default: bool = False) -> bool:
    d = "y" if default else "n"
    raw = input(f"  {prompt} (y/n) [{d}]: ").strip().lower()
    return (raw == "y") if raw else default


def _prompt_joints(n: int) -> list[float]:
    print(
        f"  Enter {n} joint positions in radians separated by spaces (default: all zeros):"
    )
    raw = input("  > ").strip()
    if not raw:
        return [0.0] * n
    parts = raw.split()
    vals = [float(p) for p in parts]
    if len(vals) < n:
        vals += [0.0] * (n - len(vals))
    return vals[:n]


_GROUP_DOF = {
    "left_arm_lift": 7,
    "right_arm_lift": 7,
}


def _feedback(fb: dict):
    print(f"  [feedback] {fb}")


# ── menu actions ───────────────────────────────────────────────────────────


def do_go_to_pose(arm: ArmGroup):
    print(f"\n=== go_to_pose (Euler RPY) → {arm.group_name} ===")
    x = _prompt_float("x (m)", 0.4)
    y = _prompt_float("y (m)", 0.0)
    z = _prompt_float("z (m)", 0.5)
    roll = _prompt_float("roll (rad)", 0.0)
    pitch = _prompt_float("pitch (rad)", 0.0)
    yaw = _prompt_float("yaw (rad)", 0.0)
    frame = input("  frame_id [base_footprint]: ").strip() or "base_footprint"
    cart = _prompt_bool("cartesian_path", False)
    block = _prompt_bool("blocking", True)

    print(
        f"  Sending → pos=({x},{y},{z}) rpy=({roll},{pitch},{yaw}) frame={frame} cart={cart}"
    )
    result = arm.go_to_pose(
        pos=[x, y, z],
        rot=[roll, pitch, yaw],
        frame_id=frame,
        cartesian_path=cart,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_go_to_pose_relative(arm: ArmGroup):
    print(f"\n=== go_to_pose_relative → {arm.group_name} ===")
    x = _prompt_float("x offset (m)", 0.0)
    y = _prompt_float("y offset (m)", 0.0)
    z = _prompt_float("z offset (m)", 0.05)
    roll = _prompt_float("roll delta (rad)", 0.0)
    pitch = _prompt_float("pitch delta (rad)", 0.0)
    yaw = _prompt_float("yaw delta (rad)", 0.0)
    frame = input("  frame_id [base_footprint]: ").strip() or "base_footprint"
    ee_frame = _prompt_bool("ee_frame (offsets in EEF-local axes)", False)
    cart = _prompt_bool("cartesian_path", False)
    block = _prompt_bool("blocking", True)

    print(
        f"  Sending → offset=({x},{y},{z}) rpy=({roll},{pitch},{yaw}) ee_frame={ee_frame}"
    )
    result = arm.go_to_pose_relative(
        pos=[x, y, z],
        rot=[roll, pitch, yaw],
        frame_id=frame,
        cartesian_path=cart,
        ee_frame=ee_frame,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_go_to_pose_quat(arm: ArmGroup):
    print(f"\n=== go_to_pose_quat (quaternion) → {arm.group_name} ===")
    use_euler = _prompt_bool("Convert from Euler RPY", True)

    x = _prompt_float("x (m)", 0.4)
    y = _prompt_float("y (m)", 0.0)
    z = _prompt_float("z (m)", 0.5)

    if use_euler:
        roll = _prompt_float("roll (rad)", 0.0)
        pitch = _prompt_float("pitch (rad)", 0.0)
        yaw = _prompt_float("yaw (rad)", 0.0)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        print(f"  Converted → qx={qx:.4f} qy={qy:.4f} qz={qz:.4f} qw={qw:.4f}")
    else:
        qx = _prompt_float("qx", 0.0)
        qy = _prompt_float("qy", 0.0)
        qz = _prompt_float("qz", 0.0)
        qw = _prompt_float("qw", 1.0)

    frame = input("  frame_id [base_footprint]: ").strip() or "base_footprint"
    cart = _prompt_bool("cartesian_path", False)
    block = _prompt_bool("blocking", True)

    print(f"  Sending → pos=({x},{y},{z}) q=({qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f})")
    result = arm.go_to_pose_quat(
        pos=[x, y, z],
        rot=[qx, qy, qz, qw],
        frame_id=frame,
        cartesian_path=cart,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_go_to_home(arm: ArmGroup):
    print(f"\n=== go_to_home → {arm.group_name} ===")
    block = _prompt_bool("blocking", True)

    print(f"  Sending go_to_home ...")
    result = arm.go_to_home(
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_gripper(arm: ArmGroup):
    print(f"\n=== gripper → {arm.group_name} ===")
    print("  Normalized: 0.0 = fully closed, 1.0 = fully open (0.04 m)")
    value = _prompt_float("value (0.0–1.0)", 1.0)
    block = _prompt_bool("blocking", True)

    print(f"  Sending gripper({value:.2f}) ...")
    result = arm.gripper(
        value,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_set_joint_position(arm: ArmGroup):
    print(f"\n=== set_joint_position → {arm.group_name} ===")
    dof = _GROUP_DOF.get(arm.group_name, 8)
    joints = _prompt_joints(dof)

    print("  Mode:")
    print("    1. commander (MoveIt, collision-checked)")
    print("    2. jtc       (direct JTC publish, high-rate, no collision check)")
    mode_choice = input("  Choose [1/2, default 1]: ").strip()
    mode = "jtc" if mode_choice == "2" else "commander"

    if mode == "jtc":
        duration = _prompt_float("trajectory duration (s)", 1.0)
        print(
            f"  Sending JTC → joints={[round(j, 3) for j in joints]} duration={duration}s"
        )
        result = arm.set_joint_position(joints, mode="jtc", duration=duration)
    else:
        block = _prompt_bool("blocking", True)
        print(f"  Sending commander → joints={[round(j, 3) for j in joints]}")
        result = arm.set_joint_position(
            joints,
            mode="commander",
            blocking=block,
            feedback_callback=_feedback if block else None,
        )
    print(f"  Result: {result}")


def do_get_ee_pose(arm: ArmGroup):
    print(f"\n=== get_ee_pose (service) → {arm.group_name} ===")
    frame = input("  frame_id [base_footprint]: ").strip() or "base_footprint"
    timeout = _prompt_float("timeout (s)", 5.0)

    print(f"  Querying EEF pose ...")
    result = arm.get_ee_pose(frame_id=frame, timeout=timeout)
    if result is None:
        print("  Result: None (service failed or timed out)")
        return

    x, y, z = result["x"], result["y"], result["z"]
    qx, qy, qz, qw = result["qx"], result["qy"], result["qz"], result["qw"]
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    print(f"  Position  : x={x:.4f}  y={y:.4f}  z={z:.4f}")
    print(f"  Quaternion: qx={qx:.4f}  qy={qy:.4f}  qz={qz:.4f}  qw={qw:.4f}")
    print(f"  Euler (rad): roll={roll:.4f}  pitch={pitch:.4f}  yaw={yaw:.4f}")
    print(
        f"  Euler (deg): roll={math.degrees(roll):.2f}  pitch={math.degrees(pitch):.2f}  yaw={math.degrees(yaw):.2f}"
    )


def do_get_joint_states_service(arm: ArmGroup):
    print(f"\n=== get_joint_states (service) → {arm.group_name} ===")
    timeout = _prompt_float("timeout (s)", 5.0)

    print(f"  Querying joint states ...")
    result = arm.get_joint_states_service(timeout=timeout)
    if result is None:
        print("  Result: None (service failed or timed out)")
        return

    names = result.get("joint_names", [])
    positions = result.get("joint_positions", [])
    print(f"  {'Joint':<30} {'rad':>10}  {'deg':>10}")
    print(f"  {'-' * 54}")
    for name, pos in zip(names, positions):
        print(f"  {name:<30} {pos:>10.4f}  {math.degrees(pos):>10.2f}")


def do_get_joint_states_topic(robot: WalkieRobot):
    print("\n=== get_joint_states (topic subscription, all joints) ===")
    result = robot.arm.get_joint_states()
    if result is None:
        print("  No data received yet — wait a moment and try again.")
        return

    for side in ("left_arm", "right_arm"):
        positions = result.get(side, {}).get("positions", [])
        print(f"\n  {side}:")
        print(f"    {'Joint':<6} {'rad':>10}  {'deg':>10}")
        for i, pos in enumerate(positions, 1):
            print(f"    {i:<6} {pos:>10.4f}  {math.degrees(pos):>10.2f}")

    lg = result.get("left_gripper")
    rg = result.get("right_gripper")
    print(f"\n  left_gripper:  {lg}")
    print(f"  right_gripper: {rg}")


# ── main menu ──────────────────────────────────────────────────────────────

_MENU_TEMPLATE = """
┌─────────────────────────────────────────────────────┐
│         Walkie Arm Sub-object Interactive Test      │
│                  Active arm: {side:<6}              │
├──────┬──────────────────────────────────────────────┤
│  l   │ switch to LEFT  arm                          │
│  r   │ switch to RIGHT arm                          │
├──────┼──────────────────────────────────────────────┤
│  1   │ go_to_pose         (Euler RPY)               │
│  2   │ go_to_pose_relative                          │
│  3   │ go_to_pose_quat    (quaternion)              │
│  4   │ go_to_home                                   │
│  5   │ gripper            (normalised 0.0–1.0)      │
│  6   │ set_joint_position (commander or JTC)        │
│  7   │ get_ee_pose        (service)                 │
│  8   │ get_joint_states   (service)                 │
│  9   │ get_joint_states   (topic, all joints)       │
│  q   │ quit                                         │
└──────┴──────────────────────────────────────────────┘"""


def main():
    parser = argparse.ArgumentParser(
        description="Arm sub-object interactive tester (bot.arm.left / bot.arm.right)"
    )
    parser.add_argument(
        "--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=9090, help="Rosbridge port (default: 9090)"
    )
    parser.add_argument(
        "--protocol", default="rosbridge", choices=["rosbridge", "zenoh"]
    )
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument(
        "--no-camera", action="store_true", help="Disable camera (speeds up connection)"
    )
    parser.add_argument(
        "--arm",
        default="left",
        choices=["left", "right"],
        help="Start arm (default: left)",
    )
    args = parser.parse_args()

    print(f"Connecting to {args.ip}:{args.port} (protocol={args.protocol}) ...")
    robot = WalkieRobot(
        ip=args.ip,
        ros_protocol=args.protocol,
        ros_port=args.port,
        camera_protocol="none" if args.no_camera else "zenoh",
        namespace=args.namespace,
    )
    print("Connected.\n")
    print("  bot.arm.left  →", robot.arm.left.group_name)
    print("  bot.arm.right →", robot.arm.right.group_name)

    active: ArmGroup = robot.arm.left if args.arm == "left" else robot.arm.right

    try:
        while True:
            print(_MENU_TEMPLATE.format(side=active.group_name.split("_")[0].upper()))
            choice = input("Choice: ").strip().lower()

            if choice in ("q", "quit", "exit"):
                break
            elif choice == "l":
                active = robot.arm.left
                print(f"  Switched to LEFT ({active.group_name})")
                continue
            elif choice == "r":
                active = robot.arm.right
                print(f"  Switched to RIGHT ({active.group_name})")
                continue

            try:
                if choice == "1":
                    do_go_to_pose(active)
                elif choice == "2":
                    do_go_to_pose_relative(active)
                elif choice == "3":
                    do_go_to_pose_quat(active)
                elif choice == "4":
                    do_go_to_home(active)
                elif choice == "5":
                    do_gripper(active)
                elif choice == "6":
                    do_set_joint_position(active)
                elif choice == "7":
                    do_get_ee_pose(active)
                elif choice == "8":
                    do_get_joint_states_service(active)
                elif choice == "9":
                    do_get_joint_states_topic(robot)
                else:
                    print(f"  Unknown choice '{choice}'")
            except KeyboardInterrupt:
                print("\n  (interrupted — back to menu)")
            except Exception as e:
                print(f"  Error: {e}")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
