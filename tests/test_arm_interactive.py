"""
Interactive arm tester — mirrors openarm_bimanual_commander_cpp commander_interactive.

Tests all arm interfaces against a real robot over the network.

Controls (main menu):
  1  go_to_pose         — absolute EEF pose (Euler RPY)
  2  go_to_pose_relative — relative EEF displacement (with ee_frame toggle)
  3  go_to_pose_quat    — absolute EEF pose (quaternion)
  4  go_to_home         — named home position
  5  control_gripper    — open / close gripper
  6  set_joint_position — direct joint angles (commander or JTC)
  7  get_ee_pose        — query current EEF pose (service)
  8  get_joint_states   — query joint positions (service, per group)
  9  get_joint_states   — read from topic subscription (all joints)
  q  quit

Usage:
    python tests/test_arm_interactive.py
    python tests/test_arm_interactive.py --ip 192.168.1.100
    python tests/test_arm_interactive.py --ip 192.168.1.100 --port 9090
    python tests/test_arm_interactive.py --ip 192.168.1.100 --namespace robot1
"""

import argparse
import math

from walkie_sdk import WalkieRobot


# ── helpers ────────────────────────────────────────────────────────────────

_GROUPS = [
    "left_arm", "right_arm",
    "left_arm_lift", "right_arm_lift",
    "both_arms", "both_arms_lift",
    "left_gripper", "right_gripper",
]

_ARM_GROUPS = [g for g in _GROUPS if "gripper" not in g]
_GRIPPER_GROUPS = ["left_gripper", "right_gripper"]


def _prompt_float(prompt: str, default: float = 0.0) -> float:
    raw = input(f"  {prompt} [{default}]: ").strip()
    return float(raw) if raw else default


def _prompt_bool(prompt: str, default: bool = False) -> bool:
    d = "y" if default else "n"
    raw = input(f"  {prompt} (y/n) [{d}]: ").strip().lower()
    return (raw == "y") if raw else default


def _prompt_group(choices: list[str]) -> str:
    print("  Groups:")
    for i, g in enumerate(choices, 1):
        print(f"    {i}. {g}")
    while True:
        raw = input(f"  Choose [1-{len(choices)}]: ").strip()
        if raw.isdigit() and 1 <= int(raw) <= len(choices):
            return choices[int(raw) - 1]
        print("  Invalid choice.")


def _prompt_joints(n: int) -> list[float]:
    print(f"  Enter {n} joint positions in radians, separated by spaces:")
    print(f"  (default: all zeros)")
    raw = input("  > ").strip()
    if not raw:
        return [0.0] * n
    parts = raw.split()
    vals = [float(p) for p in parts]
    if len(vals) < n:
        vals += [0.0] * (n - len(vals))
    return vals[:n]


_GROUP_DOF = {
    "left_arm": 7, "right_arm": 7,
    "left_arm_lift": 8, "right_arm_lift": 8,
    "both_arms": 14, "both_arms_lift": 15,
    "left_gripper": 1, "right_gripper": 1,
}


def _feedback(fb: dict):
    print(f"  [feedback] {fb}")


# ── menu actions ───────────────────────────────────────────────────────────

def do_go_to_pose(robot: WalkieRobot):
    print("\n=== go_to_pose (Euler RPY) ===")
    group = _prompt_group(_ARM_GROUPS)
    x     = _prompt_float("x (m)", 0.4)
    y     = _prompt_float("y (m)", 0.0)
    z     = _prompt_float("z (m)", 0.5)
    roll  = _prompt_float("roll (rad)", 0.0)
    pitch = _prompt_float("pitch (rad)", 0.0)
    yaw   = _prompt_float("yaw (rad)", 0.0)
    frame = input("  frame_id [base_footprint]: ").strip() or "base_footprint"
    cart  = _prompt_bool("cartesian_path", False)
    block = _prompt_bool("blocking", True)

    print(f"  Sending go_to_pose → group={group} pos=({x},{y},{z}) rpy=({roll},{pitch},{yaw}) frame={frame} cart={cart} blocking={block}")
    result = robot.arm.go_to_pose(
        x=x, y=y, z=z,
        roll=roll, pitch=pitch, yaw=yaw,
        group_name=group,
        frame_id=frame,
        cartesian_path=cart,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_go_to_pose_relative(robot: WalkieRobot):
    print("\n=== go_to_pose_relative ===")
    group    = _prompt_group(_ARM_GROUPS)
    x        = _prompt_float("x offset (m)", 0.0)
    y        = _prompt_float("y offset (m)", 0.0)
    z        = _prompt_float("z offset (m)", 0.05)
    roll     = _prompt_float("roll delta (rad)", 0.0)
    pitch    = _prompt_float("pitch delta (rad)", 0.0)
    yaw      = _prompt_float("yaw delta (rad)", 0.0)
    frame    = input("  frame_id [base_footprint]: ").strip() or "base_footprint"
    ee_frame = _prompt_bool("ee_frame (offsets in EEF-local axes)", False)
    cart     = _prompt_bool("cartesian_path", False)
    block    = _prompt_bool("blocking", True)

    print(f"  Sending go_to_pose_relative → group={group} offset=({x},{y},{z}) rpy=({roll},{pitch},{yaw}) ee_frame={ee_frame}")
    result = robot.arm.go_to_pose_relative(
        x=x, y=y, z=z,
        roll=roll, pitch=pitch, yaw=yaw,
        group_name=group,
        frame_id=frame,
        cartesian_path=cart,
        ee_frame=ee_frame,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_go_to_pose_quat(robot: WalkieRobot):
    print("\n=== go_to_pose_quat (quaternion) ===")
    print("  Hint: enter Euler to convert, or enter quaternion directly.")
    use_euler = _prompt_bool("Convert from Euler RPY", True)

    group = _prompt_group(_ARM_GROUPS)
    x = _prompt_float("x (m)", 0.4)
    y = _prompt_float("y (m)", 0.0)
    z = _prompt_float("z (m)", 0.5)

    if use_euler:
        roll  = _prompt_float("roll (rad)", 0.0)
        pitch = _prompt_float("pitch (rad)", 0.0)
        yaw   = _prompt_float("yaw (rad)", 0.0)
        # Simple ZYX Euler to quaternion
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
    cart  = _prompt_bool("cartesian_path", False)
    block = _prompt_bool("blocking", True)

    print(f"  Sending go_to_pose_quat → group={group} pos=({x},{y},{z}) q=({qx},{qy},{qz},{qw})")
    result = robot.arm.go_to_pose_quat(
        x=x, y=y, z=z,
        qx=qx, qy=qy, qz=qz, qw=qw,
        group_name=group,
        frame_id=frame,
        cartesian_path=cart,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_go_to_home(robot: WalkieRobot):
    print("\n=== go_to_home ===")
    group = _prompt_group(_ARM_GROUPS)
    block = _prompt_bool("blocking", True)

    print(f"  Sending go_to_home → group={group}")
    result = robot.arm.go_to_home(
        group_name=group,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_control_gripper(robot: WalkieRobot):
    print("\n=== control_gripper ===")
    group = _prompt_group(_GRIPPER_GROUPS)
    pos   = _prompt_float("position in meters (0.0=closed, 0.04=open)", 0.04)
    block = _prompt_bool("blocking", True)

    print(f"  Sending control_gripper → group={group} position={pos}")
    result = robot.arm.control_gripper(
        group_name=group,
        position=pos,
        blocking=block,
        feedback_callback=_feedback if block else None,
    )
    print(f"  Result: {result}")


def do_set_joint_position(robot: WalkieRobot):
    print("\n=== set_joint_position ===")
    group = _prompt_group(_ARM_GROUPS)
    dof   = _GROUP_DOF.get(group, 7)
    joints = _prompt_joints(dof)

    print("  Mode:")
    print("    1. commander (MoveIt, collision-checked, ~0.4 Hz)")
    print("    2. jtc       (direct JTC publish, high-rate, no collision check)")
    mode_choice = input("  Choose [1/2, default 1]: ").strip()
    mode = "jtc" if mode_choice == "2" else "commander"

    if mode == "jtc":
        duration = _prompt_float("trajectory duration (s)", 1.0)
        print(f"  Sending set_joint_position (JTC) → group={group} joints={[round(j,3) for j in joints]} duration={duration}s")
        result = robot.arm.set_joint_position(
            group_name=group,
            joint_positions=joints,
            mode="jtc",
            duration=duration,
        )
    else:
        block = _prompt_bool("blocking", True)
        print(f"  Sending set_joint_position (commander) → group={group} joints={[round(j,3) for j in joints]}")
        result = robot.arm.set_joint_position(
            group_name=group,
            joint_positions=joints,
            mode="commander",
            blocking=block,
            feedback_callback=_feedback if block else None,
        )
    print(f"  Result: {result}")


def do_get_ee_pose(robot: WalkieRobot):
    print("\n=== get_ee_pose (service) ===")
    group   = _prompt_group(_ARM_GROUPS)
    frame   = input("  frame_id [base_footprint]: ").strip() or "base_footprint"
    timeout = _prompt_float("timeout (s)", 5.0)

    print(f"  Querying EEF pose → group={group} frame={frame}")
    result = robot.arm.get_ee_pose(group_name=group, frame_id=frame, timeout=timeout)
    if result is None:
        print("  Result: None (service failed or timed out)")
    else:
        x, y, z = result["x"], result["y"], result["z"]
        qx, qy, qz, qw = result["qx"], result["qy"], result["qz"], result["qw"]
        # Quaternion → Euler (ZYX)
        roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
        pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
        yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        print(f"  Position : x={x:.4f}  y={y:.4f}  z={z:.4f}")
        print(f"  Quaternion: qx={qx:.4f}  qy={qy:.4f}  qz={qz:.4f}  qw={qw:.4f}")
        print(f"  Euler (rad): roll={roll:.4f}  pitch={pitch:.4f}  yaw={yaw:.4f}")
        print(f"  Euler (deg): roll={math.degrees(roll):.2f}  pitch={math.degrees(pitch):.2f}  yaw={math.degrees(yaw):.2f}")
        print(f"  Frame: {result.get('frame_id', frame)}")


def do_get_joint_states_service(robot: WalkieRobot):
    print("\n=== get_joint_states (service, per group) ===")
    group   = _prompt_group(_ARM_GROUPS)
    timeout = _prompt_float("timeout (s)", 5.0)

    print(f"  Querying joint states → group={group}")
    result = robot.arm.get_joint_states_service(group_name=group, timeout=timeout)
    if result is None:
        print("  Result: None (service failed or timed out)")
    else:
        names  = result.get("joint_names", [])
        positions = result.get("joint_positions", [])
        print(f"  {'Joint':<30} {'rad':>10}  {'deg':>10}")
        print(f"  {'-'*52}")
        for name, pos in zip(names, positions):
            print(f"  {name:<30} {pos:>10.4f}  {math.degrees(pos):>10.2f}")


def do_get_joint_states_topic(robot: WalkieRobot):
    print("\n=== get_joint_states (topic subscription, all joints) ===")
    result = robot.arm.get_joint_states()
    if result is None:
        print("  No data received yet — wait a moment and try again.")
        return

    for side in ("left_arm", "right_arm"):
        arm_data = result.get(side, {})
        positions = arm_data.get("positions", [])
        print(f"\n  {side}:")
        print(f"    {'Joint':<6} {'rad':>10}  {'deg':>10}")
        for i, pos in enumerate(positions, 1):
            print(f"    {i:<6} {pos:>10.4f}  {math.degrees(pos):>10.2f}")

    lg = result.get("left_gripper")
    rg = result.get("right_gripper")
    print(f"\n  left_gripper:  {lg}")
    print(f"  right_gripper: {rg}")


# ── main menu ──────────────────────────────────────────────────────────────

_MENU = """
┌─────────────────────────────────────────────────┐
│            Walkie Arm Interactive Test           │
├──────┬──────────────────────────────────────────┤
│  1   │ go_to_pose         (Euler RPY)           │
│  2   │ go_to_pose_relative                      │
│  3   │ go_to_pose_quat    (quaternion)          │
│  4   │ go_to_home                               │
│  5   │ control_gripper                          │
│  6   │ set_joint_position (commander or JTC)    │
│  7   │ get_ee_pose        (service)             │
│  8   │ get_joint_states   (service, per group)  │
│  9   │ get_joint_states   (topic subscription)  │
│  q   │ quit                                     │
└──────┴──────────────────────────────────────────┘
"""

_ACTIONS = {
    "1": do_go_to_pose,
    "2": do_go_to_pose_relative,
    "3": do_go_to_pose_quat,
    "4": do_go_to_home,
    "5": do_control_gripper,
    "6": do_set_joint_position,
    "7": do_get_ee_pose,
    "8": do_get_joint_states_service,
    "9": do_get_joint_states_topic,
}


def main():
    parser = argparse.ArgumentParser(description="Interactive arm tester")
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--no-camera", action="store_true", help="Disable camera (speeds up connection)")
    args = parser.parse_args()

    print(f"Connecting to {args.ip}:{args.port} ...")
    robot = WalkieRobot(
        ip=args.ip,
        camera_protocol="none" if args.no_camera else "zenoh",
        namespace=args.namespace,
    )
    print("Connected.")

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
                action(robot)
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
