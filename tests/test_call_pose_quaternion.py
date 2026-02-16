"""
Test go_to_pose_quaternion() in both MoveIt and Custom IK modes.

Usage:
    # Test MoveIt mode (default):
    python tests/test_call_pose_qurtanion.py

    # Test Custom IK mode:
    python tests/test_call_pose_qurtanion.py --mode custom_ik

    # Custom IK with custom topic:
    python tests/test_call_pose_qurtanion.py --mode custom_ik --topic /left_arm/target_pose

    # Test both modes sequentially:
    python tests/test_call_pose_qurtanion.py --mode both
"""

import argparse
import math
import time

from walkie_sdk.robot import WalkieRobot
from walkie_sdk.modules.arm import ArmControlMode


# ── Feedback callback ──────────────────────────────────────────────────


def on_arm_feedback(feedback: dict):
    """Callback to handle real-time updates from the robot."""
    print(f"\n[>> FEEDBACK] Raw Data: {feedback}")


# ── Test functions ─────────────────────────────────────────────────────


def test_moveit(robot: WalkieRobot):
    """Test go_to_pose_quaternion via MoveIt action server."""
    print("\n" + "=" * 60)
    print("  TEST: go_to_pose_quaternion (MoveIt mode)")
    print("=" * 60)

    x_pos, y_pos, z_pos = 0.38, 0.19, 0.58
    qx_val, qy_val, qz_val, qw_val = -0.5, -0.5, 0.5, 0.5
    group = "left_arm"
    link = "left_link7"

    print(f"  Target position:    ({x_pos}, {y_pos}, {z_pos})")
    print(f"  Target quaternion:  ({qx_val}, {qy_val}, {qz_val}, {qw_val})")
    print(f"  Group: {group}  Link: {link}")
    print(f"  Mode: MOVEIT (blocking)\n")

    # Draw axis triad at target pose before moving
    robot.draw_axis(
        position=[x_pos, y_pos, z_pos],
        quaternion=[qx_val, qy_val, qz_val, qw_val],
        axis_name=f"target_{group}",
    )
    print(f"  [Viz] Drew axis triad for target pose")

    status = robot.arm.go_to_pose_quaternion(
        x=x_pos,
        y=y_pos,
        z=z_pos,
        qx=qx_val,
        qy=qy_val,
        qz=qz_val,
        qw=qw_val,
        group_name=group,
        link_name=link,
        allowed_planning_time=10.0,
        blocking=True,
        feedback_callback=on_arm_feedback,
        mode=ArmControlMode.MOVEIT,
    )

    print(f"\n  [MoveIt] Motion result: {status}")
    return status


def test_custom_ik(robot: WalkieRobot, duration: float = 5.0):
    """Test go_to_pose_quaternion via Custom IK (continuous publishing)."""
    print("\n" + "=" * 60)
    print("  TEST: go_to_pose_quaternion (Custom IK mode)")
    print("=" * 60)

    # Circle parameters (YZ plane, same as the example circle_pose publisher)
    cx, cy, cz = 0.34, -0.20, 0.50
    radius = 0.08
    period = 4.0
    rate_hz = 50.0
    dt = 1.0 / rate_hz

    # Orientation: identity quaternion (let IK solver handle it)
    qx_val, qy_val, qz_val, qw_val = 0.0, 0.0, 0.0, 1.0

    print(f"  Circle center:  ({cx}, {cy}, {cz})")
    print(f"  Radius: {radius} m   Period: {period} s")
    print(f"  Publish rate: {rate_hz} Hz")
    print(f"  Target topic: {robot.arm.target_pose_topic}")
    print(f"  Duration: {duration} s")
    print(f"  Mode: CUSTOM_IK\n")

    axis_name = "ik_target"
    axis_created = False

    elapsed = 0.0
    start = time.monotonic()

    while (time.monotonic() - start) < duration:
        angle = 2.0 * math.pi * (elapsed / period)

        y = cy
        x = cx + radius * math.cos(angle)
        z = cz + radius * math.sin(angle)

        status = robot.arm.go_to_pose_quaternion(
            x=x,
            y=y,
            z=z,
            qx=qx_val,
            qy=qy_val,
            qz=qz_val,
            qw=qw_val,
            group_name="left_arm",  # ignored in custom_ik mode
            mode=ArmControlMode.CUSTOM_IK,
        )

        # Update axis triad to follow the target pose
        if not axis_created:
            robot.draw_axis(
                position=[x, y, z],
                quaternion=[qx_val, qy_val, qz_val, qw_val],
                axis_name=axis_name,
            )
            axis_created = True
        else:
            robot.update_axis(axis_name, position=[x, y, z])

        # Throttled log (once per second)
        if int(elapsed) != int(elapsed - dt) or elapsed < dt:
            deg = math.degrees(angle) % 360
            print(
                f"  [Custom IK] angle={deg:5.0f}deg  "
                f"pos=({x:.3f}, {y:.3f}, {z:.3f})  status={status}"
            )

        elapsed += dt
        time.sleep(dt)

    print(f"\n  [Custom IK] Finished {duration}s circle test.")
    return "SUCCEEDED"


# ── Main ───────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Test go_to_pose_quaternion in MoveIt and/or Custom IK mode."
    )
    parser.add_argument(
        "--mode",
        choices=["moveit", "custom_ik", "both"],
        default="moveit",
        help="Control mode to test (default: moveit)",
    )
    parser.add_argument(
        "--topic",
        default="/target_pose",
        help="Target pose topic for custom IK mode (default: /target_pose)",
    )
    parser.add_argument(
        "--ip",
        default="127.0.0.1",
        help="Robot IP address (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="Rosbridge port (default: 9090)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Duration for custom IK circle test in seconds (default: 5.0)",
    )
    args = parser.parse_args()

    # Initialize robot
    robot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        arm_target_pose_topic=args.topic,
    )

    try:
        if args.mode == "moveit":
            test_moveit(robot)

        elif args.mode == "custom_ik":
            test_custom_ik(robot, duration=args.duration)

        elif args.mode == "both":
            # Test MoveIt first (move to known pose)
            test_moveit(robot)
            print("\n  Waiting 2s before Custom IK test...")
            time.sleep(2.0)

            # Then test Custom IK (circle around that region)
            test_custom_ik(robot, duration=args.duration)

        # Keep alive so user can observe the robot state
        print("\nAll tests complete. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
