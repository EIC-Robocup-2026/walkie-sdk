"""
Give-mou demo sequence.

Sequence (press Enter before each step):
  0. Start at standby position, arms in MoveIt "standby" pose, mou in hand
  1. Walk to center of table
  2. Move to left side of table, left hand to "ready to give mou" pose
  3. Arms back to MoveIt "standby" pose
  4. Move to right side of table, right hand to "ready to give mou" pose
  5. Walk back to the standby position from step 0

All commands go through walkie_sdk:
  - Navigation: blocking bot.nav.go_to(x, y, heading)
  - Standby pose: MoveIt named scene pose via bot.arm.go_to_home(pose_name="standby")
  - Give poses: MoveIt joint command via bot.arm.set_joint_position(mode="commander")

Run:
    python examples/example_give_mou.py --ip <robot-ip>
"""

import argparse
import math
import sys
import time

from walkie_sdk import WalkieRobot

# ── Navigation waypoints (map frame: x [m], y [m], heading [rad]) ──────────
# TODO: tune these for the actual arena.
STANDBY_POSE = (0.0, 0.13, 0.0)          # step 0/5: start & return position
TABLE_CENTER = (2.75, -0.08, 1.57)          # step 1: center of table
TABLE_LEFT   = (2.52, -0.08, 1.57)          # step 2: left side of table
TABLE_RIGHT  = (3.11, -0.08, 1.57)         # step 4: right side of table

# ── Arm joint targets (radians, 7 joints per arm) ──────────────────────────
# TODO: tune these "ready to give mou" joint positions.
# LEFT_GIVE_JOINTS  = [0.0, math.radians(45), 0.0, math.radians(60), 0.0, 0.0, 0.0]
BOTH_STANDBY_JOINTS   = [0.4189, 0.0175, 0.00, 1.6850, 0.0698, 0.6981, -1.5882, -0.4189, 0.0000, 0.0698, 1.6850, 0.0698, -0.6981, 1.5533]
LEFT_GIVE_JOINTS   = [-0.6745, 0.0000, 0.0000, 0.8350, 1.4312, -0.0698, -0.0175]
RIGHT_GIVE_JOINTS  = [ 0.6745, 0.0000, 0.0000, 0.9350, -1.4312,-0.2698, -0.0175]

ARM_GROUP_BOTH  = "both_arms"
ARM_GROUP_LEFT  = "left_arm"
ARM_GROUP_RIGHT = "right_arm"


def wait_enter(step: str) -> None:
    input(f"\n>>> Press Enter to run: {step}")


def go_to(bot: WalkieRobot, name: str, pose) -> None:
    x, y, heading = pose
    print(f"[nav] going to {name}: x={x:.2f}, y={y:.2f}, heading={heading:.2f}")
    result = bot.nav.go_to(x=x, y=y, heading=heading, blocking=True)
    print(f"[nav] {name}: {result}")


def standby_pose(bot: WalkieRobot) -> None:
    print("[arm] moving to MoveIt 'standby' pose")
    # result = bot.arm.go_to_home(group_name=ARM_GROUP_BOTH, pose_name="standby", blocking=True)
    result = bot.arm.set_joint_position("both_arms", BOTH_STANDBY_JOINTS, mode="commander", blocking=True)
    print(f"[arm] standby: {result}")


def give_pose(bot: WalkieRobot, group: str, joints) -> None:
    print(f"[arm] moving {group} to give-mou joint position")
    result = bot.arm.set_joint_position(group, joints, mode="commander", blocking=True)
    print(f"[arm] {group} give pose: {result}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Give-mou demo sequence")
    parser.add_argument("--ip", required=True, help="Robot IP address")
    args = parser.parse_args()

    print(f"Connecting to robot at {args.ip} ...")
    bot = WalkieRobot(ip=args.ip, camera_protocol="none")
    print("Connected.")

    try:
        # 0. Standby pose at the start position (mou already in hand)
        # wait_enter("step 0 — standby pose (mou in hand)")
        standby_pose(bot)

        # 1. Walk to center of table
        # wait_enter("step 1 — walk to center of table")
        # go_to(bot, "table center", TABLE_CENTER)

        # # 2. Move to left, left hand ready to give mou
        wait_enter("step 2a — move to left side of table")
        go_to(bot, "table left", TABLE_LEFT)
        # wait_enter("step 2b — left hand ready to give mou")
        give_pose(bot, ARM_GROUP_LEFT, LEFT_GIVE_JOINTS)

        time.sleep(1.5)  # หยุดการทำงานชั่วคราวเป็นเวลา 2.5 วินาที
        bot.arm.left.gripper(
            1,
            blocking=True,
        )


        # # 3. Back to standby pose
        # wait_enter("step 3 — arms back to standby pose")
        standby_pose(bot)

        bot.arm.left.gripper(
            0.0,
            blocking=True,
        )

        # 4. Move to right, right hand ready to give mou
        # wait_enter("step 4a — move to right side of table")
        go_to(bot, "table right", TABLE_RIGHT)
        # wait_enter("step 4b — right hand ready to give mou")
        give_pose(bot, ARM_GROUP_RIGHT, RIGHT_GIVE_JOINTS)

        time.sleep(1.5)  # หยุดการทำงานชั่วคราวเป็นเวลา 2.5 วินาที
        bot.arm.right.gripper(
            1.0,
            blocking=True,
        )

        # 5. Return to standby position and pose
        # wait_enter("step 5a — arms back to standby pose")
        standby_pose(bot)
        bot.arm.right.gripper(
            0.0,
            blocking=True,
        )
        # wait_enter("step 5b — walk back to standby position")
        go_to(bot, "standby position", STANDBY_POSE)

        print("\nSequence complete.")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted — stopping robot.")
        bot.nav.stop()
        return 1
    finally:
        bot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
