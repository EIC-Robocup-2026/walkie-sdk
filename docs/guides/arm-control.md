# Arm Control

The SDK provides dual-arm control with joint-level commands, Cartesian
(IK) pose control, and gripper manipulation. Access via `bot.arm`.

## Control Modes

The arm supports two control modes for Cartesian pose commands:

| Mode | Value | Description |
|------|-------|-------------|
| MoveIt | `"moveit"` | Motion planning via MoveIt action servers. Blocking, with path planning. |
| Custom IK | `"custom_ik"` | Publishes `PoseStamped` to a custom IK solver node. Low-latency, for teleop. |

Set the default mode at construction:

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    arm_mode="custom_ik",  # or "moveit"
)
```

Or change it at runtime:

```python
bot.arm.default_mode = "moveit"
```

## Joint Position Control

Send joint positions directly. Each arm has 7 joints:

```python
# Set left arm joint positions (7 values in radians)
bot.arm.set_joint_positions(
    left_arm=[0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0],
)

# Set both arms + grippers
bot.arm.set_joint_positions(
    left_arm=[0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0],
    right_arm=[0.0, 0.5, 0.0, -1.0, 0.0, -0.5, 0.0],
    left_gripper=0.7,   # 0.0-1.0
    right_gripper=0.0,
)
```

## Reading Joint States

```python
states = bot.arm.get_joint_states()
if states:
    print(states["left_arm"]["positions"])    # [7 floats]
    print(states["left_arm"]["velocities"])   # [7 floats]
    print(states["left_arm"]["torques"])      # [7 floats]
    print(states["left_gripper"])             # float or None
    print(states["right_gripper"])            # float or None
```

## Cartesian Pose Control (Euler)

Move an arm end-effector to a Cartesian pose using Euler angles:

```python
result = bot.arm.go_to_pose(
    group_name="left_arm",
    x=0.38, y=0.19, z=0.58,
    roll=-1.57, pitch=0.0, yaw=1.57,
    cartesian_path=False,  # True for linear path (MoveIt only)
    blocking=True,
)
print(result)  # "SUCCEEDED" or "FAILED"
```

In `custom_ik` mode, the Euler angles are converted to a quaternion and
published to the target pose topic. The `group_name`, `cartesian_path`,
and `blocking` parameters are ignored.

## Cartesian Pose Control (Quaternion)

When you already have quaternion data (e.g., from a tracking system):

```python
result = bot.arm.go_to_pose_quaternion(
    group_name="left_arm",
    x=0.38, y=0.19, z=0.58,
    qx=0.0, qy=0.707, qz=0.0, qw=0.707,
    blocking=True,
)
```

### MoveGroup Action Interface

For direct MoveGroup action server integration with full constraint control:

```python
result = bot.arm.go_to_pose_quaternion_move_action(
    group_name="left_arm",
    x=0.38, y=0.19, z=0.58,
    qx=0.0, qy=0.707, qz=0.0, qw=0.707,
    link_name="left_link7",
    frame_id="base_footprint",
    allowed_planning_time=10.0,
)
```

## Relative Pose Movement

Move relative to the current pose:

```python
result = bot.arm.go_to_pose_relative(
    group_name="left_arm",
    x=0.05, y=0.0, z=-0.02,  # move 5cm forward, 2cm down
    roll=0.0, pitch=0.0, yaw=0.0,
)
```

## Home Position

Move an arm to its predefined home position:

```python
bot.arm.go_to_home("left_arm")
bot.arm.go_to_home("right_arm")
```

## Gripper Control

```python
# Close gripper
bot.arm.control_gripper("left_gripper", position=0.7)

# Open gripper
bot.arm.control_gripper("left_gripper", position=-15.71)
```

!!! note
    Gripper position values depend on your robot's gripper hardware.
    The values above are examples -- check your robot's configuration.

## Per-Call Mode Override

Override the default control mode for a single call:

```python
# Instance default is custom_ik, but use moveit for this call
result = bot.arm.go_to_pose(
    group_name="left_arm",
    x=0.38, y=0.19, z=0.58,
    roll=-1.57, pitch=0.0, yaw=1.57,
    mode="moveit",  # override for this call only
)
```

## Custom IK Topic

The target pose topic for custom IK mode defaults to `/target_pose`.
Change it at construction or runtime:

```python
# At construction
bot = WalkieRobot(
    ip="192.168.1.100",
    arm_target_pose_topic="/my_ik_solver/target",
)

# At runtime
bot.arm.target_pose_topic = "/my_ik_solver/target"
```
