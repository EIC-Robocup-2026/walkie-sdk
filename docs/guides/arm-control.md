# Arm Control

The SDK provides dual-arm control via the `openarm_bimanual_commander_cpp`
interfaces on the robot. From your laptop you call `bot.arm.*` — under the
hood the SDK forwards each call to a ROS 2 action, service, or JTC topic.

Access via `bot.arm` or the per-side shortcuts `bot.arm.left` and `bot.arm.right`.

## Launching the arm service

The arm API is **client-side only**. It needs four things running on the
**robot** before any `bot.arm.*` call will succeed:

| What | Why |
|------|-----|
| `rosbridge_websocket` (port 9090) | the SDK uses rosbridge to call actions/services |
| `openarm_bimanual_commander_cpp` commander | exposes the `go_to_pose*`, `go_to_home`, `control_gripper`, `set_joint_position` actions and the `get_ee_pose` / `get_joint_states` services |
| `left_joint_trajectory_controller` + `right_joint_trajectory_controller` | listen on `*/joint_trajectory` for `mode="jtc"` direct streaming |
| `joint_state_broadcaster` (or your equivalent) publishing `/joint_states` | feeds the shared `JointStateHub` so `bot.arm.get_joint_states()` and `bot.head.get_angle()` return live data |

If your robot has a single combined launch file (the project ships one called
`walkie_bringup robot_server.launch.py`), prefer that. Otherwise, run the
pieces individually on the robot:

```bash
# Terminal 1 — rosbridge (always needed)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2 — bimanual commander (provides the action/service interface)
ros2 launch openarm_bimanual_commander_cpp commander.launch.py
#   or whatever launch file your robot bringup exposes — the SDK only cares
#   about the action/service NAMES, not which file launches them.

# Terminal 3 — combined bringup if you have it
ros2 launch walkie_bringup robot_server.launch.py
```

### Verify the interface is up

Before driving anything, confirm the actions/services/topics the SDK expects
are actually being advertised. On any machine on the same ROS graph (or by
SSHing into the robot):

```bash
ros2 action list | grep -E 'go_to_pose|go_to_home|control_gripper|set_joint_position'
# Expected:
#   /go_to_pose
#   /go_to_pose_quat
#   /go_to_pose_relative
#   /go_to_home
#   /control_gripper
#   /set_joint_position

ros2 service list | grep -E 'get_ee_pose|get_joint_states'
# Expected:
#   /get_ee_pose
#   /get_joint_states

ros2 topic list | grep -E 'joint_trajectory|joint_states'
# Expected:
#   /joint_states
#   /left_joint_trajectory_controller/joint_trajectory
#   /right_joint_trajectory_controller/joint_trajectory
```

If your names differ, don't edit code — override them via env vars (see
[Topic / type configuration](#topic--type-configuration) below) or a YAML
config passed to `WalkieRobot(config_path=...)`.

From the laptop, also confirm rosbridge is reachable:

```bash
nc -zv <ROBOT_IP> 9090   # should say "succeeded"
```

## Connect

```python
from walkie_sdk import WalkieRobot

bot = WalkieRobot(ip="192.168.1.100")    # auto-connects
# bot.arm is ready as soon as the constructor returns; no extra setup needed.
```

Prefer driving from a browser instead? Install the web extra and run
`walkie-web --ip 192.168.1.100`. See the
[Web Interface guide](web-interface.md) for routes.

## MoveIt groups

Every action takes a `group_name`. The supported groups:

| Group | Joints | Use |
|-------|--------|-----|
| `left_arm` | 7 left-arm joints | left arm only, no lift |
| `right_arm` | 7 right-arm joints | right arm only, no lift |
| `left_arm_lift` | 7 left + lift | left arm coordinated with lift |
| `right_arm_lift` | 7 right + lift | right arm coordinated with lift |
| `both_arms` | 14 (7 + 7) | bimanual, no lift |
| `both_arms_lift` | 15 (7 + 7 + lift) | full bimanual + lift |
| `left_gripper` / `right_gripper` | gripper finger | only valid for `control_gripper` |

`bot.arm.left` and `bot.arm.right` are pre-bound to `left_arm_lift` and
`right_arm_lift` respectively so you can skip the `group_name=` argument on
the common path.

## Cartesian pose (Euler RPY)

Move an end-effector to an absolute pose in `frame_id` (default
`base_footprint`):

```python
result = bot.arm.go_to_pose(
    x=0.38, y=0.19, z=0.58,
    roll=-1.57, pitch=0.0, yaw=1.57,
    group_name="left_arm_lift",
    frame_id="base_footprint",
    cartesian_path=False,   # True → plan a straight-line Cartesian path
    blocking=True,          # wait for the action result
)
print(result)  # "SUCCEEDED" | "FAILED" | "IN_PROGRESS"
```

Or with the shortcut:

```python
bot.arm.left.go_to_pose(
    pos=(0.38, 0.19, 0.58),
    rot=(-1.57, 0.0, 1.57),
    cartesian_path=False,
)
```

## Cartesian pose (quaternion)

```python
bot.arm.go_to_pose_quat(
    x=0.38, y=0.19, z=0.58,
    qx=0.0, qy=0.707, qz=0.0, qw=0.707,
    group_name="left_arm_lift",
)
```

## Relative move

`ee_frame=True` interprets the offset in the end-effector's own axes (useful
for "lift the gripper 5 cm up regardless of orientation"). `ee_frame=False`
(default) uses `frame_id` world axes.

```python
bot.arm.go_to_pose_relative(
    x=0.05, y=0.0, z=-0.02,         # 5 cm forward, 2 cm down
    roll=0.0, pitch=0.0, yaw=0.0,
    group_name="left_arm_lift",
    ee_frame=False,
)
```

## Home

```python
bot.arm.go_to_home("left_arm_lift")
bot.arm.go_to_home("both_arms_lift", blocking=False)
```

## Joint position

`set_joint_position` has two modes:

| Mode | Path | Rate | Collision check |
|------|------|------|-----------------|
| `"commander"` (default) | MoveIt action server | ~0.4 Hz | yes |
| `"jtc"` | Direct publish to `*/joint_trajectory` | high-rate | **no** |

```python
# Collision-checked MoveIt plan
bot.arm.set_joint_position(
    "left_arm",
    [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0],
    mode="commander",
    blocking=True,
)

# Fire-and-forget at high rate (teleop, VLA, servo loops)
bot.arm.set_joint_position(
    "left_arm",
    [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0],
    mode="jtc",
    duration=0.5,    # JTC trajectory time-from-start, seconds
)
```

JTC mode publishes exactly 7 joints per arm — `lift_joint` is never bundled
into the JTC message even when `group_name` ends in `_lift`, because the
`*_joint_trajectory_controller` has `allow_partial_joints_goal=false`. Use
the commander mode (which goes through MoveIt) if you need lift+arm
coordinated motion.

## Gripper

`control_gripper` takes a raw position in **meters** in `[0, 0.04]` (fully
closed → fully open):

```python
bot.arm.control_gripper("left_gripper",  position=0.04)  # open
bot.arm.control_gripper("right_gripper", position=0.0)   # closed
```

Or use the `ArmGroup.gripper(value, norm=True)` shortcut which accepts a
normalized `[0, 1]` value by default:

```python
bot.arm.left.gripper(1.0)        # fully open (0.04 m)
bot.arm.right.gripper(0.0)       # fully closed
bot.arm.left.gripper(0.7)        # 70% open
bot.arm.left.gripper(0.025, norm=False)   # raw meters
```

The web `POST /api/arm/gripper` endpoint mirrors this — it defaults to
`norm: true` and reports back the meters value actually sent.

## Reading state

### Live snapshot from the shared hub

`bot.joints` owns the single `/joint_states` subscription. `bot.arm.get_joint_states()`
reads from it and parses the named joints into a usable shape:

```python
states = bot.arm.get_joint_states()
if states:
    print(states["left_arm"]["positions"])    # [7 floats, radians]
    print(states["left_arm"]["velocities"])   # [7 floats, rad/s]
    print(states["left_arm"]["torques"])      # [7 floats]
    print(states["left_gripper"])             # float or None
    print(states["right_gripper"])
```

Returns `None` until the hub has received at least one message.

### One-shot RPC per group

If you want a synchronous RPC instead of reading the topic cache:

```python
left = bot.arm.get_joint_states_service("left_arm", timeout=5.0)
# → {"joint_names": [...], "joint_positions": [...]}
```

### End-effector pose

```python
pose = bot.arm.get_ee_pose("left_arm", frame_id="base_footprint")
# → {"x", "y", "z", "qx", "qy", "qz", "qw", "frame_id"}
```

## From the web API

Every method above has a matching HTTP route — see the
[Web Interface guide](web-interface.md#arm--gripper). Quick example with
curl:

```bash
# Go to a pose
curl -X POST http://localhost:8080/api/arm/pose \
  -H 'Content-Type: application/json' \
  -d '{"x":0.38,"y":0.19,"z":0.58,"roll":-1.57,"pitch":0,"yaw":1.57,
       "group_name":"left_arm_lift","blocking":false}'

# Open the left gripper (normalized)
curl -X POST http://localhost:8080/api/arm/gripper \
  -H 'Content-Type: application/json' \
  -d '{"group_name":"left_gripper","position":1.0,"norm":true}'

# Read the EE pose
curl -X POST http://localhost:8080/api/arm/ee_pose \
  -H 'Content-Type: application/json' \
  -d '{"group_name":"left_arm","frame_id":"base_footprint"}'
```

## Topic / type configuration

If your robot uses different action/service/topic names, override them with
env vars before starting the SDK — no code edits needed:

| Item | Env var | Default |
|------|---------|---------|
| Action interface package | `WALKIE_ARM_ACTION_INTERFACE` | `my_robot_interfaces/action` |
| `go_to_pose` action | `WALKIE_ARM_ACTION_GO_TO_POSE` | `go_to_pose` |
| `go_to_pose_quat` action | `WALKIE_ARM_ACTION_GO_TO_POSE_QUAT` | `go_to_pose_quat` |
| `go_to_pose_relative` action | `WALKIE_ARM_ACTION_GO_TO_POSE_RELATIVE` | `go_to_pose_relative` |
| `go_to_home` action | `WALKIE_ARM_ACTION_GO_TO_HOME` | `go_to_home` |
| `control_gripper` action | `WALKIE_ARM_ACTION_CONTROL_GRIPPER` | `control_gripper` |
| `set_joint_position` action | `WALKIE_ARM_ACTION_SET_JOINT_POSITION` | `set_joint_position` |
| `get_ee_pose` service | `WALKIE_ARM_SVC_GET_EE_POSE` | `get_ee_pose` |
| `get_joint_states` service | `WALKIE_ARM_SVC_GET_JOINT_STATES` | `get_joint_states` |
| JTC left topic | `WALKIE_ARM_JTC_LEFT` | `left_joint_trajectory_controller/joint_trajectory` |
| JTC right topic | `WALKIE_ARM_JTC_RIGHT` | `right_joint_trajectory_controller/joint_trajectory` |
| `joint_states` topic | `WALKIE_JOINT_STATES` | `joint_states` |

For more durable overrides, copy `ros_topics.yaml` and pass it to
`WalkieRobot(config_path="my_config.yaml")`.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Every arm call returns `"FAILED"` immediately | commander not running, or wrong action name | `ros2 action list` — does the commander advertise `/go_to_pose` etc.? |
| `"FAILED"` only on `mode="jtc"` | JTC controllers not running, or wrong topic name | `ros2 topic info /left_joint_trajectory_controller/joint_trajectory` |
| `get_joint_states()` returns `None` forever | nothing publishing `/joint_states` | start `joint_state_broadcaster` (or equivalent) on the robot |
| Connect succeeds but `get_ee_pose` times out | service not advertised under expected name | `ros2 service list \| grep ee_pose`; override `WALKIE_ARM_SVC_GET_EE_POSE` |
| Calls work in `commander` mode but plan paths look wrong | MoveIt's planning frame ≠ your `frame_id` | pass an explicit `frame_id=` that matches MoveIt's planning frame |
| Sporadic timeouts on rosbridge | wrong IP / port / firewall | `nc -zv <ROBOT_IP> 9090` |
