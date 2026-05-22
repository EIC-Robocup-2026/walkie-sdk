# Walkie SDK — Testing Guide

This directory holds two kinds of tests:

| Layer | Location | How to run | Needs a robot? |
|-------|----------|------------|----------------|
| **Unit tests** | `tests/unit/` | `uv run pytest` | No — pure/offline |
| **Hardware integration scripts** | `tests/test_*.py` | `uv run python tests/<name>.py --ip <ip>` | Yes (live robot or sim) |
| **Runner** | `tests/run_tests.sh` | `tests/run_tests.sh --ip <ip> [suite]` | Depends on suite |

The hardware scripts are **not** collected by `pytest` (`pyproject.toml` pins `testpaths = ["tests/unit"]`). They are standalone CLIs you run by hand against a running robot.

> ⚠️ Scripts marked **MOVES HARDWARE** physically move the robot. Read the [Safety](#safety) section first. Motion scripts prompt `[y/N]` before each move unless you pass `--yes`.

---

## Table of contents

- [Quick start](#quick-start)
- [Prerequisites](#prerequisites)
- [Configuration & overriding topic names](#configuration--overriding-topic-names)
- [The runner: `run_tests.sh`](#the-runner-run_testssh)
- Per-script reference:
  - [`test_connection.py`](#test_connectionpy) — lifecycle / namespace / config
  - [`test_telemetry.py`](#test_telemetrypy) — `bot.status`
  - [`test_multi_camera.py`](#test_multi_camerapy) — `bot.camera` / `bot.cameras`
  - [`test_visualization.py`](#test_visualizationpy) — `bot.viz`
  - [`test_lift.py`](#test_liftpy) — `bot.lift` ⚠️
  - [`test_navigation.py`](#test_navigationpy) — `bot.nav` ⚠️
  - [`test_arm.py`](#test_armpy) — `bot.arm` ⚠️
  - [`test_tools_bbox.py`](#test_tools_bboxpy) — `bot.tools`
- [Safety](#safety)
- [Troubleshooting quick-reference](#troubleshooting-quick-reference)

---

## Quick start

```bash
# 0. Set your robot IP once (used in all examples below)
export IP=192.168.1.100

# 1. Offline sanity (no robot)
uv run pytest                              # unit tests
tests/run_tests.sh --offline              # SDK import/config check

# 2. Zero-motion sweep against the robot
tests/run_tests.sh --ip "$IP" --safe --stop-on-fail

# 3. Motion tests, one module at a time (prompts before each move)
uv run python tests/test_lift.py --ip "$IP"
uv run python tests/test_navigation.py --ip "$IP"
uv run python tests/test_arm.py --ip "$IP" --mode moveit
```

---

## Prerequisites

**On the robot** — launch the servers the layer you're testing needs:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # ALL scripts
ros2 launch nav2_bringup navigation_launch.py                 # navigation (+ localization/map)
# MoveIt / arm action servers ............................... arm (moveit mode)
# custom IK node subscribed to /target_pose ................. arm (custom_ik mode)
# lift driver node ......................................... lift
# zenoh camera bridge ...................................... cameras
# perception get_3d_poses service ......................... tools
```

**On your laptop** — confirm reachability before running anything:

```bash
nc -zv "$IP" 9090     # rosbridge port (TCP) open?
```

No ROS 2 is required on the laptop — the SDK talks over rosbridge/zenoh.

---

## Configuration & overriding topic names

Almost every hardware-test failure is a **topic / type / service name mismatch** between what the script uses and what your robot actually publishes. You can fix these **without editing code** in three ways.

### 1. Namespace (when everything is just prefixed)

```bash
uv run python tests/test_telemetry.py --ip "$IP" --namespace robot1
# odom topic becomes robot1/current_pose, cmd_vel becomes robot1/cmd_vel, etc.
```

### 2. Environment variables (whole shell session)

Every entry in `walkie_sdk/config/ros_topics.py` has an env override. Set it, then run:

| Module | Env var | Default |
|--------|---------|---------|
| Telemetry | `WALKIE_TELEMETRY_ODOM` / `..._TYPE` | `current_pose` / `nav_msgs/msg/Odometry` |
| Navigation | `WALKIE_NAV_ACTION_NAV2` / `..._TYPE` | `navigate_to_pose` / `nav2_msgs/action/NavigateToPose` |
| Navigation | `WALKIE_NAV_CMD_VEL` / `..._TYPE` | `cmd_vel` / `geometry_msgs/msg/Twist` |
| Arm | `WALKIE_ARM_STATES` / `WALKIE_ARM_COMMANDS` | `joint_states` / `walkie/arm/commands` |
| Arm | `WALKIE_ARM_TARGET_POSE` | `/target_pose` |
| Arm | `WALKIE_ARM_ACTION_INTERFACE` | `my_robot_interfaces/action` |
| Lift | `WALKIE_LIFT_CMD` / `WALKIE_LIFT_STATES` | `lift/cmd` / `lift/joint_states` |
| Camera | `WALKIE_CAM_HEAD` / `_LEFT` / `_RIGHT` | `/zed_head/.../image` / `/walkie/camera/left` / `right` |
| Visualization | `WALKIE_VIZ_MARKERS` / `_ARRAY` / `WALKIE_VIZ_TARGET_POSE` | `walkie/viz_markers` / `_array` / `walkie/target_pose` |
| Tools | `WALKIE_OB_POSE_SERVICE_NAME` / `..._TYPE` | `get_3d_poses` / `perception/srv/GetObPose` |
| Zenoh | `WALKIE_ROS_DOMAIN_ID` | `23` (must match the robot for zenoh cameras) |

```bash
export WALKIE_TELEMETRY_ODOM=/odom
export WALKIE_ARM_STATES=/joint_states
uv run python tests/test_telemetry.py --ip "$IP"
```

### 3. YAML config file

Copy the repo's `ros_topics.yaml`, edit the wrong entries, and load it. Verify the mechanism offline:

```bash
uv run python tests/test_connection.py --offline-only   # TEST 2 proves YAML override works
```

`load_config()` mutates the topic dicts **in place**, so it must run before the topics are used. The hardware scripts don't expose a `--config` flag yet — use env vars/namespace for now, or ask to have `--config` added.

To discover the right names, on any ROS 2 machine on the same graph:

```bash
ros2 topic list
ros2 topic info /<topic> --verbose     # confirm the message TYPE matches too
ros2 action list
ros2 service list
```

---

## The runner: `run_tests.sh`

Drives the scripts in ladder order and prints a per-script pass/fail summary.

```
tests/run_tests.sh [--ip IP] [--port PORT] [--namespace NS] [SUITE] [options]
```

| Suite flag | Runs | Motion |
|------------|------|--------|
| `--offline` | `test_connection.py --offline-only` | none (no robot) |
| `--safe` *(default)* | connection → telemetry → cameras → visualization | none |
| `--motion` | lift → navigation → arm | ⚠️ yes |
| `--all` | safe + motion | ⚠️ yes |

| Option | Meaning |
|--------|---------|
| `--ip IP` | robot IP (default `127.0.0.1`) |
| `--port PORT` | rosbridge port (default `9090`) |
| `--namespace NS` | passed to scripts that accept it (not `test_connection.py`) |
| `--yes` / `-y` | skip the runner's motion gate **and** pass `--yes` to nav/arm |
| `--stop-on-fail` | halt at the first non-zero exit |
| `--camera-extra "..."` | extra args for the camera script, e.g. `"--multi --show"` |
| `--arm-mode MODE` | `moveit` (default) / `custom_ik` / `both` |

**Examples**

```bash
tests/run_tests.sh --offline                                            # no robot
tests/run_tests.sh --ip "$IP"                                           # safe sweep (default)
tests/run_tests.sh --ip "$IP" --safe --stop-on-fail                     # stop at first break
tests/run_tests.sh --ip "$IP" --safe --camera-extra "--multi --show"    # safe + camera windows
tests/run_tests.sh --ip "$IP" --namespace robot1 --safe                 # namespaced robot
tests/run_tests.sh --ip "$IP" --motion                                  # motion only (prompts)
tests/run_tests.sh --ip "$IP" --all --arm-mode both                     # everything, both arm modes
tests/run_tests.sh --ip "$IP" --all --yes                               # everything, NO prompts (only when trusted)
tests/run_tests.sh --help
```

The runner uses each script's own exit code (0 = all passed) and exits non-zero if any script failed — CI-friendly.

---

## `test_connection.py`

**Module:** `WalkieRobot` facade. **Motion:** none.
Verifies connect/disconnect, `is_connected`, context manager, identity/protocol properties, namespace fan-out to all modules, invalid-protocol handling, legacy params, and YAML config override.

**Server needs:** rosbridge (the two offline tests run with no robot).

| Param | Default | Notes |
|-------|---------|-------|
| `--ip` | `127.0.0.1` | robot IP |
| `--port` | `9090` | rosbridge port |
| `--offline-only` | off | run only the 2 no-robot checks (invalid protocol, YAML override) |

**Commands**

```bash
# No robot — proves SDK + config loading are sane
uv run python tests/test_connection.py --offline-only

# Full lifecycle against the robot
uv run python tests/test_connection.py --ip "$IP"
uv run python tests/test_connection.py --ip "$IP" --port 9090
```

**Expect:** `7 passed` online (or `2 passed` with `--offline-only`). The script opens/closes several short connections — printing connect/disconnect banners is normal.

**If it fails:** can't connect → rosbridge down or wrong `--ip/--port`. The two offline tests should pass regardless; if they don't, reinstall with `uv sync`.

---

## `test_telemetry.py`

**Module:** `bot.status`. **Motion:** none (read-only).
Checks `get_position()`, `get_velocity()`, `get_raw_odom()`, `has_data`, and live streaming.

**Server needs:** odometry publisher on the configured odom topic (`current_pose`, `nav_msgs/msg/Odometry`).

| Param | Default | Notes |
|-------|---------|-------|
| `--ip` | `127.0.0.1` | |
| `--port` | `9090` | |
| `--timeout` | `10.0` | seconds to wait for the first odom message |
| `--samples` | `20` | number of live samples streamed in test 5 (~10 Hz) |
| `--namespace` | `""` | topic prefix |

**Commands**

```bash
uv run python tests/test_telemetry.py --ip "$IP"
uv run python tests/test_telemetry.py --ip "$IP" --timeout 20            # slow-to-start odom
uv run python tests/test_telemetry.py --ip "$IP" --samples 100          # watch longer
uv run python tests/test_telemetry.py --ip "$IP" --namespace robot1
WALKIE_TELEMETRY_ODOM=/odom uv run python tests/test_telemetry.py --ip "$IP"   # different topic
```

**Tip:** during the streaming test, gently push or teleop the robot so x/y/heading and velocity visibly change.

**If "no odom within Ns":** the odom topic name/type differs — set `WALKIE_TELEMETRY_ODOM` (and `..._TYPE` if needed) or `--namespace`, then re-run. `get_velocity()` returning None means your odom has no `twist` field.

---

## `test_multi_camera.py`

**Module:** `bot.camera` (single) and `bot.cameras` (`MultiCamera`). **Motion:** none.

> **Known limitation (documented by this test):** `WalkieRobot` builds a *single-camera* `ZenohCamera`, and `MultiCamera` only switches to multi-cam mode for a transport that subclasses `MultiCameraTransportInterface` — which `ZenohCamera` does not. So through `bot.cameras` you only ever get the **head** frame, regardless of the name passed, and the README's `cameras={...}` constructor arg is **not** implemented. Use `--multi` to pull head/left/right by talking to `ZenohCamera(multi_camera=True)` directly.

**Server needs:** zenoh camera bridge publishing image topics (see `CAMERA_TOPICS`). Zenoh uses `WALKIE_ROS_DOMAIN_ID` (default 23) — it must match the robot.

| Param | Default | Notes |
|-------|---------|-------|
| `--ip` | `127.0.0.1` | |
| `--port` | `9090` | rosbridge port |
| `--camera-port` | `7447` | zenoh router port |
| `--timeout` | `10.0` | seconds to wait for frames |
| `--namespace` | `""` | |
| `--multi` | off | also test direct `ZenohCamera(multi_camera=True)` (head/left/right) |
| `--show` | off | display frames in OpenCV windows |

**Commands**

```bash
uv run python tests/test_multi_camera.py --ip "$IP"                          # head only, headless
uv run python tests/test_multi_camera.py --ip "$IP" --show                   # see the feed
uv run python tests/test_multi_camera.py --ip "$IP" --multi --show           # all cameras
uv run python tests/test_multi_camera.py --ip "$IP" --camera-port 7447 --timeout 20
WALKIE_CAM_HEAD=/my/head/image uv run python tests/test_multi_camera.py --ip "$IP" --show
WALKIE_ROS_DOMAIN_ID=30 uv run python tests/test_multi_camera.py --ip "$IP" --multi   # match robot domain
```

**If no frames:** wrong camera topic (`WALKIE_CAM_*`), wrong `--camera-port`, or a `WALKIE_ROS_DOMAIN_ID` mismatch. `is_streaming=True` only means the subscriber started, not that frames arrived.

---

## `test_visualization.py`

**Module:** `bot.viz` + `WalkieRobot` convenience wrappers. **Motion:** none.
Covers `draw_marker` (all types), `update_marker`, `draw_markers` (array), `delete_marker`, `clear_markers`, `draw_pose`/`update_pose`, the `bot.draw_*` shortcuts, and expected `KeyError`s.

**Server needs:** rosbridge only. **Open RViz2** to see results: set **Fixed Frame** = your `--frame`, then add displays:
- `Marker` → `walkie/viz_markers`
- `MarkerArray` → `walkie/viz_markers_array`
- `Pose` → `walkie/target_pose`
(prefix with your namespace if set).

| Param | Default | Notes |
|-------|---------|-------|
| `--ip` | `127.0.0.1` | |
| `--port` | `9090` | |
| `--frame` | `base_link` | TF frame for markers — must exist in your TF tree / be the RViz fixed frame |
| `--delay` | `0.5` | seconds between draws (raise to watch in RViz2) |
| `--namespace` | `""` | |

**Commands**

```bash
uv run python tests/test_visualization.py --ip "$IP"                       # fast, programmatic
uv run python tests/test_visualization.py --ip "$IP" --delay 1.5           # slow enough to watch
uv run python tests/test_visualization.py --ip "$IP" --frame map           # if fixed frame is map
uv run python tests/test_visualization.py --ip "$IP" --namespace robot1
```

**Expect:** `8/8 passed`, and shapes/poses visibly appear in RViz2. Programmatic PASS only proves the publish succeeded — confirm visually.

**If nothing in RViz2:** the RViz Fixed Frame doesn't match `--frame`, the display topic is wrong, or your namespace isn't reflected in the RViz topic name.

---

## `test_lift.py`

**Module:** `bot.lift`. ⚠️ **MOVES THE LIFT.**
Reads initial position, then moves to bottom / top / midpoint / a real-cm target, tests non-blocking + status polling, and custom speed/accel.

**Server needs:** lift driver — command topic `lift/cmd` (`Float64MultiArray`), feedback `lift/joint_states` (`JointState`). Travel range 0–74.35 cm.

| Param | Default | Notes |
|-------|---------|-------|
| `--ip` | `127.0.0.1` | |
| `--port` | `9090` | |
| `--timeout` | `60.0` | max seconds to wait for each move (lift is slow) |
| `--tolerance` | `0.02` | normalized position tolerance (≈1.5 cm) |
| `--namespace` | `""` | |

**Commands**

```bash
uv run python tests/test_lift.py --ip "$IP"
uv run python tests/test_lift.py --ip "$IP" --timeout 90               # very slow lift
uv run python tests/test_lift.py --ip "$IP" --tolerance 0.03           # if it just misses target
uv run python tests/test_lift.py --ip "$IP" --namespace robot1
```

**If TIMEOUT while sitting at target:** raise `--timeout` first, then loosen `--tolerance`. **If no initial data:** wrong `WALKIE_LIFT_STATES` topic.

---

## `test_navigation.py`

**Module:** `bot.nav`. ⚠️ **MOVES THE BASE.**
Tests initial status, blocking `go_to`, non-blocking + `cancel`, emergency `stop`, and `feedback_callback`. Each move is gated by a `[y/N]` prompt (unless `--yes`).

**Server needs:** Nav2 `navigate_to_pose` action **and localization/map** (goals are in the `map` frame). Telemetry is used to compute the default goal.

| Param | Default | Notes |
|-------|---------|-------|
| `--ip` | `127.0.0.1` | |
| `--port` | `9090` | |
| `--timeout` | `60.0` | max seconds per blocking move |
| `--namespace` | `""` | |
| `--dx` | `0.0` | X offset (m) from current pose for the default goal |
| `--dy` | `0.0` | Y offset (m) from current pose for the default goal |
| `--goal-x` | none | absolute map-frame X (overrides `--dx`) |
| `--goal-y` | none | absolute map-frame Y (overrides `--dy`) |
| `--goal-heading` | none | goal heading (rad); defaults to current heading |
| `--yes` / `-y` | off | skip all confirmation prompts |

> **Default goal = current pose** (dx=dy=0), so the base barely moves while still exercising the full Nav2 handshake. Increase `--dx/--dy` for real travel, or give absolute `--goal-*`.

**Commands**

```bash
# First contact — near no-op, validates the action pipeline
uv run python tests/test_navigation.py --ip "$IP"

# Small real nudge forward (30 cm in map +X)
uv run python tests/test_navigation.py --ip "$IP" --dx 0.3

# Sideways nudge + face +Y
uv run python tests/test_navigation.py --ip "$IP" --dy 0.3 --goal-heading 1.57

# Absolute map goal
uv run python tests/test_navigation.py --ip "$IP" --goal-x 2.0 --goal-y 1.0 --goal-heading 0.0

# Longer planning window, namespaced
uv run python tests/test_navigation.py --ip "$IP" --namespace robot1 --timeout 120

# No prompts (only after you trust it)
uv run python tests/test_navigation.py --ip "$IP" --dx 0.3 --yes
```

**If goals fail immediately:** Nav2 or localization isn't running, or there's no map → the `map` frame is unavailable. **Frame note:** the goal is sent in `map`, while telemetry reads odom — if they differ, prefer explicit `--goal-*`. Press `Ctrl-C` any time; the script sends `stop()` on interrupt.

---

## `test_arm.py`

**Module:** `bot.arm` (consolidated; supersedes the ad-hoc `test_call_action.py` / `test_call_pose_quaternion.py`). ⚠️ **MOVES THE ARMS.**
Read-only checks first (`get_joint_states`, the unimplemented velocity/torque stubs), then per-mode motion tests, each gated by a prompt.

**Mode routing:**
- `set_joint_positions` — publishes `JointState` (mode-independent)
- `control_gripper`, `go_to_home`, `go_to_pose_relative` — **action only** → auto-skipped in `custom_ik`
- `go_to_pose`, `go_to_pose_quaternion`, `go_to_pose_quaternion_move_action` — route by mode (MoveIt action vs publishing a `PoseStamped`)

**Server needs:** MoveIt/robot action servers (`my_robot_interfaces`) for `moveit`; an IK node subscribed to the target-pose topic for `custom_ik`; plus the `joint_states` publisher.

| Param | Default | Notes |
|-------|---------|-------|
| `--ip` | `127.0.0.1` | |
| `--port` | `9090` | |
| `--namespace` | `""` | |
| `--mode` | `moveit` | `moveit` / `custom_ik` / `both` |
| `--group` | `left_arm` | MoveIt planning group |
| `--gripper-group` | `left_gripper` | gripper group |
| `--target-pose-topic` | `/target_pose` | custom_ik publish topic |
| `--timeout` | `10.0` | seconds to wait for joint states |
| `--x` `--y` `--z` | `0.38` `0.19` `0.58` | target position (m) — tuned for **left** arm |
| `--roll` `--pitch` `--yaw` | `-1.57` `0.0` `0.0` | target orientation (rad) |
| `--yes` / `-y` | off | skip confirmation prompts |

**Commands**

```bash
# MoveIt (planned, collision-aware) — start here
uv run python tests/test_arm.py --ip "$IP" --mode moveit

# Right arm — mirror the Y target to negative
uv run python tests/test_arm.py --ip "$IP" --group right_arm --gripper-group right_gripper --y -0.19

# Custom-IK teleop publishing to a specific topic
uv run python tests/test_arm.py --ip "$IP" --mode custom_ik --target-pose-topic /left_arm/target_pose

# Both modes back to back
uv run python tests/test_arm.py --ip "$IP" --mode both

# Custom safe target pose
uv run python tests/test_arm.py --ip "$IP" --x 0.30 --y 0.15 --z 0.50 --roll -1.57 --pitch 0 --yaw 0

# Namespaced, no prompts (trusted only)
uv run python tests/test_arm.py --ip "$IP" --namespace robot1 --mode moveit --yes
```

**Recommended first pass:** let the read-only checks confirm joint states look sane, then answer `y` only to the small/safe moves (joint nudge, gripper, home) and `n` to the larger pose moves until you trust the targets.

**If pose moves report FAILED:** the action server name/type doesn't match (`WALKIE_ARM_ACTION_INTERFACE`), the group name is wrong, or (custom_ik) no IK node is listening on the target-pose topic.

---

## `test_tools_bbox.py`

**Module:** `bot.tools.bboxes_to_positions()`. **Motion:** none.
Calls the perception service to turn 2D bounding boxes into 3D positions. (An interactive variant, `test_tools_bbox_interactive.py`, overlays results on the camera feed.)

**Server needs:** `get_3d_poses` service (`perception/srv/GetObPose`).

| Param | Default | Notes |
|-------|---------|-------|
| `--ip` | `127.0.0.1` | |
| `--port` | `9090` | |
| `--timeout` | `5.0` | service-call timeout |
| `--namespace` | `""` | |

**Commands**

```bash
uv run python tests/test_tools_bbox.py --ip "$IP"
uv run python tests/test_tools_bbox.py --ip "$IP" --timeout 10
uv run python tests/test_tools_bbox.py --ip "$IP" --namespace robot1
# Interactive (needs zenoh camera): adds --cam-port
uv run python tests/test_tools_bbox_interactive.py --ip "$IP" --cam-port 7447
```

**If it returns None / times out:** wrong `WALKIE_OB_POSE_SERVICE_NAME`/`_TYPE`, or the perception node isn't running.

---

## Safety

- **Clear the workspace** and keep a hardware **e-stop within reach** for `test_lift.py`, `test_navigation.py`, `test_arm.py`.
- **Never pass `--yes`** to a motion script you haven't already watched succeed at least once.
- **Start small:** nav defaults to a near-no-op; arm targets should be known-reachable points; decline (`n`) large moves on the first pass.
- **Prefer `moveit` over `custom_ik`** for first arm runs — MoveIt plans around collisions; custom_ik publishes a raw target with no planning.
- **`Ctrl-C`** aborts; the navigation script sends `stop()` on interrupt.

---

## Troubleshooting quick-reference

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Can't connect at all | rosbridge down / wrong IP-port / firewall | `nc -zv "$IP" 9090`; launch `rosbridge_websocket` |
| Telemetry "no odom" | odom topic/type mismatch | `WALKIE_TELEMETRY_ODOM=...` or `--namespace` |
| `get_velocity()` None | odom has no twist | expected on some sources; not a real failure |
| No camera frames | camera topic / port / ROS domain mismatch | `WALKIE_CAM_*`, `--camera-port`, `WALKIE_ROS_DOMAIN_ID` |
| `bot.cameras` only head | `MultiCamera` limitation (see note) | use `--multi` / `ZenohCamera(multi_camera=True)` |
| Nothing in RViz2 | fixed frame / topic / namespace mismatch | match `--frame` to RViz fixed frame; check display topics |
| Lift TIMEOUT at target | too tight / too slow | raise `--timeout`, then `--tolerance` |
| Nav fails instantly | Nav2/localization/map missing | launch Nav2 + AMCL/SLAM; use absolute `--goal-*` |
| Arm pose FAILED | action name/type/group wrong, or no IK node | `WALKIE_ARM_ACTION_INTERFACE`, `--group`, custom_ik topic |
| Tools returns None | service name/type wrong or node down | `WALKIE_OB_POSE_SERVICE_NAME/_TYPE` |

When a script fails it prints the **exact topic/action/service it used** — copy that, compare against `ros2 topic list` / `ros2 action list` / `ros2 service list`, then override and re-run just that script.
