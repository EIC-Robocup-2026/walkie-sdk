# Quick Start

This guide walks through connecting to a robot and using each module.

## 1. Connect

```python
from walkie_sdk import WalkieRobot

bot = WalkieRobot(ip="192.168.1.100")
```

This auto-connects using the default protocols (rosbridge + Zenoh camera).
See [Protocol Selection](../guides/protocols.md) for other options.

### Custom Configuration

You can provide a custom configuration file to override default topic names and settings:

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    config_path="ros_topics.yaml"
)
```

This allows you to customize ROS topics, actions, and other parameters for your specific robot setup. See the [ros_topics.yaml](../../ros_topics.yaml) file in the repository root for the configuration format and available options.

## 2. Read Telemetry

```python
pose = bot.status.get_position()
if pose:
    print(f"Position: ({pose['x']:.2f}, {pose['y']:.2f})")
    print(f"Heading: {pose['heading']:.2f} rad")

velocity = bot.status.get_velocity()
if velocity:
    print(f"Speed: {velocity['linear']:.2f} m/s")

pc_info = bot.status.get_point_cloud_info()
if pc_info:
    print(
        f"ZED Point Cloud: {pc_info.get('width')}x{pc_info.get('height')} "
        f"fields={pc_info.get('fields')}"
    )

    full_cloud = bot.status.get_full_point_cloud()
    if full_cloud:
        print(f"Extracted: {len(full_cloud)} points")
        print(f"First 3: {full_cloud[:3]}")
        print(f"Last 3:  {full_cloud[-3:]}")
```

## 3. Camera

```python
import cv2

frame = bot.camera.get_frame()
if frame is not None:
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)
```

For multiple cameras, see [Camera Setup](../guides/cameras.md):

```python
frames = bot.cameras.get_all_frames()
for name, frame in frames.items():
    cv2.imshow(name, frame)
```

## 4. Navigate

```python
# Blocking -- waits until the robot arrives
result = bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
print(result)  # "SUCCEEDED" or "FAILED"

# Non-blocking -- returns immediately
bot.nav.go_to(x=5.0, y=3.0, heading=1.57, blocking=False)
print(bot.nav.status)  # "IN_PROGRESS"

# Emergency stop
bot.nav.stop()
```

## 5. Arm Control

```python
# Move arm to a Cartesian pose (Euler angles)
bot.arm.go_to_pose(
    group_name="left_arm",
    x=0.38, y=0.19, z=0.58,
    roll=-1.57, pitch=0.0, yaw=1.57,
)

# Read joint states
states = bot.arm.get_joint_states()
print(states["left_arm"]["positions"])

# Control gripper
bot.arm.control_gripper("left_gripper", position=0.7)  # close
```

See [Arm Control Guide](../guides/arm-control.md) for full details.

## 6. Visualization

```python
from walkie_sdk import SPHERE

# Draw a marker in RViz2
marker_id = bot.viz.draw_marker(
    position=[1.0, 2.0, 0.0],
    # Defaults to red arrow
)

# Draw a PoseStamped (triad)
bot.viz.draw_pose(
    position=[0.5, 0.0, 0.3],
    topic="walkie/target_pose/left_arm",
)
```

See [Visualization Guide](../guides/visualization.md) for full details.

## 7. Disconnect

```python
bot.disconnect()
```

Or use a context manager for automatic cleanup:

```python
with WalkieRobot(ip="192.168.1.100") as bot:
    bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
    # auto-disconnects on exit
```
