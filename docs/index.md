# Walkie SDK

A pure Python SDK for controlling Walkie robots with **pluggable protocol support**.

Choose between WebSocket (ROSBridge), Zenoh, or mix-and-match per subsystem.
**No ROS 2 installation required on the client machine.**

## Key Features

- **Protocol-agnostic** -- switch between rosbridge and zenoh without changing application code
- **Multi-camera** -- head camera over Zenoh, wrist camera over USB, all through a unified API
- **Full robot control** -- navigation, dual-arm manipulation, gripper, visualization
- **Zero ROS 2 dependency** -- runs on any Python 3.11+ machine

## Quick Install

```bash
uv add walkie-sdk
```

## 10-Line Quick Start

```python
from walkie_sdk import WalkieRobot

bot = WalkieRobot(ip="192.168.1.100")

# Read telemetry
pose = bot.status.get_pose()
print(f"Robot at ({pose['x']:.2f}, {pose['y']:.2f})")

# Get a camera frame
frame = bot.camera.get_frame()

# Navigate
bot.nav.go_to(x=2.0, y=1.0, heading=0.0)

bot.disconnect()
```

## What's Inside

| Module | Access | Description |
|--------|--------|-------------|
| [Navigation](api/navigation.md) | `bot.nav` | Go-to-pose, cancel, emergency stop |
| [Telemetry](api/telemetry.md) | `bot.status` | Pose, velocity, raw odometry |
| [Camera](api/camera.md) | `bot.camera` | Single-camera frame access |
| [MultiCamera](api/multi-camera.md) | `bot.cameras` | Named multi-camera access |
| [Arm](api/arm.md) | `bot.arm` | Joint control, IK, gripper |
| [Visualization](api/visualization.md) | `bot.viz` | RViz2 markers, poses, axis triads |
| [Tools](api/tools.md) | `bot.tools` | 2D-to-3D bbox projection |

## Next Steps

- [Installation](getting-started/installation.md) -- full install guide
- [Quick Start](getting-started/quick-start.md) -- step-by-step walkthrough
- [Protocol Selection](guides/protocols.md) -- choose the right transport
- [Camera Setup](guides/cameras.md) -- single, multi, and mixed-transport cameras
