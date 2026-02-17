# Walkie SDK

A pure Python SDK for controlling Walkie robots with **pluggable protocol support**. Choose between WebSocket (ROSBridge), Zenoh, or mix-and-match camera transports based on your needs.

**No ROS 2 installation required on the client machine** (when using rosbridge or zenoh protocols).

> **Full documentation:** [walkie-sdk docs](https://EIC-Robocup-2026.github.io/walkie-sdk/)
> (run `mkdocs serve` locally to preview)

## Architecture

```
 ┌─────────────────────────────────────────────────────────────────────────┐
 │                              YOUR LAPTOP                                │
 │  ┌───────────────────────────────────────────────────────────────────┐  │
 │  │                          walkie_sdk                               │  │
 │  │  ┌──────────┐ ┌──────────┐ ┌───────────┐ ┌──────────┐ ┌────────┐  │  │
 │  │  │ bot.nav  │ │bot.status│ │bot.camera │ │ bot.arm  │ │bot.viz │  │  │
 │  │  │ • go_to  │ │• get_pose│ │• get_frame│ │• set_pos │ │• point │  │  │
 │  │  │ • cancel │ │• get_vel │ │• named    │ │• gripper │ │• clear │  │  │
 │  │  └────┬─────┘ └────┬─────┘ └────┬──────┘ └────┬─────┘ └───┬────┘  │  │
 │  │       └──────┬─────┴────────────┴─────────────┴───────────┘       │  │
 │  │              │                                                    │  │
 │  │    ┌─────────▼───────────────────────────────────────────────┐    │  │
 │  │    │              TransportFactory                           │    │  │
 │  │    │  ┌───────────┐ ┌──────────┐ ┌──────────┐                │    │  │
 │  │    │  │rosbridge  │ │  zenoh   │ │   usb    │                │    │  │
 │  │    │  │(WebSocket)│ │  (DDS)   │ │(V4L2/cv2)│                │    │  │
 │  │    │  └────┬──────┘ └────┬─────┘ └────┬─────┘                │    │  │
 │  │    └───────┼─────────────┼────────────┼──────────────────────┘    │  │
 │  └────────────┼─────────────┼────────────┼───────────────────────────┘  │
 └───────────────┼─────────────┼────────────┼──────────────────────────────┘
                 │             │            │
                 ▼             ▼            ▼
              Robot         Robot       Local USB
             :9090         :7447        /dev/video*
```

## Protocol Support

| Protocol | ROS2 Required | Performance | Status |
|----------|---------------|-------------|--------|
| `rosbridge` | No | Medium | Implemented |
| `zenoh` | No | Good | Implemented |

| Camera Protocol | Use Case | Status |
|-----------------|----------|--------|
| `zenoh` | Remote (pairs with zenoh) | Implemented |
| `usb` | Local USB webcam / robot-mounted camera | Implemented |
| `none` | Disable camera | Implemented |

## Installation

```bash
# Using UV (recommended)
uv add walkie-sdk

# Using pip
pip install walkie-sdk

# From source
git clone https://github.com/EIC-Robocup-2026/walkie-sdk.git
cd walkie-sdk
uv sync
```

## Quick Start

```python
from walkie_sdk import WalkieRobot

bot = WalkieRobot(ip="192.168.1.100")

# Telemetry
pose = bot.status.get_pose()
print(f"Robot at: x={pose['x']:.2f}, y={pose['y']:.2f}")

# Camera
frame = bot.camera.get_frame()

# Navigation
result = bot.nav.go_to(x=2.0, y=1.0, heading=0.0)

# Arm control
bot.arm.set_position(x=0.3, y=0.0, z=0.2)
bot.arm.gripper(open=True)

bot.disconnect()
```

### Mixed Camera Transports

Use different protocols for different cameras:

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    cameras={
        "head": {"protocol": "zenoh"},
        "wrist": {
            "protocol": "usb",
            "device": "/dev/v4l/by-id/usb-WristCam-video-index0",
        },
    },
)

# Access by name
head_frame = bot.cameras.get_frame("head")
wrist_frame = bot.cameras.get_frame("wrist")

# Or get all at once
all_frames = bot.cameras.get_all_frames()
```

> **Tip:** Use stable device paths (`ls /dev/v4l/by-id/`) instead of integer indices for USB cameras.

### Context Manager

```python
with WalkieRobot(ip="192.168.1.100") as bot:
    bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
    # Auto-disconnects when exiting
```

## Server Requirements

The robot must run ROS 2 with the appropriate servers:

```bash
# ROSBridge (for rosbridge protocol)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Nav2 (for navigation)
ros2 launch nav2_bringup navigation_launch.py

# Or use the combined launch file
ros2 launch walkie_bringup robot_server.launch.py
```

## Documentation

For full API reference, guides, and examples, see the **[documentation site](https://EIC-Robocup-2026.github.io/walkie-sdk/)**.

To build and preview locally:

```bash
uv pip install -e ".[docs]"
mkdocs serve
```

## License

MIT License - see [LICENSE](LICENSE) for details.

## Contributing

Contributions welcome! Please read the contributing guidelines before submitting PRs.
