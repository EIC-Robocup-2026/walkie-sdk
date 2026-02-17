# Installation

## Requirements

- Python 3.11 or later
- No ROS 2 installation required on the client machine

## Using uv (recommended)

```bash
uv add walkie-sdk
```

## Using pip

```bash
pip install walkie-sdk
```

## From Source

```bash
git clone https://github.com/walkie-team/walkie-sdk.git
cd walkie-sdk
uv sync
```

## Optional Dependencies

### Documentation

To build the docs site locally:

```bash
uv pip install -e ".[docs]"
mkdocs serve
```

### Development

```bash
uv pip install -e ".[dev]"
pytest
```

## Robot-Side Requirements

The robot must run ROS 2 with the appropriate servers depending on your protocol choice:

| Component | Port | Required For |
|-----------|------|--------------|
| rosbridge_server | 9090 | `ros_protocol="rosbridge"` |
| WebRTC camera server | 8554 | `camera_protocol="webrtc"` |
| Zenoh router | 7447 | `ros_protocol="zenoh"` |
| Nav2 stack | -- | Navigation (`bot.nav`) |
| MoveIt / IK solver | -- | Arm control (`bot.arm`) |

```bash
# Example: launch rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Example: launch with combined bringup
ros2 launch walkie_bringup robot_server.launch.py
```
