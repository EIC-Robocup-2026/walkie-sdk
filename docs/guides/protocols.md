# Protocol Selection

The SDK supports multiple communication protocols. Choose based on your setup.

## ROS Protocols

| Protocol | ROS 2 on Client | Latency | Status |
|----------|-----------------|---------|--------|
| `rosbridge` | Not required | Medium | Implemented |
| `zenoh` | Not required | Low | Implemented |
| `auto` | Not required | -- | Auto-detects best |

## Camera Protocols

| Protocol | Pairs With | Use Case | Status |
|----------|------------|----------|--------|
| `zenoh` | zenoh | Network compressed images | Implemented |
| `usb` | any | Local USB/V4L2 camera | Implemented |
| `none` | any | Disable camera | Implemented |

## Basic Usage

### Default (ROSBridge + Zenoh Camera)

```python
bot = WalkieRobot(ip="192.168.1.100")
# Equivalent to:
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="rosbridge",
    ros_port=9090,
    camera_protocol="zenoh",
)
```

### Zenoh

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="zenoh",
    ros_port=7447,
    camera_protocol="zenoh",
)
```

### Without Camera

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    camera_protocol="none",
)
```

### Auto-Detect

Tries zenoh first, falls back to rosbridge:

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="auto",
)
```

## Mixed Camera Transports

When different cameras use different protocols (e.g., head camera over Zenoh,
wrist camera via USB), use the `cameras` parameter:

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="zenoh",
    ros_port=7447,
    cameras={
        "head": {
            "protocol": "zenoh",
            "topic": "/zed/zed_node/rgb/color/rect/image/compressed",
        },
        "wrist": {
            "protocol": "usb",
            "device": "/dev/v4l/by-id/usb-Logitech_C920-video-index0",
        },
    },
)

# Access cameras by name
head_frame = bot.cameras.get_frame("head")
wrist_frame = bot.cameras.get_frame("wrist")
all_frames = bot.cameras.get_all_frames()  # {"head": ..., "wrist": ...}
```

When `cameras` is set, it overrides the `camera_protocol` parameter.
The single-camera accessor `bot.camera` returns frames from the `"head"` camera
(or the first camera if no `"head"` is defined).

See [Camera Setup](cameras.md) for more details.

## Port Configuration

| Protocol | Default Port | Parameter |
|----------|-------------|-----------|
| rosbridge | 9090 | `ros_port` |
| zenoh | 7447 | `ros_port` |

## Backward Compatibility

Legacy parameter names still work:

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    ws_port=9090,           # maps to ros_port
    enable_camera=False,    # maps to camera_protocol="none"
)
```
