# Camera Setup

The SDK provides flexible camera access through multiple protocols and a
unified API. Cameras can come from network streams (Zenoh) or local USB
devices -- and you can mix them.

## Single Camera

The simplest setup uses one camera protocol for all cameras:

=== "Zenoh"

    ```python
    bot = WalkieRobot(
        ip="192.168.1.100",
        ros_protocol="zenoh",
        camera_protocol="zenoh",
    )
    frame = bot.camera.get_frame()
    ```

=== "USB"

    ```python
    bot = WalkieRobot(
        ip="192.168.1.100",
        camera_protocol="usb",
    )
    frame = bot.camera.get_frame()
    ```

## Multi-Camera (Same Protocol)

When all cameras use the same protocol, use `multi_camera=True` (Zenoh):

```python
# Zenoh multi-camera
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="zenoh",
    camera_protocol="zenoh",
    multi_camera=True,
)

# Access individual cameras
head = bot.cameras.get_frame("head")
left = bot.cameras.get_frame("left")
right = bot.cameras.get_frame("right")

# Get all at once
frames = bot.cameras.get_all_frames()
# {"head": np.ndarray, "left": np.ndarray, ...}
```

## Mixed Camera Transports

When different cameras use different protocols, pass a `cameras` dict to
`WalkieRobot`. Each key is a camera name, and each value specifies its
protocol and configuration:

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
            "width": 640,
            "height": 480,
        },
    },
)
```

The `cameras` parameter overrides `camera_protocol`. Each camera connects
independently -- if one fails, the others still work.

### Accessing Mixed Cameras

```python
# By name
head_frame = bot.cameras.get_frame("head")
wrist_frame = bot.cameras.get_frame("wrist")

# All at once
frames = bot.cameras.get_all_frames()

# Single-camera accessor defaults to "head" (or first camera)
frame = bot.camera.get_frame()

# List available cameras
print(bot.cameras.camera_names)  # ["head", "wrist"]
```

## USB Camera Setup

### Finding Your Camera

USB cameras are identified by device path. Use stable paths to avoid issues
when devices are plugged/unplugged or after reboot:

```bash
# List available USB cameras with stable paths
ls /dev/v4l/by-id/
# Example output:
# usb-Logitech_HD_Pro_Webcam_C920_12345678-video-index0
# usb-046d_C922_Pro_Stream_Webcam-video-index0
```

### Device Identification Methods

| Method | Example | Stability |
|--------|---------|-----------|
| Device index | `0` | Fragile -- changes on reboot/replug |
| Device node | `/dev/video0` | Fragile |
| By-id symlink | `/dev/v4l/by-id/usb-Logitech_C920-video-index0` | Stable per device |
| By-path symlink | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0` | Stable per USB port |

!!! tip "Recommendation"
    Always use `/dev/v4l/by-id/` paths in production. They are stable across
    reboots and don't change when other USB devices are plugged in.

### USB Camera Configuration

```python
cameras={
    "wrist": {
        "protocol": "usb",
        "device": "/dev/v4l/by-id/usb-Logitech_C920-video-index0",
        "width": 640,       # optional, camera default if omitted
        "height": 480,      # optional
        "fps": 30,          # optional
    },
}
```

### Testing USB Cameras

Use the included test script to verify your USB camera:

```bash
# Auto-detect first available camera
uv run python examples/example_usb_camera.py

# Specify a device
uv run python examples/example_usb_camera.py /dev/v4l/by-id/usb-Logitech_C920-video-index0
```

### Auto-Retry

The USB camera transport automatically retries if the device disconnects.
It uses exponential backoff (1s, 2s, 4s, ...) up to 5 attempts by default.
Configure via `max_retries` and `retry_base_delay` in the camera config.

## Frame Format

All camera transports return frames in the same format:

- **Type:** `numpy.ndarray`
- **Color space:** BGR (OpenCV convention)
- **Shape:** `(height, width, 3)`
- **Dtype:** `uint8`
- **Returns `None`** when no frame is available yet

Frames are always copies -- safe to modify without affecting the internal buffer.
