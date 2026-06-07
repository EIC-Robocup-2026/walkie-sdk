# Examples

Example scripts are in the [`examples/`](https://github.com/EIC-Robocup-2026/walkie-sdk/tree/main/examples) directory.

## Running Examples

```bash
uv run python examples/<script_name>.py
```

## Available Examples

| Script | Description |
|--------|-------------|
| `example_camera.py` | Camera feed viewer with FPS overlay and snapshot saving |
| `example_depth.py` | Depth stream (get_depth) with distance readout and colourised view |
| `example_no_camera.py` | Navigation and telemetry without camera |
| `example_protocols.py` | Protocol selection examples (rosbridge, zenoh, auto, mixed cameras) |
| `example_visualization.py` | RViz2 marker, pose, and axis publishing |
| `example_arm_publisher.py` | Arm joint position publishing |
| `example_usb_camera.py` | USB camera discovery, connection, and preview |
| `arm_zenoh_teleop.py` | Arm teleoperation over Zenoh (Euler angles) |
| `arm_zenoh_teleop_quaternion.py` | Arm teleoperation over Zenoh (quaternion) |

## USB Camera Test

The USB camera example also serves as a diagnostic tool:

```bash
# Auto-detect first available USB camera
uv run python examples/example_usb_camera.py

# Specify a device by stable path
uv run python examples/example_usb_camera.py /dev/v4l/by-id/usb-Logitech_C920-video-index0

# Specify by device index
uv run python examples/example_usb_camera.py 0
```

Controls: `q` to quit, `s` to save a snapshot.
