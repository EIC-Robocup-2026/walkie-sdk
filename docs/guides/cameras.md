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

## Depth

The ZED head camera also publishes a depth stream. It is streamed automatically
by the **zenoh** camera transport, alongside the color frames -- just read it
with `get_depth()`:

```python
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="zenoh",
    camera_protocol="zenoh",
)

depth = bot.camera.get_depth()  # or bot.cameras.get_depth("head")
if depth is not None:
    h, w = depth.shape
    print(depth[h // 2, w // 2], "metres to the centre pixel")
```

### Depth Frame Format

Depth is returned in the topic's **native units** (no conversion):

| Source encoding | Dtype | Units | Invalid pixels |
|-----------------|---------|-------------|----------------|
| `32FC1` (ZED `depth_registered`, default) | `float32` | **metres** | `NaN` |
| `16UC1` / `mono16` | `uint16` | **millimetres** | `0` |

- **Shape:** `(height, width)` -- single channel, no color dimension.
- **Returns `None`** when depth is disabled, unsupported (e.g. USB cameras),
  or no frame has arrived yet.
- Frames are copies -- safe to modify.

!!! warning "Use NaN-aware math"
    For the default `32FC1` stream, invalid/unmeasured pixels are `NaN`. Use
    `np.isfinite(depth)` to mask them before reductions, e.g.
    `depth[np.isfinite(depth)].min()`.

The depth topic defaults to `/zed_head/zed_node/depth/depth_registered` and can
be overridden via the `WALKIE_DEPTH_HEAD` env var, or the `DEPTH_TOPICS` block
in a `ros_topics.yaml` config file.

See [`examples/example_depth.py`](https://github.com/EIC-Robocup-2026/walkie-sdk/blob/main/examples/example_depth.py)
for a runnable demo with a colourised live view.

### Intrinsics (projecting depth to 3D)

To back-project depth pixels into 3D you need the camera intrinsics. They are
fetched once from the camera's `camera_info` topic and cached:

```python
intr = bot.camera.get_intrinsics()
# {'fx': ..., 'fy': ..., 'cx': ..., 'cy': ..., 'width': ..., 'height': ...}

depth = bot.camera.get_depth()
z = depth[v, u]
x = (u - intr["cx"]) * z / intr["fx"]
y = (v - intr["cy"]) * z / intr["fy"]
```

`get_camera_info()` returns the full `sensor_msgs/msg/CameraInfo` dict instead
(`k`, `p`, `d`, `width`, `height`, ...) if you need more than the pinhole
parameters. The ZED's `depth_registered` stream is aligned to the left
rectified image, so the same intrinsics apply to colour and depth, and the
distortion coefficients are zero.

!!! warning "Use the optical frame"
    The back-projected `(x, y, z)` point is in the camera's **optical** frame
    (Z forward, X right, Y down) — e.g. `zed_head_left_camera_optical_frame`,
    not `zed_head_left_camera_frame` (ROS body convention, X forward).

The camera_info topic defaults to `/zed_head/zed_node/rgb/color/rect/camera_info`
and can be overridden via the `WALKIE_CAMERA_INFO_HEAD` env var, or the
`CAMERA_INFO_TOPICS` block in a `ros_topics.yaml` config file.

### Projecting depth into the map frame

Combining the intrinsics with the camera pose from `bot.transform.lookup()`
turns a depth image into a point cloud in the world frame. The math is two
steps: back-project each pixel into the camera **optical** frame, then apply
the camera pose as a rigid transform.

```python
import numpy as np
from walkie_sdk.utils.converters import quaternion_to_matrix

depth = bot.camera.get_depth()        # HxW float32, metres (NaN = invalid)
intr = bot.camera.get_intrinsics()    # fx, fy, cx, cy (cached)

# Camera pose in the map frame. Use the *optical* frame -- it matches the
# axes the pinhole math produces (Z forward, X right, Y down).
pose = bot.transform.lookup("map", "zed_head_left_camera_optical_frame")
q, p = pose["quaternion"], pose["position"]
R = quaternion_to_matrix(q["x"], q["y"], q["z"], q["w"])
t = np.array([p["x"], p["y"], p["z"]])

# Back-project every valid pixel at once (vectorised)
h, w = depth.shape
us, vs = np.meshgrid(np.arange(w), np.arange(h))
valid = np.isfinite(depth) & (depth > 0)

z = depth[valid]
x = (us[valid] - intr["cx"]) * z / intr["fx"]
y = (vs[valid] - intr["cy"]) * z / intr["fy"]
points_optical = np.column_stack([x, y, z])   # Nx3, optical frame

points_map = points_optical @ R.T + t         # Nx3, map frame
```

For a single pixel `(u, v)` the same transform is
`R @ [x, y, z] + t`.

!!! tip "Sanity checks"
    - The depth image and `CameraInfo` must be the same resolution. If the
      depth stream is scaled, scale `fx`/`cx` by `w_depth / intr["width"]`
      and `fy`/`cy` by `h_depth / intr["height"]` first.
    - The lookup must return the **camera pose in the map frame** (the
      transform that maps camera-frame points *into* map). If your cloud
      comes out behind/inside the robot, the transform is inverted — swap
      the lookup direction or invert it (`R.T`, `-R.T @ t`).
    - Depth and pose are sampled at different instants; if the robot or head
      is moving, grab the pose as close to the frame as possible.

See [`examples/example_depth_projection.py`](https://github.com/EIC-Robocup-2026/walkie-sdk/blob/main/examples/example_depth_projection.py)
for a runnable demo that projects the full cloud and drops an RViz2 marker at
the projected centre pixel for verification.
