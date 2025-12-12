# Walkie SDK

A pure Python SDK for controlling Walkie robots via WebSocket (ROSBridge) and WebRTC protocols. **No ROS 2 installation required on the client machine.**

## 🏗️ Architecture

Walkie SDK uses a **Hybrid Bridge Architecture** to separate "Mission Critical" robotics code from "Guest Science" AI logic:

```
┌────────────────────────────────────────────────────────────────────────┐
│                              YOUR LAPTOP                               │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                         walkie_sdk                               │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │  │
│  │  │   bot.nav   │  │ bot.status  │  │      bot.camera         │   │  │
│  │  │  • go_to()  │  │ • get_pose()│  │      • get_frame()      │   │  │
│  │  │  • cancel() │  │ • get_vel() │  │                         │   │  │
│  │  │  • stop()   │  │             │  │                         │   │  │
│  │  └──────┬──────┘  └──────┬──────┘  └────────────┬────────────┘   │  │
│  │         │                │                      │                │  │
│  │         └───────┬────────┘                      │                │  │
│  │                 │                               │                │  │
│  │    ┌────────────▼────────────┐     ┌────────────▼────────────┐   │  │
│  │    │     BridgeClient        │     │     WebRTCClient        │   │  │
│  │    │   (WebSocket/roslibpy)  │     │   (aiortc video)        │   │  │
│  │    └────────────┬────────────┘     └────────────┬────────────┘   │  │
│  └─────────────────┼───────────────────────────────┼────────────────┘  │
└────────────────────┼───────────────────────────────┼───────────────────┘
                     │ :9090                         │ :8554
                     │ WebSocket                     │ WebRTC
                     ▼                               ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                              THE ROBOT                                  │
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │                        ROS 2 Jazzy                                  ││
│  │  ┌────────────────┐  ┌────────────────┐  ┌────────────────────────┐ ││
│  │  │ rosbridge_srv  │  │     Nav2       │  │   webrtc_ros_server    │ ││
│  │  │    :9090       │  │ /navigate_pose │  │        :8554           │ ││
│  │  └────────────────┘  └────────────────┘  └────────────────────────┘ ││
│  └─────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
```

## 📦 Installation

### Using UV (recommended)

```bash
uv add walkie-sdk
```

### Using pip

```bash
pip install walkie-sdk
```

### From source

```bash
git clone https://github.com/walkie-team/walkie-sdk.git
cd walkie-sdk
uv sync
```

## 🚀 Quick Start

```python
from walkie_sdk import WalkieRobot
import cv2

# 1. Connect to Robot (auto-connects on init)
bot = WalkieRobot(ip="192.168.1.100")

# 2. Check Status
pose = bot.status.get_pose()
print(f"Robot at: x={pose['x']:.2f}, y={pose['y']:.2f}, heading={pose['heading']:.2f}")

velocity = bot.status.get_velocity()
print(f"Moving at: {velocity['linear']:.2f} m/s, rotating at {velocity['angular']:.2f} rad/s")

# 3. Get Camera Frame
frame = bot.camera.get_frame()
if frame is not None:
    cv2.imshow("Robot Camera", frame)
    cv2.waitKey(1)
    # ... Run your AI model on frame ...

# 4. Navigate to Target
print("Moving to target...")
result = bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
print(f"Navigation result: {result}")  # "SUCCEEDED" or "FAILED"

# 5. Emergency Stop (if needed)
bot.nav.stop()

# 6. Disconnect when done
bot.disconnect()
```

### Using Context Manager

```python
from walkie_sdk import WalkieRobot

with WalkieRobot(ip="192.168.1.100") as bot:
    bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
    # Auto-disconnects when exiting the block
```

## 📖 API Reference

### WalkieRobot

Main SDK class for controlling a Walkie robot.

```python
WalkieRobot(
    ip: str,                    # Robot IP address or hostname
    ws_port: int = 9090,        # ROSBridge WebSocket port
    webrtc_port: int = 8554,    # WebRTC signaling port
    timeout: float = 10.0,      # Connection timeout in seconds
    enable_camera: bool = True, # Enable WebRTC camera stream
    namespace: str = ""         # ROS namespace for topics/actions
)
```

**Properties:**
- `ip` → `str`: Robot IP address
- `is_connected` → `bool`: Connection status
- `namespace` → `str`: ROS namespace (can be changed at runtime)
- `nav` → `Navigation`: Navigation controller
- `status` → `Telemetry`: Telemetry provider
- `camera` → `Camera | None`: Camera interface

**Methods:**
- `disconnect()`: Disconnect from the robot

**Namespace Example:**
```python
# Without namespace (default): /odom, /cmd_vel, /navigate_to_pose
bot = WalkieRobot(ip="192.168.1.100")

# With namespace: /robot1/odom, /robot1/cmd_vel, /robot1/navigate_to_pose
bot = WalkieRobot(ip="192.168.1.100", namespace="robot1")

# Change namespace at runtime
bot.namespace = "robot2"
```

---

### bot.nav (Navigation)

Navigation controls for the robot.

#### `go_to(x, y, heading, blocking=True, timeout=None, feedback_callback=None)`

Navigate to a target pose.

| Parameter | Type | Description |
|-----------|------|-------------|
| `x` | `float` | Target X coordinate in meters (map frame) |
| `y` | `float` | Target Y coordinate in meters (map frame) |
| `heading` | `float` | Target heading in radians (0 = +X, π/2 = +Y) |
| `blocking` | `bool` | Wait for navigation to complete (default: True) |
| `timeout` | `float \| None` | Timeout in seconds (None = wait forever) |
| `feedback_callback` | `Callable` | Optional callback for progress updates |

**Returns:** `str` - Status: `"SUCCEEDED"`, `"FAILED"`, `"CANCELED"`, or `"IN_PROGRESS"`

```python
# Blocking call - waits until robot arrives
result = bot.nav.go_to(x=2.0, y=1.0, heading=0.0)

# Non-blocking call - returns immediately
bot.nav.go_to(x=2.0, y=1.0, heading=0.0, blocking=False)
# ... do other work ...
print(bot.nav.status)  # Check status later
```

#### `cancel()`

Cancel the current navigation goal.

**Returns:** `bool` - True if cancellation was sent successfully

```python
bot.nav.go_to(x=10.0, y=5.0, heading=0.0, blocking=False)
time.sleep(2)
bot.nav.cancel()  # Abort navigation
```

#### `stop()`

Emergency stop - immediately halt robot motion. Publishes zero velocity to `/cmd_vel`.

**Returns:** `bool` - True if stop command was sent successfully

```python
bot.nav.stop()  # STOP NOW!
```

#### Properties

- `status` → `str | None`: Current navigation status
- `is_navigating` → `bool`: True if navigation in progress

---

### bot.status (Telemetry)

Robot telemetry and status data.

#### `get_pose()`

Get the current robot pose.

**Returns:** `dict | None` - `{'x': float, 'y': float, 'heading': float}` or None

```python
pose = bot.status.get_pose()
if pose:
    print(f"Position: ({pose['x']:.2f}, {pose['y']:.2f})")
    print(f"Heading: {pose['heading']:.2f} rad")
```

#### `get_velocity()`

Get the current robot velocity.

**Returns:** `dict | None` - `{'linear': float, 'angular': float}` or None

```python
vel = bot.status.get_velocity()
if vel:
    print(f"Speed: {vel['linear']:.2f} m/s")
    print(f"Rotation: {vel['angular']:.2f} rad/s")
```

#### Properties

- `has_data` → `bool`: True if telemetry data is available

---

### bot.camera (Camera)

Camera/video stream interface.

#### `get_frame()`

Get the latest camera frame.

**Returns:** `numpy.ndarray | None` - BGR image (HxWx3, uint8) or None

```python
import cv2

frame = bot.camera.get_frame()
if frame is not None:
    # Frame is OpenCV-compatible BGR numpy array
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)
    
    # Run AI detection
    detections = your_model.detect(frame)
```

#### Properties

- `is_streaming` → `bool`: True if camera stream is active
- `frame_shape` → `tuple | None`: Frame dimensions (height, width, channels)

---

## 🖥️ Server Requirements

The robot must run ROS 2 with the following components:

1. **ROSBridge Server** (WebSocket at port 9090)
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **Nav2 Navigation Stack** (for navigation)
   ```bash
   ros2 launch nav2_bringup navigation_launch.py
   ```

3. **WebRTC ROS Server** (for camera, port 8554)
   ```bash
   ros2 launch webrtc_ros webrtc_server.launch.py
   ```

Or use a combined launch file:
```bash
ros2 launch walkie_bringup robot_server.launch.py
```

## 🔧 Configuration

### Connection Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ws_port` | 9090 | ROSBridge WebSocket port |
| `webrtc_port` | 8554 | WebRTC signaling port |
| `timeout` | 10.0 | Connection timeout (seconds) |

### ROS Topics Used

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Navigation action |

## 🐛 Troubleshooting

### Connection Errors

```
ConnectionError: Connection timeout after 10.0s. Is ROSBridge running at 192.168.1.100:9090?
```

**Solutions:**
1. Verify robot IP is correct and reachable: `ping 192.168.1.100`
2. Check ROSBridge is running: `ros2 node list | grep rosbridge`
3. Check firewall allows port 9090

### No Camera Frames

```
⚠ WebRTC connection failed: ...
  Camera will not be available.
```

**Solutions:**
1. Verify WebRTC server is running on robot
2. Check port 8554 is accessible
3. Disable camera if not needed: `WalkieRobot(ip="...", enable_camera=False)`

### No Odometry Data

```python
bot.status.get_pose()  # Returns None
```

**Solutions:**
1. Wait for first odometry message (may take 100ms)
2. Verify robot odometry is publishing: `ros2 topic echo /odom`

## 📄 License

MIT License - see [LICENSE](LICENSE) for details.

## 🤝 Contributing

Contributions welcome! Please read our contributing guidelines before submitting PRs.
