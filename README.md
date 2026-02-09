# Walkie SDK

A pure Python SDK for controlling Walkie robots with **pluggable protocol support**. Choose between WebSocket (ROSBridge) or Zenoh based on your needs.

**No ROS 2 installation required on the client machine** (when using rosbridge or zenoh protocols).

## 🏗️ Architecture

Walkie SDK uses a **Protocol-Agnostic Architecture** that separates high-level robot control from the underlying communication protocol:

```
 ┌─────────────────────────────────────────────────────────────────────────┐
 │                              YOUR LAPTOP                                │
 │  ┌───────────────────────────────────────────────────────────────────┐  │
 │  │                          walkie_sdk                               │  │
 │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐    │  │
 │  │  │   bot.nav   │  │ bot.status  │  │      bot.camera         │    │  │
 │  │  │  • go_to()  │  │ • get_pose()│  │      • get_frame()      │    │  │
 │  │  │  • cancel() │  │ • get_vel() │  │                         │    │  │
 │  │  │  • stop()   │  │             │  │                         │    │  │
 │  │  └──────┬──────┘  └──────┬──────┘  └────────────┬────────────┘    │  │
 │  │         │                │                      │                 │  │
 │  │         └───────┬────────┴──────────────────────┘                 │  │
 │  │                 │                                                 │  │
 │  │    ┌────────────▼─────────────────────────────────────────────┐   │  │
 │  │    │              TransportFactory                            │   │  │
 │  │    │  ┌─────────────┐ ┌─────────────┐                       │   │  │
 │  │    │  │  rosbridge  │ │   zenoh     │                       │   │  │
 │  │    │  │ (WebSocket) │ │  (DDS)      │                       │   │  │
 │  │    │  └──────┬──────┘ └──────┬──────┘                       │   │  │
 │  │    └─────────┼───────────────┼───────────────────────────────┘   │  │
 │  └──────────────┼───────────────┼───────────────────────────────────┘  │
 └─────────────────┼───────────────┼───────────────────────────────────────┘
                   │               │
                   ▼               ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                              THE ROBOT                                  │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                        ROS 2 Jazzy                                │  │
│  │  ┌────────────────┐  ┌────────────────┐  ┌────────────────────┐   │  │
│  │  │ rosbridge_srv  │  │     Nav2       │  │   Camera Server    │   │  │
│  │  │    :9090       │  │ /navigate_pose │  │   (WebRTC/Image)   │   │  │
│  │  └────────────────┘  └────────────────┘  └────────────────────┘   │  │
│  └───────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

## 🔌 Protocol Support

| Protocol | ROS2 Required | Performance | Status |
|----------|---------------|-------------|--------|
| `rosbridge` | ❌ No | Medium | ✅ Implemented |
| `zenoh` | ❌ No | Good | 🚧 Planned |

| Camera Protocol | Pairs With | Status |
|-----------------|------------|--------|
| `webrtc` | rosbridge | ✅ Implemented |
| `zenoh` | zenoh | 🚧 Planned |
| `shm` | same-host | ✅ Implemented |
| `none` | any | ✅ Implemented |

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

## 🔧 Protocol Selection

### Default: ROSBridge (WebSocket)

```python
# Default - uses rosbridge + webrtc
bot = WalkieRobot(ip="192.168.1.100")
```

### Explicit Protocol Selection

```python
from walkie_sdk import WalkieRobot

# ROSBridge with WebRTC camera (explicit)
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="rosbridge",    # WebSocket via roslibpy
    ros_port=9090,
    camera_protocol="webrtc",    # WebRTC video stream
    camera_port=8554,
)

# Without camera (telemetry/navigation only)
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="rosbridge",
    camera_protocol="none",      # Disable camera
)

# Zenoh DDS Bridge - Coming Soon
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="zenoh",        # Zenoh DDS bridge
    ros_port=7447,
    camera_protocol="zenoh",
)

# Auto-detect best available protocol
bot = WalkieRobot(
    ip="192.168.1.100",
    camera_protocol="none",      # Disable camera
)

# Zenoh DDS Bridge - Coming Soon
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="zenoh",        # Zenoh DDS bridge
    ros_port=7447,
    camera_protocol="zenoh",
)

# Auto-detect best available protocol
bot = WalkieRobot(
    ip="192.168.1.100",
    ros_protocol="auto",         # Tries: zenoh → rosbridge
)
```

### Backward Compatibility

The legacy API parameters are still supported:

```python
# Old API (still works)
bot = WalkieRobot(
    ip="192.168.1.100",
    ws_port=9090,           # Maps to ros_port
    enable_camera=False,    # Maps to camera_protocol="none"
)
```

## 📖 API Reference

### WalkieRobot

Main SDK class for controlling a Walkie robot.

```python
WalkieRobot(
    ip: str,                         # Robot IP address or hostname
    ros_protocol: str = "rosbridge", # "rosbridge", "zenoh", or "auto"
    ros_port: int = 9090,            # Port for ROS transport
    camera_protocol: str = "webrtc", # "webrtc", "zenoh", "shm", or "none"
    camera_port: int = 8554,         # Port for camera stream
    timeout: float = 10.0,           # Connection timeout in seconds
    namespace: str = ""              # ROS namespace for topics/actions
)
```

**Properties:**
- `ip` → `str`: Robot IP address
- `is_connected` → `bool`: Connection status
- `ros_protocol` → `str`: Active ROS protocol
- `camera_protocol` → `str`: Active camera protocol
- `namespace` → `str`: ROS namespace (can be changed at runtime)
- `nav` → `Navigation`: Navigation controller
- `status` → `Telemetry`: Telemetry provider
- `camera` → `Camera | None`: Camera interface (None if disabled)

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

Camera/video stream interface. Returns `None` if camera is disabled.

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

1. **ROSBridge Server** (WebSocket at port 9090) - for `rosbridge` protocol
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **Nav2 Navigation Stack** (for navigation)
   ```bash
   ros2 launch nav2_bringup navigation_launch.py
   ```

3. **WebRTC ROS Server** (for camera, port 8554) - for `webrtc` camera protocol
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
| `ros_port` | 9090 | ROSBridge WebSocket port |
| `camera_port` | 8554 | WebRTC signaling port |
| `timeout` | 10.0 | Connection timeout (seconds) |

### ROS Topics Used

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Navigation action |

## 🏛️ SDK Architecture (For Contributors)

The SDK uses a clean separation between high-level modules and transport implementations:

```
walkie_sdk/
├── robot.py                    # WalkieRobot main class
├── core/
│   ├── interfaces/             # Abstract base classes
│   │   ├── ros_transport.py    # ROSTransportInterface (ABC)
│   │   └── camera_transport.py # CameraTransportInterface (ABC)
│   ├── factory.py              # TransportFactory
│   └── transports/             # Protocol implementations
│       ├── rosbridge/          # WebSocket (roslibpy)
│       │   ├── transport.py    # ROSBridgeTransport
│       │   └── camera.py       # WebRTCCamera
│       └── zenoh/              # Zenoh DDS (stub)
└── modules/
    ├── navigation.py           # Navigation (uses interface)
    ├── telemetry.py            # Telemetry (uses interface)
    └── camera.py               # Camera (uses interface)
```

### Adding a New Protocol

1. Create a new transport in `core/transports/your_protocol/`
2. Implement `ROSTransportInterface` and optionally `CameraTransportInterface`
3. Register it in `core/factory.py`

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
3. Disable camera if not needed: `WalkieRobot(ip="...", camera_protocol="none")`

### No Odometry Data

```python
bot.status.get_pose()  # Returns None
```

**Solutions:**
1. Wait for first odometry message (may take 100ms)
2. Verify robot odometry is publishing: `ros2 topic echo /odom`

### Protocol Not Implemented

```
ValueError: Unknown ROS protocol: ...
```

**Solutions:**
1. Use `ros_protocol="rosbridge"` (default) or `ros_protocol="zenoh"`
2. Check available protocols: "rosbridge", "zenoh", "auto"

## 📄 License

MIT License - see [LICENSE](LICENSE) for details.

## 🤝 Contributing

Contributions welcome! Especially for:
- `zenoh` transport implementation
- Additional camera protocols

Please read our contributing guidelines before submitting PRs.
