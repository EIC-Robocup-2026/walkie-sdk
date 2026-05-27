# Walkie SDK

A pure Python SDK for controlling Walkie robots with **pluggable protocol support**. Choose between WebSocket (ROSBridge), Zenoh, or mix-and-match camera transports based on your needs.

**No ROS 2 installation required on the client machine** (when using rosbridge or zenoh protocols).

> **Full documentation:** [walkie-sdk docs](https://EIC-Robocup-2026.github.io/walkie-sdk/)
> (run `mkdocs serve` locally to preview)

## Architecture

```
 ┌─────────────────────────────────────────────────────────────────────────────┐
 │                              YOUR LAPTOP                                    │
 │  ┌───────────────────────────────────────────────────────────────────────┐  │
 │  │                          walkie_sdk                                   │  │
 │  │  ┌──────────┐ ┌──────────────┐ ┌───────────┐ ┌──────────┐ ┌────────┐  │  │
 │  │  │ bot.nav  │ │bot.status    │ │bot.camera │ │ bot.arm  │ │bot.viz │  │  │
 │  │  │ • go_to  │ │• get_position│ │• get_frame│ │• set_pos │ │• point │  │  │
 │  │  │ • cancel │ │• get_vel     │ │• named    │ │• gripper │ │• clear │  │  │
 │  │  └────┬─────┘ └────┬─────────┘ └────┬──────┘ └────┬─────┘ └───┬────┘  │  │
 │  │       └──────┬─────┴────────────────┴─────────────┴───────────┘       │  │
 │  │              │                                                        │  │
 │  │    ┌─────────▼───────────────────────────────────────────────┐        │  │
 │  │    │              TransportFactory                           │        │  │
 │  │    │  ┌───────────┐ ┌──────────┐ ┌──────────┐                │        │  │
 │  │    │  │rosbridge  │ │  zenoh   │ │   usb    │                │        │  │
 │  │    │  │(WebSocket)│ │  (DDS)   │ │(V4L2/cv2)│                │        │  │
 │  │    │  └────┬──────┘ └────┬─────┘ └────┬─────┘                │        │  │
 │  │    └───────┼─────────────┼────────────┼──────────────────────┘        │  │
 │  └────────────┼─────────────┼────────────┼───────────────────────────────┘  │
 └───────────────┼─────────────┼────────────┼──────────────────────────────────┘
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
pose = bot.status.get_position()
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

## Web Interface

Prefer clicking buttons over typing Python? Install the optional `web` extra and
run a browser dashboard that drives the SDK — connection, telemetry, navigation,
lift, arm/gripper, head tilt, joint states, and a live camera feed.

```bash
uv sync --extra web                 # or: uv pip install "walkie-sdk[web]"

walkie-web --port 8080              # then open http://localhost:8080
# auto-connect on startup:
walkie-web --ip 192.168.1.100
```

A FastAPI server holds a single `WalkieRobot` connection server-side and exposes
it over `/api/*` (Swagger docs at `/docs`); the browser is just a remote control.
See the [Web Interface guide](docs/guides/web-interface.md) for the full route
list and flags.

### Sharing it on your network (LAN / public IP)

By default `walkie-web` binds to `127.0.0.1`, so only the same machine can open
the dashboard. To let phones, tablets, or teammates on the same network reach
it, bind to **all interfaces** with `--host 0.0.0.0`:

```bash
walkie-web --host 0.0.0.0 --port 8080
```

On startup the server prints every LAN URL it can reach, e.g.:

```
Walkie web interface listening on port 8080:
  Local:   http://127.0.0.1:8080
  Network: http://10.0.0.201:8080
  Network: http://192.168.1.42:8080
  ↳ share the URL on the SAME network as the other machine (the API has no auth)
```

Open one of the `Network` URLs from any device on the same Wi-Fi / LAN.

If a remote device can't connect:

- **Firewall** — allow inbound TCP on the port. On Ubuntu: `sudo ufw allow 8080/tcp`.
- **Wi-Fi isolation** — some guest / hotel networks block peer-to-peer traffic;
  use a wired link or a phone hotspot instead.
- **Correct interface** — pick the URL that matches the network the *other*
  device is on (e.g. the Ethernet IP if the robot is on Ethernet but your laptop
  is on Wi-Fi, both URLs are printed).

### Exposing it over the public internet

There is **no built-in auth**, so do *not* port-forward `walkie-web` directly.
If you really need access from outside the LAN, tunnel it through something
that adds auth or short-lived URLs:

```bash
# Cloudflare quick tunnel — no account needed, ephemeral https URL
cloudflared tunnel --url http://localhost:8080

# ngrok — free tier prints a random *.ngrok.io URL
ngrok http 8080

# Plain SSH reverse tunnel from a server you already own
ssh -R 8080:localhost:8080 user@your-public-host
```

Keep the tunnel running only while you're using it, and treat the URL as a key
— anyone with it can drive the robot.

> **Heads up:** the API has no auth. Keep the default `--host 127.0.0.1` for
> local-only use, and only switch to `--host 0.0.0.0` when you intentionally
> want to share access with the rest of your LAN.

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
