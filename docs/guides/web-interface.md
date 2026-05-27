# Web Interface

Driving the robot from a Python REPL or scripts gets tedious. The optional
**web interface** wraps the SDK in a small FastAPI server and serves a browser
dashboard so you can connect, drive navigation, move the lift, control the
arm/gripper, tilt the head, inspect joint states, and watch the camera — all
from a web page.

```
┌──────────┐   HTTP / MJPEG   ┌─────────────────────┐   rosbridge/zenoh   ┌────────┐
│ Browser  │ ───────────────► │  walkie-web server  │ ──────────────────► │ Robot  │
│dashboard │ ◄─────────────── │ (one WalkieRobot)   │ ◄────────────────── │        │
└──────────┘                  └─────────────────────┘                     └────────┘
```

The browser never talks to the robot directly. The server holds a **single**
`WalkieRobot` connection and exposes it over `/api/*` routes.

## Install

The web layer is an optional extra so the base SDK stays dependency-light:

```bash
uv sync --extra web        # from source
# or
uv pip install "walkie-sdk[web]"
```

This pulls in `fastapi` and `uvicorn`.

## Run

```bash
walkie-web --port 8080
# or
python -m walkie_sdk.web --port 8080
```

Then open <http://localhost:8080> and enter the robot's IP in the **Connection**
card. To auto-connect on startup instead:

```bash
walkie-web --ip 192.168.1.100 --camera-protocol zenoh
```

| Flag | Default | Description |
|------|---------|-------------|
| `--host` | `127.0.0.1` | Bind address (use `0.0.0.0` to expose on the LAN) |
| `--port` | `8080` | Bind port |
| `--reload` | off | uvicorn auto-reload (dev) |
| `--ip` | — | Robot IP to auto-connect on startup |
| `--ros-protocol` | `rosbridge` | `rosbridge` / `zenoh` / `auto` |
| `--ros-port` | `9090` | rosbridge port |
| `--camera-protocol` | `zenoh` | `zenoh` / `usb` / `none` |
| `--camera-port` | `7447` | camera transport port |
| `--namespace` | `""` | ROS namespace |

## Sharing on your network (LAN / public IP)

By default the server only listens on `127.0.0.1`. To let phones, tablets, or
teammates on the same network use the dashboard, bind to every interface:

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

If a remote device can't connect:

- **Firewall** — open the port. On Ubuntu: `sudo ufw allow 8080/tcp`.
- **Wi-Fi isolation** — many guest/hotel networks block peer-to-peer traffic;
  use a wired link or a phone hotspot.
- **Wrong interface** — if your laptop is on Wi-Fi *and* Ethernet, pick the
  URL on the same network as the device trying to connect.

### Exposing it over the public internet

There is **no built-in auth**. Do *not* port-forward `walkie-web` directly.
Tunnel it through something that adds auth or short-lived URLs instead:

```bash
# Cloudflare quick tunnel — no account needed, ephemeral https URL
cloudflared tunnel --url http://localhost:8080

# ngrok — free tier prints a random *.ngrok.io URL
ngrok http 8080

# Plain SSH reverse tunnel from a server you already own
ssh -R 8080:localhost:8080 user@your-public-host
```

Keep the tunnel running only while you're using it, and treat the URL as a key —
anyone with it can drive the robot.

!!! warning "Exposure"
    `--host 0.0.0.0` makes the panel reachable by anyone on the network and the
    API has no authentication. Only do this on a trusted lab network, or keep
    the default `127.0.0.1` and use an SSH tunnel.

## API

Every route returns JSON `{"ok": true, ...}` on success. Calling a robot action
while disconnected returns **409** `{"ok": false, "error": "not_connected"}`.

### Connection & status

| Method | Path | Maps to |
|--------|------|---------|
| `POST` | `/api/connect` | `WalkieRobot(...)` |
| `POST` | `/api/disconnect` | `bot.disconnect()` |
| `GET`  | `/api/status` | telemetry + lift + nav + head + joints snapshot |
| `POST` | `/api/namespace` | set `bot.namespace` |

### Navigation

| Method | Path | Maps to |
|--------|------|---------|
| `POST` | `/api/nav/goto` | `bot.nav.go_to(x, y, heading)` |
| `POST` | `/api/nav/cancel` | `bot.nav.cancel()` |
| `POST` | `/api/nav/stop` | `bot.nav.stop()` |

### Lift

| Method | Path | Maps to |
|--------|------|---------|
| `POST` | `/api/lift/set` | `bot.lift.set(pos, ...)` |
| `GET`  | `/api/lift` | `bot.lift.get()` |

### Arm & gripper

| Method | Path | Maps to |
|--------|------|---------|
| `POST` | `/api/arm/pose` | `bot.arm.go_to_pose(...)` (Euler RPY) |
| `POST` | `/api/arm/pose_quat` | `bot.arm.go_to_pose_quat(...)` |
| `POST` | `/api/arm/pose_relative` | `bot.arm.go_to_pose_relative(...)` |
| `POST` | `/api/arm/home` | `bot.arm.go_to_home(group_name)` |
| `POST` | `/api/arm/gripper` | `bot.arm.control_gripper(...)` |
| `POST` | `/api/arm/joints` | `bot.arm.set_joint_position(group_name, joint_positions, mode, duration)` |
| `GET`  | `/api/arm/states` | `bot.arm.get_joint_states()` |
| `POST` | `/api/arm/ee_pose` | `bot.arm.get_ee_pose(group_name, frame_id)` |

`group_name` accepts `left_arm`, `right_arm`, `left_arm_lift`,
`right_arm_lift`, `both_arms`, `both_arms_lift`. `mode` on `/api/arm/joints` is
`"commander"` (MoveIt, collision-checked) or `"jtc"` (direct JointTrajectory
publish, high-rate, no collision check).

#### Gripper position

`/api/arm/gripper` accepts a normalized position by default:

```json
{ "group_name": "left_gripper", "position": 0.7, "norm": true }
```

- `norm: true` (default) — `position` is in `[0, 1]`. `0` is fully closed,
  `1` is fully open (0.04 m). The server scales it to meters before calling
  `control_gripper`.
- `norm: false` — `position` is raw meters in `[0, 0.04]`.

The response echoes the meters value actually sent:

```json
{ "ok": true, "result": "SUCCEEDED", "position_m": 0.028 }
```

### Head tilt

| Method | Path | Maps to |
|--------|------|---------|
| `POST` | `/api/head/tilt` | `bot.head.tilt(angle_rad)` |
| `GET`  | `/api/head` | `bot.head.get_angle()` |

`angle_rad` must be within ±π/4 (±45°). Positive values tilt the camera
downward. Out-of-range values return **400** `{"ok": false, "error": "..."}`.

### Joint state hub

The robot publishes a single `joint_states` topic that `JointStateHub`
subscribes to once and shares with all modules.

| Method | Path | Maps to |
|--------|------|---------|
| `GET`  | `/api/joints` | `bot.joints.get_all()` |
| `GET`  | `/api/joints/{name}` | position + velocity + effort for one joint |

```bash
curl http://localhost:8080/api/joints/head_servo_joint
# → {"ok": true, "position": 0.12, "velocity": 0.0, "effort": 0.0}
```

### Camera

| Method | Path | Maps to |
|--------|------|---------|
| `GET`  | `/api/camera/snapshot?camera=head` | single JPEG frame |
| `GET`  | `/api/camera/stream?camera=head` | MJPEG stream |

Interactive API docs are auto-generated at `/docs` (Swagger UI).

## Status snapshot

`GET /api/status` returns a best-effort snapshot that the dashboard polls once
a second. When disconnected it's just `{"connected": false}`. When connected:

```json
{
  "connected": true,
  "ip": "10.0.0.201",
  "namespace": "",
  "ros_protocol": "rosbridge",
  "camera_protocol": "zenoh",
  "position": {"x": 0.0, "y": 0.0, "heading": 0.0},
  "velocity": {"linear": 0.0, "angular": 0.0},
  "nav_status": null,
  "lift": 0.5,
  "lift_cm": 37.2,
  "lift_status": "SUCCEEDED",
  "head_angle": 0.12,
  "arm_states": { "left_arm": {...}, "right_arm": {...}, "left_gripper": 0.0 },
  "joints_count": 17,
  "camera_available": true,
  "cameras": ["head", "left_wrist", "right_wrist"]
}
```

Any sub-field can come back `null` if the underlying module hasn't received
data yet — the snapshot never throws.

Navigation, lift, arm, and head commands are sent **non-blocking** from the
dashboard by default; status polling shows progress (`IN_PROGRESS` →
`SUCCEEDED` / `FAILED`).
