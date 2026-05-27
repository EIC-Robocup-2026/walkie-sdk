# Web Interface

Driving the robot from a Python REPL or scripts gets tedious. The optional
**web interface** wraps the SDK in a small FastAPI server and serves a browser
dashboard so you can connect, drive navigation, move the lift, control the
arm/gripper, and watch the camera — all from a web page.

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
walkie-web --host 0.0.0.0 --port 8080
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
| `--ip` | — | Robot IP to auto-connect on startup |
| `--ros-protocol` | `rosbridge` | `rosbridge` / `zenoh` / `auto` |
| `--ros-port` | `9090` | rosbridge port |
| `--camera-protocol` | `zenoh` | `zenoh` / `usb` / `none` |
| `--namespace` | `""` | ROS namespace |

!!! warning "Exposure"
    `--host 0.0.0.0` makes the panel reachable by anyone on the network and the
    API has no authentication. Only do this on a trusted lab network, or keep
    the default `127.0.0.1` and use an SSH tunnel.

## API

Every route returns JSON `{"ok": true, ...}` on success. Calling a robot action
while disconnected returns **409** `{"ok": false, "error": "not_connected"}`.

| Method | Path | Maps to |
|--------|------|---------|
| `POST` | `/api/connect` | `WalkieRobot(...)` |
| `POST` | `/api/disconnect` | `bot.disconnect()` |
| `GET`  | `/api/status` | telemetry + lift + nav snapshot |
| `POST` | `/api/namespace` | set `bot.namespace` |
| `POST` | `/api/nav/goto` | `bot.nav.go_to(x, y, heading)` |
| `POST` | `/api/nav/cancel` | `bot.nav.cancel()` |
| `POST` | `/api/nav/stop` | `bot.nav.stop()` |
| `POST` | `/api/lift/set` | `bot.lift.set(pos, ...)` |
| `GET`  | `/api/lift` | `bot.lift.get()` |
| `POST` | `/api/arm/pose` | `bot.arm.go_to_pose(...)` |
| `POST` | `/api/arm/home` | `bot.arm.go_to_home(group)` |
| `POST` | `/api/arm/gripper` | `bot.arm.control_gripper(...)` |
| `POST` | `/api/arm/joints` | `bot.arm.set_joint_positions(...)` |
| `GET`  | `/api/arm/states` | `bot.arm.get_joint_states()` |
| `GET`  | `/api/camera/snapshot?camera=head` | single JPEG frame |
| `GET`  | `/api/camera/stream?camera=head` | MJPEG stream |

Interactive API docs are auto-generated at `/docs` (Swagger UI).

Navigation and lift commands are sent **non-blocking** from the dashboard; the
status card polls `/api/status` once a second to show progress (`IN_PROGRESS` →
`SUCCEEDED`/`FAILED`).
