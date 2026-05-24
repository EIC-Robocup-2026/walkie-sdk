# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Install dependencies
uv sync

# Run unit tests (tests/unit/ only — no robot required)
uv run pytest

# Run a single test file
uv run pytest tests/unit/test_lift.py

# Run a single test
uv run pytest tests/unit/test_lift.py::TestLiftBlocking::test_blocking_returns_succeeded_when_position_reached

# Build and preview documentation locally
uv pip install -e ".[docs]"
mkdocs serve
```

The `tests/` root contains integration/hardware tests (`test_call_action.py`, `test_lift.py`, etc.) that require a real robot; only `tests/unit/` runs without hardware. `pyproject.toml` points pytest's `testpaths` to `tests/unit`.

## Architecture

`WalkieRobot` (`walkie_sdk/robot.py`) is the single entry point. On construction it:
1. Creates a **ROS transport** via `TransportFactory` (rosbridge WebSocket or Zenoh DDS)
2. Creates a **camera transport** via `TransportFactory` (Zenoh stream, USB OpenCV, or None)
3. Instantiates all **modules** injected with the transport interface
4. Auto-connects (`_connect()`)

### Transport layer (`walkie_sdk/core/`)

All modules depend only on `ROSTransportInterface` / `CameraTransportInterface` (ABCs in `core/interfaces/`), never on concrete transports. This is the key seam for unit testing — modules are tested by injecting `MagicMock` transports.

- `TransportFactory` lazily imports only the requested transport to avoid pulling unused deps.
- Protocol selection: `ROSProtocol` enum (`rosbridge`, `zenoh`, `auto`) and `CameraProtocol` enum (`zenoh`, `usb`, `none`).
- Default ports: rosbridge → 9090, zenoh → 7447.

### Modules (`walkie_sdk/modules/`)

Each module receives a transport and a `namespace` string:

| Module | `bot.*` | Purpose |
|---|---|---|
| `Navigation` | `bot.nav` | Nav2 action (`go_to`, `cancel`, `stop`) |
| `Telemetry` | `bot.status` | Odometry subscription (`get_position`, `get_velocity`) |
| `Arm` | `bot.arm` | Joint commands, MoveIt actions, custom IK publishing |
| `Lift` | `bot.lift` | Linear actuator position (normalized 0–1 or cm 0–74.35) |
| `Camera` | `bot.camera` | Single-camera frame access |
| `MultiCamera` | `bot.cameras` | Named multi-camera frame access |
| `Visualization` | `bot.viz` | RViz2 marker/pose publishing |
| `Tools` | `bot.tools` | Utility helpers (e.g., 3D pose service) |

### Topic/action configuration (`walkie_sdk/config/ros_topics.py`)

All ROS topic and action names are centralized here as module-level dicts (`CAMERA_TOPICS`, `ARM_TOPICS`, `NAV_TOPICS`, etc.). They read from env vars (e.g., `WALKIE_NAV_CMD_VEL`) with hardcoded defaults as fallback.

At runtime, `load_config(yaml_path)` updates these dicts **in-place** from a YAML file — this propagates to all already-imported modules because they reference the same dict objects. The root `ros_topics.yaml` is the default config file.

### Namespace handling (`walkie_sdk/utils/namespace.py`)

`apply_namespace(name, namespace)` prepends a namespace prefix: `"odom"` + `"robot1"` → `"robot1/odom"`. Each module stores `_namespace` and exposes a `namespace` setter; setting `bot.namespace = "x"` propagates to all modules. Arm and Lift re-subscribe immediately on namespace change; Telemetry keeps the previous subscription until restart.
