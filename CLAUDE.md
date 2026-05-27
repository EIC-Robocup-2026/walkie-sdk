# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`walkie_sdk` is a **pure-Python client SDK** for controlling Walkie robots (EIC Robocup 2026). It wraps ROS 2 communication into plain function calls so that **no ROS 2 installation is required on the client machine** — communication goes over rosbridge (WebSocket) or Zenoh. The robot side runs ROS 2 (Nav2, MoveIt, rosbridge_server, perception services).

## Commands

This project uses **uv**. The first `uv run`/`uv sync` will create `.venv` and install dependencies from `uv.lock`.

```bash
uv sync                          # install all deps into .venv
uv run pytest                    # run the unit test suite (tests/unit only, see pytest config)
uv run pytest -q                 # quiet
uv run pytest tests/unit/test_lift.py::TestLift::test_set   # run a single test
uv run pytest -k namespace       # run tests matching a keyword

uv pip install -e ".[docs]"      # install docs extras
mkdocs serve                     # preview docs site locally at :8000
```

`pyproject.toml` pins `testpaths = ["tests/unit"]`, so a bare `pytest` only runs the offline unit tests. The other scripts under `tests/` (e.g. `tests/test_lift.py`, `tests/test_tools_bbox.py`, `tests/test_call_action.py`) are **live-robot integration/interactive scripts**, not pytest cases — run them directly with `python tests/<name>.py --ip <robot-ip>` against a running robot. Likewise `examples/*.py` are runnable demos that connect to real hardware.

## Architecture

The design is a **three-layer transport abstraction** so that high-level modules never know which wire protocol is in use:

```
WalkieRobot (robot.py)          ← user-facing facade; exposes .nav .status .arm .lift .camera .cameras .viz .tools
   │  holds one ROS transport + one camera transport, injects them into modules
   ▼
Modules (walkie_sdk/modules/)   ← Navigation, Telemetry, Arm, Lift, Camera, MultiCamera, Visualization, Tools
   │  each takes a ROSTransportInterface in __init__ and calls publish/subscribe/call_action/call_service
   ▼
TransportFactory (core/factory.py) → ROSTransportInterface / CameraTransportInterface (core/interfaces/)
   │  lazy-imports the concrete transport based on a protocol enum
   ▼
Transports (core/transports/)   ← rosbridge/ (roslibpy WebSocket), zenoh/ (DDS), usb/ (cv2/V4L2 camera)
```

Key consequences when editing:

- **Modules are protocol-agnostic.** A module must only depend on the abstract `ROSTransportInterface` (`core/interfaces/ros_transport.py`) — its `connect/disconnect/subscribe/publish/call_action/cancel_action/call_service` methods. Never import a concrete transport (`ROSBridgeTransport`, `ZenohTransport`) inside a module. To add robot capability, add a module + register it in `WalkieRobot.__init__`.
- **The factory does lazy imports.** `TransportFactory.create_ros_transport` / `create_camera_transport` import the concrete class only inside the matching `if protocol == ...` branch, so installing one protocol's deps doesn't force the others. Add new protocols by extending the `ROSProtocol`/`CameraProtocol` enums and adding a branch.
- **`ros_protocol="auto"`** tries zenoh then rosbridge (`_auto_detect_ros`).
- **Connection lifecycle:** `WalkieRobot.__init__` constructs transports + modules, then calls `_connect()`, which connects the transport and then explicitly wires up subscriptions that require a live connection — `status.start()`, `arm._setup_state_subscription()`, `lift._setup_state_subscription()`. New stateful modules that subscribe must follow this two-phase pattern (construct in `__init__`, subscribe in `_connect`).

### Topic/type configuration (important)

All ROS topic names, message types, action types, and service names live in **`walkie_sdk/config/ros_topics.py`** as module-level dicts (`CAMERA_TOPICS`, `ARM_TOPICS`, `NAV_TOPICS`, `TELEMETRY_TOPICS`, `VIZ_TOPICS`, `LIFT_TOPICS`, `OB_POSE_SERVICE`, etc.). Each entry defaults to an env var (e.g. `WALKIE_ARM_COMMANDS`) falling back to a literal.

- **Always read these via dict lookup at call time**, e.g. `ARM_TOPICS["commands"]`, never by binding a value to a local at import time. `load_config(yaml_path)` mutates these dicts **in place** (`.update(...)`) so that every module that imported the dict sees the new values. Binding `cmd = ARM_TOPICS["commands"]` at import would freeze the default and silently ignore YAML/env overrides. (This is why historical commits "read config dicts at runtime" and "use single quotes for dict keys inside f-strings in arm.py" exist.)
- A YAML config (top-level `ros_topics.yaml`, or a custom path) is applied by passing `config_path=` to `WalkieRobot(...)`, which calls `load_config` before modules are built.

### Namespacing

ROS namespaces are applied at call time via `apply_namespace(name, namespace)` (`utils/namespace.py`), which prefixes `name` with `namespace/` (slashes stripped) or returns `name` unchanged when namespace is empty. Topic/action names in config are stored **without leading slashes** so namespacing produces `robot1/joint_states`, not `//...`. Each module stores `self._namespace` and exposes namespaced topic names as `@property` getters (e.g. `Navigation.cmd_vel_topic`). Setting `WalkieRobot.namespace` fans the value out to all modules; arm/lift re-subscribe immediately, telemetry keeps its old subscription until restart.

### Conventions

- ROS messages are passed as **plain dicts** matching the ROS message structure (see `Navigation.go_to` building a `NavigateToPose` goal). Geometry conversions (Euler↔quaternion, bbox→DetectionArray, poses→arrays) live in `utils/converters.py`.
- Blocking vs non-blocking: action-based calls (nav `go_to`, arm/lift moves) take a `blocking` flag. Non-blocking runs the `call_action` in a daemon thread and returns `"IN_PROGRESS"`. When publishing a command and then polling for completion, **publish synchronously first, then spawn the polling thread** (see lift commit history) to avoid races.
- Modules degrade gracefully: camera connection failure disables `.camera` rather than aborting; service/action errors are caught and surfaced as `None`/`"FAILED"` return values, not exceptions, in the user-facing methods.

## Testing notes

- Unit tests (`tests/unit/`) are pure and offline — they exercise converters, namespace logic, imports, and module behavior with fakes/mocks. Keep new unit tests here so `pytest` stays runnable without a robot.
- `requires-python = ">=3.11"`; `.python-version` pins 3.11.
