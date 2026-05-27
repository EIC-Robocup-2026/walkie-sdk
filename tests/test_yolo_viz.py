"""
YOLO → 3D position visualizer for Walkie SDK.

Runs a YOLO model (ultralytics) LOCALLY on the robot's camera feed, draws the
detected boxes + class labels on the feed, then feeds the boxes into
bboxes_to_positions() and publishes RViz2 markers labeled with each object's
class name and 3D position.

Two views of the same result:
  - OpenCV window : live YOLO boxes + class/confidence, plus a HUD list of the
                    last query's "class: (x, y, z)" positions.
  - RViz2         : a colored sphere per object + a text label "class (x,y,z)"
                    at its 3D position.

Requirements:
  uv sync --extra yolo            # installs ultralytics (+ torch)
  # or one-off:  uv run --with ultralytics python tests/test_yolo_viz.py ...
The model file (default yolov8n.pt) is auto-downloaded by ultralytics on first use.

Server needs: rosbridge + zenoh camera + the perception get_3d_poses service.

Controls:
  Space / Enter — query bboxes_to_positions() for the current detections
  A             — toggle auto-query (every --interval seconds)
  C             — clear RViz2 markers
  Q / Esc       — quit

Usage:
    uv run --with ultralytics python tests/test_yolo_viz.py --ip 192.168.1.100
    uv run --with ultralytics python tests/test_yolo_viz.py --ip 192.168.1.100 --model yolov8s.pt --conf 0.4
    uv run --with ultralytics python tests/test_yolo_viz.py --ip 192.168.1.100 --auto --interval 2.0
    uv run --with ultralytics python tests/test_yolo_viz.py --ip 192.168.1.100 --viz-frame camera_link
"""

import argparse
import threading
import time

import cv2
import numpy as np

from walkie_sdk import WalkieRobot, SPHERE, TEXT_VIEW_FACING


# ── Colours (BGR for OpenCV) ────────────────────────────────────────────────
_GREEN = (0, 255, 0)
_WHITE = (255, 255, 255)
_BLACK = (0, 0, 0)
_GRAY = (160, 160, 160)
_CYAN = (255, 220, 0)

# RGBA marker colours (cycled per object) for RViz2
_MARKER_COLORS = [
    [1.0, 0.0, 0.0, 1.0],  # red
    [0.0, 1.0, 0.0, 1.0],  # green
    [0.0, 0.4, 1.0, 1.0],  # blue
    [1.0, 1.0, 0.0, 1.0],  # yellow
    [1.0, 0.0, 1.0, 1.0],  # magenta
    [0.0, 1.0, 1.0, 1.0],  # cyan
]


class AppState:
    def __init__(self):
        # Latest YOLO detections (updated every frame): list of dicts
        #   {name, conf, cx, cy, w, h}
        self.detections: list[dict] = []
        # Last service query results: list of dicts {name, conf, pos:[x,y,z]}
        self.last_results: list[dict] = []
        self.marker_ids: list[int] = []
        self.querying = False
        self.auto = False
        self.status = "Press Space to query 3D positions"
        self.lock = threading.Lock()


state = AppState()


# ── Overlay drawing ──────────────────────────────────────────────────────────


def _put_text_with_bg(img, text, org, scale=0.45, color=_WHITE, thickness=1):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = org
    cv2.rectangle(img, (x - 2, y - th - 4), (x + tw + 2, y + baseline), _BLACK, -1)
    cv2.putText(img, text, org, font, scale, color, thickness, cv2.LINE_AA)


def draw_overlay(frame: np.ndarray) -> np.ndarray:
    img = frame.copy()
    h, w = img.shape[:2]

    with state.lock:
        dets = list(state.detections)
        results = list(state.last_results)
        status = state.status
        auto = state.auto
        querying = state.querying

    # Live YOLO boxes + class/confidence
    for d in dets:
        x1 = int(d["cx"] - d["w"] / 2)
        y1 = int(d["cy"] - d["h"] / 2)
        x2 = int(d["cx"] + d["w"] / 2)
        y2 = int(d["cy"] + d["h"] / 2)
        cv2.rectangle(img, (x1, y1), (x2, y2), _GREEN, 2)
        _put_text_with_bg(img, f"{d['name']} {d['conf']:.2f}", (x1 + 3, y1 + 16), color=_GREEN)

    # HUD: status bar (top)
    cv2.rectangle(img, (0, 0), (w, 26), _BLACK, -1)
    mode = "AUTO" if auto else "manual"
    busy = "  [querying...]" if querying else ""
    cv2.putText(img, f"[{mode}] {status}{busy}", (6, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, _WHITE, 1, cv2.LINE_AA)

    # HUD: last query's 3D positions (left list) — "label the position of objects"
    y = 48
    _put_text_with_bg(img, "3D positions (last query):", (6, y), scale=0.45, color=_CYAN)
    y += 22
    if not results:
        _put_text_with_bg(img, "  (none yet)", (6, y), scale=0.45, color=_GRAY)
    else:
        for i, r in enumerate(results):
            px, py, pz = r["pos"]
            line = f"  {r['name']}: ({px:.2f}, {py:.2f}, {pz:.2f})"
            color = tuple(int(c * 255) for c in _MARKER_COLORS[i % len(_MARKER_COLORS)][2::-1])
            _put_text_with_bg(img, line, (6, y), scale=0.45, color=color)
            y += 20

    # HUD: controls (bottom)
    controls = "[Space] Query   [A] Auto   [C] Clear   [Q/Esc] Quit"
    cv2.rectangle(img, (0, h - 22), (w, h), _BLACK, -1)
    cv2.putText(img, controls, (6, h - 7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, _GRAY, 1, cv2.LINE_AA)
    return img


# ── Service query + RViz markers (runs in a background thread) ───────────────


def query_thread(robot: WalkieRobot, timeout: float, frame_id: str):
    with state.lock:
        dets = list(state.detections)

    if not dets:
        state.status = "No detections to query"
        state.querying = False
        return

    coords = [[d["cx"], d["cy"], d["w"], d["h"]] for d in dets]
    state.status = f"Querying {len(coords)} detection(s)..."
    positions = robot.tools.bboxes_to_positions(coords, timeout=timeout)

    with state.lock:
        # Clear previous markers
        for mid in state.marker_ids:
            try:
                robot.viz.delete_marker(mid)
            except Exception:
                pass
        state.marker_ids.clear()
        state.last_results.clear()

        if positions is None:
            state.status = "Service returned None — check perception service"
        elif len(positions) == 0:
            state.status = "Service returned 0 positions"
        else:
            n = min(len(positions), len(dets))
            if len(positions) != len(dets):
                print(f"[warn] {len(dets)} detections but {len(positions)} positions — "
                      f"labeling first {n}")
            for i in range(n):
                pos = positions[i]
                name = dets[i]["name"]
                color = _MARKER_COLORS[i % len(_MARKER_COLORS)]

                sphere_id = robot.draw_marker(
                    position=pos, quaternion=[0.0, 0.0, 0.0, 1.0],
                    marker_type=SPHERE, color=color, scale=[0.1, 0.1, 0.1],
                    frame_id=frame_id, ns="yolo_detections",
                )
                text_id = robot.draw_marker(
                    position=[pos[0], pos[1], pos[2] + 0.15], quaternion=[0.0, 0.0, 0.0, 1.0],
                    marker_type=TEXT_VIEW_FACING,
                    text=f"{name} ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                    color=[1.0, 1.0, 1.0, 1.0], scale=[0.08, 0.08, 0.08],
                    frame_id=frame_id, ns="yolo_detections_label",
                )
                state.marker_ids += [sphere_id, text_id]
                state.last_results.append({"name": name, "conf": dets[i]["conf"], "pos": pos})

            state.status = f"Got {n} 3D position(s) — see RViz2"
            for r in state.last_results:
                p = r["pos"]
                print(f"  {r['name']:15s} conf={r['conf']:.2f}  "
                      f"pos=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")

    state.querying = False


def start_query(robot, timeout, frame_id):
    if state.querying:
        return
    state.querying = True
    threading.Thread(target=query_thread, args=(robot, timeout, frame_id), daemon=True).start()


# ── Main ─────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="YOLO → 3D position visualizer")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--cam-port", type=int, default=7447, help="Zenoh camera port (default: 7447)")
    parser.add_argument("--timeout", type=float, default=5.0, help="Service timeout in seconds (default: 5.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO weights (default: yolov8n.pt, auto-downloaded)")
    parser.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold (default: 0.25)")
    parser.add_argument("--device", default=None, help="YOLO device, e.g. 'cpu' or '0' (default: auto)")
    parser.add_argument("--viz-frame", default="map", dest="viz_frame",
                        help="TF frame for RViz markers (default: map). Set to the frame the "
                             "perception service returns poses in.")
    parser.add_argument("--auto", action="store_true", help="Auto-query every --interval seconds")
    parser.add_argument("--interval", type=float, default=2.0, help="Auto-query interval seconds (default: 2.0)")
    args = parser.parse_args()

    # Import ultralytics lazily so the rest of the SDK doesn't depend on it.
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics is not installed. Install it with:\n"
              "    uv sync --extra yolo\n"
              "or run one-off:\n"
              "    uv run --with ultralytics python tests/test_yolo_viz.py ...")
        raise SystemExit(1)

    print(f"Loading YOLO model '{args.model}' ...")
    model = YOLO(args.model)

    print(f"Connecting to {args.ip} (rosbridge:{args.port}, zenoh camera:{args.cam_port}) ...")
    robot = WalkieRobot(
        ip=args.ip,
        ros_protocol="rosbridge",
        camera_protocol="zenoh",
        ros_port=args.port,
        camera_port=args.cam_port,
        namespace=args.namespace,
    )
    print("Connected. Opening camera window...")
    try:
        robot.viz.clear_markers()
    except Exception:
        pass

    state.auto = args.auto
    win = "YOLO 3D Visualizer"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    last_auto = 0.0

    try:
        while True:
            frame = robot.camera.get_frame() if robot.camera else None

            if frame is not None:
                # Run YOLO on the BGR frame
                result = model(frame, conf=args.conf, device=args.device, verbose=False)[0]
                dets = []
                if result.boxes is not None and len(result.boxes) > 0:
                    xywh = result.boxes.xywh.cpu().numpy()       # [cx, cy, w, h] pixels
                    clss = result.boxes.cls.cpu().numpy().astype(int)
                    confs = result.boxes.conf.cpu().numpy()
                    for (cx, cy, w, h), c, cf in zip(xywh, clss, confs):
                        dets.append({
                            "name": model.names.get(int(c), str(c)),
                            "conf": float(cf),
                            "cx": float(cx), "cy": float(cy), "w": float(w), "h": float(h),
                        })
                with state.lock:
                    state.detections = dets

                cv2.imshow(win, draw_overlay(frame))

                # Auto-query throttled by interval
                if state.auto and not state.querying and (time.time() - last_auto) >= args.interval:
                    last_auto = time.time()
                    start_query(robot, args.timeout, args.viz_frame)
            else:
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for camera...", (120, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, _WHITE, 1, cv2.LINE_AA)
                cv2.imshow(win, placeholder)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key in (ord(" "), 13):
                start_query(robot, args.timeout, args.viz_frame)
            elif key == ord("a"):
                state.auto = not state.auto
                state.status = f"Auto-query {'ON' if state.auto else 'OFF'}"
            elif key == ord("c"):
                with state.lock:
                    for mid in state.marker_ids:
                        try:
                            robot.viz.delete_marker(mid)
                        except Exception:
                            pass
                    state.marker_ids.clear()
                    state.last_results.clear()
                try:
                    robot.viz.clear_markers()
                except Exception:
                    pass
                state.status = "Cleared markers"

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.disconnect()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
