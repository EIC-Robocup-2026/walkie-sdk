"""
Interactive bbox → 3D position tester.

Draw bounding boxes on the live camera feed with your mouse, then press
Space/Enter to query bboxes_to_positions() and see the 3D results overlaid.

Controls:
  Left-click drag  — draw a bounding box
  Space / Enter    — call bboxes_to_positions() for all drawn boxes
  U                — undo last box
  C                — clear all boxes and labels
  Q / Esc          — quit

Usage:
    python tests/test_tools_bbox_interactive.py
    python tests/test_tools_bbox_interactive.py --ip 192.168.1.100
    python tests/test_tools_bbox_interactive.py --ip 192.168.1.100 --port 9090 --timeout 5.0
    python tests/test_tools_bbox_interactive.py --namespace robot1
"""

import argparse
import threading
import time

import cv2
import numpy as np

from walkie_sdk import WalkieRobot, SPHERE, TEXT_VIEW_FACING


# ── Colours ────────────────────────────────────────────────────────────────
_GREEN  = (0, 255, 0)
_YELLOW = (0, 220, 255)
_RED    = (0, 60, 255)
_WHITE  = (255, 255, 255)
_BLACK  = (0, 0, 0)
_GRAY   = (160, 160, 160)
_CYAN   = (255, 220, 0)


# ── Shared state ───────────────────────────────────────────────────────────

_MARKER_COLORS = [
    [1.0, 0.0, 0.0, 1.0],  # red
    [0.0, 1.0, 0.0, 1.0],  # green
    [0.0, 0.4, 1.0, 1.0],  # blue
    [1.0, 1.0, 0.0, 1.0],  # yellow
    [1.0, 0.0, 1.0, 1.0],  # magenta
]

class AppState:
    def __init__(self):
        self.boxes: list[tuple[int, int, int, int]] = []  # (x1, y1, x2, y2) pixel coords
        self.labels: dict[int, str] = {}                  # box index → "x=… y=… z=…"
        self.marker_ids: list[int] = []                   # RViz2 marker IDs to clear on next query
        self.drawing = False
        self.start_pt: tuple[int, int] | None = None
        self.cur_pt:   tuple[int, int] | None = None
        self.status = "Draw boxes, then press Space to query"
        self.querying = False
        self.lock = threading.Lock()


state = AppState()


# ── Mouse callback ─────────────────────────────────────────────────────────

def mouse_cb(event, x, y, flags, param):
    if state.querying:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        state.drawing = True
        state.start_pt = (x, y)
        state.cur_pt   = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE and state.drawing:
        state.cur_pt = (x, y)

    elif event == cv2.EVENT_LBUTTONUP and state.drawing:
        state.drawing = False
        sx, sy = state.start_pt
        x1, y1 = min(sx, x), min(sy, y)
        x2, y2 = max(sx, x), max(sy, y)
        if x2 - x1 > 8 and y2 - y1 > 8:
            with state.lock:
                state.boxes.append((x1, y1, x2, y2))
            state.status = f"{len(state.boxes)} box(es) — Space to query"
        state.start_pt = state.cur_pt = None


# ── Overlay drawing ────────────────────────────────────────────────────────

def _put_text_with_bg(img, text, org, font, scale, color, thickness=1):
    """Draw text with a dark background for readability."""
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = org
    cv2.rectangle(img, (x - 2, y - th - 4), (x + tw + 2, y + baseline), _BLACK, -1)
    cv2.putText(img, text, org, font, scale, color, thickness, cv2.LINE_AA)


def draw_overlay(frame: np.ndarray) -> np.ndarray:
    img = frame.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Committed boxes
    with state.lock:
        boxes = list(state.boxes)
        labels = dict(state.labels)

    for i, (x1, y1, x2, y2) in enumerate(boxes):
        color = _GREEN if i in labels else _YELLOW
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Index tag in top-left corner of box
        _put_text_with_bg(img, f"#{i+1}", (x1 + 4, y1 + 16), font, 0.45, color)

        # 3D label below the box (or "…" while querying)
        if i in labels:
            _put_text_with_bg(img, labels[i], (x1, y2 + 16), font, 0.45, _GREEN)
        elif state.querying:
            _put_text_with_bg(img, "querying...", (x1, y2 + 16), font, 0.45, _GRAY)

    # Live drag preview
    if state.drawing and state.start_pt and state.cur_pt:
        sx, sy = state.start_pt
        cx, cy = state.cur_pt
        cv2.rectangle(img, (min(sx, cx), min(sy, cy)), (max(sx, cx), max(sy, cy)), _CYAN, 1)

    # HUD: status bar at the top
    h, w = img.shape[:2]
    cv2.rectangle(img, (0, 0), (w, 26), _BLACK, -1)
    cv2.putText(img, state.status, (6, 18), font, 0.5, _WHITE, 1, cv2.LINE_AA)

    # HUD: controls at the bottom
    controls = "[Space/Enter] Query   [U] Undo   [C] Clear   [Q/Esc] Quit"
    cv2.rectangle(img, (0, h - 22), (w, h), _BLACK, -1)
    cv2.putText(img, controls, (6, h - 7), font, 0.38, _GRAY, 1, cv2.LINE_AA)

    return img


# ── Service query (runs in background thread) ──────────────────────────────

def query_thread(robot: WalkieRobot, timeout: float):
    with state.lock:
        boxes = list(state.boxes)

    if not boxes:
        state.status = "No boxes drawn"
        state.querying = False
        return

    coords = [
        [(x1 + x2) / 2.0, (y1 + y2) / 2.0, float(x2 - x1), float(y2 - y1)]
        for x1, y1, x2, y2 in boxes
    ]

    state.status = f"Querying {len(coords)} bbox(es)..."
    result = robot.tools.bboxes_to_positions(coords, timeout=timeout)

    with state.lock:
        # Clear previous RViz2 markers
        for mid in state.marker_ids:
            robot.viz.delete_marker(mid)
        state.marker_ids.clear()
        state.labels.clear()

        if result is None:
            state.status = "Service returned None — check connection"
        elif len(result) == 0:
            state.status = "Service returned 0 positions"
        else:
            for i, pos in enumerate(result):
                color = _MARKER_COLORS[i % len(_MARKER_COLORS)]

                # Sphere at the 3D position
                mid = robot.draw_marker(
                    position=pos,
                    quaternion=[0.0, 0.0, 0.0, 1.0],
                    marker_type=SPHERE,
                    color=color,
                    scale=[0.1, 0.1, 0.1],
                    frame_id="map",
                    ns="bbox_detections",
                )
                state.marker_ids.append(mid)

                # Text label above the sphere
                label_text = f"det_{i} ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                mid_text = robot.draw_marker(
                    position=[pos[0], pos[1], pos[2] + 0.15],
                    quaternion=[0.0, 0.0, 0.0, 1.0],
                    marker_type=TEXT_VIEW_FACING,
                    text=label_text,
                    color=[1.0, 1.0, 1.0, 1.0],
                    scale=[0.08, 0.08, 0.08],
                    frame_id="map",
                    ns="bbox_detections_label",
                )
                state.marker_ids.append(mid_text)

                state.labels[i] = f"x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f}"

            state.status = f"Got {len(result)} position(s) — draw more or press Q to quit"

    state.querying = False


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Interactive bbox → 3D position tester"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--cam-port",  type=int, default=7447, help="Zenoh camera port (default: 7447)")
    parser.add_argument("--timeout",   type=float, default=5.0, help="Service timeout in seconds (default: 5.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    args = parser.parse_args()

    print(f"Connecting to {args.ip}:{args.port} ...")
    robot = WalkieRobot(
        ip=args.ip,
        ros_protocol="rosbridge",
        camera_protocol="zenoh",
        ros_port=args.port,
        camera_port=args.cam_port,
        namespace=args.namespace,
    )
    print("Connected. Opening camera window...")

    win = "3DPoseTester"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    callback_registered = False

    try:
        while True:
            frame = robot.camera.get_frame() if robot.camera else None

            if frame is not None:
                display = draw_overlay(frame)
                cv2.imshow(win, display)

                # Register mouse callback only after the first real frame is shown,
                # matching the pattern in example_camera.py (namedWindow + imshow first).
                if not callback_registered:
                    cv2.setMouseCallback(win, mouse_cb)
                    callback_registered = True
            else:
                # Show placeholder while waiting for the first frame
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for camera...", (120, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, _WHITE, 1, cv2.LINE_AA)
                cv2.imshow(win, placeholder)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), 27):  # Q or Esc
                break

            elif key in (ord(' '), 13):  # Space or Enter
                if not state.querying and state.boxes:
                    state.querying = True
                    t = threading.Thread(
                        target=query_thread, args=(robot, args.timeout), daemon=True
                    )
                    t.start()

            elif key == ord('u'):  # Undo
                with state.lock:
                    if state.boxes:
                        idx = len(state.boxes) - 1
                        state.boxes.pop()
                        state.labels.pop(idx, None)
                        state.status = f"{len(state.boxes)} box(es)"

            elif key == ord('c'):  # Clear
                with state.lock:
                    state.boxes.clear()
                    state.labels.clear()
                state.status = "Cleared — draw new boxes"

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.disconnect()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
