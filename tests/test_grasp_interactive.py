"""
Interactive grasp tester — confirm the SDK Grasp module end-to-end.

Draw ONE bounding box around an object on the live camera feed, then press
Space/Enter to call Grasp.from_mask() in bbox mode. The returned grasp poses
(in the planning frame, e.g. base_footprint) are printed to the console and
published as RViz2 markers (an arrow per grasp, coloured by score), so you can
SEE that the SDK → grasp_server round-trip works.

This exercises bbox mode (mask=None): the server falls back to the bbox region,
so no YOLO mask is required — just point the camera at an object and box it.

Requires, on the robot side:
  - rosbridge_server (and/or a zenoh router for the default hybrid transport)
  - ros2 run walkie_perception grasp_server
  - the ZED head (RGB for this window + depth/cloud for the grasp)

Controls:
  Left-click drag  — draw a bounding box (replaces the previous one)
  Space / Enter    — call from_mask(bbox=...) for the current box
  C                — clear the box and markers
  Q / Esc          — quit

View in RViz2: Fixed Frame = the printed planning frame (base_footprint),
add a MarkerArray/Marker display on 'walkie/viz_markers'
(prefixed with your namespace if set).

Usage:
    python tests/test_grasp_interactive.py
    python tests/test_grasp_interactive.py --ip 192.168.1.100
    python tests/test_grasp_interactive.py --ip 192.168.1.100 --timeout 25
    python tests/test_grasp_interactive.py --namespace robot1
"""

import argparse
import threading

import cv2
import numpy as np

from walkie_sdk import WalkieRobot, ARROW, TEXT_VIEW_FACING


# ── Colours (BGR) ───────────────────────────────────────────────────────────
_GREEN  = (0, 255, 0)
_YELLOW = (0, 220, 255)
_RED    = (0, 60, 255)
_WHITE  = (255, 255, 255)
_BLACK  = (0, 0, 0)
_GRAY   = (160, 160, 160)
_CYAN   = (255, 220, 0)


# ── Shared state ────────────────────────────────────────────────────────────

class AppState:
    def __init__(self):
        self.box = None                 # (x1, y1, x2, y2) pixel coords, or None
        self.drawing = False
        self.start_pt = None
        self.cur_pt = None
        self.status = "Draw a box around an object, then press Space"
        self.summary = ""               # multi-line result text under the box
        self.found = False              # any grasps in the last query?
        self.querying = False
        self.marker_ids = []            # RViz2 marker IDs to clear next query
        self.frame_wh = None            # (w, h) of the displayed camera frame
        self.cam_wh = None              # (w, h) of the perception CameraInfo
        self.lock = threading.Lock()


state = AppState()


# ── RViz2 marker colour by grasp score ──────────────────────────────────────

def _score_color(score):
    if score is None:
        return [0.6, 0.6, 0.6, 1.0]
    if score > 0.5:
        return [0.0, 1.0, 0.2, 1.0]   # green
    if score > 0.3:
        return [1.0, 1.0, 0.0, 1.0]   # yellow
    return [1.0, 0.5, 0.0, 1.0]       # orange


# ── Mouse callback ──────────────────────────────────────────────────────────

def mouse_cb(event, x, y, flags, param):
    if state.querying:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        state.drawing = True
        state.start_pt = (x, y)
        state.cur_pt = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE and state.drawing:
        state.cur_pt = (x, y)
    elif event == cv2.EVENT_LBUTTONUP and state.drawing:
        state.drawing = False
        sx, sy = state.start_pt
        x1, y1 = min(sx, x), min(sy, y)
        x2, y2 = max(sx, x), max(sy, y)
        if x2 - x1 > 8 and y2 - y1 > 8:
            with state.lock:
                state.box = (x1, y1, x2, y2)
                state.summary = ""
                state.found = False
            state.status = "Box set — press Space to find grasps"
        state.start_pt = state.cur_pt = None


# ── Overlay drawing ─────────────────────────────────────────────────────────

def _text_bg(img, text, org, font, scale, color, thickness=1):
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = org
    cv2.rectangle(img, (x - 2, y - th - 4), (x + tw + 2, y + baseline), _BLACK, -1)
    cv2.putText(img, text, org, font, scale, color, thickness, cv2.LINE_AA)


def draw_overlay(frame):
    img = frame.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX

    with state.lock:
        box = state.box
        summary = state.summary
        found = state.found

    if box is not None:
        x1, y1, x2, y2 = box
        color = _GREEN if found else (_GRAY if state.querying else _YELLOW)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        if state.querying:
            _text_bg(img, "querying...", (x1, y2 + 16), font, 0.45, _GRAY)
        elif summary:
            for k, line in enumerate(summary.split("\n")):
                _text_bg(img, line, (x1, y2 + 16 + k * 18), font, 0.45,
                         _GREEN if found else _RED)

    # Live drag preview
    if state.drawing and state.start_pt and state.cur_pt:
        sx, sy = state.start_pt
        cx, cy = state.cur_pt
        cv2.rectangle(img, (min(sx, cx), min(sy, cy)),
                      (max(sx, cx), max(sy, cy)), _CYAN, 1)

    h, w = img.shape[:2]
    cv2.rectangle(img, (0, 0), (w, 26), _BLACK, -1)
    cv2.putText(img, state.status, (6, 18), font, 0.5, _WHITE, 1, cv2.LINE_AA)
    controls = "[Space/Enter] Find grasps   [C] Clear   [Q/Esc] Quit"
    cv2.rectangle(img, (0, h - 22), (w, h), _BLACK, -1)
    cv2.putText(img, controls, (6, h - 7), font, 0.38, _GRAY, 1, cv2.LINE_AA)
    return img


# ── Grasp query (background thread) ─────────────────────────────────────────

def query_thread(robot, timeout):
    with state.lock:
        box = state.box
    if box is None:
        state.status = "No box drawn"
        state.querying = False
        return

    x1, y1, x2, y2 = box
    disp_w, disp_h = state.frame_wh or (x2, y2)

    # In depth mode the node applies the bbox in the DEPTH IMAGE's pixel space
    # (which can differ from the RGB CameraInfo). Pull the depth frame to (a)
    # scale the box to that resolution and (b) report what depth is actually
    # in the box — the direct check for "0 points".
    depth = robot.camera.get_depth() if robot.camera else None
    if depth is not None:
        dh, dw = depth.shape[:2]
        sx, sy = dw / disp_w, dh / disp_h
        rx1, ry1 = max(0, int(x1 * sx)), max(0, int(y1 * sy))
        rx2, ry2 = min(dw, int(x2 * sx)), min(dh, int(y2 * sy))
        roi = depth[ry1:ry2, rx1:rx2].astype("float32")
        finite = np.isfinite(roi) & (roi != 0.0)
        in_range = finite & (roi > 0.1) & (roi < 3.0)
        n_fin, n_in = int(finite.sum()), int(in_range.sum())
        vals = roi[in_range]
        rng = (f"{vals.min():.2f}-{vals.max():.2f}m" if vals.size else "n/a")
        print(f"\n[depth] image={dw}x{dh} roi=({rx1},{ry1},{rx2},{ry2}) "
              f"pixels={roi.size} finite={n_fin} in_range(0.1-3m)={n_in} range={rng}")
        cam_w, cam_h = dw, dh           # scale the bbox to the DEPTH resolution
    else:
        print("\n[depth] get_depth() returned None — depth stream not reaching the SDK")
        cam_w, cam_h = state.cam_wh or (disp_w, disp_h)

    sx, sy = cam_w / disp_w if disp_w else 1.0, cam_h / disp_h if disp_h else 1.0
    bbox = [
        (x1 + x2) / 2.0 * sx,
        (y1 + y2) / 2.0 * sy,
        (x2 - x1) * sx,
        (y2 - y1) * sy,
    ]
    state.status = f"from_mask bbox (scaled → {cam_w}x{cam_h})..."
    print(f"[query] window box=({x1},{y1},{x2},{y2}) disp={disp_w}x{disp_h} "
          f"-> node bbox={[round(v) for v in bbox]} cam={cam_w}x{cam_h}")

    res = robot.grasp.from_mask(bbox=bbox, max_grasps=5, timeout=timeout)

    with state.lock:
        for mid in state.marker_ids:
            try:
                robot.viz.delete_marker(mid)
            except Exception:
                pass
        state.marker_ids.clear()

        if res is None:
            state.found = False
            state.summary = "from_mask returned None\n(service unreachable/timeout)"
            state.status = "FAIL — None (check grasp_server / connection)"
            print("[result] None — service call failed or timed out")
        elif not res.get("success") or not res.get("grasps"):
            state.found = False
            msg = res.get("message", "")
            state.summary = f"no grasps\n{msg[:48]}"
            state.status = f"No grasps: {msg}"
            print(f"[result] success={res.get('success')} grasps=0 msg='{msg}'")
        else:
            grasps = res["grasps"]
            frame = res["planning_frame"] or "base_footprint"
            best = grasps[0]
            state.found = True
            state.summary = (
                f"{len(grasps)} grasps | frame {frame}\n"
                f"top score {best['score']:.3f}  width {best['width']*100:.1f}cm\n"
                f"pos ({best['position'][0]:.2f}, {best['position'][1]:.2f}, "
                f"{best['position'][2]:.2f})"
            )
            state.status = f"OK — {len(grasps)} grasp(s); see RViz2 ('{frame}')"
            print(f"[result] success | {len(grasps)} grasp(s) in '{frame}':")
            for i, g in enumerate(grasps):
                print(f"  #{i}: pos={[round(v, 3) for v in g['position']]} "
                      f"quat={[round(v, 3) for v in g['orientation']]} "
                      f"score={g['score']:.3f} width={g['width']*100:.1f}cm")
                # Arrow along the grasp orientation, coloured by score.
                mid = robot.draw_marker(
                    position=g["position"],
                    quaternion=g["orientation"],
                    marker_type=ARROW,
                    color=_score_color(g["score"]),
                    scale=[0.10, 0.012, 0.012],
                    frame_id=frame,
                    ns="grasp",
                    marker_id=100 + i,
                )
                state.marker_ids.append(mid)
            # Label the best grasp.
            tid = robot.draw_marker(
                position=[best["position"][0], best["position"][1],
                          best["position"][2] + 0.06],
                marker_type=TEXT_VIEW_FACING,
                text=f"best {best['score']:.2f}",
                color=[1.0, 1.0, 1.0, 1.0],
                scale=[0.04, 0.04, 0.04],
                frame_id=frame,
                ns="grasp_label",
                marker_id=200,
            )
            state.marker_ids.append(tid)

    state.querying = False


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Interactive grasp-from-mask (bbox) tester")
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--cam-port",  type=int, default=7447, help="Zenoh camera port (default: 7447)")
    parser.add_argument("--timeout",   type=float, default=25.0, help="Grasp service timeout (default: 25.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    args = parser.parse_args()

    print(f"Connecting to {args.ip}:{args.port} ...")
    robot = WalkieRobot(
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="zenoh",
        camera_port=args.cam_port,
        namespace=args.namespace,
    )
    print("Connected.")

    # Make sure the GPU model is loaded, and report server state.
    if robot.grasp.set_standby(True):
        print("Grasp model loaded.")
    print(f"grasp status: {robot.grasp.status()}")

    # Perception camera resolution — the pixel space the node interprets the
    # bbox in. Drawn boxes are scaled from the window into this resolution.
    info = robot.camera.get_camera_info(timeout=5.0) if robot.camera else None
    if info and info.get("width") and info.get("height"):
        state.cam_wh = (int(info["width"]), int(info["height"]))
        print(f"perception camera resolution: {state.cam_wh[0]}x{state.cam_wh[1]}")
    else:
        print("⚠ no CameraInfo — assuming the window matches the perception resolution")
    print("Opening camera window...")

    win = "GraspTester"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    callback_registered = False

    try:
        while True:
            frame = robot.camera.get_frame() if robot.camera else None
            if frame is not None:
                state.frame_wh = (frame.shape[1], frame.shape[0])
                cv2.imshow(win, draw_overlay(frame))
                if not callback_registered:
                    cv2.setMouseCallback(win, mouse_cb)
                    callback_registered = True
            else:
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for camera...", (120, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, _WHITE, 1, cv2.LINE_AA)
                cv2.imshow(win, placeholder)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key in (ord(' '), 13):
                if not state.querying and state.box is not None:
                    state.querying = True
                    threading.Thread(target=query_thread, args=(robot, args.timeout),
                                     daemon=True).start()
            elif key == ord('c'):
                with state.lock:
                    state.box = None
                    state.summary = ""
                    state.found = False
                    for mid in state.marker_ids:
                        try:
                            robot.viz.delete_marker(mid)
                        except Exception:
                            pass
                    state.marker_ids.clear()
                state.status = "Cleared — draw a new box"
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.disconnect()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
