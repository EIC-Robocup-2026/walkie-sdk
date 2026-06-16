"""
Hardware integration test for camera access (bot.camera / bot.cameras) on a live robot.

Read-only — no motion. Verifies frames actually arrive and are well-formed.

Two paths are tested:

  1. Facade path (default): bot.camera and bot.cameras as wired by WalkieRobot.
     IMPORTANT: WalkieRobot builds a single-camera ZenohCamera, and MultiCamera
     only treats a transport as multi-camera when it subclasses
     MultiCameraTransportInterface — which ZenohCamera does NOT. So through
     bot.cameras you will only ever get the "head" frame today, regardless of the
     name passed. (The README's `cameras={...}` constructor arg is also not
     implemented on WalkieRobot.) This path documents/locks in that behavior.

  2. Direct multi-camera path (--multi): constructs ZenohCamera(multi_camera=True)
     directly and reads head/left/right via get_all_frames()/get_frame(name).
     This is the only way to currently pull multiple camera streams.

Requires: a robot publishing camera image topics over Zenoh (see CAMERA_TOPICS).

Usage:
    python tests/test_multi_camera.py --ip 192.168.1.100
    python tests/test_multi_camera.py --ip 192.168.1.100 --show          # cv2 windows
    python tests/test_multi_camera.py --ip 192.168.1.100 --multi         # direct multi-cam
    python tests/test_multi_camera.py --ip 192.168.1.100 --multi --show
"""

import argparse
import sys
import time

import numpy as np

from walkie_sdk import WalkieRobot


# ── Helpers ────────────────────────────────────────────────────────────────


def _section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _pass(msg: str) -> str:
    print(f"  [PASS] {msg}")
    return "PASS"


def _fail(msg: str) -> str:
    print(f"  [FAIL] {msg}")
    return "FAIL"


def _skip(msg: str) -> str:
    print(f"  [SKIP] {msg}")
    return "SKIP"


def _is_image(frame) -> bool:
    return isinstance(frame, np.ndarray) and frame.ndim in (2, 3) and frame.size > 0


def _wait_for_frame(getter, timeout: float):
    """Poll getter() until it returns an image or timeout. getter returns a frame or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        frame = getter()
        if _is_image(frame):
            return frame
        time.sleep(0.1)
    return None


def _maybe_show(name: str, frame, show: bool) -> None:
    if not show or frame is None:
        return
    try:
        import cv2
        cv2.imshow(f"camera: {name}", frame)
        cv2.waitKey(500)
    except Exception as e:
        print(f"    (display skipped: {e})")


# ── Facade-path tests (bot.camera / bot.cameras) ─────────────────────────────


def test_single_camera(bot: WalkieRobot, timeout: float, show: bool) -> str:
    _section("TEST 1: bot.camera.get_frame()")
    if bot.camera is None:
        return _fail("bot.camera is None — camera failed to connect")
    frame = _wait_for_frame(bot.camera.get_frame, timeout)
    if frame is None:
        return _fail(f"no frame within {timeout}s (is_streaming={bot.camera.is_streaming})")
    _maybe_show("head (bot.camera)", frame, show)
    return _pass(f"frame {frame.shape}, dtype={frame.dtype}, shape_prop={bot.camera.frame_shape}")


def test_cameras_facade(bot: WalkieRobot, timeout: float, show: bool) -> str:
    _section("TEST 2: bot.cameras facade (single-wrapped — head only)")
    if bot.cameras is None:
        return _fail("bot.cameras is None")
    print(f"    camera_names: {bot.cameras.camera_names}")
    print(f"    is_streaming: {bot.cameras.is_streaming}")

    head = _wait_for_frame(lambda: bot.cameras.get_frame("head"), timeout)
    if head is None:
        return _fail(f"get_frame('head') gave no frame within {timeout}s")
    _maybe_show("head (bot.cameras)", head, show)

    all_frames = bot.cameras.get_all_frames()
    print(f"    get_all_frames() keys: {list(all_frames.keys())}")
    shape = bot.cameras.get_frame_shape("head")
    if "head" not in all_frames:
        return _fail("get_all_frames() did not include 'head'")
    return _pass(f"head frame {head.shape}; get_all_frames keys={list(all_frames.keys())}; "
                 f"frame_shape={shape}")


# ── Direct multi-camera path (--multi) ───────────────────────────────────────


def test_direct_multicamera(host: str, port: int, timeout: float, show: bool) -> str:
    _section("TEST 3: direct ZenohCamera(multi_camera=True)")
    try:
        from walkie_sdk.core.transports.zenoh import ZenohCamera
    except ImportError as e:
        return _skip(f"zenoh camera transport unavailable: {e}")

    cam = ZenohCamera(host=host, port=port, multi_camera=True)
    try:
        cam.connect()
    except Exception as e:
        return _fail(f"ZenohCamera.connect() failed: {e}")

    try:
        # Give the subscribers time to receive at least one frame on any topic.
        got_any = _wait_for_frame(
            lambda: next(iter(cam.get_all_frames().values()), None), timeout
        )
        if got_any is None:
            return _fail(f"no frames on any camera topic within {timeout}s")

        frames = cam.get_all_frames()
        print(f"    cameras with frames: {list(frames.keys())}")
        for name, frame in frames.items():
            ok = _is_image(frame)
            print(f"      {name:8s}: {'OK ' + str(frame.shape) if ok else 'INVALID'}")
            _maybe_show(name, frame, show)

        # Named convenience getters
        for name in ("head", "left", "right"):
            f = cam.get_frame(name)
            print(f"      get_frame('{name}') -> {f.shape if _is_image(f) else None}")

        valid = [n for n, f in frames.items() if _is_image(f)]
        if not valid:
            return _fail("frames present but none were valid images")
        return _pass(f"received valid frames from: {valid}")
    finally:
        cam.disconnect()


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="Hardware test for camera access (bot.camera / bot.cameras)")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--camera-port", type=int, default=7447, help="Zenoh camera port (default: 7447)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for frames (default: 10.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--multi", action="store_true", help="Also test direct ZenohCamera(multi_camera=True)")
    parser.add_argument("--show", action="store_true", help="Display frames in OpenCV windows")
    args = parser.parse_args()

    print(f"\nConnecting to {args.ip} (rosbridge:{args.port}, zenoh camera:{args.camera_port}) ...")
    bot = WalkieRobot(
        ip=args.ip,
        camera_protocol="zenoh",
        camera_port=args.camera_port,
        namespace=args.namespace,
    )
    print("Connected.\n")

    results = []
    try:
        results.append(test_single_camera(bot, args.timeout, args.show))
        results.append(test_cameras_facade(bot, args.timeout, args.show))
        if args.multi:
            results.append(test_direct_multicamera(args.ip, args.camera_port, args.timeout, args.show))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        if args.show:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass
        bot.disconnect()
        print("\nDisconnected.")

    passed = results.count("PASS")
    failed = results.count("FAIL")
    skipped = results.count("SKIP")
    total = len(results)
    print(f"\n{'=' * 60}")
    print(f"  Results: {passed} passed, {failed} failed, {skipped} skipped  (of {total})")
    print("=" * 60)
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
