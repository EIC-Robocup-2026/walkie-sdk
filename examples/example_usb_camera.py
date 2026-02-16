#!/usr/bin/env python3
"""
USB Camera Test - Tests the USBCamera transport and previews the feed.

Discovers available USB/V4L2 cameras, connects to one, and displays
a live preview window with FPS overlay.

Usage:
    # Auto-detect first available USB camera
    uv run python examples/example_usb_camera.py

    # Specify a device by stable path
    uv run python examples/example_usb_camera.py /dev/v4l/by-id/usb-Logitech_C920-video-index0

    # Specify a device by index
    uv run python examples/example_usb_camera.py 0

Controls:
    q - Quit
    s - Save snapshot to current directory
"""

import os
import sys
import time
from pathlib import Path


def list_usb_cameras():
    """
    Scan /dev/v4l/by-id/ for available V4L2 camera devices.

    Returns:
        List of (symlink_path, real_device_path) tuples.
    """
    by_id = Path("/dev/v4l/by-id")
    if not by_id.exists():
        return []

    devices = []
    for entry in sorted(by_id.iterdir()):
        if entry.is_symlink():
            real = entry.resolve()
            devices.append((str(entry), str(real)))
    return devices


def main():
    # -- Step 1: Discover available cameras --
    print("=" * 60)
    print("USB Camera Test")
    print("=" * 60)

    devices = list_usb_cameras()

    print("\nAvailable USB cameras (/dev/v4l/by-id/):")
    if devices:
        for i, (symlink, real) in enumerate(devices):
            print(f"  [{i}] {symlink}")
            print(f"      -> {real}")
    else:
        print("  (none found)")

    # -- Determine which device to use --
    device = None

    if len(sys.argv) > 1:
        arg = sys.argv[1]
        # If it looks like an integer, use as device index
        try:
            device = int(arg)
            print(f"\nUsing device index: {device}")
        except ValueError:
            device = arg
            print(f"\nUsing device path: {device}")
    elif devices:
        # Auto-select first device
        device = devices[0][0]
        print(f"\nAuto-selected: {device}")
    else:
        print("\nNo USB cameras found and no device path provided.")
        print(
            "Usage: uv run python examples/example_usb_camera.py [device_path_or_index]"
        )
        sys.exit(1)

    # -- Step 2: Connect and preview --
    try:
        import cv2
    except ImportError:
        print(
            "Error: opencv-python is required. Install with: pip install opencv-python"
        )
        sys.exit(1)

    from walkie_sdk.core.transports.usb import USBCamera

    print(f"\nConnecting to camera...")
    camera = USBCamera(device=device)

    try:
        camera.connect()
    except ConnectionError as e:
        print(f"Failed to open camera: {e}")
        sys.exit(1)

    # Wait briefly for first frame
    print("Waiting for first frame...")
    deadline = time.time() + 5.0
    while camera.get_frame() is None:
        if time.time() > deadline:
            print("Timeout waiting for first frame.")
            camera.disconnect()
            sys.exit(1)
        time.sleep(0.05)

    shape = camera.frame_shape
    print(f"Receiving frames: {shape[1]}x{shape[0]} ({shape[2]}ch)")
    print("\nControls:")
    print("  q - Quit")
    print("  s - Save snapshot")

    window_name = "USB Camera Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    frame_count = 0
    fps_start = time.time()
    fps = 0.0
    snapshot_count = 0

    try:
        while True:
            frame = camera.get_frame()

            if frame is not None:
                frame_count += 1
                elapsed = time.time() - fps_start
                if elapsed >= 1.0:
                    fps = frame_count / elapsed
                    frame_count = 0
                    fps_start = time.time()

                display = frame.copy()
                h, w = display.shape[:2]

                cv2.putText(
                    display,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    display,
                    f"{w}x{h}",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )

                cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("\nQuitting...")
                break
            elif key == ord("s") and frame is not None:
                snapshot_count += 1
                filename = f"usb_snapshot_{snapshot_count:03d}.jpg"
                cv2.imwrite(filename, frame)
                print(f"  Saved: {filename}")

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        cv2.destroyAllWindows()
        camera.disconnect()
        print("Camera disconnected.")


if __name__ == "__main__":
    main()
