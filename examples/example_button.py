#!/usr/bin/env python3
"""
Walkie SDK - Button Example (Pi Pico USB HID)

Monitors the physical button connected to a Pi Pico (acting as a USB HID
keyboard). The Pico sends F1 on press and releases it on release.

Usage:
    python examples/example_button.py --ip 192.168.1.100

Requirements:
    - Pi Pico programmed with code.py (sends F1 via USB HID)
    - Button wired between GP21 and GND on the Pico
    - pynput installed (uv sync)
"""

import argparse
import time

from walkie_sdk import WalkieRobot

ROBOT_IP = "192.168.1.100"
BUTTON_KEY = "0x1008ff47"  # keysym from xev (XF86Launch7)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=ROBOT_IP)
    parser.add_argument("--key", default=BUTTON_KEY, help="Key the Pico sends (e.g. f1, f2)")
    args = parser.parse_args()

    print(f"Connecting to {args.ip} (button key: {args.key})...")
    bot = WalkieRobot(
        ip=args.ip,
        camera_protocol="none",
        button_key=args.key,
    )

    print("Listening for button. Press Ctrl+C to quit.\n")
    last_state = None

    try:
        while True:
            pressed = bot.button.is_pressed
            if pressed != last_state:
                if pressed:
                    print("Button PRESSED")
                else:
                    print("Button released")
                last_state = pressed
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        bot.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
