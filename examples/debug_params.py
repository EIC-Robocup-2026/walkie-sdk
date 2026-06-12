"""
Diagnose commander parameter get/set over the SDK transport.

Usage:
    python examples/debug_params.py [ROBOT_IP] [PARAM_NAME]

Prints the raw service response (or the full exception) for several service
name variants so we can see exactly what the rosbridge/zenoh transport accepts.
"""
import sys
import traceback

from walkie_sdk.robot import WalkieRobot

ROBOT_IP = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
PARAM = sys.argv[2] if len(sys.argv) > 2 else "gripper_speed"
NODE = "bimanual_commander"

bot = WalkieRobot(ip=ROBOT_IP, ros_protocol="rosbridge", ros_port=9090,
                  camera_protocol="zenoh", camera_port=7447, namespace="")
tr = bot.arm._transport

print(f"\n== list_parameters on /{NODE} ==")
for name in (f"/{NODE}/list_parameters", f"{NODE}/list_parameters"):
    try:
        r = tr.call_service(name, "rcl_interfaces/srv/ListParameters",
                            {"prefixes": [], "depth": 0}, timeout=5.0)
        print(f"  OK   {name}")
        print("       names:", r.get("result", {}).get("names"))
    except Exception as e:
        print(f"  FAIL {name}: {type(e).__name__}: {e}")

print(f"\n== get_parameters [{PARAM}] ==")
for name in (f"/{NODE}/get_parameters", f"{NODE}/get_parameters"):
    try:
        r = tr.call_service(name, "rcl_interfaces/srv/GetParameters",
                            {"names": [PARAM]}, timeout=5.0)
        print(f"  OK   {name}")
        print("       raw response:", r)
    except Exception:
        print(f"  FAIL {name}")
        traceback.print_exc()

print("\n== via SDK helper ==")
print("  get_param ->", bot.arm.get_param(PARAM))
