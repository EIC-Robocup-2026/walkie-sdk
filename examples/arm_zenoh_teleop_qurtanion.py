import zenoh
import time
import json
import sys
from walkie_sdk.robot import WalkieRobot

# Global robot instance
robot = None

def listener(sample):
    """
    Callback triggered when a Zenoh message arrives on 'arm_pose'.
    """
    global robot
    
    try:
        # 1. Decode and Parse
        payload = sample.payload.to_string()
        data = json.loads(payload)
        
        print(f"\n[Zenoh] Received: {payload}")

        # 2. Extract Parameters for Pose and Quaternion
        group = data.get("group_name", "left_arm")
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        z = float(data.get("z", 0.0))
        
        # Extract Quaternion components
        qx = float(data.get("qx", -0.5))
        qy = float(data.get("qy", -0.5))
        qz = float(data.get("qz", 0.5))
        qw = float(data.get("qw", 0.5))
        
        # Additional parameters
        link_name = data.get("link_name", "left_link7")
        planning_time = float(data.get("allowed_planning_time", 10.0))
        blocking = bool(data.get("blocking", True))

        # 3. Validation: Ensure robot is ready
        if robot is None or not robot.is_connected:
            print("[Error] Robot is not connected, ignoring command.")
            return

        # 4. Execute Quaternion Command
        # This calls the direct MoveGroup action method added to your Arm class
        print(f" -> Executing QUATERNION Move: {group} to link {link_name}")
        
        status = robot.arm.go_to_pose_quaternion(
            x=x, y=y, z=z,
            qx=qx, qy=qy, qz=qz, qw=qw,
            group_name=group,
            link_name=link_name,
            allowed_planning_time=planning_time,
            blocking=blocking
        )
        
        print(f" -> Result: {status}")

    except json.JSONDecodeError:
        print(f"[Error] Invalid JSON format: {sample.payload.to_string()}")
    except Exception as e:
        print(f"[Error] Failed to process command: {e}")

def main():
    global robot

    # 1. Initialize Robot Connection
    print("Connecting to WalkieRobot...")
    robot = WalkieRobot(ip="127.0.0.1", ros_port=9090) 
    
    try:
        print("✓ Robot Connected and Arm Initialized")

        # 2. Initialize Zenoh
        print("Opening Zenoh session...")
        conf = zenoh.Config()
        session = zenoh.open(conf)

        # 3. Start Subscriber
        key_expr = 'arm_pose'
        print(f"Watching for updates on Zenoh key: '{key_expr}'...")
        sub = session.declare_subscriber(key_expr, listener)

        # 4. Keep Alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nCritical Error: {e}")
    finally:
        if 'session' in locals() and session:
            session.close()
        if robot:
            robot.disconnect()

if __name__ == "__main__":
    main()