import json,sys,time

from walkie_sdk.robot import WalkieRobot

def get_input(prompt, default=None):
    """Helper to get input with a default value."""
    if default is not None:
        user_input = input(f"{prompt} (default: {default}): ")
    else:
        user_input = input(f"{prompt}: ")
    
    if not user_input and default is not None:
        return default
    return user_input

# Configuration - Change this to your robot's IP
ROBOT_IP = "127.0.0.1"
NAMESPACE = ""  # Optional: "robot1" for namespaced topics

# Protocol selection
ROS_PROTOCOL = "rosbridge"
CAMERA_PROTOCOL = "zenoh"

def main():

    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol=ROS_PROTOCOL,
            ros_port=9090,
            camera_protocol=CAMERA_PROTOCOL,
            camera_port=7447,
            timeout=10.0,
            namespace=NAMESPACE,
        )
    except ConnectionError as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)
    
    print("Format: Enter values separated by spaces.")
    print("Press 'q' or Ctrl+C at any time to quit.\n")

    # Default values to make typing easier
    defaults = {
        "group": "left_arm",
        "j1_j7": "0.0 0.0 0.0 0.0 0.0 0.0 0.0",
    }



    try:
        while True:
            print("-" * 30)
            
            

            # --- get input from keyboard ---
            input = get_input("Enter joint1-joint7", defaults["j1_j7"])
            if input.lower() == 'q': break
            defaults["j1_j7"] = input # Update default for next time
            
            try:
                j1, j2, j3, j4, j5, j6, j7 = map(float, input.split())
            except ValueError:
                print("Error: Please enter 7 numbers separated by space (e.g. 0.0 0.0 0.0 0.0 0.0 0.0 0.0)")
                continue

            # --- send command to robot ---
            print(f" >> Sent: {input}")

    except KeyboardInterrupt:
        print("\nStopping publisher...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        pass

if __name__ == "__main__":
    main()