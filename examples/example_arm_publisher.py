import zenoh
import time
import json
import sys

def get_input(prompt, default=None):
    """Helper to get input with a default value."""
    if default is not None:
        user_input = input(f"{prompt} (default: {default}): ")
    else:
        user_input = input(f"{prompt}: ")
    
    if not user_input and default is not None:
        return default
    return user_input

def main():
    # 1. Initialize Zenoh
    print("Opening Zenoh session...")
    conf = zenoh.Config()
    session = zenoh.open(conf)

    # 2. Declare Publisher
    key_expr = 'arm_pose'
    pub = session.declare_publisher(key_expr)
    
    print(f"\n--- Manual Commander for '{key_expr}' ---")
    print("Format: Enter values separated by spaces.")
    print("Example: '0.4 0.0 0.5' for X Y Z")
    print("Press 'q' or Ctrl+C at any time to quit.\n")

    # Default values to make typing easier
    defaults = {
        "group": "left_arm",
        "xyz": "0.3 0.0 0.5",
        "rpy": "0.0 1.57 0.0"
    }

    try:
        while True:
            print("-" * 30)
            
            # --- Step 1: Get Mode (Relative or Absolute) ---
            mode_in = get_input("Relative Move? (y/n)", "n")
            if mode_in.lower() == 'q': break
            is_relative = mode_in.lower().startswith('y')

            # --- Step 2: Get Position (X Y Z) ---
            xyz_in = get_input("Enter X Y Z", defaults["xyz"])
            if xyz_in.lower() == 'q': break
            defaults["xyz"] = xyz_in # Update default for next time
            
            try:
                x, y, z = map(float, xyz_in.split())
            except ValueError:
                print("Error: Please enter 3 numbers separated by space (e.g. 0.5 0.0 0.5)")
                continue

            # --- Step 3: Get Orientation (Roll Pitch Yaw) ---
            # For relative moves, we often just want 0 rotation, so we keep previous default
            rpy_in = get_input("Enter Roll Pitch Yaw", defaults["rpy"])
            if rpy_in.lower() == 'q': break
            defaults["rpy"] = rpy_in
            
            try:
                r, p, w = map(float, rpy_in.split())
            except ValueError:
                print("Error: Please enter 3 numbers separated by space.")
                continue

            # --- Step 4: Construct Payload ---
            pose_data = {
                "group_name": defaults["group"],
                "x": x,
                "y": y,
                "z": z,
                "roll": r,
                "pitch": p,
                "yaw": w,
                "cartesian_path": not is_relative, # Usually False for pure relative checks, but depends on preference
                "blocking": True,
                "relative": is_relative
            }

            # --- Step 5: Publish ---
            payload = json.dumps(pose_data)
            pub.put(payload)
            print(f" >> Sent: {payload}")

    except KeyboardInterrupt:
        print("\nStopping publisher...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        session.close()

if __name__ == "__main__":
    main()