from walkie_sdk import WalkieRobot

# Initialize Robot (Zenoh Protocol)
bot = WalkieRobot(
    ip="192.168.1.10", 
    ros_protocol="rosbridge",
    camera_protocol="zenoh"
)

# Use the new Tools module
coords_2d = [[320, 240, 50, 50]]
result_3d = bot.tools.send_3d_coords(coords_2d)

print(f"3D Result: {result_3d}")

bot.disconnect()