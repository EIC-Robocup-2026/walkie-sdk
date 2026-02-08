from walkie_sdk import WalkieRobot

# Initialize Robot (Zenoh Protocol)
bot = WalkieRobot(
    ip="10.0.0.6", 
    ros_protocol="rosbridge",
    camera_protocol="zenoh"
)

# Use the new Tools module
coords_2d = [[320, 240, 50, 50],[220, 140, 50, 50]]
result_3d = bot.tools.send_3d_coords(coords_2d)

print(f"3D Result: {result_3d}")

bot.disconnect()