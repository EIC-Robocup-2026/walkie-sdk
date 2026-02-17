# Visualization

The SDK can publish visualization markers to RViz2 for debugging and display.
Access via `bot.viz`, or use the convenience methods directly on `bot`.

## Drawing Markers

### Single Marker

```python
from walkie_sdk import ARROW, SPHERE, CUBE

# Red arrow (default)
marker_id = bot.viz.draw_marker(
    position=[1.0, 2.0, 0.0],
    quaternion=[0.0, 0.0, 0.0, 1.0],
)

# Green sphere with custom scale
marker_id = bot.viz.draw_marker(
    position=[3.0, 0.0, 0.5],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    frame_id="map",
    marker_type=SPHERE,
    color=[0.0, 1.0, 0.0, 0.8],  # [r, g, b, a]
    scale=[0.2, 0.2, 0.2],
)
```

### Multiple Markers

```python
ids = bot.viz.draw_markers([
    {
        "position": [1.0, 0.0, 0.0],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "marker_type": ARROW,
        "color": [1.0, 0.0, 0.0, 1.0],
    },
    {
        "position": [2.0, 0.0, 0.0],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "marker_type": SPHERE,
        "color": [0.0, 1.0, 0.0, 1.0],
    },
])
```

## Updating Markers

Only pass the fields you want to change. Everything else is kept from the
original `draw_marker()` call:

```python
mid = bot.viz.draw_marker([0, 0, 0], [0, 0, 0, 1])

# Update only position
bot.viz.update_marker(mid, position=[1.0, 2.0, 0.0])

# Update position and color
bot.viz.update_marker(mid, position=[3.0, 0.0, 0.0], color=[0, 1, 0, 1])
```

## Deleting Markers

```python
# Delete one marker
bot.viz.delete_marker(marker_id)

# Clear all markers
bot.viz.clear_markers()
```

## Marker Types

Import marker type constants from the top-level package:

```python
from walkie_sdk import ARROW, CUBE, SPHERE, CYLINDER, TEXT_VIEW_FACING
```

| Constant | Value | Description |
|----------|-------|-------------|
| `ARROW` | 0 | Arrow pointing along +X |
| `CUBE` | 1 | Box |
| `SPHERE` | 2 | Sphere |
| `CYLINDER` | 3 | Cylinder |
| `LINE_STRIP` | 4 | Connected line segments |
| `LINE_LIST` | 5 | Independent line pairs |
| `CUBE_LIST` | 6 | Multiple cubes |
| `SPHERE_LIST` | 7 | Multiple spheres |
| `POINTS` | 8 | Point cloud |
| `TEXT_VIEW_FACING` | 9 | Billboard text |
| `MESH_RESOURCE` | 10 | Mesh from file |
| `TRIANGLE_LIST` | 11 | Triangle mesh |

## Publishing Poses

Publish a `geometry_msgs/PoseStamped` for display in RViz2:

```python
topic = bot.viz.draw_pose(
    position=[1.0, 2.0, 0.0],
    quaternion=[0.0, 0.0, 0.0, 1.0],
)

# Update it later (only changed fields)
bot.viz.update_pose(position=[2.0, 3.0, 0.0], topic=topic)
```

Use different topics for multiple simultaneous poses:

```python
bot.viz.draw_pose(
    position=[0.5, 0.0, 0.3],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    topic="walkie/target_pose/left_arm",
)
bot.viz.draw_pose(
    position=[-0.5, 0.0, 0.3],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    topic="walkie/target_pose/right_arm",
)
```

## Axis Triads

Draw RGB axis arrows (Red=X, Green=Y, Blue=Z) to visualize a coordinate frame:

```python
name = bot.viz.draw_axis(
    position=[1.0, 0.0, 0.5],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    axis_name="ee_target",
    scale=0.15,  # arrow length in meters
)

# Update the axis pose
bot.viz.update_axis("ee_target", position=[2.0, 0.0, 0.5])
```

## Convenience Methods

`WalkieRobot` exposes shorthand methods that delegate to `bot.viz`:

```python
# These are equivalent:
bot.draw_marker([1, 0, 0], [0, 0, 0, 1])
bot.viz.draw_marker([1, 0, 0], [0, 0, 0, 1])

bot.draw_pose([1, 0, 0], [0, 0, 0, 1])
bot.viz.draw_pose([1, 0, 0], [0, 0, 0, 1])

bot.draw_axis([1, 0, 0], [0, 0, 0, 1])
bot.viz.draw_axis([1, 0, 0], [0, 0, 0, 1])
```

## Default Topics

| Function | Default Topic |
|----------|--------------|
| `draw_marker` | `walkie/viz_markers` |
| `draw_markers` | `walkie/viz_markers_array` |
| `draw_pose` | `walkie/target_pose` |
| `draw_axis` | `walkie/viz_axis` |

All topics respect the robot's namespace setting.
