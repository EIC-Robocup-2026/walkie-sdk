# Visualization

RViz2 marker publishing for debugging and visualization. Access via `bot.viz`.

## Marker Type Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `ARROW` | 0 | Arrow marker |
| `CUBE` | 1 | Cube marker |
| `SPHERE` | 2 | Sphere marker |
| `CYLINDER` | 3 | Cylinder marker |
| `LINE_STRIP` | 4 | Connected line segments |
| `LINE_LIST` | 5 | Pairs of line segments |
| `CUBE_LIST` | 6 | List of cubes |
| `SPHERE_LIST` | 7 | List of spheres |
| `POINTS` | 8 | Point cloud |
| `TEXT_VIEW_FACING` | 9 | Billboard text |
| `MESH_RESOURCE` | 10 | Mesh file |
| `TRIANGLE_LIST` | 11 | Triangle mesh |

Import these from the top-level package:

```python
from walkie_sdk import SPHERE, ARROW, CUBE
```

## API Reference

::: walkie_sdk.modules.visualization.Visualization
    options:
      show_source: false
