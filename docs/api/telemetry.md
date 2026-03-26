# Telemetry

Robot telemetry and status data. Access via `bot.status`.

## ZED point cloud

When the robot publishes a ZED `PointCloud2` message on the SDK's configured ZED
topic, `bot.status` also exposes point-cloud helpers:

- `get_point_cloud_info()` returns parsed structural metadata (width/height,
  field names, and a small sample).
- `get_full_point_cloud()` attempts to decode the payload into a list of
  `(x, y, z)` float tuples.

Notes:

- The SDK decodes the transport payload; with rosbridge-style JSON this is
  typically a base64-encoded `data` field.
- Point clouds can be large: `get_full_point_cloud()` may be expensive.

Example:

```python
pc_info = bot.status.get_point_cloud_info()
if pc_info:
    print(f"Resolution: {pc_info.get('width')}x{pc_info.get('height')}")

    full_cloud = bot.status.get_full_point_cloud()
    if full_cloud:
        print(f"Extracted: {len(full_cloud)} points")
        print(f"First 3: {full_cloud[:3]}")
        print(f"Last 3:  {full_cloud[-3:]}")
```

::: walkie_sdk.modules.telemetry.Telemetry
    options:
      show_source: false
