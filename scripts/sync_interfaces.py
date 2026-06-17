#!/usr/bin/env python3
"""Sync ROS interface (.srv/.msg) files into walkie_sdk/config/interfaces/.

The zenoh transport (``zenoh_ros2_sdk``) needs the *text* of message/service
definitions to (de)serialize them. It auto-clones the standard ROS repos, but
project-specific packages and a few otherwise-unclonable ones (e.g. ``vision_msgs``)
are not available that way. Rather than hand-transcribe definitions (which drift),
we bundle copies of the real files here so the client stays ROS-free.

Re-run this on a machine that has the walkie-ws workspace + ROS install whenever
an interface changes:

    python scripts/sync_interfaces.py

Then commit the updated ``walkie_sdk/config/interfaces/`` tree.
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

SDK_ROOT = Path(__file__).resolve().parent.parent
DEST = SDK_ROOT / "walkie_sdk" / "config" / "interfaces"
WS = Path.home() / "robocup2026" / "walkie-ws" / "src" / "walkie-ros"

# Source package directories. Each must contain a package.xml and srv/ and/or
# msg/ subdirs. The destination package name is read from package.xml <name>,
# so a dir like ``perception`` correctly lands under ``walkie_perception``.
SOURCES = [
    WS / "tools" / "walkie_tf_interfaces",
    WS / "perception",
    WS / "manipulate" / "my_robot_interfaces",
    WS / "navigation" / "walkie-navigation",
    # Standard-but-unclonable: vision_msgs (nested inside walkie_perception srvs).
    Path("/opt/ros/jazzy/share/vision_msgs"),
]


def package_name(pkg_dir: Path) -> str:
    """Return the ROS package name from package.xml (falls back to dir name)."""
    xml = pkg_dir / "package.xml"
    if xml.exists():
        m = re.search(r"<name>\s*([^<]+?)\s*</name>", xml.read_text())
        if m:
            return m.group(1)
    return pkg_dir.name


def main() -> int:
    if DEST.exists():
        shutil.rmtree(DEST)
    total = 0
    for src in SOURCES:
        if not src.exists():
            print(f"  ⚠ skip (missing): {src}")
            continue
        name = package_name(src)
        for kind in ("srv", "msg"):
            files = sorted((src / kind).glob(f"*.{kind}"))
            if not files:
                continue
            out = DEST / name / kind
            out.mkdir(parents=True, exist_ok=True)
            for f in files:
                shutil.copy2(f, out / f.name)
                total += 1
            print(f"  {name}/{kind}: {len(files)} file(s)")
    print(f"\nSynced {total} interface file(s) into {DEST.relative_to(SDK_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
