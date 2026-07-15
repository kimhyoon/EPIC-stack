#!/usr/bin/env python3
"""Static geometric check for MID360 crop and EPIC planner FOV alignment.

The crop bridge tests points in the body frame. PR #1 builds the vertical
corridor cone with lidar_pitch + fov_down/up. This tool reads the flat ROS
parameter YAML and compares those two angular intervals without ROS runtime.
"""

import argparse
import math
import re
import sys
from pathlib import Path


REQUIRED_KEYS = (
    "cloud_crop/fov_horizontal",
    "cloud_crop/fov_up",
    "cloud_crop/fov_down",
    "cloud_crop/yaw_offset_deg",
    "lidar_perception/fov_up",
    "lidar_perception/fov_down",
    "lidar_perception/lidar_pitch",
    "lidar_perception/fov_viewpoint_up",
    "lidar_perception/fov_viewpoint_down",
    "lidar_perception/yaw_fov",
)


def read_flat_ros_yaml(path: Path):
    """Read the scalar ROS parameter keys required by this checker."""
    values = {}
    pattern = re.compile(r"^\s*([^:#][^:]*):\s*([^#]*?)(?:\s+#.*)?\s*$")
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(raw_line)
        if not match:
            continue
        key = match.group(1).strip()
        value = match.group(2).strip().strip('"').strip("'")
        if key in REQUIRED_KEYS:
            try:
                values[key] = float(value)
            except ValueError as exc:
                raise ValueError(f"{key} is not a numeric scalar: {value}") from exc
    missing = [key for key in REQUIRED_KEYS if key not in values]
    if missing:
        raise KeyError("missing required keys: " + ", ".join(missing))
    return values


def interval_difference(left, right):
    """Return portions of left that are not covered by right; no wrap assumed."""
    lo, hi = left
    other_lo, other_hi = right
    result = []
    if lo < min(hi, other_lo):
        result.append((lo, min(hi, other_lo)))
    if hi > max(lo, other_hi):
        result.append((max(lo, other_hi), hi))
    return [(a, b) for a, b in result if b - a > 1e-9]


def contains(interval, value):
    return interval[0] - 1e-9 <= value <= interval[1] + 1e-9


def fmt_interval(interval):
    return f"[{interval[0]:.3f}, {interval[1]:.3f}] deg"


def fmt_pieces(pieces):
    return "none" if not pieces else ", ".join(fmt_interval(piece) for piece in pieces)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path, help="Path to mid360.yaml")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit non-zero when crop and corridor vertical intervals differ.",
    )
    args = parser.parse_args()

    cfg = read_flat_ros_yaml(args.config)
    crop_yaw = (
        cfg["cloud_crop/yaw_offset_deg"] - 0.5 * cfg["cloud_crop/fov_horizontal"],
        cfg["cloud_crop/yaw_offset_deg"] + 0.5 * cfg["cloud_crop/fov_horizontal"],
    )
    planner_yaw = (
        -0.5 * cfg["lidar_perception/yaw_fov"],
        0.5 * cfg["lidar_perception/yaw_fov"],
    )
    crop_vertical = (cfg["cloud_crop/fov_down"], cfg["cloud_crop/fov_up"])
    corridor_vertical = (
        cfg["lidar_perception/lidar_pitch"] + cfg["lidar_perception/fov_down"],
        cfg["lidar_perception/lidar_pitch"] + cfg["lidar_perception/fov_up"],
    )
    viewpoint_vertical = (
        cfg["lidar_perception/lidar_pitch"]
        + cfg["lidar_perception/fov_viewpoint_down"],
        cfg["lidar_perception/lidar_pitch"]
        + cfg["lidar_perception/fov_viewpoint_up"],
    )

    horizontal_match = crop_yaw == planner_yaw
    vertical_match = crop_vertical == corridor_vertical
    crop_only = interval_difference(crop_vertical, corridor_vertical)
    corridor_only = interval_difference(corridor_vertical, crop_vertical)

    print("MID360 crop / planner FOV static alignment report")
    print(f"config: {args.config}")
    print()
    print(f"crop horizontal:      {fmt_interval(crop_yaw)}")
    print(f"planner horizontal:   {fmt_interval(planner_yaw)}")
    print(f"horizontal alignment: {'PASS' if horizontal_match else 'FAIL'}")
    print()
    print(f"crop vertical:        {fmt_interval(crop_vertical)}")
    print(f"PR #1 corridor cone:  {fmt_interval(corridor_vertical)}")
    print(f"viewpoint FOV:        {fmt_interval(viewpoint_vertical)}")
    print(f"vertical alignment:   {'PASS' if vertical_match else 'FAIL'}")
    print(f"crop-only interval:   {fmt_pieces(crop_only)}")
    print(f"corridor-only range:  {fmt_pieces(corridor_only)}")
    print()
    print("elevation_deg,crop_keeps,pr1_corridor_keeps,viewpoint_keeps")
    samples = sorted(
        {
            crop_vertical[0] - 5.0,
            crop_vertical[0],
            corridor_vertical[0],
            -10.0,
            0.0,
            10.0,
            corridor_vertical[1],
            crop_vertical[1],
            crop_vertical[1] + 5.0,
        }
    )
    for elevation in samples:
        print(
            f"{elevation:.3f},{int(contains(crop_vertical, elevation))},"
            f"{int(contains(corridor_vertical, elevation))},"
            f"{int(contains(viewpoint_vertical, elevation))}"
        )

    if args.strict and (not horizontal_match or not vertical_match):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
