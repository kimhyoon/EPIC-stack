#!/usr/bin/env python3
"""Static geometric check for MID360 crop and EPIC planner FOV alignment.

Both the crop bridge (cloud_crop/*) and the EPIC planner's observed-FOV cone
test points in the BODY frame. lidar_perception/* values, however, are SENSOR
(boresight-relative) angles: the sensor is mounted pitched down by
lidar_perception/lidar_pitch, so a sensor-frame elevation bound converts to a
body-frame bound as (sensor_angle - lidar_pitch). This is what the production
planner does (see planner_manager.cpp:1022,1025) and, after working through
its world->sensor rotation, what the frontier/viewpoint visibility model does
too (frontier_manager.cpp:1495-1508). An earlier version of this checker used
"lidar_pitch + fov_*", the opposite sign, and reported false FAILs.

This tool reads the flat ROS parameter YAML and reproduces those two
body-frame intervals without a ROS runtime, then compares them against the
crop bridge's own cloud_crop/fov_up, cloud_crop/fov_down (which are already
body-frame per the current parameter convention: cloud_crop/* FOV values are
defined directly in the body frame, and the physical mounting pitch is instead
carried in cloud_crop/odom_mount_pitch_deg for the bridge's odom->body
rotation only -- it does not shift the crop cone's body-frame position, so it
plays no part in this alignment arithmetic).
"""

import argparse
import math
import re
import sys
from pathlib import Path


# Flat scalar ROS parameter keys this tool understands. Presence is NOT
# required globally -- each check below only needs its own subset and reports
# "CHECK NOT POSSIBLE" (not a crash) when a key it needs is missing, since the
# bridge's parameter schema is still evolving (e.g. odom_mount_pitch_deg is
# new and may not exist in every profile yet).
KNOWN_KEYS = (
    "cloud_crop/fov_horizontal",
    "cloud_crop/fov_up",
    "cloud_crop/fov_down",
    "cloud_crop/yaw_offset_deg",
    "cloud_crop/odom_mount_pitch_deg",
    "lidar_perception/fov_up",
    "lidar_perception/fov_down",
    "lidar_perception/lidar_pitch",
    "lidar_perception/fov_viewpoint_up",
    "lidar_perception/fov_viewpoint_down",
    "lidar_perception/yaw_fov",
)


def read_flat_ros_yaml(path: Path):
    """Read the scalar ROS parameter keys this tool recognises.

    Keys that are not present are simply absent from the returned dict --
    callers must check for what they need before using it. A key IS present
    but not parseable as a float is still a hard error (that is a genuine
    config mistake, not "the new schema hasn't landed yet").
    """
    values = {}
    pattern = re.compile(r"^\s*([^:#][^:]*):\s*([^#]*?)(?:\s+#.*)?\s*$")
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(raw_line)
        if not match:
            continue
        key = match.group(1).strip()
        value = match.group(2).strip().strip('"').strip("'")
        if key in KNOWN_KEYS:
            try:
                values[key] = float(value)
            except ValueError as exc:
                raise ValueError(f"{key} is not a numeric scalar: {value}") from exc
    return values


def missing_of(cfg, keys):
    """Return the subset of keys not present in cfg (empty list = all present)."""
    return [key for key in keys if key not in cfg]


def nearly_equal(a, b, eps=1e-6):
    return abs(a - b) <= eps


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
    parser.add_argument(
        "config", type=Path, help="Path to an EPIC mid360*.yaml profile (flat ROS param YAML)"
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit non-zero when crop and corridor vertical/horizontal intervals differ.",
    )
    args = parser.parse_args()

    cfg = read_flat_ros_yaml(args.config)

    print("MID360 crop / planner FOV static alignment report")
    print(f"config: {args.config}")
    print()

    # ---- horizontal alignment: crop yaw window vs. planner's observed-FOV
    # cone yaw window. Both are symmetric about body +x (planner:
    # planner_manager.cpp:1003,1016,1019 -- aL = psi+hy, aR = psi-hy,
    # hy = 0.5*yaw_fov_), so this also implicitly requires
    # cloud_crop/yaw_offset_deg == 0 and cloud_crop/fov_horizontal ==
    # lidar_perception/yaw_fov.
    horizontal_keys = (
        "cloud_crop/fov_horizontal",
        "cloud_crop/yaw_offset_deg",
        "lidar_perception/yaw_fov",
    )
    missing = missing_of(cfg, horizontal_keys)
    horizontal_match = None
    if missing:
        print(f"horizontal alignment: CHECK NOT POSSIBLE (missing keys: {', '.join(missing)})")
    else:
        crop_yaw = (
            cfg["cloud_crop/yaw_offset_deg"] - 0.5 * cfg["cloud_crop/fov_horizontal"],
            cfg["cloud_crop/yaw_offset_deg"] + 0.5 * cfg["cloud_crop/fov_horizontal"],
        )
        planner_yaw = (
            -0.5 * cfg["lidar_perception/yaw_fov"],
            0.5 * cfg["lidar_perception/yaw_fov"],
        )
        horizontal_match = nearly_equal(crop_yaw[0], planner_yaw[0]) and nearly_equal(
            crop_yaw[1], planner_yaw[1]
        )
        print(f"crop horizontal:      {fmt_interval(crop_yaw)}")
        print(f"planner horizontal:   {fmt_interval(planner_yaw)}")
        print(f"horizontal alignment: {'PASS' if horizontal_match else 'FAIL'}")
    print()

    # ---- vertical alignment: crop body-frame cone vs. planner's observed-FOV
    # corridor cone. Production sign convention (planner_manager.cpp:1022,1025):
    #   bU = (fov_up   - lidar_pitch) * pi/180   # up-edge elevation
    #   bD = (fov_down - lidar_pitch) * pi/180   # down-edge elevation
    # i.e. body-frame bound = lidar_perception/fov_* MINUS lidar_pitch, not
    # plus. cloud_crop/fov_up and cloud_crop/fov_down are already body-frame
    # under the current parameter convention, so they compare directly.
    vertical_keys = (
        "cloud_crop/fov_up",
        "cloud_crop/fov_down",
        "lidar_perception/fov_up",
        "lidar_perception/fov_down",
        "lidar_perception/lidar_pitch",
    )
    missing = missing_of(cfg, vertical_keys)
    vertical_match = None
    crop_vertical = corridor_vertical = None
    if missing:
        print(f"vertical alignment:   CHECK NOT POSSIBLE (missing keys: {', '.join(missing)})")
    else:
        crop_vertical = (cfg["cloud_crop/fov_down"], cfg["cloud_crop/fov_up"])
        corridor_vertical = (
            cfg["lidar_perception/fov_down"] - cfg["lidar_perception/lidar_pitch"],
            cfg["lidar_perception/fov_up"] - cfg["lidar_perception/lidar_pitch"],
        )
        vertical_match = nearly_equal(crop_vertical[0], corridor_vertical[0]) and nearly_equal(
            crop_vertical[1], corridor_vertical[1]
        )
        print(f"crop vertical:            {fmt_interval(crop_vertical)}")
        print(f"planner corridor cone:    {fmt_interval(corridor_vertical)}  (body frame; "
              f"planner_manager.cpp:1022,1025)")
        print(f"vertical alignment:       {'PASS' if vertical_match else 'FAIL'}")
        crop_only = interval_difference(crop_vertical, corridor_vertical)
        corridor_only = interval_difference(corridor_vertical, crop_vertical)
        print(f"crop-only interval:       {fmt_pieces(crop_only)}")
        print(f"corridor-only interval:   {fmt_pieces(corridor_only)}")
    print()

    # ---- viewpoint model (informational only -- not gated by --strict). The
    # visibility scorer transforms world points into the sensor frame with a
    # RotY(-lidar_pitch) step and then compares directly against
    # fov_viewpoint_up/down (frontier_manager.cpp:1495-1508), which is
    # algebraically equivalent to a body-frame bound of
    # fov_viewpoint_* - lidar_pitch (same sign as the corridor cone above; see
    # derivation in this file's module docstring).
    viewpoint_keys = (
        "lidar_perception/fov_viewpoint_up",
        "lidar_perception/fov_viewpoint_down",
        "lidar_perception/lidar_pitch",
    )
    missing = missing_of(cfg, viewpoint_keys)
    viewpoint_vertical = None
    if missing:
        print(f"viewpoint FOV (body):     CHECK NOT POSSIBLE (missing keys: {', '.join(missing)})")
    else:
        viewpoint_vertical = (
            cfg["lidar_perception/fov_viewpoint_down"] - cfg["lidar_perception/lidar_pitch"],
            cfg["lidar_perception/fov_viewpoint_up"] - cfg["lidar_perception/lidar_pitch"],
        )
        print(f"viewpoint FOV (body):     {fmt_interval(viewpoint_vertical)}  "
              f"(frontier_manager.cpp:1495,1508)")
    print()

    # ---- mount-pitch double-management guard. cloud_crop/odom_mount_pitch_deg
    # and lidar_perception/lidar_pitch are the same physical mounting angle
    # used for two different purposes (odom->body rotation vs. sensor->body
    # FOV conversion); they must be kept equal by hand. odom_mount_pitch_deg
    # == 0.0 legitimately means "odom already reports body attitude", so that
    # value is exempt from the mismatch warning.
    mount_keys = ("cloud_crop/odom_mount_pitch_deg", "lidar_perception/lidar_pitch")
    missing = missing_of(cfg, mount_keys)
    if missing:
        print(f"mount-pitch consistency:  CHECK NOT POSSIBLE (missing keys: {', '.join(missing)})")
    else:
        odom_mount_pitch = cfg["cloud_crop/odom_mount_pitch_deg"]
        lidar_pitch = cfg["lidar_perception/lidar_pitch"]
        if odom_mount_pitch != 0.0 and not nearly_equal(odom_mount_pitch, lidar_pitch):
            print(
                "mount-pitch consistency:  WARNING -- cloud_crop/odom_mount_pitch_deg "
                f"({odom_mount_pitch}) != lidar_perception/lidar_pitch ({lidar_pitch}) "
                "and is not 0 -- check for a mounting-angle double-management mistake."
            )
        else:
            print("mount-pitch consistency:  OK")
    print()

    if crop_vertical is not None and corridor_vertical is not None:
        print("elevation_deg,crop_keeps,corridor_keeps" + (
            ",viewpoint_keeps" if viewpoint_vertical is not None else ""
        ))
        sample_points = {
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
        for elevation in sorted(sample_points):
            row = (
                f"{elevation:.3f},{int(contains(crop_vertical, elevation))},"
                f"{int(contains(corridor_vertical, elevation))}"
            )
            if viewpoint_vertical is not None:
                row += f",{int(contains(viewpoint_vertical, elevation))}"
            print(row)
        print()
    else:
        print("per-elevation sample table skipped (crop/corridor vertical check not possible)")
        print()

    if args.strict and (horizontal_match is False or vertical_match is False):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
