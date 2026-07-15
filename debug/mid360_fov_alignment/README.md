# MID360 Crop and Planner FOV Alignment

## Purpose

This directory makes the geometric alignment check between the MID360 crop
bridge and the EPIC planner reproducible. It intentionally does not run a
vehicle, MARSIM, or ROS graph. The check compares the angular sets implied by
the active `mid360.yaml` configuration.

## Production Sources Under Test

The source manifest in `records/source_manifest.txt` records the exact source
paths, Git revision, and SHA-256 checksums used for this analysis.

The relevant implementations are:

- `src/ml_x_cropping/src/cloud_crop_bridge.cpp`
- `src/EPIC_poongsan/src/EPIC/src/local_planner/minco_planner/src/planner_manager.cpp`
- `src/EPIC_poongsan/src/EPIC/src/global_planner/frontier_manager/src/frontier_manager.cpp`
- `src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/mid360.yaml`

## Geometry Used by the Check

The crop bridge receives a cloud in the odometry/world frame and computes:

```text
p_body = R_world_to_body * (p_world - odom_position)
azimuth = atan2(p_body.y, p_body.x) - yaw_offset
elevation = atan2(p_body.z, sqrt(p_body.x^2 + p_body.y^2))
```

It keeps a point when the azimuth and elevation are inside `cloud_crop/*`.

PR #1 creates vertical observed-cone faces with:

```text
lower_corridor_edge = lidar_pitch + fov_down
upper_corridor_edge = lidar_pitch + fov_up
```

The viewpoint visibility test uses the same mount pitch but its own
`fov_viewpoint_down/up` interval.

## Acceptance Criteria

For strict body-frame equivalence:

```text
cloud_crop/fov_down = lidar_pitch + lidar_perception/fov_down
cloud_crop/fov_up   = lidar_pitch + lidar_perception/fov_up
cloud_crop/yaw_offset_deg - cloud_crop/fov_horizontal / 2
  = -lidar_perception/yaw_fov / 2
cloud_crop/yaw_offset_deg + cloud_crop/fov_horizontal / 2
  =  lidar_perception/yaw_fov / 2
```

The current checker reports whether the horizontal and vertical intervals are
identical. A mismatch is evidence that the cropped input and PR #1 corridor
cone describe different body-frame regions. It is not, by itself, proof that
the flight configuration is unsafe; the physical cloud and odometry frame
relationship must also be confirmed.

## Run

```bash
cd /workspace/EPIC-stack-onboard-pr1
python3 debug/mid360_fov_alignment/src/verify_fov_alignment.py \
  src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/mid360.yaml
```

Use `--strict` when a mismatch must fail a CI or release check:

```bash
python3 debug/mid360_fov_alignment/src/verify_fov_alignment.py \
  src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/mid360.yaml \
  --strict
```

## Current Static Finding

The current configuration has matching horizontal bounds but mismatched
vertical bounds:

```text
bridge crop:       0.0 to 37.5 deg
PR #1 cone:       -15.0 to 15.0 deg
viewpoint model:  -15.0 to 15.0 deg
```

The overlap is `0.0 to 15.0 deg`. The report under `records/` is generated
from the current configuration and must be regenerated whenever the relevant
YAML or geometry code changes.

## Next Validation Phase

The static result must be followed by a synthetic cloud test before a vehicle
test. Feed identity odometry and point samples at known body elevations into
the crop bridge, then compare published point membership with the planner cone
membership reported by this checker. Finally, use a recorded MID360 bag to
verify that `cloud_registered` is actually expressed in the same world frame
as `/Odometry`.
