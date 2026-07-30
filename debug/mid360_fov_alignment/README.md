# MID360 Crop and Planner FOV Alignment

## Purpose

This directory makes the geometric alignment check between the MID360 crop
bridge and the EPIC planner reproducible. It intentionally does not run a
vehicle, MARSIM, or ROS graph. The check compares the angular sets implied by
the active config yaml (e.g. `mid360.yaml`, `mid360_mlx.yaml`).

## Production Sources Under Test

The source manifest in `records/source_manifest.txt` records the exact source
paths, Git revision, and SHA-256 checksums used for the 2026-07-18 analysis
recorded in that directory (see the "Current Static Finding" section below
for how to reproduce an up-to-date result).

The relevant implementations are:

- `src/ml_x_cropping/src/cloud_crop_bridge.cpp`
- `src/EPIC_poongsan/src/EPIC/src/local_planner/minco_planner/src/planner_manager.cpp`
- `src/EPIC_poongsan/src/EPIC/src/global_planner/frontier_manager/src/frontier_manager.cpp`
- `src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/mid360.yaml`

## Geometry Used by the Check

The crop bridge receives a cloud in the odometry/world frame and, once per
callback, builds a single rotation matrix that folds in both the mount-pitch
correction and the yaw offset:

```text
R_crop = Rz(-yaw_offset) * Ry(+odom_mount_pitch_deg) * R_ws^T
p_body = R_crop * (p_world - odom_position)
```

There is no per-point `atan2` anymore. Both FOV membership tests are
algebraic inequalities on `p_body`, exactly equivalent to the old
`atan2`-based azimuth/elevation form but without the trig call:

```text
rho = sqrt(p_body.x^2 + p_body.y^2)          # already computed for the range test
|azimuth|   <= half_h    <=>  p_body.x >= cos(half_h) * rho             (rho > 0)
fov_down <= elevation <= fov_up
                          <=>  tan(fov_down)*rho <= p_body.z <= tan(fov_up)*rho
```

It keeps a point when both the horizontal and vertical inequalities hold and
the range is inside `[cloud_crop/min_range, cloud_crop/max_range]`.
`cloud_crop/odom_mount_pitch_deg` is entirely absorbed into `R_crop` above; it
does not appear anywhere in the two inequalities and costs nothing extra per
point (see `ws/epic/src/ml_x_cropping/README.md`, Frame Convention /
Performance Characteristics sections, for the full derivation).

EPIC's planner (`minco_planner`) creates vertical observed-cone faces with:

```text
bU = (lidar_perception/fov_up   - lidar_perception/lidar_pitch) * pi/180   # up-edge
bD = (lidar_perception/fov_down - lidar_perception/lidar_pitch) * pi/180   # down-edge
```

(`planner_manager.cpp:1022,1025`). `lidar_perception/fov_up`/`fov_down` are
sensor-frame (boresight-relative); subtracting `lidar_pitch` converts them to
the body frame the cone faces are built in.

The viewpoint visibility test in `frontier_manager.cpp` uses the same sign
convention, just reached differently: it rotates each candidate world point
into the sensor frame with a `Ry(-lidar_pitch)` step and then compares
directly against `fov_viewpoint_up`/`fov_viewpoint_down`
(`frontier_manager.cpp:1495,1508`). Working through that rotation shows it is
algebraically equivalent to a body-frame bound of
`fov_viewpoint_{up,down} - lidar_pitch` -- the same minus sign as the
corridor cone above, not a plus.

## Acceptance Criteria

For strict body-frame equivalence:

```text
cloud_crop/fov_down       = lidar_perception/fov_down - lidar_perception/lidar_pitch
cloud_crop/fov_up         = lidar_perception/fov_up   - lidar_perception/lidar_pitch
cloud_crop/fov_horizontal = lidar_perception/yaw_fov
cloud_crop/yaw_offset_deg - cloud_crop/fov_horizontal / 2
  = -lidar_perception/yaw_fov / 2
cloud_crop/yaw_offset_deg + cloud_crop/fov_horizontal / 2
  =  lidar_perception/yaw_fov / 2
```

Evidence for the sign: `planner_manager.cpp:1022,1025` (`bU`/`bD` above). An
earlier version of both this checker and (transiently) the deployed config
used `lidar_pitch + fov_*` -- the opposite sign -- which produced false FAILs
on a genuinely-aligned profile and a false PASS is not possible with that
bug (it can only disagree with the planner in the direction of reporting a
mismatch that is not real, or missing one that is). The corrected formula
above matches the production code exactly, including the crop bridge's own
runtime self-check (`checkFovAlignment()` in `cloud_crop_bridge.cpp`, gated by
`cloud_crop/strict_fov_alignment`).

The current checker reports whether the horizontal and vertical intervals are
identical (within a small floating-point tolerance). A mismatch is evidence
that the cropped input and the planner's observed-FOV corridor cone describe
different body-frame regions. It is not, by itself, proof that the flight
configuration is unsafe; the physical cloud and odometry frame relationship
must also be confirmed.

## Run

From the repository root (`epic-stack-docker`):

```bash
python3 ws/epic/debug/mid360_fov_alignment/src/verify_fov_alignment.py \
  ws/epic/src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/mid360_mlx.yaml
```

Use `--strict` when a mismatch must fail a CI or release check:

```bash
python3 ws/epic/debug/mid360_fov_alignment/src/verify_fov_alignment.py \
  ws/epic/src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/mid360_mlx.yaml \
  --strict
```

The tool takes any flat ROS parameter yaml with the `cloud_crop/*` and
`lidar_perception/*` keys it recognises; run it against `mid360.yaml`,
`mid360_mlx.yaml`, or `real.yaml` as needed. Missing keys are reported as
"CHECK NOT POSSIBLE" for that specific check rather than crashing the tool
(the `cloud_crop/*` schema is still evolving -- e.g.
`cloud_crop/odom_mount_pitch_deg` may not exist in every profile yet).

## Current Static Finding

Re-run on 2026-07-30 with the corrected (`fov_* - lidar_pitch`) sign:

`mid360_mlx.yaml` (`cloud_crop/enable: true` -- this is the profile that
actually runs the crop bridge):

```text
crop vertical:            [-17.500, 17.500] deg
planner corridor cone:    [-17.500, 17.500] deg  (body frame; planner_manager.cpp:1022,1025)
vertical alignment:       PASS
horizontal alignment:     PASS
```

`mid360.yaml` (at the time of this run, `cloud_crop/fov_up: 12.5`,
`cloud_crop/fov_down: -20.0`, `cloud_crop/enable: false`):

```text
crop vertical:            [-20.000, 12.500] deg
planner corridor cone:    [-20.000, 12.500] deg  (body frame; planner_manager.cpp:1022,1025)
vertical alignment:       PASS
horizontal alignment:     PASS
```

`mid360.yaml`'s `cloud_crop/*` values were mid-migration to the new
body-frame schema in the same work session as this fix; earlier in that
migration this file read `cloud_crop/fov_up: 37.5`, `fov_down: 0.0` (the old,
pre-body-frame convention) and this checker correctly reported vertical
alignment as FAIL against those values -- expected, since `cloud_crop/enable`
is `false` there and the bridge never runs against that profile regardless.
Because both configs are edited independently of this tool, re-run the
commands in [Run](#run) against the live yaml rather than trusting the
numbers above verbatim.

## Next Validation Phase

The static result must be followed by a synthetic cloud test before a vehicle
test. Feed identity odometry and point samples at known body elevations into
the crop bridge, then compare published point membership with the planner
cone membership reported by this checker. Finally, use a recorded MID360 bag
to verify that `cloud_registered` is actually expressed in the same world
frame as `/Odometry`. See
`ws/epic/src/ml_x_cropping/README.md` Section 10 ("Current Verification
Procedure") for the bag-replay-based equivalence regression, the required
`deny:=/cloud_registered_cropped` flag, and the build/profile checks that
complement this static tool.
