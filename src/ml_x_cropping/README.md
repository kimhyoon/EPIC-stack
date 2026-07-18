# ML-X Cloud Crop Bridge: Migration and Verification Record

## 1. Purpose

`ml_x_cropping` adapts a wide-FOV MID360 point cloud to the effective forward
ML-X field of view before EPIC consumes the cloud.

The bridge is intentionally placed after FAST-LIO:

```text
/livox/lidar
    -> FAST-LIO localization and mapping
    -> /cloud_registered                 (raw registered world cloud)
    -> cloud_crop_bridge                 (only when enabled)
    -> /cloud_registered_cropped         (ML-X-emulation cloud)
    -> EPIC exploration_node
```

FAST-LIO always keeps the raw sensor input. The bridge changes only the cloud
used by EPIC frontier, viewpoint, map, and corridor planning.

## 2. Scope and Non-Scope

This package performs one geometric point-cloud filter. For every incoming
registered cloud, it transforms each point into the odometry body frame at the
matching scan timestamp and applies horizontal FOV, vertical FOV, range, and
yaw-offset tests.

It does not:

- change FAST-LIO localization;
- crop the cloud a second time inside EPIC;
- replace PR #1 observed-FOV corridor clipping.

PR #1 is a separate downstream safety mechanism. It constrains FIRI hPolys
using active observed-FOV cone faces after EPIC receives the cropped cloud.

## 3. Migration Traceability

The bridge was moved out of `epic_planner` to make it optional for real ML-X
aircraft deployments.

| Item | Previous location | Current location |
| --- | --- | --- |
| Bridge source | `src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/src/cloud_crop_bridge.cpp` | `src/ml_x_cropping/src/cloud_crop_bridge.cpp` |
| Build target | `epic_planner` CMake target | `ml_x_cropping` CMake target |
| Flight launch node | `pkg="epic_planner"` | `pkg="ml_x_cropping"` |
| Runtime dependency | implicit internal target | `epic_planner/package.xml` declares `exec_depend` on `ml_x_cropping` |

The operational crop algorithm was compared after the move. The source hashes
are different because comments and diagnostic strings were normalized, but the
world-to-body transform and the azimuth, elevation, and range predicates are
unchanged.

## 4. Runtime Wiring

For MID360 ML-X emulation, launch with:

```bash
roslaunch epic_planner real_flight.launch \
  config_file:=mid360.yaml \
  use_cloud_crop_bridge:=true \
  enable_avoidance:=true
```

The resulting topic relationship is:

```text
/cloud_registered
    -> cloud_crop_bridge input
    -> /cloud_registered_cropped
    -> LIOInterface subscription
    -> FastExplorationFSM subscription
```

`LIOInterface` and `FastExplorationFSM` select the same cropped topic when
`cloud_crop/enable` is enabled. They subscribe to it; they do not invoke the
bridge or filter it again.

## 5. Parameter Roles

The crop profile is defined by `mid360.yaml`.

```yaml
cloud_topic: /cloud_registered
cloud_crop/enable: true
cloud_crop/output_topic: /cloud_registered_cropped
cloud_crop/fov_horizontal: 120.0
cloud_crop/fov_up: 37.5
cloud_crop/fov_down: 0.0
cloud_crop/min_range: 0.0
cloud_crop/max_range: 30.0
cloud_crop/yaw_offset_deg: 0.0
```

These `cloud_crop/*` values define the point-cloud filtering operation.
They must not be confused with `lidar_perception/*`, which defines how EPIC
interprets sensor visibility for frontier/viewpoint logic and PR #1 corridor
clipping.

## 6. Build and Runtime Modes

`BUILD_CLOUD_CROP_BRIDGE` controls whether the executable is built. The launch
argument controls whether it is run.

| Operating mode | Build option | Launch argument | EPIC input |
| --- | --- | --- | --- |
| Real ML-X | `OFF` | `use_cloud_crop_bridge:=false` | actual raw ML-X/registered cloud |
| MID360 without emulation | `OFF` or `ON` | `use_cloud_crop_bridge:=false` | `/cloud_registered` |
| MID360 ML-X emulation | `ON` | `use_cloud_crop_bridge:=true` | `/cloud_registered_cropped` |

Build MID360 ML-X emulation mode:

```bash
source /opt/ros/noetic/setup.bash
catkin config --cmake-args \
  -DROS_EDITION=ROS1 \
  -DBUILD_CLOUD_CROP_BRIDGE=ON
catkin build
source devel/setup.bash
```

Build raw MID360 mode:

```bash
catkin clean --yes ml_x_cropping
catkin config --cmake-args \
  -DROS_EDITION=ROS1 \
  -DBUILD_CLOUD_CROP_BRIDGE=OFF
catkin build
source devel/setup.bash
```

`BUILD_CLOUD_CROP_BRIDGE=OFF` and
`use_cloud_crop_bridge:=true` is invalid: the launch would request an
executable that was not built.

## 7. Verification Evidence

The following checks were performed in the `donghyuck` workspace on 2026-07-18.

| Check | Result |
| --- | --- |
| `catkin build ml_x_cropping --no-status` with bridge `ON` | PASS |
| `catkin build epic_planner --no-status` after adding runtime dependency | PASS |
| `roslaunch --nodes ... use_cloud_crop_bridge:=true` | PASS: exactly one `/cloud_crop_bridge` node |
| `roslaunch --nodes ... use_cloud_crop_bridge:=false` | PASS: no crop bridge node |
| Launch parameter dump | PASS: bridge and exploration node agree on raw/cropped topics and FOV parameters |
| Synthetic point-cloud runtime test | PASS: 2 expected points retained from 6 input points |

The launch parameter dump for crop mode confirmed:

```text
/cloud_crop_bridge/cloud_topic: /cloud_registered
/cloud_crop_bridge/odometry_topic: /Odometry
/cloud_crop_bridge/cloud_crop/output_topic: /cloud_registered_cropped
/cloud_crop_bridge/cloud_crop/enable: true
/exploration_node/cloud_crop/enable: true
/exploration_node/cloud_crop/output_topic: /cloud_registered_cropped
```

## 8. Synthetic Point-Cloud Test

An isolated ROS master and test-only topics were used. The odometry pose was
identity and the crop configuration was H 120 deg, V [0, 37.5] deg, and
0 to 30 m range.

| Input point in body frame | Expected result | Reason |
| --- | --- | --- |
| `(5, 0, 0)` | keep | forward and level |
| `(5, 0, 1)` | keep | forward and inside upper elevation bound |
| `(5, 0, -1)` | reject | below lower elevation bound |
| `(0, 5, 0)` | reject | outside horizontal FOV |
| `(5, 0, 5)` | reject | above upper elevation bound |
| `(31, 0, 0)` | reject | beyond max range |

Observed result:

```text
PASS: bridge retained exactly 2/6 expected points
```

This confirms the relocated executable applies the intended filter once and
publishes a valid cropped cloud.

## 9. Real-Aircraft Verification

With a real MID360 stack running in crop mode, run:

```bash
rosnode info /cloud_crop_bridge
rostopic info /cloud_registered
rostopic info /cloud_registered_cropped
rosparam get /exploration_node/cloud_crop/enable
```

PASS criteria:

- `/cloud_registered` has one FAST-LIO publisher;
- `/cloud_registered_cropped` has one `cloud_crop_bridge` publisher;
- `cloud_crop_bridge` subscribes to raw cloud and odometry;
- `/exploration_node/cloud_crop/enable` is `true`;
- EPIC startup logs report use of `/cloud_registered_cropped`.

## 10. Limits and Safety Notes

- The synthetic test validates bridge geometry and topic wiring, not physical
  MID360 mounting orientation.
- Validate the sign and frame meaning of the real sensor pitch and yaw with a
  real-aircraft recording before flight.
- Do not run `garage_sim.launch` and `real_flight.launch` on the same ROS
  master; each declares a node named `cloud_crop_bridge` for its own mode.
- The runtime dependency on `ml_x_cropping` was added in commit `6f9cd47` so
  deployment tooling can resolve the launch-time package requirement.
