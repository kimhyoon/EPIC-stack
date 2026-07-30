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
registered cloud, it transforms each point into the body frame at the
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
  config_file:=mid360_mlx.yaml \
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

**All `cloud_crop/*` FOV parameters (`fov_horizontal`, `fov_up`, `fov_down`,
`yaw_offset_deg`) are defined directly in the BODY frame.** There is no manual
mounting-angle correction to bake into these values by hand; the only place
the physical LiDAR mount tilt is expressed is `cloud_crop/odom_mount_pitch_deg`
(see [Frame Convention](#6-frame-convention) below). Writing a pre-tilted
`fov_up`/`fov_down` into a profile yaml, the way older profiles did, is
retired practice and will fail the bridge's own startup self-check.

| Parameter | Role | Unit |
| --- | --- | --- |
| `cloud_topic` | Raw input cloud topic (world/odom frame) | ROS topic |
| `odometry_topic` | Synchronized odometry topic; supplies the world-to-sensor rotation/translation used to transform points | ROS topic |
| `cloud_crop/enable` | Master switch. `false` -> bridge logs and exits cleanly; EPIC subscribes to raw `cloud_topic` instead. Missing key is fatal (no fallback) | bool |
| `cloud_crop/output_topic` | Cropped cloud output topic | ROS topic |
| `cloud_crop/fov_horizontal` | Body-frame horizontal full angle, centered on `yaw_offset_deg` | deg |
| `cloud_crop/fov_up` | Body-frame upper elevation bound | deg |
| `cloud_crop/fov_down` | Body-frame lower elevation bound | deg |
| `cloud_crop/yaw_offset_deg` | Body-frame azimuth of the crop cone axis (`0` = body +x, CCW positive) | deg |
| `cloud_crop/odom_mount_pitch_deg` | Forward-down tilt used to convert the odometry topic's attitude (if it reports the sensor's tilted attitude) back to body attitude; folded once per callback into the crop rotation matrix. `0.0` means the odometry topic already reports body attitude | deg |
| `cloud_crop/min_range` / `cloud_crop/max_range` | Range window on the cropped points; `max_range <= 0` means unlimited | m |
| `cloud_crop/sync_policy` | `exact` or `approximate` cloud/odom timestamp matching (`message_filters`) | enum |
| `cloud_crop/sync_queue` | Synchronizer queue depth | messages |
| `cloud_crop/sync_max_interval_sec` | Maximum stamp gap accepted under `approximate` policy; unused under `exact` | s |
| `cloud_crop/cloud_queue` / `cloud_crop/odom_queue` / `cloud_crop/pub_queue` | Subscriber/publisher queue depths | messages |
| `cloud_crop/watchdog_period_sec` | Wall-clock staleness threshold: if no cropped output has been published for longer than this, a diagnostic explains whether the cloud stream, the odom stream, or the synchronizer pairing has stalled. `0` disables the watchdog | s |
| `cloud_crop/require_frame_match` | `true` -> a cloud/odom `frame_id` mismatch is FATAL (shuts the node down) instead of a warning | bool |
| `cloud_crop/strict_fov_alignment` | `true` -> the bridge refuses to start if its crop cone does not match EPIC's `lidar_perception/*`-derived corridor cone (see [Frame Convention](#6-frame-convention)) | bool |
| `cloud_crop/output_fields` | `full` (preserve every input PointCloud2 field, e.g. intensity) or `xyz` (repack to a tight 12-byte x/y/z-only point) | enum |
| `cloud_crop/profile_log` | Emit a periodic per-stage timing line (crop loop / publish / marker) alongside the point-count log | bool |
| `cloud_crop/visualization_topic` | FOV wireframe marker topic | ROS topic |
| `cloud_crop/visualization_rate_hz` | Marker publish rate cap; `0` disables the marker entirely | Hz |
| `cloud_crop/visualization_range` | Far-plane range drawn on the wireframe | m |
| `cloud_crop/visualization_azimuth_samples` / `cloud_crop/visualization_elevation_samples` | Wireframe tessellation density | samples |

`lidar_perception/*` (`fov_up`, `fov_down`, `lidar_pitch`, `yaw_fov`, ...) is a
**separate** parameter set consumed by EPIC's planner and frontier code, not
by this bridge's crop loop. The bridge only reads it once, at startup, to
self-check consistency with the crop cone above (see below). Do not confuse
the two families: `lidar_perception/*` is sensor-frame, `cloud_crop/*` is
body-frame.

## 6. Frame Convention

Everything the crop loop tests -- `fov_horizontal`, `fov_up`, `fov_down`,
`yaw_offset_deg` -- is body frame (REP-103 ENU body: +x forward, +y left, +z
up). The physical LiDAR mounting tilt is handled by exactly one parameter,
`cloud_crop/odom_mount_pitch_deg`, which the bridge folds into a single
rotation matrix computed once per callback:

```text
R_crop = Rz(-yaw_offset) * Ry(+odom_mount_pitch) * R_ws^T
p_crop = R_crop * (p_world - t)
```

This costs nothing extra per point -- it is the same matrix-vector product the
crop already needed to go from world to body frame. There is no per-point
trig for the mount correction and no manual "pre-tilt the FOV numbers by the
mount angle" step in the yaml anymore.

`lidar_perception/*`, by contrast, is **sensor-frame** (boresight-relative):
the LiDAR is physically mounted pitched down by `lidar_perception/lidar_pitch`
degrees, and `lidar_perception/fov_up` / `fov_down` are measured relative to
that tilted boresight, not relative to the body. EPIC's planner converts them
to body frame with a minus sign:

```text
body_edge = lidar_perception/fov_up_or_down - lidar_perception/lidar_pitch
```

(`src/EPIC_poongsan/src/EPIC/src/local_planner/minco_planner/src/planner_manager.cpp:1022,1025`,
the `bU`/`bD` observed-FOV cone edges.)

Because `cloud_crop/fov_up` / `fov_down` are already body-frame, they must
equal those same differences, and `cloud_crop/fov_horizontal` must equal
`lidar_perception/yaw_fov`. The bridge self-checks this at startup
(`checkFovAlignment()` in `cloud_crop_bridge.cpp`):

```text
cloud_crop/fov_up        == lidar_perception/fov_up   - lidar_perception/lidar_pitch
cloud_crop/fov_down      == lidar_perception/fov_down - lidar_perception/lidar_pitch
cloud_crop/fov_horizontal == lidar_perception/yaw_fov
```

If `lidar_perception/{fov_up,fov_down,yaw_fov}` are not visible on the node's
namespace, the check is skipped (logged, not fatal). If they are visible and
any of the three equalities is off by more than the check's tolerance:
`cloud_crop/strict_fov_alignment: true` makes the mismatch FATAL (the bridge
refuses to start); `false` only warns and starts anyway. The standalone,
ROS-free version of the same check lives in
`debug/mid360_fov_alignment/src/verify_fov_alignment.py` (see that tool's
directory for CI/offline use).

## 7. Performance Characteristics

Measured on x86 with `-O3 -DNDEBUG`, in a benchmark harness that ported the
crop predicate logic unchanged (before/after the atan2-removal rewrite
described below):

| Item | Before | After |
| --- | --- | --- |
| Crop loop, N=10k points | 365 µs | 92 µs (3.97x) |
| Visualization marker, per scan | 20.5 µs (16.6 build + 3.9 serialize), 20,324 B | 0 with 0 subscribers |
| Output buffer | 480 KB malloc/free per callback | scratch buffer reused across callbacks |

**Why it got faster:** the original crop tested membership with two
`atan2()` calls per point (azimuth and elevation), which profiled at ~68% of
the loop. Since the range test already computes
`rho = sqrt(x^2 + y^2) >= 0` (the horizontal component of the range norm),
both FOV membership tests reduce to algebra that is exactly equivalent to the
`atan2` form, at zero extra cost (`rho` is already paid for):

```text
|azimuth|   <= half_h    <=>  x >= cos(half_h) * rho          (rho > 0)
fov_down <= elevation <= fov_up
                          <=>  tan(fov_down)*rho <= z <= tan(fov_up)*rho
```

Two hazards this depends on, both called out at the top of
`cloud_crop_bridge.cpp` and in `CMakeLists.txt`:

- **Never add `-ffast-math` (or `-Ofast`, which implies it) to this target.**
  The non-finite-point rejection is a single range-window comparison that
  relies on IEEE semantics (every comparison against NaN is false).
  `-ffast-math` lets the compiler assume no NaN/Inf exists and deletes that
  filter, letting garbage points reach EPIC's map.
- **Never nodelet-ize this node or switch its publish call to the
  `shared_ptr` overload.** The output-buffer reuse (`scratch_`/`out_` are
  member variables refilled every callback, not reallocated) is safe only
  because `ros::Publisher::publish(const M&)` serializes synchronously on the
  callback thread with no `type_info`/intra-process hand-off. The
  `shared_ptr` overload defers and can hand the message to another thread
  while this node is already overwriting the same buffer for the next scan.

## 8. Build and Runtime Modes

`BUILD_CLOUD_CROP_BRIDGE` controls whether the executable is built. The launch
argument controls whether it is run.

| Operating mode | Build option | Launch argument | EPIC input |
| --- | --- | --- | --- |
| Real ML-X | `OFF` | `use_cloud_crop_bridge:=false` | actual raw ML-X/registered cloud |
| MID360 without emulation | `OFF` or `ON` | `use_cloud_crop_bridge:=false` | `/cloud_registered` |
| MID360 ML-X emulation | `ON` | `use_cloud_crop_bridge:=true` | `/cloud_registered_cropped` |

`CMakeLists.txt` now also defaults `CMAKE_BUILD_TYPE` to `Release` whenever it
is unset (`if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)`). This
is a defense against a bypassing build (a bare `cmake`, `catkin_make`, an IDE)
silently falling back to GCC's own default of no optimization at all, which
would turn the loop above back into the "before" column above and directly
into EPIC replan latency. `modules/planner/epic/build_ws.sh` (the supported
entry point, see [Section 10](#10-current-verification-procedure)) already
passes `-DCMAKE_BUILD_TYPE=Release` explicitly every run; this default is a
second line of defense for anyone who builds outside that script.

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

## 9. Verification History (old parameter schema -- 2026-07-18)

**The two subsections below are kept for historical traceability only. They
were recorded on 2026-07-18 against the OLD `cloud_crop/*` schema
(`fov_up: 37.5`, `fov_down: 0.0`, no `odom_mount_pitch_deg`, no
`sync_policy`/`watchdog_period_sec`/`strict_fov_alignment`/etc., and an
`atan2`-based crop loop with no rotation-matrix mount correction). They do
NOT describe the current schema or the current implementation and must not
be used to judge today's build. See
[Section 10, Current Verification Procedure](#10-current-verification-procedure)
for how to re-validate against the present code.**

### 9.1 Verification Evidence (2026-07-18, old schema)

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

### 9.2 Synthetic Point-Cloud Test (2026-07-18, old schema)

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

This confirmed the relocated executable applied the intended filter once and
published a valid cropped cloud, under the old schema and the old (atan2)
crop implementation.

## 10. Current Verification Procedure

Re-run these whenever `cloud_crop_bridge.cpp`, `CMakeLists.txt`, or the
`cloud_crop/*` schema changes.

- **Equivalence regression**: replay the same bag through the old and the new
  binary and diff `/cloud_registered_cropped`'s `width` per timestamp. The
  pass criterion is **zero difference** -- the algebraic rewrite in
  [Section 7](#7-performance-characteristics) must keep every point that the
  old `atan2` form kept and reject every point it rejected.
- **Replay command**:

  ```bash
  ./modules/planner/epic/run_replay.sh <bag> config_file:=mid360_mlx.yaml deny:=/cloud_registered_cropped
  ```

  `deny:=/cloud_registered_cropped` is **required**, not optional. The
  replay tool (`bag_replay.py`, `exploration_manager/scripts/bag_replay.py`)
  has an `ALLOW_HINTS` glob `/cloud_registered*` so that live sensor input
  gets replayed; that same glob also matches the recorded
  `/cloud_registered_cropped` topic, which is the bridge's own OUTPUT, not an
  EPIC input. Without `deny:=`, the bag's recorded crop output would be
  replayed onto the exact same topic name the live bridge is about to publish
  on, so the comparison would have two simultaneous publishers on
  `/cloud_registered_cropped` and be meaningless.
- **Build**: `./setup.sh build-ws epic-x86-gpu` -- do not hand-run
  `catkin build`. `modules/planner/epic/build_ws.sh` re-asserts
  `-DCMAKE_BUILD_TYPE=Release` and `-DBUILD_CLOUD_CROP_BRIDGE` every run; a
  hand-rolled build can silently drop either.
- **Profile check**: `perf record -p $(pgrep -f cloud_crop_bridge)` while the
  bridge is running against live/replayed data. Before the rewrite,
  `__ieee754_atan2` sits at roughly 60-70% of samples; after, it is gone
  entirely and `memcpy` (the raw point copy) tops the profile instead.
- **Static self-check**: `verify_fov_alignment.py <config> --strict` (see
  `debug/mid360_fov_alignment/`), which reproduces the bridge's own
  `checkFovAlignment()` arithmetic without a ROS runtime.

## 11. Real-Aircraft Verification

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

## 12. Limits and Safety Notes

- The synthetic test in [Section 9.2](#92-synthetic-point-cloud-test-2026-07-18-old-schema)
  validated bridge geometry and topic wiring under the old schema, not
  physical MID360 mounting orientation.
- Validate the sign and frame meaning of the real sensor pitch and yaw with a
  real-aircraft recording before flight.
- Do not run `garage_sim.launch` and `real_flight.launch` on the same ROS
  master; each declares a node named `cloud_crop_bridge` for its own mode.
- The runtime dependency on `ml_x_cropping` was added in commit `6f9cd47` so
  deployment tooling can resolve the launch-time package requirement.
