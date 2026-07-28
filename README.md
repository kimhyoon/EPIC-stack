# Poongsan EPIC Stack

## Build Targets

Use one of the following build paths depending on the target machine.

### A. Real ML-X Onboard Deployment

Use this when the aircraft already has the real ML-X sensor input and does not
need MID360-to-ML-X cloud cropping.

Only clone the onboard planner tree:

```bash
 git clone --filter=blob:none --sparse --branch donghyuck \
    https://github.com/kimhyoon/EPIC-stack.git EPIC-stack-poongsan-onboard

  cd EPIC-stack-poongsan-onboard

  git sparse-checkout set \
    src/EPIC_poongsan \
    src/reactive_local_avoidance

```

Build with the crop bridge disabled:

```bash
cd EPIC-stack-poongsan-onboard
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DROS_EDITION=ROS1 -DBUILD_CLOUD_CROP_BRIDGE=OFF
catkin build
source devel/setup.bash
```

This mode keeps `real.yaml` as the onboard flight configuration.

### B. MID360 Drone, ML-X Emulation

Use this when the lab aircraft uses MID360 but should emulate the ML-X field of
view before EPIC consumes the point cloud.

Clone only the packages needed for onboard EPIC plus MID360/LiDAR support and
the optional crop bridge:

```bash
git clone --filter=blob:none --branch=donghyuck --no-checkout \
  https://github.com/kimhyoon/EPIC-stack.git EPIC-stack-mid360-mlx

cd EPIC-stack-mid360-mlx
git sparse-checkout init --cone
git sparse-checkout set \
  execution \
  src/EPIC_poongsan \
  src/FAST_LIO \
  src/livox_ros_driver2 \
  src/reactive_local_avoidance \
  src/ml_x_cropping
```

Build with the crop bridge enabled:

```bash
cd EPIC-stack-mid360-mlx
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DROS_EDITION=ROS1 -DBUILD_CLOUD_CROP_BRIDGE=ON
catkin build
source devel/setup.bash
```

Apply the MID360-to-ML-X crop profile after the build:

```bash
./execution/5_epic_mid360.sh --mlx-crop
```

The wrapper selects the following settings as one consistent runtime profile:

```text
config_file=mid360_mlx.yaml
use_cloud_crop_bridge=true
cloud_crop/enable=true

/cloud_registered
  -> cloud_crop_bridge
  -> /cloud_registered_cropped
  -> EPIC
```

Running the wrapper without `--mlx-crop` selects raw MID360 mode instead:

```bash
./execution/5_epic_mid360.sh
```

```text
config_file=mid360.yaml
use_cloud_crop_bridge=false
cloud_crop/enable=false
EPIC input=/cloud_registered
```

In this mode, make sure the runtime launch and YAML agree as a set:

```text
BUILD_CLOUD_CROP_BRIDGE=ON
use_cloud_crop_bridge=true
EPIC cloud topic = cropped cloud topic
crop input topic = MID360/Fast-LIO cloud topic
```

`use_cloud_crop_bridge` is the runtime master switch. It defaults to `false`,
so omitting the argument leaves EPIC on the raw `cloud_topic`. The selected
profile must still provide the bridge input/output values when the switch is
enabled.

Do not set `use_cloud_crop_bridge=true` if the workspace was built with
`BUILD_CLOUD_CROP_BRIDGE=OFF`.

The wrapper above is equivalent to the following manual launch command:

```bash
roslaunch epic_planner real_flight.launch \
  config_file:=mid360_mlx.yaml \
  use_cloud_crop_bridge:=true \
  enable_avoidance:=true
```

`mid360_mlx.yaml` keeps the previously validated FAST-LIO topics `/Odometry` and
`/cloud_registered`. The bridge republishes the cropped input on
`/cloud_registered_cropped`; both the EPIC LIO interface and FSM select that
topic when both `cloud_crop/enable: true` and
`use_cloud_crop_bridge:=true` are selected.

`5_epic_mid360.sh` derives its workspace path from its own location, so it is
safe after a sparse clone under a different directory name. It intentionally
does not pass the obsolete `rviz:=...` launch argument. Start the inspection
RViz separately when needed:

```bash
roslaunch ml_x_cropping mid360_epic_rviz.launch
```

Run MID360 without ML-X cropping:

```bash
# MID360 raw mode: do not emulate ML-X cropping.
# Optional but recommended when switching from a previous ON build:
catkin clean --yes ml_x_cropping

catkin config --cmake-args \
  -DROS_EDITION=ROS1 \
  -DBUILD_CLOUD_CROP_BRIDGE=OFF
catkin build
source devel/setup.bash

# BUILD_CLOUD_CROP_BRIDGE=OFF requires this argument to remain false.
roslaunch epic_planner real_flight.launch \
  config_file:=mid360.yaml \
  use_cloud_crop_bridge:=false \
  enable_avoidance:=true
```

`BUILD_CLOUD_CROP_BRIDGE=OFF` removes the bridge executable at build time.
The `use_cloud_crop_bridge:=false` launch argument is the runtime guard: it
does not start the bridge and forces EPIC to subscribe to raw
`/cloud_registered`. Do not use `use_cloud_crop_bridge:=true` in this build
mode.

### C. MARSIM Garage Simulation

Use this when the machine runs the MARSIM garage simulator, RViz, and EPIC
together.

Clone the full simulation workspace:

```bash
git clone --branch donghyuck \
  https://github.com/kimhyoon/EPIC-stack.git EPIC-stack-marsim

cd EPIC-stack-marsim
```

Build with the crop bridge enabled:

```bash
cd EPIC-stack-marsim
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DROS_EDITION=ROS1 -DBUILD_CLOUD_CROP_BRIDGE=ON
catkin build
source devel/setup.bash
```

Run the garage simulation through `sim_bringup`:

```bash
roslaunch sim_bringup garage_sim.launch use_cloud_crop_bridge:=true
```

The validated container build completed successfully with the crop bridge
enabled.

Do not launch the internal MARSIM file directly for normal use:

```bash
roslaunch mars_drone_sim garage.launch
```

That file is included by `sim_bringup` and expects parent launch arguments such
as `sensing_horizon`.

## Validation Commands

### MID360 Real-Flight Profile

Inspect the launch graph before connecting a vehicle:

```bash
roslaunch --nodes epic_planner real_flight.launch \
  config_file:=mid360_mlx.yaml \
  use_cloud_crop_bridge:=true | \
  grep -E "cloud_crop_bridge|exploration_node|traj_server|px4_ctrl_bridge"
```

After the MID360 and FAST-LIO stack are publishing, confirm that EPIC and the
bridge agree on the cloud path:

```bash
rosparam get /exploration_node/cloud_topic
rosparam get /cloud_crop_bridge/cloud_topic
rosparam get /cloud_crop_bridge/cloud_crop/output_topic
rostopic info /cloud_registered_cropped
```

Expected configuration values are:

```text
/exploration_node/cloud_topic: /cloud_registered
/cloud_crop_bridge/cloud_topic: /cloud_registered
/cloud_crop_bridge/cloud_crop/output_topic: /cloud_registered_cropped
```

The planner reports the effective cropped subscription at startup as
`[LIOInterface] cloud crop enabled: EPIC uses /cloud_registered_cropped`.

For raw MID360 A/B debugging, select the raw profile and explicitly leave the
bridge disabled:

```bash
./execution/5_epic_mid360.sh
```

Check that the expected simulation nodes are present in the launch graph:

```bash
roslaunch --nodes sim_bringup garage_sim.launch use_cloud_crop_bridge:=true | \
  grep -E "cloud_crop_bridge|exploration_node|quad0_pcl_render_node|traj_server"
```

Expected key nodes:

```text
/exploration_node
/cloud_crop_bridge
/traj_server
/quad0_pcl_render_node
```

Dump and inspect the crop-related launch parameters:

```bash
roslaunch --dump-params sim_bringup garage_sim.launch use_cloud_crop_bridge:=true > /tmp/garage_params_dump.yaml
grep -nE "cloud_crop|cloud_topic|lidar_perception/(is_360lidar|yaw_fov|fov_up|fov_down|lidar_pitch)" /tmp/garage_params_dump.yaml
```

Expected important parameters:

```text
/cloud_crop_bridge/cloud_topic: /quad0_pcl_render_node/cloud
/cloud_crop_bridge/cloud_crop/output_topic: /quad0_pcl_render_node/cloud_cropped
/exploration_node/cloud_topic: /quad0_pcl_render_node/cloud_cropped
/exploration_node/lidar_perception/is_360lidar: false
/exploration_node/lidar_perception/yaw_fov: 120.0
```

## Overview

This branch contains the Poongsan garage simulation setup based on
`kimhyoon/EPIC-stack`. It tracks the PR #1 corridor clipping change and the
garage simulation wiring that makes EPIC consume an ML-X-style cropped LiDAR
cloud.

## Summary

This branch combines two related changes for Poongsan garage simulation:

1. **PR #1 observed FOV corridor clipping**

   The local SFC/FIRI corridor generation is clipped against the observed LiDAR
   FOV cone so the planner does not expand corridor polytopes as if unobserved
   space were known free space.

2. **Garage simulation ML-X cloud crop wiring**

   MARSIM still generates the full simulated LiDAR cloud, but EPIC consumes a
   cropped ML-X-style cloud through `cloud_crop_bridge`.

The resulting runtime structure is:

```text
MARSIM local_sensing
  /quad0_pcl_render_node/cloud              # full simulated LiDAR cloud
          |
          v
cloud_crop_bridge
  /quad0_pcl_render_node/cloud_cropped      # ML-X-style cropped cloud
          |
          v
EPIC exploration_node                       # frontier/viewpoint/corridor planning
```

MARSIM still generates the original full cloud. Only the cloud consumed by EPIC
is cropped, which matches the real-flight validation pattern more closely than
modifying the MARSIM renderer itself. PR #1 then uses the cropped/observed FOV
assumption when building the local corridor for MINCO/GCOPTER planning.

## Build and LiDAR Crop Modes

This branch keeps `src/EPIC_poongsan` aligned with the onboard `psc_stack` tree.
Runtime-only simulation pieces live outside that package:

- `src/MARSIM`: simulator packages and garage map runtime
- `src/sim_bringup`: MARSIM + EPIC launch wrappers
- `src/ml_x_cropping`: optional MID360-to-ML-X cloud crop bridge

Configuration policy:

- `real.yaml`: real onboard flight configuration based on the onboard reference,
  with the 2026-07-10 LIO-SAM + ML-X frame/cloud fix applied. It uses the real
  ML-X cloud directly and must run with `use_cloud_crop_bridge:=false`.
- `mid360.yaml`: raw MID360 A/B debug profile. It keeps the bridge disabled and
  EPIC consumes `/cloud_registered` directly.
- `mid360_mlx.yaml`: MID360-to-ML-X emulation profile. It requires
  `BUILD_CLOUD_CROP_BRIDGE=ON` and `use_cloud_crop_bridge:=true`.
- `garage.yaml`: MARSIM-only configuration. It keeps MARSIM topics, map bounds,
  and simulation-specific dynamics; it is not a real-flight configuration.

The crop bridge is optional and is not built unless explicitly enabled:

```bash
catkin config --cmake-args -DBUILD_CLOUD_CROP_BRIDGE=ON
catkin build
```

For onboard ML-X use, leave the bridge off and use the real sensor topics from
`real.yaml`. For MID360 ML-X emulation, use `mid360_mlx.yaml` with the
real-flight launch command above. For MARSIM ML-X emulation, build the bridge
and launch:

```bash
roslaunch sim_bringup garage_sim.launch use_cloud_crop_bridge:=true
```

If `use_cloud_crop_bridge:=false`, EPIC consumes the raw MARSIM cloud.
If `use_cloud_crop_bridge:=true`, EPIC consumes `/quad0_pcl_render_node/cloud_cropped` while MARSIM still publishes the full cloud.
## Troubleshooting

### `gh: command not found`

Install GitHub CLI inside the container, or push using regular git HTTPS
credentials. Do not bake personal GitHub credentials into a public Docker image.

### RViz Or OpenGL Fails

This is usually a Docker/X11/NVIDIA runtime issue, not a code issue. Verify that
the container was started with GPU access, a valid `DISPLAY`, and X11 socket
mounts.

### `mars_drone_sim garage.launch` Missing Arguments

Use:

```bash
roslaunch sim_bringup garage_sim.launch use_cloud_crop_bridge:=true
```

The MARSIM launch is an internal include and is not the entry point for this
workspace.

## Git Notes

Before committing changes, check:

```bash
cd /workspace/EPIC-stack-onboard-pr1
git branch
git status
git remote -v
```

This branch should push to:

```text
origin/donghyuck
```

If GitHub CLI is installed in the container, authentication can be checked with:

```bash
gh auth status
```

## Upstream provenance

This branch vendors EPIC_poongsan from kimhyoon/psc_stack.

- Upstream repository: https://github.com/kimhyoon/psc_stack.git
- Upstream commit: efbd804ecba6642fe3ca26aec44688ff1ae01021
- Import policy: src/EPIC_poongsan keeps the onboard tree shape; simulation-only MARSIM/garage/RViz wiring must live outside EPIC_poongsan.

Source ownership in this workspace:

- `src/EPIC_poongsan`
  - Source repository: https://github.com/kimhyoon/psc_stack.git
  - Source commit: `efbd804ecba6642fe3ca26aec44688ff1ae01021`
  - Purpose: onboard planner tree. This is the only directory that should be
    treated as the onboard EPIC planner payload.
- `src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/real.yaml`
  - Source repository: https://github.com/kimhyoon/psc_stack.git
  - Source commit: `efbd804ecba6642fe3ca26aec44688ff1ae01021`
  - Base file: onboard reference `real.yaml` from the source commit above.
  - Local change: updated for the 2026-07-10 LIO-SAM + SOSLAB ML-X real-flight
    debug result. The flight config now uses `/mavros/odometry/in` for odometry,
    `/lio_sam/mapping/cloud_registered_raw` for the registered cloud, and
    `lidar_perception/cloud_frame_mode: world` so EPIC does not trust misleading
    PointCloud2 frame_id strings.
  - Reason: fixes the frame/cloud/yaw issues documented in
    `/home/dh/Downloads/20260713_FRAME_CLOUD_YAW_FIXES.md`.
- `src/ml_x_cropping`
  - Tracking repository: https://github.com/kimhyoon/EPIC-stack.git
  - Intended branch: `donghyuck`
  - Purpose: optional MID360-to-ML-X cloud crop bridge for lab and MARSIM
    validation. This package is not part of the onboard `psc_stack`
    `EPIC_poongsan` reference.
- `src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/mid360.yaml`
  - Purpose: raw MID360 A/B debug profile. It keeps the MID360 planner/FOV
    model but does not start the cloud crop bridge.
  - Baseline: the previously validated FAST-LIO MID360 profile, extended with
    the PR #1 observed-FOV corridor parameters.
  - Do not use this profile for a vehicle carrying a real ML-X sensor.
- `src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/mid360_mlx.yaml`
  - Purpose: MID360-to-ML-X emulation profile. It enables one crop bridge pass
    from `/cloud_registered` to `/cloud_registered_cropped` for EPIC.

## Branch And Source

- Upstream repository:

```text
https://github.com/kimhyoon/EPIC-stack.git
```

- Working branch:

```text
donghyuck
```

- Container workspace used during validation:

```text
/workspace/EPIC-stack-onboard-pr1
```

- Validation branch inside the container:

```text
donghyuck-onboard-pr1-port-20260714
```

## Clone Only EPIC_poongsan For Onboard Use

For real onboard deployment, the required planner payload is:

```text
src/EPIC_poongsan
```

Git cannot clone a single subdirectory directly. Use sparse checkout to fetch
only `src/EPIC_poongsan` from the `donghyuck` branch:

```bash
git clone --filter=blob:none --sparse --branch donghyuck \
  https://github.com/kimhyoon/EPIC-stack.git EPIC-stack-poongsan-onboard

cd EPIC-stack-poongsan-onboard
git sparse-checkout set src/EPIC_poongsan
```

The resulting checkout shape is:

```text
EPIC-stack-poongsan-onboard/
└── src/
    └── EPIC_poongsan/
```

Use this sparse checkout when the target machine only needs the onboard planner
tree. Do not copy `src/MARSIM`, `src/sim_bringup`, or `src/ml_x_cropping` to the
onboard payload unless that machine is also being used for simulation or lab
MID360-to-ML-X emulation.

## Version Tracking

This README tracks the exact functional changes expected in the `donghyuck`
branch.

### Applied Upstream PR

- PR:

```text
https://github.com/kimhyoon/EPIC-stack/pull/1
```

- PR title:

```text
local_planner: clip local SFC corridor to observed FOV cone (Method B)
```

- PR source branch:

```text
sanghun17:feat/clip-corridor-fov-cone
```

- Local tracking reference used during integration:

```text
pr-1
```

- PR integration commit in the validated port branch:

```text
fd91726 Port PR #1 observed FOV corridor clipping
```

### Local Garage Simulation Commit

After PR #1 was integrated, the garage simulation was additionally wired to use
the separated `ml_x_cropping` package for EPIC input cropping:

```text
5162342 Add separated MARSIM garage simulation bringup
```

That commit does not replace PR #1. It adds the simulation-side cloud path so
the PR #1 observed-FOV corridor behavior is exercised with cropped EPIC input.

## What Was Integrated

### 1. PR #1: Observed FOV Corridor Clipping

PR #1 was applied to this branch to prevent the local SFC/FIRI corridor from
expanding into regions outside the observed LiDAR FOV cone.

PR URL:

```text
https://github.com/kimhyoon/EPIC-stack/pull/1
```

PR title:

```text
local_planner: clip local SFC corridor to observed FOV cone (Method B)
```

Relevant commit in the validated port branch:

```text
fd91726 Port PR #1 observed FOV corridor clipping
```

Main files touched by PR #1:

```text
src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/garage.yaml
src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/src/fast_exploration_manager.cpp
src/EPIC_poongsan/src/EPIC/src/global_planner/frontier_manager/include/frontier_manager/frontier_manager.h
src/EPIC_poongsan/src/EPIC/src/global_planner/frontier_manager/src/frontier_manager.cpp
src/EPIC_poongsan/src/EPIC/src/local_planner/minco_planner/CMakeLists.txt
src/EPIC_poongsan/src/EPIC/src/local_planner/minco_planner/include/plan_manage/planner_manager.h
src/EPIC_poongsan/src/EPIC/src/local_planner/minco_planner/package.xml
src/EPIC_poongsan/src/EPIC/src/local_planner/minco_planner/src/planner_manager.cpp
```

The garage configuration enables the PR #1 corridor clipping behavior by
default:

```yaml
local_planning/clip_corridor_to_observed: true
local_planning/clip_cone_faces: true
```

Conceptually, PR #1 keeps the standard MINCO/GCOPTER optimization flow and
modifies the corridor generation side so local FIRI/SFC polytopes are clipped
against the observed FOV cone.

### 2. Poongsan Garage PCD Map

The MARSIM garage map is replaced with the processed Poongsan garage-style PCD:

```text
src/MARSIM/map_generator/resource/garage.pcd
```

This file is the map consumed by MARSIM when `sim_bringup garage_sim.launch` starts.
It has been prepared as a garage-style world map for simulation use.

Expected SHA256:

```text
873398fb0a9339edb0a506d20636e5e408024fa50cd79d2ae81a14584f6577fd
```

### 3. Garage Simulation ML-X Cloud Crop

After PR #1 integration, garage simulation was additionally wired so EPIC uses a
cropped cloud while MARSIM continues publishing the full cloud.

Commit:

```text
5162342 Add separated MARSIM garage simulation bringup
```

Modified files:

```text
src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/garage.yaml
src/sim_bringup/launch/garage_sim.launch
src/ml_x_cropping/*
```

Key configuration in `garage.yaml`:

```yaml
cloud_topic: /quad0_pcl_render_node/cloud

cloud_crop/enable: true
cloud_crop/output_topic: /quad0_pcl_render_node/cloud_cropped
cloud_crop/fov_horizontal: 120.0
cloud_crop/fov_up: 15.0
cloud_crop/fov_down: -15.0
cloud_crop/min_range: 0.0
cloud_crop/max_range: 30.0
cloud_crop/yaw_offset_deg: 0.0

lidar_perception/fov_up: 15.0
lidar_perception/fov_down: -15.0
lidar_perception/lidar_pitch: 0.0
lidar_perception/is_360lidar: false
lidar_perception/yaw_fov: 120.0
```

Key launch addition in `sim_bringup/launch/garage_sim.launch`:

```xml
<arg name="use_cloud_crop_bridge" default="false" />

<group if="$(arg use_cloud_crop_bridge)">
  <node pkg="ml_x_cropping" name="cloud_crop_bridge" type="cloud_crop_bridge" output="screen">
    <rosparam file="$(find epic_planner)/config/$(arg config_file)" command="load" />
  </node>
</group>
```

The intended behavior is:

- `/quad0_pcl_render_node/cloud` remains the raw MARSIM output.
- `cloud_crop_bridge` subscribes to the raw cloud.
- `cloud_crop_bridge` publishes `/quad0_pcl_render_node/cloud_cropped`.
- `exploration_node` uses the cropped topic when `cloud_crop/enable: true`.

## What Was Not Changed

- MARSIM `local_sensing` renderer was not modified for this crop behavior.
- The raw cloud topic `/quad0_pcl_render_node/cloud` remains available.
- The main trajectory optimizer is still the existing MINCO/GCOPTER path.
- Generated build outputs are not intended to be tracked:

```text
build/
devel/
logs/
.catkin_tools/
```
