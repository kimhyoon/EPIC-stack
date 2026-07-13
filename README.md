# Poongsan EPIC Stack

## Upstream provenance

This branch vendors EPIC_poongsan from kimhyoon/psc_stack.

- Upstream repository: https://github.com/kimhyoon/psc_stack.git
- Upstream commit: efbd804ecba6642fe3ca26aec44688ff1ae01021
- Import policy: src/EPIC_poongsan keeps the onboard tree shape; simulation-only MARSIM/garage/RViz wiring must live outside EPIC_poongsan.

# EPIC-stack Poongsan Garage Simulation

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
/workspace/EPIC-stack-donghyuck
```

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

- PR integration commits visible in this branch:

```text
00a0fc2 local_planner: clip local SFC corridor to observed FOV cone (Method B)
6026559 Merge PR #1 observed FOV corridor clipping
9ee31e8 Enable observed corridor clipping for Poongsan garage
```

### Local Garage Simulation Commit

After PR #1 was integrated, the garage simulation was additionally wired to use
`cloud_crop_bridge` for EPIC input cropping:

```text
91508fb garage sim: crop EPIC cloud input with cloud_crop_bridge
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

Relevant commits visible in this branch:

```text
00a0fc2 local_planner: clip local SFC corridor to observed FOV cone (Method B)
6026559 Merge PR #1 observed FOV corridor clipping
9ee31e8 Enable observed corridor clipping for Poongsan garage
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
src/EPIC_poongsan/src/EPIC/src/MARSIM/map_generator/resource/garage.pcd
```

This file is the map consumed by MARSIM when `epic_planner garage.launch` starts.
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
91508fb garage sim: crop EPIC cloud input with cloud_crop_bridge
```

Modified files:

```text
src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/garage.yaml
src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/launch/garage.launch
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

Key launch addition in `garage.launch`:

```xml
<node pkg="epic_planner" name="cloud_crop_bridge" type="cloud_crop_bridge" output="screen">
  <rosparam file="$(find epic_planner)/config/$(arg config_file)" command="load" />
</node>
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

## Build

Run this inside the ROS Noetic container:

```bash
cd /workspace/EPIC-stack-donghyuck
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DROS_EDITION=ROS1
catkin build
source devel/setup.bash
```

The validated container build result was:

```text
All 26 packages succeeded.
```

## Run Garage Simulation

Use the top-level EPIC launch:

```bash
cd /workspace/EPIC-stack-donghyuck
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch epic_planner garage.launch
```

Do not launch the internal MARSIM file directly for normal use:

```bash
roslaunch mars_drone_sim garage.launch
```

That file is included by the EPIC launch and expects parent launch arguments
such as `sensing_horizon`.

## Validation Commands

Check that the expected nodes are present in the launch graph:

```bash
roslaunch --nodes epic_planner garage.launch | grep -E "cloud_crop_bridge|exploration_node|quad0_pcl_render_node|traj_server"
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
roslaunch --dump-params epic_planner garage.launch > /tmp/garage_params_dump.yaml
grep -nE "cloud_crop|cloud_topic|lidar_perception/(is_360lidar|yaw_fov|fov_up|fov_down|lidar_pitch)" /tmp/garage_params_dump.yaml
```

Expected important parameters:

```text
/cloud_crop_bridge/cloud_topic: /quad0_pcl_render_node/cloud
/cloud_crop_bridge/cloud_crop/output_topic: /quad0_pcl_render_node/cloud_cropped
/exploration_node/cloud_crop/enable: true
/exploration_node/cloud_crop/output_topic: /quad0_pcl_render_node/cloud_cropped
/exploration_node/lidar_perception/is_360lidar: false
/exploration_node/lidar_perception/yaw_fov: 120.0
```

## Git Notes

Before committing changes, check:

```bash
cd /workspace/EPIC-stack-donghyuck
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
roslaunch epic_planner garage.launch
```

The MARSIM launch is an internal include and is not the entry point for this
workspace.

## Build and LiDAR Crop Modes

This branch keeps `src/EPIC_poongsan` aligned with the onboard `psc_stack` tree.
Runtime-only simulation pieces live outside that package:

- `src/MARSIM`: simulator packages and garage map runtime
- `src/sim_bringup`: MARSIM + EPIC launch wrappers
- `src/ml_x_cropping`: optional MID360-to-ML-X cloud crop bridge

Configuration policy:

- `real.yaml`: real onboard flight configuration. Keep this aligned with the onboard reference.
- `garage.yaml`: MID360/MARSIM garage simulation configuration.

The crop bridge is optional and is not built unless explicitly enabled:

```bash
catkin config --cmake-args -DBUILD_CLOUD_CROP_BRIDGE=ON
catkin build
```

For onboard ML-X use, leave the bridge off and use the real sensor topics from `real.yaml`.
For MID360/MARSIM ML-X emulation, build the bridge and launch simulation with:

```bash
roslaunch sim_bringup garage_sim.launch use_cloud_crop_bridge:=true
```

If `use_cloud_crop_bridge:=false`, EPIC consumes the raw MARSIM cloud.
If `use_cloud_crop_bridge:=true`, EPIC consumes `/quad0_pcl_render_node/cloud_cropped` while MARSIM still publishes the full cloud.
