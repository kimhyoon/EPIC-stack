# Poongsan EPIC Stack - Donghyuck Branch

This branch is prepared for running the EPIC + MARSIM garage simulation with the Poongsan garage-style PCD map.

## What Is Included

- Upstream EPIC-stack source from `kimhyoon/EPIC-stack` branch `donghyuck`.
- PR #1: observed FOV cone clipping for local SFC/FIRI corridor generation.
- Poongsan processed garage map at:

```text
src/EPIC_poongsan/src/EPIC/src/MARSIM/map_generator/resource/garage.pcd
```

- PR #1 clipping enabled by default in:

```text
src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config/garage.yaml
```

Enabled parameters:

```yaml
local_planning/clip_corridor_to_observed: true
local_planning/clip_cone_faces: true
```

## What Is Not Included

Generated build outputs are intentionally not tracked:

```text
build/
devel/
logs/
.catkin_tools/
```

Build the workspace after cloning or after creating a container from this source.

## Build

```bash
cd /workspace/EPIC-stack
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DROS_EDITION=ROS1
catkin build
source devel/setup.bash
```

## Run Garage Simulation

Use the top-level EPIC launch, not the internal MARSIM launch directly:

```bash
cd /workspace/EPIC-stack
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch epic_planner garage.launch
```

`mars_drone_sim garage.launch` is an internal include file and requires arguments such as `sensing_horizon` from the parent launch.

## Map Notes

The committed `garage.pcd` is the processed Poongsan garage-style map used by MARSIM. It is already aligned and downsampled for simulation use.

Current expected SHA256:

```text
873398fb0a9339edb0a506d20636e5e408024fa50cd79d2ae81a14584f6577fd
```
