# EPIC Stack — onboard release

This `main` branch is the lightweight onboard release. It contains only:

- `src/EPIC_poongsan`
- `src/reactive_local_avoidance`

The full development tree and Git history are preserved on the `dev` branch.
Simulation, LiDAR drivers, odometry packages, and ML-X cropping are intentionally
not bundled in `main`.

## Lightweight clone

```bash
git clone --depth 1 --single-branch --branch main \
  https://github.com/kimhyoon/EPIC-stack.git
cd EPIC-stack
```

## Runtime contract

The onboard system must provide ROS Noetic, MAVROS, PCL, and the odometry/cloud
topics selected by the flight profile:

| Profile | Odometry | Planner cloud |
| --- | --- | --- |
| `real1.yaml` | `/mavros/odometry/in` | `/lio_sam/mapping/cloud_registered_raw` |
| `real2.yaml` | `/mavros/odometry/in` | `/ml_/pointcloud` |

`reactive_local_avoidance` consumes `sensor_msgs/PointCloud2`; this release does
not build against `livox_ros_driver2/CustomMsg`.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_init_workspace src
catkin_make
source devel/setup.bash
```

The workspace `src/CMakeLists.txt` is generated locally by
`catkin_init_workspace` and is not tracked because its symlink target is
machine-specific.

## Run

```bash
# real1
roslaunch epic_planner real_flight_boogang1.launch

# real2
roslaunch epic_planner real_flight_boogang2.launch
```

Only `real1.yaml` and `real2.yaml` are supported on `main`. Use `dev` for the
complete simulation, replay, sensor-driver, and development environment.
