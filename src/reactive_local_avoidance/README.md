# reactive_local_avoidance

LiDAR potential-field (APF) reactive local avoidance for a MAVROS drone.
Extended from a 2D `LaserScan` node to **3D point clouds** (e.g. Livox Mid-360).

## Approach

A 3D point cloud is reduced to **two bands** (raw sensor frame, small-tilt assumption):

- **Horizontal slab** `|z| < band_z_thr` → drives **x, y** avoidance (classic 2D APF on the in-plane cut).
- **Vertical column** `hypot(x, y) < band_r_thr` (AND) → drives **z** avoidance (ceiling / floor / overhead).

This decouples the axes so a vertical wall ahead (large horizontal offset) cannot inject a
spurious vertical command from the sensor's asymmetric vertical FOV, while genuine
overhead/underfoot obstacles still produce a vertical response.

The repulsion vector is computed per band, transformed body→map with the full rotation
matrix from odometry, clamped to a max move distance, and published as a `PositionTarget`.

## I/O

| Dir | Topic | Type |
|-----|-------|------|
| sub | `/livox/lidar` | `sensor_msgs/PointCloud2` |
| sub | `/mavros/local_position/odom` | `nav_msgs/Odometry` |
| pub | `/target_avoidance` | `mavros_msgs/PositionTarget` |
| pub | `/FSM_flag_avoidance` | `std_msgs/Int16` |
| pub | `/local_avoidance_visualization` | `geometry_msgs/PoseStamped` |

## Parameters

See [`config/local_avoidance.yaml`](config/local_avoidance.yaml). Key ones:

| Param | Default | Meaning |
|-------|---------|---------|
| `repulsive_m` | 2.0 | APF influence radius |
| `repulsive_gain` | 0.15 | repulsion gain |
| `lidar_min_threshold` | 0.4 | noise cut (ignore closer points) |
| `avoidance_trigger_m` | 1.45 | activation distance |
| `emergency_avoidance_m` | 0.95 | emergency (5x repulsion) distance |
| `avoidance_moving_m` | 1.45 | max move distance (3D) |
| `band_z_thr` | 0.3 | horizontal slab half-thickness |
| `band_r_thr` | 0.3 | vertical column radius |

## Build & Run

```bash
cd ~/catkin_ws/src
git clone https://github.com/sanghun17/reactive_local_avoidance.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
roslaunch local_avoidance local_avoidance.launch
# optional topic overrides:
# roslaunch local_avoidance local_avoidance.launch lidar_topic:=/livox/lidar odom_topic:=/mavros/local_position/odom
```

Requires ROS Noetic, `pcl_ros`, `pcl_conversions`, `mavros_msgs`.
