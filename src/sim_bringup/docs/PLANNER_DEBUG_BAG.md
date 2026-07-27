# Planner Wall-Crossing Debug Bag

This instrumentation is for root-cause runs where a generated MINCO trajectory
appears to cross a wall. It is disabled by default because recording full point
clouds and internal planner evidence can affect timing measurements.

## Run

```bash
roslaunch sim_bringup garage_sim.launch \
  use_cloud_crop_bridge:=true \
  enable_avoidance:=false \
  record_results:=true \
  record_planner_debug:=true
```

Keep `enable_avoidance:=false` for the first root-cause run. In this mode the
simulation relay forwards `/position_cmd` to `/planning/pos_cmd` unchanged, so
any wall-crossing command can be attributed upstream of the avoidance MUX.
Use `enable_avoidance:=true` only for a second A/B run after the
`local_avoidance` package has been built and removed from the catkin skiplist.

The normal tracking bag remains unchanged for compatibility with the existing
EPIC report scripts. A separate LZ4 bag is written directly under the test
result directory:

```text
result/test_YYYYMMDD_HHMMSS/
  test_YYYYMMDD_HHMMSS_planner_debug.bag
  test_YYYYMMDD_HHMMSS_params.yaml
  test_YYYYMMDD_HHMMSS_manifest.txt
```

The manifest records the workspace commit/status, source PCD identity, and the
topic list. The parameter dump captures the effective ROS parameter state at
`WAIT_TRIGGER`.

After the bag is closed, run the quality gate and topology evidence summary:

```bash
rosrun sim_bringup analyze_topology_stability_bag.py \
  result/test_YYYYMMDD_HHMMSS/test_YYYYMMDD_HHMMSS_planner_debug.bag
```

The generated `summary.md` classifies odometry as `VALID`, `DEGRADED`, or
`INVALID_FOR_ALGORITHM_COMPARISON`. A bag with SLAM position jumps, impossible
derived speed, or backward timestamps must not be used for algorithm
performance claims. It remains useful as a robustness stress test.

## Replay Through the Updated Planner

Analyzing a recorded output bag does not execute the updated algorithm. To feed
the recorded odometry and cropped cloud through the current EPIC binary, run:

```bash
roslaunch sim_bringup topology_replay_validation.launch \
  bag:=/absolute/path/to/test_planner_debug.bag \
  output_bag:=/tmp/topology_replay_after_patch.bag \
  config_file:=garage.yaml \
  rviz:=true
```

The launch plays only the odometry and cropped-cloud input topics, starts EPIC
after both inputs are available, records the newly generated diagnostics, and
opens `traj.rviz`. It does not replay the old planner outputs into the updated
planner.

After replay finishes:

```bash
rosrun sim_bringup analyze_topology_stability_bag.py \
  /tmp/topology_replay_after_patch.bag
```

Run the quality gate on the source bag first. If its odometry is invalid, use
the replay only to test robustness, not to compare topology performance.

## Evidence Path

The bag preserves the complete decision chain needed to separate four failure
classes:

```text
raw MARSIM cloud
  -> cropped EPIC cloud
  -> EPIC iKD-tree and topology edge checks
  -> local guide path
  -> exact downsampled obstacle points passed to FIRI
  -> raw FIRI hPolys
  -> active observed-FOV faces
  -> final hPolys passed to GCOPTER/MINCO
  -> MINCO command
  -> post-avoidance controller command
  -> odometry
```

Use the evidence as follows:

1. If the wall is missing from the cropped cloud and
   `/debug/local_planner_obstacle_points`, the sensing/iKD-tree input is the
   first failure point.
2. If the wall is present in the planner obstacle points but a successful
   `/debug/topo_edge_checks` path crosses it, topology connection validation is
   the first failure point.
3. If the guide path is valid but `/debug/clipped_hpolys` extends through the
   wall, corridor generation or FOV clipping is the first failure point.
4. If the final hPolys do not cross the wall but the trajectory does, the
   GCOPTER/MINCO constraint path or post-check is the first failure point.
5. If `/position_cmd` is valid but `/planning/pos_cmd` crosses the wall, the
   command was changed after MINCO by the avoidance MUX.

## Internal Topic Schemas

The custom evidence topics use standard ROS message types so no new message
package is required.

### `/debug/local_planner_obstacle_points`

Type: `sensor_msgs/PointCloud2`

This is the voxel-filtered point set passed to `convexCover()` for the current
local plan. `header.seq` is the local `plan_seq`.

### `/debug/local_guide_path`

Type: `nav_msgs/Path`

This is the shortened guide path passed to corridor generation. `header.seq`
is the local `plan_seq`.

### `/debug/raw_hpolys` and `/debug/clipped_hpolys`

Type: `std_msgs/Float64MultiArray`

```text
[version=1, plan_seq, poly_count,
 rows, (nx, ny, nz, d) repeated rows times,
 ...]
```

Each row encodes the half-space `n dot x + d <= 0`. `raw_hpolys` is captured
immediately after FIRI. `clipped_hpolys` is captured after P0 insertion,
observed-FOV clipping, chain trimming, and fallback resizing, immediately before
the same vector is passed to `gcopter.setup()`.

### `/debug/active_fov_faces`

Type: `std_msgs/Float64MultiArray`

```text
[version=1, plan_seq, face_count,
 face_index, has_frontier, observed_outside, active, nx, ny, nz, d,
 ...]
```

Faces are ordered left, right, upper, lower. A face is active only when an FOV
edge frontier exists and no DENSE cell is found immediately outside it.

### `/debug/topo_edge_checks`

Type: `std_msgs/Float64MultiArray`

```text
[version=2, batch_seq, only_raycast, pair_count,
success, result, start_x, start_y, start_z, end_x, end_y, end_z,
path_point_count, path_cost, min_known_obstacle_distance,
...]
```

This records each topology node pair considered by `TopoGraph::insertNodes()`.
The distance field is computed from the same EPIC lidar map after a successful
path is returned. Failed pairs use `-1` for cost and distance.

### `/debug/topo_edge_updates`

Type: `std_msgs/Float64MultiArray`

```text
[version=1, batch_seq, pair_count,
 success, result, was_connected,
 start_x, start_y, start_z, end_x, end_y, end_z,
 consecutive_failures,
 ...]
```

This records periodic skeleton-edge validation. `result` uses the
`ParallelBubbleAstar` values (`1=REACH_END`, `2=NO_PATH`, `3=START_FAIL`,
`4=END_FAIL`, `5=TIME_OUT`) and `6=EDGE_COLLISION` for a path that was returned
but failed the existing collision check. Failed edges are removed from the
active graph and retried after a result-specific cooldown.

### Topology stability RViz markers

```text
/debug/topology_stability_nodes
  green  = node matched in the current topology update
  yellow = node retained by the miss hysteresis

/debug/topology_failed_edges
  magenta = TIME_OUT
  orange  = EDGE_COLLISION
  red     = other failed connection results
```

Both markers are available under `debug_info/topology_stability` in
`traj.rviz`. They affect visualization only.

### `/debug/trajectory_clearance`

Type: `std_msgs/Float64MultiArray`

```text
[version=1, plan_seq, traj_id, safe, collision_time,
 min_known_obstacle_distance, sample_x, sample_y, sample_z, hard_radius]
```

This reports the result of the existing trajectory collision checker. It does
not introduce a second collision policy.

## Command Topics

The two command topics must be interpreted separately:

```text
/position_cmd
  raw command published by traj_server from the MINCO trajectory

/planning/pos_cmd
  command delivered to cascadePID after the avoidance relay/MUX
```

The existing report bag intentionally normalizes its selected controller input
to `/position_cmd` for compatibility. The planner debug bag keeps both original
topic names, which is required to determine whether a wall-crossing command was
created by MINCO or introduced by the downstream avoidance path.
