#!/usr/bin/env python3
"""
Exploration Evaluation Metrics Node

Collects 4 evaluation metrics during drone exploration:
1. RTH Origin Error: distance from origin at FINISH
2. Average Flight Speed: mean odometry velocity magnitude (trigger~FINISH)
3. Path Tracking RMSE: RMSE of commanded vs actual position (trigger~FINISH)
4. Exploration Rate: explored_cells / total_voxels time series

Outputs per-iteration CSV files for batch experiment aggregation.
"""

import rospy
import csv
import os
import sys
import struct
import math
import numpy as np
from datetime import datetime
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import signal


class ExplorationEvaluator:
    def __init__(self):
        rospy.init_node('exploration_evaluator', anonymous=True)

        # Parameters
        self.odom_topic = rospy.get_param('~odom_topic', '/quad_0/lidar_slam/odom')
        self.position_cmd_topic = rospy.get_param('~position_cmd_topic', '/planning/pos_cmd')
        self.pcd_path = rospy.get_param('~pcd_path', '')
        self.cell_size = rospy.get_param('~cell_size', 0.4)
        self.map_min = np.array([
            rospy.get_param('~map_min_x', -10.0),
            rospy.get_param('~map_min_y', -6.0),
            rospy.get_param('~map_min_z', -0.2)
        ])
        self.map_max = np.array([
            rospy.get_param('~map_max_x', 182.0),
            rospy.get_param('~map_max_y', 150.2),
            rospy.get_param('~map_max_z', 3.8)
        ])
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/.ros'))
        self.auto_shutdown = rospy.get_param('~auto_shutdown', False)

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # State
        self.phase = "IDLE"  # IDLE -> ACTIVE -> DONE
        self.prev_state = ""
        self.origin_pos = None
        self.current_pos = None
        self.mission_start_time = None

        # Metric 2: speed timeseries (elapsed_time, speed)
        self.speed_timeseries = []

        # Metric 3: tracking error timeseries (elapsed_time, error)
        self.latest_cmd_pos = None
        self.tracking_error_timeseries = []

        # Metric 4: exploration rate timeseries
        self.exploration_timeseries = []
        self.total_voxels = 0

        # Precompute total voxels from garage.pcd (with cache)
        # Cache is stored in home dir to avoid issues with per-iteration output_dir
        cache_dir = os.path.expanduser('~/.ros')
        if self.pcd_path and os.path.exists(self.pcd_path):
            self.total_voxels = self._load_or_compute_voxels(self.pcd_path, cache_dir)
            rospy.loginfo("[Eval] Total voxels from PCD: %d (cell_size=%.2f)", self.total_voxels, self.cell_size)
        else:
            rospy.logwarn("[Eval] PCD file not found: %s. Exploration rate will use bounding box.", self.pcd_path)
            cell_counts = np.floor((self.map_max - self.map_min) / self.cell_size).astype(int) + 1
            self.total_voxels = int(np.prod(cell_counts))
            rospy.loginfo("[Eval] Total voxels from bounding box: %d", self.total_voxels)

        # Subscribers
        self.state_sub = rospy.Subscriber('/planning/state', Marker, self.state_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.explored_sub = rospy.Subscriber(
            '/frontier_manager/explored_cell_count', Int32, self.explored_callback)

        # Lazy import for PositionCommand (may not be on PYTHONPATH)
        try:
            from quadrotor_msgs.msg import PositionCommand
            self.cmd_sub = rospy.Subscriber(
                self.position_cmd_topic, PositionCommand, self.cmd_callback)
            rospy.loginfo("[Eval] Subscribed to %s (PositionCommand)", self.position_cmd_topic)
        except ImportError:
            rospy.logwarn("[Eval] quadrotor_msgs not found. Using generic subscriber for position_cmd.")
            self.cmd_sub = rospy.Subscriber(
                self.position_cmd_topic, rospy.AnyMsg, self.cmd_callback_generic)

        rospy.loginfo("[Eval] Exploration Evaluator started. output_dir=%s, auto_shutdown=%s",
                      self.output_dir, self.auto_shutdown)

        signal.signal(signal.SIGINT, self._signal_handler)

    def _get_cache_path(self, pcd_path, cache_dir):
        """Get cache file path based on PCD path and parameters."""
        import hashlib
        key = f"{os.path.abspath(pcd_path)}|{os.path.getmtime(pcd_path)}|{self.cell_size}|{self.map_min.tolist()}|{self.map_max.tolist()}"
        h = hashlib.md5(key.encode()).hexdigest()[:12]
        return os.path.join(cache_dir, f'voxel_cache_{h}.txt')

    def _load_or_compute_voxels(self, pcd_path, cache_dir):
        """Load voxel count from cache, or compute and cache it."""
        cache_path = self._get_cache_path(pcd_path, cache_dir)
        if os.path.exists(cache_path):
            with open(cache_path, 'r') as f:
                count = int(f.read().strip())
            rospy.loginfo("[Eval] Loaded voxel count from cache: %s (%d voxels)", cache_path, count)
            return count

        count = self._voxelize_pcd(pcd_path)
        with open(cache_path, 'w') as f:
            f.write(str(count))
        rospy.loginfo("[Eval] Cached voxel count to: %s", cache_path)
        return count

    def _voxelize_pcd(self, pcd_path):
        """Load PCD file and count unique voxels at cell_size resolution."""
        rospy.loginfo("[Eval] Loading PCD: %s ...", pcd_path)

        try:
            with open(pcd_path, 'rb') as f:
                # Parse PCD header
                num_points = 0
                data_type = 'ascii'
                fields = []
                field_sizes = []
                field_types = []
                field_counts = []

                while True:
                    line = f.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('FIELDS'):
                        fields = line.split()[1:]
                    elif line.startswith('SIZE'):
                        field_sizes = [int(x) for x in line.split()[1:]]
                    elif line.startswith('TYPE'):
                        field_types = line.split()[1:]
                    elif line.startswith('COUNT'):
                        field_counts = [int(x) for x in line.split()[1:]]
                    elif line.startswith('POINTS'):
                        num_points = int(line.split()[1])
                    elif line.startswith('DATA'):
                        data_type = line.split()[1]
                        break

                rospy.loginfo("[Eval] PCD: %d points, format: %s", num_points, data_type)

                x_idx = fields.index('x') if 'x' in fields else 0
                y_idx = fields.index('y') if 'y' in fields else 1
                z_idx = fields.index('z') if 'z' in fields else 2

                if data_type == 'binary':
                    point_size = sum(s * c for s, c in zip(field_sizes, field_counts))
                    offsets = []
                    offset = 0
                    for i in range(len(fields)):
                        offsets.append(offset)
                        offset += field_sizes[i] * field_counts[i]

                    raw_data = f.read(num_points * point_size)
                    points = np.zeros((num_points, 3), dtype=np.float32)
                    for i, idx in enumerate([x_idx, y_idx, z_idx]):
                        fmt = 'f' if field_sizes[idx] == 4 else 'd'
                        byte_offset = offsets[idx]
                        for p in range(num_points):
                            start = p * point_size + byte_offset
                            points[p, i] = struct.unpack_from(fmt, raw_data, start)[0]

                elif data_type == 'binary_compressed':
                    rospy.logwarn("[Eval] binary_compressed PCD not supported, falling back to bounding box")
                    cell_counts = np.floor((self.map_max - self.map_min) / self.cell_size).astype(int) + 1
                    return int(np.prod(cell_counts))
                else:
                    points = np.zeros((num_points, 3), dtype=np.float32)
                    for i in range(num_points):
                        line = f.readline().decode('ascii', errors='ignore').strip()
                        vals = line.split()
                        points[i, 0] = float(vals[x_idx])
                        points[i, 1] = float(vals[y_idx])
                        points[i, 2] = float(vals[z_idx])

            mask = np.all((points >= self.map_min) & (points <= self.map_max), axis=1)
            points = points[mask]

            inv_cell_size = 1.0 / self.cell_size
            indices = np.floor((points - self.map_min) * inv_cell_size).astype(np.int32)

            cell_max = np.floor((self.map_max - self.map_min) * inv_cell_size).astype(np.int64) + 1
            packed = (indices[:, 0].astype(np.int64) * cell_max[1] * cell_max[2] +
                      indices[:, 1].astype(np.int64) * cell_max[2] +
                      indices[:, 2].astype(np.int64))
            unique_count = len(np.unique(packed))

            rospy.loginfo("[Eval] PCD voxelization complete: %d points -> %d unique voxels",
                          len(points), unique_count)
            return unique_count

        except Exception as e:
            rospy.logerr("[Eval] Failed to load PCD: %s", str(e))
            cell_counts = np.floor((self.map_max - self.map_min) / self.cell_size).astype(int) + 1
            return int(np.prod(cell_counts))

    def state_callback(self, msg):
        if not msg.text:
            return

        current_state = msg.text
        if current_state == self.prev_state:
            return

        rospy.loginfo("[Eval] State: %s -> %s (phase: %s)", self.prev_state, current_state, self.phase)

        # Detect trigger: WAIT_TRIGGER -> PLAN_TRAJ_EXP
        if (self.phase == "IDLE" and
                self.prev_state == "WAIT_TRIGGER" and
                current_state == "PLAN_TRAJ_EXP"):
            self.phase = "ACTIVE"
            self.mission_start_time = rospy.Time.now()
            if self.current_pos is not None:
                self.origin_pos = self.current_pos.copy()
                rospy.loginfo("[Eval] TRIGGERED! Origin captured: (%.2f, %.2f, %.2f)",
                              self.origin_pos[0], self.origin_pos[1], self.origin_pos[2])
            else:
                rospy.logwarn("[Eval] TRIGGERED but no odom received yet!")

        # Detect finish
        if self.phase == "ACTIVE" and current_state == "FINISH":
            self.phase = "DONE"
            rospy.loginfo("[Eval] FINISH detected! Computing metrics...")
            self._compute_and_save_metrics()

            if self.auto_shutdown:
                rospy.Timer(rospy.Duration(2.0),
                            lambda e: rospy.signal_shutdown("Evaluation complete"),
                            oneshot=True)

        self.prev_state = current_state

    def odom_callback(self, msg):
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.current_pos = pos

        if self.phase != "ACTIVE":
            return

        elapsed = (rospy.Time.now() - self.mission_start_time).to_sec()

        # Metric 2: speed timeseries
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        self.speed_timeseries.append((elapsed, speed))

        # Metric 3: tracking error timeseries
        if self.latest_cmd_pos is not None:
            error = np.linalg.norm(self.latest_cmd_pos - pos)
            self.tracking_error_timeseries.append((elapsed, error))

    def cmd_callback(self, msg):
        """Callback for quadrotor_msgs/PositionCommand."""
        self.latest_cmd_pos = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])

    def cmd_callback_generic(self, msg):
        """Fallback: deserialize PositionCommand manually if msg type unavailable."""
        rospy.logwarn_once("[Eval] Generic cmd callback - position_cmd not parsed. "
                          "Tracking RMSE will be unavailable.")

    def explored_callback(self, msg):
        if self.phase != "ACTIVE":
            return
        elapsed = (rospy.Time.now() - self.mission_start_time).to_sec()
        self.exploration_timeseries.append((elapsed, msg.data))

    def _compute_and_save_metrics(self):
        mission_duration = (rospy.Time.now() - self.mission_start_time).to_sec()

        # Metric 1: RTH origin error + finish position
        rth_origin_error = -1.0
        finish_x, finish_y, finish_z = 0.0, 0.0, 0.0
        origin_x, origin_y, origin_z = 0.0, 0.0, 0.0
        if self.current_pos is not None:
            finish_x, finish_y, finish_z = self.current_pos
        if self.origin_pos is not None:
            origin_x, origin_y, origin_z = self.origin_pos
        if self.origin_pos is not None and self.current_pos is not None:
            rth_origin_error = np.linalg.norm(self.current_pos - self.origin_pos)

        # Metric 2: average flight speed
        avg_speed = 0.0
        if self.speed_timeseries:
            avg_speed = sum(s for _, s in self.speed_timeseries) / len(self.speed_timeseries)

        # Metric 3: path tracking RMSE
        tracking_rmse = -1.0
        if self.tracking_error_timeseries:
            tracking_rmse = math.sqrt(
                sum(e * e for _, e in self.tracking_error_timeseries) / len(self.tracking_error_timeseries))

        # Metric 4: final exploration rate
        final_explored = 0
        final_exploration_rate = 0.0
        if self.exploration_timeseries:
            final_explored = self.exploration_timeseries[-1][1]
            if self.total_voxels > 0:
                final_exploration_rate = final_explored / self.total_voxels

        # Print summary
        rospy.loginfo("=" * 60)
        rospy.loginfo("[Eval] MISSION COMPLETE - Evaluation Metrics")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  Mission duration:      %.2f s", mission_duration)
        rospy.loginfo("  RTH origin error:      %.4f m (%.1f mm)", rth_origin_error, rth_origin_error * 1000)
        rospy.loginfo("  RTH success (<400mm):  %s", "YES" if rth_origin_error < 0.4 else "NO")
        rospy.loginfo("  Avg flight speed:      %.3f m/s", avg_speed)
        rospy.loginfo("  Path tracking RMSE:    %.4f m", tracking_rmse)
        rospy.loginfo("  Exploration rate:      %.4f (%.1f%%)", final_exploration_rate, final_exploration_rate * 100)
        rospy.loginfo("  Explored/Total voxels: %d / %d", final_explored, self.total_voxels)
        rospy.loginfo("  Finish position:       (%.3f, %.3f, %.3f)", finish_x, finish_y, finish_z)
        rospy.loginfo("  Speed samples:         %d", len(self.speed_timeseries))
        rospy.loginfo("  Tracking samples:      %d", len(self.tracking_error_timeseries))
        rospy.loginfo("=" * 60)

        # Save all CSVs
        self._save_summary_csv(mission_duration, rth_origin_error, avg_speed,
                               tracking_rmse, final_exploration_rate, final_explored,
                               finish_x, finish_y, finish_z,
                               origin_x, origin_y, origin_z)
        self._save_speed_timeseries_csv()
        self._save_tracking_error_timeseries_csv()
        self._save_exploration_timeseries_csv()

    def _save_summary_csv(self, mission_duration, rth_origin_error, avg_speed,
                          tracking_rmse, final_exploration_rate, final_explored,
                          finish_x, finish_y, finish_z,
                          origin_x, origin_y, origin_z):
        summary_file = os.path.join(self.output_dir, 'summary.csv')

        with open(summary_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'mission_duration_s', 'rth_origin_error_m',
                'avg_flight_speed_mps', 'tracking_rmse_m',
                'final_exploration_rate', 'total_voxels', 'explored_voxels',
                'finish_x', 'finish_y', 'finish_z',
                'origin_x', 'origin_y', 'origin_z',
                'speed_samples_count', 'tracking_samples_count'
            ])
            writer.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                f"{mission_duration:.2f}", f"{rth_origin_error:.4f}",
                f"{avg_speed:.4f}", f"{tracking_rmse:.4f}",
                f"{final_exploration_rate:.6f}", self.total_voxels, final_explored,
                f"{finish_x:.4f}", f"{finish_y:.4f}", f"{finish_z:.4f}",
                f"{origin_x:.4f}", f"{origin_y:.4f}", f"{origin_z:.4f}",
                len(self.speed_timeseries), len(self.tracking_error_timeseries)
            ])

        rospy.loginfo("[Eval] Summary saved to: %s", summary_file)

    def _save_speed_timeseries_csv(self):
        if not self.speed_timeseries:
            return
        ts_file = os.path.join(self.output_dir, 'speed_timeseries.csv')
        with open(ts_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['elapsed_time_s', 'speed_mps'])
            for elapsed, speed in self.speed_timeseries:
                writer.writerow([f"{elapsed:.3f}", f"{speed:.4f}"])
        rospy.loginfo("[Eval] Speed timeseries saved to: %s", ts_file)

    def _save_tracking_error_timeseries_csv(self):
        if not self.tracking_error_timeseries:
            return
        ts_file = os.path.join(self.output_dir, 'tracking_error_timeseries.csv')
        with open(ts_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['elapsed_time_s', 'tracking_error_m'])
            for elapsed, error in self.tracking_error_timeseries:
                writer.writerow([f"{elapsed:.3f}", f"{error:.4f}"])
        rospy.loginfo("[Eval] Tracking error timeseries saved to: %s", ts_file)

    def _save_exploration_timeseries_csv(self):
        if not self.exploration_timeseries:
            return
        ts_file = os.path.join(self.output_dir, 'exploration_timeseries.csv')
        with open(ts_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['elapsed_time_s', 'explored_cells', 'exploration_rate'])
            for elapsed, explored in self.exploration_timeseries:
                rate = explored / self.total_voxels if self.total_voxels > 0 else 0.0
                writer.writerow([f"{elapsed:.3f}", explored, f"{rate:.6f}"])
        rospy.loginfo("[Eval] Exploration timeseries saved to: %s", ts_file)

    def _signal_handler(self, signum, frame):
        rospy.loginfo("[Eval] Shutting down...")
        if self.phase == "ACTIVE":
            rospy.loginfo("[Eval] Mission was still active, saving partial metrics...")
            self._compute_and_save_metrics()
        rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass


def main():
    try:
        evaluator = ExplorationEvaluator()
        evaluator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Eval] Interrupted")
    except Exception as e:
        rospy.logerr("[Eval] Error: %s", str(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
