#!/usr/bin/env python3

"""
EPIC Planning Metrics Measurement Script
Measures computation time of global planning and local planning during simulation.

Topics:
- /time_cost: Global planning time (std_msgs/Float32)
- /visualizer/totoalOptimize_timecost: Local planning time (std_msgs/Float64)
- Global planning detailed topics (std_msgs/Float32):
  * /global_planning/update_topo_skeleton_cost
  * /global_planning/update_odom_vertex_cost
  * /global_planning/vp_cluster_cost
  * /global_planning/remove_unreachable_cost
  * /global_planning/select_vp_cost
  * /global_planning/insert_viewpoint_cost
  * /global_planning/calculate_tsp_cost
  * /global_planning/lkh_solver_cost
  * /global_planning/call_planner_cost
  * /global_planning/ikd_tree_insert_cost
  * /global_planning/update_frontier_clusters_cost
- Local planning detailed topics (std_msgs/Float64):
  * /visualizer/yaw_trajectory_optimization_cost
  * /visualizer/trajectory_generation_cost
  * /visualizer/velocity_check_cost
  * /visualizer/trajOptimize_timecost
  * /visualizer/topo_graph_search_cost
  * /visualizer/path_collision_check_cost
  * /visualizer/pointCloudProcess_timecost
  * /visualizer/fast_searcher_search_cost
  * /visualizer/lbfgs_optimization_cost
  * /visualizer/collision_check_cost
  * /visualizer/bubble_astar_search_cost
  * /visualizer/PolysGenerate_timecost
  * /local_planning/bubble_astar_search_cost
  * /local_planning/fast_searcher_search_cost
  * /local_planning/topo_graph_search_cost

Output:
- CSV files with timestamp and computation times
- Can be processed later for plotting and statistical analysis
"""

import rospy
import csv
import os
import sys
from std_msgs.msg import Float32, Float64
from datetime import datetime
import signal

class PlanningMetricsMeasurer:
    def __init__(self):
        rospy.init_node('planning_metrics_measurer', anonymous=True)
        
        # Get parameters
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/catkin_ws/data'))
        self.experiment_name = rospy.get_param('~experiment_name', 'epic_planning')
        
        # Create output directory
        if not os.path.exists(str(self.output_dir)):
            os.makedirs(str(self.output_dir))
        
        # Get current timestamp for unique file naming
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # CSV file paths
        self.global_planning_file = os.path.join(
            str(self.output_dir), f"{self.experiment_name}_global_planning_{timestamp}.csv"
        )
        self.global_detailed_file = os.path.join(
            str(self.output_dir), f"{self.experiment_name}_global_detailed_{timestamp}.csv"
        )
        self.local_planning_file = os.path.join(
            str(self.output_dir), f"{self.experiment_name}_local_planning_{timestamp}.csv"
        )
        self.local_detailed_file = os.path.join(
            str(self.output_dir), f"{self.experiment_name}_local_detailed_{timestamp}.csv"
        )
        
        # Global planning detailed metrics storage
        self.global_detailed_data = {
            'update_topo_skeleton_cost': None,
            'update_odom_vertex_cost': None,
            'vp_cluster_cost': None,
            'remove_unreachable_cost': None,
            'select_vp_cost': None,
            'insert_viewpoint_cost': None,
            'calculate_tsp_cost': None,
            'lkh_solver_cost': None,
            'call_planner_cost': None,
            'ikd_tree_insert_cost': None,
            'update_frontier_clusters_cost': None
        }
        
        # Local planning detailed metrics storage
        self.local_detailed_data = {
            'yaw_trajectory_optimization_cost': None,
            'trajectory_generation_cost': None,
            'velocity_check_cost': None,
            'trajOptimize_timecost': None,
            'topo_graph_search_cost': None,
            'path_collision_check_cost': None,
            'pointCloudProcess_timecost': None,
            'fast_searcher_search_cost': None,
            'lbfgs_optimization_cost': None,
            'collision_check_cost': None,
            'bubble_astar_search_cost': None,
            'PolysGenerate_timecost': None
        }
        
        # Counters for each metric type
        self.global_detailed_counts = {key: 0 for key in self.global_detailed_data.keys()}
        self.local_detailed_counts = {key: 0 for key in self.local_detailed_data.keys()}
        
        # Initialize CSV files
        self._init_csv_files()
        
        # Simulation start time (will be set when first message arrives)
        self.sim_start_time = None
        
        # Counters for statistics
        self.global_count = 0
        self.local_count = 0
        
        # Subscribers for original topics
        self.global_sub = rospy.Subscriber(
            '/time_cost', Float32, self.global_planning_callback
        )
        self.local_sub = rospy.Subscriber(
            '/visualizer/totoalOptimize_timecost', Float64, self.local_planning_callback
        )
        
        # Subscribers for detailed global planning metrics
        self.detailed_subs = {}
        global_topics = {
            'update_topo_skeleton_cost': '/global_planning/update_topo_skeleton_cost',
            'update_odom_vertex_cost': '/global_planning/update_odom_vertex_cost',
            'vp_cluster_cost': '/global_planning/vp_cluster_cost',
            'remove_unreachable_cost': '/global_planning/remove_unreachable_cost',
            'select_vp_cost': '/global_planning/select_vp_cost',
            'insert_viewpoint_cost': '/global_planning/insert_viewpoint_cost',
            'calculate_tsp_cost': '/global_planning/calculate_tsp_cost',
            'lkh_solver_cost': '/global_planning/lkh_solver_cost',
            'call_planner_cost': '/global_planning/call_planner_cost',
            'ikd_tree_insert_cost': '/global_planning/ikd_tree_insert_cost',
            'update_frontier_clusters_cost': '/global_planning/update_frontier_clusters_cost'
        }
        
        for metric_name, topic_name in global_topics.items():
            self.detailed_subs[metric_name] = rospy.Subscriber(
                topic_name, Float32, 
                lambda msg, name=metric_name: self.global_detailed_callback(msg, name)
            )
        
        # Subscribers for detailed local planning metrics
        local_topics = {
            'yaw_trajectory_optimization_cost': '/visualizer/yaw_trajectory_optimization_cost',
            'trajectory_generation_cost': '/visualizer/trajectory_generation_cost',
            'velocity_check_cost': '/visualizer/velocity_check_cost',
            'trajOptimize_timecost': '/visualizer/trajOptimize_timecost',
            'topo_graph_search_cost': '/visualizer/topo_graph_search_cost',
            'path_collision_check_cost': '/visualizer/path_collision_check_cost',
            'pointCloudProcess_timecost': '/visualizer/pointCloudProcess_timecost',
            'fast_searcher_search_cost': '/visualizer/fast_searcher_search_cost',
            'lbfgs_optimization_cost': '/visualizer/lbfgs_optimization_cost',
            'collision_check_cost': '/visualizer/collision_check_cost',
            'bubble_astar_search_cost': '/visualizer/bubble_astar_search_cost',
            'PolysGenerate_timecost': '/visualizer/PolysGenerate_timecost',
            'local_bubble_astar_search_cost': '/local_planning/bubble_astar_search_cost',
            'local_fast_searcher_search_cost': '/local_planning/fast_searcher_search_cost',
            'local_topo_graph_search_cost': '/local_planning/topo_graph_search_cost'
        }
        
        for metric_name, topic_name in local_topics.items():
            self.detailed_subs[metric_name] = rospy.Subscriber(
                topic_name, Float64, 
                lambda msg, name=metric_name: self.local_detailed_callback(msg, name)
            )
        
        rospy.loginfo(f"📊 Planning Metrics Measurer started")
        rospy.loginfo(f"📁 Output directory: {self.output_dir}")
        rospy.loginfo(f"📈 Global planning data: {self.global_planning_file}")
        rospy.loginfo(f"📈 Global detailed data: {self.global_detailed_file}")
        rospy.loginfo(f"📈 Local planning data: {self.local_planning_file}")
        rospy.loginfo(f"📈 Local detailed data: {self.local_detailed_file}")
        rospy.loginfo(f"🎯 Experiment: {self.experiment_name}")
        rospy.loginfo(f"⏰ Waiting for planning messages...")
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def _init_csv_files(self):
        """Initialize CSV files with headers"""
        # Global planning CSV
        with open(self.global_planning_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ros_time', 'sim_time', 'global_planning_time_ms', 'sequence'])
        
        # Global planning detailed CSV
        with open(self.global_detailed_file, 'w', newline='') as f:
            writer = csv.writer(f)
            headers = ['ros_time', 'sim_time', 'metric_type', 'time_ms', 'sequence']
            writer.writerow(headers)
        
        # Local planning CSV
        with open(self.local_planning_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ros_time', 'sim_time', 'local_planning_time_ms', 'sequence'])
        
        # Local planning detailed CSV
        with open(self.local_detailed_file, 'w', newline='') as f:
            writer = csv.writer(f)
            headers = ['ros_time', 'sim_time', 'metric_type', 'time_ms', 'sequence']
            writer.writerow(headers)
    
    def global_planning_callback(self, msg):
        """Callback for global planning time"""
        current_time = rospy.Time.now()
        
        # Set simulation start time from first message
        if self.sim_start_time is None:
            self.sim_start_time = current_time
            rospy.loginfo(f"🚀 Simulation start time set: {current_time.to_sec()}")
        
        # Calculate simulation time (time since start)
        sim_time = (current_time - self.sim_start_time).to_sec()
        
        self.global_count += 1
        
        # Write to CSV
        with open(self.global_planning_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time.to_sec(),
                sim_time,
                msg.data,
                self.global_count
            ])
        
        rospy.loginfo_throttle(5, 
            f"🌍 Global planning #{self.global_count}: {msg.data:.2f}ms @ {sim_time:.2f}s"
        )
    
    def global_detailed_callback(self, msg, metric_name):
        """Callback for detailed global planning metrics"""
        current_time = rospy.Time.now()
        
        # Set simulation start time from first message if not set
        if self.sim_start_time is None:
            self.sim_start_time = current_time
            rospy.loginfo(f"🚀 Simulation start time set: {current_time.to_sec()}")
        
        # Calculate simulation time (time since start)
        sim_time = (current_time - self.sim_start_time).to_sec()
        
        # Update counter for this metric
        self.global_detailed_counts[metric_name] += 1
        
        # Write to detailed CSV
        with open(self.global_detailed_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time.to_sec(),
                sim_time,
                metric_name,
                msg.data,
                self.global_detailed_counts[metric_name]
            ])
        
        rospy.loginfo_throttle(5,
            f"🔍 {metric_name} #{self.global_detailed_counts[metric_name]}: {msg.data:.2f}ms @ {sim_time:.2f}s"
        )
    
    def local_planning_callback(self, msg):
        """Callback for local planning time"""
        current_time = rospy.Time.now()
        
        # Set simulation start time from first message if not set
        if self.sim_start_time is None:
            self.sim_start_time = current_time
            rospy.loginfo(f"🚀 Simulation start time set: {current_time.to_sec()}")
        
        # Calculate simulation time (time since start)
        sim_time = (current_time - self.sim_start_time).to_sec()
        
        self.local_count += 1
        
        # Write to CSV
        with open(self.local_planning_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time.to_sec(),
                sim_time,
                msg.data,
                self.local_count
            ])
        
        rospy.loginfo_throttle(5,
            f"🤖 Local planning #{self.local_count}: {msg.data:.2f}ms @ {sim_time:.2f}s"
        )
    
    def local_detailed_callback(self, msg, metric_name):
        """Callback for detailed local planning metrics"""
        current_time = rospy.Time.now()
        
        # Set simulation start time from first message if not set
        if self.sim_start_time is None:
            self.sim_start_time = current_time
            rospy.loginfo(f"🚀 Simulation start time set: {current_time.to_sec()}")
        
        # Calculate simulation time (time since start)
        sim_time = (current_time - self.sim_start_time).to_sec()
        
        # Update counter for this metric
        self.local_detailed_counts[metric_name] += 1
        
        # Write to detailed CSV
        with open(self.local_detailed_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time.to_sec(),
                sim_time,
                metric_name,
                msg.data,
                self.local_detailed_counts[metric_name]
            ])
        
        rospy.loginfo_throttle(5,
            f"🔧 {metric_name} #{self.local_detailed_counts[metric_name]}: {msg.data:.2f}ms @ {sim_time:.2f}s"
        )
    
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        rospy.loginfo("\n🛑 Shutting down Planning Metrics Measurer...")
        self.print_statistics()
        rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)
    
    def print_statistics(self):
        """Print basic statistics"""
        rospy.loginfo("📊 Planning Metrics Summary:")
        rospy.loginfo(f"   🌍 Global planning messages: {self.global_count}")
        rospy.loginfo(f"   🤖 Local planning messages: {self.local_count}")
        
        # Print detailed global planning statistics
        rospy.loginfo("   🔍 Global planning detailed metrics:")
        for metric_name, count in self.global_detailed_counts.items():
            if count > 0:
                rospy.loginfo(f"      - {metric_name}: {count} messages")
        
        # Print detailed local planning statistics
        rospy.loginfo("   🔧 Local planning detailed metrics:")
        for metric_name, count in self.local_detailed_counts.items():
            if count > 0:
                rospy.loginfo(f"      - {metric_name}: {count} messages")
        
        rospy.loginfo(f"   📁 Data saved to: {self.output_dir}")
        
        if self.sim_start_time is not None:
            total_sim_time = (rospy.Time.now() - self.sim_start_time).to_sec()
            rospy.loginfo(f"   ⏱️ Total simulation time: {total_sim_time:.2f}s")
            
            if self.global_count > 0:
                global_rate = self.global_count / total_sim_time
                rospy.loginfo(f"   📈 Global planning rate: {global_rate:.2f} Hz")
            
            if self.local_count > 0:
                local_rate = self.local_count / total_sim_time
                rospy.loginfo(f"   📈 Local planning rate: {local_rate:.2f} Hz")
    
    def run(self):
        """Main execution loop"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.print_statistics()

def main():
    try:
        measurer = PlanningMetricsMeasurer()
        measurer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Planning Metrics Measurer interrupted")
    except Exception as e:
        rospy.logerr(f"Error in Planning Metrics Measurer: {e}")

if __name__ == '__main__':
    main()
