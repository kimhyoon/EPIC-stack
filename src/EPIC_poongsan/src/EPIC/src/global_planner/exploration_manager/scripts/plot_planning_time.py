#!/usr/bin/env python3

"""
EPIC Planning Time Analysis and Plotting Script
Processes CSV data from measure_planning_metrics.py and generates plots and statistics.

Usage:
    python3 plot_planning_time.py --data_dir /tmp/epic_metrics --experiment epic_planning
"""

import csv
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import glob
from datetime import datetime
from collections import defaultdict

class PlanningTimeAnalyzer:
    def __init__(self, data_dir, experiment_name):
        self.data_dir = data_dir
        self.experiment_name = experiment_name
        self.global_data = None
        self.local_data = None
        self.global_detailed_data = None
        self.local_detailed_data = None
        
        # Set plot style
        try:
            plt.style.use('seaborn-v0_8')
        except:
            try:
                plt.style.use('seaborn')
            except:
                pass  # Fall back to default style if seaborn not available
        
        # Define colors for each global planning metric
        self.global_metric_colors = {
            'update_topo_skeleton_cost': '#FF6B6B',
            'update_odom_vertex_cost': '#4ECDC4', 
            'vp_cluster_cost': '#45B7D1',
            'remove_unreachable_cost': '#96CEB4',
            'select_vp_cost': '#FFEAA7',
            'insert_viewpoint_cost': '#DDA0DD',
            'calculate_tsp_cost': '#98D8C8',
            'lkh_solver_cost': '#F7DC6F',
            'call_planner_cost': '#BB8FCE',
            'ikd_tree_insert_cost': '#F8C471',
            'update_frontier_clusters_cost': '#85C1E9'
        }
        
        # Define colors for each local planning metric
        self.local_metric_colors = {
            'yaw_trajectory_optimization_cost': '#FF6B6B',
            'trajectory_generation_cost': '#4ECDC4',
            'velocity_check_cost': '#45B7D1',
            'trajOptimize_timecost': '#96CEB4',
            'topo_graph_search_cost': '#FFEAA7',
            'path_collision_check_cost': '#DDA0DD',
            'pointCloudProcess_timecost': '#98D8C8',
            'fast_searcher_search_cost': '#F7DC6F',
            'lbfgs_optimization_cost': '#BB8FCE',
            'collision_check_cost': '#F8C471',
            'bubble_astar_search_cost': '#85C1E9',
            'PolysGenerate_timecost': '#FF9F43',
            'local_bubble_astar_search_cost': '#00B894',
            'local_fast_searcher_search_cost': '#6C5CE7',
            'local_topo_graph_search_cost': '#FD79A8'
        }
    
    def load_csv_data(self, filename):
        """Load CSV data into dictionary"""
        data = {
            'ros_time': [],
            'sim_time': [],
            'planning_time_ms': [],
            'sequence': []
        }
        
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    data['ros_time'].append(float(row['ros_time']))
                    data['sim_time'].append(float(row['sim_time']))
                    if 'global_planning_time_ms' in row:
                        data['planning_time_ms'].append(float(row['global_planning_time_ms']))
                    elif 'local_planning_time_ms' in row:
                        data['planning_time_ms'].append(float(row['local_planning_time_ms']))
                    data['sequence'].append(int(row['sequence']))
        except Exception as e:
            print(f"Error reading {filename}: {e}")
            return None
        
        # Convert to numpy arrays for easier manipulation
        result = {}
        for key in data:
            result[key] = np.array(data[key])
        
        return result
    
    def load_detailed_csv_data(self, filename):
        """Load detailed planning CSV data using pure Python"""
        try:
            detailed_data = defaultdict(lambda: {
                'ros_time': [],
                'sim_time': [],
                'time_ms': [],
                'sequence': []
            })
            
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    metric_type = row['metric_type']
                    detailed_data[metric_type]['ros_time'].append(float(row['ros_time']))
                    detailed_data[metric_type]['sim_time'].append(float(row['sim_time']))
                    detailed_data[metric_type]['time_ms'].append(float(row['time_ms']))
                    detailed_data[metric_type]['sequence'].append(int(row['sequence']))
            
            # Convert to numpy arrays
            result = {}
            for metric_type, data in detailed_data.items():
                result[metric_type] = {
                    'ros_time': np.array(data['ros_time']),
                    'sim_time': np.array(data['sim_time']),
                    'time_ms': np.array(data['time_ms']),
                    'sequence': np.array(data['sequence'])
                }
            
            return result
        except Exception as e:
            print(f"Error reading detailed data {filename}: {e}")
            return None
    
    def load_data(self):
        """Load CSV data files"""
        # Find the most recent data files
        global_pattern = os.path.join(self.data_dir, f"{self.experiment_name}_global_planning_*.csv")
        global_detailed_pattern = os.path.join(self.data_dir, f"{self.experiment_name}_global_detailed_*.csv")
        local_pattern = os.path.join(self.data_dir, f"{self.experiment_name}_local_planning_*.csv")
        local_detailed_pattern = os.path.join(self.data_dir, f"{self.experiment_name}_local_detailed_*.csv")
        
        global_files = glob.glob(global_pattern)
        global_detailed_files = glob.glob(global_detailed_pattern)
        local_files = glob.glob(local_pattern)
        local_detailed_files = glob.glob(local_detailed_pattern)
        
        if not global_files:
            print(f"❌ No global planning data found in {self.data_dir}")
            return False
        
        if not local_files:
            print(f"❌ No local planning data found in {self.data_dir}")
            return False
        
        # Use the most recent files
        global_file = max(global_files, key=os.path.getmtime)
        local_file = max(local_files, key=os.path.getmtime)
        
        print(f"📂 Loading global planning data: {os.path.basename(global_file)}")
        print(f"📂 Loading local planning data: {os.path.basename(local_file)}")
        
        self.global_data = self.load_csv_data(global_file)
        self.local_data = self.load_csv_data(local_file)
        
        # Load detailed global data if available
        if global_detailed_files:
            global_detailed_file = max(global_detailed_files, key=os.path.getmtime)
            print(f"📂 Loading global detailed data: {os.path.basename(global_detailed_file)}")
            self.global_detailed_data = self.load_detailed_csv_data(global_detailed_file)
            
            if self.global_detailed_data:
                total_detailed_records = sum(len(data['time_ms']) for data in self.global_detailed_data.values())
                print(f"✅ Loaded {total_detailed_records} detailed global planning records across {len(self.global_detailed_data)} metrics")
        
        # Load detailed local data if available
        if local_detailed_files:
            local_detailed_file = max(local_detailed_files, key=os.path.getmtime)
            print(f"📂 Loading local detailed data: {os.path.basename(local_detailed_file)}")
            self.local_detailed_data = self.load_detailed_csv_data(local_detailed_file)
            
            if self.local_detailed_data:
                total_detailed_records = sum(len(data['time_ms']) for data in self.local_detailed_data.values())
                print(f"✅ Loaded {total_detailed_records} detailed local planning records across {len(self.local_detailed_data)} metrics")
        
        if self.global_data is None or self.local_data is None:
            return False
        
        print(f"✅ Loaded {len(self.global_data['planning_time_ms'])} global planning records")
        print(f"✅ Loaded {len(self.local_data['planning_time_ms'])} local planning records")
        
        return True
    
    def calculate_statistics(self):
        """Calculate and print statistics"""
        print("\n📊 PLANNING TIME STATISTICS")
        print("=" * 50)
        
        if self.global_data and len(self.global_data['planning_time_ms']) > 0:
            global_times = self.global_data['planning_time_ms']
            print(f"\n🌍 GLOBAL PLANNING:")
            print(f"   Count: {len(global_times)}")
            print(f"   Mean:  {np.mean(global_times):.2f} ms")
            print(f"   Std:   {np.std(global_times):.2f} ms")
            print(f"   Min:   {np.min(global_times):.2f} ms")
            print(f"   Max:   {np.max(global_times):.2f} ms")
            print(f"   Median: {np.median(global_times):.2f} ms")
            print(f"   95th percentile: {np.percentile(global_times, 95):.2f} ms")
        
        # Detailed global planning statistics
        if self.global_detailed_data:
            print(f"\n🔍 GLOBAL PLANNING DETAILED METRICS:")
            for metric_name, data in self.global_detailed_data.items():
                if len(data['time_ms']) > 0:
                    times = data['time_ms']
                    print(f"\n   📈 {metric_name}:")
                    print(f"      Count: {len(times)}")
                    print(f"      Mean:  {np.mean(times):.2f} ms")
                    print(f"      Std:   {np.std(times):.2f} ms")
                    print(f"      Min:   {np.min(times):.2f} ms")
                    print(f"      Max:   {np.max(times):.2f} ms")
                    print(f"      Median: {np.median(times):.2f} ms")
        
        if self.local_data and len(self.local_data['planning_time_ms']) > 0:
            local_times = self.local_data['planning_time_ms']
            print(f"\n🤖 LOCAL PLANNING:")
            print(f"   Count: {len(local_times)}")
            print(f"   Mean:  {np.mean(local_times):.2f} ms")
            print(f"   Std:   {np.std(local_times):.2f} ms")
            print(f"   Min:   {np.min(local_times):.2f} ms")
            print(f"   Max:   {np.max(local_times):.2f} ms")
            print(f"   Median: {np.median(local_times):.2f} ms")
            print(f"   95th percentile: {np.percentile(local_times, 95):.2f} ms")
        
        # Detailed local planning statistics
        if self.local_detailed_data:
            print(f"\n🔧 LOCAL PLANNING DETAILED METRICS:")
            for metric_name, data in self.local_detailed_data.items():
                if len(data['time_ms']) > 0:
                    times = data['time_ms']
                    print(f"\n   📈 {metric_name}:")
                    print(f"      Count: {len(times)}")
                    print(f"      Mean:  {np.mean(times):.2f} ms")
                    print(f"      Std:   {np.std(times):.2f} ms")
                    print(f"      Min:   {np.min(times):.2f} ms")
                    print(f"      Max:   {np.max(times):.2f} ms")
                    print(f"      Median: {np.median(times):.2f} ms")
        
        # Planning frequency analysis
        if self.global_data and len(self.global_data['sim_time']) > 0:
            total_sim_time = np.max(self.global_data['sim_time'])
            global_rate = len(self.global_data['planning_time_ms']) / total_sim_time
            print(f"\n📈 PLANNING RATES:")
            print(f"   Total simulation time: {total_sim_time:.2f} s")
            print(f"   Global planning rate: {global_rate:.2f} Hz")
            
            if self.local_data and len(self.local_data['planning_time_ms']) > 0:
                local_rate = len(self.local_data['planning_time_ms']) / total_sim_time
                print(f"   Local planning rate: {local_rate:.2f} Hz")
    
    def create_detailed_time_series_plot(self, detailed_data, metric_colors, title_prefix, filename_prefix):
        """Create detailed time series plot for planning metrics"""
        if not detailed_data:
            return None
        
        # Calculate number of subplots needed
        n_metrics = len(detailed_data)
        n_cols = 3
        n_rows = (n_metrics + n_cols - 1) // n_cols
        
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(18, 5 * n_rows))
        
        # Handle different subplot configurations
        if n_rows == 1:
            if n_metrics == 1:
                axes = [axes]
            else:
                axes = axes.reshape(1, -1)
        else:
            axes = axes.flatten()
        
        for idx, (metric_name, data) in enumerate(detailed_data.items()):
            if idx >= len(axes):
                break
                
            ax = axes[idx]
            
            if len(data['time_ms']) > 0:
                color = metric_colors.get(metric_name, '#1f77b4')
                ax.plot(data['sim_time'], data['time_ms'], 'o-', 
                       linewidth=1, markersize=2, alpha=0.7, color=color)
                
                # Add mean line
                mean_time = np.mean(data['time_ms'])
                ax.axhline(y=mean_time, color='red', linestyle='--', alpha=0.7, 
                          label=f'Mean: {mean_time:.2f}ms')
                
                ax.set_title(f"{metric_name.replace('_', ' ').title()}")
                ax.set_xlabel('Simulation Time (s)')
                ax.set_ylabel('Time (ms)')
                ax.grid(True, alpha=0.3)
                ax.legend()
        
        # Hide unused subplots
        for idx in range(len(detailed_data), len(axes)):
            axes[idx].set_visible(False)
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.experiment_name}_{filename_prefix}_time_series_{timestamp}.png"
        filepath = os.path.join(self.data_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"💾 {title_prefix} detailed time series plot saved: {filename}")
        
        return fig
    
    def create_detailed_comparison_plot(self, detailed_data, metric_colors, title_prefix, filename_prefix):
        """Create comparison plot for detailed planning metrics"""
        if not detailed_data:
            return None
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
        
        # Box plot comparison - only include metrics with data
        data_to_plot = []
        labels = []
        colors = []
        
        for metric_name, data in detailed_data.items():
            if len(data['time_ms']) > 0:
                data_to_plot.append(data['time_ms'])
                labels.append(metric_name.replace('_', '\n'))
                colors.append(metric_colors.get(metric_name, '#1f77b4'))
        
        if data_to_plot:
            # Ensure all arrays have the same length for boxplot
            max_len = max(len(data) for data in data_to_plot)
            padded_data = []
            
            for data in data_to_plot:
                if len(data) < max_len:
                    # Pad with NaN values
                    padded = np.full(max_len, np.nan)
                    padded[:len(data)] = data
                    padded_data.append(padded)
                else:
                    padded_data.append(data)
            
            box_plot = ax1.boxplot(padded_data, labels=labels, patch_artist=True)
            
            # Customize colors
            for patch, color in zip(box_plot['boxes'], colors):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)
            
            ax1.set_ylabel('Time (ms)')
            ax1.set_title(f'{title_prefix} Detailed Metrics - Distribution')
            ax1.grid(True, alpha=0.3)
            plt.setp(ax1.get_xticklabels(), rotation=45, ha='right')
        
        # Mean values bar plot
        means = []
        metric_names = []
        bar_colors = []
        
        for metric_name, data in detailed_data.items():
            if len(data['time_ms']) > 0:
                means.append(np.mean(data['time_ms']))
                metric_names.append(metric_name.replace('_', '\n'))
                bar_colors.append(metric_colors.get(metric_name, '#1f77b4'))
        
        if means:
            bars = ax2.bar(metric_names, means, color=bar_colors, alpha=0.7)
            ax2.set_ylabel('Mean Time (ms)')
            ax2.set_title(f'{title_prefix} Detailed Metrics - Mean Values')
            ax2.grid(True, alpha=0.3, axis='y')
            plt.setp(ax2.get_xticklabels(), rotation=45, ha='right')
            
            # Add value labels on bars
            for bar, mean_val in zip(bars, means):
                height = bar.get_height()
                ax2.text(bar.get_x() + bar.get_width()/2., height + height*0.01,
                        f'{mean_val:.1f}ms', ha='center', va='bottom', fontsize=8)
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.experiment_name}_{filename_prefix}_detailed_comparison_{timestamp}.png"
        filepath = os.path.join(self.data_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"💾 {title_prefix} detailed comparison plot saved: {filename}")
        
        return fig
    
    def create_detailed_pie_chart(self, detailed_data, metric_colors, title_prefix, filename_prefix):
        """Create pie chart showing contribution of each detailed metric"""
        if not detailed_data:
            return None
        
        # Calculate total time spent in each metric
        total_times = {}
        for metric_name, data in detailed_data.items():
            if len(data['time_ms']) > 0:
                total_times[metric_name] = np.sum(data['time_ms'])
        
        if not total_times:
            return None
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Pie chart for total time contribution
        labels = [name.replace('_', ' ').title() for name in total_times.keys()]
        sizes = list(total_times.values())
        colors = [metric_colors.get(name, '#1f77b4') for name in total_times.keys()]
        
        wedges, texts, autotexts = ax1.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%',
                                          startangle=90, textprops={'fontsize': 8})
        ax1.set_title(f'{title_prefix} Time Distribution\n(Total Time Contribution)')
        
        # Pie chart for average time per execution
        avg_times = {}
        for metric_name, data in detailed_data.items():
            if len(data['time_ms']) > 0:
                avg_times[metric_name] = np.mean(data['time_ms'])
        
        labels_avg = [name.replace('_', ' ').title() for name in avg_times.keys()]
        sizes_avg = list(avg_times.values())
        
        wedges2, texts2, autotexts2 = ax2.pie(sizes_avg, labels=labels_avg, colors=colors, 
                                             autopct='%1.1f%%', startangle=90, textprops={'fontsize': 8})
        ax2.set_title(f'{title_prefix} Time Distribution\n(Average Time per Execution)')
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.experiment_name}_{filename_prefix}_detailed_pie_chart_{timestamp}.png"
        filepath = os.path.join(self.data_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"💾 {title_prefix} detailed pie chart saved: {filename}")
        
        return fig
    
    def create_time_series_plots(self):
        """Create time series plots"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
        # Global planning time series
        if self.global_data and len(self.global_data['planning_time_ms']) > 0:
            ax1.plot(self.global_data['sim_time'], self.global_data['planning_time_ms'], 
                    'o-', linewidth=1, markersize=3, alpha=0.7, label='Global Planning')
            ax1.set_ylabel('Planning Time (ms)')
            ax1.set_xlabel('Simulation Time (s)')
            ax1.set_title('Global Planning Time')
            ax1.grid(True, alpha=0.3)
            ax1.legend()
            
            # Add mean line
            mean_global = np.mean(self.global_data['planning_time_ms'])
            ax1.axhline(y=mean_global, color='red', linestyle='--', alpha=0.7, 
                       label=f'Mean: {mean_global:.2f}ms')
            ax1.legend()
        
        # Local planning time series
        if self.local_data and len(self.local_data['planning_time_ms']) > 0:
            ax2.plot(self.local_data['sim_time'], self.local_data['planning_time_ms'], 
                    'o-', linewidth=1, markersize=3, alpha=0.7, color='orange', label='Local Planning')
            ax2.set_ylabel('Planning Time (ms)')
            ax2.set_xlabel('Simulation Time (s)')
            ax2.set_title('Local Planning Time')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
            
            # Add mean line
            mean_local = np.mean(self.local_data['planning_time_ms'])
            ax2.axhline(y=mean_local, color='red', linestyle='--', alpha=0.7, 
                       label=f'Mean: {mean_local:.2f}ms')
            ax2.legend()
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.experiment_name}_time_series_{timestamp}.png"
        filepath = os.path.join(self.data_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"💾 Time series plot saved: {filename}")
        
        return fig
    
    def create_histogram_plots(self):
        """Create histogram plots"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Global planning histogram
        if self.global_data and len(self.global_data['planning_time_ms']) > 0:
            global_times = self.global_data['planning_time_ms']
            ax1.hist(global_times, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
            ax1.axvline(np.mean(global_times), color='red', linestyle='--', linewidth=2, 
                       label=f'Mean: {np.mean(global_times):.2f}ms')
            ax1.axvline(np.median(global_times), color='green', linestyle='--', linewidth=2, 
                       label=f'Median: {np.median(global_times):.2f}ms')
            ax1.set_xlabel('Planning Time (ms)')
            ax1.set_ylabel('Frequency')
            ax1.set_title('Global Planning Time Distribution')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
        
        # Local planning histogram
        if self.local_data and len(self.local_data['planning_time_ms']) > 0:
            local_times = self.local_data['planning_time_ms']
            ax2.hist(local_times, bins=30, alpha=0.7, color='orange', edgecolor='black')
            ax2.axvline(np.mean(local_times), color='red', linestyle='--', linewidth=2, 
                       label=f'Mean: {np.mean(local_times):.2f}ms')
            ax2.axvline(np.median(local_times), color='green', linestyle='--', linewidth=2, 
                       label=f'Median: {np.median(local_times):.2f}ms')
            ax2.set_xlabel('Planning Time (ms)')
            ax2.set_ylabel('Frequency')
            ax2.set_title('Local Planning Time Distribution')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.experiment_name}_histograms_{timestamp}.png"
        filepath = os.path.join(self.data_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"💾 Histogram plot saved: {filename}")
        
        return fig
    
    def create_comparison_plot(self):
        """Create comparison boxplot"""
        if (not self.global_data or len(self.global_data['planning_time_ms']) == 0 or 
            not self.local_data or len(self.local_data['planning_time_ms']) == 0):
            print("⚠️ Need both global and local data for comparison plot")
            return None
        
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        
        # Prepare data for boxplot - ensure all arrays have the same length
        global_times = self.global_data['planning_time_ms']
        local_times = self.local_data['planning_time_ms']
        
        # Find the maximum length
        max_len = max(len(global_times), len(local_times))
        
        # Pad arrays with NaN values to make them the same length
        global_padded = np.full(max_len, np.nan)
        local_padded = np.full(max_len, np.nan)
        
        global_padded[:len(global_times)] = global_times
        local_padded[:len(local_times)] = local_times
        
        data_to_plot = [global_padded, local_padded]
        labels = ['Global Planning', 'Local Planning']
        
        box_plot = ax.boxplot(data_to_plot, labels=labels, patch_artist=True)
        
        # Customize colors
        colors = ['skyblue', 'orange']
        for patch, color in zip(box_plot['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
        
        ax.set_ylabel('Planning Time (ms)')
        ax.set_title('Planning Time Comparison')
        ax.grid(True, alpha=0.3)
        
        # Add statistics text
        global_mean = np.mean(self.global_data['planning_time_ms'])
        local_mean = np.mean(self.local_data['planning_time_ms'])
        
        ax.text(0.02, 0.98, f'Global Mean: {global_mean:.2f}ms\nLocal Mean: {local_mean:.2f}ms', 
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.experiment_name}_comparison_{timestamp}.png"
        filepath = os.path.join(self.data_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"💾 Comparison plot saved: {filename}")
        
        return fig
    
    def analyze_and_plot(self, show_plots=True):
        """Main analysis function"""
        if not self.load_data():
            return False
        
        # Calculate and print statistics
        self.calculate_statistics()
        
        # Create plots
        print(f"\n📈 Generating plots...")
        fig1 = self.create_time_series_plots()
        fig2 = self.create_histogram_plots()
        fig3 = self.create_comparison_plot()
        
        # Create detailed plots if data is available
        if self.global_detailed_data:
            print(f"📈 Generating global detailed plots...")
            fig4 = self.create_detailed_time_series_plot(
                self.global_detailed_data, self.global_metric_colors, 
                "Global Planning", "global_detailed"
            )
            fig5 = self.create_detailed_comparison_plot(
                self.global_detailed_data, self.global_metric_colors,
                "Global Planning", "global_detailed"
            )
            fig6 = self.create_detailed_pie_chart(
                self.global_detailed_data, self.global_metric_colors,
                "Global Planning", "global_detailed"
            )
        
        # Create detailed local plots if data is available
        if self.local_detailed_data:
            print(f"📈 Generating local detailed plots...")
            fig7 = self.create_detailed_time_series_plot(
                self.local_detailed_data, self.local_metric_colors,
                "Local Planning", "local_detailed"
            )
            fig8 = self.create_detailed_comparison_plot(
                self.local_detailed_data, self.local_metric_colors,
                "Local Planning", "local_detailed"
            )
            fig9 = self.create_detailed_pie_chart(
                self.local_detailed_data, self.local_metric_colors,
                "Local Planning", "local_detailed"
            )
        
        if show_plots:
            plt.show()
        else:
            plt.close('all')
        
        return True

def main():
    parser = argparse.ArgumentParser(description='Analyze EPIC planning time data')
    parser.add_argument('--data_dir', type=str, default=os.path.expanduser('~/catkin_ws/data'),
                       help='Directory containing CSV data files')
    parser.add_argument('--experiment', type=str, default='epic_planning',
                       help='Experiment name prefix')
    parser.add_argument('--no_show', action='store_true',
                       help='Do not show plots (just save them)')
    
    args = parser.parse_args()
    
    print(f"🔍 Analyzing planning time data...")
    print(f"📁 Data directory: {args.data_dir}")
    print(f"🎯 Experiment: {args.experiment}")
    
    analyzer = PlanningTimeAnalyzer(args.data_dir, args.experiment)
    
    if analyzer.analyze_and_plot(show_plots=not args.no_show):
        print(f"\n✅ Analysis complete!")
    else:
        print(f"\n❌ Analysis failed!")

if __name__ == '__main__':
    main() 