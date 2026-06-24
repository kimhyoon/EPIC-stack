#!/usr/bin/env python3
"""
Plot batch experiment results.

Usage: python3 plot_experiment_results.py [experiment_dir]
  experiment_dir: path to ~/.ros/epic_experiments/YYYYMMDD_HHMMSS/
  If omitted, uses the most recent experiment directory.
"""

import os
import sys
import csv
import glob
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def load_timeseries(filepath):
    """Load a 2-column timeseries CSV (time, value)."""
    times, values = [], []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            times.append(float(row[0]))
            values.append(float(row[1]))
    return np.array(times), np.array(values)


def load_exploration_timeseries(filepath):
    """Load exploration CSV (time, cells, rate)."""
    times, rates = [], []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            times.append(float(row[0]))
            rates.append(float(row[2]))
    return np.array(times), np.array(rates)


def load_summary(filepath):
    """Load summary CSV."""
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        values = next(reader)
    return dict(zip(header, values))


def interpolate_to_common_time(all_times, all_values, dt=0.5):
    """Interpolate all iterations to a common time axis for averaging."""
    max_time = max(t[-1] for t in all_times if len(t) > 0)
    common_t = np.arange(0, max_time, dt)
    interpolated = []
    for t, v in zip(all_times, all_values):
        if len(t) < 2:
            continue
        interp_v = np.interp(common_t, t, v, left=v[0], right=v[-1])
        interpolated.append(interp_v)
    return common_t, interpolated


def plot_timeseries_ax(ax, all_times, all_values, ylabel, title, color='C0', mean_label_fmt=None):
    """Plot individual iterations (thin, transparent) and mean (bold)."""
    # Interpolate for mean computation
    common_t, interpolated = interpolate_to_common_time(all_times, all_values)

    # Individual iterations - use colormap for distinguishable colors
    cmap = plt.cm.tab10 if len(all_times) <= 10 else plt.cm.tab20
    for i, (t, v) in enumerate(zip(all_times, all_values)):
        c = cmap(i % cmap.N)
        ax.plot(t, v, color=c, alpha=0.3, linewidth=0.7, label=f'iter {i+1:02d}')

    # Mean line
    if interpolated:
        mean_v = np.mean(interpolated, axis=0)
        ax.plot(common_t, mean_v, color='black', alpha=0.9, linewidth=2.5, label='mean')

        # Average value text
        overall_mean = np.mean(mean_v)
        if mean_label_fmt:
            text = mean_label_fmt.format(overall_mean)
        else:
            text = f'avg: {overall_mean:.4f}'
        ax.text(0.98, 0.95, text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc='upper left', fontsize=6, ncol=2)
    ax.grid(True, alpha=0.3)


def main():
    # Find experiment directory
    if len(sys.argv) > 1:
        exp_dir = sys.argv[1]
    else:
        # Look in package experiments dir first, then fallback to ~/.ros
        pkg_exp = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'experiments')
        exp_dirs = sorted(glob.glob(os.path.join(pkg_exp, '*')))
        if not exp_dirs:
            exp_dirs = sorted(glob.glob(os.path.expanduser('~/.ros/epic_experiments/*')))
        if not exp_dirs:
            print("No experiment directories found in ~/.ros/epic_experiments/")
            sys.exit(1)
        exp_dir = exp_dirs[-1]
        print(f"Using most recent experiment: {exp_dir}")

    # Find all iteration directories
    iter_dirs = sorted(glob.glob(os.path.join(exp_dir, 'iter_*')))
    if not iter_dirs:
        print(f"No iteration directories found in {exp_dir}")
        sys.exit(1)

    print(f"Found {len(iter_dirs)} iterations")

    # Load all data
    speed_times, speed_values = [], []
    error_times, error_values = [], []
    expl_times, expl_values = [], []
    finish_positions = []  # (x, y)
    origin_positions = []  # (x, y)
    summaries = []

    for iter_dir in iter_dirs:
        # Speed
        speed_file = os.path.join(iter_dir, 'speed_timeseries.csv')
        if os.path.exists(speed_file):
            t, v = load_timeseries(speed_file)
            speed_times.append(t)
            speed_values.append(v)

        # Tracking error
        error_file = os.path.join(iter_dir, 'tracking_error_timeseries.csv')
        if os.path.exists(error_file):
            t, v = load_timeseries(error_file)
            error_times.append(t)
            error_values.append(v)

        # Exploration rate
        expl_file = os.path.join(iter_dir, 'exploration_timeseries.csv')
        if os.path.exists(expl_file):
            t, v = load_exploration_timeseries(expl_file)
            expl_times.append(t)
            expl_values.append(v)

        # Summary
        summary_file = os.path.join(iter_dir, 'summary.csv')
        if os.path.exists(summary_file):
            s = load_summary(summary_file)
            summaries.append(s)
            finish_positions.append((float(s['finish_x']), float(s['finish_y'])))
            origin_positions.append((float(s['origin_x']), float(s['origin_y'])))

    n_iters = len(iter_dirs)

    def save_fig(fig, name):
        path = os.path.join(exp_dir, f'{name}.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f"  Saved: {path}")
        plt.close(fig)

    # 1. Speed over time
    fig1, ax1 = plt.subplots(figsize=(10, 6))
    fig1.suptitle(f'Flight Speed over Time ({n_iters} iterations)', fontsize=13, fontweight='bold')
    plot_timeseries_ax(ax1, speed_times, speed_values,
                       'Speed (m/s)', '',
                       color='#2196F3', mean_label_fmt='avg: {:.3f} m/s')
    save_fig(fig1, 'flight_speed')

    # 2. Tracking error over time
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    fig2.suptitle(f'Path Tracking Error over Time ({n_iters} iterations)', fontsize=13, fontweight='bold')
    plot_timeseries_ax(ax2, error_times, error_values,
                       'Tracking Error (m)', '',
                       color='#F44336', mean_label_fmt='RMSE: {:.4f} m')
    save_fig(fig2, 'tracking_error')

    # 3. Exploration rate over time
    fig3, ax3 = plt.subplots(figsize=(10, 6))
    fig3.suptitle(f'Exploration Rate over Time ({n_iters} iterations)', fontsize=13, fontweight='bold')
    plot_timeseries_ax(ax3, expl_times, expl_values,
                       'Exploration Rate', '',
                       color='#4CAF50', mean_label_fmt='final: {:.2%}')
    save_fig(fig3, 'exploration_rate')

    # 4. RTH finish positions (2D scatter)
    fig4, ax4 = plt.subplots(figsize=(8, 8))
    fig4.suptitle(f'RTH Finish Positions ({n_iters} iterations)', fontsize=13, fontweight='bold')
    if finish_positions:
        fx = [p[0] for p in finish_positions]
        fy = [p[1] for p in finish_positions]
        ox = origin_positions[0][0] if origin_positions else 0
        oy = origin_positions[0][1] if origin_positions else 0

        ax4.plot(ox, oy, 'k*', markersize=15, label=f'Origin ({ox:.1f}, {oy:.1f})', zorder=5)
        ax4.scatter(fx, fy, c='#F44336', s=60, alpha=0.7, edgecolors='black', linewidth=0.5, zorder=4)
        for i, (x, y) in enumerate(finish_positions):
            ax4.annotate(f'{i+1}', (x, y), textcoords="offset points",
                         xytext=(5, 5), fontsize=7, alpha=0.7)

        circle = plt.Circle((ox, oy), 0.4, fill=False, color='green',
                             linestyle='--', linewidth=1.5, label='400mm threshold')
        ax4.add_patch(circle)

        errors = [np.sqrt((x - ox)**2 + (y - oy)**2) for x, y in finish_positions]
        mean_err = np.mean(errors)
        max_err = np.max(errors)
        success_rate = sum(1 for e in errors if e < 0.4) / len(errors) * 100

        # Mean error circle
        mean_circle = plt.Circle((ox, oy), mean_err, fill=False, color='#FF9800',
                                  linestyle='-', linewidth=1.5,
                                  label=f'Mean error ({mean_err*1000:.1f}mm)')
        ax4.add_patch(mean_circle)

        stats_text = (f'Mean error: {mean_err*1000:.1f} mm\n'
                      f'Max error: {max_err*1000:.1f} mm\n'
                      f'Success rate: {success_rate:.0f}%\n'
                      f'({sum(1 for e in errors if e < 0.4)}/{len(errors)})')
        ax4.text(0.98, 0.95, stats_text, transform=ax4.transAxes, fontsize=9,
                 verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

        all_x = fx + [ox]
        all_y = fy + [oy]
        margin = max(0.5, max(max(all_x) - min(all_x), max(all_y) - min(all_y)) * 0.3)
        ax4.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax4.set_ylim(min(all_y) - margin, max(all_y) + margin)
        ax4.set_aspect('equal')

    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.legend(loc='upper left', fontsize=8)
    ax4.grid(True, alpha=0.3)
    save_fig(fig4, 'rth_positions')

    print(f"\nAll plots saved to: {exp_dir}/")


if __name__ == '__main__':
    main()
