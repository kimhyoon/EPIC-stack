#!/usr/bin/env python3

"""Render the report-style 3D and per-axis tracking error figure."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_tracking_error(path):
    columns = {
        "t_rel": [],
        "err_x": [],
        "err_y": [],
        "err_z": [],
        "err_norm": [],
    }
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = set(columns) - set(reader.fieldnames or [])
        if missing:
            raise ValueError("missing CSV columns: " + ", ".join(sorted(missing)))
        for row in reader:
            for name in columns:
                columns[name].append(float(row[name]))
    if not columns["t_rel"]:
        raise ValueError("tracking error CSV has no samples")
    return columns


def rmse(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def set_report_style():
    for style in ("seaborn-v0_8-whitegrid", "seaborn-whitegrid"):
        try:
            plt.style.use(style)
            return
        except OSError:
            continue


def render(input_csv, output_png):
    data = load_tracking_error(input_csv)
    set_report_style()

    time_s = data["t_rel"]
    error_3d_mm = [value * 1000.0 for value in data["err_norm"]]
    error_x_mm = [value * 1000.0 for value in data["err_x"]]
    error_y_mm = [value * 1000.0 for value in data["err_y"]]
    error_z_mm = [value * 1000.0 for value in data["err_z"]]

    rmse_3d_mm = rmse(error_3d_mm)
    rmse_x_mm = rmse(error_x_mm)
    rmse_y_mm = rmse(error_y_mm)
    rmse_z_mm = rmse(error_z_mm)

    figure, axes = plt.subplots(1, 2, figsize=(14, 5))

    axes[0].plot(time_s, error_3d_mm, color="blue", linewidth=0.7)
    axes[0].axhline(
        rmse_3d_mm,
        color="red",
        linewidth=1.2,
        label="RMSE: %.1f mm" % rmse_3d_mm,
    )
    axes[0].set_title("3D Position Error over Time")
    axes[0].set_xlabel("Time (seconds)")
    axes[0].set_ylabel("Error (mm)")
    axes[0].legend(loc="upper right")

    axes[1].plot(
        time_s, error_x_mm, color="red", linewidth=0.7,
        label="X - RMSE: %.1f mm" % rmse_x_mm)
    axes[1].plot(
        time_s, error_y_mm, color="green", linewidth=0.7,
        label="Y - RMSE: %.1f mm" % rmse_y_mm)
    axes[1].plot(
        time_s, error_z_mm, color="blue", linewidth=0.7,
        label="Z - RMSE: %.1f mm" % rmse_z_mm)
    axes[1].set_title("Position Error by Axis")
    axes[1].set_xlabel("Time (seconds)")
    axes[1].set_ylabel("Error (mm)")
    axes[1].legend(loc="upper right")

    figure.tight_layout(w_pad=3.0)
    output_png.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_png, dpi=150, bbox_inches="tight")
    plt.close(figure)

    print(
        "tracking RMSE [mm]: 3D=%.1f X=%.1f Y=%.1f Z=%.1f" %
        (rmse_3d_mm, rmse_x_mm, rmse_y_mm, rmse_z_mm))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_png", type=Path)
    args = parser.parse_args()
    render(args.input_csv, args.output_png)


if __name__ == "__main__":
    main()
