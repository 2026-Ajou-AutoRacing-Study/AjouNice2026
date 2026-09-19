#!/usr/bin/env python3
"""Create top-down reference/actual vehicle-trajectory plots for an OA run."""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from plot_oa_reference_tracking import (
    cumulative_station,
    find_oa_boundaries,
    read_csv,
    reference_curvature,
)


COLORS = ["#4c78a8", "#f59e0b", "#54a24b", "#9c6ade", "#72b7b2"]
LABELS = ["A Approach", "B Entry", "C Avoid", "D Return", "E Stabilize"]


def interpolate(value, station, values):
    if value <= station[0]:
        return values[0]
    if value >= station[-1]:
        return values[-1]
    low, high = 0, len(station) - 1
    while high - low > 1:
        middle = (low + high) // 2
        if station[middle] <= value:
            low = middle
        else:
            high = middle
    ratio = (value - station[low]) / (station[high] - station[low])
    return values[low] + ratio * (values[high] - values[low])


def plot_panel(axis, reference, ref_station, tracking, boundaries, zoom=False):
    ref_map_x = [float(row["x"]) for row in reference]
    ref_map_y = [float(row["y"]) for row in reference]
    ego_map_x = [float(row["ego_x_m"]) for row in tracking]
    ego_map_y = [float(row["ego_y_m"]) for row in tracking]
    # Rotate the map view so the predominantly southbound route reads left-to-right.
    ref_x = [-value for value in ref_map_y]
    ref_y = list(ref_map_x)
    ego_x = [-value for value in ego_map_y]
    ego_y = list(ego_map_x)
    ego_s = [float(row["matched_gt_station_m"]) for row in tracking]

    for index, (start, end) in enumerate(zip(boundaries, boundaries[1:])):
        ids = [i for i, value in enumerate(ref_station) if start <= value <= end]
        axis.plot(
            [ref_x[i] for i in ids], [ref_y[i] for i in ids], color=COLORS[index],
            linewidth=3.2, label=LABELS[index], zorder=2,
        )

    points = list(zip(ego_x, ego_y))
    segments = [[points[i], points[i + 1]] for i in range(len(points) - 1)]
    collection = LineCollection(
        segments, cmap="plasma", linewidth=2.0, zorder=3, label="Actual trajectory"
    )
    collection.set_array(np.arange(len(segments), dtype=float))
    axis.add_collection(collection)

    stride = max(1, len(tracking) // 28)
    for row in tracking[::stride]:
        station = float(row["matched_gt_station_m"])
        rx = interpolate(station, ref_station, ref_x)
        ry = interpolate(station, ref_station, ref_y)
        axis.plot([rx, -float(row["ego_y_m"])], [ry, float(row["ego_x_m"])],
                  color="#64748b", alpha=0.28, linewidth=0.7, zorder=1)

    axis.scatter([ego_x[0]], [ego_y[0]], marker="o", s=70, color="#16a34a", zorder=6, label="Start")
    axis.scatter([ego_x[-1]], [ego_y[-1]], marker="X", s=75, color="#dc2626", zorder=6, label="End")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    axis.set_xlabel("Longitudinal display axis, −Map Y (m)")
    axis.set_ylabel("Lateral display axis, Map X (m)")

    if zoom:
        begin = boundaries[1] - 7.0
        end = boundaries[4] + 7.0
        ids = [i for i, value in enumerate(ref_station) if begin <= value <= end]
        all_x = [ref_x[i] for i in ids]
        all_y = [ref_y[i] for i in ids]
        track_ids = [i for i, value in enumerate(ego_s) if begin <= value <= end]
        all_x += [ego_x[i] for i in track_ids]
        all_y += [ego_y[i] for i in track_ids]
        axis.set_xlim(min(all_x) - 2.0, max(all_x) + 2.0)
        axis.set_ylim(min(all_y) - 2.0, max(all_y) + 2.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tracking", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    tracking = read_csv(args.tracking)
    reference = read_csv(args.reference)
    ref_station = cumulative_station(reference)
    curvature = reference_curvature(reference, ref_station)
    boundaries = find_oa_boundaries(ref_station, curvature)

    figure, axes = plt.subplots(2, 1, figsize=(15, 10), gridspec_kw={"height_ratios": [0.8, 1.2]})
    scenario = args.tracking.parent.parent.name
    run_id = args.tracking.parent.name
    figure.suptitle(
        f"Actual Vehicle Trajectory — {scenario}\n{run_id}", fontsize=17, fontweight="bold"
    )
    plot_panel(axes[0], reference, ref_station, tracking, boundaries, zoom=False)
    plot_panel(axes[1], reference, ref_station, tracking, boundaries, zoom=True)
    axes[0].set_title("Full run")
    axes[1].set_title("Obstacle-avoidance section (zoom)")

    handles, labels = axes[1].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.04),
                  ncol=7, frameon=True)
    figure.text(
        0.99, 0.006,
        "Colored thick line: fixed reference  |  Plasma line: actual rear-axle trajectory\n"
        "Obstacle pose is not present in the recorded run metadata and is therefore not estimated.",
        ha="right", va="bottom", fontsize=9,
    )
    figure.tight_layout(rect=(0.02, 0.10, 0.99, 0.93))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight")
    print(args.output)


if __name__ == "__main__":
    main()
