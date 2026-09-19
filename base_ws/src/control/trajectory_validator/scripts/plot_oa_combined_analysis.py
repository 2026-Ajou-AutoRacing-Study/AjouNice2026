#!/usr/bin/env python3
"""Combine top-down OA trajectory and station-aligned tracking metrics."""

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from plot_oa_reference_tracking import (
    cumulative_station,
    find_oa_boundaries,
    interpolate,
    read_csv,
    reference_curvature,
)
from plot_oa_vehicle_trajectory import COLORS, LABELS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tracking", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    tracking = read_csv(args.tracking)
    reference = read_csv(args.reference)
    ref_station = cumulative_station(reference)
    ref_curvature = reference_curvature(reference, ref_station)
    boundaries = find_oa_boundaries(ref_station, ref_curvature)
    required_tracking_fields = (
        "matched_gt_station_m",
        "ego_x_m",
        "ego_y_m",
        "ego_velocity_mps",
        "ego_yaw_rate_rps",
        "pp_target_curvature_1pm",
        "lateral_error_m",
        "heading_error_rad",
    )
    valid = [
        row for row in tracking
        if row.get("control_pair_valid", "1") != "0"
        and all(row.get(field, "").strip() for field in required_tracking_fields)
    ]

    station = [float(row["matched_gt_station_m"]) for row in valid]
    controller = [float(row["pp_target_curvature_1pm"]) for row in valid]
    realized = [
        float(row["ego_yaw_rate_rps"]) / max(float(row["ego_velocity_mps"]), 0.5)
        for row in valid
    ]
    lateral = [float(row["lateral_error_m"]) for row in valid]
    heading = [math.degrees(float(row["heading_error_rad"])) for row in valid]
    speed = [float(row["ego_velocity_mps"]) for row in valid]
    realized_lateral_acceleration = [
        float(row["ego_velocity_mps"]) * float(row["ego_yaw_rate_rps"])
        for row in valid
    ]
    reference_speed = [float(row["velocity_mps"]) for row in reference]
    reference_lateral_acceleration = [
        velocity * velocity * curvature
        for velocity, curvature in zip(reference_speed, ref_curvature)
    ]

    figure, axes = plt.subplots(
        6, 1, figsize=(17, 19),
        gridspec_kw={"height_ratios": [1.55, 1.0, 0.85, 0.85, 0.75, 0.8]},
        sharex=True,
    )
    scenario = args.tracking.parent.parent.name
    run_id = args.tracking.parent.name
    figure.suptitle(
        f"Obstacle-Avoidance Trajectory & Tracking Analysis\n{scenario} / {run_id}",
        fontsize=19, fontweight="bold",
    )

    origin_x = float(reference[0]["x"])
    origin_y = float(reference[0]["y"])
    origin_yaw = float(reference[0]["yaw"])
    normal_x = -math.sin(origin_yaw)
    normal_y = math.cos(origin_yaw)
    ref_lateral_position = [
        normal_x * (float(row["x"]) - origin_x) +
        normal_y * (float(row["y"]) - origin_y)
        for row in reference
    ]
    actual_lateral_position = [
        normal_x * (float(row["ego_x_m"]) - origin_x) +
        normal_y * (float(row["ego_y_m"]) - origin_y)
        for row in valid
    ]
    for index, (start, end) in enumerate(zip(boundaries, boundaries[1:])):
        ids = [i for i, value in enumerate(ref_station) if start <= value <= end]
        axes[0].plot(
            [ref_station[i] for i in ids], [ref_lateral_position[i] for i in ids],
            color=COLORS[index], linewidth=3.3, label=LABELS[index], zorder=2,
        )
    axes[0].plot(
        station, actual_lateral_position, color="#d946aa", linewidth=2.0,
        label="Actual rear-axle trajectory", zorder=3,
    )
    connector_stride = max(1, len(valid) // 35)
    for index in range(0, len(valid), connector_stride):
        reference_x = interpolate(station[index], ref_station, ref_lateral_position)
        axes[0].plot(
            [station[index], station[index]],
            [reference_x, actual_lateral_position[index]],
            color="#64748b", alpha=0.35, linewidth=0.7, zorder=1,
        )
    axes[0].axhline(0.0, color="#334155", linestyle="--", linewidth=1.0,
                    label="Original straight = 0 m")
    axes[0].set_title("Reference and actual trajectory relative to the original straight")
    axes[0].set_ylabel("Lateral offset from straight (m)")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)

    metric_axes = axes[1:]
    for axis in metric_axes:
        for index, (start, end) in enumerate(zip(boundaries, boundaries[1:])):
            axis.axvspan(start, end, color=COLORS[index], alpha=0.12, zorder=0)
        axis.axhline(0.0, color="#475569", linewidth=0.8)
        axis.grid(True, alpha=0.23)

    # Draw the reference curvature on its original station axis. Sampling it at
    # the ego vehicle's matched station distorts the curve when nearest-point
    # association stalls or jumps after a large tracking error.
    axes[1].plot(
        ref_station, ref_curvature, color="#1368ce", linewidth=2.1,
        label="Reference κ (original path)",
    )
    axes[1].plot(station, controller, color="#f59e0b", linewidth=1.6, label="Controller target κ")
    axes[1].plot(station, realized, color="#e4572e", linewidth=1.5, label="Realized κ = yaw rate / speed")
    axes[1].set_ylabel("Curvature κ (1/m)")
    axes[1].legend(loc="lower right", ncol=3)

    axes[2].plot(station, lateral, color="#00747c", linewidth=1.8)
    axes[2].axhline(0.30, color="#f59e0b", linestyle="--", linewidth=1.0)
    axes[2].axhline(-0.30, color="#f59e0b", linestyle="--", linewidth=1.0)
    axes[2].set_ylabel("Lateral error (m)")

    axes[3].plot(station, heading, color="#7b2cbf", linewidth=1.8)
    axes[3].axhline(5.0, color="#ef4444", linestyle="--", linewidth=1.0)
    axes[3].axhline(-5.0, color="#ef4444", linestyle="--", linewidth=1.0)
    axes[3].set_ylabel("Heading error (deg)")

    axes[4].plot(station, speed, color="#2f855a", linewidth=1.8)
    axes[4].set_ylabel("Speed (m/s)")

    axes[5].plot(
        ref_station, reference_lateral_acceleration, color="#1368ce", linewidth=2.0,
        label="Reference $a_y = v_{ref}^2 \\kappa_{ref}$",
    )
    axes[5].plot(
        station, realized_lateral_acceleration, color="#dc2626", linewidth=1.6,
        label="Realized $a_y = v \\dot{\\psi}$",
    )
    axes[5].axhline(4.0, color="#f59e0b", linestyle="--", linewidth=1.0,
                    label="Example bound $|a_y| = 4.0$ m/s²")
    axes[5].axhline(-4.0, color="#f59e0b", linestyle="--", linewidth=1.0)
    axes[5].set_ylabel("Lateral accel. (m/s²)")
    axes[5].set_xlabel("Reference-path station (m)")
    axes[5].legend(loc="lower right", ncol=3, fontsize=8)

    centers = [(start + end) / 2.0 for start, end in zip(boundaries, boundaries[1:])]
    for center, label in zip(centers, LABELS):
        axes[1].text(
            center, 0.97, label, transform=axes[1].get_xaxis_transform(),
            ha="center", va="top", fontsize=9, fontweight="bold",
            bbox={"boxstyle": "round,pad=0.2", "facecolor": "white", "alpha": 0.8,
                  "edgecolor": "none"},
        )

    max_lat = max(abs(value) for value in lateral)
    max_heading = max(abs(value) for value in heading)
    figure.text(
        0.99, 0.008,
        f"Actual samples: {len(valid)}  |  Max |lateral|: {max_lat:.3f} m  |  "
        f"Max |heading|: {max_heading:.2f}°  |  No vehicle footprint or estimated obstacle pose",
        ha="right", fontsize=9,
    )
    figure.tight_layout(rect=(0.025, 0.025, 0.99, 0.955))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=175, bbox_inches="tight")
    print(args.output)


if __name__ == "__main__":
    main()
