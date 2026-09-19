#!/usr/bin/env python3
"""Plot fixed-reference and realized tracking behavior for one OA run."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def cumulative_station(rows):
    station = [0.0]
    for first, second in zip(rows, rows[1:]):
        dx = float(second["x"]) - float(first["x"])
        dy = float(second["y"]) - float(first["y"])
        station.append(station[-1] + math.hypot(dx, dy))
    return station


def unwrap(values):
    output = list(values)
    for index in range(1, len(output)):
        while output[index] - output[index - 1] > math.pi:
            output[index] -= 2.0 * math.pi
        while output[index] - output[index - 1] < -math.pi:
            output[index] += 2.0 * math.pi
    return output


def reference_curvature(rows, station, half_window=5):
    yaw = unwrap([float(row["yaw"]) for row in rows])
    curvature = []
    for index in range(len(rows)):
        begin = max(0, index - half_window)
        end = min(len(rows) - 1, index + half_window)
        ds = station[end] - station[begin]
        curvature.append((yaw[end] - yaw[begin]) / ds if ds > 0.0 else 0.0)
    return curvature


def interpolate(x, xp, fp):
    if x <= xp[0]:
        return fp[0]
    if x >= xp[-1]:
        return fp[-1]
    low, high = 0, len(xp) - 1
    while high - low > 1:
        middle = (low + high) // 2
        if xp[middle] <= x:
            low = middle
        else:
            high = middle
    ratio = (x - xp[low]) / (xp[high] - xp[low])
    return fp[low] + ratio * (fp[high] - fp[low])


def find_oa_boundaries(station, curvature, threshold=0.002):
    active = [i for i, value in enumerate(curvature) if abs(value) >= threshold]
    if not active:
        raise ValueError("reference path has no obstacle-avoidance curvature region")
    starts = [station[active[0]]]
    previous_sign = math.copysign(1.0, curvature[active[0]])
    for index in active[1:]:
        sign = math.copysign(1.0, curvature[index])
        if sign != previous_sign:
            starts.append(station[index])
            previous_sign = sign
    if len(starts) != 3:
        raise ValueError("expected two curvature sign changes in fixed OA reference")
    return [station[0], starts[0], starts[1], starts[2], station[active[-1]], station[-1]]


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

    valid = [row for row in tracking if row.get("control_pair_valid", "1") != "0"]
    station = [float(row["matched_gt_station_m"]) for row in valid]
    time = [float(row["elapsed_time_s"]) for row in valid]
    lateral = [float(row["lateral_error_m"]) for row in valid]
    heading = [math.degrees(float(row["heading_error_rad"])) for row in valid]
    speed = [float(row["ego_velocity_mps"]) for row in valid]
    controller_kappa = [float(row["pp_target_curvature_1pm"]) for row in valid]
    actual_kappa = [
        float(row["ego_yaw_rate_rps"]) / max(float(row["ego_velocity_mps"]), 0.5)
        for row in valid
    ]

    colors = ["#e8f1fb", "#fff0dc", "#e7f5e7", "#f0e8fa", "#e8f4f5"]
    labels = ["A  Approach", "B  Entry", "C  Avoid", "D  Return", "E  Stabilize"]
    fig, axes = plt.subplots(4, 1, figsize=(16, 11), sharex=True)
    run_id = args.tracking.parent.name
    scenario_name = args.tracking.parent.parent.name
    fig.suptitle(
        "Fixed-Reference Obstacle Avoidance — Actual Run\n"
        f"{scenario_name} / {run_id}",
        fontsize=17,
        fontweight="bold",
    )

    for axis in axes:
        for index, (start, end) in enumerate(zip(boundaries, boundaries[1:])):
            axis.axvspan(start, end, color=colors[index], alpha=0.65, zorder=0)
        axis.grid(True, color="#9aa0a6", alpha=0.25)
        axis.axhline(0.0, color="#4b5563", linewidth=0.8)

    # Keep the reference curve on its native station axis. Using the ego
    # matched station can skip or flatten parts of the reference after the
    # nearest-point association jumps during a large path departure.
    axes[0].plot(
        ref_station, ref_curvature, color="#1368ce", linewidth=2.2,
        label="Ideal reference curvature (original path)",
    )
    axes[0].plot(
        station, controller_kappa, color="#f59e0b", linewidth=1.6, alpha=0.9,
        label="Controller target curvature (pp_target_curvature)",
    )
    axes[0].plot(
        station, actual_kappa, color="#e4572e", linewidth=1.5, alpha=0.9,
        label="Realized vehicle curvature = yaw rate / speed",
    )
    axes[0].set_ylabel("Curvature κ (1/m)")
    axes[0].legend(loc="upper right", ncol=2)

    axes[1].plot(station, lateral, color="#006d77", linewidth=1.8, label="Signed lateral error")
    axes[1].axhline(0.30, color="#f59e0b", linestyle="--", linewidth=1.0)
    axes[1].axhline(-0.30, color="#f59e0b", linestyle="--", linewidth=1.0)
    axes[1].set_ylabel("Lateral error (m)")
    axes[1].legend(loc="upper right")

    axes[2].plot(station, heading, color="#7b2cbf", linewidth=1.8, label="Signed heading error")
    axes[2].axhline(5.0, color="#ef4444", linestyle="--", linewidth=1.0)
    axes[2].axhline(-5.0, color="#ef4444", linestyle="--", linewidth=1.0)
    axes[2].set_ylabel("Heading error (deg)")
    axes[2].legend(loc="upper right")

    axes[3].plot(station, speed, color="#2f855a", linewidth=1.8, label="Actual speed")
    axes[3].set_ylabel("Speed (m/s)")
    axes[3].set_xlabel("Matched reference-path station (m)")
    axes[3].legend(loc="upper right")

    centers = [(start + end) / 2.0 for start, end in zip(boundaries, boundaries[1:])]
    for center, label in zip(centers, labels):
        axes[0].text(
            center, 0.96, label, transform=axes[0].get_xaxis_transform(),
            ha="center", va="top", fontsize=9, fontweight="bold",
            bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "alpha": 0.82,
                  "edgecolor": "none"},
        )

    max_lat_index = max(range(len(lateral)), key=lambda i: abs(lateral[i]))
    max_heading_index = max(range(len(heading)), key=lambda i: abs(heading[i]))
    summary = (
        f"Actual recorded samples: {len(valid)}\n"
        f"Max |lateral|: {abs(lateral[max_lat_index]):.3f} m at s={station[max_lat_index]:.1f} m\n"
        f"Max |heading|: {abs(heading[max_heading_index]):.2f}° at s={station[max_heading_index]:.1f} m\n"
        f"Run duration: {time[-1]:.2f} s"
    )
    fig.text(0.985, 0.015, summary, ha="right", va="bottom", fontsize=9,
             bbox={"boxstyle": "round,pad=0.5", "facecolor": "white", "edgecolor": "#64748b"})

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0.035, 0.06, 0.99, 0.92))
    fig.savefig(args.output, dpi=170, bbox_inches="tight")
    print(args.output)
    print("A-E boundaries:", ", ".join(f"{value:.2f}" for value in boundaries))


if __name__ == "__main__":
    main()
