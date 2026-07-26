#!/usr/bin/env python3
"""Render a lanelet- and speed-annotated K-City variable trajectory."""

import argparse
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import generate_trajectory as generator  # noqa: E402


CAP_COLORS = {
    20.0: "#2563eb",
    30.0: "#06b6d4",
    60.0: "#22c55e",
    140.0: "#dc2626",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize a variable-speed trajectory over the K-City lanelet map"
    )
    parser.add_argument("--config", required=True, help="generator YAML")
    parser.add_argument("--trajectory-csv", required=True, help="generated trajectory CSV")
    parser.add_argument("--output-svg", required=True, help="output SVG")
    parser.add_argument("--output-png", help="optional PNG preview")
    return parser.parse_args()


def resolve_output_path(repository, value, label, suffix):
    path = (repository / str(value)).resolve()
    try:
        path.relative_to(repository)
    except ValueError as error:
        raise ValueError(
            "{} must stay inside repository: {}".format(label, value)
        ) from error
    if path.suffix.lower() != suffix:
        raise ValueError("{} must use a {} filename".format(label, suffix))
    return path


def route_centerline(row):
    points = []
    for token in row["centerline_xy"].split(";"):
        fields = token.split(":")
        if len(fields) < 2:
            continue
        points.append((float(fields[0]), float(fields[1])))
    if len(points) < 2:
        raise ValueError(
            "route order {} has no usable centerline_xy".format(row["order"])
        )
    return points


def path_midpoint_and_tangent(points):
    lengths = [
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    ]
    target = sum(lengths) * 0.5
    traversed = 0.0
    for index, length in enumerate(lengths):
        if traversed + length >= target:
            ratio = (target - traversed) / max(length, 1.0e-9)
            first = points[index]
            second = points[index + 1]
            midpoint = (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
            tangent = (
                (second[0] - first[0]) / max(length, 1.0e-9),
                (second[1] - first[1]) / max(length, 1.0e-9),
            )
            return midpoint, tangent
        traversed += length
    first = points[-2]
    second = points[-1]
    length = max(math.hypot(second[0] - first[0], second[1] - first[1]), 1.0e-9)
    return second, ((second[0] - first[0]) / length, (second[1] - first[1]) / length)


def load_map_boundaries(osm_path, projector, bounds):
    root = ET.parse(str(osm_path)).getroot()
    transformer, origin_easting, origin_northing, _ = projector
    nodes = {}
    for node in root.findall("node"):
        easting, northing = transformer.transform(
            float(node.attrib["lon"]), float(node.attrib["lat"])
        )
        nodes[int(node.attrib["id"])] = (
            easting - origin_easting,
            northing - origin_northing,
        )
    ways = {
        int(way.attrib["id"]): [
            nodes[int(item.attrib["ref"])]
            for item in way.findall("nd")
            if int(item.attrib["ref"]) in nodes
        ]
        for way in root.findall("way")
    }

    minimum_x, maximum_x, minimum_y, maximum_y = bounds
    selected = []
    seen = set()
    for relation in root.findall("relation"):
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in relation.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        for member in relation.findall("member"):
            if member.attrib.get("type") != "way":
                continue
            if member.attrib.get("role") not in ("left", "right", "centerline"):
                continue
            way_id = int(member.attrib["ref"])
            if way_id in seen:
                continue
            points = ways.get(way_id, [])
            if len(points) < 2:
                continue
            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            if (
                max(xs) < minimum_x
                or min(xs) > maximum_x
                or max(ys) < minimum_y
                or min(ys) > maximum_y
            ):
                continue
            selected.append(points)
            seen.add(way_id)
    return selected


def read_trajectory(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError("trajectory must contain at least two rows")
    return rows


def actual_speed_by_order(trajectory_rows):
    values = {}
    for row in trajectory_rows:
        values.setdefault(int(row["route_order"]), []).append(
            float(row["velocity_mps"]) * 3.6
        )
    return {
        order: (min(speeds), max(speeds))
        for order, speeds in values.items()
    }


def acceleration_range(trajectory_rows):
    accelerations = []
    for first, second in zip(trajectory_rows, trajectory_rows[1:]):
        first_xyz = tuple(float(first[key]) for key in ("x", "y", "z"))
        second_xyz = tuple(float(second[key]) for key in ("x", "y", "z"))
        distance = math.dist(first_xyz, second_xyz)
        first_velocity = float(first["velocity_mps"])
        second_velocity = float(second["velocity_mps"])
        accelerations.append(
            (second_velocity**2 - first_velocity**2) / (2.0 * distance)
        )
    return min(accelerations), max(accelerations)


def cap_for_row(row, speed_profile):
    return speed_profile["lanelet_velocity_kph"].get(
        row["lanelet_id"], speed_profile["default_velocity_kph"]
    )


def cap_color(speed):
    if speed in CAP_COLORS:
        return CAP_COLORS[speed]
    return "#64748b"


def pastel(color, amount=0.86):
    red, green, blue = matplotlib.colors.to_rgb(color)
    return (
        red + (1.0 - red) * amount,
        green + (1.0 - green) * amount,
        blue + (1.0 - blue) * amount,
    )


def main():
    arguments = parse_args()
    repository = generator.repository_root()
    config_path = generator.resolve_repo_path(repository, arguments.config, "config")
    trajectory_path = generator.resolve_repo_path(
        repository, arguments.trajectory_csv, "trajectory_csv"
    )
    output_svg = resolve_output_path(
        repository, arguments.output_svg, "output_svg", ".svg"
    )
    output_png = (
        resolve_output_path(
            repository, arguments.output_png, "output_png", ".png"
        )
        if arguments.output_png
        else None
    )

    config = generator.load_config(config_path)
    map_config = generator.require(config, "map", "config")
    anchors = generator.require(config, "anchors", "config")
    trajectory_config = generator.require(config, "trajectory", "config")
    speed_profile = generator.load_speed_profile(trajectory_config)
    if speed_profile is None:
        raise ValueError("visualization requires trajectory.speed_profile")

    route_path = generator.resolve_repo_path(
        repository,
        generator.require(map_config, "route_lanelets_file", "map"),
        "route_lanelets_file",
    )
    osm_path = generator.resolve_repo_path(
        repository,
        generator.require(map_config, "lanelet_map_file", "map"),
        "lanelet_map_file",
    )
    projector_path = generator.resolve_repo_path(
        repository,
        generator.require(map_config, "projector_file", "map"),
        "projector_file",
    )
    route_rows = generator.load_route(route_path)
    chain = generator.find_unique_chain(
        route_rows,
        int(anchors["start_lanelet_id"]),
        int(anchors["end_lanelet_id"]),
        anchors.get("start_route_order"),
        anchors.get("end_route_order"),
    )
    trajectory_rows = read_trajectory(trajectory_path)
    actual_ranges = actual_speed_by_order(trajectory_rows)
    minimum_acceleration, maximum_acceleration = acceleration_range(trajectory_rows)

    route_paths = [route_centerline(row) for row in chain]
    route_x = [point[0] for points in route_paths for point in points]
    route_y = [point[1] for points in route_paths for point in points]
    x_padding = 65.0
    y_padding = 55.0
    bounds = (
        min(route_x) - x_padding,
        max(route_x) + x_padding,
        min(route_y) - y_padding,
        max(route_y) + y_padding,
    )
    projector = generator.parse_projector(projector_path)
    map_boundaries = load_map_boundaries(osm_path, projector, bounds)

    matplotlib.rcParams["svg.fonttype"] = "none"
    matplotlib.rcParams["font.family"] = "DejaVu Sans"
    speed_colormap = LinearSegmentedColormap.from_list(
        "trajectory_speed",
        [
            (0.00, CAP_COLORS[20.0]),
            (0.08, CAP_COLORS[30.0]),
            (0.33, CAP_COLORS[60.0]),
            (0.67, "#f59e0b"),
            (1.00, CAP_COLORS[140.0]),
        ],
    )
    speed_normalization = Normalize(vmin=20.0, vmax=140.0)

    figure = plt.figure(figsize=(18.0, 22.0), facecolor="white")
    layout = GridSpec(
        1,
        2,
        figure=figure,
        width_ratios=(2.65, 1.35),
        left=0.035,
        right=0.985,
        top=0.955,
        bottom=0.035,
        wspace=0.025,
    )
    map_axis = figure.add_subplot(layout[0, 0])
    table_axis = figure.add_subplot(layout[0, 1])

    for points in map_boundaries:
        map_axis.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color="#cbd5e1",
            linewidth=0.35,
            alpha=0.48,
            zorder=1,
        )

    for row, points in zip(chain, route_paths):
        cap = cap_for_row(row, speed_profile)
        map_axis.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color=cap_color(cap),
            linewidth=5.8,
            alpha=0.25,
            solid_capstyle="round",
            zorder=2,
        )

    trajectory_xy = np.asarray(
        [
            (float(row["x"]), float(row["y"]))
            for row in trajectory_rows
        ],
        dtype=float,
    )
    line_segments = np.stack((trajectory_xy[:-1], trajectory_xy[1:]), axis=1)
    segment_speeds = np.asarray(
        [
            0.5
            * (
                float(first["velocity_mps"])
                + float(second["velocity_mps"])
            )
            * 3.6
            for first, second in zip(trajectory_rows, trajectory_rows[1:])
        ]
    )
    speed_line = LineCollection(
        line_segments,
        cmap=speed_colormap,
        norm=speed_normalization,
        linewidths=2.15,
        alpha=0.98,
        zorder=4,
    )
    speed_line.set_array(segment_speeds)
    map_axis.add_collection(speed_line)

    labelled_lanelets = set()
    for row, points in zip(chain, route_paths):
        lanelet_id = row["lanelet_id"]
        order = row["order"]
        cap = cap_for_row(row, speed_profile)
        midpoint, tangent = path_midpoint_and_tangent(points)
        normal = (-tangent[1], tangent[0])
        direction = 1.0 if order % 2 else -1.0
        shift = direction * (10.0 + 3.0 * (order % 3))
        label_point = (
            midpoint[0] + normal[0] * shift,
            midpoint[1] + normal[1] * shift,
        )
        label = "{}\n{} kph".format(lanelet_id, int(round(cap)))
        if lanelet_id in labelled_lanelets:
            label = "#{} {}\n{} kph".format(order, lanelet_id, int(round(cap)))
        labelled_lanelets.add(lanelet_id)
        map_axis.annotate(
            label,
            xy=midpoint,
            xytext=label_point,
            textcoords="data",
            fontsize=6.1,
            color="#0f172a",
            ha="center",
            va="center",
            linespacing=0.92,
            bbox={
                "boxstyle": "round,pad=0.18",
                "facecolor": "white",
                "edgecolor": cap_color(cap),
                "linewidth": 0.65,
                "alpha": 0.90,
            },
            arrowprops={
                "arrowstyle": "-",
                "color": cap_color(cap),
                "linewidth": 0.55,
                "alpha": 0.8,
            },
            zorder=6,
        )

    map_axis.set_xlim(bounds[0], bounds[1])
    map_axis.set_ylim(bounds[2], bounds[3])
    map_axis.set_aspect("equal", adjustable="box")
    map_axis.set_facecolor("#fbfdff")
    map_axis.grid(color="#e2e8f0", linewidth=0.35, alpha=0.55)
    map_axis.set_xlabel("K-City local map X (m)", fontsize=9)
    map_axis.set_ylabel("K-City local map Y (m)", fontsize=9)
    map_axis.tick_params(labelsize=7)
    map_axis.set_title(
        "200889_200889_v — Variable Speed Profile\n"
        "Map label: lanelet ID / cap speed · Route color: actual target speed",
        fontsize=14,
        fontweight="bold",
        loc="left",
        pad=12,
    )

    colorbar = figure.colorbar(
        speed_line,
        ax=map_axis,
        orientation="horizontal",
        fraction=0.025,
        pad=0.025,
        aspect=45,
    )
    colorbar.set_label("Actual target speed (kph)", fontsize=9)
    colorbar.set_ticks([20, 30, 60, 100, 140])
    colorbar.ax.tick_params(labelsize=7)

    cap_handles = [
        Line2D(
            [0],
            [0],
            color=CAP_COLORS[speed],
            linewidth=5.5,
            alpha=0.75,
            label="{} kph cap".format(int(speed)),
        )
        for speed in sorted(CAP_COLORS)
    ]
    map_axis.legend(
        handles=cap_handles,
        loc="lower left",
        fontsize=7,
        frameon=True,
        framealpha=0.92,
        title="Lanelet cap",
        title_fontsize=8,
    )

    table_axis.axis("off")
    table_axis.set_title(
        "Route-order review table",
        fontsize=13,
        fontweight="bold",
        loc="left",
        pad=10,
    )
    table_axis.text(
        0.0,
        0.982,
        "Cap from YAML · Actual range after +{:.1f}/-{:.1f} m/s² envelope\n"
        "Measured CSV acceleration: {:+.3f} .. {:+.3f} m/s²".format(
            speed_profile["max_acceleration_mps2"],
            speed_profile["max_deceleration_mps2"],
            minimum_acceleration,
            maximum_acceleration,
        ),
        transform=table_axis.transAxes,
        fontsize=8.2,
        va="top",
        color="#334155",
    )

    table_rows = []
    row_colors = []
    for row in chain:
        cap = cap_for_row(row, speed_profile)
        actual_minimum, actual_maximum = actual_ranges[row["order"]]
        table_rows.append(
            [
                str(row["order"]),
                str(row["lanelet_id"]),
                "{:.0f}".format(cap),
                "{:.1f}–{:.1f}".format(actual_minimum, actual_maximum),
                row["maneuver"],
            ]
        )
        row_colors.append(pastel(cap_color(cap)))

    review_table = table_axis.table(
        cellText=table_rows,
        colLabels=["#", "Lanelet", "Cap", "Actual kph", "Maneuver"],
        colWidths=[0.07, 0.22, 0.11, 0.24, 0.30],
        cellLoc="center",
        colLoc="center",
        bbox=[0.0, 0.025, 1.0, 0.92],
    )
    review_table.auto_set_font_size(False)
    review_table.set_fontsize(6.8)
    for column in range(5):
        header = review_table[(0, column)]
        header.set_facecolor("#0f172a")
        header.set_text_props(color="white", fontweight="bold")
        header.set_edgecolor("#475569")
        header.set_linewidth(0.55)
    for row_index, color in enumerate(row_colors, start=1):
        for column in range(5):
            cell = review_table[(row_index, column)]
            cell.set_facecolor(color)
            cell.set_edgecolor("#cbd5e1")
            cell.set_linewidth(0.35)
            if column in (0, 1, 2):
                cell.set_text_props(fontweight="bold")

    output_svg.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_svg, format="svg", facecolor="white")
    if output_png is not None:
        output_png.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_png, format="png", dpi=180, facecolor="white")
    plt.close(figure)

    print("speed profile SVG: {}".format(output_svg))
    if output_png is not None:
        print("speed profile PNG: {}".format(output_png))
    print(
        "lanelets={} actual_speed_kph={:.3f}..{:.3f}".format(
            len(chain),
            min(float(row["velocity_mps"]) * 3.6 for row in trajectory_rows),
            max(float(row["velocity_mps"]) * 3.6 for row in trajectory_rows),
        )
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit(
            "visualize_variable_speed_profile: {}".format(error)
        )
