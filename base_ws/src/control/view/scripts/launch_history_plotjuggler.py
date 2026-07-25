#!/usr/bin/env python3
import os
import csv
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import rospy
import yaml


TARGET_COLOR = "#00aa00"
ACTUAL_COLOR = "#0066ff"
ERROR_COLOR = "#ff0000"


PLOTS = (
    ("Velocity: target / actual / error", (
        ("target_velocity_mps", TARGET_COLOR),
        ("ego_velocity_mps", ACTUAL_COLOR),
        ("velocity_error_mps", ERROR_COLOR),
    )),
    ("Steering: command / actual / error", (
        ("cmd_steering_rad", TARGET_COLOR),
        ("actual_steering_rad", ACTUAL_COLOR),
        ("steering_error_rad", ERROR_COLOR),
    )),
    ("Tracking errors", (
        ("lateral_error_m", ERROR_COLOR),
        ("longitudinal_error_m", "#cc0000"),
        ("heading_error_rad", "#ff5555"),
    )),
    ("Matched GT / Ego yaw", (
        ("matched_gt_yaw_rad", TARGET_COLOR),
        ("ego_yaw_rad", ACTUAL_COLOR),
    )),
    ("Command to vehicle response", (
        ("response_velocity_change_mps", ACTUAL_COLOR),
        ("response_steering_change_rad", "#0044cc"),
        ("response_lateral_error_change_m", ERROR_COLOR),
    )),
)


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "base_ws/src/control/trajectory_validator/history").is_dir():
            return parent
    raise RuntimeError("cannot locate AjouNice2026 repository root")


def build_plot_csv(control_path, response_path):
    with control_path.open(newline="") as stream:
        control_reader = csv.DictReader(stream)
        control_fields = list(control_reader.fieldnames or [])
        control_rows = list(control_reader)
    with response_path.open(newline="") as stream:
        response_reader = csv.DictReader(stream)
        response_fields = list(response_reader.fieldnames or [])
        responses_by_control = {
            row["control_index"]: row for row in response_reader
        }
    if "control_index" not in control_fields or "control_index" not in response_fields:
        raise ValueError("PlotJuggler CSV files require control_index")

    merged_fields = control_fields + [
        "response_" + field for field in response_fields
    ]
    descriptor, merged_path = tempfile.mkstemp(
        prefix="trajectory_compare_data_", suffix=".csv"
    )
    with os.fdopen(descriptor, "w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=merged_fields, lineterminator="\n"
        )
        writer.writeheader()
        for control in control_rows:
            output = dict(control)
            response = responses_by_control.get(control["control_index"], {})
            output.update(
                {"response_" + field: response.get(field, "") for field in response_fields}
            )
            writer.writerow(output)
    return Path(merged_path)


def build_layout(plot_csv_path):
    with plot_csv_path.open(newline="") as stream:
        available_fields = set(next(csv.reader(stream)))
    plots = tuple(
        (title, tuple(curve for curve in curves if curve[0] in available_fields))
        for title, curves in PLOTS
    )
    plots = tuple((title, curves) for title, curves in plots if curves)
    root = ET.Element("root", version="3.9.1")
    tabbed = ET.SubElement(root, "tabbed_widget", parent="main_window", name="Main Window")
    tab = ET.SubElement(tabbed, "Tab", containers="1", tab_name="Reference vs Actual")
    container = ET.SubElement(tab, "Container")
    splitter = ET.SubElement(
        container,
        "DockSplitter",
        sizes="0.25;0.25;0.25;0.25",
        orientation="-",
        count=str(len(plots)),
    )
    splitter.set("sizes", ";".join([str(1.0 / len(plots))] * len(plots)))
    for title, curves in plots:
        area = ET.SubElement(splitter, "DockArea", name=title)
        plot = ET.SubElement(area, "plot", style="Lines", mode="TimeSeries", line_width="1.5")
        ET.SubElement(plot, "range", top="1", right="1", left="0", bottom="-1")
        ET.SubElement(plot, "limitY")
        for name, color in curves:
            ET.SubElement(plot, "curve", name=name, color=color)
    ET.SubElement(tabbed, "currentTabIndex", index="0")
    ET.SubElement(root, "use_relative_time_offset", enabled="1")
    ET.SubElement(root, "streaming_buffer_size", value="14400")

    plugins = ET.SubElement(root, "Plugins")
    csv_plugin = ET.SubElement(plugins, "plugin", ID="DataLoad CSV")
    ET.SubElement(csv_plugin, "parameters", time_axis="elapsed_time_s", delimiter="0")

    previous = ET.SubElement(root, "previouslyLoaded_Datafiles")
    file_info = ET.SubElement(previous, "fileInfo", filename=str(plot_csv_path), prefix="")
    file_plugin = ET.SubElement(file_info, "plugin", ID="DataLoad CSV")
    ET.SubElement(file_plugin, "parameters", time_axis="elapsed_time_s", delimiter="0")
    ET.SubElement(root, "customMathEquations")
    ET.SubElement(root, "snippets")
    return ET.ElementTree(root)


def main():
    rospy.init_node("viewer_plotjuggler_history_launcher", disable_signals=True)
    config_path = Path(rospy.get_param("~config_file")).resolve()
    with config_path.open() as stream:
        config = yaml.safe_load(stream) or {}
    traj_sample = str(config.get("traj_sample", ""))
    run_id = str(config.get("run_id", ""))
    run_directory = (
        repository_root()
        / "base_ws/src/control/trajectory_validator/history"
        / traj_sample
        / run_id
    ).resolve()
    control_path = run_directory / "control_cycles.csv"
    response_path = run_directory / "vehicle_responses.csv"
    if not control_path.is_file() or not response_path.is_file():
        raise FileNotFoundError("PlotJuggler schema-v3 CSV files do not exist: {}".format(run_directory))

    plot_csv_path = build_plot_csv(control_path, response_path)
    descriptor, layout_path = tempfile.mkstemp(prefix="trajectory_compare_", suffix=".xml")
    os.close(descriptor)
    build_layout(plot_csv_path).write(
        layout_path, encoding="UTF-8", xml_declaration=True
    )
    executable = "/opt/ros/noetic/lib/plotjuggler/plotjuggler"
    os.execv(executable, [
        executable,
        "--nosplash",
        "--disable_opengl",
        "--layout",
        layout_path,
        "--window_title",
        "Trajectory Reference vs Actual",
    ])


if __name__ == "__main__":
    main()
