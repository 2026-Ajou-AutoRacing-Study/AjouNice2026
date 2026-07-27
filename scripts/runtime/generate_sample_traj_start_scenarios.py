#!/usr/bin/env python3
"""Generate one MORAI start scenario for every unique sample-trajectory start."""

import argparse
import csv
import json
import math
from collections import defaultdict
from copy import deepcopy
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SAMPLE_ROOT = REPO_ROOT / "base_ws/src/control/trajectory_generator/traj_sample"
SCENARIO_ROOT = REPO_ROOT / "configs/scenario"
TEMPLATE_PATH = SCENARIO_ROOT / "start_pos.json"
STATIC_TF_PATH = REPO_ROOT / "configs/tf/morai_lanelet_static_tf.yaml"
REFERENCE_LANELET_ID = "200889"
POSE_TOLERANCE = 1e-9


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate MORAI scenarios from sample-trajectory start poses"
    )
    parser.add_argument(
        "--lanelet-id",
        help="generate only this sample-trajectory start lanelet",
    )
    parser.add_argument(
        "--initial-speed-kph",
        type=float,
        default=0.0,
        help="spawn speed in kph; a positive value is appended to the output filename",
    )
    arguments = parser.parse_args()
    if arguments.initial_speed_kph < 0.0 or not math.isfinite(
        arguments.initial_speed_kph
    ):
        parser.error("--initial-speed-kph must be finite and non-negative")
    return arguments


def read_start_rows():
    starts_by_lanelet = defaultdict(list)
    for path in sorted(SAMPLE_ROOT.glob("*kph/*.csv")):
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            try:
                row = next(reader)
            except StopIteration as error:
                raise RuntimeError(f"Empty trajectory CSV: {path}") from error

        start = {
            "x": float(row["x"]),
            "y": float(row["y"]),
            "z": float(row["z"]),
            "yaw": float(row["yaw"]),
        }
        lanelet_id = row["lanelet_id"].strip()
        if not lanelet_id:
            raise RuntimeError(f"Missing first-row lanelet_id: {path}")
        starts_by_lanelet[lanelet_id].append((path, start))

    if not starts_by_lanelet:
        raise RuntimeError(f"No sample trajectory CSVs found under {SAMPLE_ROOT}")
    return starts_by_lanelet


def assert_consistent_starts(starts_by_lanelet):
    for lanelet_id, entries in starts_by_lanelet.items():
        reference_path, reference = entries[0]
        for path, start in entries[1:]:
            mismatches = [
                key
                for key in ("x", "y", "z", "yaw")
                if abs(start[key] - reference[key]) > POSE_TOLERANCE
            ]
            if mismatches:
                fields = ", ".join(mismatches)
                raise RuntimeError(
                    f"Lanelet {lanelet_id} has conflicting starts ({fields}): "
                    f"{reference_path} vs {path}"
                )


def read_planar_static_tf():
    with STATIC_TF_PATH.open(encoding="utf-8") as stream:
        transform = yaml.safe_load(stream)["map_to_morai_map"]
    translation = [float(value) for value in transform["xyz"]]
    rpy = [float(value) for value in transform["rpy"]]
    if abs(rpy[0]) > 1e-12 or abs(rpy[1]) > 1e-12:
        raise RuntimeError(
            "Scenario generator supports a yaw-only map_to_morai_map transform"
        )
    return translation, rpy[2]


def inverse_map_pose(start, translation, tf_yaw):
    dx = start["x"] - translation[0]
    dy = start["y"] - translation[1]
    cosine = math.cos(tf_yaw)
    sine = math.sin(tf_yaw)
    morai_x = cosine * dx + sine * dy
    morai_y = -sine * dx + cosine * dy
    morai_yaw_deg = math.degrees(start["yaw"] - tf_yaw) % 360.0
    return morai_x, morai_y, morai_yaw_deg


def representative_starts(starts_by_lanelet):
    return {
        lanelet_id: entries[0][1]
        for lanelet_id, entries in starts_by_lanelet.items()
    }


def apply_initial_speed(scenario, speed_kph, morai_yaw_deg):
    ego = scenario["egoVehicle"]
    initial_position = ego["initPosition"]
    save_data = initial_position["vehicleSaveData"]
    speed_mps = speed_kph / 3.6
    yaw_rad = math.radians(morai_yaw_deg)
    velocity = {
        "x": -speed_mps * math.cos(yaw_rad),
        "y": 0.0,
        "z": -speed_mps * math.sin(yaw_rad),
    }

    ego["currentVelocity"] = speed_kph
    ego["customValue"] = speed_kph
    initial_position["gear"] = 4
    save_data["velocity"] = speed_kph
    if len(save_data["inputData"]) <= 6:
        raise RuntimeError("vehicleSaveData.inputData has no gear field")
    save_data["inputData"][6] = 4
    save_data["vehicle"]["lastVelocity"].update(velocity)
    save_data["lastVelocity"].update(velocity)


def speed_label(speed_kph):
    return "{:g}".format(speed_kph)


def generate(arguments):
    starts_by_lanelet = read_start_rows()
    assert_consistent_starts(starts_by_lanelet)
    starts = representative_starts(starts_by_lanelet)
    if REFERENCE_LANELET_ID not in starts:
        raise RuntimeError(
            f"Reference lanelet {REFERENCE_LANELET_ID} is absent from sample trajectories"
        )

    with TEMPLATE_PATH.open(encoding="utf-8") as stream:
        template = json.load(stream)
    translation, tf_yaw = read_planar_static_tf()

    template_position = template["egoVehicle"]["initPosition"]["pos"]
    vehicle_z_offset = (
        float(template_position["z"]) - starts[REFERENCE_LANELET_ID]["z"]
    )
    if arguments.lanelet_id is not None:
        if arguments.lanelet_id not in starts:
            raise RuntimeError(
                f"Lanelet {arguments.lanelet_id} is absent from sample trajectories"
            )
        starts = {arguments.lanelet_id: starts[arguments.lanelet_id]}

    generated = []
    for lanelet_id, start in sorted(starts.items(), key=lambda item: int(item[0])):
        scenario = deepcopy(template)
        position = scenario["egoVehicle"]["initPosition"]["pos"]
        rotation = scenario["egoVehicle"]["initPosition"]["rot"]
        morai_x, morai_y, morai_yaw_deg = inverse_map_pose(
            start, translation, tf_yaw
        )
        morai_z = start["z"] - translation[2] + vehicle_z_offset

        position.update(
            {
                "x": morai_x,
                "y": morai_y,
                "z": morai_z,
                "_x": f"{morai_x:.3f}",
                "_y": f"{morai_y:.3f}",
                "_z": f"{morai_z:.3f}",
            }
        )
        rotation["yaw"] = f"{morai_yaw_deg:.6f}"
        if arguments.initial_speed_kph > 0.0:
            apply_initial_speed(
                scenario, arguments.initial_speed_kph, morai_yaw_deg
            )

        speed_suffix = (
            f"_{speed_label(arguments.initial_speed_kph)}"
            if arguments.initial_speed_kph > 0.0
            else ""
        )
        output_path = SCENARIO_ROOT / f"{lanelet_id}_start{speed_suffix}.json"
        with output_path.open("w", encoding="utf-8") as stream:
            json.dump(scenario, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        generated.append((output_path, len(starts_by_lanelet[lanelet_id]), start))

    print(f"Generated {len(generated)} start scenarios")
    print(f"Vehicle z offset: {vehicle_z_offset:.9f} m")
    for path, source_count, start in generated:
        print(
            f"{path.relative_to(REPO_ROOT)} <- {source_count} CSVs, "
            f"map start=({start['x']:.9f}, {start['y']:.9f}, "
            f"{start['z']:.9f}), yaw={start['yaw']:.12f} rad"
        )


if __name__ == "__main__":
    generate(parse_args())
