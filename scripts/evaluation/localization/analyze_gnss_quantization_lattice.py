#!/usr/bin/env python3

"""Measure the MORAI GPS angular lattice and its UTM cell geometry."""

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import rosbag


ROOT = Path(__file__).resolve().parents[3]
CONVERTER = (
    ROOT
    / "base_ws/src/localization/localization_manager/scripts/"
    "morai_gps_to_local_odometry.py"
)
SPEC = importlib.util.spec_from_file_location("morai_gps_converter", CONVERTER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def minimum_lattice_step(values):
    unique = np.unique(np.asarray(values, dtype=float))
    differences = np.diff(unique)
    differences = differences[differences > 1.0e-12]
    if not differences.size:
        return None
    minimum = float(np.min(differences))
    # Average only the smallest cluster to suppress binary floating error.
    cluster = differences[differences < minimum * 1.05]
    return float(np.median(cluster))


def cell_vector(latitude, longitude, delta_latitude, delta_longitude):
    x0, y0 = MODULE.wgs84_to_utm(latitude, longitude, 52, True)
    x1, y1 = MODULE.wgs84_to_utm(
        latitude + delta_latitude, longitude + delta_longitude, 52, True
    )
    dx = x1 - x0
    dy = y1 - y0
    return {
        "dx_m": dx,
        "dy_m": dy,
        "width_m": math.hypot(dx, dy),
        "yaw_deg": math.degrees(math.atan2(dy, dx)),
    }


def analyze(path):
    latitudes = []
    longitudes = []
    with rosbag.Bag(str(path), "r") as bag:
        for _, message, _ in bag.read_messages(topics=["/sensor/gps"]):
            if message.latitude != 0.0 and message.longitude != 0.0:
                latitudes.append(float(message.latitude))
                longitudes.append(float(message.longitude))
    if not latitudes:
        raise ValueError("{} has no valid /sensor/gps samples".format(path))
    latitude_step = minimum_lattice_step(latitudes)
    longitude_step = minimum_lattice_step(longitudes)
    latitude = float(np.median(latitudes))
    longitude = float(np.median(longitudes))
    return {
        "bag": str(path.resolve()),
        "valid_sample_count": len(latitudes),
        "unique_latitude_count": len(set(latitudes)),
        "unique_longitude_count": len(set(longitudes)),
        "median_wgs84": {"latitude": latitude, "longitude": longitude},
        "angular_step_deg": {
            "latitude": latitude_step,
            "longitude": longitude_step,
        },
        "utm52n_cell": {
            "longitude_axis": cell_vector(
                latitude, longitude, 0.0, longitude_step
            ),
            "latitude_axis": cell_vector(
                latitude, longitude, latitude_step, 0.0
            ),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = {"bags": [analyze(path) for path in args.bags]}
    output = json.dumps(result, indent=2, sort_keys=True)
    print(output)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
