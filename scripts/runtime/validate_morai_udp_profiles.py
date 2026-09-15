#!/usr/bin/env python3
"""Verify that local and competition MORAI UDP profiles differ only by IP."""

import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
PAIRS = (
    (
        ROOT / "configs/sensors/Sensor.json",
        ROOT / "configs/sensors/Sensor_competition_lan.json",
    ),
    (
        ROOT / "configs/network/ego_network_1.json",
        ROOT / "configs/network/ego_network_competition_lan.json",
    ),
)

LOCAL_IP = "127.0.0.1"
COMPETITION_HOST_IP = "192.168.0.1"
COMPETITION_DESTINATION_IP = "192.168.0.10"


def normalize_and_check(value, profile, path=()):
    if isinstance(value, dict):
        result = {}
        for key, child in value.items():
            child_path = path + (str(key),)
            is_udp_endpoint = bool(path) and path[-1] == "udpConfig"
            if key == "hostIP" and is_udp_endpoint:
                expected = LOCAL_IP if profile == "local" else COMPETITION_HOST_IP
                if child != expected:
                    raise ValueError(
                        f"{profile} {'/'.join(child_path)}: expected {expected}, got {child}"
                    )
                result[key] = "<HOST_IP>"
            elif key == "destinationIP" and is_udp_endpoint:
                expected = LOCAL_IP if profile == "local" else COMPETITION_DESTINATION_IP
                if child != expected:
                    raise ValueError(
                        f"{profile} {'/'.join(child_path)}: expected {expected}, got {child}"
                    )
                result[key] = "<DESTINATION_IP>"
            else:
                result[key] = normalize_and_check(child, profile, child_path)
        return result
    if isinstance(value, list):
        return [
            normalize_and_check(child, profile, path + (str(index),))
            for index, child in enumerate(value)
        ]
    return value


def load(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main():
    for local_path, competition_path in PAIRS:
        local = normalize_and_check(load(local_path), "local")
        competition = normalize_and_check(load(competition_path), "competition")
        if local != competition:
            raise ValueError(
                f"profiles differ outside hostIP/destinationIP: "
                f"{local_path} vs {competition_path}"
            )
        print(f"PASS: {local_path.name} <-> {competition_path.name}")

    sensor = load(PAIRS[0][0])
    periods = [camera["cc"]["sensorPeriod"] for camera in sensor["cameraList"]]
    expected_period = 1.0 / 30.0
    if any(abs(period - expected_period) > 1e-8 for period in periods):
        raise ValueError(f"camera sensorPeriod is not uniformly 30 Hz: {periods}")
    print("PASS: all camera sensorPeriod values are 30 Hz")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
