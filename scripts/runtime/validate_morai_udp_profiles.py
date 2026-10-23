#!/usr/bin/env python3
"""Validate local, field-LAN adapter, and field MORAI UDP contracts."""

import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
FIELD_REFERENCE_ROOT = ROOT / "configs/morai_field_reference"
PAIRS = (
    (
        ROOT / "configs/sensors/Sensor_competition_local.json",
        ROOT / "configs/sensors/Sensor_competition_lan.json",
    ),
    (
        ROOT / "configs/network/ego_network_competition_local.json",
        ROOT / "configs/network/ego_network_competition_lan.json",
    ),
)
LEGACY_LOCAL_MIRRORS = (
    (ROOT / "configs/sensors/Sensor.json", PAIRS[0][0]),
    (ROOT / "configs/sensors/Sensor_6.json", PAIRS[0][0]),
    (ROOT / "configs/network/ego_network_1.json", PAIRS[1][0]),
)
FIELD_SENSOR = FIELD_REFERENCE_ROOT / "AjouNice_sensor.json"
FIELD_NETWORK = FIELD_REFERENCE_ROOT / "2026_molit_comp_network.json"

LOCAL_IP = "127.0.0.1"
COMPETITION_HOST_IP = "192.168.0.1"
COMPETITION_DESTINATION_IP = "192.168.0.10"

REQUIRED_RECEIVE_CHANNELS = {
    261: "/CollisionData",
    295: "/Competition_topic",
    770: "/Ego_topic",
    772: "/Object_topic",
}
CONTROL_CHANNEL_TYPE = 771
UDP_ENDPOINT_FIELDS = ("hostIP", "hostPort", "destinationIP", "destinationPort")


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


def network_entries_by_type(network):
    entries = {}
    for group in network["egoNetworkData"]:
        for entry in group.get("listEgoNetworkInfo", []):
            channel_type = int(entry["PUBSUB_TYPE"])
            if channel_type in entries:
                raise ValueError(f"duplicate PUBSUB_TYPE {channel_type}")
            entries[channel_type] = entry
    return entries


def endpoint(config):
    udp = config["udpConfig"]
    return tuple(udp[field] for field in UDP_ENDPOINT_FIELDS)


def validate_field_network_contract(field_network, adapter_network):
    field_entries = network_entries_by_type(field_network)
    adapter_entries = network_entries_by_type(adapter_network)

    for channel_type, expected_topic in REQUIRED_RECEIVE_CHANNELS.items():
        if channel_type not in field_entries or channel_type not in adapter_entries:
            raise ValueError(f"required UDP channel {channel_type} is missing")
        field_config = field_entries[channel_type]["config"]
        adapter_config = adapter_entries[channel_type]["config"]
        if int(field_config["commType"]) != 1 or int(adapter_config["commType"]) != 1:
            raise ValueError(f"required channel {channel_type} is not UDP")
        if endpoint(field_config) != endpoint(adapter_config):
            raise ValueError(
                f"channel {channel_type} endpoint differs: "
                f"field={endpoint(field_config)}, adapter={endpoint(adapter_config)}"
            )
        topic = adapter_config["rosConfig"]["Topic"]
        if topic != expected_topic:
            raise ValueError(
                f"channel {channel_type} ROS topic: expected {expected_topic}, got {topic}"
            )

    field_control = field_entries[CONTROL_CHANNEL_TYPE]["EgoCtrlConfig"]
    adapter_control = adapter_entries[CONTROL_CHANNEL_TYPE]["EgoCtrlConfig"]
    if int(field_control["commType"]) != 1 or int(adapter_control["commType"]) != 1:
        raise ValueError("Ego control channel 771 is not UDP")
    if endpoint(field_control) != endpoint(adapter_control):
        raise ValueError(
            f"control endpoint differs: field={endpoint(field_control)}, "
            f"adapter={endpoint(adapter_control)}"
        )


def main():
    for legacy_path, canonical_path in LEGACY_LOCAL_MIRRORS:
        if load(legacy_path) != load(canonical_path):
            raise ValueError(
                f"legacy local mirror differs from canonical profile: "
                f"{legacy_path} vs {canonical_path}"
            )
        print(
            f"PASS: legacy local mirror {legacy_path.name} == "
            f"{canonical_path.name}"
        )

    for local_path, competition_path in PAIRS:
        local = normalize_and_check(load(local_path), "local")
        competition = normalize_and_check(load(competition_path), "competition")
        if local != competition:
            raise ValueError(
                f"profiles differ outside hostIP/destinationIP: "
                f"{local_path} vs {competition_path}"
            )
        print(
            f"PASS: algorithm local {local_path.name} <-> "
            f"algorithm LAN {competition_path.name} (IP-only difference)"
        )

    sensor = load(PAIRS[0][0])
    periods = [camera["cc"]["sensorPeriod"] for camera in sensor["cameraList"]]
    expected_period = 1.0 / 30.0
    if any(abs(period - expected_period) > 1e-8 for period in periods):
        raise ValueError(f"camera sensorPeriod is not uniformly 30 Hz: {periods}")
    print("PASS: all camera sensorPeriod values are 30 Hz")

    field_sensor = load(FIELD_SENSOR)
    competition_sensor = load(PAIRS[0][1])
    if field_sensor != competition_sensor:
        raise ValueError(
            f"field MORAI sensor differs from algorithm LAN sensor: "
            f"{FIELD_SENSOR} vs {PAIRS[0][1]}"
        )
    print("PASS: field MORAI sensor == algorithm LAN sensor (exact JSON values)")

    validate_field_network_contract(load(FIELD_NETWORK), load(PAIRS[1][1]))
    print(
        "PASS: field MORAI network <-> algorithm LAN required UDP channels "
        "(261, 295, 770, 772, 771)"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
