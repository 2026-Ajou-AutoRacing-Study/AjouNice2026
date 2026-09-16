#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _included_arguments():
    root = ET.parse(PACKAGE_ROOT / "launch" / "tracking_prediction.launch").getroot()
    include = root.find("include")
    assert include is not None
    return {item.attrib["name"]: item.attrib["value"] for item in include.findall("arg")}


def test_production_tracking_and_prediction_values_are_explicit():
    arguments = _included_arguments()
    expected = {
        "start_detector": "false",
        "world_frame_id": "map",
        "canonicalize_vehicle_orientation_to_motion": "true",
        "compensate_tracking_output_to_current_time": "false",
        "tracking_internal_publish_rate_hz": "30.0",
        "maximum_tracking_internal_prediction_age_sec": "0.25",
        "detection_velocity_fusion_mode": "0",
        "tracking_publish_rate_hz": "0.0",
        "force_unknown_stationary": "true",
        "prediction_model": "map_based",
        "prediction_time_step": "0.1",
        "prediction_horizon": "3.0",
    }
    for name, value in expected.items():
        assert arguments.get(name) == value, (name, arguments.get(name))


def test_production_topics_and_v74_map_are_explicit():
    root = ET.parse(PACKAGE_ROOT / "launch" / "tracking_prediction.launch").getroot()
    top_level = {item.attrib["name"]: item.attrib for item in root.findall("arg")}
    arguments = _included_arguments()
    assert arguments["detection_objects"] == "/perception/object_recognition/detection/objects"
    assert arguments["measurement_tracked_objects"] == "/perception/object_recognition/tracking/measurement_objects"
    assert arguments["tracked_objects"] == "/perception/object_recognition/tracking/objects"
    assert arguments["predicted_objects"] == "/perception/object_recognition/objects"
    assert top_level["map_file"]["default"].endswith("/lanelet2_map_v74.osm")
    assert top_level["projector_file"]["default"].endswith("/map_projector_info_v1.yaml")
    assert arguments["map_file"] == "$(arg map_file)"
    assert arguments["projector_file"] == "$(arg projector_file)"
