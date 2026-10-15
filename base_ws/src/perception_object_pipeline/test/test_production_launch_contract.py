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
        "maximum_tracking_internal_prediction_age_sec": "$(arg maximum_tracking_internal_prediction_age_sec)",
        "detection_velocity_fusion_mode": "$(arg tracker_detection_velocity_fusion_mode)",
        "tracking_publish_rate_hz": "0.0",
        "force_unknown_stationary": "true",
        "prediction_model": "map_based",
        "prediction_time_step": "0.1",
        "prediction_horizon": "3.0",
    }
    for name, value in expected.items():
        assert arguments.get(name) == value, (name, arguments.get(name))
    root = ET.parse(PACKAGE_ROOT / "launch" / "tracking_prediction.launch").getroot()
    defaults = {item.attrib["name"]: item.attrib.get("default") for item in root.findall("arg")}
    # Standalone default remains 0.25; the operating alias explicitly selects
    # 0.50. Validate forwarding instead of requiring a hardcoded child value.
    assert defaults["maximum_tracking_internal_prediction_age_sec"] == "0.25"


def test_tracker_ablation_defaults_preserve_production_contract():
    root = ET.parse(PACKAGE_ROOT / "launch" / "tracking_prediction.launch").getroot()
    defaults = {item.attrib["name"]: item.attrib.get("default") for item in root.findall("arg")}
    expected = {
        "tracker_prediction_model": "1",
        "tracker_system_noise_std_vx_vy_ms": "1.0",
        "tracker_meas_noise_std_xy_m": "0.4",
        "tracker_meas_noise_std_yaw_deg": "10.0",
        "tracker_use_kinematic_model": "false",
        "tracker_detection_velocity_fusion_mode": "0",
        "tracker_detection_velocity_noise_std_mps": "1.0",
    }
    for name, value in expected.items():
        assert defaults.get(name) == value, (name, defaults.get(name))

    arguments = _included_arguments()
    for name in (
        "tracker_prediction_model",
        "tracker_system_noise_std_vx_vy_ms",
        "tracker_meas_noise_std_xy_m",
        "tracker_meas_noise_std_yaw_deg",
        "tracker_use_kinematic_model",
    ):
        assert arguments.get(name) == "$(arg {})".format(name)
    assert arguments["detection_velocity_fusion_mode"] == (
        "$(arg tracker_detection_velocity_fusion_mode)"
    )
    assert arguments["detection_velocity_noise_std_mps"] == (
        "$(arg tracker_detection_velocity_noise_std_mps)"
    )


def test_production_topics_and_v85_map_are_explicit():
    root = ET.parse(PACKAGE_ROOT / "launch" / "tracking_prediction.launch").getroot()
    top_level = {item.attrib["name"]: item.attrib for item in root.findall("arg")}
    arguments = _included_arguments()
    assert arguments["detection_objects"] == "/perception/object_recognition/detection/objects"
    assert arguments["measurement_tracked_objects"] == "/perception/object_recognition/tracking/measurement_objects"
    assert arguments["tracked_objects"] == "/perception/object_recognition/tracking/objects"
    assert arguments["predicted_objects"] == "/perception/object_recognition/objects"
    assert top_level["map_file"]["default"].endswith("/lanelet2_map_v85.osm")
    assert top_level["projector_file"]["default"].endswith("/map_projector_info_v1.yaml")
    assert arguments["map_file"] == "$(arg map_file)"
    assert arguments["projector_file"] == "$(arg projector_file)"


def test_callback_processing_diagnostics_are_opt_in_without_policy_change():
    pipeline = ET.parse(PACKAGE_ROOT / "launch" / "object_pipeline.launch").getroot()
    defaults = {item.attrib["name"]: item.attrib.get("default") for item in pipeline.findall("arg")}
    assert defaults["publish_processing_diagnostics"] == "false"
    policy = next(node for node in pipeline.findall("node") if node.attrib["name"] == "tracked_object_motion_policy")
    params = {item.attrib["name"]: item.attrib["value"] for item in policy.findall("param")}
    assert params["publish_processing_diagnostics"] == "$(arg publish_processing_diagnostics)"
    prediction = ET.parse(PACKAGE_ROOT / "launch" / "object_prediction.launch").getroot()
    prediction_defaults = {item.attrib["name"]: item.attrib.get("default") for item in prediction.findall("arg")}
    assert prediction_defaults["publish_processing_diagnostics"] == "false"
    map_node = next(node for node in prediction.iter("node") if node.attrib["name"] == "map_based_prediction")
    map_params = {item.attrib["name"]: item.attrib.get("value") for item in map_node.findall("param")}
    assert map_params["publish_processing_diagnostics"] == "$(arg publish_processing_diagnostics)"
