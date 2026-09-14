from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _launch():
    return ET.parse(
        PACKAGE_ROOT / "launch" / "morai_planning_stack.launch"
    ).getroot()


def test_integrated_stack_owns_downstream_pipeline_but_not_detector_by_default():
    args = {item.attrib["name"]: item.attrib for item in _launch().findall("arg")}
    assert args["start_object_pipeline"]["default"] == "true"
    assert args["start_detector"]["default"] == "false"
    assert args["detector"]["default"] == "fastbev"


def test_integrated_downstream_contract_matches_standalone_production_launch():
    top_level = {
        item.attrib["name"]: item.attrib for item in _launch().findall("arg")
    }
    include = next(
        item
        for item in _launch().findall(".//include")
        if item.attrib["file"].endswith(
            "perception_object_pipeline)/launch/object_pipeline.launch"
        )
    )
    values = {
        item.attrib["name"]: item.attrib["value"]
        for item in include.findall("arg")
    }
    expected = {
        "canonicalize_vehicle_orientation_to_motion": "true",
        "tracking_internal_publish_rate_hz": "30.0",
        "maximum_tracking_internal_prediction_age_sec": "$(arg maximum_tracking_internal_prediction_age_sec)",
        "detection_velocity_fusion_mode": "0",
        "tracking_publish_rate_hz": "0.0",
        "prediction_model": "$(arg prediction_model)",
        "force_unknown_stationary": "$(arg force_unknown_stationary)",
        "prediction_time_step": "0.1",
        "prediction_horizon": "3.0",
    }
    for name, value in expected.items():
        assert values.get(name) == value, (name, values.get(name))
    assert top_level["maximum_tracking_internal_prediction_age_sec"]["default"] == "0.25"
    assert values["map_file"].endswith("/lanelet2_map_v63.osm")
    assert values["projector_file"].endswith("/map_projector_info_v1.yaml")
