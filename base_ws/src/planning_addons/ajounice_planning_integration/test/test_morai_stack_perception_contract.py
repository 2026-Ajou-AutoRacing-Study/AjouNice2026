from pathlib import Path
import hashlib
import re
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[3]


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
    assert values["map_file"].endswith("/lanelet2_map_v65.osm")
    assert values["projector_file"].endswith("/map_projector_info_v1.yaml")


def test_v65_map_sha_is_identical_across_runtime_contracts():
    map_path = REPO_ROOT / "kcity_map/lanelet_map/lanelet2_map_v65.osm"
    route_path = (
        REPO_ROOT
        / "base_ws/src/planning_addons/competition_route_requester/config"
        / "kcity_v65_competition_route.yaml"
    )
    publisher_path = PACKAGE_ROOT / "scripts/map_projector_publisher"

    actual_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    route_sha = yaml.safe_load(route_path.read_text())["map"]["sha256"]
    publisher_match = re.search(
        r"hexdigest\(\) != '([0-9a-f]{64})'", publisher_path.read_text()
    )

    assert publisher_match is not None
    assert actual_sha == route_sha == publisher_match.group(1)
