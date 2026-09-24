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
    assert args["start_localization"]["default"] == "false"
    assert args["start_object_pipeline"]["default"] == "true"
    assert args["start_detector"]["default"] == "false"
    assert args["detector"]["default"] == "fastbev"
    assert args["start_traffic_light"]["default"] == "true"
    assert args["enable_traffic_light_planner"]["default"] == "true"


def test_local_load_test_can_disable_only_the_traffic_light_planner_plugin():
    planning_launch = ET.parse(
        PACKAGE_ROOT / "launch" / "planning.launch"
    ).getroot()
    args = {
        item.attrib["name"]: item.attrib for item in planning_launch.findall("arg")
    }
    assert args["enable_traffic_light_planner"]["default"] == "true"

    with_traffic = args[
        "behavior_velocity_launch_modules_with_traffic_light"
    ]["default"]
    without_traffic = args[
        "behavior_velocity_launch_modules_without_traffic_light"
    ]["default"]
    traffic_light_plugin = (
        "autoware::behavior_velocity_planner::experimental::"
        "TrafficLightModulePlugin"
    )
    assert traffic_light_plugin in with_traffic
    assert traffic_light_plugin not in without_traffic

    tier4_include = next(
        item
        for item in planning_launch.findall("include")
        if item.attrib["file"].endswith(
            "tier4_planning_launch)/launch/planning.launch"
        )
    )
    values = {
        item.attrib["name"]: item.attrib["value"]
        for item in tier4_include.findall("arg")
    }
    assert values["behavior_velocity_launch_modules"] == "$(arg behavior_velocity_launch_modules)"


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
    assert values["map_file"].endswith("/lanelet2_map_v85.osm")
    assert values["projector_file"].endswith("/map_projector_info_v1.yaml")


def test_v83_map_sha_is_identical_across_runtime_contracts():
    map_path = REPO_ROOT / "kcity_map/lanelet_map/lanelet2_map_v85.osm"
    route_path = (
        REPO_ROOT
        / "base_ws/src/planning_addons/competition_route_requester/config"
        / "kcity_v85_competition_route.yaml"
    )
    publisher_path = PACKAGE_ROOT / "scripts/map_projector_publisher"

    actual_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    route_sha = yaml.safe_load(route_path.read_text())["map"]["sha256"]
    publisher_match = re.search(
        r"hexdigest\(\) != '([0-9a-f]{64})'", publisher_path.read_text()
    )

    assert publisher_match is not None
    assert actual_sha == route_sha == publisher_match.group(1)


def test_kcity_route_marker_uses_the_curated_drivable_lanelet_sets():
    config_path = PACKAGE_ROOT / "config/mission_planner_kcity_v85.yaml"
    config = yaml.safe_load(config_path.read_text())
    assert config["route_section_primitive_overrides"] == {
        "201436": [201436, 201435, 200099, 201438],
        "200127": [200127, 200130, 200134],
        "201439": [201439, 201441, 201446],
        "200185": [200185, 200360],
        "201476": [201476, 201477],
        "201448": [201448, 201479],
        "201447": [201478, 201447],
        "201401": [201401],
        "201465": [201465, 201466, 201467, 201468],
        "201473": [201473, 201474, 201475],
    }
    assert set(config["route_section_excluded_lanelet_ids"]) == {
        201400, 201440, 200148, 201445, 200141, 200186, 201437, 201463,
        201472, 201464,
    }

    integration_launch = ET.parse(PACKAGE_ROOT / "launch/planning.launch").getroot()
    integration_args = {
        item.attrib["name"]: item.attrib for item in integration_launch.findall("arg")
    }
    assert integration_args["mission_planner_param_path"]["default"].endswith(
        "/config/mission_planner_kcity_v85.yaml"
    )
    tier4_include = next(
        item
        for item in integration_launch.findall("include")
        if item.attrib["file"].endswith("tier4_planning_launch)/launch/planning.launch")
    )
    passed_args = {
        item.attrib["name"]: item.attrib["value"] for item in tier4_include.findall("arg")
    }
    assert passed_args["mission_planner_param_path"] == "$(arg mission_planner_param_path)"

    tier4_launch = ET.parse(
        REPO_ROOT / "planning_ws/src/tier4_planning_launch/launch/planning.launch"
    ).getroot()
    mission_include = next(
        item
        for item in tier4_launch.findall(".//include")
        if item.attrib["file"].endswith(
            "autoware_mission_planner_universe)/launch/mission_planner.launch"
        )
    )
    mission_args = {
        item.attrib["name"]: item.attrib["value"] for item in mission_include.findall("arg")
    }
    assert mission_args["mission_planner_param_path"] == "$(arg mission_planner_param_path)"
