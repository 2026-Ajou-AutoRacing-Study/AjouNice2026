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
    assert args["controller_kinematic_state_topic"]["default"] == (
        "/localization/control_kinematic_state"
    )


def test_controller_state_override_does_not_remap_planning_state():
    controller_include = next(
        item
        for item in _launch().findall(".//include")
        if item.attrib["file"].endswith(
            "trajectory_follower)/launch/pp_pi_autoware.launch"
        )
    )
    values = {
        item.attrib["name"]: item.attrib["value"]
        for item in controller_include.findall("arg")
    }
    assert values["kinematic_state_topic"] == (
        "$(arg controller_kinematic_state_topic)"
    )

    planning_include = next(
        item
        for item in _launch().findall("include")
        if item.attrib["file"].endswith(
            "ajounice_planning_integration)/launch/planning.launch"
        )
    )
    planning_values = {
        item.attrib["name"]: item.attrib["value"]
        for item in planning_include.findall("arg")
    }
    assert "kinematic_state_topic" not in planning_values

    follower_launch = ET.parse(
        REPO_ROOT
        / "base_ws/src/control/trajectory_follower/launch/pp_pi_autoware.launch"
    ).getroot()
    follower_args = {
        item.attrib["name"]: item.attrib for item in follower_launch.findall("arg")
    }
    assert follower_args["kinematic_state_topic"]["default"] == (
        "/localization/kinematic_state"
    )
    controller_nodes = follower_launch.findall("node")
    assert len(controller_nodes) == 2
    for node in controller_nodes:
        topic_override = next(
            item
            for item in node.findall("param")
            if item.attrib["name"] == "topics/kinematic_state"
        )
        assert topic_override.attrib["value"] == "$(arg kinematic_state_topic)"


def test_production_localization_publishes_smooth_controller_state():
    localization_launch = ET.parse(
        REPO_ROOT
        / "base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch"
    ).getroot()
    args = {
        item.attrib["name"]: item.attrib
        for item in localization_launch.findall("arg")
    }
    assert args["publish_control_state"]["default"] == "true"
    assert args["use_quantized_gnss_control_state"]["default"] == "false"
    assert args["control_state_topic"]["default"] == (
        "/localization/control_kinematic_state"
    )
    assert args["control_translation_tau_sec"]["default"] == "0.5"
    assert args["control_yaw_tau_sec"]["default"] == "0.5"


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
        "prediction_horizon": "$(arg prediction_horizon)",
    }
    for name, value in expected.items():
        assert values.get(name) == value, (name, values.get(name))
    assert top_level["maximum_tracking_internal_prediction_age_sec"]["default"] == "0.25"
    # Empty preserves object_pipeline's current general/highway horizon policy;
    # the integration launch must not silently force the historical 3s override.
    assert top_level["prediction_horizon"]["default"] == ""
    assert values["map_file"].endswith("/lanelet2_map_v88.osm")
    assert values["projector_file"].endswith("/map_projector_info_v1.yaml")


def test_v88_map_sha_is_identical_across_runtime_contracts():
    map_path = REPO_ROOT / "kcity_map/lanelet_map/lanelet2_map_v88.osm"
    route_path = (
        REPO_ROOT
        / "base_ws/src/planning_addons/competition_route_requester/config"
        / "kcity_v88_competition_route.yaml"
    )
    publisher_path = PACKAGE_ROOT / "scripts/map_projector_publisher"

    actual_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    route_sha = yaml.safe_load(route_path.read_text())["map"]["sha256"]
    publisher_match = re.search(
        r"'~map_sha256', '([0-9a-f]{64})'", publisher_path.read_text()
    )

    assert publisher_match is not None
    assert actual_sha == route_sha == publisher_match.group(1)


def test_kcity_route_marker_uses_the_curated_drivable_lanelet_sets():
    config_path = PACKAGE_ROOT / "config/mission_planner_kcity_v88.yaml"
    config = yaml.safe_load(config_path.read_text())
    assert config["route_section_primitive_overrides"] == {
        "201439": [201439, 201441, 201446],
        "200360": [200360, 200185],
        "200185": [200360, 200185],
        "201479": [201479, 201448],
        "201448": [201479, 201448],
        "201401": [201401],
        "201465": [201465, 201466, 201467, 201468],
    }
    assert set(config["route_section_excluded_lanelet_ids"]) == {
        201440, 201445, 200141, 200186, 201463, 201464,
    }

    integration_launch = ET.parse(PACKAGE_ROOT / "launch/planning.launch").getroot()
    integration_args = {
        item.attrib["name"]: item.attrib for item in integration_launch.findall("arg")
    }
    assert integration_args["mission_planner_param_path"]["default"].endswith(
        "/config/mission_planner_kcity_v88.yaml"
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
