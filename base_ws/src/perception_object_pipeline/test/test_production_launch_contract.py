#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_highway_eight_second_production_wiring_and_rollback():
    wrapper = ET.parse(PACKAGE_ROOT / 'launch/tracking_prediction.launch').getroot()
    defaults = {a.attrib['name']: a.attrib.get('default') for a in wrapper.findall('arg')}
    assert defaults['prediction_mixed_horizon_enabled'] == 'true'
    assert defaults['prediction_highway_horizon'] == ''
    assert defaults['prediction_highway_mode'] == ''
    forwarded = _included_arguments()
    assert forwarded['prediction_horizon'] == '$(arg prediction_horizon)'
    for name in ('prediction_mixed_horizon_enabled', 'prediction_highway_horizon', 'prediction_highway_mode'):
        assert forwarded[name] == '$(arg ' + name + ')'
    pipeline = ET.parse(PACKAGE_ROOT / 'launch/object_pipeline.launch').getroot()
    generic = {a.attrib['name']: a.attrib.get('default') for a in pipeline.findall('arg')}
    assert generic['prediction_mixed_horizon_enabled'] == 'false'
    prediction = next(i for i in pipeline.findall('include') if 'object_prediction.launch' in i.attrib['file'])
    for name in ('prediction_mixed_horizon_enabled', 'prediction_highway_horizon', 'prediction_highway_mode'):
        assert prediction.find("arg[@name='" + name + "']").attrib['value'] == '$(arg ' + name + ')'
    leaf = ET.parse(PACKAGE_ROOT / 'launch/object_prediction.launch').getroot()
    node = next(n for n in leaf.iter('node') if n.attrib['name'] == 'map_based_prediction')
    assert node.find("param[@name='highway_prediction_horizon']").attrib['value'] == '$(arg prediction_highway_horizon)'
    assert node.find("param[@name='highway_prediction_mode']").attrib['value'] == '$(arg prediction_highway_mode)'
    # Ordinary CTRV/fallback objects retain the baseline 3-second horizon.
    assert leaf.find("arg[@name='prediction_horizon']").attrib['default'] == ''
    policy = yaml.safe_load((PACKAGE_ROOT / 'config/map_based_prediction.yaml').read_text())
    assert policy['prediction_horizon'] == 3.0
    assert policy['highway_prediction_horizon'] == 8.0
    assert policy['highway_prediction_mode'] == 'native'
    assert "arg('prediction_horizon') != ''" in node.find("param[@name='prediction_horizon']").attrib['if']


def test_integrated_full_stack_and_operating_alias_select_same_mixed_policy():
    repo = PACKAGE_ROOT.parents[2]
    launch = ET.parse(repo / 'base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch').getroot()
    expected = {'prediction_horizon': '', 'prediction_mixed_horizon_enabled': 'true',
                'prediction_highway_horizon': '', 'prediction_highway_mode': ''}
    include = next(i for i in launch.iter('include') if 'object_pipeline.launch' in i.attrib['file'])
    for name, value in expected.items():
        assert launch.find("arg[@name='" + name + "']").attrib['default'] == value
        assert include.find("arg[@name='" + name + "']").attrib['value'] == '$(arg ' + name + ')'
    alias = next(line for line in (repo / 'docs/common_alias.txt').read_text().splitlines()
                 if line.startswith('alias tracking='))
    assert 'prediction_mixed_horizon_enabled:=true' in alias
    assert 'prediction_highway_horizon:=' not in alias
    assert 'prediction_highway_mode:=' not in alias
    assert 'maximum_tracking_internal_prediction_age_sec:=0.5' in alias


def test_observation_consistency_candidates_are_explicit_and_default_off():
    wrapper = ET.parse(PACKAGE_ROOT / "launch" / "tracking_prediction.launch").getroot()
    pipeline = ET.parse(PACKAGE_ROOT / "launch" / "object_pipeline.launch").getroot()
    standalone = ET.parse(PACKAGE_ROOT.parent / "ekf-multi-object-tracking" /
                          "launch" / "ekf_multi_class_object_tracking.launch").getroot()
    wrapper_defaults = {a.attrib["name"]: a.attrib.get("default") for a in wrapper.findall("arg")}
    pipeline_defaults = {a.attrib["name"]: a.attrib.get("default") for a in pipeline.findall("arg")}
    standalone_defaults = {a.attrib["name"]: a.attrib.get("default") for a in standalone.findall("arg")}
    wrapper_args = {a.attrib["name"]: a.attrib["value"] for a in wrapper.find("include").findall("arg")}
    tracker_include = next(i for i in pipeline.findall("include") if "ekf_multi" in i.attrib["file"])
    tracker_args = {a.attrib["name"]: a.attrib["value"] for a in tracker_include.findall("arg")}
    params = {p.attrib["name"]: p.attrib["value"] for p in standalone.findall("param")}
    for name in ("shape_consistent_association_ranking", "guard_inconsistent_class_transitions"):
        assert wrapper_defaults["tracker_" + name] == "false"
        assert pipeline_defaults[name] == standalone_defaults[name] == "false"
        assert wrapper_args[name] == "$(arg tracker_" + name + ")"
        assert tracker_args[name] == "$(arg " + name + ")"
        assert params["configure/" + name] == "$(arg " + name + ")"


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
        "prediction_horizon": "$(arg prediction_horizon)",
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


def test_recorder_diagnostics_auto_default_and_opt_out_propagate_to_all_producers():
    wrapper = ET.parse(PACKAGE_ROOT / "launch" / "tracking_prediction.launch").getroot()
    wrapper_defaults = {item.attrib["name"]: item.attrib.get("default") for item in wrapper.findall("arg")}
    assert wrapper_defaults["diagnostics_auto_on_subscribe"] == "true"
    assert _included_arguments()["diagnostics_auto_on_subscribe"] == "$(arg diagnostics_auto_on_subscribe)"

    pipeline = ET.parse(PACKAGE_ROOT / "launch" / "object_pipeline.launch").getroot()
    defaults = {item.attrib["name"]: item.attrib.get("default") for item in pipeline.findall("arg")}
    assert defaults["diagnostics_auto_on_subscribe"] == "true"
    policy = next(node for node in pipeline.findall("node") if node.attrib["name"] == "tracked_object_motion_policy")
    assert policy.find("param[@name='diagnostics_auto_on_subscribe']").attrib["value"] == "$(arg diagnostics_auto_on_subscribe)"
    for include in pipeline.findall("include"):
        if "ekf_multi_class_object_tracking.launch" in include.attrib["file"] or "object_prediction.launch" in include.attrib["file"]:
            assert include.find("arg[@name='diagnostics_auto_on_subscribe']").attrib["value"] == "$(arg diagnostics_auto_on_subscribe)"

    prediction = ET.parse(PACKAGE_ROOT / "launch" / "object_prediction.launch").getroot()
    assert prediction.find("arg[@name='diagnostics_auto_on_subscribe']").attrib["default"] == "true"
    map_node = next(node for node in prediction.iter("node") if node.attrib["name"] == "map_based_prediction")
    assert map_node.find("param[@name='diagnostics_auto_on_subscribe']").attrib["value"] == "$(arg diagnostics_auto_on_subscribe)"

    tracker_launch = PACKAGE_ROOT.parent / "ekf-multi-object-tracking" / "launch" / "ekf_multi_class_object_tracking.launch"
    tracker = ET.parse(tracker_launch).getroot()
    assert tracker.find("arg[@name='diagnostics_auto_on_subscribe']").attrib["default"] == "true"
    tracker_node = tracker.find("node")
    assert tracker_node.find("param[@name='diagnostics_auto_on_subscribe']").attrib["value"] == "$(arg diagnostics_auto_on_subscribe)"
    for launch in (wrapper, pipeline, prediction, tracker):
        names = [item.attrib["name"] for item in launch.findall("arg")]
        assert len(names) == len(set(names)), "duplicate top-level launch arguments"
        for node in launch.iter("node"):
            params = [item.attrib["name"] for item in node.findall("param")]
            assert len(params) == len(set(params)), "duplicate node parameters"
