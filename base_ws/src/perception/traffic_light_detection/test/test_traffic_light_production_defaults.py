"""Static deployment contracts, not a live-driving or model accuracy test."""
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[5]
TRAFFIC = ROOT / 'base_ws/src/perception/traffic_light_detection/launch'
INTEGRATION = ROOT / 'base_ws/src/planning_addons/ajounice_planning_integration/launch'


def arguments(path):
    return {node.attrib['name']: node.attrib.get('default')
            for node in ET.parse(path).getroot().findall('arg')}


def include_arguments(path, target):
    include = next(node for node in ET.parse(path).getroot().iter('include')
                   if node.attrib['file'].endswith('/'+target))
    return {node.attrib['name']: node.attrib['value'] for node in include.findall('arg')}


def test_ordinary_and_v5_entrypoints_enable_the_same_production_observation_contract():
    for filename in ('traffic_light_node.launch', 'traffic_light_v5_candidate.launch'):
        args = arguments(TRAFFIC / filename)
        assert args['planner_compliance_observations'] == 'true'
        assert args['route_progress_guard'] == 'true'
        assert args['publish_debug_image'] == 'false'
        assert args['low_confidence_hold_sec'] == '0.3'
        assert args['visual_continuation_sec'] == '0.3'
        assert args['inference_python'].endswith('/.local/share/ajounice/traffic_light_gpu/bin/python')


def test_explicit_legacy_model_keeps_its_original_observation_filter_and_route_matching():
    args = include_arguments(TRAFFIC/'traffic_light_legacy.launch', 'traffic_light_node.launch')
    assert args['planner_compliance_observations'] == args['route_progress_guard'] == 'false'
    assert args['near_rescue_enabled'] == 'false'
    assert args['selection_mode'] == 'legacy_center'
    assert args['classifier_mode'] == 'hsv'
    assert args['detector_imgsz'] == '640'


def test_planning_default_is_compliance_with_explicit_front_axle_and_freshness_not_yellow_period():
    args = arguments(INTEGRATION/'planning.launch')
    assert args['enable_signal_compliance_guard'] == 'true'
    assert args['enable_yellow_passthrough'] == 'false'
    assert args['signal_compliance_front_axle_offset_m'] == '3.0'
    assert args['traffic_light_state_timeout_sec'] == '1.0'
    params = {node.attrib['name']: node.attrib.get('value')
              for node in ET.parse(INTEGRATION/'planning.launch').getroot().findall('param')}
    suffix = '/behavior_velocity_planner/traffic_light/'
    assert any(name.endswith(suffix+'enable_signal_compliance_guard') and
               value == '$(arg enable_signal_compliance_guard)' for name, value in params.items())
    assert any(name.endswith(suffix+'tl_state_timeout') and
               value == '$(arg traffic_light_state_timeout_sec)' for name, value in params.items())
    assert any(name.endswith(suffix+'enable_yellow_passthrough') and
               value == '$(arg enable_yellow_passthrough)' for name, value in params.items())
    assert args['planning_rosconsole_config'].endswith('/config/lane_change_diagnostics.rosconsole')
    envs = list(ET.parse(INTEGRATION/'planning.launch').getroot().iter('env'))
    assert any(node.attrib.get('name') == 'ROSCONSOLE_CONFIG_FILE' and
               node.attrib.get('value') == '$(arg planning_rosconsole_config)' for node in envs)


def test_integrated_stack_forwards_policy_and_local_or_lan_sensor_settings():
    path = INTEGRATION/'morai_planning_stack.launch'
    args = arguments(path)
    assert args['traffic_light_inference_python'].endswith('/.local/share/ajounice/traffic_light_gpu/bin/python')
    traffic = include_arguments(path, 'traffic_light_node.launch')
    planning = include_arguments(path, 'planning.launch')
    for name in ('planner_compliance_observations', 'route_progress_guard'):
        assert args[name] == 'true'
        assert traffic[name] == '$(arg '+name+')'
    assert traffic['sensor_config'] == '$(arg sensor_config)'
    assert args['enable_signal_compliance_guard'] == 'true'
    assert planning['enable_signal_compliance_guard'] == '$(arg enable_signal_compliance_guard)'
    assert args['enable_yellow_passthrough'] == 'false'
    assert planning['enable_yellow_passthrough'] == '$(arg enable_yellow_passthrough)'
    assert args['traffic_light_debug_image'] == 'false'
    assert traffic['publish_debug_image'] == '$(arg traffic_light_debug_image)'
    assert args['traffic_light_state_timeout_sec'] == '1.0'
    assert planning['traffic_light_state_timeout_sec'] == '$(arg traffic_light_state_timeout_sec)'
