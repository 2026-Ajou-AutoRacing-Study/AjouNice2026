"""The planner's existing stop_margin is the sole operating margin source."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
MODULE = ROOT/'planning_ws/src/autoware_behavior_velocity_traffic_light_module'
TRAFFIC = ROOT/'base_ws/src/perception/traffic_light_detection'


@pytest.mark.parametrize('name', ('behavior_velocity_modules.yaml', 'behavior_velocity_modules_normal.yaml'))
def test_all_missions_use_existing_common_margin(name):
    config = yaml.safe_load((ROOT/'planning_ws/src/autoware_launch/config/ros1/planning'/name).read_text())
    assert config['traffic_light']['stop_margin'] == 0.
    assert 'signal_stop_line_contracts' not in config['traffic_light']


def test_component_fallback_defaults_match_the_common_margin():
    ros1 = yaml.safe_load((MODULE/'config/ros1/traffic_light.yaml').read_text())
    ros2 = yaml.safe_load((MODULE/'config/traffic_light.param.yaml').read_text())
    assert ros1['traffic_light/stop_margin'] == 0.
    assert ros2['/**']['ros__parameters']['traffic_light']['stop_margin'] == 0.


@pytest.mark.parametrize('path', (
    TRAFFIC/'launch/traffic_light_node.launch',
    TRAFFIC/'launch/traffic_light_v5_candidate.launch',
    ROOT/'base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch',
    ROOT/'base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch',
))
def test_launch_does_not_load_or_forward_removed_private_stop_config(path):
    text = path.read_text()
    ET.fromstring(text)
    assert 'signal_stop_line_contract' not in text
    assert 'signal_stop_line_crossing_reference' not in text


def test_removed_private_profile_cannot_double_apply_margin():
    assert not (TRAFFIC/'config/traffic_signal_stop_lines.yaml').exists()
    for path in (MODULE/'src/scene.cpp', MODULE/'src/experimental/scene.cpp'):
        text = path.read_text()
        assert 'additional_stop_margin' not in text
        assert 'qualifyStopLineContract' not in text
        assert 'planner_param_.stop_margin' in text
