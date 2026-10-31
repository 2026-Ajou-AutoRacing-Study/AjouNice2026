"""Production launch ownership; no ROS processes or live parameters changed."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[4]
PIPE=ROOT/'base_ws/src/perception_object_pipeline/launch'

def test_operating_defaults_and_forwarding():
    for path in [PIPE/'object_pipeline.launch',PIPE/'tracking_prediction.launch',
                 ROOT/'base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch']:
        tree=ET.parse(path).getroot()
        args={a.get('name'):a.get('default') for a in tree.findall('arg')}
        assert args['hold_stationary_output_pose']=='true'
        assert args['planning_ownership_config'].endswith('/planning_ws/src/autoware_launch/config/ros1/planning/behavior_path_modules.yaml')
        forwarded=[a for a in tree.findall('.//include/arg') if a.get('name')=='hold_stationary_output_pose']
        assert len(forwarded)==1 and forwarded[0].get('value')=='$(arg hold_stationary_output_pose)'
        assert any(a.get('name')=='planning_ownership_config' and a.get('value')=='$(arg planning_ownership_config)' for a in tree.findall('.//include/arg'))

def test_unknown_hold_respects_existing_motion_policy_opt_out():
    tree=ET.parse(PIPE/'object_pipeline.launch').getroot()
    args=[a for a in tree.findall('.//include/arg') if a.get('name')=='hold_unknown_coasting_pose']
    assert len(args)==1
    assert args[0].get('value')=="$(eval arg('hold_stationary_output_pose') and arg('force_unknown_stationary'))"

def test_no_duplicate_numeric_planner_thresholds_in_pipeline():
    for name in ('object_pipeline.launch','tracking_prediction.launch'):
        text=(PIPE/name).read_text()
        assert 'output_vehicle_dynamic_enter_mps' not in text
        assert 'output_pedestrian_dynamic_enter_mps' not in text
        assert 'output_static_enter_mps' not in text
