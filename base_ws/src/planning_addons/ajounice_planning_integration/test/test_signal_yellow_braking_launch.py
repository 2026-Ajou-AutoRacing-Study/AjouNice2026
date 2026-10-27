"""Yellow launch contract; XML resolution never starts nodes or contacts a master."""
import importlib.util
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[5]
LAUNCH = ROOT/'base_ws/src/planning_addons/ajounice_planning_integration/launch'
KEY = 'signal_compliance_yellow_deceleration_mps2'
JERK_KEY = 'signal_compliance_yellow_jerk_mps3'
PREFIX = '/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light/'


class YellowBrakingLaunchTest(unittest.TestCase):
    def test_yellow_master_remains_off_and_deceleration_is_a_separate_parameter(self):
        for filename in ('planning.launch', 'morai_planning_stack.launch'):
            with self.subTest(filename=filename):
                root = ET.parse(LAUNCH/filename).getroot()
                arguments = {row.get('name'): row.get('default') for row in root.findall('arg')}
                self.assertEqual(arguments['enable_yellow_passthrough'], 'false')
                self.assertEqual(arguments[KEY], '6.0')
                self.assertEqual(arguments[JERK_KEY], '10.0')
        root = ET.parse(LAUNCH/'planning.launch').getroot()
        parameters = {row.get('name'): row for row in root.findall('param')}
        self.assertEqual(parameters[PREFIX+KEY].get('value'), '$(arg '+KEY+')')
        self.assertEqual(parameters[PREFIX+KEY].get('type'), 'double')
        self.assertEqual(parameters[PREFIX+JERK_KEY].get('value'), '$(arg '+JERK_KEY+')')
        self.assertEqual(parameters[PREFIX+JERK_KEY].get('type'), 'double')
        self.assertEqual(parameters[PREFIX+'signal_compliance_enable_timed_yellow'].get('value'), 'false')
        # The user's yellow-only estimate must not silently tighten planning or control limits.
        config = yaml.safe_load((ROOT/'planning_ws/src/autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml').read_text())
        self.assertEqual(config['max_accel'], -6.)
        self.assertEqual(config['delay_response_time'], 0.)

    def test_zero_response_delay_preserves_each_profile_braking_limits(self):
        profiles = {'behavior_velocity_modules.yaml': (-6., -10.),
                    'behavior_velocity_modules_normal.yaml': (-2.8, -5.)}
        for filename, limits in profiles.items():
            with self.subTest(profile=filename):
                config = yaml.safe_load((ROOT/'planning_ws/src/autoware_launch/config/ros1/planning'/filename).read_text())
                self.assertEqual(config['delay_response_time'], 0.)
                self.assertEqual(config['system_delay'], .5)
                self.assertEqual((config['max_accel'], config['max_jerk']), limits)

    def test_full_stack_forwards_deceleration_without_changing_the_master(self):
        root = ET.parse(LAUNCH/'morai_planning_stack.launch').getroot()
        planning = next(row for row in root.findall('include')
                        if row.get('file', '').endswith('/launch/planning.launch'))
        forwarded = {row.get('name'): row.get('value') for row in planning.findall('arg')}
        self.assertEqual(forwarded[KEY], '$(arg '+KEY+')')
        self.assertEqual(forwarded[JERK_KEY], '$(arg '+JERK_KEY+')')
        self.assertEqual(forwarded['enable_yellow_passthrough'], '$(arg enable_yellow_passthrough)')


@unittest.skipUnless(os.environ.get('AJOUNICE_TEST_LAUNCH_DUMP') == '1',
                     'Set AJOUNICE_TEST_LAUNCH_DUMP=1 for read-only source launch resolution')
class ResolvedYellowBrakingLaunchTest(unittest.TestCase):
    def test_timed_compatibility_flag_does_not_reactivate_old_policy(self):
        spec = importlib.util.spec_from_file_location('yellow_launch_dump_helper',
            ROOT/'scripts/runtime/test_traffic_light_unknown_grace_release.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        for name in ('planning', 'stack'):
            with self.subTest(launch=name):
                params = module.launch_dump(name, ['start_requester:=false',
                    'enable_yellow_passthrough:=true',
                    KEY+':=7.5', JERK_KEY+':=14.0', 'signal_compliance_enable_timed_yellow:=true',
                    'signal_compliance_min_yellow_sec:=-1.0',
                    'signal_compliance_yellow_time_margin_sec:=20.0'])
                self.assertEqual(params[PREFIX+KEY], 7.5)
                self.assertEqual(params[PREFIX+JERK_KEY], 14.)
                self.assertIs(params[PREFIX+'enable_yellow_passthrough'], True)
                self.assertIs(params[PREFIX+'signal_compliance_enable_timed_yellow'], False)
                common = PREFIX.rsplit('traffic_light/', 1)[0]
                self.assertEqual(params[common+'delay_response_time'], 0.)
                self.assertEqual(params[common+'system_delay'], .5)


if __name__ == '__main__':
    unittest.main()
