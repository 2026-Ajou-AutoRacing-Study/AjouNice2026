"""No ROS: exercise exact argv and safe compatibility of the competition entry."""
import json
import importlib.util
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/runtime/run_competition_udp.sh'


class EntryTest(unittest.TestCase):
    def invoke(self, *args):
        env = dict(os.environ)
        env['BASH_FUNC_roslaunch%%'] = '() { /usr/bin/python3 -c \'import sys,json; print(json.dumps(sys.argv[1:]))\' "$@"; }'
        return subprocess.run(['bash', str(SCRIPT), *args], env=env, capture_output=True, text=True)

    def test_profiles_keep_sensors_and_exact_paths(self):
        for profile, suffix in (('local', 'local'), ('lan', 'lan'), ('teamlead', 'lan_jihun')):
            with self.subTest(profile=profile):
                result = self.invoke('--profile', profile)
                self.assertEqual(result.returncode, 0, result.stderr)
                argv = json.loads(result.stdout)
                self.assertIn('sensor_config:=' + str(ROOT / ('configs/sensors/Sensor_competition_' + suffix + '.json')), argv)
                self.assertIn('network_config:=' + str(ROOT / ('configs/network/ego_network_competition_' + suffix + '.json')), argv)
                self.assertEqual(argv[-2:], ['enable_ego_vehicle_status:=false', 'enable_object_info:=false'])

    def test_ros_style_profile_override_and_literal_custom_paths(self):
        args = ['sensor_config:=/home/jihun/a b.json', 'network_config:=/tmp/a;$(echo NO).json']
        result = self.invoke('--profile', 'lan', 'profile:=teamlead', *args)
        self.assertEqual(result.returncode, 0, result.stderr)
        argv = json.loads(result.stdout)
        self.assertIn('teamlead', result.stderr)
        self.assertEqual(argv[-2:], args)

    def test_cannot_enable_gt_via_competition_entry(self):
        for name in ('enable_object_info', 'enable_ego_vehicle_status'):
            for value in ('true', 'True', '1'):
                result = self.invoke(name + ':=' + value)
                self.assertEqual(result.returncode, 2)
                self.assertEqual(result.stdout, '')

    def test_false_overrides_are_accepted_without_duplicate_flags(self):
        result = self.invoke('enable_ego_vehicle_status:=false', 'enable_object_info:=0')
        self.assertEqual(result.returncode, 0, result.stderr)
        argv = json.loads(result.stdout)
        self.assertEqual(sum(a.startswith('enable_object_info:=') for a in argv), 1)

    def test_unknown_profile_or_missing_value_is_explicit(self):
        for args in (('--profile',), ('profile:=mystery',)):
            result = self.invoke(*args)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(result.stdout, '')

    def test_print_command_is_read_only(self):
        result = self.invoke('--profile', 'teamlead', '--print-command')
        self.assertEqual(result.returncode, 0)
        self.assertTrue(result.stdout.startswith('roslaunch sensor_interface '))
        self.assertNotIn('[', result.stdout)

    def test_morai_test_profile_changes_only_channel_activation(self):
        spec = importlib.util.spec_from_file_location('prepare_network', SCRIPT.with_name('prepare_competition_network.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for suffix in ('local', 'lan', 'lan_jihun'):
            source = json.loads((ROOT / ('configs/network/ego_network_competition_' + suffix + '.json')).read_text())
            original = json.dumps(source, sort_keys=True)
            candidate = module.without_gt(source)
            self.assertEqual(json.dumps(source, sort_keys=True), original)
            for old_group, new_group in zip(source['egoNetworkData'], candidate['egoNetworkData']):
                for old, new in zip(old_group['listEgoNetworkInfo'], new_group['listEgoNetworkInfo']):
                    kind = old['PUBSUB_TYPE']
                    self.assertEqual(new['config']['udpConfig'], old['config']['udpConfig'])
                    self.assertEqual(new['EgoCtrlConfig']['udpConfig'], old['EgoCtrlConfig']['udpConfig'])
                    if kind not in (261, 295):
                        self.assertEqual(new['config']['commType'], 0)
                    else:
                        self.assertEqual(new['config'], old['config'])
                    if kind == 771:
                        self.assertEqual(new['EgoCtrlConfig'], old['EgoCtrlConfig'])
                    else:
                        self.assertEqual(new['EgoCtrlConfig']['commType'], 0)
                    # After restoring the two activation fields everything must be identical.
                    new['config']['commType'] = old['config']['commType']
                    new['EgoCtrlConfig']['commType'] = old['EgoCtrlConfig']['commType']
                    self.assertEqual(old, new)


if __name__ == '__main__':
    unittest.main()
