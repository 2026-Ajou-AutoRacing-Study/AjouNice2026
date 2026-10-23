"""CPU-only preflight tests; never run Docker, CUDA, ROS or network commands."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import preflight_bevfusion_competition as preflight
from bevfusion_production_profile import profile


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
    def tearDown(self): self.temp.cleanup()
    def write(self, name, text):
        path = self.root/name; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text); return path

    def test_existing_production_default_and_explicit_rollback_are_preserved(self):
        self.assertEqual(profile()['model'], 'e24')
        self.assertTrue(profile()['base5'])
        self.assertEqual(profile('e25')['model'], 'e25')
        self.assertNotIn('e12', preflight.ENV_KEYS)

    def test_local_environment_is_parsed_without_execution_or_credentials(self):
        path = self.write('.env.bevfusion.local', 'SECRET=do_not_print\n'
                          'BEVFUSION_CHECKPOINT="$HOME/weights/model.pth"\n'
                          'export BEVFUSION_IMAGE=fixture:offline\n')
        env, errors = preflight.local_overrides(path, {}, self.root, self.root/'home')
        self.assertFalse(errors)
        self.assertNotIn('SECRET', env)
        self.assertEqual(env['BEVFUSION_CHECKPOINT'], str(self.root/'home/weights/model.pth'))
        self.assertEqual(env['BEVFUSION_IMAGE'], 'fixture:offline')

    def test_shell_substitution_controls_and_unknown_variables_are_not_executed(self):
        path = self.write('.env.bevfusion.local', 'BEVFUSION_IMAGE=$(touch /tmp/never_execute)\n'
                          'if true; then\nBEVFUSION_CHECKPOINT=$UNKNOWN/model.pth\nfi\n')
        env, errors = preflight.local_overrides(path, {}, self.root, self.root)
        self.assertIn('BEVFUSION_IMAGE', errors)
        self.assertIn('BEVFUSION_CHECKPOINT', errors)
        self.assertIn('dynamic_shell_statement', errors)
        self.assertFalse(env)

    def test_flat_bridge_yaml_handles_current_scalar_and_camera_list_styles(self):
        path = self.write('bridge.yaml', 'camera_order:\n  - camera_front\n  - camera_back\n'
                          'intrinsic_source: horizontal_fov\nlatest_frame_only: true\n'
                          'server_port: 29500\nlidar_deskew_mode: off\n')
        result = preflight.flat_yaml(path)
        self.assertEqual(result['camera_order'], ['camera_front', 'camera_back'])
        self.assertIs(result['latest_frame_only'], True)
        self.assertEqual(result['server_port'], 29500)
        self.assertEqual(result['lidar_deskew_mode'], 'off')
        path.write_text('camera_order: [camera_front, camera_back]\n')
        self.assertEqual(preflight.flat_yaml(path)['camera_order'], ['camera_front', 'camera_back'])

    def test_duplicate_nested_bridge_yaml_is_rejected(self):
        for text in ('server_port: 1\nserver_port: 2\n', 'nested:\n  key: value\n'):
            path = self.write('bridge.yaml', text)
            with self.assertRaises(ValueError): preflight.flat_yaml(path)

    def test_model_config_parents_are_inspected_not_executed(self):
        parent = self.write('parent.py', 'raise RuntimeError("never executed")\n')
        child = self.write('child.py', '_base_ = "parent.py"\n')
        self.assertEqual(preflight.config_files(child), {parent.resolve(), child.resolve()})
        child.write_text('_base_ = "missing.py"\n')
        with self.assertRaises(OSError): preflight.config_files(child)

    def sensor(self):
        return dict(cameraList=[dict(cc=dict(cameraResWidth=1280, cameraResHeight=720, cameraFOV=90,
            rosConfig=dict(frameID=frame, Topic='/'+frame)), pos=dict(x=0,y=0,z=1), rot=dict(roll=0,pitch=0,yaw=0))
            for frame in preflight.CAMERAS],
            Lidar3DList=[dict(lc=dict(rosConfig=dict(frameID='lidar', Topic='/points_raw')))])

    def test_sensor_camera_order_and_finite_geometry(self):
        data = self.sensor(); path = self.write('Sensor.json', json.dumps(data))
        self.assertEqual(preflight.sensor_contract(path)['camera_order'], preflight.CAMERAS)
        data['cameraList'].reverse(); path.write_text(json.dumps(data))
        with self.assertRaises(ValueError): preflight.sensor_contract(path)
        data = self.sensor(); data['cameraList'][0]['pos']['x'] = float('nan')
        path.write_text(json.dumps(data))
        with self.assertRaises(ValueError): preflight.sensor_contract(path)

    def test_launch_semantics_ignore_comments_and_formatting_not_argument_values(self):
        source = self.write('source.launch', '<launch><!-- old --><arg name="x" default="1"/></launch>')
        installed = self.write('install.launch', '<launch>\n <arg default="1" name="x"/>\n</launch>')
        self.assertEqual(preflight.xml_contract(source), preflight.xml_contract(installed))
        installed.write_text('<launch><arg name="x" default="2"/></launch>')
        self.assertNotEqual(preflight.xml_contract(source), preflight.xml_contract(installed))

    def test_inspection_commands_are_read_only_and_do_not_require_cloud(self):
        commands = []
        def runner(command, **kwargs):
            commands.append(command)
            if command[:2] == ['nvidia-smi', '--query-gpu=name,memory.total,driver_version']:
                return 0, 'NVIDIA GeForce RTX 5060 Ti, 16311, 570.0\n'
            return 127, ''
        report = preflight.inspect_startup(self.root, environment={'WANDB_API_KEY': 'never_print'},
                                          home=self.root, runner=runner)
        serialized = json.dumps(report)
        self.assertNotIn('never_print', serialized)
        self.assertNotIn('WANDB', serialized)
        self.assertFalse(report['cloud_access_required'])
        self.assertEqual(report['selected']['model'], 'e24')
        self.assertFalse(any('run' in command or 'pull' in command or 'stop' in command or 'login' in command
                             for command in commands))
        gpu_checks = [row for row in report['checks'] if row['check'] == 'nvidia_gpu_driver']
        self.assertEqual(gpu_checks[0]['level'], 'pass')
        self.assertEqual([row for row in report['checks'] if row['check'] == 'gpu_target'][0]['level'], 'warning')
        self.assertIn('--safe', report['safe_start_command'])

    def test_safe_image_overrides_optional_acceleration_without_model_promotion(self):
        report = preflight.inspect_startup(self.root, model='e25', safe=True, home=self.root,
            environment={'BEVFUSION_IMAGE': 'optional:engine', 'BEVFUSION_CAMERA_TENSORRT_POLICY': 'require'},
            runner=lambda *_args, **_kwargs: (127, ''))
        self.assertEqual(report['selected']['model'], 'e25')
        self.assertEqual(report['selected']['image'], preflight.SAFE_IMAGE)
        self.assertEqual(report['selected']['camera_tensorrt_policy'], 'off')

    def test_missing_optional_engine_warns_for_prefer_but_blocks_require(self):
        for policy, level in [('prefer', 'warning'), ('require', 'blocker')]:
            report = preflight.inspect_startup(self.root, home=self.root,
                environment={'BEVFUSION_CAMERA_TENSORRT_POLICY': policy},
                runner=lambda *_args, **_kwargs: (127, ''))
            row = next(row for row in report['checks'] if row['check'] == 'acceleration_static_contract')
            self.assertEqual(row['level'], level)
            self.assertEqual(row['fallback_to_eager_allowed'], policy == 'prefer')

    def test_metadata_only_request_matches_server_default_prefer_policy(self):
        report = preflight.inspect_startup(self.root, home=self.root,
            environment={'BEVFUSION_CAMERA_TENSORRT_METADATA': 'missing.json'},
            runner=lambda *_args, **_kwargs: (127, ''))
        self.assertEqual(report['selected']['camera_tensorrt_policy'], 'prefer')
        row = next(row for row in report['checks'] if row['check'] == 'acceleration_static_contract')
        self.assertEqual(row['level'], 'warning')
        self.assertTrue(row['fallback_to_eager_allowed'])

    def test_unknown_acceleration_policy_never_silently_guessed(self):
        report = preflight.inspect_startup(self.root, home=self.root,
            environment={'BEVFUSION_CAMERA_TENSORRT_POLICY': 'mystery'},
            runner=lambda *_args, **_kwargs: (127, ''))
        row = next(row for row in report['checks'] if row['check'] == 'acceleration_policy')
        self.assertEqual(row['level'], 'blocker')

    def test_smoke_command_only_our_rm_network_disabled_readonly_container(self):
        report = dict(selected=dict(model='e24', checkpoint='/weights/E24.pth',
            sensor='/configs/Sensor.json', image='local:fixture'), _cache_volume='existing_cache')
        command = preflight.smoke_command(report, self.root, name='owned-preflight')
        self.assertEqual(command[:3], ['docker', 'run', '--rm'])
        self.assertEqual(command[command.index('--network')+1], 'none')
        self.assertIn('--read-only', command)
        self.assertIn('ajounice.preflight.owner=owned-preflight', command)
        self.assertIn('type=volume,src=existing_cache,dst=/root/.cache/torch_extensions', command)
        mounts = [command[i+1] for i, value in enumerate(command) if value == '--mount']
        self.assertTrue(all(mount.endswith(',readonly') for mount in mounts if mount.startswith('type=bind')))
        self.assertFalse(any(value in command for value in ['--publish', '-p', '--privileged', 'roslaunch', 'up']))

    def test_existing_receipt_rejected_before_any_inspection(self):
        receipt = self.write('existing.json', '{}')
        with patch.object(sys, 'argv', ['preflight', '--receipt', str(receipt)]), \
                patch.object(preflight, 'inspect_startup') as inspect, self.assertRaises(SystemExit):
            preflight.main()
        inspect.assert_not_called()
        self.assertEqual(receipt.read_text(), '{}')


if __name__ == '__main__':
    unittest.main()
