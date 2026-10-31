"""Environment checks must not change YAMLs, launch a stack or assume connectivity."""
import copy
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from planning_scenario_validator import doctor, gui
from planning_scenario_validator.config import load_campaign


class DoctorTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_campaign(gui.MISSIONS / 'highway/base/example.yaml', check_install=False)
        self.cfg = copy.deepcopy(self.cfg)
        self.cfg['stack'].update(detector='od_gt', traffic_light=False)

    def test_default_is_non_mutating_and_does_not_start_gpu_or_topics(self):
        before = self.cfg['path'].read_bytes()
        with patch.object(doctor, 'command', return_value=(True, 'ok')) as command, patch.object(doctor, 'observe_topics') as topics:
            rows = doctor.inspect_environment(self.cfg)
        topics.assert_not_called()
        self.assertFalse(any(call.args[0][:2] == ['docker', 'run'] for call in command.call_args_list))
        self.assertEqual(self.cfg['path'].read_bytes(), before)
        self.assertTrue(any(row['check'] == 'MORAI 최초 연결' and row['status'] == 'WARN' for row in rows))
        self.assertTrue(all(not row['remedy'] for row in rows if row['status'] == 'PASS'))

    def test_missing_dependencies_fail_with_remedy(self):
        with patch.object(doctor, 'command', return_value=(False, 'not installed')):
            rows = doctor.inspect_environment(self.cfg)
        failed = [row for row in rows if row['status'] == 'FAIL']
        self.assertTrue(failed)
        self.assertTrue(all(row['remedy'] for row in failed))

    def test_explicit_gpu_and_live_only(self):
        self.cfg['stack'].update(detector='bevfusion', bevfusion_model='e25')
        with patch.object(doctor, 'command', return_value=(True, 'ok')) as command, patch.object(doctor, 'bev_settings', return_value=('test:image', Path('/missing-checkpoint'), '0'*64)), patch.object(doctor, 'observe_topics', return_value=[]) as topics:
            doctor.inspect_environment(self.cfg, gpu=True, live=True)
        self.assertTrue(any(call.args[0][:2] == ['docker', 'run'] for call in command.call_args_list))
        topics.assert_called_once_with(self.cfg)

    def test_missing_managed_trt_image_warns_and_checks_same_e12_baseline(self):
        self.cfg['stack'].update(detector='bevfusion', bevfusion_model='base5_e12')
        contract = dict(image='optional:trt', checkpoint_path='/missing-e12.pth',
                        checkpoint_sha256='0' * 64, label='E12 production')
        selection = dict(image='baseline:e12', fallback=True,
                         reason='managed_optional_trt_image_unavailable_same_e12_eager')
        with patch.object(doctor, 'command', return_value=(True, 'ok')) as command, \
             patch.object(doctor, 'resolve_contract', return_value=contract), \
             patch.object(doctor, 'runtime_image_selection', return_value=selection):
            rows = doctor.inspect_environment(self.cfg)
        self.assertTrue(any(row['check'] == '선택적 TensorRT 이미지' and row['status'] == 'WARN'
                            for row in rows))
        self.assertTrue(any('baseline:e12' in call.args[0] for call in command.call_args_list))
        self.assertFalse(any(call.args[0][:2] == ['docker', 'run'] for call in command.call_args_list))

    def test_shell_settings_are_never_executed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / '.env.bevfusion.local').write_text('BEVFUSION_CHECKPOINT=$(touch /tmp/do-not-create)\n')
            with patch.object(doctor, 'ROOT', root), patch.object(doctor.subprocess, 'run') as execute:
                with self.assertRaisesRegex(ValueError, '셸 표현식'):
                    doctor.bev_settings()
                execute.assert_not_called()

    def test_missing_traffic_python_does_not_hide_other_checks(self):
        self.cfg['stack']['traffic_light'] = True
        with patch.object(doctor, 'command', return_value=(True, 'ok')), patch.object(doctor, 'runtime_settings', side_effect=ValueError('missing python')):
            rows = doctor.inspect_environment(self.cfg)
        self.assertTrue(any(row['check'] == '신호등 Python 경로' and row['status'] == 'FAIL' for row in rows))
        self.assertTrue(any(row['check'] == '센서·차량 실시간 연결' for row in rows))


if __name__ == '__main__':
    unittest.main()
