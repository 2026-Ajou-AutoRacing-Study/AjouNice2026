"""CPU-only production selection/command checks; no ROS master or GPU starts."""
import copy
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from planning_scenario_validator import gui, models
from planning_scenario_validator.config import ROOT
from planning_scenario_validator.executions import prepare_execution, load_execution
from planning_scenario_validator.topic_contract import model_topics


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('e12_trial_test', PACKAGE / 'scripts/run_bevfusion_trial.py')
TRIAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRIAL)


class E12ProductionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.temporary = Path(temporary.name)
        environment = patch.dict(os.environ, {
            'BEVFUSION_ACCELERATION_RECEIPT': str(self.temporary / 'absent.json')}, clear=True)
        environment.start()
        self.addCleanup(environment.stop)
        self.sensor = ROOT / 'configs/sensors/Sensor_competition_local.json'
        self.stack = dict(detector='bevfusion', bevfusion_model='base5_e12')

    def contract(self):
        return models.resolve_contract(self.stack, self.sensor, ROOT, require_assets=False)

    def test_default_gui_and_full_course_templates_select_e12(self):
        self.assertEqual(models.DEFAULT_MODEL, 'base5_e12')
        self.assertEqual(models.selected_model({}), 'base5_e12')
        for name in ('example.yaml', 'sample_only.yaml', 'base5_e12_analysis_sample.yaml',
                     'base5_e12_analysis_variants.yaml'):
            document = gui.load_document(gui.MISSIONS / 'full_course/base' / name)
            self.assertEqual(models.selected_model(document['stack']), 'base5_e12')
            self.assertEqual(gui.form_values(document)['bevfusion_model'], models.PRESETS['base5_e12']['label'])
        self.assertEqual(models.selected_model(gui.load_document(
            gui.MISSIONS / 'full_course/base/base5_e24_analysis_sample.yaml')['stack']), 'base5_e24')

    def test_e12_identity_geometry_thresholds_and_missing_engine_are_explicit(self):
        contract = self.contract()
        self.assertEqual(contract['checkpoint_sha256'],
            'f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8')
        self.assertEqual(contract['checkpoint_path'], str(
            Path.home() / 'bevfusion_artifacts/checkpoints/base5_e12_inference.pth'))
        self.assertTrue(contract['model_config'].endswith('nearest_depth_runtime.py'))
        self.assertEqual(contract['depth_raster_policy'], 'nearest_positive')
        self.assertEqual(contract['box_center_output_origin'], 'gravity_center_v1')
        self.assertEqual(contract['thresholds'], dict(vehicle=.4, pedestrian=.1, obstacle=.1))
        acceleration = contract['acceleration']
        self.assertEqual(acceleration['camera_tensorrt_policy'], 'off')
        self.assertTrue(acceleration['head_attention_sdpa'])
        self.assertTrue(acceleration['head_cache_bev_pos'])
        self.assertFalse(acceleration['selection']['receipt_loaded'])
        self.assertEqual(model_topics('base5_e12'), model_topics('base5_e24'))

    def test_e12_trial_requests_heads_with_same_e12_and_no_unprepared_engine(self):
        contract = self.contract()
        command, bridge = models.trial_commands(contract, ROOT, 'e12-test')
        for flag in ('--head-attention-sdpa', '--head-cache-bev-pos', '--amp', '--parallel-camera'):
            self.assertIn(flag, command)
        self.assertEqual(command[command.index('--camera-tensorrt-policy') + 1], 'off')
        self.assertNotIn('--camera-tensorrt-engine', command)
        self.assertIn(contract['checkpoint_path'] + ':/opt/checkpoints/model.pth:ro', command)
        self.assertIn('box_center_output_origin:=gravity_center_v1', bridge)
        self.assertIn('output_objects:=' + models.CANONICAL, bridge)

    def test_local_artifact_hashes_and_frozen_options_reach_docker(self):
        engine = self.temporary / 'camera.engine'
        metadata = self.temporary / 'camera.engine.json'
        engine.write_bytes(b'local executable fixture, not a real engine')
        metadata.write_text('{}')
        selection = dict(environment={
            'BEVFUSION_IMAGE': 'local-e12-test:trt',
            'BEVFUSION_CAMERA_TENSORRT_ENGINE': '/opt/bevfusion_tensorrt/e12/camera.engine',
            'BEVFUSION_CAMERA_TENSORRT_METADATA': '/opt/bevfusion_tensorrt/e12/camera.engine.json',
            'BEVFUSION_CAMERA_TENSORRT_POLICY': 'prefer',
            'BEVFUSION_HEAD_ATTENTION_SDPA': 'true', 'BEVFUSION_HEAD_CACHE_BEV_POS': 'true'},
            engine_path=str(engine), metadata_path=str(metadata), receipt_loaded=True,
            artifact_root=str(self.temporary))
        selection['environment']['BEVFUSION_CAMERA_TENSORRT_ENGINE'] = '/opt/bevfusion_tensorrt/camera.engine'
        selection['environment']['BEVFUSION_CAMERA_TENSORRT_METADATA'] = '/opt/bevfusion_tensorrt/camera.engine.json'
        module = type('Helper', (), {'resolve_acceleration': staticmethod(lambda *a: selection),
                                    'SAFE_IMAGE': 'baseline-test:e12'})()
        with patch.object(models.importlib.util, 'module_from_spec', return_value=module), \
             patch.object(models.importlib.util, 'spec_from_file_location') as spec:
            spec.return_value.loader.exec_module.return_value = None
            acceleration = models.production_acceleration(ROOT)
        contract = self.contract()
        contract['acceleration'] = acceleration
        command, _ = models.trial_commands(contract, ROOT, 'engine-test')
        self.assertIn(str(engine) + ':/opt/accelerator/camera.engine:ro', command)
        self.assertIn(str(metadata) + ':/opt/accelerator/camera.engine.json:ro', command)
        self.assertEqual(command[command.index('--camera-tensorrt-engine') + 1], '/opt/accelerator/camera.engine')
        self.assertEqual(acceleration['files_sha256'][str(engine)], models.sha256(engine))

    def test_unprepared_optional_engine_never_switches_checkpoint_or_blocks_prefer(self):
        with patch.dict(os.environ, {
                'BEVFUSION_CAMERA_TENSORRT_ENGINE': str(self.temporary / 'missing.engine'),
                'BEVFUSION_CAMERA_TENSORRT_METADATA': str(self.temporary / 'missing.json'),
                'BEVFUSION_CAMERA_TENSORRT_POLICY': 'prefer'}):
            contract = self.contract()
        self.assertEqual(contract['preset'], 'base5_e12')
        self.assertEqual(contract['acceleration']['camera_tensorrt_policy'], 'off')
        self.assertIn('validator_optional_artifact_unavailable', contract['acceleration']['selection']['reason'])
        self.assertTrue(contract['acceleration']['head_attention_sdpa'])
        with patch.dict(os.environ, {'BEVFUSION_CAMERA_TENSORRT_POLICY': 'require'}):
            with self.assertRaisesRegex(ValueError, 'require 정책'):
                self.contract()

    def test_managed_image_missing_uses_baseline_with_separate_truthful_receipt(self):
        contract = self.contract()
        contract['image'] = 'optional-e12:trt'
        contract['acceleration']['selection']['managed_image'] = True
        contract['acceleration']['camera_tensorrt_policy'] = 'prefer'
        original = copy.deepcopy(contract)
        missing = subprocess.CalledProcessError(1, ['docker', 'image', 'inspect'])
        with patch.object(models.subprocess, 'check_output', side_effect=[missing, 'sha256:baseline']):
            selection = models.runtime_image_selection(contract)
        self.assertTrue(selection['fallback'])
        self.assertEqual(selection['image'], 'ajounice/bevfusion:team-cu128-v1')
        self.assertIn('same_e12_eager', selection['reason'])
        runtime = models.runtime_contract(contract, selection)
        self.assertEqual(contract, original)
        self.assertEqual(runtime['checkpoint_sha256'], contract['checkpoint_sha256'])
        self.assertEqual(runtime['preset'], 'base5_e12')
        self.assertEqual(runtime['acceleration']['camera_tensorrt_policy'], 'off')
        self.assertTrue(runtime['acceleration']['head_attention_sdpa'])
        command, _ = models.trial_commands(runtime, ROOT, 'image-fallback-test')
        self.assertIn('sha256:baseline', command)
        self.assertNotIn('--camera-tensorrt-engine', command)

    def test_explicit_bad_image_is_never_silently_replaced(self):
        contract = self.contract()
        contract['image'] = 'operator:selected'
        self.assertFalse(contract['acceleration']['selection']['managed_image'])
        missing = subprocess.CalledProcessError(1, ['docker', 'image', 'inspect'])
        with patch.object(models.subprocess, 'check_output', side_effect=missing) as inspect:
            with self.assertRaises(subprocess.CalledProcessError):
                models.runtime_image_selection(contract)
        self.assertEqual(inspect.call_count, 1)

    def test_managed_image_missing_never_downgrades_explicit_require(self):
        contract = self.contract()
        contract['image'] = 'required-e12:trt'
        contract['acceleration']['selection']['managed_image'] = True
        contract['acceleration']['camera_tensorrt_policy'] = 'require'
        original = copy.deepcopy(contract)
        missing = subprocess.CalledProcessError(1, ['docker', 'image', 'inspect'])
        with patch.object(models.subprocess, 'check_output', side_effect=missing) as inspect:
            with self.assertRaises(subprocess.CalledProcessError):
                models.runtime_image_selection(contract)
        self.assertEqual(inspect.call_count, 1)
        forced_fallback = dict(image_id='sha256:baseline', fallback=True)
        with self.assertRaisesRegex(ValueError, 'require 정책'):
            models.runtime_contract(contract, forced_fallback)
        self.assertEqual(contract, original)

    def test_actual_eager_fallback_is_not_claimed_as_trt_and_requests_are_checked(self):
        contract = self.contract()
        identity = {'execution_optimization': {
            'camera_tensorrt_startup': dict(policy='off', active_backend='eager', fallback=False),
            'camera_tensorrt': dict(enabled=False),
            'head_runtime': {'requested': dict(attention_sdpa=True, cache_bev_pos=True, proposal_topk=False)}}}
        original = copy.deepcopy(identity)
        TRIAL.check_acceleration(contract, identity)
        self.assertEqual(identity, original)
        contract['acceleration']['camera_tensorrt_policy'] = 'prefer'
        identity['execution_optimization']['camera_tensorrt_startup'].update(
            policy='prefer', fallback=True, reason_code='engine_gpu_mismatch')
        TRIAL.check_acceleration(contract, identity)
        identity['execution_optimization']['head_runtime']['requested']['attention_sdpa'] = False
        with self.assertRaisesRegex(RuntimeError, 'SDPA/cache'):
            TRIAL.check_acceleration(contract, identity)

    def test_require_policy_never_accepts_eager_or_unverified_trt(self):
        contract = self.contract()
        contract['acceleration']['camera_tensorrt_policy'] = 'require'
        identity = {'execution_optimization': {
            'camera_tensorrt_startup': dict(policy='require', active_backend='eager'),
            'camera_tensorrt': dict(enabled=False)}}
        with self.assertRaisesRegex(RuntimeError, 'backend'):
            TRIAL.check_acceleration(contract, identity)
        identity['execution_optimization']['camera_tensorrt_startup']['active_backend'] = 'camera_tensorrt'
        identity['execution_optimization']['camera_tensorrt'] = dict(enabled=True, loaded=True, healthy=True)
        with self.assertRaisesRegex(RuntimeError, 'engine/runtime'):
            TRIAL.check_acceleration(contract, identity)

    def test_snapshot_freezes_promoted_model_and_missing_weight_fails_closed(self):
        import yaml
        document = gui.load_document(gui.MISSIONS / 'highway/base/example.yaml')
        document['output_root'] = str(self.temporary / 'results')
        source = self.temporary / 'campaign.yaml'
        source.write_text(yaml.safe_dump(document))
        cfg = prepare_execution(source, check_install=False)
        resumed = load_execution(cfg['output'], check_install=False)
        self.assertEqual(resumed['stack']['bevfusion_model'], 'base5_e12')
        self.assertEqual(cfg['model_contract']['preset'], 'base5_e12')
        with patch.object(models.Path, 'home', return_value=self.temporary):
            with self.assertRaisesRegex(ValueError, 'checkpoint SHA256'):
                models.resolve_contract(self.stack, self.sensor, ROOT)


if __name__ == '__main__':
    unittest.main()
