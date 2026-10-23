"""Production setup contract tests; standard library only, no Docker/GPU calls."""
from copy import deepcopy
import importlib.util
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from types import ModuleType
import sys
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('production_trt_setup',
    ROOT/'docker/bevfusion/scripts/setup_production_tensorrt.py')
setup = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(setup)


class SetupTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base/'repo'; self.root.mkdir()
        self.checkpoint = self.base/'E12.pth'; self.checkpoint.write_bytes(b'e12-fixture')
        self.sensor = self.base/'Sensor.json'
        self.sensor.write_text(json.dumps(dict(cameraList=[dict(cc=dict(rosConfig=dict(frameID=name)))
            for name in setup.camera_order()], Lidar3DList=[{}])))
        for relative in setup.SOURCE_FILES:
            path = self.root/relative; path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('frozen source fixture\n')
        self.artifacts = self.base/'artifacts'
        self.gpu = dict(name='NVIDIA GeForce RTX 4090', uuid='GPU-test', compute_capability=[8, 9], host_index=0)
        self.pinpatch = patch.multiple(setup, E12_SHA=setup.digest(self.checkpoint),
                                      CONFIG_SHA=setup.digest(self.root/setup.CONFIG))
        self.pinpatch.start(); self.addCleanup(self.pinpatch.stop)

    def args(self):
        return SimpleNamespace(root=self.root, checkpoint=self.checkpoint, sensor_config=self.sensor,
            artifact_root=self.artifacts, image=setup.IMAGE, gpu=0)

    def fixture_artifacts(self):
        directory = self.artifacts/'e12/build-test'; directory.mkdir(parents=True)
        engine = directory/'camera.engine'; engine.write_bytes(b'trusted-local-engine-fixture')
        metadata = dict(schema='base5_camera_tensorrt_v1', stage='engine',
            contract=dict(checkpoint_sha256=setup.E12_SHA, config_sha256=setup.CONFIG_SHA,
                input_name='images', input_semantics='normalized_flattened_BNCHW',
                input_shape=[4, 3, 512, 1056], input_dtype='float32',
                output_cast_policy='float16_after_neck', output_kind='tuple', outputs=[
                    dict(name='camera_feature_0', shape=[4, 256, 64, 132], dtype='float16'),
                    dict(name='camera_feature_1', shape=[4, 256, 32, 66], dtype='float16')]),
            engine=dict(sha256=setup.digest(engine), precision='fp16', tf32=False,
                fp32_layer_types=['NORMALIZATION', 'SOFTMAX'], tensorrt_version='10.13.3.9',
                torch_version='2.7.1+cu128', torch_cuda='12.8', hardware_compatibility='none',
                gpu_name=self.gpu['name'], compute_capability=[8, 9]))
        (directory/'camera.engine.json').write_text(json.dumps(metadata))
        smoke = dict(schema='bevfusion_production_tensorrt_smoke_v1',
            checkpoint_sha256=setup.E12_SHA, config_sha256=setup.CONFIG_SHA,
            tiny_cuda=True, model_warmed=True, closed=True, head_runtime=setup.HEAD,
            camera_backend='tensorrt', gpu=self.gpu)
        (directory/'SETUP_SMOKE.json').write_text(json.dumps(smoke))
        return directory, metadata, smoke

    def receipt(self, directory):
        checked = setup.validate_artifacts(directory, self.gpu)
        return dict(schema=setup.SCHEMA, complete=True, model='e12',
            checkpoint_sha256=setup.E12_SHA, config_sha256=setup.CONFIG_SHA,
            image=setup.IMAGE, image_id='sha256:image', gpu=self.gpu, head_runtime=setup.HEAD,
            source_sha256=setup.source_pins(self.root),
            engine_path=checked['engine_path'], metadata_path=checked['metadata_path'],
            engine_sha256=checked['engine_sha256'], metadata_sha256=checked['metadata_sha256'],
            container_engine_path=setup.container_path(directory/'camera.engine', self.artifacts),
            container_metadata_path=setup.container_path(directory/'camera.engine.json', self.artifacts))

    def test_dry_run_does_not_create_files_or_run_any_process(self):
        before = sorted(str(path) for path in self.base.rglob('*'))
        with patch.object(setup.subprocess, 'run', side_effect=AssertionError('No subprocess in plan')), redirect_stdout(io.StringIO()) as output:
            self.assertEqual(setup.main(['--root', str(self.root), '--checkpoint', str(self.checkpoint),
                '--sensor-config', str(self.sensor), '--artifact-root', str(self.artifacts)]), 0)
        data = json.loads(output.getvalue())
        self.assertFalse(data['execute'])
        self.assertEqual(data['model'], 'e12')
        self.assertEqual(data['head_runtime'], dict(proposal_topk=False, attention_sdpa=True, cache_bev_pos=True))
        self.assertEqual(before, sorted(str(path) for path in self.base.rglob('*')))

    def test_wrong_checkpoint_or_config_is_not_an_optional_bypass(self):
        self.checkpoint.write_bytes(b'E24 is not E12')
        with self.assertRaisesRegex(ValueError, 'frozen full E12'):
            setup.validate_inputs(self.root, self.checkpoint, self.sensor)
        self.checkpoint.write_bytes(b'e12-fixture')
        (self.root/setup.CONFIG).write_text('changed config')
        with self.assertRaisesRegex(ValueError, 'config SHA'):
            setup.validate_inputs(self.root, self.checkpoint, self.sensor)

    def test_camera_order_membership_and_lidar_count_are_required(self):
        data = json.loads(self.sensor.read_text()); data['cameraList'].pop()
        self.sensor.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'four production cameras'):
            setup.validate_inputs(self.root, self.checkpoint, self.sensor)

    def test_docker_command_is_networkless_readonly_no_implicit_pull(self):
        args = self.args(); directory = self.artifacts/'e12/build-test'
        command = setup.docker_command(args, directory, 'sha256:image')
        self.assertEqual(command[:3], ['docker', 'run', '--pull=never'])
        self.assertIn('--rm', command); self.assertIn('--network=none', command)
        self.assertIn('--read-only', command); self.assertNotIn('-p', command)
        self.assertIn('device=0', command)
        mounts = [command[i+1] for i, item in enumerate(command) if item == '--mount']
        self.assertEqual(len(mounts), 4)
        self.assertTrue(all(item.endswith(',readonly') for item in mounts[:3]))
        self.assertFalse(mounts[-1].endswith(',readonly'))
        self.assertIn('/opt/bevfusion_tensorrt/e12/build-test', command)
        self.assertIn('sha256:image', command)

    def test_exporter_exact_approved_amp_boundary(self):
        command = setup.export_command(Path('/opt/bevfusion_tensorrt/e12/build'))
        for flag, expected in (('--onnx-dtype', 'float32'), ('--output-dtype', 'float16'), ('--precision', 'fp16')):
            self.assertEqual(command[command.index(flag)+1], expected)
        self.assertIn('--build-engine', command)
        self.assertEqual(command[-4:-1], ['--fp32-layer-types', 'NORMALIZATION', 'SOFTMAX'])
        self.assertNotIn('int8', command)

    def test_paths_must_stay_under_artifact_mount(self):
        self.assertEqual(setup.container_path(self.artifacts/'e12/a', self.artifacts), '/opt/bevfusion_tensorrt/e12/a')
        with self.assertRaisesRegex(ValueError, 'escapes'):
            setup.container_path(self.base/'other/camera.engine', self.artifacts)
        self.artifacts.mkdir(); (self.artifacts/'link').symlink_to(self.base/'outside')
        with self.assertRaisesRegex(ValueError, 'escapes'):
            setup.container_path(self.artifacts/'link/engine', self.artifacts)

    def test_artifact_validation_accepts_exact_gpu_boundary_and_smoke(self):
        directory, _, _ = self.fixture_artifacts()
        result = setup.validate_artifacts(directory, self.gpu)
        self.assertEqual(result['engine_sha256'], setup.digest(directory/'camera.engine'))

    def test_artifact_validation_rejects_gpu_toolchain_or_precision_drift(self):
        directory, metadata, _ = self.fixture_artifacts()
        for section, key, bad in [('contract', 'input_dtype', 'float16'),
                ('engine', 'tensorrt_version', '10.14'), ('engine', 'tf32', True),
                ('engine', 'fp32_layer_types', []), ('engine', 'compute_capability', [12, 0]),
                ('engine', 'gpu_name', 'NVIDIA GeForce RTX 5060 Ti')]:
            changed = deepcopy(metadata); changed[section][key] = bad
            (directory/'camera.engine.json').write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                setup.validate_artifacts(directory, self.gpu)

    def test_failed_smoke_cannot_be_published_as_ready(self):
        directory, _, smoke = self.fixture_artifacts()
        for key, bad in [('tiny_cuda', False), ('model_warmed', False), ('closed', False),
                         ('camera_backend', 'eager'), ('head_runtime', dict(setup.HEAD, proposal_topk=True))]:
            changed = dict(smoke); changed[key] = bad
            (directory/'SETUP_SMOKE.json').write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, 'warmup'):
                setup.validate_artifacts(directory, self.gpu)

    def test_corrupt_engine_hash_is_rejected(self):
        directory, _, _ = self.fixture_artifacts()
        (directory/'camera.engine').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'SHA'):
            setup.validate_artifacts(directory, self.gpu)

    def test_existing_setup_is_idempotent_only_for_same_gpu_image_and_source(self):
        directory, _, _ = self.fixture_artifacts()
        value = self.receipt(directory); receipt = self.artifacts/'e12/runtime.json'
        receipt.write_text(json.dumps(value))
        args = (receipt, self.artifacts, 'sha256:image', self.gpu, setup.source_pins(self.root))
        self.assertEqual(setup.existing_receipt(*args), value)
        with self.assertRaisesRegex(ValueError, 'Existing setup differs'):
            setup.existing_receipt(receipt, self.artifacts, 'sha256:other', self.gpu, setup.source_pins(self.root))
        (self.root/setup.EXPORTER).write_text('changed')
        with self.assertRaisesRegex(ValueError, 'Existing setup differs'):
            setup.existing_receipt(receipt, self.artifacts, 'sha256:image', self.gpu, setup.source_pins(self.root))

    def test_execute_failure_keeps_previous_receipt(self):
        self.artifacts.mkdir(); (self.artifacts/'e12').mkdir()
        receipt = self.artifacts/'e12/runtime.json'; receipt.write_bytes(b'previous receipt')
        with patch.object(setup, 'inspect_image', return_value='sha256:image'), patch.object(setup, 'host_gpu', return_value=self.gpu), patch.object(setup.subprocess, 'run', side_effect=RuntimeError('build failed')):
            with self.assertRaisesRegex(RuntimeError, 'build failed'):
                setup.main(['--root', str(self.root), '--checkpoint', str(self.checkpoint),
                    '--sensor-config', str(self.sensor), '--artifact-root', str(self.artifacts), '--execute', '--rebuild'])
        self.assertEqual(receipt.read_bytes(), b'previous receipt')

    def test_timeout_cleanup_requires_our_exact_container_owner_label(self):
        output = Path('/tmp/build-owned')
        with patch.object(setup, '_checked', return_value='{"ai.ajounice.setup.owner":"someone-else"}') as call:
            setup.cleanup_owned_container(output)
            self.assertEqual(call.call_count, 1)
        with patch.object(setup, '_checked', side_effect=['{"ai.ajounice.setup.owner":"build-owned"}', '']) as call:
            setup.cleanup_owned_container(output)
            self.assertEqual(call.call_count, 2)
            self.assertEqual(call.call_args_list[-1].args[0],
                ['docker', 'rm', '--force', 'bevfusion-trt-setup-build-owned'])

    def test_receipt_publication_preserves_previous_and_has_no_temporary_files(self):
        path = self.base/'runtime.json'; path.write_text('{"previous":true}')
        setup.publish_receipt(path, {'complete': True})
        self.assertEqual(json.loads(path.read_text()), {'complete': True})
        backups = list(self.base.glob('runtime.previous-*.json'))
        self.assertEqual(len(backups), 1)
        self.assertEqual(json.loads(backups[0].read_text()), {'previous': True})
        self.assertEqual(list(self.base.glob('.runtime-*')), [])

    def test_recipe_keeps_frozen_toolchain_and_restricts_build_context(self):
        recipe = (ROOT/'docker/bevfusion/Dockerfile.production-tensorrt').read_text()
        ignore = (ROOT/'docker/bevfusion/Dockerfile.production-tensorrt.dockerignore').read_text()
        self.assertIn('ARG BASE_IMAGE=ajounice/bevfusion:team-cu128-v1', recipe)
        self.assertIn('--no-cache-dir --no-deps', recipe)
        self.assertIn("tensorrt.__version__=='10.13.3.9'", recipe)
        self.assertTrue(ignore.startswith('**\n'))
        for forbidden in ('!artifacts/', '!Downloads/', '!credentials/', '!third_party/'):
            self.assertNotIn(forbidden, ignore)
        self.assertNotIn('FROM pytorch/', recipe)

    def test_worker_uses_actual_three_key_head_report_and_normal_model_initialization(self):
        state = {'inference': False, 'closed': False}

        class Context:
            def __enter__(self): state['inference'] = True
            def __exit__(self, *_): state['inference'] = False

        class Runtime:
            def __init__(self, *_args, **kwargs):
                self_test.assertFalse(state['inference'], 'Model must not create inference-only bev_pos tensors')
                self_test.assertTrue(kwargs['head_attention_sdpa'])
                self_test.assertTrue(kwargs['head_cache_bev_pos'])
                self.warmed_up = True; self.camera_tensorrt = object()
                self.head_runtime_report = dict(enabled=True, active_flags=dict(setup.HEAD))
            def execution_status(self): return {'camera_tensorrt': {'enabled': True}}
            def close(self): state['closed'] = True

        self_test = self
        fake_torch = SimpleNamespace(__version__='2.7.1+cu128', version=SimpleNamespace(cuda='12.8'),
            cuda=SimpleNamespace(get_device_name=lambda _: self.gpu['name'], get_device_capability=lambda _: (8, 9)),
            backends=SimpleNamespace(cuda=SimpleNamespace(matmul=SimpleNamespace()), cudnn=SimpleNamespace()),
            set_num_threads=lambda _: None, inference_mode=Context,
            ones=lambda *a, **k: SimpleNamespace(sum=lambda: SimpleNamespace(item=lambda: 256)))
        modules = dict(torch=fake_torch, cv2=SimpleNamespace(setNumThreads=lambda _: None),
            onnx=SimpleNamespace(__version__='1.16.0'), tensorrt=SimpleNamespace(__version__='10.13.3.9'))
        modules.update({'app': ModuleType('app'), 'app.bevfusion_model': SimpleNamespace(BevFusionModel=Runtime),
            'app.sensor4_calibration': SimpleNamespace(load_sensor4_calibration=lambda *_: {})})
        output = self.base/'worker'; output.mkdir()
        with patch.dict(sys.modules, modules), patch.object(setup.subprocess, 'run'), patch.object(setup, 'digest', return_value='sensor'), redirect_stdout(io.StringIO()):
            setup.container_build(output)
        result = json.loads((output/'SETUP_SMOKE.json').read_text())
        self.assertTrue(result['model_warmed']); self.assertTrue(result['closed'])
        self.assertTrue(state['closed'])
        self.assertEqual(result['head_runtime'], dict(proposal_topk=False, attention_sdpa=True, cache_bev_pos=True))


if __name__ == '__main__':
    unittest.main()
