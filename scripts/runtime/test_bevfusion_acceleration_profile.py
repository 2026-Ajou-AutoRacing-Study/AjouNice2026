import json
from pathlib import Path
import tempfile
import unittest

from bevfusion_acceleration_profile import resolve_acceleration, digest, E12_SHA, CONFIG_SHA, SAFE_IMAGE


class AccelerationProfileTests(unittest.TestCase):
    def receipt(self, home):
        root = Path(home) / 'bevfusion_artifacts/tensorrt'
        folder = root / 'e12/test_gpu'
        folder.mkdir(parents=True)
        data = dict(schema='bevfusion_production_tensorrt_setup_v1', complete=True,
                    model='e12', checkpoint_sha256=E12_SHA, config_sha256=CONFIG_SHA,
                    image='ajounice/bevfusion:team-trt-cu128-261023-v1')
        for name, filename in (('engine', 'camera.engine'), ('metadata', 'camera.engine.json')):
            path = folder / filename
            path.write_bytes(name.encode())
            data[name + '_path'] = str(path)
            data['container_' + name + '_path'] = '/opt/bevfusion_tensorrt/' + str(path.relative_to(root))
            data[name + '_sha256'] = digest(path)
        path = root / 'e12/runtime.json'
        path.write_text(json.dumps(data))
        return path, data

    def test_missing_setup_keeps_e12_with_head_on_no_cloud(self):
        with tempfile.TemporaryDirectory() as home:
            result = resolve_acceleration('e12', {}, home)
            self.assertEqual(result['environment']['BEVFUSION_IMAGE'], SAFE_IMAGE)
            self.assertEqual(result['environment']['BEVFUSION_HEAD_ATTENTION_SDPA'], 'true')
            self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_POLICY'], 'off')
            self.assertFalse(result['receipt_loaded'])
            self.assertFalse((Path(home) / 'bevfusion_artifacts').exists())

    def test_valid_receipt_uses_target_gpu_paths(self):
        with tempfile.TemporaryDirectory() as home:
            _, data = self.receipt(home)
            result = resolve_acceleration('e12', {}, home)
            self.assertTrue(result['managed_image'])
            self.assertTrue(result['receipt_loaded'])
            self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_POLICY'], 'prefer')
            self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_ENGINE'], data['container_engine_path'])

    def test_legacy_baseline_image_does_not_mask_valid_e12_receipt(self):
        with tempfile.TemporaryDirectory() as home:
            _, data = self.receipt(home)
            result = resolve_acceleration('e12', {'BEVFUSION_IMAGE': SAFE_IMAGE}, home)
            self.assertEqual(result['environment']['BEVFUSION_IMAGE'], data['image'])
            self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_POLICY'], 'prefer')
            self.assertTrue(result['managed_image'])
            self.assertEqual(result['engine_sha256'], data['engine_sha256'])

    def test_valid_receipt_safe_mode_still_pins_baseline_and_original_head(self):
        with tempfile.TemporaryDirectory() as home:
            self.receipt(home)
            result = resolve_acceleration('e12', {
                'BEVFUSION_IMAGE': SAFE_IMAGE, 'BEVFUSION_SAFE_MODE': 'true',
                'BEVFUSION_CAMERA_TENSORRT_POLICY': 'require',
                'BEVFUSION_HEAD_ATTENTION_SDPA': 'true',
                'BEVFUSION_HEAD_CACHE_BEV_POS': 'true'}, home)
            self.assertEqual(result['environment']['BEVFUSION_IMAGE'], SAFE_IMAGE)
            self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_POLICY'], 'off')
            self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_ENGINE'], '')
            self.assertEqual(result['environment']['BEVFUSION_HEAD_ATTENTION_SDPA'], 'false')
            self.assertEqual(result['environment']['BEVFUSION_HEAD_CACHE_BEV_POS'], 'false')
            self.assertFalse(result['managed_image'])
            self.assertIsNone(result['engine_sha256'])

    def test_explicit_off_pins_baseline_even_with_valid_receipt_or_custom_image(self):
        for image in (None, SAFE_IMAGE, 'operator/custom:runtime'):
            with self.subTest(image=image), tempfile.TemporaryDirectory() as home:
                self.receipt(home)
                environment = {'BEVFUSION_CAMERA_TENSORRT_POLICY': 'off'}
                if image is not None:
                    environment['BEVFUSION_IMAGE'] = image
                result = resolve_acceleration('e12', environment, home)
                self.assertEqual(result['environment']['BEVFUSION_IMAGE'], SAFE_IMAGE)
                self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_ENGINE'], '')
                self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_METADATA'], '')
                self.assertEqual(result['environment']['BEVFUSION_HEAD_ATTENTION_SDPA'], 'true')
                self.assertFalse(result['managed_image'])
                self.assertIsNone(result['engine_sha256'])

    def test_custom_image_with_valid_receipt_remains_explicit_override(self):
        with tempfile.TemporaryDirectory() as home:
            _, data = self.receipt(home)
            result = resolve_acceleration('e12', {
                'BEVFUSION_IMAGE': 'operator/custom:runtime'}, home)
            self.assertEqual(result['environment']['BEVFUSION_IMAGE'], 'operator/custom:runtime')
            self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_ENGINE'], data['container_engine_path'])
            self.assertEqual(result['engine_sha256'], data['engine_sha256'])
            self.assertFalse(result['managed_image'])

    def test_custom_image_without_receipt_remains_explicit_override(self):
        for model in ('e12', 'e24', 'e25'):
            with self.subTest(model=model), tempfile.TemporaryDirectory() as home:
                result = resolve_acceleration(model, {
                    'BEVFUSION_IMAGE': 'operator/custom:eager'}, home)
                self.assertEqual(result['environment']['BEVFUSION_IMAGE'], 'operator/custom:eager')
                self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_POLICY'], 'off')
                self.assertFalse(result['managed_image'])

    def test_stale_identity_or_artifact_degrades_to_same_model(self):
        for mutation in ('identity', 'artifact', 'path', 'malformed'):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as home:
                path, data = self.receipt(home)
                if mutation == 'identity': data['checkpoint_sha256'] = 'bad'
                if mutation == 'artifact': Path(data['engine_path']).write_bytes(b'changed')
                if mutation == 'path': data['container_engine_path'] = '/outside.engine'
                path.write_text('{' if mutation == 'malformed' else json.dumps(data))
                result = resolve_acceleration('e12', {}, home)
                self.assertFalse(result['receipt_loaded'])
                self.assertEqual(result['environment']['BEVFUSION_IMAGE'], SAFE_IMAGE)

    def test_safe_overrides_stale_explicit_optimization(self):
        result = resolve_acceleration('e12', dict(BEVFUSION_SAFE_MODE='true',
            BEVFUSION_IMAGE='bad', BEVFUSION_HEAD_ATTENTION_SDPA='true'), '/unused')
        self.assertEqual(result['environment']['BEVFUSION_IMAGE'], SAFE_IMAGE)
        self.assertEqual(result['environment']['BEVFUSION_HEAD_ATTENTION_SDPA'], 'false')

    def test_explicit_off_or_custom_engine_does_not_reuse_stale_expected_engine_sha(self):
        with tempfile.TemporaryDirectory() as home:
            self.receipt(home)
            result=resolve_acceleration('e12',{'BEVFUSION_CAMERA_TENSORRT_POLICY':'off'},home)
            self.assertIsNone(result['engine_sha256'])
            self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_ENGINE'],'')
            self.assertEqual(result['environment']['BEVFUSION_HEAD_ATTENTION_SDPA'],'true')
            for variable in ('BEVFUSION_CAMERA_TENSORRT_ENGINE','BEVFUSION_CAMERA_TENSORRT_METADATA'):
                result=resolve_acceleration('e12',{variable:'/custom.engine'},home)
                self.assertIsNone(result['engine_sha256'])
                self.assertIsNone(result['engine_path'])

    def test_explicit_overrides_and_legacy_profiles_remain_supported(self):
        for model in ('e24', 'e25'):
            result = resolve_acceleration(model, {}, '/unused')
            self.assertEqual(result['environment']['BEVFUSION_HEAD_ATTENTION_SDPA'], 'false')
        result = resolve_acceleration('e12', dict(BEVFUSION_IMAGE='explicit',
            BEVFUSION_HEAD_CACHE_BEV_POS='false', BEVFUSION_CAMERA_TENSORRT_ENGINE='/custom.engine'), '/unused')
        self.assertFalse(result['managed_image'])
        self.assertEqual(result['environment']['BEVFUSION_IMAGE'], 'explicit')
        self.assertEqual(result['environment']['BEVFUSION_CAMERA_TENSORRT_POLICY'], 'prefer')


if __name__ == '__main__':
    unittest.main()
