"""Small local fixtures; no native capture, ROS, GPU, SSH or Drive execution."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import pickle
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('training_convert_adapter',
    Path(__file__).with_name('training_convert_adapter.py'))
a = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(a)
converter, qualification = a.load_modules()


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def fixture(self, primary=True, secondary=True, review=False, actors=True):
        native = self.root / 'native'
        native.mkdir()
        metadata = dict(runtime_actors=[dict(unique_id=8500), dict(unique_id=8501)] if actors else [],
                        split='train', scene_group_id='group')
        (native / 'metadata.json').write_text(json.dumps(metadata))
        components = []
        if actors:
            for uid, positive in ((8500, primary), (8501, secondary)):
                components.append(dict(parent_uid=uid, component_id=str(uid), asset='asset',
                    positive_lidar1_camera0=positive, training_class='obstacle', lidar_points=int(positive),
                    center_lidar_m=[10., 0., -1.], size_m=[1., 1., 1.], rotation_rpy=[0., 0., 0.]))
        audit = dict(components=components, review_reasons=['native_review'] if review else [])
        (native / 'audit.json').write_text(json.dumps(audit))
        (native / 'LIDAR_5').mkdir()
        (native / 'LIDAR_5/frame_Intensity.bin').write_bytes(b'fixture')
        for index in range(1, 5):
            camera = native / ('CAMERA_' + str(index))
            camera.mkdir()
            (camera / 'frame_Intensity.png').write_bytes(b'fixture')
        ledger = self.root / 'selected.jsonl'
        ledger.write_text(json.dumps(dict(id='frame', capture_root=str(native),
                                         status='accepted_candidate')) + '\n')
        identity = [[1., 0., 0., 0.], [0., 1., 0., 0.],
                    [0., 0., 1., 0.], [0., 0., 0., 1.]]
        camera = dict(ego_to_sensor=identity, intrinsic=[r[:3] for r in identity[:3]],
                      resolution_px=[1280, 720])
        calibration = dict(source_sha256='frozen', lidar=dict(lidar_to_ego=identity),
                           cameras={f'CAM_{i}': dict(camera, sensor_unique_id=i) for i in range(1, 5)})
        return ledger, calibration

    def run_pair(self, **kwargs):
        ledger, calibration = self.fixture(**kwargs)
        patched = a.adapted_convert(converter.convert)
        before = {str(p): p.read_bytes() for p in (self.root / 'native').rglob('*') if p.is_file()}
        with patch.object(converter, 'normalized_sensor_calibration', return_value=calibration):
            original = converter.convert(ledger, self.root / 'original', self.root / 'sensor.json')
            updated = patched(ledger, self.root / 'adapted', self.root / 'sensor.json')
        after = {str(p): p.read_bytes() for p in (self.root / 'native').rglob('*') if p.is_file()}
        self.assertEqual(before, after)
        return original, updated

    def test_supported_primary_infos_are_exactly_equal(self):
        original, updated = self.run_pair()
        for split in ('train', 'val', 'test'):
            name = f'morai_base5_infos_{split}.pkl'
            self.assertEqual(pickle.loads((self.root / 'original' / name).read_bytes()),
                             pickle.loads((self.root / 'adapted' / name).read_bytes()))
        self.assertEqual(updated['primary_crop_excluded_but_other_gt_retained'], [])
        for key, value in original.items():
            self.assertEqual(updated[key], value)

    def test_crop_excluded_primary_retains_secondary_only(self):
        original, updated = self.run_pair(primary=False)
        self.assertEqual(original['splits']['train'], 0)
        self.assertEqual(updated['splits']['train'], 1)
        self.assertEqual(updated['primary_crop_excluded_but_other_gt_retained'], ['frame'])
        result = pickle.loads((self.root / 'adapted/morai_base5_infos_train.pkl').read_bytes())
        self.assertEqual([i['parent_uid'] for i in result['data_list'][0]['instances']], [8501])
        self.assertFalse(updated['unsupported_primary_forced_positive'])

    def test_zero_supported_actor_gt_still_excluded(self):
        original, updated = self.run_pair(primary=False, secondary=False)
        self.assertEqual(original['splits'], updated['splits'])
        self.assertEqual(updated['splits']['train'], 0)
        self.assertEqual(updated['primary_crop_excluded_but_other_gt_retained'], [])

    def test_native_review_remains_excluded(self):
        original, updated = self.run_pair(primary=False, review=True)
        self.assertEqual(original['splits'], updated['splits'])
        self.assertEqual(updated['excluded_review_scenes'], ['frame'])

    def test_true_background_unchanged(self):
        original, updated = self.run_pair(actors=False)
        self.assertEqual(original['splits'], updated['splits'])
        self.assertEqual(updated['splits']['train'], 1)

    def test_scope_restores_after_success_and_exception(self):
        namespace = qualification.derive_infos.__globals__
        original = namespace['convert']
        with a.scoped_conversion(converter, qualification):
            self.assertIsNot(namespace['convert'], original)
        self.assertIs(namespace['convert'], original)
        with self.assertRaisesRegex(RuntimeError, 'fixture'):
            with a.scoped_conversion(converter, qualification):
                raise RuntimeError('fixture')
        self.assertIs(namespace['convert'], original)

    def test_scope_rejects_nested_or_foreign_patch(self):
        with a.scoped_conversion(converter, qualification):
            with self.assertRaisesRegex(ValueError, 'already replaced'):
                with a.scoped_conversion(converter, qualification):
                    self.fail('Nested scope was accepted')

    def test_changed_source_pin_fails_closed(self):
        changed = deepcopy(a.PINS)
        changed['build_base5_bevfusion_infos.py'] = '0' * 64
        with patch.object(a, 'PINS', changed):
            with self.assertRaisesRegex(ValueError, 'source changed'):
                a.verify_sources()


if __name__ == '__main__':
    unittest.main()
