"""CPU-only contracts for the separate next-training audit (no model imports)."""
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import pickle
import shlex
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
from PIL import Image
from mmengine.config import Config

import prepare_base5_next_training as next_training
from prepare_base5_pedestrian_merged import merge_infos


class NextTrainingTest(unittest.TestCase):
    """Fixtures are trusted test pickles; never load a downloaded pickle here."""

    def fixture(self, root):
        data = root / 'portable'
        data.mkdir()
        files, source_hashes = {}, {}
        source_classes = ('vehicle', 'pedestrian', 'obstacle', 'animal')

        def write(relative, payload):
            path = data / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
            files[relative] = dict(bytes=len(payload), sha256=hashlib.sha256(payload).hexdigest())

        for split, indices in (('train', (0, 1)), ('val', (2,)), ('test', ())):
            rows = []
            for index in indices:
                token = f's{index}'
                points = np.array([[1., 2., .5, .5], [3., 4., 1., 1.],
                                   [0., 0., 0., .2], [np.nan, 1., 1., .4]], dtype=np.float32)
                point_path = f'data/{token}/points.bin'
                write(point_path, points.tobytes())
                images = {}
                for camera in range(4):
                    path = f'data/{token}/camera{camera}.jpg'
                    buffer = io.BytesIO()
                    Image.fromarray(np.full((32, 64, 3), index * 10 + camera, np.uint8)).save(
                        buffer, format='JPEG', quality=90)
                    write(path, buffer.getvalue())
                    images[str(camera)] = dict(img_path=path, height=32, width=64)
                instances = [dict(bbox_label_3d=label, bbox_3d=[10., 2., -.5, .5, .6, 1.5, .2],
                                  asset=f'asset{label}', component_id=f'part{label}')
                             for label in range(4)]
                write(f'data/{token}/native_annotation.json', json.dumps(instances).encode())
                rows.append(dict(token=token, scene_group_id=f'g{index}', images=images,
                                 lidar_points=dict(lidar_path=point_path, num_pts_feats=4),
                                 instances=instances))
            source = dict(metainfo=dict(classes=source_classes, box_dim=7, single_frame=True,
                velocity_target=False, range_frame='lidar',
                box_geometry_policy='native_rpy_fixed_yaw_enclosure_v1',
                point_cloud_range=next_training.ROI), data_list=rows)
            source_blob = pickle.dumps(source, protocol=4)
            source_hashes[split] = hashlib.sha256(source_blob).hexdigest()
            write(f'provenance/source_infos/{split}.pkl', source_blob)
            merged, _ = merge_infos(source)
            write(f'infos/512x1056/morai_base5_infos_{split}.pkl', pickle.dumps(merged, protocol=4))
        manifest = dict(version=1, scenes=3, split_counts=dict(train=2, val=1),
                        profiles=['512x1056'], source_info_sha256={'512x1056': source_hashes},
                        source_classes=source_classes, classes=source_classes[:3], files=files,
                        bytes=sum(item['bytes'] for item in files.values()),
                        image_policy=dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'),
                        box_geometry_policy='native_rpy_fixed_yaw_enclosure_v1',
                        training_label_policy='animal_to_pedestrian_v1')
        (data / 'manifest.json').write_text(json.dumps(manifest))
        initializer = root / 'compatible_full_initializer.pth'
        initializer.write_bytes(b'CPU fixture: identity only, not a shape-qualified checkpoint')
        return next_training.parser().parse_args([
            '--data', str(data), '--work-dir', str(root / 'next_run'),
            '--init-checkpoint', str(initializer), '--epochs', '2', '--workers', '0'])

    def rewrite_info(self, data, split, change):
        relative = f'infos/512x1056/morai_base5_infos_{split}.pkl'
        path = data / relative
        info = pickle.loads(path.read_bytes())
        change(info)
        payload = pickle.dumps(info, protocol=4)
        path.write_bytes(payload)
        manifest = json.loads((data / 'manifest.json').read_text())
        manifest['files'][relative] = dict(bytes=len(payload), sha256=hashlib.sha256(payload).hexdigest())
        manifest['bytes'] = sum(item['bytes'] for item in manifest['files'].values())
        (data / 'manifest.json').write_text(json.dumps(manifest))

    @staticmethod
    def snapshot(root):
        return {str(path.relative_to(root)): path.read_bytes()
                for path in root.rglob('*') if path.is_file()}

    def test_config_only_is_read_only_and_disables_custom_imports(self):
        original = Config.fromfile
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            cfg, _ = next_training.check_config()
            cfg.custom_imports = dict(imports=['must_not_import_next_training_fixture'],
                                      allow_failed_imports=False)
            path = root / 'config.py'
            cfg.dump(str(path))
            before = self.snapshot(root)
            output = io.StringIO()
            with mock.patch.object(next_training.Config, 'fromfile', wraps=original) as load:
                with redirect_stdout(output):
                    next_training.main(['--config', str(path), '--check-config'])
            self.assertEqual(len(load.call_args_list), 1)
            self.assertIs(load.call_args.kwargs['import_custom_modules'], False)
            self.assertNotIn('must_not_import_next_training_fixture', sys.modules)
            self.assertEqual(before, self.snapshot(root))
            result = json.loads(output.getvalue())
            self.assertEqual(result['event'], 'next_training_config_checked')
            self.assertFalse(result['training_started'])

    def test_new_config_is_separate_and_keeps_completed_training_config(self):
        old_path = next_training.DEFAULT_CONFIG.with_name(
            'bevfusion_base5_fine_512_depth25_pedestrian_merged.py')
        before = old_path.read_bytes()
        old = Config.fromfile(str(old_path), import_custom_modules=False)
        cfg, settings = next_training.check_config()
        self.assertTrue(old.model.compile_camera)
        self.assertFalse(cfg.model.compile_camera)
        self.assertEqual(old_path.read_bytes(), before)
        self.assertEqual(settings['policy'], 'base5_eager_nearest_v1')
        self.assertEqual(settings['depth_hint'], 'nearest_positive')
        self.assertEqual(settings['training_intensity'], 'native_0_1_no_divide_255')
        self.assertEqual(settings['fusion_shape_yx'], [168, 504])
        with self.assertRaisesRegex(ValueError, 'new next-training config'):
            next_training.check_config(old_path)

    def test_no_run_or_resume_switches_are_accepted(self):
        for option in ('--run', '--resume'):
            with self.subTest(option=option), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    next_training.parser().parse_args([option])
                self.assertEqual(error.exception.code, 2)

    def test_cpu_plan_preserves_inputs_and_does_not_prepare_or_start_training(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            args.wandb = True
            before = self.snapshot(args.data)
            initializer = args.init_checkpoint.read_bytes()
            result = next_training.prepare(args)
            self.assertFalse(result['training_started'])
            self.assertEqual(result['counts'], dict(train=dict(scenes=2, boxes=8),
                             val=dict(scenes=1, boxes=4), test=dict(scenes=0, boxes=0)))
            self.assertEqual(result['point_cloud_samples'], 3)
            self.assertEqual(self.snapshot(args.data), before)
            self.assertEqual(args.init_checkpoint.read_bytes(), initializer)
            self.assertEqual({p.name for p in args.work_dir.iterdir()},
                             {'checked_model_config.py', 'next_training_preflight.json'})
            report = json.loads((args.work_dir / 'next_training_preflight.json').read_text())
            self.assertFalse(report['training_started'])
            self.assertFalse(report['resume_allowed'])
            self.assertFalse(report['initializer_shape_qualified'])
            self.assertFalse(report['data']['all_payload_hashes_verified'])
            self.assertEqual(report['initializer_sha256'], hashlib.sha256(initializer).hexdigest())
            command = shlex.split(report['direct_prepare_command'])
            self.assertEqual(Path(command[1]).name, 'base5_distance_diagnostics.py')
            self.assertEqual(Path(command[command.index('--frozen-entrypoint') + 1]).name,
                             'run_base5_direct_training.py')
            self.assertEqual(report['distance_diagnostics_sha256'],
                             next_training.sha256(next_training.ROOT /
                                 'scripts/runtime/base5_distance_diagnostics.py'))
            self.assertIn('--wandb', command)
            self.assertNotIn('--run', command)
            self.assertNotIn('--resume', command)
            self.assertEqual(command[command.index('--accumulate') + 1], '1')
            self.assertEqual(command[command.index('--init-checkpoint') + 1], str(args.init_checkpoint))
            self.assertIn(str(next_training.DEFAULT_CONFIG.relative_to(next_training.ROOT)),
                          report['config_files_sha256'])
            recorded_hash = report.pop('plan_sha256')
            self.assertEqual(recorded_hash,
                             hashlib.sha256(json.dumps(report, sort_keys=True).encode()).hexdigest())
            saved = Config.fromfile(str(args.work_dir / 'checked_model_config.py'),
                                    import_custom_modules=False)
            self.assertFalse(saved.model.compile_camera)
            self.assertEqual(saved.required_next_training_policy, next_training.POLICY)

    def test_config_rejects_compile_and_training_intensity_renormalization(self):
        cfg, _ = next_training.check_config()
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / 'invalid.py'
            runner_compile = deepcopy(cfg)
            runner_compile['compile'] = True
            runner_compile.dump(str(path))
            with self.assertRaisesRegex(ValueError, 'Runner-wide compile'):
                next_training.check_config(path)
            camera_compile = deepcopy(cfg)
            camera_compile.model.compile_camera = True
            camera_compile.dump(str(path))
            with self.assertRaisesRegex(ValueError, 'explicit compile OFF'):
                next_training.check_config(path)
            for component in ('pts_middle_encoder', 'bbox_head'):
                changed = deepcopy(cfg)
                changed.model[component].type = 'LegacySameShapeImplementation'
                changed.dump(str(path))
                with self.assertRaisesRegex(ValueError, 'metric-axis-corrected'):
                    next_training.check_config(path)
            for split in ('train', 'val', 'test'):
                changed = deepcopy(cfg)
                changed[split + '_dataloader'].dataset.type = 'MoraiBase3Dataset'
                changed.dump(str(path))
                with self.assertRaisesRegex(ValueError, 'dataset adapter'):
                    next_training.check_config(path)
                for field in ('norm_intensity', 'shift_height', 'use_color'):
                    with self.subTest(split=split, field=field):
                        changed = deepcopy(cfg)
                        loader = next(t for t in changed[split + '_dataloader'].dataset.pipeline
                                      if t['type'] == 'LoadPointsFromFile')
                        loader[field] = True
                        changed.dump(str(path))
                        with self.assertRaisesRegex(ValueError, 'must not be normalized or extended'):
                            next_training.check_config(path)

    def test_work_guard_preserves_existing_run_and_rejects_data_overlap(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            args.work_dir.mkdir()
            (args.work_dir / 'run_manifest.json').write_text('{"frozen":"E24"}')
            (args.work_dir / 'last_checkpoint').write_text('epoch_24.pth')
            old = self.snapshot(args.work_dir)
            with self.assertRaisesRegex(FileExistsError, 'NEW empty work-dir'):
                next_training.prepare(args)
            self.assertEqual(old, self.snapshot(args.work_dir))
            for path in (args.data, args.data / 'new_run', args.data.parent):
                with self.subTest(path=path):
                    args.work_dir = path
                    with self.assertRaisesRegex(ValueError, 'separate from data'):
                        next_training.prepare(args)

    def test_work_file_is_rejected_and_successful_plan_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            args = self.fixture(root)
            args.work_dir.write_bytes(b'not a directory')
            with self.assertRaises(FileExistsError):
                next_training.prepare(args)
            self.assertEqual(args.work_dir.read_bytes(), b'not a directory')
            args.work_dir = root / 'fresh'
            next_training.prepare(args)
            before = self.snapshot(args.work_dir)
            with self.assertRaises(FileExistsError):
                next_training.prepare(args)
            self.assertEqual(before, self.snapshot(args.work_dir))

    def test_invalid_budget_and_missing_explicit_inputs_do_not_create_work(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            changes = [dict(epochs=0), dict(batch=0), dict(workers=-1), dict(lr=0),
                       dict(lr=float('nan')), dict(lr=float('inf')), dict(sample_clouds=0),
                       dict(data=None), dict(init_checkpoint=None)]
            for change in changes:
                with self.subTest(change=change):
                    invalid = deepcopy(args)
                    for key, value in change.items():
                        setattr(invalid, key, value)
                    with self.assertRaises(ValueError):
                        next_training.prepare(invalid)
                    self.assertFalse(args.work_dir.exists())

    def test_invalid_metadata_class_provenance_and_boxes_are_rejected(self):
        cases = [
            ('meta', lambda info: info['metainfo'].update(box_dim=9)),
            ('class order', lambda info: info['metainfo'].update(classes=('pedestrian', 'vehicle', 'obstacle'))),
            ('velocity', lambda info: info['metainfo'].update(velocity_target=True)),
            ('box policy', lambda info: info['metainfo'].update(box_geometry_policy='legacy')),
            ('unmerged animal', lambda info: info['data_list'][0]['instances'][3].update(bbox_label_3d=3)),
            ('original animal', lambda info: info['data_list'][0]['instances'][3].pop('original_semantic_class')),
            ('wrong source', lambda info: info['data_list'][0]['instances'][0].update(original_semantic_class='obstacle')),
            ('9D box', lambda info: info['data_list'][0]['instances'][0].update(bbox_3d=[0.] * 9)),
            ('negative size', lambda info: info['data_list'][0]['instances'][0]['bbox_3d'].__setitem__(3, -1.)),
            ('nonfinite box', lambda info: info['data_list'][0]['instances'][0]['bbox_3d'].__setitem__(0, float('nan'))),
            ('sweeps', lambda info: info['data_list'][0].update(lidar_sweeps=[{}])),
            ('missing camera', lambda info: info['data_list'][0]['images'].pop('3')),
        ]
        for label, change in cases:
            with self.subTest(case=label), tempfile.TemporaryDirectory() as name:
                args = self.fixture(Path(name))
                self.rewrite_info(args.data, 'train', change)
                with self.assertRaises(ValueError):
                    next_training.prepare(args)
                self.assertFalse(args.work_dir.exists())

    def test_split_group_leakage_is_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            self.rewrite_info(args.data, 'val',
                              lambda info: info['data_list'][0].update(scene_group_id='g0'))
            with self.assertRaisesRegex(ValueError, 'scene-group leakage'):
                next_training.prepare(args)
            self.assertFalse(args.work_dir.exists())

    def test_native_intensity_is_not_divided_and_ros255_data_is_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            path = args.data / 'data/s0/points.bin'
            before = path.read_bytes()
            cfg, _ = next_training.check_config()
            report = next_training.check_data(args.data, cfg)
            sample = next(item for item in report['point_cloud_samples'] if item['token'] == 's0')
            self.assertEqual(sample['intensity_min'], .5)
            self.assertEqual(sample['intensity_max'], 1.)
            self.assertEqual(sample['filtered_returns'], 2)
            self.assertEqual(path.read_bytes(), before)
            points = np.frombuffer(before, dtype=np.float32).copy().reshape(-1, 4)
            points[:, 3] *= 255.
            path.write_bytes(points.tobytes())
            with self.assertRaisesRegex(ValueError, 'native intensity 0..1'):
                next_training.prepare(args)
            self.assertFalse(args.work_dir.exists())

    def test_hash_verification_is_explicit_and_detects_same_size_tampering(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            path = args.data / 'data/s0/native_annotation.json'
            payload = path.read_bytes()
            path.write_bytes(bytes([payload[0] ^ 1]) + payload[1:])
            cfg, _ = next_training.check_config()
            report = next_training.check_data(args.data, cfg, verify_hashes=False)
            self.assertFalse(report['all_payload_hashes_verified'])
            args.verify_payload_hashes = True
            with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
                next_training.prepare(args)
            self.assertFalse(args.work_dir.exists())

    def test_full_hash_audit_and_sample_limit_preserve_original_animal_provenance(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            args.verify_payload_hashes = True
            args.sample_clouds = 1
            next_training.prepare(args)
            report = json.loads((args.work_dir / 'next_training_preflight.json').read_text())
            self.assertTrue(report['data']['all_payload_hashes_verified'])
            self.assertEqual(len(report['data']['point_cloud_samples']), 2)
            info = pickle.loads((args.data /
                'infos/512x1056/morai_base5_infos_train.pkl').read_bytes())
            original = pickle.loads((args.data / 'provenance/source_infos/train.pkl').read_bytes())
            animal = info['data_list'][0]['instances'][3]
            self.assertEqual(animal['bbox_label_3d'], 1)
            self.assertEqual(animal['original_bbox_label_3d'], 3)
            self.assertEqual(animal['original_semantic_class'], 'animal')
            self.assertEqual(tuple(info['metainfo']['source_classes']),
                             tuple(original['metainfo']['classes']))
            self.assertEqual(original['data_list'][0]['instances'][3]['bbox_label_3d'], 3)
            self.assertEqual(animal['bbox_3d'], original['data_list'][0]['instances'][3]['bbox_3d'])

    def test_empty_train_or_val_is_rejected(self):
        for split in ('train', 'val'):
            with self.subTest(split=split), tempfile.TemporaryDirectory() as name:
                args = self.fixture(Path(name))
                self.rewrite_info(args.data, split, lambda info: info.update(data_list=[]))
                with self.assertRaisesRegex(ValueError, 'must be nonempty'):
                    next_training.prepare(args)
                self.assertFalse(args.work_dir.exists())

    def test_mixed_png_policy_is_not_the_current_native_jpeg_contract(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            path = args.data / 'manifest.json'
            manifest = json.loads(path.read_text())
            manifest['image_policy']['val'] = 'original_png'
            manifest['image_policy']['test'] = 'original_png'
            path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                next_training.prepare(args)
            self.assertFalse(args.work_dir.exists())


if __name__ == '__main__':
    unittest.main()
