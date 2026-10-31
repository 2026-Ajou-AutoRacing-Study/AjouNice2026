"""Tiny CPU fixtures only; no simulator, native QA, remote or training startup."""
import importlib.util
import json
from pathlib import Path
import pickle
import shutil
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('pack_training',
    Path(__file__).with_name('pack_training.py'))
p = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(p)


class PackTrainingTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.package = self.root / 'training3_jpeg_portable'
        self.package.mkdir()
        self.output = self.root / 'archive/base5.tar.zst'
        self.receipt = self.root / 'archive_receipt'
        self.selected = self.root / 'selected_ledger.jsonl'
        self.overlay = self.root / 'split_overlay.json'
        self.report = self.root / 'report.json'
        scenario = self.put(self.package / 'data/chosen/scenario.json', dict(ego='source'))
        row = dict(id='chosen', source_cell_id='original_slot', status='accepted_candidate',
                   capture_root='/trusted/native/chosen', scenario_sha256=p.finish.digest(scenario))
        self.selected.write_text(json.dumps(row) + '\n')
        self.overlay_doc = dict(schema='base5_layout_split_overlay_v1', overlay_sha256='logical_overlay',
            assignments=dict(original_slot=dict(split='train', group_id='group',
                                                scenario_sha256=row['scenario_sha256'])))
        self.put(self.overlay, self.overlay_doc)
        infos = dict(metainfo=dict(classes=('vehicle', 'pedestrian', 'obstacle'),
            box_geometry_policy='upright', layout_split_overlay_sha256='logical_overlay'), data_list=[
                dict(token='chosen', scene_group_id='group', split_overlay_sha256='logical_overlay',
                     images={}, lidar_points=dict(lidar_path='data/chosen/LIDAR_5/0_Intensity.bin'),
                     instances=[dict(bbox_label_3d=1, original_semantic_class='animal')])])
        self.put(self.package / 'data/chosen/LIDAR_5/0_Intensity.bin', b'POINTS')
        self.put(self.package / 'data/chosen/CAMERA_1/0_Instance.png', b'NATIVE MASK')
        for n in range(1, 5):
            relative = 'jpeg_rgb/camera_%s.jpg' % n
            self.put(self.package / relative, b'QUALIFIED JPEG ' + str(n).encode())
            infos['data_list'][0]['images'][str(n)] = dict(img_path=relative, width=8, height=6)
        for split in ('train', 'val', 'test'):
            source = infos if split == 'train' else dict(infos, data_list=[])
            self.put(self.package / ('infos/512x1056/morai_base5_infos_%s.pkl' % split),
                     pickle.dumps(source))
        self.manifest = dict(scenes=1, profiles=['512x1056'], split_counts=dict(train=1),
            box_geometry_policy='upright', image_policy=dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'),
            training_label_policy='animal_to_pedestrian_v1',
            provenance=dict(chosen=dict(original_capture_root=row['capture_root'])),
            files={str(path.relative_to(self.package)): dict(bytes=path.stat().st_size,
                   sha256=p.finish.digest(path)) for path in self.package.rglob('*') if path.is_file()})
        self.put(self.package / 'manifest.json', self.manifest)
        self.document = dict(schema='base5_training_input_qualification_v1',
            split_overlay=dict(sha256=p.finish.digest(self.overlay)), cohorts=dict(sunny15=dict(
                frames=1, source_unchanged=True, lost_retained_component_ids=0,
                extra_retained_component_ids=0, training_label_policy='animal_to_pedestrian_v1',
                training3_package=str(self.package), training3_manifest_sha256=p.finish.digest(self.package / 'manifest.json'),
                source_ledger_sha256=p.finish.digest(self.selected), split_frames=dict(train=1),
                rgb=dict(images_checked=4, native_jpeg_quality=90, lossless_auxiliary_files_preserved=True))))
        self.put(self.report, self.document)

    def put(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else (json.dumps(value) + '\n').encode())
        return path

    def args(self, run=False):
        argv = ['--package', str(self.package), '--qualification-report', str(self.report),
                '--selected', str(self.selected), '--split-overlay', str(self.overlay),
                '--output', str(self.output), '--receipt-dir', str(self.receipt),
                '--expected-scenes', '1', '--max-usage-gib', '1', '--min-free-gib', '0']
        return p.parser().parse_args(argv + (['--run'] if run else []))

    def test_plan_does_not_write_or_call_capture(self):
        with patch.object(p.finish, 'verify_authoritative', side_effect=AssertionError('must not run')):
            result = p.pack(self.args())
        self.assertFalse(result['execute'])
        self.assertFalse(self.output.parent.exists())
        self.assertFalse(self.receipt.exists())
        self.assertFalse(result['native_capture_requalification'])

    @unittest.skipUnless(shutil.which('zstd'), 'zstd not installed')
    def test_verified_single_archive_and_provenance(self):
        with patch.object(p.finish, 'verify_authoritative', side_effect=AssertionError('must not run')):
            result = p.pack(self.args(True))
        self.assertTrue(result['complete'])
        self.assertTrue(result['verified_decompressed_members'])
        doc = json.loads((self.receipt / 'archive_manifest.json').read_text())
        self.assertEqual(doc['sha256'], p.finish.digest(self.output))
        self.assertIn('provenance/selected_ledger.jsonl', doc['members'])
        self.assertIn('dataset/data/chosen/CAMERA_1/0_Instance.png', doc['members'])
        self.assertFalse(any(name.endswith('_Intensity.png') for name in doc['members']))
        self.assertEqual((self.receipt / 'SHA256SUMS').read_text(),
                         doc['sha256'] + '  base5.tar.zst\n')
        self.assertFalse((self.receipt / 'verified_payload.tar.zst').exists())
        self.assertEqual(list(self.output.parent.glob('*.part*')), [])

    def test_wrong_manifest_sha_blocks(self):
        self.document['cohorts']['sunny15']['training3_manifest_sha256'] = 'bad'
        self.put(self.report, self.document)
        with self.assertRaisesRegex(ValueError, 'manifest changed'):
            p.pack(self.args(True))
        self.assertFalse(self.receipt.exists())

    def test_wrong_selected_sha_blocks(self):
        self.selected.write_text(self.selected.read_text() + '\n')
        with self.assertRaisesRegex(ValueError, 'authoritative final selection'):
            p.pack(self.args(True))

    def test_wrong_overlay_sha_blocks(self):
        self.put(self.overlay, dict(self.overlay_doc, extra='changed'))
        with self.assertRaisesRegex(ValueError, 'split overlay'):
            p.pack(self.args(True))

    def test_unregistered_scratch_file_blocks(self):
        self.put(self.package / 'unexpected.tmp', b'UNREGISTERED')
        with self.assertRaisesRegex(ValueError, 'unregistered'):
            p.pack(self.args(True))

    def test_insufficient_disk_budget_blocks_before_writes(self):
        args = self.args(True)
        args.max_usage_gib = .000001
        with self.assertRaisesRegex(ValueError, 'budget'):
            p.pack(args)
        self.assertFalse(self.receipt.exists())

    def test_existing_output_is_never_overwritten(self):
        self.put(self.output, b'PRESERVE')
        with self.assertRaisesRegex(ValueError, 'Fresh archive'):
            p.pack(self.args(True))
        self.assertEqual(self.output.read_bytes(), b'PRESERVE')

    def test_symlinked_source_refused(self):
        link = self.root / 'alias'
        link.symlink_to(self.package, target_is_directory=True)
        args = self.args(True); args.package = link
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            p.pack(args)

    def test_group_assignment_change_refused(self):
        self.overlay_doc['assignments']['original_slot']['group_id'] = 'other'
        self.put(self.overlay, self.overlay_doc)
        self.document['split_overlay']['sha256'] = p.finish.digest(self.overlay)
        self.put(self.report, self.document)
        with self.assertRaisesRegex(ValueError, 'assignment differs'):
            p.pack(self.args(True))

    def test_qualified_count_and_20k_rgb_gate_required(self):
        self.document['cohorts']['sunny15']['rgb']['images_checked'] = 3
        self.put(self.report, self.document)
        with self.assertRaisesRegex(ValueError, 'Four-camera'):
            p.pack(self.args(True))

    def test_compression_failure_preserves_partial_and_sources(self):
        with patch.object(p.finish, 'archive_single', side_effect=RuntimeError('compress failed')):
            with self.assertRaisesRegex(RuntimeError, 'compress failed'):
                p.pack(self.args(True))
        self.assertTrue((self.receipt / 'INCOMPLETE.json').exists())
        self.assertTrue((self.package / 'jpeg_rgb/camera_1.jpg').exists())


if __name__ == '__main__':
    unittest.main()
