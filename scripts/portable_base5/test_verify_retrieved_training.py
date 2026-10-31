"""Tiny trusted CPU fixtures only; no SSH, Drive, simulator or training."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import pickle
import shutil
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location('verify_retrieved_training',
    Path(__file__).with_name('verify_retrieved_training.py'))
v = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(v)


@unittest.skipUnless(shutil.which('zstd'), 'zstd not installed')
class VerifyRetrievedTrainingTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.archive = self.root / 'qualified.tar.zst'
        self.manifest = self.root / 'archive_manifest.json'
        self.output = self.root / 'received'
        self.payloads = {'data/chosen/scenario.json': b'{}', 'data/chosen/points.bin': b'POINTS'}
        images = {}
        for n in range(1, 5):
            name = 'jpeg_rgb/camera_%s.jpg' % n
            self.payloads[name] = b'QUALIFIED JPEG ' + str(n).encode()
            images[str(n)] = dict(img_path=name)
        row = dict(token='chosen', scene_group_id='group', images=images,
            lidar_points=dict(lidar_path='data/chosen/points.bin'),
            instances=[dict(original_semantic_class='animal', bbox_label_3d=1)])
        for split in ('train', 'val', 'test'):
            self.payloads['infos/512x1056/morai_base5_infos_%s.pkl' % split] = pickle.dumps(dict(
                metainfo=dict(classes=('vehicle', 'pedestrian', 'obstacle'), box_geometry_policy='upright'),
                data_list=[row] if split == 'train' else []))
        self.write_fixture()

    @staticmethod
    def inventory(body):
        return dict(bytes=len(body), sha256=hashlib.sha256(body).hexdigest())

    def write_fixture(self, extra=None, mutate_manifest=None):
        package = dict(scenes=1, profiles=['512x1056'], split_counts=dict(train=1),
            image_policy=dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'),
            training_label_policy='animal_to_pedestrian_v1', box_geometry_policy='upright',
            files={name: self.inventory(body) for name, body in self.payloads.items()}, provenance={})
        if mutate_manifest:
            mutate_manifest(package)
        self.bodies = {'dataset/' + name: body for name, body in self.payloads.items()}
        self.bodies['dataset/manifest.json'] = json.dumps(package).encode()
        for name in v.REQUIRED_PROVENANCE:
            self.bodies[name] = b'{}\n'
        entries = [(name, body, tarfile.REGTYPE) for name, body in self.bodies.items()]
        entries.extend(extra or [])
        stream = io.BytesIO()
        with tarfile.open(fileobj=stream, mode='w') as target:
            for name, body, kind in entries:
                item = tarfile.TarInfo(name)
                item.type = kind
                item.size = len(body)
                if kind in (tarfile.SYMTYPE, tarfile.LNKTYPE):
                    item.linkname = '../../outside'
                target.addfile(item, io.BytesIO(body) if item.isreg() else None)
        self.archive.write_bytes(subprocess.run(['zstd', '-q', '-c'], input=stream.getvalue(),
                                                stdout=subprocess.PIPE, check=True).stdout)
        expected = dict(self.bodies)
        expected.update({name: body for name, body, _ in (extra or [])})
        self.document = dict(archive='/trusted/original/qualified.tar.zst',
            bytes=self.archive.stat().st_size, sha256=v.finish.digest(self.archive),
            members={name: self.inventory(body) for name, body in expected.items()})
        self.save_manifest()

    def save_manifest(self):
        self.manifest.write_text(json.dumps(self.document))

    def args(self, run=True):
        argv = ['--archive', str(self.archive), '--archive-manifest', str(self.manifest),
                '--output', str(self.output), '--expected-scenes', '1', '--min-free-gib', '0',
                '--trusted-generated-archive']
        return v.parser().parse_args(argv + (['--run'] if run else []))

    def test_plan_is_read_only_no_extraction(self):
        with patch.object(v, 'extract_verified', side_effect=AssertionError('no extraction')):
            result = v.restore(self.args(False))
        self.assertFalse(result['execute'])
        self.assertFalse(self.output.exists())

    def test_trusted_source_acknowledgment_required(self):
        args = self.args(); args.trusted_generated_archive = False
        with self.assertRaisesRegex(ValueError, 'trusted generated'):
            v.restore(args)
        self.assertFalse(self.output.exists())

    def test_real_restore_checks_every_member_and_existing_portable_validator(self):
        with patch.object(v.finish, 'validate_training_package',
                          wraps=v.finish.validate_training_package) as validator:
            result = v.restore(self.args())
        validator.assert_called_once_with(self.output / 'dataset', 1)
        self.assertTrue(result['complete'])
        self.assertEqual(result['frames'], 1)
        self.assertEqual(result['unique_camera_images'], 4)
        self.assertEqual(result['split_counts'], dict(train=1))
        self.assertTrue((self.output / 'COMPLETE.json').is_file())
        self.assertFalse((self.output / 'INCOMPLETE.json').exists())
        self.assertEqual((self.output / 'dataset/data/chosen/points.bin').read_bytes(), b'POINTS')
        self.assertEqual(v.finish.digest(self.archive), self.document['sha256'])

    def test_existing_output_is_never_overwritten_or_resumed(self):
        self.output.mkdir(); (self.output / 'mine').write_bytes(b'PRESERVE')
        with self.assertRaisesRegex(ValueError, 'Fresh output'):
            v.restore(self.args())
        self.assertEqual((self.output / 'mine').read_bytes(), b'PRESERVE')

    def test_symlink_input_rejected(self):
        alias = self.root / 'alias.tar.zst'; alias.symlink_to(self.archive)
        args = self.args(); args.archive = alias
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            v.restore(args)

    def test_symlink_output_parent_rejected(self):
        alias = self.root / 'alias'; alias.symlink_to(self.root, target_is_directory=True)
        args = self.args(); args.output = alias / 'received'
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            v.restore(args)

    def test_archive_size_or_sha_mismatch_blocks_before_write(self):
        self.archive.write_bytes(self.archive.read_bytes() + b'ALTERED')
        with self.assertRaisesRegex(ValueError, 'Archive size/SHA256'):
            v.restore(self.args())
        self.assertFalse(self.output.exists())

    def test_unsafe_manifest_names_rejected_before_extraction(self):
        for name in ('/dataset/absolute', 'dataset/../escape', '../escape',
                     'other/root', 'dataset//double', 'dataset/./dot', 'dataset/back\\slash',
                     'dataset/trailing/', 'dataset'):
            with self.subTest(name=name):
                original = dict(self.document['members'])
                self.document['members'][name] = self.inventory(b'')
                self.save_manifest()
                with self.assertRaisesRegex(ValueError, 'Unsafe archive'):
                    v.restore(self.args())
                self.document['members'] = original
                self.assertFalse(self.output.exists())

    def test_prefix_conflict_rejected_before_extraction(self):
        self.document['members']['dataset/data'] = self.inventory(b'')
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'prefix conflict'):
            v.restore(self.args())

    def test_missing_provenance_rejected(self):
        del self.document['members']['provenance/selected_ledger.jsonl']
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'final provenance'):
            v.restore(self.args())

    def test_duplicate_manifest_keys_rejected(self):
        self.manifest.write_text('{"members":{},"members":{}}')
        with self.assertRaisesRegex(ValueError, 'Duplicate JSON'):
            v.restore(self.args())

    def test_insufficient_disk_blocks_before_write(self):
        with patch.object(v.shutil, 'disk_usage', return_value=shutil._ntuple_diskusage(1, 1, 0)):
            with self.assertRaisesRegex(ValueError, 'Insufficient free disk'):
                v.restore(self.args())
        self.assertFalse(self.output.exists())

    def test_tar_symlink_hardlink_device_directory_and_fifo_refused(self):
        for kind in (tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.CHRTYPE,
                     tarfile.BLKTYPE, tarfile.DIRTYPE, tarfile.FIFOTYPE):
            with self.subTest(kind=kind):
                self.write_fixture([('dataset/invalid', b'', kind)])
                args = self.args(); args.output = self.root / ('failure_' + kind.decode())
                with self.assertRaisesRegex(ValueError, 'non-regular'):
                    v.restore(args)
                self.assertTrue((args.output / 'INCOMPLETE.json').is_file())
                self.assertFalse((self.root / 'outside').exists())

    def test_duplicate_tar_entry_refused_partial_preserved(self):
        self.write_fixture([('dataset/data/chosen/points.bin', b'POINTS', tarfile.REGTYPE)])
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            v.restore(self.args())
        self.assertTrue((self.output / 'INCOMPLETE.json').is_file())
        self.assertFalse((self.output / 'COMPLETE.json').exists())

    def test_unexpected_tar_member_refused(self):
        self.write_fixture([('dataset/unlisted', b'NEW', tarfile.REGTYPE)])
        del self.document['members']['dataset/unlisted']; self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'Unexpected'):
            v.restore(self.args())

    def test_missing_tar_member_refused(self):
        self.document['members']['dataset/absent'] = self.inventory(b'NEVER WRITTEN')
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'inventory differs'):
            v.restore(self.args())

    def test_member_declared_size_mismatch_refused(self):
        self.document['members']['dataset/data/chosen/points.bin']['bytes'] += 1
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'declared size'):
            v.restore(self.args())

    def test_member_sha_mismatch_refused_before_pickle_loading(self):
        self.document['members']['dataset/data/chosen/points.bin']['sha256'] = 'a' * 64
        self.save_manifest()
        with patch.object(v.finish, 'validate_training_package', side_effect=AssertionError('too early')):
            with self.assertRaisesRegex(ValueError, 'member bytes/SHA256'):
                v.restore(self.args())

    def test_unregistered_dataset_member_refused(self):
        self.write_fixture([('dataset/unregistered', b'NEW', tarfile.REGTYPE)])
        with self.assertRaisesRegex(ValueError, 'unregistered payloads'):
            v.restore(self.args())

    def test_wrong_scene_count_refused(self):
        args = self.args(); args.expected_scenes = 2
        with self.assertRaisesRegex(ValueError, 'policy differs'):
            v.restore(args)
        self.assertTrue((self.output / 'INCOMPLETE.json').is_file())

    def test_incorrect_manifest_split_counts_refused(self):
        self.write_fixture(mutate_manifest=lambda document: document.update(split_counts=dict(val=1)))
        with self.assertRaisesRegex(ValueError, 'Split counts'):
            v.restore(self.args())

    def test_non_q90_policy_refused(self):
        self.write_fixture(mutate_manifest=lambda document: document.update(
            image_policy=dict.fromkeys(('train', 'val', 'test'), 'original_png')))
        with self.assertRaisesRegex(ValueError, 'image policy'):
            v.restore(self.args())

    def test_wrong_training_classes_refused(self):
        name = 'infos/512x1056/morai_base5_infos_train.pkl'
        value = pickle.loads(self.payloads[name])
        value['metainfo']['classes'] = ('vehicle', 'pedestrian', 'obstacle', 'animal')
        self.payloads[name] = pickle.dumps(value)
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, 'training classes'):
            v.restore(self.args())

    def test_absolute_camera_path_refused(self):
        name = 'infos/512x1056/morai_base5_infos_train.pkl'
        value = pickle.loads(self.payloads[name])
        value['data_list'][0]['images']['1']['img_path'] = '/outside/camera.jpg'
        self.payloads[name] = pickle.dumps(value)
        self.write_fixture()
        with self.assertRaises(ValueError):
            v.restore(self.args())

    def test_bad_component_label_refused(self):
        name = 'infos/512x1056/morai_base5_infos_train.pkl'
        value = pickle.loads(self.payloads[name])
        value['data_list'][0]['instances'][0]['bbox_label_3d'] = 3
        self.payloads[name] = pickle.dumps(value)
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, 'semantic provenance'):
            v.restore(self.args())

    def test_changed_archive_manifest_during_extraction_refused(self):
        original = v.extract_verified
        def mutate(*args):
            original(*args)
            self.manifest.write_text(self.manifest.read_text() + '\n')
        with patch.object(v, 'extract_verified', side_effect=mutate):
            with self.assertRaisesRegex(ValueError, 'changed during restoration'):
                v.restore(self.args())
        self.assertFalse((self.output / 'COMPLETE.json').exists())


if __name__ == '__main__':
    unittest.main()
