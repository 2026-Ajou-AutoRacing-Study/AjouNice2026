import hashlib
import json
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile

import base5_production_bundle as bundle


class BundleTests(unittest.TestCase):
    def fixture(self, root, *, extra=None, model='e24'):
        weights = b'known inference weights'
        source = b'fixed runtime'
        doc = dict(schema=bundle.SCHEMA, model=model, checkpoint_sha256=hashlib.sha256(weights).hexdigest(),
                   checkpoint_bytes=len(weights), source_files_sha256={'runtime.py': hashlib.sha256(source).hexdigest()})
        member=bundle.weights_member(model)
        manifest = dict(doc, checkpoint_member=member)
        archive = root / 'bundle.zip'
        with zipfile.ZipFile(archive, 'w') as z:
            z.writestr(member, weights)
            z.writestr('reference/runtime.py', source)
            z.writestr('README.txt', 'handoff')
            z.writestr('bundle_manifest.json', json.dumps(manifest))
            if extra is not None:
                z.writestr(extra, 'untrusted')
        doc.update(archive_bytes=archive.stat().st_size, archive_sha256=bundle.digest(archive))
        descriptor = root / 'release.json'
        descriptor.write_text(json.dumps(doc))
        return archive, descriptor, doc

    def test_archive_identity_and_reference_sources(self):
        with tempfile.TemporaryDirectory() as t:
            archive, _, doc = self.fixture(Path(t))
            bundle.validate_archive(archive, doc)
            for changed in (dict(doc, archive_sha256='wrong'), dict(doc, checkpoint_bytes=1000)):
                with self.assertRaises(ValueError): bundle.validate_archive(archive, changed)

    def test_traversal_duplicate_symlink_and_unexpected_members_refused(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            link = zipfile.ZipInfo('link'); link.external_attr = (stat.S_IFLNK | 0o777) << 16
            for extra in ('../escape', '/absolute', 'bad\\path', bundle.WEIGHTS, 'unknown.txt', link):
                with self.subTest(extra=extra):
                    archive, _, doc = self.fixture(root, extra=extra)
                    with self.assertRaises(ValueError): bundle.validate_archive(archive, doc)
            self.assertFalse((root.parent / 'escape').exists())

    def test_install_extracts_only_weights_atomically_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            archive, descriptor, doc = self.fixture(root)
            destination = root / 'checkpoints/E24.pth'
            p = dict(checkpoint_path=str(destination), sha256=doc['checkpoint_sha256'], checkpoint=destination.name)
            args = SimpleNamespace(archive=archive, remote='unused', check_only=False)
            with patch.object(bundle, 'DESCRIPTOR', descriptor), patch.object(bundle, 'validate_descriptor'), \
                    patch.object(bundle, 'profile', return_value=p), \
                    patch('prepare_bevfusion_production.profile', return_value=p), \
                    patch.object(bundle.subprocess, 'run') as network:
                bundle.install(args)
                before = destination.stat().st_mtime_ns
                bundle.install(args)
                self.assertEqual(destination.stat().st_mtime_ns, before)
                self.assertEqual(bundle.digest(destination), doc['checkpoint_sha256'])
                network.assert_not_called()
                self.assertFalse((root / 'reference').exists())

    def test_existing_different_weights_are_preserved(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t); archive, descriptor, doc = self.fixture(root)
            destination = root / 'existing.pth'; destination.write_bytes(b'user weights')
            p = dict(checkpoint_path=str(destination), sha256=doc['checkpoint_sha256'])
            with patch.object(bundle, 'DESCRIPTOR', descriptor), patch.object(bundle, 'validate_descriptor'), \
                    patch.object(bundle, 'profile', return_value=p):
                with self.assertRaises(ValueError):
                    bundle.install(SimpleNamespace(archive=archive, check_only=False))
            self.assertEqual(destination.read_bytes(), b'user weights')

    def test_check_only_never_downloads_missing_weights(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t); _, descriptor, _ = self.fixture(root)
            with patch.object(bundle, 'DESCRIPTOR', descriptor), patch.object(bundle, 'validate_descriptor'), \
                    patch.object(bundle, 'profile', return_value=dict(checkpoint_path=str(root / 'absent'))), \
                    patch.object(bundle.subprocess, 'run') as network:
                with self.assertRaises(FileNotFoundError):
                    bundle.install(SimpleNamespace(archive=None, check_only=True))
                network.assert_not_called()

    def test_check_only_accepts_identical_weights_after_checkout_update(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            _, descriptor, doc = self.fixture(root)
            destination = root / 'E24.pth'
            destination.write_bytes(b'known inference weights')
            names = bundle.required_sources()
            doc['source_files_sha256'] = {
                name: bundle.digest(bundle.ROOT / name) for name in names
            }
            changed = 'base_ws/src/sensor_interface/launch/lidar.launch'
            doc['source_files_sha256'][changed] = '0' * 64
            descriptor.write_text(json.dumps(doc))
            p = dict(bundle.profile('e24'), checkpoint_path=str(destination),
                     sha256=doc['checkpoint_sha256'])
            with patch.object(bundle, 'DESCRIPTOR', descriptor), \
                    patch.object(bundle, 'profile', return_value=p), \
                    patch.object(bundle.subprocess, 'run') as network, \
                    patch.object(bundle, 'prepare') as prepare:
                bundle.install(SimpleNamespace(archive=None, check_only=True))
                network.assert_not_called()
                prepare.assert_not_called()
            self.assertEqual(destination.read_bytes(), b'known inference weights')

    def test_actual_model_config_closure_and_wrong_checkout_rejected(self):
        names = bundle.required_sources()
        self.assertFalse(any('/tools/' in name or '/scripts/benchmark' in name for name in names))
        self.assertIn('perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_pedestrian_merged_runtime.py', names)
        doc = dict(schema=bundle.SCHEMA, model='e24', checkpoint_sha256=bundle.profile('e24')['sha256'],
                   source_files_sha256={name: bundle.digest(bundle.ROOT / name) for name in names})
        bundle.validate_descriptor(doc)
        doc['source_files_sha256'][names[0]] = '0' * 64
        with self.assertRaises(ValueError): bundle.validate_descriptor(doc)
        bundle.validate_descriptor(doc, require_matching_sources=False)

    def test_pinned_release_stays_installable_without_repacking_unchanged_weights(self):
        # Snapshot drift is an optional historical audit, not a reason to
        # re-upload exactly the same E24 checkpoint on each code update.
        bundle.validate_descriptor(json.loads(bundle.DESCRIPTOR.read_text()), require_matching_sources=False)

    def test_install_descriptor_still_rejects_unsafe_or_malformed_source_pins(self):
        doc = json.loads(bundle.DESCRIPTOR.read_text())
        for name, value in (('../escape.py', '0' * 64), ('/absolute.py', '0' * 64),
                            ('bad\\path.py', '0' * 64), ('fine.py', 'not-sha256')):
            with self.subTest(name=name):
                bad = dict(doc, source_files_sha256={name: value})
                with self.assertRaises(ValueError):
                    bundle.validate_descriptor(bad, require_matching_sources=False)

    def test_install_descriptor_never_permits_a_different_checkpoint_identity(self):
        doc = json.loads(bundle.DESCRIPTOR.read_text())
        with self.assertRaises(ValueError):
            bundle.validate_descriptor(dict(doc, checkpoint_sha256='0' * 64), require_matching_sources=False)

    def test_e12_build_records_real_archive_size_and_selected_model_without_overwriting_release(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);source=root/'runtime.py';source.write_text('fixed runtime')
            checkpoint=root/'E12.pth';checkpoint.write_bytes(b'known E12 inference weights')
            archive=root/'e12.zip';descriptor=root/'e12_release.json'
            p=dict(bundle.profile('e12'),sha256=bundle.digest(checkpoint))
            with patch.object(bundle,'ROOT',root),patch.object(bundle,'profile',return_value=p), \
                    patch.object(bundle,'required_sources',return_value=['runtime.py']):
                result=bundle.build(checkpoint,archive,'model/bevfusion/production/base5_e12_trt_head_261023',
                                    model='e12',descriptor_output=descriptor)
                self.assertEqual(result,json.loads(descriptor.read_text()))
                self.assertEqual(result['model'],'e12')
                self.assertEqual(result['archive_bytes'],archive.stat().st_size)
                self.assertEqual(result['archive_sha256'],bundle.digest(archive))
                self.assertEqual(result['checkpoint_bytes'],checkpoint.stat().st_size)
                self.assertEqual(result['checkpoint_member'],'weights/base5_e12_inference.pth')
                bundle.validate_archive(archive,result)
                with zipfile.ZipFile(archive) as z:
                    manifest=json.loads(z.read('bundle_manifest.json'))
                    self.assertTrue(manifest['runtime']['camera_tensorrt_requested'])
                    self.assertTrue(manifest['runtime']['head_attention_sdpa'])
                    self.assertTrue(manifest['runtime']['head_cache_bev_pos'])
                    self.assertEqual(z.read(result['checkpoint_member']),checkpoint.read_bytes())
                    self.assertFalse(any(name.endswith(('.engine','.plan')) for name in z.namelist()))
                before=archive.read_bytes(),descriptor.read_bytes()
                with self.assertRaises(FileExistsError):
                    bundle.build(checkpoint,archive,model='e12',descriptor_output=descriptor)
                with self.assertRaises(FileExistsError):
                    bundle.build(checkpoint,root/'new.zip',model='e12',descriptor_output=descriptor)
                self.assertFalse((root/'new.zip').exists())
                self.assertEqual(before,(archive.read_bytes(),descriptor.read_bytes()))

    def test_e12_archive_installs_only_selected_weights_and_preserves_e24(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);archive,descriptor,doc=self.fixture(root,model='e12')
            destination=root/'checkpoints/base5_e12_inference.pth'
            destination.parent.mkdir();old=destination.parent/'base5_e24_inference.pth';old.write_bytes(b'old E24')
            p=dict(bundle.profile('e12'),checkpoint_path=str(destination),sha256=doc['checkpoint_sha256'])
            args=SimpleNamespace(model='e12',archive=archive,remote='unused',check_only=False)
            with patch.object(bundle,'E12_DESCRIPTOR',descriptor),patch.object(bundle,'profile',return_value=p), \
                    patch('prepare_bevfusion_production.profile',return_value=p), \
                    patch.object(bundle.subprocess,'run') as network:
                bundle.install(args);mtime=destination.stat().st_mtime_ns;bundle.install(args)
                self.assertEqual(destination.stat().st_mtime_ns,mtime)
                network.assert_not_called()
            self.assertEqual(destination.read_bytes(),b'known inference weights')
            self.assertEqual(old.read_bytes(),b'old E24')
            self.assertFalse((root/'reference').exists())

    def test_selected_e12_never_accepts_the_e24_release_descriptor(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);archive,descriptor,doc=self.fixture(root,model='e24')
            with patch.object(bundle,'E12_DESCRIPTOR',descriptor), \
                    patch.object(bundle.subprocess,'run') as network:
                with self.assertRaisesRegex(ValueError,'Pinned selected'):
                    bundle.install(SimpleNamespace(model='e12',archive=archive,check_only=False))
                network.assert_not_called()

    def test_e25_rollback_only_checks_existing_legacy_weights(self):
        with patch.object(bundle,'prepare') as prepare,patch.object(bundle.subprocess,'run') as network:
            bundle.install(SimpleNamespace(model='e25',archive=None))
            prepare.assert_called_once_with(SimpleNamespace(model='e25',source_checkpoint=None,check_only=True))
            with self.assertRaisesRegex(ValueError,'not a Base5 ZIP'):
                bundle.install(SimpleNamespace(model='e25',archive=Path('E12.zip')))
            network.assert_not_called()


if __name__ == '__main__':
    unittest.main()
