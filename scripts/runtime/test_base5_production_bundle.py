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
    def fixture(self, root, *, extra=None):
        weights = b'known inference weights'
        source = b'fixed runtime'
        doc = dict(schema=bundle.SCHEMA, model='e24', checkpoint_sha256=hashlib.sha256(weights).hexdigest(),
                   checkpoint_bytes=len(weights), source_files_sha256={'runtime.py': hashlib.sha256(source).hexdigest()})
        manifest = dict(doc, checkpoint_member=bundle.WEIGHTS)
        archive = root / 'bundle.zip'
        with zipfile.ZipFile(archive, 'w') as z:
            z.writestr(bundle.WEIGHTS, weights)
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

    def test_actual_model_config_closure_and_wrong_checkout_rejected(self):
        names = bundle.required_sources()
        self.assertFalse(any('/tools/' in name or '/scripts/benchmark' in name for name in names))
        self.assertIn('perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_pedestrian_merged_runtime.py', names)
        doc = dict(schema=bundle.SCHEMA, model='e24', checkpoint_sha256=bundle.profile('e24')['sha256'],
                   source_files_sha256={name: bundle.digest(bundle.ROOT / name) for name in names})
        bundle.validate_descriptor(doc)
        doc['source_files_sha256'][names[0]] = 'wrong'
        with self.assertRaises(ValueError): bundle.validate_descriptor(doc)

    def test_pinned_release_descriptor_matches_actual_checkout(self):
        # A fresh synthetic descriptor cannot catch a stale production ZIP
        # after an otherwise compatible launcher/telemetry source update.
        bundle.validate_descriptor(json.loads(bundle.DESCRIPTOR.read_text()))


if __name__ == '__main__':
    unittest.main()
