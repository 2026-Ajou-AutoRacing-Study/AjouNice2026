"""Tiny local tar/zstd fixtures; no real bundle, network, ROS or GPU execution."""
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import pack
import runtime


@unittest.skipUnless(shutil.which('zstd'), 'Tests require existing zstd; no installation')
class PackTests(unittest.TestCase):
    def fixture(self, root):
        bundle = root / 'bundle'
        repo = bundle / 'payload/home/luv/AjouNice2026'
        repo.mkdir(parents=True)
        release = repo / 'release.json'
        release.write_text('{"bulk_ready":true}\n')
        executable = bundle / 'bin/bwrap'
        executable.parent.mkdir()
        executable.write_bytes(b'fixture executable, never run\n')
        executable.chmod(0o755)
        regular = bundle / 'tools/runtime.py'
        regular.parent.mkdir()
        regular.write_bytes(b'# fixture source, never run\n')
        regular.chmod(0o640)
        (bundle / 'overlay').mkdir()
        (bundle / 'python').mkdir()
        files = {p.relative_to(bundle).as_posix(): dict(sha256=runtime.sha256(p), bytes=p.stat().st_size)
                 for p in (release, executable, regular)}
        doc = dict(schema=runtime.SCHEMA, files=files, can_run=False,
                   runtime=dict(repository=repo.relative_to(bundle).as_posix(),
                                release=release.relative_to(bundle).as_posix(),
                                ros_overlay='overlay', python_overlay='python'))
        (bundle / runtime.MANIFEST).write_text(json.dumps(doc))
        return bundle, doc, root / 'bundle.tar.zst'

    def expected(self, bundle, doc):
        paths = dict(doc['files'])
        paths[runtime.MANIFEST] = dict(sha256=runtime.sha256(bundle / runtime.MANIFEST),
                                      bytes=(bundle / runtime.MANIFEST).stat().st_size)
        return {bundle.name + '/' + name: item for name, item in paths.items()}

    def snapshot(self, folder):
        return {p.relative_to(folder).as_posix(): (runtime.sha256(p), p.stat().st_mode,
                p.stat().st_ino, p.stat().st_mtime_ns) for p in folder.rglob('*') if p.is_file()}

    def test_default_plan_does_not_write_or_start_compression(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            bundle, _, archive = self.fixture(root)
            before = self.snapshot(root)
            with patch.object(pack.subprocess, 'Popen') as child:
                result = pack.pack(bundle, archive)
                child.assert_not_called()
            self.assertFalse(result['can_run'])
            self.assertFalse(result['remote_touched'])
            self.assertEqual(before, self.snapshot(root))

    def test_real_archive_preserves_executable_bits_and_verifies_all_sha(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            bundle, doc, archive = self.fixture(root)
            before = self.snapshot(bundle)
            with patch.object(pack, 'verify_archive', wraps=pack.verify_archive) as verify:
                result = pack.pack(bundle, archive, run=True)
                verify.assert_called_once()
            expected = self.expected(bundle, doc)
            pack.verify_archive(archive, expected)
            result_bytes = subprocess.run(['zstd', '-q', '-d', '-c', str(archive)],
                                          capture_output=True, check=True).stdout
            with tarfile.open(fileobj=io.BytesIO(result_bytes), mode='r:') as tar:
                members = {item.name: item for item in tar}
                self.assertEqual(set(members), set(expected))
                self.assertEqual(members['bundle/bin/bwrap'].mode, 0o755)
                self.assertEqual(members['bundle/tools/runtime.py'].mode, 0o644)
                self.assertTrue(all(item.isfile() for item in members.values()))
            self.assertEqual(result['sha256'], runtime.sha256(archive))
            self.assertEqual(result['bytes'], archive.stat().st_size)
            self.assertTrue(result['verified_members'])
            self.assertTrue(result['executable_modes_preserved'])
            self.assertEqual(archive.with_name(archive.name + '.sha256').read_text(),
                             result['sha256'] + '  ' + archive.name + '\n')
            self.assertFalse(archive.with_suffix(archive.suffix + '.partial').exists())
            self.assertEqual(before, self.snapshot(bundle))

    def test_verifier_rejects_wrong_member_sha(self):
        with tempfile.TemporaryDirectory() as t:
            bundle, doc, archive = self.fixture(Path(t))
            pack.pack(bundle, archive, run=True)
            expected = self.expected(bundle, doc)
            expected['bundle/bin/bwrap']['sha256'] = '0' * 64
            with self.assertRaisesRegex(ValueError, 'verification failed'):
                pack.verify_archive(archive, expected)

    def test_existing_archive_partial_or_checksum_is_preserved_before_pack(self):
        for which in ('archive', 'partial', 'checksum'):
            with self.subTest(which=which), tempfile.TemporaryDirectory() as t:
                bundle, _, archive = self.fixture(Path(t))
                paths = dict(archive=archive, partial=archive.with_suffix(archive.suffix + '.partial'),
                             checksum=archive.with_name(archive.name + '.sha256'))
                existing = paths[which]
                existing.write_bytes(b'USER FILE')
                with patch.object(pack.subprocess, 'Popen') as child:
                    with self.assertRaises((ValueError, OSError)):
                        pack.pack(bundle, archive, run=True)
                    child.assert_not_called()
                self.assertEqual(existing.read_bytes(), b'USER FILE')
                if which != 'archive':
                    self.assertFalse(archive.exists())

    def test_dangling_output_symlink_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as t:
            bundle, _, archive = self.fixture(Path(t))
            archive.symlink_to(archive.parent / 'missing-user-target')
            with self.assertRaises((ValueError, OSError)):
                pack.pack(bundle, archive, run=True)
            self.assertTrue(archive.is_symlink())

    def test_symlink_parent_must_not_redirect_output_into_bundle(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            bundle, _, _ = self.fixture(root)
            alias = root / 'alias'
            alias.symlink_to(bundle, target_is_directory=True)
            archive = alias / 'injected.tar.zst'
            before = self.snapshot(bundle)
            with self.assertRaises((ValueError, OSError)):
                pack.pack(bundle, archive, run=True)
            self.assertEqual(before, self.snapshot(bundle))

    def test_dotdot_output_must_not_bypass_immutable_bundle_boundary(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            bundle, _, _ = self.fixture(root)
            (root / 'elsewhere').mkdir()
            archive = root / 'elsewhere/../bundle/injected.tar.zst'
            with self.assertRaises((ValueError, OSError)):
                pack.pack(bundle, archive, run=True)
            self.assertFalse((bundle / 'injected.tar.zst').exists())

    def test_direct_output_inside_bundle_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            bundle, _, _ = self.fixture(Path(t))
            with self.assertRaisesRegex(ValueError, 'outside immutable bundle'):
                pack.pack(bundle, bundle / 'inside.tar.zst', run=True)

    def test_symlink_or_changed_source_is_rejected_before_pack(self):
        for symlink in (False, True):
            with self.subTest(symlink=symlink), tempfile.TemporaryDirectory() as t:
                bundle, _, archive = self.fixture(Path(t))
                target = bundle / 'bin/bwrap'
                if symlink:
                    target.unlink()
                    target.symlink_to(bundle / 'tools/runtime.py')
                else:
                    target.write_bytes(b'CHANGED')
                with patch.object(pack.subprocess, 'Popen') as child:
                    with self.assertRaises(ValueError):
                        pack.pack(bundle, archive, run=True)
                    child.assert_not_called()
                self.assertFalse(archive.exists())

    def test_source_change_after_initial_check_cannot_publish_inconsistent_tar(self):
        with tempfile.TemporaryDirectory() as t:
            bundle, _, archive = self.fixture(Path(t))
            def raced_load(path):
                result = runtime.load_bundle(path)
                target = bundle / 'bin/bwrap'
                # Keep length constant so rejection proves checksum pinning,
                # rather than an unrelated tar short-read error.
                target.write_bytes(b'X' * target.stat().st_size)
                return result
            with patch.object(pack, 'load_bundle', side_effect=raced_load):
                with self.assertRaises(ValueError):
                    pack.pack(bundle, archive, run=True)
            self.assertFalse(archive.exists())

    def test_failed_verification_preserves_partial_and_never_publishes_final(self):
        with tempfile.TemporaryDirectory() as t:
            bundle, _, archive = self.fixture(Path(t))
            with patch.object(pack, 'verify_archive', side_effect=ValueError('fixture verification failed')):
                with self.assertRaisesRegex(ValueError, 'verification failed'):
                    pack.pack(bundle, archive, run=True)
            self.assertFalse(archive.exists())
            self.assertTrue(archive.with_suffix(archive.suffix + '.partial').is_file())
            self.assertFalse(archive.with_name(archive.name + '.sha256').exists())

    def test_missing_compressor_or_headroom_never_writes_partial(self):
        with tempfile.TemporaryDirectory() as t:
            bundle, _, archive = self.fixture(Path(t))
            with patch.object(pack.shutil, 'which', return_value=None):
                with self.assertRaisesRegex(ValueError, 'zstd'):
                    pack.pack(bundle, archive, run=True)
            with patch.object(pack.shutil, 'disk_usage', return_value=SimpleNamespace(free=0)):
                with self.assertRaisesRegex(ValueError, 'headroom'):
                    pack.pack(bundle, archive, run=True)
            self.assertFalse(archive.with_suffix(archive.suffix + '.partial').exists())


if __name__ == '__main__':
    unittest.main()
