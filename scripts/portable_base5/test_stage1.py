"""Local-fixture deployment safety regressions; never starts ROS or MORAI."""
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tarfile
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import stage1
import portable_paths
import runtime


class FakeDecompressor:
    def __init__(self, payload, returncode=0):
        self.stdout = io.BytesIO(payload)
        self.returncode = returncode
        self.terminated = False

    def wait(self):
        return self.returncode

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True


class Stage1Tests(unittest.TestCase):
    def workspace(self, home):
        root = home / stage1.WORKSPACE_NAME
        root.mkdir()
        return root

    def archive(self, root):
        incoming = root / 'incoming'
        incoming.mkdir()
        archive = incoming / stage1.ARCHIVE_NAME
        archive.write_bytes(b'fixture archive bytes')
        return archive

    def tar_payload(self, entries):
        output = io.BytesIO()
        with tarfile.open(fileobj=output, mode='w') as stream:
            for values in entries:
                name, kind, payload = values[:3]
                entry = tarfile.TarInfo(name)
                entry.type = kind
                if len(values) > 3:
                    entry.mode = values[3]
                if kind == tarfile.REGTYPE:
                    entry.size = len(payload)
                    stream.addfile(entry, io.BytesIO(payload))
                else:
                    entry.linkname = payload.decode()
                    stream.addfile(entry)
        return output.getvalue()

    def mr_fixture(self, root):
        bundle = root / 'capture_bundle'
        source = bundle / 'morai/MoraiLauncher_Lin'
        source.mkdir(parents=True)
        executable = source / 'MoraiLauncher_Lin.x86_64'
        executable.write_bytes(b'not executable; immutable fixture only')
        doc = {'files': {
            executable.relative_to(bundle).as_posix(): {
                'sha256': stage1.digest(executable),
                'bytes': executable.stat().st_size,
            },
        }}
        return bundle, source, doc

    def modes_fixture(self, root, records=None):
        """Real load_bundle validation; only graph-schema verification is stubbed."""
        bundle = root / 'capture_bundle'
        repo = bundle / 'payload/repo'
        repo.mkdir(parents=True)
        release = repo / 'release.json'
        release.write_bytes(b'{"fixture": true}')
        release.chmod(0o644)
        (bundle / 'overlay').mkdir()
        (bundle / 'python').mkdir()
        relative = release.relative_to(bundle).as_posix()
        record = dict(mode=0o664, sha256=stage1.digest(release), bytes=release.stat().st_size)
        graph = dict(files={relative: record} if records is None else records)
        graph_path = bundle / 'portable_graph.json'
        graph_path.write_text(json.dumps(graph))
        doc = dict(schema=runtime.SCHEMA,
                   files={relative: dict(sha256=stage1.digest(release), bytes=release.stat().st_size),
                          'portable_graph.json': dict(sha256=stage1.digest(graph_path),
                                                      bytes=graph_path.stat().st_size)},
                   runtime=dict(repository='payload/repo', release=relative,
                                ros_overlay='overlay', python_overlay='python'))
        manifest = bundle / 'portable_bundle.json'
        manifest.write_text(json.dumps(doc))
        return bundle, release, graph, manifest

    def test_scope_requires_exact_approved_home_child(self):
        with tempfile.TemporaryDirectory() as folder:
            home = Path(folder)
            root = self.workspace(home)
            with patch.object(Path, 'home', return_value=home):
                self.assertEqual(stage1.scope(root), root)
                for candidate in (home, home / 'other', home / 'nested' / stage1.WORKSPACE_NAME,
                                  root / '..' / stage1.WORKSPACE_NAME):
                    with self.subTest(path=candidate), self.assertRaises(ValueError):
                        stage1.scope(candidate)

    def test_scope_refuses_missing_file_foreign_owner_and_symlink(self):
        with tempfile.TemporaryDirectory() as folder:
            home = Path(folder)
            root = home / stage1.WORKSPACE_NAME
            with patch.object(Path, 'home', return_value=home):
                with self.assertRaises(ValueError):
                    stage1.scope(root)
                root.write_text('not a directory')
                with self.assertRaises(ValueError):
                    stage1.scope(root)
                root.unlink()
                root.mkdir()
                with patch.object(stage1.os, 'getuid', return_value=-1), self.assertRaises(ValueError):
                    stage1.scope(root)
                root.rmdir()
                target = home / 'other'
                target.mkdir()
                root.symlink_to(target, target_is_directory=True)
                with self.assertRaisesRegex(ValueError, 'symlink'):
                    stage1.scope(root)

    def test_member_path_accepts_only_canonical_bundle_relative_files(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.assertEqual(stage1.member_path(root, 'capture_bundle/a/file'),
                             root / 'capture_bundle/a/file')
            for member in ('', '.', '/capture_bundle/a', '../outside',
                           'capture_bundle/../outside', 'other/file',
                           './capture_bundle/file', 'capture_bundle//file',
                           'capture_bundle/./file', 'capture_bundle/file/',
                           'capture_bundle\\file'):
                with self.subTest(member=member), self.assertRaises(ValueError):
                    stage1.member_path(root, member)

    def test_member_path_refuses_destination_symlink(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            bundle = root / 'capture_bundle'
            bundle.mkdir()
            (bundle / 'alias').symlink_to(root, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                stage1.member_path(root, 'capture_bundle/alias/outside')
            (bundle / 'file').symlink_to(root / 'missing')
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                stage1.member_path(root, 'capture_bundle/file')

    def test_extract_refuses_changed_or_symlinked_input_before_process(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = self.archive(root)
            with patch.object(stage1.subprocess, 'Popen') as process:
                with self.assertRaisesRegex(ValueError, 'SHA'):
                    stage1.extract(root)
                archive.unlink()
                other = root / 'other'
                other.write_bytes(b'fixture')
                archive.symlink_to(other)
                with self.assertRaisesRegex(ValueError, 'SHA'):
                    stage1.extract(root)
                process.assert_not_called()

    def test_extract_refuses_existing_or_symlinked_destination(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = self.archive(root)
            target = root / 'capture_bundle'
            target.mkdir()
            (target / 'keep').write_text('existing evidence')
            with patch.object(stage1, 'ARCHIVE_SHA', stage1.digest(archive)), \
                    patch.object(stage1.subprocess, 'Popen') as process:
                with self.assertRaisesRegex(ValueError, 'existing extraction'):
                    stage1.extract(root)
                self.assertEqual((target / 'keep').read_text(), 'existing evidence')
                (target / 'keep').unlink()
                target.rmdir()
                target.symlink_to(root / 'missing', target_is_directory=True)
                with self.assertRaisesRegex(ValueError, 'existing extraction'):
                    stage1.extract(root)
                process.assert_not_called()

    def test_extract_refuses_low_headroom_before_process(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = self.archive(root)
            with patch.object(stage1, 'ARCHIVE_SHA', stage1.digest(archive)), \
                    patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=0)), \
                    patch.object(stage1.subprocess, 'Popen') as process:
                with self.assertRaisesRegex(ValueError, 'headroom'):
                    stage1.extract(root)
                process.assert_not_called()
                self.assertFalse((root / 'capture_bundle').exists())

    def test_extract_rejects_archive_links_directories_and_traversal(self):
        for kind, name, payload in (
                (tarfile.SYMTYPE, 'capture_bundle/alias', b'/outside'),
                (tarfile.LNKTYPE, 'capture_bundle/link', b'capture_bundle/file'),
                (tarfile.DIRTYPE, 'capture_bundle/dir', b''),
                (tarfile.REGTYPE, 'capture_bundle/../outside', b'bad'),
                (tarfile.REGTYPE, '/capture_bundle/absolute', b'bad')):
            with self.subTest(kind=kind, name=name), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                archive = self.archive(root)
                fake = FakeDecompressor(self.tar_payload([(name, kind, payload)]))
                with patch.object(stage1, 'ARCHIVE_SHA', stage1.digest(archive)), \
                        patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)), \
                        patch.object(stage1.subprocess, 'Popen', return_value=fake):
                    with self.assertRaises(ValueError):
                        stage1.extract(root)
                self.assertFalse((root / 'outside').exists())
                self.assertFalse((root / 'capture_bundle').exists())
                self.assertTrue(fake.stdout.closed)

    def test_extract_success_verifies_manifest_count_and_original_archive(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = self.archive(root)
            original = archive.read_bytes()
            manifest = b'{"fixture": true}'
            payload = self.tar_payload([
                ('capture_bundle/portable_bundle.json', tarfile.REGTYPE, manifest),
                ('capture_bundle/nested/data', tarfile.REGTYPE, b'immutable data'),
            ])
            fake = FakeDecompressor(payload)
            with patch.object(stage1, 'ARCHIVE_SHA', stage1.digest(archive)), \
                    patch.object(stage1, 'MANIFEST_SHA', hashlib.sha256(manifest).hexdigest()), \
                    patch.object(stage1, 'EXPECTED_FILES', 2), \
                    patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)), \
                    patch.object(stage1.subprocess, 'Popen', return_value=fake) as process:
                result = stage1.extract(root)
                process.assert_called_once_with(
                    ['zstd', '-q', '-d', '-c', str(archive)], stdout=stage1.subprocess.PIPE)
            self.assertEqual(result['files'], 2)
            self.assertEqual(result['bytes'], len(manifest) + len(b'immutable data'))
            self.assertEqual((root / 'capture_bundle/nested/data').read_bytes(), b'immutable data')
            self.assertEqual(archive.read_bytes(), original)
            self.assertTrue(fake.stdout.closed)

    def test_extract_refuses_wrong_final_manifest_or_file_count(self):
        for bad_manifest, expected_count in ((True, 1), (False, 2)):
            with self.subTest(manifest=bad_manifest, count=expected_count), \
                    tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                archive = self.archive(root)
                manifest = b'{}'
                fake = FakeDecompressor(self.tar_payload([
                    ('capture_bundle/portable_bundle.json', tarfile.REGTYPE, manifest),
                ]))
                expected_sha = '0' * 64 if bad_manifest else hashlib.sha256(manifest).hexdigest()
                with patch.object(stage1, 'ARCHIVE_SHA', stage1.digest(archive)), \
                        patch.object(stage1, 'MANIFEST_SHA', expected_sha), \
                        patch.object(stage1, 'EXPECTED_FILES', expected_count), \
                        patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)), \
                        patch.object(stage1.subprocess, 'Popen', return_value=fake):
                    with self.assertRaisesRegex(ValueError, 'manifest/file count differs'):
                        stage1.extract(root)
                self.assertEqual((root / 'capture_bundle/portable_bundle.json').read_bytes(), manifest)

    def test_extract_preserves_posix_bits_but_drops_special_privilege_bits(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = self.archive(root)
            manifest = b'{}'
            fake = FakeDecompressor(self.tar_payload([
                ('capture_bundle/portable_bundle.json', tarfile.REGTYPE, manifest, 0o640),
                ('capture_bundle/group-writable', tarfile.REGTYPE, b'data', 0o664),
                ('capture_bundle/program', tarfile.REGTYPE, b'not executed', 0o4770),
            ]))
            with patch.object(stage1, 'ARCHIVE_SHA', stage1.digest(archive)), \
                    patch.object(stage1, 'MANIFEST_SHA', hashlib.sha256(manifest).hexdigest()), \
                    patch.object(stage1, 'EXPECTED_FILES', 3), \
                    patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)), \
                    patch.object(stage1.subprocess, 'Popen', return_value=fake):
                stage1.extract(root)
            for name, mode in (('portable_bundle.json', 0o640), ('group-writable', 0o664),
                               ('program', 0o770)):
                self.assertEqual((root / 'capture_bundle' / name).stat().st_mode & 0o7777, mode)

    def test_extract_does_not_overwrite_duplicate_members(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = self.archive(root)
            fake = FakeDecompressor(self.tar_payload([
                ('capture_bundle/data', tarfile.REGTYPE, b'first'),
                ('capture_bundle/data', tarfile.REGTYPE, b'second'),
            ]))
            with patch.object(stage1, 'ARCHIVE_SHA', stage1.digest(archive)), \
                    patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)), \
                    patch.object(stage1.subprocess, 'Popen', return_value=fake):
                with self.assertRaises(FileExistsError):
                    stage1.extract(root)
            self.assertEqual((root / 'capture_bundle/data').read_bytes(), b'first')

    def test_extract_refuses_decompressor_failure_and_keeps_partial_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = self.archive(root)
            fake = FakeDecompressor(self.tar_payload([
                ('capture_bundle/data', tarfile.REGTYPE, b'partial evidence'),
            ]), returncode=1)
            with patch.object(stage1, 'ARCHIVE_SHA', stage1.digest(archive)), \
                    patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)), \
                    patch.object(stage1.subprocess, 'Popen', return_value=fake):
                with self.assertRaisesRegex(ValueError, 'Decompression failed'):
                    stage1.extract(root)
            self.assertEqual((root / 'capture_bundle/data').read_bytes(), b'partial evidence')

    def test_morai_copy_is_fresh_unlinked_and_source_stays_immutable(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            bundle, source, doc = self.mr_fixture(root)
            original = (source / 'MoraiLauncher_Lin.x86_64').read_bytes()
            with patch.object(stage1, 'load', return_value=(None, bundle, doc)), \
                    patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)), \
                    patch.object(stage1.subprocess, 'run') as command:
                result = stage1.prepare_morai(root)
                command.assert_not_called()
            target = Path(result['morai']) / 'MoraiLauncher_Lin.x86_64'
            self.assertFalse(os.path.samefile(source / target.name, target))
            self.assertFalse(result['hardlinked'])
            self.assertFalse(result['morai_started'])
            target.write_bytes(b'mutable runtime output')
            self.assertEqual((source / target.name).read_bytes(), original)

    def test_morai_copy_refuses_existing_target_and_parent_symlink(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            bundle, _, doc = self.mr_fixture(root)
            target = root / 'work/morai/MoraiLauncher_Lin'
            target.mkdir(parents=True)
            with patch.object(stage1, 'load', return_value=(None, bundle, doc)), \
                    patch.object(stage1.shutil, 'copytree') as copy, \
                    patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)):
                with self.assertRaisesRegex(ValueError, 'Existing'):
                    stage1.prepare_morai(root)
                target.rmdir()
                target.parent.rmdir()
                target.parent.parent.rmdir()
                outside = root / 'outside'
                outside.mkdir()
                (root / 'work').symlink_to(outside, target_is_directory=True)
                with self.assertRaisesRegex(ValueError, 'symlink'):
                    stage1.prepare_morai(root)
                copy.assert_not_called()
                self.assertEqual(list(outside.iterdir()), [])

    def test_morai_copy_refuses_low_headroom_before_copy(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            bundle, _, doc = self.mr_fixture(root)
            with patch.object(stage1, 'load', return_value=(None, bundle, doc)), \
                    patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=0)), \
                    patch.object(stage1.shutil, 'copytree') as copy:
                with self.assertRaisesRegex(ValueError, 'headroom'):
                    stage1.prepare_morai(root)
                copy.assert_not_called()

    def test_morai_copy_refuses_changed_source_or_hardlinks(self):
        for changed in (True, False):
            with self.subTest(changed=changed), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                bundle, source, doc = self.mr_fixture(root)
                if changed:
                    (source / 'MoraiLauncher_Lin.x86_64').write_bytes(b'changed after manifest')
                with patch.object(stage1, 'load', return_value=(None, bundle, doc)), \
                        patch.object(stage1.shutil, 'disk_usage', return_value=SimpleNamespace(free=20*1024**3)), \
                        patch.object(stage1.os.path, 'samefile', return_value=not changed):
                    with self.assertRaisesRegex(ValueError, 'differs or is hardlinked'):
                        stage1.prepare_morai(root)

    def test_restore_modes_changes_only_posix_permissions_not_file_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            _, release, _, manifest = self.modes_fixture(root)
            original = release.read_bytes()
            with patch.object(stage1, 'MANIFEST_SHA', stage1.digest(manifest)), \
                    patch.object(portable_paths, 'verify_graph', return_value={'files_checked': 1}) as verify, \
                    patch.object(stage1.subprocess, 'run') as child:
                result = stage1.restore_modes(root)
                verify.assert_called_once()
                child.assert_not_called()
            self.assertEqual(release.stat().st_mode & 0o777, 0o664)
            self.assertEqual(release.read_bytes(), original)
            self.assertEqual(result['mode_changes'], 1)
            self.assertFalse(result['file_contents_changed'])
            self.assertFalse(result['host_system_changed'])

    def test_restore_modes_is_idempotent(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            _, release, _, manifest = self.modes_fixture(root)
            with patch.object(stage1, 'MANIFEST_SHA', stage1.digest(manifest)), \
                    patch.object(portable_paths, 'verify_graph', return_value={'files_checked': 1}):
                self.assertEqual(stage1.restore_modes(root)['mode_changes'], 1)
                self.assertEqual(stage1.restore_modes(root)['mode_changes'], 0)
            self.assertEqual(release.stat().st_mode & 0o777, 0o664)

    def test_restore_modes_refuses_changed_manifest_graph_or_payload(self):
        for changed in ('manifest', 'graph', 'payload'):
            with self.subTest(changed=changed), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                bundle, release, _, manifest = self.modes_fixture(root)
                known_sha = stage1.digest(manifest)
                target = {'manifest': manifest, 'graph': bundle / 'portable_graph.json',
                          'payload': release}[changed]
                target.write_bytes(target.read_bytes() + b'changed')
                with patch.object(stage1, 'MANIFEST_SHA', known_sha), \
                        patch.object(portable_paths, 'verify_graph') as verify:
                    with self.assertRaises(ValueError):
                        stage1.restore_modes(root)
                    verify.assert_not_called()
                self.assertEqual(release.stat().st_mode & 0o777, 0o644)

    def test_restore_modes_refuses_symlinked_payload_or_parent(self):
        for parent in (False, True):
            with self.subTest(parent=parent), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                bundle, release, _, manifest = self.modes_fixture(root)
                original = release.read_bytes()
                outside = root / 'outside'
                outside.mkdir()
                escaped = outside / 'release.json'
                escaped.write_bytes(original)
                escaped.chmod(0o640)
                release.unlink()
                if parent:
                    release.parent.rmdir()
                    release.parent.symlink_to(outside, target_is_directory=True)
                else:
                    release.symlink_to(escaped)
                with patch.object(stage1, 'MANIFEST_SHA', stage1.digest(manifest)), \
                        patch.object(portable_paths, 'verify_graph') as verify:
                    with self.assertRaisesRegex(ValueError, 'Symlink'):
                        stage1.restore_modes(root)
                    verify.assert_not_called()
                self.assertEqual(escaped.stat().st_mode & 0o777, 0o640)
                self.assertEqual(escaped.read_bytes(), original)

    def test_restore_modes_refuses_unlisted_or_outside_graph_records(self):
        for relative in ('../outside', '/tmp/outside', 'tools/runtime.py', 'payload/unlisted'):
            with self.subTest(relative=relative), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                _, release, _, manifest = self.modes_fixture(
                    root, records={relative: dict(mode=0o664, sha256='0' * 64)})
                original = release.read_bytes()
                with patch.object(stage1, 'MANIFEST_SHA', stage1.digest(manifest)), \
                        patch.object(portable_paths, 'verify_graph') as verify:
                    with self.assertRaisesRegex(ValueError, 'outside the immutable graph'):
                        stage1.restore_modes(root)
                    verify.assert_not_called()
                self.assertEqual(release.stat().st_mode & 0o777, 0o644)
                self.assertEqual(release.read_bytes(), original)

    def test_restore_modes_refuses_untrusted_mode_or_hash(self):
        for mode in (-1, 0o1000, '0664', 6.64, None, True, False, 0o664):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                relative = 'payload/repo/release.json'
                record = dict(mode=mode, sha256='0' * 64)
                # All non-normal mode values are rejected even with a matching SHA;
                # the final normal-mode case specifically checks a mismatched SHA.
                if mode != 0o664:
                    record['sha256'] = hashlib.sha256(b'{"fixture": true}').hexdigest()
                _, release, _, manifest = self.modes_fixture(root, records={relative: record})
                original = release.read_bytes()
                with patch.object(stage1, 'MANIFEST_SHA', stage1.digest(manifest)), \
                        patch.object(portable_paths, 'verify_graph') as verify:
                    with self.assertRaisesRegex(ValueError, 'Untrusted mode record'):
                        stage1.restore_modes(root)
                    verify.assert_not_called()
                self.assertEqual(release.stat().st_mode & 0o777, 0o644)
                self.assertEqual(release.read_bytes(), original)

    def test_default_plan_makes_no_files_and_runs_no_child_actions(self):
        for action in ('extract', 'restore-modes', 'check', 'prepare-morai'):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as folder:
                home = Path(folder)
                root = self.workspace(home)
                output = io.StringIO()
                with patch.object(Path, 'home', return_value=home), \
                        patch.object(sys, 'argv', ['stage1.py', '--workspace', str(root), action]), \
                        patch.object(stage1, 'extract') as extract, \
                        patch.object(stage1, 'restore_modes') as restore, \
                        patch.object(stage1, 'check') as check, \
                        patch.object(stage1, 'prepare_morai') as prepare, \
                        patch.object(stage1.subprocess, 'run') as child, \
                        patch.object(stage1.subprocess, 'Popen') as process, \
                        contextlib.redirect_stdout(output):
                    stage1.main()
                    for unused in (extract, restore, check, prepare, child, process):
                        unused.assert_not_called()
                self.assertEqual(list(root.iterdir()), [])
                receipt = json.loads(output.getvalue())
                self.assertFalse(receipt['execute'])
                self.assertFalse(receipt['morai_started'])


if __name__ == '__main__':
    unittest.main()
