"""Installer command decisions and archive safety, never installing host packages."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch

from planning_scenario_validator import bootstrap as setup


def result(code=0, text=''):
    return subprocess.CompletedProcess([], code, text)


class BootstrapTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def tar(self, members):
        path = self.root / 'release.tar'
        with tarfile.open(path, 'w') as archive:
            for name, data, kind in members:
                member = tarfile.TarInfo(name)
                member.type = kind
                if kind == tarfile.REGTYPE:
                    member.size = len(data)
                    archive.addfile(member, io.BytesIO(data))
                else:
                    member.linkname = '/tmp/forbidden'
                    archive.addfile(member)
        return path

    def test_extract_reads_single_member_not_archive_paths(self):
        data = b'checkpoint'
        archive = self.tar([('../../escape.txt', b'ignore', tarfile.REGTYPE),
                            ('release/checkpoints/' + setup.E25_NAME, data, tarfile.REGTYPE)])
        target = self.root / 'out/model.pth'
        setup.extract_checkpoint(archive, target, hashlib.sha256(data).hexdigest())
        self.assertEqual(target.read_bytes(), data)
        self.assertEqual([p.name for p in target.parent.iterdir()], ['model.pth'])
        self.assertFalse((self.root / 'escape.txt').exists())

    def test_duplicate_or_symlink_checkpoint_is_rejected(self):
        for members in ([('a/' + setup.E25_NAME, b'x', tarfile.REGTYPE), ('b/' + setup.E25_NAME, b'x', tarfile.REGTYPE)],
                        [(setup.E25_NAME, b'', tarfile.SYMTYPE)]):
            target = self.root / 'out/model.pth'
            with self.assertRaises(setup.SetupError):
                setup.extract_checkpoint(self.tar(members), target)
            self.assertFalse(target.exists())
            self.assertEqual(list(target.parent.iterdir()), [])

    def test_bad_hash_or_existing_checkpoint_is_preserved(self):
        data = b'checkpoint'
        archive = self.tar([(setup.E25_NAME, data, tarfile.REGTYPE)])
        target = self.root / 'out/model.pth'
        with self.assertRaisesRegex(setup.SetupError, 'SHA256'):
            setup.extract_checkpoint(archive, target)
        target.write_bytes(b'user model')
        with self.assertRaises(FileExistsError):
            setup.extract_checkpoint(archive, target, hashlib.sha256(data).hexdigest())
        self.assertEqual(target.read_bytes(), b'user model')

    def test_download_reuses_verified_file_and_preserves_wrong_existing(self):
        target = self.root / 'cached.zst'
        target.write_bytes(b'good')
        with patch.object(setup, 'run') as command:
            self.assertEqual(setup.download('remote', 'id', 'source', target, hashlib.sha256(b'good').hexdigest()), target)
            command.assert_not_called()
            with self.assertRaisesRegex(setup.SetupError, '기존 압축'):
                setup.download('remote', 'id', 'source', target, '0'*64)
            self.assertEqual(target.read_bytes(), b'good')

    def test_download_uses_partial_verifies_then_renames(self):
        target = self.root / 'cached.zst'
        def fake(args, **kwargs):
            if args[:2] == ['rclone', 'copyto']:
                Path(args[3]).write_bytes(b'good')
            return result()
        with patch.object(setup, 'run', side_effect=fake) as command:
            setup.download('remote', 'id', 'source', target, hashlib.sha256(b'good').hexdigest())
        self.assertEqual(target.read_bytes(), b'good')
        self.assertFalse(target.with_suffix('.zst.part').exists())
        self.assertEqual(command.call_args_list[0].args[0][2], 'remote:source')

    def test_partial_bad_hash_never_becomes_registered_archive(self):
        target = self.root / 'cached.zst'
        def fake(args, **kwargs):
            if args[:2] == ['rclone', 'copyto']:
                Path(args[3]).write_bytes(b'corrupted')
            return result()
        with patch.object(setup, 'run', side_effect=fake):
            with self.assertRaisesRegex(setup.SetupError, 'checksum'):
                setup.download('remote', 'id', 'source', target, '0'*64)
        self.assertFalse(target.exists())
        self.assertTrue(target.with_suffix('.zst.part').exists())

    def test_dry_run_has_no_sudo_download_or_gpu_side_effects(self):
        args = argparse.Namespace(dry_run=True, remote='ajou_drive', download_dir=self.root / 'downloads', checkpoint_path=self.root / 'weights/model.pth')
        with patch.object(setup, 'ubuntu_check'), patch.object(setup.os, 'geteuid', return_value=1000), patch.object(setup, 'run') as command:
            setup.setup(args)
        command.assert_not_called()
        self.assertFalse(args.download_dir.exists())
        self.assertFalse(args.checkpoint_path.parent.exists())

    def test_conflicting_legacy_packages_are_not_removed(self):
        with patch.object(setup.shutil, 'which', return_value=None), patch.object(setup, 'installed', side_effect=lambda name: name == 'docker.io'), patch.object(setup, 'run') as command:
            with self.assertRaisesRegex(setup.SetupError, '자동 삭제하지'):
                setup.system_setup()
        command.assert_not_called()

    def test_active_containers_block_toolkit_restart(self):
        def fake(args, **kwargs):
            if args[-2:] == ['context', 'show']: return result(text='default')
            if args[-1] == '{{json .Runtimes}}': return result(text='{"runc": {}}')
            if args[-1] == '{{json .SecurityOptions}}': return result(text='[]')
            if args[-2:] == ['ps', '-q']: return result(text='training-container')
            return result()
        with patch.object(setup.shutil, 'which', return_value='/usr/bin/docker'), patch.object(setup, 'installed', return_value=True), patch.object(setup, 'run', side_effect=fake) as command:
            with self.assertRaisesRegex(setup.SetupError, '실행 중인 컨테이너'):
                setup.system_setup()
        self.assertFalse(any('restart' in call.args[0] for call in command.call_args_list))

    def test_existing_ready_system_does_not_install_or_restart(self):
        def fake(args, **kwargs):
            if args[-2:] == ['context', 'show']: return result(text='default')
            if args[-1] == '{{json .Runtimes}}': return result(text='{"nvidia": {}, "runc": {}}')
            return result()
        with patch.object(setup.shutil, 'which', return_value='/usr/bin/docker'), patch.object(setup, 'installed', return_value=True), patch.object(setup, 'run', side_effect=fake) as command:
            self.assertEqual(setup.system_setup(), ['docker'])
        self.assertFalse(any(call.args[0][0] == 'sudo' for call in command.call_args_list))

    def test_fresh_install_configures_engine_and_toolkit_in_order_without_removal(self):
        def fake(args, **kwargs):
            if args[-1] == '{{json .Runtimes}}': return result(text='{"runc": {}}')
            if args[-1] == '{{json .SecurityOptions}}': return result(text='[]')
            return result()
        with patch.object(setup.shutil, 'which', return_value=None), patch.object(setup, 'installed', side_effect=lambda name: name not in setup.CONFLICTS), patch.object(setup, 'run', side_effect=fake) as command, patch.object(setup, 'repository') as repositories, patch.object(setup, 'apt_install') as install:
            self.assertEqual(setup.system_setup(), ['docker'])
        self.assertEqual([call.args[0] for call in repositories.call_args_list], ['docker', 'nvidia'])
        self.assertEqual(install.call_args_list[0].args[0], ['docker-ce', 'docker-ce-cli', 'containerd.io', 'docker-compose-plugin'])
        self.assertEqual(install.call_args_list[1].args[0], ['nvidia-container-toolkit'])
        args = [call.args[0] for call in command.call_args_list]
        self.assertIn(['sudo', 'systemctl', 'enable', '--now', 'docker'], args)
        self.assertIn(['sudo', 'nvidia-ctk', 'runtime', 'configure', '--runtime=docker'], args)
        self.assertIn(['sudo', 'systemctl', 'restart', 'docker'], args)
        self.assertFalse(any('remove' in row or 'purge' in row for row in args))

    @unittest.skipUnless(setup.shutil.which('zstd'), 'zstd required for stream integration')
    def test_real_zstd_stream_reaches_load_command_without_tar_copy(self):
        blob = b'tar-stream-bytes' * 4096
        plain = self.root / 'source.tar'
        plain.write_bytes(blob)
        archive = self.root / 'image.tar.zst'
        subprocess.run(['zstd', '-q', str(plain), '-o', str(archive)], check=True)
        output = self.root / 'received'
        # This loader only captures stdin; no host Docker state is changed.
        loader = self.root / 'loader.py'
        loader.write_text('import sys\nfrom pathlib import Path\nPath(sys.argv[1]).write_bytes(sys.stdin.buffer.read())\n')
        setup.load_image(archive, ['/usr/bin/python3', str(loader), str(output)])
        self.assertEqual(output.read_bytes(), blob)

    def test_wrong_user_endpoint_is_not_replaced_with_root_daemon(self):
        with patch.dict(os.environ, {'DOCKER_HOST': 'tcp://custom'}), patch.object(setup, 'run', return_value=result(1)) as command:
            with self.assertRaisesRegex(setup.SetupError, 'endpoint'):
                setup.docker_command(True)
        self.assertEqual(command.call_count, 1)

    def test_working_custom_context_does_not_trigger_local_system_mutation(self):
        with patch.object(setup.shutil, 'which', return_value='/usr/bin/docker'), patch.object(setup, 'run', return_value=result(text='remote-training')) as command:
            with self.assertRaisesRegex(setup.SetupError, '기본 로컬'):
                setup.system_setup()
        self.assertFalse(any('sudo' in call.args[0] for call in command.call_args_list))

    def test_apt_missing_candidate_fails_without_install(self):
        with patch.object(setup, 'run', return_value=result(text='Candidate: (none)')) as command:
            with self.assertRaisesRegex(setup.SetupError, '패키지가 없습니다'):
                setup.apt_install(['docker-ce'])
        self.assertEqual(command.call_count, 1)

    def test_existing_system_file_is_never_overwritten(self):
        source, target = self.root / 'source', self.root / 'target'
        source.write_text('new')
        target.write_text('user key')
        with patch.object(setup, 'run') as command:
            with self.assertRaisesRegex(setup.SetupError, '덮어쓰지'):
                setup.managed_system_file(source, target)
        command.assert_not_called()
        self.assertEqual(target.read_text(), 'user key')

    def test_no_google_login_in_noninteractive_mode(self):
        with patch.object(setup, 'run', return_value=result(text='')), patch.object(setup.sys.stdin, 'isatty', return_value=False):
            with self.assertRaisesRegex(setup.SetupError, 'rclone config'):
                setup.ensure_remote('ajou_drive', 'id')

    def test_interactive_login_must_create_correct_remote(self):
        with patch.object(setup, 'run', side_effect=[result(text=''), result(), result(text='other:')]), patch.object(setup.sys.stdin, 'isatty', return_value=True):
            with self.assertRaisesRegex(setup.SetupError, 'remote가 없습니다'):
                setup.ensure_remote('ajou_drive', 'id')

    def test_user_checkpoint_mismatch_is_rejected_before_system_changes(self):
        target = self.root / 'weights.pth'
        target.write_bytes(b'user model')
        args = argparse.Namespace(dry_run=False, checkpoint_path=target)
        with patch.object(setup, 'ubuntu_check'), patch.object(setup.os, 'geteuid', return_value=1000), patch.object(setup.shutil, 'which', return_value='/bin/tool'), patch.object(setup, 'run', return_value=result()), patch.object(setup, 'system_setup') as system:
            with self.assertRaisesRegex(setup.SetupError, '기존 가중치'):
                setup.setup(args)
            system.assert_not_called()
        self.assertEqual(target.read_bytes(), b'user model')

    def test_stream_image_checks_both_processes(self):
        process = unittest.mock.Mock()
        process.stdout = io.BytesIO(b'image tar')
        process.wait.return_value = 1
        process.poll.return_value = 1
        with patch.object(setup.subprocess, 'Popen', return_value=process), patch.object(setup.subprocess, 'run', return_value=result()):
            with self.assertRaisesRegex(setup.SetupError, 'docker load 실패'):
                setup.load_image('archive.zst', ['docker'])
        self.assertTrue(process.stdout.closed)

    def test_run_allows_binary_output_file_for_zstd(self):
        output = self.root / 'output'
        with output.open('wb') as handle:
            setup.run(['/usr/bin/printf', 'tar bytes'], stdout=handle)
        self.assertEqual(output.read_bytes(), b'tar bytes')


if __name__ == '__main__':
    unittest.main()
