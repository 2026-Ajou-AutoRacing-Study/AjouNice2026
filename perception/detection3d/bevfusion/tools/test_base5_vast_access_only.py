import argparse
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import shutil

import backup_base5_vast_access_only as backup


class AccessOnlyMigrationTest(unittest.TestCase):
    def readback_fixture(self, root):
        from base5_checkpoint_parts import split_checkpoint
        source=root/'payload.pth';source.write_bytes(b'full-checkpoint-bytes')
        manifest=split_checkpoint(source,root/'parts_8',8)
        args=argparse.Namespace(destination='backup:model/bevfusion/base5/checkpoint_backups',
            rclone_remote='private',root_folder_id='authorized-root',timeout=1200)
        request=dict(request_id='probe',manifest=manifest,
            prefix=args.destination+'/run/'+manifest['sha256'])
        return args,request

    def test_local_full_readback_and_per_part_sha_are_checked(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);args,request=self.readback_fixture(root)
            def copied(command,**kwargs):
                shutil.copytree(root/'parts_8',Path(command[3]),dirs_exist_ok=True)
            with patch.object(backup.subprocess,'run',side_effect=copied) as call:
                reply=backup.verify_local_readback(args,request)
            self.assertTrue(reply['verified'])
            self.assertEqual(reply['sha256'],request['manifest']['sha256'])
            self.assertTrue(call.call_args.args[0][2].startswith('private:model/'))

    def test_local_readback_corruption_is_not_acknowledged(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);args,request=self.readback_fixture(root)
            def corrupt(command,**kwargs):
                shutil.copytree(root/'parts_8',Path(command[3]),dirs_exist_ok=True)
                (Path(command[3])/'part0000.bin').write_bytes(b'corrupt!')
            with patch.object(backup.subprocess,'run',side_effect=corrupt):
                with self.assertRaisesRegex(ValueError,'checksum'):
                    backup.verify_local_readback(args,request)

    def test_local_readback_cannot_access_another_drive_prefix(self):
        with tempfile.TemporaryDirectory() as temporary:
            args,request=self.readback_fixture(Path(temporary))
            request['prefix']='backup:unrelated/'+request['manifest']['sha256']
            with patch.object(backup.subprocess,'run') as call:
                with self.assertRaisesRegex(ValueError,'authorized'):
                    backup.verify_local_readback(args,request)
            call.assert_not_called()

    def test_remote_paths_are_explicit_and_token_is_redacted(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / '.config/rclone/rclone.conf'
            config.parent.mkdir(parents=True)
            config.write_text('[test]\nclient_id = dummy\nclient_secret = dummy\n'
                              'token = {"refresh_token":"dummy"}\n')
            response = Mock()
            response.__enter__ = Mock(return_value=io.StringIO(
                json.dumps(dict(access_token='test-access-value', expires_in=3600))))
            response.__exit__ = Mock(return_value=False)
            child = Mock()
            captured = []
            child.stdin.write.side_effect = captured.append
            child.stdout = iter(['test-access-value\n'])
            child.wait.return_value = 0
            args = argparse.Namespace(rclone_remote='test', work_dir='/workspace/run',
                destination='backup:authorized', root_folder_id='dummy', timeout=180,
                watch=False, rclone_binary='/usr/bin/rclone',
                backup_script='/workspace/repo/backup.py', log=root/'result.jsonl',
                host='test-host', port=12345)
            with patch.object(Path, 'home', return_value=root), \
                 patch.object(backup.urllib.request, 'urlopen', return_value=response), \
                 patch.object(backup.subprocess, 'Popen', return_value=child), \
                 patch('builtins.print'):
                self.assertEqual(backup.poll(args), 0)
            payload = json.loads(captured[0])
            self.assertEqual(payload['rclone_binary'], '/usr/bin/rclone')
            self.assertEqual(payload['backup_script'], '/workspace/repo/backup.py')
            self.assertNotIn('refresh_token', payload)
            self.assertNotIn('client_secret', payload)
            self.assertEqual(args.log.read_text(), '[REDACTED]\n')


if __name__ == '__main__':
    unittest.main()
