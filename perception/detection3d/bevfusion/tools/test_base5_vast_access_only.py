import argparse
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import backup_base5_vast_access_only as backup


class AccessOnlyMigrationTest(unittest.TestCase):
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
