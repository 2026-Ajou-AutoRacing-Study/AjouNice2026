import json
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from planning_scenario_validator import sharing, runner, recording_options as options
from planning_scenario_validator.topic_contract import PLANNER_TIME_PATTERNS
import re


class AnalysisSharingTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.output = self.root / 'local'
        self.folder = self.output / 'success/case'
        self.folder.mkdir(parents=True)
        self.cfg = {'output': self.output, 'name': 'campaign',
                    'upload': {'enabled': True, 'profile': 'bag', 'rclone_remote': str(self.root / 'remote')},
                    'recording': {'bag_enabled': True, 'analysis_mode': True}}
        (self.folder / 'drive.bag').write_bytes(b'bag fixture')
        (self.folder / 'screen.mp4').write_bytes(b'video fixture')
        (self.folder / 'result.json').write_text(json.dumps({'status': 'PASS', 'access_token': 'secret123'}))
        (self.output / 'campaign_config.yaml').write_text('password: secret123\ncampaign_name: fixture\n')
        (self.output / 'stack_logs').mkdir()
        (self.output / 'stack_logs/commands.json').write_text(json.dumps({'cmd': ['env', 'API_KEY=secret123']}))
        (self.output / 'stack_logs/planning.log').write_text('Authorization: Bearer secret123\n"password": "secret123"\n')
        (self.output / 'stack_logs/access_token.json').write_text('secret123')

    def test_bundle_preserves_logs_commands_sanitizes_secrets_and_respects_media(self):
        root, folder, manifest = sharing.build_bundle(self.cfg, self.folder)
        self.assertTrue((folder / 'drive.bag').is_file())
        self.assertFalse((folder / 'screen.mp4').exists())
        self.assertTrue((folder / 'stack_logs/commands.json').is_file())
        self.assertFalse((folder / 'stack_logs/access_token.json').exists())
        for path in folder.rglob('*'):
            if path.is_file() and path.suffix != '.bag':
                self.assertNotIn('secret123', path.read_text())
        self.assertIn('drive.bag', manifest['files'])
        self.assertIn('secret123', (self.output / 'campaign_config.yaml').read_text())

    def test_listing_requires_actual_size_and_hash(self):
        _, folder, _ = sharing.build_bundle(self.cfg, self.folder)
        listing = [{'Path': str(p.relative_to(folder)), 'Size': p.stat().st_size,
                    'Hashes': {'MD5': sharing.hashes(p)['md5']}}
                   for p in folder.rglob('*') if p.is_file()]
        self.assertEqual(sharing.verify_listing(folder, listing), [])
        listing[0]['Hashes'] = {}
        self.assertTrue(sharing.verify_listing(folder, listing))

    def test_manual_analysis_share_also_prepares_logs_and_manifest(self):
        self.cfg['upload']['enabled'] = False
        runner._upload(self.cfg, self.folder, {'results': {}, 'upload_errors': []}, SimpleNamespace(name='case'))
        audit = json.loads((self.folder / 'sharing_audit.json').read_text())
        self.assertEqual(audit['status'], 'READY_FOR_MANUAL_SHARE')
        self.assertFalse(audit['verified'])
        payload = Path(audit['payload'])
        self.assertTrue((payload / 'stack_logs/commands.json').is_file())
        self.assertTrue((payload / 'share_manifest.json').is_file())

    def test_analysis_warnings_and_nested_planner_timings(self):
        self.assertTrue(sharing.sharing_warnings(self.cfg['recording'], {'profile': 'video'}))
        self.assertFalse(sharing.sharing_warnings(self.cfg['recording'], {'profile': 'bag'}))
        for topic in ('/planning/path/debug/processing_time_ms',
                      '/planning/motion/debug/obstacle_stop/processing_time_ms'):
            self.assertTrue(any(re.fullmatch(pattern, topic) for pattern in PLANNER_TIME_PATTERNS))
        self.assertFalse(any(re.fullmatch(p, '/sensor/camera/front/compressed') for p in PLANNER_TIME_PATTERNS))

    @unittest.skipUnless(shutil.which('rclone'), 'requires local rclone')
    def test_real_local_upload_verifies_all_shared_files_without_drive_credentials(self):
        state = {'results': {}, 'upload_errors': []}
        runner._write_state(self.cfg, state)
        runner._upload(self.cfg, self.folder, state, SimpleNamespace(name='case'))
        audit = json.loads((self.folder / 'sharing_audit.json').read_text())
        self.assertTrue(audit['verified'], audit)
        remote = self.root / 'remote/campaign/success/case'
        self.assertTrue((remote / 'stack_logs/planning.log').is_file())
        self.assertTrue((remote / 'share_manifest.json').is_file())
        self.assertTrue((remote / 'sharing_audit.json').is_file())
        self.assertFalse((remote / 'screen.mp4').exists())
        self.assertEqual(state['upload_errors'], [])


if __name__ == '__main__':
    unittest.main()
