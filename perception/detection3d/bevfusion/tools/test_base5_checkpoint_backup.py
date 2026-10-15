import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import io
import shutil

import torch

spec = importlib.util.spec_from_file_location('backup', Path(__file__).with_name('backup_base5_checkpoints.py'))
backup = importlib.util.module_from_spec(spec); spec.loader.exec_module(backup)


class BackupTest(unittest.TestCase):
    def fixture(self, root):
        run = root/'run1'; run.mkdir()
        (run/'run_manifest.json').write_text(json.dumps(dict(contract_sha256='c1')))
        (run/'resolved_train.py').write_text('model={}\n')
        checkpoint = dict(state_dict={'w': torch.ones(2)}, optimizer={'loss_scaler': {}},
            param_schedulers=[], message_hub={}, base5_rng={}, meta=dict(epoch=1, iter=4),
            base5_contract_sha256='c1')
        torch.save(checkpoint, run/'epoch_1.pth')
        (run/'epoch_2.pth.partial-dead').write_bytes(b'partial')
        return run

    def test_complete_idempotent_and_partial_ignored(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); run = self.fixture(root); dest = root/'external'
            state = backup.backup_once(run, str(dest), {})
            self.assertEqual(state['errors'], [])
            self.assertEqual(len(state['uploaded']), 1)
            latest = json.loads((dest/'run1/LATEST.json').read_text())
            self.assertEqual(latest['iteration'], 4)
            checkpoint = Path(latest['location'])/'epoch_1.pth'
            self.assertEqual(backup.sha256(checkpoint), latest['sha256'])
            self.assertTrue((Path(latest['location'])/'COMPLETE.json').exists())
            self.assertTrue((run/'epoch_1.pth').exists())
            with patch.object(backup, 'copy_verified', side_effect=AssertionError('No repeated upload')):
                self.assertEqual(backup.backup_once(run, str(dest), state)['errors'], [])

    def test_failure_has_no_latest_pointer_and_retry(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); run=self.fixture(root); dest=root/'external'
            with patch.object(backup, 'copy_verified', side_effect=RuntimeError('upload failed')):
                state=backup.backup_once(run,str(dest),{})
            self.assertTrue(state['errors']); self.assertFalse((dest/'run1/LATEST.json').exists())
            self.assertFalse(list(run.glob('.backup_stage_*')))
            self.assertEqual(backup.backup_once(run,str(dest),state)['errors'], [])

    def test_weights_only_best_is_backed_up_but_not_advertised_as_resume(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); run=self.fixture(root); dest=root/'external'
            original=torch.load(run/'epoch_1.pth',weights_only=False)
            best={key:original[key] for key in ('state_dict','meta','base5_contract_sha256')}
            torch.save(best,run/'best_val_center_mAP_epoch_1.pth')
            state=backup.backup_once(run,str(dest),{})
            self.assertEqual(state['errors'],[])
            self.assertEqual(len(state['uploaded']),2)
            self.assertFalse(state['uploaded']['best_val_center_mAP_epoch_1.pth']['resume_capable'])
            latest=json.loads((dest/'run1/LATEST.json').read_text())
            self.assertEqual(latest['name'],'epoch_1.pth')
            self.assertTrue(latest['resume_capable'])

    def test_changed_epoch_snapshot_is_reuploaded_without_losing_old_backup(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); run=self.fixture(root); dest=root/'external'
            state=backup.backup_once(run,str(dest),{})
            old=json.loads((dest/'run1/LATEST.json').read_text())
            payload=torch.load(run/'epoch_1.pth',weights_only=False)
            payload['message_hub']={'best_score':.5}
            torch.save(payload,run/'epoch_1.pth')
            state=backup.backup_once(run,str(dest),state)
            new=json.loads((dest/'run1/LATEST.json').read_text())
            self.assertNotEqual(old['sha256'],new['sha256'])
            self.assertTrue((Path(old['location'])/'COMPLETE.json').exists())
            self.assertEqual(state['errors'],[])

    def test_dns_error_reason_does_not_expose_raw_stderr(self):
        failure=SimpleNamespace(stdout=iter(['lookup www.googleapis.com: i/o timeout; secret-do-not-log']),wait=lambda:5)
        with tempfile.TemporaryDirectory() as name:
            source=Path(name)/'payload';source.write_bytes(b'payload')
            with patch.object(backup.subprocess,'Popen',return_value=failure):
                with self.assertRaises(RuntimeError) as caught:
                    backup.copy_verified(source,'remote:checkpoint.pth')
        self.assertIn('DNS_lookup_timeout',caught.exception.safe_reasons)
        self.assertNotIn('secret-do-not-log',str(caught.exception))

    def test_corrupt_remote_readback_is_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            source=Path(name)/'payload';source.write_bytes(b'correct payload')
            process=SimpleNamespace(stdout=io.BytesIO(b'wrong payload'),wait=lambda:0)
            with patch.object(backup,'remote_copy'), \
                    patch.object(backup.subprocess,'Popen',return_value=process):
                with self.assertRaisesRegex(ValueError,'checksum failed'):
                    backup.copy_verified(source,'remote:checkpoint.pth')


    def test_stats_are_live_and_raw_log_is_not_exposed(self):
        with tempfile.TemporaryDirectory() as name:
            source=Path(name)/'payload';source.write_bytes(b'correct')
            process=SimpleNamespace(stdout=iter([
                json.dumps(dict(stats=dict(bytes=3,speed=10,totalBytes=7),msg='secret-url'))+'\n',
                'secret-access-token\n']),wait=lambda:0)
            output=io.StringIO()
            with patch.object(backup.subprocess,'Popen',return_value=process) as popen:
                with patch('sys.stdout',output):
                    backup.remote_copy(source,'remote:payload',[])
            events=[json.loads(line) for line in output.getvalue().splitlines()]
            self.assertEqual(events[1]['bytes'],3)
            self.assertEqual(events[1]['speed'],10)
            self.assertNotIn('secret',output.getvalue())
            self.assertIn('8M',popen.call_args.args[0])

    def test_multipart_backup_and_restore(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); run=self.fixture(root); cloud=root/'cloud'
            def translate(value):
                return cloud/str(value).split(':',1)[1] if ':' in str(value) else Path(value)
            def directory_copy(source,destination,options,**kwargs):
                shutil.copytree(translate(source),translate(destination),dirs_exist_ok=True)
            def file_copy(source,destination,options):
                target=translate(destination);target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(source,target)
            with patch.object(backup,'remote_copy',side_effect=directory_copy), \
                    patch.object(backup,'copy_verified',side_effect=file_copy):
                state=backup.backup_once(run,'test:backup',{},part_bytes=512)
            self.assertEqual(state['errors'],[])
            latest=json.loads((cloud/'backup/run1/LATEST.json').read_text())
            folder=translate(latest['location'])
            manifest=json.loads((folder/'PARTS.json').read_text())
            backup.verify_parts(folder/manifest['parts_dir'],manifest,root/'restored.pth')
            self.assertEqual(backup.sha256(root/'restored.pth'),backup.sha256(run/'epoch_1.pth'))
            self.assertEqual(latest['storage'],'base5_checkpoint_parts_v1')
            self.assertTrue((folder/'COMPLETE.json').exists())

    def test_restored_best_weights_need_explicit_nonresume_permission(self):
        with tempfile.TemporaryDirectory() as name:
            path=Path(name)/'restored.pth'
            torch.save(dict(state_dict={'w':torch.ones(2)},meta={'epoch':1,'iter':4},
                            base5_contract_sha256='c1'),path)
            with self.assertRaisesRegex(ValueError,'full resumable'):
                backup.checkpoint_complete(path,'c1')
            self.assertFalse(backup.checkpoint_complete(path,'c1',allow_weights_only=True)['resume_capable'])

    def test_multipart_corruption_never_commits_marker(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); run=self.fixture(root)
            def directory_copy(source,destination,options,**kwargs):
                if kwargs.get('phase')=='readback':
                    (destination/'part0000.bin').write_bytes(b'corrupt')
            with patch.object(backup,'remote_copy',side_effect=directory_copy), \
                    patch.object(backup,'copy_verified') as marker:
                state=backup.backup_once(run,'test:backup',{},part_bytes=512)
            self.assertEqual(len(state['errors']),1)
            self.assertFalse(state['uploaded'])
            marker.assert_not_called()

    def test_bridge_readback_refuses_wrong_acknowledgment(self):
        with patch.object(backup, 'emit') as emitted, \
             patch.object(backup.sys, 'stdin', io.StringIO(json.dumps(
                 dict(request_id='wrong',verified=True,sha256='wrong',bytes=1))+'\n')):
            with self.assertRaisesRegex(ValueError,'Local Drive readback'):
                backup.local_bridge_readback('test:checkpoint',dict(sha256='expected',bytes=1))
        self.assertEqual(emitted.call_args.args[0],'local_readback_request')

    def test_latest_epoch_is_protected_first_and_progress_is_durable(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);run=self.fixture(root);dest=root/'external'
            original=torch.load(run/'epoch_1.pth',weights_only=False)
            original['meta']={'epoch':2,'iter':8}
            torch.save(original,run/'epoch_2.pth')
            seen=[]
            real=backup.copy_verified
            def recording(source,destination,options):
                if Path(source).suffix=='.pth':seen.append(Path(source).name)
                return real(source,destination,options)
            with patch.object(backup,'copy_verified',side_effect=recording):
                state=backup.backup_once(run,str(dest),{})
            self.assertEqual(seen[:2],['epoch_2.pth','epoch_1.pth'])
            self.assertEqual(json.loads((run/'backup_status.json').read_text())['uploaded'],state['uploaded'])


if __name__ == '__main__': unittest.main()
