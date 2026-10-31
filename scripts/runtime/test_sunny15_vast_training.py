import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).parent))
import sunny15_vast_training as launcher


class Sunny15LauncherTests(unittest.TestCase):
    def setup_config(self, root):
        launcher.initialize_configs(root / 'configs')
        return launcher.load_config(root / 'configs/cosine.json')

    def fixture(self, root):
        c = self.setup_config(root)
        for key in ('source', 'data', 'work'):
            c[key] = str(root / key)
            Path(c[key]).mkdir()
        c['initializer'] = str(root / 'init.pth'); Path(c['initializer']).write_bytes(b'init')
        c['config'] = 'config.py'; (Path(c['source']) / c['config']).write_text('compile_camera=False')
        data=Path(c['data']); c['expected_scenes']=2
        rows=[dict(id='frame'+str(i),status='accepted_candidate',capture_root='/original/frame'+str(i)) for i in range(2)]
        ledger=data/'provenance/original_selected_ledger.jsonl';ledger.parent.mkdir()
        ledger.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        files={str(ledger.relative_to(data)):dict(sha256=launcher.sha(ledger))}
        for row in rows:
            path=data/'data'/row['id']/'metadata.json';path.parent.mkdir(parents=True)
            path.write_text(json.dumps(dict(fresh_environment_provenance=dict(environment='sunny15'))))
            files[str(path.relative_to(data))]=dict(sha256=launcher.sha(path))
        manifest=dict(scenes=2,files=files,source_ledger_sha256=launcher.sha(ledger),provenance={
            row['id']:dict(original_capture_root=row['capture_root']) for row in rows})
        (data/'manifest.json').write_text(json.dumps(manifest))
        c['dataset_manifest_sha256'] = launcher.sha(Path(c['data']) / 'manifest.json')
        c['support_hashes'] = {'config.py': launcher.sha(Path(c['source']) / 'config.py')}
        identity = dict(epochs=8, batch=4, workers=4, accumulate=1, lr=1e-4,
            seed=261018, val_enabled=True, profile='512x1056', model=dict(compile_camera=False),
            training_label_policy='animal_to_pedestrian_v1', wandb_group=launcher.GROUP,
            classes=['vehicle', 'pedestrian', 'obstacle'],
            dataset_manifest_sha256=c['dataset_manifest_sha256'],
            source_hashes=c['support_hashes'], initial_checkpoint_sha256=launcher.sha(c['initializer']),
            config_sha256=launcher.sha(Path(c['source']) / c['config']),
            param_schedulers=[dict(type='LinearLR'), dict(type='CosineAnnealingLR')])
        (Path(c['work']) / 'run_manifest.json').write_text(json.dumps(dict(identity=identity, contract_sha256='test')))
        return c

    def test_configs_are_separate_and_not_bound(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); first=self.setup_config(root)
            second=launcher.load_config(root/'configs/restarts.json')
            self.assertIsNone(first['dataset_manifest_sha256'])
            self.assertNotEqual(first['work'],second['work'])
            self.assertNotEqual(first['relay_local_port'],second['relay_local_port'])
            self.assertNotIn('base5_init_full',first['work'])
            with self.assertRaises(FileExistsError): launcher.initialize_configs(root/'configs')

    def test_no_bound_data_means_no_start(self):
        with tempfile.TemporaryDirectory() as temp:
            c=self.setup_config(Path(temp))
            with self.assertRaisesRegex(ValueError,'SHA not bound'): launcher.data_gate(c)

    def test_commands_prepare_without_run_and_resume_is_explicit(self):
        c={'source':'/source','data':'/newdata','work':'/work/cosine','variant':'cosine'}
        self.assertEqual(launcher.command(c)[:2],['bash','/source/scripts/runtime/prepare_sunny15_ab.sh'])
        self.assertNotIn('--run',launcher.command(c))
        self.assertEqual(launcher.command(c,resume=True,run=True)[-3:],['--resume','auto','--run'])

    def test_prepared_contract_accepted(self):
        with tempfile.TemporaryDirectory() as temp:
            c=self.fixture(Path(temp)); self.assertEqual(launcher.prepared_manifest(c)['contract_sha256'],'test')

    def test_old_data_or_changed_manifest_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c=self.fixture(Path(temp)); (Path(c['data'])/'manifest.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'manifest SHA differs'): launcher.prepared_manifest(c)

    def test_wrong_schedule_or_compile_is_rejected(self):
        for mutate,reason in ((lambda i:i['model'].update(compile_camera=True),'compile'),
                              (lambda i:i['param_schedulers'][0].update(type='MoraiWarmupCosineRestarts'),'schedule')):
            with tempfile.TemporaryDirectory() as temp:
                c=self.fixture(Path(temp)); p=Path(c['work'])/'run_manifest.json'; m=json.loads(p.read_text())
                mutate(m['identity']); p.write_text(json.dumps(m))
                with self.assertRaisesRegex(ValueError,reason): launcher.prepared_manifest(c)

    def test_started_run_needs_resume_and_in_scope_checkpoint(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); c=self.fixture(root); work=Path(c['work'])
            (work/'fresh_start.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'already started'): launcher.prepared_manifest(c)
            with self.assertRaisesRegex(ValueError,'No completed'): launcher.prepared_manifest(c,True)
            (root/'epoch_1.pth').write_bytes(b'outside'); (work/'last_checkpoint').write_text(str(root/'epoch_1.pth'))
            with self.assertRaisesRegex(ValueError,'outside'): launcher.prepared_manifest(c,True)

    def test_helper_drift_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c=self.fixture(Path(temp)); c['support_hashes']['config.py']='changed'
            with self.assertRaisesRegex(ValueError,'helper changed'): launcher.prepared_manifest(c)

    def test_status_starts_no_job_and_reports_unbound_data(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(launcher,'emit') as emit, \
             patch.object(launcher.subprocess,'run') as run:
            c=self.setup_config(Path(temp)); launcher.remote_status(c)
            run.assert_not_called(); self.assertFalse(emit.call_args.kwargs['ready'])
            self.assertFalse(emit.call_args.kwargs['data_and_contract_ready'])

    def test_local_check_uses_only_remote_check_and_scoped_drive_probe(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(launcher.transport,'logging_relay') as relay, \
             patch.object(launcher.transport,'ssh') as ssh, patch.object(launcher,'check_drive') as drive, \
             patch.object(launcher.subprocess,'run') as run:
            c=self.setup_config(Path(temp)); run.return_value.returncode=1; ssh.return_value.stdout='{}\n'
            launcher.local_action(c,'check',resume=True)
            self.assertIn('--remote-check --resume',ssh.call_args.args[1])
            self.assertNotIn('--remote-start',ssh.call_args.args[1]); drive.assert_called_once_with(c)

    def test_small_probe_cannot_qualify_full_backup(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); c=self.setup_config(root)
            c['backup_qualification_report']=str(root/'backup.json')
            path=Path(c['backup_qualification_report'])
            path.write_text(json.dumps(dict(valid=True,write_readback=True)))
            with self.assertRaisesRegex(ValueError,'full resumable'): launcher.backup_gate(c)
            report=dict(valid=True,status=dict(destination=c['destination'],errors=[],uploaded={
                'epoch_2.pth':dict(resume_capable=True,bytes=519331269,sha256='a'*64)}))
            path.write_text(json.dumps(report)); launcher.backup_gate(c)
            report['status']['destination']='backup:old-run';path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,'destination'): launcher.backup_gate(c)

    def test_backup_worker_waits_before_first_checkpoint(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(launcher.transport,'logging_relay') as relay, \
             patch.object(launcher,'backup_once',return_value=0) as backup, \
             patch.object(launcher.transport,'remote_status') as status, patch.object(launcher.time,'sleep') as sleep:
            c=self.setup_config(Path(temp)); c['local_logs']=str(Path(temp)/'watcher')
            relay.return_value.__enter__.return_value.poll.return_value=None
            status.side_effect=[dict(state='waiting',pending_checkpoints=[],backup_errors=[]),
                                dict(state='finished',pending_checkpoints=[],backup_errors=[])]
            self.assertEqual(launcher.backup_worker(c),0)
            self.assertEqual(backup.call_count,2);sleep.assert_called_once_with(120)

    def test_sunny13_metadata_is_not_sunny15_even_with_matching_package_sha(self):
        with tempfile.TemporaryDirectory() as temp:
            c=self.fixture(Path(temp)); data=Path(c['data']); path=data/'data/frame0/metadata.json'
            path.write_text(json.dumps(dict(fresh_environment_provenance=dict(environment='sunny13'))))
            manifest=json.loads((data/'manifest.json').read_text())
            manifest['files']['data/frame0/metadata.json']['sha256']=launcher.sha(path)
            (data/'manifest.json').write_text(json.dumps(manifest));c['dataset_manifest_sha256']=launcher.sha(data/'manifest.json')
            with self.assertRaisesRegex(ValueError,'not the new Sunny15'): launcher.data_gate(c)

    def test_provenance_ledger_mutation_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c=self.fixture(Path(temp)); path=Path(c['data'])/'provenance/original_selected_ledger.jsonl'
            path.write_text('{}\n')
            with self.assertRaisesRegex(ValueError,'provenance inventory SHA'): launcher.data_gate(c)

    def test_worker_preflight_failure_is_recorded_without_training(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(launcher,'prepared_manifest',side_effect=ValueError), \
             patch.object(launcher.subprocess,'run') as run:
            root=Path(temp); c=self.fixture(root)
            self.assertEqual(launcher.remote_worker(c),1);run.assert_not_called()
            status=json.loads((Path(c['work'])/'oneclick_status.json').read_text())
            self.assertEqual(status['state'],'failed');self.assertEqual(status['error_type'],'ValueError')


if __name__=='__main__': unittest.main()
