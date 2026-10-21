"""CPU-only scope/transport checks; never start or contact remote training."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from scripts.runtime import base5_performance_vast_training as m


class PerformanceLauncherTests(unittest.TestCase):
    def test_json_manifest_tuple_list_normalization_preserves_executable_config_order(self):
        cfg = dict(model=dict(heads={'z_branch': (1, 2), 'a_branch': [3, 4]}),
                   train_pipeline=[dict(scale=(.9, 1.1))])
        before = copy.deepcopy(cfg)
        candidate_identity = dict(model=cfg['model'], train_pipeline=cfg['train_pipeline'])
        parent_identity = json.loads(json.dumps(candidate_identity))
        self.assertNotEqual(candidate_identity, parent_identity)
        normalized = m.json_identity(candidate_identity)
        self.assertEqual(normalized, parent_identity)
        self.assertEqual(m.performance.contract_sha256(normalized),
                         m.performance.contract_sha256(candidate_identity))
        self.assertEqual(cfg, before)
        self.assertEqual(list(cfg['model']['heads']), ['z_branch', 'a_branch'])
        self.assertIsInstance(cfg['model']['heads']['z_branch'], tuple)

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.parent = self.root / 'parent'
        self.child = self.root / 'child'
        self.runtime = self.root / 'runtime'
        for path in (self.parent, self.child, self.runtime):
            path.mkdir()
        self.config_path = self.root / 'config.json'
        paths = dict(source=self.root / 'source', data=self.root / 'index.json',
            work=self.child, initializer=self.root / 'initializer.pth',
            remote_script=self.runtime / 'base5_performance_vast_training.py',
            remote_config=self.runtime / 'config.json',
            parent_manifest=self.parent / 'run_manifest.json',
            parent_checkpoint=self.parent / 'epoch_1.pth', parent_work=self.parent,
            adapter_path=self.runtime / 'full_data_prepare.py',
            qualified_metadata_script=self.runtime / 'benchmark_base5_full_runtime.py',
            resume_helper_script=self.runtime / 'base5_performance_resume.py',
            owned_storage_script=self.runtime / 'base5_metric_owned_storage.py')
        self.c = {key: str(value) for key, value in paths.items()}
        self.c.update(schema=m.SCHEMA, epochs=12, expected_scenes=30750,
            variant='cosine', compile_camera=False, parent_contract_sha256=m.PARENT_CONTRACT,
            launcher_sha256='a' * 64, resume_helper_sha256='b' * 64,
            owned_storage_sha256='c' * 64, qualified_metadata_sha256='d' * 64,
            adapter_sha256='e' * 64, parent_manifest_sha256='f' * 64,
            parent_checkpoint_sha256='1' * 64, dataset_manifest_sha256='2' * 64,
            drive_backup_route=m.BACKUP_ROUTE,
            destination='backup:model/bevfusion/base5/full30750_speed_v1/checkpoint_backups',
            train_session='base5-full30750-speed-training',
            backup_session='base5-full30750-speed-backup',
            all_train_sessions=['base5-full30750-speed-training',
                                'base5-full30750-val-logging-training'],
            local_repo=str(self.root), local_script=str(Path(m.__file__)),
            config='configs/original.py', epochs_user_approved=False,
            actual_training_start_approved=False,
            start_authority=str(self.runtime / 'USER_START_AUTHORITY.json'),
            runtime_policy=dict(train_workers=8, val_workers=8, val_batch_size=2,
                train_prefetch_factor=2, val_prefetch_factor=2,
                train_pin_memory=True, val_pin_memory=True, val_fp16=False,
                owned_storage_extension_sha256='c' * 64))
        self.c['runtime_extensions'] = {self.c[path]: self.c[checksum] for path, checksum in (
            ('remote_script', 'launcher_sha256'), ('resume_helper_script', 'resume_helper_sha256'),
            ('owned_storage_script', 'owned_storage_sha256'),
            ('qualified_metadata_script', 'qualified_metadata_sha256'),
            ('adapter_path', 'adapter_sha256'))}
        self.c['runtime_hashes'] = {}
        for name in ('sunny15_vast_training.py', 'sunny15_backup_qualification.py',
                     'base5_vast_oneclick.py', 'base5_wandb_relay.py', 'full_data_oneclick.py'):
            path = self.runtime / name
            path.write_text('# unchanged original transport fixture: ' + name + '\n')
            self.c['runtime_hashes'][name] = m.sha(path)
            self.c['runtime_extensions'][str(path)] = m.sha(path)

    def load(self, c=None):
        self.config_path.write_text(json.dumps(self.c if c is None else c))
        return m.load_config(self.config_path)

    def authorize(self, c=None):
        c = self.c if c is None else c
        c.update(epochs_user_approved=True, actual_training_start_approved=True)
        authority = dict(authorized=True, user_approved_training_start=True, epochs=12,
            parent_contract_sha256=c['parent_contract_sha256'],
            parent_checkpoint_sha256=c['parent_checkpoint_sha256'],
            dataset_manifest_sha256=c['dataset_manifest_sha256'],
            runtime_extensions=copy.deepcopy(c['runtime_extensions']),
            runtime_policy=copy.deepcopy(c['runtime_policy']))
        Path(c['start_authority']).write_text(json.dumps(authority))
        return authority

    def prepared_fixture(self):
        initializer = Path(self.c['initializer'])
        initializer.write_bytes(b'original initializer fixture')
        config = Path(self.c['source']) / self.c['config']
        config.parent.mkdir(parents=True, exist_ok=True)
        config.write_text('# original model configuration fixture\n')
        identity = dict(source_hashes={}, initial_checkpoint_sha256=m.sha(initializer),
                        config_sha256=m.sha(config), immutable_training_field='unchanged')
        parent = dict(identity=identity)
        transition = dict(parent_checkpoint_sha256=self.c['parent_checkpoint_sha256'],
            parent_checkpoint=self.c['parent_checkpoint'],
            parent_contract_sha256=self.c['parent_contract_sha256'], boundary_epoch=1,
            boundary_iteration=6534, runtime_policy=copy.deepcopy(self.c['runtime_policy']),
            runtime_extensions=copy.deepcopy(self.c['runtime_extensions']),
            helper_sha256=self.c['resume_helper_sha256'])
        child_identity = dict(identity, performance_runtime=transition)
        child = dict(identity=child_identity, parent_run=copy.deepcopy(transition),
                     contract_sha256=m.performance.contract_sha256(child_identity))
        (self.child / 'run_manifest.json').write_text(json.dumps(child))
        return parent, child

    def test_accepts_only_explicit_twelve_epoch_full_data_continuation(self):
        self.assertEqual(self.load(), self.c)
        for changed in ({'schema': 'base5_full_data_launcher_v1'}, {'epochs': 8},
                        {'expected_scenes': 30749}, {'variant': 'restarts'},
                        {'compile_camera': True}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                self.load(dict(self.c, **changed))

    def test_parent_and_child_scope_must_be_separate_and_absolute(self):
        for changed in ({'work': self.c['parent_work']},
                {'parent_checkpoint': str(self.root / 'other.pth')},
                {'parent_manifest': str(self.root / 'manifest.json')},
                {'qualified_metadata_script': 'relative.py'},
                {'owned_storage_script': '/tmp/runtime/../escape.py'},
                {'parent_contract_sha256': '0' * 64}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                self.load(dict(self.c, **changed))

    def test_every_effective_helper_must_be_bound(self):
        for path in self.c['runtime_extensions']:
            changed = copy.deepcopy(self.c)
            del changed['runtime_extensions'][path]
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, 'explicitly bound'):
                self.load(changed)
        changed = dict(self.c, owned_storage_sha256='0' * 64)
        with self.assertRaisesRegex(ValueError, 'owned-storage'):
            self.load(changed)

    def test_successor_cannot_reuse_parent_sessions_or_drive_scope(self):
        for changed in ({'train_session': 'base5-full30750-val-logging-training'},
                        {'backup_session': 'base5-full30750-val-logging-backup'},
                        {'destination': 'backup:model/bevfusion/base5/full30750_261021/backups'},
                        {'drive_backup_route': 'relay'}, {'all_train_sessions': []}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                self.load(dict(self.c, **changed))

    def test_unqualified_arbitrary_training_extension_is_rejected(self):
        changed = dict(self.c, training_extension=dict(path='/tmp/other.py', sha256='a' * 64,
            qualification_receipt='/tmp/qualification.json'))
        with self.assertRaisesRegex(ValueError, 'discarded-heatmap'):
            self.load(changed)

    def test_prepare_command_never_executes_or_uses_legacy_five_k_entry(self):
        command = m.command(self.c)
        self.assertEqual(command, ['python', '-u', self.c['remote_script'], '--config',
                                  self.c['remote_config'], '--prepare-runtime'])
        self.assertNotIn('--execute', command)
        self.assertNotIn('--resume', command)
        self.assertNotIn('--run', command)

    def test_child_resume_is_explicit_and_not_automatic_parent_resume(self):
        command = m.command(self.c, resume=True)
        self.assertEqual(command[-1], '--resume')
        self.assertNotIn('auto', command)
        self.assertNotIn('--execute', command)

    def test_execute_requires_separate_exact_start_authority(self):
        with self.assertRaisesRegex(ValueError, 'user-authorized'):
            m.command(self.c, run=True)
        self.c.update(epochs_user_approved=True, actual_training_start_approved=True)
        with self.assertRaises(FileNotFoundError):
            m.command(self.c, run=True)
        self.authorize()
        self.assertEqual(m.command(self.c, run=True)[-1], '--execute')
        self.assertEqual(m.command(self.c, run=True, resume=True)[-2:], ['--resume', '--execute'])

    def test_authority_rejects_parent_budget_data_policy_or_extension_changes(self):
        authority = self.authorize()
        for key, value in (('authorized', False), ('user_approved_training_start', False),
                ('epochs', 8), ('parent_contract_sha256', '0' * 64),
                ('parent_checkpoint_sha256', '0' * 64),
                ('dataset_manifest_sha256', '0' * 64), ('runtime_extensions', {}),
                ('runtime_policy', dict(self.c['runtime_policy'], val_batch_size=4))):
            Path(self.c['start_authority']).write_text(json.dumps(dict(authority, **{key: value})))
            with self.subTest(key=key), self.assertRaises(ValueError):
                m.training_authority(self.c)

    def test_data_gate_rejects_modified_parent_before_scanning_payload(self):
        Path(self.c['parent_manifest']).write_text('{}')
        with patch.object(m.performance, 'validate_runtime_extensions') as extension:
            with self.assertRaisesRegex(ValueError, 'Parent manifest changed'):
                m.data_gate(self.c)
            extension.assert_not_called()

    def test_prepared_manifest_retains_all_original_training_identity(self):
        parent, child = self.prepared_fixture()
        with patch.object(m, 'data_gate', return_value=parent):
            self.assertEqual(m.prepared_manifest(self.c), child)
            changed = copy.deepcopy(child)
            changed['identity']['immutable_training_field'] = 'changed'
            changed['contract_sha256'] = m.performance.contract_sha256(changed['identity'])
            (self.child / 'run_manifest.json').write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, 'original training identity'):
                m.prepared_manifest(self.c)

    def test_prepared_manifest_enforces_complete_epoch_and_runtime_policy(self):
        parent, child = self.prepared_fixture()
        for key, value in (('boundary_epoch', 2), ('boundary_iteration', 6533),
                ('runtime_policy', dict(self.c['runtime_policy'], val_batch_size=4)),
                ('runtime_extensions', {}), ('helper_sha256', '0' * 64)):
            changed = copy.deepcopy(child)
            changed['identity']['performance_runtime'][key] = value
            changed['parent_run'] = copy.deepcopy(changed['identity']['performance_runtime'])
            changed['contract_sha256'] = m.performance.contract_sha256(changed['identity'])
            (self.child / 'run_manifest.json').write_text(json.dumps(changed))
            with self.subTest(key=key), patch.object(m, 'data_gate', return_value=parent):
                with self.assertRaisesRegex(ValueError, 'reviewed configuration'):
                    m.prepared_manifest(self.c)

    def test_status_can_read_started_scope_without_permitting_fresh_start(self):
        parent, child = self.prepared_fixture()
        (self.child / 'performance_resume_start.json').write_text('{}')
        with patch.object(m, 'data_gate', return_value=parent):
            self.assertEqual(m.prepared_manifest(self.c, status_only=True), child)
            with self.assertRaisesRegex(ValueError, 'already started'):
                m.prepared_manifest(self.c)

    def test_explicit_child_resume_never_accepts_parent_checkpoint_pointer(self):
        parent, child = self.prepared_fixture()
        Path(self.c['parent_checkpoint']).write_bytes(b'parent checkpoint fixture')
        pointer = self.child / 'last_checkpoint'
        pointer.write_text(self.c['parent_checkpoint'])
        with patch.object(m, 'data_gate', return_value=parent):
            with self.assertRaisesRegex(ValueError, 'strict child-scope'):
                m.prepared_manifest(self.c, resume=True)
            checkpoint = self.child / 'epoch_2.pth'
            checkpoint.write_bytes(b'child checkpoint fixture')
            pointer.write_text(str(checkpoint))
            self.assertEqual(m.prepared_manifest(self.c, resume=True), child)

    def test_training_extension_requires_actual_equivalence_receipt(self):
        path = self.runtime / 'base5_skip_discarded_heatmap.py'
        receipt = self.runtime / 'qualification.json'
        path.write_text('# trusted fixture, not imported\n')
        row = dict(targets_exact=True, losses_exact=True, finite=True, RNG_exact=True,
            gradient=dict(exact=True, max_abs=0.), baseline_losses={'loss': 1.},
            candidate_losses={'loss': 1.})
        proof = dict(complete=True, valid=True, expected_fixture_batches=3,
            contract_sha256=self.c['parent_contract_sha256'],
            checkpoint_sha256=self.c['parent_checkpoint_sha256'],
            extension=dict(version='base5_skip_discarded_heatmap_v1', script_sha256=m.sha(path)),
            no_optimizer_step=True, no_checkpoint_write=True, model_original_unmodified=True,
            compile_camera=False, fixtures=[copy.deepcopy(row) for _ in range(3)])
        receipt.write_text(json.dumps(proof))
        self.c['training_extension'] = dict(path=str(path), sha256=m.sha(path),
            qualification_receipt=str(receipt), qualification_sha256=m.sha(receipt))
        m._training_extension_gate(self.c)
        for flag in ('targets_exact', 'losses_exact', 'finite', 'RNG_exact', 'gradient'):
            changed = copy.deepcopy(proof)
            changed['fixtures'][1][flag] = dict(exact=False, max_abs=1.) if flag == 'gradient' else False
            receipt.write_text(json.dumps(changed))
            self.c['training_extension']['qualification_sha256'] = m.sha(receipt)
            with self.subTest(flag=flag), self.assertRaisesRegex(ValueError, 'equivalence proof'):
                m._training_extension_gate(self.c)

    def test_default_cli_routes_only_read_only_status(self):
        legacy = SimpleNamespace(local_action=Mock())
        with patch.object(m, 'load_config', return_value=self.c), \
                patch.object(m, 'wire_transport', return_value=legacy):
            self.assertEqual(m.main(['--config', str(self.config_path)]), 0)
        legacy.local_action.assert_called_once_with(self.c, 'status', False)

    def test_remote_start_and_worker_enforce_authority_before_process_launch(self):
        for mode in ('--remote-start', '--remote-worker'):
            legacy = SimpleNamespace(remote_start=Mock(), remote_worker=Mock())
            with self.subTest(mode=mode), \
                    patch.object(m, 'load_config', return_value=self.c), \
                    patch.object(m, 'wire_transport', return_value=legacy):
                with self.assertRaisesRegex(ValueError, 'user-authorized'):
                    m.main(['--config', str(self.config_path), mode])
            legacy.remote_start.assert_not_called()
            legacy.remote_worker.assert_not_called()

    def test_execute_flag_cannot_skip_the_internal_preparation_gate(self):
        with patch.object(m, 'load_config', return_value=self.c), \
                patch.object(m, 'wire_transport') as wired:
            with self.assertRaisesRegex(ValueError, 'internal'):
                m.main(['--config', str(self.config_path), '--run', '--execute'])
            wired.assert_not_called()

    def test_transport_injection_preserves_original_lifecycle_and_gates(self):
        names = ('remote_check', 'remote_start', 'remote_worker', 'local_action',
                 'backup_worker', 'check_drive')
        legacy = SimpleNamespace(**{name: Mock(name=name) for name in names},
            transport=SimpleNamespace(), backup_qualification=SimpleNamespace(),
            backup_gate=Mock(return_value={}))
        untouched = {name: getattr(legacy, name) for name in names}
        full = SimpleNamespace(preparation_timeouts=Mock(), backup_once=Mock())
        with patch.object(m, 'load_module', return_value=full):
            self.assertIs(m.wire_transport(self.c, legacy), legacy)
        for name, value in untouched.items():
            self.assertIs(getattr(legacy, name), value)
        self.assertIs(legacy.command, m.command)
        self.assertIs(legacy.prepared_manifest, m.prepared_manifest)
        self.assertIs(legacy.data_gate, m.data_gate)
        self.assertEqual(legacy.__file__, self.c['local_script'])
        full.preparation_timeouts.assert_called_once_with(legacy.transport, self.c['remote_script'])
        legacy.backup_once(self.c)
        full.backup_once.assert_called_once_with(self.c, legacy)

    def test_reused_backup_gate_requires_qualified_direct_route(self):
        proof = dict(qualification_kind='reused_full_backup_qualification',
            network_route=m.BACKUP_ROUTE, new_route_small_write_readback=True,
            new_full_checkpoint_throughput_verified=False)
        original = Mock(return_value=proof)
        legacy = SimpleNamespace(transport=SimpleNamespace(),
            backup_qualification=SimpleNamespace(), backup_gate=original)
        full = SimpleNamespace(preparation_timeouts=Mock(), backup_once=Mock())
        with patch.object(m, 'load_module', return_value=full):
            m.wire_transport(self.c, legacy)
        self.assertIs(legacy.backup_gate(self.c), proof)
        for field, value in (('network_route', 'relay'),
                            ('new_route_small_write_readback', False),
                            ('new_full_checkpoint_throughput_verified', True)):
            original.return_value = dict(proof, **{field: value})
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'direct-route proof'):
                legacy.backup_gate(self.c)


if __name__ == '__main__':
    unittest.main()
