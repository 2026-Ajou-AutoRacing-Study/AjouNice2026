import copy
import json
from pathlib import Path
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.runtime import base5_performance_resume as m


def policy():
    return dict(train_workers=8, val_workers=8, val_batch_size=2,
                train_prefetch_factor=2, val_prefetch_factor=2,
                train_pin_memory=True, val_pin_memory=True,
                owned_storage_extension_sha256=m.sha256(
                    Path(m.__file__).with_name('base5_metric_owned_storage.py')))


def identity():
    return dict(batch=4, accumulate=1, workers=4, iterations_per_epoch=6534,
                train_count=26136, val_count=4614, epochs=12, lr=1e-4,
                model=dict(type='original_ordered_model'),
                optimizer_wrapper=dict(type='AmpOptimWrapper'),
                param_schedulers=[dict(type='LinearLR'), dict(type='CosineAnnealingLR')],
                dataset_manifest_sha256='a' * 64, source_hashes={'original.py': 'b' * 64})


def manifest():
    value = identity()
    return dict(identity=value, contract_sha256=m.contract_sha256(value),
                effective_batch=4, optimizer_updates_per_epoch=6534,
                paths=dict(data='/unchanged/index.json'))


def configuration(work, contract):
    return dict(work_dir=str(work), model=dict(type='original_ordered_model'),
        train_dataloader=dict(batch_size=4, num_workers=4, persistent_workers=True,
            sampler=dict(type='DefaultSampler', shuffle=True),
            dataset=dict(type='ConcatDataset', datasets=['unchanged'])),
        val_dataloader=dict(batch_size=1, num_workers=4, persistent_workers=True,
            sampler=dict(type='DefaultSampler', shuffle=False),
            dataset=dict(type='ConcatDataset', datasets=['unchanged'])),
        val_cfg=dict(type='ValLoop'), optim_wrapper=dict(type='AmpOptimWrapper'),
        train_cfg=dict(type='EpochBasedTrainLoop', max_epochs=12, val_interval=1),
        param_scheduler=[dict(type='LinearLR'), dict(type='CosineAnnealingLR')],
        custom_hooks=[dict(type='MoraiResumeEvidenceHook', contract_sha256=contract),
                      dict(type='Base5ValLoggingReceiptHook')],
        visualizer=dict(vis_backends=[dict(type='MoraiWandbVisBackend', init_kwargs={})]),
        vis_backends=[dict(type='MoraiWandbVisBackend', init_kwargs={})])


class PerformanceResumeTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.parent = self.root / 'parent'
        self.child = self.root / 'child'
        self.parent.mkdir()
        self.child.mkdir()
        self.checkpoint = self.parent / 'epoch_1.pth'
        self.checkpoint.write_bytes(b'unchanged trusted checkpoint fixture')
        self.parent_manifest = manifest()
        self.candidate = copy.deepcopy(self.parent_manifest)
        self.cfg = configuration(self.child, self.parent_manifest['contract_sha256'])

    def bind(self, **kwargs):
        options = dict(parent_work_dir=self.parent, parent_checkpoint=self.checkpoint,
                       parent_checkpoint_sha256=m.sha256(self.checkpoint), policy=policy(),
                       runtime_extensions={str(Path(m.__file__).resolve()): m.sha256(m.__file__)})
        options.update(kwargs)
        return m.bind_successor(self.cfg, self.candidate, self.parent_manifest, **options)

    def test_policy_is_explicit_and_validation_amp_defaults_off(self):
        normalized = m.normalize_policy(policy())
        self.assertFalse(normalized['val_fp16'])
        for key, value in [('train_batch_size', 8), ('arbitrary_override', {})]:
            with self.assertRaisesRegex(ValueError, 'Only explicit'):
                m.normalize_policy(dict(policy(), **{key: value}))
        for bad in (True, -1, 33):
            with self.assertRaises(ValueError):
                m.normalize_policy(dict(policy(), train_workers=bad))
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            m.normalize_policy(dict(policy(), owned_storage_extension_sha256='not-approved'))

    def test_runtime_overlay_preserves_batch_sampler_model_and_data(self):
        original = copy.deepcopy(self.cfg)
        m.apply_runtime_policy(self.cfg, policy())
        self.assertEqual(self.cfg['train_dataloader']['batch_size'], 4)
        for split in ('train', 'val'):
            for key in ('sampler', 'dataset'):
                self.assertEqual(self.cfg[split + '_dataloader'][key],
                                 original[split + '_dataloader'][key])
        self.assertEqual(self.cfg['model'], original['model'])
        self.assertEqual(self.cfg['optim_wrapper'], original['optim_wrapper'])
        m.assert_runtime_policy(self.cfg, policy())
        self.cfg['val_dataloader']['num_workers'] = 3
        with self.assertRaisesRegex(ValueError, 'loader differs'):
            m.assert_runtime_policy(self.cfg, policy())

    def test_no_worker_policy_omits_prefetch_and_persistence(self):
        value = dict(policy(), train_workers=0, val_workers=0)
        m.apply_runtime_policy(self.cfg, value)
        for split in ('train', 'val'):
            self.assertNotIn('prefetch_factor', self.cfg[split + '_dataloader'])
            self.assertFalse(self.cfg[split + '_dataloader']['persistent_workers'])

    def test_explicit_transition_preserves_parent_bytes_and_all_original_identity(self):
        before = copy.deepcopy(self.parent_manifest)
        payload = self.checkpoint.read_bytes()
        result = self.bind()
        self.assertEqual(self.parent_manifest, before)
        self.assertEqual(self.candidate, before)
        self.assertEqual(self.checkpoint.read_bytes(), payload)
        for key, value in before['identity'].items():
            self.assertEqual(result['identity'][key], value)
        self.assertNotEqual(result['contract_sha256'], before['contract_sha256'])
        self.assertEqual(result['contract_sha256'], m.contract_sha256(result['identity']))
        self.assertTrue(self.cfg['resume'])
        self.assertEqual(self.cfg['load_from'], str(self.checkpoint))
        hooks = {hook['type']: hook for hook in self.cfg['custom_hooks']}
        self.assertNotIn('MoraiResumeEvidenceHook', hooks)
        self.assertIn('Base5MetricOwnedStorageReceiptHook', hooks)
        self.assertEqual(hooks[m.HOOK]['contract_sha256'], result['contract_sha256'])
        run_id = self.cfg['visualizer']['vis_backends'][0]['init_kwargs']['id']
        self.assertEqual(run_id, self.child.name + '-' + result['contract_sha256'][:12])

    def test_changing_any_original_training_identity_is_rejected(self):
        for key in self.candidate['identity']:
            changed = copy.deepcopy(self.candidate)
            changed['identity'][key] = 'a different value'
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'original model/data'):
                m.bind_successor(self.cfg, changed, self.parent_manifest,
                    parent_work_dir=self.parent, parent_checkpoint=self.checkpoint,
                    parent_checkpoint_sha256=m.sha256(self.checkpoint), policy=policy(),
                    runtime_extensions={str(Path(m.__file__).resolve()): m.sha256(m.__file__)})

    def test_actual_configuration_cannot_differ_from_its_original_manifest(self):
        for key, replacement in (('model', {'type': 'changed'}),
                ('optim_wrapper', {'type': 'OptimWrapper'}),
                ('param_scheduler', [{'type': 'different'}])):
            before = copy.deepcopy(self.cfg)
            self.cfg[key] = replacement
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'Actual configuration'):
                self.bind()
            self.cfg = before
        self.cfg['train_cfg']['max_epochs'] = 8
        with self.assertRaisesRegex(ValueError, 'Original epoch budget'):
            self.bind()

    def test_original_tuple_cfg_matches_json_manifest_without_reordering_or_mutation(self):
        self.cfg['model'] = dict(z_branch=(1, 2), a_branch=(3, 4))
        self.parent_manifest['identity']['model'] = dict(z_branch=[1, 2], a_branch=[3, 4])
        self.parent_manifest['contract_sha256'] = m.contract_sha256(self.parent_manifest['identity'])
        self.candidate = copy.deepcopy(self.parent_manifest)
        self.cfg['custom_hooks'][0]['contract_sha256'] = self.parent_manifest['contract_sha256']
        before = copy.deepcopy(self.cfg['model'])
        self.bind()
        self.assertEqual(self.cfg['model'], before)
        self.assertEqual(list(self.cfg['model']), ['z_branch', 'a_branch'])
        self.assertIsInstance(self.cfg['model']['z_branch'], tuple)

    def test_invalid_manifest_and_changed_checkpoint_are_rejected(self):
        self.candidate['contract_sha256'] = 'c' * 64
        with self.assertRaisesRegex(ValueError, 'contract is invalid'):
            self.bind()
        self.candidate = copy.deepcopy(self.parent_manifest)
        with self.assertRaisesRegex(ValueError, 'checkpoint content'):
            self.bind(parent_checkpoint_sha256='c' * 64)

    def test_only_complete_boundary_is_allowed(self):
        with self.assertRaisesRegex(ValueError, 'complete epoch'):
            self.bind(boundary_iteration=6533)

    def test_started_or_nested_successor_is_rejected(self):
        (self.child / 'last_checkpoint').write_text(str(self.child / 'epoch_2.pth'))
        with self.assertRaisesRegex(ValueError, 'already started'):
            self.bind()
        (self.child / 'last_checkpoint').unlink()
        nested = self.parent / 'child'
        nested.mkdir()
        self.cfg['work_dir'] = str(nested)
        with self.assertRaisesRegex(ValueError, 'non-nested'):
            self.bind()

    def test_full_checkpoint_requires_same_epoch_and_all_state(self):
        result = self.bind()
        transition = result['parent_run']
        checkpoint = dict(base5_contract_sha256=transition['parent_contract_sha256'],
            meta=dict(epoch=1, iter=6534), state_dict={}, optimizer={'loss_scaler': {}},
            param_schedulers=[], message_hub={}, base5_rng={}, base5_optimizer_parameter_names=[])
        m.validate_parent_checkpoint(checkpoint, transition)
        for key in ('optimizer', 'param_schedulers', 'message_hub', 'base5_rng',
                    'base5_optimizer_parameter_names'):
            changed = copy.deepcopy(checkpoint)
            changed.pop(key)
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'missing full'):
                m.validate_parent_checkpoint(changed, transition)
        changed = copy.deepcopy(checkpoint)
        changed['optimizer'] = {}
        with self.assertRaisesRegex(ValueError, 'loss-scaler'):
            m.validate_parent_checkpoint(changed, transition)
        changed = copy.deepcopy(checkpoint)
        changed['meta']['iter'] = 6533
        with self.assertRaisesRegex(ValueError, 'complete epoch'):
            m.validate_parent_checkpoint(changed, transition)

    def test_later_child_resume_is_strict_and_keeps_the_same_contract(self):
        result = self.bind()
        child_checkpoint = self.child / 'epoch_2.pth'
        child_checkpoint.write_bytes(b'child fixture')
        fresh_cfg = configuration(self.child, self.parent_manifest['contract_sha256'])
        actual = m.bind_successor_resume(fresh_cfg, self.candidate, result,
                                        checkpoint=child_checkpoint)
        self.assertEqual(actual, result)
        self.assertEqual(fresh_cfg['load_from'], str(child_checkpoint))
        with self.assertRaisesRegex(ValueError, 'inside the child'):
            m.bind_successor_resume(configuration(self.child, self.parent_manifest['contract_sha256']),
                                    self.candidate, result, checkpoint=self.checkpoint)

    def test_hook_calls_unchanged_strict_validator_without_mutating_parent(self):
        result = self.bind()
        transition = result['parent_run']

        class Registry:
            registry = {}

            def get(self, name):
                return self.registry.get(name)

            def register_module(self):
                def register(cls):
                    self.registry[cls.__name__] = cls
                    return cls
                return register

        class OriginalStrictHook:
            def __init__(self, contract):
                self.contract_sha256 = contract
                self.pending_rng = None

            def after_load_checkpoint(self, runner, checkpoint):
                if checkpoint['base5_contract_sha256'] != self.contract_sha256:
                    raise ValueError('strict child contract mismatch')
                self.pending_rng = checkpoint['base5_rng']

        registry = ModuleType('mmdet3d.registry')
        registry.HOOKS = Registry()
        original = ModuleType('perception.detection3d.bevfusion.plugin.morai_remote_checkpoint')
        original.MoraiResumeEvidenceHook = OriginalStrictHook
        with patch.dict('sys.modules', {'mmdet3d.registry': registry,
                'perception.detection3d.bevfusion.plugin.morai_remote_checkpoint': original}), \
                patch.object(m, 'install_owned_storage'):
            cls = m.install_transition_hook()
            hook = cls(result['contract_sha256'], transition)
            checkpoint = dict(base5_contract_sha256=transition['parent_contract_sha256'],
                meta=dict(epoch=1, iter=6534), state_dict={}, optimizer={'loss_scaler': {}},
                param_schedulers=[], message_hub={}, base5_rng={'torch': 'saved'},
                base5_optimizer_parameter_names=[])
            original_checkpoint = copy.deepcopy(checkpoint)
            runner = SimpleNamespace(cfg=self.cfg)
            hook.after_load_checkpoint(runner, checkpoint)
            self.assertEqual(checkpoint, original_checkpoint)
            self.assertEqual(hook.pending_rng, {'torch': 'saved'})
            self.assertTrue(hook.accepted_parent)
            child = dict(checkpoint, base5_contract_sha256=result['contract_sha256'])
            cls(result['contract_sha256'], transition).after_load_checkpoint(runner, child)
            with self.assertRaisesRegex(ValueError, 'strict child contract'):
                cls(result['contract_sha256'], transition).after_load_checkpoint(
                    runner, dict(child, base5_contract_sha256='f' * 64))
            runner.cfg['resume'] = False
            with self.assertRaisesRegex(ValueError, 'full-state resume'):
                cls(result['contract_sha256'], transition).after_load_checkpoint(runner, checkpoint)


if __name__ == '__main__':
    unittest.main()
