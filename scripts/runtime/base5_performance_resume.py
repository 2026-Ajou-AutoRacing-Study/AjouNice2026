"""Explicit, state-preserving runtime-policy continuation of the frozen run.

Build the candidate configuration with the ORIGINAL source config and the
immutable full-data prepare adapter first.  Never rebuild a model from its
sorted resolved-config dump: optimizer parameter order is significant.

This helper does not start a process or rewrite a checkpoint.  A caller must
publish its returned manifest in a fresh work directory and use the returned
configuration with Base5AtomicRunner.  Parent and successor W&B runs remain
distinct, with an explicit parent checkpoint/contract binding.
"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path


VERSION = 'base5_performance_resume_v1'
HOOK = 'Base5PerformanceResumeEvidenceHook'
POLICY_KEYS = frozenset((
    'train_workers', 'val_workers', 'val_batch_size',
    'train_prefetch_factor', 'val_prefetch_factor',
    'train_pin_memory', 'val_pin_memory', 'val_fp16',
    'owned_storage_extension_sha256'))


def require(value, message):
    if not value:
        raise ValueError(message)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 ** 2), b''):
            digest.update(block)
    return digest.hexdigest()


def contract_sha256(identity):
    """Use the unchanged direct runner's exact identity serialization."""
    return hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()


def validate_runtime_extensions(value):
    extensions = dict(value)
    require(bool(extensions), 'Explicit effective runtime extension map is required')
    for name, checksum in extensions.items():
        path = Path(name)
        require(path.is_absolute() and '..' not in path.parts and path.suffix == '.py'
                and isinstance(checksum, str) and len(checksum) == 64
                and all(character in '0123456789abcdef' for character in checksum),
                'Runtime extension paths and SHA256 values must be explicit')
        require(sha256(path) == checksum, 'Bound runtime extension changed: ' + name)
    require(str(Path(__file__).resolve()) in extensions,
            'Resume helper must appear in its effective runtime extension map')
    return extensions


def normalize_policy(value):
    policy = dict(value)
    policy.setdefault('val_fp16', False)
    require(set(policy) == POLICY_KEYS, 'Only explicit loader/validation runtime policy is allowed')
    for key in ('train_workers', 'val_workers'):
        require(type(policy[key]) is int and 0 <= policy[key] <= 32,
                'Worker count must be an integer in [0, 32]')
    require(type(policy['val_batch_size']) is int and 1 <= policy['val_batch_size'] <= 8,
            'Validation batch size must be an integer in [1, 8]')
    for key in ('train_prefetch_factor', 'val_prefetch_factor'):
        require(type(policy[key]) is int and 1 <= policy[key] <= 8,
                'Prefetch factor must be an integer in [1, 8]')
    for key in ('train_pin_memory', 'val_pin_memory', 'val_fp16'):
        require(type(policy[key]) is bool, 'Pin-memory and validation precision flags must be bool')
    checksum = policy['owned_storage_extension_sha256']
    require(isinstance(checksum, str) and len(checksum) == 64
            and all(character in '0123456789abcdef' for character in checksum),
            'An explicit reviewed owned-storage extension SHA256 is required')
    return policy


def apply_runtime_policy(cfg, value):
    """Change only allowlisted runtime fields; train batch/sampler stay fixed."""
    policy = normalize_policy(value)
    require(cfg['train_dataloader']['batch_size'] == 4,
            'Training batch remains four; changing it requires a new training experiment')
    for split in ('train', 'val'):
        loader = cfg[split + '_dataloader']
        workers = policy[split + '_workers']
        loader['num_workers'] = workers
        loader['persistent_workers'] = workers > 0
        loader['pin_memory'] = policy[split + '_pin_memory']
        if workers:
            loader['prefetch_factor'] = policy[split + '_prefetch_factor']
        else:
            loader.pop('prefetch_factor', None)
    cfg['val_dataloader']['batch_size'] = policy['val_batch_size']
    require(cfg.get('val_cfg') is not None, 'An explicit validation loop is required')
    require(cfg['val_cfg'].get('type', 'ValLoop') == 'ValLoop',
            'Only the existing MMEngine validation loop is qualified')
    cfg['val_cfg']['fp16'] = policy['val_fp16']
    return policy


def assert_runtime_policy(cfg, value):
    """Fail closed if a caller edits runtime fields after qualification."""
    policy = normalize_policy(value)
    expected = copy.deepcopy(cfg)
    apply_runtime_policy(expected, policy)
    for split in ('train', 'val'):
        require(cfg[split + '_dataloader'] == expected[split + '_dataloader'],
                'Resolved loader differs from bound runtime policy')
    require(cfg['val_cfg'] == expected['val_cfg'],
            'Resolved validation precision differs from bound runtime policy')


def assert_original_training_configuration(cfg, identity):
    """Check actual builder output, not merely its accompanying manifest."""
    for config_key, identity_key in (('model', 'model'),
                                      ('optim_wrapper', 'optimizer_wrapper'),
                                      ('param_scheduler', 'param_schedulers')):
        require(json.loads(json.dumps(cfg[config_key], allow_nan=False))
                == json.loads(json.dumps(identity[identity_key], allow_nan=False)),
                'Actual configuration changed original ' + config_key)
    require(cfg['train_cfg'].get('type') == 'EpochBasedTrainLoop'
            and cfg['train_cfg'].get('max_epochs') == identity['epochs']
            and cfg['train_cfg'].get('val_interval') == 1,
            'Original epoch budget and validation cadence must remain unchanged')
    require(cfg['train_dataloader']['batch_size'] == identity['batch']
            and cfg['optim_wrapper'].get('accumulative_counts', 1) == identity['accumulate']
            and not cfg['train_dataloader'].get('drop_last', False),
            'Original effective batch and complete-epoch update count must remain unchanged')
    if 'train_pipeline' in identity:
        dataset = cfg['train_dataloader']['dataset']
        require(dataset.get('type') == 'ConcatDataset'
                and all(json.loads(json.dumps(child['pipeline'], allow_nan=False))
                        == json.loads(json.dumps(identity['train_pipeline'], allow_nan=False))
                        for child in dataset['datasets']),
                'Original training augmentation pipelines must remain unchanged')
    if 'val_cohort_mapping_sha256' in identity:
        mapping = cfg['val_evaluator']['base5_val_logging']['token_to_cohort']
        digest = hashlib.sha256(json.dumps(mapping, sort_keys=True,
            separators=(',', ':')).encode()).hexdigest()
        require(digest == identity['val_cohort_mapping_sha256'],
                'Actual validation cohort map changed')


def _bind_hook_and_wandb(cfg, *, parent_contract, successor_contract, transition):
    found = [hook for hook in cfg['custom_hooks']
             if hook.get('type') == 'MoraiResumeEvidenceHook']
    require(len(found) == 1 and found[0]['contract_sha256'] == parent_contract,
            'Exactly one original strict resume-evidence hook is required')
    cfg['custom_hooks'] = [hook for hook in cfg['custom_hooks']
                           if hook.get('type') != 'MoraiResumeEvidenceHook']
    cfg['custom_hooks'].append(dict(type=HOOK, contract_sha256=successor_contract,
                                   transition=copy.deepcopy(transition)))
    if not any(hook.get('type') == 'Base5MetricOwnedStorageReceiptHook'
               for hook in cfg['custom_hooks']):
        cfg['custom_hooks'].append(dict(type='Base5MetricOwnedStorageReceiptHook'))
    child_work = Path(cfg['work_dir'])
    run_id = child_work.name + '-' + successor_contract[:12]
    for backends in (cfg['visualizer']['vis_backends'], cfg['vis_backends']):
        for backend in backends:
            if backend['type'] == 'MoraiWandbVisBackend':
                backend['init_kwargs'].update(id=run_id, name=child_work.name,
                    resume='allow', allow_val_change=True)


def bind_successor(cfg, candidate_manifest, parent_manifest, *, parent_work_dir,
                   parent_checkpoint, parent_checkpoint_sha256, policy,
                   runtime_extensions, boundary_epoch=1, boundary_iteration=6534):
    """Bind a fresh continuation without weakening the original resume guard.

    Before this call candidate_manifest must describe the exact parent
    model/data/optimizer/scheduler/source configuration.  Runtime fields are
    overlaid only after every original identity field is compared.  The
    returned manifest is new; neither supplied manifest nor checkpoint changes.
    """
    require(candidate_manifest['identity'] == parent_manifest['identity'],
            'Candidate changed an original model/data/optimizer/schedule/source identity')
    parent_contract = contract_sha256(parent_manifest['identity'])
    require(parent_manifest['contract_sha256'] == parent_contract
            and candidate_manifest['contract_sha256'] == parent_contract,
            'Parent/candidate manifest contract is invalid')
    identity = parent_manifest['identity']
    assert_original_training_configuration(cfg, identity)
    require(identity.get('batch') == 4 and identity.get('accumulate') == 1
            and identity.get('iterations_per_epoch') == 6534
            and identity.get('train_count') == 26136 and identity.get('val_count') == 4614,
            'This continuation is qualified only for the existing full-data batch-four run')
    require(type(boundary_epoch) is int and boundary_epoch >= 1
            and type(boundary_iteration) is int
            and boundary_iteration == boundary_epoch * identity['iterations_per_epoch'],
            'Only a complete epoch boundary is supported')
    parent_work = Path(parent_work_dir).resolve(strict=True)
    checkpoint = Path(parent_checkpoint).resolve(strict=True)
    child_work = Path(cfg['work_dir']).resolve(strict=True)
    require(checkpoint.is_file() and checkpoint.parent == parent_work,
            'Parent checkpoint must be a trusted local file in the parent work directory')
    require(child_work != parent_work and not child_work.is_relative_to(parent_work)
            and not parent_work.is_relative_to(child_work),
            'A separate, non-nested successor work directory is required')
    require(sha256(checkpoint) == parent_checkpoint_sha256,
            'Parent checkpoint content differs from its approved SHA256')
    require(not any(child_work.glob('*.pth')) and not (child_work / 'last_checkpoint').exists()
            and not (child_work / 'fresh_start.json').exists(),
            'Successor already started; use its ordinary strict resume')
    policy = normalize_policy(policy)
    extensions = validate_runtime_extensions(runtime_extensions)
    transition = dict(version=VERSION, parent_contract_sha256=parent_contract,
        parent_checkpoint=str(checkpoint), parent_checkpoint_sha256=parent_checkpoint_sha256,
        parent_work_dir=str(parent_work), boundary_epoch=boundary_epoch,
        boundary_iteration=boundary_iteration, runtime_policy=policy,
        runtime_extensions=extensions,
        helper_sha256=sha256(__file__),
        scope='epoch boundary; full optimizer/scheduler/scaler/main RNG; '
              'worker RNG/prefetch and bit-exact future replay are not guaranteed')
    manifest = copy.deepcopy(candidate_manifest)
    # Preserve every original field under its original name.  The effective
    # loader configuration is explicitly distinct from the inherited identity.
    manifest['identity']['performance_runtime'] = copy.deepcopy(transition)
    successor_contract = contract_sha256(manifest['identity'])
    manifest['contract_sha256'] = successor_contract
    manifest['resume_scope'] = transition['scope']
    manifest['parent_run'] = copy.deepcopy(transition)
    apply_runtime_policy(cfg, policy)
    cfg['resume'] = True
    cfg['load_from'] = str(checkpoint)
    _bind_hook_and_wandb(cfg, parent_contract=parent_contract,
                        successor_contract=successor_contract, transition=transition)
    return manifest


def bind_successor_resume(cfg, candidate_manifest, successor_manifest, *, checkpoint):
    """Ordinary strict resume inside the already-established child scope.

    Rebuild candidate cfg using the same immutable original adapter in a
    separate prepare-only staging directory, then set cfg.work_dir to the child
    directory.  The parent builder cannot read a child-scope manifest as if its
    own original contract were unchanged.
    """
    manifest = copy.deepcopy(successor_manifest)
    transition = manifest['identity']['performance_runtime']
    original_identity = {key: value for key, value in manifest['identity'].items()
                         if key != 'performance_runtime'}
    require(candidate_manifest['identity'] == original_identity
            and candidate_manifest['contract_sha256'] == transition['parent_contract_sha256']
            and contract_sha256(original_identity) == transition['parent_contract_sha256'],
            'Original builder no longer matches this successor parent identity')
    assert_original_training_configuration(cfg, original_identity)
    require(manifest['contract_sha256'] == contract_sha256(manifest['identity'])
            and manifest['parent_run'] == transition
            and transition['helper_sha256'] == sha256(__file__),
            'Successor identity or helper changed')
    validate_runtime_extensions(transition['runtime_extensions'])
    work = Path(cfg['work_dir']).resolve(strict=True)
    path = Path(checkpoint).resolve(strict=True)
    require(path.is_file() and path.parent == work,
            'Strict child resume checkpoint must be inside the child work directory')
    apply_runtime_policy(cfg, transition['runtime_policy'])
    cfg['resume'] = True
    cfg['load_from'] = str(path)
    _bind_hook_and_wandb(cfg, parent_contract=transition['parent_contract_sha256'],
                        successor_contract=manifest['contract_sha256'], transition=transition)
    return manifest


def validate_parent_checkpoint(checkpoint, transition):
    require(checkpoint.get('base5_contract_sha256') == transition['parent_contract_sha256'],
            'Checkpoint is not from the approved parent contract')
    meta = checkpoint.get('meta', {})
    require(meta.get('epoch') == transition['boundary_epoch']
            and meta.get('iter') == transition['boundary_iteration'],
            'Checkpoint is not the approved complete epoch boundary')
    for key in ('state_dict', 'optimizer', 'param_schedulers', 'message_hub',
                'base5_rng', 'base5_optimizer_parameter_names'):
        require(key in checkpoint, 'Parent checkpoint is missing full resumable state: ' + key)
    require('loss_scaler' in checkpoint['optimizer'],
            'Parent AMP optimizer checkpoint is missing loss-scaler state')


def install_owned_storage(policy):
    """Import only the explicitly checksummed sibling runtime extension."""
    policy = normalize_policy(policy)
    path = Path(__file__).with_name('base5_metric_owned_storage.py')
    require(sha256(path) == policy['owned_storage_extension_sha256'],
            'Reviewed metric-owned-storage extension changed')
    spec = importlib.util.spec_from_file_location('_qualified_base5_metric_owned_storage', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.install()


def install_transition_hook():
    """Register the opt-in hook after the unchanged checkpoint plugin imports."""
    from mmdet3d.registry import HOOKS
    from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import MoraiResumeEvidenceHook
    if HOOKS.get(HOOK) is not None:
        return HOOKS.get(HOOK)

    @HOOKS.register_module()
    class Base5PerformanceResumeEvidenceHook(MoraiResumeEvidenceHook):
        def __init__(self, contract_sha256, transition):
            super().__init__(contract_sha256)
            self.transition = copy.deepcopy(transition)
            self.accepted_parent = False
            install_owned_storage(self.transition['runtime_policy'])

        def after_load_checkpoint(self, runner, checkpoint):
            require(runner.cfg.get('resume') is True,
                    'Performance continuation requires full-state resume, not weights-only load')
            assert_runtime_policy(runner.cfg, self.transition['runtime_policy'])
            require(sha256(__file__) == self.transition['helper_sha256'],
                    'Performance transition helper changed')
            validate_runtime_extensions(self.transition['runtime_extensions'])
            if checkpoint.get('base5_contract_sha256') == self.transition['parent_contract_sha256']:
                path = Path(runner.cfg['load_from']).resolve(strict=True)
                require(str(path) == self.transition['parent_checkpoint']
                        and sha256(path) == self.transition['parent_checkpoint_sha256'],
                        'Only the exact approved parent checkpoint may cross contracts')
                validate_parent_checkpoint(checkpoint, self.transition)
                # The unchanged hook still validates optimizer names/order and
                # completeness, and restores the SAME saved RNG.  A shallow
                # wrapper changes only the contract presented to that hook;
                # checkpoint bytes and the supplied dictionary are untouched.
                accepted = dict(checkpoint, base5_contract_sha256=self.contract_sha256)
                super().after_load_checkpoint(runner, accepted)
                self.accepted_parent = True
            else:
                super().after_load_checkpoint(runner, checkpoint)

        def before_save_checkpoint(self, runner, checkpoint):
            super().before_save_checkpoint(runner, checkpoint)
            checkpoint.setdefault('meta', {})['base5_performance_resume'] = copy.deepcopy(self.transition)

        def before_train(self, runner):
            super().before_train(runner)
            if self.accepted_parent:
                require(runner.epoch == self.transition['boundary_epoch']
                        and runner.iter == self.transition['boundary_iteration'],
                        'Runner did not restore the approved epoch/iteration')
            row = dict(version=VERSION, parent_accepted=self.accepted_parent,
                contract_sha256=self.contract_sha256, epoch=runner.epoch,
                iteration=runner.iter, transition=self.transition)
            path = Path(runner.work_dir) / 'performance_resume_start.json'
            path.write_text(json.dumps(row, indent=2) + '\n')

    return Base5PerformanceResumeEvidenceHook
