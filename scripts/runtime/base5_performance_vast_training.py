#!/usr/bin/env python3
"""Audited full-state continuation using the existing Vast transport gates.

Status/prepare are non-training.  Only --run with a separately bound start
authority may launch; --resume means a later checkpoint inside the child scope.
The frozen source, parent manifest/checkpoint, and ordinary AP stay untouched.
"""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import base5_performance_resume as performance

SCHEMA = 'base5_performance_launcher_v1'
GROUP = 'base5-full30750-261021'
PARENT_CONTRACT = '7de3d916b0080074bbdbe62b97f714c398579ad2f9bf638c60719a8b18a0960f'
BACKUP_ROUTE = 'vast_to_drive_direct_access_only_v1'


def require(value, message):
    performance.require(value, message)


def sha(path):
    return performance.sha256(path)


def read(path):
    return json.loads(Path(path).read_text())


def json_identity(value):
    """Compare serialized identities; never normalize the executable config."""
    return json.loads(json.dumps(value, allow_nan=False))


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_config(path):
    c = read(path)
    require(c.get('schema') == SCHEMA and c.get('epochs') == 12
            and c.get('expected_scenes') == 30750 and c.get('variant') == 'cosine'
            and c.get('compile_camera') is False,
            'Explicit full30750 twelve-epoch compile-OFF continuation required')
    for key in ('source', 'data', 'work', 'initializer', 'remote_script', 'remote_config',
                'parent_manifest', 'parent_checkpoint', 'parent_work', 'adapter_path',
                'qualified_metadata_script', 'resume_helper_script', 'owned_storage_script'):
        value = Path(c[key])
        require(value.is_absolute() and '..' not in value.parts,
                'Explicit absolute scoped paths required: ' + key)
    require(c['parent_contract_sha256'] == PARENT_CONTRACT and c['work'] != c['parent_work']
            and Path(c['parent_manifest']).parent == Path(c['parent_work'])
            and Path(c['parent_checkpoint']).parent == Path(c['parent_work']),
            'A separate successor of the qualified parent is required')
    policy = performance.normalize_policy(c['runtime_policy'])
    require(policy['owned_storage_extension_sha256'] == c['owned_storage_sha256'],
            'Runtime policy must bind the reviewed owned-storage extension')
    extensions = c['runtime_extensions']
    expected = {c['remote_script']: c['launcher_sha256'],
        c['resume_helper_script']: c['resume_helper_sha256'],
        c['owned_storage_script']: c['owned_storage_sha256'],
        c['qualified_metadata_script']: c['qualified_metadata_sha256'],
        c['adapter_path']: c['adapter_sha256']}
    for name in ('sunny15_vast_training.py', 'sunny15_backup_qualification.py',
                 'base5_vast_oneclick.py', 'base5_wandb_relay.py'):
        expected[str(Path(c['remote_script']).with_name(name))] = c['runtime_hashes'][name]
    expected[str(Path(c['adapter_path']).with_name('full_data_oneclick.py'))] = \
        c['runtime_hashes']['full_data_oneclick.py']
    require(all(extensions.get(name) == checksum for name, checksum in expected.items()),
            'Every effective launcher/prepare/storage/metadata helper must be explicitly bound')
    require(c.get('drive_backup_route') == BACKUP_ROUTE
            and '/full30750_speed_v1/' in c['destination']
            and c['train_session'] in c['all_train_sessions']
            and c['train_session'] != 'base5-full30750-val-logging-training'
            and c['backup_session'] != 'base5-full30750-val-logging-backup',
            'Separate successor training/backup sessions and Drive scope required')
    extension = c.get('training_extension')
    enabled_paths = [name for name in extensions
                     if Path(name).name == 'base5_skip_discarded_heatmap.py']
    require(enabled_paths == ([] if extension is None else [extension['path']]),
            'The effective extension map must unambiguously record discarded-heatmap enablement')
    if extension is not None:
        require(Path(extension['path']).is_absolute()
                and Path(extension['path']).name == 'base5_skip_discarded_heatmap.py'
                and extensions.get(extension['path']) == extension['sha256']
                and Path(extension['qualification_receipt']).is_absolute(),
                'Only the explicitly qualified discarded-heatmap extension is recognized')
    return c


def command(c, *, resume=False, run=False):
    result = ['python', '-u', c['remote_script'], '--config', c['remote_config'], '--prepare-runtime']
    if resume:
        result.append('--resume')
    if run:
        training_authority(c)
        result.append('--execute')
    return result


def training_authority(c):
    require(c.get('epochs_user_approved') is True
            and c.get('actual_training_start_approved') is True,
            'Explicit user-authorized continuation/start is required')
    authority = read(c['start_authority'])
    require(authority.get('authorized') is True
            and authority.get('user_approved_training_start') is True
            and authority.get('epochs') == 12
            and authority.get('parent_contract_sha256') == c['parent_contract_sha256']
            and authority.get('parent_checkpoint_sha256') == c['parent_checkpoint_sha256']
            and authority.get('dataset_manifest_sha256') == c['dataset_manifest_sha256']
            and authority.get('runtime_extensions') == c['runtime_extensions']
            and performance.normalize_policy(authority.get('runtime_policy', {}))
                == performance.normalize_policy(c['runtime_policy']),
            'Start authority differs from the exact parent/data/budget/runtime policy')


def data_gate(c):
    """Bound metadata only: no repeat image/native payload admission walk."""
    require(sha(c['parent_manifest']) == c['parent_manifest_sha256'], 'Parent manifest changed')
    parent = read(c['parent_manifest'])
    require(parent['contract_sha256'] == performance.contract_sha256(parent['identity'])
            == c['parent_contract_sha256'], 'Parent contract is invalid')
    require(sha(c['data']) == c['dataset_manifest_sha256']
            == parent['identity']['dataset_manifest_sha256'], 'Cohort index changed')
    performance.validate_runtime_extensions(c['runtime_extensions'])
    for cohort in parent['identity']['dataset_cohorts']:
        root = Path(cohort['root'])
        require(sha(root / 'manifest.json') == cohort['manifest_sha256'],
                'Qualified cohort manifest changed')
        for split, checksum in cohort['info_sha256'].items():
            require(sha(root / 'infos/512x1056' / ('morai_base5_infos_' + split + '.pkl')) == checksum,
                    'Qualified cohort annotation metadata changed')
    return parent


def _training_extension_gate(c, *, install=False):
    extension = c.get('training_extension')
    if extension is None:
        return
    require(sha(extension['path']) == extension['sha256']
            and sha(extension['qualification_receipt']) == extension['qualification_sha256'],
            'Training performance extension/proof changed')
    proof = read(extension['qualification_receipt'])
    fixtures = proof.get('fixtures', [])
    require(proof.get('complete') is True and proof.get('valid') is True
            and proof.get('expected_fixture_batches') == len(fixtures) == 3
            and proof.get('contract_sha256') == c['parent_contract_sha256']
            and proof.get('checkpoint_sha256') == c['parent_checkpoint_sha256']
            and proof.get('extension', {}).get('version') == 'base5_skip_discarded_heatmap_v1'
            and proof.get('extension', {}).get('script_sha256') == extension['sha256']
            and all(proof.get(name) is True for name in
                    ('no_optimizer_step', 'no_checkpoint_write', 'model_original_unmodified'))
            and proof.get('compile_camera') is False
            and all(row.get('targets_exact') is True and row.get('losses_exact') is True
                    and row.get('finite') is True and row.get('RNG_exact') is True
                    and row.get('gradient', {}).get('exact') is True
                    and row.get('gradient', {}).get('max_abs') == 0.
                    and row.get('baseline_losses') == row.get('candidate_losses')
                    for row in fixtures),
            'Actual parent-fixture target/loss/gradient/RNG equivalence proof is required')
    if install:
        module = load_module(extension['path'], '_qualified_base5_skip_discarded_heatmap')
        require(module.VERSION == 'base5_skip_discarded_heatmap_v1',
                'Unrecognized training performance extension version')
        module.install()


def prepared_manifest(c, resume=False, *, status_only=False):
    parent = data_gate(c)
    manifest = read(Path(c['work']) / 'run_manifest.json')
    original = {key: value for key, value in manifest['identity'].items()
                if key != 'performance_runtime'}
    transition = manifest['identity']['performance_runtime']
    require(original == parent['identity']
            and manifest['contract_sha256'] == performance.contract_sha256(manifest['identity'])
            and manifest['parent_run'] == transition,
            'Successor changed an original training identity or its own contract')
    require(transition['parent_checkpoint_sha256'] == c['parent_checkpoint_sha256']
            and transition['parent_checkpoint'] == c['parent_checkpoint']
            and transition['parent_contract_sha256'] == c['parent_contract_sha256']
            and transition['boundary_epoch'] == 1 and transition['boundary_iteration'] == 6534
            and transition['runtime_policy'] == performance.normalize_policy(c['runtime_policy'])
            and transition['runtime_extensions'] == c['runtime_extensions']
            and transition['helper_sha256'] == c['resume_helper_sha256'],
            'Successor parent/runtime policy differs from the reviewed configuration')
    for name, checksum in parent['identity']['source_hashes'].items():
        require(sha(Path(c['source']) / name) == checksum, 'Original source changed: ' + name)
    require(sha(c['initializer']) == parent['identity']['initial_checkpoint_sha256']
            and sha(Path(c['source']) / c['config']) == parent['identity']['config_sha256'],
            'Original initializer/config changed')
    _training_extension_gate(c)
    work = Path(c['work'])
    started = ((work / 'performance_resume_start.json').exists()
        or (work / 'resume_start.json').exists() or any(work.glob('*.pth'))
        or (work / 'last_checkpoint').exists())
    if status_only:
        return manifest
    if resume:
        pointer = work / 'last_checkpoint'
        require(pointer.is_file() and Path(pointer.read_text().strip()).resolve(strict=True).parent
                == work.resolve(), 'No strict child-scope checkpoint is available')
    else:
        require(not started, 'Successor already started; explicit --resume is required')
    return manifest


def prepare_runtime(c, *, resume=False, execute=False):
    """Build ordered parent config, then bind the explicit successor overlay."""
    if execute:
        training_authority(c)
    parent = data_gate(c)
    source = Path(c['source']).resolve(strict=True)
    adapter = load_module(c['adapter_path'], '_performance_original_full_adapter')
    core_path = source / adapter.CORE_REL
    require(sha(core_path) == adapter.CORE_SHA, 'Frozen original training core changed')
    sys.path[:0] = [str(source), str(source / 'third_party/mmdetection3d'), str(core_path.parent)]
    from scripts.runtime import base5_distance_diagnostics as diagnostics
    diagnostics.install()
    val_path = Path(c['adapter_path']).with_name('base5_val_logging.py')
    require(sha(val_path) == adapter.VAL_LOGGING_SHA, 'Original validation extension changed')
    load_module(val_path, '_performance_original_val_logging').install()
    metadata = load_module(c['qualified_metadata_script'], '_performance_qualified_metadata')
    config_path = source / c['config']
    metadata_args = argparse.Namespace(source=source, initializer=Path(c['initializer']),
        data_index=Path(c['data']), reuse_qualified_manifest=Path(c['parent_manifest']),
        qualified_manifest_sha256=c['parent_manifest_sha256'])
    state = metadata.qualified_metadata(metadata_args, adapter, config_path)

    def inspected(path, validate, *, verify_hashes=False):
        require(not verify_hashes and Path(path) == Path(c['data']),
                'Only the exact previously qualified metadata may be reused')
        return copy.deepcopy(state)

    adapter.inspect_index = inspected
    core = load_module(core_path, '_performance_original_ordered_core')
    work = Path(c['work'])
    work.mkdir(parents=True, exist_ok=True)
    stage = work.parent / ('.' + work.name + '_ordered_prepare')
    setup = argparse.Namespace(source=source, data_index=Path(c['data']), data=None,
        work_dir=stage, init_checkpoint=Path(c['initializer']), epochs=12, seed=c['seed'],
        wandb=True, resume=None, run=False, config=config_path, profile='512x1056',
        batch=4, accumulate=1, workers=4, lr=1e-4, warmup_updates=500,
        lr_schedule='cosine', disable_val=False, jpeg_cache=None, allow_random_init=False,
        wandb_group=GROUP)
    cfg, candidate = adapter.adapted_prepare(core_path.read_text(), vars(core))(setup)
    # The original builder contains harmless tuples; its saved JSON contains
    # lists.  Canonicalize the MANIFEST comparison only, preserving the ordered
    # executable cfg and optimizer parameter construction without modification.
    candidate = copy.deepcopy(candidate)
    candidate['identity'] = json_identity(candidate['identity'])
    require(candidate['identity'] == parent['identity']
            and performance.contract_sha256(candidate['identity'])
                == candidate['contract_sha256'] == parent['contract_sha256'],
            'Ordered parent reconstruction changed')
    diagnostics.enable_resume_metadata_updates(cfg)
    cfg.custom_hooks.append(dict(type='Base5DistanceDiagnosticReceiptHook'))
    cfg.custom_hooks.append(dict(type='Base5ValLoggingReceiptHook'))
    cfg.work_dir = str(work)
    _training_extension_gate(c, install=True)
    performance.install_transition_hook()
    manifest_path = work / 'run_manifest.json'
    if resume:
        checkpoint = Path((work / 'last_checkpoint').read_text().strip())
        manifest = performance.bind_successor_resume(cfg, candidate, read(manifest_path),
                                                     checkpoint=checkpoint)
    else:
        manifest = performance.bind_successor(cfg, candidate, parent,
            parent_work_dir=c['parent_work'], parent_checkpoint=c['parent_checkpoint'],
            parent_checkpoint_sha256=c['parent_checkpoint_sha256'], policy=c['runtime_policy'],
            runtime_extensions=c['runtime_extensions'])
        if manifest_path.exists():
            require(read(manifest_path) == manifest, 'Do not replace an existing successor manifest')
        else:
            with manifest_path.open('x') as stream:
                stream.write(json.dumps(manifest, indent=2) + '\n')
    cfg.dump(str(work / ('resolved_resume.py' if resume else 'resolved_train.py')))
    print(json.dumps(dict(event='performance_continuation_prepared',
        contract=manifest['contract_sha256'], parent_contract=c['parent_contract_sha256'],
        parent_epoch=1, parent_iteration=6534, training_started=False,
        actual_run_requested=execute, runtime_policy=c['runtime_policy'])), flush=True)
    if execute:
        from mmengine.utils import import_modules_from_strings
        import_modules_from_strings(**cfg.custom_imports)
        from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner
        Base5AtomicRunner.from_cfg(cfg).train()
    return manifest


def wire_transport(c, legacy):
    """Inject scope only; keep the proven locks, online gates and tmux lifecycle."""
    full_path = (Path(c['local_repo']) / 'scripts/runtime/full_data_oneclick.py'
        if (Path(c['local_repo']) / 'scripts/runtime/full_data_oneclick.py').is_file()
        else Path(c['adapter_path']).with_name('full_data_oneclick.py'))
    require(sha(full_path) == c['runtime_hashes']['full_data_oneclick.py'],
            'Original direct-backup/timeout transport helper changed')
    full = load_module(full_path, '_performance_original_full_transport_scope')
    full.preparation_timeouts(legacy.transport, c['remote_script'])
    legacy.command = command
    legacy.data_gate = data_gate
    legacy.prepared_manifest = prepared_manifest
    legacy.load_config = load_config
    legacy.GROUP = GROUP
    legacy.ROOT = Path(c['source'])
    legacy.transport.ROOT = Path(c['source'])
    legacy.backup_qualification.ROOT = Path(c['local_repo']) if Path(c['local_repo']).is_dir() else Path(c['source'])
    legacy.backup_once = lambda config: full.backup_once(config, legacy)
    original_backup_gate = legacy.backup_gate

    def backup_gate(config):
        proof = original_backup_gate(config)
        require(config['drive_backup_route'] == BACKUP_ROUTE, 'Direct Drive route changed')
        if proof.get('qualification_kind') == 'reused_full_backup_qualification':
            require(proof.get('network_route') == BACKUP_ROUTE
                    and proof.get('new_route_small_write_readback') is True
                    and proof.get('new_full_checkpoint_throughput_verified') is False,
                    'Explicit successor-destination direct-route proof is required')
        return proof

    legacy.backup_gate = backup_gate
    legacy.__file__ = c['local_script']
    return legacy


def remote_status(c):
    work = Path(c['work'])
    started = any((work / name).exists() for name in
                  ('performance_resume_start.json', 'resume_start.json', 'last_checkpoint'))
    result = dict(training_started=started, requires_online_preflight=True)
    for name in ('oneclick_status.json', 'backup_status.json'):
        if (work / name).is_file():
            result[name] = read(work / name)
    try:
        prepared_manifest(c, resume=started, status_only=True)
        result['data_and_contract_ready'] = True
    except (ValueError, OSError, KeyError) as error:
        result.update(data_and_contract_ready=False, blocker=str(error))
    print(json.dumps(dict(event='performance_continuation_status', **result)), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('--config', type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    for name in ('prepare-runtime', 'prepare', 'check', 'run', 'remote-status',
                 'remote-prepare', 'remote-check', 'remote-start', 'remote-worker', 'backup-worker'):
        modes.add_argument('--' + name, action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    c = load_config(args.config)
    require(not args.execute or args.prepare_runtime, '--execute is internal to --prepare-runtime')
    if args.prepare_runtime:
        prepare_runtime(c, resume=args.resume, execute=args.execute)
        return 0
    import sunny15_vast_training
    legacy = wire_transport(c, sunny15_vast_training)
    if args.remote_status:
        remote_status(c)
    elif args.remote_prepare:
        prepare_runtime(c, resume=args.resume)
        prepared_manifest(c, resume=args.resume)
    elif args.remote_check:
        legacy.remote_check(c, args.resume)
    elif args.remote_start:
        training_authority(c)
        legacy.remote_start(c, args.resume)
    elif args.remote_worker:
        training_authority(c)
        return legacy.remote_worker(c, args.resume)
    elif args.backup_worker:
        return legacy.backup_worker(c)
    else:
        if args.run:
            require(c.get('actual_training_start_approved') is True,
                    'Explicit continuation/start authorization required')
        action = 'prepare' if args.prepare else 'check' if args.check else 'start' if args.run else 'status'
        legacy.local_action(c, action, args.resume)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
