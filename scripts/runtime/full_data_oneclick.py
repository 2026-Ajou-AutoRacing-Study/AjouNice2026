#!/usr/bin/env python3
"""Full-data binding around unchanged proven W&B/Drive/tmux transport.

Epoch budget and actual start require a separate user authority. This setup
entry defaults to status; prepare is never training. Original E8/E24 runs and
source remain untouched. Credentials are handled only by original transport.
"""
import sys
sys.dont_write_bytecode=True
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shlex
import subprocess

HERE=Path(__file__).resolve().parent
BACKUP_ROUTE='vast_to_drive_direct_access_only_v1'
ACCESS_HELPER_REL='perception/detection3d/bevfusion/tools/backup_base5_vast_access_only.py'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(value,message):
    if not value:raise ValueError(message)


def read(path):return json.loads(Path(path).read_text())


def load_config(path):
    c=read(path)
    require(c.get('schema')=='base5_full_data_launcher_v1' and c.get('expected_scenes')==30749
        and c.get('variant')=='cosine','Require new explicit full-data scope')
    for key in ('source','data','work','initializer','remote_script','remote_config','prepare_script'):
        require(Path(c[key]).is_absolute() and '..' not in Path(c[key]).parts,'Absolute scoped paths required')
    require(c['source']!=c['work'] and c['epochs']>0 and c.get('compile_camera') is False,
            'Separate work and compile-OFF budget required')
    return c


def command(c,*,resume=False,run=False):
    result=['python','-u',c['prepare_script'],'--source',c['source'],'--data-index',c['data'],
        '--work-dir',c['work'],'--init-checkpoint',c['initializer'],'--epochs',str(c['epochs']),
        '--seed',str(c['seed']),'--wandb']
    if resume:result+=['--resume','auto']
    if run:
        require(c.get('epochs_user_approved') is True,'Epoch choice is still draft')
        authority=Path(c['start_authority'])
        require(authority.is_file(),'No separate actual-training-start authority')
        result+=['--start-authority',str(authority),'--run']
    return result


def data_gate(c):
    index=Path(c['data']);require(sha(index)==c['dataset_manifest_sha256'],'Data index SHA differs')
    doc=read(index)
    require(doc.get('complete') is True and doc.get('schema')=='base5_full_cohort_index_v1'
        and len(doc.get('cohorts',[]))==9,'Nine final supported cohorts not ready')
    for item in doc['cohorts']:
        require(sha(Path(item['root'])/'manifest.json')==item['manifest_sha256'],
                'Cohort manifest differs: '+item['name'])
    return doc


def prepared_manifest(c,resume=False):
    doc=data_gate(c);work=Path(c['work']);manifest=read(work/'run_manifest.json');identity=manifest['identity']
    expected=dict(epochs=c['epochs'],batch=4,workers=4,accumulate=1,lr=1e-4,seed=c['seed'],
        val_enabled=True,profile='512x1056',training_label_policy='animal_to_pedestrian_v1',
        wandb_group='base5-full30749-261021',dataset_manifest_sha256=c['dataset_manifest_sha256'])
    require(all(identity.get(k)==v for k,v in expected.items()),'Full-data prepared settings differ')
    require(list(identity.get('classes',[]))==['vehicle','pedestrian','obstacle']
        and identity['model'].get('compile_camera') is False
        and identity['full_data_adapter_sha256']==sha(c['prepare_script']), 'Fixed classes/compile/adapter differs')
    require([s['type'] for s in identity['param_schedulers']]==['LinearLR','CosineAnnealingLR']
        and identity['warmup_iterations']==500,'Plain cosine and original warmup required')
    require(identity['train_count']==sum(x['split_counts']['train'] for x in identity['dataset_cohorts'])
        and identity['val_count']==4614 and len(identity['dataset_cohorts'])==9,'Actual cohort counts differ')
    require({x['name']:x['manifest_sha256'] for x in identity['dataset_cohorts']}==
        {x['name']:x['manifest_sha256'] for x in doc['cohorts']},'Prepared cohort identities differ')
    require(identity.get('distance_diagnostic_sha256')==c['diagnostic_sha256']
        and sha(Path(c['source'])/'scripts/runtime/base5_distance_diagnostics.py')==c['diagnostic_sha256'],
        'Bound distance-logging extension differs')
    require(c.get('drive_backup_route')==identity.get('drive_backup_route')==BACKUP_ROUTE
        and identity.get('full_data_launcher_sha256')==sha(__file__),
        'Bound direct backup route/launcher differs')
    for item in identity['dataset_cohorts']:
        for split,checksum in item['info_sha256'].items():
            require(sha(Path(item['root'])/'infos/512x1056'/('morai_base5_infos_'+split+'.pkl'))==checksum,
                    'Original cohort annotations changed')
    for name,checksum in identity['source_hashes'].items():
        require(sha(Path(c['source'])/name)==checksum,'Frozen detector source changed')
    for name,checksum in c['runtime_hashes'].items():
        require(sha(Path(c['remote_script']).parent/name)==checksum,'Full-data launcher helper changed')
    require(sha(c['initializer'])==identity['initial_checkpoint_sha256']
        and sha(Path(c['source'])/c['config'])==identity['config_sha256'],'Initializer/config changed')
    started=(work/'fresh_start.json').exists() or any(work.glob('*.pth')) or (work/'last_checkpoint').exists()
    if resume:
        pointer=work/'last_checkpoint';require(pointer.is_file(),'No completed epoch to resume')
        require(Path(pointer.read_text().strip()).resolve(strict=True).parent==work.resolve(),
                'Resume checkpoint outside this run')
    else:require(not started,'Explicit resume required for a started run')
    return manifest


def direct_backup_prefix():
    # Only this access-only uploader child uses the direct Google route.
    # The separately owned W&B reverse relay is not modified or disabled.
    return "import os\nfor _key in ('HTTPS_PROXY','HTTP_PROXY','ALL_PROXY','https_proxy','http_proxy','all_proxy'):\n os.environ.pop(_key,None)\n"


def backup_once(c,legacy):
    require(c.get('drive_backup_route')==BACKUP_ROUTE,'Explicit direct backup route required')
    path=Path(c['local_repo'])/ACCESS_HELPER_REL
    require(sha(path)==c['access_only_backup_sha256'],'Unchanged access-only uploader required')
    spec=importlib.util.spec_from_file_location('full_data_original_access_only_backup',path)
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    helper.REMOTE=direct_backup_prefix()+helper.REMOTE
    previous=sys.argv
    try:
        sys.argv=[str(path)]+legacy.transport.backup_arguments(c,Path(c['local_logs']))
        try:return helper.main()
        except SystemExit as error:return error.code or 0
    finally:sys.argv=previous


def smoke(c):
    """Actual MMEngine Concat/annotation compatibility, no model/GPU execution."""
    data_gate(c)
    sys.path[:0]=[c['source'],c['source']+'/third_party/mmdetection3d',
        c['source']+'/perception/detection3d/bevfusion/tools']
    from full_data_prepare import inspect_index,attach_concat
    from prepare_base5_portable_dataset import validate
    from mmengine.config import Config
    from mmengine.utils import import_modules_from_strings
    from mmdet3d.registry import DATASETS
    from mmengine.registry import init_default_scope
    state=inspect_index(c['data'],validate,verify_hashes=False)
    cfg=Config.fromfile(str(Path(c['source'])/c['config']))
    import_modules_from_strings(**cfg.custom_imports);init_default_scope('mmdet3d')
    attach_concat(cfg,state)
    counts={};samples=[]
    for split in ('train','val'):
        dataset=DATASETS.build(cfg[split+'_dataloader'].dataset)
        require(len(dataset)==state['counts'][split],'Built Concat sample count differs')
        counts[split]=len(dataset)
        offset=0
        for child,cohort in zip(dataset.datasets,state['cohorts']):
            require(len(child)==cohort['split_counts'][split],'Built child sample count differs')
            first=dataset.get_data_info(offset) if hasattr(dataset,'get_data_info') else child.get_data_info(0)
            require(first['token'],'Built child lost native token')
            samples.append(dict(cohort=cohort['name'],split=split,first_token=first['token']))
            offset+=len(child)
    print(json.dumps(dict(event='full_data_concat_smoke_pass',frames=sum(counts.values()),
        counts=counts,children_checked=len(samples),index_sha256=state['index_sha256'],
        training_started=False,model_executed=False)),flush=True)


def main(argv=None):
    p=argparse.ArgumentParser(allow_abbrev=False);p.add_argument('--config',type=Path,required=True)
    modes=p.add_mutually_exclusive_group()
    for mode in ('prepare','check','run','smoke','backup-worker','remote-status','remote-prepare',
                 'remote-check','remote-start','remote-worker','remote-smoke'):
        modes.add_argument('--'+mode,action='store_true')
    p.add_argument('--resume',action='store_true');args=p.parse_args(argv);c=load_config(args.config)
    # Use exact original modules from repo locally / frozen sibling copies remotely.
    if not (HERE/'sunny15_vast_training.py').is_file():sys.path.insert(0,c['local_repo']+'/scripts/runtime')
    import sunny15_vast_training as old
    old.ROOT=Path(c['source'] if any(k.startswith('remote_') and v for k,v in vars(args).items()) else c['local_repo'])
    old.transport.ROOT=old.ROOT;old.backup_qualification.ROOT=old.ROOT
    old.command=command;old.data_gate=data_gate;old.prepared_manifest=prepared_manifest
    old.load_config=load_config;old.GROUP='base5-full30749-261021'
    old.backup_once=lambda config:backup_once(config,old)
    original_backup_gate=old.backup_gate
    def route_backup_gate(config):
        proof=original_backup_gate(config)
        require(config.get('drive_backup_route')==BACKUP_ROUTE,'Backup route changed')
        if proof.get('qualification_kind')=='reused_full_backup_qualification':
            require(proof.get('network_route')==BACKUP_ROUTE
                and proof.get('new_route_small_write_readback') is True
                and proof.get('new_full_checkpoint_throughput_verified') is False,
                'Separate small direct-route probe required; no new full-backup claim')
        return proof
    old.backup_gate=route_backup_gate
    # Legacy backup_worker spawns its own __file__ only through local_action.
    # Redirect that one process identity to this new entry, originals unchanged.
    old.__file__=str(Path(c['local_script']))
    if args.remote_smoke:return smoke(c)
    if args.remote_status:old.remote_status(c)
    elif args.remote_prepare:
        old.transport.checked(command(c,resume=args.resume),cwd=c['source'],env=old.remote_env(c),timeout=900)
    elif args.remote_check:old.remote_check(c,args.resume)
    elif args.remote_start:
        command(c,resume=args.resume,run=True) # authority before launching a tmux worker
        old.remote_start(c,args.resume)
    elif args.remote_worker:return old.remote_worker(c,args.resume)
    elif args.backup_worker:return old.backup_worker(c)
    elif args.smoke:
        return old.transport.ssh(c,shlex.join(['python',c['remote_script'],'--config',c['remote_config'],
            '--remote-smoke']),timeout=900).returncode
    else:
        action='prepare' if args.prepare else 'check' if args.check else 'start' if args.run else 'status'
        if args.run:require(c.get('epochs_user_approved') is True,'Epoch/start choice still unapproved')
        old.local_action(c,action,args.resume)
    return 0


if __name__=='__main__':raise SystemExit(main())
