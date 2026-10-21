#!/usr/bin/env python3
"""New-scope cohort-index/Concat adapter; frozen direct runner stays unchanged.

Default prepares only. --run additionally needs a separately supplied explicit
start-authority JSON. No payload copying, JPEG encoding, installation or SSH.
"""
import sys
sys.dont_write_bytecode = True
import argparse
import ast
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import pickle

CORE_REL = 'perception/detection3d/bevfusion/tools/run_base5_direct_training.py'
CORE_SHA = '66cf8c5fcaf1ad070b39725be91bee1725cd6ba84defc8176bdd391e66fc60d7'
DIAGNOSTIC_REL = 'scripts/runtime/base5_distance_diagnostics.py'
DIAGNOSTIC_SHA = '7248640fccdfb5538a4fa32ccca282b8b5682c9d1761d2914e384e5cb3e93818'
VAL_LOGGING_SCHEMA = 'base5_val_logging_v1'
VAL_LOGGING_SHA = '658db7bc4a2a28d947f24e9e67955c914c407aab59c1b9cf8aee337d4e088af5'
BACKUP_ROUTE = 'vast_to_drive_direct_access_only_v1'
CLASSES = ('vehicle', 'pedestrian', 'obstacle')
EXPECTED = {'sunny11':5000,'sunny13':5000,'sunny15':5000,'foggy11':5000,
            'foggy13':4999,'foggy15':5000,'supp_sunny11':250,'supp_sunny13':250,
            'supp_sunny15':250,'foggy13_completion':1}
EXPECTED_COUNTS = dict(train=26136, val=4614, test=0)
GROUP = 'base5-full30750-261021'
IGNORE_META = ['layout_split_overlay_sha256']


def require(value, message):
    if not value: raise ValueError(message)


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(4*1024**2),b''):h.update(chunk)
    return h.hexdigest()


def read(path):return json.loads(Path(path).read_text())


def safe(path, directory=False):
    p=Path(path)
    require(p.is_absolute() and '..' not in p.parts
        and not any(x.is_symlink() for x in (p,*p.parents)), 'Explicit nonsymlink absolute path required')
    require(p.is_dir() if directory else p.is_file(), 'Required input missing: '+str(p))
    return p


def inspect_index(path, validate, *, verify_hashes=False):
    """Use the unchanged portable validator for every original cohort root."""
    path=safe(path);index_sha=sha(path);doc=read(path)
    require(doc.get('schema')=='base5_full_cohort_index_v1' and doc.get('complete') is True,
            'Final ten-cohort index is not complete; do not substitute placeholders')
    cohorts=doc.get('cohorts',[])
    require(len(cohorts)==len(EXPECTED) and {x['name']:x['expected_scenes'] for x in cohorts}==EXPECTED,
            'Exact latest six supported environments, supplement750 and new Foggy13 frame required')
    tokens=set();raws=set();payloads=set();groups={};counts=dict(train=0,val=0,test=0)
    products=[];reference=None;val_tokens={}
    for item in cohorts:
        root=safe(item['root'],True);manifest=root/'manifest.json'
        require(sha(manifest)==item['manifest_sha256'], 'Cohort manifest differs: '+item['name'])
        inventory=validate(root,verify_hashes=verify_hashes)
        require(inventory['scenes']==item['expected_scenes']
            and inventory['profiles']==['512x1056']
            and inventory.get('training_label_policy')=='animal_to_pedestrian_v1'
            and inventory.get('box_geometry_policy')=='native_rpy_fixed_yaw_enclosure_v1'
            and inventory.get('image_policy')==dict.fromkeys(('train','val','test'),'native_jpeg90'),
            'Current Q90/upright/three-class cohort contract differs')
        split_counts={};info_pins={};found=set()
        for split in ('train','val','test'):
            info_path=root/'infos/512x1056'/('morai_base5_infos_'+split+'.pkl')
            info=pickle.loads(info_path.read_bytes());meta=info['metainfo']
            require(tuple(meta['classes'])==CLASSES
                and meta.get('training_label_policy')=='animal_to_pedestrian_v1'
                and meta.get('box_geometry_policy')=='native_rpy_fixed_yaw_enclosure_v1',
                'Cohort infos differ from fixed model semantics')
            common={k:v for k,v in meta.items() if k not in IGNORE_META}
            if reference is None:reference=common
            require(common==reference,'Only cohort layout-overlay provenance may differ in metainfo')
            require(split!='test' or not info['data_list'],'Independent test remains empty')
            for row in info['data_list']:
                token=row['token'];group=row['scene_group_id']
                require(token not in tokens and token not in found,'Duplicate global frame token')
                require(group not in groups or groups[group]==split,'Physical scene-group train/val leakage')
                require(row.get('split_overlay_sha256')==meta['layout_split_overlay_sha256'],
                        'Original per-frame overlay binding changed')
                raw=inventory['provenance'][token]['original_capture_root']
                payload=inventory['files']['data/'+token+'/scenario.json']['sha256']
                require(raw not in raws and payload not in payloads,'Duplicate global native observation/payload')
                require(len(row['images'])==4,'Four original views required')
                for instance in row.get('instances',[]):
                    original=instance.get('original_bbox_label_3d')
                    require(type(original) is int and original in range(4)
                        and instance.get('original_semantic_class')==('vehicle','pedestrian','obstacle','animal')[original]
                        and instance['bbox_label_3d']==(1 if original==3 else original),
                        'Original semantic provenance or Animal mapping changed')
                tokens.add(token);found.add(token);groups[group]=split;raws.add(raw);payloads.add(payload)
                if split=='val':
                    require(row.get('sample_idx')==token
                        and Path(row['lidar_points']['lidar_path']).parts.count(token)==1,
                        'Validation sample_idx/LiDAR path must preserve the native token')
                    val_tokens[token]=item['name']
            split_counts[split]=len(info['data_list']);counts[split]+=split_counts[split]
            info_pins[split]=sha(info_path)
        require(sum(split_counts.values())==item['expected_scenes']
            and {k:v for k,v in split_counts.items() if v}==inventory['split_counts'],
            'Actual info counts differ from original portable manifest')
        products.append(dict(name=item['name'],root=str(root),manifest_sha256=item['manifest_sha256'],
            scenes=item['expected_scenes'],split_counts=split_counts,info_sha256=info_pins))
    require(len(tokens)==sum(EXPECTED.values()) and counts==EXPECTED_COUNTS,
            'Full-data scene/split count differs')
    require(sha(path)==index_sha, 'Index file changed during inspection')
    return dict(index=str(path),index_sha256=index_sha,cohorts=products,counts=counts,
        train_count=counts['train'],payload_hashes_verified=verify_hashes,
        global_tokens=len(tokens),group_split_leakage=0,original_payloads_copied=0,
        val_token_to_cohort=val_tokens)


def attach_concat(cfg,state):
    """Each original dataset keeps its own root, infos and ordinary pipeline."""
    state['train_pipeline']=copy.deepcopy(list(cfg.train_dataloader.dataset.pipeline))
    for split in ('train','val'):
        loader=cfg[split+'_dataloader'];template=copy.deepcopy(loader.dataset)
        children=[]
        for cohort in state['cohorts']:
            # The replacement is train-only. Do not instantiate an empty
            # BaseDataset child (MMEngine cannot serialize an empty list).
            if cohort['split_counts'][split]==0:
                continue
            child=copy.deepcopy(template)
            child.data_root=cohort['root'];child.data_prefix=dict(pts='',img='')
            child.ann_file=str(Path(cohort['root'])/'infos/512x1056'/('morai_base5_infos_'+split+'.pkl'))
            children.append(child)
        loader.dataset=dict(type='ConcatDataset',datasets=children,ignore_keys=IGNORE_META)
    cfg.val_evaluator['base5_val_logging']=dict(
        token_to_cohort=copy.deepcopy(state['val_token_to_cohort']),
        expected_cohort_frames={c['name']:c['split_counts']['val'] for c in state['cohorts']
                                if c['split_counts']['val']})


def extend_identity(identity,state):
    identity['dataset_manifest_sha256']=state['index_sha256']
    identity['dataset_cohorts']=state['cohorts']
    identity['full_data_adapter_sha256']=sha(__file__)
    identity['distance_diagnostic_sha256']=DIAGNOSTIC_SHA
    identity['drive_backup_route']=BACKUP_ROUTE
    identity['full_data_launcher_sha256']=sha(Path(__file__).with_name('full_data_oneclick.py'))
    identity['full_data_scope']='latest6_supported29999_plus_foggy13_completion1_plus_sunny_obstacle750'
    identity['concat_ignore_metainfo_keys']=IGNORE_META
    identity['val_count']=state['counts']['val']
    identity['val_logging_schema']=VAL_LOGGING_SCHEMA
    identity['val_logging_extension_sha256']=VAL_LOGGING_SHA
    identity['val_cohort_mapping_sha256']=hashlib.sha256(json.dumps(
        state['val_token_to_cohort'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return identity


def adapted_prepare(source_text,namespace):
    """Exact, detached AST adapter; original model/optim/scheduler/resume body stays."""
    tree=ast.parse(source_text)
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='prepare')
    fn=copy.deepcopy(fn);count_changes=pipeline_changes=attach_changes=identity_changes=path_changes=0
    prefix=ast.parse("full_state = _full_inspect(args.data_index, validate, verify_hashes=getattr(args, 'run', False))\nargs.data = Path(full_state['cohorts'][0]['root'])").body
    body=[]
    for node in fn.body:
        if isinstance(node,ast.Assign):
            names=[target.id for target in node.targets if isinstance(target,ast.Name)]
            if names==['count']:
                require(ast.dump(node.value)==ast.dump(ast.parse("len(pickle.loads(train_info.read_bytes())['data_list'])",mode='eval').body),
                        'Frozen original train-count statement changed')
                node.value=ast.parse("full_state['train_count']",mode='eval').body;count_changes+=1
            if names==['previous_path']:
                body.extend(ast.parse('_extend_identity(identity, full_state)').body);identity_changes+=1
            if names==['manifest']:
                body.append(node)
                body.extend(ast.parse("manifest['paths']['data'] = str(args.data_index.resolve())\nmanifest['paths']['data_kind'] = 'cohort_index_no_payload_aggregate'").body)
                path_changes+=1;continue
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Attribute) and t.attr=='test_dataloader' for t in node.targets):
            body.extend(ast.parse('_attach_concat(cfg, full_state)').body);attach_changes+=1
        body.append(node)
    for node in ast.walk(fn):
        if isinstance(node,ast.keyword) and node.arg=='train_pipeline':
            require(ast.dump(node.value)==ast.dump(ast.parse('list(cfg.train_dataloader.dataset.pipeline)',mode='eval').body),
                    'Frozen original train-pipeline identity changed')
            node.value=ast.parse("full_state['train_pipeline']",mode='eval').body;pipeline_changes+=1
    require((count_changes,pipeline_changes,attach_changes,identity_changes,path_changes)==(1,1,1,1,1),
            'Expected exact five full-data admission/binding sites')
    fn.body=prefix+body
    scope=dict(namespace,_full_inspect=inspect_index,_attach_concat=attach_concat,_extend_identity=extend_identity)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),
                 '<reviewed-full-data-concat-adapter>','exec'),scope)
    return scope['prepare']


def start_authority(args):
    require(args.start_authority is not None,'Explicit separate training-start authority is required')
    authority=read(safe(args.start_authority))
    require(authority.get('authorized') is True and authority.get('user_approved_training_start') is True
        and authority.get('epochs')==args.epochs and authority.get('cohort_index_sha256')==sha(args.data_index)
        and authority.get('adapter_sha256')==sha(__file__), 'Training-start authority differs from reviewed data/budget/code')


def main(argv=None):
    p=argparse.ArgumentParser(allow_abbrev=False)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--data-index',type=Path,required=True)
    p.add_argument('--work-dir',type=Path,required=True);p.add_argument('--init-checkpoint',type=Path,required=True)
    p.add_argument('--epochs',type=int,default=8);p.add_argument('--seed',type=int,default=261015)
    p.add_argument('--wandb',action='store_true');p.add_argument('--resume',nargs='?',const='auto')
    p.add_argument('--start-authority',type=Path);p.add_argument('--run',action='store_true')
    args=p.parse_args(argv)
    require(args.epochs>0,'Positive epoch budget required')
    if args.run:start_authority(args)
    source=safe(args.source,True);core_path=source/CORE_REL
    require(sha(core_path)==CORE_SHA,'Exact qualified direct-training source changed')
    require(sha(source/DIAGNOSTIC_REL)==DIAGNOSTIC_SHA,'Exact distance-logging extension changed')
    logging_path=safe(Path(__file__).with_name('base5_val_logging.py'))
    require(sha(logging_path)==VAL_LOGGING_SHA,'Exact validation-logging extension changed')
    sys.path[:0]=[str(source),str(source/'third_party/mmdetection3d'),str(core_path.parent)]
    from mmengine.utils import import_modules_from_strings
    from scripts.runtime import base5_distance_diagnostics as diagnostics
    diagnostics.install()
    logging_spec=importlib.util.spec_from_file_location('_qualified_base5_val_logging',logging_path)
    logging_extension=importlib.util.module_from_spec(logging_spec)
    logging_spec.loader.exec_module(logging_extension)
    require(logging_extension.VERSION==VAL_LOGGING_SCHEMA,'Validation logging schema differs')
    logging_extension.install()
    spec=importlib.util.spec_from_file_location('_unchanged_full_data_training_core',core_path)
    core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)
    args.config=source/'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_next_training.py'
    args.profile='512x1056';args.batch=4;args.accumulate=1;args.workers=4;args.lr=1e-4
    args.warmup_updates=500;args.lr_schedule='cosine';args.disable_val=False
    args.jpeg_cache=None;args.allow_random_init=False;args.wandb_group=GROUP
    prepare=adapted_prepare(core_path.read_text(),vars(core))
    cfg,manifest=prepare(args)
    require(cfg.model.get('compile_camera') is False and not cfg.get('compile',False),'Compile remains OFF')
    diagnostics.enable_resume_metadata_updates(cfg)
    cfg.custom_hooks.append(dict(type='Base5DistanceDiagnosticReceiptHook'))
    cfg.custom_hooks.append(dict(type='Base5ValLoggingReceiptHook'))
    cfg.dump(str(args.work_dir/('resolved_resume.py' if args.resume else 'resolved_train.py')))
    print(json.dumps(dict(event='full_data_prepared',contract=manifest['contract_sha256'],
        frames=sum(c['scenes'] for c in manifest['identity']['dataset_cohorts']),
        train=manifest['identity']['train_count'],val=manifest['identity']['val_count'],
        epochs=args.epochs,optimizer_updates_per_epoch=manifest['optimizer_updates_per_epoch'],
        training_started=False,actual_run_requested=args.run)),flush=True)
    if args.run:
        import_modules_from_strings(**cfg.custom_imports)
        from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner
        Base5AtomicRunner.from_cfg(cfg).train()
    return 0


if __name__=='__main__':raise SystemExit(main())
