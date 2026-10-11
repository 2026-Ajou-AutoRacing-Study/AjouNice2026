#!/usr/bin/env python3
"""Two updates per epoch: full-model AMP, atomic save, interruption and resume.

Requires the portable 5K and CUDA environment. Bounded wiring test only, not a
training recipe or accuracy result. Outputs source tokens and full state proof.
"""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import pickle
import random

import numpy as np

import torch
from mmengine.hooks import Hook
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import HOOKS

from prepare_base5_portable_dataset import build, safe_relative, sha256, validate
from run_base5_direct_training import prepare
from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner


class PlannedStop(Exception): pass


@HOOKS.register_module(force=True)
class Base5SmokeStop(Hook):
    priority='LOWEST'
    def after_train_epoch(self,runner):
        if runner.epoch==0: raise PlannedStop('Bounded interruption after checkpoint')


def state_equal(a,b):
    """Exact recursive checkpoint comparison, including full optimizer tensors."""
    if torch.is_tensor(a):return torch.is_tensor(b) and torch.equal(a.cpu(),b.cpu())
    if isinstance(a,np.ndarray):return isinstance(b,np.ndarray) and np.array_equal(a,b)
    if isinstance(a,dict):return isinstance(b,dict) and a.keys()==b.keys() and all(state_equal(a[k],b[k]) for k in a)
    if isinstance(a,(list,tuple)):return isinstance(b,type(a)) and len(a)==len(b) and all(state_equal(x,y) for x,y in zip(a,b))
    return a==b


@HOOKS.register_module(force=True)
class Base5SmokeResumeProof(Hook):
    # Run after MoraiResumeEvidenceHook restores main-process RNG.
    priority='LOWEST'
    def before_train(self,runner):
        if not runner.cfg.get('resume',False):return
        source=torch.load(runner.cfg.load_from,map_location='cpu',weights_only=False)
        rng=source['base5_rng']
        checks=dict(model=state_equal(source['state_dict'],runner.model.state_dict()),
            optimizer_and_scaler=state_equal(source['optimizer'],runner.optim_wrapper.state_dict()),
            schedulers=state_equal(source['param_schedulers'],[p.state_dict() for p in runner.param_schedulers]),
            python_rng=state_equal(rng['python'],random.getstate()),
            numpy_rng=state_equal(rng['numpy'],np.random.get_state()),
            torch_rng=state_equal(rng['torch'],torch.get_rng_state()),
            cuda_rng=state_equal(rng['cuda'],torch.cuda.get_rng_state_all()),
            iteration=runner.iter==source['meta']['iter'],
            no_compiler_key_prefix=all('_orig_mod.' not in k for k in source['state_dict']))
        (Path(runner.work_dir)/'resume_payload_proof.json').write_text(json.dumps(checks,indent=2)+'\n')
        if not all(checks.values()):raise ValueError('Resumed full-state payload differs: '+str(checks))


def jpeg_smoke_subset(source, profile, selected, output):
    """Keep standalone JPEG paths and input policy in a tiny wiring dataset.

    JPEG RGB lives outside data/<token>; the original raw-capture exporter must
    not be used for this layout. Never regenerate JPEG or modify source files.
    """
    source, output = Path(source), Path(output)
    inventory = validate(source)
    if inventory.get('image_policy') != dict.fromkeys(('train','val','test'),'native_jpeg90'):
        raise ValueError('JPEG smoke requires current all-JPEG portable source')
    if output.exists(): raise FileExistsError('Fresh smoke subset required')
    tokens = {r['token'] for r in selected}
    if len(tokens) != len(selected): raise ValueError('Duplicate smoke token')
    required = set()
    for row in selected:
        required.add(row['lidar_points']['lidar_path'])
        required.update(c['img_path'] for c in row['images'].values())
    for path in inventory['files']:
        if any(path.startswith('data/'+token+'/') for token in tokens):
            required.add(path)
    required.add('infos/'+profile+'/calibration.json')
    output.mkdir(parents=True)
    files = {}
    for relative in sorted(required):
        path = source/safe_relative(relative)
        item = inventory['files'][relative]
        if sha256(path) != item['sha256']: raise ValueError('Smoke source hash differs')
        target = output/relative; target.parent.mkdir(parents=True,exist_ok=True)
        os.link(path,target)
        files[relative] = deepcopy(item)
    original = pickle.loads((source/'infos'/profile/'morai_base5_infos_train.pkl').read_bytes())
    for split in ('train','val','test'):
        relative = 'infos/'+profile+'/morai_base5_infos_'+split+'.pkl'
        path = output/relative
        path.write_bytes(pickle.dumps(dict(original,data_list=deepcopy(selected) if split=='train' else []),protocol=4))
        files[relative] = dict(bytes=path.stat().st_size,sha256=sha256(path))
    result = dict(version=1,scenes=len(selected),split_counts=dict(train=len(selected),val=0),
        profiles=[profile],files=files,bytes=sum(i['bytes'] for i in files.values()),
        image_policy=inventory['image_policy'],copy_mode='hardlink',
        source_manifest_sha256=sha256(source/'manifest.json'),
        source_tokens=sorted(tokens),scope='Bounded smoke only; not train/val selection or production data')
    if inventory.get('box_geometry_policy'):
        result['box_geometry_policy'] = inventory['box_geometry_policy']
    (output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    validate(output,verify_hashes=True)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True); p.add_argument('--init-checkpoint',type=Path,required=True)
    p.add_argument('--profile',choices=['448x896','512x1056'],default='448x896')
    p.add_argument('--batch',type=int,choices=[1,4],default=1)
    p.add_argument('--partial-final-batch',action='store_true',
                   help='With batch 4 use seven captures, exercising the real 5K final batch of three')
    p.add_argument('--capture',action='append',help='Explicit distinct train tokens for two smoke updates')
    a=p.parse_args()
    if a.partial_final_batch and a.batch!=4:p.error('Partial final batch probe requires batch 4')
    sample_count=2*a.batch-int(a.partial_final_batch)
    if a.output.exists(): p.error('Use a fresh output')
    a.output.mkdir(parents=True)
    source=a.data/'infos'/a.profile
    info=pickle.loads((source/'morai_base5_infos_train.pkl').read_bytes())
    records=info['data_list']; labels=lambda r:{i['bbox_label_3d'] for i in r['instances']}
    if a.capture:
        if len(a.capture)!=sample_count or len(set(a.capture))!=sample_count:p.error('Provide exactly the smoke sample count distinct train tokens')
        lookup={r['token']:r for r in records}
        selected=[lookup[token] for token in a.capture]
    else:
        selected=[next(r for r in records if 3 in labels(r))]
        missing=set(range(4))-labels(selected[0])
        selected.append(max((r for r in records if r['token']!=selected[0]['token']),key=lambda r:len(labels(r)&missing)))
        selected.extend([r for r in records if r['token'] not in {s['token'] for s in selected}][:sample_count-len(selected)])
    if set.union(*(labels(r) for r in selected)) != set(range(4)):
        raise ValueError('Selected captures do not exercise all four classes')
    policy=json.loads((a.data/'manifest.json').read_text()).get('image_policy')
    if policy:
        data=a.output/'mini_portable'
        jpeg_smoke_subset(a.data,a.profile,selected,data)
    else:
        data=raw_smoke_subset(a,info,selected,source)
    args=argparse.Namespace(data=data,profile=a.profile,config=a.config,work_dir=a.output/'training',
        epochs=2,batch=a.batch,accumulate=1,workers=0,lr=1e-4,warmup_updates=1,seed=261010,
        init_checkpoint=a.init_checkpoint,allow_random_init=False,resume=None,disable_val=True,wandb=False)
    cfg,manifest=prepare(args); import_modules_from_strings(**cfg.custom_imports)
    cfg.custom_hooks.append(dict(type='Base5SmokeStop'))
    cfg.custom_hooks.append(dict(type='Base5SmokeResumeProof'))
    try: Base5AtomicRunner.from_cfg(cfg).train()
    except PlannedStop: pass
    else: raise ValueError('Planned interruption not reached')
    first=torch.load(args.work_dir/'epoch_1.pth',map_location='cpu',weights_only=False)
    args.resume='auto'; resumed_cfg,_=prepare(args)
    resumed_cfg.custom_hooks.append(dict(type='Base5SmokeResumeProof'))
    Base5AtomicRunner.from_cfg(resumed_cfg).train()
    final=torch.load(args.work_dir/'epoch_2.pth',map_location='cpu',weights_only=False)
    evidence=json.loads((args.work_dir/'resume_start.json').read_text())
    if first['meta']['iter']!=2 or final['meta']['iter']!=4 or not evidence['resumed']:
        raise ValueError('Resume iteration mismatch')
    if not evidence['optimizer_steps'] or min(evidence['optimizer_steps'])<=0:
        raise ValueError('Optimizer did not update before resume')
    if any(not torch.isfinite(v).all() for v in final['state_dict'].values() if v.is_floating_point()):
        raise ValueError('Nonfinite trained weights')
    report=dict(valid=True,profile=a.profile,captures=[r['token'] for r in selected],classes=list(range(4)),
        batch=a.batch,resume_payload_proof=json.loads((args.work_dir/'resume_payload_proof.json').read_text()),
        partial_final_batch=a.partial_final_batch,
        image_policy=policy,
        first_checkpoint_iteration=2,final_checkpoint_iteration=4,resume_start=evidence,
        checkpoint_sha_contract=manifest['contract_sha256'],
        scope='two batches per epoch, four AMP iterations; not quality or converged model',
        atomic_partial_files=list(map(str,args.work_dir.glob('*.partial-*'))))
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


def raw_smoke_subset(a,info,selected,source):
    """Historical PNG layout smoke, kept separate from current JPEG input."""
    source_infos=a.output/'source_infos'; source_infos.mkdir()
    for split in ('train','val','test'):
        (source_infos/('morai_base5_infos_'+split+'.pkl')).write_bytes(
            pickle.dumps(dict(info,data_list=selected if split=='train' else [])))
    (source_infos/'calibration.json').write_bytes((source/'calibration.json').read_bytes())
    # Rewrite relative portable inputs back to absolute for the generic export
    # builder; re-export as independent relative mini-inputs.
    for record in selected:
        record['lidar_points']['lidar_path']=str((a.data/record['lidar_points']['lidar_path']).resolve())
        for camera in record['images'].values(): camera['img_path']=str((a.data/camera['img_path']).resolve())
    (source_infos/'morai_base5_infos_train.pkl').write_bytes(pickle.dumps(dict(info,data_list=selected)))
    ledger=a.output/'ledger.jsonl'
    ledger.write_text(''.join(json.dumps(dict(id=r['token'],capture_root=str((a.data/'data'/r['token']).resolve())))+'\n' for r in selected))
    data=a.output/'mini_portable'; build(ledger,{a.profile:source_infos},data,2,'hardlink')
    return data


if __name__=='__main__': main()
