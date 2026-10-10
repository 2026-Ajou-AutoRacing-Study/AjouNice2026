#!/usr/bin/env python3
"""Two real Base5 captures: full-model AMP, atomic save, interruption and resume.

Requires the portable 5K and CUDA environment. Bounded wiring test only, not a
training recipe or accuracy result. Outputs source tokens and full state proof.
"""
import argparse
import json
from pathlib import Path
import pickle

import torch
from mmengine.hooks import Hook
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import HOOKS

from prepare_base5_portable_dataset import build
from run_base5_direct_training import prepare
from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner


class PlannedStop(Exception): pass


@HOOKS.register_module(force=True)
class Base5SmokeStop(Hook):
    priority='LOWEST'
    def after_train_epoch(self,runner):
        if runner.epoch==0: raise PlannedStop('Bounded interruption after checkpoint')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True); p.add_argument('--init-checkpoint',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists(): p.error('Use a fresh output')
    a.output.mkdir(parents=True)
    source=a.data/'infos/448x896'
    info=pickle.loads((source/'morai_base5_infos_train.pkl').read_bytes())
    records=info['data_list']; labels=lambda r:{i['bbox_label_3d'] for i in r['instances']}
    selected=[next(r for r in records if 3 in labels(r))]
    missing=set(range(4))-labels(selected[0])
    selected.append(max((r for r in records if r['token']!=selected[0]['token']),key=lambda r:len(labels(r)&missing)))
    if set.union(*(labels(r) for r in selected)) != set(range(4)):
        raise ValueError('Selected two captures do not exercise all four classes')
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
    data=a.output/'mini_portable'; build(ledger,{'448x896':source_infos},data,2,'hardlink')
    args=argparse.Namespace(data=data,profile='448x896',config=a.config,work_dir=a.output/'training',
        epochs=2,batch=1,accumulate=1,workers=0,lr=1e-4,warmup_updates=1,seed=261010,
        init_checkpoint=a.init_checkpoint,allow_random_init=False,resume=None,disable_val=True,wandb=False)
    cfg,manifest=prepare(args); import_modules_from_strings(**cfg.custom_imports)
    cfg.custom_hooks.append(dict(type='Base5SmokeStop'))
    try: Base5AtomicRunner.from_cfg(cfg).train()
    except PlannedStop: pass
    else: raise ValueError('Planned interruption not reached')
    first=torch.load(args.work_dir/'epoch_1.pth',map_location='cpu',weights_only=False)
    args.resume='auto'; resumed_cfg,_=prepare(args)
    Base5AtomicRunner.from_cfg(resumed_cfg).train()
    final=torch.load(args.work_dir/'epoch_2.pth',map_location='cpu',weights_only=False)
    evidence=json.loads((args.work_dir/'resume_start.json').read_text())
    if first['meta']['iter']!=2 or final['meta']['iter']!=4 or not evidence['resumed']:
        raise ValueError('Resume iteration mismatch')
    if not evidence['optimizer_steps'] or min(evidence['optimizer_steps'])<=0:
        raise ValueError('Optimizer did not update before resume')
    if any(not torch.isfinite(v).all() for v in final['state_dict'].values() if v.is_floating_point()):
        raise ValueError('Nonfinite trained weights')
    report=dict(valid=True,captures=[r['token'] for r in selected],classes=list(range(4)),
        first_checkpoint_iteration=2,final_checkpoint_iteration=4,resume_start=evidence,
        checkpoint_sha_contract=manifest['contract_sha256'],
        scope='two real captures, four AMP iterations; not quality or converged model',
        atomic_partial_files=list(map(str,args.work_dir.glob('*.partial-*'))))
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
