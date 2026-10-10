#!/usr/bin/env python3
"""Run all 745 synthetic validation scenes to qualify loader/metric cost, not quality."""
import argparse
from collections import Counter
import json
from pathlib import Path
import pickle
import time

import numpy as np
import torch
from mmengine.config import Config
from mmengine.hooks import Hook
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import HOOKS
from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner


@HOOKS.register_module(force=True)
class ValidationCost(Hook):
    priority='ABOVE_NORMAL'
    def __init__(self):
        self.rows=[];self.seen=set()
    def before_val(self,runner):
        torch.cuda.synchronize();self.start=time.perf_counter();self.previous=self.start
    def before_val_iter(self,runner,batch_idx,data_batch=None):
        torch.cuda.synchronize();self.begin=time.perf_counter()
        for sample in data_batch['data_samples']:
            self.seen.add(str(sample.metainfo['scene_id']))
    def after_val_iter(self,runner,batch_idx,data_batch=None,outputs=None):
        torch.cuda.synchronize();end=time.perf_counter()
        for sample in outputs:
            pred=sample.pred_instances_3d
            if not torch.isfinite(pred.bboxes_3d.tensor).all() or not torch.isfinite(pred.scores_3d).all():
                raise ValueError('Nonfinite validation prediction')
            if not ((pred.labels_3d>=0)&(pred.labels_3d<4)).all():
                raise ValueError('Prediction outside four-class contract')
        self.rows.append(dict(step=batch_idx,model_and_metric_ms=(end-self.begin)*1000,
                              total_ms=(end-self.previous)*1000))
        self.previous=end


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--profile',choices=['448x896','512x1056'],required=True)
    p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Use fresh output')
    a.output.mkdir(parents=True)
    torch.set_num_threads(4);torch.backends.cudnn.benchmark=True
    cfg=Config.fromfile(str(a.config));import_modules_from_strings(**cfg.custom_imports)
    cfg.work_dir=str(a.output);cfg.load_from=str(a.checkpoint);cfg.resume=False
    cfg.train_dataloader=None;cfg.train_cfg=None;cfg.optim_wrapper=None;cfg.param_scheduler=None
    cfg.test_dataloader=None;cfg.test_cfg=None;cfg.test_evaluator=None
    cfg.custom_hooks=[dict(type='ValidationCost')]
    cfg.default_hooks.checkpoint=None;cfg.default_hooks.logger.interval=100
    cfg.visualizer.vis_backends=[dict(type='LocalVisBackend')]
    cfg.val_dataloader.dataset.data_root=str(a.data)
    cfg.val_dataloader.dataset.data_prefix=dict(pts='',img='')
    info=a.data/'infos'/a.profile/'morai_base5_infos_val.pkl'
    cfg.val_dataloader.dataset.ann_file=str(info)
    cfg.val_dataloader.num_workers=4;cfg.val_dataloader.persistent_workers=True
    records=pickle.loads(info.read_bytes())['data_list']
    if len(records)!=745:raise ValueError('Expected all 745 validation records')
    cfg.dump(str(a.output/'resolved_validation.py'))
    started=time.perf_counter();runner=Base5AtomicRunner.from_cfg(cfg)
    metrics=runner.val();hook=next(h for h in runner.hooks if isinstance(h,ValidationCost))
    if len(hook.rows)!=745 or len(hook.seen)!=745 or not all(np.isfinite(v) for v in metrics.values()):
        raise ValueError('Incomplete validation or nonfinite metric')
    result=dict(valid=True,profile=a.profile,distinct_validation_frames=745,
        labels=dict(Counter(i['bbox_label_3d'] for r in records for i in r['instances'])),
        epoch_validation_seconds=time.perf_counter()-hook.start,
        total_cold_seconds=time.perf_counter()-started,
        precision='existing validation config: FP32, not AMP inference',
        scope='four-iteration smoke checkpoint; metric execution only, AP is not learned model performance',
        metric_keys=sorted(metrics),per_sample=hook.rows)
    (a.output/'validation_cost.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='per_sample'},indent=2))


if __name__=='__main__':main()
