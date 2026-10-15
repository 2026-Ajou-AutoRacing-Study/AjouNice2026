#!/usr/bin/env python3
"""One opt-in GPU wiring test; not training or an accuracy/latency benchmark."""
import argparse
import json
from pathlib import Path
import time

import torch
from mmengine.config import Config
from mmengine.dataset import pseudo_collate
from mmengine.registry import init_default_scope
from mmdet3d.registry import DATASETS,MODELS


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True)
    p.add_argument('--ann-file',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--memory-fraction',type=float,default=.35)
    p.add_argument('--backward',action='store_true')
    p.add_argument('--predict',action='store_true',help='Also exercise eval/decode without accuracy claims');a=p.parse_args()
    torch.cuda.set_per_process_memory_fraction(a.memory_fraction)
    torch.manual_seed(261006)
    cfg=Config.fromfile(a.config);init_default_scope('mmdet3d')
    dcfg=cfg.train_dataloader.dataset.copy();dcfg.ann_file=a.ann_file
    dataset=DATASETS.build(dcfg)
    # Exercise the added class, not only a legacy Vehicle sample.
    index=next((i for i in range(len(dataset)) if 3 in dataset.get_ann_info(i)['gt_labels_3d']),0)
    sample=pseudo_collate([dataset[index]])
    start=time.monotonic();report=dict(config=a.config,sample=index,classes=list(cfg.classes),
        backward_requested=a.backward,memory_fraction=a.memory_fraction,scope='single-batch wiring, not model performance')
    try:
        model=MODELS.build(cfg.model).cuda();model.init_weights();model.train(a.backward)
        batch=model.data_preprocessor(sample,training=True)
        with torch.autocast('cuda',dtype=torch.float16):
            losses=model(**batch,mode='loss')
            loss=sum(sum(v) if isinstance(v,list) else v for k,v in losses.items() if 'loss' in k)
        report['losses']={k:float(sum(v).detach() if isinstance(v,list) else v.detach()) for k,v in losses.items() if 'loss' in k}
        if not torch.isfinite(loss):raise ValueError('Nonfinite loss')
        if a.backward:
            loss.backward()
            bad=[n for n,p in model.named_parameters() if p.grad is not None and not torch.isfinite(p.grad).all()]
            report['nonfinite_gradients']=bad
            if bad:raise ValueError('Nonfinite gradients')
        if a.predict:
            model.eval()
            with torch.no_grad(),torch.autocast('cuda',dtype=torch.float16):
                predictions=model(**batch,mode='predict')
            counts=[]
            for prediction in predictions:
                instances=prediction.pred_instances_3d
                if not torch.isfinite(instances.bboxes_3d.tensor).all() or not torch.isfinite(instances.scores_3d).all():
                    raise ValueError('Nonfinite decoded predictions')
                if len(instances.labels_3d) and not ((instances.labels_3d>=0)&(instances.labels_3d<4)).all():
                    raise ValueError('Decoded class outside four-class contract')
                counts.append(len(instances.labels_3d))
            report['decoded_prediction_counts']=counts
        torch.cuda.synchronize();report.update(valid=True,elapsed_sec=time.monotonic()-start,
            peak_allocated_gib=torch.cuda.max_memory_allocated()/2**30,
            peak_reserved_gib=torch.cuda.max_memory_reserved()/2**30)
    except Exception as exc:
        report.update(valid=False,error=repr(exc),elapsed_sec=time.monotonic()-start)
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');raise
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
