#!/usr/bin/env python3
"""Bounded, read-only checkpoint/query audit; never updates model parameters.

Batch-stat BN is a diagnostic control, not a validation or deployment mode.
Output records all selected samples and preserves class-conditioned failures.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch
from mmengine.config import Config
from mmengine.runner import Runner
from mmengine.registry import init_default_scope
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import MODELS


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--samples', type=int, default=64)
    p.add_argument('--running-only', action='store_true',
                   help='Skip the diagnostic-only BN batch-stat control')
    a = p.parse_args()
    torch.set_num_threads(4)
    cfg = Config.fromfile(str(a.config))
    import_modules_from_strings(**cfg.custom_imports)
    init_default_scope('mmdet3d')
    model = MODELS.build(cfg.model)
    model.init_weights()
    checkpoint = torch.load(a.checkpoint, map_location='cpu', weights_only=False)
    model.load_state_dict(checkpoint['state_dict'], strict=True)
    model._base5_compile_camera = False  # compare eager outputs, not cold compile cost
    model.cuda().eval()
    dl = cfg.val_dataloader
    dl.num_workers = 2
    dl.persistent_workers = False
    loader = Runner.build_dataloader(dl)
    indices = set(np.linspace(0, len(loader.dataset)-1, min(a.samples, len(loader.dataset)), dtype=int))
    rows = []
    with torch.no_grad():
        modes = ('running_stats',) if a.running_only else ('running_stats', 'batch_stats_control')
        for mode in modes:
            for module in model.modules():
                if isinstance(module, torch.nn.modules.batchnorm._BatchNorm):
                    module.train(mode == 'batch_stats_control')
                    module.track_running_stats = mode == 'running_stats'
            for i, batch in enumerate(loader):
                if i not in indices:
                    continue
                processed = model.data_preprocessor(batch, training=False)
                inputs, samples = processed['inputs'], processed['data_samples']
                feats = model.extract_feat(inputs, [s.metainfo for s in samples])
                head = model.bbox_head
                preds = head(feats, [s.metainfo for s in samples])
                pred = preds[0][0]
                dense = pred['dense_heatmap'].sigmoid()
                query_labels = head.query_labels[0].detach().cpu().tolist()
                # Coder decode mutates its inputs. Clone every tensor to retain
                # actual network logits/log-dimensions for the diagnostic.
                decoded_input = tuple([{key: value.clone() for key, value in layer.items()}
                                       for layer in levels] for levels in preds)
                result = head.predict_by_feat(decoded_input, [s.metainfo for s in samples])[0]
                ann = samples[0].eval_ann_info
                gt = ann['gt_bboxes_3d'].tensor.to(dense.device)
                labels = torch.as_tensor(ann['gt_labels_3d'], device=dense.device)
                pitch = head.train_cfg.voxel_size[0]*head.train_cfg.out_size_factor
                origin = gt.new_tensor(head.train_cfg.point_cloud_range[:2])
                cells = ((gt[:, :2]-origin)/pitch).long()
                stats = []
                for j in range(len(gt)):
                    label = int(labels[j])
                    x, y = map(int, cells[j])
                    hm = dense[0, label]
                    patch = hm[max(0,y-2):y+3, max(0,x-2):x+3]
                    boxes = result.bboxes_3d.tensor
                    distance = torch.linalg.vector_norm(boxes[:, :2]-gt[j, :2],dim=1)
                    same = result.labels_3d == label
                    close = same & (distance <= 1.)
                    best_close_score = (float(result.scores_3d[close].max())
                                        if close.any() else None)
                    class_scores = result.scores_3d[same]
                    best_close_rank = (int((class_scores > best_close_score).sum())+1
                                       if best_close_score is not None else None)
                    stats.append(dict(label=label, range_m=float(gt[j,:2].norm()),
                        xy=gt[j,:2].cpu().tolist(), dense_at_gt=float(hm[y,x]),
                        dense_near_gt=float(patch.max()),
                        nearest_any_m=float(distance.min()) if len(distance) else None,
                        nearest_same_m=float(distance[same].min()) if same.any() else None,
                        same_class_predictions=int(same.sum()),
                        best_same_within1m_score=best_close_score,
                        best_same_within1m_class_rank=best_close_rank))
                rows.append(dict(mode=mode,index=i,token=str(samples[0].sample_idx),
                    query_counts=[query_labels.count(c) for c in range(4)],gt=stats,
                    query_logits_min=float(pred['heatmap'].min()),
                    query_logits_max=float(pred['heatmap'].max()),
                    query_log_dim_max=float(pred['dim'].max()),
                    dense_min=float(dense.min()),dense_max=float(dense.max()),
                    box_finite=bool(torch.isfinite(result.bboxes_3d.tensor).all()),
                    scores_max=float(result.scores_3d.max()) if len(result.scores_3d) else 0))
                print(json.dumps(rows[-1]),flush=True)
    report = dict(checkpoint=str(a.checkpoint),samples_per_mode=len(indices),
                  classes=list(cfg.classes),rows=rows,
                  scope='No gradients or optimizer updates; batch-stat control is NOT validation AP')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':
    main()
