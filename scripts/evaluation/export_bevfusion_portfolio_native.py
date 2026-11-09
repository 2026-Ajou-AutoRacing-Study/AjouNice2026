#!/usr/bin/env python3
"""Same-GPU native exports; learned weights, GT and production are unchanged."""
import argparse
from copy import deepcopy
import gzip
import json
from pathlib import Path
import pickle
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'docker/bevfusion'),
               str(ROOT/'perception/detection3d/bevfusion/tools'),
               str(ROOT/'perception/detection3d/bevfusion/scripts')]
from compare_base5_common30 import adapt_intensity, gt_identity, digest
from qualify_base5_camera_tensorrt import distribution, config_closure


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('config', 'checkpoint', 'info', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    p.add_argument('--backend', choices=['fp32', 'amp', 'trt_head'], required=True)
    p.add_argument('--name', required=True)
    p.add_argument('--engine', type=Path)
    p.add_argument('--metadata', type=Path)
    p.add_argument('--expected-frames', type=int, default=4614)
    a = p.parse_args()
    if a.output.exists():
        raise ValueError('Fresh export directory required')
    a.output.mkdir(parents=True)
    import cv2
    import numpy as np
    import torch
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet3d.apis import init_model
    from mmdet3d.utils import register_all_modules
    from app.view_geometry import restore_view_geometry
    from app.inference_optimization import InferenceOptimization
    from app.camera_tensorrt import install_camera_tensorrt
    from app.head_runtime_optimization import install_head_runtime_optimization
    torch.set_num_threads(4)
    cv2.setNumThreads(1)
    torch.manual_seed(261109)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    cfg = Config.fromfile(str(a.config), lazy_import=False)
    cfg.model.img_backbone.init_cfg = None
    cfg.model.compile_camera = False
    register_all_modules(init_default_scope=True)
    import perception.detection3d.bevfusion.plugin.morai_base5_dataset  # noqa
    loader_cfg = deepcopy(cfg.val_dataloader)
    loader_cfg.update(batch_size=1, num_workers=2, persistent_workers=True,
                      sampler=dict(type='DefaultSampler', shuffle=False))
    loader_cfg.dataset.update(type='MoraiBase5PedestrianDataset', data_root=str(ROOT),
        ann_file=str(a.info), data_prefix=dict(pts='',img=''), test_mode=True,
        load_eval_anns=True, filter_empty_gt=False)
    loader = Runner.build_dataloader(loader_cfg)
    authored = pickle.loads(a.info.read_bytes())['data_list']
    if len(authored) != a.expected_frames or len(loader.dataset) != a.expected_frames:
        raise ValueError('Validation size changed')
    sources = config_closure(a.config) | {Path(__file__), a.checkpoint, a.info}
    for folder in ('docker/bevfusion/app', 'perception/detection3d/bevfusion/plugin'):
        sources.update((ROOT/folder).rglob('*.py'))
    pins = {str(f): digest(f) for f in sources}
    model = init_model(cfg, None, device='cuda:0')
    model.load_state_dict(torch.load(a.checkpoint, map_location='cpu', weights_only=False)['state_dict'], strict=True)
    restore_view_geometry(model.view_transform)
    model.eval()
    head = camera = parallel = None
    if a.backend == 'trt_head':
        if not a.engine or not a.metadata:
            raise ValueError('Local engine and metadata required')
        pins.update({str(f):digest(f) for f in (a.engine,a.metadata)})
        camera = install_camera_tensorrt(model,a.engine,metadata_path=a.metadata,
            expected_contract=dict(checkpoint_sha256=digest(a.checkpoint),config_sha256=digest(a.config)))
        head = install_head_runtime_optimization(model.bbox_head, proposal_topk=False,
                                                attention_sdpa=True, cache_bev_pos=True)
    if a.backend != 'fp32':
        parallel = InferenceOptimization(model, (), parallel_camera=True)
    rows, times = [], []
    start = time.monotonic()
    with torch.inference_mode():
        for i,batch in enumerate(loader):
            if int(batch['data_samples'][0].sample_idx) != i:
                raise ValueError('Sampler reordered frames')
            source = authored[i]
            ann = batch['data_samples'][0].eval_ann_info
            gt = ann['gt_bboxes_3d'].tensor.cpu().numpy()
            expected = np.asarray([r['bbox_3d'] for r in source['instances']],dtype=np.float32).reshape(-1,7)
            expected[:,2] -= expected[:,5]/2
            if not np.allclose(gt,expected,rtol=0,atol=1e-5):
                raise ValueError('Pipeline altered canonical GT')
            torch.cuda.synchronize()
            at=time.perf_counter()
            with torch.autocast('cuda',enabled=a.backend!='fp32',dtype=torch.float16):
                pred=model.test_step(batch)[0].pred_instances_3d
            boxes=pred.bboxes_3d.tensor.detach().cpu().numpy()
            scores=pred.scores_3d.detach().cpu().numpy()
            labels=pred.labels_3d.detach().cpu().numpy()
            torch.cuda.synchronize()
            times.append((time.perf_counter()-at)*1000)
            if not np.isfinite(boxes).all() or not np.isfinite(scores).all():
                raise ValueError('Nonfinite predictions')
            rows.append(dict(token=str(source['token']),gt_boxes=gt.tolist(),
                gt_labels=ann['gt_labels_3d'].tolist(),
                gt_points=[r['num_lidar_pts'] for r in source['instances']],
                gt_assets=[r.get('asset','legacy_unknown') for r in source['instances']],
                pred_boxes=boxes[:,:7].tolist(),pred_labels=labels.tolist(),pred_scores=scores.tolist()))
            if (i+1)%100==0:
                print(json.dumps(dict(name=a.name,frames=i+1,total=len(authored),seconds=time.monotonic()-start)),flush=True)
    if pins != {f:digest(f) for f in pins}:
        raise ValueError('Source/checkpoint/info/engine changed during export')
    receipt=dict(name=a.name,frames=len(rows),classes=['vehicle','pedestrian','obstacle'],
        checkpoint_sha256=digest(a.checkpoint),info_sha256=digest(a.info),config_sha256=digest(a.config),
        gt_identity=gt_identity(rows),strict_state_load=True,gpu=torch.cuda.get_device_name(),
        backend=a.backend,torch_version=str(torch.__version__),source_sha256=pins,
        inference_precision='FP32' if a.backend=='fp32' else 'AMP FP16',
        point_intensity_scale=1.,timing_ms=distribution(times[10:]),timing_raw_ms=times,
        peak_allocated_bytes=torch.cuda.max_memory_allocated(),
        timing_scope='Native loader test_step + CPU output; file decode excluded, not ROS/LAN.',
        head_report=head.report if head else None,production_promoted=False)
    with gzip.open(a.output/'predictions.json.gz','wt') as f:
        json.dump(dict(receipt,rows=rows),f,allow_nan=False)
    (a.output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    (a.output/'COMPLETE.json').write_text(json.dumps(dict(receipt_sha256=digest(a.output/'receipt.json')))+'\n')
    if parallel: parallel.disable()
    if head: head.close()
    if camera: camera.close()
    print(json.dumps(dict(event='complete',name=a.name,frames=len(rows),timing=receipt['timing_ms'])),flush=True)


if __name__ == '__main__':
    main()
