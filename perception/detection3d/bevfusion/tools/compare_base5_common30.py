#!/usr/bin/env python3
"""Export learned predictions and compare models on identical Base5 validation GT.

Preserves each checkpoint's own input pipeline and spatial architecture. No
training, production threshold changes, W&B mutations or source migration.
Primary scope: LiDAR-frame planar center distance <30m AND the common E25 box
range. The square [-30,30]m scope is reported separately, not called radial 30m.
"""
import argparse
import copy
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

CLASSES = ('vehicle', 'pedestrian', 'obstacle')
COMMON_BOX = (-30., -30., -3., 30., 30., 2.)
if len(Path(__file__).resolve().parents) > 4:
    repo = Path(__file__).resolve().parents[4]
    if (repo / 'perception').is_dir():
        sys.path.insert(0, str(repo))


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(8 * 1024**2), b''):
            h.update(chunk)
    return h.hexdigest()


def adapt_intensity(points, scale):
    """Explicit input-only adapter; never mutate raw files, XYZ or GT.

    Supports NumPy test arrays and the loader's Torch tensor without depending
    on Torch for CPU scoring. A 255 multiplier requires normalized input.
    """
    if scale not in (1., 255., 1./255.):
        raise ValueError('Only identity or the documented 0..1/0..255 adapters are allowed')
    if points.ndim != 2 or points.shape[1] < 4:
        raise ValueError('XYZ plus intensity required')
    if scale != 1.:
        values = points[:, 3]
        upper = 1. if scale == 255. else 255.
        if len(values) and not (bool((values >= 0).all()) and bool((values <= upper).all())):
            raise ValueError('Intensity adapter requires finite input in its explicit source range')
        points = points.clone() if hasattr(points, 'clone') else points.copy()
        points[:, 3] *= scale
    return points


def gt_identity(rows):
    fields = ('token', 'gt_boxes', 'gt_labels', 'gt_points', 'gt_assets')
    payload = [{k: r[k] for k in fields} for r in rows]
    return hashlib.sha256(json.dumps(payload, sort_keys=True,
                                    allow_nan=False).encode()).hexdigest()


def validate_payload(payload):
    rows = payload['rows']
    if tuple(payload['classes']) != CLASSES:
        raise ValueError('Prediction classes differ from the common contract')
    if len(rows) != payload['frames'] or len({r['token'] for r in rows}) != len(rows):
        raise ValueError('Prediction frame count/tokens are corrupt')
    if payload['gt_identity'] != gt_identity(rows):
        raise ValueError('Prediction GT identity is corrupt')
    for row in rows:
        for prefix in ('gt', 'pred'):
            boxes = np.asarray(row[prefix + '_boxes'], dtype=float).reshape(-1, 7)
            keys = ('labels', 'points', 'assets') if prefix == 'gt' else ('labels', 'scores')
            if any(len(row[prefix + '_' + key]) != len(boxes) for key in keys):
                raise ValueError('Prediction/GT fields disagree in length')
            if not np.isfinite(boxes).all():
                raise ValueError('Nonfinite boxes in prediction export')
            labels = np.asarray(row[prefix + '_labels'])
            if not np.isin(labels, range(len(CLASSES))).all():
                raise ValueError('Unexpected prediction/GT labels')
        scores = np.asarray(row['pred_scores'])
        if not np.isfinite(scores).all() or not ((scores >= 0) & (scores <= 1)).all():
            raise ValueError('Invalid prediction scores')


def scope(row, radial=True):
    out = dict(row)
    for prefix in ('gt', 'pred'):
        boxes = np.asarray(row[prefix + '_boxes'], dtype=np.float64).reshape(-1, 7)
        mask = ((boxes[:, :3] >= COMMON_BOX[:3]) &
                (boxes[:, :3] <= COMMON_BOX[3:])).all(axis=1)
        if radial:
            mask &= np.linalg.norm(boxes[:, :2], axis=1) < 30.
        for k, v in row.items():
            if k.startswith(prefix + '_'):
                out[k] = np.asarray(v)[mask]
        out[prefix + '_boxes'] = boxes[mask]
    return out


def fixed_matches(row, class_index, score, tolerance=1.):
    """Match against ALL eligible GT before attributing recall to support strata."""
    gt = np.flatnonzero(row['gt_labels'] == class_index)
    pred = np.flatnonzero((row['pred_labels'] == class_index) &
                         (row['pred_scores'] >= score))
    pred = pred[np.argsort(-row['pred_scores'][pred], kind='stable')]
    taken = set()
    for p in pred:
        available = [int(g) for g in gt if int(g) not in taken]
        if not available:
            break
        distances = np.linalg.norm(row['gt_boxes'][available, :2] -
                                   row['pred_boxes'][p, :2], axis=1)
        best = int(np.argmin(distances))
        if distances[best] < tolerance:
            taken.add(available[best])
    return taken, len(pred)


def evaluate(rows, radial=True):
    # Load the existing pure-NumPy evaluator directly: package __init__ also
    # imports unrelated Torch shape-merging utilities on the local CPU host.
    root = repo if 'repo' in globals() and (repo/'perception').is_dir() else Path.cwd()
    path = root/'perception/detection3d/common/evaluation/morai_detection.py'
    name = '_base5_common30_metric'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    metric = sys.modules[name]
    _evaluate_center, _filter_range = metric._evaluate_center, metric._filter_range
    DEFAULT_CENTER_THRESHOLDS = metric.DEFAULT_CENTER_THRESHOLDS
    samples = [scope(r, radial) for r in rows]
    metrics = _evaluate_center(samples, list(CLASSES), DEFAULT_CENTER_THRESHOLDS,
                               2., .1, .1, 'center', include_tp=True,
                               include_operating_points=True)
    for lo, hi in ((0., 10.), (10., 20.), (20., 30.)):
        metrics.update(_evaluate_center(_filter_range(samples, (lo, hi)),
            list(CLASSES), DEFAULT_CENTER_THRESHOLDS, 2., .1, .1,
            f'range/{lo:g}_{hi:g}m', include_tp=False,
            include_operating_points=True))
    strata = {}
    for ci, name in enumerate(CLASSES):
        for score in (.1, .15, .2, .4):
            buckets = {}
            totals = {'gt': 0, 'tp': 0, 'predictions': 0}
            for s in samples:
                matched, prediction_count = fixed_matches(s, ci, score)
                totals['predictions'] += prediction_count
                for g in np.flatnonzero(s['gt_labels'] == ci):
                    distance = np.linalg.norm(s['gt_boxes'][g, :2])
                    band = f'{int(distance // 10) * 10}_{int(distance // 10) * 10 + 10}m'
                    support = 'ge5points' if s['gt_points'][g] >= 5 else 'lt5points'
                    for key in ('all', band, support, band + '/' + support,
                                'asset/' + str(s['gt_assets'][g])):
                        b = buckets.setdefault(key, {'gt': 0, 'tp': 0})
                        b['gt'] += 1
                        b['tp'] += int(int(g) in matched)
                    totals['gt'] += 1
                    totals['tp'] += int(int(g) in matched)
            for b in buckets.values():
                b['recall'] = b['tp'] / b['gt'] if b['gt'] else None
            totals['precision'] = totals['tp'] / totals['predictions'] if totals['predictions'] else None
            totals['recall'] = totals['tp'] / totals['gt'] if totals['gt'] else None
            strata[f'{name}/score{score:g}_1m'] = dict(totals=totals, strata=buckets)
    return dict(metrics=metrics, support_recall=strata)


def validate_runtime_switches(args):
    parts = tuple(getattr(args, 'compile_parts', ()))
    if set(parts) - {'camera', 'neck', 'bev'}:
        raise ValueError('Unknown compilation part')
    if getattr(args, 'eager_camera_numerics', False) and 'camera' not in parts:
        raise ValueError('Eager camera numerics requires explicit camera compilation')
    if getattr(args, 'compile_reference_layout', False) and not parts:
        raise ValueError('Reference layout requires explicit compilation')
    if getattr(args, 'camera_cuda_graph_probe', False) and 'camera' in parts:
        raise ValueError('Choose a camera graph OR camera compilation')


def infer(args):
    validate_runtime_switches(args)
    import pickle
    import torch
    import cv2
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet3d.apis import init_model
    from mmdet3d.utils import register_all_modules
    from app.view_geometry import restore_view_geometry


    if args.output.exists():
        raise ValueError('Fresh output required; never overwrite learned predictions')
    args.output.mkdir(parents=True)
    torch.set_num_threads(4)
    cv2.setNumThreads(1)
    torch.manual_seed(261012)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    cfg = Config.fromfile(str(args.config))
    register_all_modules(init_default_scope=True)
    import perception.detection3d.bevfusion.plugin.morai_base5_dataset  # noqa: F401
    if list(cfg.metainfo.classes) != list(CLASSES):
        raise ValueError('Three-class model required; no checkpoint label remapping')
    cfg.model.img_backbone.init_cfg = None
    if 'compile_camera' in cfg.model:
        cfg.model.compile_camera = False
    loader_cfg = copy.deepcopy(cfg.val_dataloader)
    loader_cfg.batch_size = 1
    loader_cfg.num_workers = 2
    loader_cfg.persistent_workers = True
    loader_cfg.sampler = dict(type='DefaultSampler', shuffle=False)
    loader_cfg.dataset.update(type=args.dataset_type,
        data_root=str(args.data), ann_file=str(args.info), data_prefix=dict(pts='', img=''),
        test_mode=True, load_eval_anns=True, filter_empty_gt=False)
    loader = Runner.build_dataloader(loader_cfg)
    with args.info.open('rb') as f:
        authored = pickle.load(f)['data_list']
    if len(loader.dataset) != len(authored) or len(authored) != args.expected_frames:
        raise ValueError('Validation frame count differs from the explicit contract')
    checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
    state = checkpoint['state_dict']
    model = init_model(cfg, None, device='cuda:0')
    model.load_state_dict(state, strict=True)
    restore_view_geometry(model.view_transform)
    model.eval()
    optimization = None
    extra_parallel = None
    if args.amp or args.compile_parts or args.parallel_camera or args.camera_cuda_graph_probe:
        from app.inference_optimization import InferenceOptimization, validate_base5_runtime
        validate_base5_runtime(cfg)
        if args.compile_parts or args.parallel_camera or args.camera_cuda_graph_probe:
            optimization = InferenceOptimization(model, args.compile_parts,
                camera_cuda_graph=args.camera_cuda_graph_probe,
                eager_camera_numerics=args.eager_camera_numerics,
                reference_layout=args.compile_reference_layout,
                parallel_camera=args.parallel_camera and not args.camera_cuda_graph_probe)
            if args.camera_cuda_graph_probe and args.parallel_camera:
                from app.inference_parallel import ParallelBranches
                extra_parallel = ParallelBranches(model, 'camera_side')
    rows = []
    elapsed = []
    started = time.monotonic()
    with torch.inference_mode():
        for i, batch in enumerate(loader):
            # Det3DDataset replaces authored sample_idx with its integer index.
            # Keep the immutable token rather than silently joining on that index.
            if int(batch['data_samples'][0].sample_idx) != i:
                raise ValueError('Validation sampler changed frame order')
            source = authored[i]
            token = str(source['token'])
            ann = batch['data_samples'][0].eval_ann_info
            gt = ann['gt_bboxes_3d'].tensor.cpu().numpy()
            expected = np.asarray([r['bbox_3d'] for r in source['instances']],
                                  dtype=np.float32).reshape(-1, 7)
            expected[:, 2] -= expected[:, 5] / 2
            if not np.allclose(gt, expected, rtol=0, atol=1e-5):
                raise ValueError('Dataset pipeline changed canonical validation GT')
            batch['inputs']['points'] = [adapt_intensity(p, args.intensity_scale)
                                          for p in batch['inputs']['points']]
            torch.cuda.synchronize()
            t = time.monotonic()
            with torch.autocast('cuda', enabled=args.amp, dtype=torch.float16):
                pred = model.test_step(batch)[0].pred_instances_3d
            boxes = pred.bboxes_3d.tensor.detach().cpu().numpy()
            scores = pred.scores_3d.detach().cpu().numpy()
            labels = pred.labels_3d.detach().cpu().numpy()
            torch.cuda.synchronize()
            elapsed.append(time.monotonic() - t)
            if not np.isfinite(boxes).all() or not np.isfinite(scores).all():
                raise ValueError('Nonfinite learned predictions')
            rows.append(dict(token=token, gt_boxes=gt.tolist(),
                gt_labels=ann['gt_labels_3d'].tolist(),
                gt_points=[r['num_lidar_pts'] for r in source['instances']],
                gt_assets=[r.get('asset', 'legacy_unknown') for r in source['instances']],
                pred_boxes=boxes[:, :7].tolist(), pred_labels=labels.tolist(),
                pred_scores=scores.tolist()))
            if (i + 1) % 100 == 0:
                print(json.dumps(dict(model=args.name, frames=i+1, total=len(loader),
                    seconds=round(time.monotonic()-started, 1))), flush=True)
    if len({r['token'] for r in rows}) != args.expected_frames:
        raise ValueError('Duplicate or missing validation tokens')
    payload = dict(name=args.name, checkpoint_sha256=digest(args.checkpoint),
        info_sha256=digest(args.info), config_sha256=digest(args.config),
        script_sha256=digest(__file__),
        gt_identity=gt_identity(rows), frames=len(rows), classes=CLASSES,
        gpu=torch.cuda.get_device_name(), torch_version=str(torch.__version__),
        strict_state_load=True,
        inference_precision=('AMP; TF32 OFF; cuDNN benchmark OFF' if args.amp else
                             'FP32; AMP/TF32 OFF; cuDNN benchmark OFF'),
        inference_optimization=(dict(optimization.report,diagnostic_graph_parallel=True)
            if extra_parallel else optimization.report if optimization else {}),
        input_pipeline=loader_cfg.dataset.pipeline,
        point_intensity_scale=args.intensity_scale,
        point_intensity_adapter='input-only; XYZ/raw files/GT unchanged',
        inference_ms_p50=float(np.median(elapsed[10:])*1000),
        inference_timing_scope='test_step including transfer/voxelization; not ROS latency certification',
        rows=rows)
    with gzip.open(args.output/'predictions.json.gz', 'wt') as f:
        json.dump(payload, f, allow_nan=False)
    summary = {k:v for k,v in payload.items() if k != 'rows'}
    (args.output/'receipt.json').write_text(json.dumps(summary, indent=2)+'\n')
    if extra_parallel:
        extra_parallel.disable()
    if optimization:
        optimization.disable()
    print(json.dumps(dict(event='inference_complete', name=args.name, frames=len(rows),
                         gt_identity=payload['gt_identity'])), flush=True)


def compare(args):
    if args.output.exists():
        raise ValueError('Fresh comparison output required')
    args.output.mkdir(parents=True)
    report = dict(primary='Common box intersected with planar LiDAR radius <30m',
        secondary='Common square box [-30,30]m, not a radial distance contract',
        support_policy='Recall strata use matches against all eligible GT; sparse GT is never silently turned into a false positive',
        comparison_script_sha256=digest(__file__), models={})
    identity = None
    for p in args.predictions:
        with gzip.open(p, 'rt') as f:
            payload = json.load(f)
        validate_payload(payload)
        if identity is not None and payload['gt_identity'] != identity:
            raise ValueError('Models did not evaluate identical ordered scenes/GT')
        identity = payload['gt_identity']
        name = payload['name']
        if name in report['models']:
            raise ValueError('Duplicate model name')
        report['models'][name] = dict(receipt={k:v for k,v in payload.items() if k != 'rows'},
            radial30=evaluate(payload['rows']), square30=evaluate(payload['rows'], False))
    report['gt_identity'] = identity
    (args.output/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for name, value in report['models'].items():
        m = value['radial30']['metrics']
        print(name, {c: round(m[f'center/{c}_mean_AP']*100, 2) for c in CLASSES}, flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    worker = sub.add_parser('infer')
    for key in ('config', 'checkpoint', 'data', 'info', 'output'):
        worker.add_argument('--'+key, type=Path, required=True)
    worker.add_argument('--name', required=True)
    worker.add_argument('--dataset-type', choices=['MoraiBase5PedestrianDataset', 'MoraiBase3Dataset'],
                        default='MoraiBase5PedestrianDataset')
    worker.add_argument('--expected-frames', type=int, default=745)
    worker.add_argument('--intensity-scale', type=float, choices=[1., 255., 1./255.], default=1.,
                        help='Explicit checkpoint-native intensity adapter; ROS raw -> Base5 uses 0.00392156862745098')
    worker.add_argument('--amp', action='store_true', help='Base5-only execution diagnostic; default remains FP32')
    worker.add_argument('--compile-parts', nargs='*', choices=['camera','neck','bev'], default=[])
    worker.add_argument('--parallel-camera', action='store_true')
    worker.add_argument('--eager-camera-numerics', action='store_true')
    worker.add_argument('--compile-reference-layout', action='store_true')
    worker.add_argument('--camera-cuda-graph-probe', action='store_true',
                        help='Experimental offline probe only; does not qualify a production combination')
    score = sub.add_parser('compare')
    score.add_argument('--predictions', type=Path, nargs='+', required=True)
    score.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    infer(args) if args.action == 'infer' else compare(args)


if __name__ == '__main__':
    main()
