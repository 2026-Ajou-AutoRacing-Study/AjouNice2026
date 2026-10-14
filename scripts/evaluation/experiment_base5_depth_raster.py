#!/usr/bin/env python3
"""Offline E24 depth-hint repeatability and opt-in policy qualification.

Different complete forwards include natural A/A variation. No geometry, raw
input, GT, production default or checkpoint is changed. Local5060Ti runtime
is not target4090 ROS latency. Hooks add cost and are not profiling evidence.
"""

import argparse
from collections import defaultdict
import copy
import gzip
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from experiment_base5_proposal_peaks import ROOT, CAMERAS, bootstrap_source, digest, uniform_indices
from analyze_base5_proposal_peaks import distribution, paired, summarize


def delta(reference, current):
    error = np.abs(reference - current)
    quantiles = np.percentile(error, (50, 90, 95, 100))
    return dict(shape=list(reference.shape), bitwise=bool(np.array_equal(reference, current)),
                different=int(np.count_nonzero(error)), n=int(error.size), mean=float(error.mean(dtype=np.float64)),
                **dict(zip(('p50', 'p90', 'p95', 'max'), (float(v) for v in quantiles))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('inputs', 'output', 'sensor', 'config', 'checkpoint'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--diagnostic-only', action='store_true')
    parser.add_argument('--projection-cpu-only', action='store_true')
    parser.add_argument('--diagnostic-case', action='append', default=[])
    parser.add_argument('--diagnostic-repeats', type=int, default=3)
    parser.add_argument('--limit', type=int, default=0)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    bootstrap_source()
    import cv2
    import torch
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    from perception.detection3d.bevfusion.depth_raster import project_positive_camera_z, reduce_depth_pixels
    from perception.detection3d.bevfusion.plugin.morai_separated_depth_lss import MoraiSeparatedRectangularDepthLSSTransform
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True)
    manifest = json.loads(args.inputs.read_text())
    calibration = load_sensor4_calibration(args.sensor, ['camera_' + c for c in CAMERAS])

    def collision_stats(coords, image_size):
        cameras = []
        for ci, (xy, z) in enumerate(coords[0]):
            index = xy[:, 0]*image_size[1] + xy[:, 1]
            unique, counts = torch.unique(index, return_counts=True)
            nearest = reduce_depth_pixels(xy, z, image_size)
            farthest = reduce_depth_pixels(xy, z, image_size, 'farthest_positive')
            meaningful = farthest > nearest
            cameras.append(dict(camera=CAMERAS[ci], samples=len(index), unique_pixels=len(unique),
                repeated_destination_pixels=int((counts > 1).sum()), extra_samples=len(index)-len(unique),
                differing_depth_collision_pixels=int(meaningful.sum()),
                collision_depth_gap_m=distribution((farthest-nearest)[meaningful].float().cpu().tolist())))
        return cameras

    if args.projection_cpu_only:
        from mmengine.config import Config
        from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
        cfg = Config.fromfile(str(args.config))
        preprocessor = Base3OnlinePreprocessor(calibration, camera_workers=4,
                                              **preprocessing_options_from_config(cfg))
        grouped = defaultdict(list)
        for row in manifest['rows']:
            grouped[row['case']].append(row)
        selected = [values[i] for values in grouped.values() for i in uniform_indices(len(values), args.limit or 16)]
        outputs = []
        try:
            for sequence, row in enumerate(selected):
                points = np.fromfile(row['point_path'], '<f4').reshape(-1, 4)
                blobs = {key: Path(path).read_bytes() for key, path in row['images'].items()}
                preprocessor.point_intensity_source = 'native' if row['source'] == 'native' else 'ros255'
                batch = preprocessor.prepare(points, blobs, sequence, timestamp_sec=row['stamp'])
                meta = batch['data_samples'][0].metainfo
                def mat(key):
                    return torch.tensor(np.asarray(meta[key]), dtype=torch.float32)[None]
                projection = project_positive_camera_z(batch['inputs']['points'], mat('lidar2img'),
                    mat('img_aug_matrix'), mat('lidar_aug_matrix'), preprocessor.final_dim)
                outputs.append(dict(token=row['token'], source=row['source'], case=row['case'],
                    raw_point_sha256=digest(row['point_path']), filtered_points=len(batch['inputs']['points'][0]),
                    cameras=collision_stats(projection, preprocessor.final_dim)))
            result = dict(scope='CPU float32 projected actual preprocessor/calibration; not a GPU nondeterminism proof.',
                          input_sha256=digest(args.inputs), rows=outputs)
            (args.output / 'projection_collisions.json').write_text(json.dumps(result, indent=2) + '\n')
            print(json.dumps(dict(cpu_projection_frames=len(outputs), completed=True)), flush=True)
        finally:
            preprocessor.close()
        return
    model = BevFusionModel(str(args.config), str(args.checkpoint), calibration,
        use_amp=True, preprocess_workers=4, warmup_iterations=2, base5_runtime=True,
        parallel_camera=True, point_intensity_source='ros255')
    transform = model.model.view_transform
    actual_forward = transform.forward
    original_forward = MoraiSeparatedRectangularDepthLSSTransform.forward.__get__(transform, type(transform))
    stage_tensors, projection_args = {}, {}
    hooks = []
    diagnostics_active = False

    def capture(name, pre=False):
        def hook(_module, call_args, output=None):
            if not diagnostics_active:
                return
            value = call_args[0] if pre else output
            if isinstance(value, (tuple, list)):
                value = value[0]
            stage_tensors[name] = value.detach().clone()
        return hook

    hooks.append(transform.dtransform.register_forward_pre_hook(capture('camera_depth_hint', True)))
    for name, module in (('camera_bev', transform), ('lidar_sparse_bev', model.model.pts_middle_encoder),
                         ('fusion_bev', model.model.fusion_layer), ('head_input', model.model.pts_neck)):
        hooks.append(module.register_forward_hook(capture(name)))

    def projection_capture(_module, call_args):
        if diagnostics_active:
            projection_args.clear()
            projection_args.update(points=[v.detach().clone() for v in call_args[1]],
                lidar2image=call_args[2].detach().clone(), img_aug_matrix=call_args[5].detach().clone(),
                lidar_aug_matrix=call_args[6].detach().clone())
    hooks.append(transform.register_forward_pre_hook(projection_capture))

    def run(row, policy, sample_idx):
        if digest(row['point_path']) != row['point_sha256']:
            raise ValueError('Frozen point payload changed')
        images = {key: Path(path).read_bytes() for key, path in row['images'].items()}
        if any(hashlib.sha256(images[key]).hexdigest() != row['image_sha256'][key] for key in images):
            raise ValueError('Frozen image payload changed')
        points = np.fromfile(row['point_path'], '<f4').reshape(-1, 4)
        model.preprocessor.point_intensity_source = 'native' if row['source'] == 'native' else 'ros255'
        transform.forward = original_forward if policy == 'canonical' else actual_forward
        transform.depth_raster_policy = None if policy in ('canonical', 'legacy_none') else policy
        transform.depth_raster_backend = 'sorted'
        predictions, prep, latency = model.infer(points, images, sample_idx, 0., 200, timestamp_sec=row['stamp'])
        result = dict(row, policy=policy, pred_boxes=[p['center']+p['size']+[p['yaw']] for p in predictions],
                      pred_labels=[p['label'] for p in predictions], pred_scores=[p['score'] for p in predictions],
                      query_scores=[], preprocess_ms=prep, inference_ms=latency)
        if diagnostics_active:
            result['_stages'] = {k: v.float().cpu().numpy() for k, v in stage_tensors.items()}
            coords = project_positive_camera_z(**projection_args, image_size=transform.image_size)
            result['projection_collision_stats'] = collision_stats(coords, transform.image_size)
        return result

    try:
        diagnostics_active = True
        grouped = defaultdict(list)
        for row in manifest['rows']:
            grouped[row['case']].append(row)
        diagnostic_rows = [values[len(values)//2] for values in grouped.values()]
        if args.diagnostic_case:
            diagnostic_rows = [r for r in diagnostic_rows if r['case'] in args.diagnostic_case]
        # Include an actual nearby obstacle scene, not only a feature-poor frame.
        diagnostics = []
        for i, row in enumerate(diagnostic_rows):
            reference = run(row, 'canonical', i)
            baseline = reference['_stages']
            records = []
            for policy in ('canonical', 'legacy_none', 'nearest_positive', 'farthest_positive', 'unique_last_positive'):
                first = None
                first_payload = None
                for repeat in range(args.diagnostic_repeats):
                    current = run(row, policy, i)
                    stages = current.pop('_stages')
                    if first is None:
                        first = copy.deepcopy(stages)
                        first_payload = {k: current[k] for k in ('pred_boxes', 'pred_labels', 'pred_scores')}
                    records.append(dict(policy=policy, repeat=repeat,
                        difference_from_canonical={k: delta(baseline[k], v) for k, v in stages.items()},
                        repeat_difference={k: delta(first[k], v) for k, v in stages.items()},
                        raw_payload_bitwise_repeat=all(current[k] == v for k,v in first_payload.items()),
                        raw_count=len(current['pred_scores']),
                        operating_count=sum(s >= (.4,.1,.1)[l] for s,l in zip(current['pred_scores'],current['pred_labels'])),
                        projection_collision_stats=current['projection_collision_stats']))
            diagnostics.append(dict(token=row['token'], case=row['case'], input_sha256=row['point_sha256'], records=records))
            (args.output / 'stage_repeatability.json').write_text(json.dumps(diagnostics, indent=2) + '\n')
            print(json.dumps(dict(diagnostic_case=row['case'], done=True)), flush=True)
        if args.diagnostic_only:
            return
        diagnostics_active = False
        stage_tensors.clear()
        projection_args.clear()
        rows = manifest['rows']
        if args.limit:
            rows = [rows[i] for i in uniform_indices(len(rows), args.limit)]
        buffers = {p: [] for p in ('legacy_none', 'nearest_positive', 'farthest_positive', 'unique_last_positive')}
        for index, row in enumerate(rows):
            for policy in buffers:
                buffers[policy].append(run(row, policy, index))
            if (index + 1) % 50 == 0:
                print(json.dumps(dict(frames=index+1, total=len(rows), policies=list(buffers))), flush=True)
        results = dict(input_sha256=digest(args.inputs), checkpoint_sha256=digest(args.checkpoint),
            gpu=torch.cuda.get_device_name(), scope='Full independently executed forwards; legacy A/A variability is separately measured. No target4090 timing certification.', groups={})
        for policy, output in buffers.items():
            with gzip.open(args.output / (policy + '.json.gz'), 'wt') as stream:
                json.dump(dict(rows=output, input_sha256=results['input_sha256'], checkpoint_sha256=results['checkpoint_sha256']), stream)
        for case in grouped:
            selected = {p: [r for r in v if r['case'] == case] for p,v in buffers.items()}
            if not selected['legacy_none']:
                continue
            results['groups'][case] = dict(policies={p: summarize(v, True) for p,v in selected.items()},
                paired={p: paired(selected['legacy_none'], selected[p]) for p in buffers if p != 'legacy_none'})
        (args.output / 'sensitivity.json').write_text(json.dumps(results, indent=2, default=lambda x: int(x) if isinstance(x,np.integer) else bool(x)) + '\n')
    finally:
        for handle in hooks:
            handle.remove()
        model.preprocessor.close()


if __name__ == '__main__':
    main()
