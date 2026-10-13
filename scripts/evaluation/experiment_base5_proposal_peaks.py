#!/usr/bin/env python3
"""Frozen E24 query-peak experiment: prepare, infer, CPU all-class analysis.

No source raw/GT/default/model weights are changed. Exact state is shared
between policies, with an original/None/[3,1,1] end-to-end parity gate first.
Local RTX5060Ti cost is not target4090 or actual ROS/LAN latency evidence.
"""

import argparse
from collections import Counter, defaultdict
import copy
import gzip
import hashlib
import json
from pathlib import Path
import pickle
import shutil
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAMERAS = ('front', 'front_left', 'front_right', 'back')
CLASSES = ('vehicle', 'pedestrian', 'obstacle')


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for blob in iter(lambda: stream.read(1048576), b''):
            h.update(blob)
    return h.hexdigest()


def uniform_indices(length, count):
    if length < 1:
        return []
    return sorted(set(np.rint(np.linspace(0, length - 1, min(count, length))).astype(int).tolist()))


def author_row(source, root, kind):
    source = copy.deepcopy(source)
    images = {}
    for camera in CAMERAS:
        aliases = ('camera_' + camera, 'Camera_' + camera)
        value = next((source['images'][key] for key in aliases if key in source['images']), None)
        if value is None:
            raise ValueError('Missing authored camera: ' + camera)
        path = Path(value['img_path'])
        images['image/camera_' + camera] = str(path if path.is_absolute() else root / path)
    point_path = Path(source['lidar_points']['lidar_path'])
    point_path = point_path if point_path.is_absolute() else root / point_path
    gt = np.asarray([r['bbox_3d'] for r in source['instances']], dtype=float).reshape(-1, 7)
    gt[:, 2] -= gt[:, 5] * .5
    labels = [1 if r['bbox_label_3d'] == 3 else r['bbox_label_3d'] for r in source['instances']]
    return dict(token=source['token'], source=kind, case=source['scene_id'] if kind == 'dev' else 'native_val',
                stamp=float(source['timestamp']), point_path=str(point_path), images=images,
                scoring_eligible=bool(source.get('scoring_eligible', True)),
                gt_boxes=gt.tolist(), gt_labels=labels,
                gt_points=[r['num_lidar_pts'] for r in source['instances']],
                gt_assets=[r.get('asset', 'unknown') for r in source['instances']],
                parent_uids=[r.get('parent_uid') for r in source['instances']],
                raw_lidar_index=source.get('raw_lidar_index'),
                authored_source=dict(token=source['token'], images=source['images']))


def prepare(args):
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    sources = [args.dev_info, args.dev_manifest, args.native_info]
    hashes = {str(p): digest(p) for p in sources}
    with args.dev_info.open('rb') as stream:
        dev = pickle.load(stream)['data_list']
    with args.native_info.open('rb') as stream:
        native = pickle.load(stream)['data_list']
    grouped = defaultdict(list)
    for row in dev:
        if row.get('scoring_eligible', True):
            grouped[row['scene_id']].append(row)
    selected = []
    for case, rows in sorted(grouped.items()):
        selected += [author_row(rows[i], args.dev_info.parent, 'dev')
                     for i in uniform_indices(len(rows), args.dev_per_case)]
    selected += [author_row(r, args.native_root, 'native') for r in native]
    if len({r['token'] for r in selected}) != len(selected):
        raise ValueError('Duplicate tokens across frozen sources')
    for row in selected:
        row['point_sha256'] = digest(row['point_path'])
        row['image_sha256'] = {key: digest(path) for key, path in row['images'].items()}
    if hashes != {str(p): digest(p) for p in sources}:
        raise RuntimeError('Frozen source changed during preparation')
    args.output.mkdir(parents=True)
    result = dict(rows=selected, input_sha256=hashes, cases=dict(Counter(r['case'] for r in selected)),
                  scope='Complete745 nativeval plus deterministic evenly spaced eligible four-bag dev subset; no train use.',
                  source_contracts=dict(dev='Raw ROS intensity0..255, original JPEG, reconstructed GT.',
                                        native='Native normalized intensity0..1, Q90 cache, upright GT animal->ped derived only.'))
    (args.output / 'inputs.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(dict(output=str(args.output), frames=len(selected), cases=result['cases']), indent=2))


def bootstrap_source():
    # Snapshot actual vendored Python while retaining the image's compatible
    # compiled extensions. Temporary copied code is not a repo source edit.
    root = Path('/tmp/agent-base5-peak-mmdetection3d')
    if root.exists():
        raise FileExistsError('Fresh isolated source bootstrap required')
    shutil.copytree(ROOT / 'third_party/mmdetection3d', root,
                    ignore=shutil.ignore_patterns('.git', '__pycache__'))
    original = Path('/opt/mmdetection3d')
    for path in original.rglob('*.so'):
        dest = root / path.relative_to(original)
        dest.parent.mkdir(parents=True, exist_ok=True)
        if dest.exists() or dest.is_symlink():
            dest.unlink()
        dest.symlink_to(path)
    sys.path.insert(0, str(root))


def infer(args):
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    bootstrap_source()
    import cv2
    import torch
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import MoraiRectangularTransFusionHead

    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    manifest = json.loads(args.inputs.read_text())
    args.output.mkdir(parents=True)
    calibration = load_sensor4_calibration(args.sensor, ['camera_' + c for c in CAMERAS])
    model = BevFusionModel(str(args.config), str(args.checkpoint), calibration,
                           use_amp=True, preprocess_workers=4, warmup_iterations=2,
                           base5_runtime=True, parallel_camera=True, point_intensity_source='ros255')
    # Test all policies with one actual learned model; no parameter reloads or
    # numerical compile changes. Published production remains unrelated.
    head = model.model.bbox_head
    original_forward = head.forward_single
    base_forward = MoraiRectangularTransFusionHead.forward_single.__get__(head, type(head))
    policies = dict(legacy_none=None, explicit_legacy=[3, 1, 1], peak3=[3, 1, 3], peak5=[3, 1, 5])
    snapshots, parity, frozen = {}, [], {}

    def capture(module, call_args):
        if args.frozen_features:
            feature = call_args[0]
            frozen['features'] = (feature.detach().clone() if torch.is_tensor(feature)
                                  else [v.detach().clone() for v in feature])
            frozen['metas'] = copy.deepcopy(call_args[1])
    capture_handle = head.register_forward_pre_hook(capture)

    def collect(module, call_args, output):
        value = output[0][0]
        labels = module.query_labels.detach().clone()
        n = module.num_proposals
        logits = value['heatmap'][..., -n:].detach().clone()
        dense = value['query_heatmap_score'].detach().clone()
        snapshots.clear()
        snapshots.update(labels=labels, logits=logits, dense=dense)
        if args.parity_debug:
            features = call_args[0][0] if isinstance(call_args[0], list) else call_args[0]
            snapshots.update(features=features.detach().clone(),
                             dense_full=value['dense_heatmap'].detach().clone())
    handle = head.register_forward_hook(collect)
    init_positions = {}
    def positions(_module, _args, kwargs):
        if args.parity_debug:
            init_positions['value'] = kwargs['query_pos'].detach().clone()
    position_handle = head.decoder[0].register_forward_pre_hook(positions, with_kwargs=True)

    def select_policy(policy):
        if policy == 'canonical':
            head.forward_single = base_forward
        else:
            head.forward_single = original_forward
            head.proposal_peak_kernels = policies[policy]

    def run(row, policy, sequence, replay=False):
        if digest(row['point_path']) != row['point_sha256']:
            raise ValueError('Frozen point payload changed')
        images = {key: Path(path).read_bytes() for key, path in row['images'].items()}
        if any(hashlib.sha256(images[key]).hexdigest() != row['image_sha256'][key] for key in images):
            raise ValueError('Frozen JPEG payload changed')
        points = np.fromfile(row['point_path'], '<f4').reshape(-1, 4)
        model.preprocessor.point_intensity_source = 'native' if row['source'] == 'native' else 'ros255'
        if row['source'] == 'native' and not ((points[:, 3] >= 0).all() and (points[:, 3] <= 1).all()):
            raise ValueError('Native normalized intensity contract changed')
        select_policy(policy)
        if replay:
            if frozen.get('token') != row['token']:
                raise ValueError('Cannot replay a different frame feature')
            torch.cuda.synchronize()
            started = time.perf_counter()
            with torch.inference_mode(), torch.autocast('cuda', dtype=torch.float16, enabled=model.use_amp):
                result = head.predict(frozen['features'], frozen['metas'])[0]
            torch.cuda.synchronize()
            latency = (time.perf_counter() - started) * 1000
            prep = 0.0
            boxes = result.bboxes_3d.tensor.detach().cpu()
            scores = result.scores_3d.detach().cpu()
            labels = result.labels_3d.detach().cpu()
            keep = torch.nonzero(scores >= 0, as_tuple=False).flatten()
            keep = keep[torch.argsort(scores[keep], descending=True)][:200]
            predictions = [dict(center=boxes[i, :3].tolist(), size=boxes[i, 3:6].tolist(),
                                yaw=float(boxes[i, 6]), label=int(labels[i]), score=float(scores[i]))
                           for i in keep.tolist()]
        else:
            predictions, prep, latency = model.infer(points, images, sequence, 0, 200,
                                                      timestamp_sec=row['stamp'])
            if args.frozen_features:
                frozen['token'] = row['token']
        labels = snapshots['labels']
        probabilities = snapshots['logits'].sigmoid()
        classifier = probabilities.gather(1, labels[:, None]).flatten().float().cpu().tolist()
        dense = snapshots['dense'].gather(1, labels[:, None]).flatten().float().cpu().tolist()
        labels = labels.flatten().cpu().tolist()
        result = dict(row, policy=policy,
                      pred_boxes=[p['center'] + p['size'] + [p['yaw']] for p in predictions],
                      pred_labels=[p['label'] for p in predictions], pred_scores=[p['score'] for p in predictions],
                      query_scores=[dict(query=i, label=label, classifier=cls, dense=ds, product=cls*ds)
                                    for i, (label, cls, ds) in enumerate(zip(labels, classifier, dense))],
                      preprocess_ms=prep, inference_ms=latency, frozen_feature_replay=replay)
        if args.parity_debug:
            result['_debug'] = {key: value.float().cpu().numpy() for key, value in snapshots.items()}
            result['_debug']['query_pos'] = init_positions['value'].float().cpu().numpy()
        return result

    try:
        if args.parity_debug:
            candidates = [r for r in manifest['rows'] if r['case'] == 'O02_164012']
            diagnostic_row = candidates[len(candidates) // 2]
            records = {}
            numerical_profiles = ('amp_parallel', 'amp_serial', 'fp32_serial', 'fp32_serial_tf32_off')
            for profile in numerical_profiles:
                if profile == 'amp_serial':
                    model.inference_optimization.disable()
                    model.inference_optimization = None
                if profile.startswith('fp32'):
                    model.use_amp = False
                if profile.endswith('tf32_off'):
                    torch.backends.cuda.matmul.allow_tf32 = False
                    torch.backends.cudnn.allow_tf32 = False
                    torch.backends.cudnn.benchmark = False
                all_runs = {policy: [run(diagnostic_row, policy, n) for n in range(3)]
                            for policy in ('canonical', 'legacy_none', 'explicit_legacy')}
                reference = all_runs['canonical'][0]
                comparisons = []
                for policy, values in all_runs.items():
                    for repeat, value in enumerate(values):
                        debug = {}
                        for key, expected in reference['_debug'].items():
                            actual = value['_debug'][key]
                            delta = np.abs(expected - actual)
                            debug[key] = dict(bitwise=bool(np.array_equal(expected, actual)),
                                              different=int(np.count_nonzero(delta)),
                                              mean=float(delta.mean()), maximum=float(delta.max()),
                                              p95=float(np.percentile(delta, 95)))
                        operating_a = [(b, l, s) for b, l, s in zip(reference['pred_boxes'], reference['pred_labels'], reference['pred_scores'])
                                       if s >= (.4, .1, .1)[l]]
                        operating_b = [(b, l, s) for b, l, s in zip(value['pred_boxes'], value['pred_labels'], value['pred_scores'])
                                       if s >= (.4, .1, .1)[l]]
                        comparisons.append(dict(policy=policy, repeat=repeat, layers=debug,
                            raw_payload_equal=all(reference[k] == value[k] for k in ('pred_boxes', 'pred_labels', 'pred_scores')),
                            operating_before_nms_equal=operating_a == operating_b,
                            operating_counts=[len(operating_a), len(operating_b)],
                            inference_ms=value['inference_ms']))
                records[profile] = dict(token=diagnostic_row['token'], comparisons=comparisons)
                print(json.dumps(dict(profile=profile, done=True)), flush=True)
                (args.output / 'parity_debug.json').write_text(json.dumps(records, indent=2) + '\n')
            return
        # Across sources and missions, not just a single benign empty scene.
        by_case = defaultdict(list)
        for row in manifest['rows']:
            by_case[row['case']].append(row)
        parity_rows = [r for values in by_case.values() for r in (values[0], values[len(values)//2])]
        for i, row in enumerate(parity_rows):
            outputs = [run(row, 'canonical', i)]
            if args.frozen_features:
                replayed = run(row, 'canonical', i, replay=True)
                if any(outputs[0][k] != replayed[k] for k in ('pred_boxes', 'pred_labels', 'pred_scores')):
                    (args.output / 'replay_failed.json').write_text(json.dumps([outputs[0], replayed], indent=2) + '\n')
                    raise RuntimeError('Canonical frozen head replay differs from captured full forward')
            outputs += [run(row, policy, i, replay=args.frozen_features)
                        for policy in ('legacy_none', 'explicit_legacy')]
            for policy, output in zip(('legacy_none', 'explicit_legacy'), outputs[1:]):
                exact = all(outputs[0][k] == output[k] for k in ('pred_boxes', 'pred_labels', 'pred_scores'))
                parity.append(dict(token=row['token'], source=row['source'], case=row['case'],
                                   policy=policy, bitwise_payload=exact))
                if not exact:
                    (args.output / 'parity_failed.json').write_text(json.dumps(outputs, indent=2) + '\n')
                    raise RuntimeError('Canonical/None/explicit legacy end-to-end parity gate failed')
        (args.output / 'parity.json').write_text(json.dumps(parity, indent=2) + '\n')
        if args.frozen_features:
            buffers = {policy: [] for policy in ('legacy_none', 'peak3', 'peak5')}
            for index, row in enumerate(manifest['rows']):
                buffers['legacy_none'].append(run(row, 'legacy_none', index))
                for policy in ('peak3', 'peak5'):
                    buffers[policy].append(run(row, policy, index, replay=True))
                if (index + 1) % 50 == 0:
                    print(json.dumps(dict(policies=list(buffers), frames=index+1, total=len(manifest['rows']))), flush=True)
            for policy, outputs in buffers.items():
                with gzip.open(args.output / (policy + '.json.gz'), 'wt') as stream:
                    json.dump(dict(rows=outputs, input_sha256=digest(args.inputs),
                                   checkpoint_sha256=digest(args.checkpoint), config_sha256=digest(args.config),
                                   gpu=torch.cuda.get_device_name(), amp=True, parallel_camera=True, compile=False,
                                   feature_contract='One full learned backbone per frame; identical frozen head input for all three policies. Candidate timings are head-only, not total latency.'),
                              stream, allow_nan=False)
            return
        for policy in ('legacy_none', 'peak3', 'peak5'):
            outputs = []
            for index, row in enumerate(manifest['rows']):
                outputs.append(run(row, policy, index))
                if (index + 1) % 50 == 0:
                    print(json.dumps(dict(policy=policy, frames=index+1, total=len(manifest['rows']),
                                          inference_ms=outputs[-1]['inference_ms'])), flush=True)
            with gzip.open(args.output / (policy + '.json.gz'), 'wt') as stream:
                json.dump(dict(rows=outputs, input_sha256=digest(args.inputs),
                               checkpoint_sha256=digest(args.checkpoint), config_sha256=digest(args.config),
                               gpu=torch.cuda.get_device_name(), amp=True, parallel_camera=True, compile=False),
                          stream, allow_nan=False)
    finally:
        handle.remove()
        capture_handle.remove()
        position_handle.remove()
        model.preprocessor.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    make = commands.add_parser('prepare')
    for name in ('dev-info', 'dev-manifest', 'native-info', 'native-root', 'output'):
        make.add_argument('--' + name, type=Path, required=True)
    make.add_argument('--dev-per-case', type=int, default=50)
    worker = commands.add_parser('infer')
    for name in ('inputs', 'output', 'sensor', 'config', 'checkpoint'):
        worker.add_argument('--' + name, type=Path, required=True)
    worker.add_argument('--parity-debug', action='store_true')
    worker.add_argument('--frozen-features', action='store_true',
                        help='Research isolation: replay exactly the same captured BEV feature through each head policy.')
    args = parser.parse_args()
    (prepare if args.command == 'prepare' else infer)(args)


if __name__ == '__main__':
    main()
