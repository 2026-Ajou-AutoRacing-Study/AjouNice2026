#!/usr/bin/env python3
"""Cross-evaluate frozen E25/E9/E24 on Base3 val without changing its labels.

Historical raw-decoder AP and deployment-style floor/NMS AP stay separate.
Original native annotations explain proximity to objects omitted by Base3's
five-point rule; these diagnostics never add GT or certify hallucinations.
Base5 was not trained from E25, so this is not a pure forgetting ablation.
"""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path
import pickle
import sys

import numpy as np

from calibrate_base5_thresholds import assignments, prepare, score_curve
from compare_base5_common30 import CLASSES, digest, evaluate, scope, validate_payload
from summarize_base5_rosbag_quick import bridge_nms

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'dataset/morai_od/scripts'))
from build_fastbev_sunny13_1500_dataset import coarse_class, parse_lidar_instances
from build_base3_bevfusion_infos import points_in_box_count
from prepare_base3_self_return_probe import mask_roof

CHECKPOINTS = {
    'E25': '36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011',
    'E9': '067205c3f5f5b0abc1bb775116c2b8bca2c9a7f31a0727b959db0ab1ce70b271',
    'E24': '84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746',
}
POLICIES = {'E25': (.4, .15, .1), 'E9': (.3, .2, .12), 'E24': (.4, .1, .1)}


def role(payload):
    matches = [k for k,v in CHECKPOINTS.items() if v == payload['checkpoint_sha256']]
    if len(matches) != 1:
        raise ValueError('Require the frozen selected E25/E9/E24 checkpoints')
    return matches[0]


def omitted_annotations(infos, capture):
    omitted = {}
    counts = Counter()
    retained = 0
    fingerprints = []
    for src in infos:
        relative = Path(src['lidar_points']['lidar_path']).relative_to('/datasets/base3')
        cloud = capture / relative
        txt = cloud.with_name(cloud.name.replace('_Intensity.bin', '_Instance.txt'))
        native = parse_lidar_instances(txt)
        fingerprints.append((src['token'], digest(txt)))
        kept = {int(i['instance_id']): i for i in src['instances']}
        for identity, inst in kept.items():
            box = native[identity]
            actual = box['center'] + box['size'] + [box['rotation_rpy'][2]]
            if not np.allclose(actual, inst['bbox_3d'], atol=1e-5, rtol=0):
                raise ValueError('Frozen GT no longer matches original native annotations')
            if CLASSES[inst['bbox_label_3d']] != coarse_class(box['raw_class']):
                raise ValueError('Original Base3 label mapping changed')
            retained += 1
        missing = [(identity, r) for identity,r in native.items() if identity not in kept]
        rows = []
        if missing:
            points = np.fromfile(cloud, dtype='<f4').reshape(-1, 4)
            for identity, record in missing:
                box = record['center'] + record['size'] + [record['rotation_rpy'][2]]
                if not np.isfinite(box).all() or any(v <= 0 for v in box[3:6]):
                    continue
                support = points_in_box_count(points, box, .03)
                if support >= 5:
                    raise ValueError('Unretained valid native actor has >=5 points; do not explain it as sparse')
                cls = coarse_class(record['raw_class'])
                bottom = list(box)
                bottom[2] -= bottom[5] / 2
                selected = scope(dict(gt_boxes=[bottom], gt_labels=[CLASSES.index(cls)],
                    pred_boxes=[], pred_labels=[], pred_scores=[]))
                if len(selected['gt_boxes']):
                    rows.append(dict(box=bottom, label=CLASSES.index(cls), points=support, id=identity))
                    counts[cls] += 1
        omitted[src['token']] = rows
    import hashlib
    return omitted, dict(retained_native_geometry_matches=retained,
        omitted_radial30_counts=dict(counts),
        original_annotation_fingerprint=hashlib.sha256(json.dumps(fingerprints).encode()).hexdigest(),
        note='Omitted annotations remain diagnostic only; no GT change or visibility assertion.')


def source_assets(infos, capture):
    """Join frozen native instance IDs to the recorded scenario metadata.

    Missing/component IDs stay explicitly unresolved, never guessed from size.
    Asset provenance is not added to or used to filter the primary GT payload.
    """
    cache = {}
    hashes = {}
    result = {}
    unresolved = 0
    for src in infos:
        scene = src['scene_id']
        if scene not in cache:
            batch = Path(src['lidar_points']['lidar_path']).relative_to('/datasets/base3').parts[0]
            clip_path = capture / batch / 'dynamic_metadata' / (scene + '.clip.json')
            mapping = {}
            if clip_path.is_file():
                clip = json.loads(clip_path.read_text())
                meta_path = ROOT / clip['source_scene_metadata']
                if meta_path.is_file():
                    meta = json.loads(meta_path.read_text())
                    mapping = {int(a['unique_id']): a['name'] for a in meta['runtime_actors']}
                    hashes[str(meta_path.relative_to(ROOT))] = digest(meta_path)
            cache[scene] = mapping
        keys = [cache[scene].get(int(i['instance_id']), 'unresolved_component_or_metadata')
                for i in src['instances']]
        unresolved += keys.count('unresolved_component_or_metadata')
        result[src['token']] = keys
    return result, dict(scenes=len(cache), unresolved_gt_observations=unresolved,
                        metadata_sha256=hashes)


def training_exposure(base3_info, base5_info, capture):
    old_path = base3_info.with_name('morai_base3_infos_train.pkl')
    old = pickle.loads(old_path.read_bytes())['data_list']
    names, audit = source_assets(old, capture)
    counts = Counter(names[r['token']][i] for r in old for i,g in enumerate(r['instances'])
                     if g['bbox_label_3d'] == 2)
    result = dict(note='Annotated training GT observations, not independent objects or equal effective gradient exposure. No new split/filter.',
        base3=dict(path=str(old_path), sha256=digest(old_path), frames=len(old),
            obstacle_observations=sum(counts.values()), obstacle_asset_types=len(counts),
            per_asset=dict(counts), unresolved_gt_observations=audit['unresolved_gt_observations']))
    if base5_info:
        payload = pickle.loads(base5_info.read_bytes())
        if list(payload['metainfo']['classes']).index('obstacle') != 2:
            raise ValueError('Training exposure requires the original obstacle label 2')
        rows = payload['data_list']
        new = Counter(g['asset'] for r in rows for g in r['instances'] if g['bbox_label_3d'] == 2)
        result['base5'] = dict(path=str(base5_info), sha256=digest(base5_info), frames=len(rows),
            obstacle_observations=sum(new.values()), obstacle_asset_types=len(new), per_asset=dict(new))
    return result


def unmatched_kind(box, label, retained_boxes, retained_labels, omitted):
    def nearest(boxes):
        a = np.asarray(boxes, float).reshape(-1, 7)
        return float(np.linalg.norm(a[:, :2]-np.asarray(box[:2]), axis=1).min()) if len(a) else float('inf')
    own = nearest(retained_boxes[retained_labels == label])
    missing = nearest([r['box'] for r in omitted if r['label'] == label])
    other = nearest(retained_boxes[retained_labels != label])
    if own < 1.: return 'retained_same_class_gt_lt1m'
    if missing < 1.: return 'omitted_sparse_same_class_annotation_lt1m'
    if own < 2.: return 'retained_same_class_gt_1_to_2m'
    if missing < 2.: return 'omitted_sparse_same_class_annotation_1_to_2m'
    if other < 2.: return 'retained_other_class_gt_lt2m'
    return 'unexplained_by_these_annotations'


def operating(rows, thresholds, omitted, assets=None):
    selected = prepare([scope(bridge_nms(dict(r, gt_asset_keys=assets[r['token']]) if assets else r)) for r in rows])
    result = {}
    for i, cls in enumerate(CLASSES):
        counts = Counter()
        by_asset = {}
        examples = []
        for r in selected:
            pairs = assignments(r, i, 1.)
            if assets:
                hits = {g for p,g in pairs.items() if r['pred_scores'][p] >= thresholds[i]}
                hits2 = {g for p,g in assignments(r, i, 2.).items() if r['pred_scores'][p] >= thresholds[i]}
                for g in np.flatnonzero(r['gt_labels'] == i):
                    key = str(r['gt_asset_keys'][g])
                    entry = by_asset.setdefault(key, dict(gt=0, tp1m=0, tp2m=0))
                    entry['gt'] += 1
                    entry['tp1m'] += int(g in hits)
                    entry['tp2m'] += int(g in hits2)
            for p in np.flatnonzero((r['pred_labels'] == i) & (r['pred_scores'] >= thresholds[i])):
                if p in pairs:
                    continue
                kind = unmatched_kind(r['pred_boxes'][p], i, r['gt_boxes'], r['gt_labels'], omitted[r['token']])
                counts[kind] += 1
                if len(examples) < 12:
                    examples.append(dict(token=r['token'], kind=kind,
                        box=r['pred_boxes'][p].tolist(), score=float(r['pred_scores'][p])))
        value = score_curve(selected, i, thresholds[i])
        if sum(counts.values()) != value['fp']:
            raise ValueError('Unmatched partition does not preserve original metric counts')
        result[cls] = dict(value, unmatched_annotation_proximity=dict(counts), examples=examples,
                          per_asset_counts=by_asset)
    return result


def compact(metrics):
    return {c: metrics[f'center/{c}_mean_AP'] for c in CLASSES}


def roof_population(paths):
    counts = [int(mask_roof(np.fromfile(p,'<f4').reshape(-1,4)).sum()) for p in paths]
    return dict(frames=len(counts), nonzero_frames=sum(c>0 for c in counts),
        minimum=int(np.min(counts)), median=float(np.median(counts)),
        p95=float(np.percentile(counts,95)), maximum=int(np.max(counts)))


def probe_result(original, filtered):
    payloads = []
    for path in (original, filtered):
        with gzip.open(path,'rt') as f:
            p = json.load(f)
        validate_payload(p)
        if (p['checkpoint_sha256'] != CHECKPOINTS['E24']
                or not p['inference_precision'].startswith('FP32')
                or p.get('inference_optimization')
                or p.get('point_intensity_scale') != 1. / 255.):
            raise ValueError('Roof probe requires the same frozen E24 FP32 checkpoint')
        payloads.append(p)
    if any(payloads[0][k] != payloads[1][k] for k in ('gt_identity','frames','config_sha256')):
        raise ValueError('Probe changed GT/config/frame contract')
    result = dict(scope='32-frame diagnostic input ablation; NOT a production near-object mask.', variants={})
    for path,p in zip((original,filtered),payloads):
        count = sum(int(l==0 and s>=.4 and np.linalg.norm(b[:2])<2.)
                    for r in p['rows'] for b,l,s in zip(r['pred_boxes'],r['pred_labels'],r['pred_scores']))
        result['variants'][p['name']] = dict(path=str(path), sha256=digest(path),
            near_origin_vehicle_observations=count, raw_ap=compact(evaluate(p['rows'])['metrics']))
    return result


def summarize(args):
    if args.output.exists():
        raise ValueError('Fresh report required')
    if bool(args.base5_data_root) != bool(args.dev_inputs):
        raise ValueError('Population audit requires both Base5 data root and dev inputs')
    if args.base5_data_root and not args.base5_train_info:
        raise ValueError('Population audit requires the corresponding Base5 train infos')
    data = pickle.loads(args.base3_info.read_bytes())
    if data['metainfo']['minimum_gt_points'] != 5 or len(data['data_list']) != 1635:
        raise ValueError('Require the original frozen 1,635-frame five-point Base3 val')
    infos = data['data_list']
    omitted, annotation_audit = omitted_annotations(infos, args.capture_root)
    assets, asset_audit = source_assets(infos, args.capture_root)
    result = dict(scope='Cross-distribution validation; not a controlled initialization/depth/grid ablation or live ROS certification.',
        base3_info_sha256=digest(args.base3_info), annotation_audit=annotation_audit, asset_audit=asset_audit,
        training_exposure=training_exposure(args.base3_info,args.base5_train_info,args.capture_root),
        frames=1635, unique_scenes=len({r['scene_id'] for r in infos}), models={},
        caveats=['Native GT/PNG inputs retained. Base3 GT >=5-point policy differs from Base5 visibility policy.',
                 'No training or silent raw-image JPEG conversion. Each model keeps its learned architecture/crop.',
                 'Intensity only: original 0..255 for E25, fixed divide255 for Base5.',
                 'Raw decoder AP reproduces historical evaluation. ROS floor/NMS AP is separate.',
                 'Proximity to an omitted annotation is neither a new TP nor proof that it is visibly recognizable.',
                 'Base5 is not an E25 fine-tune: lower old-distribution AP alone does not prove catastrophic forgetting.'])
    identity = None
    for path in args.predictions:
        with gzip.open(path, 'rt') as f:
            p = json.load(f)
        validate_payload(p)
        name = role(p)
        if name in result['models'] or p['frames'] != 1635 or p['info_sha256'] != digest(args.base3_info):
            raise ValueError('Incomplete/duplicate model or different Base3 split')
        if identity is not None and p['gt_identity'] != identity:
            raise ValueError('Model GT identities differ')
        identity = p['gt_identity']
        if not p['inference_precision'].startswith('FP32') or p.get('inference_optimization'):
            raise ValueError('Historical cross-validation requires FP32, compile and parallel OFF')
        expected_scale = 1. if name == 'E25' else 1./255.
        if p.get('point_intensity_scale', 1.) != expected_scale:
            raise ValueError('Checkpoint-native intensity adapter differs')
        raw = {s: evaluate(p['rows'], radial=s == 'radial30') for s in ('radial30','square30')}
        post = evaluate([bridge_nms(r) for r in p['rows']])
        result['models'][name] = dict(checkpoint_sha256=p['checkpoint_sha256'],
            predictions_path=str(path), predictions_sha256=digest(path),
            receipt={k:v for k,v in p.items() if k != 'rows'}, raw_decoder=raw,
            ros_floor_top100_nms=post,
            common_thresholds=operating(p['rows'], (.4,.15,.1), omitted, assets),
            selected_thresholds=list(POLICIES[name]),
            selected_operating_point=operating(p['rows'], POLICIES[name], omitted, assets))
        print(name, 'Base3 val raw', {k: round(v*100, 2) for k,v in compact(raw['radial30']['metrics']).items()}, flush=True)
    if set(result['models']) != set(CHECKPOINTS):
        raise ValueError('All three frozen models required')
    result['gt_identity'] = identity
    if args.base5_data_root and args.dev_inputs:
        base5_val = args.base5_train_info.with_name('morai_base5_infos_val.pkl')
        newer = pickle.loads(base5_val.read_bytes())['data_list']
        dev = pickle.loads((args.dev_inputs/'infos_val.pkl').read_bytes())['data_list']
        result['roof_return_populations'] = dict(
            volume_lower=[-1.2,-.75,-.35],volume_upper=[1.,.75,.1],
            base3_val=roof_population([args.capture_root/Path(s['lidar_points']['lidar_path']).relative_to('/datasets/base3') for s in infos]),
            base5_val=roof_population([args.base5_data_root/s['lidar_points']['lidar_path'] for s in newer]),
            current_dev_bags=roof_population([args.dev_inputs/s['lidar_points']['lidar_path'] for s in dev]))
    if args.self_probe_original or args.self_probe_filtered:
        if not args.self_probe_original or not args.self_probe_filtered:
            raise ValueError('Both original and filtered probe predictions required')
        result['roof_probe'] = probe_result(args.self_probe_original,args.self_probe_filtered)
    other = json.loads(args.base5_comparison.read_text())
    result['base5_reference'] = dict(path=str(args.base5_comparison), sha256=digest(args.base5_comparison),
                                   gt_identity=other['gt_identity'], models={})
    for entry in other['models'].values():
        name = role(entry['receipt'])
        result['base5_reference']['models'][name] = compact(entry['radial30']['metrics'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('base3-info','capture-root','base5-comparison','output'):
        parser.add_argument('--'+key, type=Path, required=True)
    parser.add_argument('--predictions', type=Path, nargs='+', required=True)
    parser.add_argument('--base5-train-info', type=Path,
                        help='Optional existing upright infos; obstacle labels/counts are unchanged by the animal merge.')
    parser.add_argument('--base5-data-root',type=Path)
    parser.add_argument('--dev-inputs',type=Path)
    parser.add_argument('--self-probe-original',type=Path)
    parser.add_argument('--self-probe-filtered',type=Path)
    summarize(parser.parse_args())
