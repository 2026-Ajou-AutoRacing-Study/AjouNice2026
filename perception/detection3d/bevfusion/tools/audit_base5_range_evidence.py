#!/usr/bin/env python3
"""CPU-only join of frozen Base5 GT, native point counts and actual crop masks.

This is an information-support audit, not model recall or an occlusion oracle.
Only load trusted local pickles. Sensor files, GT and collection contracts stay intact.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

CLASSES = ('vehicle', 'pedestrian', 'obstacle')
CAMERAS = {'CAMERA_1': 'Camera_front', 'CAMERA_2': 'Camera_front_left',
           'CAMERA_3': 'Camera_front_right', 'CAMERA_4': 'Camera_back'}
EDGES = (0, 10, 20, 30, 50, 80, 100, float('inf'))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def distance_bin(distance):
    for lo, hi in zip(EDGES[:-1], EDGES[1:]):
        if lo <= distance < hi:
            return f'{lo}_{hi}m' if np.isfinite(hi) else '100plus_corner'
    raise ValueError('Invalid distance')


def camera_key(token, camera):
    return (token, camera['camera'], camera['camera_instance_id'],
            camera.get('annotation_class', camera.get('raw_class')), tuple(camera['native_rect']))


def camera_summary(observations, ambiguous):
    """Use actual visible foreground, never projected box area as visibility."""
    values = [o['candidates']['bottom512w1056'] for o in observations]
    best = max(values, key=lambda v: v['model_mask_pixels_nearest'], default=None)
    # Same-parent same-class masks cannot certify individual component support.
    gate = None if ambiguous else any(v['candidate_camera_gate'] for v in values)
    rich = None if ambiguous else any(v['model_mask_pixels_nearest'] >= 64 and
                                     min(v['model_bbox_extent_px']) >= 8 for v in values)
    return dict(camera_gate=gate, camera_rich_proxy=rich,
                mask_matching_ambiguous=ambiguous,
                best_visible_mask_pixels=None if ambiguous else
                (best['model_mask_pixels_nearest'] if best else 0),
                best_visible_bbox_extent_px=None if ambiguous or not best else best['model_bbox_extent_px'],
                any_extra_crop_loss=None if ambiguous else any(v['native_fraction'] < 1 for v in values),
                camera_observations=len(values))


def summarize(rows):
    known = [r for r in rows if not r['mask_matching_ambiguous']]
    weak = [r for r in known if r['points'] < 5 and not r['camera_gate']]
    return dict(gt=len(rows), frames=len({r['token'] for r in rows}),
                point_bins=dict(Counter('0' if r['points'] == 0 else '1_4' if r['points'] < 5
                                        else '5_19' if r['points'] < 20 else '20plus' for r in rows)),
                camera_gate=sum(r['camera_gate'] is True for r in rows),
                camera_unknown=len(rows) - len(known),
                camera_only=sum(r['points'] == 0 for r in rows),
                weak_support_proxy=len(weak),
                weak_support_pct=100 * len(weak) / len(rows) if rows else None,
                rich_support_proxy=sum(r['points'] >= 20 or r['camera_rich_proxy'] is True for r in rows),
                crop_loss=sum(r['any_extra_crop_loss'] is True for r in rows),
                mask_pixels_p10_p50_p90=np.percentile([r['best_visible_mask_pixels'] for r in known],
                                                     [10, 50, 90]).tolist() if known else None,
                lidar_points_p10_p50_p90=np.percentile([r['points'] for r in rows], [10, 50, 90]).tolist() if rows else None)


def compare_metrics(groups, path):
    """Attach existing validation receipt only after GT denominator agreement."""
    receipt = json.loads(Path(path).read_text())
    rows = []
    for cls in CLASSES:
        for name in ('0_10m', '10_20m', '20_30m', '30_50m', '50_80m', '80_100m'):
            prefix = f'val/range/{name}/{cls}_'
            gt = groups.get(f'val/{cls}/{name}', {}).get('gt', 0)
            if receipt['metrics'][prefix + 'GT'] != gt:
                raise ValueError('Validation GT denominator mismatch: ' + prefix)
            rows.append(dict(cls=cls, distance_bin=name, gt=gt,
                             mean_AP=receipt['metrics'][prefix + 'mean_AP']))
    return dict(epoch=receipt['epoch'], receipt_sha256=sha256(path), comparisons=rows,
                scope='Existing aggregate AP and local support denominators; no raw prediction attribution')


def inventory_planned_primaries(suite, rows):
    """Prospective parent quota audit; explicitly not captured component GT."""
    asset_classes = defaultdict(set)
    for row in rows:
        asset_classes[row['asset']].add(row['cls'])
    reports = []
    for path in sorted(Path(suite).glob('*/manifest.json')):
        before = sha256(path)
        manifest = json.loads(path.read_text())
        counts, digest = Counter(), hashlib.sha256()
        for scene in manifest['scenes']:
            meta_path = Path(scene['metadata'])
            if not meta_path.is_absolute():
                meta_path = path.parent / meta_path
            payload = meta_path.read_bytes()
            digest.update(str(meta_path).encode() + b'\0' + hashlib.sha256(payload).digest())
            meta = json.loads(payload)
            if not meta['runtime_actors']:
                continue
            asset = meta['runtime_actors'][0]['name']
            if asset not in asset_classes:
                raise ValueError('Unmapped prospective asset: ' + asset)
            distance = meta['planned_primary_distance_m']
            pool = '+'.join(sorted(asset_classes[asset]))
            counts[(scene['split'], pool, distance_bin(distance))] += 1
        if before != sha256(path):
            raise ValueError('Prospective manifest changed')
        reports.append(dict(environment=path.parent.name, frames=len(manifest['scenes']),
                            manifest_sha256=before, metadata_digest=digest.hexdigest(),
                            counts=[dict(split=s, observed_asset_semantic_pool=p, planned_distance_bin=b, parents=n)
                                    for (s, p, b), n in sorted(counts.items())]))
    if not reports:
        raise ValueError('No prospective environment manifests')
    return dict(environments=reports, scope='Planned primary parent snapshots, NOT captured/visible component GT',
                limitations=['Distance is the recorded planned Ego-relative distance, not measured native LiDAR center distance.',
                             'Semantic pool uses classes observed for this asset in Sunny13; composites span multiple classes.',
                             'No visibility/point support can be certified before actual environment capture.'])


def audit(data, vertical, crops, output, metrics=None, fresh_suite=None):
    data, vertical, crops, output = map(Path, (data, vertical, crops, output))
    if output.exists():
        raise FileExistsError('Fresh output required')
    if data.resolve() == output.resolve() or data.resolve() in output.resolve().parents:
        raise ValueError('Analysis output must be outside source dataset')
    inputs = [data / 'manifest.json', vertical / 'objects.json', vertical / 'report.json',
              crops / 'observations.json', crops / 'report.json']
    inputs += [data / 'infos/512x1056' / f'morai_base5_infos_{s}.pkl' for s in ('train', 'val')]
    if metrics is not None:
        inputs.append(Path(metrics))
    before = {str(p): sha256(p) for p in inputs}
    manifest = json.loads(inputs[0].read_text())
    crop_report = json.loads((crops / 'report.json').read_text())
    if not crop_report['full_snapshot'] or crop_report['accepted_scenes'] != 5000:
        raise ValueError('Complete 5K crop audit required')
    if crop_report['source_sha256'] != manifest['source_ledger_sha256']:
        raise ValueError('Crop/source ledger mismatch')
    cached_points = {}
    for row in json.loads((vertical / 'objects.json').read_text()):
        key = (row['token'], row['component_id'])
        if key in cached_points:
            raise ValueError('Duplicate native support row')
        cached_points[key] = row
    observations = {}
    for row in json.loads((crops / 'observations.json').read_text()):
        key = camera_key(row['scene'], row)
        if key in observations:
            raise ValueError('Duplicate camera observation')
        observations[key] = row
    rows, groups, frames, rejected, images, seen = [], defaultdict(list), Counter(), Counter(), {}, set()
    for split in ('train', 'val'):
        path = data / 'infos/512x1056' / f'morai_base5_infos_{split}.pkl'
        records = pickle.loads(path.read_bytes())
        if tuple(records['metainfo']['classes']) != CLASSES:
            raise ValueError('Current merged three-class data required')
        for record in records['data_list']:
            token = record['token']
            if token in seen:
                raise ValueError('Duplicate frame / split leakage')
            seen.add(token)
            root = data / 'data' / token
            native = json.loads((root / 'audit.json').read_text())
            meta = json.loads((root / 'metadata.json').read_text())
            spec = meta['spec']
            components = {c['component_id']: c for c in native['components']}
            active = {i['component_id'] for i in record['instances']}
            for c in native['components']:
                if c['component_id'] not in active:
                    rejected['total'] += 1
                    rejected['no_saved_positive' if not c['positive_lidar1_camera0'] else 'roi_or_other'] += 1
            images[token] = record['images']
            frames[split] += 1
            for inst in record['instances']:
                c = components[inst['component_id']]
                p = cached_points[(token, inst['component_id'])]
                if (p['split'] != split or p['asset'] != inst['asset'] or
                        c['lidar_points'] != inst['num_lidar_pts'] or
                        not 0 <= p['native_xy_support'] <= c['lidar_points'] or
                        abs(p['distance_m'] - np.linalg.norm(c['center_lidar_m'][:2])) > 1e-4 or
                        not np.allclose(inst['bbox_3d'][:3], c['center_lidar_m'], atol=1e-5)):
                    raise ValueError('Cached native points/GT mismatch: ' + token)
                camera_rows = []
                for cam in c['cameras']:
                    key = camera_key(token, cam)
                    if key not in observations:
                        if cam['native_instance_pixels'] == 0:
                            continue  # Existing crop audit explicitly omits zero foreground.
                        raise ValueError('Missing camera measurement: ' + str(key))
                    camera_rows.append(observations[key])
                row = dict(token=token, component_id=inst['component_id'], split=split,
                           asset=inst['asset'], cls=CLASSES[inst['bbox_label_3d']],
                           original_semantic=c['training_class'], family=spec['family'],
                           mission=spec['mission'], role=spec['role'], primary=c['parent_uid'] ==
                           meta['runtime_actors'][0]['unique_id'], distance_m=p['distance_m'],
                           distance_bin=distance_bin(p['distance_m']),
                           direction='front' if c['center_lidar_m'][0] >= 0 else 'rear',
                           points=p['current_support'], native_points=p['native_xy_support'],
                           size_m=c['size_m'], center_lidar_m=c['center_lidar_m'],
                           camera_observation_keys=[list(camera_key(token, cam)) for cam in c['cameras']
                                                    if camera_key(token, cam) in observations],
                           **camera_summary(camera_rows, c['camera_component_ambiguous']))
                rows.append(row)
                for key in ('all', split, f"{split}/{row['cls']}/{row['distance_bin']}",
                            f"all/{row['cls']}/{row['distance_bin']}", 'asset/' + row['asset'],
                            f"family/{row['family']}/{'mission' if row['mission'] else 'generic'}/{row['distance_bin']}",
                            f"direction/{row['cls']}/{row['direction']}/{row['distance_bin']}",
                            'primary' if row['primary'] else 'secondary'):
                    groups[key].append(row)
    if dict(frames) != manifest['split_counts'] or len(rows) != len(cached_points):
        raise ValueError('Frame/GT coverage mismatch')
    if before != {str(p): sha256(p) for p in inputs}:
        raise ValueError('Inputs changed during read-only analysis')
    summaries = {k: summarize(v) for k, v in sorted(groups.items())}
    comparison = compare_metrics(summaries, metrics) if metrics is not None else None
    prospective = inventory_planned_primaries(fresh_suite, rows) if fresh_suite is not None else None
    output.mkdir(parents=True)
    report = dict(created_at_utc=datetime.now(timezone.utc).isoformat(), input_sha256=before,
                  source_ledger_sha256=manifest['source_ledger_sha256'], frames=dict(frames),
                  raw_components_excluded=dict(rejected), gt=len(rows),
                  groups=summaries, validation_receipt_comparison=comparison, prospective_primary_quotas=prospective,
                  rules=dict(weak='LiDAR <5 and current actual mask camera gate false; ambiguous excluded',
                             rich='LiDAR >=20 OR one actual mask >=64 pixels and clipped bbox minimum extent >=8px',
                             camera_gate='Existing actual foreground >=16 pixels AND minimum bbox extent >=4px',
                             distance='Native BBox center radial XY distance in LiDAR frame; [lo,hi)',
                             crop='Current deterministic bottom 512x1056; native Q90 JPEG model input'),
                  limitations=['Support proxies are not AP, recall, correctness, or negative transfer proof.',
                               'Mask counts are actual simulator foreground, not geometry-only projection.',
                               'Large visible rectangles do not mean every enclosed pixel is foreground.',
                               'Ambiguous composite camera components remain unknown, not invisible.',
                               'Training augmentation changes crop/points; this is deterministic validation geometry.',
                               'Cached native support is verified by component identity/asset/center/count, not rescanned.',
                               'No raw predictions: no per-object misses, confidence, temporal gaps or localization errors.'])
    (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    (output / 'objects.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    draw(rows, observations, images, data, output)
    print(json.dumps(dict(output=str(output), frames=dict(frames), gt=len(rows),
                          excluded=dict(rejected), groups=report['groups']['all']), ensure_ascii=False))
    return report


def draw(rows, observations, images, data, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    import cv2
    cv2.setNumThreads(1)
    labels = [distance_bin(v) for v in (0, 10, 20, 30, 50, 80)]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    for cls, ax in zip(CLASSES, axes):
        subsets = [[r for r in rows if r['cls'] == cls and r['distance_bin'] == b] for b in labels]
        bottom = np.zeros(len(labels))
        for name, colour, predicate in [
            ('weak proxy', '#ef4444', lambda r: r['points'] < 5 and r['camera_gate'] is False),
            ('other / ambiguous', '#fbbf24', lambda r: not (r['points'] < 5 and r['camera_gate'] is False)
             and not (r['points'] >= 20 or r['camera_rich_proxy'] is True)),
            ('rich proxy', '#22c55e', lambda r: r['points'] >= 20 or r['camera_rich_proxy'] is True)]:
            counts = np.array([sum(predicate(r) for r in subset) for subset in subsets])
            ax.bar(labels, counts, bottom=bottom, label=name, color=colour)
            bottom += counts
        ax.set_title(cls); ax.set_ylabel('GT count, train + val'); ax.tick_params(axis='x', rotation=35)
    axes[-1].legend(fontsize=8); fig.tight_layout(); fig.savefig(output / 'range_support.png', dpi=150); plt.close(fig)
    selected = []
    for cls in CLASSES:
        for weak in (True, False):
            candidates = [r for r in rows if r['cls'] == cls and 50 <= r['distance_m'] < 100 and
                          r['camera_observation_keys'] and not r['mask_matching_ambiguous'] and
                          ((r['points'] < 5 and not r['camera_gate']) if weak else r['camera_rich_proxy'])]
            if candidates:
                candidates.sort(key=lambda r: (r['best_visible_mask_pixels'], r['token']))
                selected.append(candidates[len(candidates) // 2])
    if not selected:
        return
    fig, axes = plt.subplots(len(selected), 2, figsize=(13, 3.5 * len(selected)), squeeze=False)
    review = []
    for axes_row, row in zip(axes, selected):
        matches = []
        for key in row['camera_observation_keys']:
            key = tuple(key[:-1]) + (tuple(key[-1]),)
            matches.append(observations[key])
        obs = max(matches, key=lambda o: o['candidates']['bottom512w1056']['model_mask_pixels_nearest'])
        cam = images[row['token']][CAMERAS[obs['camera']]]
        bgr = cv2.imread(str(data / cam['img_path']), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError('Cannot decode Q90 JPEG')
        scale = 1056 / bgr.shape[1]
        bgr = cv2.resize(bgr, (1056, int(bgr.shape[0] * scale)), interpolation=cv2.INTER_CUBIC)[-512:]
        image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rect = np.array(obs['native_rect']) * scale
        rect[[1, 3]] -= int(cam['height'] * scale) - 512
        for ax in axes_row:
            ax.imshow(image); ax.add_patch(Rectangle(rect[:2], rect[2] - rect[0], rect[3] - rect[1],
                                                    fill=False, edgecolor='magenta', linewidth=1))
        axes_row[0].set_title(f"{row['cls']} / {row['asset']} / {row['distance_m']:.1f}m / "
                              f"LiDAR {row['points']} / mask {row['best_visible_mask_pixels']} px", fontsize=9)
        axes_row[0].set_xlim(0, 1056); axes_row[0].set_ylim(512, 0)
        cx, cy = (rect[:2] + rect[2:]) / 2
        axes_row[1].set_xlim(max(0, cx - 80), min(1056, cx + 80))
        axes_row[1].set_ylim(min(512, cy + 60), max(0, cy - 60)); axes_row[1].set_title('Magnified input (no extra sensor detail)', fontsize=9)
        for ax in axes_row:
            ax.set_xticks([]); ax.set_yticks([])
        review.append(dict(token=row['token'], component_id=row['component_id'], camera=obs['camera']))
    fig.tight_layout(); fig.savefig(output / 'far_input_examples.png', dpi=150); plt.close(fig)
    (output / 'review_samples.json').write_text(json.dumps(review, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('data', 'vertical', 'crops', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--metrics', type=Path, help='Existing local epoch receipt; never contact remote training')
    parser.add_argument('--fresh-suite', type=Path, help='Read-only prospective environment manifests')
    args = parser.parse_args()
    audit(args.data, args.vertical, args.crops, args.output, args.metrics, args.fresh_suite)
