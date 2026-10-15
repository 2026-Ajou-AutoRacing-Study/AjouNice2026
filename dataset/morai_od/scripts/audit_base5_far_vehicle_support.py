#!/usr/bin/env python3
"""CPU-only vehicle support audit; preserve data, labels and physical splits.

Frozen infos define the actual GT. A previously measured 512-input mask cache
only supplies camera evidence after a token/component/center/source-lineage join.
Support buckets are diagnostics, never capture acceptance or detector accuracy.
Only load trusted local pickle files.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import pickle

import numpy as np

from base5_capture_safety import atomic_json, sha256


EDGES = (0, 30, 50, 80, 100, float('inf'))


def range_band(distance):
    if not math.isfinite(distance) or distance < 0:
        raise ValueError('Finite nonnegative distance required')
    for lo, hi in zip(EDGES[:-1], EDGES[1:]):
        if lo <= distance < hi:
            return f'{lo}_{hi}m' if math.isfinite(hi) else '100plus_corner'
    raise AssertionError('Unreachable distance')


def direction(center, policy='x_sign'):
    if policy not in ('x_sign', 'sector'):
        raise ValueError('Unknown direction policy')
    x, y = center[:2]
    if policy == 'x_sign' or abs(x) >= abs(y):
        return 'front' if x >= 0 else 'rear'
    return 'left' if y >= 0 else 'right'


def lateral_proxy(center):
    """Ego/LiDAR-axis offset, NOT an adjacent-lane identity measurement."""
    value = abs(center[1])
    return '0_2m' if value < 2 else '2_6m' if value < 6 else '6plus_m'


def quantiles(values):
    return (dict(zip(('p10', 'p50', 'p90', 'p95'),
                     np.percentile(values, (10, 50, 90, 95)).tolist()))
            if values else None)


def summarize(rows):
    count = len(rows)
    known = [r for r in rows if r.get('camera_gate') is not None]
    points = Counter('0' if r['points'] == 0 else '1_4' if r['points'] < 5
                     else '5_19' if r['points'] < 20 else '20plus' for r in rows)
    gates = sum(r['camera_gate'] is True for r in rows)
    return dict(gt=count, scenes=len({r['token'] for r in rows}),
                scene_groups=len({r['scene_group_id'] for r in rows}),
                assets=len({r['asset'] for r in rows}),
                point_counts={k: points[k] for k in ('0', '1_4', '5_19', '20plus')},
                point_fraction={k: points[k] / count if count else None
                                for k in ('0', '1_4', '5_19', '20plus')},
                camera_gate=gates, camera_known=len(known),
                camera_gate_fraction_known=gates / len(known) if known else None,
                camera_only=sum(r['points'] == 0 and r['camera_gate'] is True for r in rows),
                lidar_points=quantiles([r['points'] for r in rows]),
                mask_pixels=quantiles([r['best_visible_mask_pixels'] for r in known
                                       if r.get('best_visible_mask_pixels') is not None]),
                bbox_short_side_pixels=quantiles([min(r['best_visible_bbox_extent_px'])
                    for r in known if r.get('best_visible_bbox_extent_px')]),
                splits=dict(Counter(r['split'] for r in rows)),
                families=dict(Counter(r['family'] for r in rows)),
                asset_gt=dict(sorted(Counter(r['asset'] for r in rows).items())))


def join_infos(infos_by_split, observations):
    index = {}
    for row in observations:
        key = (row['token'], row['component_id'])
        if key in index:
            raise ValueError('Duplicate camera observation component: ' + str(key))
        index[key] = row
    result = []
    expected_keys = set()
    for split, doc in infos_by_split.items():
        names = doc['metainfo']['classes']
        for frame in doc['data_list']:
            for inst in frame['instances']:
                if names[inst['bbox_label_3d']].lower() != 'vehicle':
                    continue
                key = (frame['token'], inst['component_id'])
                if key in expected_keys:
                    raise ValueError('Duplicate vehicle GT component: ' + str(key))
                expected_keys.add(key)
                cached = index.get(key)
                if (cached is None or cached['cls'] != 'vehicle'
                        or cached['split'] != split or cached['asset'] != inst['asset']
                        or not 0 <= cached['points'] <= cached['native_points'] <= inst['num_lidar_pts']
                        or not np.allclose(cached['center_lidar_m'], inst['bbox_3d'][:3],
                                           atol=1e-5, rtol=0)):
                    raise ValueError('Frozen GT / camera cache mismatch: ' + str(key))
                result.append(dict(cached, split=split,
                    scene_group_id=frame.get('scene_group_id', frame.get('scene_id', frame['token'])),
                    center_lidar_m=list(inst['bbox_3d'][:3]),
                    raw_annotation_points=inst['num_lidar_pts'],
                    distance_m=math.hypot(*inst['bbox_3d'][:2])))
    cache_keys = {(r['token'], r['component_id']) for r in observations if r['cls'] == 'vehicle'}
    if cache_keys != expected_keys:
        raise ValueError('Vehicle GT denominator differs from camera cache')
    return result


def captured_report(rows):
    groups = defaultdict(list)
    for row in rows:
        band = range_band(row['distance_m'])
        for split in ('all', row['split']):
            groups[f'{split}/{band}'].append(row)
            groups[f'{split}/{band}/x_sign/{direction(row["center_lidar_m"])}'].append(row)
            groups[f'{split}/{band}/sector/{direction(row["center_lidar_m"], "sector")}'].append(row)
            groups[f'{split}/{band}/ego_lateral_proxy/{lateral_proxy(row["center_lidar_m"])}'].append(row)
    summaries = {key: summarize(value) for key, value in sorted(groups.items())}
    warnings = []
    for band in ('50_80m', '80_100m'):
        val = summaries.get('val/' + band, {'gt': 0, 'scenes': 0, 'assets': 0})
        if val['gt'] < 30:
            warnings.append(dict(code='SMALL_FAR_VALIDATION_DENOMINATOR', band=band,
                val_gt=val['gt'], val_scenes=val['scenes'], val_assets=val['assets'],
                action='Report uncertainty; do not move existing samples across physical split.'))
    return dict(vehicle_gt=len(rows), groups=summaries, warnings=warnings,
                split_modified=False)


def planned_warnings(groups, physical_links):
    warnings = []
    for band in ('50_80m', '80_100m'):
        count = groups.get('val/' + band, {}).get('planned_parent_scenes', 0)
        links = len(physical_links.get(band, {}).get('val', set()))
        if count < 30 or links < 2:
            warnings.append(dict(code='PLANNED_FAR_VALIDATION_SUPPORT_IMBALANCE', band=band,
                planned_val_parents=count, planned_val_ego_links=links,
                action='Audit a separate prospective evaluation design; never reassign the frozen split.'))
    return warnings


def planned_report(path):
    """Summarize immutable planned primaries, not captured/visible component GT."""
    path = Path(path)
    before = sha256(path)
    deck = json.loads(path.read_text())
    groups = defaultdict(Counter)
    assets = defaultdict(Counter)
    digest = hashlib.sha256()
    physical_links = defaultdict(lambda: defaultdict(set))
    violations = []
    for entry in deck['scenes']:
        if entry['family'] != 'highway':
            continue
        metadata = Path(entry['metadata'])
        if not metadata.is_absolute():
            metadata = path.parent / metadata
        payload = metadata.read_bytes()
        digest.update(entry['id'].encode() + b'\0' + hashlib.sha256(payload).digest())
        meta = json.loads(payload)
        actors = meta['runtime_actors']
        if not actors:
            raise ValueError('Highway primary missing')
        if any(a['source_list'] != 'vehicleList' for a in actors):
            violations.append(entry['id'])
        primary = actors[0]
        band = range_band(primary['planned_center_distance_m'])
        for prefix in ('all', entry['split']):
            key = prefix + '/' + band
            groups[key]['planned_parent_scenes'] += 1
            groups[key]['mission' if entry['mission'] else 'generic'] += 1
            groups[key][meta['spec']['topology']] += 1
            groups[key]['x_sign_' + direction((primary['longitudinal_m'], primary['lateral_m']))] += 1
            assets[key][primary['name']] += 1
        physical_links[band][entry['split']].add(meta['split_group'])
    if sha256(path) != before:
        raise ValueError('Manifest changed during read-only audit')
    return dict(manifest_sha256=before, highway_metadata_digest=digest.hexdigest(),
        groups={k: dict(v, distinct_assets=len(assets[k]), asset_counts=dict(assets[k]))
                for k, v in sorted(groups.items())},
        physical_link_counts={b: {s: len(v) for s, v in splits.items()}
                              for b, splits in sorted(physical_links.items())},
        warnings=planned_warnings(groups, physical_links), split_modified=False,
        vehicle_only_violations=violations,
        scope='Planned primary parents only; no observed GT/point/mask/recall claim.')


def audit(train, val, observations, output, manifest=None):
    train, val, observations, output = map(Path, (train, val, observations, output))
    if output.exists():
        raise FileExistsError('A new audit output is required')
    source_manifest = train.parents[2] / 'manifest.json'
    if source_manifest != val.parents[2] / 'manifest.json':
        raise ValueError('Train and validation must share one frozen export')
    cache_report = observations.parent / 'report.json'
    source_lineage = json.loads(source_manifest.read_text())['source_ledger_sha256']
    if source_lineage != json.loads(cache_report.read_text())['source_ledger_sha256']:
        raise ValueError('Frozen GT / camera cache source ledger mismatch')
    files = (train, val, observations, source_manifest, cache_report)
    hashes = {str(p.resolve()): sha256(p) for p in files}
    rows = join_infos({'train': pickle.loads(train.read_bytes()),
                       'val': pickle.loads(val.read_bytes())}, json.loads(observations.read_text()))
    report = dict(source_sha256=hashes, source_ledger_sha256=source_lineage,
        captured=captured_report(rows),
        limitations=[
            'Camera support is measured foreground after the current 512x1056 transform, not detector accuracy.',
            'Counts are component GT observations; scenes, scene groups and assets are separate denominators.',
            'Scene-group count is bookkeeping, not proof of statistically independent 100m views.',
            'Ego lateral offset is not a lane ID: yaw and curvature can move a same-lane target sideways.',
            'X sign and dominant-axis sector are reported separately; neither infers world lane topology.',
            'Sparse support and 30 GT validation warning are diagnostic signals, not acceptance thresholds.',
            'Point buckets use measured current Z/XY-filtered support, not unfiltered annotation num_lidar_pts.',
            'Filtered support may be below native/annotation counts; same-ledger identity and bounded counts are verified.',
            'The old split, GT, classes and source files are never modified.'])
    if manifest:
        report['planned_highway'] = planned_report(manifest)
    if hashes != {str(p.resolve()): sha256(p) for p in files}:
        raise ValueError('Source changed during audit')
    atomic_json(output, report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('train', 'val', 'observations', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--manifest', type=Path)
    args = parser.parse_args()
    result = audit(args.train, args.val, args.observations, args.output, args.manifest)
    print(json.dumps(dict(vehicle_gt=result['captured']['vehicle_gt'],
                          warnings=result['captured']['warnings'], output=str(args.output)), ensure_ascii=False))
