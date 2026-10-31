#!/usr/bin/env python3
"""Compare native compound evidence without relabelling or changing QA.

The baseline is a dataset composition audit; the live set is a factor-targeted
diagnostic, not a random sample, detector evaluation, or matched-pose A/B.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

import numpy as np

from base5_capture_safety import atomic_json, sha256


LIMITATIONS = [
    'Baseline composition and targeted diagnostics have different sampling; raw rates are not a causal improvement estimate.',
    'Parent local +X observation angle is not a certified human- or child-mesh facing direction.',
    'Small Pedestrian leaf is a geometric diagnostic stratum, not a new semantic label or a native child-ID match.',
    'BBox returns and sibling-exclusive returns are geometric evidence, not semantic point-instance labels.',
    'Fully shared points can be legitimate nested geometry; identical-geometry review and legitimate overlap are separate.',
    'Same-class camera correspondence remains unknown; native camera child suffixes are not LiDAR ordinals.',
    'Saved positive is the historical capture decision, not a recomputed 512-crop training label.',
    'No source GT, acceptance policy, selected ledger or training labels are modified.',
]


def quantiles(values):
    values = [float(x) for x in values if x is not None]
    if not values:
        return dict(n=0)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Nonfinite comparison value')
    return dict(n=len(values), mean=float(np.mean(values)),
                p50=float(np.percentile(values, 50)), p90=float(np.percentile(values, 90)),
                p95=float(np.percentile(values, 95)), maximum=max(values), minimum=min(values))


def distance_band(distance):
    if distance is None:
        return 'unknown'
    if not math.isfinite(distance) or distance < 0:
        raise ValueError('Invalid component distance')
    for low, high in ((0, 10), (10, 20), (20, 30), (30, 50), (50, 80), (80, 100)):
        if low <= distance < high:
            return f'{low}_{high}m'
    return '100m_plus'


def component_stratum(parent, component):
    if parent['asset'].startswith('BabyStroller'):
        if component['native_raw_class'] == 'Obstacle':
            return 'stroller_body_obstacle'
        if component['native_raw_class'] == 'Pedestrian':
            return ('stroller_small_pedestrian_leaf' if component['size_m'][2] < .9
                    else 'stroller_large_pedestrian_leaf')
    return component['native_raw_class']


def parent_view(metadata, parent_uid):
    actor = next((x for x in metadata.get('runtime_actors', [])
                  if int(x['unique_id']) == int(parent_uid)), None)
    if not actor or 'geometric_center_estimate' not in actor or 'placement_yaw_deg' not in actor:
        return dict(view='unknown', angle_deg=None)
    ego = metadata['ego']
    center = actor['geometric_center_estimate']
    dx, dy = ego['x'] - center[0], ego['y'] - center[1]
    if not all(math.isfinite(x) for x in (dx, dy, actor['placement_yaw_deg'])):
        raise ValueError('Nonfinite parent observation geometry')
    if math.hypot(dx, dy) < .1:
        return dict(view='unknown', angle_deg=None)
    angle = (math.degrees(math.atan2(dy, dx)) - actor['placement_yaw_deg'] + 180.) % 360. - 180.
    view = ('front' if abs(angle) <= 35. else 'rear' if abs(angle) >= 145.
            else 'side' if abs(abs(angle)-90.) <= 35. else 'oblique')
    return dict(view=view, angle_deg=angle)


def records(report, metadata_by_root):
    rows = []
    for capture in report['captures']:
        metadata = metadata_by_root.get(capture['capture_root'], {})
        for parent in capture['parents']:
            view = parent_view(metadata, parent['parent_uid'])
            for component in parent['components']:
                points = component['bbox_point_count']
                exclusive = component['sibling_exclusive_bbox_points']
                shared = component['shared_sibling_bbox_points']
                if min(points, exclusive, shared) < 0 or points != exclusive + shared:
                    raise ValueError('Inconsistent sibling point partition')
                body_ids = {c['component_id'] for c in parent['components']
                            if c['native_raw_class'] == 'Obstacle'
                            and c['component_id'] != component['component_id']}
                fully_shared_with_body = points > 0 and any(
                    component['component_id'] in pair['component_ids'] and
                    bool(body_ids.intersection(pair['component_ids'])) and
                    pair['shared_bbox_point_count'] == points
                    for pair in parent['same_parent_pairs'])
                rows.append(dict(capture_root=capture['capture_root'],
                    scene_id=capture.get('scene_id'), parent_uid=parent['parent_uid'],
                    asset=parent['asset'], component_id=component['component_id'],
                    stratum=component_stratum(parent, component), view=view['view'],
                    observer_angle_deg=view['angle_deg'],
                    distance_band=distance_band(component.get('range_xy_m')),
                    range_xy_m=component.get('range_xy_m'), size_m=component['size_m'],
                    points=points, exclusive_points=exclusive, shared_points=shared,
                    fully_shared_with_body=fully_shared_with_body,
                    camera_support_state=component['camera_support_state'],
                    diagnostic_sensor_state=component['diagnostic_sensor_state'],
                    saved_positive=component['saved_positive']))
    return rows


def summarize_rows(rows):
    positive_lidar = [r for r in rows if r['points'] > 0]
    return dict(components=len(rows), distinct_captures=len({r['capture_root'] for r in rows}),
        distinct_parents=len({(r['capture_root'], r['parent_uid']) for r in rows}),
        saved_positive=sum(r['saved_positive'] for r in rows),
        lidar_positive=len(positive_lidar), lidar20_plus=sum(r['points'] >= 20 for r in rows),
        zero_exclusive_among_lidar_positive=sum(r['exclusive_points'] == 0 for r in positive_lidar),
        fully_shared_with_body=sum(r['fully_shared_with_body'] for r in rows),
        camera_correspondence_unknown=sum(r['camera_support_state'] == 'unknown_component_correspondence'
                                          for r in rows),
        camera_unknown_zero_lidar=sum(r['camera_support_state'] == 'unknown_component_correspondence'
                                      and r['points'] == 0 for r in rows),
        views=dict(sorted(Counter(r['view'] for r in rows).items())),
        distance_bands=dict(sorted(Counter(r['distance_band'] for r in rows).items())),
        sensor_states=dict(sorted(Counter(r['diagnostic_sensor_state'] for r in rows).items())),
        range_xy_m=quantiles(r['range_xy_m'] for r in rows),
        bbox_points=quantiles(r['points'] for r in rows),
        exclusive_points=quantiles(r['exclusive_points'] for r in rows),
        shared_fraction_among_lidar_positive=quantiles(r['shared_points']/r['points'] for r in positive_lidar),
        size_m={axis: quantiles(r['size_m'][index] for r in rows)
                for index, axis in enumerate(('x', 'y', 'z'))})


def summarize_report(report, metadata_by_root, source_type, dataset_captures=None):
    if not report.get('diagnostic_only') or not report.get('labels_and_acceptance_unchanged'):
        raise ValueError('Expected a read-only compound evidence report')
    roots = [c['capture_root'] for c in report['captures']]
    if len(roots) != len(set(roots)):
        raise ValueError('Duplicate captures in one evidence report')
    rows = records(report, metadata_by_root)
    groups = defaultdict(list)
    for row in rows:
        key = (row['asset'], row['stratum'], row['distance_band'], row['view'])
        groups[key].append(row)
    by_stratum = defaultdict(list)
    for row in rows:
        by_stratum[row['stratum']].append(row)
    result = dict(source_type=source_type, observations=summarize_rows(rows),
        by_component_stratum={k: summarize_rows(v) for k, v in sorted(by_stratum.items())},
        stratified=[dict(asset=key[0], component_stratum=key[1], distance_band=key[2],
                         parent_observer_view=key[3], **summarize_rows(values))
                    for key, values in sorted(groups.items())],
        identical_geometry_review_pairs=sum(pair['same_class_identical_geometry_review']
            for c in report['captures'] for p in c['parents'] for pair in p['same_parent_pairs']),
        compound_captures=len(roots), source_report_counts=report['counts'])
    if dataset_captures is not None:
        if dataset_captures < len(roots):
            raise ValueError('Dataset capture count is below observed compound captures')
        result.update(dataset_captures=dataset_captures,
            compound_capture_fraction=len(roots)/dataset_captures if dataset_captures else None)
    return result


def load_metadata(report):
    result = {}
    for capture in report['captures']:
        path = Path(capture['capture_root'])/'metadata.json'
        if sha256(path) != capture['metadata_sha256']:
            raise ValueError('Capture metadata changed after source evidence report')
        result[capture['capture_root']] = json.loads(path.read_text())
    return result


def compare(baseline, diagnostic, baseline_metadata, diagnostic_metadata, dataset_captures=5000):
    if baseline['profile'] != diagnostic['profile']:
        raise ValueError('Cannot compare different camera diagnostic profiles')
    if ({c['capture_root'] for c in baseline['captures']} &
            {c['capture_root'] for c in diagnostic['captures']}):
        raise ValueError('Baseline and diagnostic captures overlap')
    return dict(schema=1, diagnostic_only=True, labels_and_acceptance_unchanged=True,
        profile=baseline['profile'],
        baseline=summarize_report(baseline, baseline_metadata, 'dataset_composition', dataset_captures),
        diagnostic=summarize_report(diagnostic, diagnostic_metadata, 'targeted_static_diagnostic'),
        limitations=LIMITATIONS)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--diagnostic', type=Path, required=True)
    parser.add_argument('--dataset-captures', type=int, default=5000)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline = json.loads(args.baseline.read_text())
    diagnostic = json.loads(args.diagnostic.read_text())
    output = args.output.resolve()
    if output in (args.baseline.resolve(), args.diagnostic.resolve()):
        raise ValueError('Do not overwrite source evidence reports')
    for capture in baseline['captures'] + diagnostic['captures']:
        root = Path(capture['capture_root']).resolve()
        if output == root or root in output.parents:
            raise ValueError('Do not write diagnostics inside a source capture')
    result = compare(baseline, diagnostic, load_metadata(baseline), load_metadata(diagnostic),
                     args.dataset_captures)
    result['sources'] = {kind: dict(path=str(path.resolve()), sha256=sha256(path))
                         for kind, path in (('baseline', args.baseline), ('diagnostic', args.diagnostic))}
    atomic_json(output, result)
    print(json.dumps(dict(output=str(output),
        baseline_components=result['baseline']['observations']['components'],
        diagnostic_components=result['diagnostic']['observations']['components'])))


if __name__ == '__main__':
    main()
