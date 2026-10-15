#!/usr/bin/env python3
"""Summarize frozen A/A/compile core traces and preserve divergence windows.

No inference, live ROS, replay policy change or numeric-ID correctness claim.
The XY cost diagnostic is read-only: actual outcomes come from saved C++ core
states; it does not replace the core's global greedy assignment/tie ordering.
"""
import argparse
from collections import defaultdict
import gzip
import json
import math
from pathlib import Path

import numpy as np

from audit_tracking_continuity import stats
from audit_tracking_continuity_events import sha
from audit_tracking_continuity_saved_outputs import prepared_cases


def load_gzip(path):
    with gzip.open(path, 'rt') as handle:
        return json.load(handle)


def xy_cost(measurement, prior, maximum_distance=5.):
    """Current original XY Mahalanobis cost (not Gaussian research rank)."""
    delta = np.asarray(measurement['center'][:2])-prior['center'][:2]
    distance = float(np.linalg.norm(delta))
    xx, xy, yy = prior['covariance']
    covariance = np.asarray([[xx, xy], [xy, yy]])
    if not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).min() <= 0:
        return dict(l2_m=distance, cost=None, eligible=False,
                    note='Invalid prior covariance; do not invent a core assignment.')
    raw = math.sqrt(max(0., float(delta @ np.linalg.solve(covariance, delta))))
    cost = min(distance*3., max(distance/5., raw))
    ped_measurement = measurement['class_name'] == 'pedestrian'
    ped_track = prior['core_class'] == 3
    if (distance > maximum_distance or ped_measurement != ped_track or
            (ped_measurement and distance > 2.)):
        cost = 1000.
    return dict(l2_m=distance, raw_mahalanobis=raw, cost=cost,
                eligible=cost < maximum_distance)


def event_key(event):
    return event['stamp'], event['gt_key']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--sensor', type=Path, required=True)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    source = json.loads((args.audit/'variant_tracker_report.json').read_text())
    reference = source['reference']
    names = (reference, args.candidate)
    cases = {name: prepared_cases(args.inputs, Path(source['variants'][name]['path']),
                                 args.sensor)[0] for name in names}
    result = dict(production_changed=False, original_core_binary=source['core_binary'],
        original_core_sha256=source['core_binary_sha256'],
        report_sha256=sha(args.audit/'variant_tracker_report.json'),
        reference=reference, candidate=args.candidate,
        aggregate_output_pairs={}, aggregate_gt_groups={}, cases={},
        limits=source['limits']+[
            'Window costs are original pairwise XY costs, not rerun global greedy assignment; same-class swaps/near-actors can be ambiguous.',
            'UNKNOWN output velocity is forced zero in the actual motion-policy node; core obstacle velocity deltas alone are not planner speed regression.'])
    all_pairs = []
    all_motion = defaultdict(list)
    windows = []
    for case, summary in source['cases'].items():
        pair_rows = load_gzip(args.audit/(case+'_'+args.candidate+'_paired.json.gz'))['outputs']
        pair_rows = [dict(row, case=case) for row in pair_rows]
        all_pairs.extend(pair_rows)
        cores = {name: load_gzip(args.audit/(case+'_'+name+'_core.json.gz')) for name in names}
        evidence = {name: load_gzip(args.audit/(case+'_'+name+'_evidence.json.gz')) for name in names}
        for name in names:
            all_motion[name].extend(dict(row, case=case) for row in evidence[name]['motion']['matched'])
        selectors = []
        if pair_rows:
            for field in ('center_m', 'yaw_axis_deg'):
                row = max(pair_rows, key=lambda x: x[field])
                selectors.append(dict(stamp=row['stamp'], reason='maximum_'+field, pair=row))
        reference_switches = {event_key(row) for row in summary['variants'][reference]['catalog_gate2m']['switch_events']}
        for event in summary['variants'][args.candidate]['catalog_gate2m']['switch_events']:
            if event_key(event) not in reference_switches:
                selectors.append(dict(stamp=event['stamp'], reason='candidate_additional_gt_id_switch', event=event))
        reference_duplicates = {event_key(row) for row in evidence[reference]['catalog']['multiplicity_candidates']}
        for event in evidence[args.candidate]['catalog']['multiplicity_candidates']:
            if event_key(event) not in reference_duplicates:
                selectors.append(dict(stamp=event['stamp'], reason='candidate_additional_gt_multiplicity',
                                      event={k: v for k, v in event.items() if k != 'objects'}))
        seen_stamps = set()
        selections = []
        for selector in selectors:
            if selector['stamp'] in seen_stamps:
                continue
            seen_stamps.add(selector['stamp'])
            selections.append(selector)
            window = dict(case=case, selector=selector, variants={})
            for name in names:
                detections = {r['stamp']: r for r in cases[name][case]['frames']}
                selected = []
                for row in cores[name]:
                    if abs(row['stamp']-selector['stamp']) > .65:
                        continue
                    det = detections[row['stamp']]
                    pairwise = [dict(detection_index=i, track_id=track['id'],
                        **xy_cost(measurement, track)) for i, measurement in enumerate(det['world_objects'])
                        for track in row['before']
                        if math.dist(measurement['center'][:2], track['center'][:2]) <= 5.]
                    selected.append(dict(core=row, detector=det, pairwise_original_cost=pairwise))
                window['variants'][name] = selected
            windows.append(window)
        result['cases'][case] = dict(input_frames=summary['input_frames'],
            same_contract_eager_aa_exact=summary['comparisons'].get('nearest_eager_1', {}).get('exact_same_core_trace'),
            nearest_compile_pair_stats=summary['comparisons'][args.candidate]['same_class_paired_core_outputs'],
            selected_windows=selections,
            variant_metrics={name: dict(catalog=summary['variants'][name]['catalog_gate2m']['primary_class_preserving'],
                switches=summary['variants'][name]['catalog_gate2m']['switch_candidates'],
                multiplicity=summary['variants'][name]['catalog_gate2m']['same_class_multiple_uuids_near_one_clear_gt'],
                matched_coast=summary['variants'][name]['velocity_and_coast']['groups']['all']['coasting'],
                unmatched_candidates=summary['variants'][name]['velocity_and_coast']['unmatched_track_candidates']) for name in names})
    for cls in ('all', 'vehicle', 'pedestrian', 'obstacle'):
        rows = [r for r in all_pairs if cls == 'all' or r['class_name'] == cls]
        result['aggregate_output_pairs'][cls] = dict(
            count=len(rows), **{field: stats([r[field] for r in rows]) for field in
                ('center_m', 'velocity_delta_mps', 'yaw_axis_deg', 'size_linf_m', 'score_abs')},
            worst_xy=sorted(rows, key=lambda x: -x['center_m'])[:10],
            worst_yaw=sorted(rows, key=lambda x: -x['yaw_axis_deg'])[:10])
    for name, rows in all_motion.items():
        group_names = ['all', 'vehicle', 'pedestrian', 'obstacle']+sorted({r['class_name']+'/'+r['band'] for r in rows})
        result['aggregate_gt_groups'][name] = {}
        for group in group_names:
            selected = [r for r in rows if group == 'all' or r['class_name'] == group or r['class_name']+'/'+r['band'] == group]
            result['aggregate_gt_groups'][name][group] = dict(
                matched=len(selected), coasting=sum(not r['associated'] for r in selected),
                **{field: stats([r[field] for r in selected if r.get(field) is not None]) for field in
                    ('center_error_m', 'velocity_error_mps', 'post_unknown_policy_velocity_error_mps',
                     'last_association_age_sec')})
    args.output.mkdir(parents=True)
    with gzip.open(args.output/'divergence_windows.json.gz', 'wt') as handle:
        json.dump(windows, handle, allow_nan=False)
    (args.output/'review.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output), windows=len(windows),
                         paired=len(all_pairs), classes={k: v['count'] for k, v in result['aggregate_output_pairs'].items()})))


if __name__ == '__main__':
    main()
