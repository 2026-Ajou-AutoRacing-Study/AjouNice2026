#!/usr/bin/env python3
"""Frozen model variants through the original EKF core, no policy retuning.

Prepared inputs/actor geometry and measurement cadence are shared. Numeric
IDs from independent runs are not compared as identities: GT correspondence
and class-preserving output matching provide separate, bounded diagnostics.
"""
import argparse
from bisect import bisect_left
from collections import defaultdict
import gzip
import json
import math
from pathlib import Path
import pickle

import numpy as np

from audit_tracking_continuity import ROOT, TRACK, evaluate_gt, match, output_identity_summary, stats
from audit_tracking_continuity_events import replay, sha
from audit_tracking_continuity_packets import cadence, confirmed_frames, lifecycle, paired_correspondence
from audit_tracking_continuity_saved_outputs import prepared_cases

CLASSES = ('vehicle', 'pedestrian', 'obstacle')


def declared_infos_sha(model):
    declarations = [model.get('info_sha256'), model.get('source_lineage', {}).get('original_infos_sha256')]
    declarations = [value for value in declarations if value is not None]
    if not declarations or len(set(declarations)) != 1:
        raise ValueError('Missing or contradictory original infos SHA declarations')
    return declarations[0]


def finite_difference_velocity(series, stamp, maximum_step=.25):
    """BBox-center secant velocity, not UDP instantaneous rear-axle velocity."""
    times, points = series
    i = bisect_left(times, stamp)
    if i == len(times) or abs(times[i]-stamp) > 1.e-8:
        return None
    low = i-1 if i and times[i]-times[i-1] <= maximum_step else i
    high = i+1 if i+1 < len(times) and times[i+1]-times[i] <= maximum_step else i
    if low == high:
        return None
    return ((np.asarray(points[high]['center'][:2])-points[low]['center'][:2]) /
            (times[high]-times[low])).tolist()


def state_lookup(core):
    return {(f['stamp'], str(t['id'])): t for f in core for t in f['tracks'] if t['confirmed']}


def velocity_and_coast(core, evidence, truth):
    state = state_lookup(core)
    actors = {series[1][0]['key']: series for series in truth['gt'] if series[1]}
    rows = []
    for matched in evidence['matched_rows']:
        tracked = state[matched['stamp'], matched['id']]
        velocity = finite_difference_velocity(actors[matched['gt_key']], matched['stamp'])
        current = dict(matched, core_velocity=tracked['velocity'], associated=tracked['associated'],
                       last_association_age_sec=matched['stamp']-tracked['last_measurement_time'],
                       gt_velocity_estimate=velocity,
                       velocity_error_mps=math.dist(velocity, tracked['velocity']) if velocity is not None else None,
                       speed_error_abs_mps=abs(math.hypot(*velocity)-math.hypot(*tracked['velocity'])) if velocity is not None else None)
        # Production UNKNOWN motion-policy projection, not a modification of
        # the diagnostic core. The other classes retain their core velocity.
        effective = [0., 0.] if matched['class_name'] == 'obstacle' else tracked['velocity']
        current['post_unknown_policy_velocity_error_mps'] = math.dist(velocity, effective) if velocity is not None else None
        rows.append(current)
    summary = {}
    groups = ['all', *CLASSES]+sorted({r['class_name']+'/'+r['band'] for r in rows})
    for group in groups:
        selected = [r for r in rows if group == 'all' or group == r['class_name'] or group == r['class_name']+'/'+r['band']]
        summary[group] = dict(matched=len(selected), associated=sum(r['associated'] for r in selected),
            coasting=sum(not r['associated'] for r in selected),
            velocity_error_mps=stats([r['velocity_error_mps'] for r in selected if r['velocity_error_mps'] is not None]),
            speed_error_abs_mps=stats([r['speed_error_abs_mps'] for r in selected if r['speed_error_abs_mps'] is not None]),
            post_unknown_policy_velocity_error_mps=stats([r['post_unknown_policy_velocity_error_mps'] for r in selected
                if r['post_unknown_policy_velocity_error_mps'] is not None]),
            matched_last_association_age_sec=stats([r['last_association_age_sec'] for r in selected]),
            coast_center_error_m=stats([r['center_error_m'] for r in selected if not r['associated']]))
    unmatched = []
    for row in evidence['unmatched_tracks']:
        tracked = state[row['stamp'], row['id']]
        unmatched.append(dict(row, associated=tracked['associated'], velocity=tracked['velocity'],
                              last_association_age_sec=row['stamp']-tracked['last_measurement_time']))
    unmatched_summary = {cls: dict(observations=sum(r['class_name'] == cls for r in unmatched),
        coasting=sum(r['class_name'] == cls and not r['associated'] for r in unmatched),
        last_association_age_sec=stats([r['last_association_age_sec'] for r in unmatched if r['class_name'] == cls])) for cls in CLASSES}
    return dict(groups=summary, unmatched_track_candidates=unmatched_summary), dict(matched=rows, unmatched=unmatched)


def paired_outputs(before, after, gate=2.):
    if [r['stamp'] for r in before] != [r['stamp'] for r in after]:
        raise ValueError('Variant core timelines differ')
    rows, counts = [], defaultdict(lambda: defaultdict(int))
    for reference, candidate in zip(before, after):
        stamp = reference['stamp']
        pairs, missing, extra = match(reference['objects'], candidate['objects'], gate)
        for o in reference['objects']:
            counts[o['class_name']]['reference'] += 1
        for o in candidate['objects']:
            counts[o['class_name']]['candidate'] += 1
        for i in missing:
            counts[reference['objects'][i]['class_name']]['reference_unpaired'] += 1
        for i in extra:
            counts[candidate['objects'][i]['class_name']]['candidate_unpaired'] += 1
        for i, j, error in pairs:
            a, b = reference['objects'][i], candidate['objects'][j]
            counts[a['class_name']]['paired'] += 1
            rows.append(dict(stamp=stamp, class_name=a['class_name'], reference_id=a['id'], candidate_id=b['id'],
                center_m=error, velocity_delta_mps=math.dist(a['velocity'], b['velocity']),
                yaw_axis_deg=math.degrees(abs((a['yaw']-b['yaw']+math.pi/2) % math.pi-math.pi/2)),
                size_linf_m=max(abs(x-y) for x, y in zip(a['size'], b['size'])), score_abs=abs(a['score']-b['score']),
                association_age_abs_sec=abs(a['last_measurement_time']-b['last_measurement_time']),
                reference_coasting=not a['associated'], candidate_coasting=not b['associated']))
    summary = {}
    for cls in CLASSES:
        selected = [r for r in rows if r['class_name'] == cls]
        fixed_counts = {key: counts[cls][key] for key in
                        ('reference', 'candidate', 'paired', 'reference_unpaired', 'candidate_unpaired')}
        summary[cls] = dict(fixed_counts, **{field: stats([r[field] for r in selected]) for field in
            ('center_m', 'velocity_delta_mps', 'yaw_axis_deg', 'size_linf_m', 'score_abs', 'association_age_abs_sec')},
            reference_coasting=sum(r['reference_coasting'] for r in selected),
            candidate_coasting=sum(r['candidate_coasting'] for r in selected))
    return dict(gate_m=gate, classes=summary,
                note='Pairing is geometric/class-preserving. Numeric IDs across independent runs are not a switch metric.'), rows


def write_gzip(path, value):
    with gzip.open(path, 'wt') as handle:
        json.dump(value, handle, allow_nan=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--sensor', type=Path, required=True)
    p.add_argument('--core-binary', type=Path, required=True)
    p.add_argument('--variant', action='append', required=True, help='unique_name=/path/predictions.json.gz')
    p.add_argument('--reference', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--saved-fixture-selfcheck', action='store_true',
                   help='Explicitly mark repeated saved file checks as harness validation, not independent GPU A/A')
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Fresh output required')
    variants = {}
    for value in a.variant:
        name, path = value.split('=', 1)
        if not name or name in variants or not name.replace('_', '').isalnum():
            raise ValueError('Unique filename-safe variant name required')
        variants[name] = Path(path)
    if a.reference not in variants:
        raise ValueError('Reference is not a supplied variant')
    a.output.mkdir(parents=True)
    inputs_sha = sha(a.inputs/'infos_val.pkl')
    original_infos = {r['token']: r for r in pickle.loads((a.inputs/'infos_val.pkl').read_bytes())['data_list']}
    report = dict(core_binary=str(a.core_binary), core_binary_sha256=sha(a.core_binary),
        core_build_receipt=json.loads((a.core_binary.parent/'build_receipt.json').read_text()),
        inputs_infos_sha256=inputs_sha, inputs_manifest_sha256=sha(a.inputs/'manifest.json'),
        sensor_sha256=sha(a.sensor), reference=a.reference, variants={}, cases={},
        fixed_operating_contract=dict(server_floor=.1, top100=True, rotated_nms_iou=.5, vehicle=.4, pedestrian=.1, obstacle=.1),
        saved_fixture_selfcheck=a.saved_fixture_selfcheck, production_changed=False,
        limits=['Original core at prepared measurement stamps only; not 30Hz scheduler, LAN queue, TF scheduling, planner or closed-loop certification.',
                'Prepared input omissions and retained gaps (including any multi-second gap) are preserved, not filled by synthetic detector frames.',
                'Prepared catalog actor correspondence is not visibility-qualified native AP. Unmatched/coasting output is not a proven false-positive remnant.',
                'GT center velocity is a central/one-sided secant from prepared world BBox centers, max adjacent step .25s; not UDP instantaneous rear-axle velocity.',
                'Class-preserving geometric output pairing can be ambiguous in nearby same-class actors. It is separate from actor-matched ID histories.',
                'Historical detector accuracy, native val and independently repeated CUDA A/A are different evidence.',
                'UNKNOWN effective velocity error is a zero-motion projection only; no production node was run or modified.'])
    report['limits'].append('Predicted Z retains the original MMDet bottom-center export/production bridge convention; GT keeps catalog geometric center. Only XY/XY velocity/axis yaw are scored, not GT Z or ground contact.')
    prepared, models = {}, {}
    for name, path in variants.items():
        source_payload = json.load(gzip.open(path, 'rt'))
        for row in source_payload['rows']:
            if 'stamp' in row and row['stamp'] != original_infos[row['token']]['timestamp']:
                raise ValueError('Variant declared measurement timestamp changed: '+row['token'])
        prepared[name], models[name] = prepared_cases(a.inputs, path, a.sensor)
        if declared_infos_sha(models[name]) != inputs_sha:
            raise ValueError('Variant declared inputs SHA differs: '+name)
        if models[name].get('sensor_sha256', sha(a.sensor)) != sha(a.sensor):
            raise ValueError('Variant sensor bytes differ from the replay calibration: '+name)
        if models[name].get('classes') != list(CLASSES):
            raise ValueError('Variant class mapping changed: '+name)
        report['variants'][name] = dict(path=str(path), sha256=sha(path), recorded_model=models[name])
    checkpoint = {m['checkpoint_sha256'] for m in models.values()}
    if len(checkpoint) != 1:
        raise ValueError('Variant checkpoints differ')
    reference_cases = prepared[a.reference]
    for case_name, reference_case in reference_cases.items():
        result = dict(input_frames=len(reference_case['frames']), raw_bag=reference_case['raw_bag'],
                      input_timeline=cadence(reference_case['frames']),
                      original_frame_resolution=reference_case['original_frame_resolution'], variants={}, comparisons={})
        report['cases'][case_name] = result
        core_rows, confirmed, evidence_rows = {}, {}, {}
        for name in variants:
            case = prepared[name][case_name]
            if case['gt'] != reference_case['gt'] or [f['stamp'] for f in case['frames']] != [f['stamp'] for f in reference_case['frames']]:
                raise ValueError('Variant GT/measurement times changed')
            core_rows[name] = replay(a.core_binary, case['frames'])
            confirmed[name] = confirmed_frames(core_rows[name])
            audit_case = dict(case['gt'], frames={TRACK: confirmed[name]})
            summary, evidence_rows[name] = evaluate_gt(audit_case, 2.)
            motion_summary, motion_evidence = velocity_and_coast(core_rows[name], evidence_rows[name], case['gt'])
            life_summary, life_evidence = lifecycle(core_rows[name])
            result['variants'][name] = dict(catalog_gate2m=summary, velocity_and_coast=motion_summary,
                lifecycle=life_summary, identity=output_identity_summary(confirmed[name]),
                gate_sensitivity={str(g): evaluate_gt(audit_case, g)[0] for g in (1., 3.)})
            write_gzip(a.output/(case_name+'_'+name+'_core.json.gz'), core_rows[name])
            write_gzip(a.output/(case_name+'_'+name+'_evidence.json.gz'),
                       dict(catalog=evidence_rows[name], motion=motion_evidence, lifecycle=life_evidence))
        anchors = {f['stamp']: f['stamp'] for f in reference_case['frames']}
        for name in variants:
            paired, paired_rows = paired_outputs(confirmed[a.reference], confirmed[name])
            gt_pair, gt_pair_rows = paired_correspondence(evidence_rows[a.reference], evidence_rows[name], anchors, anchors)
            # Exact core equality is informative for A/A but not our primary
            # cross-variant identity score; IDs can differ after early birth.
            result['comparisons'][name] = dict(same_class_paired_core_outputs=paired,
                same_actor_paired_gt_error=gt_pair,
                exact_same_core_trace=core_rows[a.reference] == core_rows[name],
                note='Exact same trace is a reproducibility diagnostic, not numeric-ID-based cross-variant correctness.')
            write_gzip(a.output/(case_name+'_'+name+'_paired.json.gz'), dict(outputs=paired_rows, catalog=gt_pair_rows))
        print(case_name, {name: dict(matched=v['catalog_gate2m']['primary_class_preserving'].get('all', {}).get('matched_observations', 0),
            switches=v['catalog_gate2m']['switch_candidates'], duplicates=v['catalog_gate2m']['same_class_multiple_uuids_near_one_clear_gt'])
            for name, v in result['variants'].items()}, flush=True)
    (a.output/'variant_tracker_report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
