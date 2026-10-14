#!/usr/bin/env python3
"""Original EKF core replay of common-anchor offline packet model outputs.

No live ROS, new inference, production edits, policy/threshold changes, or
30Hz publisher simulation. The eligible common subset preserves its recorded
gaps. Each policy's declared measurement stamp determines both core dt and
bounded Ego pose interpolation; the shared footer is only a pairing anchor.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import gzip
import json
import math
from pathlib import Path

import numpy as np
import yaml

from audit_tracking_continuity import (
    ROOT, TRACK, evaluate_gt, output_identity_summary, sample_series, stats)
from audit_tracking_continuity_events import build_original, core_summary, replay, sha
from analyze_lidar_packet_candidates import POLICIES, catalog_series, operating, to_world


CLASSES = ('vehicle', 'pedestrian', 'obstacle')


def core_class(label):
    return 'pedestrian' if label in (3, 4) else 'vehicle' if label in (1, 2) else 'obstacle'


def bounded_ego(path):
    import rosbag
    poses = {}
    with rosbag.Bag(str(path)) as bag:
        for _, m, _ in bag.read_messages(topics=['/Ego_topic']):
            poses[m.header.stamp.to_sec()] = dict(
                center=[float(m.position.x), float(m.position.y), float(m.position.z)],
                yaw=math.radians(m.heading-1.347))
    times = sorted(poses)
    return times, [poses[t] for t in times]


def confirmed_frames(rows):
    return [dict(stamp=f['stamp'], objects=[dict(t, class_name=core_class(t['core_class']),
                 id=str(t['id'])) for t in f['tracks'] if t['confirmed']]) for f in rows]


def lifecycle(rows):
    """Core birth/death/coast evidence, not GT identity or publisher lifetime."""
    births, confirmations, deaths, coasting = [], [], [], []
    previous = {}
    count = Counter()
    for frame in rows:
        now = {t['id']: t for t in frame['tracks']}
        for identity in now.keys()-previous.keys():
            t = now[identity]
            births.append(dict(stamp=frame['stamp'], id=identity,
                               class_name=core_class(t['core_class']), state=t))
        for identity, t in now.items():
            cls = core_class(t['core_class'])
            if t['confirmed'] and (identity not in previous or not previous[identity]['confirmed']):
                confirmations.append(dict(stamp=frame['stamp'], id=identity, class_name=cls))
            if t['confirmed']:
                count[cls+'/confirmed_observations'] += 1
                count[cls+('/associated' if t['associated'] else '/coasting')] += 1
                if not t['associated']:
                    coasting.append(dict(stamp=frame['stamp'], id=identity, class_name=cls,
                                         last_association_gap_sec=frame['stamp']-t['last_measurement_time']))
        for identity in previous.keys()-now.keys():
            t = previous[identity]
            deaths.append(dict(stamp=frame['stamp'], id=identity,
                               class_name=core_class(t['core_class']), confirmed=t['confirmed'],
                               last_association_gap_sec=frame['stamp']-t['last_measurement_time'],
                               previous_state=t))
        previous = now
    summary = dict(counts=dict(count), births_by_class=dict(Counter(r['class_name'] for r in births)),
        deaths_by_class=dict(Counter(r['class_name'] for r in deaths)),
        confirmations_by_class=dict(Counter(r['class_name'] for r in confirmations)),
        coasting_last_association_gap_sec={cls: stats([r['last_association_gap_sec'] for r in coasting
                                                     if r['class_name'] == cls]) for cls in CLASSES},
        confirmed_deletion_last_association_gap_sec={cls: stats([r['last_association_gap_sec'] for r in deaths
                                  if r['class_name'] == cls and r['confirmed']]) for cls in CLASSES})
    return summary, dict(births=births, confirmations=confirmations, deaths=deaths, coasting=coasting)


def cadence(frames):
    stamps = [f['stamp'] for f in frames]
    gaps = [b-a for a, b in zip(stamps, stamps[1:])]
    return dict(first_stamp=stamps[0], last_stamp=stamps[-1], frames=len(frames),
                span_sec=stamps[-1]-stamps[0], interframe_sec=stats(gaps),
                gaps_over_threshold_sec={str(t): sum(g > t for g in gaps) for t in (.15, .25, .5, 1.)})


def eligible_pairs(groups, ego):
    """Exclude an entire common anchor if any policy lacks its own pose."""
    kept, excluded = {}, []
    for pair, group in groups.items():
        failures = []
        for policy, row in group.items():
            if sample_series(ego, row['stamp'], .15) is None:
                index = bisect_left(ego[0], row['stamp'])
                previous = ego[0][index-1] if index else None
                following = ego[0][index] if index < len(ego[0]) else None
                failures.append(dict(policy=policy, token=row['token'], stamp=row['stamp'],
                    previous_ego_stamp=previous, next_ego_stamp=following,
                    ego_bracket_gap_sec=following-previous if previous is not None and following is not None else None))
        if failures:
            excluded.append(dict(pair=pair, failures=failures))
        else:
            kept[pair] = group
    return kept, excluded


def paired_correspondence(reference, candidate, reference_pairs, candidate_pairs):
    """Same actor/common anchor only; never compare independent numeric IDs."""
    def index(evidence, anchors):
        return {(anchors[r['stamp']], r['gt_key']): r for r in evidence['matched_rows']}
    before, after = index(reference, reference_pairs), index(candidate, candidate_pairs)
    common = before.keys() & after.keys()
    rows = [dict(pair=key[0], gt_key=key[1], class_name=before[key]['class_name'],
                 baseline_band=before[key]['band'], baseline_stamp=before[key]['stamp'],
                 candidate_stamp=after[key]['stamp'],
                 baseline_error_m=before[key]['center_error_m'], candidate_error_m=after[key]['center_error_m'],
                 error_delta_candidate_minus_baseline_m=after[key]['center_error_m']-before[key]['center_error_m'])
            for key in sorted(common)]
    groups = ['all', *CLASSES]
    groups += sorted({r['class_name']+'/'+r['baseline_band'] for r in rows})
    summary = {}
    for group in groups:
        chosen = [r for r in rows if group == 'all' or group == r['class_name'] or
                  group == r['class_name']+'/'+r['baseline_band']]
        summary[group] = dict(paired_actor_observations=len(chosen),
            baseline_error_m=stats([r['baseline_error_m'] for r in chosen]),
            candidate_error_m=stats([r['candidate_error_m'] for r in chosen]),
            error_delta_candidate_minus_baseline_m=stats([r['error_delta_candidate_minus_baseline_m'] for r in chosen]))
    return dict(groups=summary, baseline_only_matched=len(before.keys()-after.keys()),
                candidate_only_matched=len(after.keys()-before.keys())), rows


def switch_lifecycle_evidence(events, rows, raw_life, parameters):
    """An output-match gap is not a gap since the last detector update."""
    birth = {str(r['id']): r for r in raw_life['births']}
    death = {str(r['id']): r for r in raw_life['deaths']}
    findings = []
    for event in events:
        old, new = event['previous_id'], event['id']
        expired = death.get(old)
        old_state = expired['previous_state'] if expired else None
        timeout = None
        if old_state:
            cls = core_class(old_state['core_class'])
            stationary = cls == 'obstacle' or (cls == 'vehicle' and
                math.hypot(*old_state['velocity']) <= parameters['lifecycle_stationary_speed_threshold_mps'])
            timeout = parameters['confirmed_stationary_track_max_coast_time_sec'] if stationary else parameters['confirmed_track_max_coast_time_sec']
        coexistence = [f['stamp'] for f in rows if
            {old, new} <= {str(t['id']) for t in f['tracks'] if t['confirmed']}]
        findings.append(dict(event=event, old_death=expired, new_birth=birth.get(new),
            old_last_update_to_deletion_sec=expired['last_association_gap_sec'] if expired else None,
            old_configured_timeout_sec=timeout,
            old_deleted_before_new_gt_match=bool(expired and expired['stamp'] <= event['stamp']),
            old_deletion_timeout_consistent=bool(expired and expired['confirmed'] and
                timeout is not None and expired['last_association_gap_sec'] > timeout),
            confirmed_old_new_coexistence_stamps=coexistence,
            note='A short GT-matched output gap can cross ordinary coasting expiry; it is not proof of association stealing.'))
    return findings


def write_gzip(path, value):
    with gzip.open(path, 'wt') as handle:
        json.dump(value, handle, allow_nan=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--predictions', type=Path, required=True)
    p.add_argument('--parameters', type=Path, required=True)
    p.add_argument('--sensor', type=Path, default=ROOT/'configs/sensors/Sensor_competition_local.json')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Fresh output required')
    a.output.mkdir(parents=True)
    inputs = json.loads(a.inputs.read_text())
    model = json.loads(a.predictions.read_text())
    if [r['token'] for r in inputs['rows']] != [r['token'] for r in model['rows']]:
        raise ValueError('Prediction/input order mismatch')
    if sha(a.inputs) != model['input_sha256']:
        raise ValueError('Prediction input checksum mismatch')
    from app.sensor4_calibration import load_sensor4_calibration
    matrix = load_sensor4_calibration(a.sensor, ['camera_'+n for n in
        ('front', 'front_left', 'front_right', 'back')])['base_from_lidar']
    if not np.allclose(matrix[:3, :3], np.eye(3)):
        raise ValueError('Recorded mounting rotation must be zero for the paired helper')
    if sha(a.sensor) != model['sensor_sha256']:
        raise ValueError('Sensor checksum differs from model inference')
    parameters = yaml.safe_load(a.parameters.read_text())['configure']
    binary, build = build_original(a.output/'core_build', parameters)
    report = dict(inputs_sha256=sha(a.inputs), predictions_sha256=sha(a.predictions),
        parameters_source=str(a.parameters), parameters_sha256=sha(a.parameters), configure=parameters,
        sensor_source=str(a.sensor), sensor_sha256=sha(a.sensor), tracker_build=build,
        checkpoint_sha256=model['checkpoint_sha256'], source_model_config=model['config'],
        model_inference_settings={k: model[k] for k in ('amp', 'parallel_camera', 'compile')},
        scope='Original core at each policy declared measurement timeline; common eligible paired subset.',
        operating_contract=dict(server_floor=.1, top_k=100, rotated_nms_iou=.5,
                                vehicle=.4, pedestrian=.1, obstacle=.1),
        maximum_interpolation_gap_sec=.15, production_changed=False, cases={},
        limitations=[
            'Not full raw-policy cadence: common footer intersection, boundary and camera-availability exclusions remain.',
            'No live driver/publication/queue, 30Hz copy scheduler, motion-policy node, Map-CV or planner replay.',
            'Deletion observation age can exceed the configured timeout across subset input gaps; it is not proof of continuous stale publication.',
            'Strict catalog-resolved actors only; unresolved/ambiguous actors are excluded from correspondence, not treated as absent.',
            'Unmatched output and unmatched GT are not visibility-qualified FP/FN/AP; nearest GT duplicate and ID switches are candidates.',
            'cut0_footer changes declared timestamp, camera selection and world pose as well as packet geometry relative to fixed51.',
            'Native footer is closest-hour single-PC experiment, not a ray-acquisition or synchronized-LAN clock oracle.',
            'World uses bounded planar Ego status and zero-rotation sensor mounting, not full RPY acquisition truth.',
            'Independent trackers use separate numeric ID counters; policy-to-policy numeric ID inequality alone is not switching.'])
    for details in inputs['cases']:
        index, source = details['case'], Path(details['source'])
        if not source.is_absolute():
            source = ROOT/source
        ego = bounded_ego(source)
        series, resolution, names = catalog_series(source, index)
        truth = dict(ego=ego, gt=series, unresolved=[], maximum_interpolation_gap_sec=.15)
        source_params = source.parent/'ros_parameters.yaml'
        if source_params.is_file():
            recorded = yaml.safe_load(source_params.read_text()).get('configure')
            if recorded and recorded != parameters:
                raise ValueError('Recorded case tracker parameters differ from the frozen production contract')
        selected = [r for r in model['rows'] if r['case'] == index]
        pairs = defaultdict(dict)
        for r in selected:
            if r['policy'] in pairs[r['pair']]:
                raise ValueError('Repeated policy/pair')
            pairs[r['pair']][r['policy']] = r
        if any(set(group) != set(POLICIES) or len({r['native_latest_stamp'] for r in group.values()}) != 1
               for group in pairs.values()):
            raise ValueError('Policies do not share the same common latest-footer anchors')
        eligible, excluded_pairs = eligible_pairs(pairs, ego)
        report['cases'][str(index)] = dict(source=str(source), source_sha256=sha(source),
            input_case=details, original_input_pair_count=len(pairs), pair_count=len(eligible),
            common_pose_excluded_pairs=excluded_pairs, catalog_resolution=resolution,
            explicit_asset_names=names, supported_actors=[dict(key=s[1][0]['key'], asset=s[1][0]['asset'],
                class_name=s[1][0]['class_name']) for s in series if s[1]], policies={})
        baseline_evidence = baseline_pairs = None
        for policy in POLICIES:
            records = sorted((r for r in selected if r['policy'] == policy and r['pair'] in eligible),
                             key=lambda r: r['stamp'])
            frames = []
            for row in records:
                pose = sample_series(ego, row['stamp'], .15)
                if pose is None:
                    raise ValueError('No bounded Ego pose for '+row['token'])
                frames.append(dict(stamp=row['stamp'], token=row['token'], pair=row['pair'],
                    native_latest_stamp=row['native_latest_stamp'],
                    world_objects=to_world(operating(row['predictions']), pose, matrix[:3, 3])))
            core = replay(binary, frames)
            confirmed = confirmed_frames(core)
            case = dict(truth, frames={TRACK: confirmed})
            primary, evidence = evaluate_gt(case, 2.)
            life, raw_life = lifecycle(core)
            event_life = switch_lifecycle_evidence(primary['switch_events'], core, raw_life, parameters)
            result = dict(core=core_summary(core), lifecycle=life, input_timeline=cadence(frames),
                declared_to_common_latest_footer_ms=stats([(r['native_latest_stamp']-r['stamp'])*1000 for r in records]),
                output_identity=output_identity_summary(confirmed), catalog_gate2m=primary,
                catalog_switch_lifecycle_evidence=event_life,
                gate_sensitivity={str(g): evaluate_gt(case, g)[0] for g in (1., 3.)},
                max_gt_interpolation_gap_sec=.15)
            row_pairs = {f['stamp']: f['pair'] for f in frames}
            if policy == 'fixed51':
                baseline_evidence, baseline_pairs = evidence, row_pairs
            paired, paired_rows = paired_correspondence(baseline_evidence, evidence, baseline_pairs, row_pairs)
            result['paired_fixed51_catalog_gate2m'] = paired
            report['cases'][str(index)]['policies'][policy] = result
            write_gzip(a.output/('case{}_{}_core.json.gz'.format(index, policy)), dict(input_frames=frames, rows=core))
            write_gzip(a.output/('case{}_{}_evidence.json.gz'.format(index, policy)),
                       dict(catalog_correspondence=evidence, lifecycle=raw_life,
                            paired_fixed51_catalog_rows=paired_rows, catalog_switch_lifecycle_evidence=event_life))
            print('case', index, policy, 'confirmed_ids', result['core']['confirmed_unique_ids'],
                  'switch_candidates', primary['switch_candidates'],
                  'long_gap_reacquisitions', primary['long_gap_different_id_reacquisitions'],
                  'multiple_near_gt', primary['same_class_multiple_uuids_near_one_clear_gt'], flush=True)
    (a.output/'packet_tracker_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
