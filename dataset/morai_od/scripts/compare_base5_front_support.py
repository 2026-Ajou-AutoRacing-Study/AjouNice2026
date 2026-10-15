#!/usr/bin/env python3
"""Read-only comparison of v150/v151 diagnostic mixed-scene captures.

Same slot/asset/count is not necessarily the same physical pose. Report added
actor support separately from the original population. This report neither
fills collection quota nor retroactively certifies an interrupted shutdown.
"""
import argparse
from collections import Counter
import copy
import json
import math
from pathlib import Path
import statistics

from base5_capture_safety import atomic_json, sha256
from report_base5_diagnostic_deck import completed_lines
from report_base5_obstacle_support import pilot_report, sector


PLAN_KEY = 'secondary_obstacle_front_support'


def preserve_contract(slot, before_row, after_row, before, after):
    """Verify identities/frozen quotas, without claiming pose preservation."""
    plan = after['spec'].get(PLAN_KEY)
    pair = after.get('obstacle_front_support_pair', {})
    if not plan or pair.get('source_id') != slot:
        raise ValueError('Missing source slot/front support provenance: ' + slot)
    if pair.get('source_scenario_sha256') != before_row['scenario_sha256']:
        raise ValueError('Source scenario checksum differs: ' + slot)
    expected = copy.deepcopy(before['spec'])
    expected[PLAN_KEY] = plan
    if after['spec'] != expected:
        raise ValueError('Original frozen spec changed: ' + slot)
    keys = ('unique_id', 'data_id', 'name', 'source_list', 'motion')
    identities = lambda m: [[a.get(k) for k in keys] for a in m['runtime_actors']]
    if identities(before) != identities(after):
        raise ValueError('Actor identities/count/motion changed: ' + slot)
    if before['split'] != after['split']:
        raise ValueError('Split changed: ' + slot)
    ordinal = plan['ordinal']
    actors = after['runtime_actors']
    if (ordinal != len(actors) - 1 or ordinal <= 0
            or before['spec']['secondary_obstacle_support']['new_actor_ordinals'] != [ordinal]
            or plan['data_id'] != actors[ordinal]['data_id']):
        raise ValueError('Added actor ordinal/identity differs: ' + slot)
    added_uid = actors[ordinal]['unique_id']
    delta_xy = math.hypot(after['ego']['x'] - before['ego']['x'],
                         after['ego']['y'] - before['ego']['y'])
    delta_yaw = (after['ego']['yaw'] - before['ego']['yaw'] + 180) % 360 - 180
    return dict(original_spec_preserved=True, actor_ids_count_motion_preserved=True,
                split_preserved=True, primary_data_id=actors[0]['data_id'],
                actor_count=len(actors), added_uid=added_uid, added_asset=actors[ordinal]['name'],
                selected=plan['selected'], selected_bounds_ego_m=plan['distance_bounds_m'],
                source_scenario_sha256=pair['source_scenario_sha256'],
                source_metadata_sha256_recorded=pair.get('source_metadata_sha256'),
                candidate_scenario_sha256=after_row['scenario_sha256'],
                ego_xy_change_m=delta_xy, ego_yaw_change_deg=delta_yaw,
                ego_pose_equal=before['ego'] == after['ego'],
                original_actor_positions_equal=all(a['position'] == b['position']
                    for a, b in zip(before['runtime_actors'][:ordinal], actors[:ordinal])))


def component_rows(capture, native_audit):
    natives = {c['component_id']: c for c in native_audit['components']}
    rows = []
    for c in capture['components']:
        n = natives[c['component_id']]
        center = n.get('center_ego_m')
        if center is None:
            raise ValueError('Ego-frame annotation center unavailable')
        best = max(c['cameras'], key=lambda x: x['model_mask_pixels_nearest'], default=None)
        rows.append(dict(component_id=c['component_id'], parent_uid=n['parent_uid'],
            asset=c['asset'], training_class=c['class'], lidar_points=c['points'],
            camera_gate512=c['camera_gate512'], kept512=c['kept512'],
            camera_component_ambiguous=n.get('camera_component_ambiguous', False),
            best_model_mask_pixels=best['model_mask_pixels_nearest'] if best else None,
            best_model_bbox_short_side_px=min(best['model_bbox_extent_px']) if best else None,
            center_ego_m=center, range_ego_xy_m=math.hypot(*center[:2]),
            range_lidar_xy_m=c['range'], actual_ego_sector=sector(center)))
    return rows


def support_summary(rows):
    points = [r['lidar_points'] for r in rows]
    return dict(components=len(rows), actors=len({r['parent_uid'] for r in rows}),
        kept512=sum(r['kept512'] for r in rows),
        camera_gate512=sum(r['camera_gate512'] for r in rows),
        lidar_zero=sum(p == 0 for p in points), lidar_1_4=sum(1 <= p < 5 for p in points),
        lidar_5_19=sum(5 <= p < 20 for p in points), lidar_20_plus=sum(p >= 20 for p in points),
        point_mean=statistics.mean(points) if points else None,
        point_median=statistics.median(points) if points else None,
        front=sum(r['actual_ego_sector'] == 'front' for r in rows),
        front_under10m=sum(r['actual_ego_sector'] == 'front' and r['range_ego_xy_m'] < 10
                          for r in rows),
        actual_ego_sectors=dict(Counter(r['actual_ego_sector'] for r in rows)))


def session_safety(ledger):
    path = Path(ledger).parent / 'shutdown_safety.json'
    return dict(ledger=str(Path(ledger).resolve()), shutdown_record_exists=path.exists(),
                shutdown_record_sha256=sha256(path) if path.exists() else None,
                shutdown_record=json.loads(path.read_text()) if path.exists() else None,
                original_shutdown_park_verified=(json.loads(path.read_text()).get('park_verified') is True)
                    if path.exists() else None)


def load_side(ledgers):
    result, sessions = {}, []
    for ledger in map(Path, ledgers):
        lines = completed_lines(ledger)
        seen = set(result)
        for row in lines:
            if row['id'] in seen:
                raise ValueError('Duplicate slot across same-side ledgers: ' + row['id'])
            seen.add(row['id'])
        measured = pilot_report(ledger)
        if measured['errors']:
            raise ValueError('Native mask remeasurement failed: ' + str(measured['errors']))
        captures = {c['id']: c for c in measured['captures']}
        for row in lines:
            slot = row['id']
            if slot in result:
                raise ValueError('Duplicate slot across same-side ledgers: ' + slot)
            if slot not in captures:
                raise ValueError('Closed ledger row has no captured raw: ' + slot)
            root = Path(row['capture_root'])
            metadata = json.loads((root / 'metadata.json').read_text())
            audit = json.loads((root / 'audit.json').read_text())
            if sha256(root / 'scenario.json') != row['scenario_sha256']:
                raise ValueError('Captured scenario checksum differs: ' + slot)
            result[slot] = dict(row=row, capture=captures[slot], metadata=metadata,
                components=component_rows(captures[slot], audit),
                sidecar_sha256={name: sha256(root / name) for name in
                                ('metadata.json', 'scenario.json', 'audit.json')})
        sessions.append(session_safety(ledger))
    return result, sessions


def compare(before_ledgers, after_ledgers, output, recovery=None):
    output = Path(output)
    if output.exists():
        raise FileExistsError('Use a new front-support comparison output')
    sources = [Path(p) for p in before_ledgers + after_ledgers]
    if recovery is not None:
        sources.append(Path(recovery))
    frozen = {str(p.resolve()): sha256(p) for p in sources}
    before, before_sessions = load_side(before_ledgers)
    after, after_sessions = load_side(after_ledgers)
    if set(before) != set(after):
        raise ValueError('Before/after slots differ; do not silently select successful overlaps')
    pairs, all_before, all_after, old_before, old_after = [], [], [], [], []
    for slot in sorted(before):
        b, a = before[slot], after[slot]
        contract = preserve_contract(slot, b['row'], a['row'], b['metadata'], a['metadata'])
        uid = contract['added_uid']
        components_before, components_after = b['components'], a['components']
        added_b = [r for r in components_before if r['parent_uid'] == uid]
        added_a = [r for r in components_after if r['parent_uid'] == uid]
        originals_b = [r for r in components_before if r['parent_uid'] != uid]
        originals_a = [r for r in components_after if r['parent_uid'] != uid]
        # Retain native missing components as absence, not synthetic zero-point labels.
        matched_ids = sorted({r['component_id'] for r in originals_b} &
                             {r['component_id'] for r in originals_a})
        pairs.append(dict(id=slot, contract=contract,
            before_status=b['row']['status'], after_status=a['row']['status'],
            before_review_reasons=b['capture']['review_reasons'],
            after_review_reasons=a['capture']['review_reasons'],
            added_before=added_b, added_after=added_a,
            original_before=support_summary(originals_b), original_after=support_summary(originals_a),
            original_component_ids_only_before=sorted({r['component_id'] for r in originals_b} - set(matched_ids)),
            original_component_ids_only_after=sorted({r['component_id'] for r in originals_a} - set(matched_ids)),
            original_component_changes=[dict(component_id=cid,
                before=next(r for r in originals_b if r['component_id'] == cid),
                after=next(r for r in originals_a if r['component_id'] == cid)) for cid in matched_ids],
            sidecar_sha256=dict(before=b['sidecar_sha256'], after=a['sidecar_sha256'])))
        all_before.extend(dict(r, scene_id=slot) for r in added_b)
        all_after.extend(dict(r, scene_id=slot) for r in added_a)
        old_before.extend(dict(r, scene_id=slot) for r in originals_b)
        old_after.extend(dict(r, scene_id=slot) for r in originals_a)
    # Across scenes, UID is reused; actor exposure is (scene, UID), not UID alone.
    def exposures(rows):
        result = support_summary(rows)
        result['actors'] = len({(r['scene_id'], r['parent_uid']) for r in rows})
        result['scenes'] = len({r['scene_id'] for r in rows})
        return result
    result = dict(diagnostic_only=True, fills_training_quota=False,
        source_sha256=frozen, paired_scene_count=len(pairs), pairs=pairs,
        added_actor_support=dict(before=exposures(all_before), after=exposures(all_after)),
        original_population_support=dict(before=exposures(old_before), after=exposures(old_after)),
        capture_outcomes=dict(before=dict(Counter(v['row']['status'] for v in before.values())),
                              after=dict(Counter(v['row']['status'] for v in after.values()))),
        sessions=dict(before=before_sessions, after=after_sessions),
        separate_post_interruption_recovery=(dict(path=str(Path(recovery).resolve()),
            sha256=sha256(recovery), evidence=json.loads(Path(recovery).read_text()),
            does_not_certify_original_shutdown=True) if recovery is not None else None),
        limitations=[
            'Same slot/assets/count/spec are preserved; regenerated Ego/actor poses may differ. Not fixed-pose causal A/B.',
            'A missing shutdown_safety.json remains unknown; later Park recovery never certifies the interrupted shutdown.',
            'Actual component Ego-frame range is separate from planned actor range and LiDAR-frame model ROI.',
            'Current 512x1056 mask support is label evidence, not learned detector confidence or AP.',
            '20-point richness is diagnostic only, never a new acceptance gate or retry target.',
            'Native component omissions remain explicit; unknown compound camera correspondence is not invisibility.',
            'Small diagnostic captures do not establish full-deck success rate or training improvement.'])
    if frozen != {str(p.resolve()): sha256(p) for p in sources}:
        raise ValueError('Source ledger/recovery changed during analysis')
    atomic_json(output, result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before-ledger', type=Path, nargs='+', required=True)
    parser.add_argument('--after-ledger', type=Path, nargs='+', required=True)
    parser.add_argument('--post-interruption-recovery', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.before_ledger, args.after_ledger, args.output,
                     args.post_interruption_recovery)
    print(json.dumps({k: result[k] for k in ('paired_scene_count', 'capture_outcomes',
                     'added_actor_support', 'original_population_support')}, ensure_ascii=False))
