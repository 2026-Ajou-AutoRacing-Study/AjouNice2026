#!/usr/bin/env python3
"""Read-only native diagnostic report across disjoint, completed capture ledgers.

Reuse the current 512x1056 instance-mask audit. Report scene/GT support and
runtime safety separately from detector AP and literal mesh ground contact.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

from base5_capture_safety import atomic_json, sha256
from report_base5_obstacle_support import pilot_report


def completed_lines(path):
    payload = Path(path).read_bytes()
    if payload and not payload.endswith(b'\n'):
        raise ValueError('Ledger has an unfinished trailing row; retry after writer closes it')
    return [json.loads(line) for line in payload.splitlines()]


def metadata_subset(expected, observed, path='metadata'):
    """Keep every deck field, allowing only extra capture-time evidence fields."""
    if isinstance(expected, dict):
        if not isinstance(observed, dict):
            raise ValueError('Capture/deck metadata mismatch: ' + path)
        for key, value in expected.items():
            if key not in observed:
                raise ValueError('Capture/deck metadata missing field: ' + path + '.' + key)
            metadata_subset(value, observed[key], path + '.' + key)
    elif isinstance(expected, list):
        if not isinstance(observed, list) or len(expected) != len(observed):
            raise ValueError('Capture/deck metadata list mismatch: ' + path)
        for index, (value, actual) in enumerate(zip(expected, observed)):
            metadata_subset(value, actual, f'{path}[{index}]')
    elif (isinstance(expected, bool) != isinstance(observed, bool)
            or expected != observed):
        raise ValueError('Capture/deck metadata mismatch: ' + path)


def capture_provenance(root, entry, manifest_directory):
    """Reject swapped native payloads even when a ledger retains the right SHA."""
    root, manifest_directory = Path(root), Path(manifest_directory)
    source_scenario, source_metadata = (Path(entry[key]) for key in ('scenario', 'metadata'))
    if not source_scenario.is_absolute():
        source_scenario = manifest_directory / source_scenario
    if not source_metadata.is_absolute():
        source_metadata = manifest_directory / source_metadata
    expected_sha = entry['scenario_sha256']
    if sha256(source_scenario) != expected_sha:
        raise ValueError('Diagnostic deck scenario checksum mismatch: ' + entry['id'])
    if sha256(root / 'scenario.json') != expected_sha:
        raise ValueError('Native capture scenario checksum mismatch: ' + entry['id'])
    meta = json.loads((root / 'metadata.json').read_text())
    metadata_subset(json.loads(source_metadata.read_text()), meta)
    return meta, json.loads((root / 'scenario.json').read_text())


def scenario_actor_position(scene, actor):
    item = next(a for a in scene[actor['source_list']] if a['UNIQUEID'] == actor['unique_id'])
    return item['initPosition']['pos'] if 'initPosition' in item else item['pos']


def runtime_actor(objects, uid):
    return next((a for key in ('npc_list', 'pedestrian_list', 'obstacle_list')
                 for a in (objects or {}).get(key, []) if a['unique_id'] == uid), None)


def pivot_z_summary(scene, actor, telemetry):
    serialized = float(scenario_actor_position(scene, actor)['z'])
    before = runtime_actor(telemetry.get('objects_before'), actor['unique_id'])
    after = runtime_actor(telemetry.get('objects_after'), actor['unique_id'])
    a = before['position']['z'] if before else None
    b = after['position']['z'] if after else None
    return dict(generator_ground_reference_z_m=actor['position']['z'],
        serialized_scenario_pivot_z_m=serialized,
        serialized_minus_generator_reference_z_m=serialized - actor['position']['z'],
        runtime_pivot_before_z_m=a, runtime_pivot_after_z_m=b,
        runtime_minus_serialized_pivot_z_m=a - serialized if a is not None else None,
        capture_window_pivot_z_change_m=b - a if a is not None and b is not None else None)


def range_summary(component, actor):
    return dict(planned_center_distance_ego_xy_m=actor['planned_center_distance_m'],
        range_lidar_xy_m=component['range_xy_m'],
        range_ego_xy_m=component.get('range_ego_xy_m'),
        basis='Planned quota is Ego-frame center distance; native label/model ROI is LiDAR-frame center distance.')


def support_summary(rows):
    return dict(scenes=len({r['id'] for r in rows}), gt=len(rows),
        assets=len({r['asset'] for r in rows}),
        lidar_points=[r['native_lidar_points'] for r in rows],
        lidar_zero=sum(r['native_lidar_points'] == 0 for r in rows),
        mask512_gate=sum(r['camera_gate512'] for r in rows),
        camera_only=sum(r['native_lidar_points'] == 0 and r['camera_gate512'] for r in rows),
        model_mask_pixels=[r['best_model_mask_pixels'] for r in rows],
        short_side_pixels=[r['best_bbox_short_side_px'] for r in rows],
        measured_ground_verified=sum(r.get('measured_ground_verified', False) for r in rows),
        measured_ground_unverified=sum(not r.get('measured_ground_verified', False) for r in rows),
        measured_annotation_ground_gaps_m=[r['measured_annotation_ground_gap_m'] for r in rows
                                           if r.get('measured_annotation_ground_gap_m') is not None])


def report(manifest, ledgers, output):
    manifest, output = Path(manifest), Path(output)
    if output.exists():
        raise FileExistsError('A new diagnostic report output is required')
    deck = json.loads(manifest.read_text())
    expected = {e['id']: e for e in deck['scenes']}
    if len(expected) != len(deck['scenes']):
        raise ValueError('Duplicate manifest IDs')
    files = [manifest] + [Path(p) for p in ledgers]
    for entry in deck['scenes']:
        for key in ('scenario', 'metadata'):
            path = Path(entry[key])
            files.append(path if path.is_absolute() else manifest.parent / path)
    hashes = {str(p.resolve()): sha256(p) for p in files}
    captures, outcomes, observed_ids = [], Counter(), set()
    raw_digest = {}
    for ledger in ledgers:
        lines = completed_lines(ledger)
        for line in lines:
            if line['id'] not in expected:
                raise ValueError('Capture ID not in diagnostic deck: ' + line['id'])
            if line['scenario_sha256'] != expected[line['id']]['scenario_sha256']:
                raise ValueError('Capture scenario differs from diagnostic deck: ' + line['id'])
            if line['id'] in observed_ids:
                raise ValueError('Duplicate diagnostic ID across ledgers: ' + line['id'])
            observed_ids.add(line['id'])
            outcomes[line['status']] += 1
        measured = pilot_report(Path(ledger))
        if measured['errors']:
            raise ValueError('Native mask remeasurement errors: ' + str(measured['errors']))
        captures.extend(measured['captures'])
    rows = []
    for capture in captures:
        root = Path(capture['capture_root'])
        meta, scene = capture_provenance(root, expected[capture['id']], manifest.parent)
        telemetry = json.loads((root / 'telemetry.json').read_text())
        audit = json.loads((root / 'audit.json').read_text())
        for filename in ('metadata.json', 'scenario.json', 'telemetry.json', 'audit.json'):
            raw_digest[str(root / filename)] = sha256(root / filename)
        condition = meta.get('far_vehicle_diagnostic') or meta.get('near_shape_diagnostic')
        if not condition or not condition.get('not_training'):
            raise ValueError('Missing diagnostic-only capture provenance')
        for component in capture['components']:
            native = next(c for c in audit['components'] if c['component_id'] == component['component_id'])
            actor = next(a for a in meta['runtime_actors'] if a['unique_id'] == native['parent_uid'])
            cameras = component['cameras']
            best = max(cameras, key=lambda c: c['model_mask_pixels_nearest'], default=None)
            clearance = next((c for c in audit.get('actual_annotation_clearances', [])
                              if c['parent_uid'] == actor['unique_id']), {})
            ground = next((g for g in audit.get('ground_support', {}).get('actors', [])
                           if g['uid'] == actor['unique_id']), {})
            raw_heading_delta = (telemetry['ego_before']['heading'] -
                float(scene['egoVehicle']['initPosition']['rot']['yaw']) + 180) % 360 - 180
            rows.append(dict(id=capture['id'], status=capture['status'], asset=component['asset'],
                component_id=component['component_id'], training_class=component['class'],
                condition=condition, range=range_summary(native, actor),
                native_lidar_points=component['points'], kept512=component['kept512'],
                camera_gate512=component['camera_gate512'],
                best_model_mask_pixels=best['model_mask_pixels_nearest'] if best else None,
                best_bbox_short_side_px=min(best['model_bbox_extent_px']) if best else None,
                best_camera=best['camera'] if best else None,
                annotation_surface_gap_m=clearance.get('annotation_surface_gap_m'),
                measured_ground_verified=ground.get('verified', False),
                measured_annotation_ground_gap_m=ground.get('minimum_annotation_ground_gap_m'),
                ground_warning=ground.get('reason'),
                no_collision_verified=audit['capture_safety']['collision'].get('no_collision_verified'),
                ego_z_span_m=audit['capture_safety']['support'].get('z_span_m'),
                ego_heading_minus_serialized_yaw_deg=raw_heading_delta,
                pivot_z=pivot_z_summary(scene, actor, telemetry)))
    groups = defaultdict(list)
    for row in rows:
        condition = row['condition']
        for key in ('all', 'range/' + str(condition['distance_m']), 'asset/' + row['asset']):
            groups[key].append(row)
        if 'role' in condition:
            groups['direction/' + condition['role']].append(row)
        if 'lane' in condition:
            groups['lane/' + condition['lane']].append(row)
        if 'nominal_road_yaw_deg' in condition:
            groups['yaw/' + str(condition['nominal_road_yaw_deg'])].append(row)
    result = dict(source_sha256=hashes, native_sidecar_sha256=raw_digest,
        expected_scenes=len(expected), completed_scene_rows=len(observed_ids),
        missing_ids=sorted(expected.keys() - observed_ids), outcomes=dict(outcomes),
        groups={k: support_summary(v) for k, v in sorted(groups.items())}, rows=rows,
        limitations=[
            'Diagnostic factor-matched support, not detector AP, tracker bootstrap or statistical success-rate proof.',
            'Point counts are native annotation counts, not an exact remeasurement after model Z/XY filtering.',
            'Requested/planned center distances are Ego-frame; native GT/model-ROI distance is LiDAR-frame.',
            'The longitudinal LiDAR-to-Ego extrinsic changes radial distance; do not compare these bases as placement drift.',
            'Camera foreground is remeasured at the current 512x1056 input; gate is not detector confidence.',
            'Absent local ground plane stays unverified, even if Park/stability/collision checks pass.',
            'Serialized NPC pivot can intentionally include a Z offset from generator ground reference.',
            'Only runtime minus serialized pivot is a like-for-like Z comparison; neither is literal mesh contact.',
            'Missing frames/failed rows stay explicit; no automatic replacement, GT edits or old dataset changes.'])
    if hashes != {str(p.resolve()): sha256(p) for p in files}:
        raise ValueError('Ledger changed during read-only analysis; retry after collection closes')
    atomic_json(output, result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--ledger', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = report(args.manifest, args.ledger, args.output)
    print(json.dumps({k: result[k] for k in ('expected_scenes', 'completed_scene_rows', 'missing_ids', 'outcomes')},
                     ensure_ascii=False))
