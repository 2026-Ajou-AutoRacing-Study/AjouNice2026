#!/usr/bin/env python3
"""Read-only native scale probe evidence, never a naturalness/QA approval.

The scenario scale is a requested Unity-local transform. Native 3D annotation
dimensions are measurements and are NEVER multiplied by that scale here.
Use a fresh output, explicit scale_probe pairing, and preserved attempt ledgers.
"""
import argparse
from collections import Counter, defaultdict
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw

from analyze_base5_crop_candidates import geometry
from base5_asset_scale import WHITELIST
from base5_capture_audit import rotation_matrix
from base5_capture_safety import atomic_json, sha256
from base5_ground_probe import ego_to_world
from report_base5_camera_crop import CAMERAS
from report_base5_compound_evidence import (_camera_records, _validate_native_geometry,
                                          camera_state, point_memberships)
from base5_native_box_corners import load_matching_corners, derive_geometry
from audit_morai_lidar_gt_visibility import parse_instances


PROFILE = dict(id='bottom512w1056', h=512, w=1056, side_start=None)
SCHEMA = 'base5_scale_probe_native_v1'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def runtime_sources():
    """Pin the CPU helper sources actually imported by this diagnostic process."""
    directory = Path(__file__).resolve().parent
    paths = {Path(__file__).resolve()}
    for module in list(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            path = Path(name).resolve()
            if path.suffix == '.py' and path.parent == directory and path.is_file():
                paths.add(path)
    return {str(path): sha256(path) for path in sorted(paths)}


def scale_xyz(value):
    """Scenario scale axes remain x/y/z, distinct from native length/width/height."""
    values = [value[k] for k in ('x', 'y', 'z')] if isinstance(value, dict) else value
    result = np.asarray(values, dtype=float)
    require(result.shape == (3,) and np.isfinite(result).all() and (result > 0).all(),
            'Scale must have three finite positive Unity x/y/z values')
    return result.tolist()


def inventory(ledgers, rows):
    paths = {Path(p).resolve(strict=True) for p in ledgers}
    for ledger in map(Path, ledgers):
        exit_record = ledger.parent / 'shutdown_safety.json'
        if exit_record.is_file():
            paths.add(exit_record.resolve())
    for row in rows:
        if not row.get('capture_root'):
            continue
        root = Path(row['capture_root']).resolve(strict=True)
        for path in root.rglob('*'):
            require(not path.is_symlink(), 'Native evidence symlink is not immutable')
            if path.is_file():
                paths.add(path.resolve())
        if row.get('audit_path'):
            paths.add(Path(row['audit_path']).resolve(strict=True))
    return {str(path): dict(sha256=sha256(path), bytes=path.stat().st_size,
                           inode=path.stat().st_ino, mtime_ns=path.stat().st_mtime_ns)
            for path in sorted(paths)}


def resolve_probe(metadata, scenario):
    probe = metadata.get('scale_probe')
    require(isinstance(probe, dict), 'Explicit scale_probe registration required')
    require(probe.get('pair_key') and probe.get('condition') in ('control', 'scaled'),
            'Explicit pair_key and control/scaled condition required')
    uid = int(probe['parent_uid'])
    actors = [a for a in metadata['runtime_actors'] if int(a['unique_id']) == uid]
    require(len(actors) == 1, 'Probe parent UID is missing or duplicated')
    actor = actors[0]
    require(probe['asset'] == actor['name'], 'Probe asset and runtime parent differ')
    container = actor['source_list']
    values = [a for a in scenario.get(container, []) if int(a['UNIQUEID']) == uid]
    require(len(values) == 1, 'Probe scenario parent is missing or duplicated')
    payload = values[0]
    actual = scale_xyz(payload.get('scale', [1., 1., 1.]))
    requested = scale_xyz(probe['requested_scale'])
    require(np.allclose(actual, requested, atol=1e-8, rtol=0.),
            'Registered scale and actual scenario scale differ')
    if probe['condition'] == 'control':
        require(np.allclose(actual, [1., 1., 1.], atol=1e-8, rtol=0.),
                'Control must have exact unit scale')
    # Physical objectList membership alone is not semantic eligibility. A
    # vehicle-shaped OBJ or composite can also be an objectList parent.
    allowed = WHITELIST.get(actor['name'])
    excluded = (not allowed or actor.get('data_id') != allowed[0]
                or container != 'objectList' or actor.get('training_class') in ('vehicle', 'pedestrian', 'animal')
                or actor['name'].startswith(('OBJ_', 'BabyStroller', 'Ped_', 'Bike', 'Scooter')))
    return dict(probe, parent_uid=uid, actual_scenario_scale_xyz=actual,
                scale_eligibility_violation=bool(excluded and actual != [1., 1., 1.])), actor, payload


def axis_hypotheses(control_size, scaled_size, requested_scale):
    """Diagnostic permutations only; measurements do not establish mesh axis policy."""
    before, after = np.asarray(control_size, float), np.asarray(scaled_size, float)
    require(before.shape == after.shape == (3,) and np.isfinite(np.r_[before, after]).all()
            and (before > 0).all() and (after > 0).all(), 'Invalid native pair dimensions')
    ratios = after / before
    scale = np.asarray(scale_xyz(requested_scale))
    hypotheses = []
    for axes in itertools.permutations(range(3)):
        expected = scale[list(axes)]
        hypotheses.append(dict(native_lwh_from_unity_xyz=["xyz"[i] for i in axes],
            expected_dimension_ratio=expected.tolist(),
            maximum_absolute_ratio_error=float(np.max(np.abs(ratios - expected)))))
    hypotheses.sort(key=lambda h: h['maximum_absolute_ratio_error'])
    return dict(measured_native_lwh_ratio=ratios.tolist(), hypotheses=hypotheses,
        policy='Ranked hypotheses only. Annotation margins, rotation and mesh changes may prevent exact scaling; no axis mapping is auto-adopted.')


def pivot_diagnostic(component, metadata, telemetry):
    """Static observed-pivot offset in native OBB axes; never a contact certificate.

    run_base5_capture.py schema 3 serializes EgoVehicleStatus.heading unchanged
    in ego_cb. Existing base5_ground_probe.ego_to_world applies -1.347 degrees
    once. Native sensor RPY, scenario yaw and Object heading are NOT corrected.
    """
    unavailable = lambda reason, **evidence: dict(available=False, reason=reason, **evidence)
    if not telemetry or telemetry.get('schema_version') != 3:
        return unavailable('Raw collector schema-3 telemetry required')
    ego = telemetry.get('ego_before', {})
    if ego.get('heading_corrected') or telemetry.get('ego_heading_already_corrected'):
        return unavailable('Corrected Ego heading cannot receive raw-heading correction again')
    if not all(key in ego for key in ('x', 'y', 'z', 'heading')) or metadata.get('lidar_to_ego') is None:
        return unavailable('Actual Ego pose or explicit LiDAR-to-Ego calibration missing')
    require(np.isfinite([ego[k] for k in ('x', 'y', 'z', 'heading')]).all(), 'Nonfinite raw Ego telemetry')
    corrected = float(ego['heading']) - 1.347
    expected_yaw = metadata.get('ego', {}).get('yaw')
    if expected_yaw is None:
        return unavailable('Serialized Ego yaw missing; cannot cross-check raw heading contract')
    heading_error = (corrected - expected_yaw + 180.) % 360. - 180.
    if abs(heading_error) > 2.:
        return unavailable('Raw heading contract conflicts with serialized Ego yaw',
                           corrected_heading_error_deg=heading_error)
    uid = int(component['parent_uid'])
    def position(key):
        actors = [actor for name in ('npc_list', 'pedestrian_list', 'obstacle_list')
                  for actor in telemetry.get(key, {}).get(name, []) if int(actor['unique_id']) == uid]
        return actors[0]['position'] if len(actors) == 1 else None
    before, after = position('objects_before'), position('objects_after')
    if before is None or after is None:
        return unavailable('Actual static parent pivot before/after missing or ambiguous')
    pivot = np.asarray([before[k] for k in ('x', 'y', 'z')], float)
    moved = float(np.linalg.norm(pivot - [after[k] for k in ('x', 'y', 'z')]))
    if moved > .05:
        return unavailable('Parent moved; instantaneous pivot correspondence is not proven', displacement_m=moved)
    transform = np.asarray(metadata['lidar_to_ego'], float)
    require(transform.shape == (4, 4) and np.isfinite(transform).all(), 'Invalid explicit LiDAR-to-Ego calibration')
    center_ego = (transform @ np.r_[component['center_lidar_m'], 1.])[:3]
    center_world = ego_to_world(np.asarray([center_ego]), ego)[0]
    angle = math.radians(corrected)
    ego_to_world_rotation = np.asarray([[math.cos(angle), -math.sin(angle), 0.],
        [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])
    native_to_world = ego_to_world_rotation @ transform[:3, :3] @ rotation_matrix(component['rotation_rpy'])
    offset = native_to_world.T @ (center_world - pivot)
    return dict(available=True, parent_uid=uid, source='telemetry.objects_before and raw ego_before',
        telemetry_schema=3, ego_raw_heading_deg=ego['heading'], heading_correction_applied_once_deg=-1.347,
        corrected_ego_heading_deg=corrected, corrected_heading_error_deg=heading_error,
        parent_before_after_displacement_m=moved, actual_parent_pivot_world_m=pivot.tolist(),
        native_bbox_center_world_m=center_world.tolist(), pivot_to_native_bbox_center_local_m=offset.tolist(),
        offset_axes='native annotation OBB axes; not assumed Unity scale xyz axes',
        raw_heading_source=str(Path(__file__).resolve().parent / 'run_base5_capture.py') + ':ego_cb/schema_version=3',
        limitation='Stationary diagnostic with yaw-only Ego world transform. Ego roll/pitch and exact native/UDP timestamp equality are not established. Not mesh contact, ground height, or automatic production calibration.')


def _safe_name(index, ident):
    # Ledger ID is untrusted path text; prefixes also disambiguate repeated IDs.
    return f'{index:03d}_' + ''.join(c if c.isalnum() or c in '-_' else '_' for c in ident)[:100]


def _fit(image, width, height):
    ratio = min(width / image.width, height / image.height)
    return image.resize((max(1, round(image.width * ratio)), max(1, round(image.height * ratio))),
                        Image.BILINEAR), ratio


def montage(root, observations, output, title):
    """All four full RGB views plus parent crops; same-class ambiguity stays visible."""
    sheet = Image.new('RGB', (1280, 1020), '#e5e7eb')
    draw = ImageDraw.Draw(sheet)
    draw.text((8, 6), title, fill='black')
    by_camera = defaultdict(list)
    for item in observations:
        by_camera[item['camera']].append(item)
    for index, camera in enumerate(CAMERAS):
        x, y = (index % 2) * 640, 30 + (index // 2) * 480
        rgb = list((root / camera).glob('*_Intensity.png'))
        require(len(rgb) == 1, 'Expected one original RGB view: ' + camera)
        with Image.open(rgb[0]) as source:
            raw = source.convert('RGB')
        thumb, ratio = _fit(raw, 632, 350)
        sheet.paste(thumb, (x + 4, y + 20))
        draw.text((x + 4, y + 3), camera + ' / native RGB, yellow: 512 crop top', fill='black')
        geo = geometry(camera, PROFILE)
        crop_top = geo['native_window_xyxy'][1]
        draw.line((x + 4, y + 20 + crop_top * ratio,
                   x + 4 + thumb.width, y + 20 + crop_top * ratio), fill='#facc15', width=2)
        rects = []
        for item in by_camera[camera]:
            rect = item['native_rect']
            draw.rectangle((x + 4 + rect[0] * ratio, y + 20 + rect[1] * ratio,
                            x + 4 + rect[2] * ratio, y + 20 + rect[3] * ratio),
                           outline='#22c55e', width=2)
            rects.append(rect)
        if rects:
            left = max(0, math.floor(min(r[0] for r in rects)) - 15)
            top = max(0, math.floor(min(r[1] for r in rects)) - 15)
            right = min(raw.width, math.ceil(max(r[2] for r in rects)) + 15)
            bottom = min(raw.height, math.ceil(max(r[3] for r in rects)) + 15)
            if right > left and bottom > top:
                crop, _ = _fit(raw.crop((left, top, right, bottom)), 620, 88)
                sheet.paste(crop, (x + 4, y + 375))
    sheet.save(output)


def lidar_projection(points, components, output, title):
    """Observed returns in annotation-local axes, never a mesh reconstruction."""
    canvas = Image.new('RGB', (1080, 400), 'white')
    draw = ImageDraw.Draw(canvas)
    draw.text((8, 5), title + ' / native OBB-local returns, not mesh', fill='black')
    masks = point_memberships(points, components)
    colors = ['#2563eb', '#ea580c', '#16a34a', '#9333ea']
    for panel, (a, b) in enumerate(((0, 1), (0, 2), (1, 2))):
        x0, y0 = 20 + panel * 360, 35
        maximum = max(max(c['size_m'][a], c['size_m'][b]) for c in components) / 2 * 1.15
        maximum = max(maximum, .2)
        factor = 300 / (2 * maximum)
        cx, cy = x0 + 150, y0 + 165
        draw.text((x0, y0), 'XYZ'[a] + ' / ' + 'XYZ'[b] + f' (+/-{maximum:.2f}m)', fill='black')
        for index, (component, mask) in enumerate(zip(components, masks)):
            color = colors[index % len(colors)]
            size = np.asarray(component['size_m'])
            draw.rectangle((cx - size[a] / 2 * factor, cy - size[b] / 2 * factor,
                            cx + size[a] / 2 * factor, cy + size[b] / 2 * factor), outline=color, width=2)
            local = (points[mask, :3] - component['center_lidar_m']) @ rotation_matrix(component['rotation_rpy'])
            # Deterministic bounded rendering; JSON retains exact point counts.
            for value in local[::max(1, math.ceil(len(local) / 6000))]:
                x, y = cx + value[a] * factor, cy - value[b] * factor
                draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=color)
    canvas.save(output)


def capture_row(row, output, index):
    root = Path(row['capture_root']).resolve(strict=True)
    require(sha256(root / 'scenario.json') == row['scenario_sha256'], 'Captured scenario SHA differs from ledger')
    metadata, scenario = read(root / 'metadata.json'), read(root / 'scenario.json')
    audit_path = Path(row.get('audit_path', root / 'audit.json'))
    audit = read(audit_path)
    if isinstance(row.get('audit'), dict):
        require(row['audit'] == audit, 'Ledger audit differs from saved audit')
    annotation = _validate_native_geometry(root, audit['components'])
    probe, actor, payload = resolve_probe(metadata, scenario)
    components = [c for c in audit['components'] if int(c['parent_uid']) == probe['parent_uid']]
    require(bool(components) and all(c['asset'] == probe['asset'] for c in components),
            'Native parent missing or asset differs')
    if probe['actual_scenario_scale_xyz'] != [1., 1., 1.] and any(
            c['raw_class'] != 'Obstacle' or c['training_class'] != 'obstacle' for c in components):
        probe['scale_eligibility_violation'] = True
    files = list(root.glob('LIDAR_*/*_Intensity.bin'))
    require(len(files) == 1, 'Expected one native XYZI')
    raw = np.fromfile(files[0], dtype=np.float32)
    require(raw.size and raw.size % 4 == 0 and np.isfinite(raw).all(), 'Invalid native XYZI')
    points = raw.reshape(-1, 4)
    points = points[(points[:, :3] != 0).any(axis=1)]
    memberships = point_memberships(points, components)
    for component, mask in zip(components, memberships):
        require(int(mask.sum()) == component['lidar_points'], 'Native point count differs from saved audit')
    cameras, completeness, camera_sources = _camera_records(root, {probe['parent_uid']})
    all_complete = all(c['complete'] for c in completeness.values()) and len(completeness) == 4
    measured = []
    corner_path = annotation.with_name(annotation.name.replace('_Instance.txt', '_Instance_8Points.txt'))
    corner_records = load_matching_corners(annotation) if corner_path.is_file() else {}
    summary_boxes, summary_ordinals = {}, Counter()
    for box in parse_instances(annotation):
        key = (box['unique_id'], box['raw_class'])
        summary_boxes[(key[0], key[1], summary_ordinals[key])] = box
        summary_ordinals[key] += 1
    class_counts = Counter(c['raw_class'] for c in components)
    transform = metadata.get('lidar_to_ego')
    signs = np.array(list(itertools.product((-1., 1.), repeat=3)))
    roi = ((points[:, :3] >= [-100., -30., -3.]) & (points[:, :3] < [100., 30., 2.])).all(axis=1)
    for c, mask in zip(components, memberships):
        observations = cameras.get((probe['parent_uid'], c['raw_class']), [])
        for evidence in c.get('cameras', []):
            matches = [e for e in observations if e['camera'] == evidence['camera']
                       and e['camera_instance_id'] == evidence['camera_instance_id']
                       and e['native_rect'] == evidence['native_rect']]
            require(len(matches) == 1 and matches[0]['native_instance_pixels']
                    == evidence['native_instance_pixels'],
                    'Saved audit camera evidence differs from current native mask')
        state, supported, parent_state = camera_state(class_counts[c['raw_class']], observations, all_complete)
        corners = signs * c['size_m'] / 2 @ rotation_matrix(c['rotation_rpy']).T + c['center_lidar_m']
        ego_corners = ((np.c_[corners, np.ones(8)] @ np.asarray(transform).T)[:, :3]
                       if transform is not None else None)
        measured.append(dict(component_id=c['component_id'], raw_class=c['raw_class'],
            training_class=c['training_class'], native_size_lwh_m=c['size_m'],
            native_center_lidar_m=c['center_lidar_m'], native_rotation_rpy=c['rotation_rpy'],
            lidar_bbox_points=c['lidar_points'], model_xyz_roi_bbox_points=int((mask & roi).sum()),
            camera512_state=state, camera512_supported=supported, parent_camera512_state=parent_state,
            camera_observations=observations,
            native_annotation_bottom_lidar_m=float(corners[:, 2].min()),
            native_annotation_bottom_ego_m=float(ego_corners[:, 2].min()) if ego_corners is not None else None))
        key = (int(c['parent_uid']), c['raw_class'], int(c['component_id'].rsplit(':', 1)[1]))
        measured[-1]['native_summary_size_lwh_m'] = summary_boxes[key]['size'].tolist()
        if key in corner_records:
            derived = derive_geometry(summary_boxes[key], corner_records[key], allow_dimension_change=True)
            actual_box = dict(center_lidar_m=derived['center'], rotation_rpy=derived['rotation_rpy'],
                              size_m=derived['size'])
            measured[-1]['native_eight_corner_geometry'] = dict(available=True,
                center_lidar_m=derived['center'], size_lwh_m=derived['size'], rotation_rpy=derived['rotation_rpy'],
                lidar_bbox_points=int(point_memberships(points, [actual_box])[0].sum()),
                evidence=derived['evidence'], diagnostic_only=True,
                acceptance_unchanged=True, raw_annotation_unchanged=True)
        else:
            measured[-1]['native_eight_corner_geometry'] = dict(available=False,
                reason='Exact synchronous native eight-corner companion missing; never synthesize from scale')
    all_observations = [item for group in cameras.values() for item in group]
    stem = _safe_name(index, row['id'])
    rgb_path, lidar_path = output / (stem + '_cameras.png'), output / (stem + '_lidar.png')
    title = f"{row['id']} / {probe['asset']} / Unity xyz {probe['actual_scenario_scale_xyz']}"
    montage(root, all_observations, rgb_path, title)
    lidar_projection(points, components, lidar_path, title)
    shutdown_path = root / 'shutdown_safety.json'
    shutdown = read(shutdown_path) if shutdown_path.is_file() else None
    session_shutdown_path = Path(row['source_ledger']).parent / 'shutdown_safety.json'
    session_shutdown = (dict(path=str(session_shutdown_path.resolve()), sha256=sha256(session_shutdown_path),
        evidence=read(session_shutdown_path)) if session_shutdown_path.is_file() else None)
    safety = audit.get('capture_safety', metadata.get('capture_safety', {}))
    ground = [g for g in (audit.get('ground_support') or {}).get('actors', [])
              if int(g['uid']) == probe['parent_uid']]
    telemetry_path = root / 'telemetry.json'
    telemetry = read(telemetry_path) if telemetry_path.is_file() else None
    for component, measured_component in zip(components, measured):
        measured_component['pivot_diagnostic'] = pivot_diagnostic(component, metadata, telemetry)
    return dict(id=row['id'], status=row['status'], source_ledger=row['source_ledger'],
        capture_root=str(root), scenario_sha256=row['scenario_sha256'], probe=probe,
        metadata_sha256=sha256(root / 'metadata.json'), audit_sha256=sha256(audit_path),
        scenario_parent_position=payload.get('pos'), estimated_size_m=actor.get('estimated_size'),
        estimated_offset_m=actor.get('estimated_offset'), runtime_actor=actor,
        components=measured, ground_support=ground, ground_warnings=audit.get('ground_warnings', []),
        review_reasons=audit.get('review_reasons', []), capture_safety=safety,
        shutdown=shutdown, session_shutdown=session_shutdown,
        camera_completeness=completeness, camera_sources=camera_sources,
        montage=str(rgb_path.resolve()), lidar_projection=str(lidar_path.resolve()),
        human_semantic_review=dict(status='pending', automatic_approval=False,
            criteria=['road-context naturalness', 'proportion/material/support structure',
                      'mesh-ground appearance', 'LiDAR/native annotation consistency']))


def pair_reports(rows):
    grouped, result = defaultdict(list), []
    for row in rows:
        grouped[row['probe']['pair_key']].append(row)
    for key, values in sorted(grouped.items()):
        controls = [r for r in values if r['probe']['condition'] == 'control']
        scaled = [r for r in values if r['probe']['condition'] == 'scaled']
        if len(controls) != 1:
            result.append(dict(pair_key=key, comparable=False,
                reason='Exactly one explicit unit-scale control required', ids=[r['id'] for r in values]))
            continue
        control = controls[0]
        for current in scaled:
            common = ('asset', 'parent_uid', 'requested_yaw_deg')
            require(all(control['probe'].get(k) == current['probe'].get(k) for k in common),
                    'Explicit scale pair changed parent/asset/yaw')
            if len(control['components']) != 1 or len(current['components']) != 1:
                result.append(dict(pair_key=key, comparable=False, control=control['id'],
                    scaled=current['id'], reason='Multi-component axis matching intentionally unresolved'))
                continue
            a, b = control['components'][0], current['components'][0]
            require(a['raw_class'] == b['raw_class'], 'Paired native semantic class differs')
            comparison = dict(pair_key=key, comparable=True, control=control['id'], scaled=current['id'],
                asset=current['probe']['asset'], requested_yaw_deg=current['probe'].get('requested_yaw_deg'),
                measured_center_difference_lidar_m=(np.asarray(b['native_center_lidar_m'])
                    - a['native_center_lidar_m']).tolist(),
                control_native_size_lwh_m=a['native_size_lwh_m'], scaled_native_size_lwh_m=b['native_size_lwh_m'],
                comparison_control_size_lwh_m=a.get('native_eight_corner_geometry', {}).get('size_lwh_m', a['native_size_lwh_m']),
                comparison_scaled_size_lwh_m=b.get('native_eight_corner_geometry', {}).get('size_lwh_m', b['native_size_lwh_m']),
                geometry_comparison_source='measured_native_eight_corners' if all(
                    x.get('native_eight_corner_geometry', {}).get('available') for x in (a, b)) else 'saved_audit_geometry',
                **axis_hypotheses(a.get('native_eight_corner_geometry', {}).get('size_lwh_m', a['native_size_lwh_m']),
                                  b.get('native_eight_corner_geometry', {}).get('size_lwh_m', b['native_size_lwh_m']),
                                  current['probe']['actual_scenario_scale_xyz']))
            pivots = [c.get('pivot_diagnostic', {}) for c in (a, b)]
            if all(p.get('available') for p in pivots):
                comparison['measured_pivot_local_offset_difference_m'] = (np.asarray(
                    pivots[1]['pivot_to_native_bbox_center_local_m']) -
                    pivots[0]['pivot_to_native_bbox_center_local_m']).tolist()
            result.append(comparison)
    return result


def asset_comparison_sheets(rows, output):
    """Same pixel window across conditions, not independently magnified crops."""
    groups, result = defaultdict(list), {}
    for row in rows:
        groups[row['probe']['asset']].append(row)
    for asset, values in sorted(groups.items()):
        values.sort(key=lambda r: (r['probe']['requested_yaw_deg'],
            r['probe']['condition'] != 'control', max(r['probe']['actual_scenario_scale_xyz']),
            r['probe'].get('profile', ''), r['id']))
        counts, records = Counter(), defaultdict(list)
        for row in values:
            for component in row['components']:
                for observation in component['camera_observations']:
                    counts[observation['camera']] += observation['native_instance_pixels']
                    records[observation['camera']].append(observation)
        if not counts:
            continue
        camera = max(counts, key=counts.get)
        _, width, height = CAMERAS[camera]
        observations = records[camera]
        window = [max(0, math.floor(min(o['native_rect'][0] for o in observations)) - 40),
            max(0, math.floor(min(o['native_rect'][1] for o in observations)) - 40),
            min(width, math.ceil(max(o['native_rect'][2] for o in observations)) + 40),
            min(height, math.ceil(max(o['native_rect'][3] for o in observations)) + 40)]
        canvas = Image.new('RGB', (1440, math.ceil(len(values) / 3) * 350 + 30), '#e5e7eb')
        draw = ImageDraw.Draw(canvas)
        draw.text((8, 6), asset + ' / same native pixel window / visual decision pending', fill='black')
        for index, row in enumerate(values):
            x, y = index % 3 * 480, index // 3 * 350 + 30
            path = next((Path(row['capture_root']) / camera).glob('*_Intensity.png'))
            with Image.open(path) as raw:
                crop = raw.convert('RGB').crop(window)
            thumb, _ = _fit(crop, 470, 285)
            canvas.paste(thumb, (x + 5, y + 50))
            draw.text((x + 5, y + 3), str(row['probe']['actual_scenario_scale_xyz']) +
                ' yaw=' + str(row['probe']['requested_yaw_deg']), fill='black')
            count = sum(c.get('native_eight_corner_geometry', {}).get('lidar_bbox_points', c['lidar_bbox_points'])
                        for c in row['components'])
            draw.text((x + 5, y + 19), row['status'] + ' / native box points=' + str(count), fill='black')
            draw.text((x + 5, y + 35), row['probe'].get('profile', ''), fill='black')
        filename = 'asset_' + ''.join(c if c.isalnum() or c in '-_' else '_' for c in asset) + '.png'
        path = output / filename
        canvas.save(path)
        result[asset] = dict(path=str(path.resolve()), camera=camera, fixed_native_window_xyxy=window,
            ids=[r['id'] for r in values], human_visual_review='pending', automatic_approval=False,
            limitation='Same pixel crop, not metric rectification or a detector accuracy test.')
    return result


def report(ledgers, output):
    output = Path(output)
    require(not output.exists(), 'Fresh diagnostic output required; never rewrite an earlier decision')
    rows = []
    for path in map(Path, ledgers):
        data = path.read_bytes()
        require(data and data.endswith(b'\n'), 'Empty or unterminated source ledger')
        rows.extend(dict(json.loads(line), source_ledger=str(path.resolve())) for line in data.splitlines())
    identities = [(r['id'], r.get('capture_root')) for r in rows]
    require(len(set(identities)) == len(rows), 'Repeated identical capture attempt supplied twice')
    for row in rows:
        if row.get('capture_root'):
            native = Path(row['capture_root']).resolve()
            require(output.resolve() != native and native not in output.resolve().parents,
                    'Diagnostic output cannot be inside immutable native evidence')
    before = inventory(ledgers, rows)
    helpers_before = runtime_sources()
    output.mkdir(parents=True)
    captured, failures = [], []
    for index, row in enumerate(rows):
        if not row.get('capture_root') or not (Path(row['capture_root']) / 'audit.json').is_file():
            failures.append(row)
            continue
        captured.append(capture_row(row, output, index))
    comparisons = pair_reports(captured)
    sheets = asset_comparison_sheets(captured, output)
    after = inventory(ledgers, rows)
    require(before == after, 'Native evidence changed during read-only diagnosis')
    require(helpers_before == runtime_sources(), 'Diagnostic runtime source changed during diagnosis')
    result = dict(schema=SCHEMA, ledgers=[dict(path=str(Path(p).resolve()), sha256=sha256(Path(p))) for p in ledgers],
        captures=captured, comparisons=comparisons, asset_comparison_sheets=sheets,
        failed_without_complete_native_audit=failures,
        summary=dict(attempts=len(rows), captured=len(captured), outcomes=dict(Counter(r['status'] for r in rows)),
            assets=len({r['probe']['asset'] for r in captured}),
            eligibility_violations=sum(r['probe']['scale_eligibility_violation'] for r in captured),
            comparable_scale_pairs=sum(r['comparable'] for r in comparisons)),
        native_unchanged=True, source_inventory=before, runtime_sources=helpers_before,
        script_sha256=sha256(Path(__file__)),
        limitations=['No new capture, approval, GT rewrite, training or detector AP measurement.',
            'Scenario Unity scale is requested metadata; native GT dimensions are measured once, never rescaled.',
            'BBox point membership is geometric, not per-point semantic object identity.',
            'Annotation bottom, saved ground warnings and Park do not certify mesh-ground contact.',
            'Center deltas are LiDAR-frame differences, not a standalone pivot-calibration measurement.',
            'Naturalness requires inspecting raw RGB context and crops; this report leaves every human decision pending.'])
    atomic_json(output / 'report.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ledger', type=Path, action='append', required=True,
                        help='Authoritative attempt or selection ledger; repeat explicitly for later probes')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    value = report(args.ledger, args.output)
    print(json.dumps(value['summary'], ensure_ascii=False, indent=2))
