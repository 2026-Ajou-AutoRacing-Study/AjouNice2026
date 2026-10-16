#!/usr/bin/env python3
"""CPU qualification of selected native cohorts -> Q90/upright/three-class infos.

Use one authoritative selected ledger per cohort, not parent plus completion.
All products use fresh directories. Native files are read or hardlinked only;
never edit the hardlinked products. No training, simulator, ROS or GPU imports.
Only trusted repository scripts and trusted local pickle inputs are supported.
"""
import argparse
import ast
from collections import Counter, defaultdict
from copy import deepcopy
from datetime import datetime, timedelta, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import pickle
import re
import shutil
import sys
import time
import types

import cv2
import numpy as np
from PIL import Image

from base5_capture_safety import atomic_json, sha256
from base5_upright_boxes import BOX_GEOMETRY_POLICY, component_bbox
from build_base5_bevfusion_infos import CLASSES, in_roi
from build_base5_support_pilot_infos import build as derive_infos
from report_base5_compound_evidence import (_camera_records, _validate_native_geometry,
                                          camera_state, point_memberships)
from report_base5_obstacle_support import pilot_report


REPO = Path(__file__).resolve().parents[3]
TOOLS = REPO / 'perception/detection3d/bevfusion/tools'
sys.path.insert(0, str(TOOLS))
import prepare_base5_portable_dataset as portable
import prepare_base5_pedestrian_merged as merged

SCHEMA = 'base5_training_input_qualification_v1'
PROFILE = '512x1056'
BANDS = ((0, 10), (10, 20), (20, 30), (30, 50), (50, 80), (80, 100))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def ledger(path):
    payload = Path(path).read_bytes()
    require(not payload or payload.endswith(b'\n'), 'Unterminated ledger; never repair source')
    rows = [json.loads(line) for line in payload.splitlines()]
    require(bool(rows), 'Empty selected cohort')
    require(all(r['status'] == 'accepted_candidate' for r in rows),
            'Provide authoritative approved selections only, not attempted capture ledgers')
    for key in ('id', 'capture_root', 'scenario_sha256'):
        require(len({r[key] for r in rows}) == len(rows), 'Duplicate selected ' + key)
    return rows


def snapshot(source_ledger, rows):
    paths = {Path(source_ledger).resolve()}
    parent = Path(source_ledger).parent
    for name in ('status.json', 'quota.json', 'state.json'):
        if (parent / name).is_file():
            paths.add((parent / name).resolve())
    for row in rows:
        root = Path(row['capture_root']).resolve(strict=True)
        for path in root.rglob('*'):
            require(not path.is_symlink(), 'Source symlink is not an immutable native file')
            if path.is_file():
                paths.add(path.resolve())
        if row.get('audit_path'):
            paths.add(Path(row['audit_path']).resolve(strict=True))
    return {str(p): dict(sha256=sha256(p), bytes=p.stat().st_size,
                        inode=p.stat().st_ino, mtime_ns=p.stat().st_mtime_ns)
            for p in sorted(paths)}


def pure_functions(path, names, namespace):
    """Run original CPU function bodies, without unrelated Torch/MMCV imports."""
    path = Path(path)
    tree = ast.parse(path.read_text())
    nodes = [node for node in tree.body
             if isinstance(node, ast.FunctionDef) and node.name in names]
    require({n.name for n in nodes} == set(names), 'Expected upstream pure functions changed')
    scope = dict(namespace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])),
                 str(path), 'exec'), scope)
    return {name: scope[name] for name in names}


def jpeg_helpers():
    namespace = dict(json=json, Path=Path, hashlib=hashlib, np=np, cv2=cv2,
                     Image=Image, time=time)
    cache = pure_functions(REPO / 'perception/detection3d/bevfusion/scripts/'
                           'benchmark_base5_jpeg_cache.py',
                           ('sha_bytes', 'stats', 'build_cache'), namespace)
    validation = pure_functions(REPO / 'perception/detection3d/bevfusion/plugin/'
        'morai_jpeg_cache.py', ('validate_jpeg_cache',), namespace)
    export = pure_functions(TOOLS / 'prepare_base5_jpeg_portable.py', ('build',),
        dict(deepcopy=deepcopy, json=json, os=os, Path=Path, pickle=pickle,
             shutil=shutil, safe_relative=portable.safe_relative,
             sha256=portable.sha256, validate=portable.validate, **validation))
    return cache['build_cache'], export['build']


def resize_crop(camera):
    """Execute the actual runtime's pure geometry method, without model imports."""
    path = REPO / 'docker/bevfusion/app/online_preprocessor.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'Base3OnlinePreprocessor')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == '_resize_crop')
    scope = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                 str(path), 'exec'), scope)
    runtime = types.SimpleNamespace(final_dim=(512, 1056), resize=.825,
                                    reference_width=1280.)
    return scope['_resize_crop'](runtime, camera)


def shared_compound_points(points, components, masks=None):
    """Geometric bbox overlap is not semantic per-point instance assignment."""
    masks = point_memberships(points, components) if masks is None else masks
    parents = Counter(c['parent_uid'] for c in components)
    result = {}
    for index, component in enumerate(components):
        if parents[component['parent_uid']] < 2:
            continue
        siblings = [mask for other, mask in zip(components, masks)
                    if other['parent_uid'] == component['parent_uid']
                    and other['component_id'] != component['component_id']]
        shared = masks[index] & np.logical_or.reduce(siblings)
        result[component['component_id']] = int(shared.sum())
    return result


def assert_native(root, audit, scenario_sha, details=None):
    root = Path(root)
    require(sha256(root / 'scenario.json') == scenario_sha, 'Selected scenario SHA changed')
    require(not audit.get('review_reasons'), 'Reviewed native capture cannot qualify')
    ids = [c['component_id'] for c in audit['components']]
    require(len(set(ids)) == len(ids), 'Duplicate native component ID')
    _validate_native_geometry(root, audit['components'])
    paths = list(root.glob('LIDAR_*/*_Intensity.bin'))
    require(len(paths) == 1, 'One synchronous native LiDAR file required')
    points = np.fromfile(paths[0], dtype=np.float32)
    require(points.size > 0 and points.size % 4 == 0 and np.isfinite(points).all(),
            'Invalid native XYZI')
    points = points.reshape(-1, 4)
    points = points[(points[:, :3] != 0).any(axis=1)]
    masks = point_memberships(points, audit['components'])
    shared = shared_compound_points(points, audit['components'], masks)
    roi_mask = ((points[:, :3] >= [-100., -30., -3.]) &
                (points[:, :3] < [100., 30., 2.])).all(axis=1)
    counts = {}
    for component, mask in zip(audit['components'], masks):
        require(int(mask.sum()) == component['lidar_points'],
                'Native full-RPY point support differs: ' + component['component_id'])
        counts[component['component_id']] = int((mask & roi_mask).sum())
        if details is not None:
            details[component['component_id']] = dict(
                compound_shared_bbox_points=shared.get(component['component_id'], 0),
                xy_roi_lidar_points=int((mask & ((points[:, :2] >= [-100., -30.]) &
                                                (points[:, :2] < [100., 30.])).all(axis=1)).sum()))
    return counts


def assert_upright(component, instance):
    require(instance['parent_uid'] == component['parent_uid']
            and instance['component_id'] == component['component_id']
            and instance['asset'] == component['asset'], 'Derived component identity changed')
    require(CLASSES[instance['bbox_label_3d']] == component['training_class'],
            'Derived semantic class changed')
    expected = component_bbox(component)
    require(np.allclose(instance['bbox_3d'], expected, rtol=0., atol=1e-10),
            'Derived upright box differs')
    require(instance['num_lidar_pts'] == component['lidar_points'], 'Point count changed')
    # Independent full-RPY corner containment check; do not merely call converter twice.
    from base5_capture_audit import rotation_matrix
    signs = np.array(list(itertools.product((-1., 1.), repeat=3)))
    center = np.asarray(component['center_lidar_m'])
    corners = (signs * component['size_m'] / 2) @ rotation_matrix(
        component['rotation_rpy']).T + center
    box = np.asarray(instance['bbox_3d'])
    local = (corners - box[:3]) @ rotation_matrix([0., 0., box[6]])
    require((np.abs(local) <= box[3:6] / 2 + 1e-8).all(),
            'Upright box does not enclose all native corners')


def distance_band(value):
    return next((f'{lo}_{hi}m' for lo, hi in BANDS if lo <= value < hi), 'outside_radial_100m')


def point_bucket(value):
    return '0' if value == 0 else '1_4' if value < 5 else '5_19' if value < 20 else '20plus'


def measure_selected(source_ledger, rows):
    """Reuse strict compound-aware camera measurement, including empty views.

    The older mixed pilot reporter requires an Instance.png in every camera.
    MORAI can omit empty annotations; retain that unknown/incomplete state.
    Evidence recorded in the original audit must still exist and match exactly.
    """
    frames = []
    for row in rows:
        root = Path(row['capture_root'])
        audit = read(row.get('audit_path', root / 'audit.json'))
        grouped, completeness, _ = _camera_records(
            root, {c['parent_uid'] for c in audit['components']})
        complete = all(v['complete'] for v in completeness.values())
        components = []
        for c in audit['components']:
            views = grouped.get((c['parent_uid'], c['raw_class']), [])
            measured = {(v['camera'], v['camera_instance_id'], v['native_raw_class'],
                         tuple(v['native_rect'])): v for v in views}
            saved_keys = set()
            for saved in c['cameras']:
                key = (saved['camera'], saved['camera_instance_id'], saved['annotation_class'],
                       tuple(saved['native_rect']))
                saved_keys.add(key)
                require(key in measured and measured[key]['native_instance_pixels'] ==
                        saved['native_instance_pixels'], 'Saved camera evidence is missing or changed')
            require(set(measured) == saved_keys, 'Native camera/audit observation set differs')
            state, gate, _ = camera_state(2 if c.get('camera_component_ambiguous') else 1,
                                           views, complete)
            supported = gate is True
            cameras = [dict(camera=v['camera'],
                model_mask_pixels_nearest=v['model_512x1056_pixels'],
                model_bbox_extent_px=v['model_bbox_extent_px'],
                candidate_camera_gate=v['diagnostic_camera_gate']) for v in views]
            components.append(dict(component_id=c['component_id'], camera_gate512=supported,
                cameras=cameras, camera_state=state, kept512=(c['lidar_points'] >= 1
                    or supported) and in_roi(c['center_lidar_m'])))
        frames.append(dict(id=row['id'], components=components, camera_files=completeness))
    return dict(ledger_sha256=sha256(source_ledger), captures=frames, errors=[],
                profile=PROFILE, acceptance_unchanged=True,
                missing_empty_camera_annotation_is_unknown_not_invented_support=True)


def planned_summary(metadata):
    """Planned parent/Ego distances, explicitly separate from component GT."""
    groups = defaultdict(Counter)
    frames = Counter()
    for meta in metadata:
        split = meta['split']
        frames[split] += 1
        for actor in meta['runtime_actors']:
            group = ('physical_obstacle' if actor.get('source_list') == 'objectList'
                     and str(actor.get('population_pool', '')).startswith('obstacle')
                     else actor.get('population_pool', 'unknown'))
            distance = actor.get('planned_center_distance_m')
            require(distance is not None and math.isfinite(distance), 'Missing planned parent distance')
            band = distance_band(distance)
            for key in (f'{split}/{group}/{band}', f'all/{group}/{band}'):
                groups[key]['parents'] += 1
                groups[key]['primary'] += int(actor['unique_id'] == meta['runtime_actors'][0]['unique_id'])
    return dict(scope='Planned parent observations / Ego XY center distance; not native or retained GT',
                frames=dict(frames), strata={k: dict(v) for k, v in sorted(groups.items())})


def summarize(objects, retained_only=False):
    totals = Counter()
    strata = defaultdict(Counter)
    assets = defaultdict(Counter)
    for row in objects:
        if retained_only and not row['retained']:
            continue
        cls = row['training_class']
        key = '/'.join((cls, distance_band(row['distance_lidar_xy_m']), row['direction']))
        for group in (totals, strata[key], assets[row['asset']]):
            group['components'] += 1
            group['retained'] += int(row['retained'])
            group['lidar_' + point_bucket(row['lidar_points'])] += 1
            group['camera512_supported'] += int(row['camera512_supported'] is True)
            group['camera512_unknown'] += int(row['camera512_supported'] is None)
            group['camera_component_ambiguous'] += int(row['camera_component_ambiguous'])
            group['camera_only'] += int(row['retained'] and row['lidar_points'] == 0)
    return dict(totals=dict(totals), strata={k: dict(v) for k, v in sorted(strata.items())},
                assets={k: dict(v) for k, v in sorted(assets.items())})


def input_support_diagnostics(objects):
    retained = [o for o in objects if o['retained']]
    gaps = [o for o in retained if o['lidar_points'] >= 1
            and o['model_roi_lidar_points'] == 0 and o['camera512_supported'] is not True]
    ambiguous = [o for o in retained if o['camera_component_ambiguous']]
    overlap = [o for o in retained if o.get('compound_shared_bbox_points', 0) > 0]
    return dict(
        native_positive_but_current_model_zero_and_no_resolved_camera=len(gaps),
        gap_examples=gaps,
        gap_strata=dict(Counter('/'.join((o['training_class'],
            distance_band(o['distance_lidar_xy_m']), o['split'])) for o in gaps)),
        ambiguous_retained_components=len(ambiguous),
        ambiguous_with_shared_bbox_points=sum(o.get('compound_shared_bbox_points', 0) > 0
                                             for o in ambiguous),
        retained_compound_components_with_shared_bbox_points=len(overlap),
        shared_bbox_point_examples=overlap,
        interpretation='Shared points mean geometric full-RPY bbox overlap, not confirmed semantic instance support; no GT policy changes.')


def object_rows(rows, measurement, converted, model_points=None, support_details=None):
    measured = {frame['id']: frame for frame in measurement['captures']}
    require(len(measured) == len(rows), 'Measurement frame identity differs')
    by_token = {}
    for split, document in converted.items():
        for frame in document['data_list']:
            require(frame['token'] not in by_token, 'Duplicate derived frame across splits')
            by_token[frame['token']] = (split, frame)
    require(set(by_token) == {r['id'] for r in rows}, 'Converted selected frame set differs')
    result = []
    targets = []
    for row in rows:
        root = Path(row['capture_root'])
        meta = read(root / 'metadata.json')
        audit = read(row.get('audit_path', root / 'audit.json'))
        split, frame = by_token[row['id']]
        require(split == meta['split'], 'Derived split changed')
        require(frame['scene_group_id'] == meta.get('scene_group_id',
                row.get('source_cell_id', row['id'])), 'Derived group changed')
        instances = {i['component_id']: i for i in frame['instances']}
        require(len(instances) == len(frame['instances']), 'Duplicate converted component')
        support = {c['component_id']: c for c in measured[row['id']]['components']}
        require(set(support) == {c['component_id'] for c in audit['components']},
                'Measured/native component IDs differ')
        expected = set()
        parent_counts = Counter(c['parent_uid'] for c in audit['components'])
        for c in audit['components']:
            m = support[c['component_id']]
            camera = bool(m['camera_gate512'])
            require(not c.get('camera_component_ambiguous') or not camera,
                    'Ambiguous component acquired invented camera support')
            retained = (c['lidar_points'] >= 1 or camera) and in_roi(c['center_lidar_m'])
            if retained:
                expected.add(c['component_id'])
                require(c['component_id'] in instances, 'Supported component missing in infos')
                assert_upright(c, instances[c['component_id']])
            state = m.get('camera_state', 'unknown_component_correspondence'
                if c.get('camera_component_ambiguous') else 'supported'
                if camera else 'below_gate_or_no_pixels')
            pixels = [x['model_mask_pixels_nearest'] for x in m['cameras']]
            result.append(dict(token=row['id'], source_cell_id=row.get('source_cell_id', row['id']),
                capture_root=str(root.resolve()), component_id=c['component_id'],
                parent_uid=c['parent_uid'], asset=c['asset'], data_id=c['data_id'],
                native_semantic=c['training_class'], training_class=('pedestrian'
                    if c['training_class'] == 'animal' else c['training_class']),
                split=split, family=meta['spec']['family'], primary=(
                    c['parent_uid'] == meta['runtime_actors'][0]['unique_id']),
                compound_parent=parent_counts[c['parent_uid']] > 1,
                center_lidar_m=c['center_lidar_m'], native_size_m=c['size_m'],
                native_rpy=c['rotation_rpy'], distance_lidar_xy_m=math.hypot(
                    *c['center_lidar_m'][:2]), direction='front' if c['center_lidar_m'][0] >= 0 else 'rear',
                lidar_points=c['lidar_points'], camera512_supported=camera,
                model_roi_lidar_points=(model_points or {}).get((row['id'], c['component_id'])),
                **(support_details or {}).get((row['id'], c['component_id']), {}),
                camera512_best_mask_pixels=max(pixels, default=0),
                camera_component_ambiguous=bool(c.get('camera_component_ambiguous')),
                camera_state=state, retained=retained,
                exclusion_reason=None if retained else 'outside_roi' if not in_roi(
                    c['center_lidar_m']) else 'no_resolved_current_sensor_evidence'))
        require(set(instances) == expected, 'Extra or missing converted component identity')
        plan = meta.get('spec', {}).get('secondary_far_vehicle')
        if plan:
            matching = [r for r in result if r['token'] == row['id']
                        and r['parent_uid'] == plan['target_uid']
                        and r['training_class'] == 'vehicle']
            require(len(matching) == 1 and matching[0]['retained'],
                    'Native-qualified far target did not survive actual training GT conversion')
            targets.append(dict(matching[0], planned_ego_band_m=plan['distance_bounds_m'],
                                planned_sampling_window_m=plan['sampling_bounds_m'],
                                allocated_role=plan['role']))
    return result, targets


def verify_rgb(png, jpeg, records):
    checked = 0
    geometry = {}
    for frame in records:
        for name, camera in frame['images'].items():
            path = jpeg / camera['img_path']
            require(path.suffix == '.jpg', 'Current model input is not JPEG')
            with Image.open(path) as image:
                require(image.format == 'JPEG' and image.size == (
                    camera['width'], camera['height']), 'JPEG native dimensions changed')
                rgb = np.asarray(image.convert('RGB'))
            resize, crop = resize_crop(dict(width=camera['width'], height=camera['height'],
                                           frame_id=name))
            transformed = cv2.resize(rgb, resize, interpolation=cv2.INTER_CUBIC)
            left, top, right, bottom = crop
            require(transformed[top:bottom, left:right].shape == (512, 1056, 3),
                    'Actual JPEG cannot produce current model input')
            geometry[name] = dict(native_wh=[camera['width'], camera['height']],
                                  resize_wh=list(resize), crop_xyxy=list(crop))
            checked += 1
    source = read(png / 'manifest.json')
    target = read(jpeg / 'manifest.json')
    rgb_sources = {p for p in source['files'] if p.endswith('_Intensity.png')
                   and Path(p).parts[-2].startswith('CAMERA_')}
    for relative, item in source['files'].items():
        if relative in rgb_sources or relative.startswith('infos/'):
            continue
        require(target['files'].get(relative) == item, 'Lossless auxiliary inventory changed')
        require(sha256(png / relative) == sha256(jpeg / relative),
                'Lossless auxiliary payload changed')
    return dict(images_checked=checked, actual_rgb_crop_shape=[512, 1056, 3],
                native_jpeg_quality=90, cameras=geometry,
                lossless_auxiliary_files_preserved=True,
                scope='Actual native Q90 encoding and runtime resize/crop geometry; no tensor loader/model/GPU qualification')


def qualify_cohort(name, source_ledger, output, sensor):
    source_ledger = Path(source_ledger).resolve(strict=True)
    require(not output.exists(), 'Fresh cohort output required')
    rows = ledger(source_ledger)
    before = snapshot(source_ledger, rows)
    model_points, support_details = {}, {}
    for row in rows:
        root = Path(row['capture_root'])
        meta = read(root / 'metadata.json')
        require(not meta.get('calibration_only'), 'Calibration must not fill a training cohort')
        audit = read(row.get('audit_path', root / 'audit.json'))
        details = {}
        model_points.update({(row['id'], component): count for component, count in
                             assert_native(root, audit, row['scenario_sha256'], details).items()})
        support_details.update({(row['id'], key): value for key, value in details.items()})
    output.mkdir(parents=True)
    measurement = measure_selected(source_ledger, rows)
    require(not measurement['errors'], 'Current mask measurement has unresolved input errors')
    atomic_json(output / 'mask_measurement.json', measurement)
    derived = output / 'derived'
    conversion = derive_infos(source_ledger, output / 'mask_measurement.json', derived, sensor)
    require(sum(conversion['splits'].values()) == len(rows), 'Approved cohort lost a primary/frame in infos')
    converted = {split: pickle.loads((derived / 'infos' /
        f'morai_base5_infos_{split}.pkl').read_bytes()) for split in ('train', 'val', 'test')}
    objects, targets = object_rows(rows, measurement, converted, model_points, support_details)
    png, cache, jpeg = output / 'png_portable', output / 'jpeg_cache', output / 'jpeg_portable'
    portable.build(derived / 'derived_selected_ledger.jsonl', {PROFILE: derived / 'infos'},
                   png, expected_scenes=len(rows), copy_mode='hardlink')
    native = []
    for split in ('train', 'val', 'test'):
        data = pickle.loads((png / 'infos' / PROFILE / f'morai_base5_infos_{split}.pkl').read_bytes())
        for record in data['data_list']:
            for camera in record['images'].values():
                camera['img_path'] = str(png / camera['img_path'])
        native.extend(data['data_list'])
    build_cache, export_jpeg = jpeg_helpers()
    build_cache(png, native, cache)
    export_jpeg(png, cache, jpeg, hardlink=True)
    three = output / 'training3_jpeg_portable'
    merged.build(jpeg, three, hardlink=True)
    final_records = []
    for split in ('train', 'val', 'test'):
        original = converted[split]
        actual = pickle.loads((three / 'infos' / PROFILE / f'morai_base5_infos_{split}.pkl').read_bytes())
        expected, _ = merged.merge_infos(original)
        require(len(actual['data_list']) == len(expected['data_list']), 'Three-class frames changed')
        for a, e in zip(actual['data_list'], expected['data_list']):
            require(a['instances'] == e['instances'], 'Current three-class boxes/component semantics changed')
            require(a['scene_group_id'] == e['scene_group_id'], 'Three-class group changed')
            require({k: v for k, v in a['lidar_points'].items() if k != 'lidar_path'} ==
                    {k: v for k, v in e['lidar_points'].items() if k != 'lidar_path'},
                    'Three-class LiDAR calibration changed')
            require(set(a['images']) == set(e['images']), 'Camera identity changed')
            for key in a['images']:
                require({k: v for k, v in a['images'][key].items() if k != 'img_path'} ==
                        {k: v for k, v in e['images'][key].items() if k != 'img_path'},
                        'Three-class camera calibration changed')
        final_records.extend(actual['data_list'])
    pixels = verify_rgb(png, jpeg, final_records)
    after = snapshot(source_ledger, rows)
    require(before == after, 'Source RAW/ledger/status changed during qualification')
    atomic_json(output / 'objects.json', objects)
    atomic_json(output / 'far_targets.json', targets)
    atomic_json(output / 'source_snapshot.json', before)
    metadata = [read(Path(row['capture_root']) / 'metadata.json') for row in rows]
    report = dict(schema=SCHEMA, cohort=name, source_ledger=str(source_ledger),
        source_ledger_sha256=sha256(source_ledger), frames=len(rows),
        split_frames=dict(Counter(m['split'] for m in metadata)),
        planned_parent_observations=sum(len(m['runtime_actors']) for m in metadata),
        planned_parent_summary=planned_summary(metadata),
        native_component_observations=len(objects), retained_gt=sum(o['retained'] for o in objects),
        lost_retained_component_ids=0, extra_retained_component_ids=0,
        full_rpy_points_recounted=True, upright_corner_enclosure_verified=True,
        compound_native=sum(o['compound_parent'] for o in objects),
        compound_retained=sum(o['compound_parent'] and o['retained'] for o in objects),
        ambiguous_camera_components=sum(o['camera_component_ambiguous'] for o in objects),
        ambiguous_zero_lidar=sum(o['camera_component_ambiguous'] and o['lidar_points'] == 0 for o in objects),
        far_targets=len(targets), far_camera_only=sum(t['lidar_points'] == 0 for t in targets),
        summary_native=summarize(objects), summary_retained=summarize(objects, True),
        summary_retained_model_points=summarize([
            dict(o, lidar_points=o['model_roi_lidar_points']) for o in objects], True),
        input_support_diagnostics=input_support_diagnostics(objects),
        camera_state_counts=dict(Counter(o['camera_state'] for o in objects)),
        rgb=pixels, source_unchanged=True, source_snapshot_sha256=sha256(output / 'source_snapshot.json'),
        training3_package=str(three.resolve()), training3_manifest_sha256=sha256(three / 'manifest.json'),
        box_geometry_policy=BOX_GEOMETRY_POLICY, training_label_policy=merged.LABEL_POLICY,
        limitations=['Selected cohort, not an independent generalization/accuracy test.',
            'Raw full-RPY bbox counts are not semantic per-point instance labels.',
            'Ambiguous compound camera correspondence stays unknown, not invented or merged.',
            '20+ points is an analysis bucket, never a new native/GT acceptance gate.',
            'Hardlinks preserve source inodes: derived portable payloads must remain immutable.'])
    atomic_json(output / 'report.json', report)
    return report


def baseline_report(data, support):
    data, support = Path(data).resolve(strict=True), Path(support).resolve(strict=True)
    measured = read(support)
    lookup = {(r['token'], r['component_id']): r for r in measured}
    require(len(lookup) == len(measured), 'Duplicate baseline support identity')
    objects, counts, weak_far = [], Counter(), Counter()
    source_metadata, native_metadata = {}, []
    paths = []
    for split in ('train', 'val'):
        path = data / 'infos' / PROFILE / f'morai_base5_infos_{split}.pkl'
        paths.append(path)
        info = pickle.loads(path.read_bytes())
        counts[split] = len(info['data_list'])
        for frame in info['data_list']:
            metadata_path = data / 'data' / frame['token'] / 'metadata.json'
            metadata_bytes = metadata_path.read_bytes()
            native_metadata.append(json.loads(metadata_bytes))
            source_metadata[str(metadata_path)] = hashlib.sha256(metadata_bytes).hexdigest()
            native_audit = read(data / 'data' / frame['token'] / 'audit.json')
            components = {c['component_id']: c for c in native_audit['components']}
            parent_counts = Counter(c['parent_uid'] for c in components.values())
            shared = {}
            if any(n > 1 for n in parent_counts.values()):
                raw = np.fromfile(data / frame['lidar_points']['lidar_path'], dtype=np.float32).reshape(-1, 4)
                raw = raw[(raw[:, :3] != 0).any(axis=1)]
                shared = shared_compound_points(raw, list(components.values()))
            for instance in frame['instances']:
                row = lookup.pop((frame['token'], instance['component_id']))
                native_component = components[instance['component_id']]
                assert_upright(native_component, instance)
                require(row['asset'] == instance['asset'] and
                        0 <= row['points'] <= row['native_points'] <= instance['num_lidar_pts'],
                        'Baseline support/infos identity differs')
                require(row['original_semantic'] == CLASSES[instance['bbox_label_3d']],
                        'Baseline semantic differs')
                if split == 'train' and row['cls'] != 'vehicle' and row['distance_m'] >= 50:
                    state = ('rich_proxy' if row['points'] >= 20 or row['camera_rich_proxy']
                             else 'camera_supported_sparse' if row['camera_gate']
                             else 'lidar_supported_camera_unconfirmed')
                    weak_far[state] += 1
                objects.append(dict(token=frame['token'], component_id=instance['component_id'],
                    asset=row['asset'], split=split, training_class=row['cls'],
                    parent_uid=native_component['parent_uid'],
                    compound_parent=parent_counts[native_component['parent_uid']] > 1,
                    compound_shared_bbox_points=shared.get(instance['component_id'], 0),
                    distance_lidar_xy_m=row['distance_m'], direction=row['direction'],
                    lidar_points=instance['num_lidar_pts'], xy_roi_lidar_points=row['native_points'],
                    model_roi_lidar_points=row['points'],
                    camera512_supported=row['camera_gate'],
                    camera_component_ambiguous=row['mask_matching_ambiguous'], retained=True))
    require(not lookup, 'Baseline support contains unmatched retained components')
    return dict(scope='Existing retained GT only; not all native components/planned parents',
        frames=dict(counts), retained_gt=len(objects), all=summarize(objects, True),
        all_model_points=summarize([dict(o, lidar_points=o['model_roi_lidar_points'])
                                   for o in objects], True),
        native_vs_model_point_count_differences=sum(o['lidar_points'] != o['model_roi_lidar_points']
                                                  for o in objects),
        native_vs_xy_only_point_count_differences=sum(o['lidar_points'] != o['xy_roi_lidar_points']
                                                    for o in objects),
        point_count_contract=dict(full_native='infos.num_lidar_pts / immutable raw audit',
            xy_only='support cache native_points (native_xy_support), not full-native',
            current_model='support cache points (XY and Z ROI applied)'),
        planned_parent_summary=planned_summary(native_metadata),
        source_metadata_sha256=source_metadata,
        train_far_nonvehicle_support=dict(weak_far),
        input_support_diagnostics=input_support_diagnostics(objects),
        train=summarize([r for r in objects if r['split'] == 'train'], True),
        inputs_sha256={str(p): sha256(p) for p in paths + [support]})


def run(cohorts, output, sensor, baseline=None, support=None, plan=None):
    output = Path(output).absolute()
    require(not output.exists(), 'Never overwrite previous qualification output')
    require(cohorts and all(re.fullmatch(r'[A-Za-z0-9_-]+', name) for name in cohorts),
            'Named unique cohorts required')
    output.mkdir(parents=True)
    started = time.monotonic()
    reports = {}
    seen_roots, seen_payloads, seen_cells = set(), Counter(), Counter()
    for name, source in cohorts.items():
        rows = ledger(source)
        roots = {str(Path(r['capture_root']).resolve()) for r in rows}
        require(not roots & seen_roots, 'Same raw observation selected by multiple cohorts')
        seen_roots.update(roots)
        seen_payloads.update(r['scenario_sha256'] for r in rows)
        seen_cells.update(r.get('source_cell_id', r['id']) for r in rows)
        print(json.dumps(dict(event='qualifying_cohort', cohort=name, frames=len(rows))), flush=True)
        reports[name] = qualify_cohort(name, source, output / name, Path(sensor))
    result = dict(schema=SCHEMA, created_kst=datetime.now(timezone(timedelta(hours=9))).isoformat(),
        report_script_sha256=sha256(__file__), sensor_sha256=sha256(sensor), cohorts=reports,
        unique_raw_observations=len(seen_roots), unique_payloads=len(seen_payloads),
        repeated_payload_observations=sum(v - 1 for v in seen_payloads.values()),
        unique_source_cells=len(seen_cells), elapsed_sec=time.monotonic() - started,
        baseline=baseline_report(baseline, support) if baseline and support else None,
        training_started=False, capture_started=False, detector_ap_improvement_claim=False,
        limitations=['Pilot frames are intentionally selected, not an unbiased full5K distribution.',
            'Cohorts stay separate; repeated logical source cells are not independent training examples.',
            'Baseline retained GT and new native/planned-parent denominators must not be equated.',
            'No MMDet tensor loader, training loss, model inference, GPU or weather generalization qualification.'])
    if plan:
        document = read(plan)
        metadata, pins = [], {}
        for entry in document['scenes']:
            path = Path(entry['metadata'])
            payload = path.read_bytes()
            metadata.append(json.loads(payload))
            pins[str(path.resolve())] = hashlib.sha256(payload).hexdigest()
        result['latest_plan'] = dict(manifest=str(Path(plan).resolve()), manifest_sha256=sha256(plan),
            summary=planned_summary(metadata), metadata_sha256=pins,
            limitation='5K planned parents only; not a captured 5K or expected retained GT guarantee')
    dependencies = [Path(__file__), REPO / 'docker/bevfusion/app/online_preprocessor.py',
        Path(pilot_report.__code__.co_filename), Path(derive_infos.__code__.co_filename),
        Path(component_bbox.__code__.co_filename), Path(_camera_records.__code__.co_filename),
        Path(portable.__file__), Path(merged.__file__),
        REPO / 'perception/detection3d/bevfusion/scripts/benchmark_base5_jpeg_cache.py',
        REPO / 'perception/detection3d/bevfusion/plugin/morai_jpeg_cache.py',
        TOOLS / 'prepare_base5_jpeg_portable.py']
    result['helper_sha256'] = {str(p): sha256(p) for p in dependencies}
    atomic_json(output / 'report.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cohort', action='append', required=True, help='name=/absolute/selected_ledger.jsonl')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sensor', type=Path, default=REPO / 'configs/sensors/Sensor_competition_local.json')
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--baseline-support', type=Path)
    parser.add_argument('--plan', type=Path, help='Frozen new full5K manifest; report planned parents separately')
    args = parser.parse_args()
    values = dict(item.split('=', 1) for item in args.cohort)
    require(len(values) == len(args.cohort), 'Duplicate cohort name')
    require(bool(args.baseline) == bool(args.baseline_support), 'Both baseline inputs required')
    result = run(values, args.output, args.sensor, args.baseline, args.baseline_support, args.plan)
    print(json.dumps({k: v for k, v in result.items() if k not in
                     ('cohorts', 'baseline', 'helper_sha256', 'latest_plan')}, indent=2))
