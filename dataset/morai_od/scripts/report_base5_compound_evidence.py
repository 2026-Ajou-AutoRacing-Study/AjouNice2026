#!/usr/bin/env python3
"""Read-only parent/component evidence for native Base5 compound captures.

This diagnostic does not resolve child identities, relabel GT, change capture
acceptance, or certify mesh contact. In particular, camera child ID suffixes
must not be interpreted as the synthetic LiDAR component ordinal.
"""
import argparse
import ast
import copy
from collections import Counter, defaultdict
from itertools import combinations
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from analyze_base5_crop_candidates import geometry
from audit_morai_lidar_gt_visibility import parse_instances
from base5_capture_audit import rotation_matrix
from base5_capture_safety import atomic_json, sha256
from base5_native_box_corners import load_matching_corners, derive_geometry
from report_base5_camera_crop import CAMERAS


PROFILE = dict(id='bottom512w1056', h=512, w=1056, side_start=None)
LIMITATIONS = [
    'Diagnostic only: source captures, GT, visibility gates and acceptance are unchanged.',
    'A synthetic component_id is an ordinal within parent/raw class, not a native child identity.',
    'Camera child ID suffixes are retained but never matched to LiDAR component ordinals.',
    'LiDAR counts measure returns inside full-RPY annotation boxes, not semantic instance returns.',
    'Sibling-exclusive returns are exclusive only among this parent\'s boxes; background may remain.',
    'No observed camera pixels does not prove physical invisibility or correct camera calibration.',
    'Box overlap can be legitimate in compounds; duplicate-like geometry is a review cue, not a merge rule.',
    'Annotation-bottom and saved ground checks do not independently certify mesh contact.',
]


def camera_parent(instance_id):
    """Native parent UID or the documented -(parent*100+child) encoding."""
    return instance_id if instance_id >= 0 else abs(instance_id) // 100


def _camera_records(root, wanted_uids):
    groups = defaultdict(list)
    completeness = {}
    sources = []
    for camera in CAMERAS:
        folder = root / camera
        images = sorted(folder.glob('*_Instance.png'))
        annotations = sorted(folder.glob('*_BBox2D.txt'))
        complete = len(images) == len(annotations) == 1
        completeness[camera] = dict(complete=complete, mask_files=len(images),
                                    annotation_files=len(annotations))
        if not complete:
            continue
        sources.append(dict(camera=camera, annotation_path=str(annotations[0]),
                            annotation_sha256=sha256(annotations[0]),
                            instance_path=str(images[0]), instance_sha256=sha256(images[0])))
        with Image.open(images[0]) as image:
            pixels = np.asarray(image.convert('RGBA'))
        _, width, height = CAMERAS[camera]
        if pixels.shape[:2] != (height, width):
            raise ValueError('Unexpected native camera dimensions: ' + str(folder))
        geo = geometry(camera, PROFILE)
        left, top, right, bottom = geo['crop_xyxy']
        measured = {}
        seen = set()
        for line in annotations[0].read_text().splitlines():
            if not line.strip():
                continue
            fields = line.split(maxsplit=6)
            if len(fields) != 7:
                raise ValueError('Malformed camera annotation: ' + str(annotations[0]))
            instance_id = int(fields[5])
            uid = camera_parent(instance_id)
            if uid not in wanted_uids:
                continue
            rect = tuple(map(float, fields[1:5]))
            colour = tuple(ast.literal_eval(fields[6]))
            if (len(colour) != 4 or any(not isinstance(x, int) or not 0 <= x <= 255 for x in colour)
                    or not np.isfinite(rect).all()):
                raise ValueError('Invalid camera rectangle/colour: ' + str(annotations[0]))
            key = (instance_id, fields[0], rect, colour)
            if key in seen:
                continue
            seen.add(key)
            if colour not in measured:
                mask = (pixels == np.asarray(colour)).all(axis=2).astype(np.uint8)
                resized = cv2.resize(mask, tuple(geo['resize_wh']), interpolation=cv2.INTER_NEAREST)
                measured[colour] = (int(mask.sum()), int(resized[top:bottom, left:right].sum()))
            native_count, model_count = measured[colour]
            x1, y1, x2, y2 = rect
            scale = geo['scale']
            extent = [max(0., min(right, min(width, x2)*scale)-max(left, max(0., x1)*scale)),
                      max(0., min(bottom, min(height, y2)*scale)-max(top, max(0., y1)*scale))]
            groups[(uid, fields[0])].append(dict(camera=camera, camera_instance_id=instance_id,
                native_child_suffix=abs(instance_id) % 100 if instance_id < 0 else None,
                native_raw_class=fields[0], native_rect=list(rect), instance_colour=list(colour),
                native_instance_pixels=native_count, model_512x1056_pixels=model_count,
                model_bbox_extent_px=extent, diagnostic_camera_gate=model_count >= 16 and min(extent) >= 4))
    return groups, completeness, sources


def point_memberships(points, components):
    """Full-RPY OBB membership; preserve zero-return exclusion from native audit."""
    masks = []
    for component in components:
        center = np.asarray(component['center_lidar_m'])
        size = np.asarray(component['size_m'])
        rpy = np.asarray(component['rotation_rpy'])
        if not np.isfinite(np.r_[center, size, rpy]).all() or (size <= 0).any():
            raise ValueError('Invalid component geometry')
        local = (points[:, :3] - center) @ rotation_matrix(rpy)
        masks.append((np.abs(local) <= size/2 + 1e-5).all(axis=1))
    return np.asarray(masks, dtype=bool)


def camera_state(same_class_count, observations, all_complete):
    """Do not transform unresolved component correspondence into invisibility."""
    native = any(x['native_instance_pixels'] > 0 for x in observations)
    gate = any(x['diagnostic_camera_gate'] for x in observations)
    group_state = ('supported' if gate else 'below_gate' if native else
                   'no_observed_instance_pixels' if all_complete else 'incomplete')
    if same_class_count > 1:
        return 'unknown_component_correspondence', None, group_state
    return group_state, gate if all_complete or native else None, group_state


def sensor_state(points, camera_support):
    if points:
        return 'lidar_bbox_supported_camera_unknown' if camera_support.startswith('unknown') else 'lidar_bbox_supported'
    return {
        'supported': 'camera_supported_zero_lidar',
        'below_gate': 'camera_pixels_below_gate_zero_lidar',
        'no_observed_instance_pixels': 'no_observed_sensor_evidence',
        'incomplete': 'camera_incomplete_zero_lidar',
        'unknown_component_correspondence': 'camera_correspondence_unknown_zero_lidar',
    }[camera_support]


def _validate_native_geometry(root, components, *, original_capture_root=None):
    """Validate raw geometry, optionally with an explicit byte-exact relocation.

    The old absolute source paths remain immutable evidence. A caller moving a
    portable capture must supply its recorded original root; only the root
    prefix may differ, never relative sensor paths, hashes, bytes or geometry.
    """
    root = Path(root).resolve()
    from base5_scale_capture_contract import registered_variants
    metadata_path = root/'metadata.json'
    metadata = json.loads(metadata_path.read_text()) if metadata_path.is_file() else {}
    variants = registered_variants(root,metadata)
    annotations = sorted(root.glob('LIDAR_*/*_Instance.txt'))
    if len(annotations) != 1:
        raise ValueError('Expected one native LiDAR annotation: ' + str(root))
    boxes = parse_instances(annotations[0])
    if len(boxes) != len([x for x in annotations[0].read_text().splitlines() if x.strip()]):
        raise ValueError('Malformed native LiDAR annotation')
    ordinals, lookup = Counter(), {}
    for box in boxes:
        key = (box['unique_id'], box['raw_class'])
        ident = f'{key[0]}:{key[1]}:{ordinals[key]}'
        ordinals[key] += 1
        lookup[ident] = box
    corner_records = None
    for c in components:
        native = lookup.get(c['component_id'])
        if native is None or native['unique_id'] != c['parent_uid'] or native['raw_class'] != c['raw_class']:
            raise ValueError('Audit/native component identity mismatch')
        has_corners = 'native_corner_geometry' in c
        if int(c['parent_uid']) in variants and not has_corners:
            raise ValueError('Captured nonunit actor lacks measured native eight-corner audit geometry')
        if has_corners != ('native_summary_size_m' in c):
            raise ValueError('Incomplete native corner/summary provenance markers')
        if has_corners:
            if corner_records is None:
                corner_records = load_matching_corners(annotations[0])
            ordinal = int(c['component_id'].rsplit(':', 1)[1])
            record = corner_records.get((int(c['parent_uid']), c['raw_class'], ordinal))
            if record is None:
                raise ValueError('Missing exact native eight-corner component source')
            derived = derive_geometry(native, record, allow_dimension_change=True)
            saved = c['native_corner_geometry']
            if original_capture_root is not None:
                original = Path(original_capture_root)
                if not original.is_absolute():raise ValueError('Explicit absolute original capture root required')
                saved = copy.deepcopy(saved)
                for label,pin in saved['source_files'].items():
                    old = Path(pin['path'])
                    if not old.is_absolute():raise ValueError('Absolute original native evidence source required')
                    relative = old.relative_to(original)
                    if '..' in relative.parts:raise ValueError('Unsafe native source relocation')
                    new = root/relative
                    if str(new)!=derived['evidence']['source_files'][label]['path']:
                        raise ValueError('Native source relocation changed relative sensor path')
                    pin['path']=str(new)
            if saved != derived['evidence']:
                raise ValueError('Native eight-corner source/geometry evidence differs')
            if (root/'scenario.json').is_file() and derived['evidence']['dimensions_changed'] and int(c['parent_uid']) not in variants:
                raise ValueError('Changed native corner geometry lacks bound nonunit scenario actor')
            if not np.allclose(c['native_summary_size_m'], native['size'], rtol=0., atol=1e-6):
                raise ValueError('Preserved native summary dimensions differ')
            native = dict(native, center=derived['center'], size=derived['size'],
                          rotation_rpy=derived['rotation_rpy'])
        for key, source_key in [('center_lidar_m', 'center'), ('size_m', 'size'), ('rotation_rpy', 'rotation_rpy')]:
            if not np.allclose(c[key], native[source_key], rtol=0., atol=1e-6):
                raise ValueError('Audit/native geometry mismatch: ' + c['component_id'])
    if len(lookup) != len(components):
        raise ValueError('Audit/native component count mismatch')
    for uid in variants:
        matching=[c for c in components if int(c['parent_uid'])==uid]
        if len(matching)!=1 or matching[0]['raw_class']!='Obstacle':
            raise ValueError('Captured nonunit actor requires exactly one measured native Obstacle component')
    return annotations[0]


def capture_report(root, audit_path=None):
    root = Path(root).resolve()
    audit_path = Path(audit_path) if audit_path else root / 'audit.json'
    metadata = json.loads((root / 'metadata.json').read_text())
    audit = json.loads(audit_path.read_text())
    components = audit['components']
    if len({c['component_id'] for c in components}) != len(components):
        raise ValueError('Duplicate synthetic component_id in audit')
    annotation = _validate_native_geometry(root, components)
    actors = {int(x['unique_id']): x for x in metadata['runtime_actors']}
    by_parent = defaultdict(list)
    for c in components:
        by_parent[int(c['parent_uid'])].append(c)
    compounds = {uid: cs for uid, cs in by_parent.items() if len(cs) > 1}
    cameras, completeness, camera_sources = _camera_records(root, set(compounds)) if compounds else ({}, {}, [])
    all_complete = len(completeness) == len(CAMERAS) and all(x['complete'] for x in completeness.values())
    lidar_files = sorted(root.glob('LIDAR_*/*_Intensity.bin'))
    if len(lidar_files) != 1:
        raise ValueError('Expected one native LiDAR XYZI: ' + str(root))
    raw = np.fromfile(lidar_files[0], dtype=np.float32)
    if not raw.size or raw.size % 4 or not np.isfinite(raw).all():
        raise ValueError('Incomplete or nonfinite native LiDAR')
    points = raw.reshape(-1, 4)
    zeros = int((points[:, :3] == 0).all(axis=1).sum())
    points = points[(points[:, :3] != 0).any(axis=1)]
    parents = []
    ground_rows = (audit.get('ground_support') or {}).get('actors', [])
    for uid, cs in sorted(compounds.items()):
        actor = actors.get(uid)
        if actor is None or any(c['asset'] != actor['name'] for c in cs):
            raise ValueError('Unmapped native parent')
        memberships = point_memberships(points, cs)
        hits = memberships.sum(axis=0)
        class_counts = Counter(c['raw_class'] for c in cs)
        class_rows = []
        for raw_class, count in sorted(class_counts.items()):
            observations = cameras.get((uid, raw_class), [])
            state, gate, observed = camera_state(count, observations, all_complete)
            class_rows.append(dict(raw_class=raw_class, component_count=count,
                component_ids=[c['component_id'] for c in cs if c['raw_class'] == raw_class],
                component_camera_support_state=state, component_camera_gate=gate,
                parent_class_camera_observation_state=observed, observations=observations))
        class_lookup = {x['raw_class']: x for x in class_rows}
        rows = []
        for index, c in enumerate(cs):
            count = int(memberships[index].sum())
            if count != c['lidar_points']:
                raise ValueError('Native and saved LiDAR count disagree: ' + c['component_id'])
            cam = class_lookup[c['raw_class']]
            rot = rotation_matrix(c['rotation_rpy'])
            vertical_half_extent = float(np.abs(rot[2]) @ (np.asarray(c['size_m'])/2))
            rows.append(dict(parent_uid=uid, component_id=c['component_id'], asset=c['asset'],
                native_raw_class=c['raw_class'], recorded_training_class=c['training_class'],
                center_lidar_m=c['center_lidar_m'], size_m=c['size_m'], rotation_rpy=c['rotation_rpy'],
                range_xy_m=c.get('range_ego_xy_m', c.get('range_xy_m')),
                annotation_lowest_lidar_z_m=float(c['center_lidar_m'][2]-vertical_half_extent),
                bbox_point_count=count, sibling_exclusive_bbox_points=int((memberships[index] & (hits == 1)).sum()),
                shared_sibling_bbox_points=int((memberships[index] & (hits > 1)).sum()),
                camera_support_state=cam['component_camera_support_state'],
                camera_gate512_component=cam['component_camera_gate'],
                diagnostic_sensor_state=sensor_state(count, cam['component_camera_support_state']),
                saved_camera_ambiguous=bool(c.get('camera_component_ambiguous')),
                saved_positive=bool(c['positive_lidar1_camera0']), saved_ignore_reason=c.get('ignore_reason')))
        pairs = []
        for i, j in combinations(range(len(cs)), 2):
            a, b = cs[i], cs[j]
            geometry_equal = all(np.allclose(a[k], b[k], rtol=0., atol=1e-5)
                                 for k in ('center_lidar_m', 'size_m', 'rotation_rpy'))
            shared = int((memberships[i] & memberships[j]).sum())
            pairs.append(dict(component_ids=[a['component_id'], b['component_id']],
                shared_bbox_point_count=shared,
                same_class_identical_geometry_review=bool(a['raw_class'] == b['raw_class'] and geometry_equal)))
        parents.append(dict(parent_uid=uid, asset=actor['name'], data_id=actor.get('data_id'),
            source_list=actor.get('source_list'), motion=actor.get('motion'), component_count=len(cs),
            sum_component_bbox_points=int(memberships.sum()), unique_parent_bbox_returns=int((hits > 0).sum()),
            shared_bbox_returns=int((hits > 1).sum()), components=rows, same_parent_pairs=pairs,
            camera_class_groups=class_rows,
            saved_ground_support=[x for x in ground_rows if x.get('uid', x.get('parent_uid')) == uid]))
    return dict(capture_root=str(root), scene_id=metadata.get('scene_id', root.name),
        family=metadata.get('spec', {}).get('family'), mission=metadata.get('spec', {}).get('mission'),
        saved_accepted_candidate=audit.get('accepted_candidate'), saved_review_reasons=audit.get('review_reasons'),
        saved_positive_policy=audit.get('positive_policy', 'unspecified; do not assume current 512 crop'),
        metadata_sha256=sha256(root/'metadata.json'), audit_path=str(audit_path.resolve()), audit_sha256=sha256(audit_path),
        native_annotation_sha256=sha256(annotation), lidar_sha256=sha256(lidar_files[0]),
        camera_sources=camera_sources, camera_completeness=completeness,
        zero_xyz_returns_excluded=zeros, parents=parents)


def summarize(captures):
    counts = Counter(captures=len(captures))
    assets = defaultdict(Counter)
    for capture in captures:
        for parent in capture['parents']:
            counts['compound_parents'] += 1
            ap = assets[parent['asset']]
            ap['parents'] += 1
            ap['components'] += parent['component_count']
            ap['shared_bbox_returns'] += parent['shared_bbox_returns']
            for c in parent['components']:
                counts['components'] += 1
                state = c['diagnostic_sensor_state']
                counts[state] += 1
                ap[state] += 1
                ap['saved_positive'] += c['saved_positive']
                ap['lidar20_plus'] += c['bbox_point_count'] >= 20
                ap['camera_correspondence_unknown'] += c['camera_support_state'] == 'unknown_component_correspondence'
                ap['saved_ignored_camera_unknown_zero_lidar'] += (
                    not c['saved_positive'] and state == 'camera_correspondence_unknown_zero_lidar')
            duplicate_like = sum(x['same_class_identical_geometry_review'] for x in parent['same_parent_pairs'])
            ap['same_class_identical_geometry_pairs'] += duplicate_like
            counts['same_class_identical_geometry_pairs'] += duplicate_like
    return dict(counts=dict(counts), assets={k: dict(v) for k, v in sorted(assets.items())})


def inputs_from_ledgers(paths):
    entries = []
    for path in paths:
        for line in Path(path).read_text().splitlines():
            row = json.loads(line)
            if row.get('capture_root'):
                entries.append((Path(row['capture_root']), row.get('audit_path')))
    return entries


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ledger', type=Path, action='append', default=[])
    p.add_argument('--capture-root', type=Path, action='append', default=[])
    p.add_argument('--limit-compound-captures', type=int, default=0,
                   help='0: all supplied captures; limit counts compounds, not raw ledger rows')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if not a.ledger and not a.capture_root:
        p.error('Supply --ledger or --capture-root')
    if a.limit_compound_captures < 0:
        p.error('Limit must be nonnegative')
    # This report may run beside MORAI capture. Do not silently create a large
    # OpenCV thread pool for small nearest-neighbour mask measurements.
    cv2.setNumThreads(1)
    entries = inputs_from_ledgers(a.ledger) + [(x, None) for x in a.capture_root]
    reports, seen, skipped_single = [], {}, 0
    for root, audit_path in entries:
        root = root.resolve()
        resolved_audit = (Path(audit_path) if audit_path else root/'audit.json').resolve()
        if a.output.resolve() == root or root in a.output.resolve().parents:
            raise ValueError('Do not write diagnostics inside a source capture')
        if root in seen:
            if seen[root] != resolved_audit:
                raise ValueError('One capture has conflicting audit inputs')
            continue
        seen[root] = resolved_audit
        audit = json.loads(resolved_audit.read_text())
        parent_counts = Counter(x['parent_uid'] for x in audit['components'])
        if not any(n > 1 for n in parent_counts.values()):
            skipped_single += 1
            continue
        report = capture_report(root, resolved_audit)
        reports.append(report)
        if a.limit_compound_captures and len(reports) >= a.limit_compound_captures:
            break
    result = dict(schema=1, analysis='compound parent/component native sensor evidence',
        diagnostic_only=True, labels_and_acceptance_unchanged=True, profile=PROFILE,
        ledger_sources=[dict(path=str(x.resolve()), sha256=sha256(x)) for x in a.ledger],
        skipped_single_component_captures=skipped_single,
        **summarize(reports), captures=reports, limitations=LIMITATIONS)
    atomic_json(a.output, result)
    print(json.dumps(dict(output=str(a.output.resolve()), **result['counts']), ensure_ascii=False))


if __name__ == '__main__':
    main()
