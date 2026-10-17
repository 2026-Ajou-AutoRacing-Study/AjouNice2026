#!/usr/bin/env python3
"""Read-only paired mask/native-extent diagnostic; never reclassifies raw QA.

Pairs require identical full scenario payloads after replacing only objectList
UIDs by their ordinals. Camera masks are compared directly, without cylinder,
mesh, depth or native-box containment assumptions. Repeats are not independent.
"""
import argparse
import ast
from collections import Counter
import copy
import hashlib
from itertools import combinations
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from audit_morai_lidar_gt_visibility import parse_instances
from base5_capture_safety import atomic_json, sha256
from base5_native_box_corners import derive_geometry, load_matching_corners
import base5_unit_geometry_contract as unit
from dynamic_capture_metadata import normalized_sensor_calibration
from report_base5_camera_crop import CAMERAS
from report_base5_compound_evidence import camera_parent


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def physical_payload(scene):
    """Strip only tested object UIDs; every non-identity field remains bound."""
    value = copy.deepcopy(scene)
    ids = [actor['UNIQUEID'] for actor in value['objectList']]
    if len(ids) != len(set(ids)):
        raise ValueError('Unique physical object IDs required')
    for ordinal, actor in enumerate(value['objectList']):
        actor['UNIQUEID'] = ordinal
    return digest(value)


def camera_masks(pixels, rows, uids):
    """Join BBox2D UID/class to its exact RGBA, refusing colour ambiguity."""
    colours, grouped = {}, {uid: [] for uid in uids}
    for line in rows.splitlines():
        if not line.strip():
            continue
        fields = line.split(maxsplit=6)
        if len(fields) != 7:
            raise ValueError('Malformed BBox2D record')
        uid, colour = camera_parent(int(fields[5])), tuple(ast.literal_eval(fields[6]))
        if len(colour) != 4 or any(type(v) is not int or not 0 <= v <= 255 for v in colour):
            raise ValueError('Explicit integer RGBA required')
        identity = (uid, fields[0])
        if colour in colours and colours[colour] != identity:
            raise ValueError('Camera RGBA reused by different parent/class')
        colours[colour] = identity
        if uid in grouped:
            if fields[0] != 'Obstacle':
                raise ValueError('Physical actor has unexpected camera class')
            grouped[uid].append(dict(native_instance_id=int(fields[5]), rgba=list(colour),
                bbox2d_xyxy=[float(v) for v in fields[1:5]]))
    result = {}
    for uid, records in grouped.items():
        mask = np.zeros(pixels.shape[:2], bool)
        for record in records:
            mask |= (pixels == np.asarray(record['rgba'])).all(2)
        yy, xx = np.where(mask)
        result[uid] = (mask, dict(records=records, mask_pixels=int(mask.sum()),
            bounds_xyxy=[int(xx.min()), int(yy.min()), int(xx.max())+1, int(yy.max())+1] if len(xx) else None,
            boolean_mask_sha256=hashlib.sha256(np.packbits(mask).tobytes()).hexdigest(),
            touches_image_edge=bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any()),
            occlusion_not_resolved=True))
    return result


def mask_comparison(a, b):
    if a.shape != b.shape:
        raise ValueError('Synchronous masks must use the same camera resolution')
    intersection, union = int((a & b).sum()), int((a | b).sum())
    return dict(iou=intersection/union if union else None, xor_pixels=int((a ^ b).sum()),
        identical=bool(np.array_equal(a, b)), area_ratio_b_over_a=float(b.sum()/a.sum()) if a.any() else None)


def rgb_comparison(a, b, mask):
    """Compare actual RGB in the union of independently mapped visible masks."""
    if a.shape != b.shape or a.shape[:2] != mask.shape:
        raise ValueError('Matched RGB/mask dimensions required')
    difference = np.abs(a[mask].astype(np.int16)-b[mask].astype(np.int16))
    return dict(union_mask_pixels=int(mask.sum()),
        mean_absolute_channel_difference_0_255=float(difference.mean()) if len(difference) else None,
        p95_absolute_channel_difference_0_255=float(np.percentile(difference,95)) if len(difference) else None,
        identical_rgb_pixel_fraction=float((difference == 0).all(1).mean()) if len(difference) else None,
        photometric_or_rasterization_change_not_mesh_metric=True)


def _xyz(value):
    return np.asarray([value[k] for k in 'xyz'], float)


def inspect(root, label, policy, pins, masks):
    root = Path(root).resolve(strict=True)
    read = lambda name: json.loads((root/name).read_text())
    meta, scene, audit = (read(name) for name in ('metadata.json', 'scenario.json', 'audit.json'))
    for name in ('metadata.json', 'scenario.json', 'audit.json', 'sensor_reference.json'):
        pins[str(root/name)] = sha256(root/name)
    if meta['sensor_source_sha256'] != sha256(root/'sensor_reference.json'):
        raise ValueError('Saved calibration bytes changed')
    normalized_sensor_calibration(root/'sensor_reference.json')
    annotations = list(root.glob('LIDAR_*/*_Instance.txt'))
    if len(annotations) != 1:
        raise ValueError('Exactly one native timestamp required')
    annotation = annotations[0]
    stamp = annotation.name[:-len('_Instance.txt')]
    corners = load_matching_corners(annotation)
    boxes = {box['unique_id']: box for box in parse_instances(annotation)}
    summary = {int(f[-1]): np.asarray([float(v) for v in f[7:10]])
        for line in annotation.read_text().splitlines() if (f := line.split())}
    if len(boxes) != len(corners) or len(boxes) != len(summary):
        raise ValueError('Only exact single-component parents supported')
    for path in (annotation, annotation.with_name(stamp+'_Instance_8Points.txt')):
        pins[str(path)] = sha256(path)
    actors = meta['runtime_actors']
    payload = scene['objectList']
    uids = [actor['unique_id'] for actor in actors]
    if (len(uids) != len(set(uids)) or len(payload) != len(actors)
            or [raw['UNIQUEID'] for raw in payload] != uids):
        raise ValueError('Exact ordered scenario/runtime actor identity required')
    unit.validate_actor_identities(meta, policy)
    remap = meta.get('object_uid_counterfactual')
    old_ids = uids
    if remap:
        mapping = remap['actor_uid_mapping']
        if [record['new_uid'] for record in mapping] != uids:
            raise ValueError('UID counterfactual provenance differs from actual actor identity')
        old_ids = [record['old_uid'] for record in mapping]
        if (len(set(old_ids)) != len(old_ids) or len(remap['source_actor_identity']) != len(actors)
                or [a['unique_id'] for a in remap['source_actor_identity']] != old_ids
                or [a['unique_id'] for a in remap['new_actor_identity']] != uids):
            raise ValueError('UID provenance is not a one-to-one actor mapping')
    result = dict(group=label, capture_root=str(root), timestamp=stamp,
        physical_payload_sha256=physical_payload(scene), scenario_sha256=sha256(root/'scenario.json'),
        calibration_sha256=sha256(root/'sensor_reference.json'),
        source_layout=meta['scale_stability']['source_id'], repeat_ordinal=meta['scale_stability']['repeat_ordinal'],
        ego_actual=meta['capture_runtime_ego'], accepted_candidate=audit['accepted_candidate'],
        review_reasons=audit['review_reasons'], uid_provenance_verified=True, actors=[])
    for ordinal, (actor, raw) in enumerate(zip(actors, payload)):
        uid = actor['unique_id']
        ref = policy['assets'][actor['name']]
        if (actor['data_id'] != raw['DataID'] or ref['data_id'] != raw['DataID']
                or actor['source_list'] != 'objectList' or uid not in boxes):
            raise ValueError('DataID/name/container/native parent mapping differs')
        record, box = corners[(uid, 'Obstacle', 0)], boxes[uid]
        if not np.allclose(summary[uid], box['size'], atol=1.e-6, rtol=1.e-6):
            raise ValueError('Independent Instance line and production parser disagree')
        measured = derive_geometry(box, record, allow_dimension_change=True)
        scale = [raw['scale'][k] for k in 'xyz']
        if len(set(scale)) != 1:
            raise ValueError('This comparison only supports isotropic or unit scenarios; no axis inference')
        expected = np.asarray(ref['native_size_lwh_m']) * scale[0]
        extent = np.asarray(measured['evidence']['corner_extent_local_m'])
        ratio = extent/expected
        nominal = bool(np.allclose(ratio, 1., atol=.003, rtol=.003))
        extra125 = bool(np.allclose(ratio, 1.25, atol=.003, rtol=.003))
        runtime = [row for packet in meta['capture_runtime_objects'] for row in packet['obstacle_list']
                   if row['unique_id'] == uid]
        if not runtime or any(row['name'] != actor['name'] for row in runtime):
            raise ValueError('Actual ObjectInfo UID/name identity missing or contradictory')
        latest = runtime[-1]
        result['actors'].append(dict(actor_ordinal=ordinal, parent_uid=uid, source_parent_uid=old_ids[ordinal],
            name=actor['name'], data_id=actor['data_id'], scenario_scale_xyz=scale,
            scenario_position=raw['pos'], scenario_rotation=raw['rot'], actual_actor=latest,
            native_summary_size_lwh_m=summary[uid].tolist(), native_corner_extent_lwh_m=extent.tolist(),
            native_center_lidar_m=measured['center'], native_rotation_rpy=measured['rotation_rpy'],
            expected_unit_times_requested_scale_lwh_m=expected.tolist(),
            observed_over_expected_lwh_ratio=ratio.tolist(),
            state='nominal' if nominal else 'extra125' if extra125 else 'other',
            annotation_line=record['annotation_line'], corner_line=record['corner_line'],
            corner_record_sha256=record['record_sha256'], center_residual_m=measured['evidence']['center_residual_m'],
            maximum_corner_residual_m=measured['evidence']['maximum_corner_residual_m'],
            exact_timestamp_uid_class_eight_vertex_mapping_verified=True, cameras={}))
    for name in CAMERAS:
        paths = [root/name/(stamp+suffix) for suffix in ('_Instance.png', '_BBox2D.txt', '_Intensity.png')]
        for path in paths:
            if not path.is_file():
                raise ValueError('Exact synchronous mask/BBox2D/RGB companion missing')
            pins[str(path)] = sha256(path)
        with Image.open(paths[0]) as image:
            pixels = np.asarray(image.convert('RGBA'))
        with Image.open(paths[2]) as image:
            masks[(str(root), 'rgb', name)] = np.asarray(image.convert('RGB'))
        if pixels.shape[:2] != (CAMERAS[name][2], CAMERAS[name][1]):
            raise ValueError('Actual image size differs from native camera contract')
        joined = camera_masks(pixels, paths[1].read_text(), uids)
        for actor in result['actors']:
            mask, evidence = joined[actor['parent_uid']]
            actor['cameras'][name] = evidence
            masks[(str(root), actor['actor_ordinal'], name)] = mask
    return result


def compare(a, b, masks):
    if (a['physical_payload_sha256'] != b['physical_payload_sha256']
            or a['calibration_sha256'] != b['calibration_sha256']):
        raise ValueError('Pair comparison requires identical physical scenario and calibration')
    rows = []
    for x, y in zip(a['actors'], b['actors']):
        if (x['actor_ordinal'], x['name'], x['data_id'], x['source_parent_uid']) != \
                (y['actor_ordinal'], y['name'], y['data_id'], y['source_parent_uid']):
            raise ValueError('Repeated actor identity mapping differs')
        cameras = {}
        for name in CAMERAS:
            p, q = x['cameras'][name], y['cameras'][name]
            if not p['mask_pixels'] and not q['mask_pixels']:
                continue
            ma, mb = (masks[(c['capture_root'], x['actor_ordinal'], name)] for c in (a,b))
            v = mask_comparison(ma, mb)
            v['actual_rgb_comparison'] = rgb_comparison(masks[(a['capture_root'], 'rgb', name)],
                masks[(b['capture_root'], 'rgb', name)], ma | mb)
            v.update(bounds_a=p['bounds_xyxy'], bounds_b=q['bounds_xyxy'],
                mask_pixels_a=p['mask_pixels'], mask_pixels_b=q['mask_pixels'],
                bbox2d_records_a=p['records'], bbox2d_records_b=q['records'])
            if p['bounds_xyxy'] and q['bounds_xyxy']:
                bp, bq = np.asarray(p['bounds_xyxy']), np.asarray(q['bounds_xyxy'])
                v['extent_ratio_b_over_a'] = ((bq[2:]-bq[:2])/(bp[2:]-bp[:2])).tolist()
                v['max_absolute_bound_change_px'] = int(abs(bq-bp).max())
            cameras[name] = v
        px, py = _xyz(x['actual_actor']['position']), _xyz(y['actual_actor']['position'])
        ex, ey = _xyz(a['ego_actual']), _xyz(b['ego_actual'])
        dx, dy = np.linalg.norm(px-ex), np.linalg.norm(py-ey)
        rows.append(dict(actor_ordinal=x['actor_ordinal'], name=x['name'], source_parent_uid=x['source_parent_uid'],
            uid_a=x['parent_uid'], uid_b=y['parent_uid'], state_a=x['state'], state_b=y['state'],
            native_extent_ratio_b_over_a=(np.asarray(y['native_corner_extent_lwh_m'])/x['native_corner_extent_lwh_m']).tolist(),
            actual_actor_position_delta_m=float(np.linalg.norm(py-px)), actual_ego_position_delta_m=float(np.linalg.norm(ey-ex)),
            actual_actor_heading_delta_deg=float(abs((y['actual_actor']['heading']-x['actual_actor']['heading']+180)%360-180)),
            actual_ego_heading_delta_deg=float(abs((b['ego_actual']['heading']-a['ego_actual']['heading']+180)%360-180)),
            actual_actor_pivot_distance_m=[float(dx), float(dy)],
            inverse_distance_ratio_a_over_b=float(dx/dy),
            distance_only_not_full_projection=True, cameras=cameras))
    return dict(root_a=a['capture_root'], root_b=b['capture_root'], group_a=a['group'], group_b=b['group'],
        source_layout=a['source_layout'], same_full_physical_payload_except_object_uid=True, actors=rows)


def run(groups, reference, output):
    start = time.monotonic()
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh analysis output required')
    roots = [(label, root) for label, path in groups for root in sorted((Path(path)/'captures').iterdir())
             if (root/'metadata.json').is_file()]
    if not roots or len({root.resolve() for _, root in roots}) != len(roots):
        raise ValueError('Explicit distinct native capture roots required')
    if any(root.resolve() == output or root.resolve() in output.parents for _, root in roots):
        raise ValueError('No analysis writes inside native raw')
    ref = dict(path=str(Path(reference).resolve(strict=True)), sha256=sha256(reference))
    policy = unit.load(ref)
    pins, masks = {ref['path']:ref['sha256']}, {}
    for source in policy['sources'] + [policy['catalog_reference']]:
        pins[source['path']] = source['sha256']
    captures = [inspect(root, label, policy, pins, masks) for label, root in roots]
    pairs = [compare(a,b,masks) for a,b in combinations(captures,2)
             if a['physical_payload_sha256'] == b['physical_payload_sha256']]
    summary = {}
    for label, _ in groups:
        selected = [c for c in captures if c['group'] == label]
        byasset = {}
        for asset in sorted({actor['name'] for c in selected for actor in c['actors']}):
            actors = [a for c in selected for a in c['actors'] if a['name'] == asset]
            byasset[asset] = dict(Counter(a['state'] for a in actors))
        summary[label] = dict(captures=len(selected), independent_physical_layouts=len({c['source_layout'] for c in selected}),
            byasset=byasset, accepted_candidate=sum(c['accepted_candidate'] for c in selected))
    if any(sha256(path) != checksum for path, checksum in pins.items()):
        raise RuntimeError('Immutable source changed during analysis')
    result = dict(schema='base5_paired_native_extent_and_visible_mask_diagnostic_v1', groups=summary,
        captures=captures, paired_comparisons=pairs, sources=pins, source_unchanged=True,
        implementation_sha256={str(Path(__file__).resolve()):sha256(__file__),
            **{str(Path(__file__).with_name(name).resolve()):sha256(Path(__file__).with_name(name))
               for name in ('base5_native_box_corners.py','base5_unit_geometry_contract.py',
                    'base5_capture_safety.py','audit_morai_lidar_gt_visibility.py',
                    'dynamic_capture_metadata.py','report_base5_camera_crop.py','report_base5_compound_evidence.py')}},
        elapsed_seconds=time.monotonic()-start,
        limitations=['Read-only RAM diagnostics; no raw/GT/QA status, training labels or approval changed.',
            'Repeats and cross-condition pairs share two physical layouts; not independent statistical samples.',
            'ObjectInfo position is a runtime witness, not exact exposure-time pose or a native size reference.',
            'Direct same-payload mask comparison requires no mesh proxy, but occlusion and rasterization remain unresolved.',
            'Stable silhouettes with changing native extents support annotation-bound instability, not exact mesh/collider recovery.',
            'UID, pooling, LOD, shared bounds or cache are unproven implementation hypotheses.'])
    output.mkdir(parents=True)
    atomic_json(output/'report.json', result)
    return output/'report.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-group', action='append', required=True, help='LABEL=parent containing captures/')
    parser.add_argument('--unit-reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(run([value.split('=',1) for value in args.capture_group], args.unit_reference, args.output))
