#!/usr/bin/env python3
"""Read-only continuity/overlap audit with catalog-corrected MORAI GT.

This is not a detector AP benchmark: a simulator actor being in the ROI does
not establish sensor visibility. Associations below are offline evaluation
correspondences, not the production EKF's association algorithm.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
from datetime import datetime
from dataclasses import replace
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'base_ws/src/tracking_prediction_evaluation/src'),
               str(ROOT/'base_ws/src/perception/bevfusion_ros/src'),
               str(ROOT/'perception/detection3d/bevfusion/tools')]
from tracking_prediction_evaluation.trajectory import (
    read_morai_trajectories, transform_segments_planar)
from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
from bevfusion_ros.rotated_bev_nms import rotated_bev_iou
from review_tracker_bag_inventory import inspect

TRACK = '/perception/object_recognition/tracking/objects'
DET = '/perception/object_recognition/detection/objects'
PRED = '/perception/object_recognition/objects'
CLASSES = ('vehicle', 'pedestrian', 'obstacle')
BANDS = (0., 10., 20., 30., 50., 80., 100., float('inf'))


def stats(values):
    if not values:
        return dict(n=0, mean=None, p50=None, p90=None, p95=None, p99=None, maximum=None)
    a = np.asarray(values, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite audit statistic')
    return dict(n=len(a), mean=float(a.mean()), p50=float(np.median(a)),
                p90=float(np.percentile(a, 90)), p95=float(np.percentile(a, 95)),
                p99=float(np.percentile(a, 99)), maximum=float(a.max()))


def label_name(label):
    return 'vehicle' if 1 <= label <= 6 else 'pedestrian' if label == 7 else 'obstacle' if label == 0 else 'other'


def band(distance):
    for low, high in zip(BANDS, BANDS[1:]):
        if low <= distance < high:
            return '{}_{}m'.format(int(low), int(high) if math.isfinite(high) else 'plus')
    raise ValueError('Negative distance')


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def match(gt, output, gate, class_preserving=True):
    """Maximize valid pair cardinality before minimizing XY distance."""
    if not gt or not output:
        return [], list(range(len(gt))), list(range(len(output)))
    distances = np.linalg.norm(np.asarray([g['center'][:2] for g in gt])[:, None] -
                               np.asarray([o['center'][:2] for o in output])[None], axis=-1)
    valid = distances <= gate
    if class_preserving:
        valid &= np.asarray([[g['class_name'] == o['class_name'] for o in output] for g in gt])
    rows, cols = linear_sum_assignment(np.where(valid, distances, 1.e6))
    pairs = [(int(i), int(j), float(distances[i, j])) for i, j in zip(rows, cols) if valid[i, j]]
    return pairs, [i for i in range(len(gt)) if i not in {p[0] for p in pairs}], \
        [j for j in range(len(output)) if j not in {p[1] for p in pairs}]


def sample_series(series, stamp, maximum_gap=.2):
    times, points = series
    i = bisect_left(times, stamp)
    if i < len(times) and abs(times[i]-stamp) < 1.e-8:
        return points[i]
    if i == 0 or i == len(times) or times[i]-times[i-1] > maximum_gap:
        return None
    alpha = (stamp-times[i-1])/(times[i]-times[i-1])
    a, b = points[i-1], points[i]
    result = dict(a)
    result['center'] = [(1-alpha)*x+alpha*y for x, y in zip(a['center'], b['center'])]
    delta = math.atan2(math.sin(b['yaw']-a['yaw']), math.cos(b['yaw']-a['yaw']))
    result['yaw'] = a['yaw']+alpha*delta
    return result


def in_roi(obj, ego):
    dx, dy = obj['center'][0]-ego['center'][0], obj['center'][1]-ego['center'][1]
    c, s = math.cos(ego['yaw']), math.sin(ego['yaw'])
    return abs(c*dx+s*dy) <= 100. and abs(-s*dx+c*dy) <= 30.


def identity_events(observations, maximum_gap=.5):
    """Separate rapid UUID substitutions from long-gap reacquisition.

    Rows are GT matched. The same UUID after a visibility/match gap is a
    reacquisition/fragmentation, not a new identity. A changed UUID within the
    bounded interval is a switch candidate; GT reconstruction is not oracle.
    """
    last = {}
    switches, reacquisitions = [], []
    for row in sorted(observations, key=lambda r: r['stamp']):
        old = last.get(row['gt_key'])
        if old:
            dt = row['stamp']-old['stamp']
            if dt < 0:
                raise ValueError('Out-of-order identity observations')
            if row['id'] != old['id']:
                event = dict(row, previous_id=old['id'], gap_sec=dt,
                             previous_center_error_m=old['center_error_m'])
                (switches if dt <= maximum_gap else reacquisitions).append(event)
        last[row['gt_key']] = row
    return switches, reacquisitions


def output_identity_summary(frames):
    by_id = defaultdict(list)
    repeated_ids = []
    class_changes = []
    for frame in frames:
        seen = Counter(o['id'] for o in frame['objects'] if 'id' in o)
        if any(n > 1 for n in seen.values()):
            repeated_ids.append(dict(stamp=frame['stamp'], ids={k: n for k, n in seen.items() if n > 1}))
        for o in frame['objects']:
            if 'id' not in o:
                continue
            history = by_id[o['id']]
            if history and history[-1][1]['class_name'] != o['class_name']:
                class_changes.append(dict(stamp=frame['stamp'], id=o['id'],
                                          previous_class=history[-1][1]['class_name'], class_name=o['class_name']))
            history.append((frame['stamp'], o))
    actors = []
    for identity, rows in by_id.items():
        actors.append(dict(id=identity, classes=sorted({o['class_name'] for _, o in rows}),
                           observations=len(rows), output_span_sec=rows[-1][0]-rows[0][0],
                           gaps_sec=stats([b[0]-a[0] for a, b in zip(rows, rows[1:])]),
                           score=stats([o['score'] for _, o in rows])))
    return dict(same_uuid_twice_in_one_frame=repeated_ids, uuid_class_changes=class_changes,
                ids=actors, output_spans_sec=stats([r['output_span_sec'] for r in actors]),
                note='Output lifespan is not age since the last detector association; coasting cannot be certified from it.')


def point_segment_distance(point, start, end):
    v = np.asarray(end[:2])-start[:2]
    w = np.asarray(point[:2])-start[:2]
    denominator = float(v @ v)
    alpha = max(0., min(1., float(w @ v)/denominator)) if denominator else 0.
    return float(np.linalg.norm(w-alpha*v))


def scenario_bindings(path, observed):
    """Optional explicit scenario evidence, not size-based nonvehicle guessing.

    Require the original scenario UNIQUEID/DataID and a tight static-position
    or whole-waypoint-corridor validation for *every* recorded observation of
    the ID. Keep this a separately selected sensitivity run, never silently
    repair catalog-only evaluation or infer identity from a bag filename.
    """
    scenario_path = path.parent/'scenario.json'
    if not scenario_path.exists():
        raise ValueError('Scenario-assisted audit requires the companion scenario.json')
    scenario = json.loads(scenario_path.read_bytes())
    catalog = ROOT/'dataset/morai_od/configs/morai_asset_catalog_mr_base5_261005.json'
    assets = {int(v['data_id']): v for v in json.loads(catalog.read_bytes())['assets']}
    paths = {v['pathName']: v.get('waypointData', []) for v in scenario.get('waypointDataList', [])}
    bindings = {}
    for group, cls, prefix in [('objectList', 'obstacle', 'ObstaclePath'),
                               ('pedestrianList', 'pedestrian', 'PedPath')]:
        for obj in scenario.get(group, []):
            identity = int(obj['UNIQUEID']); key = '{}:{}'.format(cls, identity)
            samples = observed.get(key, [])
            asset = assets.get(int(obj['DataID']))
            if not samples or not asset or (asset.get('semantic_class') or '').lower() != cls:
                continue
            if any(name and name != asset['name'] for _, name in samples):
                raise ValueError('Recorded asset name contradicts companion scenario: '+key)
            initial = [float(obj['pos'][k]) for k in 'xyz']
            unity = paths.get('{} - {}'.format(prefix, identity), [])
            route = [[-float(p['pos']['x']), -float(p['pos']['z']), float(p['pos']['y'])] for p in unity]
            moving = cls == 'pedestrian' and len(route) >= 2
            errors = [min(point_segment_distance(p, a, b) for a, b in zip(route, route[1:]))
                      if moving else math.dist(p[:2], initial[:2]) for p, _ in samples]
            if max(errors) > .25:
                raise ValueError('Scenario ID reused or trajectory contradicts its static/waypoint corridor: '+key)
            scale = [float(obj.get('scale', {}).get(k, 1.)) for k in 'xyz']
            if min(scale) <= 0 or not np.isfinite(scale).all():
                raise ValueError('Invalid scenario scale')
            bindings[key] = dict(name=asset['name'], scale=scale, observations=len(samples),
                                 maximum_xy_corridor_error_m=max(errors), scenario_unique_id=identity,
                                 scenario_data_id=int(obj['DataID']), validation='all-status-positions vs explicit scenario path/position')
    return bindings, dict(path=str(scenario_path), sha256=hashlib.sha256(scenario_path.read_bytes()).hexdigest(),
                          asset_catalog_sha256=hashlib.sha256(catalog.read_bytes()).hexdigest())


def overlap_pairs(objects, threshold=.5):
    rows = []
    for i, a in enumerate(objects):
        for j in range(i+1, len(objects)):
            b = objects[j]
            # A cheap circumscribed-radius rejection; no false negative.
            distance = math.dist(a['center'][:2], b['center'][:2])
            if distance > .5*(math.hypot(*a['size'][:2])+math.hypot(*b['size'][:2])):
                continue
            iou = float(rotated_bev_iou(a, b))
            if iou > threshold:
                rows.append(dict(i=i, j=j, a_id=a.get('id'), b_id=b.get('id'),
                                 same_class=a['class_name'] == b['class_name'],
                                 class_a=a['class_name'], class_b=b['class_name'],
                                 center_distance_m=distance, rotated_bev_iou=iou))
    return rows


def object_row(obj, predicted=False):
    label = max(obj.classification, key=lambda c: c.probability).label if obj.classification else 0
    kinematics = obj.kinematics
    pose = (kinematics.initial_pose_with_covariance if predicted else kinematics.pose_with_covariance).pose
    row = dict(class_name=label_name(int(label)), label=int(label),
               center=[float(getattr(pose.position, k)) for k in 'xyz'],
               yaw=yaw_of(pose.orientation), size=[float(getattr(obj.shape.dimensions, k)) for k in 'xyz'],
               score=float(obj.existence_probability))
    if hasattr(obj, 'object_id'):
        row['id'] = bytes(obj.object_id.uuid).hex()
    if hasattr(kinematics, 'twist_with_covariance'):
        tw = kinematics.twist_with_covariance.twist
        row['speed'] = float(math.hypot(tw.linear.x, tw.linear.y))
    return row


def read_case(path, assisted=False):
    import rosbag
    alignment = None
    frames = defaultdict(list)
    ego_rows = []
    observed = defaultdict(list)
    with rosbag.Bag(str(path)) as bag:
        start = bag.get_start_time()
        selected_topics = [TRACK, DET, PRED, '/Ego_topic', '/tf_static'] + (['/Object_topic'] if assisted else [])
        for topic, msg, receipt in bag.read_messages(topics=selected_topics):
            stamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') else receipt.to_sec()
            stamp = stamp or receipt.to_sec()
            if topic == '/tf_static':
                for t in msg.transforms:
                    if t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'morai_map':
                        if abs(t.transform.rotation.x) > 1.e-8 or abs(t.transform.rotation.y) > 1.e-8:
                            raise ValueError('Nonplanar map alignment unsupported')
                        value = dict(xyz=[float(getattr(t.transform.translation, k)) for k in 'xyz'],
                                     yaw=yaw_of(t.transform.rotation))
                        if alignment and alignment != value:
                            raise ValueError('Recorded map alignment changed')
                        alignment = value
            elif topic == '/Ego_topic':
                ego_rows.append((stamp, dict(center=[msg.position.x, msg.position.y, msg.position.z],
                                             yaw=math.radians(msg.heading-1.347))))
            elif topic == '/Object_topic':
                for cls, items in [('obstacle', msg.obstacle_list), ('pedestrian', msg.pedestrian_list)]:
                    for obj in items:
                        observed['{}:{}'.format(cls, int(obj.unique_id))].append(
                            ([float(getattr(obj.position, k)) for k in 'xyz'], str(obj.name)))
            else:
                if topic == TRACK and msg.header.frame_id.lstrip('/') != 'map':
                    raise ValueError('Expected world-map tracking output; no guessed TF')
                frames[topic].append(dict(stamp=stamp, receipt=receipt.to_sec(), frame_id=msg.header.frame_id,
                                          objects=[object_row(o, topic == PRED) for o in msg.objects]))
    if not alignment:
        raise ValueError('Recorded map/morai_map transform absent')
    window = None
    events_file = path.parent/'events.jsonl'
    if events_file.exists():
        events = [json.loads(l) for l in events_file.read_text().splitlines() if l.strip()]
        begin = next((e['time'] for e in events if e['event'] == 'CONTROLLER_START'), None)
        end = next((e['time'] for e in events if e['event'] == 'RUN_FINISHED'), None)
        if begin and end:
            window = (datetime.fromisoformat(begin).timestamp(), datetime.fromisoformat(end).timestamp())
            for topic in frames:
                frames[topic] = [r for r in frames[topic] if window[0] <= r['receipt'] <= window[1]]
    a = alignment['yaw']; c, s = math.cos(a), math.sin(a); tx, ty, tz = alignment['xyz']
    ego_by_stamp = {}
    for stamp, row in ego_rows:
        x, y, z = row['center']
        ego_by_stamp[stamp] = dict(center=[c*x-s*y+tx, s*x+c*y+ty, z+tz], yaw=row['yaw']+a)
    ego = sorted(ego_by_stamp.items())
    bindings, companion = scenario_bindings(path, observed) if assisted else ({}, None)
    raw = read_morai_trajectories(path, include_classes=CLASSES, minimum_points=2, heading_correction_deg=-1.347,
                                asset_name_map={k: b['name'] for k, b in bindings.items()})
    corrected, resolution = transform_segments_to_asset_catalog_bbox_center(
        raw, ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json')
    # Catalog offsets/sizes are measured at unit scale. Scale the pivot-local
    # offset as well as the geometric BBox, not ObjectStatus's basic size.
    scaled = []
    for segment in corrected:
        choices = [b for b in bindings.values() if b['name'] == segment.asset_name]
        if len({tuple(b['scale']) for b in choices}) > 1:
            raise ValueError('Multiple scenario scales for one asset require explicit segment binding')
        scale = choices[0]['scale'] if choices else [1., 1., 1.]
        if scale != [1., 1., 1.]:
            pivot_segment = next(s for s in raw if s.key == segment.key)
            points = []
            for point, pivot in zip(segment.points, pivot_segment.points):
                c, s = math.cos(point.yaw), math.sin(point.yaw)
                dx, dy = point.x-pivot.x, point.y-pivot.y
                local = [c*dx+s*dy, -s*dx+c*dy]
                local = [local[0]*scale[0], local[1]*scale[1]]
                points.append(replace(point, x=pivot.x+c*local[0]-s*local[1],
                                      y=pivot.y+s*local[0]+c*local[1], z=pivot.z+(point.z-pivot.z)*scale[2],
                                      length=point.length*scale[0], width=point.width*scale[1], height=point.height*scale[2]))
            segment = replace(segment, points=tuple(points))
        scaled.append(segment)
    corrected = scaled
    corrected = transform_segments_planar(corrected, alignment['xyz'], alignment['yaw'])
    supported = {s.key for s in corrected}
    unresolved = transform_segments_planar([s for s in raw if s.key not in supported],
                                           alignment['xyz'], alignment['yaw'])
    def convert(seg):
        return ([p.timestamp for p in seg.points],
                [dict(key=seg.key, class_name=seg.class_name, asset=seg.asset_name,
                      center=[p.x, p.y, p.z], yaw=p.yaw, size=[p.length, p.width, p.height]) for p in seg.points])
    return dict(start=start, window=window, alignment=alignment, frames=frames,
                ego=([r[0] for r in ego], [r[1] for r in ego]),
                gt=[convert(s) for s in corrected], unresolved=[convert(s) for s in unresolved],
                gt_resolution=resolution, scenario_assisted_bindings=bindings, scenario_companion=companion)


def evaluate_gt(case, gate):
    rows, frame_rows, overlaps, confusion, excluded = [], [], [], Counter(), Counter()
    maximum_gap = case.get('maximum_interpolation_gap_sec', .2)
    multiplicity = []
    histories = defaultdict(list)
    class_changes = []
    last_label = {}
    geometric = defaultdict(Counter)
    counts = defaultdict(Counter)
    no_gt_matches = []
    for frame in case['frames'][TRACK]:
        stamp = frame['stamp']
        ego = sample_series(case['ego'], stamp, maximum_gap)
        if ego is None:
            excluded['no_bracketed_ego'] += 1
            continue
        unknown = [g for s in case['unresolved'] if (g := sample_series(s, stamp, maximum_gap)) is not None and in_roi(g, ego)]
        if unknown:
            excluded['unresolved_gt_in_roi'] += 1
            continue
        gt = [g for s in case['gt'] if (g := sample_series(s, stamp, maximum_gap)) is not None and in_roi(g, ego)]
        output = [o for o in frame['objects'] if in_roi(o, ego)]
        pairs, missing, extra = match(gt, output, gate)
        assigned = {j: gt[i] for i, j, _ in pairs}
        matched_gt = {i for i, _, _ in pairs}
        for g in gt:
            distance = math.dist(g['center'][:2], ego['center'][:2])
            for key in (g['class_name'], g['class_name']+'/'+band(distance), 'all'):
                counts[key]['gt_roi_observations'] += 1
        for i, j, distance_error in pairs:
            g, o = gt[i], output[j]
            distance = math.dist(g['center'][:2], ego['center'][:2])
            row = dict(stamp=stamp, gt_key=g['key'], asset=g['asset'], id=o['id'],
                       class_name=g['class_name'], distance_m=distance, band=band(distance),
                       center_error_m=distance_error, score=o['score'],
                       size_linf_error_m=max(abs(a-b) for a, b in zip(g['size'], o['size'])))
            rows.append(row)
            histories[g['key']].append(row)
            for key in (g['class_name'], g['class_name']+'/'+band(distance), 'all'):
                counts[key]['matched_observations'] += 1
        for o in output:
            distance = math.dist(o['center'][:2], ego['center'][:2])
            for key in (o['class_name'], o['class_name']+'/'+band(distance), 'all'):
                counts[key]['track_roi_observations'] += 1
            if o['id'] in last_label and last_label[o['id']] != o['class_name']:
                class_changes.append(dict(stamp=stamp, id=o['id'], previous_class=last_label[o['id']],
                                          class_name=o['class_name'], distance_m=distance))
            last_label[o['id']] = o['class_name']
        for j in extra:
            o = output[j]
            distance = math.dist(o['center'][:2], ego['center'][:2])
            no_gt_matches.append(dict(stamp=stamp, id=o['id'], class_name=o['class_name'],
                                       distance_m=distance, center=o['center'], score=o['score']))
            for key in (o['class_name'], o['class_name']+'/'+band(distance), 'all'):
                counts[key]['unmatched_tracks'] += 1
        # Secondary class-agnostic correspondence is a confusion candidate,
        # not a replacement for the primary class-preserving metric.
        for i, j, _ in match(gt, output, gate, False)[0]:
            confusion[gt[i]['class_name']+'->'+output[j]['class_name']] += 1
        # Low-IoU shape/position outliers can still be two IDs for one actor.
        # Keep this separate from overlap NMS evidence and require a clear
        # same-class nearest GT rather than merely two nearby output boxes.
        nearest_groups = defaultdict(list)
        for j, o in enumerate(output):
            ranked = sorted((math.dist(g['center'][:2], o['center'][:2]), i)
                            for i, g in enumerate(gt) if g['class_name'] == o['class_name'])
            if ranked and ranked[0][0] <= gate and (len(ranked) == 1 or ranked[1][0]-ranked[0][0] >= .5):
                nearest_groups[ranked[0][1]].append(j)
        for i, indices in nearest_groups.items():
            if len(indices) > 1:
                multiplicity.append(dict(stamp=stamp, gt_key=gt[i]['key'], class_name=gt[i]['class_name'],
                                         distance_m=math.dist(gt[i]['center'][:2], ego['center'][:2]),
                                         ids=[output[j]['id'] for j in indices],
                                         objects=[output[j] for j in indices],
                                         pair_center_distance_m=[math.dist(output[a]['center'][:2], output[b]['center'][:2])
                                                                for n, a in enumerate(indices) for b in indices[n+1:]],
                                         pair_iou=[float(rotated_bev_iou(output[a], output[b]))
                                                   for n, a in enumerate(indices) for b in indices[n+1:]]))
        for pair in overlap_pairs(output):
            nearest = []
            margins = []
            for index in (pair['i'], pair['j']):
                ranked = sorted((math.dist(g['center'][:2], output[index]['center'][:2]), g['key']) for g in gt)
                nearest.append(ranked[0] if ranked and ranked[0][0] <= gate else None)
                margins.append(ranked[1][0]-ranked[0][0] if len(ranked) > 1 else 1.e6)
            same_gt = bool(nearest[0] and nearest[1] and nearest[0][1] == nearest[1][1])
            clear_same_gt = same_gt and min(margins) >= .5
            overlaps.append(dict(stamp=stamp, **pair, same_nearest_gt=same_gt,
                                 same_gt_unambiguous_margin05=clear_same_gt,
                                 gt_key=nearest[0][1] if same_gt else None))
        frame_rows.append(dict(stamp=stamp, gt=len(gt), matched=len(pairs), missed=len(missing),
                               track=len(output), unmatched=len(extra)))
    switches, reacquisitions = identity_events(rows)
    id_gt_keys = defaultdict(set)
    for row in rows:
        id_gt_keys[row['id']].add(row['gt_key'])
    # Merge/reuse candidates are separate from GT -> UUID switching.
    reused = {k: sorted(v) for k, v in id_gt_keys.items() if len(v) > 1}
    summary = {}
    for key, count in counts.items():
        selected = [r for r in rows if key == 'all' or key == r['class_name'] or key == r['class_name']+'/'+r['band']]
        summary[key] = dict(count, center_error_m=stats([r['center_error_m'] for r in selected]),
                            size_linf_error_m=stats([r['size_linf_error_m'] for r in selected]))
    actors = [dict(gt_key=key, asset=values[0]['asset'], class_name=values[0]['class_name'],
                   matched_observations=len(values), unique_track_ids=len({r['id'] for r in values}),
                   ids=sorted({r['id'] for r in values}),
                   matched_range_m=stats([r['distance_m'] for r in values]),
                   match_gaps_sec=stats([b['stamp']-a['stamp'] for a, b in zip(values, values[1:])]))
              for key, values in histories.items()]
    return dict(gate_m=gate, evaluated_frames=len(frame_rows), excluded_frames=dict(excluded),
                primary_class_preserving=summary, class_agnostic_correspondence=dict(confusion),
                switch_candidates=len(switches), long_gap_different_id_reacquisitions=len(reacquisitions),
                same_uuid_matched_multiple_gt_actors=reused,
                uuid_class_changes=class_changes, actors=actors,
                overlap_candidates=len(overlaps),
                overlaps_same_nearest_gt=sum(r['same_nearest_gt'] for r in overlaps),
                overlaps_same_gt_margin05=sum(r['same_gt_unambiguous_margin05'] for r in overlaps),
                same_class_multiple_uuids_near_one_clear_gt=len(multiplicity),
                multiplicity_by_class=dict(Counter(r['class_name'] for r in multiplicity)),
                multiplicity_pair_center_distance_m=stats([v for r in multiplicity for v in r['pair_center_distance_m']]),
                multiplicity_pair_iou=stats([v for r in multiplicity for v in r['pair_iou']]),
                multiplicity_examples=multiplicity[:20],
                switch_events=switches, reacquisition_events=reacquisitions,
                overlap_examples=sorted(overlaps, key=lambda r: -r['rotated_bev_iou'])[:30]), \
           dict(matched_rows=rows, frame_rows=frame_rows, unmatched_tracks=no_gt_matches, overlaps=overlaps,
                multiplicity_candidates=multiplicity)


def audit(path, assisted=False):
    case = read_case(path, assisted)
    geometry = {}
    for topic, frames in case['frames'].items():
        overlaps = [dict(stamp=f['stamp'], **p) for f in frames for p in overlap_pairs(f['objects'])]
        ids = [o['id'] for f in frames for o in f['objects'] if 'id' in o]
        geometry[topic] = dict(frames=len(frames), object_observations=sum(len(f['objects']) for f in frames),
                              unique_uuids=len(set(ids)), overlap_iou_gt05=len(overlaps),
                              overlap_same_class=sum(p['same_class'] for p in overlaps),
                              overlap_cross_class=sum(not p['same_class'] for p in overlaps),
                              overlaps_iou=stats([p['rotated_bev_iou'] for p in overlaps]),
                              output_identity=output_identity_summary(frames),
                              examples=overlaps[:30])
    results, details = {}, {}
    for gate in (.5, 1., 2., 3.):
        results[str(gate)], details[str(gate)] = evaluate_gt(case, gate)
    return dict(inventory=inspect(path), recorded_alignment=case['alignment'], controller_window=case['window'],
                gt_resolution=case['gt_resolution'], geometry=geometry, gt_gate_sensitivity=results,
                scenario_assisted_bindings=case['scenario_assisted_bindings'], scenario_companion=case['scenario_companion'],
                limitations=['MORAI persistent GT identities are offline geometry-reconstructed, not guaranteed simulator oracle IDs.',
                             'Only catalog-resolved BBox centers are scored; any unresolved GT inside ego-aligned ROI excludes that frame.',
                             'GT/ego interpolation bracket <=0.2s; no extrapolation. Ego ROI uses MORAI actor pivot and corrected heading.',
                             'No visibility labels: unmatched ROI GT is not an AP false negative; unmatched output is not proof of false positive.',
                             '30Hz tracked headers are output times, not last observation ages. Measurement topic alone does not reveal last association.',
                             'Switch candidate is a different matched UUID within 0.5s; long-gap UUID replacement reported separately.',
                             'IoU overlap alone is a candidate: real crowded/compound objects may overlap legitimately.',
                             'Non-unit scenario scales are sensitivity assumptions applied to asset-local XYZ; exact Unity scale-axis/mesh shape is not certified by this bag.',
                             'No actual consumed planner path indices; selection-policy emulation is not closed-loop decision certification.']), details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario-assisted', action='store_true',
                        help='Separate sensitivity run requiring companion scenario IDs and all-observation geometry checks')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required; source recordings are never modified')
    args.output.mkdir(parents=True)
    rows = []
    for index, path in enumerate(args.bags):
        print('Auditing '+str(path), flush=True)
        report, details = audit(path, args.scenario_assisted)
        rows.append(report)
        with gzip.open(args.output/('case_{}_rows.json.gz'.format(index)), 'wt') as out:
            json.dump(details, out, allow_nan=False)
        (args.output/('case_{}_audit.json'.format(index))).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        selected = report['gt_gate_sensitivity']['2.0']
        print(json.dumps(dict(case=index, frames=selected['evaluated_frames'],
                              excluded=selected['excluded_frames'], switches=selected['switch_candidates'],
                              overlaps_same_gt=selected['overlaps_same_gt_margin05'],
                              classes=selected['primary_class_preserving']), indent=2), flush=True)
    from audit_base5_predicted_path_selection import source_contract
    payload = dict(source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   production_changed=False, scenario_assisted=args.scenario_assisted, gt_match_gate_primary_m=2., rows=rows,
                   planner_source_contract=source_contract(ROOT))
    (args.output/'summary.json').write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
