#!/usr/bin/env python3
"""Inspect manual mission bags without detector inference or image/cloud decoding."""
import argparse
import bisect
import json
import math
import struct
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import rosbag


def stats(values):
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    return {k: float(v) for k, v in zip(
        ('min', 'p50', 'p95', 'max', 'mean'),
        [a.min(), np.percentile(a, 50), np.percentile(a, 95), a.max(), a.mean()])}


def inspect(path):
    records, headers, delay = defaultdict(list), defaultdict(list), defaultdict(list)
    ego, objects, collisions = [], [], []
    names = defaultdict(set)
    trajectories = defaultdict(list)
    reported_vectors = defaultdict(list)
    counts = Counter()
    with rosbag.Bag(str(path)) as bag:
        info = bag.get_type_and_topic_info()
        start, end = bag.get_start_time(), bag.get_end_time()
        topics = [t for t in info.topics if t.startswith('/sensor/')]
        topics += [t for t in ('/Ego_topic', '/Object_topic', '/CollisionData') if t in info.topics]
        for topic, raw, bt in bag.read_messages(topics=topics, raw=True):
            stamp = bt.to_sec()
            data = raw[1]
            records[topic].append(stamp)
            if len(data) >= 12:
                _, sec, ns = struct.unpack_from('<III', data)
                hs = sec + ns * 1e-9
                headers[topic].append(hs)
                delay[topic].append(stamp - hs)
            if topic.startswith('/sensor/'):
                continue
            message = raw[-1]().deserialize(data)
            if topic == '/Ego_topic':
                ego.append((stamp, message.position.x, message.position.y,
                            message.heading, message.velocity.x))
            elif topic == '/Object_topic':
                rows = []
                for cls, field in (('vehicle', 'npc_list'), ('pedestrian', 'pedestrian_list'),
                                   ('obstacle', 'obstacle_list')):
                    for obj in getattr(message, field):
                        names[cls].add(obj.name)
                        trajectories[(cls, obj.unique_id)].append(
                            (stamp, obj.position.x, obj.position.y))
                        reported_vectors[cls].append(
                            math.sqrt(obj.velocity.x**2 + obj.velocity.y**2 + obj.velocity.z**2))
                        rows.append((cls, obj.unique_id, obj.position.x, obj.position.y,
                                     obj.velocity.x))
                objects.append((stamp, rows))
            elif getattr(message, 'collision_object', []) or getattr(message, 'collision_objecta', []):
                collisions.append(stamp)
    timing = {}
    for topic, ts in records.items():
        dt = np.diff(ts)
        hd = np.diff(headers[topic])
        timing[topic] = dict(count=len(ts), record_hz=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else None,
                             record_dt_sec=stats(dt.tolist()), header_dt_sec=stats(hd.tolist()),
                             header_zero_count=sum(x == 0 for x in headers[topic]),
                             header_backward_count=int((hd < 0).sum()),
                             header_repeat_count=int((hd == 0).sum()),
                             receipt_minus_header_sec=stats(delay[topic]))
    movement = None
    nearby = defaultdict(list)
    nearby_counts = Counter()
    nearby_ids = defaultdict(set)
    nearby_times = defaultdict(set)
    target_distances = defaultdict(list)
    if ego:
        a = np.asarray(ego)
        distances = np.linalg.norm(np.diff(a[:, 1:3], axis=0), axis=1)
        jumps = np.flatnonzero(distances > 3)
        travel = float(distances.sum())
        velocity_integral = float(np.trapz(np.abs(a[:, 4]), a[:, 0]))
        movement = dict(speed_kmh=stats((np.abs(a[:, 4])*3.6).tolist()),
                        stopped_fraction=float((np.abs(a[:, 4]) < .5).mean()),
                        path_length_m=travel, velocity_integral_m=velocity_integral,
                        distance_to_velocity_integral=travel/velocity_integral if velocity_integral else None,
                        step_distance_m=stats(distances.tolist()),
                        jumps_over_3m=[dict(time_sec=float(a[i+1, 0]-start), distance_m=float(distances[i])) for i in jumps],
                        gaps_over_300ms=[dict(start_sec=float(a[i,0]-start),
                            end_sec=float(a[i+1,0]-start),
                            step_distance_m=float(distances[i]))
                            for i in np.flatnonzero(np.diff(a[:,0]) > .3)],
                        xy_bounds=[a[:,1:3].min(axis=0).tolist(),a[:,1:3].max(axis=0).tolist()])
        et = a[:,0].tolist()
        for stamp, rows in objects:
            j = bisect.bisect_left(et, stamp)
            candidates = [i for i in (j-1,j) if 0 <= i < len(et)]
            i = min(candidates,key=lambda k:abs(et[k]-stamp))
            if abs(et[i]-stamp) > .1:
                counts['object_without_ego_within_100ms'] += 1
                continue
            yaw = math.radians(a[i,3]-1.347)
            for cls, uid, x, y, vx in rows:
                dx, dy = x-a[i,1], y-a[i,2]
                distance = math.hypot(dx,dy)
                target_distances[cls].append(distance)
                if distance <= 30:
                    nearby[cls].append(abs(vx)*3.6)
                    nearby_counts[cls+'_'+('front' if dx*math.cos(yaw)+dy*math.sin(yaw)>=0 else 'rear')] += 1
                    nearby_ids[cls].add(uid)
                    nearby_times[cls].add(stamp)
    camera_alignment = {}
    lidar = sorted(headers.get('/sensor/lidar1', []))
    for topic in records:
        if '/camera/' not in topic:
            continue
        stamps = sorted(headers[topic])
        nearest = []
        for t in lidar:
            j = bisect.bisect_left(stamps,t)
            choices = [abs(stamps[k]-t) for k in (j-1,j) if 0 <= k < len(stamps)]
            if choices:
                nearest.append(min(choices))
        camera_alignment[topic] = dict(nearest_header_delta_sec=stats(nearest),
                                      over_100ms_count=sum(d>.1 for d in nearest))
    events = []
    for stamp in collisions:
        if not events or stamp-events[-1]['last_absolute'] > .5:
            events.append(dict(start_sec=stamp-start,end_sec=stamp-start,last_absolute=stamp,messages=1))
        else:
            events[-1].update(end_sec=stamp-start,last_absolute=stamp,messages=events[-1]['messages']+1)
    for event in events:
        del event['last_absolute']
    object_motion = []
    for (cls, uid), observations in trajectories.items():
        a = np.asarray(observations)
        d = np.linalg.norm(np.diff(a[:,1:3], axis=0), axis=1)
        object_motion.append(dict(class_name=cls, unique_id=uid,
            observations=len(observations),
            xy_span_m=np.ptp(a[:,1:3],axis=0).tolist(),
            accumulated_pivot_travel_m=float(d.sum()),
            max_step_m=float(d.max()) if len(d) else 0))
    return dict(path=str(path), duration_sec=end-start,
                topic_counts={t:v.message_count for t,v in info.topics.items()},
                timing=timing, ego=movement, collision_events=events,
                nearby_pivot_30m=dict(speed_kmh={c:stats(v) for c,v in nearby.items()},
                    unique_ids={c:len(v) for c,v in nearby_ids.items()},
                    object_frames={c:len(v) for c,v in nearby_times.items()},
                    front_rear_observations=dict(nearby_counts)),
                object_pivot_distance_m={c:stats(v) for c,v in target_distances.items()},
                object_names={c:sorted(v) for c,v in names.items()}, diagnostics=dict(counts),
                object_motion=object_motion,
                object_reported_vector_norm_mps={c:stats(v) for c,v in reported_vectors.items()},
                camera_lidar_header_alignment=camera_alignment)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    results = []
    for path in sorted(args.root.rglob('*.bag')):
        result = inspect(path)
        results.append(result)
        e = result['ego']
        print(path.name, 'ego_max_kmh=%.1f'%e['speed_kmh']['max'],
              'path_m=%.1f'%e['path_length_m'], 'jumps=%d'%len(e['jumps_over_3m']),
              'collision_events=%d'%len(result['collision_events']),
              'nearby='+str(result['nearby_pivot_30m']['speed_kmh']), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(schema_version=1, scope='raw-only; no model inference',
        caveats=['Object distance uses topic pivot, not catalog-adjusted BBox center.',
                 'Nearest camera deltas use both past/future frames; causal live alignment differs.',
                 'Topic names are opaque identifiers, not necessarily asset model names.',
                 'Velocity uses MORAI forward velocity.x; receipt time is the bag time.'], bags=results), indent=2)+'\n')


if __name__ == '__main__':
    main()
