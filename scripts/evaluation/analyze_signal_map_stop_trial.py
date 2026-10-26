#!/usr/bin/env python3
"""Read a closed validation bag; measure actual near-line stops and save original JPEGs.

Distances refer to the selected Lanelet ref_line, not an official penalty line.
No native state is used as planner input. No inference or image editing occurs.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def quantiles(values):
    values = sorted(values)
    if not values:
        return None
    return dict(min=values[0], p50=values[len(values)//2], max=values[-1])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--observer', type=Path, required=True)
    p.add_argument('--geometry', type=Path, required=True)
    p.add_argument('--paint-geometry', type=Path,
                   help='Optional independent original MGeo paint proxy; never replaces runtime geometry')
    p.add_argument('--group-id', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Use a fresh output; never overwrite evidence')
    if a.bag.suffix != '.bag':
        p.error('Use a closed .bag, never an active recording')
    bag_stat = a.bag.stat()
    observer_path = a.observer/'observations.jsonl'
    observer_stat = observer_path.stat()
    observer_sha = hashlib.sha256()
    import rosbag
    group = next(g for g in json.loads(a.geometry.read_text())['groups'] if g['group_id'] == a.group_id)
    first, last = group['stop_line_xy'][0], group['stop_line_xy'][-1]
    dx, dy = last[0]-first[0], last[1]-first[1]
    length = math.hypot(dx, dy)
    nx, ny = -dy/length, dx/length
    # Approach headings from the static physical signal geometry, not raw Ego yaw.
    root = Path(__file__).resolve().parents[2]
    physical = json.loads((root/'base_ws/src/perception/traffic_light_detection/config/traffic_light_map_geometry.json').read_text())
    heading = math.radians(next(h['heading'] for h in physical['heads'] if h.get('group_id') == a.group_id)+180.)
    if nx*math.cos(heading)+ny*math.sin(heading) < 0:
        nx, ny = -nx, -ny
    center = [(first[i]+last[i])/2 for i in range(2)]
    rows, frames, images = [], [], []
    topics = ['/localization/kinematic_state', '/traffic_light_node/debug_runtime',
              '/sensor/camera/front/compressed']
    with rosbag.Bag(str(a.bag)) as bag:
        for topic, m, t in bag.read_messages(topics=topics):
            if topic == topics[0]:
                q = m.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                x = m.pose.pose.position.x+3.845*math.cos(yaw)
                y = m.pose.pose.position.y+3.845*math.sin(yaw)
                gap = (center[0]-x)*nx+(center[1]-y)*ny
                lateral = ((x-center[0])*dx+(y-center[1])*dy)/length
                speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
                rows.append(dict(stamp=m.header.stamp.to_nsec(), gap=gap, speed=speed, lateral=lateral,
                                 bumper_xy=[x,y]))
            elif topic == topics[1]:
                body = json.loads(m.data)
                if (body.get('frame_decision') or {}).get('group_id') == a.group_id:
                    frames.append(body)
            else:
                images.append(m.header.stamp.to_nsec())
    # Closed, contiguous near-line stationary periods; not startup P at 15..20m.
    windows, current = [], []
    for row in rows:
        eligible = (row['speed'] < .15 and -.5 < row['gap'] < 8.
                    and abs(row['lateral']) <= length/2+1.5)
        if current and (not eligible or (row['stamp']-current[-1]['stamp']) > 500_000_000):
            if current[-1]['stamp']-current[0]['stamp'] >= 1_000_000_000:
                windows.append(current)
            current = []
        if eligible:
            current.append(row)
    if current and current[-1]['stamp']-current[0]['stamp'] >= 1_000_000_000:
        windows.append(current)
    native, poses = [], []
    with observer_path.open('rb') as stream:
        for line in stream:
            observer_sha.update(line)
            r = json.loads(line)
            if r['event'] == 'native_read_only_sample' and r['native_id'] in group['native_ids']:
                native.append(r)
            elif r['event'] == 'ego_front_GT_proxy':
                poses.append(r)
    paint_crossings = []
    if a.paint_geometry is not None:
        from observe_traffic_light_compliance import segment_crossing, evaluate_crossing, load_geometry
        paint_groups, _ = load_geometry(a.paint_geometry)
        paint = next(g for g in paint_groups if g['group_id'] == a.group_id)
        for prev, row in zip(poses, poses[1:]):
            dt = row['estimated_source_monotonic']-prev['estimated_source_monotonic']
            if not 0 < dt < .5 or math.dist(prev['front_point_map_xyz_m'], row['front_point_map_xyz_m']) > 20:
                continue
            for left, right in zip(paint['stop_line_xy'], paint['stop_line_xy'][1:]):
                crossing = segment_crossing(prev['front_point_map_xyz_m'][:2], row['front_point_map_xyz_m'][:2], left, right)
                if crossing is None:
                    continue
                fraction = crossing['motion_fraction']
                event = dict(crossing, group_id=a.group_id,
                    pose_time_verified=prev['pose_time_verified'] and row['pose_time_verified'],
                    estimated_source_monotonic=prev['estimated_source_monotonic']+fraction*dt)
                paint_crossings.append(dict(event, native_bracket_evaluation=evaluate_crossing(event,native,paint)))
    reports, wanted = [], set()
    for window in windows:
        start, end = window[0]['stamp'], window[-1]['stamp']
        seen = [f for f in frames if start <= f.get('source_stamp_ns', 0) <= end]
        native_seen = [r for r in native if r.get('valid') and start <= r['request_ros_ns'] <= r['response_ros_ns'] <= end]
        mid = (start+end)//2
        i = bisect_left(images, mid)
        choices = images[max(0,i-1):min(len(images),i+1)]
        jpeg = min(choices, key=lambda s:abs(s-mid)) if choices else None
        if jpeg is not None:
            wanted.add(jpeg)
        reports.append(dict(start_stamp_ns=start, end_stamp_ns=end, duration_sec=(end-start)/1e9,
            front_bumper_gap_to_map_line_m=quantiles([r['gap'] for r in window]),
            selection_status=dict(Counter(f['frame_decision'].get('selection_status') for f in seen)),
            classifier_known=sum(f['frame_decision'].get('classifier_known') is True for f in seen),
            recorded_runtime_frames=len(seen),
            observed_lamps=dict(Counter(str(f['frame_decision'].get('observed_tuples')) for f in seen)),
            native_sample_colors=dict(Counter(str(r['color']) for r in native_seen)),
            jpeg_source_stamp_ns=jpeg,
            jpeg_delta_to_midpoint_sec=(jpeg-mid)/1e9 if jpeg is not None else None))
    a.output.mkdir(parents=True)
    with rosbag.Bag(str(a.bag)) as bag:
        for _, m, _ in bag.read_messages(topics=[topics[2]]):
            if m.header.stamp.to_nsec() in wanted:
                stamp = m.header.stamp.to_nsec()
                raw = bytes(m.data)
                target = a.output/f'stopped_{stamp}.jpg'
                target.write_bytes(raw)
                for r in reports:
                    if r['jpeg_source_stamp_ns'] == stamp:
                        r.update(jpeg=str(target.resolve()), jpeg_sha256=hashlib.sha256(raw).hexdigest())
                wanted.remove(stamp)
    result = dict(mission=group['mission'], group_id=a.group_id, bag=str(a.bag.resolve()),
        bag_bytes=bag_stat.st_size, observer_sha256=observer_sha.hexdigest(),
        geometry_sha256=hashlib.sha256(a.geometry.read_bytes()).hexdigest(),
        front_bumper_offset_m=3.845, distance_source='recorded localization/kinematic_state, not independent ground truth',
        stationary_windows=reports, no_stop_is_not_a_stop_test_pass=not bool(reports),
        original_paint_proxy_crossings=paint_crossings,
        paint_geometry_sha256=hashlib.sha256(a.paint_geometry.read_bytes()).hexdigest() if a.paint_geometry else None,
        limit='Native samples and pixel inspection are separate from official penalty certification')
    if ((a.bag.stat().st_size, a.bag.stat().st_mtime_ns) != (bag_stat.st_size, bag_stat.st_mtime_ns)
            or (observer_path.stat().st_size, observer_path.stat().st_mtime_ns) !=
            (observer_stat.st_size, observer_stat.st_mtime_ns)):
        raise ValueError('Input changed during analysis; partial output is not qualified')
    (a.output/'measurements.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
