#!/usr/bin/env python3
"""Read-only temporal correlation around recorded UUID replacements.

No consumed-object/path trace exists in the selected bag, so temporal overlap
is explicitly not a causal attribution or a closed-loop replay.
"""
import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path

from audit_tracking_continuity import stats


def nearest(rows, stamp, max_age=.1):
    times = [r['stamp'] for r in rows]
    i = bisect_left(times, stamp)
    choices = rows[max(0, i-1):i+1]
    if not choices:
        return None
    row = min(choices, key=lambda r: abs(r['stamp']-stamp))
    return row if abs(row['stamp']-stamp) <= max_age else None


def summarize(rows, fields):
    result = {name: stats([r[name] for r in rows if r.get(name) is not None]) for name in fields}
    for name in fields:
        result[name+'_adjacent_abs_change'] = stats([abs(b[name]-a[name]) for a, b in zip(rows, rows[1:])
                                                   if a.get(name) is not None and b.get(name) is not None])
    return result


def audit(bag_path, continuity):
    import rosbag
    events = continuity['gt_gate_sensitivity']['2.0']['switch_events']
    centers = []
    for event in events:
        if not centers or event['stamp']-centers[-1]['stamp'] > 1.:
            centers.append(event)
    ego, ctrl, validation, trajectory, logs = [], [], [], [], []
    alignment = None
    topics = ['/Ego_topic', '/ctrl_cmd', '/planning/trajectory', '/planning/validation_status', '/rosout', '/tf_static']
    with rosbag.Bag(str(bag_path)) as bag:
        start = bag.get_start_time()
        for topic, msg, received in bag.read_messages(topics=topics):
            t = received.to_sec()
            header = msg.header.stamp.to_sec() if hasattr(msg, 'header') else t
            if topic == '/tf_static':
                for tr in msg.transforms:
                    if tr.header.frame_id.lstrip('/') == 'map' and tr.child_frame_id.lstrip('/') == 'morai_map':
                        q = tr.transform.rotation
                        value = dict(x=tr.transform.translation.x, y=tr.transform.translation.y,
                                     yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                        if alignment and alignment != value:
                            raise ValueError('Recorded map alignment changed')
                        alignment = value
            elif topic == '/Ego_topic':
                ego.append(dict(stamp=header or t, receipt=t, x=msg.position.x, y=msg.position.y,
                                heading=msg.heading, speed_mps=math.hypot(msg.velocity.x, msg.velocity.y)))
            elif topic == '/ctrl_cmd':
                ctrl.append(dict(stamp=t, mode=int(msg.longlCmdType), accel=float(msg.accel), brake=float(msg.brake),
                                 steering=float(msg.steering), velocity_field=float(msg.velocity),
                                 acceleration_field=float(msg.acceleration)))
            elif topic == '/planning/validation_status':
                failed = [k for k in msg.__slots__ if k.startswith('is_valid_') and not getattr(msg, k)]
                validation.append(dict(stamp=header or t, receipt=t, failed=failed, latency=float(msg.latency)))
            elif topic == '/planning/trajectory':
                points = msg.points
                velocity = [float(p.longitudinal_velocity_mps) for p in points]
                trajectory.append(dict(stamp=header or t, receipt=t, points=len(points),
                                       speed_first=velocity[0] if velocity else 0.,
                                       speed_max=max(velocity) if velocity else 0.,
                                       speed_first20_min=min(velocity[:20]) if velocity else 0.,
                                       xy=[[p.pose.position.x, p.pose.position.y] for p in points], velocity=velocity))
            elif topic == '/rosout' and msg.level >= 4:
                logs.append(dict(stamp=t, node=msg.name, level=msg.level, text=msg.msg))
    for rows in (ego, ctrl, validation, trajectory):
        rows.sort(key=lambda r: r['stamp'])
    if alignment is None:
        raise ValueError('No recorded map alignment for nearest-Ego trajectory sampling')
    c, s = math.cos(alignment['yaw']), math.sin(alignment['yaw'])
    for row in trajectory:
        ep = nearest(ego, row['stamp'])
        row['speed_nearest_ego'] = None
        row['nearest_ego_offset_m'] = None
        if ep and row['xy']:
            x = c*ep['x']-s*ep['y']+alignment['x']
            y = s*ep['x']+c*ep['y']+alignment['y']
            index = min(range(len(row['xy'])), key=lambda i: math.hypot(row['xy'][i][0]-x, row['xy'][i][1]-y))
            row['speed_nearest_ego'] = row['velocity'][index]
            row['nearest_ego_offset_m'] = math.hypot(row['xy'][index][0]-x, row['xy'][index][1]-y)
        del row['xy'], row['velocity']
    windows = []
    for center in centers:
        t = center['stamp']; duration = 2.
        selected = lambda rows: [r for r in rows if t-duration <= r['stamp'] <= t+duration]
        er, cr, vr, tr, lr = map(selected, (ego, ctrl, validation, trajectory, logs))
        ep = nearest(ego, t)
        windows.append(dict(event=center, seconds_from_bag_start=t-start,
                            window_sec=[t-duration-start, t+duration-start], ego_at_event=ep,
                            ego=summarize(er, ('speed_mps',)), control=summarize(cr, ('accel', 'brake', 'steering')),
                            control_modes=sorted({r['mode'] for r in cr}),
                            trajectory=summarize(tr, ('speed_first', 'speed_max', 'speed_first20_min', 'speed_nearest_ego', 'nearest_ego_offset_m')),
                            trajectory_messages=len(tr), validation_messages=len(vr),
                            validation_invalid=[r for r in vr if r['failed']], warning_logs=lr,
                            raw_ego_rows=er, raw_control_rows=cr, raw_trajectory_rows=tr))
    return dict(windows=windows,
                global_validation_invalid=[r for r in validation if r['failed']],
                limitation='Nearest GT UUID switch candidate is not the planner-consumed identity. No causal attribution: '
                           'traffic-light/crosswalk decisions, actor motion and detector delay can coexist. '
                           'Nearest-Ego trajectory sample uses closest recorded Ego plus static map alignment, not actual controller-consumed point. '
                           'Ctrl mode1 means accel/brake controls; unused velocity/acceleration fields are not target commands.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--continuity-report', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Fresh output required')
    a.output.mkdir(parents=True)
    report = audit(a.bag, json.loads(a.continuity_report.read_text()))
    (a.output/'planning_correlation.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    compact = [{k: v for k, v in window.items() if not k.startswith('raw_')} for window in report['windows']]
    print(json.dumps(compact, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
