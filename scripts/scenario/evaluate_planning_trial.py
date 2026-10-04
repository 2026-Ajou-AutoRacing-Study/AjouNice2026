#!/usr/bin/env python3
"""Compact bag evaluation; raw collision messages alone never imply ego impact."""
import argparse
import html
import json
import math
from collections import deque
from pathlib import Path
import rosbag
from shapely.geometry import Polygon
from perception_object_pipeline.morai_gt_detection import vehicle_bbox_center_offset
from gt_event_clock import ObjectClockOverlaps


def footprint(x, y, heading, length, width, offset):
    angle = math.radians(heading)
    c, s = math.cos(angle), math.sin(angle)
    x, y = x+offset*c, y+offset*s
    return Polygon([(x+u*c-v*s, y+u*s+v*c) for u,v in [
        (length/2,width/2),(length/2,-width/2),(-length/2,-width/2),(-length/2,width/2)]])


def write_trial_summary(root):
    rows = []
    for path in sorted(root.glob('official*/evaluation.json')):
        data = json.loads(path.read_text())
        values = [path.parent.name, data.get('evaluation_waypoint_reached'),
                  data.get('driving_time_to_evaluation_waypoint_sec'),
                  data.get('overlap_collision_ids', 'not evaluated'),
                  data.get('geometric_overlap_ids', 'not evaluated'),
                  data.get('unresolved_near_collision_ids', 'not evaluated'),
                  data.get('invalid_counts'), data.get('warning_counts')]
        cells = ''.join('<td>'+html.escape(str(v))+'</td>' for v in values)
        rows.append('<tr>'+cells+'</tr>')
    (root/'trial_summary.html').write_text(
        '<!doctype html><meta charset="utf-8"><title>시뮬레이션 반복 평가</title>'
        '<style>body{font:16px sans-serif;margin:30px}td,th{border:1px solid #aaa;padding:8px}'
        'table{border-collapse:collapse}</style><h1>회전교차로 → 고주로 → 지정 종료점</h1>'
        '<p>최대 100회. 도달 여부와 무충돌 보고만으로 미션 완성을 판정하지 않음. '
        '교통 난수 때문에 서로 다른 회차의 시간 차이는 단독 튜닝 효과가 아님.</p>'
        '<table><tr><th>회차</th><th>도달</th><th>주행시간 s</th><th>접촉+겹침 ID</th><th>GT 박스 겹침 후보</th>'
        '<th>미확정 근접 ID</th><th>궤적 검증 실패 표본</th><th>경고</th></tr>'+
        ''.join(rows)+'</table>')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    a = p.parse_args()
    intervention_file = a.bag.with_name('intervention.json')
    intervention = json.loads(intervention_file.read_text()) if intervention_file.exists() else None
    intervention_stamp = intervention.get('stamp') if intervention else None
    phase_stats = {name: dict(invalid_counts={}, warning_counts={}, geometric_overlaps={})
                   for name in ('before_intervention', 'after_intervention', 'unknown_boundary')}
    result = dict(near_ego_collision_ids=[], distant_collision_ids=[],
                  max_speed_mps=0., endpoint_min_distance_m=None,
                  invalid_counts={}, warning_counts={}, collision_events=[],
                  first_motion_sec=None, evaluation_waypoint_time_sec=None,
                  hard_deceleration_events=[], validation_extrema={},
                  optimizer_failure_events=[])
    ego = None
    ego_time = -1.
    object_clock = ObjectClockOverlaps(intervention_stamp)
    near, far = set(), set()
    gt = {}
    gt_time = -1.
    geometry = {}
    geometric_overlaps = {}
    proximity = {}
    fresh_gt_ego_samples = 0
    history = deque()
    in_decel = False
    active_factors = {}
    timing = {}
    with rosbag.Bag(str(a.bag)) as bag:
        t0 = bag.get_start_time()
        result['duration_sec'] = bag.get_end_time()-t0
        factor_topics = [n for n in bag.get_type_and_topic_info().topics
                         if n.startswith('/planning/planning_factors/')]
        timing_topics = [n for n in bag.get_type_and_topic_info().topics if n.endswith((
            '/path_optimizer/debug/processing_time_ms',
            '/path_smoother/debug/processing_time_ms',
            '/behavior_path_planner/debug/total_time/processing_time_ms'))]
        for topic, m, t in bag.read_messages(topics=[
                '/Ego_topic', '/Object_topic', '/CollisionData', '/planning/validation_status', '/rosout'] + factor_topics + timing_topics):
            elapsed = t.to_sec()-t0
            phase = ('unknown_boundary' if intervention_stamp is None else
                     'before_intervention' if t.to_sec() < intervention_stamp else 'after_intervention')
            phase_result = phase_stats[phase]
            if topic == '/Ego_topic':
                ego = m
                ego_time = elapsed
                # CollisionData can remain empty even on an impact. Check
                # fresh GT geometry independently; overlap is a review gate,
                # not calibrated physical-contact ground truth by itself.
                if 0 <= elapsed-gt_time < .2:
                    fresh_gt_ego_samples += 1
                    ep=footprint(m.position.x,m.position.y,m.heading,4.635,1.892,1.5275)
                    for object_id,npc in gt.items():
                        if (npc.size.x<=0 or npc.size.y<=0 or
                            math.hypot(npc.position.x-m.position.x,npc.position.y-m.position.y)>25):
                            continue
                        np=footprint(npc.position.x,npc.position.y,npc.heading,npc.size.x,npc.size.y,
                            vehicle_bbox_center_offset(npc.size.x,npc.size.y,npc.size.z))
                        stats = proximity.setdefault(object_id, dict(samples=0, min_gap_m=math.inf))
                        stats['samples'] += 1
                        gap = ep.distance(np)
                        if gap < stats['min_gap_m']:
                            stats.update(min_gap_m=gap, min_gap_sec=elapsed)
                        if ep.intersects(np):
                            g=geometric_overlaps.setdefault(object_id,dict(first_sec=elapsed,samples=0))
                            g['last_sec']=elapsed
                            g['samples']+=1
                            pg = phase_result['geometric_overlaps'].setdefault(
                                object_id, dict(first_sec=elapsed, samples=0))
                            pg.update(last_sec=elapsed, samples=pg['samples']+1)
                result['max_speed_mps'] = max(result['max_speed_mps'], math.sqrt(
                    m.velocity.x**2+m.velocity.y**2+m.velocity.z**2))
                speed = abs(m.velocity.x)
                if speed > .5 and result['first_motion_sec'] is None:
                    result['first_motion_sec'] = elapsed
                history.append((elapsed, speed))
                while len(history)>1 and elapsed-history[1][0]>.5:
                    history.popleft()
                if elapsed-history[0][0]>.4 and result['evaluation_waypoint_time_sec'] is None:
                    decel=(speed-history[0][1])/(elapsed-history[0][0])
                    if decel < -2 and not in_decel:
                        result['hard_deceleration_events'].append(dict(t=elapsed,
                            accel_mps2=decel, speed_mps=speed,
                            reported_factor_modules=[k for k,v in active_factors.items() if v]))
                        in_decel = True
                    elif decel >= -1.5:
                        in_decel = False
                # Route B wp_200183: explicit evaluation waypoint, not a
                # claim of surveyed tollgate position or whole-mission success.
                d = math.hypot(m.position.x+131.559130220-202.157329135,
                               m.position.y+545.911338186-132.337284715)
                prev = result['endpoint_min_distance_m']
                result['endpoint_min_distance_m'] = d if prev is None else min(prev, d)
                if d < 10 and result['evaluation_waypoint_time_sec'] is None:
                    result['evaluation_waypoint_time_sec'] = elapsed
            elif topic in factor_topics:
                active_factors[topic.rsplit('/',1)[-1]] = bool(m.factors)
            elif topic in timing_topics and math.isfinite(m.data):
                timing.setdefault(topic, []).append(m.data)
            elif topic == '/Object_topic':
                gt = {q.unique_id:q for q in m.npc_list}
                gt_time = elapsed
                # The driving review gate observes Object callbacks. Ego-only
                # replay can miss a short overlap immediately before a brake
                # latch, then incorrectly attribute all geometry to the brake.
                # Keep both event clocks separately and never rewrite stamps.
                if ego is not None and 0 <= elapsed-ego_time < .2:
                    ep = footprint(ego.position.x, ego.position.y, ego.heading,
                                   4.635, 1.892, 1.5275)
                    for object_id, npc in gt.items():
                        if (npc.size.x <= 0 or npc.size.y <= 0 or
                            math.hypot(npc.position.x-ego.position.x,
                                       npc.position.y-ego.position.y) > 25):
                            continue
                        np = footprint(npc.position.x, npc.position.y, npc.heading,
                                       npc.size.x, npc.size.y,
                                       vehicle_bbox_center_offset(npc.size.x, npc.size.y, npc.size.z))
                        if ep.intersects(np):
                            object_clock.record(object_id, t.to_sec(), elapsed,
                                                elapsed-ego_time, ep.intersection(np).area)
            elif topic == '/CollisionData' and ego is not None:
                for q in m.collision_object:
                    d = math.hypot(q.position.x-ego.position.x, q.position.y-ego.position.y)
                    bucket = near if d < 15 else far
                    if q.unique_id not in bucket:
                        result['collision_events'].append(dict(t=elapsed,
                            object_id=q.unique_id, distance_m=d, near_ego=d < 15))
                    bucket.add(q.unique_id)
                    npc = gt.get(q.unique_id)
                    if d < 15 and npc is not None and elapsed-gt_time < .2 and npc.size.x>0 and npc.size.y>0:
                        ep=footprint(ego.position.x,ego.position.y,ego.heading,4.635,1.892,1.5275)
                        np=footprint(npc.position.x,npc.position.y,npc.heading,npc.size.x,npc.size.y,
                            vehicle_bbox_center_offset(npc.size.x,npc.size.y,npc.size.z))
                        g=geometry.setdefault(q.unique_id,dict(min_gap_m=math.inf,overlap_samples=0))
                        g['min_gap_m']=min(g['min_gap_m'],ep.distance(np))
                        if ep.intersects(np):
                            g['overlap_samples'] += 1
                            g.setdefault('first_overlap_sec', elapsed)
                            g['last_overlap_sec'] = elapsed
            elif topic == '/planning/validation_status':
                for key in ['max_lateral_acc', 'max_lateral_jerk', 'min_longitudinal_acc']:
                    value = getattr(m, key)
                    prior = result['validation_extrema'].get(key)
                    if prior is None or abs(value) > abs(prior['value']):
                        result['validation_extrema'][key] = dict(value=value, t=elapsed)
                for key in m.__slots__:
                    if key.startswith('is_valid_') and not getattr(m, key):
                        result['invalid_counts'][key] = result['invalid_counts'].get(key, 0)+1
                        counts = phase_result['invalid_counts']
                        counts[key] = counts.get(key, 0)+1
            elif topic == '/rosout':
                if '[MPT] Optimization failed' in m.msg:
                    result['optimizer_failure_events'].append(dict(
                        t=elapsed, node=m.name, message=m.msg))
                for pattern in ['primal infeasible', 'Path miss detected', 'Ego is out of route',
                                'Stop distance calculation failed', 'MVP planning failed']:
                    if pattern in m.msg:
                        result['warning_counts'][pattern] = result['warning_counts'].get(pattern, 0)+1
                        counts = phase_result['warning_counts']
                        counts[pattern] = counts.get(pattern, 0)+1
    result['near_ego_collision_ids'] = sorted(near)
    result['processing_time_ms'] = {}
    for name, samples in timing.items():
        samples.sort()
        result['processing_time_ms'][name] = dict(count=len(samples),
            median=samples[len(samples)//2], p95=samples[int((len(samples)-1)*.95)],
            maximum=samples[-1])
    result['distant_collision_ids'] = sorted(far)
    result['collision_footprint_checks'] = geometry
    result['ego_clock_geometric_overlap_ids'] = sorted(geometric_overlaps)
    result['geometric_overlap_ids'] = sorted(set(geometric_overlaps) | set(object_clock.all))
    result['object_clock_geometric_overlap_candidates'] = object_clock.all
    result['object_clock_intervention_phase_statistics'] = object_clock.by_phase
    result['object_clock_samples_are_callbacks_not_unique_frames'] = True
    result['fresh_gt_ego_samples'] = fresh_gt_ego_samples
    result['nearby_gt_footprint_checks'] = proximity
    result['geometric_overlap_candidates'] = geometric_overlaps
    result['overlap_collision_ids'] = sorted(k for k,v in geometry.items() if v['overlap_samples'])
    result['unresolved_near_collision_ids'] = sorted(near-set(geometry))
    result['evaluation_waypoint_reached'] = (
        result['endpoint_min_distance_m'] is not None and result['endpoint_min_distance_m'] < 15)
    result['mission_pass'] = None
    result['intervention'] = intervention
    result['intervention_phase_statistics'] = phase_stats
    result['attribution_note'] = (
        'Totals retain all events. After-intervention events are not autonomous-planner-only evidence; '
        'unknown boundary is not assumed to be pre-intervention. '
        'Ego-clock and Object-clock GT geometry are separately reported; '
        'their ID union is not proof of calibrated physical contact.')
    termination = a.bag.with_name('termination.txt')
    result['termination_reason'] = termination.read_text().strip() if termination.exists() else None
    result['planner_safety_failure'] = result['termination_reason'] == 'planner_no_safe_maneuver'
    if result['first_motion_sec'] is not None and result['evaluation_waypoint_time_sec'] is not None:
        result['driving_time_to_evaluation_waypoint_sec'] = (
            result['evaluation_waypoint_time_sec']-result['first_motion_sec'])
    result['note'] = 'Near collision is a contact candidate; inspect GT geometry. Waypoint completion is not full mission validation.'
    text = json.dumps(result, indent=2, ensure_ascii=False)
    a.bag.with_name('evaluation.json').write_text(text+'\n')
    a.bag.with_name('evaluation.html').write_text('<!doctype html><meta charset="utf-8">'
        '<h1>Planning trial evidence</h1><pre>'+html.escape(text)+'</pre>')
    write_trial_summary(a.bag.parent.parent)
    print(text)


if __name__ == '__main__':
    main()
