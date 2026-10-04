#!/usr/bin/env python3
"""Evidence-only fixed-NPC detector A/B summary. Does not imply randomized replay."""
import argparse
import html
import json
import math
from pathlib import Path
import rosbag
import rospy
import tf2_ros
import tf2_geometry_msgs
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from evaluate_planning_trial import footprint, vehicle_bbox_center_offset


def analyze(path, object_id):
    result = dict(bag=str(path), first={}, approach_first={}, path_at_distance={}, tf_failures=0, min_footprint_gap_m=None,
                  npc_max_speed_mps=0., npc_max_displacement_m=0.,
                  npc_pre_overlap_max_speed_mps=0., npc_pre_overlap_max_displacement_m=0.)
    result['experiment']=json.loads(path.with_name('experiment.json').read_text())
    result['_trace']=[]
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2000))
    with rosbag.Bag(str(path)) as bag:
        t0 = bag.get_start_time()
        for topic, msg, _ in bag.read_messages(topics=['/tf','/tf_static']):
            for transform in msg.transforms:
                if topic == '/tf_static':
                    buffer.set_transform_static(transform, 'bag')
                else:
                    buffer.set_transform(transform, 'bag')
        ego, npc, origin = None, None, None
        topics=['/Ego_topic','/Object_topic','/ctrl_cmd',
            '/evaluation/od_gt/detection/objects',
            '/perception/object_recognition/detection/objects',
            '/perception/object_recognition/objects','/planning/trajectory']
        for topic,msg,t in bag.read_messages(topics=topics):
            dt=t.to_sec()-t0
            if topic == '/Ego_topic':
                ego=msg
            elif topic == '/Object_topic':
                npc=next((q for q in msg.npc_list if q.unique_id==object_id),None)
                if npc:
                    if origin is None:
                        origin=(npc.position.x,npc.position.y)
                        result['npc_initial_xy']=origin
                    result['npc_max_speed_mps']=max(result['npc_max_speed_mps'],abs(npc.velocity.x))
                    result['npc_max_displacement_m']=max(result['npc_max_displacement_m'],
                        math.hypot(npc.position.x-origin[0],npc.position.y-origin[1]))
                    if 'footprint_overlap' not in result['first']:
                        result['npc_pre_overlap_max_speed_mps']=max(
                            result['npc_pre_overlap_max_speed_mps'],abs(npc.velocity.x))
                        result['npc_pre_overlap_max_displacement_m']=max(
                            result['npc_pre_overlap_max_displacement_m'],
                            math.hypot(npc.position.x-origin[0],npc.position.y-origin[1]))
            if not ego or not npc:
                continue
            dx,dy=npc.position.x-ego.position.x,npc.position.y-ego.position.y
            distance=math.hypot(dx,dy)
            def mark(key, **extra):
                event=dict(t=dt, reference_point_distance_m=distance,
                    ego_speed_kph=abs(ego.velocity.x)*3.6,**extra)
                result['first'].setdefault(key,event)
                if dt>30 and 0 < ego.position.y-npc.position.y < 200 and abs(dx)<25:
                    result['approach_first'].setdefault(key,event)
            if topic == '/Ego_topic':
                if dt>30 and -50 < ego.position.y-npc.position.y < 200 and abs(dx)<25:
                    result['_trace'].append((ego.position.y-npc.position.y,
                        abs(ego.velocity.x)*3.6,ego.position.x-npc.position.x))
                ep=footprint(ego.position.x,ego.position.y,ego.heading,4.635,1.892,1.5275)
                np=footprint(npc.position.x,npc.position.y,npc.heading,npc.size.x,npc.size.y,
                    vehicle_bbox_center_offset(npc.size.x,npc.size.y,npc.size.z))
                gap=ep.distance(np)
                prev=result['min_footprint_gap_m']
                result['min_footprint_gap_m']=gap if prev is None else min(prev,gap)
                if distance<100: mark('within_100m')
                if distance<150 and abs(ego.velocity.x)<.1: mark('stopped_near_obstacle')
                if ep.intersects(np): mark('footprint_overlap')
            elif topic == '/ctrl_cmd':
                if distance<150 and msg.brake>.1:mark('brake_over_0_1')
            elif topic == '/planning/trajectory' and msg.points and dt>30 and ego.position.y>npc.position.y:
                for threshold in [200,150,100,75,50,30]:
                    if distance>threshold or str(threshold) in result['path_at_distance']:continue
                    target_y=npc.position.y+545.911338186
                    nearest=min(msg.points,key=lambda q:abs(q.pose.position.y-target_y))
                    if abs(nearest.pose.position.y-target_y)>2:continue
                    result['path_at_distance'][str(threshold)]=dict(t=dt,
                        reference_point_distance_m=distance,ego_speed_kph=abs(ego.velocity.x)*3.6,
                        path_x_morai=nearest.pose.position.x-131.559130220,
                        npc_x=npc.position.x,
                        lateral_center_separation_m=abs(nearest.pose.position.x-131.559130220-npc.position.x),
                        planned_velocity_kph=nearest.longitudinal_velocity_mps*3.6)
            elif topic.endswith('/objects'):
                key= ('od_gt_detection' if topic.startswith('/evaluation/') else
                      'bevfusion_detection' if '/detection/' in topic else 'planner_prediction')
                if key in result['first'] and key in result['approach_first']: continue
                offset=vehicle_bbox_center_offset(npc.size.x,npc.size.y,npc.size.z)
                angle=math.radians(npc.heading)
                target=(npc.position.x+offset*math.cos(angle),npc.position.y+offset*math.sin(angle))
                for obj in msg.objects:
                    pose=PoseStamped(header=msg.header)
                    k=obj.kinematics
                    pose.pose=(k.initial_pose_with_covariance if key=='planner_prediction' else k.pose_with_covariance).pose
                    try:
                        transform=buffer.lookup_transform('morai_map',pose.header.frame_id,pose.header.stamp)
                        point=tf2_geometry_msgs.do_transform_pose(pose,transform).pose.position
                    except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
                        result['tf_failures']+=1
                        continue
                    error=math.hypot(point.x-target[0],point.y-target[1])
                    if error<5:
                        mark(key,center_match_error_m=error,
                             message_age_sec=t.to_sec()-msg.header.stamp.to_sec())
                        break
    result['controlled_stationary_case_valid']=bool(origin is not None and
        result['npc_pre_overlap_max_speed_mps']<.2 and result['npc_pre_overlap_max_displacement_m']<.5)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bags',type=Path,nargs='+')
    p.add_argument('--object-id',type=int,default=38)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    rospy.init_node('compare_detector_pair',anonymous=True,disable_signals=True)
    results=[analyze(b,a.object_id) for b in a.bags]
    a.output.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for result in results:
        trace=result.pop('_trace')
        label=result['experiment']['detector']+' / '+str(result['experiment'].get('speed_limit_kph',0))+' kph cap'
        if trace:
            axes[0].plot([p[0] for p in trace],[p[1] for p in trace],label=label)
            axes[1].plot([p[0] for p in trace],[p[2] for p in trace],label=label)
    axes[0].set_ylabel('Ego speed [km/h]')
    axes[1].set_ylabel('Ego x - NPC x [m]')
    axes[1].set_xlabel('Ego y - stationary NPC y [m]; approaching -> passing')
    axes[1].set_xlim(200,-40)
    for ax in axes:
        ax.axvline(0,color='black',linestyle=':'); ax.grid(True); ax.legend()
    fig.tight_layout(); fig.savefig(a.output/'approach.svg'); plt.close(fig)
    text=json.dumps(results,indent=2,ensure_ascii=False)
    (a.output/'comparison.json').write_text(text+'\n')
    (a.output/'comparison.html').write_text('<!doctype html><meta charset="utf-8">'
        '<h1>고정 장애물 검출 입력 A/B</h1><p>17회차 랜덤 동적 차량 사건의 동일 재현이 아님. '
        '동일 고정 상대차·출발점·플래너·트래커를 사용한 별도 통제 시험. '
        '거리는 Ego/NPC reference point 사이 거리이며 범퍼 간 제동거리와 다름. '
        'GT 주행에서도 BEV 검출을 shadow 기록하여 같은 ego 상태에서 검출 가용성을 대조함. '
        'approach_first는 고주로 전방200m 분석 창 진입 이후 첫 관측이며 전역 최초 검출 시각이 아님. '
        '객체 대응은 보정한 GT box center와 5m 이내. 박스 간격은 형상 모델 근사이며 CollisionData와 독립적으로 확인한다. '
        '단일 회차 쌍으로 일반적인 성공률이나 실차 안전성을 주장하지 않음.</p>'
        '<img src="approach.svg" style="max-width:100%"><details><summary>정량 결과</summary><pre>'+
        html.escape(text)+'</pre></details>')
    print(text)


if __name__=='__main__': main()
