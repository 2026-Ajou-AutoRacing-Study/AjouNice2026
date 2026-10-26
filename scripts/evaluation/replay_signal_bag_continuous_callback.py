#!/usr/bin/env python3
"""Continuous current traffic callbacks on a closed bag; no ROS publishers.

Extract runs the real ROI callback and route matcher against recorded TF/poses.
Infer runs the real inference callback with fixed v5 YOLO/CNN and fake transports.
Baseline is a current-model ablation, NOT a claim to reconstruct the old runtime.
"""
import argparse
from collections import Counter,deque,defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import types
from unittest.mock import Mock,patch
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=ROOT/'base_ws/src/perception/traffic_light_detection'
sys.path.insert(0,str(PACKAGE/'scripts'))
WINDOWS=(('A',70.,77.),('C',100.,109.),('D',130.,142.))
GROUPS=dict(A=201351,B=201365,C=201290,D=201326)
MODES=('ablation_no_near_no_soft','current_without_visual','current')


def digest(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()


def normalize_windows(windows):
    if not isinstance(windows,(list,tuple)) or not windows:
        raise ValueError('Windows must be a nonempty JSON list of [mission, start_sec, end_sec]')
    result=[]
    for row in windows:
        if not isinstance(row,(list,tuple)) or len(row)!=3:
            raise ValueError('Each window must be [mission, start_sec, end_sec]')
        name,start,end=row
        if not isinstance(name,str) or name not in GROUPS or any(isinstance(t,bool) or not isinstance(t,(int,float))
                or not math.isfinite(t) for t in (start,end)) or not 0<=start<end:
            raise ValueError('Windows require A/B/C/D and finite 0 <= start < end seconds')
        result.append((name,float(start),float(end)))
    result.sort(key=lambda row:row[1])
    if any(right[1]<=left[2] for left,right in zip(result,result[1:])):
        raise ValueError('Mission windows must not overlap or share an inclusive endpoint')
    return tuple(result)


def file_identity(path):
    stat=Path(path).stat()
    return dict(size=stat.st_size,mtime_ns=stat.st_mtime_ns)


def require_unchanged(path,identity):
    if file_identity(path)!=identity:raise RuntimeError('Replay input changed: '+str(path))


def replay_configuration(map_file=None,windows_json=None):
    explicit_map=map_file is not None
    path=Path(map_file or ROOT/'kcity_map/lanelet_map/lanelet2_map_v87.osm').resolve(strict=True)
    before=file_identity(path);version=None
    for event,element in ET.iterparse(path,events=('start','end')):
        if event=='start' and element.tag=='MetaInfo':
            version=element.get('map_version');break
        if event=='end':element.clear()
    if not version:raise ValueError('Explicit map must have an embedded MetaInfo map_version')
    if version=='94' and (not explicit_map or windows_json is None):
        raise ValueError('v94 replay requires explicit --map-file and --windows-json; old times cannot be reused silently')
    windows=WINDOWS;window_source=None
    if windows_json is not None:
        windows_path=Path(windows_json).resolve(strict=True)
        window_before=file_identity(windows_path);payload=windows_path.read_bytes()
        windows=normalize_windows(json.loads(payload))
        require_unchanged(windows_path,window_before)
        window_source=dict(file=str(windows_path),sha256=hashlib.sha256(payload).hexdigest(),
                           identity=window_before)
    map_source=dict(file=str(path),sha256=digest(path),embedded_map_version=version,identity=before)
    require_unchanged(path,before)
    return windows,map_source,window_source


def section(t,windows=WINDOWS):
    return next((k for k,a,b in windows if a<=t<=b),None)


def extract(bag_path,output,yaw_cap=3.,visual_sec=0.,map_file=None,windows_json=None):
    windows,map_source,window_source=replay_configuration(map_file,windows_json)
    bag_before=file_identity(bag_path)
    source_paths=(Path(__file__),PACKAGE/'scripts/traffic_light_map_roi_node.py',
        PACKAGE/'scripts/lanelet_route_matcher.py',PACKAGE/'config/traffic_signal_judging_lines.yaml',
        PACKAGE/'scripts/traffic_light_map_selector.py',PACKAGE/'scripts/traffic_light_visual_continuation.py',
        PACKAGE/'scripts/traffic_light_route_group_id_node.py',PACKAGE/'scripts/signal_judging_line_geometry.py',
        PACKAGE/'config/traffic_light_map_geometry.json',ROOT/'configs/sensors/Sensor_competition_local.json',
        ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    source_hashes={str(path):digest(path) for path in source_paths}
    import rosbag,rospy,tf2_py,yaml
    from traffic_light_map_selector import load_camera,load_geometry,DEFAULT_SELECTOR_CONFIG
    from traffic_light_map_roi_node import TrafficLightMapRoiNode
    from lanelet_route_matcher import LaneletRouteGroupIdMatcher
    from traffic_light_route_group_id_node import restrict_route_groups,MISSION_GROUP_IDS
    from dataclasses import replace
    if not 3.<=yaw_cap<=5.:raise ValueError('Audit yaw cap must be within 3..5 degrees')
    output=output.resolve();output.relative_to(ROOT/'planner_logs/analysis')
    if output.exists():raise FileExistsError('Fresh extraction output required')
    buffer=tf2_py.BufferCore(rospy.Duration(1000.))
    with rosbag.Bag(str(bag_path)) as bag:
        start=bag.get_start_time()
        for topic,msg,t in bag.read_messages(topics=['/tf','/tf_static']):
            for transform in msg.transforms:
                (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(transform,'recorded_bag')
        cfg=yaml.safe_load((PACKAGE/'config/traffic_signal_judging_lines.yaml').read_text())
        matcher=LaneletRouteGroupIdMatcher(map_source['file'],
            'LocalCartesianUTM',str(ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml'),
            excluded_group_ids={201395,201396},route_progress_guard=True,
            signal_judging_lines=cfg['signal_judging_lines'])
        if not matcher.available:raise RuntimeError('System Lanelet2 unavailable')
        matcher.lanelet_group_id=restrict_route_groups(matcher.lanelet_group_id,sorted(MISSION_GROUP_IDS))
        helper=object.__new__(TrafficLightMapRoiNode)
        helper.geometry=load_geometry(PACKAGE/'config/traffic_light_map_geometry.json')
        helper.camera=load_camera(ROOT/'configs/sensors/Sensor_competition_local.json')
        helper.map_frame,helper.base_frame='map','base_link'
        helper.route_stale,helper.pose_delta,helper.pose_age=2.,.15,.5
        helper.visual_continuation_sec=visual_sec
        helper.selector_config=replace(DEFAULT_SELECTOR_CONFIG,max_yaw_sigma_degrees=yaw_cap)
        helper.max_position_sigma=helper.selector_config.max_position_sigma_m
        helper.max_vertical_sigma=helper.selector_config.max_vertical_sigma_m
        helper.max_yaw_sigma=helper.selector_config.max_yaw_sigma_degrees
        helper.projection_options=helper.selector_config.projection_kwargs()
        helper._lock=threading.Lock();helper._poses=deque(maxlen=100)
        helper._group_id,helper._group_receipt=0,rospy.Time(0)
        helper.buffer=types.SimpleNamespace(lookup_transform=lambda target,source,stamp,timeout:
            buffer.lookup_transform_core(target,source,stamp))
        helper.pub=Mock();rows=[];current_group=0;last_group_clock=-float('inf')
        output.mkdir(parents=True);(output/'images').mkdir()
        topics=['/localization/kinematic_state','/planning/route_state','/planning/mission_planning/route',
                '/sensor/camera/front/compressed']
        for topic,msg,t in bag.read_messages(topics=topics):
            elapsed=t.to_sec()-start
            with patch('rospy.Time.now',return_value=t):
                if topic==topics[0]:matcher._odom_callback(msg);helper.pose_callback(msg)
                elif topic==topics[1]:matcher._route_state_callback(msg)
                elif topic==topics[2]:matcher._route_callback(msg)
                if t.to_sec()-last_group_clock>=.1:
                    current_group=matcher.get_group_id() or 0
                    helper.route_callback(types.SimpleNamespace(data=current_group))
                    last_group_clock=t.to_sec()
                name=section(elapsed,windows)
                if topic!=topics[3] or name is None:continue
                helper.image_callback(msg);packet=json.loads(helper.pub.publish.call_args[0][0].data)
                relative='images/frame_%04d_%s.jpg'%(len(rows),name);payload=bytes(msg.data)
                (output/relative).write_bytes(payload)
                rows.append(dict(t=elapsed,section=name,group_id=current_group,
                    source_stamp_ns=msg.header.stamp.to_nsec(),receipt_stamp_ns=t.to_nsec(),
                    frame_id=msg.header.frame_id,image_path=relative,
                    image_sha256=hashlib.sha256(payload).hexdigest(),map_packet=packet))
    require_unchanged(bag_path,bag_before)
    require_unchanged(map_source['file'],map_source['identity'])
    if window_source:require_unchanged(window_source['file'],window_source['identity'])
    if any(digest(path)!=value for path,value in source_hashes.items()):
        raise RuntimeError('Replay source/configuration changed during extraction')
    report=dict(schema='continuous_signal_callback_inputs_v1',bag=str(bag_path),windows=windows,
        bag_identity=bag_before,map=map_source,windows_source=window_source,
        replay_role='current_source_counterfactual_not_recorded_runtime',
        raw_frames=len(rows),yaw_cap_degrees=yaw_cap,visual_continuation_sec=visual_sec,
        sources=source_hashes,
        records=rows,limitations=['Current source/geometry/model replay, not recorded traffic output.',
            'TF preloaded for exact source queries: live TF arrival timing and wait failures are not recreated.',
            'ROIs generated by actual callback at recorded image receipt; actual GPU callback receipt latency is unknown.',
            'Ordered-route helper timer approximated by >=100ms bag event updates; no live ROS nodes.',
            'Baseline ablation uses same current v5 weights, not unrecorded original version.'])
    (output/'inputs.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),raw_frames=len(rows),route_counts=dict(Counter(r['group_id'] for r in rows)))))


class CachedDetector:
    def __init__(self,model):self.wrapper=model;self.model=model.model;self.key=None;self.cache={}
    def predict(self,**kw):
        key=(self.key,kw['imgsz'])
        if key not in self.cache:self.cache[key]=self.wrapper.predict(**kw)
        return self.cache[key]


def summarize(rows,windows=WINDOWS):
    result={}
    for mode in MODES:
        output={}
        for name in dict.fromkeys(row[0] for row in normalize_windows(windows)):
            all_samples=[r for r in rows if r['section']==name and mode in r['outcomes']]
            samples=[r for r in all_samples if r['group_id']==GROUPS[name]]
            counts=Counter(window_frames=len(all_samples),outside_active_mission=len(all_samples)-len(samples))
            runs=[];active=None;previous_window=None;previous_row=None
            for row in samples:
                current_window=next((i for i,(label,start,end) in enumerate(windows)
                                     if label==name and start<=row['t']<=end),None)
                if current_window is None:raise ValueError('Replay record lies outside configured mission windows')
                if active is not None and current_window!=previous_window:
                    active.update(end=previous_row['t'],span_sec=previous_row['t']-active['start'],right_censored=True)
                    active['reasons']=dict(active['reasons']);runs.append(active);active=None
                d=row['outcomes'][mode];lamps=d['decision']['final_tuples']
                known=bool(lamps)
                counts['frames']+=1;counts['known']+=known;counts['UNKNOWN']+=not known
                counts['near_retry']+=d['decision']['near_rescue'].get('attempted',False)
                counts['near_recovered']+=d['decision']['near_rescue'].get('accepted',False)
                counts['soft_held']+=d['decision']['observation_timing'].get('output_origin')=='low_confidence_hold'
                counts['visual_current_crop']+=d['decision'].get('visual_continuation',{}).get('status')=='matched'
                if not known and active is None:active=dict(start=row['t'],reasons=Counter())
                if not known:active['reasons'][d['classifier_reason']+'|'+d['selection_reason']]+=1
                elif active is not None:
                    active.update(end=row['t'],span_sec=row['t']-active['start']);active['reasons']=dict(active['reasons']);runs.append(active);active=None
                previous_window,previous_row=current_window,row
            if active is not None:
                active.update(end=samples[-1]['t'],span_sec=samples[-1]['t']-active['start'],right_censored=True)
                active['reasons']=dict(active['reasons']);runs.append(active)
            output[name]=dict(counts=counts,unknown_runs=runs)
        result[mode]=output
    return result


def infer(inputs,output,requested_device='auto',visual_sec=0.,cpu_threads=4):
    if not isinstance(cpu_threads,int) or not 1<=cpu_threads<=8:
        raise ValueError('Inference CPU threads must be within 1..8')
    import cv2,torch
    from train_traffic_light_yolo import offline_runtime
    from ultralytics import YOLO
    sys.path.insert(0,str(PACKAGE/'test'))
    import test_traffic_light_candidate_node as fixture
    from signal_phase_policy import SignalPhasePolicy
    from signal_smoother import SignalSmoother
    from traffic_light_near_rescue import NearRescueConfig
    from traffic_light_cnn_classifier import TrafficLightCnnClassifier
    source_identity=file_identity(inputs);source_payload=inputs.read_bytes()
    source=json.loads(source_payload);source_digest=hashlib.sha256(source_payload).hexdigest()
    windows=normalize_windows(source.get('windows',WINDOWS))
    output=output.resolve();output.relative_to(ROOT/'planner_logs/analysis')
    if output.exists():raise FileExistsError('Fresh inference output required')
    output.mkdir(parents=True);cv2.setNumThreads(1);torch.set_num_threads(cpu_threads)
    device=('cuda:0' if torch.cuda.is_available() else 'cpu') if requested_device=='auto' else requested_device
    if device.startswith('cuda') and not torch.cuda.is_available():raise RuntimeError('Requested CUDA unavailable')
    models=Path.home()/'.local/share/ajounice/traffic_light_models/v5_3d29ee7f'
    yolo_path=models/'yolo_final/selected.pt';cnn_path=models/'cnn_v5/best.pt'
    model_hashes={str(path):digest(path) for path in (yolo_path,cnn_path)}
    source_hashes={str(path):digest(path) for path in (Path(__file__),PACKAGE/'scripts/traffic_light_node.py',
        PACKAGE/'scripts/signal_phase_policy.py',PACKAGE/'scripts/traffic_light_observation_handoff.py',
        PACKAGE/'scripts/traffic_light_visual_continuation.py',PACKAGE/'config/traffic_signal_profiles.yaml',
        PACKAGE/'scripts/traffic_light_near_rescue.py',PACKAGE/'scripts/traffic_light_cnn_classifier.py',
        PACKAGE/'scripts/signal_smoother.py',PACKAGE/'test/test_traffic_light_candidate_node.py')}
    module=fixture.load_node();case=fixture.CandidateNodeTest();case.module=module
    with offline_runtime():
        model=CachedDetector(YOLO(str(yolo_path)))
        classifier=TrafficLightCnnClassifier(cnn_path,device=device,min_bit_confidence=.6,expected_sha256=digest(cnn_path))
        nodes={}
        for mode in MODES:
            node=case.node(classifier='cnn');node.model=model;node.inference_device=0 if device!='cpu' else 'cpu'
            node.detector_imgsz=960;node._min_period=fixture.Duration(.1)
            node.planner_compliance_observations=True
            node.near_rescue_config=NearRescueConfig(enabled=mode!='ablation_no_near_no_soft')
            node.low_confidence_hold_sec=.3 if mode!='ablation_no_near_no_soft' else 0.
            node.visual_continuation_sec=visual_sec if mode=='current' else 0.
            node.signal_phase_policy=SignalPhasePolicy.from_yaml(str(PACKAGE/'config/traffic_signal_profiles.yaml'))
            node.signal_smoother=SignalSmoother(7,4)
            node._cnn_startup=types.SimpleNamespace(ready=True,classifier=classifier)
            nodes[mode]=node
        results=[]
        for index,row in enumerate(source['records']):
            path=inputs.parent/row['image_path'];payload=path.read_bytes()
            if hashlib.sha256(payload).hexdigest()!=row['image_sha256']:raise ValueError('JPEG changed')
            sec,nsec=divmod(row['source_stamp_ns'],1_000_000_000)
            msg=types.SimpleNamespace(data=payload,header=types.SimpleNamespace(
                stamp=fixture.Stamp(sec,nsec),frame_id=row['frame_id']))
            sec,nsec=divmod(row['receipt_stamp_ns'],1_000_000_000);clock=fixture.Stamp(sec,nsec)
            model.key=row['source_stamp_ns'];model.cache={}
            outcomes={}
            for mode,node in nodes.items():
                node._resolve_group_id=lambda gid=row['group_id']:gid
                node._map_packet_for_image.return_value=row['map_packet']
                before=node.pub.publish.call_count
                with patch.object(module.rospy.Time,'now',return_value=clock):node.image_callback(msg)
                if node.pub.publish.call_count==before:continue
                decision=case.debug(node,'runtime_debug_pub')['frame_decision']
                classifier_debug=case.debug(node,'classifier_debug_pub')
                selection=case.debug(node,'selection_debug_pub')
                wire=node.pub.publish.call_args[0][0]
                outcomes[mode]=dict(decision=decision,classifier_reason=classifier_debug['reason'],
                    classifier=classifier_debug,selection_reason=selection['reason'],selection=selection,
                    output_source_stamp_ns=wire.stamp.to_nsec())
            saved_predictions={str(size):[[*map(float,box.xyxy[0].tolist()),float(box.conf[0])]
                for box in result[0].boxes] for (stamp,size),result in model.cache.items()}
            if outcomes:results.append(dict(t=row['t'],section=row['section'],group_id=row['group_id'],
                source_stamp_ns=row['source_stamp_ns'],receipt_stamp_ns=row['receipt_stamp_ns'],
                map_status=row['map_packet']['status'],map_reason=row['map_packet']['reason'],outcomes=outcomes,
                detector_predictions=saved_predictions))
            if (index+1)%50==0:print('processed inputs',index+1,'/',len(source['records']),flush=True)
    require_unchanged(inputs,source_identity)
    if any(digest(path)!=value for path,value in {**source_hashes,**model_hashes}.items()):
        raise RuntimeError('Replay source/model changed during inference')
    report=dict(schema='continuous_real_callback_candidate_replay_v1',source_inputs=str(inputs),
        source_inputs_sha256=source_digest,device=device,visual_continuation_sec=visual_sec,cpu_threads=cpu_threads,
        windows=windows,map=source.get('map'),bag_identity=source.get('bag_identity'),
        replay_role='current_source_counterfactual_not_recorded_runtime',
        models=model_hashes,sources=source_hashes,
        summary=summarize(results,windows),records=results,limitations=source['limitations']+[
            'Actual inference callback and source-preserving policy run with fake ROS transports, no live closed-loop control.',
            'Original receipt clock governs 10Hz gating; CPU compute time is not injected as GPU latency.',
            'No recorded traffic outputs/native GT: baseline-current differences are component ablation, not historical accuracy.'])
    (output/'replay.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),device=device,summary=report['summary'])))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['extract','infer'])
    p.add_argument('--bag',type=Path);p.add_argument('--inputs',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--yaw-cap',type=float,default=3.)
    p.add_argument('--device',choices=['auto','cpu','cuda:0'],default='auto')
    p.add_argument('--visual-sec',type=float,default=0.)
    p.add_argument('--cpu-threads',type=int,default=4,help='infer: torch intra-op threads (1..8); default keeps old replay setting')
    p.add_argument('--map-file',type=Path,help='extract: explicit OSM; omitted keeps the old v87 replay fixture')
    p.add_argument('--windows-json',type=Path,help='extract: JSON [["A",start,end],...] in bag-relative seconds; required for v94')
    a=p.parse_args()
    if a.stage=='extract':extract(a.bag.resolve(strict=True),a.output,a.yaw_cap,a.visual_sec,a.map_file,a.windows_json)
    else:infer(a.inputs.resolve(strict=True),a.output,a.device,a.visual_sec,a.cpu_threads)
