#!/usr/bin/env python3
"""Bounded MR synchronous capture with brake, owned-file cleanup and resumable ledger."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import threading
import time
import uuid
import fcntl

from base5_capture_audit import audit_capture
from base5_capture_safety import prepare_session,archive_incomplete,collision_window,atomic_json,status_row,runtime_presence_review,confirm_park,park_request_fields,motion_capture_review

ROOT=Path(__file__).resolve().parents[3]


def message_dict(value):
    if hasattr(value,'__slots__'):
        return {slot:message_dict(getattr(value,slot)) for slot in value.__slots__}
    if isinstance(value,(list,tuple)): return [message_dict(v) for v in value]
    return value


def bounded_call(function, timeout=5.):
    done=threading.Event(); result=[]
    def invoke():
        try: result.append((True,function()))
        except Exception as exc: result.append((False,exc))
        finally: done.set()
    threading.Thread(target=invoke,daemon=True).start()
    if not done.wait(timeout): raise RuntimeError('MORAI EventCmd response timeout')
    if not result[0][0]: raise RuntimeError(str(result[0][1]))
    return result[0][1]


def actor_position(objects, uid):
    if not objects: return None
    for key in ('npc_list','pedestrian_list','obstacle_list'):
        for actor in objects.get(key,[]):
            if actor.get('unique_id')==uid:return actor['position']
    return None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scenario-dir',type=Path,default=Path.home()/'MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025')
    p.add_argument('--sensor-dir',type=Path,default=Path.home()/'MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/SensorData')
    p.add_argument('--limit',type=int,default=48);p.add_argument('--indices',help='Comma-separated manifest row indices')
    p.add_argument('--approve-bulk-capture',action='store_true',help='Explicit user authorization required; not used for pilots')
    p.add_argument('--prepare-only',action='store_true',help='Write selected plan/status without importing ROS or controlling MR')
    p.add_argument('--network-template',type=Path,default=ROOT/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json')
    p.add_argument('--stop-after-new-captures',type=int,help='Graceful resume smoke-test stop; does not change selected plan')
    p.add_argument('--test-interrupt-after-raw-move',help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.limit<1 or (args.limit>64 and not args.approve_bulk_capture): p.error('Pilot limited to 64; bulk requires explicit approval')
    if args.stop_after_new_captures is not None and args.stop_after_new_captures<1:p.error('Stop count must be positive')
    if args.test_interrupt_after_raw_move and args.limit>64:p.error('Interruption injection is pilot-only')
    manifest=json.loads(args.manifest.read_text())
    from dynamic_capture_metadata import normalized_sensor_calibration
    sensor_source=ROOT/'configs/sensors/Sensor_competition_local.json'
    calibration=normalized_sensor_calibration(sensor_source)
    rows=manifest['scenes']
    if args.indices: rows=[rows[int(i)] for i in args.indices.split(',')]
    rows=rows[:args.limit]
    args.output.mkdir(parents=True,exist_ok=True)
    process_lock=(args.output/'.runner.lock').open('a')
    try:fcntl.flock(process_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('Another runner owns this output directory')
    if args.prepare_only:
        if (args.output/'ledger.jsonl').exists():
            raise ValueError('Prepare-only refuses to replace an existing capture session')
        atomic_json(args.output/'selected_plan.json',dict(manifest=str(args.manifest.resolve()),scenes=rows))
        atomic_json(args.output/'status.json',dict(stage='prepared_waiting_for_MR_reconnect',
            current=None,error='Live pilot not validated; reconnect MR before capture.',updated_at=time.time(),
            requested=len(rows),finished=0,accepted_candidate=0,review=0,failed=0,eta_sec=0,results=[]))
        print(str(args.output/'status.json'))
        return
    session=uuid.uuid4().hex[:8]
    ledger=args.output/'ledger.jsonl'; stopfile=args.output/'STOP'
    previous,completed=prepare_session(args.output,args.manifest,args.network_template,sensor_source)
    if any(r.get('scenario_sha256')!=next((s['scenario_sha256'] for s in rows if s['id']==r['id']),r.get('scenario_sha256')) for r in previous):
        raise ValueError('Resume scenario provenance mismatch')
    if rows and all(r['id'] in completed for r in rows):
        # Completed-session resume is read-only with respect to MR, even offline.
        saved=json.loads((args.output/'status.json').read_text()) if (args.output/'status.json').exists() else {}
        finished=[completed[r['id']] for r in rows]
        exit_record=args.output/'shutdown_safety.json'
        unsafe_exit=exit_record.exists() and not json.loads(exit_record.read_text())['park_verified']
        saved.update(stage='halted_exit_safety' if unsafe_exit else 'complete',current=None,
            error='Previous exit Park not verified; raw complete, no live resume permitted' if unsafe_exit else None,
            updated_at=time.time(),requested=len(rows),finished=len(rows),
            accepted_candidate=sum(r['status']=='accepted_candidate' for r in finished),
            review=sum(r['status']=='review' for r in finished),failed=0,eta_sec=0,results=[status_row(r) for r in finished])
        atomic_json(args.output/'status.json',saved)
        return
    exit_record=args.output/'shutdown_safety.json'
    if exit_record.exists() and not json.loads(exit_record.read_text())['park_verified']:
        raise RuntimeError('Previous exit Park not verified; check MR and use a new output session')
    import rospy
    from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList,SaveSensorData,ScenarioLoad,CollisionData
    from morai_msgs.srv import MoraiEventCmdSrv,MoraiEventCmdSrvRequest
    rospy.init_node('base5_sync_capture',anonymous=True,disable_signals=True)
    def interrupted(signum,frame): raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    state={'ego':None,'objects':None,'ego_t':0.,'object_t':0.,'collision_t':0.}; traces=[];collisions=[]
    lock=threading.RLock(); halt=threading.Event();object_traces=[]
    def ego_cb(m):
        with lock:
            row=dict(t=time.monotonic(),x=m.position.x,y=m.position.y,z=m.position.z,heading=m.heading,vx=m.velocity.x,vy=m.velocity.y)
            state.update(ego=row,ego_t=row['t']);traces.append(row)
            if len(traces)>2000: del traces[:1000]
    def object_cb(m):
        with lock:
            now=time.monotonic();objects=message_dict(m)
            state.update(objects=objects,object_t=now)
            for key in ('npc_list','pedestrian_list','obstacle_list'):
                for actor in objects.get(key,[]):
                    object_traces.append(dict(t=now,uid=actor['unique_id'],**actor['position']))
            if len(object_traces)>10000:del object_traces[:5000]
    def collision_cb(m):
        with lock:
            now=time.monotonic();state['collision_t']=now
            collisions.append(dict(t=now,message=message_dict(m)))
            if len(collisions)>20000:del collisions[:10000]
    rospy.Subscriber('/Ego_topic',EgoVehicleStatus,ego_cb,queue_size=100)
    rospy.Subscriber('/Object_topic',ObjectStatusList,object_cb,queue_size=20)
    rospy.Subscriber('/CollisionData',CollisionData,collision_cb,queue_size=100)
    control=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
    loader=rospy.Publisher('/ScenarioLoad',ScenarioLoad,queue_size=1)
    capture=rospy.Publisher('/SaveSensorData',SaveSensorData,queue_size=1)
    def brake():
        while not halt.is_set() and not rospy.is_shutdown():
            control.publish(CtrlCmd(longlCmdType=1,brake=1.,accel=0.,steering=0.));time.sleep(.025)
    threading.Thread(target=brake,daemon=True).start()
    def fresh(gap=.8):
        with lock:
            if state['ego'] is None or any(time.monotonic()-state[k]>gap for k in ('ego_t','object_t','collision_t')):
                raise RuntimeError('Ego/Object/CollisionData connection stale')
            return dict(state['ego'])
    started=time.time(); results=list(previous); owned=[]; failures=0;new_captures=0
    def status(stage,current=None,error=None):
        selected={r['id'] for r in rows}
        latest={r['id']:r for r in results if r['id'] in selected}
        finished=list(latest.values())
        accepted=sum(r['status']=='accepted_candidate' for r in finished)
        reviewed=sum(r['status']=='review' for r in finished)
        elapsed=time.time()-started
        atomic_json(args.output/'status.json',dict(stage=stage,current=current,error=error,updated_at=time.time(),
            requested=len(rows),finished=len(finished),accepted_candidate=accepted,review=reviewed,
            failed=sum(r['status']=='failed' for r in finished),elapsed_sec=elapsed,
            eta_sec=elapsed/max(1,len(finished)-len(previous))*max(0,len(rows)-len(finished)),
            manifest=str(args.manifest.resolve()),ledger=str(ledger.resolve()),results=[status_row(r) for r in finished]))
    try:
        status('preflight');time.sleep(1.3);fresh()
        if any(not pub.get_num_connections() for pub in (control,loader,capture)):
            raise RuntimeError('Missing ScenarioLoad/SaveSensorData/ctrl subscriber; stop without loading')
        if not args.scenario_dir.is_dir() or not args.sensor_dir.is_dir(): raise ValueError('MR paths missing')
        for entry in rows:
            if stopfile.exists() or rospy.is_shutdown(): status('stopped');break
            if entry['id'] in completed: continue
            meta=json.loads(Path(entry['metadata']).read_text()); scene=json.loads(Path(entry['scenario']).read_text())
            meta['lidar_to_ego']=calibration['lidar']['lidar_to_ego']
            meta['sensor_source_sha256']=calibration['source_sha256']
            import hashlib
            if hashlib.sha256(Path(entry['scenario']).read_bytes()).hexdigest()!=entry['scenario_sha256']:
                raise ValueError('Scenario checksum changed')
            t0=time.time(); name='zz_b5_'+session+'_'+entry['id']; dest=args.output/'captures'/entry['id']
            archived=archive_incomplete(args.output,entry['id'])
            scenario=args.scenario_dir/(name+'.json'); sidecar=args.scenario_dir/'EgoNetwork'/(name+'_MN.json')
            if scenario.exists() or sidecar.exists(): raise FileExistsError('Runtime overwrite refused')
            sidecar.parent.mkdir(exist_ok=True)
            shutil.copy2(entry['scenario'],scenario);owned.append(scenario)
            shutil.copy2(args.network_template,sidecar);owned.append(sidecar)
            try:
                status('scene_load',entry['id']);load_time=time.monotonic()
                loader.publish(ScenarioLoad(file_name=name,load_network_connection_data=True,load_ego_vehicle_data=True,
                    load_surrounding_vehicle_data=True,load_pedestrian_data=True,load_obstacle_data=True,set_pause=False))
                expected=scene['egoVehicle']['initPosition']['pos'];deadline=time.monotonic()+12
                while time.monotonic()<deadline:
                    with lock: e=state['ego']
                    if e and e['t']>load_time+.5 and time.monotonic()-e['t']<.3 and math.hypot(e['x']-expected['x'],e['y']-expected['y'])<3.5: break
                    time.sleep(.05)
                else: raise RuntimeError('Scene load pose not confirmed')
                pose_confirmed=time.monotonic()
                rospy.wait_for_service('/Service_MoraiEventCmd',timeout=4.)
                req=MoraiEventCmdSrvRequest();req.request.option=3;req.request.ctrl_mode=3;req.request.gear=4
                # Network reload can replace rosbridge's advertised service after
                # Ego starts publishing. Retry the idempotent mode/gear request
                # once; never bypass response verification or wait indefinitely.
                time.sleep(.7)
                for attempt in range(2):
                    try:
                        response=bounded_call(lambda:rospy.ServiceProxy('/Service_MoraiEventCmd',MoraiEventCmdSrv)(req))
                        break
                    except RuntimeError:
                        if attempt:raise
                        fresh();time.sleep(1.)
                if response.response.ctrl_mode not in (3,4,5) or response.response.gear!=4:
                    raise RuntimeError('External control mode/Drive gear not confirmed')
                time.sleep(.2)
                e0=fresh()
                if meta.get('yaw_probe'):
                    corrected=e0['heading']+meta['yaw_probe'].get('ego_heading_correction_deg',-1.347)
                    error=(corrected-meta['ego']['yaw']+180.)%360.-180.
                    if abs(error)>.2:raise ValueError('Yaw probe loaded heading differs from requested yaw')
                primary_uid=meta['runtime_actors'][0]['unique_id'] if meta['runtime_actors'] else None
                with lock: movement_before=actor_position(state['objects'],primary_uid)
                # Capture as soon as real motion is established. A fixed 1.2s
                # delay systematically pushes close targets out of their bin.
                deadline=time.monotonic()+manifest['config']['capture']['dynamic_wait_sec']
                if entry['motion'] not in ('static','standing'):
                    while time.monotonic()<deadline:
                        with lock: moving=actor_position(state['objects'],primary_uid)
                        if moving and movement_before and math.hypot(moving['x']-movement_before['x'],moving['y']-movement_before['y'])>=.05:break
                        fresh();time.sleep(.025)
                else:time.sleep(.2)
                arrival_evidence=None
                if meta.get('motion_probe') and entry['motion'] not in ('static','standing'):
                    probe=meta['motion_probe'];target=probe['target_position']
                    status('waiting_motion_target',entry['id'])
                    deadline=time.monotonic()+probe['arrival_timeout_sec'];closest=float('inf')
                    while time.monotonic()<deadline:
                        with lock:current=actor_position(state['objects'],primary_uid)
                        if current:
                            distance=math.hypot(current['x']-target['x'],current['y']-target['y'])
                            closest=min(closest,distance)
                            if distance<=probe['target_tolerance_m']:
                                arrival_evidence=dict(target_error_m=distance,position=current,
                                    load_to_arrival_sec=time.monotonic()-load_time);break
                        fresh();time.sleep(.015)
                    else:
                        with lock:failed_objects=state['objects'];failed_trace=list(object_traces)
                        atomic_json(args.output/'failures'/(entry['id']+'_'+str(time.time_ns())+'.json'),
                            dict(reason='motion_probe_arrival_not_observed',metadata=meta,
                                closest_observed_target_error_m=closest if math.isfinite(closest) else None,objects=failed_objects,
                                object_position_samples=[r for r in failed_trace if r['t']>=load_time],
                                load_time_monotonic=load_time))
                        raise ValueError('Motion probe target arrival not observed; no capture')
                e1=fresh()
                if math.hypot(e1['x']-e0['x'],e1['y']-e0['y'])>manifest['config']['capture']['maximum_ego_drift_m']:
                    raise ValueError('Stationary Ego drift exceeded')
                with lock: objects_before=state['objects']
                if meta.get('yaw_probe'):
                    from base5_reference_frames import yaw_runtime_review
                    frozen_reasons=yaw_runtime_review(meta,objects_before)
                    if frozen_reasons:
                        atomic_json(args.output/'failures'/(entry['id']+'_'+str(time.time_ns())+'.json'),
                            dict(reason='yaw_probe_frozen_world_failed',review_reasons=frozen_reasons,
                                metadata=meta,objects_before=objects_before,ego_before=e1))
                        raise ValueError('Yaw probe NPC is not at the fixed world pose')
                movement_after=actor_position(objects_before,primary_uid)
                movement=math.hypot(movement_after['x']-movement_before['x'],movement_after['y']-movement_before['y']) if movement_before and movement_after else None
                if entry['motion'] not in ('static','standing') and (movement is None or movement<.05):
                    atomic_json(args.output/'failures'/(entry['id']+'_'+str(time.time_ns())+'.json'),
                        dict(reason='motion_not_established',metadata=meta,load_time_monotonic=load_time,
                            pose_confirmed_monotonic=pose_confirmed,observed_at_monotonic=time.monotonic(),
                            objects_before=objects_before,primary_before=movement_before,primary_after=movement_after,
                            displacement_m=movement,ego_before=e0,ego_after=e1))
                    raise ValueError('Requested animation/motion has no measured actor displacement')
                before={f for f in args.sensor_dir.rglob('*') if f.is_file()}
                dest.mkdir(parents=True,exist_ok=False)
                atomic_json(dest/'metadata.json',meta)
                shutil.copy2(entry['scenario'],dest/'scenario.json')
                shutil.copy2(args.network_template,dest/'network.json')
                shutil.copy2(sensor_source,dest/'sensor_reference.json')
                atomic_json(dest/'attempt.json',dict(stage='capture_requested',load_time=load_time,
                    archived_previous_attempt=archived,scenario_sha256=entry['scenario_sha256']))
                capture_requested_at=time.monotonic()
                status('capture',entry['id']);capture.publish(SaveSensorData(is_custom_file_name=False))
                deadline=time.monotonic()+20; new=set();signature=None; stable=time.monotonic()
                while time.monotonic()<deadline:
                    new={f for f in args.sensor_dir.rglob('*') if f.is_file() and f not in before}
                    sig=sorted((str(f),f.stat().st_size) for f in new)
                    if sig!=signature: signature=sig;stable=time.monotonic()
                    cams={f.parent.name for f in new if f.name.endswith('_Intensity.png')}
                    lidar=[f for f in new if f.name.endswith('_Intensity.bin')]
                    annotation=[f for f in new if f.parent.name.startswith('LIDAR') and f.name.endswith('_Instance.txt')]
                    if len(cams)==4 and len(lidar)==1 and len(annotation)==1 and lidar[0].stat().st_size and time.monotonic()-stable>.5: break
                    fresh(4.);time.sleep(.05)
                else: raise ValueError('Synchronous file completeness timeout')
                # Retain raw before quality decisions; never remove files from previous captures.
                for f in sorted(new):
                    target=dest/f.relative_to(args.sensor_dir);target.parent.mkdir(parents=True,exist_ok=True)
                    shutil.move(str(f),str(target))
                if args.test_interrupt_after_raw_move==entry['id']:
                    raise KeyboardInterrupt('Injected smoke-test interruption after preserving raw')
                deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    try: e2=fresh();break
                    except RuntimeError: time.sleep(.05)
                else: raise RuntimeError('Telemetry did not resume after capture')
                # Preserve provenance even if annotation auditing raises.
                atomic_json(dest/'metadata.json',meta)
                shutil.copy2(entry['scenario'],dest/'scenario.json')
                shutil.copy2(args.network_template,dest/'network.json')
                shutil.copy2(sensor_source,dest/'sensor_reference.json')
                with lock:
                    collision_samples=[r for r in collisions if r['t']>=load_time]
                    samples=[r for r in traces if r['t']>=pose_confirmed]
                    objects_after=state['objects']
                    actor_samples=[r for r in object_traces if r['t']>=load_time]
                safety=collision_window(collision_samples,pose_confirmed,time.monotonic())
                # Load-time contacts are retained conservatively even before pose confirmation.
                all_peers=sorted({int(a['unique_id']) for r in collision_samples
                    for key in ('collision_object','collision_objecta') for a in r['message'][key] if int(a['unique_id'])!=0})
                safety['non_ego_uids']=all_peers
                safety['no_collision_verified']=safety['observed'] and not all_peers
                zs=[r['z'] for r in samples]
                support=dict(stationary_support_observed=bool(zs) and max(zs)-min(zs)<.04,
                    z_span_m=max(zs)-min(zs) if zs else None,
                    limitation='stationary vertical stability proxy; not paved-road or mesh support certification')
                meta['capture_safety']=dict(collision=safety,support=support,evidence_path=str((dest/'collision_history.json').resolve()))
                atomic_json(dest/'collision_history.json',dict(summary=safety,samples=collision_samples,
                    load_time_monotonic=load_time,pose_confirmed_monotonic=pose_confirmed))
                audit=audit_capture(dest,meta)
                motion_evidence=motion_capture_review(meta['runtime_actors'][0] if meta['runtime_actors'] else None,
                    objects_before,objects_after,actor_samples,capture_requested_at)
                motion_actors=[dict(uid=a['unique_id'],asset=a['name'],**motion_capture_review(a,
                    objects_before,objects_after,actor_samples,capture_requested_at)) for a in meta['runtime_actors']]
                audit['motion_capture']=motion_evidence
                audit['motion_capture_actors']=motion_actors
                if meta.get('yaw_probe'):
                    audit['yaw_runtime_review']=yaw_runtime_review(meta,objects_before,objects_after)
                    audit['review_reasons'].extend(audit['yaw_runtime_review'])
                for evidence in motion_actors:audit['review_reasons'].extend(evidence['review_reasons'])
                runtime_missing,annotation_only=runtime_presence_review(meta,objects_before,objects_after,audit['components'])
                if runtime_missing:audit['review_reasons'].append('missing_runtime_actor')
                audit['runtime_missing_uids']=runtime_missing
                audit['runtime_annotation_only_uids']=annotation_only
                audit['capture_safety']=meta['capture_safety']
                audit['review_reasons']=sorted(set(audit['review_reasons']))
                audit['accepted_candidate']=not audit['review_reasons']
                drift=math.hypot(e2['x']-e1['x'],e2['y']-e1['y'])
                if drift>.08: audit['review_reasons'].append('capture_ego_drift');audit['accepted_candidate']=False
                with lock: objects_after=state['objects'];samples=[r for r in traces if r['t']>=load_time]
                atomic_json(dest/'telemetry.json',dict(ego_before=e1,ego_after=e2,objects_before=objects_before,
                    objects_after=objects_after,ego_samples=samples,capture_drift_m=drift,
                    primary_displacement_during_wait_m=movement,object_position_samples=actor_samples,
                    capture_requested_at_monotonic=capture_requested_at,motion_capture=motion_evidence,
                    motion_probe_arrival=arrival_evidence,
                    motion_capture_actors=motion_actors))
                atomic_json(dest/'metadata.json',meta);atomic_json(dest/'audit.json',audit)
                atomic_json(dest/'attempt.json',dict(stage='complete',scenario_sha256=entry['scenario_sha256'],archived_previous_attempt=archived))
                result=dict(id=entry['id'],scenario_sha256=entry['scenario_sha256'],family=entry['family'],
                    role=entry['role'],distance_bounds=entry['distance_bounds'],subfamily=entry['subfamily'],
                    status='accepted_candidate' if audit['accepted_candidate'] else 'review',
                    duration_sec=time.time()-t0,capture_root=str(dest.resolve()),audit=audit,
                    bytes=sum(f.stat().st_size for f in dest.rglob('*') if f.is_file()))
                if entry.get('source_cell_id'):result['source_cell_id']=entry['source_cell_id']
                failures=0
            except (ValueError,RuntimeError) as exc:
                result=dict(id=entry['id'],scenario_sha256=entry['scenario_sha256'],family=entry['family'],
                    status='failed',error=str(exc),duration_sec=time.time()-t0)
                failures+=1
                if isinstance(exc,RuntimeError): failures=3 # communication failures halt immediately
            finally:
                for f in (scenario,sidecar):
                    if f in owned and f.exists(): f.unlink()
            results.append(result)
            with ledger.open('a') as f: f.write(json.dumps(result,ensure_ascii=False)+'\n');f.flush();os.fsync(f.fileno())
            status('running',entry['id']); print(json.dumps({k:v for k,v in result.items() if k!='audit'}),flush=True)
            new_captures+=1
            if args.stop_after_new_captures and new_captures>=args.stop_after_new_captures:
                status('stopped_for_resume_test',entry['id']);break
            if failures>=3: raise RuntimeError('Capture halted after connection failure or three consecutive scene failures')
        else: status('complete')
    except BaseException as exc:
        status('interrupted' if isinstance(exc,KeyboardInterrupt) else 'halted',error=str(exc));raise
    finally:
        active_exception=sys.exc_info()[0] is not None
        # Keep the 40Hz brake alive throughout both bounded, idempotent Park
        # attempts. A single publication followed by a short timeout is not
        # sufficient exit-state evidence.
        try:
            def park_request():
                fresh(1.)
                if not control.get_num_connections():raise RuntimeError('Exit control subscriber missing')
                control.publish(CtrlCmd(longlCmdType=1,brake=1.,accel=0.,steering=0.))
                # Exit changes only the gear; control mode was confirmed after load.
                park=MoraiEventCmdSrvRequest()
                for key,value in park_request_fields().items():setattr(park.request,key,value)
                response=bounded_call(lambda:rospy.ServiceProxy('/Service_MoraiEventCmd',MoraiEventCmdSrv)(park),timeout=3.)
                return response.response.gear
            shutdown=confirm_park(park_request)
            shutdown['request_fields']=park_request_fields()
            shutdown['brake_held_during_requests']=True
            atomic_json(args.output/'shutdown_safety.json',shutdown)
            if not shutdown['park_verified']:
                status('halted_exit_safety',error='Park not confirmed: '+'; '.join(shutdown['errors']))
                rospy.logerr('Capture exit Park NOT confirmed; stop further collection')
                if not active_exception:raise RuntimeError('Capture exit Park not confirmed')
        finally:
            halt.set()
            for f in owned:
                if f.exists(): f.unlink()


if __name__=='__main__': main()
