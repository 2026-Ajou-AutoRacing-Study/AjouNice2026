#!/usr/bin/env python3
"""Bounded MR synchronous capture with brake, owned-file cleanup and resumable ledger."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import threading
import time
import uuid

from base5_capture_audit import audit_capture

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


def atomic_json(path, data):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(path)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scenario-dir',type=Path,default=Path.home()/'MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025')
    p.add_argument('--sensor-dir',type=Path,default=Path.home()/'MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/SensorData')
    p.add_argument('--limit',type=int,default=48);p.add_argument('--indices',help='Comma-separated manifest row indices')
    p.add_argument('--approve-bulk-capture',action='store_true',help='Explicit user authorization required; not used for pilots')
    p.add_argument('--prepare-only',action='store_true',help='Write selected plan/status without importing ROS or controlling MR')
    args=p.parse_args()
    if args.limit<1 or (args.limit>64 and not args.approve_bulk_capture): p.error('Pilot limited to 64; bulk requires explicit approval')
    manifest=json.loads(args.manifest.read_text())
    from dynamic_capture_metadata import normalized_sensor_calibration
    sensor_source=ROOT/'configs/sensors/Sensor_competition_local.json'
    calibration=normalized_sensor_calibration(sensor_source)
    rows=manifest['scenes']
    if args.indices: rows=[rows[int(i)] for i in args.indices.split(',')]
    rows=rows[:args.limit]
    args.output.mkdir(parents=True,exist_ok=True)
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
    previous=[json.loads(s) for s in ledger.read_text().splitlines()] if ledger.exists() else []
    completed={r['id']:r for r in previous if r['status'] in ('accepted_candidate','review')}
    if any(r.get('scenario_sha256')!=next((s['scenario_sha256'] for s in rows if s['id']==r['id']),r.get('scenario_sha256')) for r in previous):
        raise ValueError('Resume scenario provenance mismatch')
    import rospy
    from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList,SaveSensorData,ScenarioLoad
    from morai_msgs.srv import MoraiEventCmdSrv,MoraiEventCmdSrvRequest
    rospy.init_node('base5_sync_capture',anonymous=True,disable_signals=True)
    def interrupted(signum,frame): raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    state={'ego':None,'objects':None,'ego_t':0.,'object_t':0.}; traces=[]
    lock=threading.RLock(); halt=threading.Event()
    def ego_cb(m):
        with lock:
            row=dict(t=time.monotonic(),x=m.position.x,y=m.position.y,z=m.position.z,heading=m.heading,vx=m.velocity.x,vy=m.velocity.y)
            state.update(ego=row,ego_t=row['t']);traces.append(row)
            if len(traces)>2000: del traces[:1000]
    def object_cb(m):
        with lock: state.update(objects=message_dict(m),object_t=time.monotonic())
    rospy.Subscriber('/Ego_topic',EgoVehicleStatus,ego_cb,queue_size=100)
    rospy.Subscriber('/Object_topic',ObjectStatusList,object_cb,queue_size=20)
    control=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
    loader=rospy.Publisher('/ScenarioLoad',ScenarioLoad,queue_size=1)
    capture=rospy.Publisher('/SaveSensorData',SaveSensorData,queue_size=1)
    def brake():
        while not halt.is_set() and not rospy.is_shutdown():
            control.publish(CtrlCmd(longlCmdType=1,brake=1.,accel=0.,steering=0.));time.sleep(.025)
    threading.Thread(target=brake,daemon=True).start()
    def fresh(gap=.8):
        with lock:
            if state['ego'] is None or time.monotonic()-state['ego_t']>gap or time.monotonic()-state['object_t']>gap:
                raise RuntimeError('Ego/Object connection stale')
            return dict(state['ego'])
    started=time.time(); results=list(previous); owned=[]; failures=0
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
            manifest=str(args.manifest.resolve()),ledger=str(ledger.resolve()),results=finished))
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
            if dest.exists(): raise FileExistsError('Capture overwrite refused '+str(dest))
            scenario=args.scenario_dir/(name+'.json'); sidecar=args.scenario_dir/'EgoNetwork'/(name+'_MN.json')
            if scenario.exists() or sidecar.exists(): raise FileExistsError('Runtime overwrite refused')
            sidecar.parent.mkdir(exist_ok=True)
            shutil.copy2(entry['scenario'],scenario);owned.append(scenario)
            shutil.copy2(ROOT/'configs/network/ego_network_mr_base5_savesensordata_local.json',sidecar);owned.append(sidecar)
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
                e1=fresh()
                if math.hypot(e1['x']-e0['x'],e1['y']-e0['y'])>manifest['config']['capture']['maximum_ego_drift_m']:
                    raise ValueError('Stationary Ego drift exceeded')
                with lock: objects_before=state['objects']
                movement_after=actor_position(objects_before,primary_uid)
                movement=math.hypot(movement_after['x']-movement_before['x'],movement_after['y']-movement_before['y']) if movement_before and movement_after else None
                if entry['motion'] not in ('static','standing') and (movement is None or movement<.05):
                    raise ValueError('Requested animation/motion has no measured actor displacement')
                before={f for f in args.sensor_dir.rglob('*') if f.is_file()}
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
                deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    try: e2=fresh();break
                    except RuntimeError: time.sleep(.05)
                else: raise RuntimeError('Telemetry did not resume after capture')
                # Preserve provenance even if annotation auditing raises.
                atomic_json(dest/'metadata.json',meta)
                shutil.copy2(entry['scenario'],dest/'scenario.json')
                shutil.copy2(ROOT/'configs/network/ego_network_mr_base5_savesensordata_local.json',dest/'network.json')
                shutil.copy2(sensor_source,dest/'sensor_reference.json')
                audit=audit_capture(dest,meta)
                drift=math.hypot(e2['x']-e1['x'],e2['y']-e1['y'])
                if drift>.08: audit['review_reasons'].append('capture_ego_drift');audit['accepted_candidate']=False
                with lock: objects_after=state['objects'];samples=[r for r in traces if r['t']>=load_time]
                atomic_json(dest/'telemetry.json',dict(ego_before=e1,ego_after=e2,objects_before=objects_before,
                    objects_after=objects_after,ego_samples=samples,capture_drift_m=drift,
                    primary_displacement_during_wait_m=movement))
                atomic_json(dest/'metadata.json',meta);atomic_json(dest/'audit.json',audit)
                result=dict(id=entry['id'],scenario_sha256=entry['scenario_sha256'],family=entry['family'],
                    role=entry['role'],distance_bounds=entry['distance_bounds'],subfamily=entry['subfamily'],
                    status='accepted_candidate' if audit['accepted_candidate'] else 'review',
                    duration_sec=time.time()-t0,capture_root=str(dest.resolve()),audit=audit,
                    bytes=sum(f.stat().st_size for f in dest.rglob('*') if f.is_file()))
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
            if failures>=3: raise RuntimeError('Capture halted after connection failure or three consecutive scene failures')
        else: status('complete')
    except BaseException as exc:
        status('interrupted' if isinstance(exc,KeyboardInterrupt) else 'halted',error=str(exc));raise
    finally:
        halt.set()
        for f in owned:
            if f.exists(): f.unlink()


if __name__=='__main__': main()
