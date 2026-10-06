"""Pure collision-window and crash-safe session helpers; no ROS imports."""
import hashlib
import json
import os
from pathlib import Path
import time
import math


def vertical_motion_review(actor,samples,window_start,reference_z=None,step_limit_m=.10,height_limit_m=.05,ground_support_verified=False):
    """Observed moving-window Z gate; absolute reference must be independently validated."""
    if not actor or actor['motion'] in ('static','standing'):
        return dict(required=False,review_reasons=[])
    trace=sorted([r for r in samples if r['uid']==actor['unique_id'] and r['t']>=window_start
                  and all(math.isfinite(r.get(k,float('nan'))) for k in ('t','z'))],key=lambda r:r['t'])
    steps=[abs(b['z']-a['z']) for a,b in zip(trace,trace[1:]) if 0<b['t']-a['t']<=.20]
    reasons=[]
    if not steps:reasons.append('motion_vertical_trace_unverified')
    elif max(steps)>step_limit_m:reasons.append('motion_vertical_jump')
    # A constant floating object has no Z jump: do not certify it from stability alone.
    if actor.get('source_list')=='objectList' and reference_z is None and not ground_support_verified:
        reasons.append('motion_vertical_ground_reference_unverified')
    error=max((abs(r['z']-reference_z) for r in trace),default=None) if reference_z is not None else None
    if error is not None and error>height_limit_m:reasons.append('motion_vertical_reference_mismatch')
    return dict(required=True,review_reasons=reasons,maximum_observed_z_step_m=max(steps) if steps else None,
        reference_z_m=reference_z,ground_support_verified=ground_support_verified,maximum_height_error_m=error,step_limit_m=step_limit_m,
        height_limit_m=height_limit_m,window_start_monotonic=window_start,
        limitation='Capture-window receive samples only; stable Z is not ground support. Scalar reference is for independently matched flat-ground probes, not arbitrary slopes or pivots.')


def motion_capture_review(actor, before, after, samples, requested_at):
    """Conservative endpoint/real-displacement witness, not animation or velocity GT."""
    if not actor or actor['motion'] in ('static','standing'):
        return dict(required=False,review_reasons=[])
    path=actor.get('motion_path') or {}
    end=(path.get('points') or [{}])[-1]
    uid=actor['unique_id']
    def pos(objects):
        return next((a['position'] for k in ('npc_list','pedestrian_list','obstacle_list')
            for a in (objects or {}).get(k,[]) if a['unique_id']==uid),None)
    a,b=pos(before),pos(after)
    remaining=lambda p:math.hypot(p['x']-end['x'],p['y']-end['y']) if p and 'x' in end else None
    ra,rb=remaining(a),remaining(b)
    reasons=[]
    if ra is None or rb is None:reasons.append('motion_capture_missing_endpoint_witness')
    elif min(ra,rb)<.10:reasons.append('motion_endpoint_reached_during_capture_window')
    trace=[r for r in samples if r['uid']==uid and requested_at-.30<=r['t']<=requested_at]
    displacement=math.hypot(trace[-1]['x']-trace[0]['x'],trace[-1]['y']-trace[0]['y']) if len(trace)>1 else None
    span=trace[-1]['t']-trace[0]['t'] if len(trace)>1 else 0.
    if span<.05 or displacement is None or displacement<.001:
        reasons.append('motion_not_observed_immediately_before_capture')
    return dict(required=True,review_reasons=reasons,endpoint_remaining_before_m=ra,
        endpoint_remaining_after_m=rb,pre_request_trace_span_sec=span,
        pre_request_displacement_m=displacement,observed_speed_mps=displacement/span if span else None,
        limitation='Receive-time finite difference and endpoint bounds; not exact sensor timestamp or animation certification')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def status_row(row):
    """Dashboard must not rewrite every full camera audit on each scene at 10K scale."""
    compact={k:v for k,v in row.items() if k!='audit'}
    audit=row.get('audit',{})
    compact['review_reasons']=audit.get('review_reasons',[])
    compact['collision_verified']=audit.get('capture_safety',{}).get('collision',{}).get('no_collision_verified')
    return compact


def runtime_presence_review(metadata, before, after, components):
    """AI actors may have sensor GT but no Object_topic row; never waive ordinary actors."""
    seen=[]
    for objects in (before,after):
        seen.append({a['unique_id'] for key in ('npc_list','pedestrian_list','obstacle_list')
            for a in (objects or {}).get(key,[])})
    missing=[];annotation_only=[]
    for actor in metadata['runtime_actors']:
        uid=actor['unique_id']
        if all(uid in ids for ids in seen):continue
        witness=any(c['parent_uid']==uid and c.get('positive_lidar1_camera0') for c in components)
        if actor['source_list']=='aiPedstrianList' and actor['motion'] in ('static','standing') and witness:
            annotation_only.append(uid)
        else:missing.append(uid)
    return sorted(missing),sorted(annotation_only)


def park_request_fields():
    """Exit changes gear only; do not reconfigure the already-confirmed control mode."""
    return dict(option=2,gear=1)


def confirm_park(request, attempts=2, retry_delay=.3):
    """Idempotent gear-P confirmation; caller must bound each request and keep braking."""
    errors=[];started=time.monotonic()
    for index in range(attempts):
        try:
            gear=request()
            if gear!=1:raise RuntimeError('Park response gear is '+str(gear))
            return dict(park_verified=True,gear=gear,attempts=index+1,errors=errors,
                elapsed_sec=time.monotonic()-started)
        except Exception as exc:errors.append(str(exc))
        if index+1<attempts:time.sleep(retry_delay)
    return dict(park_verified=False,gear=None,attempts=attempts,errors=errors,
        elapsed_sec=time.monotonic()-started)


def atomic_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    with temp.open('w') as f:
        json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    temp.replace(path)
    directory=os.open(str(path.parent),os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(directory)
    finally:os.close(directory)


def load_ledger(path):
    """Repair only a torn last append, backing up all original bytes first."""
    path=Path(path)
    if not path.exists():return []
    raw=path.read_bytes();lines=raw.splitlines(keepends=True);rows=[]
    for i,line in enumerate(lines):
        try:rows.append(json.loads(line))
        except (ValueError,UnicodeDecodeError):
            if i!=len(lines)-1:raise ValueError('Corrupt nonterminal ledger record')
            backup=path.with_name(path.name+'.torn.'+str(time.time_ns()))
            backup.write_bytes(raw)
            temp=path.with_suffix('.repair')
            with temp.open('wb') as f:
                f.write(b''.join(lines[:i]));f.flush();os.fsync(f.fileno())
            temp.replace(path)
    if path.stat().st_size and not path.read_bytes().endswith(b'\n'):
        with path.open('ab') as f:f.write(b'\n');f.flush();os.fsync(f.fileno())
    return rows


def prepare_session(output,manifest,network,sensor,runner_options=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    scripts=Path(__file__).parent
    contract=dict(schema_version=6,manifest_sha256=sha256(manifest),
        network_sha256=sha256(network),sensor_sha256=sha256(sensor),
        safety_code_sha256={name:sha256(scripts/name) for name in ('base5_capture_safety.py','base5_capture_audit.py','base5_ground_support.py','base5_ground_probe.py','run_base5_capture.py','base5_capture_timing.py')},
        safety_policy='collision-history-v1; missing-runtime-actors; AI-standing-visible-annotation-v1; local-ground-plane-v1; all-actor-motion-endpoint-window-v1; vertical-step-window-v1')
    if runner_options is not None:contract['runner_options']=runner_options
    path=output/'session_contract.json'
    if path.exists():
        if json.loads(path.read_text())!=contract:raise ValueError('Resume session contract mismatch')
    elif (output/'ledger.jsonl').exists():
        raise ValueError('Legacy ledger lacks safety contract; use a new output directory')
    else:atomic_json(path,contract)
    previous=load_ledger(output/'ledger.jsonl');latest={r['id']:r for r in previous}
    completed={k:r for k,r in latest.items() if r['status'] in ('accepted_candidate','review')}
    for ident,row in completed.items():
        if ((runner_options or {}).get('shared_ros_batch') or (runner_options or {}).get('batch_reuse_network')) and not row.get('between_scene_park',{}).get('park_verified'):
            raise ValueError('Batch completed row lacks safe boundary Park: '+ident)
        capture=Path(row['capture_root'])
        if any(not (capture/f).is_file() for f in ('scenario.json','metadata.json','audit.json','telemetry.json','collision_history.json')):
            raise ValueError('Completed ledger has incomplete provenance: '+ident)
        if sha256(capture/'scenario.json')!=row['scenario_sha256']:
            raise ValueError('Completed capture checksum mismatch: '+ident)
        cameras=list(capture.glob('CAMERA_*/*_Intensity.png'))
        lidar=list(capture.glob('LIDAR_*/*_Intensity.bin'))
        labels=list(capture.glob('LIDAR_*/*_Instance.txt'))
        if len(cameras)!=4 or len(lidar)!=1 or len(labels)!=1 or any(p.stat().st_size==0 for p in cameras+lidar):
            raise ValueError('Completed ledger has missing/empty sensor files: '+ident)
    return previous,completed


def archive_incomplete(output,ident):
    """Keep orphan/failed capture directories intact before a fresh attempt."""
    output=Path(output);capture=output/'captures'/ident
    if not capture.exists():return None
    dest=output/'incomplete_attempts'/(ident+'_'+str(time.time_ns()))
    dest.parent.mkdir(parents=True,exist_ok=True);capture.rename(dest)
    return str(dest)


def collision_window(samples,start,end,max_gap=2.):
    rows=[r for r in samples if start<=r['t']<=end]
    peers=sorted({int(a['unique_id']) for r in rows for key in ('collision_object','collision_objecta')
        for a in r['message'].get(key,[]) if int(a['unique_id'])!=0})
    times=sorted(r['t'] for r in rows)
    gaps=[b-a for a,b in zip([start]+times,times+[end])]
    maximum=max(gaps,default=end-start)
    observed=len(rows)>=2 and maximum<=max_gap
    return dict(observed=observed,no_collision_verified=observed and not peers,
        non_ego_uids=peers,messages=len(rows),maximum_gap_sec=maximum,
        window_start_monotonic=start,window_end_monotonic=end,
        contract='Ego UID0 alone is not a contact; both collision arrays inspected')
