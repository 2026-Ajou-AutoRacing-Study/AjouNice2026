"""Pure collision-window and crash-safe session helpers; no ROS imports."""
import hashlib
import json
import os
from pathlib import Path
import time
import math
import uuid


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


def verified_shutdown(path):
    """Missing, malformed or merely truthy Park evidence is not a safe exit."""
    try:
        record=json.loads(Path(path).read_text())
        return isinstance(record,dict) and record.get('park_verified') is True
    except (OSError,ValueError,TypeError):
        return False


class SceneCollisionError(RuntimeError):
    """Confirmed per-scene contact, not a transport or exit-safety failure."""


class ScenePoseUnverifiedError(RuntimeError):
    """Keep raw mismatch evidence without weakening the collision epoch gate."""
    def __init__(self, evidence):
        super().__init__('Collision stream new-scene Ego pose unverified; no capture')
        self.evidence = evidence


def write_scene_pose_failure(output, ident, scenario_hash, evidence, ego_history):
    """Append immutable evidence even if the same failed cell is attempted again."""
    path=Path(output)/'failures'/(ident+'_collision_pose_'+uuid.uuid4().hex+'.json')
    atomic_json(path,dict(evidence,scenario_sha256=scenario_hash,ego_load_samples=ego_history))
    return dict(failure_kind='collision_scene_pose_unverified',
        failure_evidence=str(path.resolve()),failure_evidence_sha256=sha256(path))


def is_stationary_failure(row):
    return row.get('failure_kind')=='ego_stationary_support' and row.get('status')=='failed'


def validate_stationary_failure(row):
    """Only fresh, controlled, no-capture failures may be deferred to quota repair."""
    if not is_stationary_failure(row):raise ValueError('Not a rejected Ego scene')
    path=Path(row['failure_evidence'])
    if sha256(path)!=row['failure_evidence_sha256']:raise ValueError('Ego evidence checksum mismatch')
    proof=json.loads(path.read_text());safety=proof.get('continuation_safety',{})
    if (proof.get('scenario_sha256')!=row['scenario_sha256'] or not proof.get('no_capture_requested')
            or proof.get('reason') not in ('ego_pre_capture_not_stationary','ego_post_park_not_stationary','stationary_ego_drift_exceeded')
            or not safety.get('topic_freshness_verified') or not safety.get('park_control_verified')
            or not safety.get('collision_epoch',{}).get('matched')
            or not safety.get('collision',{}).get('observed')
            or safety.get('collision',{}).get('non_ego_uids')
            or not safety.get('brake_held')):
        raise ValueError('Ego failure lacks safe continuation evidence')
    return proof


def is_collision_failure(row):
    return (row.get('failure_kind')=='physical_collision' or
        'physical_collision_during_capture_window' in row.get('audit',{}).get('review_reasons',[]))


def validate_collision_failure(row):
    """Only authoritative contact evidence may become a skippable quota deficit."""
    if not is_collision_failure(row) or row.get('status')=='accepted_candidate':
        raise ValueError('Not a rejected collision scene')
    if row.get('failure_kind')=='physical_collision':
        path=Path(row['failure_evidence'])
        if sha256(path)!=row['failure_evidence_sha256']:raise ValueError('Collision evidence checksum mismatch')
        proof=json.loads(path.read_text())
        if proof.get('scenario_sha256')!=row['scenario_sha256'] or not proof.get('non_ego_uids'):
            raise ValueError('Collision evidence lacks matching peer contact')
    else:
        if not row.get('capture_root'):raise ValueError('Collision review lacks capture provenance')
        path=Path(row['capture_root'])/'collision_history.json'
        summary=json.loads(path.read_text())['summary']
        if not summary.get('observed') or not summary.get('non_ego_uids'):
            raise ValueError('Collision review lacks actual contact witness')


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
    deck=json.loads(Path(manifest).read_text())
    from base5_layout_split import validate_manifest_overlay,overlay_for_metadata,resolve as resolve_split
    validate_manifest_overlay(deck,allow_subset=True)
    scale_policy=None
    if deck.get('config',{}).get('asset_scale_policy'):
        from base5_asset_scale import bind_config
        deck['config']=bind_config(deck['config'])
        scale_policy=deck['config']['asset_scale_policy']
    from base5_far_vehicle_native import validate_manifest as validate_far_manifest
    far_source_pins=validate_far_manifest(deck)
    from base5_shadow_ground_reuse import validate_manifest as validate_shadow_reuse
    shadow_source_pins=validate_shadow_reuse(deck,sensor)
    from generate_dynamic_object_capture_set import DEFAULT_MGEO
    map_sources={name:sha256(DEFAULT_MGEO/name) if (DEFAULT_MGEO/name).exists() else None
        for name in ('link_set.json','lane_boundary_set.json')}
    contract=dict(schema_version=7,manifest_sha256=sha256(manifest),
        ego_support_map_sha256=map_sources,
        network_sha256=sha256(network),sensor_sha256=sha256(sensor),
        safety_code_sha256={name:sha256(scripts/name) for name in ('base5_capture_safety.py','base5_capture_audit.py','base5_far_vehicle_native.py','base5_population_budget.py','base5_ground_support.py','base5_static_ground_policy.py','base5_ground_probe.py','run_base5_capture.py','base5_capture_timing.py','base5_event_cmd.py','base5_motion_ground.py','base5_static_ground_reference.py','base5_map_surface.py','base5_intersection_ground.py','base5_intersection_placement.py','base5_shadow_observation.py','base5_region_contract.py','base5_placement_geometry.py')},
        safety_policy='collision-history-v1; missing-runtime-actors; AI-standing-visible-annotation-v1; local-ground-plane-v1; all-actor-motion-endpoint-window-v1; vertical-step-window-v1; final-ego-road-footprint-v114; post-park-stationary-v114')
    reference=deck.get('config',{}).get('static_placement_geometry')
    for name in ('base5_scale_capture_contract.py','base5_asset_scale.py','base5_native_box_corners.py','base5_export_readiness.py','base5_layout_split.py','base5_unit_geometry_contract.py','base5_unit_capture_provenance.py'):
        contract['safety_code_sha256'][name]=sha256(scripts/name)
    if deck.get('config',{}).get('static_mission_extent'):
        contract['safety_code_sha256']['base5_static_mission_extent.py']=sha256(scripts/'base5_static_mission_extent.py')
    if deck.get('config',{}).get('obstacle_support_policy'):
        contract['safety_code_sha256']['base5_obstacle_support_policy.py']=sha256(scripts/'base5_obstacle_support_policy.py')
        contract['safety_code_sha256']['base5_obstacle_rich_floor.py']=sha256(scripts/'base5_obstacle_rich_floor.py')
    if deck.get('config',{}).get('obstacle_front_support_policy'):
        contract['safety_code_sha256']['base5_obstacle_front_support.py']=sha256(scripts/'base5_obstacle_front_support.py')
    if deck.get('config',{}).get('secondary_far_vehicle_policy'):
        for name in ('base5_secondary_far_vehicle.py','base5_far_vehicle_native.py',
                     'base5_far_vehicle_visibility.py','base5_primary_visibility_proposal.py'):
            contract['safety_code_sha256'][name]=sha256(scripts/name)
        contract['secondary_far_vehicle_source_files']=far_source_pins
    if shadow_source_pins:
        contract['safety_code_sha256']['base5_shadow_ground_reuse.py']=sha256(scripts/'base5_shadow_ground_reuse.py')
        contract['shadow_ground_reuse_source_files']=shadow_source_pins
    range_policy=deck.get('config',{}).get('primary_obstacle_range_policy')
    if range_policy:
        contract['safety_code_sha256']['base5_primary_obstacle_range.py']=sha256(scripts/'base5_primary_obstacle_range.py')
        range_reference=range_policy['reference']
        if sha256(range_reference['path'])!=range_reference['sha256']:
            raise ValueError('Primary obstacle range reference changed')
        contract['primary_obstacle_range_reference']=range_reference
    if deck.get('config',{}).get('parked_npc_placement'):
        for name in ('base5_parked_npc_placement.py','base5_static_yaw.py'):
            contract['safety_code_sha256'][name]=sha256(scripts/name)
    if reference:
        if sha256(reference['path'])!=reference['sha256']:
            raise ValueError('Placement geometry reference changed')
        contract['static_placement_geometry']=reference
    if runner_options is not None:contract['runner_options']=runner_options
    if deck.get('split_overlay') is not None:contract['training_split_overlay']=deck['split_overlay']
    if deck.get('config',{}).get('native_unit_geometry_reference') is not None:
        from base5_unit_geometry_contract import load as load_unit_geometry
        unit_reference=deck['config']['native_unit_geometry_reference']
        load_unit_geometry(unit_reference)
        contract['native_unit_geometry_reference']=unit_reference
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
        source_entry=next((e for e in deck.get('scenes',[]) if e['id']==ident),None)
        captured_metadata=json.loads((capture/'metadata.json').read_text())
        if source_entry is not None:
            planned_metadata=json.loads(Path(source_entry['metadata']).read_text())
            if captured_metadata.get('training_split_overlay')!=planned_metadata.get('training_split_overlay'):
                raise ValueError('Completed capture changed or removed training split overlay: '+ident)
            if captured_metadata.get('native_unit_geometry_reference')!=planned_metadata.get('native_unit_geometry_reference'):
                raise ValueError('Completed capture changed or removed native unit geometry reference: '+ident)
        overlay=overlay_for_metadata(captured_metadata)
        if overlay is not None:
            resolve_split(overlay,ident,row,captured_metadata)
        from base5_far_vehicle_native import verify_capture as verify_far_capture
        verify_far_capture(source_entry,capture)
        from base5_shadow_ground_reuse import verify_capture as verify_shadow_capture
        verify_shadow_capture(source_entry,capture,deck)
        from base5_scale_capture_contract import verify_capture as verify_scale_capture
        verify_scale_capture(source_entry,capture,deck)
        from base5_unit_capture_provenance import verify_capture as verify_unit_capture
        verify_unit_capture(source_entry,capture,deck,row)
        cameras=list(capture.glob('CAMERA_*/*_Intensity.png'))
        lidar=list(capture.glob('LIDAR_*/*_Intensity.bin'))
        labels=list(capture.glob('LIDAR_*/*_Instance.txt'))
        if len(cameras)!=4 or len(lidar)!=1 or len(labels)!=1 or any(p.stat().st_size==0 for p in cameras+lidar):
            raise ValueError('Completed ledger has missing/empty sensor files: '+ident)
    if scale_policy is not None:
        from base5_asset_scale import verify_frozen_policy
        verify_frozen_policy(scale_policy)
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


def scene_collision_epoch(samples,load_time,expected,max_pose_error=.35):
    """Exclude only the pre-teleport prefix; retain every contact after matching Ego."""
    import math
    rows=[r for r in samples if r['t']>=load_time]
    for i,row in enumerate(rows):
        ego=next((a for key in ('collision_object','collision_objecta')
                  for a in row['message'].get(key,[]) if int(a['unique_id'])==0),None)
        if ego and math.hypot(ego['position']['x']-expected['x'],ego['position']['y']-expected['y'])<=max_pose_error:
            yaw=(float(ego['heading'])-1.347-expected['yaw']+180)%360-180
            if abs(yaw)<=1.:
                return rows[i:],dict(matched=True,start_monotonic=row['t'],excluded_transition_messages=i,
                    contract='Trim only pre-new-pose prefix; never discard later collision-induced drift')
    observations=[]
    for row in rows:
        for key in ('collision_object','collision_objecta'):
            for ego in row['message'].get(key,[]):
                if int(ego['unique_id'])!=0:continue
                raw_delta=(float(ego['heading'])-expected['yaw']+180)%360-180
                observations.append(dict(t=row['t'],array=key,position=ego['position'],
                    heading_raw_deg=ego['heading'],
                    position_error_m=math.hypot(ego['position']['x']-expected['x'],ego['position']['y']-expected['y']),
                    raw_heading_minus_requested_deg=raw_delta,
                    corrected_heading_error_deg=(raw_delta-1.347+180)%360-180))
    raise ScenePoseUnverifiedError(dict(reason='collision_scene_pose_unverified',expected=expected,
        load_time_monotonic=load_time,max_xy_error_m=max_pose_error,max_yaw_error_deg=1.,
        heading_correction_deg=-1.347,ego_observations=observations,collision_samples=rows,
        no_capture_requested=True,acceptance_unchanged=True,
        limitation='Diagnostic evidence only. Do not infer a new heading correction or erase contacts from a mismatch.'))
