#!/usr/bin/env python3
"""Bounded Base5 quality recovery: capture -> Z/probe/replacement -> recheck."""
import argparse
import copy
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import time
import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256,prepare_session,load_ledger,is_collision_failure,validate_collision_failure
from prepare_base5_ground_retry import prepare as height_retry
from prepare_base5_empty_ground_probe import prepare as empty_probe
from prepare_base5_probed_retry import prepare as probed_retry
from prepare_base5_setup_budget_pilot import prepare as budget_retry
import base5_motion_ground as motion_ground
from prepare_base5_static_height_reference import prepare as static_height_reference
from base5_static_ground_reference import prepare as static_reference_retry,reaudit_original


def arrival_retry_eligible(result,manifest):
    if result.get('status')!='failed' or result.get('error')!='Natural target arrival not observed for all moving actors; no capture':
        return False
    if gen.geo.load_json(manifest)['config']['capture'].get('natural_setup_budget_sec',9)>=9:return False
    deck=gen.geo.load_json(manifest)
    if any(a.get('motion_path',{}).get('ground_path_evidence') for s in deck.get('scenes',[]) if s.get('metadata')
           for a in gen.geo.load_json(Path(s['metadata']))['runtime_actors']):return False
    safety=result.get('no_capture_safety',{})
    return (all(safety.get(k) is True for k in ('no_capture_requested','topic_freshness_verified',
        'actor_presence_verified','ego_stationary_verified','actor_z_steps_verified'))
        and safety.get('collision',{}).get('no_collision_verified') is True)


def validate_miss_evidence(row,manifest):
    if not arrival_retry_eligible(row,manifest):raise ValueError('Unsafe/non-budget target failure')
    path=Path(row['failure_evidence'])
    if sha256(path)!=row['failure_evidence_sha256']:raise ValueError('Target miss evidence changed')
    data=gen.geo.load_json(path)
    if (data['scenario_sha256']!=row['scenario_sha256'] or data['no_capture_safety']!=row['no_capture_safety']
            or data['reason']!='natural_targets_not_simultaneously_observed'):
        raise ValueError('Target miss evidence does not match ledger')


def replacement(manifest,output):
    m=gen.geo.load_json(manifest);old=m['scenes'][0];meta=gen.geo.load_json(Path(old['metadata']))
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    for retry in range(meta['retry']+1,meta['retry']+513):
        try:scene,new=gen.build(meta['spec'],m['config'],ctx,retry)
        except ValueError:continue
        if new['split']!=meta['split']:continue
        if new['runtime_actors'][0]['data_id']!=meta['runtime_actors'][0]['data_id']:raise ValueError('Replacement changed primary asset')
        break
    else:raise ValueError('Same-cell/split replacement unavailable')
    output.mkdir();sid=old['id']+'_replace';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
    new['replacement']=dict(source_cell_id=old.get('source_cell_id',old['id']),source_scenario_sha256=old['scenario_sha256'],
        policy='same primary asset, family, mission, motion request, role, distance cell and split; no GT rewriting')
    atomic_json(sp,scene);atomic_json(mp,new)
    old.update(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),motion=new['runtime_actors'][0]['motion'])
    old['source_cell_id']=new['replacement']['source_cell_id']
    atomic_json(output/'manifest.json',m);return output/'manifest.json'


def validate_seed(path,original):
    """Reuse only authoritative, safe same-payload batch review; never transplant raw."""
    seed=gen.geo.load_json(path);root=Path(seed['capture_root']);manifest=Path(seed['manifest'])
    safety=gen.geo.load_json(root/'shutdown_safety.json')
    if not safety.get('park_verified'):raise ValueError('Seed final Park unverified')
    session=gen.geo.load_json(root/'session_contract.json');repo=Path(__file__).resolve().parents[3]
    rows,_=prepare_session(root,manifest,
        repo/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
        repo/'configs/sensors/Sensor_competition_local.json',session.get('runner_options'))
    if not session.get('runner_options',{}).get('shared_ros_batch'):
        raise ValueError('Seed must be a guarded shared-ROS batch')
    row=next((r for r in reversed(rows) if r['id']==seed['id']),None)
    if row is None or row['id']!=original['id'] or row['scenario_sha256']!=original['scenario_sha256']:
        raise ValueError('Seed is not the original quota payload')
    missed=row['status']=='failed'
    if missed:validate_miss_evidence(row,manifest)
    elif row['status']!='review':raise ValueError('Only reviewed seed may enter recovery')
    if any((r['status'] not in ('accepted_candidate','review') and r is not row)
            or not r.get('between_scene_park',{}).get('park_verified') for r in rows):
        raise ValueError('Seed batch unsafe or failed')
    if missed:return copy.deepcopy(row),root,safety
    capture=Path(row['capture_root'])
    if gen.geo.load_json(capture/'audit.json')!=row['audit']:raise ValueError('Seed audit differs from authoritative ledger')
    if gen.geo.load_json(capture/'metadata.json').get('calibration_only'):raise ValueError('Calibration seed forbidden')
    if 'physical_collision_during_capture_window' in row['audit'].get('review_reasons',[]):
        raise ValueError('Physical collision seed forbidden')
    return copy.deepcopy(row),root,safety


def execute(source,indices,output,resume=False,stop_path=None,progress=None,seed_capture=None):
    started=time.monotonic()
    if output.exists() and not resume:raise FileExistsError('New quality-loop output required; use campaign resume')
    if not 1<=len(indices)<=4 or len(set(indices))!=len(indices):raise ValueError('Pilot limited to 1–4 cells')
    if seed_capture is not None and len(indices)!=1:raise ValueError('Seed recovery is single-cell only')
    output.mkdir(parents=True,exist_ok=True);lock=(output/'.quality.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    prior_elapsed=(gen.geo.load_json(output/'report.json').get('elapsed_sec',0.)
        if resume and (output/'report.json').exists() else 0.)
    source_data=gen.geo.load_json(source);attempts=[];selected=[];unresolved=[]
    runner=Path(__file__).with_name('run_base5_capture.py')
    def stopped():return (output/'STOP').exists() or (stop_path is not None and stop_path.exists())
    def prepared(path,make):
        if path.exists():
            # Only complete, durable preparation decks are reusable.
            deck=gen.geo.load_json(path)
            for item in deck['scenes']:
                if sha256(Path(item['scenario']))!=item['scenario_sha256']:raise ValueError('Prepared deck checksum mismatch')
                gen.geo.load_json(Path(item['metadata']))
            return path
        return make()
    def save(stage,error=None):
        # Preserve usable selections even when a later child fails. Unsafe
        # children never enter this list or the conversion handoff.
        target=output/'selected_ledger.jsonl';temp=output/'.selected_ledger.tmp'
        temp.write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in selected));temp.replace(target)
        elapsed=prior_elapsed+time.monotonic()-started
        atomic_json(output/'report.json',dict(stage=stage,error=error,elapsed_sec=elapsed,source_manifest_sha256=sha256(source),
            attempts=attempts,selected=selected,unresolved=unresolved,bulk_capture=False))
        atomic_json(output/'status.json',dict(stage=stage,error=error,current=attempts[-1]['cell'] if attempts else None,
            requested=len(indices),finished=len(selected)+len(unresolved),accepted_candidate=len(selected),review=len(unresolved),
            failed=int(stage=='halted'),updated_at=time.time(),elapsed_sec=elapsed,eta_sec=0,results=[]))
    def capture(manifest,root,cell,kind):
        if stopped():raise RuntimeError('Quality-loop STOP requested')
        root.mkdir(parents=True,exist_ok=True)
        if (root/'session_contract.json').exists():
            safety_path=root/'shutdown_safety.json'
            if not safety_path.exists() or not gen.geo.load_json(safety_path).get('park_verified'):
                raise RuntimeError('Interrupted/unsafe child requires manual safety recovery and a new campaign')
            if resume:
                session=gen.geo.load_json(root/'session_contract.json');repo=Path(__file__).resolve().parents[3]
                rows,_=prepare_session(root,manifest,
                    repo/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
                    repo/'configs/sensors/Sensor_competition_local.json',session.get('runner_options'))
                if rows and rows[-1]['status'] in ('accepted_candidate','review'):
                    row=rows[-1]
                    if gen.geo.load_json(Path(row['capture_root'])/'audit.json')!=row['audit']:
                        raise RuntimeError('Resumed capture audit differs from ledger')
                    if is_collision_failure(row):validate_collision_failure(row)
                    attempts.append(dict(cell=cell,kind=kind,status=row['status'],capture_root=str(root),
                        reasons=row['audit']['review_reasons'],reused_completed_child=True,
                        shutdown=gen.geo.load_json(safety_path)))
                    save('running');return row
        cmd=[sys.executable,str(runner),'--manifest',str(manifest),'--output',str(root),'--limit','1']
        with (root.parent/(root.name+'.log')).open('w') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+100.
            heartbeat=0.
            while child.poll() is None:
                if progress and time.monotonic()>=heartbeat:
                    progress(cell,kind,root);heartbeat=time.monotonic()+2.
                if stopped() or time.monotonic()>deadline:
                    (root/'STOP').touch();child.terminate()
                    try:child.wait(timeout=15.)
                    except subprocess.TimeoutExpired:raise RuntimeError('Child remains alive after interrupt; no further loads')
                    raise RuntimeError('Child interrupted; check shutdown evidence before any resume')
                try:time.sleep(.1)
                except KeyboardInterrupt:
                    (root/'STOP').touch();child.terminate()
                    child.wait(timeout=15.)
                    raise RuntimeError('Interrupted; child shutdown must be checked before resume')
        if child.returncode:
            safety=gen.geo.load_json(root/'shutdown_safety.json') if (root/'shutdown_safety.json').exists() else {}
            rows=load_ledger(root/'ledger.jsonl')
            child_status=gen.geo.load_json(root/'status.json') if (root/'status.json').exists() else {}
            collision_exit=child_status.get('error') in (
                'Batch review/failure or unsafe boundary Park; halt before next scene',
                'Capture halted after connection failure or three consecutive scene failures')
            if rows and is_collision_failure(rows[-1]) and safety.get('park_verified') and collision_exit:
                validate_collision_failure(rows[-1])
                row=rows[-1]
                attempts.append(dict(cell=cell,kind=kind,capture_root=str(root),status=row['status'],
                    reasons=['physical_collision'],shutdown=safety))
                save('running');return row
            attempts.append(dict(cell=cell,kind=kind,capture_root=str(root),status='failed',
                reasons=['child_failed_or_exit_unsafe'],shutdown=safety))
            raise RuntimeError('Capture child failed; see '+str(root)+'.log')
        safety=gen.geo.load_json(root/'shutdown_safety.json')
        if not safety['park_verified']:raise RuntimeError('Exit Park unverified; no further loads')
        rows=[json.loads(l) for l in (root/'ledger.jsonl').read_text().splitlines() if l]
        row=rows[-1];attempts.append(dict(cell=cell,kind=kind,capture_root=str(root),status=row['status'],
            reasons=row.get('audit',{}).get('review_reasons',[]),shutdown=safety));save('running')
        if is_collision_failure(row):validate_collision_failure(row);return row
        if row['status']=='failed' and (kind!='initial' or not arrival_retry_eligible(row,manifest)):
            raise RuntimeError('Scene failed without safe budget-retry evidence; no automatic continuation')
        return row
    try:
        save('running')
        for index in indices:
            original=copy.deepcopy(source_data['scenes'][index]);cell=original['id'];base=output/cell;base.mkdir(exist_ok=True)
            m=copy.deepcopy(source_data);m.update(scenes=[original],count=1)
            manifest=base/'manifest.json';atomic_json(manifest,m)
            if seed_capture is not None:
                if stopped():raise RuntimeError('STOP before seeded recovery')
                result,seed_root,safety=validate_seed(seed_capture,original)
                attempts.append(dict(cell=cell,kind='seeded_batch_target_miss' if result['status']=='failed' else 'seeded_batch_review',capture_root=str(seed_root),
                    status=result['status'],reasons=result.get('audit',{}).get('review_reasons',[]),shutdown=safety,
                    reused_raw=True,seed_source=str(seed_capture)))
                save('running')
            else:
                metadata=gen.geo.load_json(Path(original['metadata'])) if original.get('metadata') else {}
                if motion_ground.moving_actors(metadata):
                    for actor in motion_ground.moving_actors(metadata):
                        path=actor.get('motion_path',{})
                        if path.get('policy')!='natural-target-v1' or 'target_position' not in path:
                            raise ValueError('Moving pilot requires natural-target-v1 metadata before calibration')
                    # Observe terrain and this asset at the target before loading
                    # an animated path. Calibration frames never fill training quota.
                    probes=prepared(base/'motion_terrain_deck'/'manifest.json',
                        lambda:empty_probe(manifest,[0],base/'motion_terrain_deck',backward_route_m=6.))
                    terrain_result=capture(probes,base/'motion_terrain',cell,'motion_empty_terrain')
                    if is_collision_failure(terrain_result):
                        unresolved.append(dict(cell=cell,reasons=['physical_collision'],evidence=terrain_result));save('running');continue
                    refs=prepared(base/'motion_static_deck'/'manifest.json',
                        lambda:motion_ground.static_reference(manifest,base/'motion_static_deck'))
                    reference=capture(refs,base/'motion_static',cell,'motion_static_reference')
                    if is_collision_failure(reference):
                        unresolved.append(dict(cell=cell,reasons=['physical_collision'],evidence=reference));save('running');continue
                    try:
                        manifest=prepared(base/'motion_ground_deck'/'manifest.json',
                            lambda:motion_ground.prepare(manifest,base/'motion_terrain',base/'motion_static',base/'motion_ground_deck'))
                    except ValueError as exc:
                        unresolved.append(dict(cell=cell,reasons=['motion_ground_preparation_unavailable'],error=str(exc)))
                        save('running');continue
                result=capture(manifest,base/'initial',cell,'initial')
            if is_collision_failure(result):
                unresolved.append(dict(cell=cell,reasons=['physical_collision'],evidence=result));save('running');continue
            if result['status']=='failed':
                if not arrival_retry_eligible(result,manifest):
                    raise RuntimeError('Target miss lacks safe evidence or already used 9-second budget')
                manifest=prepared(base/'budget9_deck'/'manifest.json',
                    lambda:budget_retry(manifest,[0],base/'budget9_deck',9.))
                result=capture(manifest,base/'budget9',cell,'natural_budget9_retry')
                if result['status']=='failed':raise RuntimeError('9-second target retry failed; no further loads')
            for ordinal in range(2):
                if result['status']=='accepted_candidate':break
                reasons=set(result.get('audit',{}).get('review_reasons',[]))
                if is_collision_failure(result):break
                current=Path(attempts[-1]['capture_root'])
                if ordinal==0 and reasons=={'actor_annotation_above_ground'}:
                    next_manifest=prepared(base/'height_deck'/'manifest.json',lambda:height_retry(manifest,current,base/'height_deck'))
                    kind='input_z_retry'
                elif reasons and reasons<={'actor_ground_plane_unverified','actor_annotation_deep_below_ground','actor_annotation_above_ground'}:
                    try:
                        probe_meta=gen.geo.load_json(Path(gen.geo.load_json(manifest)['scenes'][0]['metadata']))
                        failed_ground=[a['uid'] for a in result['audit'].get('ground_support',{}).get('actors',[]) if a.get('reason')]
                        target_uid=((failed_ground or [probe_meta['runtime_actors'][0]['unique_id']])[0]
                            if probe_meta.get('ground_support_check',{}).get('policy')=='all-actors-observed-ground-v1'
                            and probe_meta.get('runtime_actors') else None)
                        terrain_deck=base/f'terrain_deck_{ordinal}';terrain_root=base/f'terrain_{ordinal}'
                        retry_deck=base/f'probed_deck_{ordinal}'
                        target=next((a for a in probe_meta.get('runtime_actors',[]) if a['unique_id']==target_uid),None)
                        components=[c for c in result['audit'].get('components',[]) if c['parent_uid']==target_uid]
                        if (reasons=={'actor_ground_plane_unverified'} and target
                                and len(components)==1 and target['planned_center_distance_m']>20.
                                and not motion_ground.moving_actors(probe_meta)):
                            near_deck=base/f'static_near_deck_{ordinal}';near_root=base/f'static_near_{ordinal}'
                            probes=prepared(near_deck/'manifest.json',lambda:static_height_reference(manifest,0,near_deck,target_uid=target_uid))
                            reference=capture(probes,near_root,cell,'static_near_ground_reference')
                            if is_collision_failure(reference):result=reference;break
                            next_manifest=prepared(retry_deck/'manifest.json',lambda:static_reference_retry(manifest,near_root,retry_deck))
                            kind='static_rigid_ground_retry'
                            result=reaudit_original(result,next_manifest,base/f'reaudited_{ordinal}')
                            attempts.append(dict(cell=cell,kind='static_rigid_raw_reaudit',capture_root=result['capture_root'],
                                status=result['status'],reasons=result['audit']['review_reasons'],reused_sensor_capture=True,
                                shutdown=gen.geo.load_json(near_root/'shutdown_safety.json')))
                            manifest=next_manifest
                            save('running')
                            continue
                        else:
                            probes=prepared(terrain_deck/'manifest.json',lambda:empty_probe(manifest,[0],terrain_deck,target_uid=target_uid))
                            terrain_result=capture(probes,terrain_root,cell,'empty_terrain')
                            if is_collision_failure(terrain_result):result=terrain_result;break
                            next_manifest=prepared(retry_deck/'manifest.json',lambda:probed_retry(manifest,current,terrain_root,retry_deck))
                            kind='independent_ground_retry'
                    except ValueError as exc:
                        # Unsupported terrain is a quality deficit, not permission
                        # to waive the gate. Communication exceptions stay fatal.
                        attempts.append(dict(cell=cell,kind='ground_preparation_unavailable',error=str(exc)))
                        fallback_deck=base/f'fallback_deck_{ordinal}'
                        next_manifest=prepared(fallback_deck/'manifest.json',lambda:replacement(manifest,fallback_deck))
                        kind='same_cell_replacement'
                else:
                    next_manifest=prepared(base/f'replacement_deck_{ordinal}'/'manifest.json',lambda:replacement(manifest,base/f'replacement_deck_{ordinal}'))
                    kind='same_cell_replacement'
                if not gen.geo.load_json(next_manifest)['scenes']:break
                manifest=next_manifest
                result=capture(manifest,base/f'retry_{ordinal}',cell,kind)
            if result['status']=='accepted_candidate':
                result['source_cell_id']=cell;selected.append(result)
            else:unresolved.append(dict(cell=cell,reasons=result.get('audit',{}).get('review_reasons',[result.get('error','quality deficit')])))
            save('running')
        save('complete');return 0
    except Exception as exc:save('halted',str(exc));print(str(exc),file=sys.stderr);return 1
    finally:lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--indices',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raise SystemExit(execute(a.manifest,[int(i) for i in a.indices.split(',')],a.output))
