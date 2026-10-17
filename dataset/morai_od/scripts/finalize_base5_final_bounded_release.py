#!/usr/bin/env python3
"""Freeze current bounded native/input/resume gates; never run capture or bulk."""
import argparse
from collections import Counter
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import pickle
import tempfile

import audit_base5_scenarios
import base5_front_release_inputs as pins
import base5_asset_scale as scale
import base5_layout_split as layout
import base5_scale_capture_contract as scale_capture
import base5_shadow_ground_reuse as shadow
import base5_unit_geometry_contract as unit
import base5_unit_capture_provenance as unit_provenance
import prepare_base5_final_bounded_smoke as smoke
import report_base5_scale_stability as stability_report
import qualify_base5_training_inputs as training
import run_base5_fresh_completion as completion
from base5_capture_safety import atomic_json,collision_window,prepare_session,sha256
from finalize_base5_precollection_release import verify_resume,verify_training_inputs
from finalize_base5_scale_extent_release import test_results
from verify_base5_completed_resume import snapshot

SCHEMA='base5_obstacle_support_release_v1'
POLICY='final-scale-unit-layout-bounded-native-qualified-v156'


def read(path):return json.loads(Path(path).read_text())


def require(value,message):
    if not value:raise ValueError(message)


def verify_native(context,capture):
    capture=Path(capture).resolve(strict=True)
    reconstructed,missing,_=completion.verify_parent(context['subset_path'],capture)
    rows=training.ledger(capture/'selected_ledger.jsonl') if reconstructed else []
    require(reconstructed==rows,'Current fresh reconstruction differs from selected ledger')
    expected=context['selected'];count=len(expected)
    ids=[row.get('source_cell_id') for row in rows]
    require(len(set(ids))==len(ids) and set(ids)<=set(expected) and set(missing)==set(expected)-set(ids),
        'Current native logical-slot accounting differs')
    state=read(capture/'status.json')
    require(state.get('stage') in ('complete','complete_with_deficit') and state.get('requested')==count,
        'Terminal preregistered bounded campaign required')
    blockers=[];covered=set();observations=[];profile_counts=Counter()
    if missing or len(rows)!=count or state.get('accepted_candidate')!=count or state.get('review')!=0:
        blockers.append('current_exact_smoke_native_deficit')
    for row in rows:
        ident=row['source_cell_id'];entry=context['entries'][ident]
        raw=Path(row['capture_root']).resolve(strict=True);raw.relative_to(capture)
        meta,audit=read(raw/'metadata.json'),read(raw/'audit.json')
        require(row.get('audit')==audit and sha256(raw/'scenario.json')==row['scenario_sha256']
            and row['scenario_sha256']==entry['scenario_sha256'],
            'Native smoke cannot substitute a recovered/foreign geometry for its preregistered source')
        require(all(key in meta and meta[key]==value for key,value in context['metadata'][ident].items()),
            'Native smoke changed exact planned metadata fields')
        require(meta.get(unit.KEY)==context['deck']['config'][unit.KEY],'Native unit reference changed')
        unit_provenance.verify_capture(entry,raw,context['deck'],row)
        scale_capture.verify_capture(entry,raw,context['deck'])
        layout.resolve(context['overlay'],row['id'],row,meta)
        shadow.verify_capture(entry,raw,context['deck'])
        unit_review=unit.review(meta,audit)
        require(audit.get('native_unit_geometry_review')==unit_review,'Native unit review receipt differs from actual current QA')
        scale_review=scale_capture.native_size_review(meta,audit)
        require(audit.get('asset_scale_review')==scale_review,'Native scale review receipt differs from actual current QA')
        if (audit.get('review_reasons') or audit.get('accepted_candidate') is not True
                or unit_review['review_reasons'] or scale_review['review_reasons']):
            blockers.append('current_native_review:'+ident);continue
        collision=read(raw/'collision_history.json');summary=collision['summary']
        actual=collision_window(collision['samples'],summary['window_start_monotonic'],summary['window_end_monotonic'])
        peers={int(a['unique_id']) for sample in collision['samples'] for key in ('collision_object','collision_objecta')
            for a in sample['message'].get(key,[]) if int(a['unique_id'])!=0}
        require(summary.get('observed') is True and summary.get('no_collision_verified') is True
            and summary.get('non_ego_uids')==[] and actual['observed'] and actual['no_collision_verified'] and not peers,
            'Current native collision evidence is incomplete or nonclear')
        require(read(raw.parents[1]/'shutdown_safety.json').get('park_verified') is True,'Current native final Park missing')
        training.assert_native(raw,audit,row['scenario_sha256'])
        effective=layout.resolve(context['overlay'],row['id'],row,meta)['split']
        features=smoke.features(meta,context['pairs'],effective,context['unit_assets']);covered.update(features)
        profile_counts.update(features)
        observations.append(dict(id=ident,capture_root=str(raw),scenario_sha256=row['scenario_sha256'],
            coverage=sorted(features),components=len(audit['components']),native_unit_review_verified=True))
    reviews=[]
    for path in capture.rglob('ledger.jsonl'):
        for line in path.read_text().splitlines():
            row=json.loads(line)
            if row.get('status') in ('review','failed'):
                reviews.append(dict(ledger=str(path),id=row.get('id'),status=row['status']))
    if reviews:blockers.append('preserved_current_smoke_review_or_failure')
    if not set(context['doc']['coverage']['required'])<=covered:blockers.append('accepted_native_required_coverage_missing')
    if any(profile_counts[token]<minimum for token,minimum in context['doc']['coverage']['profile_minima'].items()):
        blockers.append('accepted_native_rare_profile_minimum_missing')
    require(all(read(path.parent/'shutdown_safety.json').get('park_verified') is True
        for path in capture.rglob('session_contract.json')),'A current native session lacks final Park')
    for path in capture.rglob('session_contract.json'):
        events=[json.loads(line) for line in (path.parent/'event_cmd_requests.jsonl').read_text().splitlines()]
        require(events and all(type(event.get('recorded_at')) in (int,float) and math.isfinite(event['recorded_at'])
            and event['recorded_at']>=context['doc']['registered_at_unix'] for event in events),
            'Current smoke native events predate preregistration; no retroactive promotion')
    return dict(requested=count,accepted=len(rows),missing=missing,observations=observations,
        covered=sorted(covered),preserved_reviews=reviews,external_support_approvals_counted=0,
        accepted_profile_frames={token:profile_counts[token] for token in context['doc']['coverage']['profile_minima']},
        source_ledger=str(capture/'selected_ledger.jsonl')),sorted(set(blockers))


def verify_partial(partial_snapshot,partial_receipt,manifest,capture,selected_ids):
    before,receipt=read(partial_snapshot),read(partial_receipt);current=snapshot(Path(capture))
    require(before.get('manifest_sha256')==sha256(manifest)
        and Path(before['capture_root']).resolve()==Path(capture).resolve()
        and before.get('preserved_selected')==4 and len(before.get('selected_rows',[]))==4,
        'Exact four-row preregistered partial snapshot required')
    require({row['source_cell_id'] for row in before['selected_rows']}==set(selected_ids[:4]),
        'Partial snapshot is not the registered first chunk')
    require(receipt.get('passed') is True and receipt.get('source_partial_sha256')==sha256(partial_snapshot)
        and receipt.get('manifest_sha256')==sha256(manifest) and receipt.get('preserved_selected')==4
        and receipt.get('prior_raw_unchanged') is True and receipt.get('before')==before['before']
        and receipt.get('after')==current,'Partial-resume receipt/source/current inventory differs')
    require(all(before['before'][key] and all(current[key].get(k)==v for k,v in before['before'][key].items())
        for key in ('raw_files','event_logs')),'Prior partial RAW/events changed')
    rows={r['source_cell_id']:r for r in training.ledger(Path(capture)/'selected_ledger.jsonl')}
    require(all(rows.get(r['source_cell_id'])==r for r in before['selected_rows'])
        and current['cumulative_elapsed_sec']>=before['before']['cumulative_elapsed_sec'],
        'Prior partial approvals/timing changed')
    return dict(passed=True,preserved_selected=4,raw_event_and_selection_unchanged=True,
        snapshot_sha256=sha256(partial_snapshot),receipt_sha256=sha256(partial_receipt))


def verify_inputs(report,loader_report,capture,frames):
    doc=read(report);expected={name:cohort.get('frames') for name,cohort in doc.get('cohorts',{}).items()}
    require(expected,'Native input report has no cohorts')
    verified=verify_training_inputs(report,expected)
    matching=[(name,cohort) for name,cohort in verified['cohorts'].items()
        if Path(cohort['source_ledger']).resolve()==(Path(capture)/'selected_ledger.jsonl').resolve()]
    require(len(matching)==1 and matching[0][1]['frames']==frames,'Exact current smoke native input cohort required')
    name,cohort=matching[0];loaded=read(loader_report)
    from qualify_base5_training_loader_cpu import __file__ as loader_helper
    require(loaded.get('schema')=='base5_cpu_training_loader_qualification_v1'
        and loaded.get('script_sha256')==sha256(loader_helper)
        and Path(loaded['qualification_report']).resolve()==Path(report).resolve()
        and loaded.get('qualification_sha256')==sha256(report)
        and loaded.get('config_sha256')==sha256(loaded['config'])
        and loaded.get('cuda_available') is False and loaded.get('gpu_execution') is False
        and loaded.get('training_started') is False,'Current exact CPU loader receipt/helper/config required')
    parsed=loaded.get('cohorts',{}).get(name,{})
    require(parsed.get('package_manifest_sha256')==cohort['training3_manifest_sha256']
        and parsed.get('counts',{}).get('source_gt')==cohort['retained_gt']
        and parsed.get('counts',{}).get('train_frames')==frames
        and parsed.get('counts',{}).get('val_frames')==frames
        and len(parsed.get('frames',[]))==frames
        and len({r['token'] for r in parsed['frames']})==frames,'CPU loader frame/package identity differs')
    source_records={}
    for split in ('train','val','test'):
        records=pickle.loads((Path(cohort['training3_package'])/'infos/512x1056'/
            ('morai_base5_infos_'+split+'.pkl')).read_bytes())['data_list']
        for record in records:
            token=record['token'];require(token not in source_records,'Repeated qualified package token')
            source_records[token]=(split,len(record['instances']))
    require(len(source_records)==frames and {frame['token'] for frame in parsed['frames']}==set(source_records),
        'CPU loader token inventory differs from actual current package infos')
    for frame in parsed['frames']:
        require((frame.get('original_split'),frame.get('source_gt'))==source_records[frame['token']],
            'CPU loader per-frame split/retained GT differs from actual package infos')
        require(set(frame.get('modes',{}))=={'train','val'},'Both current CPU pipelines required')
        require(all(mode.get('image_shape')==[4,3,512,1056] and mode.get('image_dtype')=='torch.uint8'
            for mode in frame['modes'].values()),'Current Q90 512x1056 input shape/dtype differs')
    require(sum(frame.get('source_gt',-1) for frame in parsed['frames'])==cohort['retained_gt']
        and all(frame.get('source_gt',-1)>=0 for frame in parsed['frames']),
        'CPU loader lost current retained component identities')
    require(verified.get('sensor_sha256')==sha256(pins.ROOT/'configs/sensors/Sensor_competition_local.json'),
        'Native input sensor differs from current collection sensor')
    evidence={str(Path(loaded['config']).resolve()):loaded['config_sha256']}
    for path,digest in verified.get('helper_sha256',{}).items():
        resolved=Path(path) if Path(path).is_absolute() else pins.ROOT/path
        evidence[str(resolved.resolve())]=digest
    for cohort_name,value in verified['cohorts'].items():
        package=Path(value['training3_package']).resolve()
        for path in package.rglob('*'):
            if path.is_file():evidence[str(path)]=sha256(path)
        source_snapshot=Path(report).parent/cohort_name/'source_snapshot.json'
        evidence[str(source_snapshot.resolve())]=value['source_snapshot_sha256']
        evidence[str(Path(value['source_ledger']).resolve())]=value['source_ledger_sha256']
    pins.verify_hashes(evidence,'Qualified current input products/config/helpers')
    return dict(report_sha256=sha256(report),loader_report_sha256=sha256(loader_report),
        smoke_cohort=name,smoke_frames=frames,retained_gt=cohort['retained_gt'],cpu_train_and_val_pipelines_verified=True,
        evidence_hashes=evidence)


def paired_cost(rows,metadata):
    """Retries remain in cost, never in the first-attempt denominator."""
    grouped={}
    for row in rows:grouped.setdefault(row['id'],[]).append(row)
    result={}
    for condition in ('original','scaled'):
        selected=[values for ident,values in grouped.items() if metadata[ident]['scale_stability']['condition']==condition]
        total=sum(row['duration_sec'] for values in selected for row in values)
        initial_accepted=sum(values[0]['status']=='accepted_candidate' for values in selected)
        approved=sum(values[-1]['status']=='accepted_candidate' for values in selected)
        result[condition]=dict(initial_scenes=len(selected),initial_accepted=initial_accepted,
            initial_approval_fraction=initial_accepted/len(selected) if selected else None,
            first_nonaccept_fraction=1-initial_accepted/len(selected) if selected else None,
            latest_approved=approved,all_attempt_ledger_sec=total,
            ledger_sec_per_latest_approval=total/approved if approved else None)
    original,scaled=result['original'],result['scaled']
    result['scaled_nonaccept_excess']=(scaled['first_nonaccept_fraction']-original['first_nonaccept_fraction']
        if original['initial_scenes'] and scaled['initial_scenes'] else None)
    baseline=original['ledger_sec_per_latest_approval'];current=scaled['ledger_sec_per_latest_approval']
    result['scaled_cost_per_approval_ratio']=current/baseline if baseline and current is not None else None
    return result


def stability_threshold_blockers(metrics,*,profile=None):
    prefix='paired_profile_review/'+profile if profile else 'paired'
    blockers=[]
    if not profile:
        for condition in ('original','scaled'):
            if (metrics[condition]['initial_approval_fraction'] is None
                    or metrics[condition]['initial_approval_fraction']<.90-1e-12):
                blockers.append(prefix+'/initial_approval_below_90pct/'+condition)
    excess=metrics['scaled_nonaccept_excess'];cost=metrics['scaled_cost_per_approval_ratio']
    if excess is None or excess>.05+1e-12:blockers.append(prefix+'/nonaccept_excess_above_5pp')
    if cost is None or cost>1.10+1e-12:blockers.append(prefix+'/cost_per_approval_above_110pct')
    return blockers


def verify_stability_session(session,paired,manifests):
    options=session.get('runner_options',{});declared=options.get('selected_scene_ids')
    require(set(options)<={'shared_ros_batch','selected_scene_ids','optimized_adapter'}
        and type(options.get('shared_ros_batch')) is bool,'Only current ordinary/optimized native runners are qualified')
    if 'optimized_adapter' in options:
        adapter=options['optimized_adapter'];profile=adapter.get('profile')
        require(profile in ('baseline','static-ready') and adapter==dict(profile=profile,
            adapter_sha256=sha256(Path(__file__).with_name('run_base5_capture_optimized.py')),
            source_sha256=sha256(Path(__file__).with_name('run_base5_capture.py')),
            policy='static conditional setup delay only; native gates'),'Current exact runner adapter required')
    matching=[path for path in manifests if sha256(path)==session.get('manifest_sha256')]
    require(matching,'Actual paired session manifest must be supplied or preserved under the campaign root')
    manifest=matching[0];deck=read(manifest)
    require({key:value for key,value in deck.items() if key not in ('count','scenes')}==
        {key:value for key,value in paired['deck'].items() if key not in ('count','scenes')}
        and deck.get('count')==len(deck.get('scenes',[]))>0
        and len({entry['id'] for entry in deck['scenes']})==len(deck['scenes'])
        and all(entry==paired['entries'].get(entry['id']) for entry in deck['scenes'])
        and declared==[entry['id'] for entry in deck['scenes'] if entry['id'] in declared],
        'Paired session config/entries/order are not an exact registered full deck or subset')
    with tempfile.TemporaryDirectory(prefix='base5-stability-session-') as directory:
        output=Path(directory)/'session'
        prepare_session(output,manifest,
            pins.ROOT/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
            pins.ROOT/'configs/sensors/Sensor_competition_local.json',runner_options=options)
        require(read(output/'session_contract.json')==session,'Paired complete current session contract differs')
    return manifest


def verify_stability(context,capture_root,ledgers,cause_review=None,session_manifests=()):
    """Recompute the paired report from complete chronological raw sessions."""
    paired=context['paired_stability'];root=Path(capture_root).resolve(strict=True)
    paths=[Path(path).resolve(strict=True) for path in ledgers]
    actual={path.resolve() for path in root.rglob('ledger.jsonl')}
    sessions={path.parent.resolve() for path in root.rglob('session_contract.json')}
    require(paths and len(paths)==len(set(paths)) and set(paths)==actual,
        'Every raw paired session ledger under a dedicated campaign root is required')
    require(sessions=={path.parent for path in paths},'Orphan paired session or raw ledger cannot be omitted')
    manifests=[paired['manifest']]+sorted(root.rglob('manifest.json'))+[Path(path).resolve(strict=True) for path in session_manifests]
    root_inventory={str(path.resolve()):sha256(path) for path in root.rglob('*') if path.is_file()}
    inventory=dict(root_inventory)
    inventory.update({str(path):sha256(path) for path in manifests})
    require(inventory,'Empty paired native campaign')
    first_ids=[];seen=set();all_rows=[];previous_end=-math.inf
    for path in paths:
        session=read(path.parent/'session_contract.json');options=session.get('runner_options',{})
        declared=options.get('selected_scene_ids')
        require(isinstance(declared,list) and declared and len(set(declared))==len(declared)
            and set(declared)<=set(paired['entries'])
            and session.get('sensor_sha256')==sha256(pins.ROOT/'configs/sensors/Sensor_competition_local.json')
            and session.get(unit.KEY)==paired['deck']['config'][unit.KEY]
            and session.get('safety_code_sha256',{}).get('base5_unit_capture_provenance.py')==sha256(unit_provenance.__file__)
            and read(path.parent/'shutdown_safety.json').get('park_verified') is True,
            'Paired raw session lacks current declared identities, unit provenance, sensor or final Park')
        state=read(path.parent/'status.json')
        require(state.get('stage')=='complete' and state.get('requested')==len(declared),
            'Paired raw session must be terminal complete, never halted/interrupted')
        for name,digest in session['safety_code_sha256'].items():
            require(Path(name).name==name and sha256(Path(__file__).with_name(name))==digest,
                'Paired capture safety code changed')
        verify_stability_session(session,paired,manifests)
        events=[json.loads(line) for line in (path.parent/'event_cmd_requests.jsonl').read_text().splitlines()]
        require(events and all(type(event.get('recorded_at')) in (int,float)
            and math.isfinite(event['recorded_at']) for event in events),'Actual paired event chronology required')
        start,end=min(event['recorded_at'] for event in events),max(event['recorded_at'] for event in events)
        require(start>=context['doc']['registered_at_unix'],
            'Paired native events predate fixed-threshold preregistration; no retroactive promotion')
        require(start>=previous_end,'Paired raw ledgers reordered or overlapping; explicit chronological sessions required')
        previous_end=end
        payload=path.read_bytes();require(payload.endswith(b'\n'),'Unterminated paired raw ledger')
        rows=[json.loads(line) for line in payload.splitlines()]
        require(rows and set(row['id'] for row in rows)==set(declared),'Paired session ledger omitted or added declared attempts')
        for row in rows:
            ident=row['id'];entry=paired['entries'][ident]
            require(row.get('scenario_sha256')==entry['scenario_sha256']
                and row.get('status') in ('accepted_candidate','review','failed')
                and type(row.get('duration_sec')) in (int,float) and math.isfinite(row['duration_sec'])
                and row['duration_sec']>0,'Exact paired identity/status/positive ledger timing required')
            if ident not in seen:first_ids.append(ident);seen.add(ident)
            if row['status'] in ('accepted_candidate','review'):
                if options['shared_ros_batch']:
                    require(row.get('between_scene_park',{}).get('park_verified') is True,
                        'Shared paired session lacks verified between-scene Park')
                raw=Path(row['capture_root']).resolve(strict=True);raw.relative_to(path.parent/'captures')
                meta,audit=read(raw/'metadata.json'),read(raw/'audit.json')
                require(row.get('audit')==audit and all(meta.get(key)==value for key,value in paired['metadata'][ident].items()),
                    'Paired actual RAW audit or planned metadata changed')
                unit_provenance.verify_capture(entry,raw,paired['deck'],row)
                scale_capture.verify_capture(entry,raw,paired['deck'])
                require(audit.get('asset_scale_review')==scale_capture.native_size_review(meta,audit),
                    'Paired saved scale review differs from actual current QA')
                if row['status']=='accepted_candidate':
                    require(audit.get('accepted_candidate') is True,'Paired approval contradicts native QA')
                    training.assert_native(raw,audit,row['scenario_sha256'])
                    collision=read(raw/'collision_history.json');summary=collision['summary']
                    recomputed=collision_window(collision['samples'],summary['window_start_monotonic'],summary['window_end_monotonic'])
                    require(summary.get('observed') is True and summary.get('no_collision_verified') is True
                        and not summary.get('non_ego_uids') and recomputed['observed'] and recomputed['no_collision_verified'],
                        'Paired approval lacks verified collision-clear native history')
            all_rows.append(row)
    require(first_ids==[entry['id'] for entry in paired['deck']['scenes']],
        'Paired initial observations must preserve all 120 preregistered identities and AB order')
    with tempfile.TemporaryDirectory(prefix='base5-stability-report-') as directory:
        report=read(stability_report.report(paired['manifest'],paths,Path(directory)/'report.json',cause_review=cause_review))
    require(report['observed_initial_scenes']==120 and not report['unobserved_registered_ids']
        and report['registered_scenes']==120 and len(report['pair_results'])==60
        and all(row['both_initial_observed'] for row in report['pair_results'])
        and all(report['initial_by_condition'][condition]['observed_initial_scenes']==60
            and report['initial_by_condition'][condition]['distinct_source_geometry_units']==60
            for condition in ('original','scaled')),'Paired report has missing or repeated independent source geometry')
    metrics=paired_cost(all_rows,paired['metadata']);blockers=stability_threshold_blockers(metrics)
    if any(row['status']=='failed' for row in all_rows):blockers.append('paired/failed_attempt_requires_review')
    if any(row.get('ordinary_deficit') is False for row in all_rows):
        blockers.append('paired/nonordinary_configuration_or_safety_failure')
    by_profile={}
    for name,profile in sorted(context['pairs']):
        ids={ident for pair in paired['paired'].values()
            if (name,profile) in smoke.stability.variants(paired['metadata'][pair['scaled']]) for ident in pair.values()}
        label=name+'/'+profile;value=paired_cost([row for row in all_rows if row['id'] in ids],paired['metadata'])
        require(value['scaled']['initial_scenes']>=3,'Paired native profile minimum lost')
        by_profile[label]=value;blockers.extend(stability_threshold_blockers(value,profile=label))
    causes={condition:dict(Counter(row['cause_classification'] for row in report['attempts']
        if row['attempt_ordinal']==0 and row['status']!='accepted_candidate'
        and paired['metadata'][row['id']]['scale_stability']['condition']==condition)) for condition in ('original','scaled')}
    if any(counts.get('safety',0) for counts in causes.values()):blockers.append('paired/safety_failure_requires_review')
    require(root_inventory=={str(path.resolve()):sha256(path) for path in root.rglob('*') if path.is_file()},
        'Paired native inventory changed during read-only validation')
    inventory.update(report['source_pins']);inventory.update(paired['input_hashes']);pins.verify_hashes(inventory)
    return dict(gate=smoke.STABILITY_GATE,metrics=metrics,profiles=by_profile,initial_failure_causes_by_condition=causes,
        independent_pairs=60,first_attempt_observations=120,raw_native_approvals_counted_as_final_smoke=0,
        source_manifest=str(paired['manifest']),source_manifest_sha256=sha256(paired['manifest']),
        evidence_hashes=inventory,recomputed_report=report),sorted(set(blockers))


def verify_cpu_regression(receipt,xml_paths,runtime):
    doc=read(receipt);paths=[Path(path).resolve(strict=True) for path in xml_paths]
    hashes={str(path):sha256(path) for path in paths}
    require(doc.get('schema')=='base5_final_cpu_regression_v1' and doc.get('helper_sha256')==sha256(smoke.__file__)
        and doc.get('runtime_hashes_before')==doc.get('runtime_hashes_after')==runtime
        and doc.get('test_hashes')==smoke.test_hashes() and doc.get('returncode')==0
        and doc.get('pytest_receipt_hashes')==hashes and len(paths)==len(hashes)==1
        and doc.get('fixed_exclusions')==smoke.CPU_EXCLUSIONS
        and doc.get('pytest_environment_controls')==smoke.CPU_ENVIRONMENT
        and doc.get('capture_started') is False and doc.get('training_started') is False
        and doc.get('gpu_execution') is False,'Current complete CPU execution/source/XML receipt required')
    require(doc.get('command')==smoke.cpu_command(paths[0]),'Only the complete fixed-scope current Base5 CPU suite qualifies')
    results=test_results(paths)
    require(doc.get('test_results')==results,'CPU receipt totals differ from actual unique JUnit cases')
    return results


def freeze(args):
    output=Path(args.output).resolve();capture=Path(args.capture_root).resolve(strict=True)
    require(not output.exists() and output!=capture and capture not in output.parents,'Fresh release receipt outside RAW required')
    before=snapshot(capture)
    context=smoke.registration_context(args.manifest,args.smoke_registration)
    require(output!=context['full'].parent and context['full'].parent not in output.parents,'Release cannot modify its source directory')
    native,blockers=verify_native(context,capture)
    paired,paired_blockers=verify_stability(context,args.stability_capture_root,args.stability_ledger,
        args.stability_cause_review,args.stability_session_manifest)
    blockers=sorted(set(blockers+paired_blockers))
    completed=verify_resume(args.completed_resume_receipt,capture,context['subset_path'])
    require(completed.get('requested')==len(context['selected']),'Completed replay targets another bounded smoke')
    partial=verify_partial(args.partial_snapshot,args.partial_resume_receipt,context['subset_path'],capture,
        [row['id'] for row in context['doc']['selected']])
    inputs=verify_inputs(args.training_input_report,args.loader_report,capture,native['accepted'])
    tests=verify_cpu_regression(args.cpu_regression_receipt,args.pytest_xml,context['runtime'])
    audit=audit_base5_scenarios.audit(context['full'])
    require(audit.get('status')=='OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes')==audit.get('unique_payloads')==5000,'Current exact full5K offline audit failed')
    require(snapshot(capture)==before,'Release validation changed native originals')
    refs={str(Path(path).resolve()):sha256(path) for path in [args.manifest,args.smoke_registration,
        args.completed_resume_receipt,args.partial_snapshot,args.partial_resume_receipt,args.training_input_report,
        args.loader_report,args.cpu_regression_receipt,*args.pytest_xml,__file__,context['subset_path']]}
    hashes=dict(context['doc']['input_hashes']);hashes.update(refs)
    hashes.update(inputs['evidence_hashes'])
    hashes.update(paired['evidence_hashes'])
    for key in ('qualification_evidence_hashes','support_evidence_hashes'):hashes.update(context['doc'][key])
    for relative,record in before['raw_files'].items():hashes[str(capture/relative)]=record['sha256']
    for relative,digest in before['event_logs'].items():hashes[str(capture/relative)]=digest
    hashes[str(capture/'selected_ledger.jsonl')]=before['selected_sha256']
    pins.verify_hashes(context['runtime']);pins.verify_hashes(hashes);scale.verify_frozen_policy(context['policy'])
    planned=Path(args.capture_output).resolve()
    require(planned!=capture and capture not in planned.parents,'Full5K output cannot overwrite native smoke')
    require(not planned.exists(),'Full5K output must be a new, unoccupied path')
    release=dict(schema=SCHEMA,qualification_schema='base5_final_bounded_qualification_v1',release_policy=POLICY,
        generated_at_utc=datetime.now(timezone.utc).isoformat(),bulk_ready=not blockers,bulk_blockers=blockers,
        bulk_started=False,training_started=False,completion_live_qualified=False,
        completion_qualification_scope='CPU recovery and finalization contracts only; no live automatic completion approval.',
        qualified_environments=['sunny13'],full_manifest=str(context['full']),full_manifest_sha256=sha256(context['full']),
        policy=context['deck']['config']['obstacle_support_policy'],runtime_hashes=context['runtime'],input_hashes=hashes,
        environments=dict(sunny13=dict(manifest=str(context['full']),manifest_sha256=sha256(context['full']),capture_output=str(planned))),
        operational_hours_range=[8,12],operational_storage_budget_gib=45.,
        smoke=native,training_inputs=inputs,cpu_tests=tests,partial_resume=partial,
        paired_stability=paired,
        completed_resume=dict(receipt_sha256=sha256(args.completed_resume_receipt),additional_native_captures=0),
        full_collector_preflight=dict(scenes=5000,actual_scale_admission=True,layout_source_pins=True,
            native_unit_reference=True,materialized_shadow_registry=True),
        separate_support_evidence=dict(hashes=context['doc']['support_evidence_hashes'],native_smoke_approvals_counted=0),
        limitations=['Only the preregistered exact current-source smoke counts as current smoke approvals.',
            'Paired stability, unit controls and old/native review history remain separate preserved evidence.',
            'No full5K capture, GPU/model/training/AP qualification or automatic --complete approval is claimed.'])
    atomic_json(output/'offline_audit.json',audit);atomic_json(output/'release.json',release)
    return output/'release.json'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','smoke-registration','capture-root','completed-resume-receipt','partial-snapshot',
                 'partial-resume-receipt','training-input-report','loader-report','capture-output','output','stability-capture-root',
                 'cpu-regression-receipt'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--pytest-xml',type=Path,action='append',required=True)
    parser.add_argument('--stability-ledger',type=Path,action='append',required=True)
    parser.add_argument('--stability-cause-review',type=Path)
    parser.add_argument('--stability-session-manifest',type=Path,action='append',default=[])
    print(freeze(parser.parse_args()))
