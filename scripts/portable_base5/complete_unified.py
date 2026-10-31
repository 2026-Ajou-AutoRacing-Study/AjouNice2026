#!/usr/bin/env python3
"""별도 출력에 통합 수집의 부족분만 보충한다. 기본은 읽기 전용 계획이다.

원형의 재배치·ordered roster·strict direction reference·native QA를 재사용한다.
기존 승인/실패/선택은 변경하지 않는다. --prepare는 geometry만 준비하고,
--run만 명시적으로 native 수집을 허용한다. quota 완화나 unsafe resume는 없다.
STOP/PAUSE/시간 예산은 현재 단일 셀의 native 안전 종료 이후 다음 셀 경계에서 적용한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import ExitStack, contextmanager
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time

import unified_capture as common
import recovery_population
import overlay_read_cache

POLICY = 'unified-terminal-exact-deficit-completion-v1'
TERMINAL = ('complete', 'complete_with_deficit')


def require(value, message):
    if not value:
        raise RuntimeError(message)


def read(path):
    return common.read(path)


def rows(path):
    content = Path(path).read_bytes()
    require(not content or content.endswith(b'\n'), 'Complete newline-terminated ledger required')
    return [json.loads(line) for line in content.splitlines() if line.strip()]


def atomic(path, value):
    path = Path(path)
    temporary = path.with_name('.'+path.name+'.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)


def atomic_rows(path, values):
    path = Path(path)
    body = ''.join(json.dumps(row, ensure_ascii=False)+'\n' for row in values)
    if path.exists() and path.read_text() == body:
        return
    temporary = path.with_name('.'+path.name+'.tmp')
    with temporary.open('w') as stream:
        stream.write(body); stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)


def summary(profile, output):
    doc = common.validate_profile(read(profile))
    baseline, output = common.physical(doc['output']), common.physical(output)
    require(output != baseline and output not in baseline.parents and baseline not in output.parents,
            'Completion must use a separate output, never its parent or child')
    status, quota = read(baseline/'status.json'), read(baseline/'quota.json')
    selected = rows(baseline/'selected_ledger.jsonl')
    deck = read(doc['manifest']); ids = [entry['id'] for entry in deck['scenes']]
    deficits = quota['cells']
    require(len(ids) == len(set(ids)) == deck['count'] == 5000,
            'Exact original 5K quota required')
    require(status.get('stage') in TERMINAL and status.get('error') is None
        and status.get('failed') == 0 and status.get('current') is None
        and status.get('finished') == status.get('checkpoint_finished') == len(ids)
        and status.get('accepted_candidate') == quota.get('accepted') == len(selected)
        and status.get('review') == quota.get('deficit') == len(deficits)
        and quota.get('pending') == 0 and quota.get('target') == len(ids),
        'Safe terminal parent with authoritative quota required')
    chosen = [row['source_cell_id'] for row in selected]
    missing = [row['id'] for row in deficits]
    require(len(chosen+missing) == len(set(chosen+missing)) == len(ids)
        and set(chosen+missing) == set(ids), 'Duplicate, missing or foreign quota slot')
    require(all(row.get('status') == 'accepted_candidate'
        and not row.get('audit', {}).get('review_reasons') for row in selected),
        'Reviewed raw cannot become an approval')
    pins = {str(baseline/name):common.sha(baseline/name) for name in
        ('status.json','quota.json','selected_ledger.jsonl','batch_contract.json',
         'static_attitude_batch_contract.json','imported_prefix.json','unified_live_qualification.json')}
    boundaries = list(baseline.glob('integrity_boundary_*.json'))
    require(boundaries and not list(baseline.glob('integrity_failed_*.json')),
            'Successful terminal fixed-input boundary evidence required')
    require(all(read(path).get('passed') is True for path in boundaries),
            'Terminal boundary failed')
    pins.update({str(path):common.sha(path) for path in boundaries})
    return doc, baseline, output, deck, selected, deficits, pins


def verify_baseline(doc, baseline, selected, deficits):
    """확정된 부모 선택의 계약만 확인한다. 기존 native QA를 재심사하지 않는다.

    기존 4,941장은 수집 당시 승인과 정상 종료를 신뢰하는 입력이다. 픽셀·
    포인트·full-RPY·가시성/bitrot 재감사는 학습 전달본 검증과 별도다.
    새 후보는 capture()/verify_attempt()의 실제 strict native QA를 거친다.
    """
    parent, direction = common.modules()
    boundary = common.boundary_for(doc)
    before = {name:common.sha(baseline/name) for name in
              ('status.json','quota.json','selected_ledger.jsonl','deficit_manifest.json')}
    try:
        with boundary.installed(scripts_dir=common.SCRIPTS, direction_module=direction):
            authority = common.source_authority(doc, parent)
            require(authority == doc['source_authority'], 'Original source authority changed')
            guard=read(baseline/'static_attitude_batch_contract.json')
            require(guard.get('schema') == parent.SCHEMA and guard.get('policy') == parent.POLICY
                and guard.get('manifest') == doc['manifest']
                and guard.get('manifest_sha256') == doc['manifest_sha256']
                and guard.get('environment') == doc['environment']
                and guard.get('indices') == list(range(5000))
                and guard.get('runtime_profile') == parent.profiles()
                and guard.get('runtime_hashes') == doc['runtime_hashes'] == parent.runtime_hashes()
                and all(doc['fixed_pins'].get(name) == checksum for name,checksum in guard['input_hashes'].items()),
                'Terminal parent source/runtime profile differs')
            expected_authority=dict(authority,unified_runtime_policy=common.POLICY,
                runtime_profile=parent.profiles(),original_source_qualification_only=True,
                new_output_handoff=True,live_profile_qualification_separate=True)
            require(guard['qualified_release'] == expected_authority, 'Original source qualification link changed')
            imported=baseline/'imported_prefix.json'
            require(common.sha(imported) == doc['import_sha256'], 'Original imported prefix changed')
            prefix=read(imported)
            require(prefix.get('schema') == 1 and prefix.get('manifest_sha256') == doc['manifest_sha256']
                and prefix.get('original_output') == doc['old_output'], 'Imported source lineage differs')
            chosen={row['source_cell_id']:row for row in selected}
            missing={row['id']:row for row in deficits}
            require(all(chosen.get(row['source_cell_id']) == row for row in prefix['selected'])
                and all(missing.get(row['id']) == row for row in prefix['deficits']),
                'Historical approval/deficit was changed during parent handoff')
            expected_batch=dict(guard['legacy_batch_contract'],imported_prefix_sha256=doc['import_sha256'],
                import_driver_sha256=common.sha(common.SCRIPTS/'base5_campaign_import.py'))
            require(read(baseline/'batch_contract.json') == expected_batch, 'Terminal parent batch/import contract differs')
            qualified=read(baseline/'unified_live_qualification.json')
            require(qualified.get('passed') is True and qualified.get('policy') == common.POLICY
                and qualified.get('environment') == doc['environment']
                and qualified.get('manifest_sha256') == doc['manifest_sha256']
                and qualified.get('helper_pins') == doc['helper_pins']
                and qualified.get('actual_pause_resume') is True
                and qualified.get('completed_replay_no_new_capture') is True
                and common.sha(qualified['pilot_manifest']) == qualified['pilot_manifest_sha256'],
                'Original common runtime qualification differs')
            for name,checksum in qualified['proof_hashes'].items():
                require(common.sha(name) == checksum, 'Original five-cell runtime qualification evidence changed')
            for path in baseline.glob('integrity_boundary_*.json'):
                proof=read(path)
                require(proof.get('policy') == common.POLICY and proof.get('result') == 0
                    and proof.get('passed') is True, 'Original parent did not exit its input boundary normally')
            # Seed the existing delta input graph once, without parent/native
            # replay. Fresh candidate scopes reuse this registered source map.
            common.warm_boundary(boundary,doc,direction)
    finally:
        boundary.verify()
    require(all(common.sha(baseline/name) == digest for name,digest in before.items()),
            'Read-only parent verifier changed historical selection')
    print(json.dumps(dict(event='unified_completion_parent_contract_verified',approved=len(selected),
        deficit=len(deficits),existing_native_qa_recomputed=False),ensure_ascii=False),flush=True)
    return boundary


def prior_payloads(baseline, deck):
    """보존된 조상·실패 native의 payload도 회피한다. 품질 판정을 재해석하지 않는다."""
    known = {entry['scenario_sha256'] for entry in deck['scenes']}
    current, seen = baseline, set()
    while True:
        require(current not in seen and len(seen) < 16, 'Cyclic or excessive parent ancestry')
        seen.add(current)
        for path in current.rglob('ledger.jsonl'):
            for row in rows(path):
                digest = row.get('scenario_sha256')
                if digest:
                    require(isinstance(digest,str) and len(digest) == 64
                        and set(digest) <= set('0123456789abcdef'), 'Malformed prior payload digest')
                    known.add(digest)
        imported = current/'imported_prefix.json'
        if not imported.exists():
            break
        current = common.physical(read(imported)['original_output'])
    return known


def identity(meta):
    return dict(spec=meta['spec'], split=meta['split'], group=meta.get('scene_group_id'),
        actors=[{key:copy.deepcopy(actor.get(key)) for key in
            ('unique_id','data_id','name','source_list','motion','asset_scale')}
                for actor in meta['runtime_actors']],
        unit=meta.get('native_unit_geometry_reference'),
        environment=meta.get('fresh_environment_provenance'))


def validate_proposal(original, manifest, known):
    deck = read(manifest)
    require(deck['count'] == len(deck['scenes']) == 1, 'One new same-quota proposal required')
    entry = deck['scenes'][0]
    directory=common.physical(manifest).parent
    require(all(common.inside(common.physical(entry[key]),directory) for key in ('scenario','metadata')),
            'New scenario/metadata payload must remain inside its own placement directory')
    require(common.sha(entry['scenario']) == entry['scenario_sha256']
        and entry['scenario_sha256'] not in known, 'Never repeat a prior or changed native payload')
    require(identity(original) == identity(read(entry['metadata'])),
            'Proposal changed frozen spec/ordered actors/motion/split/group/unit/environment')
    return entry


def validate_state(state, output, deck, selected, deficits, attempt_cap):
    """Durable journal is not arbitrary authority for a new load or a path."""
    require(state.get('source_order') == [entry['id'] for entry in deck['scenes']]
        and state.get('deficits') == [entry['id'] for entry in deficits]
        and state.get('baseline_selected') == selected,
        'Completion journal changed original selection/quota order')
    lookup={entry['id']:entry for entry in deck['scenes']}; seen=set()
    initial=set(state.get('initial_known_payloads',[]))
    for attempt in state.get('attempts',[]):
        ident, ordinal = attempt.get('id'), attempt.get('ordinal')
        require(ident in state['deficits'] and type(ordinal) is int and 1 <= ordinal <= attempt_cap
            and (ident,ordinal) not in seen, 'Foreign, duplicate or out-of-budget attempt identity')
        seen.add((ident,ordinal))
        require('/' not in ident and ident not in ('.','..'), 'Unsafe quota directory identity')
        folder=output/'cells'/ident/f'attempt_{ordinal}'
        require(common.physical(attempt.get('child','')) == folder/'campaign',
                'Attempt child escaped its exact registered output')
        if attempt.get('manifest') is None:
            require(attempt.get('unavailable') or state.get('stage') in ('preparing','halted'),
                    'Missing prepared proposal requires explicit offline inspection')
            continue
        require(common.physical(attempt['manifest']) == folder/'placement'/'manifest.json'
            and common.sha(attempt['manifest']) == attempt.get('manifest_sha256'),
                'Attempt manifest escaped its exact registered source or changed')
        entry=validate_proposal(read(lookup[ident]['metadata']),Path(attempt['manifest']),initial)
        require(entry['scenario_sha256'] == attempt.get('scenario_sha256'), 'Recorded proposal hash changed')
        initial.add(entry['scenario_sha256'])


def generate(entry, deck, known, folder, previous=None):
    parent, _ = common.modules()
    import base5_spatial_diversity as spatial
    quality = parent.batch.campaign.loop
    folder.mkdir(parents=True, exist_ok=False)
    seed = folder/'seed_manifest.json'
    source = read(previous) if previous else parent.batch.campaign.subset_deck(deck,[entry])
    atomic(seed, source)
    with spatial.installed(), recovery_population.installed(quality):
        result = quality.replacement(seed, folder/'placement',
            require_ground_observable=True, known_payloads=set(known))
    validate_proposal(read(entry['metadata']), result, known)
    return result


def merged(state):
    chosen = {row['source_cell_id']:copy.deepcopy(row) for row in state['baseline_selected']}
    for attempt in state['attempts']:
        require(attempt['id'] in state['deficits'] and attempt['id'] in state['source_order'],
                'Attempt refers to a foreign or originally approved quota')
        row = (attempt.get('result') or {}).get('selected')
        if row is None:
            continue
        require(attempt['id'] not in chosen, 'Duplicate selected quota')
        row = copy.deepcopy(row)
        row.update(source_cell_id=attempt['id'], unified_completion=dict(policy=POLICY,
            source_manifest=attempt['manifest'], source_manifest_sha256=attempt['manifest_sha256'],
            campaign=attempt['child'], attempt=attempt['ordinal']))
        chosen[attempt['id']] = row
    ordered = [chosen[ident] for ident in state['source_order'] if ident in chosen]
    missing = [ident for ident in state['deficits'] if ident not in chosen]
    require(len(ordered)+len(missing) == len(state['source_order']), 'Selection reconstruction lost a quota')
    return ordered, missing


def publish(output, state, deck):
    selected, missing = merged(state)
    atomic(output/'state.json', state)
    atomic_rows(output/'selected_ledger.jsonl', selected)
    atomic(output/'quota.json', dict(target=len(state['source_order']),accepted=len(selected),
        deficit=len(missing),pending=0,cells=[dict(id=ident,reason='strict_same_quota_completion_pending') for ident in missing]))
    atomic(output/'status.json', dict(stage=state['stage'],error=state.get('error'),
        selected_not_certified=state.get('selected_not_certified',False),requested=len(state['source_order']),
        finished=len(state['source_order']),checkpoint_finished=len(state['source_order']),checkpoint_only=True,
        accepted_candidate=len(selected),review=len(missing),failed=int(state['stage']=='halted'),
        initial_accepted=len(state['baseline_selected']),replenished=len(selected)-len(state['baseline_selected']),
        attempts=len(state['attempts']),current=state.get('current'),updated_at=time.time(),eta_sec=0,
        elapsed_sec=state['baseline_elapsed_sec']+state.get('elapsed_sec',0)))
    lookup={entry['id']:entry for entry in deck['scenes']}
    from run_base5_fresh_completion import adopted_entry
    parent, _ = common.modules()
    adopted=parent.batch.campaign.subset_deck(deck,[adopted_entry(row,lookup) for row in selected])
    adopted.update(layout_split_finalization_pending=bool(deck.get('split_overlay')),bulk_ready=False,
        unified_completion=dict(policy=POLICY,parent=state['contract']['baseline'],exact_quota_preserved=True,
            native_qa_unchanged=True,derived_layout_requires_finalization=True))
    atomic(output/'adopted_manifest.json', adopted)


def verify_attempt(attempt, original):
    parent, direction = common.modules()
    manifest, child = Path(attempt['manifest']), Path(attempt['child'])
    require(common.sha(manifest) == attempt['manifest_sha256'], 'Attempt source changed')
    parent.batch.verify_recovery(manifest, 0, child)
    direction.verify_campaign(child, manifest, [0])
    selected = rows(child/'selected_ledger.jsonl')
    require(len(selected) <= 1, 'One native approval at most per quota')
    if selected:
        proof=direction.verify_completed_campaign(manifest,0,child)
        row=proof['accepted_row']
        require(row == selected[0] and identity(read(Path(row['capture_root'])/'metadata.json')) == identity(original),
                'Selected native changed source quota')
        selected=row
    else:
        selected=None
    return dict(selected=selected,stage=read(child/'status.json')['stage'],
        quota_sha256=common.sha(child/'quota.json'),ledger_sha256=common.sha(child/'selected_ledger.jsonl'))


@contextmanager
def nonrepeat(known):
    """Nested strict QA also avoids historical failed native payloads."""
    parent, _ = common.modules()
    quality = parent.batch.campaign.loop
    saved = quality.replacement
    def replacement(*args, **kwargs):
        kwargs['known_payloads'] = set(kwargs.get('known_payloads') or ()) | set(known)
        return saved(*args, **kwargs)
    quality.replacement = replacement
    try:
        yield
    finally:
        changed = quality.replacement is not replacement
        quality.replacement = saved
        require(not changed, 'Nonrepeat replacement adapter ownership changed')


@contextmanager
def inherited_native(direction, known):
    """Pass retained history into original quality's pre-capture SHA guard."""
    saved=direction.compiled_execute
    def compile_with_history(*args,**kwargs):
        execute=saved(*args,**kwargs)
        def guarded(*values,**options):
            options['known_payloads']=set(options.get('known_payloads') or ()) | set(known)
            return execute(*values,**options)
        return guarded
    direction.compiled_execute=compile_with_history
    try:
        yield
    finally:
        changed=direction.compiled_execute is not compile_with_history
        direction.compiled_execute=saved
        require(not changed,'Direction nonrepeat compiler adapter ownership changed')


def capture(attempt, output, original, known=()):
    _, direction = common.modules()
    child=Path(attempt['child'])
    if child.exists():
        require((child/'status.json').exists() and read(child/'status.json').get('stage') in TERMINAL,
                'Interrupted/unsafe child requires explicit safety inspection, no automatic recapture')
        return verify_attempt(attempt,original)
    args=argparse.Namespace(enable_final_direction_reference=True,approve_bulk_capture=False,
        manifest=Path(attempt['manifest']),output=child,indices='0',limit=1,resume=True,
        quota_rounds=2,seed_capture=None)
    # The current fresh proposal is allowed once. All other retained sources,
    # prepared candidates and actual historical native payloads stay forbidden.
    inherited=set(known)-{attempt['scenario_sha256']}
    with recovery_population.installed(), nonrepeat(known), inherited_native(direction,inherited):
        code=direction.run(args)
    require(code == 0, 'Strict native child failed; no next MORAI load')
    return verify_attempt(attempt,original)


def execute(args):
    require(1 <= args.attempts <= 3 and 30 <= args.time_budget_sec <= 3600,
            'Finite 1..3 exact attempts and 30..3600 second budget required')
    require(args.stop_after_cells >= 0, 'Nonnegative cell limit required')
    doc, baseline, output, deck, selected, deficits, baseline_pins = summary(args.profile,args.output)
    if args.run:
        require(args.confirmed_environment == doc['environment'], 'Explicit matching MORAI UI environment required')
    else:
        require(args.confirmed_environment is None, 'Environment confirmation only belongs to --run')
    if not args.run and not args.prepare and not args.verify_baseline:
        return dict(event='unified_completion_plan',baseline=str(baseline),output=str(output),
            approved=len(selected),deficit=len(deficits),quota_relaxation=False,
            capture_started=False,files_written=False,pause_semantics='after current native cell safety boundary')
    # The original verifier acquires the historical parent's own flock files.
    # Do not hold a second open-file-description flock around that call.
    boundary=verify_baseline(doc,baseline,selected,deficits)
    with common.handoff.idle_locks(baseline):
        require(all(common.sha(name) == digest for name,digest in baseline_pins.items()),
                'Historical parent changed between verification and lock acquisition')
        if args.verify_baseline and not args.run and not args.prepare:
            return dict(event='unified_completion_baseline_verified',approved=len(selected),
                deficit=len(deficits),files_written=False,capture_started=False)
        output.mkdir(parents=True,exist_ok=True)
        with (output/'.completion.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            code_pins={str(Path(__file__).absolute()):common.sha(__file__),
                str(Path(overlay_read_cache.__file__).absolute()):common.sha(overlay_read_cache.__file__),
                **common.helpers()}
            contract=dict(policy=POLICY,baseline=str(baseline),profile=str(common.physical(args.profile)),
                profile_sha256=common.sha(args.profile),baseline_pins=baseline_pins,code_pins=code_pins,
                attempts=args.attempts,source_sha256=doc['manifest_sha256'])
            path=output/'state.json'
            if path.exists():
                state=read(path)
                require(state['contract'] == contract, 'Completion source/runtime/cumulative attempt contract changed')
                require(state['stage'] != 'halted', 'Halted completion needs safety diagnosis, no forced resume')
                require(state['initial_known_payloads'] == sorted(prior_payloads(baseline,deck)),
                        'Initial nonrepeat inventory changed or was weakened')
                validate_state(state,output,deck,selected,deficits,args.attempts)
                for attempt in state['attempts']:
                    if attempt.get('result'):
                        require(verify_attempt(attempt,read(next(e['metadata'] for e in deck['scenes'] if e['id']==attempt['id']))) == attempt['result'],
                                'Completed native attempt evidence changed')
                require(merged(state)[0] == rows(output/'selected_ledger.jsonl'), 'Published merged selection differs')
            else:
                require(not any(output.iterdir()) or list(output.iterdir()) == [output/'.completion.lock'],
                        'Refuse unregistered completion files')
                state=dict(contract=contract,stage='prepared',baseline_selected=selected,
                    deficits=[entry['id'] for entry in deficits],source_order=[entry['id'] for entry in deck['scenes']],
                    baseline_elapsed_sec=read(baseline/'status.json')['elapsed_sec'],elapsed_sec=0.,attempts=[],current=None,
                    initial_known_payloads=sorted(prior_payloads(baseline,deck)))
                publish(output,state,deck)
            if state['stage'] in TERMINAL:
                return dict(event='unified_completion_already_terminal',approved=len(merged(state)[0]),
                    deficit=len(merged(state)[1]),capture_started=False)
            start=time.monotonic(); prior=state['elapsed_sec']; processed=0
            lookup={entry['id']:entry for entry in deck['scenes']}
            known=set(state['initial_known_payloads']) | {a['scenario_sha256'] for a in state['attempts'] if a.get('scenario_sha256')}
            for completed in state['attempts']:
                if completed.get('result') is not None:
                    known.update(prior_payloads(Path(completed['child']), dict(scenes=[])))
            try:
                with overlay_read_cache.installed(exclude_roots=(output,)), \
                        boundary.installed(scripts_dir=common.SCRIPTS,direction_module=common.modules()[1],
                        delta_input_manifest=doc['manifest']), common.route_compatibility(doc):
                    common.warm_boundary(boundary,doc,common.modules()[1])
                    for ordinal in range(1,(args.attempts if args.run else 1)+1):
                        for ident in state['deficits']:
                            if ident not in merged(state)[1]:
                                continue
                            attempt=next((a for a in state['attempts'] if a['id']==ident and a['ordinal']==ordinal),None)
                            if attempt and (attempt.get('result') is not None or attempt.get('unavailable')):
                                continue
                            if (time.monotonic()-start >= args.time_budget_sec
                                or any((output/name).exists() for name in ('STOP','PAUSE'))
                                or (args.stop_after_cells and processed >= args.stop_after_cells)):
                                state.update(stage='paused',current=None,elapsed_sec=prior+time.monotonic()-start)
                                publish(output,state,deck)
                                return dict(event='unified_completion_paused',approved=len(merged(state)[0]),deficit=len(merged(state)[1]))
                            state['current']=ident
                            if attempt is None:
                                history=[a for a in state['attempts'] if a['id']==ident and a.get('manifest')]
                                previous=history[-1]['manifest'] if history else None
                                folder=output/'cells'/ident/f'attempt_{ordinal}'
                                attempt=dict(id=ident,ordinal=ordinal,manifest=None,child=str(folder/'campaign'),result=None)
                                state['attempts'].append(attempt)
                                state['stage']='preparing';publish(output,state,deck)
                                try:
                                    manifest=generate(lookup[ident],deck,known,folder,previous)
                                except common.modules()[0].batch.campaign.loop.PlacementProposalExhausted as error:
                                    attempt['unavailable']=str(error);processed+=1;publish(output,state,deck);continue
                                entry=read(manifest)['scenes'][0]
                                attempt.update(manifest=str(manifest),manifest_sha256=common.sha(manifest),
                                    scenario_sha256=entry['scenario_sha256'])
                                known.add(entry['scenario_sha256']);publish(output,state,deck)
                            require(attempt.get('manifest') is not None, 'Interrupted offline proposal needs inspection')
                            if args.prepare and not args.run:
                                processed+=1
                                continue
                            state['stage']='running';publish(output,state,deck)
                            attempt['result']=capture(attempt,output,read(lookup[ident]['metadata']),known)
                            known.update(prior_payloads(Path(attempt['child']), dict(scenes=[])))
                            state['elapsed_sec']=prior+time.monotonic()-start
                            publish(output,state,deck);processed+=1
                            print(json.dumps(dict(event='unified_completion_cell',id=ident,attempt=ordinal,
                                accepted=bool(attempt['result']['selected']),remaining=len(merged(state)[1])),ensure_ascii=False),flush=True)
                    state.update(stage=('prepared' if not args.run else 'complete_with_deficit' if merged(state)[1] else 'complete'),
                        current=None,elapsed_sec=prior+time.monotonic()-start)
                    publish(output,state,deck)
            except BaseException as error:
                state.update(stage='halted',error=str(error),current=None,selected_not_certified=True,
                    elapsed_sec=prior+time.monotonic()-start)
                publish(output,state,deck)
                raise
            finally:
                try:
                    boundary.verify()
                    require(all(common.sha(name) == digest for name,digest in code_pins.items()),
                            'Completion helper source changed during the session')
                    require(all(common.sha(name) == digest for name,digest in baseline_pins.items()),
                            'Historical parent changed during completion')
                except BaseException as integrity_error:
                    # Published rows remain raw evidence, never certified success
                    # after a failed terminal boundary. Avoid adopted regeneration
                    # through code/source that just failed the integrity check.
                    state.update(stage='halted',error='Completion integrity boundary failed: '+str(integrity_error),
                        current=None,selected_not_certified=True)
                    atomic(output/'state.json',state)
                    status=read(output/'status.json')
                    atomic(output/'status.json',dict(status,stage='halted',error=state['error'],failed=1,
                        current=None,selected_not_certified=True,updated_at=time.time()))
                    atomic(output/('integrity_failed_'+str(time.time_ns())+'.json'),dict(policy=POLICY,
                        passed=False,error=str(integrity_error),selected_not_certified=True))
                    raise
            return dict(event='unified_completion_result',stage=state['stage'],approved=len(merged(state)[0]),
                deficit=len(merged(state)[1]),quota_relaxation=False,output=str(output))


def parser():
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('--profile',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--prepare',action='store_true');mode.add_argument('--run',action='store_true')
    p.add_argument('--verify-baseline',action='store_true')
    p.add_argument('--confirmed-environment',choices=common.ENVIRONMENTS)
    p.add_argument('--attempts',type=int,default=2);p.add_argument('--time-budget-sec',type=int,default=3600)
    p.add_argument('--stop-after-cells',type=int,default=0)
    return p


if __name__ == '__main__':
    try:
        print(json.dumps(execute(parser().parse_args()),ensure_ascii=False),flush=True)
    except (OSError,ValueError,RuntimeError) as error:
        print(json.dumps(dict(event='unified_completion_blocked',error=str(error),fallback_allowed=False),ensure_ascii=False),file=sys.stderr)
        raise SystemExit(1)
