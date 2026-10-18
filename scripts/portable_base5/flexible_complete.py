#!/usr/bin/env python3
"""보존된 승인분에 같은 학습 목적의 새 장면을 더한다. 기본은 읽기 전용 계획.

정확한 asset/개수/도로/방향/거리 슬롯을 복원하지 않는다. 패밀리·미션·split,
실제 동작 종류와 근거리 장애물/80~100m 차량 목적은 보존한다. native QA,
충돌·정지·P 및 통신 계약은 완화하지 않는다. --run만 실제 취득을 허용한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import ast
from collections import Counter
from contextlib import contextmanager
import copy
import fcntl
import hashlib
import inspect
from pathlib import Path
import random
import textwrap
import time

import complete_unified as exact

common = exact.common
read, rows, atomic, atomic_rows, require = exact.read, exact.rows, exact.atomic, exact.atomic_rows, exact.require
POLICY = 'same-family-purpose-fresh-donor-completion-v1'
SAFE_STAGES = ('paused', 'prepared', 'complete', 'complete_with_deficit')
RELAXED = ('exact_primary_asset', 'ordered_actor_roster', 'actor_count', 'exact_distance_cell',
           'role', 'yaw_limit', 'road_position', 'historical_unobserved_source_train_floors')


def moving_modes(meta):
    return sorted({a['motion'] for a in meta['runtime_actors'] if a['motion'] not in ('static', 'standing')})


def distance(actor):
    import math
    center = actor.get('geometric_center_estimate')
    ego = actor.get('ego_relative_center')
    if ego is not None:
        return math.hypot(ego[0], ego[1])
    return actor.get('planned_distance_m')


def primary_range(meta):
    bounds = meta['spec'].get('distance_bounds') or (0, 100)
    value = meta.get('planned_primary_distance_m')
    if value is None:
        value = sum(bounds)/2
    return float(value)


def far_purpose(meta):
    plan = meta['spec'].get('secondary_far_vehicle')
    tagged_far = bool(plan and min(plan.get('distance_bounds_m', plan.get('distance_bounds', (0,)))) >= 80)
    return tagged_far or (meta['spec']['family'] in ('highway', 'other_vehicle') and primary_range(meta) >= 80)


def range_purpose(meta):
    if far_purpose(meta):
        return 'vehicle_80_100m'
    d = primary_range(meta)
    if d < 20:
        return 'near_0_20m'
    if d <= 30:
        return 'near_20_30m'
    if d < 80:
        return 'middle_30_80m'
    return 'far_80_100m'


def compatible(source, candidate, pools):
    a, b = source['spec'], candidate['spec']
    if (a['family'], bool(a['mission']), source['split']) != (b['family'], bool(b['mission']), candidate['split']):
        return False
    aa, bb = source['runtime_actors'], candidate['runtime_actors']
    if not aa or not bb or pools[aa[0]['data_id']] != pools[bb[0]['data_id']]:
        return False
    if moving_modes(source) != moving_modes(candidate):
        return False
    if range_purpose(source) != range_purpose(candidate):
        return False
    # A far purpose must remain protected by a native far-secondary gate or
    # the native primary visibility gate. An untagged distant extra is not enough.
    if far_purpose(source) and not far_purpose(candidate):
        return False
    return not any(a.get('asset_scale') is not None for a in bb)


def donor_order(source, donors, pools, ident, ordinal, used):
    eligible = [(entry, meta) for entry, meta in donors if compatible(source, meta, pools)
                and entry['scenario_sha256'] not in used]
    primary = source['runtime_actors'][0]['data_id']
    seed = int(hashlib.sha256(f'{POLICY}:{ident}:{ordinal}'.encode()).hexdigest()[:16], 16)
    rng = random.Random(seed)
    rng.shuffle(eligible)
    # Prefer a different observed-success primary and fewer mandatory actors.
    # This does not waive any visibility threshold on the chosen new scene.
    window = {'near_0_20m':(0,20),'near_20_30m':(20,30),
        'middle_30_80m':(30,80),'vehicle_80_100m':(80,100),'far_80_100m':(80,100)}[range_purpose(source)]
    def stable(meta):
        plan = meta['spec'].get('secondary_far_vehicle') if far_purpose(meta) else None
        bounds = plan.get('distance_bounds_m') if plan else meta['spec'].get('distance_bounds')
        return bool(bounds and window[0] <= bounds[0] <= bounds[1] <= window[1])
    eligible.sort(key=lambda item: (not stable(item[1]),len(item[1]['runtime_actors']),
                                   item[1]['runtime_actors'][0]['data_id'] == primary))
    return eligible


def summary(profile, parent, output):
    doc, baseline, _, deck, original, deficits, pins = exact.summary(profile, output)
    parent, output = common.physical(parent), common.physical(output)
    require(parent != output and parent not in output.parents and output not in parent.parents,
            'Flexible output must be independent of its preserved parent')
    status, state = read(parent/'status.json'), read(parent/'state.json')
    require(status.get('stage') in SAFE_STAGES and state.get('stage') == status['stage']
        and not status.get('error') and not status.get('selected_not_certified')
        and status.get('failed') == 0 and status.get('current') is None,
        'Safely paused/terminal parent required; never bypass halted/Park uncertainty')
    require(state['contract']['baseline'] == str(baseline)
        and state['contract']['source_sha256'] == doc['manifest_sha256']
        and state['baseline_selected'] == original,
        'Completion parent must preserve this exact original approved prefix')
    selected, missing = exact.merged(state)
    require(selected == rows(parent/'selected_ledger.jsonl')
        and status['accepted_candidate'] == len(selected) and status['review'] == len(missing)
        and read(parent/'quota.json')['accepted'] == len(selected), 'Parent snapshot counts/selection differ')
    require(not list(parent.glob('integrity_failed_*.json')), 'Parent integrity failure is not an ordinary deficit')
    for attempt in state['attempts']:
        require(attempt.get('result') is not None or attempt.get('unavailable'),
                'Every previously started candidate must have ended at its native safety boundary')
    pins.update({str(parent/name):common.sha(parent/name) for name in
                 ('state.json','status.json','selected_ledger.jsonl','quota.json','adopted_manifest.json')})
    require(all(common.sha(path) == sha for path, sha in state['contract']['baseline_pins'].items()),
            'Trusted original parent snapshot changed')
    require(all(common.sha(path) == sha for path, sha in state['contract']['code_pins'].items()),
            'Trusted exact-completion implementation changed')
    return doc, baseline, parent, output, deck, selected, missing, pins


def rebind_clone(layout):
    """Relax exactly the two placement-contract gates and the obsolete floor.

    No native/audit function is transformed. Immutable source, payload, group,
    alias, leakage and current geometry checks remain the original statements.
    Existing approved scenes are an append-only prefix, so replacing a failed
    original proposal cannot remove any already observed training coverage.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(layout.register_recovery_overlay)))
    fn = tree.body[0]
    removed = []
    for node in list(fn.body):
        if not isinstance(node, ast.If):
            continue
        message = None
        for child in ast.walk(node):
            if isinstance(child, ast.Raise) and isinstance(child.exc, ast.Call) and child.exc.args:
                value = child.exc.args[0]
                if isinstance(value, ast.Constant):
                    message = value.value
        if message in ('Recovery changed frozen family/range/phase/yaw contract',
                       'Recovery changed actor identity/motion/scale roster'):
            fn.body.remove(node)
            removed.append(message)
    require(len(removed) == 2, 'Original layout contract changed; flexible AST must be reviewed')
    # Keep audit errors fatal. Only the original failed-source count floors are
    # irrelevant: no successful baseline observation is removed or overwritten.
    changes = 0
    for node in ast.walk(fn):
        if isinstance(node, ast.If) and isinstance(node.test, ast.BoolOp) and isinstance(node.test.op, ast.Or):
            if ast.dump(node.test) == ast.dump(ast.parse("audit['errors'] or violated",mode='eval').body):
                node.test = ast.parse("audit['errors']", mode='eval').body
                changes += 1
    require(changes == 1, 'Original layout audit changed; preserve audit errors')
    scope = dict(vars(layout))
    ast.fix_missing_locations(tree)
    exec(compile(tree, str(Path(__file__).absolute())+'::explicit_flexible_rebind', 'exec'), scope)
    return scope['register_recovery_overlay']


@contextmanager
def flexible_rebinding(pools):
    common.modules()
    import base5_layout_split as layout
    original = layout.register_recovery_overlay
    clone = rebind_clone(layout)
    def register(source_entry, source_meta, candidate, proposed, output):
        if not compatible(source_meta, proposed, pools):
            raise common.modules()[0].batch.campaign.loop.PlacementProposalExhausted(
                'Recovery layout leakage/coverage changed: Flexible geometry crossed its family/motion/split/distance purpose; choose another donor')
        proof = dict(policy=POLICY, relaxed_fields=list(RELAXED),
            portable_adapter_sha256=common.sha(__file__), frozen_layout_implementation_sha256=common.sha(layout.__file__),
            appended_to_approved_prefix=True, native_qa_unchanged=True)
        proposed['flexible_completion_policy'] = proof
        # The original writes source_metadata before returning; include explicit
        # semantics in that immutable planned core and its content binding.
        try:
            reference = clone(source_entry, source_meta, candidate, proposed, output)
        except ValueError as error:
            if str(error).startswith(('Recovery duplicates another actual layout; propose distinct geometry',
                                      'Recovery layout leakage/coverage changed:')):
                raise common.modules()[0].batch.campaign.loop.PlacementProposalExhausted(str(error)) from error
            raise
        document = read(reference['path'])
        document['audit']['flexible_completion_policy'] = proof
        document['flexible_recovery_policy'] = proof
        document.pop('overlay_sha256')
        document['overlay_sha256'] = layout.digest(document)
        atomic(reference['path'], document)
        reference.update(sha256=common.sha(reference['path']), overlay_sha256=document['overlay_sha256'])
        return reference
    layout.register_recovery_overlay = register
    try:
        yield
    finally:
        changed = layout.register_recovery_overlay is not register
        layout.register_recovery_overlay = original
        require(not changed, 'Flexible layout adapter ownership changed')


def donor_pool(deck, selected):
    parent, _ = common.modules()
    gen = parent.batch.campaign.loop.gen
    ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
    semantic_pools = ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider')
    pools = {a['data_id']:name for name in semantic_pools for a in ctx['pools'][name]}
    source_entries = {entry['id']:entry for entry in deck['scenes']}
    donors = []
    for row in selected:
        raw = Path(row['capture_root'])
        meta = read(raw/'metadata.json')
        if not meta.get('runtime_actors'):
            continue
        entry = dict(id=row['id'],source_cell_id=row['id'],scenario=str(raw/'scenario.json'),
            scenario_sha256=row['scenario_sha256'],metadata=str(raw/'metadata.json'),split=meta['split'],
            family=meta['spec']['family'],mission=meta['spec']['mission'],role=meta['spec']['role'],
            distance_bounds=meta['spec']['distance_bounds'],scene_group_id=meta['scene_group_id'])
        source = source_entries[row['source_cell_id']]
        entry.update(derived_training_split=source['derived_training_split'],
                     derived_layout_group_id=source['derived_layout_group_id'])
        donors.append((entry, meta))
    return donors, pools


def generate(source, original, donor, donor_meta, deck, folder, known, ordinal):
    parent, _ = common.modules()
    import base5_spatial_diversity as spatial
    quality = parent.batch.campaign.loop
    folder.mkdir(parents=True, exist_ok=False)
    seed_entry = copy.deepcopy(donor)
    seed_meta = copy.deepcopy(donor_meta)
    if donor_meta.get('training_split_overlay'):
        # RAW metadata carries runtime diagnostics. The layout validator owns
        # the immutable planned core, not those diagnostic additions.
        tag = donor_meta['training_split_overlay']
        require(common.sha(tag['source_metadata_path']) == tag['source_metadata_sha256'],
                'Observed-success donor planned metadata changed')
        seed_meta = read(tag['source_metadata_path'])
        seed_meta['training_split_overlay'] = copy.deepcopy(tag)
        require(exact.identity(seed_meta) == exact.identity(donor_meta),
                'Donor native/planned spec, roster, unit, split or group differs')
        sidecar = folder/'donor_planned_metadata.json'
        atomic(sidecar,seed_meta)
        seed_entry['metadata'] = str(sidecar)
    seed_deck = parent.batch.campaign.subset_deck(deck, [seed_entry])
    if donor_meta.get('training_split_overlay'):
        seed_deck['split_overlay'] = dict(donor_meta['training_split_overlay'], policy='known-map-layout-group-v156')
    seed = folder/'seed_manifest.json'
    atomic(seed, seed_deck)
    with spatial.installed(), exact.recovery_population.installed(quality):
        proposed = quality.replacement(seed,folder/'donor_proposal',require_ground_observable=True,
                                      known_payloads=set(known))
    generated = read(proposed)['scenes'][0]
    meta = read(generated['metadata'])
    scene = read(generated['scenario'])
    # The observation has its own explicit spec. Only its logical quota slot
    # and effective split/group are mapped back to the failed original source.
    for key in ('quality_replacement_contract','training_split_overlay'):
        meta.pop(key, None)
    donor_provenance = {key:copy.deepcopy(read(generated['metadata']).get(key)) for key in
                        ('quality_replacement_contract','replacement','fresh_completion_source')}
    meta['flexible_donor_proposal_provenance'] = dict(donor_entry=donor,
        donor_metadata_sha256=common.sha(donor['metadata']),proposal_manifest=str(proposed),
        proposal_manifest_sha256=common.sha(proposed),original_lineage=donor_provenance)
    meta['scene_group_id'] = original['scene_group_id']
    meta['environment_split_contract'] = dict(environment=deck['config']['environment_tag'],
        scene_group_id=original['scene_group_id'],split=original['split'],policy=POLICY,
        limitation='Different fresh scene; not exact original asset/count/range/role quota')
    meta['fresh_completion_source'] = dict(source_cell_id=source['id'],
        source_scenario_sha256=source['scenario_sha256'],ordinal=ordinal,flexible_policy=POLICY,
        portable_adapter_sha256=common.sha(__file__))
    meta['quality_replacement_contract'] = dict(root_entry=copy.deepcopy(source),
        root_metadata_sha256=common.sha(source['metadata']),ordinal=ordinal,
        immediate_parent_entry=copy.deepcopy(source),immediate_parent_metadata_sha256=common.sha(source['metadata']),
        prior_proposal_sha256=sorted(set(known)|{source['scenario_sha256']}),
        flexible_asset_roster_change=True,actor_identity_map=None,
        layout_split_finalization_pending=True,raw_gt_or_parent_modified=False,
        separate_completion_invoked=True,flexible_policy=POLICY)
    sid = source['id']+f'_flex{ordinal:02}'
    output = folder/'placement'; output.mkdir()
    sp, mp = output/(sid+'.json'), output/(sid+'_metadata.json')
    atomic(sp,scene); atomic(mp,meta)
    require(common.sha(sp) not in known, 'Never capture a prior attempted native payload')
    entry = dict(source,id=sid,source_cell_id=sid,scenario=str(sp),metadata=str(mp),
        scenario_sha256=common.sha(sp),scene_group_id=meta['scene_group_id'],role=meta['spec']['role'],
        distance_bounds=meta['spec']['distance_bounds'],motion=meta['runtime_actors'][0]['motion'])
    child = parent.batch.campaign.subset_deck(deck,[entry])
    import base5_layout_split as layout
    reference = layout.register_recovery_overlay(source,original,entry,meta,output/'layout_split_rebind')
    meta['training_split_overlay'] = reference
    atomic(mp,meta)
    child['split_overlay'] = dict(reference,policy=layout.POLICY)
    child.update(flexible_completion_policy=POLICY,layout_split_finalization_pending=True,bulk_ready=False)
    manifest = output/'manifest.json'; atomic(manifest,child)
    layout.validate_manifest_overlay(child,allow_subset=True)
    return manifest, entry


def merge(state, baseline):
    chosen = {row['source_cell_id']:copy.deepcopy(row) for row in baseline}
    for row in state.get('imported_selected',[]):
        ident = row['source_cell_id']
        require(ident in state['deficits'] and ident not in chosen,'Duplicate/foreign imported native approval')
        chosen[ident] = copy.deepcopy(row)
    for attempt in state['attempts']:
        result = (attempt.get('result') or {}).get('selected')
        if not result:
            continue
        require(attempt['id'] not in chosen and attempt['id'] in state['deficits'], 'Duplicate/foreign approval')
        result = copy.deepcopy(result)
        result.update(source_cell_id=attempt['id'],flexible_completion=dict(policy=POLICY,
            original_cell_id=attempt['id'],donor_cell_id=attempt['donor_id'],manifest=attempt['manifest'],
            adapter_sha256=state['contract']['adapter_sha256'],relaxed_fields=list(RELAXED)))
        chosen[attempt['id']] = result
    return [chosen[i] for i in state['source_order'] if i in chosen], [i for i in state['deficits'] if i not in chosen]


def import_completion(sources, parent, deck, baseline, deficits, source_sha):
    """Import only independently complete native children from a stopped wrapper.

    Old halted status/history are never changed. A wrapper's CPU-selection
    failure is not a native safety failure, and cannot erase seven completed
    native approvals. Only those new children are directly reverified here.
    """
    selected,history,pins,known = [],[],{},set()
    chosen = {r['source_cell_id']:r for r in baseline}
    order = [e['id'] for e in deck['scenes']]
    originals = {e['id']:e for e in deck['scenes']}
    for source in sources:
        source = common.physical(source)
        state,status = read(source/'state.json'),read(source/'status.json')
        require(state['contract']['parent'] == str(parent)
            and state['contract']['source_sha256'] == source_sha
            and state['source_order'] == order and state['deficits'] == deficits
            and state['baseline_selected_sha256'] == common.sha(parent/'selected_ledger.jsonl'),
            'Imported supplemental source/parent/quota binding differs')
        require(status.get('current_cell_id') is None and state.get('current') is None
            and status['stage'] == state['stage'] and state['stage'] in SAFE_STAGES+('halted',),
            'Only an idle completed/paused or diagnosed CPU-stopped supplement can be imported')
        if state['stage']=='halted':
            require(str(state.get('error','')).startswith((
                'Flexible family/motion/split/distance purpose changed',
                'Flexible geometry crossed its family/motion/split/distance purpose')),
                'Unknown/native safety failure cannot be imported as a CPU proposal miss')
        require(all(common.sha(path)==sha for path,sha in state['contract']['parent_pins'].items()),
                'Imported supplement original parent changed')
        recorded,unused = merge(state,baseline)
        require(recorded == rows(source/'selected_ledger.jsonl') and len(recorded)==status['accepted_candidate'],
                'Imported canonical selection differs from its snapshot')
        observed=[]
        for attempt in state['attempts']:
            ident,ordinal=attempt['id'],attempt['ordinal']
            require(ident in deficits and type(ordinal) is int and 1<=ordinal<=state['contract']['attempts'],
                    'Imported attempt escaped its original deficit/budget')
            history.append(dict(id=ident,ordinal=ordinal,source=str(source),
                donor_sha256=attempt.get('donor_sha256'),manifest=attempt.get('manifest')))
            result = attempt.get('result')
            if result is None:
                require(not attempt.get('child') or not Path(attempt['child']).exists(),
                        'Unverified native child cannot become an ordinary imported CPU failure')
                continue
            manifest = common.physical(attempt['manifest']);child=common.physical(attempt['child'])
            require(common.inside(manifest,source/'cells'/ident/f'attempt_{ordinal}')
                and child == manifest.parent.parent/'campaign'
                and common.sha(manifest)==attempt['manifest_sha256'], 'Imported candidate escaped or changed')
            meta=read(read(manifest)['scenes'][0]['metadata'])
            declaration=meta.get('fresh_completion_source',{})
            require(declaration.get('source_cell_id')==ident
                and declaration.get('source_scenario_sha256')==originals[ident]['scenario_sha256'],
                'Imported candidate does not map to the authoritative original deficit')
            require(verify_attempt(attempt,meta)==result,'Imported native completed proof changed')
            pins.update(evidence_pins(attempt))
            if result.get('selected'):
                row = next(r for r in recorded if r['source_cell_id']==ident)
                require(ident not in chosen,'Duplicate imported supplemental approval')
                require(common.inside(common.physical(row['capture_root']),child), 'Imported native root escaped child')
                chosen[ident]=row;observed.append(row);selected.append(copy.deepcopy(row))
        require(len(observed)==len(recorded)-len(baseline)-len(state.get('imported_selected',[])),
                'Imported selection has no directly verified native evidence')
        # Nested imports remain explicit and pinned; avoid silently copying
        # approvals whose direct child evidence was not inspected in this pass.
        require(not state.get('imported_selected'), 'Chained imports require explicit direct-source arguments')
        pins.update({str(source/name):common.sha(source/name) for name in
            ('state.json','status.json','selected_ledger.jsonl','quota.json')})
        known.update(exact.prior_payloads(source,dict(scenes=[])))
    return dict(selected=selected,history=history,pins=pins,known_payloads=sorted(known),
                policy=POLICY,existing_native_qa_recomputed=False,new_children_reverified=True)


def evidence_pins(attempt):
    """Pin new small control/selection/native-QA records, not the old 4,944 raw."""
    child = Path(attempt['child'])
    names = {'status.json','quota.json','report.json','session_contract.json','shutdown_safety.json',
             'selected_ledger.jsonl','ledger.jsonl','direction_reference_runtime_contract.json'}
    values = {str(path):common.sha(path) for path in child.rglob('*') if path.is_file() and path.name in names}
    row = (attempt.get('result') or {}).get('selected')
    if row:
        raw = Path(row['capture_root'])
        values.update({str(raw/name):common.sha(raw/name) for name in ('scenario.json','metadata.json','audit.json')})
    require(values, 'Completed new attempt must retain control/native-QA evidence')
    return values


def validate_resume(state, output, deck, deficits, baseline):
    require(state['source_order'] == [e['id'] for e in deck['scenes']] and state['deficits'] == deficits,
            'Resume quota/deficit inventory changed')
    seen = set()
    for attempt in state['attempts']:
        ident, ordinal = attempt['id'],attempt['ordinal']
        require(ident in deficits and '/' not in ident and type(ordinal) is int
            and 1 <= ordinal <= state['contract']['attempts'] and (ident,ordinal) not in seen,
            'Foreign/duplicate/out-of-budget attempt')
        seen.add((ident,ordinal))
        if attempt.get('unavailable'):
            require(attempt.get('manifest') is None and attempt.get('result') is None,'Offline unavailable cannot approve')
            continue
        require(attempt.get('manifest'),'Interrupted proposal must be inspected, not silently regenerated')
        folder = output/'cells'/ident/f'attempt_{ordinal}'
        manifest = common.physical(attempt['manifest'])
        require(common.inside(manifest,folder) and manifest.name == 'manifest.json'
            and common.sha(manifest) == attempt['manifest_sha256'], 'Candidate manifest escaped its pinned folder or changed')
        entry = read(manifest)['scenes'][0]
        require(common.inside(common.physical(entry['scenario']),folder)
            and common.inside(common.physical(entry['metadata']),folder)
            and common.sha(entry['metadata']) == attempt['metadata_sha256']
            and common.sha(entry['scenario']) == entry['scenario_sha256'] == attempt['scenario_sha256'],
            'Candidate payload escaped its folder or changed')
        require(common.physical(attempt['child']) == manifest.parent.parent/'campaign',
                'New capture output escaped its registered attempt')
        if attempt.get('result') is not None:
            require(attempt.get('evidence_pins')
                and all(common.sha(path) == sha for path,sha in attempt['evidence_pins'].items()),
                'New completed native evidence changed')
            actual = rows(Path(attempt['child'])/'selected_ledger.jsonl')
            observed = attempt['result'].get('selected')
            require(actual == ([observed] if observed else []), 'Native child selection differs from saved result')
            if observed:
                require(common.inside(common.physical(observed['capture_root']),common.physical(attempt['child'])),
                        'Native approval escaped its own child campaign')
        elif Path(attempt['child']).exists():
            require(read(Path(attempt['child'])/'status.json')['stage'] in exact.TERMINAL,
                    'Interrupted/unsafe native capture needs explicit diagnosis')
    require(merge(state,baseline)[0] == rows(output/'selected_ledger.jsonl'),'Published merged selection changed')


def finalize_selection(source, output, state, baseline):
    """Finalize actual selected geometry without resurrecting failed-plan quotas.

    Original 4,944 accepted rows are unchanged and are a literal subset of this
    selection. Only old hypothetical *unobserved* source train-count floors are
    omitted. All geometry grouping/leakage, native audit, lineage and alias
    checks remain unchanged. This is CPU input qualification, not model quality.
    """
    import finalize_base5_layout_split as finalizer
    selected = rows(output/'selected_ledger.jsonl')
    require(state['stage'] == 'complete' and len(selected) == 5000 and not merge(state,baseline)[1],
            'One complete 5K approved selection required before finalization')
    lookup = {r['source_cell_id']:r for r in selected}
    require(all(lookup.get(r['source_cell_id']) == r for r in baseline), 'Finalization cannot remove/change approved prefix')
    target = output/'layout_split_final'
    result_path = target/'result.json'
    if result_path.exists():
        result = read(result_path)
        require(common.sha(result['manifest']) == result['manifest_sha256']
            and common.sha(result['split_overlay']) == result['split_overlay_sha256']
            and common.sha(result['derived_selected_ledger']) == result['derived_selected_ledger_sha256'],
            'Existing finalization changed')
        return result
    tree = ast.parse(textwrap.dedent(inspect.getsource(finalizer.finalize)))
    replacements = 0
    for node in ast.walk(tree):
        if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) \
                and node.targets[0].id == 'floors':
            require(ast.dump(node.value) == ast.dump(ast.parse("base['audit'].get('train_floors', {})",mode='eval').body),
                    'Original finalizer floor semantics changed')
            node.value = ast.Dict(keys=[],values=[]);replacements += 1
    require(replacements == 1, 'Exactly one obsolete planned-floor binding required')
    scope = dict(vars(finalizer));ast.fix_missing_locations(tree)
    exec(compile(tree,str(Path(__file__).absolute())+'::explicit_flexible_finalizer','exec'),scope)
    result = scope['finalize'](source,output/'adopted_manifest.json',output/'selected_ledger.jsonl',target)
    declaration = dict(policy=POLICY,portable_adapter_sha256=common.sha(__file__),
        frozen_finalizer_sha256=common.sha(finalizer.__file__),preserved_approved_prefix=len(baseline),
        baseline_selection_sha256=state['baseline_selected_sha256'],
        old_unobserved_planned_floors_omitted=True,actual_approved_observations_removed=0,
        native_qa_unchanged=True,geometry_and_leakage_checks_unchanged=True)
    manifest = read(result['manifest'])
    manifest.setdefault('provenance',{})['flexible_selected_finalization'] = declaration
    atomic(result['manifest'],manifest)
    result.update(manifest_sha256=common.sha(result['manifest']),flexible_selected_finalization=declaration)
    atomic(result_path,result)
    return result


def publish(output, state, baseline, baseline_elapsed):
    selected, missing = merge(state,baseline)
    atomic(output/'state.json',state)
    # Canonical ledger changes only after a new approval, not on each heartbeat.
    atomic_rows(output/'selected_ledger.jsonl',selected)
    atomic(output/'quota.json',dict(target=len(state['source_order']),accepted=len(selected),deficit=len(missing),
        pending=0,cells=[dict(id=i,reason='fresh_same_purpose_replacement_pending') for i in missing]))
    atomic(output/'status.json',dict(stage=state['stage'],error=state.get('error'),
        requested=len(state['source_order']),finished=len(state['source_order']),
        checkpoint_finished=len(state['source_order']),checkpoint_only=True,
        accepted_candidate=len(selected),review=len(missing),failed=int(state['stage']=='halted'),
        selected_not_certified=state.get('selected_not_certified',False),
        initial_accepted=len(baseline),replenished=len(selected)-len(baseline),attempts=len(state['attempts']),
        current=state.get('current_child') or state.get('current'),current_cell_id=state.get('current'),updated_at=time.time(),eta_sec=0,
        elapsed_sec=baseline_elapsed+state.get('elapsed_sec',0),flexible_completion=True,
        mode='failure_aware_replenishment',qa_unchanged=True,exact_slot_contract=False))
    return selected,missing


def adopted(output,state,baseline_manifest,selected,deck):
    # Reuse the trusted previous adopted entries; do not re-audit/re-hash the
    # entire 4,944 native prefix when publishing a supplemental checkpoint.
    parent = read(baseline_manifest)
    lookup = {e['id']:e for e in parent['scenes']}
    source = {e['id']:e for e in deck['scenes']}
    for row in selected:
        ident = row['source_cell_id']
        if ident in lookup:
            continue
        from run_base5_fresh_completion import adopted_entry
        entry = adopted_entry(row,source)
        meta = read(entry['metadata'])
        entry.update(family=meta['spec']['family'],mission=meta['spec']['mission'],motion=meta['runtime_actors'][0]['motion'])
        lookup[ident] = entry
    parent['scenes'] = [lookup[row['source_cell_id']] for row in selected]
    parent['count'] = len(selected)
    parent.update(flexible_completion=dict(policy=POLICY,preserved_approved_prefix=len(read(output/'baseline_snapshot.json')['source_ids']),
        native_qa_unchanged=True,exact_quota_preserved=False,relaxed_fields=list(RELAXED),
        layout_finalization_required=True,adapter_sha256=state['contract']['adapter_sha256']),bulk_ready=False)
    atomic(output/'adopted_manifest.json',parent)


def verify_attempt(attempt, original):
    """Reconstruct exactly one declared round; the old batch API fixes two.

    This keeps the original report/native/Park/direction proof and only makes
    the supplemental attempt budget explicit instead of claiming a two-round
    parent recovery contract that this new runner did not execute.
    """
    parent, direction = common.modules()
    manifest, child = Path(attempt['manifest']),Path(attempt['child'])
    require(common.sha(manifest) == attempt['manifest_sha256'],'Attempt source changed')
    status, quota = read(child/'status.json'),read(child/'quota.json')
    require(status['stage'] in exact.TERMINAL,'Completed native recovery evidence required; no recapture')
    campaign = parent.batch.campaign
    require(read(child/'campaign_contract.json') == campaign.contract(manifest,[0],1),
            'Flexible recovery must retain its explicitly declared one-round contract')
    source = read(manifest)['scenes'][0]
    report = read(child/source['id']/'round_0'/'report.json')
    require(report['stage'] == 'complete','Incomplete quality report')
    campaign.validate_selected(report)
    require(len(report['selected']) <= 1,'At most one native selection per quota')
    reconstructed = []
    for row in report['selected']:
        row = copy.deepcopy(row);row['source_cell_id'] = source['id'];reconstructed.append(row)
    selected = rows(child/'selected_ledger.jsonl')
    require(selected == reconstructed,'Native selected ledger differs from its quality report')
    require(quota['accepted'] == len(selected) and quota['deficit'] == 1-len(selected)
        and quota.get('pending') == 0,'Native quota differs from its evidence')
    direction.verify_campaign(child,manifest,[0])
    chosen = None
    if selected:
        proof = direction.verify_completed_campaign(manifest,0,child)
        chosen = proof['accepted_row']
        require(chosen == selected[0] and exact.identity(read(Path(chosen['capture_root'])/'metadata.json'))
            == exact.identity(original),'Native selection changed its own candidate spec/roster/group/unit')
    return dict(selected=chosen,stage=status['stage'],quota_sha256=common.sha(child/'quota.json'),
        ledger_sha256=common.sha(child/'selected_ledger.jsonl'))


def capture(attempt, known):
    _, direction = common.modules()
    child = Path(attempt['child'])
    original = read(read(attempt['manifest'])['scenes'][0]['metadata'])
    if child.exists():
        require((child/'status.json').exists() and read(child/'status.json')['stage'] in exact.TERMINAL,
                'Interrupted/unsafe child requires explicit safety inspection')
        return verify_attempt(attempt,original)
    args = argparse.Namespace(enable_final_direction_reference=True,approve_bulk_capture=False,
        manifest=Path(attempt['manifest']),output=child,indices='0',limit=1,resume=True,
        quota_rounds=1,seed_capture=None)
    with exact.recovery_population.installed(), exact.nonrepeat(known), \
            exact.inherited_native(direction,set(known)-{attempt['scenario_sha256']}):
        code = direction.run(args)
    require(code == 0,'Native safety/communication/Park child failed; no next load')
    return verify_attempt(attempt,original)


def execute(args):
    require(1 <= args.attempts <= 5 and 30 <= args.time_budget_sec <= 7200 and args.stop_after_cells >= 0,
            'Finite attempt/time/cell budget required')
    doc, baseline, parent, output, deck, selected, deficits, pins = summary(args.profile,args.parent,args.output)
    if not args.run and not args.prepare and not args.finalize:
        return dict(event='flexible_completion_plan',approved=len(selected),deficit=len(deficits),
            capture_started=False,files_written=False,native_qa_unchanged=True,relaxed_fields=list(RELAXED))
    require(not args.run or args.confirmed_environment == doc['environment'],'Explicit matching MORAI environment required')
    boundary = exact.verify_baseline(doc,baseline,rows(baseline/'selected_ledger.jsonl'),read(baseline/'quota.json')['cells'])
    with common.handoff.idle_locks(baseline), (parent/'.completion.lock').open('a') as parent_lock:
        fcntl.flock(parent_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        require(all(common.sha(path) == sha for path,sha in pins.items()),'Preserved parent snapshot changed')
        output.mkdir(parents=True,exist_ok=True)
        with (output/'.flexible.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            contract = dict(policy=POLICY,profile=str(common.physical(args.profile)),profile_sha256=common.sha(args.profile),
                source_sha256=doc['manifest_sha256'],parent=str(parent),parent_pins=pins,
                adapter_sha256=common.sha(__file__),exact_helper_sha256=common.sha(exact.__file__),
                helpers=common.helpers(),attempts=args.attempts,
                import_sources=[str(common.physical(p)) for p in args.import_native_completion])
            path = output/'state.json'
            if path.exists():
                state = read(path)
                require(state['contract'] == contract and state['stage'] in SAFE_STAGES,
                        'Resume source/runtime changed or safety diagnosis needed')
                validate_resume(state,output,deck,deficits,selected)
                require(all(common.sha(path)==sha for path,sha in state.get('import_receipt',{}).get('pins',{}).items()),
                        'Imported source/native snapshot changed')
            else:
                require(not any(p.name != '.flexible.lock' for p in output.iterdir()),'Fresh registered output required')
                with exact.overlay_read_cache.installed(exclude_roots=(output,)), \
                        boundary.installed(scripts_dir=common.SCRIPTS,direction_module=common.modules()[1],
                            delta_input_manifest=doc['manifest']),common.route_compatibility(doc):
                    common.warm_boundary(boundary,doc,common.modules()[1])
                    imported=import_completion(args.import_native_completion,parent,deck,selected,deficits,doc['manifest_sha256'])
                state = dict(contract=contract,source_order=[e['id'] for e in deck['scenes']],deficits=deficits,
                    baseline_selected_sha256=common.sha(parent/'selected_ledger.jsonl'),attempts=[],stage='prepared',
                    elapsed_sec=0.,current=None,imported_selected=imported['selected'],import_receipt=imported)
                atomic(output/'baseline_snapshot.json',dict(parent=str(parent),native_qa_recomputed=False,
                    source_ids=[r['source_cell_id'] for r in selected],selected_sha256=state['baseline_selected_sha256']))
            if args.finalize:
                return dict(event='flexible_completion_finalized',**finalize_selection(doc['manifest'],output,state,selected))
            start, prior, processed = time.monotonic(),state['elapsed_sec'],0
            elapsed = read(parent/'status.json')['elapsed_sec']
            publish(output,state,selected,elapsed)
            known = exact.prior_payloads(baseline,deck) | exact.prior_payloads(parent,dict(scenes=[]))
            known.update(exact.prior_payloads(output,dict(scenes=[])))
            known.update(state.get('import_receipt',{}).get('known_payloads',[]))
            known.update(a['scenario_sha256'] for a in state['attempts'] if a.get('scenario_sha256'))
            lookup = {e['id']:e for e in deck['scenes']}
            try:
                with exact.overlay_read_cache.installed(exclude_roots=(output,)), \
                        boundary.installed(scripts_dir=common.SCRIPTS,direction_module=common.modules()[1],
                            delta_input_manifest=doc['manifest']),common.route_compatibility(doc):
                    common.warm_boundary(boundary,doc,common.modules()[1])
                    donors,pools = donor_pool(deck,selected)
                    with flexible_rebinding(pools):
                        for ordinal in range(1,args.attempts+1):
                            for ident in deficits:
                                if ident not in merge(state,selected)[1]:
                                    continue
                                previous = [a for a in state['attempts'] if a['id'] == ident]
                                historical=[a for a in state.get('import_receipt',{}).get('history',[]) if a['id']==ident]
                                if any(a['ordinal']==ordinal for a in historical):
                                    continue
                                attempt = next((a for a in previous if a['ordinal'] == ordinal),None)
                                if attempt and (attempt.get('result') is not None or attempt.get('unavailable')):
                                    continue
                                if time.monotonic()-start >= args.time_budget_sec or any((output/n).exists() for n in ('PAUSE','STOP')) \
                                        or args.stop_after_cells and processed >= args.stop_after_cells:
                                    state.update(stage='paused',current=None,current_child=None,elapsed_sec=prior+time.monotonic()-start)
                                    chosen,_ = publish(output,state,selected,elapsed)
                                    adopted(output,state,parent/'adopted_manifest.json',chosen,deck)
                                    return dict(event='flexible_completion_paused',approved=len(chosen),deficit=5000-len(chosen))
                                state.update(current=ident,stage='preparing')
                                if attempt is None:
                                    original = read(lookup[ident]['metadata'])
                                    used = {a['donor_sha256'] for a in previous+historical if a.get('donor_sha256')}
                                    options = donor_order(original,donors,pools,ident,ordinal,used)
                                    attempt = dict(id=ident,ordinal=ordinal,result=None,manifest=None)
                                    state['attempts'].append(attempt);publish(output,state,selected,elapsed)
                                    for number,(donor,donor_meta) in enumerate(options[:4],1):
                                        folder = output/'cells'/ident/f'attempt_{ordinal}'/f'template_{number}'
                                        try:
                                            manifest,entry = generate(lookup[ident],original,donor,donor_meta,deck,folder,known,ordinal)
                                        except common.modules()[0].batch.campaign.loop.PlacementProposalExhausted as error:
                                            attempt.setdefault('offline_rejections',[]).append(dict(donor=donor['id'],error=str(error)))
                                            continue
                                        attempt.update(donor_id=donor['id'],donor_sha256=donor['scenario_sha256'],
                                            manifest=str(manifest),manifest_sha256=common.sha(manifest),
                                            metadata_sha256=common.sha(entry['metadata']),
                                            scenario_sha256=entry['scenario_sha256'],child=str(folder/'campaign'))
                                        known.add(entry['scenario_sha256']);break
                                    if not attempt.get('manifest'):
                                        attempt['unavailable'] = 'No fresh same-purpose donor within bounded four-template CPU search'
                                        processed += 1;publish(output,state,selected,elapsed);continue
                                require(attempt.get('manifest'),'Incomplete offline proposal needs inspection')
                                if not args.run:
                                    processed += 1;publish(output,state,selected,elapsed);continue
                                state.update(stage='running',current_child=attempt['child']);publish(output,state,selected,elapsed)
                                attempt['result'] = capture(attempt,known)
                                attempt['evidence_pins'] = evidence_pins(attempt)
                                known.update(exact.prior_payloads(Path(attempt['child']),dict(scenes=[])))
                                state['elapsed_sec'] = prior+time.monotonic()-start
                                chosen,missing = publish(output,state,selected,elapsed);processed += 1
                                print(__import__('json').dumps(dict(event='flexible_completion_cell',id=ident,attempt=ordinal,
                                    donor=attempt['donor_id'],accepted=bool(attempt['result']['selected']),
                                    approved=len(chosen),deficit=len(missing)),ensure_ascii=False),flush=True)
                        state.update(stage='complete' if not merge(state,selected)[1] else 'complete_with_deficit',
                            current=None,current_child=None,elapsed_sec=prior+time.monotonic()-start)
                        if not args.run:state['stage']='prepared'
                        chosen,_ = publish(output,state,selected,elapsed)
                        adopted(output,state,parent/'adopted_manifest.json',chosen,deck)
            except BaseException as error:
                state.update(stage='halted',error=str(error),selected_not_certified=True,current=None,current_child=None,
                    elapsed_sec=prior+time.monotonic()-start)
                publish(output,state,selected,elapsed)
                raise
            finally:
                try:
                    boundary.verify()
                    require(all(common.sha(path) == sha for path,sha in pins.items()),'Parent snapshot changed during completion')
                    require(all(common.sha(path)==sha for path,sha in state.get('import_receipt',{}).get('pins',{}).items()),
                            'Imported completed snapshot changed during completion')
                    require(common.sha(__file__) == contract['adapter_sha256']
                        and common.sha(exact.__file__) == contract['exact_helper_sha256'], 'Completion implementation changed')
                except BaseException as error:
                    state.update(stage='halted',error=str(error),selected_not_certified=True,current=None,current_child=None)
                    publish(output,state,selected,elapsed)
                    raise
            return dict(event='flexible_completion_result',approved=len(merge(state,selected)[0]),
                deficit=len(merge(state,selected)[1]),output=str(output),native_qa_unchanged=True)


def parser():
    p = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('--profile',type=Path,required=True);p.add_argument('--parent',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--import-native-completion',type=Path,action='append',default=[])
    mode=p.add_mutually_exclusive_group();mode.add_argument('--prepare',action='store_true');mode.add_argument('--run',action='store_true')
    mode.add_argument('--finalize',action='store_true')
    p.add_argument('--confirmed-environment',choices=common.ENVIRONMENTS)
    p.add_argument('--attempts',type=int,default=3);p.add_argument('--time-budget-sec',type=int,default=7200)
    p.add_argument('--stop-after-cells',type=int,default=0)
    return p


if __name__ == '__main__':
    try:
        print(__import__('json').dumps(execute(parser().parse_args()),ensure_ascii=False),flush=True)
    except (OSError,ValueError,RuntimeError) as error:
        print(__import__('json').dumps(dict(event='flexible_completion_blocked',error=str(error),fallback_allowed=False),
            ensure_ascii=False),file=sys.stderr)
        raise SystemExit(1)
