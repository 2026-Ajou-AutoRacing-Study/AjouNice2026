#!/usr/bin/env python3
"""Common pre-native CPU feasibility screening, never sensor-outcome selection.

Every finally allocated geometry is built and cached before a diagnostic subset
is registered. Rejected CPU layouts remain explicit; full materialization reuses
the exact accepted payloads instead of discovering late unfillable slots.
"""
from collections import Counter
import copy
from pathlib import Path
import time

SCHEMA='base5_reinforcement_cpu_feasibility_v1'
POLICY='pre-native-bounded-layout-screen-and-exact-payload-reuse-v1'


def key(assignment):
    """An ID can move between rules after a previous CPU rejection.

    Cache and reject the exact (slot, rule) request, never the slot alone. A
    30–45m failure must not exclude the same slot's valid 20–30m geometry.
    """
    return (assignment['id'], assignment['rule'])


def compatible(meta,rule):
    """A near/mid route-following helper can hide the same-sector far primary.

    This is a general metadata predicate, not a blacklist of observed scene IDs,
    assets or native failures. Existing helpers are never removed to pass it.
    """
    if rule['target_bounds_m'][0]<30:
        return True
    role=meta['spec']['role']
    for actor in meta['runtime_actors'][1:]:
        if (actor['source_list']=='objectList'
                and actor.get('population_pool','').startswith('obstacle')
                and actor['actual_ego_role']==role
                and 15<actor['planned_center_distance_m']<rule['target_bounds_m'][0]):
            return False
    return True


def prepare(request,records,config,output,*,driver):
    output=driver.physical(output)
    driver.require(not output.exists(),'New pre-native CPU screening root required')
    profile=request['profile'];controls=profile['cpu_feasibility']
    maximum=controls['maximum_attempts'];tests_limit=controls['maximum_candidate_tests']
    native=driver.unit.load(config[driver.unit.KEY])
    sizes={name:row['native_size_lwh_m'] for name,row in native['assets'].items()}
    originals={row['id']:row for row in records}
    tested={};rejected=set();rounds=[];ctx=None;started=time.monotonic()
    output.mkdir(parents=True)
    requested_counts=None
    while True:
        allocation=driver.allocate(records,profile,sizes,request['source']['sha256'],
                                   cpu_rejected_pairs=rejected)
        counts={(row['rule'],row['asset']):row['selected'] for row in allocation['decisions']}
        if requested_counts is None:
            requested_counts=counts
        if counts!=requested_counts:
            driver.atomic_json(output/'failure.frozen',dict(
                reason='preallocated_rule_asset_target_cannot_be_filled',
                records=list(tested.values()),rounds=rounds,decisions=allocation['decisions'],
                preallocated_counts=[dict(rule=rule,asset=asset,count=count)
                    for (rule,asset),count in sorted(requested_counts.items())],
                native_outcomes_used_for_selection=False,bulk_ready=False))
            raise ValueError('CPU screening cannot silently reduce a preallocated rule/asset target')
        pending=[row for row in allocation['assignments'] if key(row) not in tested]
        if not pending:
            break
        rounds.append([list(key(row)) for row in pending])
        for assignment in pending:
            driver.require(len(tested)<tests_limit,'Explicit pre-native CPU candidate-test budget exhausted')
            original=originals[assignment['id']];old=original['meta']
            spec=driver.changed_spec(old,assignment)
            if ctx is None:
                ctx=driver.prewarm(driver.spatial.gen.context(driver.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                    driver.spatial.gen.geo.DEFAULT_MGEO),copy.deepcopy(config))
            reasons=Counter();accepted=None
            for retry in range(old['retry'],old['retry']+maximum):
                try:
                    scenario,metadata=driver.spatial.build(spec,config,ctx,retry)
                    driver.verify_transition(old,metadata,assignment)
                    for field in driver.PRESERVED_META:
                        if field in old:metadata[field]=copy.deepcopy(old[field])
                    accepted=(scenario,metadata,retry)
                    break
                except (driver.native_range.RangeContractError,driver.unit.UnitGeometryContractError):
                    raise
                except ValueError as error:
                    reasons[str(error)]+=1
            row=dict(assignment=copy.deepcopy(assignment),accepted=accepted is not None,
                maximum_attempts=maximum,rejected_reasons=dict(reasons),native_observed=False)
            if accepted is None:
                row.update(attempts=maximum,reason='bounded_cpu_geometry_exhausted')
                rejected.add(key(assignment))
            else:
                scenario,metadata,retry=accepted
                cache_key=driver.digest(list(key(assignment)))
                scenario_path=output/'scenarios'/(cache_key+'.json')
                metadata_path=output/'metadata'/(cache_key+'.json')
                driver.atomic_json(scenario_path,scenario);driver.atomic_json(metadata_path,metadata)
                row.update(scenario=driver.pin(scenario_path),metadata=driver.pin(metadata_path),
                    attempts=retry-old['retry']+1,actual_retry=retry)
            tested[key(assignment)]=row
            # Save failed geometry evidence as it happens, including when a
            # later target-shortfall or budget guard intentionally aborts.
            driver.atomic_json(output/'trial_ledger.frozen',dict(
                schema=SCHEMA,policy=POLICY,records=list(tested.values()),rounds=rounds,
                native_outcomes_used_for_selection=False,complete=False))
            driver.atomic_json(output/'progress.json',dict(policy=POLICY,profile=request['profile_file'],
                tested=len(tested),accepted=sum(row['accepted'] for row in tested.values()),
                rejected=len(rejected),current_id=assignment['id'],native_observed=False,
                elapsed_sec=time.monotonic()-started))
    selected={key(row) for row in allocation['assignments']}
    driver.require(selected and all(tested[ident]['accepted'] for ident in selected),
                   'Every finally allocated CPU geometry must actually be cached')
    receipt=dict(schema=SCHEMA,policy=POLICY,source=request['source'],source_build=request['source_build'],
        profile_sha256=driver.digest(profile),implementation=driver.pin(driver.__file__),
        screen_implementation=driver.pin(__file__),controls=copy.deepcopy(controls),
        records=list(tested.values()),rounds=rounds,
        preallocated_counts=[dict(rule=rule,asset=asset,count=count)
                             for (rule,asset),count in sorted(requested_counts.items())],
        cpu_rejected_pairs=[list(pair) for pair in sorted(rejected)],
        selected_pairs=[list(pair) for pair in sorted(selected)],
        decisions=allocation['decisions'],tested=len(tested),accepted=sum(row['accepted'] for row in tested.values()),
        rejected=len(rejected),native_outcomes_used_for_selection=False,native_qualified=False,
        unchanged_qa=True,spatial_windows_widened=False,helpers_dropped=False,
        elapsed_sec=time.monotonic()-started)
    path=output/'screen.frozen';driver.atomic_json(path,receipt)
    return path


def verify(request,records,config,path,*,driver):
    path=driver.physical(path);document=driver.read(path)
    profile=request['profile']
    driver.require(document.get('schema')==SCHEMA and document.get('policy')==POLICY
        and document.get('source')==request['source'] and document.get('source_build')==request['source_build']
        and document.get('profile_sha256')==driver.digest(profile)
        and document.get('implementation')==driver.pin(driver.__file__)
        and document.get('screen_implementation')==driver.pin(__file__)
        and document.get('controls')==profile['cpu_feasibility']
        and document.get('native_outcomes_used_for_selection') is False
        and document.get('spatial_windows_widened') is False and document.get('helpers_dropped') is False,
        'Exact pre-native CPU screening source/profile/implementation required')
    native=driver.unit.load(config[driver.unit.KEY])
    sizes={name:row['native_size_lwh_m'] for name,row in native['assets'].items()}
    originals={row['id']:row for row in records};tested={};rejected=set();pins={str(path):driver.sha256(path)}
    rows=document['records'];offset=0
    # Replay selector rounds, not geometry trials: this proves all rejected rows
    # were actually selected before a later same-asset replacement was tried.
    for round_keys in document['rounds']:
        allocation=driver.allocate(records,profile,sizes,request['source']['sha256'],
                                   cpu_rejected_pairs=rejected)
        pending=[row for row in allocation['assignments'] if key(row) not in tested]
        driver.require(round_keys==[list(key(row)) for row in pending] and pending,
                       'CPU screening selection history differs from deterministic allocation')
        for assignment in pending:
            driver.require(offset<len(rows),'CPU screening round records are missing')
            row=rows[offset];offset+=1
            driver.require(row['assignment']==assignment,'CPU record assignment changed within its selection round')
            _verify_record(row,originals,profile,sizes,tested,rejected,pins,driver=driver)
    driver.require(offset==len(rows),'CPU records include an unselected trial')
    driver.require(len(tested)==document['tested']<=profile['cpu_feasibility']['maximum_candidate_tests']
        and [list(pair) for pair in sorted(rejected)]==document['cpu_rejected_pairs'],
        'CPU screening inventory/rejections changed')
    allocation=driver.allocate(records,profile,sizes,request['source']['sha256'],cpu_rejected_pairs=rejected)
    driver.require(document['preallocated_counts']==[
        dict(rule=row['rule'],asset=row['asset'],count=row['selected'])
        for row in sorted(allocation['decisions'],key=lambda row:(row['rule'],row['asset']))],
        'CPU screening silently reduced a preallocated rule/asset target')
    driver.require(document['selected_pairs']==[list(pair) for pair in sorted(key(row) for row in allocation['assignments'])]
        and allocation['decisions']==document['decisions']
        and all(tested.get(key(row),{}).get('accepted') is True for row in allocation['assignments']),
        'Final deterministic allocation does not match actual CPU-feasible cached rows')
    allocation['cpu_feasibility']=driver.pin(path)
    allocation['cpu_rejected_pairs']=[list(pair) for pair in sorted(rejected)]
    selected={row['id']:tested[key(row)] for row in allocation['assignments']}
    return allocation,selected,pins


def _verify_record(row,originals,profile,sizes,tested,rejected,pins,*,driver):
        assignment=row['assignment'];ident=assignment['id'];ident_key=key(assignment)
        driver.require(ident in originals and ident_key not in tested and row.get('native_observed') is False,
                       'Unique CPU-only screening record required')
        old=originals[ident]['meta'];rule=next((r for r in profile['rules'] if r['id']==assignment['rule']),None)
        driver.require(rule is not None and driver.eligible(old,rule,sizes)
            and row.get('maximum_attempts')==profile['cpu_feasibility']['maximum_attempts']
            and type(row.get('attempts')) is int and 1<=row['attempts']<=row['maximum_attempts'],
            'CPU record escaped its structural scope or finite budget')
        driver.changed_spec(old,assignment)
        if row['accepted']:
            for field in ('scenario','metadata'):
                driver.require(driver.pin(row[field]['path'])==row[field],'CPU cached payload/metadata changed')
                pins[row[field]['path']]=row[field]['sha256']
            metadata=driver.read(row['metadata']['path'])
            driver.verify_transition(old,metadata,assignment)
            driver.require(metadata['retry']==row['actual_retry']
                and metadata['retry']==old['retry']+row['attempts']-1,'Cached first valid CPU retry differs')
        else:
            driver.require(row['attempts']==row['maximum_attempts']
                and sum(row['rejected_reasons'].values())==row['maximum_attempts']
                and row.get('reason')=='bounded_cpu_geometry_exhausted','CPU rejection must retain its full finite search evidence')
            rejected.add(ident_key)
        tested[ident_key]=row
