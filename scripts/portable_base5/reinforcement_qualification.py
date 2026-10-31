#!/usr/bin/env python3
"""명시적 공통 보강 build extension을 원래 여섯 native 검사에 연결한다.

원래 assembly receipt를 위조하거나 QA gate를 생략하지 않는다. 이 adapter는
새 extension의 계보·변경 행·단위 크기·분할을 검증하고 source 등록 부분만
명시적으로 연결한다. native/음영/입력/CPU/두 resume 검사는 원형 그대로다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from collections import Counter
from contextlib import contextmanager
import copy
import json
import math
from pathlib import Path
import threading
import time

import reinforcement_profile as profile
import fresh_environment_qualification as fresh

unit_smoke = fresh.unit_smoke
original = fresh.original
SCHEMA = fresh.SCHEMA
QUALIFICATION_SCHEMA = 'base5_reinforced_environment_release_qualification_v1'
POLICY = 'explicit-common-reinforcement-six-native-gates-v1'
GATES = fresh.GATES
BASE_INSPECT = unit_smoke.inspect_source
BASE_QUALIFY = fresh.qualify
_LOCK = threading.Lock()


def adapter_pin():
    return dict(path=str(profile.physical(__file__)), sha256=profile.sha256(__file__),
                registration_schema=unit_smoke.SCHEMA)


def extension_context(manifest, build_extension, *, allow_pilot=False):
    manifest, build_extension = map(profile.physical, (manifest, build_extension))
    deck, extension = profile.read(manifest), profile.read(build_extension)
    profile.require(extension.get('schema') == profile.BUILD_SCHEMA
        and extension.get('policy') == profile.BUILD_POLICY
        and extension.get('phase') in (('pilot','full') if allow_pilot else ('full',))
        and extension.get('all_assignments_materialized') == (extension.get('phase') == 'full')
        and extension.get('status') == 'CPU_ONLY_REINFORCEMENT; native profile qualification required'
        and extension.get('source_and_baseline_preserved') is True,
        'True explicit complete reinforcement build extension required')
    profile.require(extension.get('final_manifest') == str(manifest)
        and extension.get('final_manifest_sha256') == profile.sha256(manifest)
        and extension.get('environment') == deck.get('config',{}).get('environment_tag')
        and deck.get('count') == len(deck.get('scenes',[])) == extension.get('count') == 5000
        and deck.get('native_qualified') is False and deck.get('bulk_ready') is False
        and extension.get('native_qualified') is False and extension.get('bulk_ready') is False,
        'Reinforcement final source/environment/SHA differs or falsely claims native qualification')
    for key in ('source','source_build','profile','allocation','implementation','full_audit',
                'cpu_feasibility','screen_implementation'):
        record = extension.get(key)
        profile.require(isinstance(record,dict) and set(record) == {'path','sha256'}
            and profile.sha256(record['path']) == record['sha256'], 'Reinforcement extension pin changed: '+key)
    profile.require(extension['implementation'] == profile.pin(profile.__file__),
                    'Reinforcement generator implementation changed')
    profile.require(extension['screen_implementation']==profile.pin(profile.cpu_screen.__file__),
                    'Reinforcement CPU screening implementation changed')
    profile.inputs.verify_hashes(extension['original_input_hashes'], 'Reinforcement source evidence')
    profile.inputs.verify_hashes(extension['runtime_hashes'], 'Reinforcement native runtime')
    parent = BASE_INSPECT(extension['source']['path'], extension['source_build']['path'])
    old_deck, old_rows = profile.records_from_manifest(extension['source']['path'])
    document = profile.validate_profile(profile.read(extension['profile']['path']))
    native = profile.unit.load(old_deck['config'][profile.unit.KEY])
    sizes = {name:row['native_size_lwh_m'] for name,row in native['assets'].items()}
    allocation = profile.read(extension['allocation']['path'])
    screen_request=dict(source=extension['source'],source_build=extension['source_build'],profile=document)
    expected,cached,screen_pins=profile.cpu_screen.verify(screen_request,old_rows,old_deck['config'],
                                    extension['cpu_feasibility']['path'],driver=profile)
    for key,value in expected.items():
        profile.require(allocation.get(key) == value, 'Declared reinforcement allocation changed: '+key)
    profile.require(allocation.get('source') == extension['source']
        and allocation.get('source_build') == extension['source_build'], 'Reinforcement allocation lineage differs')
    selected = (profile.pilot_assignments(allocation, extension['changed_scenes'])
                if extension['phase'] == 'pilot' else allocation['assignments'])
    targets = {row['id']:row for row in selected}
    changes = extension.get('changes',[])
    profile.require(len(changes) == len(targets) == extension.get('changed_scenes')
        and len({row['assignment']['id'] for row in changes}) == len(changes)
        and {row['assignment']['id']:row['assignment'] for row in changes} == targets
        and extension.get('allocated_scenes') == len(allocation['assignments']),
        'Actual changed rows differ from the preregistered bounded allocation')
    _, rows = profile.records_from_manifest(manifest)
    before = {row['id']:row for row in old_rows}
    profile.require(set(before) == {row['id'] for row in rows}, 'Reinforcement dropped/added source slots')
    config = deck.get('config',{}).get(profile.CONFIG_KEY,{})
    profile.require(config.get('profile') == extension['profile']
        and config.get('allocation') == extension['allocation']
        and config.get('phase') == extension['phase']
        and config.get('implementation') == extension['implementation']
        and config.get('cpu_feasibility')==extension['cpu_feasibility']
        and config.get('screen_implementation')==extension['screen_implementation']
        and config.get('environment_specific_branch') is False
        and config.get('new_sampling_behavior_required') is False,
        'Explicit common profile/config identity differs')
    metadata, entries, scale_counts = {}, {}, Counter()
    catalog = {row['data_id']:row for row in profile.read(native['catalog_reference']['path'])['assets']}
    for row in rows:
        ident, entry, meta = row['id'], row['entry'], row['meta']
        old = before[ident]
        profile.require(meta['split'] == old['meta']['split'] == entry['split'], 'Capture split was changed')
        if ident in targets:
            target = targets[ident]
            profile.verify_transition(old['meta'],meta,target)
            profile.require(entry['scenario_sha256']==cached[ident]['scenario']['sha256']
                and meta['retry']==cached[ident]['actual_retry'],
                'Materialized geometry differs from its pre-native CPU feasibility cache')
            provenance = meta.get(profile.PROVENANCE_KEY,{})
            profile.require(provenance.get('assignment') == target
                and provenance.get('profile') == extension['profile']
                and provenance.get('allocation') == extension['allocation']
                and provenance.get('previous_primary_range_plan') == old['meta']['spec'].get(profile.native_range.PLAN_KEY)
                and provenance.get('source_scenario') == profile.pin(old['entry']['scenario'])
                and provenance.get('source_metadata') == profile.pin(old['entry']['metadata'])
                and provenance.get('range_contract_replaced_explicitly') is True,
                'Changed primary lacks truthful previous-contract/source provenance')
        else:
            # A newly calculated training overlay is the only allowed change to
            # an unselected metadata record. Payload stays byte-identical.
            strip = lambda value:{k:v for k,v in value.items() if k!='training_split_overlay'}
            profile.require(entry['scenario_sha256'] == old['entry']['scenario_sha256']
                and strip(meta) == strip(old['meta']), 'Unselected source payload/metadata changed')
        profile.require(meta.get(profile.unit.KEY) == deck['config'].get(profile.unit.KEY)
            and meta.get('fresh_environment_provenance',{}).get('environment') == extension['environment'],
            'Unit reference or independently generated environment identity changed')
        scenario = profile.read(entry['scenario'])
        unit_smoke.raw_unit.reject_scale_metadata(meta)
        scale_counts.update(unit_smoke.raw_unit.raw_unit_scales(scenario))
        unit_smoke.raw_unit.bind_actors(scenario,meta,catalog)
        unit_smoke.validate_entry(entry,deck)
        for container in unit_smoke.raw_unit.CONTAINERS:
            profile.require([a['UNIQUEID'] for a in scenario.get(container,[])] ==
                [a['unique_id'] for a in meta['runtime_actors'] if a['source_list']==container],
                'Raw actor container/order changed')
        if unit_smoke.motion_kind(meta) == 'editor_prepared':
            unit_smoke.ground.validate_motion_ground(meta,scenario)
        metadata[ident],entries[ident] = meta,entry
    near = profile.near_counts(rows,sizes)
    profile.require(dict(near) == extension['final_near_parent_counts']
        and sum(near.values()) >= document['minimum_remaining_near_actors'],
        'Actual near-field preservation differs from the build extension')
    layout_audit = profile.layout.validate_manifest_overlay(deck)
    overlay = profile.layout.read_overlay(deck['split_overlay']['path'])
    audit = profile.read(extension['full_audit']['path'])
    profile.require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors'), 'Reinforcement full CPU audit failed')
    pins = dict(parent['input_hashes'])
    pins.update(screen_pins)
    pins.update(profile.inputs.collect_inputs(manifest))
    pins.update(extension['original_input_hashes'])
    pins.update({str(build_extension):profile.sha256(build_extension),
                 str(profile.physical(__file__)):profile.sha256(__file__),
                 str(profile.physical(fresh.__file__)):profile.sha256(fresh.__file__)})
    profile.inputs.verify_hashes(pins)
    return dict(deck=deck,entries=entries,metadata=metadata,overlay=overlay,
        unit_assets=set(native['assets']),input_hashes=pins,build_receipt=build_extension,
        layout_audit=layout_audit,raw_scale_counts=dict(scale_counts),environment=extension['environment'])


def inspect_source(manifest, build_extension):
    return extension_context(manifest,build_extension)


@contextmanager
def installed():
    """Explicit scoped source-registration adapter, with original six gates."""
    profile.require(_LOCK.acquire(blocking=False), 'Concurrent/nested reinforcement qualification forbidden')
    previous = (unit_smoke.inspect_source,fresh.QUALIFICATION_SCHEMA,fresh.POLICY,fresh.adapter_pin,fresh.qualify)
    unit_smoke.inspect_source = inspect_source
    fresh.QUALIFICATION_SCHEMA,fresh.POLICY,fresh.adapter_pin = QUALIFICATION_SCHEMA,POLICY,adapter_pin
    fresh.qualify=qualify_with_diagnostic
    try:
        yield
    finally:
        changed = (unit_smoke.inspect_source is not inspect_source or fresh.adapter_pin is not adapter_pin
                   or fresh.QUALIFICATION_SCHEMA != QUALIFICATION_SCHEMA or fresh.POLICY != POLICY
                   or fresh.qualify is not qualify_with_diagnostic)
        unit_smoke.inspect_source,fresh.QUALIFICATION_SCHEMA,fresh.POLICY,fresh.adapter_pin,fresh.qualify = previous
        _LOCK.release()
        profile.require(not changed, 'Reinforcement qualification adapter ownership changed')


def register(manifest, build_extension, output, target=32, maximum=48):
    with installed():
        return unit_smoke.prepare(manifest,build_extension,output,target,maximum)


def register_pilot(manifest, build_extension, output):
    """Register changed-row diagnostic capture, not the final six-gate smoke."""
    context = extension_context(manifest,build_extension,allow_pilot=True)
    extension = profile.read(build_extension)
    profile.require(extension['phase']=='pilot', 'Diagnostic registration requires an explicit pilot extension')
    output = profile.physical(output)
    profile.require(not output.exists(), 'New diagnostic registration output required')
    fresh.require_safe_output(output,dict(context,full=manifest,registration=build_extension))
    ids = [row['assignment']['id'] for row in extension['changes']]
    profile.require(1 <= len(ids) <= 48, 'Bounded actual changed-row diagnostic required')
    deck = copy.deepcopy(context['deck']);deck['scenes']=[context['entries'][ident] for ident in ids]
    deck.update(count=len(ids),bulk_ready=False,native_qualified=False,bulk_started=False,
        status='REGISTERED_COMMON_REINFORCEMENT_DIAGNOSTIC; not full native qualification')
    output.mkdir(parents=True)
    profile.atomic_json(output/'manifest.json',deck)
    profile.atomic_json(output/'registration.json',dict(
        schema='base5_reinforcement_changed_row_pilot_registration_v1',
        helper=profile.pin(__file__), registered_at_unix=time.time(),
        manifest=profile.pin(manifest),build_extension=profile.pin(build_extension),
        diagnostic_manifest=profile.pin(output/'manifest.json'),ids=ids,
        selected_before_capture=True,native_outcomes_used_for_selection=False,
        canonical_exact32_qualification_claimed=False,bulk_ready=False))
    return output/'manifest.json'


def verify_event_registration(path,stamp):
    # Event requests are append-only JSONL objects with phase/scene/time, not
    # selected-ledger rows: the latter reader requires an unrelated `id` field.
    events=[json.loads(line) for line in profile.physical(path).read_text().splitlines() if line.strip()]
    profile.require(events and all(isinstance(event,dict)
        and isinstance(event.get('phase'),str) and isinstance(event.get('scene'),str)
        and type(event.get('recorded_at')) in (int,float)
        and math.isfinite(event['recorded_at']) and event['recorded_at']>=stamp for event in events),
        'Actual diagnostic native events predate preregistration or have an invalid schema')
    return events


def verify_pilot(registration,capture,output=None):
    """Verify the exact changed-row diagnostic, including actual native/Park proof.

    Review/replacement rows do not qualify an unchanged proposed profile. Every
    failure remains in its real output; a revised profile needs a new pilot.
    """
    registration,capture=map(profile.physical,(registration,capture))
    declared=profile.read(registration)
    profile.require(declared.get('schema')=='base5_reinforcement_changed_row_pilot_registration_v1'
        and declared.get('helper')==profile.pin(__file__)
        and declared.get('selected_before_capture') is True
        and declared.get('native_outcomes_used_for_selection') is False,
        'Actual unchanged preregistered reinforcement diagnostic required')
    stamp=declared.get('registered_at_unix')
    profile.require(type(stamp) in (int,float) and math.isfinite(stamp) and 0<stamp<=time.time(),
                    'Finite actual diagnostic preregistration timestamp required')
    for key in ('manifest','build_extension','diagnostic_manifest'):
        record=declared[key]
        profile.require(profile.pin(record['path'])==record,'Reinforcement pilot source changed: '+key)
    diagnostic=profile.physical(declared['diagnostic_manifest']['path'])
    deck=profile.read(diagnostic);ids=declared['ids']
    profile.require(1<=len(ids)<=48 and [entry['id'] for entry in deck['scenes']]==ids
        and deck['count']==len(ids),'Diagnostic subset changed after registration')
    status=profile.read(capture/'status.json')
    profile.require(status.get('stage')=='complete' and status.get('finished')==len(ids)
        and status.get('accepted_candidate')==len(ids) and status.get('review')==0
        and status.get('failed')==0,'Every exact proposed diagnostic must be approved; review is preserved, not bypassed')
    guard=profile.read(capture/original.driver.GUARD)
    profile.require(guard.get('manifest_sha256')==profile.sha256(diagnostic)
        and guard.get('environment')==deck['config']['environment_tag']
        and guard.get('indices')==list(range(len(ids)))
        and type(guard.get('chunk_size')) is int and 1<=guard['chunk_size']<=16
        and guard.get('runtime_profile')==original.driver.profiles()
        and guard.get('runtime_hashes')==original.driver.runtime_hashes()
        and guard.get('prototype_only') is True and guard.get('qualified_release') is None,
        'Diagnostic must use the unchanged current bounded native regular/recovery driver')
    rows=original.qualification.rows(capture/'selected_ledger.jsonl')
    selected={row['id']:row for row in rows}
    profile.require(len(selected)==len(rows)==len(ids) and set(selected)==set(ids)
        and all(row.get('source_cell_id')==row['id'] for row in rows),
        'Exact registered diagnostic rows required; recovered/renamed geometry cannot qualify')
    before_inventory=original.file_inventory(capture)
    pins={str(registration):profile.sha256(registration),
          str(capture/'status.json'):profile.sha256(capture/'status.json'),
          str(capture/'selected_ledger.jsonl'):profile.sha256(capture/'selected_ledger.jsonl'),
          str(capture/original.driver.GUARD):profile.sha256(capture/original.driver.GUARD),
          str(profile.physical(__file__)):profile.sha256(__file__)}
    observations=[];sessions={};roots=set()
    child_context=dict(subset=deck,subset_path=diagnostic,
        entries={entry['id']:entry for entry in deck['scenes']},selected=set(ids))
    reference=profile.unit.load(deck['config'][profile.unit.KEY])
    for entry in deck['scenes']:
        row=selected[entry['id']];root=profile.physical(row['capture_root'])
        root.relative_to(capture)
        profile.require(root not in roots,'One raw capture cannot qualify multiple diagnostic rows');roots.add(root)
        session=root.parents[1]
        if session not in sessions:
            original_manifest,native_manifest,native_deck=original.native_child(session,child_context)
            original.driver.verify_regular(session,original_manifest)
            native_contract=profile.read(session/'session_contract.json')
            with original.tempfile.TemporaryDirectory(prefix='base5-reinforcement-native-contract-') as temporary:
                scratch=Path(temporary)/'session'
                original.prepare_session(scratch,native_manifest,
                    original.pins.ROOT/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
                    original.pins.ROOT/'configs/sensors/Sensor_competition_local.json',
                    runner_options=native_contract.get('runner_options'))
                profile.require(profile.read(scratch/'session_contract.json')==native_contract,
                                'Current complete native sensor/safety/input contract differs')
            local_rows=original.qualification.rows(session/'ledger.jsonl')
            profile.require(len(local_rows)==native_deck['count']
                and {row['id'] for row in local_rows}=={entry['id'] for entry in native_deck['scenes']}
                and all(row.get('status')=='accepted_candidate'
                    and row.get('between_scene_park',{}).get('park_verified') is True for row in local_rows),
                'Diagnostic child has a hidden review or missing between-scene Park')
            verify_event_registration(session/'event_cmd_requests.jsonl',stamp)
            sessions[session]=(native_manifest,native_deck,local_rows)
        native_manifest,native_deck,local_rows=sessions[session]
        exact=[value for value in native_deck['scenes'] if value['id']==entry['id']]
        profile.require(len(exact)==1 and exact[0]['scenario_sha256']==entry['scenario_sha256'],
                        'Diagnostic cannot be certified by a replacement payload')
        local=[value for value in local_rows if value['id']==entry['id']]
        profile.require(len(local)==1 and {key:value for key,value in row.items() if key!='source_cell_id'}==local[0],
                        'Native/parent diagnostic ledger differs')
        observed=original.qualification.verify_accepted_native(native_manifest,exact[0],native_deck,local[0])
        profile.require(observed['opt_in_attitude_evidence'] is not None,
                        'Diagnostic must prove the current native static-attitude wrapper')
        original.verify_unit_corners(root,profile.read(root/'metadata.json'),profile.read(root/'audit.json'),reference)
        original.pin_files(pins,observed['original_native_evidence'])
        original.pin_files(pins,observed['session_inventory'])
        pins[str(native_manifest)]=profile.sha256(native_manifest)
        observations.append(dict(id=entry['id'],capture_root=str(root),
            scenario_sha256=entry['scenario_sha256'],native_verified=True))
    for key in ('manifest','build_extension','diagnostic_manifest'):
        pins[declared[key]['path']]=declared[key]['sha256']
    profile.require({path.parent.resolve() for path in capture.rglob('session_contract.json')}==set(sessions),
                    'Orphan diagnostic native sessions cannot be omitted')
    profile.require(original.file_inventory(capture)==before_inventory,
                    'Diagnostic native files changed during qualification')
    extension=profile.read(declared['build_extension']['path'])
    result=dict(schema='base5_reinforcement_diagnostic_native_receipt_v1',passed=True,
        helper=profile.pin(__file__),registration=profile.pin(registration),
        capture_root=str(capture),profile=extension['profile'],allocation=extension['allocation'],
        source=extension['source'],approved=len(ids),exact_registered_ids=ids,
        observations=observations,evidence_hashes=pins,original_native_qa_unchanged=True,
        full5K_native_qualified=False,bulk_started=False)
    profile.inputs.verify_hashes(pins,'Actual reinforcement diagnostic native evidence')
    if output is not None:
        output=profile.physical(output)
        profile.require(not output.exists() and capture not in output.parents and output not in capture.parents,
                        'New native diagnostic receipt outside preserved capture required')
        profile.atomic_json(output,result)
    return result


def verify_required_diagnostic(arguments):
    receipt_path=arguments.get('reinforcement_pilot_receipt')
    profile.require(receipt_path is not None,'Actual changed-row native diagnostic receipt is required before full release')
    receipt=profile.read(receipt_path)
    profile.require(receipt.get('passed') is True,'Reinforcement diagnostic was not approved')
    actual=verify_pilot(receipt['registration']['path'],receipt['capture_root'])
    profile.require(receipt==actual,'Diagnostic receipt differs from actual preserved native evidence')
    full=profile.read(arguments['manifest']);config=full['config'][profile.CONFIG_KEY]
    profile.require(config['phase']=='full' and config['profile']['sha256']==receipt['profile']['sha256']
        and config['allocation']['sha256']==receipt['allocation']['sha256'] and config['source']==receipt['source'],
        'Full profile/allocation/source differs from its actual changed-row native diagnostic')
    entries={row['id']:row for row in full['scenes']}
    profile.require(all(row['id'] in entries and entries[row['id']]['scenario_sha256']==row['scenario_sha256']
        for row in receipt['observations']),'Full geometry changed after its exact native diagnostic')
    return receipt


def qualify_with_diagnostic(arguments):
    diagnostic=verify_required_diagnostic(arguments)
    result=BASE_QUALIFY(arguments)
    original.qualification.add_pin(result['input_hashes'],arguments['reinforcement_pilot_receipt'])
    for path,digest in diagnostic['evidence_hashes'].items():
        original.qualification.add_pin(result['input_hashes'],path,digest)
    profile.inputs.verify_hashes(result['input_hashes'],'All native qualification and changed-row diagnostic inputs')
    return result


def freeze(arguments,output):
    with installed():
        return fresh.freeze(arguments,output)


def qualified_release_context(release,environment,manifest):
    with installed():
        return fresh.qualified_release_context(release,environment,manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='action',required=True)
    for action in ('register','register-pilot'):
        command=commands.add_parser(action)
        for name in ('manifest','build-extension','output'):
            command.add_argument('--'+name,type=Path,required=True)
    diagnostic=commands.add_parser('verify-pilot')
    for name in ('registration','capture','output'):
        diagnostic.add_argument('--'+name,type=Path,required=True)
    final=commands.add_parser('freeze')
    for name in ('manifest','smoke-registration','capture-root','shadow-cases','partial-snapshot',
        'partial-resume-receipt','completed-resume-receipt','training-input-report','loader-report',
        'cpu-regression-receipt','capture-output','output'):
        final.add_argument('--'+name,type=Path,required=True)
    final.add_argument('--pytest-xml',type=Path,action='append',required=True)
    final.add_argument('--reinforcement-pilot-receipt',type=Path,required=True)
    args=parser.parse_args()
    if args.action in ('register','register-pilot'):
        function=register if args.action=='register' else register_pilot
        print(function(args.manifest,args.build_extension,args.output));return
    if args.action=='verify-pilot':
        print(verify_pilot(args.registration,args.capture,args.output));return
    values={key:([str(profile.physical(p)) for p in value] if isinstance(value,list)
                  else str(profile.physical(value))) for key,value in vars(args).items()
            if key not in ('action','output')}
    print(freeze(values,args.output))


if __name__=='__main__':
    main()
