#!/usr/bin/env python3
"""Freeze new v155 scale/extent evidence; never promote historical reviews.

Read-only reconstruction of an exact registered fresh-batch smoke, including
the two official shadow ground-reuse adoptions. Only a fresh receipt directory
is written. No simulator, capture, training, old release or raw file is changed.
"""
import argparse
from collections import Counter
from datetime import datetime,timezone
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import audit_base5_scenarios
import base5_asset_scale as scale
import base5_front_release_inputs as pins
import base5_shadow_ground_reuse as reuse
import base5_scale_capture_contract as collector_contract
import prepare_base5_scale_extent_smoke as smoke
import qualify_base5_training_inputs as training
import run_base5_fresh_completion as completion
import verify_base5_fresh_terminal_resume as terminal
from base5_capture_safety import atomic_json,collision_window,sha256
from finalize_base5_precollection_release import verify_resume,verify_training_inputs
from verify_base5_completed_resume import snapshot

SCHEMA='base5_obstacle_support_release_v1'
POLICY='registered-scale-extent-native-input-qualified-v155'
FRAMES=17


def read(path):return json.loads(Path(path).read_text())


def require(value,message):
    if not value:raise ValueError(message)


def test_results(paths):
    """Count actual unique passing testcases, not overlapping suite totals."""
    require(bool(paths),'JUnit receipts required')
    cases=set();counts=Counter();receipts={}
    for path in paths:
        path=Path(path).resolve(strict=True)
        require(str(path) not in receipts,'Duplicate JUnit receipt path')
        root=ET.parse(path).getroot()
        suites=[root] if root.tag=='testsuite' else list(root.findall('.//testsuite'))
        require(suites,'JUnit receipt contains no suites')
        require(all(int(s.get('failures','0'))==int(s.get('errors','0'))==0 for s in suites),
                'JUnit failures/errors cannot qualify a release')
        rows=list(root.iter('testcase'))
        require(rows,'JUnit receipt contains no actual testcases')
        for case in rows:
            require(case.find('failure') is None and case.find('error') is None,'Failed JUnit testcase')
            if case.find('skipped') is not None:counts['skipped']+=1;continue
            ident=(case.get('classname',''),case.get('name'))
            require(bool(ident[1]),'Unnamed JUnit testcase')
            cases.add(ident)
        counts['reported_tests']+=sum(int(s.get('tests','0')) for s in suites)
        receipts[str(path)]=sha256(path)
    require(len(cases)>=1000,'At least 1000 unique passing CPU tests required')
    return dict(unique_passed=len(cases),skipped=counts['skipped'],failures=0,errors=0,
                reported_tests=counts['reported_tests'],receipt_sha256=receipts)


def registration_context(full,registration):
    full,registration=Path(full).resolve(strict=True),Path(registration).resolve(strict=True)
    doc=read(registration);deck=read(full);subset_path=registration.parent/'manifest.json'
    require(doc.get('schema')==smoke.SCHEMA and doc.get('policy')==smoke.POLICY,
            'Explicit v155 scale/extent smoke registration required')
    require(doc.get('source_manifest_sha256')==sha256(full)
        and Path(doc['source_manifest']).resolve()==full
        and doc.get('helper_sha256')==sha256(smoke.__file__), 'Smoke registration source/helper changed')
    require(doc.get('native_qa_unchanged') is True and doc.get('bulk_started') is False
        and doc.get('training_started') is False and doc.get('live_qualified') is False,
        'Preregistration must not claim native outcomes or bulk/training')
    runtime=pins.collect_runtime(entrypoints=pins.ENTRYPOINTS+(Path(smoke.__file__).name,))
    require(doc.get('runtime_hashes')==runtime,'Smoke registered runtime differs from current source')
    for key in ('input_hashes','source_audit_hashes','qualification_evidence_hashes'):
        pins.verify_hashes(doc[key],'Smoke '+key)
    origin=deck.get('provenance',{}).get('asset_scale',{})
    original_path=Path(origin.get('source_manifest',''))
    require(original_path.is_file() and sha256(original_path)==origin.get('source_manifest_sha256'),
            'Frozen scale full5K ancestor differs')
    expected_inputs=dict(pins.collect_inputs(full))
    expected_inputs[str(original_path.resolve())]=sha256(original_path)
    expected_inputs.update(reuse.validate_manifest(deck))
    require(doc['input_hashes']==expected_inputs,'Current full input inventory differs from preregistration')
    require(doc['source_audit_hashes']==smoke.source_checks(full),'Current full source audit ancestry differs')
    subset=read(subset_path)
    entries={entry['id']:entry for entry in deck['scenes']}
    selected=doc.get('selected',[])
    require(deck.get('count')==5000 and len(deck['scenes'])==len(entries)==5000,'Exact unique full5K required')
    require(len(selected)==FRAMES and len({row['id'] for row in selected})==FRAMES,
            'Exactly seventeen preregistered smoke entries required')
    require(doc.get('subset_manifest_sha256')==sha256(subset_path)
        and subset.get('config')==deck['config'] and subset.get('count')==FRAMES
        and subset.get('scenes')==[entries[row['id']] for row in selected],
        'Smoke subset must preserve exact final full5K source entries/config')
    policy=scale.bind_config(deck['config'])[scale.CONFIG_KEY]
    pairs=smoke.approved_pairs(policy)
    require(doc.get('policy_sha256')==scale.digest(policy)
        and doc.get('qualified_profiles')==[list(pair) for pair in sorted(pairs)],'Current exact approved scale profiles differ')
    cover=set()
    for row in selected:
        entry=entries[row['id']];meta=read(entry['metadata'])
        require(row.get('source_entry_preserved') is True
            and row.get('scenario_sha256')==entry['scenario_sha256']
            and row.get('metadata_sha256')==sha256(entry['metadata']),'Registered exact source metadata/payload changed')
        features=smoke.features(meta,pairs)
        require(row.get('coverage')==sorted(features)
            and row.get('scaled_actors')==smoke.variants(meta,pairs),'Registered scale/extent coverage differs from raw source metadata')
        cover.update(features)
    coverage=doc['coverage'];required=set(coverage.get('required',[]))
    mandatory={'profile/'+name+'/'+profile for name,profile in pairs}
    mandatory.update('extent_stratum/'+value for value in ('before','intersection','after'))
    mandatory.update('extent_phase/'+value for value in smoke.PHASES)
    mandatory.update(('extent_split/train','extent_split/val'))
    require(coverage.get('coverage_passed') is True and coverage.get('native_outcomes_used_for_selection') is False
        and coverage.get('protected_frames')==8 and coverage.get('frames')==FRAMES
        and mandatory<=required<=cover and coverage.get('covered')==sorted(cover),
        'Preregistered profile/extent coverage incomplete or post-hoc')
    # Any context/role available in the actual full5K must remain in the smoke;
    # an edited registration cannot relabel a hard case as unavailable.
    available=set();metadata={entry['id']:read(entry['metadata']) for entry in deck['scenes']}
    for meta in metadata.values():available.update(smoke.features(meta,pairs))
    optional={'scaled_context/'+v for v in smoke.CONTEXTS}|{'scaled_role/primary','scaled_role/auxiliary'}
    require(optional&available<=required and set(coverage.get('unavailable_scale_context_or_roles',[]))==optional-available,
            'Available scale contexts/roles were removed from smoke')
    originals={entry['id']:entry for entry in read(original_path)['scenes']}
    unchanged={ident for ident,entry in entries.items() if entry==originals.get(ident)}
    registry=read(deck['config'][reuse.KEY]['registration']['path'])
    shadows=[row['source_id'] for row in registry['entries'] if row['mode']=='exact_source']
    target=coverage.get('target_frames')
    require(type(target) is int and target==FRAMES,
            'Preregistered fixed native target must remain exactly seventeen frames')
    rerows,recoverage=smoke.select(deck,metadata,pairs,shadows,unchanged,
                                  coverage['maximum_frames'],target=target)
    require(recoverage==coverage and [row['id'] for row in rerows]==[row['id'] for row in selected],
            'Preregistered metadata-only protected/set-cover selection changed')
    for before,after in zip(rerows,selected):
        require(all(after.get(key)==value for key,value in before.items())
            and after.get('original_ancestor_entry_preserved')==(after['id'] in unchanged),
            'Preregistered protected controls/selection features differ')
    return dict(full=full,registration=registration,doc=doc,deck=deck,subset=subset,subset_path=subset_path,
                entries=entries,selected={row['id']:row for row in selected},runtime=runtime,pairs=pairs,
                scale_policy=policy)


def verify_native(context,capture):
    capture=Path(capture).resolve(strict=True)
    reconstructed,missing,_=completion.verify_parent(context['subset_path'],capture)
    rows=training.ledger(capture/'selected_ledger.jsonl') if reconstructed else []
    require(reconstructed==rows,'Official fresh parent reconstruction differs from selected ledger')
    expected=context['selected'];actual_ids=[row.get('source_cell_id') for row in rows]
    require(len(set(actual_ids))==len(actual_ids) and set(actual_ids)<=set(expected)
        and set(missing)==set(expected)-set(actual_ids),'Native smoke logical slot accounting differs')
    status=read(capture/'status.json');blockers=[]
    require(status.get('stage') in ('complete','complete_with_deficit') and status.get('requested')==FRAMES,
            'Terminal registered seventeen-frame fresh campaign required')
    if missing or len(rows)!=FRAMES or status.get('accepted_candidate')!=FRAMES or status.get('review')!=0:
        blockers.append('current_smoke_native_approval_deficit')
    coverage=set();reused=0;native=[]
    for row in rows:
        ident=row['source_cell_id'];source=context['entries'][ident];raw=Path(row['capture_root']).resolve(strict=True)
        raw.relative_to(capture)
        audit=read(raw/'audit.json');meta=read(raw/'metadata.json')
        require(row.get('audit')==audit and sha256(raw/'scenario.json')==row['scenario_sha256'],
                'Native selected audit/payload does not equal preserved raw evidence')
        if (audit.get('review_reasons') or audit.get('accepted_candidate') is not True
                or row.get('status')!='accepted_candidate'):
            blockers.append('current_smoke_native_review:'+ident)
            continue
        collision=read(raw/'collision_history.json');summary=collision['summary']
        recomputed=collision_window(collision['samples'],summary['window_start_monotonic'],summary['window_end_monotonic'])
        peers={int(actor['unique_id']) for sample in collision['samples']
            for key in ('collision_object','collision_objecta') for actor in sample['message'].get(key,[])
            if int(actor['unique_id'])!=0}
        require(summary.get('observed') is True and summary.get('no_collision_verified') is True
            and summary.get('non_ego_uids')==[] and recomputed['observed'] and recomputed['no_collision_verified']
            and not peers,'Actual native collision evidence does not verify clear contact')
        require(read(raw.parents[1]/'shutdown_safety.json').get('park_verified') is True,'Native session final Park not verified')
        shadow=any(role.startswith('protected/shadow_exact/') for role in expected[ident]['selection_roles'])
        if shadow:
            adoption_path=raw.parents[2]/'motion_reuse_deck/manifest.json';adoption=read(adoption_path)
            require(adoption.get('config')==context['deck']['config'] and adoption.get('count')==1
                and len(adoption.get('scenes',[]))==1 and adoption['scenes'][0]['id']==ident
                and read(raw.parents[1]/'session_contract.json').get('manifest_sha256')==sha256(adoption_path)
                and adoption['scenes'][0]['scenario_sha256']==row['scenario_sha256'],
                'Official shadow adoption session/source differs')
            reuse.verify_capture(adoption['scenes'][0],raw,adoption);reused+=1
        else:
            reuse.verify_capture(source,raw,context['deck'])
            original=read(source['metadata'])
            require(row['scenario_sha256']==source['scenario_sha256']
                and all(key in meta and meta[key]==value for key,value in original.items()),
                'Non-shadow native smoke differs from exact registered full5K entry')
        training.assert_native(raw,audit,row['scenario_sha256'])
        features=smoke.features(meta,context['pairs']);coverage.update(features)
        native.append(dict(id=ident,capture_root=str(raw),scenario_sha256=row['scenario_sha256'],
            measured_components=len(audit['components']),coverage=sorted(features),shadow_ground_adoption=shadow))
    # A rejected new-runtime observation stays a review even if a later attempt
    # succeeded. Old static16 captures are outside this root and never counted.
    attempted_reviews=[]
    for path in capture.rglob('ledger.jsonl'):
        for line in path.read_text().splitlines():
            row=json.loads(line)
            if row.get('status') in ('review','failed'):attempted_reviews.append(dict(ledger=str(path),id=row.get('id'),status=row['status']))
    if attempted_reviews:blockers.append('current_smoke_has_preserved_native_review_or_failure')
    if not set(context['doc']['coverage']['required'])<=coverage:blockers.append('accepted_native_profile_or_context_coverage_incomplete')
    if reused!=2:blockers.append('two_official_shadow_ground_adoptions_not_native_approved')
    require(all(read(path.parent/'shutdown_safety.json').get('park_verified') is True
                for path in capture.rglob('session_contract.json')),'Some actual native session lacks final Park')
    return dict(requested=FRAMES,accepted=len(rows),missing=missing,observations=native,
        covered=sorted(coverage),registered_profiles=len(context['pairs']),shadow_ground_adoptions=reused,
        preserved_new_native_reviews=attempted_reviews,source_ledger=str(capture/'selected_ledger.jsonl')),sorted(set(blockers))


def verify_training(path,capture,frames):
    doc=read(path)
    expected={name:cohort.get('frames') for name,cohort in doc.get('cohorts',{}).items()}
    require(bool(expected),'Training-input report contains no cohorts')
    verified=verify_training_inputs(path,expected)
    require(verified.get('sensor_sha256')==sha256(pins.ROOT/'configs/sensors/Sensor_competition_local.json'),
            'Training-input sensor differs from current collection sensor')
    matching=[(name,cohort) for name,cohort in verified['cohorts'].items()
        if Path(cohort['source_ledger']).resolve()==(Path(capture)/'selected_ledger.jsonl').resolve()]
    require(len(matching)==1 and matching[0][1]['frames']==frames,'Training inputs lack exact current selected smoke cohort')
    return dict(report=str(Path(path).resolve()),report_sha256=sha256(path),
        smoke_cohort=matching[0][0],smoke_frames=frames,lost_retained_component_ids=0,
        extra_retained_component_ids=0,portable_hashes_verified=True,
        cohorts={name:cohort['frames'] for name,cohort in verified['cohorts'].items()})


def verify_full_collector_preflight(deck):
    """Use the actual capture admission contract, not only generator checks."""
    require(deck.get('count')==5000 and len(deck.get('scenes',[]))==5000,
            'Collector preflight requires the exact full5K source')
    # bind_config returns a new outer config and an immutable policy snapshot.
    # Never insert this runtime-only object into the source JSON/deck itself.
    view=dict(deck,config=scale.bind_config(deck['config']))
    for entry in view['scenes']:collector_contract.validate_entry(entry,view)
    scale.verify_frozen_policy(view['config'][scale.CONFIG_KEY])
    return dict(passed=True,scenes=5000,actual_collector_validate_entry=True,
        collector_contract_sha256=sha256(collector_contract.__file__),
        source_manifest_modified=False,policy_snapshot='explicit immutable config view')


def freeze(args):
    output=Path(args.output).resolve()
    require(not output.exists(),'Fresh v155 release receipt directory required')
    capture=Path(args.capture_root).resolve(strict=True)
    require(output!=capture and capture not in output.parents,'Release receipt must remain outside capture originals')
    before=snapshot(capture)
    context=registration_context(args.manifest,args.smoke_registration)
    collector_preflight=verify_full_collector_preflight(context['deck'])
    require(output!=context['full'].parent and context['full'].parent not in output.parents,
            'Release receipt must remain outside frozen full source directory')
    native,blockers=verify_native(context,capture)
    resumed=verify_resume(args.completed_resume_receipt,capture,context['subset_path'])
    require(resumed.get('requested')==FRAMES,'Completed replay is not the registered seventeen-frame smoke')
    tests=test_results(args.pytest_xml)
    inputs=verify_training(args.training_input_report,capture,native['accepted'])
    audit=audit_base5_scenarios.audit(context['full'])
    require(audit.get('status')=='OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes')==audit.get('unique_payloads')==5000,'Current exact full5K offline audit failed')
    require(snapshot(capture)==before,'Release reconstruction changed native capture evidence')
    for mapping in (context['runtime'],context['doc']['input_hashes'],context['doc']['source_audit_hashes'],
                    context['doc']['qualification_evidence_hashes']):pins.verify_hashes(mapping)
    scale.verify_frozen_policy(context['scale_policy'])
    refs={str(Path(path).resolve()):sha256(path) for path in
        [args.manifest,args.smoke_registration,args.completed_resume_receipt,args.training_input_report,
         *args.pytest_xml,__file__,context['subset_path']]}
    input_hashes=dict(context['doc']['input_hashes'])
    input_hashes.update(context['doc']['source_audit_hashes']);input_hashes.update(context['doc']['qualification_evidence_hashes'])
    input_hashes.update(refs)
    for relative,record in before['raw_files'].items():input_hashes[str(capture/relative)]=record['sha256']
    for relative,digest in before['event_logs'].items():input_hashes[str(capture/relative)]=digest
    input_hashes[str(capture/'selected_ledger.jsonl')]=before['selected_sha256']
    capture_output=Path(getattr(args,'capture_output',None) or output.parent/'collection_5k_sunny13_v155').resolve()
    require(capture_output!=capture and capture not in capture_output.parents,'Full5K output cannot overwrite native smoke')
    release=dict(schema=SCHEMA,qualification_schema='base5_scale_extent_qualification_v1',
        release_policy=POLICY,generated_at_utc=datetime.now(timezone.utc).isoformat(),
        bulk_ready=not blockers,bulk_blockers=blockers,bulk_started=False,training_started=False,
        completion_live_qualified=False,
        completion_qualification_scope='Historical ordinary/unit recovery is not claimed as live scaled-profile recovery.',
        qualified_environments=['sunny13'],full_manifest=str(context['full']),full_manifest_sha256=sha256(context['full']),
        policy=context['deck']['config']['obstacle_support_policy'],runtime_hashes=context['runtime'],input_hashes=input_hashes,
        environments=dict(sunny13=dict(manifest=str(context['full']),manifest_sha256=sha256(context['full']),
            capture_output=str(capture_output))),
        operational_hours_range=[8,12],operational_storage_budget_gib=45.,
        cost_basis='Explicit conservative operating budget; no new scene-weighted full5K timing estimate claimed.',
        source_runtime_hashes=context['runtime'],source_input_hashes=context['doc']['input_hashes'],
        receipt_input_hashes=refs,smoke=native,training_inputs=inputs,cpu_tests=tests,
        completed_resume=dict(receipt=str(Path(args.completed_resume_receipt).resolve()),additional_native_captures=0,
            raw_selected_elapsed_unchanged=True,official_fresh_terminal_verifier=True),
        full_collector_preflight=collector_preflight,
        historical_static_extent_pilot=dict(qualification_role='historical_only_not_merged_into_new17',
            raw_and_reviews_preserved=True,native_approval_count_reused_for_this_release=0),
        limitations=['Current seventeen-frame native/input regression, not a captured full5K or detector AP result.',
            'Earlier static16 approval/review history is not overwritten, relabelled or added to current approvals.',
            'Exactly two official shadow ground-preparation adoptions are allowed; their old raw is not reused as new training data.',
            'Only Sunny13 is qualified; other environments require fresh registered sources and separate native smoke.',
            'Frozen scaled deficit recovery has CPU contract checks; this receipt does not invent a separate live failure-recovery qualification.',
            'JUnit receipts are verified artifacts, not a cryptographic attestation of execution time.'])
    atomic_json(output/'offline_audit.json',audit)
    atomic_json(output/'release.json',release)
    return output/'release.json'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','smoke-registration','capture-root','completed-resume-receipt','training-input-report','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--pytest-xml',type=Path,action='append',required=True)
    parser.add_argument('--capture-output',type=Path,help='Planned new full5K output; never created by this finalizer')
    print(freeze(parser.parse_args()))


if __name__=='__main__':main()
