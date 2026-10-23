#!/usr/bin/env python3
"""Freeze differential native/CPU evidence without starting any collection.

Historical near/mid qualification remains historical. Current far32 and the
eight changed-path regressions qualify this runtime separately. No old release,
ledger, raw, training source, or native review is rewritten or upgraded.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import audit_base5_scenarios
import base5_front_release_inputs as pins
import base5_secondary_far_vehicle as far
import base5_shadow_ground_reuse as reuse
from base5_capture_safety import atomic_json, sha256
from report_base5_far_vehicle_qualification import ledger
from verify_base5_completed_resume import snapshot


POLICY = 'precollection-differential-native-and-input-qualified-v154'


def read(path):
    return json.loads(Path(path).read_text())


def require(value, message):
    if not value:
        raise ValueError(message)


def test_results(path):
    root = ET.parse(path).getroot()
    suites = [root] if root.tag == 'testsuite' else list(root.findall('testsuite'))
    require(bool(suites), 'JUnit result contains no test suites')
    counts = {key:sum(int(s.get(key, '0')) for s in suites)
              for key in ('tests', 'failures', 'errors', 'skipped')}
    require(counts['tests'] - counts['skipped'] >= 1000
            and counts['failures'] == counts['errors'] == 0,
            'Complete passing Base5 CPU regression required')
    return counts


def verify_resume(path, capture, manifest):
    doc = read(path)
    require(doc.get('passed') is True and doc.get('additional_native_captures') == 0
        and doc.get('raw_unchanged') is True and doc.get('selected_unchanged') is True
        and doc.get('elapsed_unchanged') is True
        and doc.get('manifest_sha256') == sha256(manifest)
        and Path(doc['capture_root']).resolve() == Path(capture).resolve(),
        'Current-source terminal no-capture replay required')
    from verify_base5_fresh_terminal_resume import __file__ as verifier
    current = snapshot(Path(capture))
    require(doc.get('schema') == 'base5_fresh_terminal_resume_v1'
        and doc.get('verifier_sha256') == sha256(verifier)
        and bool(current['raw_files']) and bool(current['event_logs'])
        and doc.get('before') == doc.get('after') == current,
        'Terminal replay inventory/raw/ledger/elapsed differs from its receipt')
    return doc


def verify_smoke(registration, capture, full, runtime):
    registration, capture = Path(registration), Path(capture)
    doc = read(registration)
    require(doc.get('policy') == 'precollection-native-differential-smoke-v154'
        and doc.get('source_manifest_sha256') == sha256(full)
        and doc.get('runtime_hashes') == runtime, 'Smoke source/runtime registration differs')
    pins.verify_hashes(doc['reuse_input_hashes'], 'Smoke reuse source')
    import prepare_base5_precollection_smoke as smoke
    subset_path = registration.parent/'manifest.json'
    subset = read(subset_path)
    full_deck = read(full)
    lookup = {entry['id']:entry for entry in full_deck['scenes']}
    require(doc.get('helper_sha256') == sha256(smoke.__file__)
        and doc.get('subset_manifest_sha256') == sha256(subset_path)
        and subset.get('config') == full_deck['config']
        and subset.get('count') == 8
        and subset.get('scenes') == [lookup[row['id']] for row in doc['selected']],
        'Smoke helper/subset source changed')
    import run_base5_fresh_completion as completion
    reconstructed, missing, _ = completion.verify_parent(subset_path, capture)
    selected = ledger(capture/'selected_ledger.jsonl')
    require(reconstructed == selected and not missing, 'Official smoke reconstruction differs')
    status = read(capture/'status.json')
    require(status.get('stage') == 'complete' and status.get('requested') == 8
        and status.get('accepted_candidate') == 8 and status.get('review') == 0
        and len(selected) == 8, 'Eight current smoke approvals without deficit required')
    expected = {r['id']:r for r in doc['selected']}
    require(len(expected) == 8 and {r.get('source_cell_id') for r in selected} == set(expected),
            'Smoke source identities differ')
    deck = read(full)
    entries = {e['id']:e for e in deck['scenes']}
    reused = 0
    for row in selected:
        ident, raw = row['source_cell_id'], Path(row['capture_root'])
        source = entries[ident]
        require(sha256(source['metadata']) == expected[ident]['metadata_sha256']
            and source['scenario_sha256'] == expected[ident]['scenario_sha256'], 'Smoke source changed')
        audit, collision = read(raw/'audit.json'), read(raw/'collision_history.json')['summary']
        require(row['status'] == 'accepted_candidate' and not audit.get('review_reasons')
            and row.get('audit') == audit and audit.get('accepted_candidate') is True
            and sha256(raw/'scenario.json') == row['scenario_sha256']
            and collision.get('observed') is True and collision.get('no_collision_verified') is True
            and collision.get('non_ego_uids') == [], 'Smoke native QA/contact verification failed')
        require(read(raw.parents[1]/'shutdown_safety.json').get('park_verified') is True,
                'Smoke session final Park missing')
        label = expected[ident]['stratum']
        if label.startswith('shadow_reuse_'):
            adoption_path = raw.parents[2]/'motion_reuse_deck/manifest.json'
            adoption = read(adoption_path)
            require(adoption.get('config') == deck['config']
                and adoption.get('count') == 1 and len(adoption.get('scenes', [])) == 1
                and adoption['scenes'][0]['id'] == ident
                and read(raw.parents[1]/'session_contract.json').get('manifest_sha256') == sha256(adoption_path)
                and adoption['scenes'][0]['scenario_sha256'] == row['scenario_sha256'],
                'Smoke shadow adoption differs from its actual session/source')
            reuse.verify_capture(adoption['scenes'][0], raw, adoption)
        else:
            reuse.verify_capture(source, raw, deck)
        if label.startswith('intersection_'):
            actual = read(raw/'metadata.json')
            require(actual['spec'].get('family') == 'pedestrian_related'
                and actual['spec'].get('mission') is True
                and any(a.get('motion_path', {}).get('ground_path_evidence', {}).get('policy')
                        == 'editor-intersection-path-v116' for a in actual['runtime_actors']),
                'Intersection smoke lacks its actual editor path contract')
        if not label.startswith('shadow_reuse_'):
            source_metadata, actual_metadata = read(source['metadata']), read(raw/'metadata.json')
            require(row['scenario_sha256'] == source['scenario_sha256']
                and all(key in actual_metadata and actual_metadata[key] == value
                        for key, value in source_metadata.items()),
                'Smoke native payload/metadata differs from its exact registered source')
        if reuse.KEY in read(raw/'metadata.json'):
            reused += 1
    require(reused == 2, 'Both registered ground reuses require new native observations')
    return dict(accepted=8, shadow_preparation_reused=2, old_raw_used_as_new_training=False)


def verify_training_inputs(path, expected_frames):
    import qualify_base5_training_inputs as qualification
    path = Path(path)
    doc = read(path)
    require(doc.get('schema') == qualification.SCHEMA
        and doc.get('report_script_sha256') == sha256(qualification.__file__)
        and doc.get('training_started') is False and doc.get('capture_started') is False
        and {name:r.get('frames') for name,r in doc.get('cohorts', {}).items()} == expected_frames,
        'Exact native/Q90/upright cohort qualification required')
    helpers = dict(doc['helper_sha256'])
    # Direct CLI execution records __file__ relative to the repository, while
    # imported helpers record absolute paths. Normalize only this known script;
    # never resolve arbitrary relative pins against an operator's working dir.
    script = Path(qualification.__file__).resolve()
    relative_script = str(script.relative_to(pins.ROOT))
    if relative_script in helpers:
        digest = helpers.pop(relative_script)
        require(str(script) not in helpers or helpers[str(script)] == digest,
                'Conflicting qualification script pins')
        helpers[str(script)] = digest
    pins.verify_hashes(helpers, 'Training input qualification helper')
    for name, cohort in doc['cohorts'].items():
        require(cohort.get('source_unchanged') is True
            and cohort.get('lost_retained_component_ids') == cohort.get('extra_retained_component_ids') == 0
            and cohort.get('full_rpy_points_recounted') is True
            and cohort.get('upright_corner_enclosure_verified') is True
            and sha256(cohort['source_ledger']) == cohort['source_ledger_sha256'],
            'Training cohort provenance/geometry qualification failed')
        source_snapshot = path.parent/name/'source_snapshot.json'
        require(sha256(source_snapshot) == cohort['source_snapshot_sha256']
            and read(source_snapshot) == qualification.snapshot(
                cohort['source_ledger'], qualification.ledger(cohort['source_ledger'])),
            'Native training source inventory changed after conversion')
        package = Path(cohort['training3_package'])
        require(sha256(package/'manifest.json') == cohort['training3_manifest_sha256'],
                'Training package manifest changed')
        qualification.portable.validate(package, verify_hashes=True)
    return doc


def freeze(a):
    require(not a.output.exists(), 'Fresh release directory required')
    runtime = pins.collect_runtime()
    receipt = read(a.runtime_receipt)
    require(receipt.get('runtime_hashes') == runtime, 'Runtime changed since CPU test snapshot')
    tests = test_results(a.test_results)
    deck = read(a.manifest)
    far_doc, far_registration = read(a.far_report), read(a.far_registration)
    geometry_source = Path(far_registration['source_candidate'])
    geometry = read(geometry_source)
    expected = json.loads(json.dumps(geometry))
    expected['config'][reuse.KEY] = deck['config'][reuse.KEY]
    require(deck == expected, 'Final manifest changed geometry/spec outside explicit reuse config')
    require(Path(far_doc['baseline']).resolve() == a.far_capture.resolve()
        and Path(far_doc['manifest']).resolve() == a.far_manifest.resolve()
        and Path(far_doc['registration']).resolve() == a.far_registration.resolve(),
        'Far report is bound to another capture/manifest/registration')
    from report_base5_far_vehicle_qualification import report as reconstruct_far
    regenerated = reconstruct_far(a.far_registration, a.far_manifest, a.far_capture,
                                 a.output.parent/(a.output.name+'_far_reconstructed.json'))
    require(regenerated['observations'] == far_doc['observations']
        and regenerated['final_accepted'] == far_doc['final_accepted']
        and regenerated['remaining'] == far_doc['remaining'], 'Far native reconstruction differs')
    require(far_doc.get('native_support_qualified_all_registered_targets') is True
        and far_doc.get('requested') == far_doc.get('final_accepted') == 32
        and far_doc.get('remaining') == []
        and far_doc.get('registration_sha256') == sha256(a.far_registration)
        and far_registration.get('source_candidate_sha256') == sha256(geometry_source),
        'Current far32 qualification incomplete or stale')
    pins.verify_hashes(far_doc['source_input_sha256'], 'Current far qualification')
    historical = read(a.historical_release)
    require(historical.get('bulk_ready') is True and historical.get('completion_live_qualified') is True,
            'Historical bounded completion qualification required')
    base_sha = historical['environments']['sunny13']['manifest_sha256']
    count = 0
    for entry in deck['scenes']:
        meta = read(entry['metadata'])
        if far.has_plan(meta):
            require(meta['spec'][far.PLAN_KEY]['source_manifest']['sha256'] == base_sha,
                    'Far allocation does not preserve the qualified historical base')
            count += 1
    require(count == 160, 'Frozen far allocation count changed')
    reuse_inputs = reuse.validate_manifest(deck)
    smoke = verify_smoke(a.smoke_registration, a.smoke_capture, a.manifest, runtime)
    verify_resume(a.far_resume, a.far_capture, a.far_manifest)
    verify_resume(a.smoke_resume, a.smoke_capture, a.smoke_manifest)
    training = verify_training_inputs(a.training_inputs,
        dict(static64=64, moving8=8, range53=53, far32=32, compound22=22))
    require(sum(r['retained_gt'] for r in training['cohorts'].values()) == 601,
            'Historical preregistered 601 GT conversion differs')
    current_training = verify_training_inputs(a.current_training_inputs, dict(current_far32=32, current_smoke8=8))
    require(current_training['cohorts']['current_far32']['far_targets'] == 32,
            'Every current far target must survive native-to-training conversion')
    require(Path(current_training['cohorts']['current_far32']['source_ledger']).resolve()
                == (a.far_capture/'selected_ledger.jsonl').resolve()
        and Path(current_training['cohorts']['current_smoke8']['source_ledger']).resolve()
                == (a.smoke_capture/'selected_ledger.jsonl').resolve(),
        'Current training input cohorts are bound to another capture')
    inputs = pins.collect_inputs(a.manifest)
    inputs.update(reuse_inputs)
    for name in ('runtime_receipt','test_results','far_report','far_registration','far_manifest',
                 'far_resume','smoke_registration','smoke_manifest','smoke_resume',
                 'training_inputs','current_training_inputs','historical_release'):
        path = Path(getattr(a,name)).resolve()
        inputs[str(path)] = sha256(path)
    inputs[str(Path(__file__).resolve())] = sha256(__file__)
    audit = audit_base5_scenarios.audit(a.manifest)
    require(audit.get('status') == 'OFFLINE_PASS' and audit.get('unique_payloads') == 5000,
            'Final full5K offline audit failed')
    pins.verify_hashes(runtime, 'Release runtime')
    pins.verify_hashes(inputs, 'Release input')
    release = dict(schema='base5_obstacle_support_release_v1', release_policy=POLICY,
        generated_at_utc=datetime.now(timezone.utc).isoformat(), policy=deck['config']['obstacle_support_policy'],
        qualified_environments=['sunny13'], bulk_ready=True, bulk_blockers=[], bulk_started=False,
        completion_live_qualified=True, runtime_hashes=runtime, input_hashes=inputs,
        environments=dict(sunny13=dict(manifest=str(a.manifest.resolve()), manifest_sha256=sha256(a.manifest),
            capture_output=str(a.capture_output.resolve()))),
        qualification_basis=dict(current_far_native=32, current_changed_path_smoke=smoke,
            historical_geometry_and_completion=str(a.historical_release.resolve()),
            historical_evidence_is_not_new_runtime_native=True, cpu_tests=tests,
            training_input_qualification=str(a.training_inputs.resolve()),
            current_training_input_qualification=str(a.current_training_inputs.resolve())),
        estimated_5k_hours=historical['estimated_5k_hours'], estimated_5k_gib=historical['estimated_5k_gib'],
        operational_hours_range=[8,12], operational_storage_budget_gib=45.,
        cost_basis='Historical v152 group-weighted estimate; fresh terrain and failures still cost time.',
        limitations=['Sunny13 only; other five environments require fresh-source policies and separate live smoke.',
            'Native support and conversion qualification, not new model AP or a guarantee of every5K capture.',
            'Original windows survive bounded recovery; uniform retained-GT distance is not guaranteed.',
            '5K bulk capture and training have not started.'])
    atomic_json(a.output/'offline_audit.json', audit)
    atomic_json(a.output/'release.json', release)
    return a.output/'release.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot-runtime', type=Path)
    for name in ('manifest','far-report','far-registration','far-capture','far-manifest','far-resume',
                 'smoke-registration','smoke-capture','smoke-manifest','smoke-resume','training-inputs','current-training-inputs',
                 'historical-release','runtime-receipt','test-results','output','capture-output'):
        parser.add_argument('--'+name, type=Path)
    args = parser.parse_args()
    if args.snapshot_runtime:
        if args.snapshot_runtime.exists():
            parser.error('Fresh runtime snapshot required')
        atomic_json(args.snapshot_runtime, dict(runtime_hashes=pins.collect_runtime()))
    else:
        if any(getattr(args,k.replace('-','_')) is None for k in ('manifest','far-report','far-registration',
                'far-capture','far-manifest','far-resume','smoke-registration','smoke-capture','smoke-manifest',
                'smoke-resume','training-inputs','current-training-inputs','historical-release','runtime-receipt','test-results','output','capture-output')):
            parser.error('All qualification inputs are required')
        print(freeze(args))
