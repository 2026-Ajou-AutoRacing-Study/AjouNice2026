"""새 환경 승인 수와 실제 원문·재개·runtime 근거를 분리하는 회귀 검사."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import finalize_base5_environment_release as final


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))


@pytest.fixture
def child(tmp_path):
    entries = [dict(id='a'), dict(id='b')]
    deck = dict(config=dict(environment_tag='sunny15'), status='registered', count=2, scenes=entries)
    path = write(tmp_path/'manifest.json', deck)
    context = dict(subset=deck, entries={e['id']:e for e in entries}, selected={e['id']:{} for e in entries})
    return path, context


def test_actual_child_exact_order_and_header_are_required(child):
    path, context = child
    assert final.exact_child(path, context)['count'] == 2


@pytest.mark.parametrize('kind', ['header', 'count', 'foreign', 'entry', 'order', 'duplicate', 'too_large'])
def test_changed_child_is_not_exact_native(child, kind):
    path, context = child;deck = final.read(path)
    if kind == 'header':deck['config']['environment_tag'] = 'sunny13'
    if kind == 'count':deck['count'] = 1
    if kind == 'foreign':deck['scenes'][0]['id'] = 'foreign'
    if kind == 'entry':deck['scenes'][0]['scenario_sha256'] = 'different'
    if kind == 'order':deck['scenes'].reverse()
    if kind == 'duplicate':deck['scenes'][1] = copy.deepcopy(deck['scenes'][0])
    if kind == 'too_large':deck.update(count=17, scenes=[dict(id=str(i)) for i in range(17)])
    write(path, deck)
    with pytest.raises(ValueError):final.exact_child(path, context)


def test_full_native_file_inventory_detects_new_file_and_symlink(tmp_path):
    write(tmp_path/'raw'/'metadata.json', {})
    before = final.file_inventory(tmp_path/'raw')
    write(tmp_path/'raw'/'extra.json', {})
    assert final.file_inventory(tmp_path/'raw') != before
    (tmp_path/'raw'/'alias').symlink_to(tmp_path/'raw'/'metadata.json')
    with pytest.raises(ValueError, match='symlink'):final.file_inventory(tmp_path/'raw')


@pytest.fixture
def paused(tmp_path, monkeypatch):
    profile = dict(test='actual-profile');hashes = {str(tmp_path/'runtime.py'):'hash'}
    monkeypatch.setattr(final.driver, 'profiles', lambda:copy.deepcopy(profile))
    monkeypatch.setattr(final.driver, 'runtime_hashes', lambda:dict(hashes))
    deck = dict(config=dict(environment_tag='sunny15'), count=32, scenes=[dict(id=f'id{i:02d}') for i in range(32)])
    manifest = write(tmp_path/'registered'/'manifest.json', deck)
    capture = tmp_path/'capture';rows = []
    session = capture/'chunk_0000'/'capture'
    write(session/'session_contract.json', {})
    write(session/'shutdown_safety.json', dict(park_verified=True))
    write(session/'event_cmd_requests.jsonl', dict(recorded_at=1))
    for entry in deck['scenes'][:16]:
        raw = session/'captures'/entry['id'];write(raw/'metadata.json', {})
        rows.append(dict(id=entry['id'], source_cell_id=entry['id'], capture_root=str(raw), status='accepted_candidate'))
    write_rows(session/'ledger.jsonl', rows);write_rows(capture/'selected_ledger.jsonl', rows)
    write(capture/'status.json', dict(stage='paused', requested=32, elapsed_sec=16))
    write(capture/final.driver.GUARD, dict(manifest_sha256=final.sha256(manifest), chunk_size=16,
        runtime_hashes=hashes, runtime_profile=profile))
    return manifest, capture, rows, tmp_path/'partial.json'


def finish(paused):
    manifest, capture, old, saved = paused
    rows = copy.deepcopy(old);session = capture/'chunk_0001'/'capture'
    write(session/'session_contract.json', {});write(session/'shutdown_safety.json', dict(park_verified=True))
    write(session/'event_cmd_requests.jsonl', dict(recorded_at=2))
    for entry in final.read(manifest)['scenes'][16:]:
        raw = session/'captures'/entry['id'];write(raw/'metadata.json', {})
        rows.append(dict(id=entry['id'], source_cell_id=entry['id'], capture_root=str(raw), status='accepted_candidate'))
    write_rows(session/'ledger.jsonl', rows[16:]);write_rows(capture/'selected_ledger.jsonl', rows)
    write(capture/'status.json', dict(stage='complete', requested=32, elapsed_sec=32))
    return rows


def test_real_chunk16_partial_preservation_is_not_four_row_adapter(paused):
    manifest, capture, _, saved = paused
    final.save_partial_snapshot(manifest, capture, saved)
    finish(paused)
    report = final.verify_partial(saved, manifest, capture)
    assert report['passed'] and report['preserved_selected'] == 16 and report['selected_after'] == 32
    assert not report['resume_executed']


@pytest.mark.parametrize('kind', ['raw', 'new_raw_file', 'new_session_file', 'session', 'event', 'selected', 'profile', 'snapshot_helper'])
def test_partial_resume_must_preserve_every_prior_evidence(paused, kind):
    manifest, capture, old, saved = paused
    final.save_partial_snapshot(manifest, capture, saved);finish(paused)
    root = Path(old[0]['capture_root']);session = root.parents[1]
    if kind == 'raw':write(root/'metadata.json', dict(changed=True))
    if kind == 'new_raw_file':write(root/'new.json', {})
    if kind == 'new_session_file':write(session/'new.json', {})
    if kind == 'session':write(session/'session_contract.json', dict(changed=True))
    if kind == 'event':write(session/'event_cmd_requests.jsonl', dict(recorded_at=99))
    if kind == 'selected':
        rows = final.qualification.rows(capture/'selected_ledger.jsonl');rows[0]['status'] = 'review'
        write_rows(capture/'selected_ledger.jsonl', rows)
    if kind == 'profile':
        doc = final.read(saved);doc['runtime_profile'] = {};write(saved, doc)
    if kind == 'snapshot_helper':
        doc = final.read(saved);doc['helper_sha256'] = 'changed';write(saved, doc)
    with pytest.raises(ValueError):final.verify_partial(saved, manifest, capture)


def test_partial_snapshot_requires_pause_and_new_external_output(paused):
    manifest, capture, _, _ = paused
    with pytest.raises(ValueError, match='outside raw'):
        final.save_partial_snapshot(manifest, capture, capture/'snapshot.json')
    write(capture/'status.json', dict(stage='running', elapsed_sec=16))
    with pytest.raises(ValueError, match='Paused'):
        final.save_partial_snapshot(manifest, capture, capture.parent/'snapshot.json')


def test_completed_receipt_requires_new_driver_actual_no_capture_replay(paused, tmp_path):
    manifest, capture, _, _ = paused;finish(paused)
    current = final.completed_snapshot(capture);guard = final.read(capture/final.driver.GUARD)
    receipt = write(tmp_path/'completed.json', dict(schema='base5_static_attitude_batch_terminal_resume_v1',
        passed=True, driver_sha256=final.sha256(final.driver.__file__), runtime_profile=final.driver.profiles(),
        manifest_sha256=final.sha256(manifest), capture_root=str(capture), requested=32, chunk_size=guard['chunk_size'],
        parent_contract_sha256=final.sha256(capture/final.driver.GUARD), additional_native_captures=0,
        new_subprocesses_permitted=False, before=current, after=current))
    assert final.verify_completed(receipt, manifest, capture)['passed']
    document = final.read(receipt);document['new_subprocesses_permitted'] = True;write(receipt, document)
    with pytest.raises(ValueError, match='completed no-capture'):
        final.verify_completed(receipt, manifest, capture)


def test_historical_source_cannot_be_accepted_as_new_release(tmp_path):
    manifest = write(tmp_path/'manifest.json', {})
    for value in ({}, dict(schema=final.SCHEMA, bulk_ready=True),
        dict(schema=final.SCHEMA, qualification_schema=final.QUALIFICATION_SCHEMA, release_policy=final.POLICY,
            helper_sha256=final.sha256(final.__file__), bulk_ready=False)):
        release = write(tmp_path/'release.json', value)
        with pytest.raises(ValueError, match='Current separately qualified'):
            final.qualified_release_context(release, 'sunny15', manifest)


def test_bulk_ready_is_never_inferred_from_mock_or_missing_gate_files(tmp_path, monkeypatch):
    manifest = write(tmp_path/'source'/'manifest.json', {})
    release = write(tmp_path/'release.json', dict(schema=final.SCHEMA,
        qualification_schema=final.QUALIFICATION_SCHEMA, release_policy=final.POLICY,
        helper_sha256=final.sha256(final.__file__), bulk_ready=True, bulk_started=False, training_started=False,
        completion_live_qualified=False, qualified_environments=['sunny15'], environment='sunny15',
        full_manifest=str(manifest), full_manifest_sha256=final.sha256(manifest),
        qualification_inputs=dict(manifest=str(manifest), capture_output=str(tmp_path/'planned')),
        capture_output=str(tmp_path/'changed'), environments={}))
    with pytest.raises(ValueError, match='output/environment'):
        final.qualified_release_context(release, 'sunny15', manifest)


@pytest.mark.parametrize('kind', ['review', 'failed', 'missing', 'not_complete', 'old_profile', 'old_driver'])
def test_native_gate_rejects_old_or_incomplete_pilot_before_geometry(tmp_path, monkeypatch, kind):
    capture = tmp_path/'raw';manifest = write(tmp_path/'registered'/'manifest.json', {})
    count = 32;state = dict(stage='complete', requested=count, accepted_candidate=count, review=0, failed=0)
    if kind == 'review':state['review'] = 1
    if kind == 'failed':state['failed'] = 1
    if kind == 'missing':state['accepted_candidate'] = 31
    if kind == 'not_complete':state['stage'] = 'paused'
    guard = dict(manifest_sha256=final.sha256(manifest), environment='sunny15', indices=list(range(count)),
        chunk_size=16, runtime_profile=final.driver.profiles(), runtime_hashes=final.driver.runtime_hashes(),
        prototype_only=True, qualified_release=None)
    if kind == 'old_profile':guard['runtime_profile'] = {}
    if kind == 'old_driver':guard['runtime_hashes'] = {}
    write(capture/'status.json', state);write(capture/final.driver.GUARD, guard)
    context = dict(selected={str(i):{} for i in range(count)}, subset_path=manifest, environment='sunny15')
    with pytest.raises(ValueError):final.verify_native(context, capture)


def test_readiness_claim_requires_each_generated_gate_to_recompute(tmp_path, monkeypatch):
    source = write(tmp_path/'source'/'manifest.json', {})
    assembly = write(tmp_path/'assembly.json', dict(source_manifest=str(tmp_path/'original'/'manifest.json')))
    helper = write(tmp_path/'code.py', {})
    hashes = {str(helper):final.sha256(helper)}
    results = {name:dict(passed=True, proof=name) for name in final.GATES}
    results['native_exact']['accepted'] = 32
    shadow = write(tmp_path/'shadow'/'native.json', {}).parent
    results['independent_shadow']['protected_native_roots'] = [str(shadow)]
    gates = {}
    for name,result in results.items():
        path = write(tmp_path/'gates'/(name+'.json'), result)
        gates[name] = dict(passed=True, path=str(path), sha256=final.sha256(path))
    audit = write(tmp_path/'audit.json', dict(status='OFFLINE_PASS'))
    arguments = dict(manifest=str(source), capture_output=str(tmp_path/'planned'), capture_root=str(tmp_path/'raw'))
    runtime = {str(helper):final.sha256(helper)}
    document = dict(schema=final.SCHEMA, qualification_schema=final.QUALIFICATION_SCHEMA,
        release_policy=final.POLICY, helper_sha256=final.sha256(final.__file__), bulk_ready=True,
        bulk_started=False, training_started=False, completion_live_qualified=False,
        qualified_environments=['sunny15'], environment='sunny15', full_manifest=str(source),
        full_manifest_sha256=final.sha256(source), qualification_inputs=arguments,
        capture_output=str(tmp_path/'planned'),
        environments=dict(sunny15=dict(manifest=str(source),manifest_sha256=final.sha256(source),capture_output=str(tmp_path/'planned'))),
        qualified_runtime=dict(profile=final.PROFILE, runtime_profile=final.driver.profiles(),
            entrypoints=[str(Path(final.driver.__file__).resolve()),str(final.driver.CHILD),str(final.driver.RECOVERY)]),
        runtime_hashes=runtime, input_hashes=hashes, native_exact_observations=32,
        bulk_training_observations=0, gate_evidence=gates,
        full_cpu_audit=dict(path=str(audit),sha256=final.sha256(audit)))
    monkeypatch.setattr(final.cpu, 'runtime_hashes', lambda:runtime)
    monkeypatch.setattr(final, 'qualify', lambda args:dict(context=dict(assembly_receipt=assembly),
        results=copy.deepcopy(results), input_hashes=hashes, runtime_hashes=runtime))
    release = write(tmp_path/'release.json', document)
    assert final.qualified_release_context(release, 'sunny15', source)['native_exact_observations'] == 32
    changed = copy.deepcopy(document);del changed['gate_evidence']['independent_shadow'];write(release, changed)
    with pytest.raises(ValueError, match='All six'):
        final.qualified_release_context(release, 'sunny15', source)
    write(release, document);write(Path(gates['native_exact']['path']), dict(passed=True, accepted=5000))
    with pytest.raises(ValueError, match='Qualified gate'):
        final.qualified_release_context(release, 'sunny15', source)


@pytest.mark.parametrize('name,pool,training,components,expected', [
    ('Dog1','animal','animal',['animal'],'animal'),
    ('Ped_Pet1','pet_person','component_annotation',['pedestrian','animal'],'pedestrian_or_compound'),
    ('BabyStroller1','stroller','component_annotation',['pedestrian','obstacle','pedestrian'],'pedestrian_or_compound'),
    ('Pedestrian1','person','component_annotation',['pedestrian'],'pedestrian_or_compound'),
])
def test_shadow_category_preserves_actual_roster_and_component_semantics(name,pool,training,components,expected):
    meta = dict(spec=dict(subfamily=pool), runtime_actors=[dict(unique_id=8500,name=name,
        population_pool=pool,training_class=training,motion='walking')])
    audit = dict(components=[dict(parent_uid=8500,training_class=kind) for kind in components])
    assert final.shadow_category(meta,audit) == expected
    assert meta['spec']['subfamily'] == pool


def test_shadow_category_does_not_relabel_compound_or_animal_without_native_evidence():
    meta = dict(runtime_actors=[dict(unique_id=8500,name='Ped_Pet1',population_pool='pet_person',
        training_class='component_annotation',motion='running')])
    with pytest.raises(ValueError, match='child native'):
        final.shadow_category(meta,dict(components=[dict(parent_uid=8500,training_class='pedestrian')]))


@pytest.fixture
def direction_case(tmp_path, monkeypatch):
    from types import SimpleNamespace
    meta = dict(spec=dict(subfamily='pet_person'), runtime_actors=[], **{final.unit.KEY:{'sha256':'unit'}})
    oldmeta = write(tmp_path/'original_meta.json', meta)
    old = dict(id='canonical', metadata=str(oldmeta), scenario_sha256='scenario')
    actual_meta = copy.deepcopy(meta);actual_meta['training_split_overlay'] = dict(path='new')
    actual = write(tmp_path/'new_meta.json', actual_meta)
    deck = dict(config=dict(environment_tag='sunny15', **{final.unit.KEY:{'sha256':'unit'}}),
        count=1, scenes=[dict(old, metadata=str(actual))])
    manifest = write(tmp_path/'campaign_manifest.json', deck)
    helper = write(tmp_path/'adapter.py', {})
    proof = dict(passed=True, source_id='canonical', native_qa_unchanged=True,
        input_hashes={str(helper):final.sha256(helper)}, prepared_manifest='prepared', selected_ledger='selected')
    module = SimpleNamespace(__file__=str(helper), verify_completed_campaign=lambda *args:copy.deepcopy(proof))
    monkeypatch.setattr(final.importlib, 'import_module', lambda name:module)
    root = tmp_path/'campaign';root.mkdir()
    case = dict(campaign_manifest=str(manifest), campaign_index=0,campaign_root=str(root))
    return case, deck, old, proof, module, actual


def test_new_direction_shadow_api_requires_actual_canonical_source_and_guard(direction_case):
    case, original, old, proof, _, _ = direction_case;hashes = {}
    assert final.direction_case(case, 'canonical', original, old, hashes) == proof
    assert str(Path(case['campaign_manifest'])) in hashes


@pytest.mark.parametrize('kind', ['no_api','not_passed','different_source','qa_changed','foreign_geometry','spec_changed'])
def test_new_direction_shadow_cannot_promote_old_or_foreign_proof(direction_case, kind):
    case, original, old, proof, module, actual = direction_case
    if kind == 'no_api':module.verify_completed_campaign = None
    if kind == 'not_passed':proof['passed'] = False
    if kind == 'different_source':proof['source_id'] = 'foreign'
    if kind == 'qa_changed':proof['native_qa_unchanged'] = False
    if kind == 'foreign_geometry':
        deck = final.read(case['campaign_manifest']);deck['scenes'][0]['scenario_sha256'] = 'foreign'
        write(Path(case['campaign_manifest']), deck)
    if kind == 'spec_changed':write(actual, dict(spec=dict(subfamily='animal'),runtime_actors=[]))
    with pytest.raises(ValueError):final.direction_case(case, 'canonical', original, old, {})


def test_actual_execution_manifest_copy_is_bound_to_original_chunk(child, tmp_path):
    path, context = child
    chunk = tmp_path/'chunk';chunk.mkdir()
    original = chunk/'manifest.json';original.write_bytes(path.read_bytes())
    session = chunk/'capture';session.mkdir()
    execution = final.driver.prepare_execution_source(original,session)
    actual_original,actual_execution,deck = final.native_child(session,context)
    assert actual_original == original and actual_execution == execution
    assert deck == final.read(original)
    assert execution.parent.name == final.driver.EXECUTION_SOURCE
    execution.write_text(json.dumps(deck,indent=2)+'\n')
    with pytest.raises(ValueError,match='Execution source bytes'):
        final.native_child(session,context)


@pytest.fixture
def shadow_scope(tmp_path):
    accepted_session = tmp_path/'shadow_campaign'/'case'/'initial'
    accepted = write(accepted_session/'captures'/'accepted'/'audit.json', {}).parent
    write(accepted_session/'shutdown_safety.json', dict(park_verified=True))
    review_session = tmp_path/'old_shadow_review'/'initial'
    review = write(review_session/'captures'/'review'/'audit.json', {}).parent
    write(review_session/'shutdown_safety.json', dict(park_verified=True))
    manifest = write(tmp_path/'shadow_source'/'manifest.json', {})
    document = dict(preserved_review_captures=[str(review)],cases=[dict(adapter='final_direction_reference',
        campaign_root=str(tmp_path/'shadow_campaign'),campaign_manifest=str(manifest))])
    return document,accepted,accepted_session,review,review_session


@pytest.mark.parametrize('which',['accepted','accepted_session','review','review_session','source','alias'])
def test_shadow_source_native_and_session_outputs_are_all_protected(shadow_scope,tmp_path,which):
    document,accepted,accepted_session,review,review_session = shadow_scope
    roots = final.shadow_declared_protected_roots(document)
    choices = dict(accepted=accepted,accepted_session=accepted_session,review=review,
        review_session=review_session,source=Path(document['cases'][0]['campaign_manifest']).parent)
    if which == 'alias':
        parent = tmp_path/'alias';parent.symlink_to(accepted_session,target_is_directory=True)
    else:parent = choices[which]
    output = parent/'new_output'
    before = sorted(str(p) for p in tmp_path.rglob('*'))
    with pytest.raises(ValueError,match='preserved shadow'):
        final.require_shadow_output_safe(output,roots)
    assert not output.exists() and sorted(str(p) for p in tmp_path.rglob('*')) == before


@pytest.mark.parametrize('role',['release','planned'])
def test_freeze_rejects_shadow_overlap_before_qualification_or_writes(shadow_scope,tmp_path,monkeypatch,role):
    document,accepted,_,_,review_session = shadow_scope
    source = write(tmp_path/'final_source'/'manifest.json', {})
    original = write(tmp_path/'old_source'/'manifest.json', {})
    assembly = write(tmp_path/'assembly.json',dict(source_manifest=str(original)))
    registration = write(tmp_path/'registration.json',dict(assembly_receipt=dict(path=str(assembly))))
    capture = write(tmp_path/'exact_native'/'status.json',{}).parent
    cases = write(tmp_path/'shadow_cases.json',document)
    output = accepted/'new_release' if role == 'release' else tmp_path/'release'
    planned = review_session/'new_bulk' if role == 'planned' else tmp_path/'new_bulk'
    arguments = dict(manifest=str(source),smoke_registration=str(registration),capture_root=str(capture),
        capture_output=str(planned),shadow_cases=str(cases))
    monkeypatch.setattr(final,'qualify',lambda *args:pytest.fail('Unsafe path must be rejected before qualification'))
    before = sorted(str(p) for p in tmp_path.rglob('*'))
    with pytest.raises(ValueError,match='preserved shadow'):
        final.freeze(arguments,output)
    assert not output.exists() and not planned.exists()
    assert sorted(str(p) for p in tmp_path.rglob('*')) == before


@pytest.mark.parametrize('location',['accepted_raw','accepted_session','review_raw','review_session'])
def test_shadow_inventory_detects_new_files_not_only_old_hashes(shadow_scope,location):
    _,accepted,accepted_session,review,review_session = shadow_scope
    roots = [accepted_session,review_session]
    result = dict(protected_native_roots=[str(p) for p in roots],
        native_inventories={str(root):final.file_inventory(root) for root in roots},
        evidence_hashes={str(accepted/'audit.json'):final.sha256(accepted/'audit.json')})
    final.verify_shadow_preserved(result)
    choices = dict(accepted_raw=accepted,accepted_session=accepted_session,review_raw=review,review_session=review_session)
    write(choices[location]/'new_file.json',{})
    with pytest.raises(ValueError,match='including new files'):
        final.verify_shadow_preserved(result)


@pytest.mark.parametrize('phase',['audit','writing'])
def test_freeze_rechecks_shadow_inventory_before_and_after_writing(shadow_scope,tmp_path,monkeypatch,phase):
    document,accepted,accepted_session,review,review_session = shadow_scope
    source = write(tmp_path/'final_source'/'manifest.json', {})
    original = write(tmp_path/'old_source'/'manifest.json', {})
    assembly = write(tmp_path/'assembly.json',dict(source_manifest=str(original)))
    registration = write(tmp_path/'registration.json',dict(assembly_receipt=dict(path=str(assembly))))
    capture = write(tmp_path/'exact_native'/'status.json',{}).parent
    cases = write(tmp_path/'shadow_cases.json',document)
    helper = write(tmp_path/'helper.py',{})
    shadow_result = dict(passed=True,protected_native_roots=[str(accepted_session),str(review_session)],
        native_inventories={str(root):final.file_inventory(root) for root in (accepted_session,review_session)},
        evidence_hashes={str(accepted/'audit.json'):final.sha256(accepted/'audit.json')})
    results = {name:dict(passed=True) for name in final.GATES}
    results['independent_shadow'] = shadow_result;results['native_exact']['accepted'] = 32
    monkeypatch.setattr(final,'qualify',lambda *args:dict(context=dict(environment='sunny15'),results=results,
        input_hashes={str(helper):final.sha256(helper)},runtime_hashes={str(helper):final.sha256(helper)}))
    def audit(*args):
        if phase == 'audit':write(accepted/'added_during_audit.json',{})
        return dict(status='OFFLINE_PASS',errors=[],scenes=5000,unique_payloads=5000)
    monkeypatch.setattr(final.scenario_audit,'audit',audit)
    original_atomic = final.atomic_json
    def atomic(path,value):
        original_atomic(path,value)
        if phase == 'writing' and path.name == 'native_exact.json':write(review_session/'added_during_writing.json',{})
    monkeypatch.setattr(final,'atomic_json',atomic)
    arguments = dict(manifest=str(source),smoke_registration=str(registration),capture_root=str(capture),
        capture_output=str(tmp_path/'new_bulk'),shadow_cases=str(cases))
    output = tmp_path/'release'
    with pytest.raises(ValueError,match='including new files'):
        final.freeze(arguments,output)
    assert not (output/'release.json').exists()
    if phase == 'audit':assert not output.exists()
