"""원문을 승인으로 재작성하지 않는 새 환경 CPU 조립 계약."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_environment_qualification as qualification


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


@pytest.fixture
def binding(tmp_path, monkeypatch):
    reference = dict(path='unit', sha256='unit-sha')
    meta = dict(split='train', spec=dict(family='obstacle', mission=False, role='front', seed=17),
        runtime_actors=[dict(unique_id=8500, data_id=1, name='Box', source_list='objectList', motion='static')],
        fresh_environment_provenance=dict(environment='sunny15'), **{qualification.unit.KEY:reference})
    old_path = write(tmp_path/'old.json', meta)
    original = dict(id='canonical', metadata=str(old_path), scenario_sha256='old-sha', split='train')
    candidate = dict(id='canonical_visibility01', metadata='new.json', scenario_sha256='new-sha', split='train')
    proposed = copy.deepcopy(meta)
    proposed['fresh_completion_source'] = dict(source_cell_id='canonical', source_scenario_sha256='old-sha', ordinal=1)
    proposed['quality_replacement_contract'] = dict(root_entry=original, ordinal=1,
        root_metadata_sha256=qualification.sha256(old_path), raw_gt_or_parent_modified=False,
        layout_split_finalization_pending=True)
    overlay = dict(overlay_sha256='overlay', recovery_aliases={candidate['id']:'canonical'},
        audit=dict(recovery_mapping=dict(source_id='canonical', candidate_id=candidate['id'],
            old_scenario_sha256='old-sha', new_scenario_sha256='new-sha')))
    monkeypatch.setattr(qualification.layout, 'overlay_for_metadata', lambda _:overlay)
    monkeypatch.setattr(qualification.layout, 'verify_source_pins', lambda _:None)
    monkeypatch.setattr(qualification.layout, 'resolve', lambda *args:dict(split='train', group_id='group'))
    return original, meta, candidate, proposed, dict(config={qualification.unit.KEY:reference}), overlay


def test_exact_recovery_binding_and_canonical_merge(binding):
    original, meta, candidate, proposed, deck, _ = binding
    result = qualification.replacement_shape(original, meta, candidate, proposed, deck)
    assert result['canonical_id'] == 'canonical'
    assert result['runtime_candidate_id'] == candidate['id']
    candidate.update(scenario='/new.json', metadata='/new_meta.json')
    untouched = dict(id='other', scenario='/other.json', metadata='/other_meta.json', scenario_sha256='other')
    merged = qualification.merge_entries([original, untouched], {'canonical':candidate})
    assert merged[0]['id'] == 'canonical'
    assert merged[0]['scenario'] == '/new.json'
    assert merged[1] == untouched
    assert original['metadata'] != merged[0]['metadata']
    assert original['scenario_sha256'] == 'old-sha'


@pytest.mark.parametrize('kind', ['spec','split','actor','unit','environment','canonical','root','oldpayload','alias','mapping','unsafe_id'])
def test_replacement_contract_corruption_is_not_hidden(binding, kind):
    original, meta, candidate, proposed, deck, overlay = binding
    if kind == 'spec':proposed['spec']['role'] = 'rear'
    if kind == 'split':proposed['split'] = 'val'
    if kind == 'actor':proposed['runtime_actors'][0]['data_id'] = 9
    if kind == 'unit':proposed[qualification.unit.KEY] = {}
    if kind == 'environment':proposed['fresh_environment_provenance']['environment'] = 'sunny13'
    if kind == 'canonical':proposed['fresh_completion_source']['source_cell_id'] = 'other'
    if kind == 'root':proposed['quality_replacement_contract']['root_metadata_sha256'] = 'wrong'
    if kind == 'oldpayload':candidate['scenario_sha256'] = 'old-sha'
    if kind == 'alias':overlay['recovery_aliases'][candidate['id']] = 'other'
    if kind == 'mapping':overlay['audit']['recovery_mapping']['new_scenario_sha256'] = 'other'
    if kind == 'unsafe_id':candidate['id'] = '../escape'
    with pytest.raises(ValueError):
        qualification.replacement_shape(original, meta, candidate, proposed, deck)


def test_planned_5k_is_not_captured_5k_and_storage_is_not_waived():
    plan = qualification.evidence_plan('sunny15', '/final', '/future',
        dict(requested=32, observed=32, status_counts=dict(accepted_candidate=28, review=4)),
        [dict(canonical_id='one'), dict(canonical_id='two')], dict(on_demand_scene_ids=['shadow']), 46.)
    assert plan['planned_source_scenes'] == 5000
    assert plan['bulk_training_observations_collected'] == 0
    assert plan['newly_verified_replacement_observations'] == 2
    assert not plan['newly_assembled_source_is_historical_exact_smoke']
    assert not plan['bulk_ready'] and not plan['native_qualified']
    assert not plan['completion_live_qualified']
    assert not plan['storage_gate_passed']
    assert 'insufficient_storage_headroom' in plan['bulk_blockers']
    other = qualification.evidence_plan('sunny15', '/final', '/future', {}, [], {}, 55.)
    assert other['storage_gate_passed'] and not other['bulk_ready']


def test_duplicate_parent_or_foreign_replacement_fails():
    with pytest.raises(ValueError, match='Duplicate'):
        qualification.merge_entries([dict(id='same')]*2, {})
    with pytest.raises(ValueError, match='Foreign'):
        qualification.merge_entries([dict(id='only')], {'unknown':{}})


def test_nonaccepted_native_never_enters_assembly(tmp_path):
    root = tmp_path/'session'/'captures'/'one';root.mkdir(parents=True)
    entry = dict(id='one', scenario_sha256='hash')
    with pytest.raises(ValueError, match='accepted'):
        qualification.verify_accepted_native('unused', entry, {},
            dict(id='one', scenario_sha256='hash', capture_root=str(root), status='review'))


def test_raw_inventory_detects_new_file_and_symlink(tmp_path):
    write(tmp_path/'source.json', {})
    before = qualification.raw_pins(tmp_path)
    write(tmp_path/'added.json', {})
    assert qualification.raw_pins(tmp_path) != before
    (tmp_path/'alias.json').symlink_to(tmp_path/'source.json')
    with pytest.raises(ValueError, match='symlink'):
        qualification.raw_pins(tmp_path)


def test_partial_or_duplicate_history_ledger_rejected(tmp_path):
    ledger = tmp_path/'ledger'
    ledger.write_text(json.dumps(dict(id='one')))
    with pytest.raises(ValueError, match='Complete'):
        qualification.rows(ledger)
    ledger.write_text((json.dumps(dict(id='one'))+'\n')*2)
    with pytest.raises(ValueError, match='Unique'):
        qualification.rows(ledger)


def test_wrong_new_assembly_policy_cannot_impersonate_original_build(tmp_path):
    manifest = write(tmp_path/'manifest.json', dict(config=dict(environment_tag='sunny15')))
    receipt = write(tmp_path/'receipt.json', dict(schema='old-build-schema', policy='old-build'))
    with pytest.raises(ValueError, match='Matching new assembly'):
        qualification.assembled_source_context(manifest, receipt)


def test_native_registration_is_new_schema_not_legacy_policy_adapter(tmp_path):
    manifest = write(tmp_path/'manifest.json', {})
    registration = write(tmp_path/'registration.json', dict(schema='base5_unit_environment_smoke_registration_v1'))
    with pytest.raises(ValueError, match='Current assembled-environment'):
        qualification.registration_context(manifest, registration)


def test_native_inventory_is_rechecked_after_assembly(tmp_path):
    root = tmp_path/'raw'
    write(root/'metadata.json', {})
    receipt = dict(original_native_inventory={str(root):qualification.raw_pins(root)})
    qualification.verify_preserved_native(receipt)
    write(root/'metadata.json', dict(changed=True))
    with pytest.raises(ValueError, match='inventory changed'):
        qualification.verify_preserved_native(receipt)


def test_current_native_opt_in_wrapper_is_explicitly_in_runtime_closure():
    hashes = qualification.runtime_hashes()
    assert hashes[str(Path(qualification.__file__).resolve())] == qualification.sha256(qualification.__file__)
    assert hashes[str(Path(qualification.attitude.__file__).resolve())] == qualification.sha256(qualification.attitude.__file__)
    assert any(Path(path).name == 'base5_static_surface_attitude_candidate.py' for path in hashes)
    assert any(Path(path).name == 'run_base5_capture.py' for path in hashes)


@pytest.fixture
def registered(tmp_path, monkeypatch):
    entries = [dict(id='id'+str(i)) for i in range(32)]
    deck = dict(count=5000, scenes=entries, config={})
    source = write(tmp_path/'source'/'manifest.json', deck)
    receipt = write(tmp_path/'assembly.json', dict(source_manifest=str(tmp_path/'original'/'manifest.json')))
    pin = write(tmp_path/'runtime.json', {})
    hashes = {str(pin):qualification.sha256(pin)}
    context = dict(full=source, deck=deck, entries={e['id']:e for e in entries}, metadata={},
        overlay={}, unit_assets=set(), input_hashes={str(source):qualification.sha256(source)},
        assembly_receipt=receipt, environment='sunny15', raw_scale_counts={})
    selection = [dict(id=e['id']) for e in entries]
    coverage = dict(target_frames=32, maximum_frames=48)
    monkeypatch.setattr(qualification, 'assembled_source_context', lambda *args:copy.deepcopy(context))
    monkeypatch.setattr(qualification, 'runtime_hashes', lambda:dict(hashes))
    monkeypatch.setattr(qualification.environment_smoke, 'select', lambda *args:(selection, coverage))
    monkeypatch.setattr(qualification.scenario_audit, 'audit', lambda *args:
        dict(status='OFFLINE_PASS', errors=[], scenes=5000, unique_payloads=5000))
    monkeypatch.setattr(qualification.attitude, 'configuration', lambda path,*args:
        dict(enabled=True, allow_static_pedestrians=True, setup_profile='static-ready', manifest_sha256=qualification.sha256(path)))
    output = tmp_path/'registered'
    qualification.register_smoke(source, receipt, output)
    return source, output, pin


def test_new_exact_registration_replays_without_rewriting_original_build(registered):
    source, output, _ = registered
    context = qualification.registration_context(source, output/'registration.json')
    assert len(context['selected']) == 32
    assert context['doc']['schema'] == qualification.REGISTRATION_SCHEMA
    assert context['doc']['policy'] == qualification.REGISTRATION_POLICY
    assert not context['doc']['bulk_ready']
    assert context['doc']['native_runtime_profile']['allow_static_pedestrians']
    assert not context['doc']['native_profile_production_bulk_authorized']


@pytest.mark.parametrize('kind', ['runtime','selection','subset','profile','bulk'])
def test_new_registration_cannot_be_retroactively_changed(registered, kind):
    source, output, pin = registered
    path = output/'registration.json';document = qualification.read(path)
    if kind == 'runtime':write(pin, dict(changed=True))
    if kind == 'selection':document['selected'].reverse()
    if kind == 'subset':write(output/'manifest.json', dict(changed=True))
    if kind == 'profile':document['native_runtime_profile']['enabled'] = False
    if kind == 'bulk':document['bulk_ready'] = True
    write(path, document)
    with pytest.raises((ValueError, RuntimeError)):
        qualification.registration_context(source, path)


@pytest.fixture
def accepted_native(tmp_path, monkeypatch):
    session = tmp_path/'session';root = session/'captures'/'one'
    metadata = write(tmp_path/'planned_metadata.json', {})
    scenario = write(tmp_path/'planned_scenario.json', {})
    write(root/'metadata.json', {});write(root/'scenario.json', {})
    sensor = write(root/'sensor_reference.json', {})
    audit = dict(accepted_candidate=True, review_reasons=[],
        components=[dict(component_id='one', lidar_points=1)])
    write(root/'audit.json', audit)
    binary = root/'LIDAR_5'/'test_Intensity.bin';binary.parent.mkdir()
    binary.write_bytes(qualification.np.array([[1., 2., 3., 1.]], dtype='float32').tobytes())
    write(root/'collision_history.json', dict(samples=[], summary=dict(observed=True,
        no_collision_verified=True, non_ego_uids=[], window_start_monotonic=1, window_end_monotonic=2)))
    reference = dict(path='unit', sha256='unit-sha')
    deck = dict(config={qualification.unit.KEY:reference})
    manifest = write(tmp_path/'manifest.json', deck)
    entry = dict(id='one', metadata=str(metadata), scenario=str(scenario),
        scenario_sha256=qualification.sha256(scenario))
    row = dict(id='one', scenario_sha256=entry['scenario_sha256'], capture_root=str(root),
        status='accepted_candidate', audit=audit)
    write(session/'shutdown_safety.json', dict(park_verified=True))
    write(session/'session_contract.json', dict(manifest_sha256=qualification.sha256(manifest),
        sensor_sha256=qualification.sha256(sensor), **{qualification.unit.KEY:reference}))
    write(session/'ledger.jsonl', row)
    monkeypatch.setattr(qualification.provenance, 'verify_capture', lambda *args:None)
    monkeypatch.setattr(qualification, '_validate_native_geometry', lambda *args:None)
    monkeypatch.setattr(qualification, 'point_memberships', lambda *args:[qualification.np.array([True])])
    monkeypatch.setattr(qualification, 'collision_window', lambda *args:dict(observed=True, no_collision_verified=True))
    return manifest, entry, deck, row, root, session


def test_accepted_native_records_beginning_raw_and_session_snapshot(accepted_native):
    manifest, entry, deck, row, root, session = accepted_native
    result = qualification.verify_accepted_native(manifest, entry, deck, row)
    assert result['original_native_evidence'] == qualification.raw_pins(root)
    assert result['session_inventory'] == qualification.native_session_files(session)


@pytest.mark.parametrize('location', ['raw', 'session'])
def test_native_verification_rejects_mutation_during_checks(accepted_native, monkeypatch, location):
    manifest, entry, deck, row, root, session = accepted_native
    def mutate(*args):
        write((root if location == 'raw' else session)/'new_during_verification.json', {})
    monkeypatch.setattr(qualification, '_validate_native_geometry', mutate)
    with pytest.raises(ValueError, match='changed during verification'):
        qualification.verify_accepted_native(manifest, entry, deck, row)


@pytest.mark.parametrize('location', ['original_raw', 'original_session', 'replacement_raw',
    'replacement_session', 'original_source', 'replacement_source', 'symlink_alias'])
def test_registration_cannot_write_into_preserved_evidence(tmp_path, monkeypatch, location):
    source = write(tmp_path/'final'/'manifest.json', {})
    original_source = write(tmp_path/'original'/'manifest.json', {})
    original = write(tmp_path/'old_session'/'captures'/'one'/'audit.json', {}).parent
    replacement = write(tmp_path/'new_session'/'captures'/'one'/'audit.json', {}).parent
    replacement_manifest = write(tmp_path/'replacement_source'/'manifest.json', {})
    receipt = write(tmp_path/'receipt.json', dict(source_manifest=str(original_source),
        original_native_inventory={str(original):{}},
        replacement_native=[dict(capture_root=str(replacement))],
        mapping=[dict(replacement_manifest=str(replacement_manifest))]))
    roots = dict(original_raw=original, original_session=original.parents[1],
        replacement_raw=replacement, replacement_session=replacement.parents[1],
        original_source=original_source.parent, replacement_source=replacement_manifest.parent)
    if location == 'symlink_alias':
        alias = tmp_path/'alias';alias.symlink_to(original.parents[1], target_is_directory=True)
        output = alias/'new_registration'
    else:
        output = roots[location]/'new_registration'
    before = sorted(str(path) for path in tmp_path.rglob('*'))
    monkeypatch.setattr(qualification, 'assembled_source_context',
        lambda *args:pytest.fail('Must reject before verification or creation'))
    with pytest.raises(ValueError, match='preserved source/native/session'):
        qualification.register_smoke(source, receipt, output)
    assert sorted(str(path) for path in tmp_path.rglob('*')) == before
    assert not output.exists()


@pytest.fixture
def historical_contract(tmp_path, monkeypatch):
    source = write(tmp_path/'manifest.json', {})
    helper = tmp_path/'helper.py';helper.write_text('CURRENT\n')
    snapshot = tmp_path/'helper_preserved.py';snapshot.write_text('ORIGINAL\n')
    other = tmp_path/'other.py';other.write_text('OTHER\n')
    monkeypatch.setattr(qualification.environment_smoke, '__file__', str(helper))
    document = dict(schema=qualification.environment_smoke.SCHEMA, policy=qualification.environment_smoke.POLICY,
        source_manifest=str(source), source_manifest_sha256=qualification.sha256(source),
        helper_sha256=qualification.sha256(snapshot), registered_at_unix=qualification.time.time()-10,
        native_observations_must_follow_registration=True, native_qa_unchanged=True,
        runtime_hashes={str(helper):qualification.sha256(snapshot), str(other):qualification.sha256(other)})
    return document, source, snapshot, other


def test_historical_helper_is_evidence_not_current_reapproval(historical_contract):
    document, source, snapshot, _ = historical_contract
    with pytest.raises(ValueError, match='explicit byte-exact'):
        qualification.historical_registration_contract(document, source)
    result = qualification.historical_registration_contract(document, source, snapshot)
    assert result['helper_differs_from_current']
    assert result['helper_evidence_path'] == str(snapshot)
    assert not result['current_helper_replay_performed']
    assert not result['historical_observations_promoted_to_current_approvals']
    assert result['registered_helper_sha256'] != result['current_helper_sha256']


@pytest.mark.parametrize('kind', ['schema', 'policy', 'time', 'gate', 'snapshot', 'runtime'])
def test_historical_registration_guards_are_not_bypassed(historical_contract, kind):
    document, source, snapshot, other = historical_contract
    if kind in ('schema', 'policy'):document[kind] = 'wrong'
    if kind == 'time':document['registered_at_unix'] = float('nan')
    if kind == 'gate':document['native_qa_unchanged'] = False
    if kind == 'snapshot':snapshot.write_text('NOT ORIGINAL\n')
    if kind == 'runtime':other.write_text('CHANGED\n')
    with pytest.raises(ValueError):
        qualification.historical_registration_contract(document, source, snapshot)


def test_original_session_inventory_changes_are_detected(tmp_path):
    session = tmp_path/'session';write(session/'ledger.jsonl', dict(id='one'))
    before = qualification.native_session_files(session)
    receipt = dict(original_session_inventory={str(session):before})
    qualification.verify_preserved_native(receipt)
    write(session/'new.json', {})
    with pytest.raises(ValueError, match='Original native session changed'):
        qualification.verify_preserved_native(receipt)
