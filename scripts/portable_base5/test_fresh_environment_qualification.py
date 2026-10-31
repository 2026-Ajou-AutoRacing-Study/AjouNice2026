"""Offline fresh-source qualification guards; never launches ROS or MORAI."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import fresh_environment_qualification as fresh


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value));return path


@pytest.fixture
def source(tmp_path,monkeypatch):
    manifest = save(tmp_path/'source/manifest.json',dict(count=5000,config=dict(environment_tag='sunny11'),
        scenes=[dict(id='scene%04d'%i) for i in range(5000)]))
    build = save(tmp_path/'source/build_receipt.json',dict(final_manifest=str(manifest),final_manifest_sha256=fresh.sha(manifest)))
    registration = save(tmp_path/'registered/registration.json',dict(schema=fresh.unit_smoke.SCHEMA,
        source_manifest=str(manifest),source_manifest_sha256=fresh.sha(manifest)))
    subset = save(tmp_path/'registered/manifest.json',{'count':32,'scenes':[]})
    runtime = tmp_path/'simulator/Runtime';runtime.mkdir(parents=True)
    monkeypatch.setattr(fresh.original.driver.attitude.candidate.gen.geo,'DEFAULT_RUNTIME_ASSET_DIR',runtime)
    context = dict(deck=fresh.read(manifest),environment='sunny11',build_receipt=build,doc=fresh.read(registration),
        registration=registration,subset_path=subset,input_hashes={str(manifest):fresh.sha(manifest),str(build):fresh.sha(build)})
    verifier = Mock(side_effect=lambda *args:copy.deepcopy(context))
    monkeypatch.setattr(fresh.unit_smoke,'registration_context',verifier)
    return manifest,registration,build,context,verifier


def test_fresh_context_uses_actual_unit_registration_and_has_no_historical_assembly(source):
    manifest,registration,_,_,verifier = source
    context = fresh.registration_context(manifest,registration)
    verifier.assert_called_once_with(manifest,registration)
    assert context['full'] == manifest and 'assembly_receipt' not in context
    assert context['fresh_source_adapter'] == fresh.adapter_pin()


@pytest.mark.parametrize('change',['build_path','build_sha','registration_schema','environment','registration_source',
    'registration_sha','count','assembly_context','assembly_registration'])
def test_fresh_context_rejects_wrong_source_environment_or_legacy_assembly(source,change):
    manifest,registration,build,context,verifier = source
    if change.startswith('build'):
        receipt = fresh.read(build)
        receipt['final_manifest' if change == 'build_path' else 'final_manifest_sha256'] = 'wrong'
        save(build,receipt)
    elif change == 'registration_schema':context['doc']['schema'] = 'old-assembled-smoke'
    elif change == 'environment':context['environment'] = 'sunny15'
    elif change == 'registration_source':context['doc']['source_manifest'] += '.old'
    elif change == 'registration_sha':context['doc']['source_manifest_sha256'] = 'a'*64
    elif change == 'count':context['deck']['count'] = 4999
    elif change == 'assembly_context':context['assembly_receipt'] = 'fake assembly'
    else:context['doc']['assembly_receipt'] = {'path':'fake assembly'}
    with pytest.raises(ValueError,match='fresh source|masquerade'):
        fresh.registration_context(manifest,registration)


def test_fresh_context_propagates_real_unit_validator_failure_without_fallback(source):
    manifest,registration,_,_,verifier = source
    verifier.side_effect = ValueError('Unit source scale/native coverage changed')
    with pytest.raises(ValueError,match='scale/native coverage'):fresh.registration_context(manifest,registration)


def test_scope_selects_only_fresh_source_and_shadow_then_restores_all_originals():
    old_context = fresh.original.qualification.registration_context;old_shadow = fresh.original.verify_shadow
    with fresh.fresh_source_scope():
        assert fresh.original.qualification.registration_context is fresh.registration_context
        assert fresh.original.verify_shadow is fresh.verify_shadow
        with pytest.raises(ValueError,match='nested'):
            with fresh.fresh_source_scope():pass
    assert fresh.original.qualification.registration_context is old_context
    assert fresh.original.verify_shadow is old_shadow


def test_scope_restores_on_failure_and_refuses_ownership_replacement():
    old_context = fresh.original.qualification.registration_context;old_shadow = fresh.original.verify_shadow
    with pytest.raises(RuntimeError,match='native failure'):
        with fresh.fresh_source_scope():raise RuntimeError('native failure')
    assert fresh.original.qualification.registration_context is old_context
    with pytest.raises(ValueError,match='ownership'):
        with fresh.fresh_source_scope():fresh.original.verify_shadow = Mock()
    assert fresh.original.qualification.registration_context is old_context
    assert fresh.original.verify_shadow is old_shadow
    with fresh.fresh_source_scope():pass


def test_qualify_reuses_original_six_gate_body_and_adds_actual_adapter_pin(source,tmp_path,monkeypatch):
    native = save(tmp_path/'actual_native.json',{'actual':'native'})
    manifest,registration,build,_,_ = source
    context = fresh.registration_context(manifest,registration)
    source_audit = save(registration.parent/'source_audit.json',{'status':'OFFLINE_PASS'})
    context['doc'].update(input_hashes=dict(context['input_hashes']),runtime_hashes={str(native):fresh.sha(native)},
        subset_manifest_sha256=fresh.sha(context['subset_path']),source_audit_sha256=fresh.sha(source_audit),
        build_receipt=dict(path=str(build),sha256=fresh.sha(build)))
    build_value = fresh.read(build);build_value.update(original_input_hashes={str(native):fresh.sha(native)},
        runtime_hashes={str(native):fresh.sha(native)})
    save(build,build_value)
    context['input_hashes'][str(build)] = fresh.sha(build)
    context['doc']['input_hashes'][str(build)] = fresh.sha(build)
    context['doc']['build_receipt']['sha256'] = fresh.sha(build)
    before = fresh.original.qualification.registration_context
    def original_qualify(arguments):
        assert fresh.original.qualification.registration_context is fresh.registration_context
        assert fresh.original.verify_shadow is fresh.verify_shadow
        assert arguments == {'actual':'inputs'}
        return dict(results={name:dict(passed=True) for name in fresh.GATES},
            input_hashes={str(native):fresh.sha(native)},context=context)
    verifier = Mock(side_effect=original_qualify);monkeypatch.setattr(fresh.original,'qualify',verifier)
    value = fresh.qualify({'actual':'inputs'})
    assert value['input_hashes'][str(Path(fresh.__file__).absolute())] == fresh.sha(fresh.__file__)
    assert value['input_hashes'][str(context['subset_path'])] == fresh.sha(context['subset_path'])
    assert value['input_hashes'][str(source_audit)] == fresh.sha(source_audit)
    assert fresh.original.qualification.registration_context is before
    verifier.assert_called_once_with({'actual':'inputs'})


@pytest.mark.parametrize('change',['missing','failed'])
def test_qualify_cannot_reduce_or_ignore_any_of_six_original_gates(monkeypatch,change):
    gates = {name:dict(passed=True) for name in fresh.GATES}
    if change == 'missing':gates.pop('partial_resume')
    else:gates['independent_shadow']['passed'] = False
    monkeypatch.setattr(fresh.original,'qualify',Mock(return_value={'results':gates}))
    with pytest.raises(ValueError,match='All six original'):fresh.qualify({})


@pytest.mark.parametrize('where',['source','inside_source','source_ancestor','registration','runtime','inside_runtime',
    'native','inside_native','native_ancestor','pinned_file','pinned_ancestor'])
def test_output_guards_protect_source_registration_runtime_native_and_all_pin_files(source,tmp_path,where):
    manifest,registration,_,_,_ = source
    context = fresh.registration_context(manifest,registration)
    native = tmp_path/'native';native.mkdir()
    pin = save(tmp_path/'external/pinned/report.json',{})
    context['input_hashes'][str(pin)] = fresh.sha(pin)
    paths = dict(source=manifest.parent,inside_source=manifest.parent/'release',source_ancestor=tmp_path,
        registration=registration.parent,runtime=tmp_path/'simulator/Runtime',
        inside_runtime=tmp_path/'simulator/Runtime/release',native=native,inside_native=native/'release',
        native_ancestor=tmp_path,pinned_file=pin,pinned_ancestor=pin.parent)
    with pytest.raises(ValueError,match='cannot overlap|cannot contain'):
        fresh.require_safe_output(paths[where],context,protected=(native,))
    assert fresh.require_safe_output(tmp_path/'safe_release',context,protected=(native,)) == tmp_path/'safe_release'


def test_output_guard_never_follows_symlink(source,tmp_path):
    manifest,registration,_,_,_ = source;context = fresh.registration_context(manifest,registration)
    link = tmp_path/'alias';link.symlink_to(manifest.parent,target_is_directory=True)
    with pytest.raises(ValueError,match='nonsymlink'):fresh.require_safe_output(link/'release',context)


@pytest.fixture
def release_fixture(source,tmp_path,monkeypatch):
    manifest,registration,_,_,_ = source;context = fresh.registration_context(manifest,registration)
    capture = tmp_path/'native';capture.mkdir()
    planned = tmp_path/'bulk';output = tmp_path/'qualified'
    protected = tmp_path/'shadow_native';protected.mkdir()
    cases = save(tmp_path/'shadow_cases.json',{})
    evidence = save(tmp_path/'actual_evidence.json',{'native':'original byte proof'})
    inputs = dict(context['input_hashes']);inputs[str(Path(fresh.__file__).absolute())] = fresh.sha(fresh.__file__)
    inputs[str(evidence)] = fresh.sha(evidence)
    runtime_file = tmp_path/'runtime.py';runtime_file.write_text('# original runtime\n')
    runtime = {str(runtime_file):fresh.sha(runtime_file)}
    results = {name:dict(passed=True,actual=name) for name in fresh.GATES}
    results['native_exact']['accepted'] = 32
    results['independent_shadow']['protected_native_roots'] = [str(protected)]
    verified = dict(context=context,results=results,input_hashes=inputs,runtime_hashes=runtime)
    arguments = dict(manifest=str(manifest),smoke_registration=str(registration),capture_root=str(capture),
        capture_output=str(planned),shadow_cases=str(cases))
    verifier = Mock(side_effect=lambda *args:copy.deepcopy(verified));monkeypatch.setattr(fresh,'qualify',verifier)
    monkeypatch.setattr(fresh.original,'shadow_declared_protected_roots',lambda doc:{protected})
    preserved = Mock();monkeypatch.setattr(fresh.original,'verify_shadow_preserved',preserved)
    monkeypatch.setattr(fresh.original.scenario_audit,'audit',lambda path:
        dict(status='OFFLINE_PASS',errors=[],scenes=5000,unique_payloads=5000))
    monkeypatch.setattr(fresh.original.cpu,'runtime_hashes',lambda:dict(runtime))
    return arguments,output,verified,verifier,preserved,evidence


def test_freeze_and_context_roundtrip_recompute_all_actual_six_gate_results(release_fixture):
    arguments,output,verified,verifier,preserved,_ = release_fixture
    path = fresh.freeze(arguments,output);value = fresh.read(path)
    assert value['qualification_schema'] == fresh.QUALIFICATION_SCHEMA
    assert value['fresh_source_adapter'] == fresh.adapter_pin()
    assert value['bulk_training_observations'] == 0 and value['native_exact_observations'] == 32
    assert set(value['gate_evidence']) == set(fresh.GATES)
    assert value['qualified_runtime']['runtime_profile'] == fresh.original.driver.profiles()
    context = fresh.qualified_release_context(path,'sunny11',Path(arguments['manifest']))
    assert context['passed'] is True and context['native_exact_observations'] == 32
    assert context['input_hashes'] == verified['input_hashes']
    assert verifier.call_count == 2 and preserved.call_count >= 3


@pytest.mark.parametrize('change',['adapter_sha','adapter_schema','schema','old_policy','environment','source','output',
    'runtime_profile','native31','native49','bulk_observations','gate_missing','gate_changed','audit_changed','native_bytes'])
def test_fresh_release_context_rejects_any_changed_authority_or_actual_evidence(release_fixture,change):
    arguments,output,_,_,_,evidence = release_fixture
    path = fresh.freeze(arguments,output);document = fresh.read(path)
    if change == 'adapter_sha':document['fresh_source_adapter']['sha256'] = 'a'*64
    elif change == 'adapter_schema':document['fresh_source_adapter']['registration_schema'] = 'historical-assembled'
    elif change == 'schema':document['qualification_schema'] = fresh.original.QUALIFICATION_SCHEMA
    elif change == 'old_policy':document['release_policy'] = fresh.original.POLICY
    elif change == 'environment':document['environment'] = 'sunny15'
    elif change == 'source':document['full_manifest'] += '.old'
    elif change == 'output':document['capture_output'] += '.old'
    elif change == 'runtime_profile':document['qualified_runtime']['runtime_profile'] = {}
    elif change == 'native31':document['native_exact_observations'] = 31
    elif change == 'native49':document['native_exact_observations'] = 49
    elif change == 'bulk_observations':document['bulk_training_observations'] = 1
    elif change == 'gate_missing':document['gate_evidence'].pop('completed_resume')
    elif change == 'gate_changed':save(Path(document['gate_evidence']['native_exact']['path']),{'passed':True,'fake':True})
    elif change == 'audit_changed':save(Path(document['full_cpu_audit']['path']),{'status':'FAILED'})
    else:evidence.write_text('changed native bytes')
    save(path,document)
    with pytest.raises(ValueError):fresh.qualified_release_context(path,'sunny11',Path(arguments['manifest']))


def test_freeze_failure_never_publishes_release_and_never_launches_a_subprocess(release_fixture,monkeypatch):
    arguments,output,_,verifier,_,_ = release_fixture
    verifier.side_effect = ValueError('Native corner verification failed')
    popen = Mock(side_effect=AssertionError('No process may launch during qualification'))
    monkeypatch.setattr(fresh.original.driver.subprocess,'Popen',popen)
    with pytest.raises(ValueError,match='Native corner'):fresh.freeze(arguments,output)
    assert not output.exists();popen.assert_not_called()


def test_original_assembled_source_context_is_never_used_by_fresh_release(source,monkeypatch):
    manifest,registration,_,_,_ = source
    original = Mock(side_effect=AssertionError('No historical assembly expected'))
    monkeypatch.setattr(fresh.original.qualification,'assembled_source_context',original)
    fresh.registration_context(manifest,registration)
    original.assert_not_called()


def test_partial_resume_cli_delegates_unchanged_original_verifier(tmp_path,monkeypatch):
    snapshot,manifest,capture,output = (tmp_path/name for name in ('snapshot','manifest','capture','receipt'))
    verifier = Mock(return_value=output);monkeypatch.setattr(fresh.original,'verify_partial',verifier)
    assert fresh.main(['verify-partial','--snapshot',str(snapshot),'--manifest',str(manifest),
        '--capture-root',str(capture),'--output',str(output)]) == 0
    verifier.assert_called_once_with(snapshot,manifest,capture,output)


@pytest.fixture
def shadow_fixture(tmp_path,monkeypatch):
    sensor = save(tmp_path/'configs/sensors/Sensor_competition_local.json',{'sensor':'actual fixture'})
    monkeypatch.setattr(fresh.original.pins,'ROOT',tmp_path)
    entries,metadata,cases,proofs = [],{},[],{}
    for index,kind in enumerate(('animal','person')):
        ident = 'shadow_'+kind
        actor = dict(name='dog' if kind == 'animal' else 'pedestrian',unique_id=index+1,
            motion='walking',population_pool=kind,training_class='animal' if kind == 'animal' else 'pedestrian')
        meta = dict(spec=dict(family='shadow',shadow_moving=True),runtime_actors=[actor])
        meta_path = save(tmp_path/'source'/ident/'metadata.json',meta)
        scenario = save(meta_path.with_name('scenario.json'),{'uid':index})
        entry = dict(id=ident,metadata=str(meta_path),scenario=str(scenario),scenario_sha256=fresh.sha(scenario))
        entries.append(entry);metadata[ident] = meta
        prepared = save(tmp_path/'prepared'/ident/'manifest.json',dict(count=1,scenes=[entry]))
        session = tmp_path/'proof'/ident/'session';raw = session/'captures'/ident
        audit = dict(motion_capture=dict(required=True,review_reasons=[]),motion_capture_actors=[dict(review_reasons=[])],
            components=[dict(parent_uid=index+1,training_class=actor['training_class'])])
        save(raw/'audit.json',audit)
        save(raw/'scenario.json',{'uid':index})
        row = dict(id=ident,scenario_sha256=entry['scenario_sha256'],capture_root=str(raw),status='accepted_candidate')
        ledger = session/'ledger.jsonl';ledger.write_text(json.dumps(row)+'\n')
        campaign = tmp_path/'campaign'/ident;campaign.mkdir(parents=True)
        save(campaign/'status.json',{'stage':'complete'})
        cases.append(dict(source_id=ident,adapter='final_direction_reference',campaign_manifest=str(prepared),
            campaign_root=str(campaign),campaign_index=0))
        proofs[ident] = dict(prepared_manifest=str(prepared),selected_ledger=str(ledger),passed=True)
    manifest = save(tmp_path/'source/manifest.json',dict(count=2,config=dict(environment_tag='sunny11'),scenes=entries))
    document = dict(schema=fresh.original.SHADOW_SCHEMA,environment='sunny11',
        source_manifest_sha256=fresh.sha(manifest),cases=cases,preserved_review_captures=[])
    path = save(tmp_path/'shadow_cases.json',document)
    context = dict(full=manifest,deck=fresh.read(manifest),entries={e['id']:e for e in entries},metadata=metadata,environment='sunny11')
    calls = []
    def direction(case,canonical,deck,old,hashes):
        assert deck == context['deck'] and old == context['entries'][canonical]
        calls.append(canonical)
        fresh.original.qualification.add_pin(hashes,case['campaign_manifest'])
        return proofs[canonical]
    monkeypatch.setattr(fresh.original,'direction_case',direction)
    native = Mock(return_value=dict(original_native_evidence={},session_inventory={}))
    monkeypatch.setattr(fresh.original.qualification,'verify_accepted_native',native)
    return context,path,document,proofs,calls,native


def test_shadow_binds_actual_fresh_full_source_and_reuses_direction_native_motion_and_inventory_guards(shadow_fixture):
    context,path,_,_,calls,native = shadow_fixture
    result = fresh.verify_shadow(context,path)
    assert result['passed'] is True and set(calls) == {'shadow_animal','shadow_person'}
    assert {row['category'] for row in result['observations']} == {'animal','pedestrian_or_compound'}
    assert native.call_count == 2 and result['native_inventories']
    assert result['exact_smoke_approvals_added'] == result['materialized_bulk_source_scenes'] == 0
    assert result['evidence_hashes'][str(context['full'])] == fresh.sha(context['full'])


@pytest.mark.parametrize('change',['wrong_source_sha','wrong_environment','duplicate','unknown_source','one_category',
    'source_changed','not_moving','motion_review','actor_review','not_accepted','unknown_adapter','geometry_changed','native_failure'])
def test_shadow_cannot_weaken_original_source_semantic_motion_acceptance_or_native_guards(shadow_fixture,change):
    context,path,document,proofs,_,native = shadow_fixture
    case = document['cases'][0];proof = proofs[case['source_id']]
    prepared = fresh.read(proof['prepared_manifest']);meta_path = Path(prepared['scenes'][0]['metadata'])
    rows = fresh.original.qualification.rows(proof['selected_ledger']);raw = Path(rows[0]['capture_root'])
    if change == 'wrong_source_sha':document['source_manifest_sha256'] = 'a'*64
    elif change == 'wrong_environment':document['environment'] = 'sunny15'
    elif change == 'duplicate':document['cases'][1] = copy.deepcopy(case)
    elif change == 'unknown_source':case['source_id'] = 'old_scene'
    elif change == 'one_category':
        second = proofs['shadow_person'];audit_path = Path(fresh.original.qualification.rows(second['selected_ledger'])[0]['capture_root'])/'audit.json'
        metadata_path = Path(fresh.read(second['prepared_manifest'])['scenes'][0]['metadata'])
        meta = fresh.read(metadata_path);meta['runtime_actors'][0].update(population_pool='animal',training_class='animal')
        save(metadata_path,meta);context['metadata']['shadow_person'] = meta
        audit = fresh.read(audit_path);audit['components'][0]['training_class'] = 'animal';save(audit_path,audit)
    elif change == 'source_changed':save(context['full'],{})
    elif change == 'not_moving':
        meta = fresh.read(meta_path);meta['spec']['shadow_moving'] = False;save(meta_path,meta)
        context['metadata'][case['source_id']] = meta
    elif change in ('motion_review','actor_review'):
        audit = fresh.read(raw/'audit.json')
        (audit['motion_capture'] if change == 'motion_review' else audit['motion_capture_actors'][0])['review_reasons'] = ['bad ground']
        save(raw/'audit.json',audit)
    elif change == 'not_accepted':
        rows[0]['status'] = 'review';Path(proof['selected_ledger']).write_text(json.dumps(rows[0])+'\n')
    elif change == 'unknown_adapter':case['adapter'] = 'skip-native'
    elif change == 'geometry_changed':context['entries'][case['source_id']]['scenario_sha256'] = 'a'*64
    else:native.side_effect = ValueError('Native collision/ground proof invalid')
    save(path,document)
    with pytest.raises((ValueError,KeyError)):fresh.verify_shadow(context,path)
