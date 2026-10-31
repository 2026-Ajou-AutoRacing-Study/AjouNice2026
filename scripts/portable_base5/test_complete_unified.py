"""CPU-only regression: never launch ROS/MORAI/native capture."""
import copy
from contextlib import contextmanager
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import complete_unified as completion


def write(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def metadata():
    return dict(spec=dict(actors=1,family='highway',role='rear',yaw_limit=3),split='train',
        scene_group_id='original-group',runtime_actors=[dict(unique_id=1,data_id=101,name='car',
            source_list='vehicleList',motion='static')],native_unit_geometry_reference={'sha256':'abc'},
        fresh_environment_provenance={'environment':'sunny15'})


def selection(ident):
    return dict(id=ident,source_cell_id=ident,status='accepted_candidate',audit={'review_reasons':[]})


def state():
    return dict(source_order=['a','b','c'],baseline_selected=[selection('a')],deficits=['b','c'],attempts=[])


def test_atomic_rows_preserves_unchanged_inode_and_newline(tmp_path):
    path=tmp_path/'selected.jsonl'
    completion.atomic_rows(path,[selection('a')]);before=path.stat()
    completion.atomic_rows(path,[selection('a')])
    assert path.stat().st_ino == before.st_ino
    assert completion.rows(path) == [selection('a')]
    path.write_bytes(b'{}')
    with pytest.raises(RuntimeError,match='newline'):
        completion.rows(path)


def test_merge_preserves_baseline_identity_and_orders_source():
    s=state();original=copy.deepcopy(s['baseline_selected'])
    s['attempts']=[dict(id='c',ordinal=1,manifest='/source',manifest_sha256='abc',
        child='/child',result={'selected':selection('generated')})]
    selected,missing=completion.merged(s)
    assert selected[0] == original[0] and s['baseline_selected'] == original
    assert [row['source_cell_id'] for row in selected] == ['a','c']
    assert missing == ['b']


def test_merge_refuses_duplicate_and_foreign_success():
    s=state();a=dict(id='a',ordinal=1,manifest='m',manifest_sha256='s',child='c',result={'selected':selection('x')})
    s['attempts']=[a]
    with pytest.raises(RuntimeError):
        completion.merged(s)
    s['attempts']=[dict(a,id='foreign')]
    # A foreign quota must not be silently dropped while an ordinary deficit remains.
    with pytest.raises(RuntimeError):
        completion.merged(s)


@pytest.mark.parametrize('field', ['spec','split','scene_group_id','runtime_actors',
    'native_unit_geometry_reference','fresh_environment_provenance'])
def test_proposal_must_preserve_full_quota_identity(tmp_path,field):
    old=metadata();new=copy.deepcopy(old)
    if field == 'spec':new[field]['yaw_limit']=15
    elif field == 'runtime_actors':new[field][0]['data_id']=202
    else:new[field]='changed'
    source=write(tmp_path/'scenario.json',{'pose':'new'})
    mp=write(tmp_path/'metadata.json',new)
    manifest=write(tmp_path/'manifest.json',dict(count=1,scenes=[dict(scenario=str(source),metadata=str(mp),
        scenario_sha256=completion.common.sha(source))]))
    with pytest.raises(RuntimeError,match='frozen'):
        completion.validate_proposal(old,manifest,set())


def test_proposal_cannot_replay_a_prior_failed_native(tmp_path):
    source=write(tmp_path/'scenario.json',{'pose':'old-failed'})
    mp=write(tmp_path/'metadata.json',metadata());sha=completion.common.sha(source)
    manifest=write(tmp_path/'manifest.json',dict(count=1,scenes=[dict(scenario=str(source),metadata=str(mp),scenario_sha256=sha)]))
    with pytest.raises(RuntimeError,match='Never repeat'):
        completion.validate_proposal(metadata(),manifest,{sha})
    assert completion.validate_proposal(metadata(),manifest,set())['scenario_sha256'] == sha


def test_cumulative_prior_payloads_include_imported_failed_ledgers(tmp_path):
    old=tmp_path/'old';current=tmp_path/'current'
    write(old/'native'/'ledger.jsonl',{'scenario_sha256':'a'*64})
    write(current/'native'/'ledger.jsonl',{'scenario_sha256':'b'*64})
    write(current/'imported_prefix.json',{'original_output':str(old)})
    assert completion.prior_payloads(current,dict(scenes=[{'scenario_sha256':'c'*64}])) == {'a'*64,'b'*64,'c'*64}
    write(old/'imported_prefix.json',{'original_output':str(current)})
    with pytest.raises(RuntimeError,match='Cyclic'):
        completion.prior_payloads(current,dict(scenes=[]))


def test_interrupted_child_does_not_start_direction_or_recapture(tmp_path,monkeypatch):
    child=tmp_path/'campaign';child.mkdir()
    write(child/'status.json',dict(stage='halted'))
    calls=[]
    monkeypatch.setattr(completion.common,'modules',lambda:(None,SimpleNamespace(run=lambda args:calls.append(args))))
    with pytest.raises(RuntimeError,match='Interrupted/unsafe'):
        completion.capture({'manifest':'/unused','child':str(child)},tmp_path,metadata())
    assert not calls


def test_completed_child_only_revalidates_never_recaptures(tmp_path,monkeypatch):
    child=tmp_path/'campaign';child.mkdir();write(child/'status.json',dict(stage='complete'))
    calls=[];expected={'selected':selection('new')}
    monkeypatch.setattr(completion.common,'modules',lambda:(None,SimpleNamespace(run=lambda args:calls.append(args))))
    monkeypatch.setattr(completion,'verify_attempt',lambda attempt,original:expected)
    assert completion.capture({'manifest':'/unused','child':str(child)},tmp_path,metadata()) == expected
    assert calls == []


def test_nested_nonrepeat_unions_history_and_restores(monkeypatch):
    calls=[]
    def original(*args,**kwargs):calls.append(kwargs);return 'ok'
    quality=SimpleNamespace(replacement=original)
    parent=SimpleNamespace(batch=SimpleNamespace(campaign=SimpleNamespace(loop=quality)))
    monkeypatch.setattr(completion.common,'modules',lambda:(parent,None))
    with completion.nonrepeat({'old'}):
        assert quality.replacement('manifest','output',known_payloads={'new'}) == 'ok'
    assert calls == [{'known_payloads':{'old','new'}}] and quality.replacement is original


def test_history_reaches_original_quality_precapture_guard_and_restores():
    calls=[]
    def compiler(contract):
        def execute(*args,**kwargs):calls.append(kwargs);return 'native-guarded'
        return execute
    direction=SimpleNamespace(compiled_execute=compiler)
    with completion.inherited_native(direction,{'historical-failed'}):
        assert direction.compiled_execute({})('source',known_payloads={'inner-observed'}) == 'native-guarded'
    assert calls == [{'known_payloads':{'historical-failed','inner-observed'}}]
    assert direction.compiled_execute is compiler


@pytest.mark.parametrize('change',['id','ordinal','duplicate','child','manifest','selected'])
def test_resume_rejects_foreign_paths_and_quota_mutations(tmp_path,change):
    old=metadata();mp=write(tmp_path/'old_meta.json',old)
    deck=dict(scenes=[dict(id='a',metadata=str(mp)),dict(id='b',metadata=str(mp)),dict(id='c',metadata=str(mp))])
    output=tmp_path/'completion';folder=output/'cells'/'b'/'attempt_1'
    sp=write(folder/'placement'/'scenario.json',{'pose':'fresh'});fresh=write(folder/'placement'/'metadata.json',old)
    manifest=write(folder/'placement'/'manifest.json',dict(count=1,scenes=[dict(scenario=str(sp),
        metadata=str(fresh),scenario_sha256=completion.common.sha(sp))]))
    s=state();s.update(stage='prepared',initial_known_payloads=[])
    attempt=dict(id='b',ordinal=1,manifest=str(manifest),manifest_sha256=completion.common.sha(manifest),
        scenario_sha256=completion.common.sha(sp),child=str(folder/'campaign'),result=None)
    s['attempts']=[attempt]
    completion.validate_state(s,output,deck,[selection('a')],[{'id':'b'},{'id':'c'}],2)
    if change == 'id':attempt['id']='foreign'
    elif change == 'ordinal':attempt['ordinal']=3
    elif change == 'duplicate':s['attempts'].append(copy.deepcopy(attempt))
    elif change == 'child':attempt['child']=str(tmp_path/'outside')
    elif change == 'manifest':attempt['manifest']=str(mp)
    elif change == 'selected':s['baseline_selected']=[]
    with pytest.raises(RuntimeError):
        completion.validate_state(s,output,deck,[selection('a')],[{'id':'b'},{'id':'c'}],2)


def test_default_plan_cannot_verify_write_or_capture(tmp_path,monkeypatch):
    profile=tmp_path/'profile';output=tmp_path/'new'
    doc={'environment':'sunny15'}
    monkeypatch.setattr(completion,'summary',lambda *args:(doc,tmp_path/'old',output,{},[selection('a')],[{'id':'b'}],{}))
    monkeypatch.setattr(completion,'verify_baseline',lambda *args:pytest.fail('Plan must not run native verifier'))
    args=completion.parser().parse_args(['--profile',str(profile),'--output',str(output)])
    result=completion.execute(args)
    assert result['approved'] == 1 and result['deficit'] == 1
    assert result['capture_started'] is False and result['files_written'] is False and not output.exists()


def test_run_requires_matching_ui_confirmation_before_any_verification(tmp_path,monkeypatch):
    monkeypatch.setattr(completion,'summary',lambda *args:({'environment':'sunny15'},tmp_path,tmp_path/'new',{},[],[],{}))
    monkeypatch.setattr(completion,'verify_baseline',lambda *args:pytest.fail('No verifier before confirmation'))
    args=completion.parser().parse_args(['--profile','unused','--output','unused','--run'])
    with pytest.raises(RuntimeError,match='Explicit matching'):
        completion.execute(args)


@pytest.mark.parametrize('attempts,budget',[(0,60),(4,60),(2,29),(2,3601)])
def test_finite_bounds_precede_any_source_action(attempts,budget,monkeypatch):
    monkeypatch.setattr(completion,'summary',lambda *args:pytest.fail('Bounds first'))
    args=completion.parser().parse_args(['--profile','unused','--output','unused',
        '--attempts',str(attempts),'--time-budget-sec',str(budget)])
    with pytest.raises(RuntimeError,match='Finite'):
        completion.execute(args)


def terminal_contract_fixture(tmp_path,monkeypatch):
    """Trusted 4,941 selections without any native image/point files."""
    baseline=tmp_path/'trusted_parent';baseline.mkdir()
    scripts=tmp_path/'scripts';scripts.mkdir()
    importer=scripts/'base5_campaign_import.py';importer.write_text('# frozen importer\n')
    source=write(tmp_path/'source.json',{'frozen':'source'})
    pilot=write(tmp_path/'five_cell_pilot.json',{'qualified':True})
    proof=write(tmp_path/'five_cell_proof.json',{'passed':True})
    fixed=write(tmp_path/'input.json',{'fixed':True})
    selected=[selection(f'cell_{index:05d}') for index in range(4941)]
    deficits=[{'id':f'cell_{index:05d}','index':index,'reasons':['sensor_evidence']} for index in range(4941,5000)]
    completion.atomic_rows(baseline/'selected_ledger.jsonl',selected)
    write(baseline/'status.json',dict(stage='complete_with_deficit',finished=5000,
        accepted_candidate=4941,review=59,error=None,failed=0,current=None,checkpoint_finished=5000))
    write(baseline/'quota.json',dict(target=5000,accepted=4941,deficit=59,pending=0,cells=deficits))
    write(baseline/'deficit_manifest.json',{'count':59})
    runtime={'/frozen/runtime.py':'a'*64};profiles={'strict':'original-direction-native-qa'}
    authority={'original':'qualified-source'}
    doc=dict(manifest=str(source),manifest_sha256=completion.common.sha(source),environment='sunny15',
        old_output=str(tmp_path/'historical_original'),runtime_hashes=runtime,
        source_authority=authority,fixed_pins={str(fixed):completion.common.sha(fixed)},helper_pins={'helper':'b'*64})
    imported=write(baseline/'imported_prefix.json',dict(schema=1,manifest_sha256=doc['manifest_sha256'],
        original_output=doc['old_output'],selected=selected[:2],deficits=deficits[:2],
        hashes={'/never-open/native-frame.json':'c'*64},sensor_stats={'/never-open/raw-image.png':[123,456]}))
    doc['import_sha256']=completion.common.sha(imported)
    expected_authority=dict(authority,unified_runtime_policy=completion.common.POLICY,
        runtime_profile=profiles,original_source_qualification_only=True,new_output_handoff=True,
        live_profile_qualification_separate=True)
    legacy={'frozen':'legacy-batch-contract'}
    guard=dict(schema='fixture-static-schema',policy='fixture-static-policy',manifest=str(source),
        manifest_sha256=doc['manifest_sha256'],environment='sunny15',indices=list(range(5000)),
        runtime_profile=profiles,runtime_hashes=runtime,input_hashes=doc['fixed_pins'],
        qualified_release=expected_authority,legacy_batch_contract=legacy)
    write(baseline/'static_attitude_batch_contract.json',guard)
    write(baseline/'batch_contract.json',dict(legacy,imported_prefix_sha256=doc['import_sha256'],
        import_driver_sha256=completion.common.sha(importer)))
    qualification=dict(passed=True,policy=completion.common.POLICY,environment='sunny15',
        manifest_sha256=doc['manifest_sha256'],helper_pins=doc['helper_pins'],actual_pause_resume=True,
        completed_replay_no_new_capture=True,pilot_manifest=str(pilot),pilot_manifest_sha256=completion.common.sha(pilot),
        proof_hashes={str(proof):completion.common.sha(proof)})
    write(baseline/'unified_live_qualification.json',qualification)
    write(baseline/'integrity_boundary_example.json',dict(policy=completion.common.POLICY,passed=True,result=0))
    calls={'native':0,'boundary_exit':0}
    def native_forbidden(*args,**kwargs):
        calls['native']+=1
        pytest.fail('Trusted historical selections must not invoke native QA or collector')
    parent=SimpleNamespace(SCHEMA=guard['schema'],POLICY=guard['policy'],profiles=lambda:profiles,
        runtime_hashes=lambda:runtime,run=native_forbidden,
        batch=SimpleNamespace(execute=native_forbidden,verify_recovery=native_forbidden),
        verify_regular=native_forbidden,verify_strict_recovery=native_forbidden)
    class Boundary:
        @contextmanager
        def installed(self,**kwargs):yield self
        def verify(self):calls['boundary_exit']+=1
    boundary=Boundary()
    monkeypatch.setattr(completion.common,'modules',lambda:(parent,SimpleNamespace()))
    monkeypatch.setattr(completion.common,'boundary_for',lambda value:boundary)
    monkeypatch.setattr(completion.common,'warm_boundary',lambda *args:None)
    monkeypatch.setattr(completion.common,'source_authority',lambda value,module:copy.deepcopy(authority))
    monkeypatch.setattr(completion.common,'SCRIPTS',scripts)
    return SimpleNamespace(baseline=baseline,doc=doc,selected=selected,deficits=deficits,
        parent=parent,boundary=boundary,calls=calls,guard=guard,legacy=legacy,
        qualification=qualification,pilot=pilot,proof=proof,imported=imported,importer=importer)


def test_trusted_4941_terminal_contract_never_recomputes_native_qa(tmp_path,monkeypatch,capsys):
    f=terminal_contract_fixture(tmp_path,monkeypatch)
    # No capture_root, pixels or points even exist in this fixture. It also
    # intentionally names nonexistent historical native hashes/sensor_stats.
    files=list(f.baseline.iterdir())
    before={str(path):(path.read_bytes(),path.stat().st_ino,path.stat().st_mtime_ns) for path in files}
    result=completion.verify_baseline(f.doc,f.baseline,f.selected,f.deficits)
    assert result is f.boundary and f.calls == {'native':0,'boundary_exit':1}
    after={str(path):(path.read_bytes(),path.stat().st_ino,path.stat().st_mtime_ns) for path in files}
    assert before == after
    report=json.loads(capsys.readouterr().out)
    assert report['approved'] == 4941 and report['deficit'] == 59
    assert report['existing_native_qa_recomputed'] is False


@pytest.mark.parametrize('field',['schema','policy','manifest','manifest_sha256','environment','indices',
    'runtime_profile','runtime_hashes','input_hashes','qualified_release'])
def test_terminal_contract_rejects_static_guard_mutations_without_native_qa(tmp_path,monkeypatch,field):
    f=terminal_contract_fixture(tmp_path,monkeypatch)
    guard=copy.deepcopy(f.guard)
    if field == 'indices':guard[field]=list(range(4999))
    elif field == 'input_hashes':guard[field]={'/foreign/input.json':'d'*64}
    else:guard[field]='changed'
    write(f.baseline/'static_attitude_batch_contract.json',guard)
    with pytest.raises(RuntimeError):
        completion.verify_baseline(f.doc,f.baseline,f.selected,f.deficits)
    assert f.calls == {'native':0,'boundary_exit':1}


@pytest.mark.parametrize('change',['sha','schema','manifest','original_output','selected','deficits'])
def test_terminal_contract_rejects_import_link_or_row_changes(tmp_path,monkeypatch,change):
    f=terminal_contract_fixture(tmp_path,monkeypatch)
    imported=json.loads(f.imported.read_text())
    if change == 'sha':f.doc['import_sha256']='f'*64
    else:
        if change == 'schema':imported['schema']=99
        elif change == 'manifest':imported['manifest_sha256']='f'*64
        elif change == 'original_output':imported['original_output']='/foreign/parent'
        elif change == 'selected':imported['selected'][0]['id']='tampered'
        elif change == 'deficits':imported['deficits'][0]['reasons']=['changed']
        write(f.imported,imported)
        f.doc['import_sha256']=completion.common.sha(f.imported)
    with pytest.raises(RuntimeError):
        completion.verify_baseline(f.doc,f.baseline,f.selected,f.deficits)
    assert f.calls == {'native':0,'boundary_exit':1}


@pytest.mark.parametrize('field',['passed','policy','environment','manifest_sha256','helper_pins',
    'actual_pause_resume','completed_replay_no_new_capture','pilot_manifest_sha256','proof_hashes'])
def test_terminal_contract_rejects_qualification_changes(tmp_path,monkeypatch,field):
    f=terminal_contract_fixture(tmp_path,monkeypatch)
    q=copy.deepcopy(f.qualification)
    q[field]=({str(f.proof):'f'*64} if field == 'proof_hashes' else 'changed')
    write(f.baseline/'unified_live_qualification.json',q)
    with pytest.raises(RuntimeError):
        completion.verify_baseline(f.doc,f.baseline,f.selected,f.deficits)
    assert f.calls == {'native':0,'boundary_exit':1}


def test_terminal_contract_rejects_batch_and_failed_boundary(tmp_path,monkeypatch):
    f=terminal_contract_fixture(tmp_path,monkeypatch)
    original=json.loads((f.baseline/'batch_contract.json').read_text())
    write(f.baseline/'batch_contract.json',dict(original,import_driver_sha256='f'*64))
    with pytest.raises(RuntimeError,match='batch/import'):
        completion.verify_baseline(f.doc,f.baseline,f.selected,f.deficits)
    write(f.baseline/'batch_contract.json',original)
    write(f.baseline/'integrity_boundary_example.json',dict(policy=completion.common.POLICY,passed=True,result=1))
    with pytest.raises(RuntimeError,match='normally'):
        completion.verify_baseline(f.doc,f.baseline,f.selected,f.deficits)
    assert f.calls['native'] == 0


def test_terminal_contract_detects_ledger_mutation_during_check(tmp_path,monkeypatch):
    f=terminal_contract_fixture(tmp_path,monkeypatch)
    def authority_with_unexpected_writer(doc,parent):
        ledger=f.baseline/'selected_ledger.jsonl'
        ledger.write_bytes(ledger.read_bytes()+b'\n')
        return copy.deepcopy(doc['source_authority'])
    monkeypatch.setattr(completion.common,'source_authority',authority_with_unexpected_writer)
    with pytest.raises(RuntimeError,match='historical selection'):
        completion.verify_baseline(f.doc,f.baseline,f.selected,f.deficits)
    assert f.calls == {'native':0,'boundary_exit':1}
