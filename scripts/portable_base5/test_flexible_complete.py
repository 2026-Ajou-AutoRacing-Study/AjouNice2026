"""Pure CPU policy tests. No MORAI/ROS/network or capture entrypoint is run."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

import flexible_complete as flex


def meta(asset=1, distance=15., actors=2, motion='static', family='obstacle'):
    return dict(spec=dict(family=family,mission=False,distance_bounds=[10,20],role='front',yaw_limit=3),
        split='train',planned_primary_distance_m=distance,scene_group_id='group',
        runtime_actors=[dict(data_id=asset,motion=motion,source_list='obstacleList') for _ in range(actors)])


def test_asset_roster_count_direction_and_road_may_change_but_purpose_stays():
    source=meta();candidate=meta(asset=2,actors=1)
    candidate['spec'].update(role='right',yaw_limit=15,road_position='new')
    assert flex.compatible(source,candidate,{1:'obstacle',2:'obstacle'})


@pytest.mark.parametrize('field', ['family','mission','split','pool','motion','distance','scale'])
def test_learning_purpose_guard_is_not_relaxed(field):
    source=meta();candidate=meta(asset=2,actors=1);pools={1:'obstacle',2:'obstacle'}
    if field=='family':candidate['spec']['family']='shadow'
    elif field=='mission':candidate['spec']['mission']=True
    elif field=='split':candidate['split']='val'
    elif field=='pool':pools[2]='person'
    elif field=='motion':candidate['runtime_actors'][0]['motion']='running'
    elif field=='distance':candidate['planned_primary_distance_m']=25.
    else:candidate['runtime_actors'][0]['asset_scale']={'profile_id':'large'}
    assert not flex.compatible(source,candidate,pools)


def test_far_secondary_60_80_cannot_replace_80_100_goal():
    source=meta(family='highway');source['spec']['secondary_far_vehicle']={'distance_bounds_m':[90,95]}
    candidate=meta(asset=2,family='highway')
    candidate['spec']['secondary_far_vehicle']={'distance_bounds_m':[60,80]}
    assert not flex.compatible(source,candidate,{1:'vehicle',2:'vehicle'})
    candidate['spec']['secondary_far_vehicle']['distance_bounds_m']=[80,90]
    assert flex.compatible(source,candidate,{1:'vehicle',2:'vehicle'})
    candidate['spec'].pop('secondary_far_vehicle')
    candidate['planned_primary_distance_m']=85.
    assert flex.compatible(source,candidate,{1:'vehicle',2:'vehicle'})


def test_fresh_donors_prefer_other_asset_and_fewer_required_actors():
    source=meta();donors=[({'scenario_sha256':'old'},source),
        ({'scenario_sha256':'many'},meta(asset=2,actors=5)),
        ({'scenario_sha256':'simple'},meta(asset=2,actors=1))]
    ordered=flex.donor_order(source,donors,{1:'obstacle',2:'obstacle'},'slot',1,set())
    assert ordered[0][0]['scenario_sha256']=='simple'
    assert all(e['scenario_sha256']!='simple' for e,m in
        flex.donor_order(source,donors,{1:'obstacle',2:'obstacle'},'slot',2,{'simple'}))


def test_merge_is_append_only_and_one_approval_per_deficit():
    original={'source_cell_id':'a','id':'native-a','capture_root':'a'}
    state=dict(source_order=['a','b'],deficits=['b'],contract={'adapter_sha256':'sha'},attempts=[])
    selected,missing=flex.merge(state,[original])
    assert selected==[original] and missing==['b']
    attempt=dict(id='b',donor_id='donor',manifest='manifest',result={'selected':{
        'source_cell_id':'candidate','id':'native-b','capture_root':'b'}})
    state['attempts']=[attempt]
    selected,missing=flex.merge(state,[original])
    assert selected[0]==original and selected[1]['source_cell_id']=='b' and not missing
    state['attempts'].append(copy.deepcopy(attempt))
    with pytest.raises(RuntimeError,match='Duplicate'):
        flex.merge(state,[original])


def test_default_plan_never_prepares_verifies_or_captures(tmp_path,monkeypatch):
    monkeypatch.setattr(flex,'summary',lambda *args:({},None,None,tmp_path,{},[{}],['one'],{}))
    monkeypatch.setattr(flex.exact,'verify_baseline',lambda *args:pytest.fail('No baseline verifier in plan'))
    args=flex.parser().parse_args(['--profile','unused','--parent','unused','--output','unused'])
    result=flex.execute(args)
    assert not result['capture_started'] and not result['files_written']


def test_capture_keeps_current_native_verifier_and_one_outer_round(tmp_path,monkeypatch):
    calls=[]
    direction=SimpleNamespace(run=lambda args:calls.append(args) or 0)
    monkeypatch.setattr(flex.common,'modules',lambda:(None,direction))
    manifest=tmp_path/'manifest.json';manifest.write_text('{"scenes":[{"metadata":"meta"}]}')
    monkeypatch.setattr(flex,'read',lambda path:{'scenes':[{'metadata':'meta'}]} if path==str(manifest) else meta())
    from contextlib import nullcontext
    monkeypatch.setattr(flex.exact.recovery_population,'installed',lambda:nullcontext())
    monkeypatch.setattr(flex.exact,'nonrepeat',lambda *args:nullcontext())
    monkeypatch.setattr(flex.exact,'inherited_native',lambda *args:nullcontext())
    monkeypatch.setattr(flex,'verify_attempt',lambda attempt, original:{'selected':'verified'})
    result=flex.capture(dict(child=str(tmp_path/'new'),manifest=str(manifest),scenario_sha256='fresh'),{'old','fresh'})
    assert result=={'selected':'verified'} and calls[0].quota_rounds==1
    assert calls[0].enable_final_direction_reference and not calls[0].approve_bulk_capture


def test_ast_adapter_only_removes_explicit_placement_guards():
    source=Path(__file__).parents[2]/'dataset/morai_od/scripts/base5_layout_split.py'
    tree=ast.parse(source.read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='register_recovery_overlay')
    module=ast.Module(body=[fn],type_ignores=[])
    scope={};exec(compile(module,str(source),'exec'),scope)
    # inspect.getsource resolves the real immutable source via its co_filename.
    layout=SimpleNamespace(register_recovery_overlay=scope['register_recovery_overlay'])
    clone=flex.rebind_clone(layout)
    messages={c for c in clone.__code__.co_consts if isinstance(c,str)}
    assert 'Recovery changed actor identity/motion/scale roster' not in messages
    assert 'Recovery payload checksum differs' in messages
    assert 'Recovery mapping/source split/group is not explicitly preserved' in messages


def test_ast_adapter_fails_closed_when_original_shape_changes(monkeypatch):
    monkeypatch.setattr(flex.inspect,'getsource',lambda fn:'def changed():\n    return True\n')
    with pytest.raises(RuntimeError,match='must be reviewed'):
        flex.rebind_clone(SimpleNamespace(register_recovery_overlay=lambda:None))


def test_success_donor_preserves_effective_split_and_ignores_overlapping_size_pools(tmp_path,monkeypatch):
    import json
    native=tmp_path/'native';native.mkdir()
    metadata=meta(asset=1,actors=1)
    metadata['spec']['distance_bounds']=[10,20]
    (native/'metadata.json').write_text(json.dumps(metadata))
    pools={name:[] for name in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider')}
    pools['obstacle']=[{'data_id':1}]
    pools['obstacle_low']=[{'data_id':1}]
    gen=SimpleNamespace(geo=SimpleNamespace(DEFAULT_RUNTIME_ASSET_DIR='a',DEFAULT_MGEO='m'),
        context=lambda *args:{'pools':pools})
    monkeypatch.setattr(flex.common,'modules',lambda:(SimpleNamespace(batch=SimpleNamespace(
        campaign=SimpleNamespace(loop=SimpleNamespace(gen=gen)))),None))
    deck={'scenes':[dict(id='source',derived_training_split='val',derived_layout_group_id='layout-group')]}
    donors,classification=flex.donor_pool(deck,[dict(id='runtime',source_cell_id='source',
        capture_root=str(native),scenario_sha256='sha')])
    assert classification[1]=='obstacle'
    assert donors[0][0]['derived_training_split']=='val'
    assert donors[0][0]['derived_layout_group_id']=='layout-group'
    assert donors[0][0]['split']=='train'  # Capture split is not rewritten.


def test_finalizer_requires_complete_append_only_selection_before_any_cpu_rebuild(tmp_path,monkeypatch):
    # The finalizer import is lazy; a fake module verifies the initial gate
    # rejects incomplete/non-preserved selection before inspecting any native.
    import sys,json
    monkeypatch.setitem(sys.modules,'finalize_base5_layout_split',SimpleNamespace())
    (tmp_path/'selected_ledger.jsonl').write_text(json.dumps({'source_cell_id':'a'})+'\n')
    state=dict(stage='complete_with_deficit',source_order=['a','b'],deficits=['b'],attempts=[],contract={})
    with pytest.raises(RuntimeError,match='complete 5K'):
        flex.finalize_selection('unused',tmp_path,state,[{'source_cell_id':'a'}])


def prepared_fixture(tmp_path):
    import json
    output=tmp_path/'flexible';folder=output/'cells'/'b'/'attempt_1'/'template_1'
    placement=folder/'placement';placement.mkdir(parents=True)
    scenario=placement/'scenario.json';scenario.write_text('{"fresh":true}\n')
    metadata=placement/'metadata.json';metadata.write_text('{"actors":1}\n')
    manifest=placement/'manifest.json'
    manifest.write_text(json.dumps({'scenes':[dict(scenario=str(scenario),metadata=str(metadata),
        scenario_sha256=flex.common.sha(scenario))]})+'\n')
    attempt=dict(id='b',ordinal=1,manifest=str(manifest),manifest_sha256=flex.common.sha(manifest),
        metadata_sha256=flex.common.sha(metadata),scenario_sha256=flex.common.sha(scenario),
        child=str(folder/'campaign'),result=None)
    baseline=[dict(source_cell_id='a',id='old-a')]
    state=dict(source_order=['a','b'],deficits=['b'],contract={'attempts':2,'adapter_sha256':'adapter'},
        attempts=[attempt])
    flex.atomic_rows(output/'selected_ledger.jsonl',baseline)
    return output,state,baseline,attempt,metadata


def test_prepared_resume_accepts_unchanged_sidecar_but_rejects_changed_metadata(tmp_path):
    output,state,baseline,attempt,metadata=prepared_fixture(tmp_path)
    deck={'scenes':[{'id':'a'},{'id':'b'}]}
    flex.validate_resume(state,output,deck,['b'],baseline)
    metadata.write_text('{"actors":2}\n')
    with pytest.raises(RuntimeError,match='payload escaped'):
        flex.validate_resume(state,output,deck,['b'],baseline)


@pytest.mark.parametrize('change',['foreign_row','escaped_capture','child_path','duplicate','ordinal'])
def test_selected_resume_binds_saved_result_to_real_child_and_registered_slot(tmp_path,change):
    output,state,baseline,attempt,metadata=prepared_fixture(tmp_path)
    child=Path(attempt['child']);child.mkdir()
    row=dict(id='new-b',source_cell_id='new-b',capture_root=str(child/'raw'))
    attempt.update(donor_id='donor',result={'selected':row})
    ledger=child/'selected_ledger.jsonl';flex.atomic_rows(ledger,[row])
    attempt['evidence_pins']={str(ledger):flex.common.sha(ledger)}
    flex.atomic_rows(output/'selected_ledger.jsonl',flex.merge(state,baseline)[0])
    deck={'scenes':[{'id':'a'},{'id':'b'}]}
    flex.validate_resume(state,output,deck,['b'],baseline)
    if change=='foreign_row':
        flex.atomic_rows(ledger,[dict(row,id='another-native')])
        attempt['evidence_pins']={str(ledger):flex.common.sha(ledger)}
    elif change=='escaped_capture':
        row['capture_root']=str(tmp_path/'outside')
        flex.atomic_rows(ledger,[row]);attempt['evidence_pins']={str(ledger):flex.common.sha(ledger)}
    elif change=='child_path':attempt['child']=str(tmp_path/'outside')
    elif change=='duplicate':state['attempts'].append(copy.deepcopy(attempt))
    elif change=='ordinal':attempt['ordinal']=3
    with pytest.raises(RuntimeError):
        flex.validate_resume(state,output,deck,['b'],baseline)


def test_resume_nonrepeat_inventory_includes_actual_fallback_native_ledgers(tmp_path):
    import inspect,json
    output=tmp_path/'flexible';native=output/'cells'/'slot'/'template'/'campaign'/'retry_0'
    native.mkdir(parents=True)
    (native/'ledger.jsonl').write_text(json.dumps({'scenario_sha256':'f'*64})+'\n')
    assert 'f'*64 in flex.exact.prior_payloads(output,dict(scenes=[]))
    # Ensure execute actually consumes that output inventory, rather than only
    # its prepared-manifest SHAs (a fallback can have a different native SHA).
    tree=ast.parse(inspect.getsource(flex.execute))
    calls=[node for node in ast.walk(tree) if isinstance(node,ast.Call)
        and isinstance(node.func,ast.Attribute) and node.func.attr=='prior_payloads']
    assert any(call.args and isinstance(call.args[0],ast.Name) and call.args[0].id=='output' for call in calls)


@pytest.mark.parametrize('rounds,accepted',[(1,True),(1,False),(2,True)])
def test_native_verifier_uses_one_actual_round_and_retains_original_completed_proof(tmp_path,monkeypatch,rounds,accepted):
    import json
    child=tmp_path/'campaign';child.mkdir()
    source=tmp_path/'manifest.json';source.write_text('{"scenes":[{"id":"candidate"}]}\n')
    native=child/'raw';native.mkdir();(native/'metadata.json').write_text(json.dumps(meta())+'\n')
    selected=[dict(id='actual',source_cell_id='candidate',capture_root=str(native))] if accepted else []
    values={'status.json':{'stage':'complete' if accepted else 'complete_with_deficit'},
        'quota.json':dict(accepted=int(accepted),deficit=int(not accepted),pending=0),
        'campaign_contract.json':{'rounds':rounds},
        'candidate/round_0/report.json':dict(stage='complete',selected=selected)}
    for name,value in values.items():
        path=child/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value)+'\n')
    flex.atomic_rows(child/'selected_ledger.jsonl',selected)
    calls=[]
    campaign=SimpleNamespace(contract=lambda manifest,indices,count:dict(rounds=count),
        validate_selected=lambda report:calls.append('native-report'))
    direction=SimpleNamespace(verify_campaign=lambda *args:calls.append('direction'),
        verify_completed_campaign=lambda *args:calls.append('completed-native-proof') or {'accepted_row':selected[0]})
    parent=SimpleNamespace(batch=SimpleNamespace(campaign=campaign,
        verify_recovery=lambda *args:pytest.fail('Old API incorrectly fixes rounds=2')))
    monkeypatch.setattr(flex.common,'modules',lambda:(parent,direction))
    attempt=dict(manifest=str(source),manifest_sha256=flex.common.sha(source),child=str(child))
    if rounds==2:
        with pytest.raises(RuntimeError,match='one-round'):
            flex.verify_attempt(attempt,meta())
    else:
        result=flex.verify_attempt(attempt,meta())
        assert bool(result['selected'])==accepted and 'direction' in calls and 'native-report' in calls
        assert ('completed-native-proof' in calls)==accepted
