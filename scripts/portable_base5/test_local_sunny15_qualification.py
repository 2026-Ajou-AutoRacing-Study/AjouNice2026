"""Local Sunny15 CPU fixtures only; no ROS, capture, remote, or frozen edits."""
from contextlib import contextmanager
import copy
import importlib
import json
from pathlib import Path
import socket
import subprocess
import sys
from types import SimpleNamespace

import pytest

import local_sunny15_qualification as q


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + '\n')
    return path


def row(name, tokens):
    return dict(entry=dict(id=name), metadata=dict(spec=dict(family='obstacle')),
                tokens=sorted(tokens))


def test_local_matrix_forces_four_actual_cells_and_nested_axis_before_outcomes():
    rows = [row(name, {'mandatory/'+name, 'lineage/nested' if index >= 2 else 'lineage/original'})
            for index,name in enumerate(q.MANDATORY)]
    rows += [row('empty', {'roster/empty'}), row('far', {'far/rear'})]
    selected,matrix = q.select_cases(rows)
    selected_again,matrix_again = q.select_cases(list(reversed(rows)))
    assert [r['entry']['id'] for r in selected[:4]] == list(q.MANDATORY)
    assert selected == selected_again and matrix == matrix_again
    assert matrix['all_branches_selected'] and not matrix['missing']
    assert matrix['selection_before_outcomes'] and len(selected) <= 24
    assert 'not the full Cartesian' in matrix['coverage_scope']


def test_matrix_missing_mandatory_or_uncovered_axes_is_not_silently_accepted():
    mandatory = [row(name, {'axis/'+name}) for name in q.MANDATORY]
    with pytest.raises(RuntimeError, match='mandatory'):
        q.select_cases(mandatory[1:])
    selected,matrix = q.select_cases(mandatory+[row(str(n), {'unique/'+str(n)}) for n in range(30)])
    assert len(selected) == 24 and matrix['missing'] and not matrix['all_branches_selected']


def test_exact_v159_source_contract_verifies_all_438_runtime_and_input_pins(tmp_path,monkeypatch):
    runtime={str(tmp_path/('runtime_%03d.py'%index)):'a'*64 for index in range(438)}
    source=tmp_path/'manifest.json';release=tmp_path/'release.json';input_path=tmp_path/'source_metadata.json'
    monkeypatch.setattr(q,'MANIFEST',source);monkeypatch.setattr(q,'RELEASE',release)
    monkeypatch.setattr(q.files,'sha256',lambda p:q.MANIFEST_SHA if Path(p)==source else q.RELEASE_SHA)
    monkeypatch.setattr(q.files,'read',lambda p:dict(runtime_hashes=runtime,input_hashes={str(input_path):'b'*64}))
    calls=[];monkeypatch.setattr(q.shared,'verify_pins',lambda pins:calls.append(copy.deepcopy(pins)))
    result=q.source_contract()
    assert len(result)==441 and calls==[result]
    assert result[str(source)]==q.MANIFEST_SHA and result[str(release)]==q.RELEASE_SHA
    assert set(runtime)<=set(result) and str(input_path) in result


@pytest.mark.parametrize('failure',['source_sha','runtime_count','conflicting_pin'])
def test_source_contract_rejects_wrong_release_or_incomplete_runtime_before_cpu(tmp_path,monkeypatch,failure):
    source,release=tmp_path/'manifest.json',tmp_path/'release.json'
    runtime={str(tmp_path/('runtime_%03d.py'%index)):'a'*64 for index in range(438)}
    inputs={}
    if failure=='runtime_count':runtime.pop(next(iter(runtime)))
    if failure=='conflicting_pin':inputs[next(iter(runtime))]='b'*64
    monkeypatch.setattr(q,'MANIFEST',source);monkeypatch.setattr(q,'RELEASE',release)
    monkeypatch.setattr(q.files,'sha256',lambda p:('wrong' if failure=='source_sha' else q.MANIFEST_SHA)
                        if Path(p)==source else q.RELEASE_SHA)
    monkeypatch.setattr(q.files,'read',lambda p:dict(runtime_hashes=runtime,input_hashes=inputs))
    monkeypatch.setattr(q.shared,'verify_pins',lambda pins:pytest.fail('Unhealthy qualification must stop before pin verification'))
    with pytest.raises(RuntimeError):q.source_contract()


def candidate(tmp_path, nested=False, empty=False):
    source,output = tmp_path/'source',tmp_path/'case'
    original_scene = write(source/'scenario.json', dict(scene='current qualified parent'))
    actor = dict(data_id=10,unique_id=8500,name='FixtureBox',source_list='objectList',motion='static',
                 population_pool='obstacle',training_class='obstacle',population_cost=2,
                 asset_scale=dict(profile_id='native',scale_xyz=[1.,1.,1.]))
    actors = [] if empty else [actor]
    old = dict(spec=dict(family='background' if empty else 'obstacle',actors=len(actors),
                        primary_identity=None if empty else 10,population_mode='generic_single'),
               split='train',runtime_actors=actors,native_unit=dict(path='fixture',sha256='f'*64),
               scene_group_id='source-group',logical_quota_slot_id='logical-slot',
               environment_split_contract=dict(policy='same'),fresh_environment_provenance=dict(source='same'),
               population_budget=dict(mode='generic_single',layout='clustered',actor_count=len(actors),
                   costs=[] if empty else [2],total_cost=0 if empty else 2,
                   requested_bounds=[0,3],required_visible_actors=len(actors)))
    metadata_path = write(source/'metadata.json', old)
    before = dict(id='canonical',scenario=str(original_scene),scenario_sha256=q.files.sha256(original_scene),
                  metadata=str(metadata_path),family=old['spec']['family'],mission=False,role='front',
                  distance_bounds=[0,20],subfamily='same',motion='static',split='train')
    root = copy.deepcopy(before)
    if nested:
        root_scene = write(source/'older_scenario.json', dict(scene='original canonical root'))
        root_metadata = write(source/'older_metadata.json', old)
        root.update(scenario=str(root_scene),scenario_sha256=q.files.sha256(root_scene),metadata=str(root_metadata))
        old['quality_replacement_contract'] = dict(root_entry=root,ordinal=1)
        write(metadata_path,old)
    context = dict(overlay=True,root_entry=root,root_metadata_sha256=q.files.sha256(root['metadata']),
                   ordinal=2 if nested else 1)
    new = copy.deepcopy(old)
    new['quality_replacement_contract'] = dict(root_entry=root,root_metadata_sha256=context['root_metadata_sha256'],
        ordinal=context['ordinal'],immediate_parent_entry=before,
        immediate_parent_metadata_sha256=q.files.sha256(metadata_path),raw_gt_or_parent_modified=False)
    new['fresh_completion_source'] = dict(source_cell_id=root['id'],source_scenario_sha256=root['scenario_sha256'])
    scene = write(output/'scenario.json', dict(scene='different candidate'))
    metadata = write(output/'metadata.json',new)
    entry = dict(before,id='canonical_replace',scenario=str(scene),scenario_sha256=q.files.sha256(scene),metadata=str(metadata))
    deck = dict(config=dict(native_unit=old['native_unit']))
    child = write(output/'manifest.json',dict(deck,count=1,scenes=[entry]))
    quality = SimpleNamespace(replacement_contract=lambda *args:copy.deepcopy(context))
    calls = []
    unit = SimpleNamespace(KEY='native_unit',validate_entry=lambda *args:calls.append('unit'))
    layout = SimpleNamespace(validate_manifest_overlay=lambda *args,**kwargs:calls.append('layout'),
        overlay_for_metadata=lambda metadata:metadata,
        resolve=lambda overlay,ident,entry,meta:dict(split=meta.get('effective_split','train'),group_id=meta.get('effective_group','same-group')))
    record = dict(entry=before,metadata=old,tokens=['lineage/nested' if nested else 'lineage/original'])
    return dict(deck=deck,record=record,manifest=child,quality=quality,layout=layout,unit=unit,
                new=new,metadata=metadata,entry=entry,output=output,root=root,context=context,calls=calls)


def verify(f):
    return q.verify_replacement(f['deck'],f['record'],f['manifest'],quality=f['quality'],
                               layout=f['layout'],unit=f['unit'],output=f['output'])


@pytest.mark.parametrize('nested,empty', [(False,False),(True,False),(False,True)])
def test_candidate_retains_current_parent_canonical_root_and_empty_roster(tmp_path,nested,empty):
    f = candidate(tmp_path,nested=nested,empty=empty)
    old_bytes = Path(f['record']['entry']['metadata']).read_bytes()
    result = verify(f)
    assert result['passed'] and result['nested_parent'] == nested
    assert result['lineage_ordinal'] == (2 if nested else 1)
    assert result['actor_count'] == (0 if empty else 1)
    assert set(f['calls']) == {'unit','layout'}
    assert old_bytes == Path(f['record']['entry']['metadata']).read_bytes()
    assert result['native_capture_executed'] is False and result['qa_changed'] is False


@pytest.mark.parametrize('change', [
    'spec','roster','motion','scale_profile','config','unit','capture_split','effective_split','effective_group',
    'scene_group_id','environment_split_contract','logical_quota_slot_id','fresh_environment_provenance',
    'budget_count','budget_cost','budget_visibility','root','root_metadata_sha','ordinal','immediate_parent',
    'immediate_metadata_sha','fresh_root_sha','parent_modified','same_payload','scenario_sha','payload_escape'])
def test_candidate_rejects_spec_roster_motion_unit_budget_split_and_nested_lineage_changes(tmp_path,change):
    f = candidate(tmp_path,nested=True)
    new,proof = f['new'],f['new']['quality_replacement_contract']
    if change == 'spec':new['spec']['actors'] = True
    elif change == 'roster':new['runtime_actors'].append(copy.deepcopy(new['runtime_actors'][0]))
    elif change == 'motion':new['runtime_actors'][0]['motion'] = 'walking'
    elif change == 'scale_profile':new['runtime_actors'][0]['asset_scale']['profile_id']='foreign'
    elif change == 'config':
        child=q.files.read(f['manifest']);child['config']['extra']=True;write(f['manifest'],child)
    elif change == 'unit':new['native_unit'] = dict(foreign=True)
    elif change == 'capture_split':new['split']='val'
    elif change == 'effective_split':new['effective_split']='val'
    elif change == 'effective_group':new['effective_group']='foreign-group'
    elif change in ('scene_group_id','environment_split_contract','logical_quota_slot_id','fresh_environment_provenance'):
        new[change]='foreign'
    elif change == 'budget_count':new['population_budget']['actor_count']=5
    elif change == 'budget_cost':new['population_budget']['costs']=[99]
    elif change == 'budget_visibility':new['population_budget']['required_visible_actors']=0
    elif change == 'root':proof['root_entry']=f['record']['entry']
    elif change == 'root_metadata_sha':proof['root_metadata_sha256']='bad'
    elif change == 'ordinal':proof['ordinal']=1
    elif change == 'immediate_parent':proof['immediate_parent_entry']=f['root']
    elif change == 'immediate_metadata_sha':proof['immediate_parent_metadata_sha256']='bad'
    elif change == 'fresh_root_sha':new['fresh_completion_source']['source_scenario_sha256']=f['record']['entry']['scenario_sha256']
    elif change == 'parent_modified':proof['raw_gt_or_parent_modified']=True
    elif change in ('same_payload','scenario_sha','payload_escape'):
        child=q.files.read(f['manifest'])
        if change == 'same_payload':
            p=Path(child['scenes'][0]['scenario']);p.write_bytes(Path(f['record']['entry']['scenario']).read_bytes())
            child['scenes'][0]['scenario_sha256']=q.files.sha256(p)
        elif change == 'scenario_sha':child['scenes'][0]['scenario_sha256']='bad'
        else:child['scenes'][0]['metadata']=f['record']['entry']['metadata']
        write(f['manifest'],child)
    write(f['metadata'],new)
    with pytest.raises(RuntimeError):verify(f)


def test_actual_unit_and_layout_body_validation_errors_are_not_swallowed(tmp_path):
    f=candidate(tmp_path)
    def reject_body(*args):raise ValueError('Original unit/body identity mismatch')
    f['unit'].validate_entry=reject_body
    with pytest.raises(ValueError,match='unit/body'):verify(f)


def test_cpu_and_output_guards_deny_ros_socket_subprocess_and_old_writes(tmp_path):
    output=tmp_path/'cpu';output.mkdir();old=write(tmp_path/'old.json',dict(preserved=True))
    before=old.read_bytes();originals=(socket.socket,subprocess.Popen)
    with q.isolated.cpu_only(),q.shared.output_only(output):
        write(output/'new.json',dict(cpu=True))
        for operation in (lambda:importlib.import_module('rospy'),lambda:socket.socket(),
                          lambda:subprocess.Popen(['not-executed']),lambda:old.write_text('forbidden'),
                          lambda:(output/'../old.json').write_text('forbidden')):
            with pytest.raises(RuntimeError):operation()
    assert old.read_bytes()==before and originals==(socket.socket,subprocess.Popen)


@pytest.fixture
def orchestration(tmp_path,monkeypatch):
    final=tmp_path/'final';final.mkdir()
    manifest=write(final/'manifest.json',dict(count=5000,scenes=[],config={}))
    output=final/'cpu'
    monkeypatch.setattr(q,'MANIFEST',manifest);monkeypatch.setattr(q,'OUTPUT',output)
    monkeypatch.setattr(q,'source_contract',lambda:{str(manifest):q.files.sha256(manifest)})
    records=[row(name,{'mandatory/'+name}) for name in q.MANDATORY]
    for r in records:
        r['entry']['scenario_sha256']='a'*64
        if r['entry']['id'] in q.MANDATORY[2:]:r['metadata']['quality_replacement_contract']=dict(original=True)
    monkeypatch.setattr(q.shared,'source_inventory',lambda *args,**kwargs:
        (records,dict(passed=True,source_scenes=5000),{}))
    active=[];calls=[]
    class Exhausted(Exception):pass
    def replacement(source,destination,**kwargs):
        assert active==['spatial','population','direction']
        assert kwargs['known_payloads']=={'a'*64}
        calls.append(str(source))
        return write(Path(destination)/'manifest.json',dict(cpu=True))
    quality=SimpleNamespace(replacement=replacement,PlacementProposalExhausted=Exhausted,__file__=__file__)
    @contextmanager
    def spatial_scope():
        active.append('spatial')
        try:yield
        finally:active.pop()
    @contextmanager
    def population_scope(**kwargs):
        assert active==['spatial'];active.append('population')
        try:yield dict(policy=q.shared.POPULATION_POLICY,native_qa_unchanged=True)
        finally:active.pop()
    @contextmanager
    def direction_scope(contract):
        assert active==['spatial','population'];active.append('direction')
        previous=getattr(quality,'execute',None)
        quality.execute=SimpleNamespace(__globals__=dict(replacement=quality.replacement))
        try:yield
        finally:quality.execute=previous;active.pop()
    modules=dict(run_base5_quality_loop=quality,
        run_base5_capture_campaign=SimpleNamespace(subset_deck=lambda deck,entries:dict(count=1,scenes=entries)),
        base5_spatial_diversity=SimpleNamespace(installed=spatial_scope),base5_layout_split=SimpleNamespace(),
        base5_unit_geometry_contract=SimpleNamespace(),
        run_base5_capture_campaign_direction_reference=SimpleNamespace(installed=direction_scope),
        recovery_population=SimpleNamespace(POLICY=q.shared.POPULATION_POLICY,__file__=__file__,installed=population_scope))
    for name,module in modules.items():monkeypatch.setitem(sys.modules,name,module)
    monkeypatch.setattr(q,'verify_replacement',lambda deck,record,path,**kwargs:
        dict(passed=True,source_id=record['entry']['id'],native_capture_executed=False,qa_changed=False))
    return output,quality,records,active,calls


def test_plan_is_readonly_and_actual_cpu_recipe_order_is_preregistered(orchestration):
    output,quality,records,active,calls=orchestration
    plan=q.run(False)
    assert plan['plan_only'] and not plan['files_written'] and not output.exists() and calls==[]
    report_path=q.run(True);report=q.files.read(report_path)
    assert report['passed'] and report['replacement_count']==4 and len(calls)==4
    assert report['matrix']['selected']==plan['recipe']['matrix']['selected']
    assert report['direction_ast_uses_guard'] and not active
    assert report['captures_requested']==0 and not report['native_capture_executed']
    assert not report['bulk_authorized'] and 'no new native capture' in report['limitation']
    assert {'lineage/original','lineage/nested'} <= set(report['matrix']['required'])


def test_bounded_exhaustion_remains_failure_without_outcome_based_reselection(orchestration):
    output,quality,records,active,calls=orchestration
    def exhausted(*args,**kwargs):
        calls.append('bounded');raise quality.PlacementProposalExhausted('bounded original geometry exhausted')
    quality.replacement=exhausted
    path=q.run(True);report=q.files.read(path)
    assert not report['passed'] and report['bounded_exhaustions']==4 and len(calls)==4
    assert not report['problem_00082_candidate_pass'] and not active


def test_unexpected_contract_failure_stops_and_never_reports_success(orchestration):
    output,quality,records,active,calls=orchestration
    quality.replacement=lambda *args,**kwargs:(_ for _ in ()).throw(RuntimeError('real contract change'))
    with pytest.raises(RuntimeError,match='real contract'):q.run(True)
    assert not active and not (output/'qualification.json').exists()
    failure=q.files.read(output/'qualification_failed.json')
    assert not failure['passed'] and failure['unexpected_contract_stops']==1


def test_existing_qualification_output_is_never_overwritten(orchestration):
    output,*_=orchestration;output.mkdir();marker=write(output/'old.json',dict(preserved=True))
    before=marker.read_bytes()
    with pytest.raises(RuntimeError,match='Preserve old'):q.run(False)
    assert marker.read_bytes()==before


def test_wrong_population_profile_stops_before_any_candidate(orchestration,monkeypatch):
    output,quality,records,active,calls=orchestration
    monkeypatch.setattr(sys.modules['recovery_population'],'POLICY','unqualified-count-only')
    with pytest.raises(RuntimeError,match='population policy'):q.run(True)
    assert not output.exists() and calls==[] and not active


def test_direction_ast_must_reference_the_actual_installed_population_wrapper(orchestration,monkeypatch):
    output,quality,records,active,calls=orchestration
    @contextmanager
    def unguarded_ast(contract):
        quality.execute=SimpleNamespace(__globals__=dict(replacement=lambda *args:None))
        yield
    monkeypatch.setattr(sys.modules['run_base5_capture_campaign_direction_reference'],'installed',unguarded_ast)
    with pytest.raises(RuntimeError,match='omitted the population guard'):q.run(True)
    assert calls==[] and not active and not (output/'qualification.json').exists()


def test_cli_plan_prints_only_compact_counts_not_full_hash_recipe(monkeypatch,capsys):
    recipe=dict(matrix=dict(selected=[{},{}],required=['one','two'],all_branches_selected=True),
                source_hashes={'private-large-path':'unused'})
    monkeypatch.setattr(q,'run',lambda execute:dict(recipe=recipe,output='/fresh/cpu'))
    assert q.main([])==0
    message=json.loads(capsys.readouterr().out)
    assert message['selected']==2 and message['branches']==2 and not message['native_capture_executed']
    assert not message['files_written'] and 'source_hashes' not in message and 'recipe' not in message
