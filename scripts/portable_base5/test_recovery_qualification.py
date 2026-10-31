"""CPU fixtures only: no ROS, native capture, remote or frozen source edits."""
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

import recovery_qualification as q


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def actor(ident=1):
    return dict(data_id=ident,unique_id=8500,name='FixtureAsset',source_list='objectList',
                motion='static',population_pool='obstacle',population_cost=2)


def metadata(family='obstacle',**spec):
    actors = [] if family == 'background' else [actor()]
    return dict(spec=dict(family=family,actors=len(actors),population_mode='generic_single',**spec),
                split='train',runtime_actors=actors)


def row(name,tokens):
    return dict(entry=dict(id=name),metadata=metadata(),tokens=sorted(tokens))


def test_matrix_deterministic_covers_all_before_outcome_and_forces_actual_failures():
    records = [row(q.MANDATORY_IDS[0],{'mode/dense','close','family/obstacle'}),
               row(q.MANDATORY_IDS[1],{'extent','capture/train'}),
               row('vehicle',{'far/rear','family/highway','capture/val'}),
               row('empty',{'empty','family/background'})]
    first,matrix = q.select_cases(records)
    second,again = q.select_cases(list(reversed(records)))
    assert [r['entry']['id'] for r in first[:2]] == list(q.MANDATORY_IDS)
    assert first == second and matrix == again
    assert not matrix['missing'] and matrix['all_branches_selected']
    assert matrix['selection_before_outcomes'] and len(first)<=24


def test_matrix_does_not_silently_relax_missing_coverage_or_mandatory_id():
    base = [row(name,{'mandatory/'+name}) for name in q.MANDATORY_IDS]
    records = base+[row('scene'+str(n),{'unique/'+str(n)}) for n in range(30)]
    selected,matrix = q.select_cases(records)
    assert len(selected) == 24 and matrix['missing'] and not matrix['all_branches_selected']
    with pytest.raises(RuntimeError,match='Mandatory'):
        q.select_cases(records[1:])


def test_branch_signature_covers_roster_rng_support_close_far_and_empty_shadow():
    meta = metadata(population_rng_reference=dict(actors=1),frozen_population_data_ids=[1],
                    avoidance_origin='generic_close',close_primary_observation={},
                    secondary_obstacle_support=dict(new_actor_ordinals=[1],protected_mid_ordinals=[2]),
                    secondary_far_vehicle=dict(target_ordinal=0),static_mission_extent=dict(stratum='after'))
    meta['runtime_actors'][0]['actual_ego_role']='rear'
    tokens=q.branch_tokens(dict(derived_training_split='val'),meta)
    assert {'roster/frozen','roster/legacy_rng_reference','support/added','support/protected_mid_ordinals',
            'close/generic','branch/close_primary_observation','far/rear','extent/after','effective_split/val'} <= tokens
    empty=metadata('shadow');empty['spec']['actors']=0;empty['runtime_actors']=[]
    assert {'roster/empty','shadow/empty'} <= q.branch_tokens({},empty)


@pytest.fixture
def inventory(monkeypatch):
    metas={};entries=[]
    families=sorted(q.FAMILIES)
    for n in range(5000):
        family=families[n%len(families)];meta=metadata(family)
        filename='/fixture/meta_'+str(n)+'.json';metas[filename]=meta
        entries.append(dict(id='scene_'+str(n),metadata=filename,scenario='/fixture/payload_'+str(n)+'.json',
            scenario_sha256='/fixture/payload_'+str(n)+'.json',family=family,split='train',
            derived_training_split='train',derived_layout_group_id='group_'+str(n)))
    monkeypatch.setattr(q.compat,'no_symlink',Path)
    monkeypatch.setattr(q.compat,'read',lambda path:copy.deepcopy(metas[str(path)]))
    monkeypatch.setattr(q.compat,'sha256',lambda path:str(path))
    layout=SimpleNamespace(read_overlay=lambda path:{},resolve=lambda overlay,name,entry,meta:
                           dict(split='train',group_id=entry['derived_layout_group_id']))
    return dict(count=5000,scenes=entries,split_overlay=dict(path='/fixture/overlay')),metas,layout


def test_actual5000_inventory_counts_preserves_specs_and_empty_shadow_contract(inventory):
    deck,metas,layout=inventory
    empty=next(meta for meta in metas.values() if meta['spec']['family']=='shadow')
    empty['runtime_actors']=[];empty['spec']['actors']=0
    before=copy.deepcopy(metas)
    records,audit,pins=q.source_inventory(deck,layout=layout)
    assert audit['passed'] and len(records)==5000 and len(pins)==10000
    assert audit['branch_counts']['shadow/empty']==1 and metas==before


@pytest.mark.parametrize('change',['count','roster','primary','budget','frozen','empty','duplicate'])
def test_inventory_rejects_unhealthy_source_contract_without_modifying_it(inventory,change):
    deck,metas,layout=inventory
    entry=next(e for e in deck['scenes'] if e['family']=='obstacle');meta=metas[entry['metadata']]
    if change=='count':meta['spec']['actors']=2
    if change=='roster':meta['runtime_actors'][0]['data_id']=True
    if change=='primary':meta['spec']['primary_identity']=2
    if change=='budget':meta['population_budget']=dict(actor_count=1,costs=[2],total_cost=8)
    if change=='frozen':meta['spec']['frozen_population_data_ids']=[2]
    if change=='empty':meta['runtime_actors']=[];meta['spec']['actors']=0
    if change=='duplicate':deck['scenes'][1]['id']=deck['scenes'][0]['id']
    with pytest.raises(RuntimeError):q.source_inventory(deck,layout=layout)


def test_destination_rejects_old_raw_symlinks_and_nonfixed_outputs(tmp_path):
    workspace=tmp_path/'ajounice_base5_capture'
    doc=dict(workspace=str(workspace),manifest=str(tmp_path/'source/manifest.json'),
             release=str(tmp_path/'release/release.json'),output=str(workspace/'collections/old69'),
             witness=str(workspace/'checks/witness.json'))
    assert q.destination(doc,workspace/'work/recovery-qualification/v1') == workspace/'work/recovery-qualification/v1'
    for output in (Path(doc['output']),workspace/'work/recovery-qualification/v2',tmp_path/'arbitrary'):
        with pytest.raises(RuntimeError):q.destination(doc,output)
    (tmp_path/'alias').symlink_to(workspace,target_is_directory=True)
    with pytest.raises(RuntimeError):q.destination(doc,tmp_path/'alias/work/recovery-qualification/v1')


def test_cpu_scope_denies_ros_network_and_process_then_restores():
    before=(subprocess.Popen,socket.socket)
    with q.original.cpu_only():
        for operation in (lambda:subprocess.Popen(['not-executed']),lambda:socket.socket(),
                          lambda:importlib.import_module('rospy')):
            with pytest.raises(RuntimeError):operation()
    assert before==(subprocess.Popen,socket.socket)


def test_output_guard_blocks_external_file_mutation_and_fresh_directory(tmp_path):
    output=tmp_path/'cpu';output.mkdir();old=write(tmp_path/'old.json',dict(preserved=True))
    before=old.read_bytes()
    with q.output_only(output):
        write(output/'new.json',dict(cpu=True))
        with pytest.raises(RuntimeError):old.write_text('changed')
        with pytest.raises(RuntimeError):(tmp_path/'outside').mkdir()
    assert old.read_bytes()==before and not (tmp_path/'outside').exists()


@pytest.fixture
def orchestration(tmp_path,monkeypatch):
    workspace=tmp_path/'ajounice_base5_capture';source=tmp_path/'immutable'
    manifest=write(source/'manifest.json',dict(count=5000,scenes=[],config={}))
    parent=write(workspace/'checks/route_compat_intent.json',dict(unit_fixture=True))
    witness=write(workspace/'checks/witness.json',dict(unit_fixture=True))
    release=write(tmp_path/'release/release.json',dict(unit_fixture=True))
    document=dict(workspace=str(workspace),manifest=str(manifest),release=str(release),
                  output=str(workspace/'collections/old69'),witness=str(witness),witness_sha256=q.compat.sha256(witness))
    monkeypatch.setattr(q.route_dispatch,'validate_intent',lambda path:document)
    monkeypatch.setattr(q.route_dispatch,'RELEASE_SHA',q.compat.sha256(release))
    records=[row(name,{'token/'+name}) for name in q.MANDATORY_IDS]+[row('another',{'branch/another'})]
    monkeypatch.setattr(q,'source_inventory',lambda *a,**kw:(records,dict(passed=True,source_scenes=5000),{}))
    active=set()
    @contextmanager
    def scope(name):
        active.add(name)
        try:
            if name=='population':
                yield dict(policy=q.POPULATION_POLICY,forced_roster=True,source_roster_preserved=True,
                           transient_choose_row_only=True,original_choose_rng_and_pool_budget_checks=True,native_qa_unchanged=True)
            else:yield
        finally:active.remove(name)
    class Exhausted(Exception):pass
    gen=SimpleNamespace()
    def replacement(path,output,**kwargs):
        assert active=={'route','spatial','population'}
        assert len(kwargs['known_payloads'])==1
        return write(Path(output)/'manifest.json',dict(fixture=True))
    quality=SimpleNamespace(gen=gen,replacement=replacement,PlacementProposalExhausted=Exhausted)
    modules=dict(generate_base5_independent=gen,base5_static_mission_extent=SimpleNamespace(),
        run_base5_quality_loop=quality,run_base5_capture_campaign=SimpleNamespace(subset_deck=lambda deck,entries:dict(count=1,scenes=entries)),
        base5_spatial_diversity=SimpleNamespace(gen=gen,installed=lambda:scope('spatial')),
        base5_layout_split=SimpleNamespace(),base5_unit_geometry_contract=SimpleNamespace(),
        recovery_population=SimpleNamespace(POLICY=q.POPULATION_POLICY,__file__=__file__,installed=lambda **kw:scope('population')))
    monkeypatch.setattr(q.compat,'installed',lambda *a,**kw:scope('route'))
    monkeypatch.setattr(q,'verify_replacement',lambda deck,row,path,**kw:dict(passed=True,source_id=row['entry']['id']))
    for r in records:r['entry']['scenario_sha256']='1'*64
    monkeypatch.setitem(sys.modules,'generate_base5_independent',gen)
    for name,module in modules.items():monkeypatch.setitem(sys.modules,name,module)
    return workspace,parent,quality,records,active


def test_plan_is_readonly_and_actual_run_preserves_preregistered_recipe(orchestration):
    workspace,parent,_,records,active=orchestration
    output=workspace/'work/recovery-qualification/v1'
    plan=q.run(parent,output)
    assert plan['plan_only'] and not output.exists()
    path=q.run(parent,output,execute=True);report=q.compat.read(path)
    assert path.name=='qualification.json' and report['passed']
    assert report['population_policy']==q.POPULATION_POLICY and report['replacement_count']==3
    assert str(Path(q.__file__).resolve()) in report['helper_hashes']
    assert report['matrix']['selected']==plan['recipe']['matrix']['selected']
    assert report['captures_requested']==0 and not report['native_capture_executed']
    assert not active


def test_bounded_exhaustion_is_not_approval_or_an_outcome_selected_replacement(orchestration):
    workspace,parent,quality,records,active=orchestration
    calls=[]
    def replacement(path,output,**kwargs):
        calls.append(path)
        raise quality.PlacementProposalExhausted('512 geometry / 8 layout bindings exhausted')
    quality.replacement=replacement
    path=q.run(parent,workspace/'work/recovery-qualification/v1',True);report=q.compat.read(path)
    assert not report['passed'] and report['bounded_exhaustions']==len(records)
    assert len(calls)==len(records) and report['unexpected_contract_stops']==0
    assert not report['problem_00082_candidate_pass'] and not active


def test_unexpected_contract_error_stops_without_claiming_partial_success(orchestration):
    workspace,parent,quality,_,active=orchestration
    quality.replacement=lambda *a,**kw:(_ for _ in ()).throw(RuntimeError('true frozen contract error'))
    output=workspace/'work/recovery-qualification/v1'
    with pytest.raises(RuntimeError,match='true frozen'):
        q.run(parent,output,True)
    assert not (output/'qualification.json').exists() and not active
    failed=q.compat.read(output/'qualification_failed.json')
    assert not failed['passed'] and failed['unexpected_contract_stops']==1


def test_new_profile_is_required_even_if_count_guard_imports(orchestration,monkeypatch):
    workspace,parent,_,_,_=orchestration
    monkeypatch.setattr(sys.modules['recovery_population'],'POLICY','old-count-only')
    with pytest.raises(RuntimeError,match='roster-preserving'):
        q.run(parent,workspace/'work/recovery-qualification/v1',True)


def candidate_fixture(tmp_path,background=False):
    source=tmp_path/'source';out=tmp_path/'case'
    scene=write(source/'scenario.json',dict(original=True))
    old=metadata('background' if background else 'obstacle');old['native_unit']='same-unit'
    old_meta=write(source/'metadata.json',old)
    before=dict(id='canonical',scenario=str(scene),scenario_sha256=q.compat.sha256(scene),
                metadata=str(old_meta),family=old['spec']['family'],mission=False,motion='static',split='train')
    new=copy.deepcopy(old)
    new['quality_replacement_contract']=dict(root_entry=before,immediate_parent_entry=before,
        immediate_parent_metadata_sha256=q.compat.sha256(old_meta),raw_gt_or_parent_modified=False)
    new['fresh_completion_source']=dict(source_cell_id='canonical',source_scenario_sha256=before['scenario_sha256'])
    candidate_scene=write(out/'scenario.json',dict(original=False))
    candidate_meta=write(out/'metadata.json',new)
    entry=dict(before,id='canonical_replace',scenario=str(candidate_scene),metadata=str(candidate_meta),
               scenario_sha256=q.compat.sha256(candidate_scene))
    deck=dict(config=dict(native_unit='same-unit'))
    manifest=write(out/'manifest.json',dict(deck,count=1,scenes=[entry]))
    quality=SimpleNamespace(replacement_contract=lambda *a:dict(overlay=True,root_entry=before))
    layout=SimpleNamespace(validate_manifest_overlay=lambda *a,**kw:None,overlay_for_metadata=lambda meta:{},
                           resolve=lambda *a:dict(split='train',group_id='same-group'))
    unit=SimpleNamespace(KEY='native_unit',validate_entry=lambda *a:None)
    return deck,dict(entry=before,metadata=old,tokens=[]),manifest,quality,layout,unit,new,candidate_meta


@pytest.mark.parametrize('background',[False,True])
def test_shared_candidate_verifier_accepts_exact_identity_and_real_empty_roster(tmp_path,background):
    deck,row,manifest,quality,layout,unit,_,_=candidate_fixture(tmp_path,background)
    old=Path(row['entry']['metadata']).read_bytes()
    result=q.verify_replacement(deck,row,manifest,quality=quality,layout=layout,unit=unit,output=tmp_path/'case')
    assert result['passed'] and result['empty_background']==background
    assert Path(row['entry']['metadata']).read_bytes()==old


@pytest.mark.parametrize('change',['data_id','unique_id','name','source_list','motion','population_pool',
                                  'spec_bool','config_bool','budget','parent','source_hash','unit'])
def test_shared_candidate_verifier_rejects_identity_metadata_and_parent_contract_changes(tmp_path,change):
    deck,row,manifest,quality,layout,unit,new,path=candidate_fixture(tmp_path)
    if change in q.IDENTITY_FIELDS:new['runtime_actors'][0][change]='foreign'
    elif change=='spec_bool':new['spec']['actors']=True
    elif change=='config_bool':
        deck['config']['flag']=False
        child=q.compat.read(manifest);child['config']['flag']=0;write(manifest,child)
    elif change=='budget':new['population_budget']=dict(actor_count=9)
    elif change=='parent':new['quality_replacement_contract']['immediate_parent_entry']={}
    elif change=='source_hash':new['fresh_completion_source']['source_scenario_sha256']='foreign'
    elif change=='unit':new['native_unit']='foreign'
    write(path,new)
    with pytest.raises(RuntimeError):
        q.verify_replacement(deck,row,manifest,quality=quality,layout=layout,unit=unit,output=tmp_path/'case')
