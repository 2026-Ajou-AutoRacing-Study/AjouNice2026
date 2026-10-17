"""원형 smoke의 관측 전 결정적 선정과 immutable 등록 계약을 검사한다."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_unit_release_smoke as smoke


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value)+'\n');return path


@pytest.fixture
def selection(tmp_path,monkeypatch):
    assets = {'physical'+str(i) for i in range(19)};entries = [];metadata = {}
    def add(ident,family='obstacle',actors=(),spec=None,split='train',shadow=False):
        meta = dict(spec=dict(family=family,mission=False,**(spec or {})),runtime_actors=list(actors),
            split=split,training_split_overlay=dict(enabled=True),**{smoke.unit.KEY:dict(reference=True)})
        if shadow:meta[smoke.shadow.KEY] = dict(materialized=True)
        path = write(tmp_path/'source'/(ident+'.json'),meta)
        entry = dict(id=ident,metadata=str(path),scenario_sha256='a'*64,split=split)
        metadata[ident] = meta;entries.append(entry)
    for i in range(2):add('shadow'+str(i),'shadow',[dict(name='person',motion='walking',source_list='pedestrianList',
        motion_path=dict(ground_path_evidence=dict(policy='independent-terrain-sampled-v46')))],
        spec=dict(shadow_moving=True),shadow=True)
    for crossing in smoke.CROSSWALKS:
        add(crossing,'pedestrian_related',[dict(name='person',motion='running',source_list='pedestrianList',mission_crossing_id=crossing,
            motion_path=dict(ground_path_evidence=dict(policy=smoke.EDITOR_MOTION_POLICY)))])
    for lo,hi in ((50,80),(80,100)):
        for role in ('front','rear'):
            add(f'far{lo}{role}','highway',[dict(name='car',motion='static',source_list='vehicleList',
                actual_ego_role=role,planned_center_distance_m=(lo+hi)/2)])
    for i,name in enumerate(sorted(assets)):
        spec = dict(mission=True,static_mission_extent=dict(stratum=('before','intersection','after')[i%3]),
            avoidance_phase=smoke.PHASES[i%4])
        # mission 기본값과의 중복 인자는 add 안에서 명시적 update로 처리한다.
        meta_spec = dict(spec);meta_spec.pop('mission')
        add(name,actors=[dict(name=name,motion='static',source_list='objectList',planned_center_distance_m=10 if i%2 else 25)],
            spec=meta_spec,split='val' if i%2 else 'train')
        metadata[name]['spec']['mission'] = True
        write(Path(entries[-1]['metadata']),metadata[name])
    for name in smoke.COMPOSITES:add(name,'pedestrian_related',[dict(name=name,motion='standing',source_list='objectList')])
    for i in range(20):add('extra'+str(i))
    for i in range(22):add('natural'+str(i),'shadow',
        [dict(name='person',motion='walking',source_list='pedestrianList')],spec=dict(shadow_moving=True))
    deck = dict(count=len(entries),scenes=entries,config={smoke.unit.KEY:dict(reference=True)})
    monkeypatch.setattr(smoke.layout,'resolve',lambda overlay,ident,entry,meta:dict(split=meta['split']))
    return dict(deck=deck,metadata=metadata,overlay={},unit_assets=assets,shadow_ids=['shadow0','shadow1'],
        entries={e['id']:e for e in entries})


def choose(selection,target=32,maximum=48):
    return smoke.select(selection['deck'],selection['metadata'],selection['overlay'],selection['unit_assets'],
        selection['shadow_ids'],target,maximum)


def test_exact32_preserves19_assets_four_compounds_and_eight_protected_controls(selection):
    rows,coverage = choose(selection)
    assert len(rows) == len({r['id'] for r in rows}) == 32
    assert coverage['protected_frames'] == 8 and coverage['native_outcomes_used_for_selection'] is False
    assert set(coverage['required']) <= set(coverage['covered'])
    assert {'physical_asset/'+name for name in selection['unit_assets']} <= set(coverage['covered'])
    assert {'compound/'+name for name in smoke.COMPOSITES} <= set(coverage['covered'])
    assert all(row['source_entry_preserved'] for row in rows)
    original = rows,coverage
    selection['deck']['scenes'].reverse();selection['metadata'] = dict(reversed(list(selection['metadata'].items())))
    assert choose(selection) == original


def test_explicit48_is_possible_but_never_automatic(selection):
    assert len(choose(selection,48,48)[0]) == 48
    assert len(choose(selection)[0]) == 32


def test_unprepared22_are_only_excluded_from_exact_smoke_without_changing_full_source(selection):
    before = copy.deepcopy((selection['deck'],selection['metadata']))
    source_hashes = {entry['metadata']:smoke.sha256(entry['metadata']) for entry in selection['deck']['scenes']}
    rows,coverage = choose(selection)
    eligible,report = smoke.selection_eligibility(selection['metadata'],selection['shadow_ids'])
    assert report == coverage['selection_eligibility']
    assert report['source_scenes'] == 73 and report['eligible_count'] == 51
    assert report['eligible_scene_counts'] == dict(static=47,editor_prepared=2,independent_terrain_materialized=2)
    assert report['excluded_count'] == 22 and report['excluded_scene_counts'] == dict(unprepared_natural_shadow=22)
    assert report['excluded_ids'] == sorted('natural'+str(i) for i in range(22))
    assert all(row['id'] in eligible for row in rows)
    assert report['source_quota_modified'] is report['native_outcomes_used_for_selection'] is False
    assert report['separate_on_demand_preparation_approvals_counted'] == 0
    assert (selection['deck'],selection['metadata']) == before
    assert all(smoke.sha256(path) == digest for path,digest in source_hashes.items())


def test_unprepared_unique_coverage_cannot_fill_exact_smoke_even_at48(selection):
    physical = selection['metadata']['physical0'];actor = physical['runtime_actors'].pop()
    selection['metadata']['natural0']['runtime_actors'].append(actor)
    with pytest.raises(ValueError,match='이미 준비된 exact source에 필수 coverage가 없다'):
        choose(selection,48,48)


def test_smoke_ineligible_source_still_requires_valid_global_overlay(selection,monkeypatch):
    def resolve(overlay,ident,entry,meta):
        if ident == 'natural0':raise ValueError('미선정 full source의 overlay도 엄격 검증한다')
        return dict(split=meta['split'])
    monkeypatch.setattr(smoke.layout,'resolve',resolve)
    with pytest.raises(ValueError,match='미선정 full source의 overlay'):
        choose(selection)


def test_static_reviewed_source_is_eligible_without_id_shape_or_outcome_filter(selection):
    # 실제 review ID도 일반 static과 동일하게 취급한다. 결과로 blacklist하지 않는다.
    old = 'extra0';ident = 'sunny13_sunny11_b5v2_03598'
    entry = selection['entries'].pop(old);entry['id'] = ident;selection['entries'][ident] = entry
    meta = selection['metadata'].pop(old);selection['metadata'][ident] = meta
    baseline = choose(selection)
    meta.update(status='review',review_reasons=['primary_no_sensor_evidence'],native_outcomes=['review']*4,
        diagnostic_shape=dict(height_m=.01))
    assert ident in smoke.selection_eligibility(selection['metadata'],selection['shadow_ids'])[0]
    assert choose(selection) == baseline


@pytest.mark.parametrize('target,maximum',[(31,48),(49,49),(32,49),(32,31),(True,48),(32,True)])
def test_declared_finite_cap_rejects_invalid_target(selection,target,maximum):
    with pytest.raises(ValueError):choose(selection,target,maximum)


@pytest.mark.parametrize('kind',['physical','compound','crosswalk','far','shadow','extent','split','range'])
def test_missing_required_metadata_is_fail_closed(selection,kind):
    metadata = selection['metadata']
    if kind == 'physical':metadata['physical0']['runtime_actors'] = []
    if kind == 'compound':metadata['Ped_Pet1']['runtime_actors'] = []
    if kind == 'crosswalk':metadata[smoke.CROSSWALKS[0]]['runtime_actors'][0]['motion'] = 'standing'
    if kind == 'far':metadata['far80rear']['runtime_actors'][0]['planned_center_distance_m'] = 49
    if kind == 'shadow':metadata['shadow0']['runtime_actors'][0]['motion'] = 'static'
    if kind == 'extent':
        for meta in metadata.values():meta['spec'].pop('static_mission_extent',None)
    if kind == 'split':
        for meta in metadata.values():meta['split'] = 'train'
    if kind == 'range':
        for meta in metadata.values():
            for actor in meta['runtime_actors']:
                if actor['name'] in selection['unit_assets']:actor['planned_center_distance_m'] = 10
    with pytest.raises(ValueError):choose(selection)


@pytest.fixture
def registered(selection,tmp_path,monkeypatch):
    full = write(tmp_path/'full'/'manifest.json',selection['deck'])
    pin = write(tmp_path/'runtime.json',{});hashes = {str(pin):smoke.sha256(pin)}
    context = dict(selection,input_hashes={str(full):smoke.sha256(full)},raw_scale_counts={'objectList/explicit111':19},
        motion_preparation_inventory=dict(fixture_only=True,all_source_moving_paths_prepared=False))
    monkeypatch.setattr(smoke,'runtime_hashes',lambda:dict(hashes))
    monkeypatch.setattr(smoke,'inspect_source',lambda manifest:copy.deepcopy(context))
    monkeypatch.setattr(smoke.audit_base5_scenarios,'audit',lambda path:dict(status='OFFLINE_PASS',scenes=5000,unique_payloads=5000,errors=[]))
    output = tmp_path/'registered';smoke.prepare(full,output)
    return full,output,context,pin


def test_registration_replays_exact_subset_and_only_history_is_external(registered):
    full,output,_,_ = registered
    context = smoke.registration_context(full,output/'registration.json')
    assert len(context['selected']) == 32 and context['doc']['scale_paired_gate_required'] is False
    assert context['doc']['completion_live_qualified'] is False
    assert 'paired_stability' not in context['doc'] and 'qualified_profiles' not in context['doc']
    assert context['subset']['config'] == context['deck']['config']
    assert context['subset']['scenes'] == [context['entries'][r['id']] for r in context['doc']['selected']]
    assert context['doc']['selection_eligibility'] == context['doc']['coverage']['selection_eligibility']
    assert context['doc']['selection_eligibility']['excluded_count'] == 22
    assert len(context['deck']['scenes']) == 73


@pytest.mark.parametrize('kind',['helper','release_helper','source','subset','audit','selection','runtime','time_nan','time_future',
    'completion','paired','unit_reference','partial','input','support','motion_inventory',
    'eligibility_count','eligibility_ids','eligibility_states','coverage_eligibility'])
def test_preregistration_changes_do_not_retroactively_qualify(registered,kind):
    full,output,_,pin = registered;path = output/'registration.json';doc = smoke.read(path)
    if kind == 'helper':doc['helper_sha256'] = '0'*64
    if kind == 'release_helper':doc['release_helper_sha256'] = '0'*64
    if kind == 'source':write(full,{})
    if kind == 'subset':write(output/'manifest.json',{})
    if kind == 'audit':write(output/'source_audit.json',{})
    if kind == 'selection':doc['selected'].reverse()
    if kind == 'runtime':write(pin,dict(changed=True))
    if kind == 'time_nan':doc['registered_at_unix'] = float('nan')
    if kind == 'time_future':doc['registered_at_unix'] = 10**15
    if kind == 'completion':doc['completion_live_qualified'] = True
    if kind == 'paired':doc['scale_paired_gate_required'] = True
    if kind == 'unit_reference':doc['unit_reference'] = {}
    if kind == 'partial':doc['partial_resume_plan']['chunk_size'] = 8
    if kind == 'input':doc['input_hashes'] = {}
    if kind == 'support':doc['support_evidence_hashes'] = {str(pin):'0'*64}
    if kind == 'motion_inventory':doc['motion_preparation_inventory']['all_source_moving_paths_prepared'] = True
    if kind == 'eligibility_count':doc['selection_eligibility']['excluded_count'] = 0
    if kind == 'eligibility_ids':doc['selection_eligibility']['excluded_ids'] = []
    if kind == 'eligibility_states':doc['selection_eligibility']['allowed_preparation_states'].append('unprepared_natural_shadow')
    if kind == 'coverage_eligibility':doc['coverage']['selection_eligibility']['excluded_count'] = 0
    write(path,doc)
    with pytest.raises((ValueError,KeyError)):smoke.registration_context(full,path)


def test_registration_never_overwrites_existing_source_or_output(registered):
    full,output,_,_ = registered
    with pytest.raises(ValueError):smoke.prepare(full,output)
    with pytest.raises(ValueError):smoke.prepare(full,full.parent)


def test_runtime_freezes_unit_adapters_and_shared_native_input_cpu_contracts():
    hashes = smoke.runtime_hashes()
    for filename in ('prepare_base5_unit_release_smoke.py','finalize_base5_unit_release.py',
        'base5_native_box_corners.py','base5_unit_capture_provenance.py','qualify_base5_training_loader_cpu.py',
        'finalize_base5_final_bounded_release.py'):
        path = Path(smoke.__file__).with_name(filename)
        assert hashes[str(path.resolve())] == smoke.sha256(path)


@pytest.fixture
def source(tmp_path,monkeypatch):
    names = ['physical'+str(i) for i in range(19)]
    catalog = write(tmp_path/'catalog.json',dict(assets=[dict(data_id=100+i,name=name,source_list='objectList',semantic_class='obstacle')
        for i,name in enumerate(names)]))
    reference = dict(path=str(tmp_path/'policy.json'),sha256='a'*64)
    policy = dict(assets={name:{} for name in names},catalog_reference=dict(path=str(catalog),sha256=smoke.sha256(catalog)))
    scenario = write(tmp_path/'scenario.json',dict(objectList=[dict(UNIQUEID=8500,DataID=100,scale=dict(x=1.,y=1.,z=1.))]))
    metadata = write(tmp_path/'metadata.json',dict(split='train',spec={},runtime_actors=[dict(unique_id=8500,data_id=100,
        name=names[0],source_list='objectList',motion='static')],**{smoke.unit.KEY:reference}))
    registry = write(tmp_path/'registry.json',dict(entries=[dict(source_id='0'),dict(source_id='1')]))
    overlay = write(tmp_path/'overlay.json',{})
    entries = [dict(id=str(i),scenario=str(scenario),metadata=str(metadata),scenario_sha256=smoke.sha256(scenario),split='train') for i in range(5000)]
    deck = dict(count=5000,scenes=entries,config=dict(environment_tag='sunny13',**{
        smoke.unit.KEY:reference,smoke.shadow.KEY:dict(registration=dict(path=str(registry)))}),split_overlay=dict(path=str(overlay)))
    full = write(tmp_path/'manifest.json',deck)
    monkeypatch.setattr(smoke.unit,'load',lambda *args:copy.deepcopy(policy))
    monkeypatch.setattr(smoke.layout,'validate_manifest_overlay',lambda *args:dict(passed=True))
    monkeypatch.setattr(smoke.layout,'read_overlay',lambda *args:{})
    monkeypatch.setattr(smoke.shadow,'validate_manifest',lambda *args:{str(registry):smoke.sha256(registry)})
    monkeypatch.setattr(smoke.pins,'collect_inputs',lambda *args:{str(full):smoke.sha256(full)})
    # 이 fixture는 raw admission만 격리한다. 이동 inventory는 아래 별도 fixture로 검사한다.
    monkeypatch.setattr(smoke,'motion_preparation_inventory',lambda *args:dict(fixture_only=True))
    return full,deck,scenario,metadata


def test_source_admission_validates_every_raw_scale_and_actor_not_only_selected32(source):
    full,_,_,_ = source
    context = smoke.inspect_source(full)
    assert len(context['entries']) == 5000 and context['raw_scale_counts'] == {'objectList/explicit111':5000}


@pytest.mark.parametrize('kind',['policy','nonunit','object_missing_scale','scale_assignment','reference','capture_split','duplicate_id','actor_identity'])
def test_source_admission_rejects_active_scale_or_identity_mismatch(source,kind):
    full,deck,scenario,metadata = source;meta = smoke.read(metadata);scene = smoke.read(scenario)
    if kind == 'policy':deck['config']['asset_scale_policy'] = {}
    if kind == 'nonunit':scene['objectList'][0]['scale']['x'] = 1.25
    if kind == 'object_missing_scale':scene['objectList'][0].pop('scale')
    if kind == 'scale_assignment':meta['runtime_actors'][0]['asset_scale'] = dict(scale_xyz=[1,1,1])
    if kind == 'reference':meta[smoke.unit.KEY] = {}
    if kind == 'capture_split':deck['scenes'][0]['split'] = 'val'
    if kind == 'duplicate_id':deck['scenes'][1]['id'] = '0'
    if kind == 'actor_identity':meta['runtime_actors'][0]['name'] = 'physical1'
    write(metadata,meta);write(scenario,scene)
    for entry in deck['scenes']:entry['scenario_sha256'] = smoke.sha256(scenario)
    write(full,deck)
    with pytest.raises(ValueError):smoke.inspect_source(full)


@pytest.fixture
def motion_inventory(tmp_path,monkeypatch):
    metadata = {};entries = {};calls = []
    for ident,kind in [('static','static'),('background','static'),('editor','editor'),('terrain0','terrain'),
                       ('terrain1','terrain'),('natural','natural')]:
        actor = dict(name='person',unique_id=8500,motion='static' if kind == 'static' else 'walking')
        if kind == 'editor':actor['motion_path'] = dict(ground_path_evidence=dict(policy=smoke.EDITOR_MOTION_POLICY))
        if kind == 'terrain':actor['motion_path'] = dict(ground_path_evidence=dict(policy='independent-terrain-sampled-v46'))
        meta = dict(spec=dict(family='shadow' if kind in ('terrain','natural') else 'pedestrian_related',
            shadow_moving=kind in ('terrain','natural')),runtime_actors=[] if ident == 'background' else [actor])
        if kind == 'terrain':meta[smoke.shadow.KEY] = dict(materialized=True)
        metadata[ident] = meta;entries[ident] = dict(scenario=str(write(tmp_path/(ident+'.json'),dict(id=ident))))
    monkeypatch.setattr(smoke.motion_ground,'validate_motion_ground',lambda meta,scene:calls.append(scene['id']))
    return entries,metadata,['terrain0','terrain1'],calls


def test_actual_motion_states_are_partitioned_without_native_or_preparation_claims(motion_inventory):
    entries,metadata,registered,calls = motion_inventory
    #  계보/진단 태그만 생겨도 실제 terrain 준비 완료가 되지는 않는다.
    metadata['natural'].update(calibration_layout_provenance=dict(diagnostic=True),quality_replacement_contract=dict(bound=True))
    result = smoke.motion_preparation_inventory(entries,metadata,registered)
    assert result['scene_counts'] == dict(static=2,editor_prepared=1,independent_terrain_materialized=2,unprepared_natural_shadow=1)
    assert result['total_scenes'] == 6 and result['moving_scenes'] == 4
    assert result['moving_parent_counts']['unprepared_natural_shadow'] == 1
    assert result['on_demand_preparation_required']['scene_ids'] == ['natural']
    assert result['on_demand_preparation_required']['must_finish_before_training_capture'] is True
    assert result['all_source_moving_paths_prepared'] is False and result['native_capture_approval_claimed'] is False
    assert set(calls) == {'editor','terrain0','terrain1'}


@pytest.mark.parametrize('kind',['unknown_motion','missing_motion','unknown_proof','partial_preparation','nonshadow_unprepared',
    'static_shadow','forged_registry','unregistered_terrain','calibration_only','invalid_witness'])
def test_motion_inventory_scope_or_witness_mismatch_is_strict_error(motion_inventory,kind,monkeypatch):
    entries,metadata,registered,_ = motion_inventory
    if kind == 'unknown_motion':metadata['natural']['runtime_actors'][0]['motion'] = 'unknown'
    if kind == 'missing_motion':metadata['natural']['runtime_actors'][0].pop('motion')
    if kind == 'unknown_proof':metadata['natural']['runtime_actors'][0]['motion_path'] = dict(ground_path_evidence=dict(policy='claimed_prepared'))
    if kind == 'partial_preparation':metadata['editor']['runtime_actors'].append(dict(name='other',motion='walking'))
    if kind == 'nonshadow_unprepared':metadata['natural']['spec']['family'] = 'pedestrian_related'
    if kind == 'static_shadow':metadata['terrain0']['runtime_actors'][0]['motion'] = 'static'
    if kind == 'forged_registry':metadata['natural'][smoke.shadow.KEY] = dict(materialized=True)
    if kind == 'unregistered_terrain':
        metadata['natural']['runtime_actors'][0]['motion_path'] = dict(ground_path_evidence=dict(policy='independent-terrain-sampled-v46'))
    if kind == 'calibration_only':metadata['natural']['calibration_only'] = True
    if kind == 'invalid_witness':
        def reject(*args):raise ValueError('원문 waypoint/SHA witness가 다르다')
        monkeypatch.setattr(smoke.motion_ground,'validate_motion_ground',reject)
    with pytest.raises(ValueError):smoke.motion_preparation_inventory(entries,metadata,registered)
