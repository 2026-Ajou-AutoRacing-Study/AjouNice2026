"""최종 source의 사전 등록, 엄격한 증거 연결, release gate 회귀 검사."""
import copy
import json
from pathlib import Path
import pickle
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_final_bounded_smoke as smoke
import finalize_base5_final_bounded_release as release


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n');return path


def jsonl(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows));return path


def selection_fixture(tmp_path,monkeypatch):
    pairs={('CargoBox','iso300')}|{('asset'+str(i),'qualified') for i in range(16)}
    required={'profile/'+name+'/'+profile for name,profile in pairs}
    required.update('extent_stratum/'+value for value in ('before','intersection','after'))
    required.update('extent_phase/'+value for value in smoke.legacy_features.PHASES)
    required.update(('extent_training_split/train','extent_training_split/val','pipeline/layout_overlay',
        'pipeline/unit_reference','pipeline/materialized_shadow','unit_control/physical'))
    entries=[];metadata={}
    def add(ident,family,actors,coverage=(),scaled=False):
        meta=dict(spec=dict(family=family),runtime_actors=actors,_coverage=sorted(coverage),
            _scaled=scaled,_effective='train')
        path=write(tmp_path/'source'/(ident+'_metadata.json'),meta)
        entry=dict(id=ident,metadata=str(path),scenario_sha256='a'*64)
        entries.append(entry);metadata[ident]=meta
        return meta
    for index in range(2):
        meta=add('shadow'+str(index),'shadow',[dict(motion='walking')],required-{t for t in required if t.startswith('profile/')})
        meta[smoke.shadow.KEY]={'materialized':True}
    for crossing in smoke.legacy_features.CROSSWALKS:
        add(crossing,'pedestrian_related',[dict(motion='running',mission_crossing_id=crossing)])
    for lo,hi in ((50,80),(80,100)):
        for role in ('front','rear'):
            add(f'far_{lo}_{role}','highway',[dict(source_list='vehicleList',actual_ego_role=role,
                planned_center_distance_m=(lo+hi)/2)])
    others=sorted(required & {'profile/'+name+'/'+profile for name,profile in pairs if name!='CargoBox'})
    for index in range(13):
        tokens=set(others[index*2:index*2+2])
        if index<3:tokens.add('profile/CargoBox/iso300')
        add('scale'+str(index),'obstacle',[],tokens,True)
    for index in range(8):add('extra'+str(index),'obstacle',[])
    deck=dict(count=len(entries),scenes=entries,config={})
    monkeypatch.setattr(smoke,'features',lambda meta,*args:set(meta['_coverage']))
    monkeypatch.setattr(smoke.legacy_features,'variants',lambda meta,*args:[dict(fixture=True)] if meta['_scaled'] else [])
    monkeypatch.setattr(smoke.layout,'resolve',lambda overlay,ident,entry,meta:dict(split=meta['_effective']))
    def choose(target=24,maximum=64):
        return smoke.select(deck,metadata,pairs,{},set(),['shadow0','shadow1'],target,maximum)
    return deck,metadata,pairs,choose


def test_dynamic17_profiles_exact24_and_all_three_rare_cargo_are_preregistered(tmp_path,monkeypatch):
    deck,metadata,pairs,choose=selection_fixture(tmp_path,monkeypatch)
    rows,coverage=choose();ids={r['id'] for r in rows}
    assert len(rows)==len(ids)==24 and len(pairs)==17
    assert coverage['profile_minima']['profile/CargoBox/iso300']==3
    assert coverage['source_profile_frames']['profile/CargoBox/iso300']==3
    assert coverage['selected_profile_frames']['profile/CargoBox/iso300']==3
    assert {'scale0','scale1','scale2'}<=ids
    assert coverage['protected_frames']==8 and coverage['native_outcomes_used_for_selection'] is False
    assert set(coverage['required'])<=set(coverage['covered'])
    expected=rows,coverage
    deck['scenes'].reverse();original=copy.deepcopy(metadata)
    metadata.clear();metadata.update(reversed(list(original.items())))
    assert choose()==expected


@pytest.mark.parametrize('target,maximum',[(True,64),(0,64),(65,64),(24,65),(24,True),(8,64)])
def test_selection_is_finitely_bounded_and_never_increases_cap(tmp_path,monkeypatch,target,maximum):
    _,_,_,choose=selection_fixture(tmp_path,monkeypatch)
    with pytest.raises(ValueError):choose(target,maximum)


@pytest.mark.parametrize('kind',['rare_shortage','crosswalk_static','shadow_static','far_missing','protected_scaled'])
def test_required_source_geometry_shortage_is_fail_closed(tmp_path,monkeypatch,kind):
    _,metadata,_,choose=selection_fixture(tmp_path,monkeypatch)
    if kind=='rare_shortage':metadata['scale2']['_coverage'].remove('profile/CargoBox/iso300')
    if kind=='crosswalk_static':metadata[smoke.legacy_features.CROSSWALKS[0]]['runtime_actors'][0]['motion']='standing'
    if kind=='shadow_static':metadata['shadow0']['runtime_actors'][0]['motion']='standing'
    if kind=='far_missing':metadata['far_80_rear']['runtime_actors'][0]['planned_center_distance_m']=49
    if kind=='protected_scaled':metadata['shadow0']['_scaled']=True
    with pytest.raises(ValueError):choose()


def test_qualified_profile_count_is_dynamic_not_legacy7_by14(monkeypatch):
    policy=dict(assets={'asset'+str(i):dict(profiles={'p'+str(j):{} for j in range(3)}) for i in range(8)})
    monkeypatch.setattr(smoke.scale,'validate_policy',lambda value:None)
    monkeypatch.setattr(smoke.scale,'_qualified',lambda value:True)
    assert len(smoke.qualified_pairs(policy))==24
    policy['assets']={}
    with pytest.raises(ValueError):smoke.qualified_pairs(policy)


def test_runtime_freezes_both_new_helpers_core_unit_validator_and_cross_root_tools():
    hashes=smoke.runtime_hashes()
    for path in (Path(smoke.__file__),Path(release.__file__),Path(release.unit_provenance.__file__),
            smoke.pins.ROOT/'perception/detection3d/bevfusion/tools/prepare_base5_portable_dataset.py',
            smoke.pins.ROOT/'perception/detection3d/bevfusion/tools/prepare_base5_pedestrian_merged.py'):
        assert hashes[str(path.resolve())]==smoke.sha256(path)


def registration_fixture(tmp_path,monkeypatch):
    deck,metadata,pairs,choose=selection_fixture(tmp_path,monkeypatch)
    rows,coverage=choose()
    evidence=write(tmp_path/'profile_evidence.json',dict(native=True))
    ref=dict(path=str(evidence),sha256=smoke.sha256(evidence))
    policy=dict(assets={name:dict(profiles={profile:dict(native_qualification=dict(evidence=[ref]))}) for name,profile in pairs})
    pin=write(tmp_path/'runtime_pin.json',{});hashes={str(pin):smoke.sha256(pin)}
    full=write(tmp_path/'full'/'manifest.json',deck)
    context=dict(deck=deck,entries={entry['id']:entry for entry in deck['scenes']},metadata=metadata,
        pairs=pairs,overlay={},unit_assets=set(),shadow_ids=['shadow0','shadow1'],policy=policy,input_hashes=hashes)
    monkeypatch.setattr(smoke,'runtime_hashes',lambda:hashes)
    monkeypatch.setattr(smoke,'inspect_source',lambda manifest:copy.deepcopy(context))
    paired_manifest=write(tmp_path/'paired'/'manifest.json',dict(fixture=True))
    paired=dict(manifest=paired_manifest,input_hashes=hashes,profile_scene_counts={},rare_source_ids=[],
        gate=copy.deepcopy(smoke.STABILITY_GATE))
    monkeypatch.setattr(smoke,'stability_context',lambda *args:copy.deepcopy(paired))
    def select(*args):
        assert args[-2:]==(24,64)
        return copy.deepcopy(rows),copy.deepcopy(coverage)
    monkeypatch.setattr(smoke,'select',select)
    monkeypatch.setattr(smoke.audit_base5_scenarios,'audit',lambda manifest:dict(status='OFFLINE_PASS',errors=[],scenes=5000,unique_payloads=5000))
    subset=smoke.prepare(full,tmp_path/'smoke',support_evidence=[evidence],stability_manifest=paired_manifest)
    registration=subset.parent/'registration.json'
    return full,registration,smoke.read(registration),context


@pytest.mark.parametrize('kind',['valid','own_helper','release_helper','runtime','source','subset','selection',
    'policy','evidence_truncated','evidence_changed','support_changed','posthoc','completion','partial_plan','audit',
    'paired_threshold','paired_source','paired_pins','timestamp','retrospective'])
def test_registration_pins_both_helpers_exact_inputs_and_complete_evidence(tmp_path,monkeypatch,kind):
    full,path,doc,context=registration_fixture(tmp_path,monkeypatch)
    if kind=='own_helper':doc['helper_sha256']='0'*64
    if kind=='release_helper':doc['release_helper_sha256']='0'*64
    if kind=='runtime':doc['runtime_hashes']={}
    if kind=='source':write(full,dict(changed=True))
    if kind=='subset':write(path.parent/'manifest.json',dict(changed=True))
    if kind=='selection':doc['selected'][0]['selection_roles']=['posthoc']
    if kind=='policy':doc['policy_sha256']='0'*64
    if kind=='evidence_truncated':doc['qualification_evidence_hashes']={}
    if kind=='evidence_changed':write(Path(next(iter(doc['qualification_evidence_hashes']))),dict(changed=True))
    if kind=='support_changed':doc['support_evidence_hashes'][str(full)]='0'*64
    if kind=='posthoc':doc['live_qualified']=True
    if kind=='completion':doc['completion_live_qualified']=True
    if kind=='partial_plan':doc['partial_resume_plan']['preserved_selected']=3
    if kind=='audit':write(path.parent/'source_audit.json',dict(changed=True))
    if kind=='paired_threshold':doc['paired_stability']['gate']['minimum_initial_approval_fraction']=.80
    if kind=='paired_source':doc['paired_stability']['manifest_sha256']='0'*64
    if kind=='paired_pins':doc['paired_stability']['input_hashes']={}
    if kind=='timestamp':doc['registered_at_unix']=-1
    if kind=='retrospective':doc['native_observations_must_follow_registration']=False
    write(path,doc)
    if kind=='valid':
        verified=smoke.registration_context(full,path)
        assert len(verified['selected'])==24
        assert verified['doc']['completion_live_qualified'] is False
    else:
        with pytest.raises(ValueError):smoke.registration_context(full,path)


def native_fixture(tmp_path,monkeypatch):
    capture=tmp_path/'capture';entries={};metadata={};selected={};rows=[]
    reference={'fixture':'pinned_unit_reference'};required={'pipeline/layout_overlay','profile/CargoBox/iso300'}
    for index in range(24):
        ident='slot_'+str(index);meta=dict(spec=dict(family='obstacle'),runtime_actors=[],split='train',
            _cargo=index<3,**{release.unit.KEY:reference})
        scenario=write(tmp_path/'source'/ident/'scenario.json',dict(id=ident))
        path=write(scenario.with_name('metadata.json'),meta)
        entry=dict(id=ident,scenario=str(scenario),metadata=str(path),scenario_sha256=release.sha256(scenario))
        entries[ident]=entry;metadata[ident]=copy.deepcopy(meta);selected[ident]=dict(id=ident)
        raw=capture/'cells'/ident/'initial'/'captures'/ident
        write(raw/'scenario.json',dict(id=ident));meta['capture_runtime_ego']={'diagnostic_only':True}
        write(raw/'metadata.json',meta)
        review=dict(review_reasons=[])
        audit=dict(components=[dict(component_id='actor0')],accepted_candidate=True,review_reasons=[],native_unit_geometry_review=review)
        audit['asset_scale_review']=release.scale_capture.native_size_review(meta,audit)
        write(raw/'audit.json',audit)
        samples=[dict(t=t,message=dict(collision_object=[dict(unique_id=0)],collision_objecta=[])) for t in (0.,.5,1.)]
        write(raw/'collision_history.json',dict(summary=release.collision_window(samples,0.,1.),samples=samples))
        write(raw.parents[1]/'shutdown_safety.json',dict(park_verified=True))
        write(raw.parents[1]/'session_contract.json',{})
        jsonl(raw.parents[1]/'event_cmd_requests.jsonl',[dict(recorded_at=1.,phase='park')])
        rows.append(dict(id=ident,source_cell_id=ident,status='accepted_candidate',capture_root=str(raw),
            scenario_sha256=entry['scenario_sha256'],audit=audit))
    jsonl(capture/'selected_ledger.jsonl',rows)
    write(capture/'status.json',dict(stage='complete',requested=24,accepted_candidate=24,review=0,elapsed_sec=10.))
    subset=write(tmp_path/'subset'/'manifest.json',{})
    context=dict(subset_path=subset,entries=entries,metadata=metadata,selected=selected,pairs={('CargoBox','iso300')},
        overlay={},unit_assets=set(),deck=dict(config={release.unit.KEY:reference,'obstacle_support_policy':'fixture-policy'}),
        doc=dict(registered_at_unix=.5,selected=[dict(id=ident) for ident in selected],coverage=dict(required=sorted(required),
            profile_minima={'profile/CargoBox/iso300':3})))
    monkeypatch.setattr(release.completion,'verify_parent',lambda *args:(release.training.ledger(capture/'selected_ledger.jsonl'),[],10.))
    monkeypatch.setattr(release.layout,'resolve',lambda *args:dict(split='train'))
    monkeypatch.setattr(release.shadow,'verify_capture',lambda *args:None)
    monkeypatch.setattr(release.unit,'review',lambda *args:dict(review_reasons=[]))
    monkeypatch.setattr(release.unit_provenance,'verify_capture',lambda *args:None)
    native=[];monkeypatch.setattr(release.training,'assert_native',lambda *args:native.append(args))
    monkeypatch.setattr(release.smoke,'features',lambda meta,*args:{'pipeline/layout_overlay'}|({'profile/CargoBox/iso300'} if meta['_cargo'] else set()))
    return context,capture,rows,native


def test_exact24_native_and_actual_rare_profile_counts_are_rechecked(tmp_path,monkeypatch):
    context,capture,_,native=native_fixture(tmp_path,monkeypatch)
    result,blockers=release.verify_native(context,capture)
    assert not blockers and result['accepted']==24 and len(native)==24
    assert result['accepted_profile_frames']['profile/CargoBox/iso300']==3
    assert result['external_support_approvals_counted']==0


def test_native_rechecks_core_unit_provenance_with_exact_source_and_durable_row(tmp_path,monkeypatch):
    context,capture,rows,_=native_fixture(tmp_path,monkeypatch);calls=[]
    monkeypatch.setattr(release.unit_provenance,'verify_capture',lambda *args:calls.append(args))
    release.verify_native(context,capture)
    assert len(calls)==24
    assert calls[0]==(context['entries'][rows[0]['source_cell_id']],Path(rows[0]['capture_root']),context['deck'],rows[0])
    def reject(*args):raise ValueError('actual native annotation mismatch')
    monkeypatch.setattr(release.unit_provenance,'verify_capture',reject)
    with pytest.raises(ValueError):release.verify_native(context,capture)


@pytest.mark.parametrize('kind',['scenario','planned_meta','audit','unit_review','scale_review','collision','park','foreign_raw','pre_registration'])
def test_native_tamper_or_foreign_raw_never_qualifies(tmp_path,monkeypatch,kind):
    context,capture,rows,_=native_fixture(tmp_path,monkeypatch);raw=Path(rows[0]['capture_root'])
    if kind=='scenario':write(raw/'scenario.json',dict(changed=True))
    if kind=='planned_meta':
        meta=release.read(raw/'metadata.json');meta['spec']['family']='changed';write(raw/'metadata.json',meta)
    if kind=='audit':write(raw/'audit.json',{})
    if kind=='unit_review':
        monkeypatch.setattr(release.unit,'review',lambda *args:dict(review_reasons=['actual_geometry_changed']))
    if kind=='scale_review':
        monkeypatch.setattr(release.scale_capture,'native_size_review',lambda *args:dict(review_reasons=['actual_scale_changed']))
    if kind=='collision':
        collision=release.read(raw/'collision_history.json');collision['samples'][0]['message']['collision_objecta']=[dict(unique_id=123)]
        write(raw/'collision_history.json',collision)
    if kind=='park':write(raw.parents[1]/'shutdown_safety.json',dict(park_verified=False))
    if kind=='pre_registration':context['doc']['registered_at_unix']=2.
    if kind=='foreign_raw':
        rows[0]['capture_root']=str(tmp_path/'foreign');(tmp_path/'foreign').mkdir()
        jsonl(capture/'selected_ledger.jsonl',rows)
    with pytest.raises(ValueError):release.verify_native(context,capture)


def test_preserved_review_and_deficit_remain_blockers_not_external_approvals(tmp_path,monkeypatch):
    context,capture,rows,_=native_fixture(tmp_path,monkeypatch)
    jsonl(capture/'preserved'/'ledger.jsonl',[dict(id='failed_probe',status='review')])
    missing=rows.pop(0)['source_cell_id'];jsonl(capture/'selected_ledger.jsonl',rows)
    write(capture/'status.json',dict(stage='complete_with_deficit',requested=24,accepted_candidate=23,review=1))
    monkeypatch.setattr(release.completion,'verify_parent',lambda *args:(rows,[missing],10.))
    result,blockers=release.verify_native(context,capture)
    assert result['accepted']==23 and result['external_support_approvals_counted']==0
    assert set(blockers)>={'current_exact_smoke_native_deficit','preserved_current_smoke_review_or_failure',
        'accepted_native_rare_profile_minimum_missing'}


def input_fixture(tmp_path,monkeypatch):
    capture=tmp_path/'capture';capture.mkdir()
    config=write(tmp_path/'config.json',{})
    ledger=jsonl(capture/'selected_ledger.jsonl',[dict(fixture=True)])
    source_snapshot=write(tmp_path/'current'/'source_snapshot.json',dict(fixture=True))
    package_manifest=write(tmp_path/'package'/'manifest.json',dict(fixture=True))
    for split in ('train','val','test'):
        path=package_manifest.parent/'infos/512x1056'/('morai_base5_infos_'+split+'.pkl')
        path.parent.mkdir(parents=True,exist_ok=True)
        records=[dict(token='a',instances=[{}]),dict(token='b',instances=[{},{}])] if split=='train' else []
        path.write_bytes(pickle.dumps(dict(data_list=records)))
    cohort=dict(frames=2,source_ledger=str(ledger),source_ledger_sha256=release.sha256(ledger),
        training3_manifest_sha256=release.sha256(package_manifest),training3_package=str(package_manifest.parent),
        source_snapshot_sha256=release.sha256(source_snapshot),retained_gt=3)
    doc=dict(sensor_sha256=release.sha256(release.pins.ROOT/'configs/sensors/Sensor_competition_local.json'),cohorts=dict(current=cohort))
    report=write(tmp_path/'inputs.json',doc)
    import qualify_base5_training_loader_cpu as loader
    modes={mode:dict(image_shape=[4,3,512,1056],image_dtype='torch.uint8') for mode in ('train','val')}
    loaded=dict(schema='base5_cpu_training_loader_qualification_v1',script_sha256=release.sha256(loader.__file__),
        qualification_report=str(report),qualification_sha256=release.sha256(report),config=str(config),
        config_sha256=release.sha256(config),cuda_available=False,gpu_execution=False,training_started=False,
        cohorts=dict(current=dict(package_manifest_sha256=cohort['training3_manifest_sha256'],counts=dict(source_gt=3,train_frames=2,val_frames=2),
            frames=[dict(token='a',source_gt=1,original_split='train',modes=copy.deepcopy(modes)),
                dict(token='b',source_gt=2,original_split='train',modes=copy.deepcopy(modes))])))
    loader_report=write(tmp_path/'loader.json',loaded)
    monkeypatch.setattr(release,'verify_training_inputs',lambda *args:doc)
    return capture,report,loader_report,doc,loaded


@pytest.mark.parametrize('kind',['valid','foreign_ledger','helper','report','config','gpu','frame_count','gt_count','token',
    'foreign_token','per_frame_gt','original_split','shape','val_missing','sensor'])
def test_exact_current_native_inputs_and_actual_cpu_both_modes_are_required(tmp_path,monkeypatch,kind):
    capture,report,loader_report,doc,loaded=input_fixture(tmp_path,monkeypatch)
    current=loaded['cohorts']['current']
    if kind=='foreign_ledger':doc['cohorts']['current']['source_ledger']=str(tmp_path/'old_ledger.jsonl')
    if kind=='helper':loaded['script_sha256']='0'*64
    if kind=='report':loaded['qualification_sha256']='0'*64
    if kind=='config':loaded['config_sha256']='0'*64
    if kind=='gpu':loaded['gpu_execution']=True
    if kind=='frame_count':current['counts']['val_frames']=1
    if kind=='gt_count':current['frames'][0]['source_gt']=0
    if kind=='token':current['frames'][1]['token']='a'
    if kind=='foreign_token':current['frames'][1]['token']='foreign'
    if kind=='per_frame_gt':current['frames'][0]['source_gt'],current['frames'][1]['source_gt']=2,1
    if kind=='original_split':current['frames'][0]['original_split']='val'
    if kind=='shape':current['frames'][0]['modes']['train']['image_shape']=[4,3,512,1055]
    if kind=='val_missing':current['frames'][0]['modes'].pop('val')
    if kind=='sensor':doc['sensor_sha256']='0'*64
    write(loader_report,loaded)
    if kind=='valid':assert release.verify_inputs(report,loader_report,capture,2)['cpu_train_and_val_pipelines_verified'] is True
    else:
        with pytest.raises(ValueError):release.verify_inputs(report,loader_report,capture,2)


def test_partial_receipt_rechecks_exact_first4_and_unchanged_prior_raw(tmp_path,monkeypatch):
    capture=tmp_path/'capture';capture.mkdir();manifest=write(tmp_path/'manifest.json',{})
    rows=[dict(source_cell_id='slot_'+str(i),id=str(i),status='accepted_candidate',
        capture_root=str(capture/str(i)),scenario_sha256=str(i)) for i in range(4)]
    jsonl(capture/'selected_ledger.jsonl',rows)
    initial=dict(raw_files={'r':dict(sha256='a')},event_logs={'e':'b'},cumulative_elapsed_sec=1.)
    current=copy.deepcopy(initial);current['cumulative_elapsed_sec']=2.
    before=dict(manifest_sha256=release.sha256(manifest),capture_root=str(capture),preserved_selected=4,
        selected_rows=rows,before=initial)
    path=write(tmp_path/'partial.json',before)
    receipt=dict(passed=True,source_partial_sha256=release.sha256(path),manifest_sha256=release.sha256(manifest),
        preserved_selected=4,prior_raw_unchanged=True,before=initial,after=current)
    result=write(tmp_path/'partial_resume.json',receipt)
    monkeypatch.setattr(release,'snapshot',lambda root:copy.deepcopy(current))
    ids=[row['source_cell_id'] for row in rows]
    assert release.verify_partial(path,result,manifest,capture,ids)['passed'] is True
    current['raw_files']['r']['sha256']='changed';receipt['after']=current;write(result,receipt)
    with pytest.raises(ValueError):release.verify_partial(path,result,manifest,capture,ids)


@pytest.mark.parametrize('blockers',[[],['current_exact_smoke_native_deficit']])
def test_release_requires_all_gates_and_keeps_live_completion_false(tmp_path,monkeypatch,blockers):
    context,capture,_,_=native_fixture(tmp_path,monkeypatch)
    full=write(tmp_path/'full'/'manifest.json',dict(count=5000))
    pin=write(tmp_path/'pin.json',{});checks={str(pin):release.sha256(pin)}
    context.update(full=full,runtime=checks,policy={})
    context['doc'].update(input_hashes=checks,qualification_evidence_hashes=checks,support_evidence_hashes=checks)
    monkeypatch.setattr(release.smoke,'registration_context',lambda *args:context)
    monkeypatch.setattr(release,'verify_native',lambda *args:(dict(accepted=24),list(blockers)))
    monkeypatch.setattr(release,'verify_stability',lambda *args:(dict(evidence_hashes=checks),[]))
    monkeypatch.setattr(release,'verify_resume',lambda *args:dict(requested=24))
    monkeypatch.setattr(release,'verify_partial',lambda *args:dict(passed=True,preserved_selected=4))
    monkeypatch.setattr(release,'verify_inputs',lambda *args:dict(smoke_frames=24,evidence_hashes=checks))
    monkeypatch.setattr(release,'verify_cpu_regression',lambda *args:dict(unique_passed=1648))
    monkeypatch.setattr(release.audit_base5_scenarios,'audit',lambda *args:dict(status='OFFLINE_PASS',errors=[],scenes=5000,unique_payloads=5000))
    monkeypatch.setattr(release.scale,'verify_frozen_policy',lambda *args:None)
    evidence=write(tmp_path/'evidence.json',{})
    args=SimpleNamespace(manifest=full,smoke_registration=evidence,capture_root=capture,completed_resume_receipt=evidence,
        partial_snapshot=evidence,partial_resume_receipt=evidence,training_input_report=evidence,loader_report=evidence,
        pytest_xml=[evidence],capture_output=tmp_path/'new_full_capture',output=tmp_path/'release',
        stability_capture_root=tmp_path/'paired_capture',stability_ledger=[evidence],stability_cause_review=None,
        stability_session_manifest=[],cpu_regression_receipt=evidence)
    original=release.snapshot(capture)
    receipt=release.read(release.freeze(args))
    assert receipt['bulk_ready']==(not blockers) and receipt['bulk_blockers']==blockers
    assert receipt['completion_live_qualified'] is False and receipt['bulk_started'] is False
    assert receipt['separate_support_evidence']['native_smoke_approvals_counted']==0
    assert release.snapshot(capture)==original
    with pytest.raises(ValueError):release.freeze(args)
    for gate in ('verify_resume','verify_partial','verify_inputs','verify_cpu_regression','verify_stability'):
        def reject(*args):raise ValueError('missing '+gate)
        with monkeypatch.context() as scope:
            scope.setattr(release,gate,reject);args.output=tmp_path/('missing_'+gate)
            with pytest.raises(ValueError):release.freeze(args)
            assert not args.output.exists()


def cpu_fixture(tmp_path,monkeypatch,*,returncode=0,change=None):
    runtime=write(tmp_path/'runtime_pin.json',{})
    test_source=write(tmp_path/'test_pin.json',{})
    monkeypatch.setattr(smoke,'runtime_hashes',lambda:{str(runtime):smoke.sha256(runtime)})
    monkeypatch.setattr(smoke,'test_hashes',lambda:{str(test_source):smoke.sha256(test_source)})
    calls=[]
    def run(command,**kwargs):
        calls.append((command,kwargs))
        path=Path(command[-1].split('=',1)[1]);root=ET.Element('testsuites')
        suite=ET.SubElement(root,'testsuite',tests='1001',failures='0',errors='0',skipped='0')
        for index in range(1001):ET.SubElement(suite,'testcase',classname='current_cpu',name='test_'+str(index))
        ET.ElementTree(root).write(path)
        if change=='runtime':write(runtime,dict(changed=True))
        if change=='tests':write(test_source,dict(changed=True))
        return SimpleNamespace(returncode=returncode)
    monkeypatch.setattr(smoke.subprocess,'run',run)
    return calls


def test_cpu_fixed_relative_node_is_actually_deselected_by_pytest(tmp_path,monkeypatch):
    # pytest reports relative node IDs even when the selected tests directory
    # and --rootdir are absolute. Exercise the real command, not a mocked run.
    node=smoke.CPU_EXCLUSIONS[0]['node'];relative_file,excluded_name=node.split('::')
    source=tmp_path/relative_file;source.parent.mkdir(parents=True)
    source.write_text('def test_kept_cpu_control():\n    assert True\n\n'
        'def '+excluded_name+'():\n    raise AssertionError("fixed excluded node was executed")\n')
    monkeypatch.setattr(smoke.pins,'ROOT',tmp_path)
    xml=tmp_path/'actual_pytest.xml';command=smoke.cpu_command(xml)
    assert '--deselect='+node in command
    environment=dict(smoke.os.environ);environment.update(smoke.CPU_ENVIRONMENT)
    result=smoke.subprocess.run(command,cwd=tmp_path,env=environment,capture_output=True,text=True,check=False)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    cases=list(ET.parse(xml).getroot().iter('testcase'))
    assert [case.get('name') for case in cases]==['test_kept_cpu_control']
    assert all(case.find('failure') is None and case.find('error') is None for case in cases)
    assert '1 deselected' in result.stdout


@pytest.mark.parametrize('kind',['valid','process_failure','runtime_race','test_race'])
def test_cpu_receipt_requires_actual_full_suite_and_unchanged_pre_post_source(tmp_path,monkeypatch,kind):
    calls=cpu_fixture(tmp_path,monkeypatch,returncode=1 if kind=='process_failure' else 0,
        change='runtime' if kind=='runtime_race' else 'tests' if kind=='test_race' else None)
    output=tmp_path/'cpu'
    if kind=='valid':
        receipt=smoke.read(smoke.qualify_cpu(output))
        assert receipt['test_results']['unique_passed']==1001
        assert receipt['runtime_hashes_before']==receipt['runtime_hashes_after']
        assert str(smoke.pins.ROOT/'dataset/morai_od/tests') in receipt['command']
        assert receipt['fixed_exclusions']==smoke.CPU_EXCLUSIONS
        assert len([value for value in receipt['command'] if value.startswith('--deselect=')])==1
        assert receipt['capture_started'] is False and receipt['training_started'] is False
    else:
        with pytest.raises(ValueError):smoke.qualify_cpu(output)
        assert not (output/'receipt.json').exists()
    assert len(calls)==1 and calls[0][1]['cwd']==smoke.pins.ROOT and calls[0][1]['check'] is False
    assert {key:calls[0][1]['env'][key] for key in smoke.CPU_ENVIRONMENT}==smoke.CPU_ENVIRONMENT


@pytest.mark.parametrize('kind',['valid','old_runtime','helper','test_source','xml','summary','command','capture','exclusions','pytest_env'])
def test_release_rejects_old_or_unbound_junit_even_if_1000_cases_pass(tmp_path,monkeypatch,kind):
    cpu_fixture(tmp_path,monkeypatch)
    path=smoke.qualify_cpu(tmp_path/'cpu');doc=smoke.read(path)
    if kind=='old_runtime':doc['runtime_hashes_before']={}
    if kind=='helper':doc['helper_sha256']='0'*64
    if kind=='test_source':doc['test_hashes']={}
    if kind=='xml':doc['pytest_receipt_hashes']={}
    if kind=='summary':doc['test_results']['unique_passed']=9999
    if kind=='command':doc['command'].insert(4,'-k=old_tests')
    if kind=='capture':doc['capture_started']=True
    if kind=='exclusions':doc['fixed_exclusions']=[]
    if kind=='pytest_env':doc['pytest_environment_controls']['PYTEST_ADDOPTS']='-k old_tests'
    write(path,doc)
    if kind=='valid':assert release.verify_cpu_regression(path,[path.parent/'tests.xml'],smoke.runtime_hashes())['unique_passed']==1001
    else:
        with pytest.raises(ValueError):release.verify_cpu_regression(path,[path.parent/'tests.xml'],smoke.runtime_hashes())
