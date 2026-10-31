"""합성 native 원문과 gate fixture로 원형 release 어댑터 경계를 검사한다.

이 테스트의 합성 loader/회귀 receipt는 실제 학습 입력 승인 증거가 아니다.
"""
import copy
import itertools
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import finalize_base5_unit_release as release


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value)+'\n');return path


def jsonl(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(''.join(json.dumps(row)+'\n' for row in rows));return path


def native_files(raw,actors,*,corner_multiplier=1.):
    summaries = [];corners = [];components = []
    for uid,name,count,raw_class in actors:
        for ordinal in range(count):
            center = np.asarray([5.+ordinal,1.,2.]);size = np.asarray([1.,2.,.5]);rpy = [.02,-.03,.4]
            vertices = np.asarray(list(itertools.product((-1.,1.),repeat=3)))*size*corner_multiplier/2
            vertices = vertices @ release.corners._rotation(rpy).T+center
            summaries.append(' '.join([raw_class,*map(str,center),*map(str,rpy),*map(str,size),'5','0','0','0',str(uid)]))
            corners.append(' '.join([raw_class,*map(str,vertices.reshape(-1)),*map(str,rpy),'5','0','0','0',str(uid)]))
            components.append(dict(parent_uid=uid,asset=name,component_id=f'{uid}:{raw_class}:{ordinal}',raw_class=raw_class,
                size_m=size.tolist(),center_lidar_m=center.tolist(),rotation_rpy=rpy,data_id=100))
    annotation = raw/'LIDAR_5'/'timestamp_Instance.txt';annotation.parent.mkdir(parents=True,exist_ok=True)
    annotation.write_text('\n'.join(summaries)+'\n')
    annotation.with_name('timestamp_Instance_8Points.txt').write_text('\n'.join(corners)+'\n')
    return components


@pytest.fixture
def native(tmp_path,monkeypatch):
    capture = tmp_path/'capture';entries = {};metadata = {};rows = []
    reference = dict(fixture='QA19')
    policy = dict(assets={'physical'+str(i):dict(native_size_lwh_m=[1.,2.,.5]) for i in range(19)})
    for index in range(32):
        ident = 'slot'+str(index);name = 'physical'+str(index%19)
        actors = [dict(name=name,unique_id=8500,data_id=100,source_list='objectList',motion='static')]
        native_actors = [(8500,name,1,'Obstacle')]
        if index < 4:
            compound = release.smoke.COMPOSITES[index]
            actors.append(dict(name=compound,unique_id=8501,data_id=101,source_list='objectList',motion='static'))
            native_actors.append((8501,compound,2,'Pedestrian'))
        meta = dict(spec=dict(family='obstacle'),runtime_actors=actors,split='train',**{release.unit.KEY:reference})
        scene = write(tmp_path/'source'/ident/'scenario.json',dict(id=ident))
        meta_path = write(scene.with_name('metadata.json'),meta)
        entry = dict(id=ident,scenario=str(scene),metadata=str(meta_path),scenario_sha256=release.sha256(scene))
        raw = capture/'cells'/ident/'initial'/'captures'/ident
        write(raw/'scenario.json',dict(id=ident));write(raw/'metadata.json',meta)
        audit = dict(components=native_files(raw,native_actors),native_unit_geometry_review=dict(review_reasons=[]),
            review_reasons=[],accepted_candidate=True)
        audit['asset_scale_review'] = release.common.scale_capture.native_size_review(meta,audit);write(raw/'audit.json',audit)
        samples = [dict(t=t,message=dict(collision_object=[dict(unique_id=0)],collision_objecta=[])) for t in (0.,.5,1.)]
        write(raw/'collision_history.json',dict(samples=samples,summary=release.common.collision_window(samples,0.,1.)))
        write(raw.parents[1]/'shutdown_safety.json',dict(park_verified=True));write(raw.parents[1]/'session_contract.json',{})
        jsonl(raw.parents[1]/'event_cmd_requests.jsonl',[dict(scene=ident,recorded_at=1.)])
        rows.append(dict(id=ident,source_cell_id=ident,status='accepted_candidate',capture_root=str(raw),
            scenario_sha256=entry['scenario_sha256'],audit=audit))
        entries[ident] = entry;metadata[ident] = meta
    jsonl(capture/'selected_ledger.jsonl',rows)
    write(capture/'status.json',dict(stage='complete',requested=32,accepted_candidate=32,review=0,elapsed_sec=10.))
    subset = write(tmp_path/'subset'/'manifest.json',{})
    required = {'physical_asset/'+name for name in policy['assets']}|{'compound/'+name for name in release.smoke.COMPOSITES}
    context = dict(subset_path=subset,selected={r['id']:dict(id=r['id']) for r in rows},entries=entries,metadata=metadata,
        overlay={},unit_assets=set(policy['assets']),deck=dict(config={release.unit.KEY:reference,'obstacle_support_policy':'unit-policy'}),
        doc=dict(registered_at_unix=.5,selected=[dict(id=r['id']) for r in rows],coverage=dict(required=sorted(required))))
    monkeypatch.setattr(release.common.completion,'verify_parent',lambda *args:(release.common.training.ledger(capture/'selected_ledger.jsonl'),[],10.))
    monkeypatch.setattr(release.layout,'resolve',lambda *args:dict(split='train'))
    monkeypatch.setattr(release.common.shadow,'verify_capture',lambda *args:None)
    monkeypatch.setattr(release.unit,'load',lambda *args:copy.deepcopy(policy))
    monkeypatch.setattr(release.unit,'review',lambda *args:dict(review_reasons=[]))
    provenance_calls = []
    monkeypatch.setattr(release.common.unit_provenance,'verify_capture',lambda *args:provenance_calls.append(args))
    monkeypatch.setattr(release.common.training,'assert_native',lambda *args:None)
    return context,capture,rows,provenance_calls,policy


def test_native_reuses_exact_source_safety_and_rechecks_qa19_corners_and_compounds(native):
    context,capture,_,calls,_ = native
    result,blockers = release.verify_native(context,capture)
    assert not blockers and result['requested'] == result['accepted'] == 32
    assert len(calls) == 32 and len(result['physical_assets_observed']) == 19
    assert len(result['physical_corner_observations']) == 32 and len(result['compound_parent_components']) == 4
    assert result['historical_unit_approvals_counted'] == 0 and result['scale_paired_gate_applied'] is False
    assert 'accepted_profile_frames' not in result
    assert all(row['absolute_axis_error_m'] == pytest.approx([0.,0.,0.],abs=1e-6) for row in result['physical_corner_observations'])
    assert 'profile_minima' not in context['doc']['coverage']


@pytest.mark.parametrize('kind',['source','metadata','audit','unit_provenance','corner_extent','compound_corner_extent',
    'compound_corner_center','compound_corner_rpy','missing_corner','audit_size','reference','collision','park','time'])
def test_actual_native_or_source_contradiction_cannot_qualify(native,kind,monkeypatch):
    context,capture,rows,_,policy = native;raw = Path(rows[0]['capture_root'])
    if kind == 'source':write(raw/'scenario.json',{})
    if kind == 'metadata':write(raw/'metadata.json',{})
    if kind == 'audit':write(raw/'audit.json',{})
    if kind == 'unit_provenance':
        def reject(*args):raise ValueError('실제 native provenance가 다르다')
        monkeypatch.setattr(release.common.unit_provenance,'verify_capture',reject)
    if kind == 'corner_extent':native_files(raw,[(8500,'physical0',1,'Obstacle'),(8501,'BabyStroller1',2,'Pedestrian')],corner_multiplier=1.25)
    if kind.startswith('compound_corner_'):
        path = raw/'LIDAR_5/timestamp_Instance_8Points.txt';lines = path.read_text().splitlines()
        fields = lines[1].split();vertices = np.asarray(list(map(float,fields[1:25]))).reshape(8,3)
        if kind == 'compound_corner_extent':vertices = (vertices-vertices.mean(0))*1.25+vertices.mean(0)
        if kind == 'compound_corner_center':vertices += [1.,0.,0.]
        fields[1:25] = list(map(str,vertices.reshape(-1)))
        if kind == 'compound_corner_rpy':fields[27] = '1.5'
        lines[1] = ' '.join(fields);path.write_text('\n'.join(lines)+'\n')
    if kind == 'missing_corner':(raw/'LIDAR_5/timestamp_Instance_8Points.txt').unlink()
    if kind == 'audit_size':
        audit = release.read(raw/'audit.json');audit['components'][0]['size_m'][0] = 2.
        write(raw/'audit.json',audit);rows[0]['audit'] = audit;jsonl(capture/'selected_ledger.jsonl',rows)
    if kind == 'reference':policy['assets']['physical0']['native_size_lwh_m'] = [2.,2.,.5]
    if kind == 'collision':
        collision = release.read(raw/'collision_history.json');collision['samples'][0]['message']['collision_objecta'] = [dict(unique_id=5)]
        write(raw/'collision_history.json',collision)
    if kind == 'park':write(raw.parents[1]/'shutdown_safety.json',dict(park_verified=False))
    if kind == 'time':context['doc']['registered_at_unix'] = 2.
    with pytest.raises((ValueError,KeyError,release.corners.BoxCornerContractError)):release.verify_native(context,capture)


def test_single_observed_compound_component_is_coverage_blocker_without_gt_or_status_repair(native):
    context,capture,rows,_,_ = native;raw = Path(rows[0]['capture_root'])
    audit = release.read(raw/'audit.json');audit['components'] = native_files(raw,[(8500,'physical0',1,'Obstacle'),(8501,'BabyStroller1',1,'Pedestrian')]);write(raw/'audit.json',audit)
    rows[0]['audit'] = audit;jsonl(capture/'selected_ledger.jsonl',rows)
    saved = (raw/'audit.json').read_bytes()
    result,blockers = release.verify_native(context,capture)
    assert 'unit_exact_native_required_coverage_missing' in blockers
    assert result['accepted'] == 32 and result['compound_parent_components'][0]['components'][0]['raw_class'] == 'Pedestrian'
    assert (raw/'audit.json').read_bytes() == saved and rows[0]['status'] == 'accepted_candidate'


def test_one_single_component_parent_does_not_erase_same_asset_other_parent_coverage(native):
    context,capture,rows,_,_ = native;raw = Path(rows[0]['capture_root'])
    meta = release.read(raw/'metadata.json');meta['runtime_actors'].append(dict(name='BabyStroller1',unique_id=8502,
        data_id=101,source_list='objectList',motion='static'))
    context['metadata'][rows[0]['id']] = copy.deepcopy(meta);write(raw/'metadata.json',meta)
    audit = release.read(raw/'audit.json');audit['components'] = native_files(raw,
        [(8500,'physical0',1,'Obstacle'),(8501,'BabyStroller1',3,'Pedestrian'),(8502,'BabyStroller1',1,'Pedestrian')])
    write(raw/'audit.json',audit);rows[0]['audit'] = audit;jsonl(capture/'selected_ledger.jsonl',rows)
    result,blockers = release.verify_native(context,capture)
    assert not blockers and 'compound/BabyStroller1' in result['covered']
    parents = [row for row in result['compound_parent_components'] if row['asset'] == 'BabyStroller1']
    assert [p['observed_component_count'] for p in parents] == [3,1]
    assert [p['historical_count_matched'] for p in parents] == [True,False]
    assert all(p['expected_historical_component_count'] == 3 and p['native_component_corners_verified'] for p in parents)


def test_preserved_failed_attempt_is_not_cured_by_complete_selected_smoke(native):
    context,capture,_,_,_ = native
    jsonl(capture/'preserved'/'ledger.jsonl',[dict(id='old_failed',status='failed')])
    result,blockers = release.verify_native(context,capture)
    assert result['accepted'] == 32 and 'preserved_current_smoke_review_or_failure' in blockers


@pytest.fixture
def frozen(native,tmp_path,monkeypatch):
    context,capture,_,_,_ = native
    full = write(tmp_path/'full'/'manifest.json',dict(count=5000));pin = write(tmp_path/'pin.json',{})
    checks = {str(pin):release.sha256(pin)}
    context.update(full=full,runtime=checks)
    context['motion_preparation_inventory'] = dict(scene_counts=dict(static=4822,editor_prepared=154,
        independent_terrain_materialized=2,unprepared_natural_shadow=22),all_source_moving_paths_prepared=False,
        on_demand_preparation_required=dict(count=22,scene_ids=['unprepared_'+str(i) for i in range(22)]))
    context['doc'].update(input_hashes=checks,support_evidence_hashes=checks,
        selection_eligibility=dict(eligible_count=4978,excluded_count=22,
            excluded_ids=context['motion_preparation_inventory']['on_demand_preparation_required']['scene_ids'],
            source_quota_modified=False,separate_on_demand_preparation_approvals_counted=0))
    monkeypatch.setattr(release.smoke,'registration_context',lambda *args:context)
    monkeypatch.setattr(release,'verify_native',lambda *args:(dict(accepted=32),[]))
    monkeypatch.setattr(release,'verify_resume',lambda *args:dict(requested=32))
    monkeypatch.setattr(release,'verify_partial',lambda *args:dict(passed=True,preserved_selected=4))
    monkeypatch.setattr(release,'verify_inputs',lambda *args:dict(smoke_frames=32,evidence_hashes=checks))
    monkeypatch.setattr(release,'verify_cpu_regression',lambda *args:dict(unique_passed=1900))
    monkeypatch.setattr(release.audit_base5_scenarios,'audit',lambda *args:dict(status='OFFLINE_PASS',errors=[],scenes=5000,unique_payloads=5000))
    evidence = write(tmp_path/'evidence.json',{})
    args = SimpleNamespace(manifest=full,smoke_registration=evidence,capture_root=capture,completed_resume_receipt=evidence,
        partial_snapshot=evidence,partial_resume_receipt=evidence,training_input_report=evidence,loader_report=evidence,
        cpu_regression_receipt=evidence,pytest_xml=[evidence],capture_output=tmp_path/'planned_full5k',output=tmp_path/'release')
    return args,context,pin


def test_release_never_inherits_old_approval_or_live_completion(frozen):
    args,_,_ = frozen;before = release.snapshot(args.capture_root)
    result = release.read(release.freeze(args))
    assert result['bulk_ready'] is True and not result['bulk_started'] and not result['training_started']
    assert result['completion_live_qualified'] is False and result['scale_paired_gate_required'] is False
    assert result['separate_support_evidence']['native_smoke_approvals_counted'] == 0
    assert 'paired_stability' not in result and result['qualification_schema'] == release.QUALIFICATION_SCHEMA
    assert result['motion_preparation_inventory']['scene_counts']['unprepared_natural_shadow'] == 22
    assert result['selection_eligibility']['excluded_count'] == 22
    assert result['selection_eligibility']['eligible_count'] == 4978
    assert result['selection_eligibility']['source_quota_modified'] is False
    assert result['selection_eligibility']['separate_on_demand_preparation_approvals_counted'] == 0
    assert result['selection_eligibility']['excluded_ids'] == result['motion_preparation_inventory']['on_demand_preparation_required']['scene_ids']
    assert result['full_collector_preflight']['materialized_shadow_scenes'] == 2
    assert result['full_collector_preflight']['on_demand_motion_preparation_required'] == 22
    assert result['full_collector_preflight']['all_source_moving_paths_prepared'] is False
    assert release.snapshot(args.capture_root) == before
    with pytest.raises(ValueError):release.freeze(args)


@pytest.mark.parametrize('gate',['verify_native','verify_resume','verify_partial','verify_inputs','verify_cpu_regression'])
def test_every_current_exact_gate_is_mandatory(frozen,gate,monkeypatch):
    args,_,_ = frozen
    def reject(*a):raise ValueError('필수 증거가 없다: '+gate)
    monkeypatch.setattr(release,gate,reject)
    with pytest.raises(ValueError):release.freeze(args)
    assert not args.output.exists()


def test_native_blocker_writes_only_not_ready_receipt(frozen,monkeypatch):
    args,_,_ = frozen
    monkeypatch.setattr(release,'verify_native',lambda *args:(dict(accepted=32),['unit_exact_native_required_coverage_missing']))
    result = release.read(release.freeze(args))
    assert result['bulk_ready'] is False and result['bulk_blockers'] == ['unit_exact_native_required_coverage_missing']


@pytest.mark.parametrize('kind',['stale_pin','existing_capture','overlapping_output','raw_changed','offline_failed','resume_other_count'])
def test_release_cannot_weaken_source_output_or_resume_contract(frozen,kind,monkeypatch):
    args,_,pin = frozen
    if kind == 'stale_pin':write(pin,dict(changed=True))
    if kind == 'existing_capture':args.capture_output.mkdir()
    if kind == 'overlapping_output':args.capture_output = args.output/'raw'
    if kind == 'raw_changed':
        original = release.snapshot;calls = []
        def changing(path):
            value = original(path);calls.append(1)
            if len(calls) > 1:value['selected_sha256'] = '0'*64
            return value
        monkeypatch.setattr(release,'snapshot',changing)
    if kind == 'offline_failed':monkeypatch.setattr(release.audit_base5_scenarios,'audit',lambda *a:dict(status='FAIL'))
    if kind == 'resume_other_count':monkeypatch.setattr(release,'verify_resume',lambda *a:dict(requested=24))
    with pytest.raises(ValueError):release.freeze(args)
    assert not args.output.exists()


def test_same_strict_input_partial_and_actual_cpu_receipts_are_reused_without_translation():
    assert release.verify_inputs is release.common.verify_inputs
    assert release.verify_partial is release.common.verify_partial
    assert release.verify_cpu_regression is release.common.verify_cpu_regression
    assert release.verify_resume is release.common.verify_resume


def test_background_does_not_claim_physical_corner_comparison_or_require_fake_components(tmp_path):
    result = release.verify_unit_corners(tmp_path,dict(runtime_actors=[]),dict(components=[]),dict(assets={'physical':{}}))
    assert result == []
