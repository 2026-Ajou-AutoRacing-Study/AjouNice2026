"""원형 collector는 고정하고 마지막 QA 경계의 opt-in 동작만 검사한다."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_capture_static_attitude as runner


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def baseline(extra=()):
    return dict(components=[dict(component_id='8501:Obstacle:0',size_m=[1,2,3])],
        review_reasons=['static_ground_severe_below',*extra],accepted_candidate=False,
        ground_support=dict(actors=[dict(uid=8501,verified=False,reason='actor_annotation_deep_below_ground',
            minimum_annotation_ground_gap_m=-6.,plane=dict(coefficients=[0,0,6]))]),
        motion_capture=dict(required=False,review_reasons=[]),
        capture_safety=dict(collision=dict(observed=True,no_collision_verified=True)))


def observation(audit):
    return dict(attitude_qualified=True,classification_changed=True,
        original_review_reasons=list(audit['review_reasons']),
        candidate_review_reasons=[r for r in audit['review_reasons'] if r!='static_ground_severe_below'],
        actors=[dict(uid=8501,eligible=True,pedestrian_extension_opt_in=False)])


def contract(enabled=True,pedestrians=False):
    return dict(enabled=enabled,allow_static_pedestrians=pedestrians,map_source_sha256={})


def test_existing_frozen_ast_has_one_post_telemetry_final_hook_without_importing_ros():
    source=runner.SOURCE;before=source.read_bytes()
    for profile in ('baseline','static-ready'):
        code=runner.compiled_source(profile,runner.sha256(source))
        assert code.co_filename==str(source)
    assert source.read_bytes()==before


@pytest.mark.parametrize('change',['hash','duplicate','missing','ordering','options','delay'])
def test_collector_source_and_ast_structure_changes_fail_closed(tmp_path,change):
    text=runner.SOURCE.read_text()
    if change=='duplicate':text=text.replace("atomic_json(dest/'audit.json',audit)",
        "atomic_json(dest/'audit.json',audit);atomic_json(dest/'audit.json',audit)")
    if change=='missing':text=text.replace("atomic_json(dest/'audit.json',audit)","atomic_json(dest/'other.json',audit)")
    if change=='ordering':text=text.replace("atomic_json(dest/'metadata.json',meta);atomic_json(dest/'audit.json',audit)",
        "atomic_json(dest/'audit.json',audit);atomic_json(dest/'metadata.json',meta)")
    if change=='options':text=text.replace('runner_options=dict(', 'different_options=dict(')
    if change=='delay':text=text.replace('time.sleep(.7)','time.sleep(.71)')
    source=tmp_path/'source.py';source.write_text(text)
    expected='0'*64 if change=='hash' else runner.sha256(source)
    with pytest.raises(ValueError):runner.compiled_source('static-ready',expected,source)


def test_only_supported_ground_classification_changes_other_qa_and_geometry_are_preserved():
    base=baseline(['capture_ego_drift','physical_collision_during_capture_window'])
    original=copy.deepcopy(base);result=runner.adjudicated_audit(base,observation(base),contract())
    assert base==original
    assert result['review_reasons']==['capture_ego_drift','physical_collision_during_capture_window']
    assert not result['accepted_candidate']
    assert result['components']==base['components'] and result['ground_support']==base['ground_support']
    assert result['capture_safety']==base['capture_safety'] and result['motion_capture']==base['motion_capture']
    assert not result['static_surface_attitude_adjudication']['ground_contact_verified']


def test_all_actors_with_severe_below_need_independent_evidence():
    base=baseline();base['ground_support']['actors'].append(dict(uid=8502,verified=False,
        reason='actor_annotation_deep_below_ground',minimum_annotation_ground_gap_m=-2.))
    with pytest.raises(ValueError,match='Every severe-below'):
        runner.adjudicated_audit(base,observation(base),contract())


@pytest.mark.parametrize('change',['attitude','extra_reason_removed','base_mismatch','no_original_severe','pedestrian'])
def test_decision_cannot_waive_unrelated_qa_or_expand_scope(change):
    base=baseline(['missing_runtime_actor']);obs=observation(base)
    if change=='attitude':obs['attitude_qualified']=False
    if change=='extra_reason_removed':obs['candidate_review_reasons']=[]
    if change=='base_mismatch':obs['original_review_reasons']=[]
    if change=='no_original_severe':base['review_reasons']=['missing_runtime_actor']
    if change=='pedestrian':obs['actors'][0]['pedestrian_extension_opt_in']=True
    with pytest.raises(ValueError):runner.adjudicated_audit(base,obs,contract())


def test_disabled_or_unchanged_audit_is_byte_equivalent_qa_without_added_labels():
    base=baseline()
    assert runner.adjudicated_audit(base,observation(base),contract(enabled=False))==base
    assert runner.adjudicated_audit(base,dict(classification_changed=False),contract())==base
    accepted=dict(components=[],ground_support={},review_reasons=[],accepted_candidate=True)
    assert runner.adjudicated_audit(accepted,dict(classification_changed=False),contract())==accepted


@pytest.fixture
def new_capture(tmp_path,monkeypatch):
    root=tmp_path/'new';meta=dict(runtime='fixture')
    write(root/'metadata.json',meta);write(root/'telemetry.json',dict(stopped=True))
    write(root/'attempt.json',dict(stage='capture_requested'))
    write(root/'native.json',dict(immutable=True))
    base=baseline();obs=observation(base);policy=contract()
    monkeypatch.setattr(runner,'validate_contract',lambda *args:None)
    monkeypatch.setattr(runner.candidate,'analyze',lambda *args,**kwargs:copy.deepcopy(obs))
    return root,meta,base,obs,policy


def test_hook_runs_only_at_new_final_boundary_preserving_base_audit_and_immutable_native(new_capture):
    root,meta,base,_,policy=new_capture
    original=copy.deepcopy(base);native=(root/'native.json').read_bytes()
    result=runner.final_audit_hook(root,meta,base,policy)
    assert base==original and (root/'native.json').read_bytes()==native
    assert runner.read(root/runner.BASE_AUDIT)==base and not (root/'audit.json').exists()
    assert result['accepted_candidate'] and result['review_reasons']==[]
    # 원형 collector가 이 다음에 저장하는 audit와 운영상태 변경을 모사한다.
    write(root/'audit.json',result);write(root/'attempt.json',dict(stage='complete'))
    verified=runner.verify_capture(root,policy)
    assert verified['verified'] and verified['classification_changed'] and not verified['raw_modified']


@pytest.mark.parametrize('change',['existing_audit','existing_receipt','metadata','telemetry'])
def test_hook_refuses_old_or_incomplete_capture(new_capture,change):
    root,meta,base,_,policy=new_capture
    if change=='existing_audit':write(root/'audit.json',base)
    if change=='existing_receipt':write(root/runner.RECEIPT,{})
    if change=='metadata':write(root/'metadata.json',dict(different=True))
    if change=='telemetry':(root/'telemetry.json').unlink()
    with pytest.raises(ValueError):runner.final_audit_hook(root,meta,base,policy)


@pytest.mark.parametrize('change',['audit','base_audit','decision','native','contract'])
def test_completed_resume_rechecks_proofs_and_all_original_qa(new_capture,change):
    root,meta,base,_,policy=new_capture
    result=runner.final_audit_hook(root,meta,base,policy);write(root/'audit.json',result)
    if change=='audit':write(root/'audit.json',dict(accepted_candidate=True,review_reasons=[]))
    if change=='base_audit':write(root/runner.BASE_AUDIT,{})
    if change=='decision':write(root/runner.DECISION,{})
    if change=='native':write(root/'native.json',dict(changed=True))
    if change=='contract':policy['allow_static_pedestrians']=True
    with pytest.raises(ValueError):runner.verify_capture(root,policy)


def test_runtime_closure_contains_adapter_candidate_and_original_qa():
    hashes=runner.pins.collect_runtime(entrypoints=(Path(runner.__file__).name,))
    for path in (runner.__file__,runner.candidate.__file__,runner.SOURCE):
        assert hashes[str(Path(path).resolve())]==runner.sha256(path)
