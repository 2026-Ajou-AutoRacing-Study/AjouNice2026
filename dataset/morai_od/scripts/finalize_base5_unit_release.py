#!/usr/bin/env python3
"""새 원형 exact smoke·재개·입력·전체 CPU 회귀만으로 release를 동결한다.

확대 paired gate는 적용하지 않으며 과거 unit 안정성은 이력이다.
이 도구는 native/GT를 수정하거나 본수집/자동 보충을 실행하지 않는다.
"""
import argparse
from collections import Counter
import copy
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

import audit_base5_scenarios
from audit_morai_lidar_gt_visibility import parse_instances
import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_native_box_corners as corners
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
import finalize_base5_final_bounded_release as common
import prepare_base5_unit_release_smoke as smoke
from verify_base5_completed_resume import snapshot

SCHEMA = 'base5_obstacle_support_release_v1'
QUALIFICATION_SCHEMA = 'base5_unit_release_qualification_v1'
POLICY = 'unit-only-exact-source-native-qualified-v157'
COMPOUND_REFERENCE_COUNTS = dict(BabyStroller1=3,BabyStroller2=3,BabyStroller3=3,Ped_Pet1=2)
read = common.read
require = common.require
verify_partial = common.verify_partial
verify_inputs = common.verify_inputs
verify_cpu_regression = common.verify_cpu_regression
verify_resume = common.verify_resume


def verify_compound_corner(box, record, component):
    """복합 성분도 원문 8점이 summary/audit의 full-RPY 상자인지 대조한다."""
    vertices = np.asarray(record['corners_lidar_m'],float)
    center = np.asarray(box['center'],float);size = np.asarray(box['size'],float);rpy = np.asarray(box['rotation_rpy'],float)
    require(vertices.shape == (8,3) and np.isfinite(vertices).all() and np.all(size > 0), '유한한 복합 8점이 필요하다')
    angle_error = np.abs((np.asarray(record['rotation_rpy'])-rpy+np.pi)%(2*np.pi)-np.pi)
    require(np.max(angle_error) <= corners.ROTATION_TOLERANCE_RAD
        and np.linalg.norm(vertices.mean(0)-center) <= corners.CENTER_TOLERANCE_M,
        '복합 성분의 원문 8점 중심/RPY가 다르다')
    local = (vertices-center) @ corners._rotation(rpy)
    extent = np.ptp(local,axis=0);signs = local >= 0
    expected = np.where(signs,extent/2,-extent/2)
    require(np.all(extent > corners.CORNER_TOLERANCE_M*2)
        and len({tuple(row) for row in signs.tolist()}) == 8
        and np.all(np.abs(local-expected) <= np.maximum(corners.CORNER_TOLERANCE_M,extent*1e-4)),
        '복합 성분의 8점이 실제 대칭 full-RPY 상자를 이루지 않는다')
    require(np.allclose(extent,size,atol=corners.SIZE_ABSOLUTE_TOLERANCE_M,rtol=corners.SIZE_RELATIVE_TOLERANCE),
        '복합 성분의 원문 8점과 summary 치수가 다르다')
    for saved,actual in ((component['size_m'],extent),(component['center_lidar_m'],center),(component['rotation_rpy'],rpy)):
        require(np.allclose(saved,actual,atol=.001,rtol=.001), '복합 성분의 원문 8점과 저장 audit가 다르다')


def verify_unit_corners(raw, meta, audit, reference):
    """단일 물리 parent의 원문 8점을 RAM에서 검사한다. GT 수정은 없다."""
    if not any(a['name'] in reference['assets'] or a['name'] in smoke.COMPOSITES for a in meta['runtime_actors']):
        return []
    annotations = list(raw.glob('LIDAR_*/*_Instance.txt'))
    require(len(annotations) == 1, '원문 native annotation 하나가 필요하다')
    annotation = annotations[0];records = corners.load_matching_corners(annotation)
    boxes = parse_instances(annotation);ordinals = Counter();by_uid = {}
    actors = {a['unique_id']:a for a in meta['runtime_actors']}
    components = {c['component_id']:c for c in audit['components']}
    for box in boxes:
        pair = (box['unique_id'],box['raw_class']);key = pair+(ordinals[pair],);ordinals[pair] += 1
        by_uid.setdefault(pair[0],[]).append((box,records[key]))
        if actors[pair[0]]['name'] in smoke.COMPOSITES:
            verify_compound_corner(box,records[key],components[f'{key[0]}:{key[1]}:{key[2]}'])
    result = []
    for actor in meta['runtime_actors']:
        if actor['name'] not in reference['assets']:continue
        matches = by_uid.get(actor['unique_id'],[])
        require(len(matches) == 1, '원형 물리 parent의 native component가 하나가 아니다')
        box,record = matches[0];derived = corners.derive_geometry(box,record)
        measured = np.asarray(derived['evidence']['corner_extent_local_m'])
        components = [c for c in audit['components'] if c['parent_uid'] == actor['unique_id']]
        require(len(components) == 1 and np.allclose(measured,components[0]['size_m'],atol=.001,rtol=.001),
            '원문 8점과 저장 audit 치수가 다르다')
        expected = np.asarray(reference['assets'][actor['name']]['native_size_lwh_m'])
        error = np.abs(measured-expected)
        require(np.all(error <= np.maximum(.03,.03*expected)), '원문 8점이 원형 QA19 치수와 다르다')
        result.append(dict(parent_uid=actor['unique_id'],asset=actor['name'],native_corner_size_lwh_m=measured.tolist(),
            expected_unit_size_lwh_m=expected.tolist(),absolute_axis_error_m=error.tolist(),
            source_files=record['source_files'],raw_or_audit_modified=False))
    return result


def verify_native(context, capture):
    # 공통 함수는 exact source/원형 QA/충돌/Park/시각/실패 보존을 담당한다.
    # 확대 profile coverage를 원형 승인으로 오해하지 않도록 명시적으로 위임하고,
    # 모든 원형 coverage는 아래에서 새 features와 실제 component로 다시 판정한다.
    adapter = dict(context,pairs=set(),doc=copy.deepcopy(context['doc']))
    adapter['doc']['coverage'] = dict(required=[],profile_minima={})
    native,blockers = common.verify_native(adapter,capture)
    reference = unit.load(context['deck']['config'][unit.KEY]);covered = set();physical = [];compounds = []
    for observation in native['observations']:
        ident = observation['id'];raw = Path(observation['capture_root']);meta = read(raw/'metadata.json');audit = read(raw/'audit.json')
        effective = layout.resolve(context['overlay'],ident,context['entries'][ident],meta)['split']
        tokens = smoke.features(meta,effective,context['unit_assets'])
        measured = verify_unit_corners(raw,meta,audit,reference)
        physical.extend(dict(scene_id=ident,**row) for row in measured)
        compound_covered = set()
        for actor in meta['runtime_actors']:
            if actor['name'] not in smoke.COMPOSITES:continue
            components = [c for c in audit['components'] if c['parent_uid'] == actor['unique_id']]
            compounds.append(dict(scene_id=ident,parent_uid=actor['unique_id'],asset=actor['name'],
                expected_historical_component_count=COMPOUND_REFERENCE_COUNTS[actor['name']],
                observed_component_count=len(components),observed_raw_class_counts=dict(Counter(c['raw_class'] for c in components)),
                historical_count_matched=len(components) == COMPOUND_REFERENCE_COUNTS[actor['name']],
                native_component_corners_verified=True,
                components=[dict(component_id=c['component_id'],raw_class=c['raw_class']) for c in components]))
            if len(components) > 1:compound_covered.add('compound/'+actor['name'])
        tokens -= {token for token in tokens if token.startswith('compound/') and token not in compound_covered}
        covered.update(tokens);observation['coverage'] = sorted(tokens)
    if not set(context['doc']['coverage']['required']) <= covered:
        blockers.append('unit_exact_native_required_coverage_missing')
    native.update(covered=sorted(covered),physical_corner_observations=physical,compound_parent_components=compounds,
        physical_assets_observed=sorted({r['asset'] for r in physical}),scale_paired_gate_applied=False,
        coverage_contract='원형 19종·거리·정적 미션·split·이동·실제 복합 component를 별도 검증한다.',
        historical_unit_approvals_counted=0)
    native.pop('accepted_profile_frames',None)
    return native,sorted(set(blockers))


def freeze(args):
    output = Path(args.output).resolve();capture = Path(args.capture_root).resolve(strict=True)
    require(not output.exists() and output != capture and capture not in output.parents, 'RAW 밖의 새 release 경로가 필요하다')
    before = snapshot(capture);context = smoke.registration_context(args.manifest,args.smoke_registration)
    require(output != context['full'].parent and context['full'].parent not in output.parents, 'source 안에 release를 쓸 수 없다')
    native,blockers = verify_native(context,capture)
    completed = verify_resume(args.completed_resume_receipt,capture,context['subset_path'])
    require(completed.get('requested') == len(context['selected']), '완료 무촬영 재개 대상이 다르다')
    partial = verify_partial(args.partial_snapshot,args.partial_resume_receipt,context['subset_path'],capture,
        [row['id'] for row in context['doc']['selected']])
    inputs = verify_inputs(args.training_input_report,args.loader_report,capture,native['accepted'])
    tests = verify_cpu_regression(args.cpu_regression_receipt,args.pytest_xml,context['runtime'])
    audit = audit_base5_scenarios.audit(context['full'])
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes') == audit.get('unique_payloads') == 5000, '현재 원형 full5K 감사가 실패했다')
    require(snapshot(capture) == before, '검증 중 원본 RAW가 바뀌었다')
    hashes = dict(context['doc']['input_hashes']);hashes.update(context['doc']['support_evidence_hashes'])
    hashes.update(inputs['evidence_hashes'])
    for path in [args.manifest,args.smoke_registration,args.completed_resume_receipt,args.partial_snapshot,
                 args.partial_resume_receipt,args.training_input_report,args.loader_report,args.cpu_regression_receipt,
                 *args.pytest_xml,__file__,context['subset_path']]:
        hashes[str(Path(path).resolve())] = sha256(path)
    for relative,record in before['raw_files'].items():hashes[str(capture/relative)] = record['sha256']
    for relative,digest in before['event_logs'].items():hashes[str(capture/relative)] = digest
    hashes[str(capture/'selected_ledger.jsonl')] = before['selected_sha256']
    pins.verify_hashes(context['runtime']);pins.verify_hashes(hashes)
    planned = Path(args.capture_output).resolve()
    require(planned != capture and capture not in planned.parents and not planned.exists(), 'full5K 본수집은 별도의 새 경로여야 한다')
    require(planned != output and output not in planned.parents and planned not in output.parents,
        'release와 본수집 출력은 겹칠 수 없다')
    release = dict(schema=SCHEMA,qualification_schema=QUALIFICATION_SCHEMA,release_policy=POLICY,
        generated_at_utc=datetime.now(timezone.utc).isoformat(),bulk_ready=not blockers,bulk_blockers=blockers,
        bulk_started=False,training_started=False,completion_live_qualified=False,
        completion_qualification_scope='CPU 복구 계약만 검증하며 live 자동 보충은 승인하지 않는다.',
        qualified_environments=['sunny13'],full_manifest=str(context['full']),full_manifest_sha256=sha256(context['full']),
        policy=context['deck']['config']['obstacle_support_policy'],runtime_hashes=context['runtime'],input_hashes=hashes,
        environments=dict(sunny13=dict(manifest=str(context['full']),manifest_sha256=sha256(context['full']),capture_output=str(planned))),
        operational_hours_range=[8,12],operational_storage_budget_gib=45.,smoke=native,training_inputs=inputs,cpu_tests=tests,
        partial_resume=partial,completed_resume=dict(receipt_sha256=sha256(args.completed_resume_receipt),additional_native_captures=0),
        full_collector_preflight=dict(scenes=5000,raw_unit_scale_admission=True,layout_source_pins=True,
            native_unit_reference=True,materialized_shadow_registry=True,
            materialized_shadow_scenes=context['motion_preparation_inventory']['scene_counts']['independent_terrain_materialized'],
            all_source_moving_paths_prepared=context['motion_preparation_inventory']['all_source_moving_paths_prepared'],
            on_demand_motion_preparation_required=context['motion_preparation_inventory']['on_demand_preparation_required']['count']),
        motion_preparation_inventory=context['motion_preparation_inventory'],
        selection_eligibility=context['doc']['selection_eligibility'],
        common_verifier_adapter=dict(scale_pairs=[],scale_profile_minima={},coverage_verified_by='원형 전용 metadata/native component 계약'),
        separate_support_evidence=dict(hashes=context['doc']['support_evidence_hashes'],native_smoke_approvals_counted=0),
        scale_paired_gate_required=False,
        limitations=['현재 사전 등록 exact-source smoke만 새 승인으로 센다. 과거 89장 원형 안정성은 별도 이력이다.',
            'exact smoke는 static/editor 준비/이미 materialize된 shadow만 선정한다. 미준비 shadow22의 별도 준비 실험은 이 smoke 승인에 합산하지 않는다.',
            'materialized shadow registry는 등록된 두 장면만 뜻한다. 미준비 natural shadow는 온디맨드 준비와 엄격한 경로 검증 후에만 촬영할 수 있다.',
            'Q90 upright 3클래스 512 CPU 입력 검증은 GPU/모델/학습/AP 승인이 아니다.',
            '본수집과 live 자동 보충/--complete를 실행하거나 승인하지 않는다.'])
    atomic_json(output/'offline_audit.json',audit);atomic_json(output/'release.json',release)
    return output/'release.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','smoke-registration','capture-root','completed-resume-receipt','partial-snapshot',
                 'partial-resume-receipt','training-input-report','loader-report','cpu-regression-receipt','capture-output','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--pytest-xml',type=Path,action='append',required=True)
    print(freeze(parser.parse_args()))
