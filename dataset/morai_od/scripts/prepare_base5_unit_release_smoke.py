#!/usr/bin/env python3
"""원형 full5K의 exact-source 소량 검증을 관측 전에 사전 등록한다.

metadata만으로 32장(명시적 최대 48장)을 선정한다. 과거 원형 안정성이나
확대 paired 결과는 이번 smoke 승인 수에 합산하지 않는다. MR/ROS를 실행하지 않는다.
"""
import argparse
from collections import Counter
import copy
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import time

import audit_base5_scenarios
import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_motion_ground as motion_ground
import base5_shadow_ground_reuse as shadow
import base5_unit_geometry_contract as unit
from base5_intersection_ground import POLICY as EDITOR_MOTION_POLICY
from base5_capture_safety import atomic_json, sha256
from base5_scale_capture_contract import validate_entry
import prepare_base5_final_bounded_smoke as common
import prepare_base5_unit_only_stability as original

SCHEMA = 'base5_unit_release_smoke_registration_v1'
POLICY = 'unit-only-exact-source-bounded-release-v157'
STATUS = 'PREREGISTERED_UNIT_RELEASE_SMOKE_ONLY; no bulk or completion'
DEFAULT_TARGET = 32
MAX_FRAMES = 48
RELEASE_HELPER = 'finalize_base5_unit_release.py'
COMPOSITES = ('BabyStroller1', 'BabyStroller2', 'BabyStroller3', 'Ped_Pet1')
PHASES = common.legacy_features.PHASES
CROSSWALKS = common.legacy_features.CROSSWALKS
read = common.read
require = common.require
runtime_hashes = common.runtime_hashes
qualify_cpu = common.qualify_cpu


def motion_preparation_kind(ident, meta, shadow_ids):
    """준비 상태만 분류한다. 실제 증거 검증은 full source 검사에서 수행한다."""
    require(not meta.get('calibration_only'), '학습 full source에 calibration-only 장면이 있다')
    require(all(a.get('motion') in ('static','standing','walking','running') for a in meta['runtime_actors']),
        '알 수 없거나 누락된 source actor motion')
    actors = motion_ground.moving_actors(meta)
    if not actors:
        require(ident not in shadow_ids and not meta.get('spec',{}).get('shadow_moving'),
            '등록된 이동 shadow를 static으로 바꿀 수 없다')
        return 'static'
    policies = [a.get('motion_path',{}).get('ground_path_evidence',{}).get('policy') for a in actors]
    if all(policy == EDITOR_MOTION_POLICY for policy in policies):
        require(ident not in shadow_ids, '독립 terrain 등록 shadow를 editor 준비로 바꿀 수 없다')
        return 'editor_prepared'
    if all(policy == 'independent-terrain-sampled-v46' for policy in policies):
        require(ident in shadow_ids and meta.get(shadow.KEY), '독립 terrain 준비는 현재 materialized registry에 묶여야 한다')
        return 'independent_terrain_materialized'
    require(all(not a.get('motion_path',{}).get('ground_path_evidence') for a in actors)
        and meta.get('spec',{}).get('family') == 'shadow'
        and meta['spec'].get('shadow_moving') is True and ident not in shadow_ids and not meta.get(shadow.KEY),
        '미준비 natural shadow 외의 알 수 없는/부분 준비 moving source')
    return 'unprepared_natural_shadow'


def selection_eligibility(metadata, shadow_ids):
    """exact smoke만 이미 준비된 payload로 제한한다. full5K quota는 그대로다."""
    allowed = ('static','editor_prepared','independent_terrain_materialized')
    classified = {ident:motion_preparation_kind(ident,meta,shadow_ids) for ident,meta in metadata.items()}
    eligible = {ident for ident,kind in classified.items() if kind in allowed}
    excluded = sorted(set(metadata)-eligible)
    return eligible,dict(schema='base5_exact_source_smoke_selection_eligibility_v1',
        policy='already-prepared-training-payload-only-v1',source_scenes=len(metadata),
        allowed_preparation_states=list(allowed),eligible_count=len(eligible),
        eligible_scene_counts={kind:sum(value == kind for value in classified.values()) for kind in allowed},
        excluded_count=len(excluded),excluded_scene_counts=dict(Counter(classified[ident] for ident in excluded)),
        excluded_ids=excluded,source_quota_modified=False,native_outcomes_used_for_selection=False,
        separate_on_demand_preparation_approvals_counted=0,
        reason='미준비 natural shadow는 준비 시 payload가 바뀌므로 exact-source smoke에서만 제외한다. full5K에는 유지하며 별도 온디맨드 준비 실험으로 검증한다.')


def motion_preparation_inventory(entries, metadata, shadow_ids):
    """원형 source의 실제 이동 준비 상태다. 준비와 native 승인은 다르다."""
    require(set(entries) == set(metadata) and len(shadow_ids) == len(set(shadow_ids)) == 2,
        '전체 motion inventory와 등록된 shadow 2개가 필요하다')
    kinds = ('static','editor_prepared','independent_terrain_materialized','unprepared_natural_shadow')
    counts = dict.fromkeys(kinds,0);moving_parents = dict.fromkeys(kinds,0)
    moving_ids = {kind:[] for kind in kinds[1:]}
    for ident,meta in metadata.items():
        kind = motion_preparation_kind(ident,meta,shadow_ids)
        actors = motion_ground.moving_actors(meta)
        if kind == 'static':
            counts['static'] += 1;continue
        if kind in ('editor_prepared','independent_terrain_materialized'):
            motion_ground.validate_motion_ground(meta,read(entries[ident]['scenario']))
        counts[kind] += 1;moving_parents[kind] += len(actors);moving_ids[kind].append(ident)
    require(set(moving_ids['independent_terrain_materialized']) == set(shadow_ids), 'materialized shadow inventory가 registry와 다르다')
    require(sum(counts.values()) == len(entries), 'motion inventory가 전체 source를 분할하지 못했다')
    for values in moving_ids.values():values.sort()
    unprepared = moving_ids['unprepared_natural_shadow']
    return dict(schema='base5_full_source_motion_preparation_inventory_v1',total_scenes=len(entries),scene_counts=counts,
        moving_scenes=len(entries)-counts['static'],moving_parent_counts=moving_parents,moving_scene_ids=moving_ids,
        all_source_moving_paths_prepared=not unprepared,
        on_demand_preparation_required=dict(count=len(unprepared),scene_ids=unprepared,
            must_finish_before_training_capture=True,strict_motion_validation_required=True,
            scope='각 장면의 독립 terrain·원형 참조·실제 waypoint 증거를 새로 준비하고 검증해야 한다.'),
        native_capture_approval_claimed=False,
        limitation='editor/독립 terrain 준비는 원문 경로 증거다. static 변환·calibration 태그·split 계보는 이동 준비 완료가 아니며, 현재 native 승인도 아니다.')


def order_key(label, ident):
    return hashlib.sha256((POLICY+':'+label+':'+ident).encode()).hexdigest(), ident


def features(meta, effective_split, unit_assets):
    spec = meta['spec'];actors = meta['runtime_actors']
    result = {'training_split/'+effective_split}
    if meta.get('training_split_overlay'):result.add('pipeline/layout_overlay')
    if meta.get(unit.KEY):result.add('pipeline/unit_reference')
    if meta.get(shadow.KEY):result.add('pipeline/materialized_shadow')
    if spec.get('static_mission_extent'):
        require(spec.get('family') == 'obstacle' and spec.get('mission') is True
            and spec['static_mission_extent'].get('stratum') in ('before','intersection','after')
            and spec.get('avoidance_phase') in PHASES, '정적 미션 extent 계약이 다르다')
        result.update(('extent_stratum/'+spec['static_mission_extent']['stratum'],
            'extent_phase/'+spec['avoidance_phase'], 'extent_training_split/'+effective_split))
    for actor in actors:
        name = actor['name'];distance = actor.get('planned_center_distance_m', -1)
        if name in unit_assets:
            result.add('physical_asset/'+name)
            if 0 <= distance < 20:result.add('obstacle_range/0_20m')
            if 20 <= distance < 30:result.add('obstacle_range/20_30m')
        if name in COMPOSITES:result.add('compound/'+name)
        if (spec.get('family') != 'shadow' and actor.get('mission_crossing_id') in CROSSWALKS
                and actor['motion'] in ('walking','running')):
            result.add('moving_crosswalk/'+actor['mission_crossing_id'])
        if spec.get('family') == 'highway' and actor['source_list'] == 'vehicleList':
            for lo,hi in ((50,80),(80,100)):
                if lo <= distance < hi and actor.get('actual_ego_role') in ('front','rear'):
                    result.add(f'far/{lo}_{hi}m/'+actor['actual_ego_role'])
    return result


def select(deck, metadata, overlay, unit_assets, shadow_ids, target=DEFAULT_TARGET, maximum=MAX_FRAMES):
    require(type(target) is int and type(maximum) is int and 32 <= target <= maximum <= 48,
        '원형 smoke는 명시적인 32~48장 상한 안에 있어야 한다')
    entries = {entry['id']:entry for entry in deck['scenes']}
    require(len(entries) == len(deck['scenes']) and set(entries) == set(metadata), '중복 없는 전체 metadata가 필요하다')
    require(len(unit_assets) == 19 and len(shadow_ids) == len(set(shadow_ids)) == 2,
        '물리 19종과 materialized shadow 2개가 필요하다')
    eligible,eligibility = selection_eligibility(metadata,shadow_ids)
    effective = {ident:layout.resolve(overlay,ident,entries[ident],meta)['split'] for ident,meta in metadata.items()}
    covers = {ident:features(metadata[ident],effective[ident],unit_assets) for ident in eligible}
    required = {'physical_asset/'+name for name in unit_assets}
    required.update('compound/'+name for name in COMPOSITES)
    required.update('extent_stratum/'+v for v in ('before','intersection','after'))
    required.update('extent_phase/'+v for v in PHASES)
    required.update(('extent_training_split/train','extent_training_split/val', 'training_split/train','training_split/val',
        'pipeline/layout_overlay','pipeline/unit_reference','pipeline/materialized_shadow',
        'obstacle_range/0_20m','obstacle_range/20_30m'))
    protected = [('shadow/'+ident,ident) for ident in sorted(shadow_ids)]
    selected = [];labels = {}
    def add(ident,label):
        require(ident in eligible and ident not in labels, '준비된 보호 source가 없거나 중복됐다: '+label)
        labels[ident] = [label];selected.append(ident)
    for label,ident in protected:
        meta = metadata[ident]
        require(meta.get(shadow.KEY) and meta['spec']['family'] == 'shadow'
            and any(a['motion'] not in ('static','standing') for a in meta['runtime_actors']),
            '이미 materialize된 실제 이동 shadow가 필요하다')
        add(ident,'protected/'+label)
    controls = ['moving_crosswalk/'+crossing for crossing in CROSSWALKS]
    controls += [f'far/{lo}_{hi}m/{role}' for lo,hi in ((50,80),(80,100)) for role in ('front','rear')]
    required.update(controls)
    for token in controls:
        choices = [ident for ident in eligible if ident not in labels and token in covers[ident]]
        require(choices, '보호 metadata 구간이 없다: '+token)
        add(min(choices,key=lambda ident:order_key(token,ident)), 'protected/'+token)
    available = set().union(*covers.values());frequency = Counter(t for values in covers.values() for t in values)
    require(required <= available, '이미 준비된 exact source에 필수 coverage가 없다: '+str(sorted(required-available)))
    def missing(ids):return required-set().union(*(covers[ident] for ident in ids))
    uncovered = missing(selected)
    while uncovered:
        choices = [ident for ident in eligible if ident not in labels and covers[ident]&uncovered]
        require(choices and len(selected) < target, '명시적 장수 안에 coverage를 담을 수 없다: '+str(sorted(uncovered)))
        ident = min(choices,key=lambda ident:(-len(covers[ident]&uncovered),
            -sum((Fraction(1,frequency[t]) for t in covers[ident]&uncovered),Fraction()),order_key('cover',ident)))
        add(ident,'metadata_set_cover');uncovered = missing(selected)
    for ident in selected[8:][::-1]:
        if not missing([value for value in selected if value != ident]):
            selected.remove(ident);del labels[ident]
    set_cover_frames = len(selected)
    extras = sorted((ident for ident in eligible if ident not in labels),key=lambda ident:order_key('fill',ident))
    require(len(extras) >= target-len(selected), '고정 smoke 장수보다 source가 적다')
    for ident in extras[:target-len(selected)]:add(ident,'fixed_target_extra')
    rows = [dict(id=ident,selection_roles=labels[ident],coverage=sorted(covers[ident]),
        effective_training_split=effective[ident],scenario_sha256=entries[ident]['scenario_sha256'],
        metadata_sha256=sha256(entries[ident]['metadata']),source_entry_preserved=True) for ident in selected]
    return rows,dict(required=sorted(required),covered=sorted(set().union(*(covers[i] for i in selected))),
        frames=len(rows),target_frames=target,maximum_frames=maximum,protected_frames=8,set_cover_frames=set_cover_frames,
        coverage_passed=True,native_outcomes_used_for_selection=False,selection_eligibility=eligibility)


def inspect_source(manifest):
    manifest = Path(manifest).resolve(strict=True);deck = read(manifest)
    require(deck.get('count') == len(deck.get('scenes',[])) == 5000, '고유 원형 full5K가 필요하다')
    require(deck['config'].get('environment_tag') == 'sunny13' and 'asset_scale_policy' not in deck['config'],
        '확대 정책 없는 Sunny13 원형 source가 필요하다')
    require(deck.get('split_overlay') and deck['config'].get(unit.KEY), '최종 layout 및 원형 QA 참조가 필요하다')
    audit = layout.validate_manifest_overlay(deck);overlay = layout.read_overlay(deck['split_overlay']['path'])
    policy = unit.load(deck['config'][unit.KEY]);require(len(policy['assets']) == 19, '원형 QA19가 필요하다')
    catalog = {a['data_id']:a for a in read(policy['catalog_reference']['path'])['assets']}
    require({a['name'] for a in catalog.values() if a.get('semantic_class') == 'obstacle'} == set(policy['assets']),
        'QA19가 catalog 물리 장애물 집합과 다르다')
    reuse_pins = shadow.validate_manifest(deck)
    require(reuse_pins and shadow.KEY in deck['config'], '현재 shadow materialization 등록이 필요하다')
    registry = read(deck['config'][shadow.KEY]['registration']['path'])
    shadow_ids = [row['source_id'] for row in registry['entries']]
    entries = {};metadata = {};scale_counts = Counter()
    for entry in deck['scenes']:
        ident = entry['id'];require(ident not in entries, '중복 source ID')
        require(sha256(entry['scenario']) == entry['scenario_sha256'], '원형 scenario SHA가 다르다')
        scene = read(entry['scenario']);meta = read(entry['metadata'])
        require(entry['split'] == meta['split'] and meta.get(unit.KEY) == deck['config'][unit.KEY],
            'source capture split 또는 원형 참조가 다르다')
        original.reject_scale_metadata(meta);scale_counts.update(original.raw_unit_scales(scene))
        original.bind_actors(scene,meta,catalog);validate_entry(entry,deck)
        for container in original.CONTAINERS:
            require([a['UNIQUEID'] for a in scene.get(container,[])] ==
                [a['unique_id'] for a in meta['runtime_actors'] if a['source_list'] == container], 'actor 순서가 다르다')
        entries[ident] = entry;metadata[ident] = meta
    inputs = pins.collect_inputs(manifest);inputs.update(reuse_pins)
    inputs.update(overlay.get('source_hashes',{}));inputs.update(overlay.get('finalization_source_hashes',{}))
    inputs[str(Path(deck['split_overlay']['path']).resolve())] = sha256(deck['split_overlay']['path'])
    pins.verify_hashes(inputs)
    return dict(deck=deck,entries=entries,metadata=metadata,overlay=overlay,unit_assets=set(policy['assets']),
        shadow_ids=shadow_ids,input_hashes=inputs,layout_audit=audit,raw_scale_counts=dict(scale_counts),
        motion_preparation_inventory=motion_preparation_inventory(entries,metadata,shadow_ids))


def subset(context, rows):
    result = copy.deepcopy(context['deck'])
    result.update(count=len(rows),scenes=[copy.deepcopy(context['entries'][r['id']]) for r in rows],status=STATUS,bulk_ready=False)
    return result


def prepare(manifest, output, target=DEFAULT_TARGET, maximum=MAX_FRAMES, support_evidence=()):
    manifest = Path(manifest).resolve(strict=True);output = Path(output).resolve()
    import generate_base5_independent as gen
    runtime_dir = gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    require(not output.exists() and output != manifest.parent and manifest.parent not in output.parents
        and output != runtime_dir and runtime_dir not in output.parents, 'source/runtime 밖의 새 출력이 필요하다')
    runtime = runtime_hashes();context = inspect_source(manifest)
    rows,coverage = select(context['deck'],context['metadata'],context['overlay'],context['unit_assets'],
        context['shadow_ids'],target,maximum)
    audit = audit_base5_scenarios.audit(manifest)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes') == audit.get('unique_payloads') == 5000, '현재 full5K offline 감사가 실패했다')
    evidence = {str(Path(path).resolve(strict=True)):sha256(path) for path in support_evidence}
    atomic_json(output/'manifest.json',subset(context,rows));atomic_json(output/'source_audit.json',audit)
    doc = dict(schema=SCHEMA,policy=POLICY,helper_sha256=sha256(__file__),
        release_helper_sha256=sha256(Path(__file__).with_name(RELEASE_HELPER)),registered_at_unix=time.time(),
        native_observations_must_follow_registration=True,source_manifest=str(manifest),source_manifest_sha256=sha256(manifest),
        subset_manifest_sha256=sha256(output/'manifest.json'),source_audit_sha256=sha256(output/'source_audit.json'),
        runtime_hashes=runtime,input_hashes=context['input_hashes'],selected=rows,coverage=coverage,
        selection_eligibility=coverage['selection_eligibility'],
        raw_scale_counts=context['raw_scale_counts'],unit_reference=context['deck']['config'][unit.KEY],
        motion_preparation_inventory=context['motion_preparation_inventory'],
        native_qa_unchanged=True,live_qualified=False,bulk_started=False,training_started=False,completion_live_qualified=False,
        partial_resume_plan=dict(chunk_size=4,preserved_selected=4),support_evidence_hashes=evidence,
        support_evidence_role='과거 진단 이력만 보존하며 이번 exact smoke 승인 수에는 0장을 합산한다.',
        scale_paired_gate_required=False,source_entry_policy='최종 원형 source entry/config 그대로, 관측 후 재선정 없음')
    atomic_json(output/'registration.json',doc)
    for hashes in (runtime,context['input_hashes']):pins.verify_hashes(hashes)
    if evidence:pins.verify_hashes(evidence)
    return output/'manifest.json'


def registration_context(manifest, registration):
    manifest = Path(manifest).resolve(strict=True);registration = Path(registration).resolve(strict=True)
    doc = read(registration)
    require(doc.get('schema') == SCHEMA and doc.get('policy') == POLICY
        and doc.get('helper_sha256') == sha256(__file__)
        and doc.get('release_helper_sha256') == sha256(Path(__file__).with_name(RELEASE_HELPER)),
        '현재 원형 smoke/release helper 사전 등록이 필요하다')
    require(doc.get('source_manifest') == str(manifest) and doc.get('source_manifest_sha256') == sha256(manifest),
        '사전 등록 full source가 다르다')
    require(doc.get('live_qualified') is False and doc.get('bulk_started') is False
        and doc.get('training_started') is False and doc.get('completion_live_qualified') is False
        and doc.get('native_qa_unchanged') is True and doc.get('scale_paired_gate_required') is False,
        '사전 등록은 native 승인이나 완료를 주장할 수 없다')
    registered = doc.get('registered_at_unix')
    require(type(registered) in (int,float) and math.isfinite(registered) and 0 < registered <= time.time()
        and doc.get('native_observations_must_follow_registration') is True, '관측 이전 명시적 등록 시각이 필요하다')
    runtime = runtime_hashes();require(doc.get('runtime_hashes') == runtime, '등록 이후 runtime이 바뀌었다')
    context = inspect_source(manifest)
    require(doc.get('input_hashes') == context['input_hashes'] and doc.get('raw_scale_counts') == context['raw_scale_counts']
        and doc.get('unit_reference') == context['deck']['config'][unit.KEY]
        and doc.get('motion_preparation_inventory') == context['motion_preparation_inventory'], '등록 입력/원형/이동 준비 계약이 바뀌었다')
    rows,coverage = select(context['deck'],context['metadata'],context['overlay'],context['unit_assets'],
        context['shadow_ids'],doc['coverage']['target_frames'],doc['coverage']['maximum_frames'])
    require(rows == doc.get('selected') and coverage == doc.get('coverage'), 'metadata 기반 사전 선정이 바뀌었다')
    require(doc.get('selection_eligibility') == coverage['selection_eligibility'], 'exact smoke 준비 상태별 선정 자격이 바뀌었다')
    subset_path = registration.parent/'manifest.json'
    require(sha256(subset_path) == doc.get('subset_manifest_sha256') and read(subset_path) == subset(context,rows)
        and sha256(registration.parent/'source_audit.json') == doc.get('source_audit_sha256'), '등록 subset/audit가 바뀌었다')
    require(doc.get('partial_resume_plan') == dict(chunk_size=4,preserved_selected=4), '4장 재개 계획이 바뀌었다')
    if doc['support_evidence_hashes']:pins.verify_hashes(doc['support_evidence_hashes'])
    pins.verify_hashes(runtime)
    context.update(full=manifest,registration=registration,doc=doc,runtime=runtime,subset_path=subset_path,
        subset=read(subset_path),selected={row['id']:row for row in rows})
    return context


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--target-frames',type=int,default=DEFAULT_TARGET)
    parser.add_argument('--maximum-frames',type=int,default=MAX_FRAMES)
    parser.add_argument('--support-evidence',type=Path,action='append',default=[])
    parser.add_argument('--cpu-regression-only',action='store_true')
    args = parser.parse_args()
    if args.cpu_regression_only:print(qualify_cpu(args.output))
    else:
        parser.error('--manifest가 필요하다') if args.manifest is None else None
        print(prepare(args.manifest,args.output,args.target_frames,args.maximum_frames,args.support_evidence))
