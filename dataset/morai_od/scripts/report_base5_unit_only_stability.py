#!/usr/bin/env python3
"""원형-only 원본·native 치수·반복 안정성을 읽기 전용으로 보고한다.

독립 64개, 동일 payload 반복 16개, 재시도와 별도 cold control의 분모를
합치지 않는다. 기존 QA/원본 GT를 수정하거나 이 보고서로 bulk를 승인하지 않는다.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import tempfile

import numpy as np

from audit_morai_lidar_gt_visibility import parse_instances
from base5_capture_safety import atomic_json, prepare_session, sha256
import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_native_box_corners as native
import base5_unit_capture_provenance as provenance
import base5_unit_geometry_contract as unit
import prepare_base5_unit_only_stability as preparation

SCHEMA = 'base5_unit_only_stability_report_v1'
REGISTRATION = 'unit_only_stability_registration'
DIMENSION_REASONS = {'unit_native_bbox_size_mismatch', 'unit_native_component_contract_changed',
    'scaled_native_bbox_size_mismatch'}


def require(value, message):
    if not value:
        raise ValueError(message)


def read_bytes(path, inventory, expected=None):
    path = Path(path).absolute()
    require(path.is_file() and not path.is_symlink(), '고정된 일반 파일이 필요하다: '+str(path))
    before = path.stat();data = path.read_bytes();after = path.stat()
    stamp = lambda s:(s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
    digest = hashlib.sha256(data).hexdigest()
    require(stamp(before) == stamp(after) and (expected is None or digest == expected)
        and (str(path) not in inventory or inventory[str(path)] == digest),
        '읽는 동안 원본 또는 SHA가 변경됐다: '+str(path))
    inventory[str(path)] = digest
    return data


def read(path, inventory, expected=None):
    return json.loads(read_bytes(path, inventory, expected))


def runtime_hashes():
    return pins.collect_runtime(entrypoints=pins.ENTRYPOINTS+
        (Path(preparation.__file__).name,))


def unit_policy(deck, inventory):
    reference = deck.get('config', {}).get(unit.KEY)
    value = unit.load(reference)
    require(len(value['assets']) == 19, '원형 QA 19자산 참조가 필요하다')
    for ref in [reference, value['catalog_reference'], *value['sources']]:
        read_bytes(ref['path'], inventory, ref['sha256'])
    catalog = {a['data_id']:a for a in read(value['catalog_reference']['path'], inventory)['assets']}
    physical = {a['name'] for a in catalog.values() if a.get('semantic_class') == 'obstacle'}
    require(physical == set(value['assets']), 'QA19와 catalog의 물리 장애물 집합이 다르다')
    return value, catalog


def bind_unit(scene, metadata, catalog, *, legacy_control=False):
    # 기존 control의 비활성 변환 계보는 보존한다. 실제 plan/assignment/probe나
    # raw nonunit을 허용하는 예외가 아니다.
    checked = {k:v for k,v in metadata.items() if not (legacy_control and k == 'asset_scale_preparation')}
    preparation.reject_scale_metadata(checked)
    require(metadata.get('runtime_actors') and all(a.get('motion') in ('static', 'standing')
        for a in metadata['runtime_actors']), '모든 actor의 명시적 정적 motion이 필요하다')
    scales = preparation.raw_unit_scales(scene)
    preparation.bind_actors(scene, metadata, catalog)
    # 원본 container 안의 순서와 planned actor 순서도 보존한다.
    for container in preparation.CONTAINERS:
        require([a['UNIQUEID'] for a in scene.get(container, [])] ==
            [a['unique_id'] for a in metadata['runtime_actors'] if a['source_list'] == container],
            'container별 actor 순서가 다르다')
    return dict(scales)


def load_context(manifest, registration=None, *, control=False):
    manifest = Path(manifest).resolve(strict=True);inventory = {}
    deck = read(manifest, inventory)
    require(deck.get('count') == len(deck.get('scenes', [])) and deck['count'] > 0,
        'manifest 개수와 실제 entry가 다르다')
    policy, catalog = unit_policy(deck, inventory)
    entries = {entry['id']:entry for entry in deck['scenes']}
    require(len(entries) == deck['count'], '중복된 scene ID')
    metadata, scale_counts = {}, Counter()
    for entry in deck['scenes']:
        scene = read(entry['scenario'], inventory, entry['scenario_sha256'])
        meta = read(entry['metadata'], inventory)
        require(meta.get(unit.KEY) == deck['config'][unit.KEY], '계획 원형 참조가 config와 다르다')
        scale_counts.update(bind_unit(scene, meta, catalog, legacy_control=control));metadata[entry['id']] = meta
    if control:
        require('all_physical_unit_registration' in deck and REGISTRATION not in deck,
            '별도 control은 기존 all_physical_unit_registration 계약이어야 한다')
        for path, digest in deck['all_physical_unit_registration'].get('source_files', {}).items():
            read_bytes(path, inventory, digest)
        return dict(manifest=manifest, deck=deck, entries=entries, metadata=metadata, policy=policy,
            inventory=inventory, registered_at=None, modern=False, scale_counts=dict(scale_counts))
    registration = Path(registration or manifest.with_name('registration.json')).resolve(strict=True)
    doc = read(registration, inventory)
    require(doc.get('schema') == preparation.SCHEMA and doc.get('policy') == preparation.POLICY
        and doc.get('manifest') == str(manifest) and doc.get('manifest_sha256') == sha256(manifest)
        and deck.get(REGISTRATION) == {k:v for k,v in doc.items() if k not in ('manifest', 'manifest_sha256')},
        '원형-only 사전 등록과 manifest 요약이 다르다')
    require(deck['count'] == doc.get('count') == 80 and doc.get('independent_frames') == 64
        and doc.get('repeated_frames') == 16 and len(doc.get('selected', [])) == 80
        and doc.get('unit_reference') == deck['config'][unit.KEY]
        and doc.get('capture_started') is False and doc.get('raw_modified') is False
        and doc.get('bulk_ready') is False and doc.get('training_started') is False
        and doc.get('native_outcomes_used_for_selection') is False,
        '80장 진단용 사전 등록 계약이 다르다')
    registered = doc.get('registered_at_unix')
    require(type(registered) in (int, float) and math.isfinite(registered) and registered > 0,
        '명시적 등록 시각이 필요하다')
    require(doc.get('runtime_hashes') == runtime_hashes(), '등록 뒤 runtime이 변경됐다')
    for path, digest in doc['runtime_hashes'].items():
        read_bytes(path, inventory, digest)
    for path, digest in doc['input_hashes'].items():
        read_bytes(path, inventory, digest)
    source = read(doc['source_manifest'], inventory, doc['source_manifest_sha256'])
    source_entries = {entry['id']:entry for entry in source['scenes']}
    require(source.get('count') == len(source_entries) == len(source['scenes']) == 5000,
        '고정된 원형 full5K source가 필요하다')
    independent, repeated, groups, records = {}, [], set(), []
    for index, (entry, row) in enumerate(zip(deck['scenes'], doc['selected'])):
        meta = metadata[entry['id']];tag = meta.get(preparation.KEY, {})
        expected = dict(id=entry['id'], scenario_sha256=entry['scenario_sha256'],
            metadata_sha256=sha256(entry['metadata']), scenario=str(Path(entry['scenario']).resolve()),
            metadata=str(Path(entry['metadata']).resolve()), **tag)
        require(row == expected and tag.get('sequence_ordinal') == index
            and tag.get('native_outcomes_used_for_selection') is False,
            '등록된 metadata/선정 순서가 변경됐다')
        original = source_entries.get(tag.get('source_id'))
        require(original and str(Path(original['metadata']).resolve()) == tag.get('source_metadata_path')
            and Path(original['scenario']).resolve() == Path(entry['scenario']).resolve()
            and original['scenario_sha256'] == tag.get('source_scenario_sha256') == entry['scenario_sha256'],
            '원형 source payload 계보가 다르다')
        original_meta = read(tag['source_metadata_path'], inventory, tag['source_metadata_sha256'])
        require({k:v for k,v in meta.items() if k not in (unit.KEY, preparation.KEY)} == original_meta
            and layout.digest(layout.geometry(meta)) == tag.get('geometry_sha256'),
            '원본 metadata 또는 실제 geometry fingerprint가 다르다')
        if index < 64:
            require(tag.get('phase') == 'independent' and tag.get('independent_ordinal') == index
                and tag.get('repeat_of_id') is None and tag.get('repeat_ordinal') == 0
                and tag.get('payload_reused_exact') is False
                and tag.get('independent_group_id') not in groups, '64개 독립 배치 등록이 다르다')
            independent[entry['id']] = entry;groups.add(tag['independent_group_id'])
            records.append(dict(id=entry['id'], meta=meta, scenario_sha256=entry['scenario_sha256']))
        else:
            parent = independent.get(tag.get('repeat_of_id'))
            require(parent and tag.get('phase') == 'repeat' and tag.get('repeat_ordinal') == 1
                and tag.get('payload_reused_exact') is True and parent['scenario_sha256'] == entry['scenario_sha256']
                and {k:v for k,v in meta.items() if k != preparation.KEY} ==
                    {k:v for k,v in metadata[parent['id']].items() if k != preparation.KEY}
                and tag['independent_group_id'] == metadata[parent['id']][preparation.KEY]['independent_group_id']
                and tag['independent_ordinal'] == metadata[parent['id']][preparation.KEY]['independent_ordinal'],
                '반복은 첫 64개와 동일 payload·metadata·actor여야 한다')
            repeated.append(parent['id'])
    actual_groups, _, _ = layout.groups(records, layout.configuration())
    require(len(set(actual_groups.values())) == 64 and len(set(repeated)) == len(repeated) == 16,
        '실제 독립 geometry 64개 또는 서로 다른 반복 16개가 아니다')
    return dict(manifest=manifest, deck=deck, entries=entries, metadata=metadata, policy=policy,
        inventory=inventory, registered_at=registered, modern=True, registration=doc,
        scale_counts=dict(scale_counts))


def verify_session(path, context, inventory, session_manifests):
    session = path.parent
    contract = read(session/'session_contract.json', inventory)
    park = read(session/'shutdown_safety.json', inventory)
    require(park.get('park_verified') is True, '종료 Park가 확인되지 않았다')
    if (session/'status.json').exists():
        read(session/'status.json', inventory)
    options = contract.get('runner_options', {});selected = options.get('selected_scene_ids')
    require(isinstance(selected, list) and selected and len(set(selected)) == len(selected)
        and set(selected) <= set(context['entries']) and type(options.get('shared_ros_batch')) is bool
        and set(options) <= {'shared_ros_batch', 'selected_scene_ids', 'optimized_adapter'},
        '명시적인 현재 collector 선택 계약이 필요하다')
    if 'optimized_adapter' in options:
        adapter = options['optimized_adapter'];profile = adapter.get('profile')
        require(profile in ('baseline', 'static-ready') and adapter == dict(profile=profile,
            adapter_sha256=sha256(Path(__file__).with_name('run_base5_capture_optimized.py')),
            source_sha256=sha256(Path(__file__).with_name('run_base5_capture.py')),
            policy='static conditional setup delay only; native gates'), 'collector adapter가 변경됐다')
    manifests = [context['manifest'], *session_manifests]
    matches = [m for m in manifests if sha256(m) == contract.get('manifest_sha256')]
    require(matches, '실제 사용한 session manifest의 고정 SHA가 필요하다')
    manifest = matches[0];deck = read(manifest, inventory)
    require({k:v for k,v in deck.items() if k not in ('count', 'scenes')} ==
        {k:v for k,v in context['deck'].items() if k not in ('count', 'scenes')}
        and deck.get('count') == len(deck.get('scenes', []))
        and all(entry == context['entries'].get(entry['id']) for entry in deck['scenes'])
        and selected == [entry['id'] for entry in deck['scenes'] if entry['id'] in selected],
        'session이 등록된 원형 manifest 또는 정확한 부분집합이 아니다')
    with tempfile.TemporaryDirectory(prefix='base5-unit-session-') as directory:
        output = Path(directory)/'contract'
        prepare_session(output, manifest,
            pins.ROOT/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
            pins.ROOT/'configs/sensors/Sensor_competition_local.json', runner_options=options)
        require(json.loads((output/'session_contract.json').read_text()) == contract,
            '현재 native 안전/runtime/session 계약과 다르다')
    events = [json.loads(line) for line in read_bytes(session/'event_cmd_requests.jsonl', inventory).splitlines()]
    require(events and all(type(event.get('recorded_at')) in (int, float)
        and math.isfinite(event['recorded_at']) for event in events), '실제 native event 시각이 필요하다')
    start, end = min(e['recorded_at'] for e in events), max(e['recorded_at'] for e in events)
    require(context['registered_at'] is None or start >= context['registered_at'],
        '등록 이전 native 관측은 소급 재사용하지 않는다')
    return selected, options['shared_ros_batch'], start, end, {event.get('scene') for event in events}


def native_parents(entry, row, context, inventory):
    capture = Path(row['capture_root']).resolve(strict=True)
    meta = read(capture/'metadata.json', inventory)
    audit = read(capture/'audit.json', inventory)
    scene = read(capture/'scenario.json', inventory, entry['scenario_sha256'])
    expected = context['metadata'][entry['id']]
    require(row.get('audit') == audit and all(meta.get(k) == v for k,v in expected.items()),
        '실제 RAW audit 또는 계획 metadata가 다르다')
    catalog = {a['data_id']:a for a in read(context['policy']['catalog_reference']['path'], inventory)['assets']}
    bind_unit(scene, meta, catalog, legacy_control=not context['modern'])
    require(provenance.actor_identity(meta) == provenance.actor_identity(expected), 'actor 순서·정체성·motion이 다르다')
    provenance.verify_capture(entry, capture, context['deck'], row)
    require((row['status'] == 'accepted_candidate') == (audit.get('accepted_candidate') is True),
        'ledger 상태와 저장 native QA 상태가 다르다')
    annotations = list(capture.glob('LIDAR_*/*_Instance.txt'))
    require(len(annotations) == 1, '정확한 native Instance.txt 하나가 필요하다')
    annotation = annotations[0]
    read_bytes(annotation, inventory)
    companion = annotation.with_name(annotation.name[:-len('_Instance.txt')]+'_Instance_8Points.txt')
    read_bytes(companion, inventory)
    corners = native.load_matching_corners(annotation);boxes = parse_instances(annotation)
    require(len(boxes) == len(corners), 'native summary/corner component 수가 다르다')
    ordinals, by_uid = Counter(), defaultdict(list)
    components = {c['component_id']:c for c in audit['components']}
    actors = {a['unique_id']:a for a in meta['runtime_actors']}
    for box in boxes:
        uid, raw_class = box['unique_id'], box['raw_class'];key = (uid, raw_class, ordinals[(uid, raw_class)])
        ordinals[(uid, raw_class)] += 1
        component = components[f'{uid}:{raw_class}:{key[2]}']
        actor = actors[uid]
        if actor['name'] not in context['policy']['assets']:
            continue
        record = corners[key];reasons = []
        if raw_class != 'Obstacle' or record['parent_component_count'] != 1:
            by_uid[uid].append(dict(parent_uid=uid, asset=actor['name'], comparable=False,
                diagnostic_review_reasons=['unit_native_component_contract_changed']))
            continue
        try:
            native.derive_geometry(box, record)
        except native.BoxCornerContractError as error:
            # 이 오류는 모든 corner shape/center/RPY 검사를 통과한 뒤의
            # summary-size 불일치다. GT를 바꾸지 않고 측정치 차이만 보고한다.
            require(str(error) == 'Native corner dimensions differ; explicit bound variant policy required', str(error))
            reasons.append('unit_native_corners_summary_mismatch')
        vertices = np.asarray(record['corners_lidar_m'], float)
        center = np.asarray(box['center'], float)
        local = (vertices-center) @ native._rotation(box['rotation_rpy'])
        measured = np.ptp(local, axis=0)
        summary = np.asarray(box['size'], float);saved = np.asarray(component['size_m'], float)
        reference = np.asarray(context['policy']['assets'][actor['name']]['native_size_lwh_m'], float)
        delta = np.abs(measured-reference);tolerance = np.maximum(.03, .03*reference)
        if not np.allclose(measured, saved, atol=.001, rtol=.001):
            reasons.append('unit_native_corners_audit_mismatch')
        if np.any(delta > tolerance):
            reasons.append('unit_native_corners_size_mismatch')
        by_uid[uid].append(dict(parent_uid=uid, asset=actor['name'], data_id=actor['data_id'], comparable=True,
            observed=True, native_component_count=1,
            component_id=component['component_id'], native_corner_size_lwh_m=measured.tolist(),
            native_summary_size_lwh_m=summary.tolist(), saved_audit_size_lwh_m=saved.tolist(),
            expected_unit_size_lwh_m=reference.tolist(), absolute_axis_error_m=delta.tolist(),
            audit_corner_absolute_axis_error_m=np.abs(saved-measured).tolist(),
            matched_unit_reference=bool(np.all(delta <= tolerance)), diagnostic_review_reasons=reasons))
    result = []
    for actor in meta['runtime_actors']:
        if actor['name'] not in context['policy']['assets']:
            continue
        values = by_uid[actor['unique_id']]
        if len(values) != 1 or not values[0]['comparable']:
            result.append(dict(parent_uid=actor['unique_id'], asset=actor['name'], comparable=False,
                observed=bool(values), native_component_count=len(values),
                not_compared_reason='native_component_missing' if not values else 'not_single_physical_obstacle_component',
                diagnostic_review_reasons=['unit_native_component_contract_changed']))
        else:
            result.append(values[0])
    return result


def axes(values):
    if not values:
        return dict(count=0, minimum=None, maximum=None, median=None, p95=None)
    values = np.asarray(values, float)
    return dict(count=len(values), minimum=values.min(0).tolist(), maximum=values.max(0).tolist(),
        median=np.median(values, axis=0).tolist(), p95=np.percentile(values, 95, axis=0).tolist())


def asset_statistics(context, initial, phase=None):
    """반복 관측을 독립 표본으로 합산하지 않도록 phase별 분모를 제공한다."""
    if phase is not None:
        initial = [row for row in initial if row['phase'] == phase]
    assets = {}
    for name, reference in sorted(context['policy']['assets'].items()):
        parents = [(row,p) for row in initial for p in row['parents'] if p['asset'] == name]
        comparable = [(row,p) for row,p in parents if p['comparable']]
        groups = {row['independent_group_id'] for row,p in comparable if row['phase'] == 'independent'}
        planned = [(ident,actor) for ident,meta in context['metadata'].items()
            for actor in meta['runtime_actors'] if actor['name'] == name
            and (phase is None or meta.get(preparation.KEY, {}).get('phase', 'separate_control') == phase)]
        observed_groups = {row['independent_group_id'] for row,p in parents
            if row['phase'] == 'independent' and p['observed']}
        assets[name] = dict(expected_unit_size_lwh_m=reference['native_size_lwh_m'],
            planned_parent_occurrences=len(planned), expected_initial_parent_occurrences=len(parents),
            observed_initial_parent_occurrences=sum(p['observed'] for _,p in parents),
            compared_initial_parent_occurrences=len(comparable),
            not_compared_initial_parent_occurrences=len(parents)-len(comparable),
            missing_native_parent_occurrences=sum(not p['observed'] and row['status'] != 'failed' for row,p in parents),
            failed_attempt_parent_occurrences=sum(row['status'] == 'failed' for row,p in parents),
            unobserved_registered_parent_occurrences=len(planned)-len(parents),
            initial_parent_observations=sum(p['observed'] for _,p in parents), comparable_parent_observations=len(comparable),
            independent_geometry_groups=len(groups), observed_independent_geometry_groups=len(observed_groups),
            unmatched_or_missing_components=len(parents)-len(comparable),
            native_corner_size_lwh_m=axes([p['native_corner_size_lwh_m'] for _,p in comparable]),
            native_summary_size_lwh_m=axes([p['native_summary_size_lwh_m'] for _,p in comparable]),
            absolute_axis_error_m=axes([p['absolute_axis_error_m'] for _,p in comparable]),
            audit_corner_absolute_axis_error_m=axes([p['audit_corner_absolute_axis_error_m'] for _,p in comparable]),
            native_reference_mismatch_parents=sum(not p['matched_unit_reference'] for _,p in comparable))
    return assets


def repeat_parent_inventory(row, expected):
    parents = {p['parent_uid']:p for p in row['parents']} if row else {}
    observed = {uid for uid,p in parents.items() if p['observed']}
    compared = {uid for uid,p in parents.items() if p['comparable']}
    return dict(initial_attempt_available=row is not None,
        initial_native_available=bool(row and row['native_geometry_available']),
        expected_parent_uids=sorted(expected), expected_parents=len(expected),
        observed_parent_uids=sorted(observed), observed_parents=len(observed),
        compared_parent_uids=sorted(compared), compared_parents=len(compared),
        missing_parent_uids=sorted(expected-observed), not_compared_parent_uids=sorted(expected-compared),
        all_expected_compared=bool(expected) and expected == compared)


def summarize(context, observations):
    initial = [row for row in observations if row['attempt_ordinal'] == 0]
    assets = asset_statistics(context, initial)
    phases = ('independent', 'repeat') if context['modern'] else ('separate_control',)
    assets_by_phase = {phase:asset_statistics(context, initial, phase) for phase in phases}
    observed = {row['id']:row for row in initial};repeats = []
    if context['modern']:
        for entry in context['deck']['scenes'][64:]:
            ident = entry['id'];parent_id = context['metadata'][ident][preparation.KEY]['repeat_of_id']
            before, after = observed.get(parent_id), observed.get(ident)
            expected = {actor['unique_id'] for actor in context['metadata'][parent_id]['runtime_actors']
                if actor['name'] in context['policy']['assets']}
            before_inventory = repeat_parent_inventory(before, expected)
            after_inventory = repeat_parent_inventory(after, expected)
            left = {p['parent_uid']:p for p in before['parents'] if p['comparable']} if before else {}
            right = {p['parent_uid']:p for p in after['parents'] if p['comparable']} if after else {}
            deltas = [dict(parent_uid=uid, asset=left[uid]['asset'],
                absolute_corner_size_delta_m=np.abs(np.asarray(right[uid]['native_corner_size_lwh_m'])-
                    left[uid]['native_corner_size_lwh_m']).tolist(),
                absolute_summary_size_delta_m=np.abs(np.asarray(right[uid]['native_summary_size_lwh_m'])-
                    left[uid]['native_summary_size_lwh_m']).tolist()) for uid in sorted(left.keys() & right.keys())]
            repeats.append(dict(id=ident, repeat_of_id=parent_id,
                expected_parent_uids=sorted(expected), expected_parents=len(expected),
                before=before_inventory, after=after_inventory,
                both_initial_native_observed=before_inventory['initial_native_available'] and after_inventory['initial_native_available'],
                all_expected_compared=before_inventory['all_expected_compared'] and after_inventory['all_expected_compared'],
                unmatched_parent_uids=sorted(left.keys() ^ right.keys()), compared_parents=len(deltas), parents=deltas,
                independent_samples_added=0))
    return dict(assets=assets, assets_by_phase=assets_by_phase,
        asset_statistics_basis='assets는 최초 independent/repeat 관측을 합친 기술 통계다. 독립 표본 추론에는 assets_by_phase.independent만 사용한다.',
        repeat_comparisons=repeats,
        independent_initial_scenes=sum(row['phase'] == 'independent' for row in initial),
        repeat_initial_scenes=sum(row['phase'] == 'repeat' for row in initial),
        initial_status_counts=dict(Counter(row['status'] for row in initial)),
        initial_qa_review_reason_counts=dict(Counter(reason for row in initial for reason in row['qa_review_reasons'])),
        initial_dimension_review_reason_counts=dict(Counter(reason for row in initial for reason in row['dimension_review_reasons'])),
        initial_dimension_review_scenes=sum(bool(row['dimension_review_reasons']) for row in initial),
        initial_other_qa_review_scenes=sum(bool(set(row['qa_review_reasons'])-DIMENSION_REASONS) for row in initial),
        retry_attempts=sum(row['attempt_ordinal'] > 0 for row in observations),
        retry_status_counts=dict(Counter(row['status'] for row in observations if row['attempt_ordinal'] > 0)),
        unobserved_registered_ids=sorted(set(context['entries'])-set(observed)),
        initial_observed_scenes=len(initial), initial_attempted_scenes=len(initial),
        initial_native_available_scenes=sum(row['native_geometry_available'] for row in initial),
        initial_native_missing_or_failed_scenes=sum(not row['native_geometry_available'] for row in initial),
        total_attempts=len(observations),
        denominator_contract='최초 scene, 독립 layout, repeat, parent, 재시도를 별도 집계한다. review의 native도 치수 통계에 포함한다.')


def analyze(context, ledgers, output, session_manifests=()):
    inventory = dict(context['inventory']);paths = [Path(p).resolve(strict=True) for p in ledgers]
    require(paths and len(paths) == len(set(paths)) and all(p.name == 'ledger.jsonl' for p in paths),
        'selected ledger가 아닌 중복 없는 실제 raw ledger.jsonl 목록이 필요하다')
    observations = [];counts = Counter();roots = set();previous_end = -math.inf
    for path in paths:
        require(output != path.parent and path.parent not in output.parents, '보고서는 RAW session 밖에 저장한다')
        selected, shared, start, end, event_scenes = verify_session(path, context, inventory, session_manifests)
        require(start >= previous_end, 'raw session ledger를 실제 시간순으로 제공해야 한다')
        previous_end = end;payload = read_bytes(path, inventory)
        require(not payload or payload.endswith(b'\n'), '중간에 잘린 raw ledger는 수정하지 않고 거부한다')
        for line in payload.splitlines():
            row = json.loads(line);ident = row['id'];entry = context['entries'].get(ident)
            require(entry and ident in selected and row.get('scenario_sha256') == entry['scenario_sha256']
                and row.get('status') in ('accepted_candidate', 'review', 'failed'), '외부/변경된 raw 시도')
            require(ident in event_scenes, '실제 시도 scene에 대응하는 등록 이후 native event가 없다')
            ordinal = counts[ident];counts[ident] += 1;parents = []
            if row['status'] != 'failed':
                root = Path(row['capture_root']).resolve(strict=True);root.relative_to(path.parent/'captures')
                require(root not in roots, '같은 RAW capture를 재시도나 다른 ledger로 중복 합산할 수 없다')
                roots.add(root)
                if shared:
                    require(row.get('between_scene_park', {}).get('park_verified') is True,
                        '공유 session의 장면 간 Park 증거가 필요하다')
                parents = native_parents(entry, row, context, inventory)
            else:
                parents = [dict(parent_uid=actor['unique_id'], asset=actor['name'], observed=False,
                    comparable=False, native_component_count=0, not_compared_reason='failed_attempt_no_native_geometry',
                    diagnostic_review_reasons=[]) for actor in context['metadata'][ident]['runtime_actors']
                    if actor['name'] in context['policy']['assets']]
            if row.get('failure_evidence'):
                read_bytes(row['failure_evidence'], inventory, row['failure_evidence_sha256'])
            tag = context['metadata'][ident].get(preparation.KEY, {})
            qa = row.get('audit', {}).get('review_reasons', [])
            dimensional = (set(qa) & DIMENSION_REASONS) | {reason for p in parents for reason in p['diagnostic_review_reasons']}
            observations.append(dict(id=ident, attempt_ordinal=ordinal, status=row['status'], source_ledger=str(path),
                scenario_sha256=row['scenario_sha256'], capture_root=row.get('capture_root'),
                independent_group_id=tag.get('independent_group_id', row['scenario_sha256']),
                phase=tag.get('phase', 'separate_control'), parents=parents,
                qa_review_reasons=qa, dimension_review_reasons=sorted(dimensional),
                saved_unit_review=row.get('audit', {}).get('native_unit_geometry_review'),
                error=row.get('error'), failure_kind=row.get('failure_kind'),
                native_geometry_available=row['status'] != 'failed'))
    if context['modern']:
        observed = {row['id'] for row in observations}
        require([row['id'] for row in observations if row['attempt_ordinal'] == 0] ==
            [entry['id'] for entry in context['deck']['scenes'] if entry['id'] in observed],
            '최초 관측 순서가 등록된 64+16 순서와 다르다')
    pins.verify_hashes(inventory, '보고서 입력 전후 불변')
    result = summarize(context, observations)
    result.update(manifest=str(context['manifest']), observations=observations, input_hashes=inventory,
        registered_at_verified=context['modern'], modern_registration=context['modern'],
        raw_scale_counts=context['scale_counts'], terminal_park_verified=True)
    return result


def report(manifest, ledgers, output, *, registration=None, control_cohorts=(), session_manifests=()):
    output = Path(output).resolve();require(not output.exists(), '새 보고서 경로가 필요하다')
    reporter_inventory = {}
    read_bytes(__file__, reporter_inventory)
    context = load_context(manifest, registration)
    require(output != context['manifest'].parent and context['manifest'].parent not in output.parents,
        '보고서는 source manifest 밖에 저장한다')
    manifests = [Path(path).resolve(strict=True) for path in session_manifests]
    main = analyze(context, ledgers, output, manifests)
    controls = []
    for control_manifest, ledger in control_cohorts:
        control = load_context(control_manifest, control=True)
        require(output != control['manifest'].parent and control['manifest'].parent not in output.parents,
            '보고서는 control source 밖에 저장한다')
        controls.append(analyze(control, [ledger], output, manifests))
    for cohort in [main, *controls]:
        pins.verify_hashes(cohort['input_hashes'], '최종 보고서 source/RAW 불변')
    pins.verify_hashes(reporter_inventory, '보고서 코드 전후 불변')
    payload = dict(schema=SCHEMA, helper_sha256=reporter_inventory[str(Path(__file__).absolute())],
        main=main, separate_controls=controls,
        bulk_ready=False, bulk_approval_claimed=False, training_started=False, raw_or_audit_modified=False,
        native_gt_modified=False, completion_live_qualified=False,
        limitations=['core 80장/독립 64개와 cold control은 합산하지 않는다.',
            '치수 mismatch와 가시성·지면 등 다른 review는 별도 축이며 서로 겹칠 수 있다.',
            '8Points extent는 읽기 전용 진단치다. scenario scale/참조값을 GT에 곱하거나 저장 audit를 고치지 않는다.',
            '기존 control은 새 80장 사전 등록 시각 gate를 통과한 것으로 승격하지 않는다.'])
    atomic_json(output, payload)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--registration', type=Path)
    parser.add_argument('--ledger', type=Path, action='append', required=True)
    parser.add_argument('--control-cohort', type=Path, nargs=2, action='append', default=[], metavar=('MANIFEST', 'LEDGER'))
    parser.add_argument('--session-manifest', type=Path, action='append', default=[])
    args = parser.parse_args()
    print(report(args.manifest, args.ledger, args.output, registration=args.registration,
        control_cohorts=args.control_cohort, session_manifests=args.session_manifest))
