#!/usr/bin/env python3
"""새 환경의 원형 5K를 CPU에서 조립한다. ROS/MORAI/수집은 실행하지 않는다.

이미 동결한 Sunny13를 reseed하지 않는다. pre-front v150 계보에서 새 배치를
생성하고 원형 v158 정책을 다시 적용한다. 기본 실행은 읽기 전용 계획이며
--run도 offline 생성만 허용한다. 결과는 native 미승인 후보이다.
"""
import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
import copy
import json
import math
from pathlib import Path
import time

import audit_base5_scenarios as scenario_audit
import base5_close_primary_observation as close
import base5_front_release_inputs as inputs
import base5_layout_split as layout
import base5_spatial_diversity as spatial
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
from dynamic_capture_metadata import normalized_sensor_calibration
import prepare_base5_close_primary_diagnostic as close_factory
import prepare_base5_far_vehicle_spread as spread
import prepare_base5_fresh_environment as fresh
import prepare_base5_layout_split as split
import prepare_base5_obstacle_front_support as front
import prepare_base5_primary_obstacle_range as primary_range
import prepare_base5_secondary_far_vehicle_candidate as far
import prepare_base5_static_mission_extent as extent
import prepare_base5_unit_geometry_contract as unit_prepare
import prepare_base5_unit_only_stability as unit_only
from run_base5_failure_aware_replenishment_fast import prewarm

ROOT = Path(__file__).resolve().parents[3]
ENVIRONMENTS = ('sunny11', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
POLICY = 'independent-unit-environment-assembly-v159'
SCHEMA = 'base5_unit_environment_offline_assembly_v1'
ADJACENT_POLICY = 'frozen-ordinary-secondary-legal-neighbor-strata-v1'
SPREAD_POLICY = 'nearest-seven-plus-guarded-source-eighth-v1'
FORBIDDEN_SOURCE_POLICIES = (
    'obstacle_front_support_policy', 'primary_obstacle_range_policy',
    'secondary_far_vehicle_policy', 'static_mission_extent', 'asset_scale_policy',
    'native_unit_geometry_reference', 'shadow_motion_ground_reuse',
    'close_primary_observation',
)


def read(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def records(manifest):
    deck = read(manifest)
    rows = []
    for entry in deck['scenes']:
        require(sha256(entry['scenario']) == entry['scenario_sha256'],
                'Source payload checksum differs: '+entry['id'])
        rows.append(dict(id=entry['id'], entry=entry, meta=read(entry['metadata']),
                         scenario_sha256=entry['scenario_sha256']))
    require(len({r['id'] for r in rows}) == len(rows), 'Duplicate source scene IDs')
    return deck, rows


def plan(source, baseline, output, environment, seed=261017, continue_far=None, continue_far_sha=None):
    """읽기만 한다. 기존 output 및 source/runtime 안의 출력을 거부한다."""
    source, baseline = Path(source).resolve(strict=True), Path(baseline).resolve(strict=True)
    raw_output = Path(output).expanduser()
    require(not raw_output.is_symlink(), 'Output cannot be a symlink')
    output = raw_output.resolve()
    require(environment in ENVIRONMENTS, 'Only the remaining five environments are allowed')
    require(type(seed) is int and 0 <= seed < 2**63, 'Explicit nonnegative integer seed required')
    runtime = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    require(not output.exists(), 'Fresh output directory required')
    for protected in (source.parent, baseline.parent, runtime):
        require(output != protected and protected not in output.parents,
                'Output must be outside frozen sources and MORAI runtime')
    deck, reference = read(source), read(baseline)
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000,
            'Complete pre-front 5K source required')
    require(deck['config'].get('obstacle_support_policy', {}).get('variant') == 'richfloor',
            'Pre-front richfloor source required')
    require(not any(key in deck['config'] for key in FORBIDDEN_SOURCE_POLICIES)
            and not deck.get('split_overlay'), 'Use pre-front v150; do not reseed frozen later policies')
    require(reference.get('count') == len(reference.get('scenes', [])) == 5000
            and reference['config'].get('environment_tag') == 'sunny13'
            and reference.get('split_overlay'), 'Read-only final Sunny13 reference required')
    require('asset_scale_policy' not in reference['config'], 'Unit-only reference required')
    native = unit.load(reference['config'][unit.KEY])
    range_ref = reference['config']['primary_obstacle_range_policy']['reference']
    require(sha256(range_ref['path']) == range_ref['sha256'], 'Primary range reference changed')
    require(bool(continue_far) == bool(continue_far_sha), 'Continuation manifest and SHA must be supplied together')
    continuation = None
    if continue_far is not None:
        path = Path(continue_far).resolve(strict=True)
        require(sha256(path) == continue_far_sha, 'Continuation manifest checksum differs')
        require(output != path.parent and path.parent not in output.parents,
                'Continuation output must be outside its preserved input')
        continuation = dict(path=str(path), sha256=continue_far_sha,
                            native_qualified=False, source_capture_or_resume_performed=False)
    return dict(schema=SCHEMA, policy=POLICY, source=str(source), baseline=str(baseline),
        source_sha256=sha256(source), baseline_sha256=sha256(baseline), output=str(output),
        environment=environment, seed=seed, expected_scenes=5000,
        continue_from_unqualified_far_stage=continuation, far_spread_proposal_policy=SPREAD_POLICY,
        unit_reference_specs=copy.deepcopy(native['sources']),
        range_reference=copy.deepcopy(range_ref),
        static_extent_counts=copy.deepcopy(reference['config']['static_mission_extent']['counts']),
        stages=['fresh_geometry', 'freeze_capture_split', 'front_support', 'primary_range', 'far_secondary',
                'far_spread', 'static_extent', 'unit_review', 'close_primary',
                'layout_overlay', 'cross_environment_audit'],
        native_qualified=False, bulk_ready=False, bulk_started=False,
        simulator_access=False, remote_access=False)


def quota_key(meta, include_range=False):
    """보조 actor 수가 아닌 primary·카테고리·방향·phase 계약이다."""
    row = meta['spec']; actors = meta['runtime_actors']
    value = [row['family'], bool(row['mission']), row['role'],
        actors[0]['data_id'] if actors else None, row.get('population_mode'),
        row.get('avoidance_phase'), row.get('vehicle_close_phase'), row.get('roundabout_phase'),
        row.get('dynamic_observation_phase'), row.get('crossing_direction_sign'),
        row.get('shadow_observation_stage'),
        any(a['motion'] not in ('static', 'standing') for a in actors)]
    if include_range:
        value.append(row.get('distance_bounds'))
    return json.dumps(value, sort_keys=True)


def required_adjacent_sides(spec, ctx):
    """기존 ordinary 차량 배치가 실제 요구하는 lane 방향만 계산한다."""
    if spec.get('family') != 'other_vehicle':
        return ()
    catalog = {a['data_id']:a for pool in ctx['pools'].values() for a in pool}
    ids = spec.get('frozen_population_data_ids')
    if ids is None:
        ids = [spec.get('primary_identity')]
    required = set()
    roles = ('front', 'left', 'rear', 'right')
    for ordinal, ident in enumerate(ids):
        if catalog.get(ident, {}).get('source_list') != 'vehicleList':
            continue
        if ordinal == 0 or spec.get('population_layout') == 'clustered':
            role = spec['role']
        elif spec.get('population_layout') == 'distributed':
            role = roles[(ordinal+roles.index(spec['role'])) % len(roles)]
        else:
            # 동결되지 않은 random secondary의 방향을 미리 거짓 확정하지 않는다.
            continue
        if role in ('left', 'right'):
            required.add(role)
    return tuple(sorted(required))


def screen_adjacent_intervals(ctx, intervals, sides):
    """MGeo의 실제 lane-change flag/destination으로 허용 arc만 남긴다."""
    if not sides:
        return intervals
    cache = ctx.setdefault('_unit_environment_adjacent_intervals', {})
    key = (tuple(sides), tuple(tuple(v) for v in intervals))
    if key in cache:
        return cache[key]
    valid = []
    for start, end in intervals:
        count = max(1, math.ceil((end-start)/2.))
        for index in range(count):
            lo = start+(end-start)*index/count
            hi = start+(end-start)*(index+1)/count
            pose = spatial.gen.geo.continuous_route_pose_at_arc(ctx['route'], ctx['arc'], (lo+hi)/2)
            source = ctx['links'][pose['link_idx']]
            if not all(source.get('can_move_'+side+'_lane')
                    and source.get(side+'_lane_change_dst_link_idx') in ctx['links'] for side in sides):
                continue
            if valid and abs(valid[-1][1]-lo) < 1e-7:
                valid[-1][1] = hi
            else:
                valid.append([lo, hi])
    require(valid, 'No primary-and-secondary legal adjacent lane intervals: '+str(sides))
    cache[key] = valid
    return valid


@contextmanager
def adjacent_feasibility_adapter():
    """새 후보 조립에서만 사용한다. 기존 helper 파일과 frozen 정책은 불변이다."""
    original_assign, original_feasible = spatial.assign_plans, spatial.feasible_intervals

    def feasible(ctx, intervals, spec):
        return screen_adjacent_intervals(ctx, original_feasible(ctx, intervals, spec),
                                         required_adjacent_sides(spec, ctx))

    def assign(items, intervals, ctx=None):
        require(ctx is not None, 'Adjacent-lane screening requires the real map context')
        partitions = defaultdict(list)
        for item in items:
            partitions[required_adjacent_sides(item[1], ctx)].append(item)
        for sides, values in sorted(partitions.items()):
            original_assign(values, intervals, ctx)
            for _, spec, _ in values:
                if sides and spec.get('spatial_sampling_plan'):
                    spec['spatial_sampling_plan']['ordinary_adjacent_lane_screening'] = dict(
                        policy=ADJACENT_POLICY, required_sides=list(sides),
                        lane_direction_or_actor_quota_relaxed=False)

    spatial.assign_plans, spatial.feasible_intervals = assign, feasible
    try:
        yield
    finally:
        spatial.assign_plans, spatial.feasible_intervals = original_assign, original_feasible


def fresh_geometry(source, output, environment, seed):
    with adjacent_feasibility_adapter():
        manifest = fresh.generate(source, output, environment, seed, known_map_layout_split=True)
    normalize_environment(manifest, environment)
    return manifest


def capture_split_spec(metadata, config):
    """geometry hash로 얻은 임시 capture split만 지원된 row override로 고정한다."""
    require(layout.enabled(config), 'Known-map generation config is required')
    require(metadata.get('split') in ('train', 'val'), 'Explicit provisional capture split required')
    spec = copy.deepcopy(metadata['spec'])
    require('layout_split' not in spec, 'Capture split is already frozen')
    spec['layout_split'] = metadata['split']
    probe = dict(metadata, spec=spec)
    require(layout.provisional_split(probe, config) == metadata['split'],
            'Supported capture split override was not respected')
    return spec


def freeze_capture_splits(source, output):
    """새 pre-front source만 복사한다. meta.split 값을 사후 덮어쓰지 않는다."""
    source, output = Path(source).resolve(), Path(output).resolve()
    require(not output.exists() and source.parent not in output.parents,
            'Fresh capture-split output outside preserved source required')
    deck, rows = records(source)
    require(deck.get('count') == len(rows) == 5000, 'Complete fresh 5K required')
    require(not deck.get('split_overlay'), 'Freeze capture split before derived training overlay')
    source_pins = {str(source):sha256(source)}
    candidate = copy.deepcopy(deck); frozen = []
    for entry, record in zip(candidate['scenes'], rows):
        original = record['meta']; metadata = copy.deepcopy(original)
        metadata['spec'] = capture_split_spec(original, candidate['config'])
        require(metadata['split'] == entry['split'], 'Capture split differs from its source entry')
        source_pins[str(Path(entry['metadata']).resolve())] = sha256(entry['metadata'])
        source_pins[str(Path(entry['scenario']).resolve())] = entry['scenario_sha256']
        metadata['provisional_capture_split_contract'] = dict(
            source_manifest=str(source), source_manifest_sha256=source_pins[str(source)],
            source_metadata=str(Path(entry['metadata']).resolve()),
            source_metadata_sha256=source_pins[str(Path(entry['metadata']).resolve())],
            policy='explicit-known-map-capture-split-before-regeneration-v1',
            source_spec_sha256=layout.digest(original['spec']), frozen_capture_split=original['split'],
            training_split_allocation_deferred=True, geometry_or_native_gt_changed=False)
        target = output/'metadata'/(entry['id']+'.json')
        atomic_json(target, metadata)
        entry['metadata'] = str(target)
        frozen.append(metadata['spec'])
    frozen_path = output/'frozen_specs.json'
    atomic_json(frozen_path, frozen)
    candidate['config'].update(frozen_specs_path=str(frozen_path), frozen_specs_sha256=sha256(frozen_path))
    candidate.setdefault('provenance', {})['capture_split_freeze'] = dict(
        source_manifest=str(source), source_manifest_sha256=source_pins[str(source)],
        training_split_unchanged_by_this_step=True, final_training_overlay_still_required=True)
    candidate.update(status='PROVISIONAL_CAPTURE_SPLIT_FROZEN_CPU_ONLY', bulk_ready=False, native_qualified=False)
    manifest = output/'manifest.json'; atomic_json(manifest, candidate)
    inputs.verify_hashes(source_pins)
    atomic_json(output/'capture_split_audit.json', dict(frames=len(rows),
        source_manifest_sha256=source_pins[str(source)], candidate_manifest_sha256=sha256(manifest),
        source_pins=source_pins, capture_splits=dict(Counter(r['meta']['split'] for r in rows)),
        raw_scenario_bytes_and_paths_preserved=True, source_metadata_preserved=True,
        supported_spec_override_added=True, training_overlay_still_required=True,
        native_qualified=False, bulk_started=False))
    return manifest


def verify_far_continuation(request):
    """완료한 CPU stage만 새 입력으로 받는다. native 승인/수집 resume가 아니다."""
    from prepare_base5_far_vehicle_qualification import source_audit_pins
    pin = request['continue_from_unqualified_far_stage']
    path = Path(pin['path'])
    require(sha256(path) == pin['sha256'], 'Continuation manifest checksum differs')
    audit_pins = source_audit_pins(path)
    deck, rows = records(path)
    config = deck['config']; policy = far.far
    require(len(rows) == deck.get('count') == 5000
            and config.get('environment_tag') == request['environment'], 'Continuation environment/count differs')
    require(config.get('fresh_geometry_policy', {}).get('seed') == request['seed']
            and deck['provenance'].get('fresh_source_manifest_sha256') == request['source_sha256'],
            'Continuation belongs to another seed or source')
    require(not deck.get('split_overlay') and not any(k in config for k in
        ('asset_scale_policy', unit.KEY, 'static_mission_extent', 'shadow_motion_ground_reuse', close.KEY)),
        'Continuation must be the pre-spread, unit-only far stage')
    frozen_path = Path(config['frozen_specs_path'])
    require(sha256(frozen_path) == config['frozen_specs_sha256'], 'Continuation frozen specs changed')
    frozen = read(frozen_path)
    require(len(frozen) == 5000, 'Continuation frozen spec count differs')
    preservation = read(path.parent/'preservation_audit.json')
    inputs.verify_hashes(preservation['source_hashes'])
    parent = Path(preservation['source_manifest'])
    require(sha256(parent) == preservation['source_manifest_sha256'], 'Continuation parent source changed')
    source_entries = {entry['id']:entry for entry in read(parent)['scenes']}
    require(set(source_entries) == set(preservation['source_files']), 'Continuation parent inventory changed')
    for ident, expected in preservation['source_files'].items():
        entry = source_entries[ident]
        require(sha256(entry['scenario']) == expected['scenario_sha256']
                and sha256(entry['metadata']) == expected['metadata_sha256'],
                'Continuation parent scenario/metadata changed: '+ident)
    plans = 0; adopted_pins = {**audit_pins, str(frozen_path):sha256(frozen_path), str(path):pin['sha256']}
    for record, spec in zip(rows, frozen):
        meta = record['meta']; entry = record['entry']
        require(meta['spec'] == spec and meta['split'] == entry['split'] == spec.get('layout_split'),
                'Continuation spec/capture split changed')
        source_id = meta.get('logical_quota_slot_id')
        require(isinstance(source_id, str)
                and entry['id'] == request['environment']+'_'+source_id
                and spec['seed'] == spatial.seed_for(request['environment'], source_id, request['seed']),
                'Continuation logical slot or geometry seed differs')
        require(not meta.get('shadow_motion_ground_reuse'), 'Fresh shadow cannot inherit old exact reuse')
        if policy.has_plan(meta):
            plan_value = policy.validate_plan(spec, config)
            policy.verify_preserved(meta, meta)
            old_scene, _ = policy._read_source(plan_value)
            policy.verify_payload(read(entry['scenario']), old_scene, plan_value, meta)
            plans += 1
        else:
            expected = preservation['source_files'][entry['id']]
            require(sha256(entry['scenario']) == expected['scenario_sha256']
                    and sha256(entry['metadata']) == expected['metadata_sha256'],
                    'Continuation non-target payload or metadata changed')
        adopted_pins[str(Path(entry['metadata']).resolve())] = sha256(entry['metadata'])
        adopted_pins[str(Path(entry['scenario']).resolve())] = entry['scenario_sha256']
    require(plans == 160 and config[policy.CONFIG_KEY]['allocation']['total'] == 160,
            'Continuation must preserve the complete 160-slot far allocation')
    return path, adopted_pins, dict(manifest_sha256=pin['sha256'], frames=len(rows),
        far_plans=plans, source_metadata_and_non_target_payload_verified=True,
        frozen_specs_and_environment_seed_verified=True, native_qualified=False,
        original_failed_outputs_preserved=True, source_capture_or_resume_performed=False)


def numeric_summary(values):
    values = sorted(float(v) for v in values)
    if not values:
        return dict(count=0)
    def quantile(q):
        index = (len(values)-1)*q
        lo, hi = math.floor(index), math.ceil(index)
        return values[lo]+(values[hi]-values[lo])*(index-lo)
    return dict(count=len(values), mean=sum(values)/len(values), p50=quantile(.5),
                p90=quantile(.9), p95=quantile(.95), maximum=values[-1])


def guarded_spread(source, output):
    """같은 8회 CPU guard에서 가까운 7점과 기존 정상 1점을 검사한다."""
    deck, rows = records(source)
    policy = far.far
    original_order = spread.proposal_order
    accepted = {r['id']:r for r in rows if policy.has_plan(r['meta'])}
    reserved = {}

    def order(plan_value, candidates, target, maximum=512):
        ranked = original_order(plan_value, candidates, target, maximum)
        record = accepted[plan_value['logical_slot_id']]
        previous = record['meta']['secondary_far_vehicle_generation']['attempt']
        require(type(previous) is int and 0 <= previous < maximum,
                'Previous guarded source attempt is outside the frozen CPU proposal budget')
        index = policy._draw_index(plan_value, previous, len(candidates))
        pair = next((pair for pair in ranked if pair[0] == index), None)
        require(pair is not None, 'Previously guarded road point is missing from the same frozen candidates')
        actor = record['meta']['runtime_actors'][plan_value['target_ordinal']]
        require(abs(candidates[index][4]-actor['planned_center_distance_m']) < 1e-6,
                'Previously guarded source point disagrees with its saved geometry')
        fallback = pair not in ranked[:policy.RANKED_CANDIDATES]
        reserved[record['id']] = dict(source_attempt=previous, source_point_index=index,
            reserved_attempt=pair[1], fallback_reserved=fallback,
            target_distance_m=target, previous_distance_m=actor['planned_center_distance_m'])
        if not fallback:
            return ranked
        limit = policy.RANKED_CANDIDATES-1
        return ranked[:limit]+[pair]+[p for p in ranked[limit:] if p[0] != index]

    spread.proposal_order = order
    try:
        manifest = spread.prepare(source, output)
    finally:
        spread.proposal_order = original_order
    _, result = records(manifest)
    evidence = []
    for record in result:
        if record['id'] not in reserved:
            continue
        item = reserved[record['id']]
        proof = record['meta']['secondary_far_vehicle_generation']['initial_distance_spread']
        evidence.append(dict(id=record['id'], **item,
            fallback_selected=item['fallback_reserved'] and proof['retry'] == item['reserved_attempt'],
            actual_distance_m=proof['actual_distance_ego_xy_m'], target_error_m=proof['target_error_m'],
            rejected=proof['rejected']))
    report = dict(policy=SPREAD_POLICY, source_manifest=str(Path(source).resolve()),
        source_manifest_sha256=sha256(source), candidate_manifest_sha256=sha256(manifest),
        plans=len(evidence), maximum_guarded_candidates_per_slot=policy.RANKED_CANDIDATES,
        fallback_reserved_count=sum(r['fallback_reserved'] for r in evidence),
        fallback_selected_count=sum(r['fallback_selected'] for r in evidence),
        target_error_m=numeric_summary(r['target_error_m'] for r in evidence), rows=evidence,
        original_windows_and_ego_primary_preserved=True, existing_guards_rechecked=True,
        native_qa_relaxed=False, exact_uniform_distance_distribution_claimed=False,
        native_qualified=False, bulk_started=False)
    atomic_json(Path(output)/'guarded_source_fallback_audit.json', report)
    return manifest


def cross_environment_audit(baseline_rows, candidate_rows, policy=None):
    """환경별 group ID만 믿지 않고 실제 geometry를 합쳐 검사한다."""
    policy = policy or layout.configuration()
    tagged = []
    for origin, rows in (('baseline', baseline_rows), ('candidate', candidate_rows)):
        for row in rows:
            tagged.append(dict(row, id=origin+':'+row['id'], origin=origin))
    group_of, fingerprints, near_pairs = layout.groups(tagged, policy)
    members = defaultdict(list)
    for row in tagged:
        split_name = row['entry'].get('derived_training_split', row['meta']['split'])
        members[group_of[row['id']]].append(dict(id=row['id'], origin=row['origin'],
            split=split_name, geometry_sha256=fingerprints[row['id']]))
    duplicates = [dict(group_id=group, members=values) for group, values in members.items()
                  if len({r['origin'] for r in values}) > 1]
    conflicts = [r for r in duplicates if len({m['split'] for m in r['members']}) > 1]
    return dict(policy=layout.POLICY, baseline_scenes=len(baseline_rows),
        candidate_scenes=len(candidate_rows), combined_layout_groups=len(members),
        cross_environment_duplicate_groups=duplicates, cross_split_duplicate_groups=conflicts,
        near_duplicate_pairs=near_pairs, passed=not duplicates,
        baseline_modified=False, full_geographical_independence_claimed=False,
        limitation='Known-map background overlap is allowed. Actual repeated layouts are not independent geometry.')


def close_candidate(source, output):
    """새 환경의 eligible 전수에 동일 [1,1.5]m 정책을 적용한다."""
    source, output = Path(source).resolve(), Path(output).resolve()
    require(not output.exists(), 'Fresh close-primary output required')
    deck, rows = records(source)
    inventory, config = close_factory.inventory(deck)
    eligible = {row['id'] for row in inventory['records']}
    require(close.KEY not in deck['config'], 'Close-primary source is already transformed')
    config = copy.deepcopy(config)
    calibration = normalized_sensor_calibration(ROOT/'configs/sensors/Sensor_competition_local.json')
    ctx = None
    candidate = copy.deepcopy(deck)
    candidate.update(config=config, bulk_ready=False, bulk_started=False, native_qualified=False,
                     status='FRESH_ENVIRONMENT_CLOSE_PRIMARY_CPU_ONLY')
    candidate.pop('split_overlay', None)
    output.mkdir(parents=True)
    frozen, changes = [], []
    for entry, original in zip(candidate['scenes'], rows):
        meta = original['meta']
        if entry['id'] in eligible:
            if ctx is None:
                ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                    spatial.gen.geo.DEFAULT_MGEO), config)
            row = close.tagged_row(meta, config)
            rejected = Counter()
            for attempt in range(meta['retry'], meta['retry']+512):
                try:
                    scenario, proposed = spatial.build(copy.deepcopy(row), config, ctx, attempt)
                    if close_factory.roster(proposed) != close_factory.roster(meta):
                        raise ValueError('Close-primary actor roster changed')
                    if proposed['split'] != meta['split']:
                        raise ValueError('Close-primary capture split changed')
                    proposal = close.projection_proposal(proposed, calibration)
                    if not proposal['eligible']:
                        raise ValueError('Conservative whole-OBB camera proposal failed')
                    require({k:v for k,v in proposed['spec'].items() if k != close.KEY} == meta['spec'],
                            'Close-primary changed another frozen spec field')
                    close_factory.unit_payload(scenario)
                    for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                                'fresh_environment_provenance', 'obstacle_support_pair'):
                        if key in meta:
                            proposed[key] = copy.deepcopy(meta[key])
                    proposed[unit.KEY] = copy.deepcopy(config[unit.KEY])
                    unit.validate_actor_identities(proposed, unit.load(config[unit.KEY]))
                    break
                except ValueError as error:
                    rejected[str(error)] += 1
            else:
                atomic_json(output/'failure.json', dict(id=entry['id'], reasons=dict(rejected),
                    maximum_proposals=512, native_qualified=False, bulk_started=False))
                raise ValueError('Bounded close-primary proposals exhausted: '+entry['id'])
            path = output/'scenarios'/(entry['id']+'.json')
            metadata = path.with_name(entry['id']+'_metadata.json')
            proposed['close_primary_source'] = dict(policy=close.POLICY,
                source_entry=copy.deepcopy(entry), source_metadata_sha256=sha256(entry['metadata']),
                primary_ego_x_bounds_m=[1., 1.5], diagnostic_payload_or_outcome_read=False,
                native_acceptance_unchanged=True)
            atomic_json(path, scenario); atomic_json(metadata, proposed)
            entry.update(scenario=str(path), metadata=str(metadata), scenario_sha256=sha256(path))
            close.validate_entry(entry, candidate)
            changes.append(dict(id=entry['id'], proposals=attempt-meta['retry']+1,
                                rejected=dict(rejected), camera_proposal=proposal))
            meta = proposed
            print('close-primary', len(changes), '/', len(eligible), flush=True)
        frozen.append(copy.deepcopy(meta['spec']))
    frozen_path = output/'frozen_specs.json'
    atomic_json(frozen_path, frozen)
    config.update(frozen_specs_path=str(frozen_path), frozen_specs_sha256=sha256(frozen_path))
    atomic_json(output/'manifest.json', candidate)
    atomic_json(output/'close_primary_audit.json', dict(eligibility=inventory, changes=changes,
        changed_scenes=len(changes), expected_eligible_scenes=len(eligible),
        full_band=[1., 1.5], native_qualified=False, bulk_started=False))
    return output/'manifest.json'


def normalize_environment(manifest, environment):
    """새 생성물만 명시적으로 정리한다. source/old release는 수정하지 않는다."""
    deck = read(manifest)
    deck['config'].pop('new_sunny13_diagnostic_plan', None)
    deck['config']['environment_suite']['split_policy'] = (
        'Independent environment geometry; actual repeated layouts share split; logical quota slots are not groups')
    deck['config']['fresh_geometry_policy']['additional_constraint'] = dict(
        policy=ADJACENT_POLICY, scope='other_vehicle frozen ordinary adjacent secondary/primary',
        native_qa_or_actor_contract_relaxed=False)
    deck.update(native_qualified=False, bulk_ready=False, bulk_started=False, training_started=False)
    require(deck['config']['environment_tag'] == environment, 'Generated environment tag differs')
    atomic_json(manifest, deck)


def distribution(rows, physical_assets):
    counts = Counter(); moving_shadow = []
    for row in rows:
        meta = row['meta']
        for actor in meta['runtime_actors']:
            counts['parents'] += 1
            counts['asset/'+actor['name']] += 1
            distance = actor['planned_center_distance_m']
            if actor['name'] in physical_assets:
                counts['physical/'+('0_20m' if distance < 20 else '20_30m' if distance < 30 else '30plus_m')] += 1
        if meta['spec']['family'] == 'shadow' and any(a['motion'] not in ('static','standing') for a in meta['runtime_actors']):
            require(not meta.get('shadow_motion_ground_reuse'), 'Fresh shadow cannot inherit exact old preparation')
            moving_shadow.append(row['id'])
    return dict(planned_parent_counts=dict(sorted(counts.items())),
        fresh_shadow_on_demand_preparation_required=moving_shadow,
        old_exact_shadow_reuse_adopted=False, native_retained_gt_audited=False)


def prepare(source, baseline, output, environment, seed=261017, continue_far=None, continue_far_sha=None):
    request = plan(source, baseline, output, environment, seed, continue_far, continue_far_sha)
    output = Path(request['output'])
    original_inputs = inputs.collect_inputs(Path(request['source']))
    original_inputs.update(inputs.collect_inputs(Path(request['baseline'])))
    continuation = None
    if request['continue_from_unqualified_far_stage']:
        continuation, adopted_pins, continuation_proof = verify_far_continuation(request)
        original_inputs.update(inputs.collect_inputs(continuation))
        original_inputs.update(adopted_pins)
    runtime = inputs.collect_runtime(entrypoints=(Path(__file__).name,))
    _, source_rows = records(request['source'])
    _, baseline_rows = records(request['baseline'])
    started = time.monotonic(); output.mkdir(parents=True)
    atomic_json(output/'plan.json', request)
    stages = []

    def stage(name, function, *args, **kwargs):
        print('STAGE', name, 'START', flush=True)
        tick = time.monotonic()
        manifest = Path(function(*args, **kwargs))
        stages.append(dict(stage=name, manifest=str(manifest), manifest_sha256=sha256(manifest),
                           elapsed_sec=time.monotonic()-tick))
        if name == 'far_spread':
            evidence = manifest.parent/'guarded_source_fallback_audit.json'
            stages[-1]['proposal_adapter_evidence'] = dict(path=str(evidence), sha256=sha256(evidence))
        atomic_json(output/'progress.json', dict(stages=stages, native_qualified=False,
            bulk_started=False, elapsed_sec=time.monotonic()-started))
        print('STAGE', name, 'DONE', round(time.monotonic()-tick, 1), flush=True)
        return manifest

    try:
        if continuation is None:
            current = stage('fresh_geometry', fresh_geometry, Path(request['source']), output/'01_fresh',
                environment, seed)
            _, fresh_rows = records(current)
            require(Counter(quota_key(r['meta'], True) for r in source_rows)
                    == Counter(quota_key(r['meta'], True) for r in fresh_rows),
                    'Fresh geometry changed primary/quota/range/phase/motion allocation')
            current = stage('freeze_capture_split', freeze_capture_splits, current, output/'01b_capture_split')
            current = stage('front_support', front.prepare, current, output/'02_front')
            current = stage('primary_range', primary_range.prepare, current,
                Path(request['range_reference']['path']), output/'03_range')
            current = stage('far_secondary', far.prepare, current, output/'04_far')
        else:
            current = continuation
            atomic_json(output/'continued_far_input_audit.json', continuation_proof)
            stages.append(dict(stage='adopt_unqualified_far_cpu_input', manifest=str(continuation),
                manifest_sha256=sha256(continuation), verification=continuation_proof))
        current = stage('far_spread', guarded_spread, current, output/'05_spread')
        current = stage('static_extent', extent.prepare, current, output/'06_extent',
            counts=request['static_extent_counts'], known_map_layout_split=True)
        current = stage('unit_review', unit_prepare.prepare, current,
            request['unit_reference_specs'], output/'07_unit')
        current = stage('close_primary', close_candidate, current, output/'08_close')
        print('STAGE layout_overlay START', flush=True)
        result = split.prepare(current, output/'candidate_sunny15' if environment == 'sunny15'
                               else output/('candidate_'+environment))
        final = Path(result['manifest'])
        stages.append(dict(stage='layout_overlay', manifest=str(final), manifest_sha256=sha256(final)))
        deck, final_rows = records(final)
        require(Counter(quota_key(r['meta']) for r in source_rows)
                == Counter(quota_key(r['meta']) for r in final_rows),
                'Final primary/quota/phase/motion allocation changed')
        latest_counts = Counter(quota_key(r['meta'], True) for r in baseline_rows)
        actual_counts = Counter(quota_key(r['meta'], True) for r in final_rows)
        quota_report = dict(primary_quota_phase_motion_preserved=True,
            latest_reference_range_allocation_equal=latest_counts == actual_counts,
            range_allocation_removed=dict(latest_counts-actual_counts),
            range_allocation_added=dict(actual_counts-latest_counts),
            auxiliary_identity_and_count_may_resample=True, capture_split_is_not_training_split=True)
        require(latest_counts == actual_counts,
                'Final primary range allocation differs from latest unit reference')
        cross = cross_environment_audit(baseline_rows, final_rows)
        atomic_json(output/'cross_environment_audit.json', cross)
        require(cross['passed'], 'Cross-environment repeated geometry; preserve baseline and revise new candidate')
        audit = scenario_audit.audit(final)
        atomic_json(output/'scenario_audit.json', audit)
        require(audit['status'] == 'OFFLINE_PASS' and not audit['errors']
                and audit['scenes'] == audit['unique_payloads'] == 5000, 'Final scenario audit failed')
        scale_counts = Counter()
        for row in final_rows:
            unit_only.reject_scale_metadata(row['meta'])
            scale_counts.update(unit_only.raw_unit_scales(read(row['entry']['scenario'])))
        native = unit.load(deck['config'][unit.KEY])
        stats = distribution(final_rows, set(native['assets']))
        inputs.verify_hashes(original_inputs); inputs.verify_hashes(runtime)
        receipt = dict(request, final_manifest=str(final), final_manifest_sha256=sha256(final),
            stages=stages, quota_audit=quota_report, layout_audit=result['audit'],
            cross_environment_audit=cross, distribution=stats, raw_unit_scale_counts=dict(scale_counts),
            original_input_hashes=original_inputs, runtime_hashes=runtime,
            elapsed_sec=time.monotonic()-started, source_and_baseline_preserved=True,
            status='CPU_ONLY_PASS; native environment qualification required',
            completion_live_qualified=False, training_started=False)
        atomic_json(output/'build_receipt.json', receipt)
        print('FINAL', final, sha256(final), flush=True)
        return final
    except BaseException as error:
        atomic_json(output/'build_failure.json', dict(schema=SCHEMA, environment=environment,
            error_type=type(error).__name__, error=str(error), stages=stages,
            elapsed_sec=time.monotonic()-started, native_qualified=False, bulk_started=False,
            source_and_baseline_writes_performed=False))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='Complete pre-front v150 richfloor manifest')
    parser.add_argument('--baseline', type=Path, required=True, help='Immutable current Sunny13 unit manifest')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--environment', choices=ENVIRONMENTS, required=True)
    parser.add_argument('--seed', type=int, default=261017)
    parser.add_argument('--continue-from-far', type=Path,
                        help='Explicit completed CPU far-stage input; never a capture resume')
    parser.add_argument('--continue-from-far-sha256', help='Required immutable manifest SHA for CPU-stage continuation')
    parser.add_argument('--run', action='store_true', help='CPU generation only; no native capture approval')
    args = parser.parse_args()
    if args.run:
        print(prepare(args.source, args.baseline, args.output, args.environment, args.seed,
                      args.continue_from_far, args.continue_from_far_sha256))
    else:
        print(json.dumps(plan(args.source, args.baseline, args.output, args.environment, args.seed,
                             args.continue_from_far, args.continue_from_far_sha256), indent=2))


if __name__ == '__main__':
    main()
