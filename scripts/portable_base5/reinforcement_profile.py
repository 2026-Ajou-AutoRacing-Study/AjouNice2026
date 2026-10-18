#!/usr/bin/env python3
"""공통 근거 기반 거리 재할당. 기존 source와 native QA는 변경하지 않는다.

환경별 Python 분기 없이 명시적인 profile을 적용한다. profile은 static generic
physical-obstacle primary의 거리만 재할당하며 actor 추가·삭제나 의미 변경을
허용하지 않는다. 기본은 읽기 전용 계획이다. pilot도 별도 5K 후보이며 실제로
재생성한 행만 기록한다. CPU 준비는 native 승인이나 본수집 승인이 아니다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from collections import Counter, defaultdict
import copy
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[2] / 'dataset/morai_od/scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import audit_base5_scenarios as scenario_audit
import base5_front_release_inputs as inputs
import base5_layout_split as layout
import base5_primary_obstacle_range as native_range
import base5_spatial_diversity as spatial
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
import prepare_base5_layout_split as split_builder
import prepare_base5_unit_environment as assembly
import prepare_base5_unit_environment_smoke as unit_smoke
from run_base5_failure_aware_replenishment_fast import prewarm
import reinforcement_screen as cpu_screen

SCHEMA = 'base5_reinforcement_profile_v2'
POLICY = 'generic-native-support-range-reallocation-v2'
ALLOCATION_SCHEMA = 'base5_reinforcement_allocation_v2'
BUILD_SCHEMA = 'base5_reinforcement_build_extension_v2'
BUILD_POLICY = 'common-profile-explicit-primary-range-extension-v2'
CONFIG_KEY = 'reinforcement_profile'
PROVENANCE_KEY = 'reinforcement_provenance'
IDENTITY_KEYS = ('data_id', 'unique_id', 'name', 'source_list', 'population_pool', 'motion')
PRESERVED_META = ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                  'fresh_environment_provenance', unit.KEY)
MAX_CHANGED = 256


def require(value, message):
    if not value:
        raise ValueError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink path required: ' + str(path))
    return path


def read(path):
    return json.loads(physical(path).read_text())


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def pin(path):
    path = physical(path)
    return dict(path=str(path), sha256=sha256(path))


def bounds(value):
    require(isinstance(value, list) and len(value) == 2
            and all(type(v) in (int, float) and math.isfinite(v) for v in value)
            and 0 <= value[0] < value[1] <= 100,
            'Explicit finite increasing range within model ROI required')
    return list(value)


def validate_profile(document, *, verify_evidence=True):
    require(isinstance(document, dict) and set(document) == {
        'schema', 'policy', 'name', 'seed', 'evidence', 'maximum_changed_scenes',
        'minimum_remaining_near_actors', 'maximum_near_loss_fraction',
        'unchanged_classes', 'rules', 'cpu_feasibility'}, 'Exact common profile schema required; no environment selector')
    require(document['schema'] == SCHEMA and document['policy'] == POLICY,
            'Unknown common reinforcement profile')
    require(isinstance(document['name'], str) and document['name'].strip()
            and type(document['seed']) is int and 0 <= document['seed'] < 2**63,
            'Named profile and independent allocation seed required')
    require(type(document['maximum_changed_scenes']) is int
            and 1 <= document['maximum_changed_scenes'] <= MAX_CHANGED,
            'Finite explicit scene-change bound required')
    require(type(document['minimum_remaining_near_actors']) is int
            and document['minimum_remaining_near_actors'] >= 0
            and type(document['maximum_near_loss_fraction']) in (int, float)
            and 0 <= document['maximum_near_loss_fraction'] <= .25,
            'Explicit conservative near-field preservation budget required')
    controls=document['cpu_feasibility']
    require(isinstance(controls,dict) and set(controls)=={'maximum_attempts','maximum_candidate_tests'}
            and type(controls['maximum_attempts']) is int and 1<=controls['maximum_attempts']<=4096
            and type(controls['maximum_candidate_tests']) is int
            and document['maximum_changed_scenes']<=controls['maximum_candidate_tests']<=1024,
            'Explicit bounded pre-native full-allocation CPU feasibility budget required')
    evidence = document['evidence']
    require(isinstance(evidence, list) and evidence
            and all(isinstance(row, dict) and set(row) == {'path', 'sha256'} for row in evidence)
            and len({row['path'] for row in evidence}) == len(evidence),
            'Unique pinned actual training/support evidence required')
    if verify_evidence:
        inputs.verify_hashes({row['path']: row['sha256'] for row in evidence}, 'Reinforcement evidence')
    unchanged = document['unchanged_classes']
    require(isinstance(unchanged, dict) and set(unchanged) == {'vehicle', 'pedestrian'}
            and all(isinstance(reason, str) and reason.strip() for reason in unchanged.values()),
            'Explicit vehicle and human/animal/compound no-allocation reasons required')
    rules = document['rules']
    require(isinstance(rules, list) and rules and len(rules) <= 8,
            'Explicit bounded shared rule list required')
    seen = set()
    for rule in rules:
        require(isinstance(rule, dict) and set(rule) == {
            'id', 'assets', 'source_bounds_m', 'target_bounds_m', 'roles',
            'maximum_asset_near_fraction', 'minimum_asset_near_remaining'},
            'Exact range-reallocation rule required')
        require(isinstance(rule['id'], str) and rule['id'] and rule['id'] not in seen,
                'Unique profile rule ID required')
        seen.add(rule['id'])
        source, target = bounds(rule['source_bounds_m']), bounds(rule['target_bounds_m'])
        require(source[1] <= 20 <= target[0], 'Only explicit abundant-near to midrange allocation is supported')
        require(isinstance(rule['roles'], list) and rule['roles']
                and len(set(rule['roles'])) == len(rule['roles'])
                and set(rule['roles']) <= {'front', 'rear', 'left', 'right'}, 'Explicit unique actor roles required')
        require(type(rule['maximum_asset_near_fraction']) in (int, float)
                and 0 < rule['maximum_asset_near_fraction'] <= .4
                and type(rule['minimum_asset_near_remaining']) is int
                and rule['minimum_asset_near_remaining'] >= 1,
                'Per-asset diversity floor and bounded loss fraction required')
        require(isinstance(rule['assets'], dict) and rule['assets']
                and all(isinstance(name, str) and name and type(count) is int and 1 <= count <= 32
                        for name, count in rule['assets'].items()), 'Explicit per-asset requested counts required')
    return document


def actor_identity(actors):
    return [{key: actor.get(key) for key in IDENTITY_KEYS} for actor in actors]


def near_counts(records, native_sizes):
    result = Counter()
    for record in records:
        for actor in record['meta']['runtime_actors']:
            if actor['name'] in native_sizes and actor['planned_center_distance_m'] < 20:
                result[actor['name']] += 1
    return result


def near_role_counts(records, native_sizes):
    result = Counter()
    for record in records:
        for actor in record['meta']['runtime_actors']:
            if actor['name'] in native_sizes and actor['planned_center_distance_m'] < 20:
                result[(actor['name'],actor['actual_ego_role'])] += 1
    return result


def eligible(meta, rule, native_sizes):
    spec, actors = meta['spec'], meta['runtime_actors']
    if (not actors or spec['family'] != 'obstacle' or spec['mission']
            or spec.get('shadow_phase') or spec.get('avoidance_phase')
            or spec.get('static_mission_extent') or spec.get('close_primary_observation')
            or spec.get('topology') != 'route_general'
            or any(a['motion'] not in ('static', 'standing') for a in actors)):
        return False
    actor = actors[0]
    if (actor['name'] not in rule['assets'] or actor['name'] not in native_sizes
            or actor['source_list'] != 'objectList' or spec['role'] not in rule['roles']):
        return False
    lo, hi = rule['source_bounds_m']
    if not lo <= actor['planned_center_distance_m'] < hi:
        return False
    target = rule['target_bounds_m']
    # Retain the existing repeated-native-size guard. A profile is not permission
    # to push thin/compact assets outside their proven native range policy.
    cap = native_range.classify(native_sizes[actor['name']])
    if cap is not None and target[1] > cap:
        return False
    previous_cap = spec.get(native_range.PLAN_KEY,{}).get('cap_m')
    if previous_cap is not None and target[1] > previous_cap:
        return False
    if spec.get('population_layout') == 'clustered':
        # The legacy clustered builder derives ordinary secondary bounds from
        # the primary bounds. Moving this primary can force every unprotected
        # near secondary beyond20m. Recognize that impossible contract before
        # spending CPU/live retries; explicit rich-floor bands remain usable.
        plan=spec.get('secondary_obstacle_support',{})
        protected=set(plan.get('ordinals',[])+plan.get('protected_near_ordinals',[])
                      +plan.get('protected_mid_ordinals',[]))
        for ordinal,secondary in enumerate(actors[1:],1):
            if (secondary['source_list']=='objectList'
                    and secondary.get('population_pool','').startswith('obstacle')
                    and secondary['planned_center_distance_m'] < 30 and ordinal not in protected):
                return False
    if target[0] >= 30 and spec['role'] not in ('front', 'rear'):
        return False
    return cpu_screen.compatible(meta,rule)


def allocate(records, profile, native_sizes, source_sha256, *, cpu_rejected_pairs=()):
    """Pure deterministic allocation; never use native outcomes to reselect rows."""
    validate_profile(profile)
    require(records and len({r['id'] for r in records}) == len(records), 'Unique nonempty source records required')
    before = near_counts(records, native_sizes)
    role_before = near_role_counts(records,native_sizes)
    selected, used, removed, role_removed = [], set(), Counter(), Counter()
    rejected=set(cpu_rejected_pairs)
    require(all(isinstance(pair,tuple) and len(pair)==2 and all(isinstance(value,str) for value in pair)
                for pair in rejected),'Explicit CPU-only (slot, rule) rejection keys required')
    # Every rule protects the same original near stratum. Use the strictest
    # declared cumulative cap when an asset participates in several rules.
    asset_caps={name:min(min(math.floor(before[name]*rule['maximum_asset_near_fraction']),
                             max(0,before[name]-rule['minimum_asset_near_remaining']))
                         for rule in profile['rules'] if name in rule['assets'])
                for name in {name for rule in profile['rules'] for name in rule['assets']}}
    global_limit = min(profile['maximum_changed_scenes'],
        math.floor(sum(before.values()) * profile['maximum_near_loss_fraction']),
        max(0, sum(before.values()) - profile['minimum_remaining_near_actors']))
    decisions = []
    for rule in profile['rules']:
        for name, requested in sorted(rule['assets'].items()):
            require(name in native_sizes, 'Profile asset lacks pinned native unit dimensions: ' + name)
            candidates = [record for record in records if record['id'] not in used
                          and (record['id'],rule['id']) not in rejected
                          and eligible(record['meta'], rule, native_sizes)
                          and record['meta']['runtime_actors'][0]['name'] == name]
            candidates.sort(key=lambda row: (digest([POLICY, profile['seed'], rule['id'], row['id']]), row['id']))
            cap = asset_caps[name]
            available = max(0, min(requested, cap - removed[name], global_limit - len(selected)))
            picked = []
            for candidate in candidates if available else ():
                key = (name,candidate['meta']['runtime_actors'][0]['actual_ego_role'])
                if role_removed[key] >= max(0,role_before[key]-1):
                    continue
                picked.append(candidate);role_removed[key] += 1
                if len(picked) == available:
                    break
            decisions.append(dict(rule=rule['id'], asset=name, requested=requested,
                eligible=len(candidates), selected=len(picked), source_near_parents=before[name],
                maximum_asset_removals=cap, target_bounds_m=rule['target_bounds_m']))
            for record in picked:
                meta = record['meta']; primary = meta['runtime_actors'][0]
                assignment = dict(id=record['id'], rule=rule['id'], asset=name,
                    data_id=primary['data_id'], source_spec_sha256=digest(meta['spec']),
                    source_scenario_sha256=record['entry']['scenario_sha256'],
                    source_metadata_sha256=sha256(record['entry']['metadata']),
                    source_actual_distance_m=primary['planned_center_distance_m'],
                    original_bounds_m=list(meta['spec']['distance_bounds']),
                    target_bounds_m=list(rule['target_bounds_m']), role=meta['spec']['role'],
                    capture_split=meta['split'], actor_identity=actor_identity(meta['runtime_actors']))
                selected.append(assignment); used.add(record['id']); removed[name] += 1
    require(selected, 'No eligible bounded reinforcement assignment; do not silently claim a new profile')
    return dict(schema=ALLOCATION_SCHEMA, policy=POLICY, source_manifest_sha256=source_sha256,
        profile_sha256=digest(profile), profile=copy.deepcopy(profile), assignments=selected,
        decisions=decisions, original_near_parent_counts=dict(before),
        planned_near_primary_loss=dict(removed), global_change_limit=global_limit,
        protected_near_asset_role_presence={name+'/'+role:count for (name,role),count in role_before.items()},
        maximum_scenes=profile['maximum_changed_scenes'], primary_asset_quotas_preserved=True,
        no_actor_addition=True, native_outcomes_used_for_selection=False,
        native_qualified=False, bulk_ready=False, training_started=False)


def changed_spec(meta, assignment):
    require(digest(meta['spec']) == assignment['source_spec_sha256']
            and actor_identity(meta['runtime_actors']) == assignment['actor_identity']
            and meta['split'] == assignment['capture_split'], 'Reallocation source identity/spec/split changed')
    result = copy.deepcopy(meta['spec'])
    result.pop(native_range.PLAN_KEY, None)
    result.update(distance_bounds=list(assignment['target_bounds_m']),
        distance_bin='reinforcement/' + assignment['rule'],
        frozen_population_data_ids=[a['data_id'] for a in meta['runtime_actors']],
        layout_split=meta['split'])
    return result


def verify_transition(before, after, assignment):
    require(after['spec'] == changed_spec(before, assignment), 'Unapproved spec field change during reinforcement')
    require(actor_identity(before['runtime_actors']) == actor_identity(after['runtime_actors'])
            and before['split'] == after['split'], 'Actor roster/motion/capture split changed during reinforcement')
    primary = after['runtime_actors'][0]
    lo, hi = assignment['target_bounds_m']
    require(lo <= primary['planned_center_distance_m'] < hi
            and primary['actual_ego_role'] == assignment['role'], 'Reinforced primary escaped declared range/role')
    # Rebuilding a mixture may change other poses, but must not sacrifice its
    # existing near/mid physical support to create the targeted primary.
    for old, new in zip(before['runtime_actors'][1:], after['runtime_actors'][1:]):
        if old['source_list'] == 'objectList' and old.get('population_pool', '').startswith('obstacle'):
            distance = old['planned_center_distance_m']; actual = new['planned_center_distance_m']
            require(not (distance < 20 <= actual), 'Protected near secondary was moved out of near field')
            require(not (20 <= distance < 30 and not 20 <= actual < 30), 'Protected midrange secondary changed band')
    return True


def records_from_manifest(manifest):
    deck = read(manifest)
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000, 'Complete frozen 5K source required')
    records = []
    for entry in deck['scenes']:
        require(sha256(entry['scenario']) == entry['scenario_sha256'], 'Source payload SHA changed: ' + entry['id'])
        records.append(dict(id=entry['id'], entry=entry, meta=read(entry['metadata'])))
    require(len({r['id'] for r in records}) == 5000, 'Unique source slots required')
    return deck, records


def plan(source, source_build, profile):
    source, source_build, profile = map(physical, (source, source_build, profile))
    # Validate the real original assembly; never synthesize an old-policy receipt
    # for a modified manifest.
    context = unit_smoke.inspect_source(source, source_build)
    deck, records = records_from_manifest(source)
    native = unit.load(deck['config'][unit.KEY])
    sizes = {name: row['native_size_lwh_m'] for name, row in native['assets'].items()}
    profile_doc = validate_profile(read(profile))
    result = allocate(records, profile_doc, sizes, sha256(source))
    result.update(source=pin(source), source_build=pin(source_build), profile_file=pin(profile),
        original_input_hashes=context['input_hashes'], implementation=pin(__file__),
        screen_implementation=pin(cpu_screen.__file__))
    return result


def pilot_assignments(allocation, maximum):
    require(type(maximum) is int and 1 <= maximum <= 48, 'Explicit bounded pilot size <=48 required')
    buckets = defaultdict(list)
    for row in allocation['assignments']:
        buckets[(row['rule'], row['asset'])].append(row)
    selected = []
    for offset in range(max(len(values) for values in buckets.values())):
        for key in sorted(buckets):
            if offset < len(buckets[key]):
                selected.append(buckets[key][offset])
                if len(selected) == maximum:
                    return selected
    return selected


def safe_output(output, allocation):
    output = physical(output)
    require(not output.exists(), 'New reinforcement output required')
    for record in (allocation['source'], allocation['source_build'], allocation['profile_file']):
        protected = physical(record['path'])
        require(output != protected and output not in protected.parents
                and protected.parent not in output.parents and output != protected.parent,
                'New output must not overlap preserved source/profile/evidence')
    for filename in allocation['original_input_hashes']:
        require(output not in physical(filename).parents, 'Output cannot contain a preserved input')
    return output


def prepare(source, source_build, profile, output, *, phase='pilot', pilot_size=32,
            maximum_attempts=None, cpu_screen_path=None):
    require(phase in ('pilot', 'full'), 'Explicit pilot/full phase required')
    allocation = plan(source, source_build, profile)
    require(maximum_attempts is None or maximum_attempts==allocation['profile']['cpu_feasibility']['maximum_attempts'],
            'CPU search budget belongs to the frozen profile, not an untracked CLI override')
    output = safe_output(output, allocation)
    deck, records = records_from_manifest(source)
    config = copy.deepcopy(deck['config'])
    output.mkdir(parents=True)
    # They are typed, byte-pinned documents interpreted by this explicit module,
    # not arbitrary input-graph containers. Evidence reports may reference JSONL
    # logs named *.json; recursive generic JSON parsing would misread those logs.
    atomic_json(output/'profile.frozen', read(profile))
    request=allocation
    # runpy.run_module(..., run_name='__main__') need not replace the caller's
    # sys.modules['__main__']; bind this exact module namespace, not that caller.
    driver=SimpleNamespace(**globals())
    if cpu_screen_path is None:
        cpu_screen_path=cpu_screen.prepare(request,records,config,output/'cpu_screen',driver=driver)
    allocation,cached,screen_pins=cpu_screen.verify(request,records,config,cpu_screen_path,driver=driver)
    allocation.update({key:request[key] for key in ('source','source_build','profile_file',
                      'original_input_hashes','implementation','screen_implementation')})
    atomic_json(output/'allocation.frozen', allocation)
    profile_pin, allocation_pin = pin(output/'profile.frozen'), pin(output/'allocation.frozen')
    assignments = (pilot_assignments(allocation, pilot_size) if phase == 'pilot'
                   else allocation['assignments'])
    assigned = {row['id']: row for row in assignments}
    config[CONFIG_KEY] = dict(policy=POLICY, name=allocation['profile']['name'],
        profile=profile_pin, allocation=allocation_pin, phase=phase,
        source=allocation['source'], source_build=allocation['source_build'],
        implementation=pin(__file__), cpu_feasibility=pin(cpu_screen_path),
        screen_implementation=pin(cpu_screen.__file__), environment_specific_branch=False,
        native_qa_unchanged=True, new_sampling_behavior_required=False)
    native = unit.load(config[unit.KEY]); sizes = {name:r['native_size_lwh_m'] for name,r in native['assets'].items()}
    before_near = near_counts(records, sizes)
    entries, specs, changed = [], [], []
    started = time.monotonic()
    for original in records:
        entry, old = original['entry'], original['meta']
        if entry['id'] in assigned:
            target = assigned[entry['id']]
            require(sha256(entry['metadata']) == target['source_metadata_sha256'], 'Allocated source metadata changed')
            spec = changed_spec(old, target)
            cached_row=cached[entry['id']]
            require(cached_row['assignment']==target,'CPU cache belongs to a different rule/assignment')
            scenario,meta=read(cached_row['scenario']['path']),read(cached_row['metadata']['path'])
            retry=cached_row['actual_retry'];rejected=cached_row['rejected_reasons']
            verify_transition(old,meta,target)
            meta[PROVENANCE_KEY] = dict(policy=POLICY, profile=profile_pin, allocation=allocation_pin,
                assignment=copy.deepcopy(target),
                previous_primary_range_plan=copy.deepcopy(old['spec'].get(native_range.PLAN_KEY)),
                source_scenario=pin(entry['scenario']), source_metadata=pin(entry['metadata']),
                original_retry=old['retry'], actual_retry=retry,
                range_contract_replaced_explicitly=True, original_qa_unchanged=True,
                model_ap_improvement_claimed=False)
            scenario_path = output/'changed_scenarios'/(entry['id']+'.json')
            meta_path = output/'changed_metadata'/(entry['id']+'.json')
            atomic_json(scenario_path, scenario); atomic_json(meta_path, meta)
            updated = dict(entry, scenario=str(scenario_path), metadata=str(meta_path),
                scenario_sha256=sha256(scenario_path), distance_bounds=spec['distance_bounds'])
            updated.pop('derived_training_split', None); updated.pop('derived_layout_group_id', None)
            changed.append(dict(assignment=target, scenario=pin(scenario_path), metadata=pin(meta_path),
                actual_distance_m=meta['runtime_actors'][0]['planned_center_distance_m'],
                retries=retry-old['retry']+1, rejected=dict(rejected)))
        else:
            updated, meta = copy.deepcopy(entry), copy.deepcopy(old)
        entries.append(updated); specs.append(copy.deepcopy(meta['spec']))
    frozen = output/'frozen_specs.json'; atomic_json(frozen, specs)
    config.update(frozen_specs_path=str(frozen), frozen_specs_sha256=sha256(frozen))
    staged = copy.deepcopy(deck)
    staged.pop('split_overlay', None)
    staged.update(config=config, scenes=entries, bulk_ready=False, native_qualified=False,
        bulk_started=False, training_started=False, status='COMMON_REINFORCEMENT_CPU_ONLY')
    staged_path = output/'staged_manifest.json'; atomic_json(staged_path, staged)
    # Every resulting geometry, including unmodified layouts, receives one new
    # consistent overlay. Existing capture split fields remain untouched.
    overlay = split_builder.prepare(staged_path, output/'layout')
    final = physical(overlay['manifest']); final_deck, final_rows = records_from_manifest(final)
    after_near = near_counts(final_rows, sizes)
    before_roles,after_roles=near_role_counts(records,sizes),near_role_counts(final_rows,sizes)
    profile_doc = allocation['profile']; loss = sum(before_near.values()) - sum(after_near.values())
    require(sum(after_near.values()) >= profile_doc['minimum_remaining_near_actors']
            and loss <= math.floor(sum(before_near.values())*profile_doc['maximum_near_loss_fraction']),
            'Actual regeneration exceeded the protected near-field budget')
    for rule in profile_doc['rules']:
        for name in rule['assets']:
            require(after_near[name] >= rule['minimum_asset_near_remaining']
                    and before_near[name]-after_near[name] <= math.floor(before_near[name]*rule['maximum_asset_near_fraction']),
                    'Actual regeneration exceeded asset near-field preservation: '+name)
    require(all(after_roles[key] >= 1 for key in before_roles),
            'Actual regeneration removed an existing near-field asset/role stratum')
    require(Counter(tuple((a['name'], a['data_id'], a['motion']) for a in row['meta']['runtime_actors']) for row in records)
            == Counter(tuple((a['name'], a['data_id'], a['motion']) for a in row['meta']['runtime_actors']) for row in final_rows),
            'Full-deck actor asset/motion quotas changed')
    cross = assembly.cross_environment_audit(records, final_rows)
    # Most unchanged layouts intentionally overlap their parent. Do not call
    # this parent comparison an independent-weather audit.
    cross['interpretation'] = 'Expected immutable parent reuse; changed rows audited separately'
    audit = scenario_audit.audit(final)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors'), 'New full5K scenario audit failed')
    atomic_json(output/'scenario_audit.json', audit)
    inputs.verify_hashes(allocation['original_input_hashes'], 'Preserved reinforcement source')
    require(pin(profile) == allocation['profile_file'] and pin(__file__) == allocation['implementation'],
            'Profile implementation/evidence changed during CPU preparation')
    pins = dict(allocation['original_input_hashes'])
    pins.update(screen_pins)
    pins.update(inputs.collect_inputs(final))
    pins.update({row['path']:row['sha256'] for row in (allocation['source'], allocation['source_build'],
                 allocation['profile_file'], allocation['implementation'],
                 allocation['screen_implementation'],profile_pin, allocation_pin)})
    pins.update({row['path']:row['sha256'] for row in profile_doc['evidence']})
    report = dict(schema=BUILD_SCHEMA, policy=BUILD_POLICY, phase=phase,
        source=allocation['source'], source_build=allocation['source_build'], profile=profile_pin,
        allocation=allocation_pin, implementation=pin(__file__),
        cpu_feasibility=pin(cpu_screen_path),screen_implementation=pin(cpu_screen.__file__),
        final_manifest=str(final), final_manifest_sha256=sha256(final),
        environment=final_deck['config']['environment_tag'], count=5000,
        changed_scenes=len(changed), allocated_scenes=len(allocation['assignments']), changes=changed,
        source_and_baseline_preserved=True, original_input_hashes=pins,
        runtime_hashes=inputs.collect_runtime(entrypoints=('prepare_base5_unit_environment.py',)),
        original_near_parent_counts=dict(before_near), final_near_parent_counts=dict(after_near),
        actual_near_loss=loss, full_audit=pin(output/'scenario_audit.json'),
        layout_audit=overlay['audit'], parent_layout_comparison=cross,
        native_qualified=False, bulk_ready=False, bulk_started=False, training_started=False,
        status='CPU_ONLY_REINFORCEMENT; native profile qualification required',
        all_assignments_materialized=phase=='full', elapsed_sec=time.monotonic()-started,
        native_qa_unchanged=True, class_and_asset_quotas_preserved=True,
        mission_motion_capture_split_preserved=True)
    atomic_json(output/'build_extension.json', report)
    return final


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'source-build', 'profile'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--phase', choices=('pilot','full'), default='pilot')
    parser.add_argument('--pilot-size', type=int, default=32)
    parser.add_argument('--maximum-attempts', type=int)
    parser.add_argument('--cpu-screen', type=Path)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    if args.run:
        require(args.output is not None, 'Explicit new output required')
        print(prepare(args.source,args.source_build,args.profile,args.output,
            phase=args.phase,pilot_size=args.pilot_size,maximum_attempts=args.maximum_attempts,
            cpu_screen_path=args.cpu_screen))
    else:
        result = plan(args.source,args.source_build,args.profile)
        print(json.dumps({key:value for key,value in result.items() if key!='original_input_hashes'},indent=2))


if __name__ == '__main__':
    main()
