#!/usr/bin/env python3
"""Prepare an explicit v152 5K candidate; never capture or mutate frozen v151.

Only ordinary generic static physical-obstacle primaries can receive a declared
native-size range cap. Whole actor lists, seeds, yaw sectors, spatial windows,
split and all existing v151 front assignments remain frozen. Unknown, already
near and out-of-scope payloads are copied byte-for-byte.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import shutil
import time

import audit_base5_scenarios
import base5_primary_obstacle_range as policy
import base5_spatial_diversity as spatial
import prepare_base5_front_support_qualification as qualification
from base5_capture_safety import atomic_json, sha256
from run_base5_failure_aware_replenishment_fast import prewarm


def read(path):
    return json.loads(Path(path).read_text())


def verify_transition(before, after, expected):
    """A new declared quota, not a claim that the old far quota was fulfilled."""
    if after['spec'] != expected:
        raise policy.RangeContractError('Primary range preparation changed the frozen candidate spec')
    if (policy.identity(before['runtime_actors']) != policy.identity(after['runtime_actors'])
            or before['split'] != after['split']):
        raise policy.RangeContractError('Primary range preparation changed actor identity/motion/count/split')
    plan = expected[policy.PLAN_KEY]
    old = before['spec']
    if plan['source_spec_sha256'] != policy.digest(old):
        raise policy.RangeContractError('Primary range source spec provenance changed')
    declared = copy.deepcopy(old)
    declared.update(distance_bounds=plan['distance_bounds_m'], distance_bin=plan['distance_bin'],
                    frozen_population_data_ids=[a['data_id'] for a in before['runtime_actors']])
    declared[policy.PLAN_KEY] = copy.deepcopy(plan)
    if expected != declared:
        raise policy.RangeContractError('Primary range changed fields outside declared bounds/bin/plan/frozen IDs')
    policy.verify_preserved(after, after)


def prepare(source, size_reference, output, maximum_attempts=4096):
    source, size_reference, output = Path(source).resolve(), Path(size_reference).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('New primary range candidate output required')
    if not 1 <= maximum_attempts <= 4096:
        raise ValueError('Finite CPU proposal budget between one and 4096 required')
    deck = read(source)
    entries, originals, _ = qualification.validate_source(deck)
    if deck['config'].get(policy.CONFIG_KEY) or any(policy.has_plan(m) for m in originals.values()):
        raise policy.RangeContractError('Primary range candidate is already frozen; do not reassign')
    config = copy.deepcopy(deck['config'])
    config[policy.CONFIG_KEY] = policy.configuration(size_reference)
    config[policy.CONFIG_KEY].update(source_manifest=str(source), source_manifest_sha256=sha256(source))
    catalog_path = spatial.gen.catalog.DEFAULT_CATALOG
    catalog = {row['data_id']: row for row in read(catalog_path)['assets']}
    scripts = Path(__file__).parent
    watched = {str(scripts / name): sha256(scripts / name) for name in
        (Path(__file__).name, 'base5_primary_obstacle_range.py', 'base5_front_completion_adapter.py',
         'base5_obstacle_front_support.py', 'base5_obstacle_support_policy.py', 'base5_obstacle_rich_floor.py',
         'generate_base5_independent.py', 'base5_spatial_diversity.py', 'base5_static_yaw.py',
         'base5_placement_geometry.py', 'base5_population_budget.py', 'audit_base5_scenarios.py',
         'prepare_base5_front_support_qualification.py')}
    input_pins = {str(source): sha256(source), str(size_reference): sha256(size_reference),
                  str(Path(catalog_path).resolve()): sha256(catalog_path)}
    source_pins = {ident: dict(scenario_sha256=entry['scenario_sha256'],
                              metadata_sha256=sha256(entry['metadata'])) for ident, entry in entries.items()}
    output.mkdir(parents=True)
    candidate_entries, frozen_specs, changes = [], [], []
    decisions, rejections = Counter(), Counter()
    context, started = None, time.monotonic()
    for index, entry in enumerate(deck['scenes']):
        ident, old = entry['id'], originals[entry['id']]
        actors = old['runtime_actors']
        asset = catalog.get(actors[0]['data_id'], {}) if actors else {}
        spec, decision = policy.attach(old['spec'], actors, asset, config[policy.CONFIG_KEY], ident, old['split'])
        decisions[decision['reason']] += 1
        changed = decision['changed']
        geometry_changed = False
        if changed:
            policy.validate_plan(spec, config)
            lower, upper = spec['distance_bounds']
            distance = actors[0]['planned_center_distance_m']
            geometry_changed = not lower <= distance < upper
            if not geometry_changed:
                scene, meta = None, copy.deepcopy(old)
                meta['spec'] = copy.deepcopy(spec)
                verify_transition(old, meta, spec)
            else:
                if context is None:
                    context = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                                          spatial.gen.geo.DEFAULT_MGEO), config)
                for retry in range(old['retry'], old['retry'] + maximum_attempts):
                    try:
                        scene, meta = spatial.build(spec, config, context, retry)
                    except (policy.RangeContractError, policy.front_completion.FrozenFrontContractError):
                        raise
                    except ValueError as exc:
                        rejections[str(exc)] += 1
                        continue
                    verify_transition(old, meta, spec)
                    break
                else:
                    atomic_json(output / 'failure.json', dict(id=ident, completed=index,
                        maximum_offline_proposals=maximum_attempts, reasons=dict(rejections),
                        bulk_started=False, source_preserved=True))
                    raise policy.RangeGeometryExhausted('Bounded primary range preparation exhausted: ' + ident)
            for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                        'fresh_environment_provenance'):
                if key in old: meta[key] = copy.deepcopy(old[key])
            meta['primary_obstacle_range_preparation'] = dict(source_id=ident,
                source_scenario_sha256=entry['scenario_sha256'],
                source_metadata_sha256=source_pins[ident]['metadata_sha256'],
                original_retry=old['retry'], actual_retry=meta['retry'],
                original_primary_distance_m=actors[0]['planned_center_distance_m'],
                current_primary_distance_m=meta['runtime_actors'][0]['planned_center_distance_m'],
                geometry_changed=geometry_changed,
                policy=policy.POLICY, original_quota_replaced_explicitly=True,
                native_qa_unchanged=True, model_ap_improvement_certified=False)
        else:
            scene, meta = None, copy.deepcopy(old)
        scenario_path = output / 'scenarios' / (ident + '.json')
        metadata_path = output / 'scenarios' / (ident + '_metadata.json')
        scenario_path.parent.mkdir(parents=True, exist_ok=True)
        if changed:
            if geometry_changed:
                atomic_json(scenario_path, scene)
            else:
                shutil.copyfile(entry['scenario'], scenario_path)
                if sha256(scenario_path) != entry['scenario_sha256']:
                    raise policy.RangeContractError('Already-in-bounds primary payload changed')
            atomic_json(metadata_path, meta)
        else:
            shutil.copyfile(entry['scenario'], scenario_path)
            shutil.copyfile(entry['metadata'], metadata_path)
            if (sha256(scenario_path) != entry['scenario_sha256']
                    or sha256(metadata_path) != source_pins[ident]['metadata_sha256']):
                raise policy.RangeContractError('Unchanged source payload/metadata is not byte-equivalent')
        row = copy.deepcopy(entry)
        row.update(scenario=str(scenario_path), metadata=str(metadata_path), scenario_sha256=sha256(scenario_path))
        if changed:
            row['distance_bounds'] = list(meta['spec']['distance_bounds'])
            changes.append(dict(id=ident, asset=actors[0]['name'], data_id=actors[0]['data_id'],
                cap_m=decision['cap_m'], role=spec['role'], split=old['split'],
                original_bounds=old['spec']['distance_bounds'], current_bounds=spec['distance_bounds'],
                original_distance_m=actors[0]['planned_center_distance_m'],
                current_distance_m=meta['runtime_actors'][0]['planned_center_distance_m'],
                geometry_changed=geometry_changed,
                actor_data_ids=[a['data_id'] for a in actors], original_retry=old['retry'], actual_retry=meta['retry'],
                source_scenario_sha256=entry['scenario_sha256'], scenario_sha256=row['scenario_sha256']))
        candidate_entries.append(row)
        frozen_specs.append(copy.deepcopy(meta['spec']))
        if (index + 1) % 100 == 0:
            print('primary-range', index + 1, 'changed', len(changes),
                  'elapsed_sec', round(time.monotonic() - started, 1), flush=True)
    frozen = output / 'frozen_specs.json'
    atomic_json(frozen, frozen_specs)
    config.update(frozen_specs_path=str(frozen), frozen_specs_sha256=sha256(frozen))
    candidate = copy.deepcopy(deck)
    candidate.update(config=config, scenes=candidate_entries, count=len(candidate_entries),
        status='OFFLINE_ONLY_PRIMARY_RANGE_V152; native qualification pending; no bulk collection')
    candidate.setdefault('provenance', {})['primary_obstacle_range_source_hashes'] = watched
    manifest = output / 'manifest.json'
    atomic_json(manifest, candidate)
    errors, primary_before, primary_after = [], Counter(), Counter()
    front_before, front_after, actor_before, actor_after = [], [], [], []
    vehicle_far_preserved = 0
    for row in candidate_entries:
        ident = row['id']
        before, after = originals[ident], read(row['metadata'])
        if (policy.identity(before['runtime_actors']) != policy.identity(after['runtime_actors'])
                or before['split'] != after['split'] or before['spec']['seed'] != after['spec']['seed']):
            errors.append([ident, 'actor identity/count/motion/seed/split changed'])
        front_before.append([ident, before['spec'].get('secondary_obstacle_front_support')])
        front_after.append([ident, after['spec'].get('secondary_obstacle_front_support')])
        actor_before.append([ident, policy.identity(before['runtime_actors'])])
        actor_after.append([ident, policy.identity(after['runtime_actors'])])
        for meta, counts in ((before, primary_before), (after, primary_after)):
            if meta['runtime_actors']:
                actor = meta['runtime_actors'][0]
                counts[str((meta['spec']['family'], actor['data_id'], meta['spec']['role'],
                            tuple(meta['spec'].get('distance_bounds', []))))] += 1
        if before['runtime_actors']:
            primary = before['runtime_actors'][0]
            asset = catalog.get(primary['data_id'], {})
            if asset.get('semantic_class') == 'vehicle' and primary['planned_center_distance_m'] >= 50:
                if row['scenario_sha256'] != entries[ident]['scenario_sha256']:
                    errors.append([ident, 'far Vehicle payload changed'])
                vehicle_far_preserved += 1
    if front_before != front_after: errors.append(['all', 'frozen v151 front allocation changed'])
    for path, expected in {**watched, **input_pins}.items():
        if sha256(path) != expected: errors.append([path, 'source/reference/runtime changed'])
    for ident, pins in source_pins.items():
        entry = entries[ident]
        if sha256(entry['scenario']) != pins['scenario_sha256'] or sha256(entry['metadata']) != pins['metadata_sha256']:
            errors.append([ident, 'frozen source payload/metadata changed'])
    audit = audit_base5_scenarios.audit(manifest)
    atomic_json(output / 'scenario_audit.json', audit)
    if audit.get('status') != 'OFFLINE_PASS' or audit.get('errors'):
        errors.append(['all', 'scenario audit failed'])
    geometry_changed_count = sum(row['geometry_changed'] for row in changes)
    preservation = dict(schema='base5_primary_obstacle_range_preservation_v1', passed=not errors,
        policy=policy.POLICY, source_manifest=str(source), source_manifest_sha256=input_pins[str(source)],
        candidate_manifest_sha256=sha256(manifest), size_reference=config[policy.CONFIG_KEY]['reference'],
        frames=len(candidate_entries), changed_scenes=len(changes), planned_scenes=len(changes),
        payload_changed_scenes=geometry_changed_count,
        planned_payloads_preserved=len(changes) - geometry_changed_count,
        declared_plan_count=len(changes), geometry_changed_scenes=geometry_changed_count,
        declared_only_payloads=len(changes) - geometry_changed_count,
        unchanged_payloads=len(candidate_entries) - geometry_changed_count, errors=errors,
        actor_identity_digest_before=policy.digest(actor_before), actor_identity_digest_after=policy.digest(actor_after),
        front_allocation_digest_before=policy.digest(front_before), front_allocation_digest_after=policy.digest(front_after),
        front_selected_count=sum(bool(m['spec'].get('secondary_obstacle_front_support', {}).get('selected')) for m in originals.values()),
        far_vehicle_primary_50m_payloads_preserved=vehicle_far_preserved,
        declared_distance_distribution_before=dict(primary_before), declared_distance_distribution_after=dict(primary_after),
        source_hashes=watched, input_hashes=input_pins, source_files=source_pins,
        decisions=dict(decisions), generation_rejections=dict(rejections),
        elapsed_sec=time.monotonic() - started, live_qualified=False, bulk_started=False,
        limitation='Explicit new CPU distribution; unchanged native visibility/ground/collision/stop/Park QA and new live qualification required')
    atomic_json(output / 'preservation_audit.json', preservation)
    atomic_json(output / 'primary_range_delta.json', dict(policy=policy.POLICY, source_manifest=str(source),
        candidate_manifest=str(manifest), changes=changes, decision_counts=dict(decisions),
        cap_counts=dict(Counter(str(row['cap_m']) for row in changes)), live_qualified=False, bulk_started=False))
    if errors:
        raise policy.RangeContractError('Full primary range candidate audit failed; preserve diagnostic output')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--size-reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--maximum-attempts', type=int, default=4096)
    args = parser.parse_args()
    print(prepare(args.manifest, args.size_reference, args.output, args.maximum_attempts))
