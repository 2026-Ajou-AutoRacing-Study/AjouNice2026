#!/usr/bin/env python3
"""Register a bounded v151 native regression before observing new outcomes.

Reads the complete candidate, old registration and old deficit evidence only.
Never changes a scenario, source manifest, raw capture or MORAI runtime file.
Historical failures are explicit negative-regression controls, not fabricated
new failures and not evidence that the new qualification has already passed.
"""
import argparse
from collections import Counter, defaultdict
import copy
import hashlib
import json
from pathlib import Path

import base5_obstacle_front_support as front
from base5_capture_safety import atomic_json, sha256
from base5_motion_ground import moving_actors

POLICY = 'front-support-native-qualification-v151'
ASSET_STRATA = 19
NPC_SLOTS = 24


def read(path):
    return json.loads(Path(path).read_text())


def order_key(ident):
    return (hashlib.sha256((POLICY + ':' + ident).encode()).hexdigest(), ident)


def validate_source(deck):
    """Check all 5K payloads and frozen plans, not just favorable pilot rows."""
    if deck.get('count') != 5000 or len(deck.get('scenes', [])) != 5000:
        raise ValueError('Complete independent 5K candidate required')
    config = deck['config']
    policy = config.get('obstacle_front_support_policy', {})
    if policy.get('policy') != front.POLICY or policy.get('source_sha256') != front.implementation_sha256():
        raise ValueError('Current frozen v151 helper/policy required')
    frozen = Path(config['frozen_specs_path'])
    if sha256(frozen) != config['frozen_specs_sha256']:
        raise ValueError('Frozen full-deck specs changed')
    specs = read(frozen)
    if len(specs) != len(deck['scenes']):
        raise ValueError('Frozen spec count differs from full deck')
    entries, metadata, buckets = {}, {}, defaultdict(list)
    for entry, spec in zip(deck['scenes'], specs):
        ident = entry['id']
        if ident in entries:
            raise ValueError('Duplicate full-deck scene ID')
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ValueError('Full-deck scenario checksum changed: ' + ident)
        meta = read(entry['metadata'])
        if meta['spec'] != spec or meta['split'] != entry['split']:
            raise ValueError('Full-deck frozen spec/split differs: ' + ident)
        plan = front.validate_plan(spec)
        if plan:
            actors = meta['runtime_actors']
            if (moving_actors(meta) or plan['logical_slot_id'] != ident
                    or spec['frozen_population_data_ids'] != [a['data_id'] for a in actors]):
                raise ValueError('Front stratum identity or static population changed')
            actor = actors[plan['ordinal']]
            front.verify_actual(spec, plan['ordinal'], actor['planned_center_distance_m'],
                                actor['actual_ego_role'])
            if plan['selected'] and actor.get('secondary_obstacle_support', {}).get('front_support') != plan:
                raise ValueError('Selected actor lacks its frozen front proof')
            buckets[(plan['data_id'], plan['selected'])].append(ident)
        entries[ident], metadata[ident] = entry, meta
    assets = {data_id for data_id, _ in buckets}
    if len(assets) != ASSET_STRATA or set(buckets) != {(asset, value) for asset in assets for value in (False, True)}:
        raise ValueError('Every one of 19 assets needs selected and legacy strata')
    for asset in assets:
        plans = [metadata[ident]['spec'][front.PLAN_KEY]
                 for selected in (False, True) for ident in buckets[(asset, selected)]]
        if (len({p['asset_stratum_index'] for p in plans}) != len(plans)
                or {p['asset_stratum_count'] for p in plans} != {len(plans)}
                or abs(sum(p['selected'] for p in plans) - sum(not p['selected'] for p in plans)) > 1):
            raise ValueError('Full-deck asset allocation changed')
    return entries, metadata, buckets


def select(deck, previous, historical_quota):
    entries, metadata, buckets = validate_source(deck)
    if previous['source_manifest_sha256'] != deck['config']['obstacle_front_support_policy']['source_manifest_sha256']:
        raise ValueError('Previous registration belongs to another v150 source')
    static, roles = [], {}

    def add(ident, role):
        if ident not in entries or ident in roles or moving_actors(metadata[ident]):
            raise ValueError('Static qualification slot is missing, duplicated or moving')
        static.append(ident)
        roles[ident] = role

    for asset in sorted({asset for asset, _ in buckets}):
        for selected in (True, False):
            ident = min(buckets[(asset, selected)], key=order_key)
            add(ident, dict(kind='front_selected' if selected else 'front_legacy', data_id=asset,
                            selected=selected))
    old_static = {row['id']: row['sha256'] for row in previous['static']}
    repaired = previous['all_changed_slots']
    if len(repaired) != NPC_SLOTS or len(set(repaired)) != NPC_SLOTS:
        raise ValueError('Original 24 repaired NPC slots required')
    for ident in repaired:
        if (ident not in entries or entries[ident]['scenario_sha256'] != old_static.get(ident)
                or metadata[ident].get('parked_npc_support_source', {}).get('geometry_changed') is not True):
            raise ValueError('NPC repaired payload differs from registered v150 source')
        add(ident, dict(kind='parked_npc_repaired_regression'))
    if historical_quota.get('pending') != 0 or historical_quota.get('target') != 64:
        raise ValueError('Terminal historical static64 deficit evidence required')
    failures = {row['id']: row for row in historical_quota.get('cells', [])}
    if len(failures) != historical_quota.get('deficit'):
        raise ValueError('Historical deficit evidence count differs')
    controls = []
    for ident in sorted(failures, key=order_key):
        if ident in entries and ident not in roles and not moving_actors(metadata[ident]):
            controls.append(ident)
            add(ident, dict(kind='historical_deficit_negative_regression', historical_evidence=copy.deepcopy(failures[ident]),
                            new_outcome='PENDING_NATIVE_CAPTURE; never force failure'))
            if len(static) == 64:
                break
    # Prior fixed control order is preserved; remaining full-deck rows provide
    # a deterministic fallback only when the previous registration is smaller.
    fallback = [row['id'] for row in previous['static']] + sorted(entries, key=order_key)
    for ident in fallback:
        if len(static) == 64:
            break
        if ident in entries and ident not in roles and not moving_actors(metadata[ident]):
            controls.append(ident)
            add(ident, dict(kind='fixed_static_regression_control'))
    if len(static) != 64 or len(controls) != 2:
        raise ValueError('Expected 38 asset strata + 24 NPC slots + two controls')
    animated = previous['animated']
    moving = [row['id'] for row in animated]
    if len(moving) != 8 or len(set(moving)) != 8 or set(moving) & set(static):
        raise ValueError('Eight unique independent moving regressions required')
    for row in animated:
        ident = row['id']
        if (ident not in entries or entries[ident]['scenario_sha256'] != row['sha256']
                or not moving_actors(metadata[ident])):
            raise ValueError('Registered moving payload or actual motion changed')
    return entries, metadata, static, moving, controls, roles


def prepare(manifest, previous_registration, historical_quota, output):
    if output.exists():
        raise FileExistsError('New bounded v151 qualification registration required')
    paths = (manifest, previous_registration, historical_quota)
    before = {str(p.resolve()): sha256(p) for p in paths}
    deck, previous, quota = (read(p) for p in paths)
    entries, metadata, static, moving, controls, roles = select(deck, previous, quota)
    registration = dict(policy=POLICY, source_manifest=str(manifest.resolve()),
        source_manifest_sha256=sha256(manifest), front_helper_sha256=front.implementation_sha256(),
        frozen_specs_sha256=deck['config']['frozen_specs_sha256'], input_sha256=before,
        all_changed_slots=list(previous['all_changed_slots']), controls=controls,
        static=[dict(id=ident, sha256=entries[ident]['scenario_sha256'],
                     metadata_sha256=sha256(entries[ident]['metadata']),
                     qualification_role=roles[ident]) for ident in static],
        animated=[dict(id=ident, sha256=entries[ident]['scenario_sha256'],
                       metadata_sha256=sha256(entries[ident]['metadata'])) for ident in moving],
        strata_counts=dict(Counter(role['kind'] for role in roles.values())),
        selection='Full-deck deterministic ID-hash strata, original NPC cohort, historical deficits then fixed controls; chosen before new live outcomes',
        historical_control_limitation='Old failure selects a negative regression only; no new failure is guaranteed or fabricated. All rows need new current-source native capture.',
        live_qualified=False, bulk_started=False)
    for path in paths:
        if sha256(path) != before[str(path.resolve())]:
            raise ValueError('Qualification source changed during registration')
    for name, ids in (('static64', static), ('animated8', moving)):
        subset = copy.deepcopy(deck)
        subset.update(scenes=[entries[ident] for ident in ids], count=len(ids),
            status='BOUNDED_V151_QUALIFICATION_PENDING_NATIVE; not training or bulk')
        atomic_json(output / name / 'manifest.json', subset)
    atomic_json(output / 'registration.json', registration)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'previous-registration', 'historical-quota', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.previous_registration, args.historical_quota, args.output))
