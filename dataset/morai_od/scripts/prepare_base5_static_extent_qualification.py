#!/usr/bin/env python3
"""Preregister 16 static-extent native checks before observing their outcomes."""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path

import audit_base5_scenarios
import base5_front_release_inputs as pins
import base5_static_mission_extent as extent
import generate_base5_independent as gen
import prepare_base5_front_support_qualification as qualification
from base5_capture_safety import atomic_json, sha256
from prepare_base5_far_vehicle_qualification import source_audit_pins


POLICY = 'stationary-mission-stratum-native-pilot-v155'
PHASES = ('approach', 'entry', 'alongside', 'clearing')
POOLS = ('vehicle', 'static_vehicle', 'obstacle', 'person', 'dummy', 'animal',
         'pet_person', 'stroller', 'rider')
SLOTS = [(name, split, phase) for name, split in (
    ('before', 'train'), ('intersection', 'val'), ('after', 'train'), ('after', 'val'))
    for phase in PHASES]


def read(path):
    return json.loads(Path(path).read_text())


def select(metadata):
    """Cover all route/phase cells and all nine source pools without live ranking."""
    groups = {slot: [] for slot in SLOTS}
    for ident, meta in metadata.items():
        row = meta.get('spec', {})
        plan = row.get(extent.PLAN_KEY)
        if not plan:
            continue
        key = (plan['stratum'], meta['split'], row['avoidance_phase'])
        if key not in groups or not extent.eligible(row):
            raise extent.ExtentContractError('Foreign static extent source cell')
        if row.get('stationary_mission_pool') not in POOLS:
            raise extent.ExtentContractError('Unknown stationary primary pool')
        groups[key].append((ident, meta))
    if sum(map(len, groups.values())) != 600 or any(not values for values in groups.values()):
        raise extent.ExtentContractError('Complete 600-slot source and all 16 route/split/phase cells required')
    available = {pool: [slot for slot in SLOTS if any(
        meta['spec']['stationary_mission_pool'] == pool for _, meta in groups[slot])]
        for pool in POOLS}
    if any(not values for values in available.values()):
        raise extent.ExtentContractError('Pilot must cover all nine stationary asset pools')
    assigned = {}

    def match(pool, seen):
        for slot in sorted(available[pool], key=lambda value: (extent.digest([POLICY, pool, value]), value)):
            if slot in seen:
                continue
            seen.add(slot)
            if slot not in assigned or match(assigned[slot], seen):
                assigned[slot] = pool
                return True
        return False

    for pool in sorted(POOLS, key=lambda value: (len(available[value]), value)):
        if not match(pool, set()):
            raise extent.ExtentContractError('Nine source pools cannot fit the fixed native pilot cells')
    selected, pool_counts, used_assets = [], Counter(), set()
    for slot in SLOTS:
        candidates = groups[slot]
        if slot in assigned:
            candidates = [(ident, meta) for ident, meta in candidates
                if meta['spec']['stationary_mission_pool'] == assigned[slot]]
        ident, meta = min(candidates, key=lambda item: (
            item[1]['runtime_actors'][0]['data_id'] in used_assets,
            pool_counts[item[1]['spec']['stationary_mission_pool']],
            extent.digest([POLICY, slot, item[0]]), item[0]))
        primary = meta['runtime_actors'][0]
        label = '_'.join(slot)
        selected.append(dict(stratum=label, route_stratum=slot[0], split=slot[1], phase=slot[2],
            primary_pool=meta['spec']['stationary_mission_pool'], primary_asset=primary['name'],
            primary_data_id=primary['data_id'], id=ident))
        pool_counts[meta['spec']['stationary_mission_pool']] += 1
        used_assets.add(primary['data_id'])
    if len({row['id'] for row in selected}) != 16 or set(pool_counts) != set(POOLS):
        raise extent.ExtentContractError('Pilot source identity/pool coverage differs')
    return selected


def prepare(manifest, output):
    manifest, output = Path(manifest).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('New static extent qualification output required')
    if output == manifest.parent or manifest.parent in output.parents:
        raise ValueError('Pilot output must remain outside the frozen source candidate')
    deck = read(manifest)
    source_checks = source_audit_pins(manifest)
    inputs = pins.collect_inputs(manifest)
    entries, metadata, _ = qualification.validate_source(deck)
    selected = select(metadata)
    ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
    for row in selected:
        meta = copy.deepcopy(metadata[row['id']])
        proof = copy.deepcopy(meta.get(extent.PLAN_KEY))
        extent.finish(gen, ctx, meta, deck['config'])
        if meta[extent.PLAN_KEY] != proof:
            raise extent.ExtentContractError('Pilot source extent proof is stale or forged')
    current_audit = audit_base5_scenarios.audit(manifest)
    if current_audit.get('status') != 'OFFLINE_PASS' or current_audit.get('errors'):
        raise extent.ExtentContractError('Source full scenario audit no longer passes')
    runtime = pins.collect_runtime(entrypoints=pins.ENTRYPOINTS+(Path(__file__).name,))
    subset = copy.deepcopy(deck)
    subset.update(count=16, scenes=[copy.deepcopy(entries[row['id']]) for row in selected],
        status='REGISTERED_STATIC_EXTENT_NATIVE_PILOT_ONLY; no bulk or training')
    atomic_json(output/'manifest.json', subset)
    atomic_json(output/'source_audit.json', current_audit)
    for row in selected:
        entry = entries[row['id']]
        row.update(scenario_sha256=entry['scenario_sha256'], metadata_sha256=sha256(entry['metadata']))
    pins.verify_hashes(inputs)
    pins.verify_hashes(source_checks)
    pins.verify_hashes(runtime)
    atomic_json(output/'registration.json', dict(schema='base5_static_mission_extent_qualification_v1',
        policy=POLICY, helper_sha256=sha256(__file__), source_manifest=str(manifest),
        source_manifest_sha256=inputs[str(manifest)], subset_manifest_sha256=sha256(output/'manifest.json'),
        input_hashes=inputs, source_audit_hashes=source_checks, runtime_hashes=runtime,
        selected=selected, frames=16, primary_pool_counts=dict(Counter(row['primary_pool'] for row in selected)),
        traffic_signal_interaction_constraint=False, native_qa_unchanged=True,
        bulk_started=False, training_started=False, live_qualified=False))
    return output/'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
