#!/usr/bin/env python3
"""Prepare a new offline-only v151 deck from a frozen v145/v150 candidate.

No MORAI runtime directory, raw capture, old manifest or selected ledger is
written. Allocate once over the complete source deck; subset pilots and failed
scene replacements must reuse the resulting frozen spec, never rebalance.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import shutil
import time

import base5_obstacle_front_support as front
import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json, sha256
from base5_obstacle_support_policy import verify_protected_near
from run_base5_failure_aware_replenishment_fast import prewarm


def _read(path):
    return json.loads(Path(path).read_text())


def preservation(before, after, selected):
    """CPU contract proof, not native visibility, collision or contact QA."""
    old, new = before['runtime_actors'], after['runtime_actors']
    ids = lambda actors: [a['data_id'] for a in actors]
    expected = copy.deepcopy(before['spec'])
    if front.PLAN_KEY in after['spec']:
        expected[front.PLAN_KEY] = copy.deepcopy(after['spec'][front.PLAN_KEY])
    if after['spec'] != expected:
        raise ValueError('Front support modified an original frozen spec field')
    if ids(new) != ids(old) or len(new) != len(old):
        raise ValueError('Front support modified original actor identities/count')
    if [a['motion'] for a in new] != [a['motion'] for a in old]:
        raise ValueError('Front support modified original actor motion')
    if after['split'] != before['split']:
        raise ValueError('Front support modified split')
    for ordinal, actor in enumerate(new):
        verify_protected_near(after['spec'], ordinal, actor['planned_center_distance_m'])
        front.verify_actual(after['spec'], ordinal, actor['planned_center_distance_m'],
                            actor['actual_ego_role'])
    if selected:
        ordinal = after['spec'][front.PLAN_KEY]['ordinal']
        proof = new[ordinal].get('secondary_obstacle_support', {}).get('front_support')
        if proof != after['spec'][front.PLAN_KEY]:
            raise ValueError('Front support actor lacks frozen plan proof')
    return dict(actor_count=len(new), actor_ids=ids(new), split=after['split'],
                primary_and_original_spec_preserved=True,
                protected_near_mid_preserved=True, native_qa_changed=False)


def prepare(source, output, maximum_attempts=4096):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError('New front-support candidate output required')
    if not 1 <= maximum_attempts <= 4096:
        raise ValueError('Offline proposal bound must be between 1 and 4096')
    deck = _read(source)
    if not deck['config'].get('obstacle_support_policy'):
        raise ValueError('Frozen obstacle-support source deck required')
    scripts = Path(__file__).parent
    watched = {str(scripts / name): sha256(scripts / name) for name in
               ('prepare_base5_obstacle_front_support.py', 'base5_obstacle_front_support.py',
                'base5_obstacle_support_policy.py', 'base5_obstacle_rich_floor.py',
                'generate_base5_independent.py', 'base5_spatial_diversity.py',
                'base5_placement_geometry.py', 'base5_static_yaw.py',
                'base5_population_budget.py', 'base5_parked_npc_placement.py')}
    source_manifest_hash = sha256(source)
    originals = {}
    source_hashes = {}
    for entry in deck['scenes']:
        if entry['id'] in originals:
            raise ValueError('Duplicate source scene identity')
        actual_hash = sha256(entry['scenario'])
        if actual_hash != entry['scenario_sha256']:
            raise ValueError('Source scenario checksum mismatch')
        originals[entry['id']] = _read(entry['metadata'])
        source_hashes[entry['id']] = dict(scenario_sha256=actual_hash,
                                         metadata_sha256=sha256(entry['metadata']))
    rows = front.assign_plans((entry['id'], originals[entry['id']]['spec'],
                               originals[entry['id']]['runtime_actors'])
                              for entry in deck['scenes'])
    if not any(row.get(front.PLAN_KEY, {}).get('selected') for row in rows.values()):
        raise ValueError('Source deck contains no selected added-actor front support')
    config = copy.deepcopy(deck['config'])
    config['obstacle_front_support_policy'] = dict(
        policy=front.POLICY, base_policy=front.BASE_POLICY, assignment=front.ASSIGNMENT,
        source_manifest=str(source.resolve()), source_manifest_sha256=source_manifest_hash,
        source_sha256=front.implementation_sha256(),
        scope='existing added actors only; primary/original population unchanged',
        fresh_and_replacement='reuse frozen full-deck plan; never rebalance subsets',
        bulk_started=False)
    output.mkdir(parents=True)
    context, entries, specs = None, [], []
    changes, rejections = [], Counter()
    started = time.monotonic()
    selection_counts, asset_counts = Counter(), {}
    for index, entry in enumerate(deck['scenes']):
        old = originals[entry['id']]
        row = rows[entry['id']]
        plan = front.validate_plan(row)
        selected = bool(plan and plan['selected'])
        if plan:
            selection_counts['annotated_added_actors'] += 1
            selection_counts['selected_added_actors' if selected else 'legacy_added_actors'] += 1
            key = str(plan['data_id'])
            asset_counts.setdefault(key, Counter())['selected' if selected else 'legacy'] += 1
        if selected:
            if context is None:
                context = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                                     spatial.gen.geo.DEFAULT_MGEO), config)
            for attempt in range(old['retry'], old['retry'] + maximum_attempts):
                try:
                    scene, metadata = spatial.build(row, config, context, attempt)
                    preservation(old, metadata, selected=True)
                    break
                except ValueError as error:
                    # A changed frozen policy is not a placement miss. The
                    # actual-role/band check may legitimately reject a pose.
                    front.validate_plan(row)
                    if str(error).startswith(('Unknown ', 'Frozen ', 'Front support asset/',
                                              'Placement geometry reference changed',
                                              'Front support modified ',
                                              'Front support actor lacks ')):
                        raise
                    rejections[str(error)] += 1
            else:
                atomic_json(output / 'failure.json', dict(scene_id=entry['id'],
                    completed=len(entries), rejection_counts=dict(rejections),
                    maximum_attempts=maximum_attempts, bulk_started=False))
                raise ValueError('Bounded front-support proposal exhausted: ' + entry['id'])
        else:
            scene, metadata = None, copy.deepcopy(old)
            metadata['spec'] = copy.deepcopy(row)
            preservation(old, metadata, selected=False)
        for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                    'fresh_environment_provenance', 'obstacle_support_pair',
                    'parked_npc_support_source'):
            if key in old:
                metadata[key] = copy.deepcopy(old[key])
        metadata['obstacle_front_support_pair'] = dict(
            source_id=entry['id'], source_scenario_sha256=entry['scenario_sha256'],
            source_metadata_sha256=source_hashes[entry['id']]['metadata_sha256'],
            original_retry=old['retry'], actual_retry=metadata['retry'],
            selected=selected, geometry_changed=selected,
            limitation='Same identities/spec; selected rebuild may move original poses within quotas')
        scenario_path = output / 'scenarios' / (entry['id'] + '.json')
        metadata_path = output / 'scenarios' / (entry['id'] + '_metadata.json')
        scenario_path.parent.mkdir(parents=True, exist_ok=True)
        if selected:
            atomic_json(scenario_path, scene)
        else:
            shutil.copyfile(entry['scenario'], scenario_path)
            if sha256(scenario_path) != entry['scenario_sha256']:
                raise ValueError('Unselected original payload is not byte-equivalent')
        atomic_json(metadata_path, metadata)
        entries.append(dict(entry, scenario=str(scenario_path.resolve()),
                            metadata=str(metadata_path.resolve()),
                            scenario_sha256=sha256(scenario_path)))
        specs.append(copy.deepcopy(metadata['spec']))
        if selected:
            before = old['runtime_actors'][plan['ordinal']]
            after = metadata['runtime_actors'][plan['ordinal']]
            changes.append(dict(id=entry['id'], data_id=plan['data_id'], asset=after['name'],
                ordinal=plan['ordinal'], before_role=before['actual_ego_role'],
                after_role=after['actual_ego_role'],
                before_distance_m=before['planned_center_distance_m'],
                after_distance_m=after['planned_center_distance_m'],
                original_retry=old['retry'], actual_retry=metadata['retry']))
        if (index + 1) % 100 == 0:
            print('front-support', index + 1, 'selected', len(changes),
                  'elapsed_sec', round(time.monotonic() - started, 1), flush=True)
    if sha256(source) != source_manifest_hash:
        raise ValueError('Source front-support manifest changed during preparation')
    for entry in deck['scenes']:
        if (sha256(entry['scenario']) != source_hashes[entry['id']]['scenario_sha256']
                or sha256(entry['metadata']) != source_hashes[entry['id']]['metadata_sha256']):
            raise ValueError('Source front-support payload changed during preparation')
    if any(sha256(path) != digest for path, digest in watched.items()):
        raise ValueError('Front-support generator changed during offline preparation')
    frozen = output / 'frozen_specs.json'
    atomic_json(frozen, specs)
    config.update(frozen_specs_path=str(frozen.resolve()), frozen_specs_sha256=sha256(frozen))
    candidate = copy.deepcopy(deck)
    candidate.update(config=config, scenes=entries, count=len(entries),
                     status='OFFLINE_ONLY_FRONT_SUPPORT_V151; live qualification pending; no bulk collection')
    candidate.setdefault('provenance', {})['obstacle_front_support_source_hashes'] = watched
    manifest = output / 'manifest.json'
    atomic_json(manifest, candidate)
    atomic_json(output / 'front_support_delta.json', dict(
        policy=front.POLICY, source_manifest=str(source.resolve()),
        source_manifest_sha256=source_manifest_hash, manifest_sha256=sha256(manifest),
        source_hashes=watched, frames=len(entries), changed_scenes=len(changes),
        untouched_payloads=len(entries) - len(changes), selection_counts=dict(selection_counts),
        asset_selection_counts={key: dict(value) for key, value in asset_counts.items()},
        rejection_counts=dict(rejections), changes=changes,
        preserved=dict(primary=True, original_actor_ids_count=True, seed=True,
                       near_mid_bands=True, mission=True, split=True, free_yaw=True),
        live_qualified=False, bulk_started=False, elapsed_sec=time.monotonic() - started,
        limitation='Offline CPU pose/identity evidence, not sensor support, native contact QA or model AP'))
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--maximum-attempts', type=int, default=4096)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output, args.maximum_attempts))
