#!/usr/bin/env python3
"""Offline-only repair of long parked NPC footprints, preserving logical slots.

Free static yaw is retained unless the particular long physics vehicle lacks
road support. A road-aligned fallback is explicit per actor, not a class mask.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path

import base5_parked_npc_placement as placement
import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json, sha256
from run_base5_failure_aware_replenishment_fast import prewarm


def prepare(source, output):
    if output.exists():
        raise FileExistsError('New NPC-support candidate output required')
    deck = json.loads(source.read_text())
    config = copy.deepcopy(deck['config'])
    config['parked_npc_placement'] = copy.deepcopy(placement.DEFAULT)
    scripts = Path(__file__).parent
    watched = {str(scripts / name): sha256(scripts / name) for name in
               ('prepare_base5_parked_npc_support.py', 'base5_parked_npc_placement.py',
                'generate_base5_independent.py', 'base5_static_yaw.py', 'base5_spatial_diversity.py',
                'base5_shadow_observation.py', 'base5_avoidance_geometry.py')}
    ctx = None
    entries, specs, changes = [], [], []
    rejections = Counter()
    for entry in deck['scenes']:
        old = json.loads(Path(entry['metadata']).read_text())
        risks = [(i, a) for i, a in enumerate(old['runtime_actors'])
                 if (placement.actor_screen(a, config['parked_npc_placement'], old['spec']) or {}).get('road_supported') is False]
        row = copy.deepcopy(old['spec'])
        if risks:
            row['frozen_population_data_ids'] = [a['data_id'] for a in old['runtime_actors']]
            overrides = row.setdefault('parked_npc_yaw_overrides', {})
            for i, actor in risks:
                if actor.get('static_yaw_policy', {}).get('mode') == 'uniform':
                    overrides[str(i)] = dict(policy=placement.POLICY, data_id=actor['data_id'],
                                             mode='road_aligned', original_yaw=actor['static_yaw_policy'],
                                             reason='original long physics NPC footprint lacks road support')
            if ctx is None:
                ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                                 spatial.gen.geo.DEFAULT_MGEO), config)
            for attempt in range(old['retry'], old['retry'] + 4096):
                try:
                    scene, metadata = spatial.build(row, config, ctx, attempt)
                    if metadata['split'] != old['split']:
                        raise ValueError('NPC support proposal changed split')
                    if [a['data_id'] for a in metadata['runtime_actors']] != row['frozen_population_data_ids']:
                        raise ValueError('NPC support proposal changed original actor list')
                    break
                except ValueError as error:
                    if str(error).startswith(('Unknown ', 'Invalid explicit long NPC', 'Frozen population',
                                              'Placement geometry reference changed')):
                        raise
                    rejections[str(error)] += 1
            else:
                atomic_json(output / 'failure.json', dict(scene_id=entry['id'], completed=len(entries),
                            rejection_counts=dict(rejections), bulk_started=False))
                raise ValueError('NPC support offline proposals exhausted: ' + entry['id'])
        else:
            scene = json.loads(Path(entry['scenario']).read_text())
            metadata = copy.deepcopy(old)
            for actor in metadata['runtime_actors']:
                proof = placement.actor_screen(actor, config['parked_npc_placement'], old['spec'])
                if proof is not None:
                    actor['parked_npc_placement'] = proof
        for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                    'fresh_environment_provenance', 'obstacle_support_pair'):
            if key in old:
                metadata[key] = copy.deepcopy(old[key])
        if risks and metadata.get('obstacle_support_pair'):
            metadata['obstacle_support_pair'].update(actual_retry=metadata['retry'],
                recovery_geometry_changed=True, pair_precision='same identity; NPC support repaired geometry')
        metadata['parked_npc_support_source'] = dict(source_id=entry['id'],
            source_scenario_sha256=entry['scenario_sha256'], geometry_changed=bool(risks),
            source_metadata_sha256=sha256(entry['metadata']), original_risk_ordinals=[i for i, _ in risks],
            explicit_yaw_overrides=copy.deepcopy(row.get('parked_npc_yaw_overrides', {})))
        path = output / 'scenarios' / (entry['id'] + '.json')
        meta_path = output / 'scenarios' / (entry['id'] + '_metadata.json')
        atomic_json(path, scene); atomic_json(meta_path, metadata)
        entries.append(dict(entry, scenario=str(path.resolve()), metadata=str(meta_path.resolve()),
                            scenario_sha256=sha256(path)))
        specs.append(metadata['spec'])
        if risks:
            changes.append(dict(id=entry['id'], family=row['family'], mission=row['mission'],
                                ordinals=[i for i, _ in risks], override_ordinals=list(row.get('parked_npc_yaw_overrides', {})),
                                retry=metadata['retry']))
    if any(sha256(path) != digest for path, digest in watched.items()):
        raise ValueError('NPC generator changed during offline preparation')
    atomic_json(output / 'frozen_specs.json', specs)
    config.update(frozen_specs_path=str((output / 'frozen_specs.json').resolve()),
                  frozen_specs_sha256=sha256(output / 'frozen_specs.json'))
    candidate = copy.deepcopy(deck)
    candidate.update(config=config, scenes=entries, count=len(entries),
                     status='OFFLINE_ONLY_LONG_NPC_SUPPORT_CANDIDATE; not bulk collection')
    atomic_json(output / 'manifest.json', candidate)
    atomic_json(output / 'npc_support_delta.json', dict(source_manifest=str(source), source_sha256=sha256(source),
                manifest_sha256=sha256(output / 'manifest.json'), source_hashes=watched,
                changed_scenes=len(changes), untouched_payloads=len(entries)-len(changes), changes=changes,
                rejection_counts=dict(rejections), assets_preserved=True, native_qa_changed=False, bulk_started=False))
    return output / 'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
