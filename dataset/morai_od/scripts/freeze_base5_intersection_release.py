#!/usr/bin/env python3
"""Generate a fresh offline 5K contract; never installs scenes or starts MR capture."""
import argparse
from collections import Counter
import copy
import json
import math
from pathlib import Path

import generate_base5_independent as gen
import base5_intersection_placement as placement
from base5_capture_safety import atomic_json, sha256
from base5_intersection_ground import load_lookup


def freeze(lookup_path, output, base_config):
    if output.exists():
        raise FileExistsError('Fresh release output required')
    lookup = load_lookup(lookup_path)
    if lookup['status'] != 'LIVE_VALIDATED':
        raise ValueError('Freeze requires live-validated heights')
    config = gen.geo.load_json(base_config)
    config['dataset_id'] = 'base5_independent_5k_sunny13_editor_ground_v116'
    config['capture'].update(dynamic_region=placement.SCOPE, natural_setup_budget_sec=6.,
        intersection_ground_lookup=dict(path=str(lookup_path.resolve()), sha256=sha256(lookup_path),
                                        require_covered_moving_paths=True))
    output.mkdir(parents=True)
    atomic_json(output/'config.json', config)
    manifest_path = gen.generate(config, output/'scenarios_5k', 0, config['target_scenes'])
    deck = gen.geo.load_json(manifest_path)
    families, primary, motions, directions, phases, attempts = Counter(), Counter(), Counter(), Counter(), Counter(), Counter()
    static = moving = 0
    candidates = []
    for entry in deck['scenes']:
        meta = gen.geo.load_json(Path(entry['metadata'])); actors = meta['runtime_actors']
        families[entry['family']] += 1
        if actors:
            primary[actors[0]['name']] += 1
        movers = [a for a in actors if a['motion'] not in ('standing', 'static')]
        attempts[str(meta['retry'])] += 1
        if movers:
            if not placement.ready(meta) or not meta['spec'].get('dynamic_observation_phase'):
                raise ValueError('Unprepared/out-of-region moving scene in release')
            moving += 1
            for actor in movers:
                path = actor['motion_path']
                axis = lookup['profiles'][path['ground_path_evidence']['profile_key']]['axis']
                yaw = math.radians(path['departure_yaw_deg'])
                sign = 1 if math.cos(yaw)*axis[0]+math.sin(yaw)*axis[1] > 0 else -1
                directions[str((actor['mission_crossing_id'], actor['motion'], sign))] += 1
                motions[actor['name']] += 1
            phases[meta['spec']['dynamic_observation_phase']] += 1
        else:
            static += 1
        candidates.append((entry, meta, movers))
    if families != Counter(config['families']) or min(primary.values()) < config['coverage_policy']['minimum_primary_scenes_per_asset']:
        raise ValueError('Family/primary asset coverage regressed')
    # Select before live outcomes, covering walking/running, crossing/clearing,
    # animal/pet and two-mover scenes plus normal/generic/static observations.
    predicates = [
        lambda e,m,a: len(a) >= 2,
        lambda e,m,a: bool(a) and a[0]['motion'] == 'running' and a[0]['mission_crossing_id'] == 'CW000002',
        lambda e,m,a: bool(a) and a[0]['motion'] == 'running' and a[0]['mission_crossing_id'] == 'CW000000',
        lambda e,m,a: bool(a) and m['spec']['subfamily'] == 'animal',
        lambda e,m,a: bool(a) and m['spec']['subfamily'] == 'pet_person',
        lambda e,m,a: bool(a) and m['spec']['dynamic_observation_phase'] == 'clearing',
        lambda e,m,a: bool(a) and m['spec']['dynamic_observation_phase'] == 'yielding',
        lambda e,m,a: bool(a) and m['spec']['dynamic_observation_phase'] == 'approach',
        lambda e,m,a: not a and e['family'] == 'pedestrian_related' and not e['mission'] and m['spec']['motion'] != 'standing',
        lambda e,m,a: not a and e['family'] == 'obstacle' and bool(m.get('avoidance_observation')),
        lambda e,m,a: not a and e['family'] == 'highway',
        lambda e,m,a: not a and e['family'] == 'obstacle' and not e['mission'] and len(m['runtime_actors']) >= 3,
    ]
    chosen, indices = [], []
    for predicate in predicates:
        for index, (entry, meta, movers) in enumerate(candidates):
            if index not in indices and predicate(entry, meta, movers):
                indices.append(index); chosen.append(entry); break
        else:
            raise ValueError('Representative pilot stratum unavailable')
    pilot = copy.deepcopy({k:v for k,v in deck.items() if k != 'scenes'})
    pilot.update(count=len(chosen), scenes=chosen, status='REPRESENTATIVE_RELEASE_PILOT_PENDING_LIVE')
    atomic_json(output/'pilot_manifest.json', pilot)
    atomic_json(output/'offline_audit.json', dict(status='OFFLINE_GENERATED_NOT_BULK_CAPTURED',
        scenes=len(deck['scenes']), static_scenes=static, moving_scenes=moving, families=dict(families),
        primary_assets=len(primary), primary_minimum=min(primary.values()), primary_counts=dict(primary),
        moving_asset_counts=dict(motions), moving_phases=dict(phases), moving_direction_counts=dict(directions),
        generation_retry_counts=dict(attempts), selected_pilot_indices=indices,
        lookup_sha256=sha256(lookup_path), manifest_sha256=sha256(manifest_path),
        limitation='Offline geometry/quota audit, not bulk terrain/visibility or capture success certification.'))
    return manifest_path


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lookup', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--config', type=Path, default=gen.geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_independent_5k_sunny13.json')
    a = p.parse_args(); print(freeze(a.lookup, a.output, a.config))
