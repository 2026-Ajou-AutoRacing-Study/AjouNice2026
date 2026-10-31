#!/usr/bin/env python3
"""Preregister eight changed-path regressions, never a bulk collection."""
import argparse
import copy
import json
from pathlib import Path

import base5_shadow_ground_reuse as reuse
from base5_capture_safety import atomic_json, sha256
from base5_front_release_inputs import collect_runtime


POLICY = 'precollection-native-differential-smoke-v154'


def select(deck, metadata, registry):
    chosen, used = [], set()
    def choose(label, predicate):
        values = [e for e in deck['scenes'] if e['id'] not in used and predicate(metadata[e['id']])]
        if not values:
            raise ValueError('No source for required smoke stratum: ' + label)
        entry = min(values, key=lambda e:(reuse.digest([POLICY, label, e['id']]), e['id']))
        used.add(entry['id']); chosen.append((label, entry))
    def primary(meta):
        return meta.get('runtime_actors', [{}])[0] if meta.get('runtime_actors') else {}
    def ordinary(meta):
        return (meta['spec']['family'] == 'obstacle' and not meta['spec']['mission']
                and not meta['spec'].get('avoidance_phase') and not meta['spec'].get('shadow_phase')
                and meta['spec']['motion'] == 'static')
    choose('low_obstacle_near', lambda m:ordinary(m)
        and primary(m).get('name') == 'CargoBox'
        and 6 <= primary(m).get('planned_center_distance_m', 0) < 20)
    choose('wide_obstacle_mid', lambda m:ordinary(m)
        and primary(m).get('name') in ('concrete-barrier', 'Concrete_Barricade01', 'Steel_Barricade')
        and 20 <= primary(m).get('planned_center_distance_m', 0) < 30)
    choose('stroller_components', lambda m:primary(m).get('name', '').startswith('BabyStroller')
        and primary(m).get('motion') == 'static'
        and not m['spec'].get('shadow_phase') and primary(m)['planned_center_distance_m'] < 20)
    choose('pet_components', lambda m:primary(m).get('name', '').startswith('Ped_Pet')
        and primary(m).get('motion') in ('standing', 'static')
        and not m['spec'].get('shadow_phase') and primary(m)['planned_center_distance_m'] < 20)
    for motion in ('walking', 'running'):
        choose('intersection_'+motion, lambda m, motion=motion:
            not m['spec'].get('shadow_phase')
            and any(a.get('motion') == motion for a in m.get('runtime_actors', [])))
    by_id = {entry['id']:entry for entry in deck['scenes']}
    exact = sorted((r for r in registry['entries'] if r['mode'] == 'exact_source'), key=lambda r:r['source_id'])
    if len(exact) != 2:
        raise ValueError('Two explicit exact-source reuse entries required for this smoke')
    for row in exact:
        if row['source_id'] not in by_id or row['source_id'] in used:
            raise ValueError('Duplicate/missing registered shadow smoke source')
        chosen.append(('shadow_reuse_'+metadata[row['source_id']]['runtime_actors'][0]['name'],
                       by_id[row['source_id']]))
        used.add(row['source_id'])
    if len(chosen) != 8 or len(used) != 8:
        raise ValueError('Eight independent smoke source slots required')
    return chosen


def prepare(manifest, output):
    manifest, output = Path(manifest).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh smoke-registration output required')
    deck = json.loads(manifest.read_text())
    if deck.get('count') != 5000 or len(deck['scenes']) != 5000:
        raise ValueError('Full 5K source required')
    pins = reuse.validate_manifest(deck)
    registration = deck['config'][reuse.KEY]['registration']
    registry = json.loads(Path(registration['path']).read_text())
    metadata = {}
    for entry in deck['scenes']:
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ValueError('Smoke source scenario changed')
        metadata[entry['id']] = json.loads(Path(entry['metadata']).read_text())
    selected = select(deck, metadata, registry)
    subset = copy.deepcopy(deck)
    subset.update(scenes=[copy.deepcopy(e) for _, e in selected], count=8,
                  status='REGISTERED_NATIVE_SMOKE_ONLY; no bulk or training')
    atomic_json(output/'manifest.json', subset)
    runtime = collect_runtime()
    atomic_json(output/'registration.json', dict(policy=POLICY,
        source_manifest=str(manifest), source_manifest_sha256=sha256(manifest),
        subset_manifest_sha256=sha256(output/'manifest.json'), runtime_hashes=runtime,
        reuse_input_hashes=pins, helper_sha256=sha256(__file__),
        selected=[dict(stratum=label, id=e['id'], scenario_sha256=e['scenario_sha256'],
            metadata_sha256=sha256(e['metadata'])) for label,e in selected],
        bulk_started=False, training_started=False, historical_raw_reused_as_training=False))
    return output/'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
