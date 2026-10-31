#!/usr/bin/env python3
"""Preregister 64 static mixed unit layouts and 16 byte-exact warm repeats.

This diagnostic selects only planned metadata, never native outcomes. All
source payloads, actor UIDs and original metadata fields remain unchanged.
Object scale must be explicit 111; omitted vehicle/pedestrian scale is recorded
as serializer-default/no-override, never promoted to a measured mesh assertion.
"""
import argparse
from collections import Counter, defaultdict
import copy
import json
import math
from pathlib import Path
import time

import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
from base5_scale_capture_contract import validate_entry
from build_base5_mr_asset_catalog import DEFAULT_CATALOG

SCHEMA = 'base5_unit_only_stability_registration_v1'
KEY = 'unit_only_stability'
POLICY = 'unit-only-static-mixed-64-layout-16-repeat-v1'
CONTAINERS = ('objectList', 'vehicleList', 'pedestrianList', 'aiPedstrianList')
PROBLEM_ASSETS = ('WoodBox', 'RedBarrel')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def raw_unit_scales(scene):
    """Check every serialized actor, not only the diagnostic target parent."""
    counts = Counter()
    for container in CONTAINERS:
        values = scene.get(container, [])
        require(isinstance(values, list), 'Raw actor container must be a list')
        for actor in values:
            if 'scale' not in actor:
                require(container != 'objectList', 'objectList requires explicit scale111')
                counts[container + '/serializer_default_no_override'] += 1
                continue
            value = actor['scale']
            require(isinstance(value, dict) and all(type(value.get(axis)) in (int, float)
                and math.isfinite(value[axis]) and value[axis] == 1. for axis in 'xyz'),
                'Every explicit raw actor scale must be exactly111')
            for axis in 'xyz':
                if '_'+axis in value:
                    mirror = value['_'+axis]
                    require(type(mirror) in (str,int,float) and math.isfinite(float(mirror))
                        and float(mirror) == value[axis], 'Raw scale UI mirror must agree with111')
            counts[container + '/explicit111'] += 1
    return counts


def reject_scale_metadata(metadata):
    require('asset_scale_plan' not in metadata.get('spec', {})
        and 'scale_probe' not in metadata and 'asset_scale_preparation' not in metadata
        and not any('asset_scale' in actor or 'asset_scale_geometry' in actor
                    for actor in metadata.get('runtime_actors', [])),
        'Use an unscaled ancestor without scale policy, plan, assignment or probe')


def bind_actors(scene, metadata, catalog):
    observed = {}
    for container in CONTAINERS:
        for actor in scene.get(container, []):
            uid, data_id = actor.get('UNIQUEID'), actor.get('DataID')
            require(type(uid) is int and type(data_id) is int and uid not in observed,
                'Raw actor UID/DataID must be explicit and unique')
            observed[uid] = (data_id, container)
    expected = {}
    for actor in metadata['runtime_actors']:
        uid, data_id = actor.get('unique_id'), actor.get('data_id')
        require(type(uid) is int and uid not in expected and data_id in catalog
            and catalog[data_id]['name'] == actor.get('name')
            and catalog[data_id]['source_list'] == actor.get('source_list'),
            'Planned parent identity differs from the authoritative catalog')
        expected[uid] = (data_id, actor['source_list'])
    require(observed == expected, 'Raw actor roster differs from planned parent metadata')


def planned_overlap(metadata):
    """Coarse XY angular envelope overlap only; not sensor visibility/occlusion GT."""
    ego = metadata['ego']
    rows = []
    for actor in metadata['runtime_actors']:
        x, y = actor['geometric_center_estimate'][:2]
        dx, dy = x - ego['x'], y - ego['y']
        distance = math.hypot(dx, dy)
        radius = math.hypot(*actor['estimated_size'][:2]) / 2
        rows.append((distance, math.atan2(dy, dx), math.asin(min(1., radius/max(distance, 1e-9)))))
    for i, (distance, angle, width) in enumerate(rows):
        for other, bearing, half_width in rows[:i]:
            difference = abs((angle-bearing+math.pi) % (2*math.pi)-math.pi)
            if abs(distance-other) > .5 and difference <= width+half_width:
                return 'angular_overlap_proxy'
    return 'separated_angular_proxy'


def features(metadata, physical_assets):
    spec = metadata['spec']
    actors = metadata['runtime_actors']
    result = {'context/'+spec['family']+'/'+('mission' if spec['mission'] else 'generic'),
        'load/'+('2_3' if len(actors) <= 3 else '4_5' if len(actors) <= 5 else '6plus'),
        'occlusion_proxy/'+planned_overlap(metadata)}
    extent = spec.get('static_mission_extent')
    if extent:
        result.add('extent/'+extent['stratum'])
    for index, actor in enumerate(actors):
        if actor['name'] not in physical_assets:
            continue
        name = actor['name']
        yaw = int((actor['placement_yaw_deg'] % 360) // 90)
        distance = actor['planned_center_distance_m']
        band = '0_5m' if distance < 5 else '5_20m' if distance < 20 else '20_30m' if distance < 30 else '30_50m' if distance < 50 else '50plus_m'
        result.update(('asset/'+name, 'yaw/'+str(yaw), 'distance/'+band,
            'ego_role/'+actor.get('actual_ego_role', 'unknown')))
        if name in PROBLEM_ASSETS:
            result.update(('problem_yaw/'+name+'/'+str(yaw),
                'problem_distance/'+name+('/0_20m' if distance < 20 else '/20_30m' if distance < 30 else '/30plus_m'),
                'problem_role/'+name+'/'+('primary' if index == 0 else 'auxiliary')))
    return result


def order_key(label, ident):
    return layout.digest([POLICY, label, ident]), ident


def select(records, physical_assets, target=64, repeats=16):
    require(type(target) is int and target == 64 and type(repeats) is int and repeats == 16,
        'This preregistration is fixed at64 independent layouts plus16 repeats')
    group_of, geometry_hashes, near_pairs = layout.groups(records, layout.configuration())
    indexed = {record['id']: record for record in records}
    require(len(indexed) == len(records), 'Duplicate candidate source IDs')
    covers = {ident: features(record['meta'], physical_assets) for ident, record in indexed.items()}
    frequency = Counter(token for value in covers.values() for token in value)
    minima = {'asset/'+name:16 if name in PROBLEM_ASSETS else 4 for name in physical_assets}
    for token in frequency:
        if token.startswith('problem_yaw/'):
            minima[token] = 2
        elif not token.startswith('asset/'):
            minima[token] = 1
    availability = {name:len({group_of[ident] for ident in indexed if 'asset/'+name in covers[ident]})
        for name in sorted(physical_assets)}
    require(all(availability[name] >= minimum for name, minimum in
        ((name, 16 if name in PROBLEM_ASSETS else 4) for name in physical_assets)),
        'Insufficient independent static mixed geometry inventory for declared asset minima')
    selected, used_groups, counts = [], set(), Counter()
    while len(selected) < target:
        deficits = {token:minimum-counts[token] for token, minimum in minima.items() if counts[token] < minimum}
        choices = [ident for ident in indexed if group_of[ident] not in used_groups
            and (not deficits or covers[ident] & set(deficits))]
        require(choices, 'Fixed64 budget cannot satisfy independent context/asset coverage')
        def score(ident):
            tokens = covers[ident] & set(deficits)
            return (-sum((3 if token.startswith('asset/') else 1)*deficits[token] for token in tokens),
                -sum(1/frequency[token] for token in tokens), order_key('independent', ident))
        ident = min(choices, key=score)
        selected.append(ident)
        used_groups.add(group_of[ident])
        counts.update(covers[ident])
    require(all(counts[token] >= minimum for token, minimum in minima.items()),
        'Declared64 layouts do not meet all outcome-free coverage minima')
    repeated, repeat_counts = [], Counter()
    for _ in range(repeats):
        choices = [ident for ident in selected if ident not in repeated
            and any('asset/'+name in covers[ident] for name in PROBLEM_ASSETS)]
        require(choices, 'Insufficient distinct protected payloads for16 repeats')
        def repeat_score(ident):
            return (-sum(max(0, 8-repeat_counts[name]) for name in PROBLEM_ASSETS if 'asset/'+name in covers[ident]),
                -sum(1/counts[token] for token in covers[ident] if token.startswith(('problem_yaw/', 'problem_distance/'))),
                order_key('repeat', ident))
        ident = min(choices, key=repeat_score)
        repeated.append(ident)
        repeat_counts.update(name for name in PROBLEM_ASSETS if 'asset/'+name in covers[ident])
    require(all(repeat_counts[name] >= 8 for name in PROBLEM_ASSETS), 'Both problem assets require at least8 repeated payloads')
    return selected, repeated, dict(independent_layouts=64, repeat_frames=16,
        physical_assets=sorted(physical_assets), independent_asset_inventory=availability,
        required_feature_minima=minima, selected_feature_counts=dict(sorted(counts.items())),
        repeated_problem_asset_counts=dict(repeat_counts), missing_required_features=[],
        source_near_geometry_pairs=near_pairs, group_of=group_of, geometry_sha256=geometry_hashes,
        native_outcomes_used_for_selection=False,
        occlusion_limitation='XY angular envelope proxy only, not actual sensor occlusion, mask support or native GT.')


def prepare(source, reference_path, output, *, source_sha256, reference_sha256):
    source, reference_path, output = map(lambda p:Path(p).resolve(), (source, reference_path, output))
    require(not output.exists(), 'Fresh unit-only diagnostic output required')
    import generate_dynamic_object_capture_set as geo
    runtime_dir = geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    require(all(output != p and p not in output.parents for p in (source.parent, reference_path.parent, runtime_dir)),
        'Diagnostic output cannot be inside preserved source, references or MORAI runtime')
    require(sha256(source) == source_sha256 and sha256(reference_path) == reference_sha256,
        'Explicit source/unit reference SHA differs')
    reference = dict(path=str(reference_path), sha256=reference_sha256)
    review = unit.load(reference)
    catalog = {asset['data_id']:asset for asset in read(DEFAULT_CATALOG)['assets']}
    physical = {asset['name'] for asset in catalog.values() if asset.get('semantic_class') == 'obstacle'}
    require(len(physical) == 19 and set(review['assets']) == physical,
        'Exactly all19 catalog physical-obstacle unit references are required')
    deck = read(source)
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000
        and 'asset_scale_policy' not in deck['config']
        and 'asset_scale_probe' not in deck['config'].get('capture', {})
        and unit.KEY not in deck['config'] and not deck.get('split_overlay'),
        'An exact unscaled plain full5K ancestor is required')
    source_pins = {str(source):source_sha256, str(reference_path):reference_sha256,
        str(DEFAULT_CATALOG.resolve()):sha256(DEFAULT_CATALOG)}
    for item in [review['catalog_reference'], *review['sources']]:
        source_pins[str(Path(item['path']).resolve())] = item['sha256']
    runtime = pins.collect_runtime(entrypoints=pins.ENTRYPOINTS+(Path(__file__).name,))
    records, entries, metadata = [], {}, {}
    scale_counts, contexts = Counter(), Counter()
    for entry in deck['scenes']:
        ident = entry['id']
        require(ident not in entries, 'Repeated source scene identity')
        require(sha256(entry['scenario']) == entry['scenario_sha256'], 'Source scenario changed')
        mp = Path(entry['metadata']).resolve()
        meta, scene = read(mp), read(entry['scenario'])
        require(entry['split'] == meta['split'] and unit.KEY not in meta and KEY not in meta
            and 'training_split_overlay' not in meta, 'Plain source metadata/split contract differs')
        reject_scale_metadata(meta)
        scale_counts.update(raw_unit_scales(scene))
        bind_actors(scene, meta, catalog)
        source_pins[str(Path(entry['scenario']).resolve())] = entry['scenario_sha256']
        source_pins[str(mp)] = sha256(mp)
        entries[ident], metadata[ident] = entry, meta
        if len(meta['runtime_actors']) < 2 or any(actor['motion'] not in ('static', 'standing') for actor in meta['runtime_actors']):
            continue
        if not any(actor['name'] in physical for actor in meta['runtime_actors']):
            continue
        records.append(dict(id=ident, meta=meta, scenario_sha256=entry['scenario_sha256']))
        contexts[meta['spec']['family']+'/'+('mission' if meta['spec']['mission'] else 'generic')] += 1
    independent, repeated, selection = select(records, physical)
    group_of, geometry = selection.pop('group_of'), selection.pop('geometry_sha256')
    selection.update(pool_static_mixed_scenes=len(records), pool_context_scenes=dict(contexts))
    registered_at = time.time()
    chosen, manifest_entries = [], []
    independent_ids = {ident:f'unitonly_{index:03d}_{ident}' for index,ident in enumerate(independent)}
    for index, ident in enumerate(independent+repeated):
        original = entries[ident]
        phase = 'independent' if index < 64 else 'repeat'
        diagnostic_id = independent_ids[ident] if phase == 'independent' else f'unitonly_repeat_{index-64:03d}_{ident}'
        tag = dict(source_id=ident, source_scenario_sha256=original['scenario_sha256'],
            source_metadata_path=str(Path(original['metadata']).resolve()),
            source_metadata_sha256=source_pins[str(Path(original['metadata']).resolve())],
            independent_group_id=group_of[ident], geometry_sha256=geometry[ident], phase=phase,
            sequence_ordinal=index, independent_ordinal=independent.index(ident),
            repeat_of_id=independent_ids[ident] if phase == 'repeat' else None,
            repeat_ordinal=1 if phase == 'repeat' else 0, payload_reused_exact=phase == 'repeat',
            native_outcomes_used_for_selection=False)
        meta = copy.deepcopy(metadata[ident])
        meta[unit.KEY], meta[KEY] = reference, tag
        mp = output/'metadata'/(diagnostic_id+'.json')
        atomic_json(mp, meta)
        manifest_entries.append(dict(original, id=diagnostic_id, metadata=str(mp)))
        chosen.append(dict(id=diagnostic_id, scenario_sha256=original['scenario_sha256'],
            metadata_sha256=sha256(mp), scenario=str(Path(original['scenario']).resolve()),
            metadata=str(mp), **tag))
    registration = dict(schema=SCHEMA, policy=POLICY, registered_at_unix=registered_at,
        source_manifest=str(source), source_manifest_sha256=source_sha256,
        unit_reference=reference, runtime_hashes=runtime, input_hashes=source_pins,
        selected=chosen, selection=selection, full_source_scale_counts=dict(scale_counts),
        scale_absence_policy='objectList explicit111 required; absent scale in vehicle/pedestrian/aiPedstrian means serializer-default no-override, not measured mesh proof',
        ui_scale_mirrors_checked_when_present=True,
        full_source_actor_bindings_verified=True, source_frames_checked=5000,
        full_source_scale_policy_assignment_or_probe_found=False,
        independent_frames=64, repeated_frames=16, count=80,
        independent_layouts=64, repeat_frames=16, planned_frames=80,
        native_outcomes_used_for_selection=False, raw_modified=False, capture_started=False,
        bulk_ready=False, training_started=False,
        session_plan=[dict(start_index=0,stop_index_exclusive=64,frames=64),
                      dict(start_index=64,stop_index_exclusive=80,frames=16)])
    result = copy.deepcopy(deck)
    result['config'][unit.KEY] = reference
    result.update(count=80, scenes=manifest_entries, dataset_id='base5_unit_only_stability_261017',
        status='PREREGISTERED_UNIT_ONLY_64_LAYOUTS_16_REPEATS; no bulk or training approval',
        unit_only_stability_registration=registration, bulk_ready=False)
    for entry in manifest_entries:
        validate_entry(entry,result)
        unit.validate_entry(entry,result)
        actual = read(entry['metadata'])
        preserved = {key:value for key,value in actual.items() if key not in (unit.KEY,KEY)}
        require(preserved == metadata[actual[KEY]['source_id']], 'Original metadata fields changed')
        require(sha256(entry['scenario']) == entry['scenario_sha256'], 'Exact diagnostic payload changed')
    pins.verify_hashes(source_pins, 'Unit-only source/reference inventory')
    pins.verify_hashes(runtime, 'Unit-only preparer/runtime')
    atomic_json(output/'manifest.json',result)
    atomic_json(output/'registration.json',dict(registration,
        manifest=str(output/'manifest.json'),manifest_sha256=sha256(output/'manifest.json')))
    return output/'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','unit-reference','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--source-sha256',required=True)
    parser.add_argument('--unit-reference-sha256',required=True)
    args = parser.parse_args()
    print(prepare(args.manifest,args.unit_reference,args.output,
        source_sha256=args.source_sha256,reference_sha256=args.unit_reference_sha256))
