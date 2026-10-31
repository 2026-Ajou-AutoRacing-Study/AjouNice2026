#!/usr/bin/env python3
"""UID-only four-scene counterfactual of the preserved all-Red-unit mixed deck.

No physical pose, size, scale, non-object UID, sensor, raw or GT is changed.
Object identities, ground-required IDs and normal scale plans are rebound.
Fresh IDs test UID dependence; they do not prove a cache mechanism or recovery.
"""
import argparse
import copy
import json
from pathlib import Path

import base5_asset_scale as scale
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
from base5_layout_split import digest, geometry
from base5_scale_capture_contract import validate_entry
from prepare_base5_scale_qualification import fresh_output


def read(path):
    return json.loads(Path(path).read_text())


def scenario_uids(value):
    result = []
    if isinstance(value, dict):
        for key, child in value.items():
            if key in ('UNIQUEID', 'm_SensorUniqueID'):
                result.append(child)
            result.extend(scenario_uids(child))
    elif isinstance(value, list):
        for child in value:
            result.extend(scenario_uids(child))
    return result


def remap_objects(scene, metadata, uid_base, scene_ordinal, policy):
    """Return a fresh identity without moving or resizing any physical actor."""
    if (type(uid_base) is not int or type(scene_ordinal) is not int or not 0 <= scene_ordinal < 4
            or not 10000 <= uid_base or uid_base + 399 > 65535):
        raise ValueError('Four bounded positive uint16-compatible UID bands required')
    actors = metadata.get('runtime_actors')
    objects = scene.get('objectList')
    if (not isinstance(actors, list) or not isinstance(objects, list) or not 1 <= len(actors) < 100
            or len(actors) != len(objects) or any(actor['source_list'] != 'objectList' for actor in actors)):
        raise ValueError('Exact mixed physical objectList/runtime actor population required')
    old_ids = [actor['unique_id'] for actor in actors]
    object_ids = [actor['UNIQUEID'] for actor in objects]
    if (any(type(uid) is not int for uid in old_ids) or len(set(old_ids)) != len(old_ids)
            or len(set(object_ids)) != len(object_ids) or set(old_ids) != set(object_ids)):
        raise ValueError('Unique source object UID/runtime correspondence required')
    mapping = {uid: uid_base + scene_ordinal * 100 + ordinal for ordinal, uid in enumerate(old_ids)}
    if set(mapping.values()) & set(scenario_uids(scene)):
        raise ValueError('Fresh object UID overlaps an existing object/Ego/other identity')
    result, current = copy.deepcopy(scene), copy.deepcopy(metadata)
    for actor in result['objectList']:
        actor['UNIQUEID'] = mapping[actor['UNIQUEID']]
    for actor in current['runtime_actors']:
        actor['unique_id'] = mapping[actor['unique_id']]
    required = current.get('ground_support_check', {}).get('required_uids')
    if required is not None:
        if not isinstance(required, list) or any(uid not in mapping for uid in required):
            raise ValueError('Ground-required UID is not in the frozen object population')
        current['ground_support_check']['required_uids'] = [mapping[uid] for uid in required]
    old_plan = metadata['spec'].get(scale.PLAN_KEY)
    if not old_plan:
        raise ValueError('Normal frozen mixed scale plan required before UID rebinding')
    profiles = old_plan['profile_ids']
    current['spec'] = scale.attach_plan(old_plan['source_spec'], current['runtime_actors'], policy, profiles)
    if scale.PLAN_KEY not in current['spec']:
        raise ValueError('Other qualified variants must stay active after UID rebinding')
    for ordinal, actor in enumerate(current['runtime_actors']):
        expected = scale.select(scale._identity_asset(actor), current['spec'], ordinal,
                                actor['motion'], policy, profile_id=profiles[ordinal])
        if actor.get('asset_scale') != expected:
            raise ValueError('UID-only counterfactual unexpectedly changed a scale assignment')
        if {k: v for k, v in actor.items() if k != 'unique_id'} != \
                {k: v for k, v in actors[ordinal].items() if k != 'unique_id'}:
            raise ValueError('UID-only counterfactual changed non-identity actor metadata')
    inverse = {new: old for old, new in mapping.items()}
    restored = copy.deepcopy(result)
    for actor in restored['objectList']:
        actor['UNIQUEID'] = inverse[actor['UNIQUEID']]
    if restored != scene or digest(geometry(current)) != digest(geometry(metadata)):
        raise ValueError('UID-only counterfactual changed physical scene geometry or non-object fields')
    return result, current, mapping


def prepare(manifest, output, *, uid_base=60000):
    source = Path(manifest).resolve(strict=True)
    original = read(source)
    if (original.get('count') != 4 or len(original.get('scenes', [])) != 4
            or not original.get('redbarrel_all_unit_registration')
            or original['config'].get('capture', {}).get('asset_scale_probe') is True
            or original.get('split_overlay') is not None):
        raise ValueError('Exactly four registered all-Red-unit mixed diagnostics required')
    preserved_roots = [source.parent] + [Path(entry[field]).resolve().parent
        for entry in original['scenes'] for field in ('scenario', 'metadata')]
    output = fresh_output(output, preserved_roots)
    frozen = scale.freeze_policy(original['config'][scale.CONFIG_KEY])
    reference = original['config'].get(unit.KEY)
    if reference is None or len(unit.load(reference)['assets']) != 19:
        raise ValueError('Pinned full 19-asset unit QA must remain enabled')
    pins = {str(source): sha256(source)}
    for path, checksum in original['redbarrel_all_unit_registration']['source_files'].items():
        if sha256(path) != checksum:
            raise ValueError('Preserved all-Red-unit source lineage changed')
        pins[str(Path(path).resolve())] = checksum
    pins[reference['path']] = reference['sha256']
    records, mappings, new_ids = [], [], set()
    checked = dict(original, config=dict(original['config'], **{scale.CONFIG_KEY: frozen}))
    for ordinal, entry in enumerate(original['scenes']):
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ValueError('All-Red-unit source scenario changed')
        validate_entry(entry, checked)
        unit.validate_entry(entry, checked)
        for path in (entry['scenario'], entry['metadata']):
            pins[str(Path(path).resolve())] = sha256(path)
        scene, meta = read(entry['scenario']), read(entry['metadata'])
        if (not meta.get('all_red_unit_counterfactual') or meta.get('training_split_overlay') is not None
                or meta.get('prepared_shadow_ground_reuse') is not None):
            raise ValueError('Only explicit unbound all-Red-unit diagnostics may receive new object UIDs')
        current_scene, current, mapping = remap_objects(scene, meta, uid_base, ordinal, frozen)
        if new_ids & set(mapping.values()):
            raise ValueError('Fresh UIDs repeated between diagnostic scenes')
        new_ids.update(mapping.values())
        for actor in current['runtime_actors']:
            raw = next(row for row in current_scene['objectList'] if row['UNIQUEID'] == actor['unique_id'])
            if actor['name'] == 'RedBarrel' and [raw['scale'][axis] for axis in 'xyz'] != [1.,1.,1.]:
                raise ValueError('Fresh UID transform changed the all-Red-unit condition')
        # Original source UID lists and old scale plans inside the all-unit
        # counterfactual receipt are historical provenance, not active QA IDs.
        previous_diagnostics = dict(redbarrel_mixed_replay=copy.deepcopy(current.get('redbarrel_mixed_replay')),
                                    scale_stability=copy.deepcopy(current.get('scale_stability')))
        current['redbarrel_mixed_replay']['uid_and_actor_load_preserved'] = False
        current['redbarrel_mixed_replay']['actor_load_preserved'] = True
        current['redbarrel_mixed_replay']['fresh_object_uid_counterfactual'] = True
        current['scale_stability']['source_actor_uids'] = [actor['unique_id'] for actor in current['runtime_actors']]
        current['scale_stability']['original_source_actor_uids'] = [actor['unique_id'] for actor in meta['runtime_actors']]
        current['scale_stability']['uid_counterfactual_only'] = True
        proof = dict(source_scene_id=entry['id'], scene_ordinal=ordinal,
            source_scenario_sha256=entry['scenario_sha256'],
            actor_uid_mapping=[dict(old_uid=old, new_uid=new) for old, new in mapping.items()],
            source_actor_identity=old_plan_identity(meta), new_actor_identity=old_plan_identity(current),
            geometry_sha256=digest(geometry(meta)), physical_geometry_unchanged=True,
            ego_sensor_and_nonobject_uids_unchanged=True, object_population_order_unchanged=True,
            native_qa_unchanged=True, automatic_retry_authorized=False,
            independent_geometry_samples=False, uid_cache_causality_proven=False,
            historical_uid_fields_remain_source_provenance=True, previous_diagnostics=previous_diagnostics)
        current['object_uid_counterfactual'] = proof
        ident = 'freshuid_' + entry['id']
        sp, mp = (output/'scenarios'/(ident+suffix) for suffix in ('.json','_metadata.json'))
        atomic_json(sp, current_scene); atomic_json(mp, current)
        records.append(dict(entry, id=ident, scenario=str(sp), metadata=str(mp), scenario_sha256=sha256(sp)))
        mappings.append(proof)
    specs = output/'frozen_specs.json'
    atomic_json(specs, [read(entry['metadata'])['spec'] for entry in records])
    candidate = copy.deepcopy(original)
    candidate['config'][scale.CONFIG_KEY] = frozen
    candidate['config']['frozen_specs_path'] = str(specs)
    candidate['config']['frozen_specs_sha256'] = sha256(specs)
    candidate.update(dataset_id='base5_all_red_unit_fresh_object_uids_261017', count=4, scenes=records,
        status='UID_ONLY_COUNTERFACTUAL_DIAGNOSTIC; no physical geometry, native QA, or policy change',
        object_uid_counterfactual_registration=dict(source_manifest=str(source), source_files=pins,
            first_attempt_scenes=4, uid_base=uid_base, uid_band_width=100, unique_new_object_uids=len(new_ids),
            unique_serialized_payloads=4, independent_physical_layouts=2, repeated_poses_not_independent=True,
            all_red_unit_and_nonred_profiles_preserved=True, native_19_asset_unit_qa_preserved=True,
            source_raw_gt_and_acceptance_unchanged=True, cache_mechanism_not_established=True,
            mappings=mappings, preparation_script_sha256=sha256(Path(__file__))))
    for entry in records:
        validate_entry(entry, candidate)
        unit.validate_entry(entry, candidate)
    if len({entry['scenario_sha256'] for entry in records}) != 4:
        raise ValueError('Four distinct UID-only serialized payloads required')
    if any(sha256(path) != checksum for path, checksum in pins.items()):
        raise ValueError('Preserved UID counterfactual source changed')
    scale.verify_frozen_policy(frozen)
    candidate['config'] = scale.plain_config(candidate['config'])
    path = output/'manifest.json'
    atomic_json(path, candidate)
    return path


def old_plan_identity(metadata):
    return copy.deepcopy(metadata['spec'][scale.PLAN_KEY]['actor_identity'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--uid-base', type=int, default=60000)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output, uid_base=args.uid_base))
