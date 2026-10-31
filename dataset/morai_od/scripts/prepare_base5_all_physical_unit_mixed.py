#!/usr/bin/env python3
"""Prepare the four fixed mixed layouts with every physical object at scale 1.

Cold-unit-before-scale and hot-unit-after-scale can consume the same payloads.
No process restart, simulator action, GT edit, retry or approval occurs here.
"""
import argparse
import copy
import json
from pathlib import Path

import base5_asset_scale as scale
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
from base5_scale_capture_contract import validate_entry, raw_variants
from prepare_base5_scale_qualification import fresh_output, merge_references
from prepare_base5_scale_stability import unit_control


def read(path):
    return json.loads(Path(path).read_text())


def convert(scene, metadata, units):
    original_scene, original = copy.deepcopy(scene), copy.deepcopy(metadata)
    result, current = unit_control(scene, metadata, units)
    proofs = []
    for old, actor in zip(original['runtime_actors'], current['runtime_actors']):
        if actor['source_list'] != 'objectList':
            raise ValueError('All-physical-unit mixed diagnostics require physical objectList actors only')
        raw_before = next(row for row in original_scene['objectList'] if row['UNIQUEID'] == old['unique_id'])
        raw_after = next(row for row in result['objectList'] if row['UNIQUEID'] == actor['unique_id'])
        factors = [raw_before['scale'][axis] for axis in 'xyz']
        if [raw_after['scale'][axis] for axis in 'xyz'] != [1.,1.,1.]:
            raise ValueError('Physical actor did not become exact unit scale')
        was_unit = factors == [1.,1.,1.]
        if was_unit and (raw_after != raw_before or actor['position'] != old['position']
                or actor.get('static_height_initializer') != old.get('static_height_initializer')):
            raise ValueError('Original unit actor raw pose/height initializer changed')
        if actor.get('asset_scale') is not None or actor.get('asset_scale_geometry') is not None:
            raise ValueError('All-unit diagnostic retained an active scale assignment')
        proofs.append(dict(parent_uid=actor['unique_id'], asset=actor['name'], source_scale_xyz=factors,
            target_scale_xyz=[1.,1.,1.], source_position=old['position'], target_position=actor['position'],
            original_unit_pose_and_height_preserved=was_unit, source_size_proxy_m=old['estimated_size'],
            target_size_proxy_m=actor['estimated_size'], native_gt_modified=False,
            ground_certified=False, initializer='unchanged original unit' if was_unit
                else 'qualified scaled-native bottom / pinned unit initializer'))
    if raw_variants(result) or scale.PLAN_KEY in current['spec']:
        raise ValueError('All-unit control retained nonunit payload or source scale plan')
    return result, current, proofs


def prepare(manifest, output, references):
    source = Path(manifest).resolve(strict=True)
    original = read(source)
    if (original.get('count') != 4 or len(original.get('scenes', [])) != 4
            or not original.get('redbarrel_all_unit_registration')
            or original['config'].get('capture', {}).get('asset_scale_probe') is True):
        raise ValueError('Exactly four explicit all-Red-unit mixed source diagnostics required')
    roots = [source.parent] + [Path(path).resolve().parent for path in references]
    roots += [Path(entry[key]).resolve().parent for entry in original['scenes'] for key in ('scenario','metadata')]
    output = fresh_output(output, roots)
    frozen = scale.freeze_policy(original['config'][scale.CONFIG_KEY])
    expected_unit = original['config'].get(unit.KEY)
    if expected_unit is None or len(unit.load(expected_unit)['assets']) != 19:
        raise ValueError('Explicit 19-asset native unit QA required')
    # Existing unit actors are not repositioned or recast from their envelope.
    # References are needed for enlarged-to-unit actors; Red's small reference
    # is also pinned explicitly to document the unit condition under test.
    required = {'RedBarrel'}
    for entry in original['scenes']:
        for actor in read(entry['metadata'])['runtime_actors']:
            if actor.get('asset_scale') and actor['asset_scale']['profile_id'] != scale.BASE:
                required.add(actor['name'])
    units, pins = merge_references(references, required)
    pins[str(source)] = sha256(source)
    pins[expected_unit['path']] = expected_unit['sha256']
    for path, checksum in original['redbarrel_all_unit_registration']['source_files'].items():
        if sha256(path) != checksum:
            raise ValueError('Preserved mixed source changed')
        pins[str(Path(path).resolve())] = checksum
    result, proofs = [], []
    checked = dict(original, config=dict(original['config'], **{scale.CONFIG_KEY:frozen}))
    for ordinal, entry in enumerate(original['scenes']):
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ValueError('Frozen all-Red-unit source scenario changed')
        validate_entry(entry, checked); unit.validate_entry(entry, checked)
        for path in (entry['scenario'], entry['metadata']):
            pins[str(Path(path).resolve())] = sha256(path)
        metadata = read(entry['metadata'])
        scene, current, actors = convert(read(entry['scenario']), metadata, units['assets'])
        current['all_physical_unit_counterfactual'] = dict(source_id=entry['id'],
            source_scenario_sha256=entry['scenario_sha256'], source_spec=metadata['spec'],
            source_scale_plan=metadata['spec'][scale.PLAN_KEY], actor_transforms=actors,
            active_scale_plan_removed_explicitly=True, native_19_asset_unit_qa_unchanged=True,
            unit_reference_sources=units['sources'], cold_or_hot_process_state_not_inferred=True,
            independent_physical_layouts=2, first_attempt_scenes=4, repeats_are_not_independent=True,
            automatic_extra_attempts=False, native_gt_or_source_acceptance_modified=False)
        current['scale_stability'].update(condition='all_physical_unit',
            cold_or_hot_process_state='operator records actual state; not inferred by preparer')
        ident = 'allphysicalunit_' + entry['id']
        sp, mp = (output/'scenarios'/(ident+suffix) for suffix in ('.json','_metadata.json'))
        atomic_json(sp, scene); atomic_json(mp, current)
        result.append(dict(entry, id=ident, scenario=str(sp), metadata=str(mp), scenario_sha256=sha256(sp)))
        proofs.append(current['all_physical_unit_counterfactual'])
    specs = output/'frozen_specs.json'
    atomic_json(specs, [read(entry['metadata'])['spec'] for entry in result])
    candidate = copy.deepcopy(original)
    # Keep the source policy as validation provenance only. With no variant
    # payload/assignment, ordinary unit capture and its explicit native unit QA
    # own the diagnostic; no historical Red scale approval is imported.
    historical_policy = candidate['config'].pop(scale.CONFIG_KEY)
    candidate['config']['frozen_specs_path'] = str(specs)
    candidate['config']['frozen_specs_sha256'] = sha256(specs)
    candidate.update(dataset_id='base5_all_physical_unit_mixed_261017', scenes=result, count=4,
        status='ALL_PHYSICAL_UNIT_COUNTERFACTUAL_ONLY; no cold/hot state or production approval inferred',
        all_physical_unit_registration=dict(source_manifest=str(source), source_files=pins,
            first_attempt_scenes=4, independent_physical_layouts=2, repeats=2,
            cold_and_hot_must_use_same_frozen_payload=True, native_19_asset_unit_qa_preserved=True,
            source_scale_policy_provenance=historical_policy, transformations=proofs,
            core_native_safety_checks_unchanged=True, simulator_or_raw_modified=False,
            preparation_script_sha256=sha256(Path(__file__))))
    for entry in result:
        validate_entry(entry, candidate); unit.validate_entry(entry, candidate)
    if len({entry['scenario_sha256'] for entry in result}) != 2:
        raise ValueError('Two physical layouts repeated twice required')
    if any(sha256(path) != checksum for path, checksum in pins.items()):
        raise ValueError('Preserved all-unit source/reference changed')
    scale.verify_frozen_policy(frozen)
    path = output/'manifest.json'
    atomic_json(path, candidate)
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--unit-reference', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest,args.output,args.unit_reference))
