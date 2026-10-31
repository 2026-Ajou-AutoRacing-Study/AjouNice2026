#!/usr/bin/env python3
"""Prepare four all-Red-unit mixed counterfactuals; no live or source mutation.

Only RedBarrel scales change. Other actors and non-Red scale profiles stay
frozen. Native unit QA uses its own explicit 19-asset policy, never an old
placement envelope. The historical scale plan is replaced and revalidated by
normal scale-contract checks; the single-parent probe exception is not used.
"""
import argparse
import copy
import json
import math
from pathlib import Path

import base5_asset_scale as scale
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
from base5_reference_frames import references
from base5_scale_capture_contract import validate_entry
from prepare_base5_redbarrel_mixed_replay import IDS
from prepare_base5_scale_qualification import fresh_output, merge_references
from prepare_base5_scale_stability import envelope, unit_control


def read(path):
    return json.loads(Path(path).read_text())


def original_unit_geometry(actor, metadata, reference):
    """Correct a diagnostic size proxy without moving an existing unit pivot.

    A 1.606m conservative unit envelope is not measured mesh height. Its bottom
    must not be used to lower the existing unit actor by approximately 15 cm.
    """
    size, offset = reference['bbox_size_m'], reference['pivot_to_bbox_center_local_m']
    actor['estimated_size'] = list(size)
    actor['geometry_reference_known'] = True
    yaw = math.radians(actor['placement_yaw_deg'])
    center = [actor['position']['x']+math.cos(yaw)*offset[0]-math.sin(yaw)*offset[1],
              actor['position']['y']+math.sin(yaw)*offset[0]+math.cos(yaw)*offset[1]]
    actor['geometric_center_estimate'] = center
    actor['planned_center_distance_m'] = math.hypot(center[0]-metadata['ego']['x'], center[1]-metadata['ego']['y'])
    frame = references(center, metadata['ego'], metadata['base']['yaw'])
    actor['reference_frames'] = frame
    actor['longitudinal_m'], actor['lateral_m'] = frame['ego_longitudinal_m'], frame['ego_lateral_m']


def all_red_unit_geometry(scene, metadata, reference):
    """Reuse matched unit conversion for enlarged Red only; retain other scales."""
    before_scene, before = copy.deepcopy(scene), copy.deepcopy(metadata)
    temporary = copy.deepcopy(metadata)
    for actor in temporary['runtime_actors']:
        if actor['name'] != 'RedBarrel':
            actor.pop('asset_scale', None)
            actor.pop('asset_scale_geometry', None)
    # Source-qualified enlarged Red geometry supplies the bottom initializer.
    # Original Red BASE assignments are deliberately skipped by unit_control.
    current_scene, current = unit_control(scene, temporary, {'RedBarrel': reference})
    proofs = []
    for index, (old, actor) in enumerate(zip(before['runtime_actors'], current['runtime_actors'])):
        if old['name'] != 'RedBarrel':
            current['runtime_actors'][index] = copy.deepcopy(old)
            continue
        assignment = old.get('asset_scale') or {}
        factors = assignment.get('scale_xyz', [1., 1., 1.])
        was_unit = list(factors) == [1., 1., 1.]
        if was_unit:
            original_unit_geometry(actor, current, reference)
            # These are saved/map initializers, not native contact labels.
            if old.get('static_height_initializer') is not None:
                actor['static_height_initializer'] = copy.deepcopy(old['static_height_initializer'])
            if actor['position'] != old['position']:
                raise ValueError('Original unit Red pivot/height must remain exactly unchanged')
        raw = [row for row in current_scene['objectList'] if row['UNIQUEID'] == old['unique_id']]
        if len(raw) != 1 or raw[0]['DataID'] != 40100003:
            raise ValueError('Explicit RedBarrel native scenario identity required')
        if not was_unit and any(raw[0]['scale'][axis] != 1. for axis in 'xyz'):
            raise ValueError('Enlarged Red was not converted to exact unit scale')
        proofs.append(dict(parent_uid=old['unique_id'], old_scale_xyz=list(factors), new_scale_xyz=[1.,1.,1.],
            old_estimated_size_m=old['estimated_size'], native_unit_reference_size_m=reference['bbox_size_m'],
            old_position=old['position'], new_position=copy.deepcopy(actor['position']),
            old_height_initializer=old.get('static_height_initializer'),
            new_height_initializer=actor.get('static_height_initializer'),
            original_unit_pose_and_height_preserved=was_unit,
            ground_policy='existing unit pose unchanged' if was_unit else 'qualified scaled-native bottom to unit initializer',
            old_conservative_unit_envelope_is_mesh_ground_truth=False, native_gt_modified=False))
    if len(proofs) < 2:
        raise ValueError('Mixed counterfactual requires multiple original RedBarrel parents')
    ego = dict(geometric_center_estimate=[before['ego']['x']+1.35*math.cos(math.radians(before['ego']['yaw'])),
        before['ego']['y']+1.35*math.sin(math.radians(before['ego']['yaw']))],
        placement_yaw_deg=before['ego']['yaw'], estimated_size=[4.8,2.,1.])
    for index, actor in enumerate(current['runtime_actors']):
        old = before['runtime_actors'][index]
        gap = envelope(actor).distance(envelope(ego))
        if gap < min(.3, envelope(old).distance(envelope(ego)))-.03:
            raise ValueError('All-Red-unit control worsens frozen Ego clearance')
        if actor['name'] == 'RedBarrel':
            actor['planned_surface_clearance_m'] = gap
        for other in range(index):
            if envelope(actor).distance(envelope(current['runtime_actors'][other]))+.03 < \
                    envelope(old).distance(envelope(before['runtime_actors'][other])):
                raise ValueError('All-Red-unit control worsens a frozen actor clearance')
    for old in before['runtime_actors']:
        if old['name'] == 'RedBarrel':
            continue
        container = old['source_list']
        raw_before = [row for row in before_scene[container] if row['UNIQUEID'] == old['unique_id']]
        raw_after = [row for row in current_scene[container] if row['UNIQUEID'] == old['unique_id']]
        if raw_before != raw_after:
            raise ValueError('Non-Red serialized actor changed')
    return current_scene, current, proofs


def prepare(manifest, output, unit_reference, unit_review_reference, *, repeats=2):
    if type(repeats) is not int or repeats != 2:
        raise ValueError('Exactly two layouts x two repeats required; no automatic extra attempts')
    source = Path(manifest).resolve(strict=True)
    reference_path = Path(unit_reference).resolve(strict=True)
    output = fresh_output(output, [source.parent, reference_path.parent,
                                   Path(unit_review_reference['path']).resolve().parent])
    original = read(source)
    if original['config'].get('capture', {}).get('asset_scale_probe') is True:
        raise ValueError('Mixed counterfactual cannot use the single-parent probe exception')
    frozen = scale.freeze_policy(original['config'][scale.CONFIG_KEY])
    policy = unit.load(unit_review_reference)
    units, unit_pins = merge_references([reference_path], {'RedBarrel'})
    reference = units['assets']['RedBarrel']
    if len(policy['assets']) != 19 or policy['assets']['RedBarrel']['native_size_lwh_m'] != reference['bbox_size_m']:
        raise ValueError('Pinned 19-asset unit QA and small Red reference must agree')
    pins = dict(unit_pins)
    for path in (source, unit_review_reference['path'], policy['catalog_reference']['path']):
        pins[str(Path(path).resolve())] = sha256(path)
    for item in policy['sources']:
        if sha256(item['path']) != item['sha256']:
            raise ValueError('Native unit QA source changed')
        pins[str(Path(item['path']).resolve())] = item['sha256']
    for path, digest in original.get('redbarrel_mixed_replay_registration', {}).get('source_files', {}).items():
        if sha256(path) != digest:
            raise ValueError('Preserved historical mixed source changed')
        pins[str(Path(path).resolve())] = digest
    selected = {}
    for entry in original['scenes']:
        meta = read(entry['metadata'])
        ident = meta.get('redbarrel_mixed_replay', {}).get('source_id')
        if ident not in IDS or ident in selected:
            continue
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ValueError('Frozen mixed source scenario changed')
        check = dict(original, config=dict(original['config'], **{scale.CONFIG_KEY:frozen}))
        validate_entry(entry, check)
        for path in (entry['scenario'], entry['metadata']):
            fresh_output(output, [Path(path).resolve().parent])
            pins[str(Path(path).resolve())] = sha256(path)
        selected[ident] = (entry, meta)
    if set(selected) != set(IDS):
        raise ValueError('Both exact historical mixed layouts required')
    result, transformations = [], {}
    candidate = copy.deepcopy(original)
    candidate['config'][scale.CONFIG_KEY] = frozen
    candidate['config'][unit.KEY] = copy.deepcopy(unit_review_reference)
    for repeat in range(repeats):
        for position, ident in enumerate(IDS if repeat == 0 else IDS[::-1]):
            old_entry, metadata = selected[ident]
            scene, current, proofs = all_red_unit_geometry(read(old_entry['scenario']), metadata, reference)
            old_plan = metadata['spec'][scale.PLAN_KEY]
            profiles = [scale.BASE if actor['name'] == 'RedBarrel' else old_plan['profile_ids'][ordinal]
                        for ordinal, actor in enumerate(current['runtime_actors'])]
            current['spec'] = scale.attach_plan(old_plan['source_spec'], current['runtime_actors'], frozen, profiles)
            if scale.PLAN_KEY not in current['spec']:
                raise ValueError('Expected non-Red qualified variants must remain in both mixed layouts')
            for ordinal, actor in enumerate(current['runtime_actors']):
                assignment = scale.select(scale._identity_asset(actor), current['spec'], ordinal,
                    actor['motion'], frozen, profile_id=profiles[ordinal])
                old_actor = metadata['runtime_actors'][ordinal]
                if actor['name'] != 'RedBarrel':
                    if assignment != old_actor.get('asset_scale') or actor != old_actor:
                        raise ValueError('Non-Red metadata/profile changed')
                    continue
                actor['asset_scale'] = assignment
                _, _, geometry = scale.placement_geometry(reference['bbox_size_m'],
                    reference['pivot_to_bbox_center_local_m'], assignment, frozen)
                actor['asset_scale_geometry'] = geometry
                actor['redbarrel_unit_counterfactual_reference'] = dict(path=str(reference_path),
                    sha256=pins[str(reference_path)], diagnostic_only=True, mesh_contact_certified=False)
            current[unit.KEY] = copy.deepcopy(unit_review_reference)
            current['redbarrel_mixed_replay'] = dict(source_id=ident, repeat_ordinal=repeat,
                within_repeat_order=position, counterfactual_all_red_unit=True,
                source_scaled_spec_sha256=scale.digest(metadata['spec']),
                source_scaled_plan_sha256=scale.digest(old_plan),
                scenario_geometry_identical_across_repeats=True, independent_geometry_samples=False,
                historical_policy_validation_only=True, redbarrel_enlargement_reapproved=False,
                automatic_retry_authorized=False, uid_and_actor_load_preserved=True)
            current['all_red_unit_counterfactual'] = dict(diagnostic_only=True, source_id=ident,
                source_scenario_sha256=old_entry['scenario_sha256'], source_spec=metadata['spec'],
                source_scale_plan=old_plan, replacement_profile_ids=profiles, transformed_red_actors=proofs,
                non_red_actors_and_scales_unchanged=True, actual_gt_or_source_policy_modified=False,
                original_scale1_red_pose_preserved=True, independent_layouts=2, repeats_are_not_independent=True)
            current['scale_stability'] = dict(condition='all_red_unit', source_id=ident,
                independent_geometry_unit=ident, repeat_ordinal=repeat, source_actor_load=len(current['runtime_actors']),
                source_actor_uids=[actor['unique_id'] for actor in current['runtime_actors']],
                process_state='warm_only; operator records actual state', native_qa_unchanged=True,
                counterfactual_distribution_claim=False)
            new_id = f'red_all_unit_{repeat}_{position}_{ident}'
            sp, mp = (output/'scenarios'/(new_id+suffix) for suffix in ('.json','_metadata.json'))
            atomic_json(sp, scene); atomic_json(mp, current)
            result.append(dict(old_entry, id=new_id, scenario=str(sp), metadata=str(mp), scenario_sha256=sha256(sp)))
            transformations[ident] = proofs
    specs = output/'frozen_specs.json'
    atomic_json(specs, [read(entry['metadata'])['spec'] for entry in result])
    candidate['config']['frozen_specs_path'] = str(specs)
    candidate['config']['frozen_specs_sha256'] = sha256(specs)
    candidate.update(dataset_id='base5_redbarrel_all_unit_mixed_261017', count=4, scenes=result,
        status='COUNTERFACTUAL_DIAGNOSTIC_ONLY; all Red unit, other scales preserved; no production approval',
        redbarrel_all_unit_registration=dict(source_manifest=str(source), source_files=pins,
            source_ids=list(IDS), first_attempt_scenes=4, independent_payloads=2, repeats=2,
            single_parent_probe_exception_used=False, native_unit_qa_reference=unit_review_reference,
            native_size_ground_collision_park_checks_unchanged=True, automatic_extra_attempts=False,
            original_source_and_raw_preserved=True, transformations=transformations,
            preparation_script_sha256=sha256(Path(__file__))))
    for entry in result:
        validate_entry(entry, candidate)
        unit.validate_entry(entry, candidate)
    if len({entry['scenario_sha256'] for entry in result}) != 2:
        raise ValueError('Exactly two distinct counterfactual payloads repeated twice required')
    if any(sha256(path) != digest for path, digest in pins.items()):
        raise ValueError('Historical mixed/unit source changed during preparation')
    scale.verify_frozen_policy(frozen)
    candidate['config'] = scale.plain_config(candidate['config'])
    path = output/'manifest.json'
    atomic_json(path, candidate)
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--unit-reference', type=Path, required=True)
    parser.add_argument('--unit-review-reference', type=Path, required=True)
    parser.add_argument('--unit-review-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output, args.unit_reference,
        dict(path=str(args.unit_review_reference.resolve()), sha256=args.unit_review_sha256)))
