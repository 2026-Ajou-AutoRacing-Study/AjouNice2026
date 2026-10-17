#!/usr/bin/env python3
"""CPU-only original2/forward-side4 diagnostic; no capture or source mutation.

Each new geometry has its own strict canonical recovery overlay and one-frame
manifest. Different variants of one canonical cell are intentionally NOT merged
under a misleading shared overlay. A future training source needs common full-
deck split finalization and independent qualification after these probes.
"""
import argparse
from collections import Counter
import copy
import json
import math
from pathlib import Path
import time

import base5_close_primary_observation as close
import base5_layout_split as layout
import base5_spatial_diversity as spatial
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
from dynamic_capture_metadata import normalized_sensor_calibration
from run_base5_failure_aware_replenishment_fast import prewarm
from run_base5_quality_loop import (bind_replacement_contract, replacement_contract,
    ReplacementGeometryRejected)


def roster(metadata):
    return [(a['unique_id'], a['data_id'], a['name'], a['source_list'], a['motion'],
        a.get('asset_scale')) for a in metadata['runtime_actors']]


def unit_payload(scenario):
    for actor in scenario.get('objectList', []):
        scale = actor.get('scale', {})
        if (any(type(scale.get(axis)) not in (int,float) or scale[axis] != 1. for axis in 'xyz')
                or any('_'+axis in scale and scale['_'+axis] != '1.000' for axis in 'xyz')):
            raise close.ClosePrimaryContractError('Diagnostic physical payload must remain scale111')


def inventory(deck):
    config = copy.deepcopy(deck['config'])
    config[close.KEY] = close.configuration(config[unit.KEY])
    policy, native = close.policy_for(config)
    records = []
    for entry in deck['scenes']:
        meta = json.loads(Path(entry['metadata']).read_text())
        if not close.eligible(meta, policy, native):
            continue
        if abs(meta['runtime_actors'][0]['planned_surface_clearance_m'] - .30) > .001:
            raise close.ClosePrimaryContractError('Eligible source placement gap is not 0.30m')
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise close.ClosePrimaryContractError('Eligible source payload changed')
        unit_payload(json.loads(Path(entry['scenario']).read_text()))
        actor = meta['runtime_actors'][0]
        records.append(dict(id=entry['id'], name=actor['name'], data_id=actor['data_id'],
            role=meta['spec']['role'], mission=meta['spec']['mission'],
            primary_ego_xy_m=[actor['longitudinal_m'], actor['lateral_m']],
            original_gap_bounds_m=meta['spec']['avoidance_gap_bounds_m'],
            scenario_sha256=entry['scenario_sha256'], metadata_sha256=sha256(entry['metadata'])))
    return dict(eligible_count=len(records), unchanged_if_applied=len(deck['scenes'])-len(records),
        by_asset=dict(Counter(r['name'] for r in records)),
        by_role=dict(Counter(r['role'] for r in records)), records=records,
        selection_uses_native_outcomes=False, proposed_geometry_count_if_full_applied=len(records),
        full_source_regenerated=False, full_source_qualified=False), config


def _entry_deck(deck, entry, config):
    result = copy.deepcopy(deck)
    result.update(config=copy.deepcopy(config), count=1, scenes=[copy.deepcopy(entry)],
        status='CLOSE_PRIMARY_DIAGNOSTIC_CPU_ONLY; NOT A TRAINING RELEASE',
        bulk_ready=False, bulk_started=False, completion_live_qualified=False)
    return result


def generate_variant(deck, source_entry, original, config, ctx, calibration, output, band, ordinal):
    """Bounded factory retaining original family, roster, motion, phase and side."""
    frozen = replacement_contract(deck, source_entry, original)
    row = close.tagged_row(original, config, band)
    rejections, binding_rejections = Counter(), []
    start = original['retry']
    for attempt in range(start, start + 512):
        try:
            scene, meta = spatial.gen.build(copy.deepcopy(row), config, ctx, attempt)
            if roster(meta) != roster(original):
                rejections['ordered_roster_changed'] += 1
                continue
            if meta['split'] != original['split']:
                rejections['capture_split_changed'] += 1
                continue
            actor = meta['runtime_actors'][0]
            if (not band[0] - 1e-6 <= actor['longitudinal_m'] <= band[1] + 1e-6
                    or abs(actor['longitudinal_m']) >= abs(actor['lateral_m'])
                    or actor['actual_ego_role'] != row['role']
                    or abs(actor['planned_surface_clearance_m'] - .30) > .001):
                raise close.ClosePrimaryContractError('Generated close-primary geometry violates frozen band/side/gap')
            proposal = close.projection_proposal(meta, calibration)
            if not proposal['eligible']:
                rejections['whole_proxy_camera_proposal_failed'] += 1
                continue
            unit_payload(scene)
            for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                        'fresh_environment_provenance', 'obstacle_support_pair'):
                if key in original:
                    meta[key] = copy.deepcopy(original[key])
            meta['close_primary_diagnostic'] = dict(source_entry=copy.deepcopy(source_entry),
                source_metadata_sha256=sha256(source_entry['metadata']), band_m=band,
                source_spec=copy.deepcopy(original['spec']), source_spec_changed_only_by=close.KEY,
                camera_proposal=proposal, diagnostic_only=True, native_acceptance_unchanged=True,
                source_raw_or_gt_modified=False, independent_success_rate_sample=False)
            ident = source_entry['id'] + f'_close_q{ordinal}'
            directory = output / f'proposal_{attempt:05d}'
            directory.mkdir(parents=True)
            scenario_path, meta_path = directory / (ident + '.json'), directory / (ident + '_metadata.json')
            atomic_json(scenario_path, scene)
            atomic_json(meta_path, meta)
            entry = copy.deepcopy(source_entry)
            entry.update(id=ident, scenario=str(scenario_path.resolve()), metadata=str(meta_path.resolve()),
                scenario_sha256=sha256(scenario_path), source_cell_id=ident)
            child = _entry_deck(deck, entry, config)
            try:
                bind_replacement_contract(source_entry, original, child['scenes'][0], meta,
                    child, directory, context=frozen)
            except ReplacementGeometryRejected as error:
                binding_rejections.append(dict(retry=attempt, reason=str(error)))
                if len(binding_rejections) >= 8:
                    break
                continue
            unit.validate_entry(child['scenes'][0], child)
            close.validate_entry(child['scenes'][0], child)
            layout.validate_manifest_overlay(child, allow_subset=True)
            path = directory / 'manifest.json'
            atomic_json(path, child)
            return dict(kind='new_forward_side_geometry', manifest=str(path.resolve()),
                manifest_sha256=sha256(path), canonical_source_id=source_entry['id'],
                source_entry=copy.deepcopy(source_entry), new_id=ident, retry=attempt,
                scenario_sha256=entry['scenario_sha256'], metadata_sha256=sha256(meta_path),
                primary_ego_xy_m=[actor['longitudinal_m'], actor['lateral_m']],
                placement_obb_gap_m=actor['planned_surface_clearance_m'],
                camera_proposal=proposal, rejections=dict(rejections),
                binding_rejections=binding_rejections, geometry_proposals=attempt-start+1,
                training_overlay_finalization_pending=True)
        except ValueError as error:
            rejections[str(error)] += 1
    raise RuntimeError('Bounded close-primary proposal exhausted without QA waiver: '
        +json.dumps(dict(source=source_entry['id'], band=band, rejections=dict(rejections),
            binding_rejections=binding_rejections)))


def prepare(source, output, source_ids, sensor_config=None):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('New close-primary diagnostic output required')
    if len(source_ids) != 2 or len(set(source_ids)) != 2:
        raise ValueError('Exactly two distinct explicit control source IDs required')
    runtime = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if output == runtime or runtime in output.parents:
        raise ValueError('Diagnostic output cannot be the live runtime')
    begin = time.monotonic()
    source_sha = sha256(source)
    deck = json.loads(source.read_text())
    layout.validate_manifest_overlay(deck, allow_subset=True)
    inv, config = inventory(deck)
    available = {r['id'] for r in inv['records']}
    if not set(source_ids) <= available:
        raise ValueError('Diagnostic source IDs outside outcome-independent policy population')
    entries = {e['id']:e for e in deck['scenes']}
    sensor = Path(sensor_config) if sensor_config else Path(__file__).resolve().parents[3] / 'configs/sensors/Sensor_competition_local.json'
    calibration = normalized_sensor_calibration(sensor)
    pins = {str(source):source_sha, str(sensor.resolve()):sha256(sensor)}
    ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
        spatial.gen.geo.DEFAULT_MGEO), config)
    output.mkdir(parents=True)
    atomic_json(output / 'eligibility.json', inv)
    results = []
    with spatial.installed():
        for source_index, ident in enumerate(source_ids):
            entry = entries[ident]
            original = json.loads(Path(entry['metadata']).read_text())
            pins[str(Path(entry['scenario']).resolve())] = entry['scenario_sha256']
            pins[str(Path(entry['metadata']).resolve())] = sha256(entry['metadata'])
            control = _entry_deck(deck, entry, deck['config'])
            path = output / f'control_{source_index}' / 'manifest.json'
            atomic_json(path, control)
            unit.validate_entry(entry, control)
            layout.validate_manifest_overlay(control, allow_subset=True)
            results.append(dict(kind='exact_original_control', manifest=str(path.resolve()),
                manifest_sha256=sha256(path), canonical_source_id=ident, source_entry=copy.deepcopy(entry),
                scenario_sha256=entry['scenario_sha256'], metadata_sha256=sha256(entry['metadata']),
                primary_ego_xy_m=[original['runtime_actors'][0][k] for k in ('longitudinal_m','lateral_m')],
                camera_proposal=close.projection_proposal(original, calibration)))
            for band_index, band in enumerate(([1.,1.25], [1.25,1.5])):
                results.append(generate_variant(deck, entry, original, config, ctx, calibration,
                    output / f'source_{source_index}_quarter_{band_index}', band, band_index))
    for path, expected in pins.items():
        if sha256(path) != expected:
            raise close.ClosePrimaryContractError('Source/sensor changed during diagnostic preparation')
    modules = [Path(__file__), Path(close.__file__), Path(spatial.gen.__file__),
        Path(spatial.gen.__file__).with_name('base5_avoidance_geometry.py')]
    receipt = dict(schema='base5_close_primary_diagnostic_registration_v1', source_manifest=str(source),
        source_manifest_sha256=source_sha, policy=config[close.KEY], input_pins=pins,
        implementation_hashes={str(p.resolve()):sha256(p) for p in modules},
        calibration_source_sha256=calibration['source_sha256'], eligibility=inv,
        planned_frames=6, exact_controls=2, new_geometry_frames=4, source_layouts=2,
        tasks=results, elapsed_sec=time.monotonic()-begin,
        outcome_informed_diagnostic_selection=True, independent_approval_samples=0,
        full_source_regenerated=False, bulk_started=False, completion_live_qualified=False,
        native_gt_modified=False, native_acceptance_unchanged=True,
        limitation='Six factor-matched probes, not six independent layouts or a training release. '
            'Camera proxy cannot certify mesh visibility or writer output. '
            'Each manifest has its own strict canonical overlay; never union their different references. '
            'Future full-source adoption requires common layout finalization and exact native qualification.')
    atomic_json(output / 'registration.json', receipt)
    return output / 'registration.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--source-ids', required=True, help='Two comma-separated exact source IDs')
    parser.add_argument('--sensor-config', type=Path)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output, args.source_ids.split(','), args.sensor_config))
