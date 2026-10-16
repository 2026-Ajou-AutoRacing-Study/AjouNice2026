#!/usr/bin/env python3
"""Prepare a new bounded stationary-mission candidate; no simulator writes."""
import argparse
from collections import Counter, defaultdict
import copy
import json
import math
from pathlib import Path
import time

import audit_base5_scenarios
import base5_static_mission_extent as extent
import base5_spatial_diversity as spatial
import base5_front_release_inputs as pins
import prepare_base5_front_support_qualification as qualification
from base5_capture_safety import atomic_json, sha256
from run_base5_failure_aware_replenishment_fast import prewarm


def read(path):
    return json.loads(Path(path).read_text())


def density(metadata):
    result = {}
    for name in extent.STRATA:
        rows = [meta for meta in metadata if meta['static_mission_extent']['stratum'] == name]
        arcs = [meta['static_mission_extent']['primary_center_route']['arc_m'] for meta in rows]
        bins = {}
        for width in (2, 5):
            occupancy = Counter(str(math.floor(arc/width)*width) for arc in arcs)
            members = defaultdict(list)
            for row in rows:
                arc = row['static_mission_extent']['primary_center_route']['arc_m']
                members[str(math.floor(arc/width)*width)].append(row)
            details = {}
            for bucket, group in sorted(members.items(), key=lambda item: float(item[0])):
                phases = Counter(row['spec']['avoidance_phase'] for row in group)
                assets = Counter(row['runtime_actors'][0]['name'] for row in group)
                details[bucket] = dict(frames=len(group), phases=dict(phases), assets=dict(assets))
            bins[str(width)+'m'] = dict(occupied_bins=len(occupancy), maximum_frames_in_bin=max(occupancy.values(), default=0),
                frames_per_bin=dict(sorted(occupancy.items(), key=lambda item: float(item[0]))),
                maximum_same_asset_frames_in_bin=max((max(row['assets'].values()) for row in details.values()), default=0),
                maximum_same_phase_frames_in_bin=max((max(row['phases'].values()) for row in details.values()), default=0),
                bin_composition=details)
        primary_xy = Counter(tuple(round(v, 3) for v in row['runtime_actors'][0]['geometric_center_estimate']) for row in rows)
        ego_xy = Counter((round(row['ego']['x'], 3), round(row['ego']['y'], 3)) for row in rows)
        result[name] = dict(frames=len(rows), primary_arc_min_m=min(arcs, default=None),
            primary_arc_max_m=max(arcs, default=None), split=dict(Counter(meta['split'] for meta in rows)),
            phases=dict(Counter(meta['spec']['avoidance_phase'] for meta in rows)),
            assets=dict(Counter(meta['runtime_actors'][0]['name'] for meta in rows)), bins=bins,
            primary_xy_unique_mm=len(primary_xy), primary_xy_duplicate_frames_mm=sum(v-1 for v in primary_xy.values()),
            ego_xy_unique_mm=len(ego_xy), ego_xy_duplicate_frames_mm=sum(v-1 for v in ego_xy.values()),
            final_ego_arc_min_m=min((meta['static_mission_extent']['final_ego_route']['arc_m'] for meta in rows), default=None),
            final_ego_arc_max_m=max((meta['static_mission_extent']['final_ego_route']['arc_m'] for meta in rows), default=None))
    return result


def prepare(source, output, counts=None, maximum_attempts=1024):
    source, output = Path(source).resolve(), Path(output).resolve()
    runtime = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if output.exists():
        raise FileExistsError('New static mission extent output required')
    if output == runtime or runtime in output.parents:
        raise ValueError('Preparation must not write to MORAI runtime')
    if not 1 <= maximum_attempts <= 4096:
        raise ValueError('Finite proposal budget of 1..4096 required')
    deck = read(source)
    entries, originals, _ = qualification.validate_source(deck)
    if deck['config'].get(extent.CONFIG_KEY):
        raise extent.ExtentContractError('Extent allocation already exists; do not silently reassign')
    watched = pins.collect_runtime(entrypoints=(Path(__file__).name,))
    inputs = pins.collect_inputs(source)
    config = copy.deepcopy(deck['config'])
    ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR, spatial.gen.geo.DEFAULT_MGEO), config)
    config[extent.CONFIG_KEY] = extent.configuration(ctx, counts)
    assigned = extent.assign(originals, config)
    source_audit = audit_base5_scenarios.audit(source)
    if source_audit.get('status') != 'OFFLINE_PASS' or source_audit.get('errors'):
        raise extent.ExtentContractError('Source scenario audit failed')
    frozen, candidate_entries, changed, completed = [], [], [], []
    rejections = Counter()
    started = time.monotonic()
    for entry in deck['scenes']:
        ident, before = entry['id'], originals[entry['id']]
        if ident not in assigned:
            candidate_entries.append(copy.deepcopy(entry))
            frozen.append(copy.deepcopy(before['spec']))
            continue
        spec = assigned[ident]
        extent.validate_plan(spec, config)
        for attempt in range(before['retry'], before['retry']+maximum_attempts):
            try:
                scene, meta = spatial.build(copy.deepcopy(spec), config, ctx, attempt)
            except extent.ExtentContractError:
                raise
            except ValueError as error:
                rejections[str(error)] += 1
                continue
            extent.verify_transition(before, meta, spec)
            if not meta.get('static_mission_extent', {}).get('primary_center_not_ego_used_for_coverage'):
                raise extent.ExtentContractError('Generator has not installed the extent metadata hook')
            break
        else:
            atomic_json(output/'failure.json', dict(id=ident, completed=len(completed),
                maximum_attempts=maximum_attempts, rejected=dict(rejections), bulk_started=False))
            raise ValueError('Bounded stationary mission preparation exhausted: '+ident)
        for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id', 'fresh_environment_provenance'):
            if key in before:
                meta[key] = copy.deepcopy(before[key])
        meta['static_mission_extent_preparation'] = dict(policy=extent.POLICY, source_id=ident,
            source_scenario_sha256=entry['scenario_sha256'], source_metadata_sha256=inputs[str(Path(entry['metadata']).resolve())],
            original_base_arc_m=before['base']['route_arc_m'], original_ego=before['ego'],
            geometry_changed=True, native_qa_unchanged=True, model_ap_improvement_certified=False)
        scenario, metadata = output/'scenarios'/(ident+'.json'), output/'scenarios'/(ident+'_metadata.json')
        atomic_json(scenario, scene)
        atomic_json(metadata, meta)
        updated = copy.deepcopy(entry)
        updated.update(scenario=str(scenario), metadata=str(metadata), scenario_sha256=sha256(scenario))
        candidate_entries.append(updated)
        frozen.append(copy.deepcopy(spec))
        completed.append(meta)
        changed.append(dict(id=ident, primary_data_id=meta['runtime_actors'][0]['data_id'],
            source_scenario_sha256=entry['scenario_sha256'], scenario_sha256=updated['scenario_sha256'],
            **meta['static_mission_extent']))
        if len(completed) % 25 == 0:
            print('static-extent', len(completed), '/600', round(time.monotonic()-started, 1), flush=True)
    frozen_path = output/'frozen_specs.json'
    atomic_json(frozen_path, frozen)
    config.update(frozen_specs_path=str(frozen_path), frozen_specs_sha256=sha256(frozen_path))
    candidate = copy.deepcopy(deck)
    candidate.update(config=config, scenes=candidate_entries,
        status='STATIC_MISSION_EXTENT_CPU_ONLY; native qualification pending; no bulk collection')
    candidate.setdefault('provenance', {})['static_mission_extent'] = dict(policy=extent.POLICY,
        source_manifest=str(source), source_manifest_sha256=inputs[str(source)], source_hashes=watched)
    manifest = output/'manifest.json'
    atomic_json(manifest, candidate)
    errors = []
    for path, checksum in {**inputs, **watched}.items():
        if sha256(path) != checksum:
            errors.append([path, 'source/helper changed during preparation'])
    for entry in candidate_entries:
        ident = entry['id']
        if ident in assigned:
            after = read(entry['metadata'])
            extent.verify_transition(originals[ident], after, assigned[ident])
            extent.finish(spatial.gen, ctx, after, config)
            if sha256(entry['scenario']) != entry['scenario_sha256']:
                errors.append([ident, 'output payload checksum mismatch'])
        elif entry != entries[ident]:
            errors.append([ident, 'non-target entry changed'])
    audit = audit_base5_scenarios.audit(manifest)
    atomic_json(output/'scenario_audit.json', audit)
    if audit.get('status') != 'OFFLINE_PASS' or audit.get('errors'):
        errors.append(['all', 'full scenario audit failed'])
    before_counts = Counter((meta['split'], meta['spec']['avoidance_phase'], meta['runtime_actors'][0]['data_id'])
        for ident, meta in originals.items() if ident in assigned)
    after_counts = Counter((meta['split'], meta['spec']['avoidance_phase'], meta['runtime_actors'][0]['data_id']) for meta in completed)
    if before_counts != after_counts:
        errors.append(['all', 'split/phase/primary asset allocation changed'])
    report = dict(schema='base5_static_mission_extent_preparation_v1', policy=extent.POLICY,
        passed=not errors, errors=errors, frames=5000, changed_frames=len(changed), unchanged_entry_paths_and_payloads=4400,
        source_manifest=str(source), source_manifest_sha256=inputs[str(source)], source_files=inputs,
        implementation_files=watched, candidate_manifest_sha256=sha256(manifest),
        configuration=config[extent.CONFIG_KEY], primary_not_ego_coverage=density(completed), changes=changed,
        source_primary_assets=len({meta['runtime_actors'][0]['data_id'] for ident, meta in originals.items() if ident in assigned}),
        target_primary_assets=len({meta['runtime_actors'][0]['data_id'] for meta in completed}),
        rejected=dict(rejections), bulk_ready=False, bulk_started=False, live_qualified=False,
        limitation='Road support is a placement proxy. Signal interaction is deliberately not constrained; native safety/visibility/GT gates remain mandatory.')
    atomic_json(output/'preservation_audit.json', report)
    if errors:
        raise extent.ExtentContractError('Static mission extent output audit failed')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--counts', type=int, nargs=3, metavar=('BEFORE', 'INTERSECTION', 'AFTER'))
    parser.add_argument('--maximum-attempts', type=int, default=1024)
    args = parser.parse_args()
    counts = dict(zip(extent.STRATA, args.counts)) if args.counts else None
    print(prepare(args.manifest, args.output, counts, args.maximum_attempts))
