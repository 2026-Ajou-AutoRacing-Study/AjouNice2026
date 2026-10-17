#!/usr/bin/env python3
"""Prepare a versioned split overlay/candidate; preserve every source byte."""
import argparse
import copy
import json
import math
from pathlib import Path

import base5_layout_split as layout
from base5_capture_safety import atomic_json, sha256


def prepare(source, output, val_fraction=.15, train_asset_floor=5):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('New split overlay directory required')
    document = json.loads(source.read_text())
    records, pins = [], {str(source): sha256(source)}
    for entry in document['scenes']:
        path, scenario = Path(entry['metadata']), Path(entry['scenario'])
        checksum = sha256(scenario)
        if checksum != entry['scenario_sha256']:
            raise ValueError('Source scenario checksum differs: '+entry['id'])
        pins[str(path.resolve())], pins[str(scenario.resolve())] = sha256(path), checksum
        records.append(dict(id=entry['id'], meta=json.loads(path.read_text()), scenario_sha256=checksum))
    config = layout.configuration(val_fraction, train_asset_floor)
    overlay = layout.allocate(records, config)
    source_assignments = {ident: dict(row, split=row['source_split']) for ident, row in overlay['assignments'].items()}
    overlay['source_split_audit'] = layout.audit_assignments(records, source_assignments, config)
    if document.get('config', {}).get('target_scenes') == 5000:
        import base5_spatial_diversity as spatial
        ctx = spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR, spatial.gen.geo.DEFAULT_MGEO)
        route_config = dict(document['config'], **{layout.CONFIG_KEY: config})
        intervals = spatial.allowed_intervals(ctx, route_config)[('background', 'all')]
        poses = []
        for a, b in intervals:
            count = max(1, math.ceil((b-a)/10.))
            poses.extend(spatial.gen.geo.continuous_route_pose_at_arc(ctx['route'], ctx['arc'],
                a+(b-a)*(index+.5)/count) for index in range(count))
        overlay['audit']['allowed_route_coverage'] = layout.route_coverage(records, overlay['assignments'], poses)
    overlay.update(source_manifest=str(source), source_manifest_sha256=pins[str(source)],
        source_hashes=pins, raw_preserved=True, geometry_changed=False,
        bulk_ready=False, bulk_started=False, training_started=False)
    overlay['overlay_sha256'] = layout.digest(overlay)
    if overlay['audit']['status'] != 'CPU_PASS':
        raise ValueError('Split allocation failed: '+str(overlay['audit']['errors']))
    output.mkdir(parents=True)
    overlay_path = output/'split_overlay.json'
    atomic_json(overlay_path, overlay)
    overlay_file_hash = sha256(overlay_path)
    candidate = copy.deepcopy(document)
    candidate['split_overlay'] = dict(path=str(overlay_path), sha256=overlay_file_hash, policy=layout.POLICY)
    candidate.update(status='LAYOUT_SPLIT_OVERLAY_CPU_ONLY; not a capture release',
        bulk_ready=False, bulk_started=False)
    for entry in candidate['scenes']:
        row = overlay['assignments'][entry['id']]
        original_metadata_path = entry['metadata']
        meta = json.loads(Path(original_metadata_path).read_text())
        meta['training_split_overlay'] = dict(path=str(overlay_path), sha256=overlay_file_hash,
            overlay_sha256=overlay['overlay_sha256'], source_metadata_path=original_metadata_path,
            source_metadata_sha256=pins[str(Path(original_metadata_path).resolve())])
        new_metadata_path = output/'metadata'/(entry['id']+'.json')
        atomic_json(new_metadata_path, meta)
        entry['metadata'] = str(new_metadata_path)
        entry['derived_training_split'] = row['split']
        entry['derived_layout_group_id'] = row['group_id']
    candidate.setdefault('provenance', {})['layout_split'] = dict(source_manifest=str(source),
        source_manifest_sha256=pins[str(source)], helper_sha256=sha256(layout.__file__),
        script_sha256=sha256(__file__), original_metadata_and_payloads_preserved=True,
        candidate_metadata_change='Training overlay provenance only; split/spec/geometry unchanged')
    atomic_json(output/'manifest.json', candidate)
    for path, checksum in pins.items():
        if sha256(path) != checksum:
            raise ValueError('Source changed during split preparation: '+path)
    atomic_json(output/'split_audit.json', overlay['audit'])
    layout.read_overlay(overlay_path)
    return dict(manifest=str(output/'manifest.json'), manifest_sha256=sha256(output/'manifest.json'),
        split_overlay=str(overlay_path), split_overlay_sha256=sha256(overlay_path), audit=overlay['audit'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--val-fraction', type=float, default=.15)
    parser.add_argument('--train-asset-floor', type=int, default=5)
    args = parser.parse_args()
    result = prepare(args.manifest, args.output, args.val_fraction, args.train_asset_floor)
    print(json.dumps({k: v for k, v in result.items() if k != 'audit'}, indent=2))
    print(json.dumps({k: v for k, v in result['audit'].items() if k in
        ('status', 'frames', 'splits', 'final_ego_coverage', 'validation_shortfall', 'train_missing_assets')}, indent=2))
