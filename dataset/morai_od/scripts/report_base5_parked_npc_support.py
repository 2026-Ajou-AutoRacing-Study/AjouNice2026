#!/usr/bin/env python3
"""Read-only map-footprint risk audit of planned parked NPC vehicles.

Map height availability is not proof of road contact or collision clearance.
This report never approves/rejects a capture or rewrites a scenario/GT.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

from base5_capture_safety import atomic_json, sha256


def assess(actor):
    if actor.get('source_list') != 'vehicleList' or actor.get('motion') != 'static':
        return None
    footprint = actor.get('map_ground_initializer', {}).get('footprint', {})
    samples = footprint.get('samples', [])
    corners = samples[1:] if len(samples) == 5 else []
    unknown = not corners
    outside = [s for s in corners if not s.get('usable') or
               s.get('confidence') not in ('road_strip', 'near_centerline')]
    distances = [s.get('nearest_road_distance_m') for s in corners
                 if s.get('nearest_road_distance_m') is not None]
    size = actor.get('estimated_size', [0, 0, 0])
    long = size[0] >= 8
    return dict(asset=actor['name'], uid=actor['unique_id'], long_vehicle=long,
                estimated_size_m=size,
                road_relative_yaw_deg=actor.get('static_yaw_policy', {}).get('road_relative_yaw_deg'),
                map_height_available_all=footprint.get('all_supported'),
                corner_samples=len(corners), roadside_or_unsupported_corners=len(outside),
                maximum_corner_road_distance_m=max(distances) if distances else None,
                map_height_span_m=footprint.get('map_height_span_m'),
                screening_risk=unknown or len(outside) >= 2,
                reason='footprint_unknown' if unknown else 'two_or_more_nonroad_corners' if len(outside) >= 2 else None,
                contact_verified=False)


def report(manifest):
    deck = json.loads(manifest.read_text())
    counts, assets, families, rows = Counter(), Counter(), Counter(), []
    for entry in deck['scenes']:
        metadata = json.loads(Path(entry['metadata']).read_text())
        for actor in metadata['runtime_actors']:
            value = assess(actor)
            if value is None:
                continue
            counts['parked_npc'] += 1
            counts['long_parked_npc'] += int(value['long_vehicle'])
            if value['screening_risk']:
                counts['map_screening_risk'] += 1
                counts['long_map_screening_risk'] += int(value['long_vehicle'])
                assets[value['asset']] += 1
                families[metadata['spec']['family']] += 1
            rows.append(dict(scene_id=entry['id'], family=metadata['spec']['family'],
                             mission=metadata['spec']['mission'], **value))
    return dict(manifest=str(manifest.resolve()), manifest_sha256=sha256(manifest),
                frames=len(deck['scenes']), counts=dict(counts), risk_assets=dict(assets),
                risk_families=dict(families), actors=rows, capture_qa_changed=False,
                limitation='Map-only hypothesis screening, not actual mesh contact, collider overlap, '
                           'or native failure probability. all_supported means usable height estimates.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = report(args.manifest)
    atomic_json(args.output, result)
    print(json.dumps({k: v for k, v in result.items() if k != 'actors'}, ensure_ascii=False, indent=2))
