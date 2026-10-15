#!/usr/bin/env python3
"""Attach measured ObjectStatus size signatures to audited geometric boxes.

Print JSON to stdout; asset geometry remains unchanged. Reference metadata is
from controlled named actors, never from detector outputs or evaluation scores.
"""
import argparse
import glob
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path


def build(catalog, pattern):
    catalog['schema_version'] = 2
    observations = defaultdict(Counter)
    digest = hashlib.sha256()
    files = sorted(glob.glob(pattern, recursive=True))
    if not files:
        raise ValueError('No controlled metadata found')
    for filename in files:
        payload = Path(filename).read_bytes()
        digest.update(payload)
        for line in payload.splitlines():
            frame = json.loads(line)
            for actor in frame.get('objects', {}).get('actors', []):
                if actor.get('class') != 'vehicle':
                    continue
                size = actor.get('size_m', {})
                signature = tuple(round(float(size[a]), 3) for a in ('x', 'y', 'z'))
                if min(signature) > 0:
                    observations[actor['name']][signature] += 1
    for name, counts in observations.items():
        if name not in catalog['assets']:
            continue
        if len(counts) != 1:
            raise ValueError(f'Unstable reference dimensions for {name}: {counts}')
        signature, count = counts.most_common(1)[0]
        catalog['assets'][name]['object_status_size_m'] = list(signature)
        catalog['assets'][name]['object_status_size_observations'] = count
    catalog['status_size_reference'] = dict(
        source='260920_base3_online_highway_static_v2 controlled named actors',
        metadata_files=len(files), concatenated_metadata_sha256=digest.hexdigest(),
        assets=sum('object_status_size_m' in a for a in catalog['assets'].values()),
        matching='Match ObjectStatus.size to measured ObjectStatus.size, NOT geometric bbox_size_m',
        maximum_error_m=0.03, minimum_margin_m=0.02,
        shared_geometry_maximum_component_spread_m=0.01)
    return catalog


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catalog', type=Path, default=Path('dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json'))
    p.add_argument('--reference-glob', default='/home/luv/morai_cap/260920_032434_base3_online_highway_static_v2/**/dynamic_metadata/*.frames.jsonl')
    args = p.parse_args()
    print(json.dumps(build(json.loads(args.catalog.read_text()), args.reference_glob), indent=2))


if __name__ == '__main__':
    main()
