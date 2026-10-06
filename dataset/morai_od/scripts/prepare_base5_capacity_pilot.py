#!/usr/bin/env python3
"""Prepare a family-stratified 32-cell capture-cost sample without loading MR."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen
from select_base5_pilot import select

COUNTS = dict(highway=10, roundabout=2, other_vehicle=1,
              obstacle=9, pedestrian_related=8, background=2)


def prepare(source, output):
    if output.exists():
        raise FileExistsError('New pilot output required')
    data = gen.geo.load_json(source)
    indices = []
    for family, count in COUNTS.items():
        candidates = [(i, r) for i, r in enumerate(data['scenes']) if r['family'] == family]
        chosen = select(dict(scenes=[r for _, r in candidates]), count)
        if len(chosen) != count:
            raise ValueError('Insufficient family candidates: ' + family)
        indices.extend(candidates[i][0] for i in chosen)
    indices.sort()
    pilot = copy.deepcopy(data)
    pilot.update(scenes=[data['scenes'][i] for i in indices], count=len(indices),
        status='32_CELL_CAPACITY_PILOT; LIVE_PENDING',
        capacity_sample=dict(source_manifest=str(source.resolve()), source_sha256=gen.geo.sha256(source),
            source_indices=indices, counts=COUNTS,
            limitation='Rounded family quotas with coverage-biased selection; not unbiased 10K throughput estimator'))
    output.mkdir(parents=True)
    gen.geo.write_json(output / 'manifest.json', pilot)
    return output / 'manifest.json'


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(prepare(a.manifest, a.output))
