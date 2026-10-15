#!/usr/bin/env python3
"""Prepare twelve bounded smoke cells offline; never loads MR or starts capture."""
import argparse
import copy
from pathlib import Path

import generate_base5_independent as gen
from prepare_base5_appearance_pilot import prepare


def prepare_smoke(output):
    manifest_path = prepare(output)  # Nine ordinary near/mixed appearance cells.
    manifest = gen.geo.load_json(manifest_path)
    config = manifest['config']
    ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
    rows = gen.specs(config)
    for offset, role in enumerate(('front', 'rear')):
        row = copy.deepcopy(next(r for r in rows if r['family'] == 'highway' and not r['mission']))
        row.update(index=33000 + offset, role=role, actors=1, distance_bounds=[80, 99],
                   topology='highway_three_lane', yaw_limit=3, motion='static',
                   asset_index=next(i for i, a in enumerate(ctx['pools']['vehicle'])
                                    if a['name'] == '2016_Hyundai_Ioniq'))
        append_cell(manifest, output, row, ctx, f'b5official33_{offset:02d}')
    row = copy.deepcopy(next(r for r in rows if r['family'] == 'obstacle'
                             and r.get('avoidance_phase') == 'alongside'))
    # avoidance_side is Ego's passing side; the object is on the opposite side.
    row.update(index=33002, actors=1, role='left', avoidance_side='right',
               motion='static')
    append_cell(manifest, output, row, ctx, 'b5official33_02')
    manifest.update(count=len(manifest['scenes']),
                    status='OFFLINE_ONLY; LIVE_CONNECTION_AND_CAPTURE_PENDING')
    gen.geo.write_json(manifest_path, manifest)
    return manifest_path


def append_cell(manifest, output, row, ctx, sid):
    failures = {}
    for retry in range(1024):
        try:
            scene, meta = gen.build(row, manifest['config'], ctx, retry)
            if row.get('avoidance_phase') == 'alongside' and meta['runtime_actors'][0]['actual_ego_role'] != row['role']:
                raise ValueError('Alongside actual side differs from smoke quota')
            break
        except ValueError as exc:
            failures[str(exc)] = failures.get(str(exc), 0) + 1
    else:
        raise ValueError(f'Unfillable smoke cell {sid}: {failures}')
    sp, mp = output / f'{sid}.json', output / f'{sid}_metadata.json'
    gen.geo.write_json(sp, scene)
    gen.geo.write_json(mp, meta)
    manifest['scenes'].append(dict(id=sid, scenario=str(sp.resolve()), metadata=str(mp.resolve()),
        scenario_sha256=gen.geo.sha256(sp), split=meta['split'], family=row['family'],
        mission=row['mission'], distance_bounds=row['distance_bounds'], role=row['role'],
        motion=meta['runtime_actors'][0]['motion'], subfamily=row['subfamily']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(prepare_smoke(parser.parse_args().output))
