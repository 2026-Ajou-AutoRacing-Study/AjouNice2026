#!/usr/bin/env python3
"""Archive exactly the ten requested editor results and preserve their actual XYZ."""
import argparse
import json
from pathlib import Path
import shutil

from base5_capture_safety import atomic_json, sha256
from base5_intersection_ground import POLICY, profile_from_scene


def collect(prepared, output, move=False):
    if output.exists():
        raise FileExistsError('New manual archive required')
    manifest = json.loads(prepared.read_text()); runtime = Path(manifest['runtime'])
    files, profiles, missing = [], {}, []
    for row in manifest['scenes']:
        original, corrected = runtime/row['original_file'], runtime/row['corrected_file']
        if sha256(original) != row['scenario_sha256']:
            raise ValueError('Prepared original scenario changed: '+str(original))
        scene = json.loads(corrected.read_text())
        for container in ('vehicleList','objectList','spawnPointList','pedspawnPointList','aiPedstrianList'):
            if scene.get(container):
                raise ValueError('Manual calibration unexpectedly gained other actors/spawns')
        key = row['crosswalk']['crosswalk_id']+'/'+row['asset']
        profiles[key] = profile_from_scene(scene, row)
        files += [(original, Path('original')/original.name), (corrected, Path('corrected')/corrected.name)]
        for group, stem in (('original',row['name']),('corrected',str(row['number']))):
            src = runtime/'EgoNetwork'/(stem+'_MN.json')
            if src.exists(): files.append((src,Path(group)/'EgoNetwork'/src.name))
            else: missing.append(str(src))
    inventory = [(src, rel, sha256(src)) for src, rel in files]
    output.mkdir(parents=True)
    sources = []
    for src, rel, digest in inventory:
        dst = output/rel; dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,dst)
        if sha256(dst) != digest:
            raise ValueError('Manual backup checksum mismatch')
        sources.append(dict(path=str(dst.resolve()),sha256=digest,original_path=str(src)))
    shutil.copy2(prepared,output/'prepared_manifest.json')
    sources.append(dict(path=str((output/'prepared_manifest.json').resolve()),sha256=sha256(prepared)))
    lookup = dict(schema_version=1,policy=POLICY,status='MANUAL_SAMPLES_PENDING_LIVE',
                  profiles=profiles,sources=sources,missing_sidecars=missing,
                  limitation='Actual editor waypoints; finite strip interpolation only. First seed may remain uncorrected. Not a universal mesh/contact certificate.')
    atomic_json(output/'height_lookup_pending.json',lookup)
    if any(sha256(src)!=digest for src,_,digest in inventory):
        raise ValueError('MR source changed during backup; do not move')
    if move:
        for src,_,_ in inventory: src.unlink()
    atomic_json(output/'archive_report.json',dict(files=sources,missing_sidecars=missing,
                moved_from_mr=move,profile_count=len(profiles),waypoint_count=sum(len(p['knots']) for p in profiles.values())))
    return output/'height_lookup_pending.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared-manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--move-from-mr',action='store_true')
    a=p.parse_args();print(collect(a.prepared_manifest,a.output,a.move_from_mr))
