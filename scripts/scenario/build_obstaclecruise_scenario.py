#!/usr/bin/env python3
"""Copy an official scenario, replacing only ego position and rotation."""
import argparse
import copy
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--pose-source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('output already exists; refusing overwrite')
    raw = a.source.read_bytes()
    original = json.loads(raw)
    pose = json.loads(a.pose_source.read_text())['egoVehicle']['initPosition']
    result = copy.deepcopy(original)
    for key in ('pos', 'rot'):
        result['egoVehicle']['initPosition'][key] = copy.deepcopy(pose[key])
    check = copy.deepcopy(result)
    for key in ('pos', 'rot'):
        check['egoVehicle']['initPosition'][key] = original['egoVehicle']['initPosition'][key]
    assert check == original, 'unexpected scenario modification'
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(source=str(a.source), source_sha256=hashlib.sha256(raw).hexdigest(),
        output=str(a.output), ego_pose=result['egoVehicle']['initPosition']['pos'],
        npc_count=len(result['vehicleList']), spawn_count=len(result['spawnPointList']),
        changed_fields=['egoVehicle.initPosition.pos', 'egoVehicle.initPosition.rot']), indent=2))


if __name__ == '__main__':
    main()
