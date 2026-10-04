#!/usr/bin/env python3
"""Controlled stopped-lead comparison, not reproduction of randomized trial 17."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('source', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
if a.output.exists():
    p.error('refusing overwrite')
raw = a.source.read_bytes()
s = json.loads(raw)
v = copy.deepcopy(next(v for v in s['vehicleList'] if v['UNIQUEID'] == 38))
pos = v['initPosition']['pos']
for axis, value in [('x', 74.9), ('y', -100.0), ('z', 28.7)]:
    pos[axis] = value
    if '_'+axis in pos:
        pos['_'+axis] = str(value)
v['initPosition']['rot']['yaw'] = '-88.34'
for key in ['constantVelocity', 'desiredVelocity', 'currentVelocity', 'viewCurrentVelocity']:
    v[key] = 0.0
v['velocityType'] = 2
v['isLaneChange'] = False
v['isCloseLoop'] = False
v['spawnPointUniqueID'] = -1
saved = v['initPosition'].get('vehicleSaveData', {})
saved['velocity'] = 0.0
for data in [saved, saved.get('vehicle', {})]:
    if 'lastVelocity' in data:
        data['lastVelocity'] = dict(x=0.0, y=0.0, z=0.0)
s['vehicleList'] = [v]
for key in ['spawnPointList', 'pedestrianList', 'pedspawnPointList', 'aiPedstrianList']:
    s[key] = []
a.output.write_text(json.dumps(s, indent=2)+'\n')
print(json.dumps(dict(source_sha256=hashlib.sha256(raw).hexdigest(),
    output_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),
    npc_id=38, position=pos, desired_velocity=0,
    note='Verify actual NPC position and zero velocity in bag before accepting pair.'),indent=2))
