#!/usr/bin/env python3
"""Register all NPC-repaired slots plus unchanged controls before capture."""
import argparse
import copy
import json
from pathlib import Path

from base5_capture_safety import atomic_json, sha256
from base5_motion_ground import moving_actors


def prepare(manifest, previous_registration, output):
    if output.exists():
        raise FileExistsError('New bounded qualification registration required')
    deck = json.loads(manifest.read_text())
    previous = json.loads(previous_registration.read_text())
    index = {entry['id']: entry for entry in deck['scenes']}
    changed = [entry for entry in deck['scenes'] if json.loads(Path(entry['metadata']).read_text())
               .get('parked_npc_support_source', {}).get('geometry_changed')]
    if not changed or len(changed) > 64:
        raise ValueError('Changed NPC cohort must fit a bounded 1–64 scene pilot')
    ids = {entry['id'] for entry in changed}
    controls = [index[row['id']] for row in previous['static'] if row['id'] not in ids]
    selected = changed + controls[:64-len(changed)]
    moving = [index[row['id']] for row in previous['animated']]
    if len(selected) != 64 or len(moving) != 8:
        raise ValueError('Expected static64 and moving8 regression coverage')
    if any(moving_actors(json.loads(Path(entry['metadata']).read_text())) for entry in selected):
        raise ValueError('Actual moving actor entered static pilot')
    if any(not moving_actors(json.loads(Path(entry['metadata']).read_text())) for entry in moving):
        raise ValueError('Moving regression lacks actual mover')
    for name, entries in (('static64', selected), ('animated8', moving)):
        subset = copy.deepcopy(deck); subset.update(scenes=entries, count=len(entries),
            status='BOUNDED_NPC_SUPPORT_QUALIFICATION_ONLY; not training or bulk')
        atomic_json(output / name / 'manifest.json', subset)
    atomic_json(output / 'registration.json', dict(source_manifest_sha256=sha256(manifest),
                previous_registration_sha256=sha256(previous_registration),
                all_changed_slots=[entry['id'] for entry in changed],
                controls=[entry['id'] for entry in selected[len(changed):]],
                static=[dict(id=entry['id'], sha256=entry['scenario_sha256']) for entry in selected],
                animated=[dict(id=entry['id'], sha256=entry['scenario_sha256']) for entry in moving],
                selection='All repaired slots, then earlier fixed control order; no result-based cherry-picking',
                bulk_started=False))
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'previous-registration', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args(); print(prepare(args.manifest, args.previous_registration, args.output))
