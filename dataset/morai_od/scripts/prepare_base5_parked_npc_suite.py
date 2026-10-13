#!/usr/bin/env python3
"""Offline NPC footprint rollout to future environments, not collection."""
import argparse
import copy
import json
from pathlib import Path

from audit_base5_parked_npc_plan import audit
from base5_capture_safety import atomic_json, sha256
from prepare_base5_parked_npc_support import prepare


def build(source_suite, output):
    if output.exists():
        raise FileExistsError('New NPC-support suite required')
    source = json.loads(source_suite.read_text())
    entries, audits = [], {}
    for environment in source['environments']:
        name = environment['environment']
        if name == 'sunny13':
            entries.append(copy.deepcopy(environment))
            continue
        original = Path(environment['manifest'])
        candidate = prepare(original, output / name)
        proof = audit(original, candidate)
        atomic_json(output / name / 'npc_preservation_audit.json', proof)
        if proof['errors']:
            raise ValueError('NPC-support invariant failed in '+name)
        audits[name] = proof
        entries.append(dict(environment, manifest=str(candidate.resolve()), manifest_sha256=sha256(candidate),
                            capture_output=str((output.parent / ('collection_5k_npc_support_'+name)).resolve()),
                            live_qualified=False, weather_qualified=False))
        print(name, proof['changed_scenes'], 'repaired offline', flush=True)
    suite = copy.deepcopy(source)
    suite.update(environments=entries, source_suite=str(source_suite.resolve()),
                 source_suite_sha256=sha256(source_suite), bulk_started=False,
                 note='Original completed Sunny13 preserved; future25K offline NPC support, native qualification pending')
    atomic_json(output / 'suite.json', suite)
    atomic_json(output / 'npc_suite_audit.json', dict(environments=audits, bulk_started=False,
                                                   all_original_assets_and_slots_preserved=True))
    return output / 'suite.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-suite', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); print(build(args.source_suite, args.output))
