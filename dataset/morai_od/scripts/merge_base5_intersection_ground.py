#!/usr/bin/env python3
"""Publish a fresh lookup, preserving independently verified per-asset live reports."""
import argparse
import copy
import json
from pathlib import Path

from base5_capture_safety import atomic_json, sha256
from base5_intersection_ground import POLICY, load_lookup


def merge(inputs, output):
    if output.exists():
        raise FileExistsError('New immutable release directory required')
    profiles, sources, parents = {}, {}, []
    captures = accepted = 0
    for path in inputs:
        data = load_lookup(path)
        if data['status'] != 'LIVE_VALIDATED':
            raise ValueError('Only live-validated lookups may be merged')
        proof = data['live_evidence']; ep = Path(proof['path'])
        if sha256(ep) != proof['sha256']:
            raise ValueError('Live report changed')
        evidence = json.loads(ep.read_text())
        if evidence['status'] != 'COMPLETE' or set(data['profiles']) != set(evidence['validated_profiles']):
            raise ValueError('Lookup profiles differ from authoritative live report')
        if evidence['accepted'] != evidence['captures']:
            raise ValueError('Release cannot omit failed validation evidence')
        for item in evidence.get('files', []):
            if sha256(Path(item['path'])) != item['sha256']:
                raise ValueError('Archived live evidence changed')
        for key, profile in data['profiles'].items():
            if key in profiles:
                raise ValueError('Ambiguous duplicate profile ' + key)
            profiles[key] = copy.deepcopy(profile)
        for item in data['sources'] + [dict(path=str(path.resolve()), sha256=sha256(path)),
                                       dict(path=str(ep.resolve()), sha256=sha256(ep))]:
            old = sources.setdefault(item['path'], item)
            if old['sha256'] != item['sha256']:
                raise ValueError('Conflicting source checksum')
        captures += evidence['captures']; accepted += evidence['accepted']
        parents.append(dict(lookup=str(path.resolve()), lookup_sha256=sha256(path),
                            report=str(ep.resolve()), report_sha256=sha256(ep),
                            captures=evidence['captures'], accepted=evidence['accepted']))
    output.mkdir(parents=True)
    scope = ('Operational QA of each asset in two directions on its measured intersection path; '
             'not a terrain mesh or physical collider contact certificate. No unlimited extrapolation.')
    ep = output/'live_report.json'
    atomic_json(ep, dict(status='COMPLETE', parents=parents, captures=captures, accepted=accepted,
                         validated_profiles=sorted(profiles), validation_scope=scope))
    lookup = dict(policy=POLICY, status='LIVE_VALIDATED', profiles=profiles,
                  sources=list(sources.values()), validation_scope=scope,
                  live_evidence=dict(path=str(ep.resolve()), sha256=sha256(ep), validated_profiles=sorted(profiles)))
    atomic_json(output/'height_lookup_live.json', lookup)
    return output/'height_lookup_live.json'


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', type=Path, nargs='+', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); print(merge(a.inputs, a.output))
