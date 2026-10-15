#!/usr/bin/env python3
"""Offline five-environment support rollout; preserve the completed Sunny13."""
import argparse
import json
from pathlib import Path

from prepare_base5_obstacle_support import prepare
from audit_base5_obstacle_support_plan import audit
from base5_capture_safety import atomic_json,sha256


def build(source_suite,output):
    if output.exists():raise FileExistsError('New suite output required')
    original=json.loads(source_suite.read_text());environments=[];proofs={}
    for environment in original['environments']:
        tag=environment['environment']
        if tag=='sunny13':
            environments.append(dict(environment,preserved_completed=True))
            continue
        source=Path(environment['manifest'])
        candidate=prepare(source,output/tag,'richfloor',full=True)
        proof=audit(source,candidate);atomic_json(output/tag/'preservation_audit.json',proof)
        if proof['errors']:raise ValueError('Support rollout invariant failed '+tag)
        proofs[tag]=proof
        environments.append(dict(environment,manifest=str(candidate.resolve()),manifest_sha256=sha256(candidate),
            capture_output=str((output.parent/('collection_5k_support_'+tag)).resolve()),
            live_qualified=False,weather_qualified=False))
    doc=dict(schema=3,total_expected_frames=30000,environments=environments,
        source_suite=str(source_suite.resolve()),source_suite_sha256=sha256(source_suite),
        original_sunny13_preserved=True,new_environment_capture_started=False,bulk_started=False,
        note='Five offline 5K candidates plus original completed Sunny13; separate Sunny13 preview is not added to 30K')
    atomic_json(output/'suite.json',doc)
    atomic_json(output/'suite_audit.json',dict(environments=proofs,bulk_started=False,
        weather_qualified=False,all_primary_and_original_assets_preserved=True))
    return output/'suite.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-suite',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(build(a.source_suite,a.output))
