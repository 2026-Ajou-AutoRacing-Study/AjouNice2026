#!/usr/bin/env python3
"""Merge independently qualified disjoint asset sets without changing approvals."""
import argparse
import copy
import json
from pathlib import Path

import base5_asset_scale as scale
from base5_capture_safety import atomic_json, sha256


def merge(paths, output):
    output=Path(output).resolve()
    if output.exists():
        raise FileExistsError('New policy output required')
    result=None; sources=[]
    for source in paths:
        source=Path(source).resolve()
        policy=scale.validate_policy(json.loads(source.read_text()))
        if result is None:
            result={k:copy.deepcopy(policy[k]) for k in
                ('schema','policy','continuous_jitter','training_gt_modified')}
            result['assets']={}
        if any(policy[k]!=result[k] for k in ('schema','policy','continuous_jitter','training_gt_modified')):
            raise scale.ScaleContractError('Qualification policy versions differ')
        if set(result['assets'])&set(policy['assets']):
            raise scale.ScaleContractError('Asset overlaps require a new explicit review, not overwrite')
        if not policy['assets'] or any(not scale._qualified(p)
                for row in policy['assets'].values() for p in row['profiles'].values()):
            raise scale.ScaleContractError('Only completely qualified nonempty policies can merge')
        result['assets'].update(copy.deepcopy(policy['assets']))
        sources.append(dict(path=str(source),sha256=sha256(source)))
    if result is None:
        raise ValueError('At least one qualified source policy required')
    scale._pins(sources)
    result['qualification']=dict(schema='base5_disjoint_native_scale_policy_merge_v1',
        source_policies=sources,profiles_changed=False,raw_files_modified=False,bulk_started=False)
    scale.validate_policy(result)
    atomic_json(output,result)
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--policy',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(merge(a.policy,a.output))
