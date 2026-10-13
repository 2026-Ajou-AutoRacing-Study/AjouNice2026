#!/usr/bin/env python3
"""Merge trusted local four-class infos without splitting same-scene environments."""
import argparse
import pickle
from pathlib import Path
from collections import Counter
from base5_capture_safety import atomic_json


def merge(roots, output):
    if output.exists():raise FileExistsError('Fresh infos output required')
    splits={s:[] for s in ('train','val','test')};groups={};tokens=set();environments=Counter();metainfo=None
    for root in roots:
        for split in splits:
            payload=pickle.loads((root/f'morai_base5_infos_{split}.pkl').read_bytes())
            if metainfo is None:metainfo=payload['metainfo']
            elif metainfo!=payload['metainfo']:raise ValueError('Environment infos contracts differ')
            for row in payload['data_list']:
                tag=row.get('environment_tag');group=row.get('scene_group_id')
                if not tag or not group:raise ValueError('Missing environment tag or scene group')
                if group in groups and groups[group]!=split:raise ValueError('Same-scene environment leaks across splits')
                groups[group]=split
                if row['token'] in tokens:raise ValueError('Duplicate sample token')
                tokens.add(row['token']);environments[tag]+=1;splits[split].append(row)
    output.mkdir(parents=True)
    for split,rows in splits.items():
        with (output/f'morai_base5_infos_{split}.pkl').open('wb') as f:
            pickle.dump(dict(metainfo=metainfo,data_list=rows),f)
    result=dict(samples=len(tokens),independent_scene_groups=len(groups),environments=dict(environments),
        splits={s:len(r) for s,r in splits.items()},group_split_leakage=0,random_resplit=False)
    atomic_json(output/'environment_merge_audit.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--infos',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(merge(a.infos,a.output))
