#!/usr/bin/env python3
"""Six same-geometry 5K manifests with fixed scene groups, separate output roots."""
import argparse
import copy
import json
from pathlib import Path
from base5_capture_safety import atomic_json, sha256


def prepare(source, output):
    if output.exists():raise FileExistsError('Fresh suite output required')
    original=json.loads(source.read_text())
    if len(original['scenes'])!=5000:raise ValueError('Exactly 5K geometry cells required')
    output.mkdir(parents=True)
    catalog=[]
    for weather in ('sunny','foggy'):
        for hour in (11,13,15):
            tag=f'{weather}{hour}';deck=copy.deepcopy(original)
            deck['dataset_id']=f'base5_regions_v123_{tag}'
            deck['config']['dataset_id']=deck['dataset_id']
            deck['config']['collection_environment']=dict(weather=weather,hour=hour,
                policy='Operator UI verification required; this manifest does not apply MORAI weather/time')
            deck['config']['environment_tag']=tag
            deck['provenance']['environment_source_manifest_sha256']=sha256(source)
            deck['provenance']['config_sha256']=__import__('hashlib').sha256(json.dumps(deck['config'],sort_keys=True).encode()).hexdigest()
            deck['status']='ENVIRONMENT_PREPARED; UI confirmation and bulk approval required'
            path=output/tag/'manifest.json';atomic_json(path,deck)
            catalog.append(dict(environment=tag,weather=weather,hour=hour,manifest=str(path.resolve()),
                manifest_sha256=sha256(path),capture_output=str((output.parent/('collection_5k_regions_v123_'+tag)).resolve()),
                expected_frames=5000))
    for item in catalog:
        deck=json.loads(Path(item['manifest']).read_text())
        if deck['scenes']!=original['scenes']:raise ValueError('Environment changed geometry or split')
    atomic_json(output/'suite.json',dict(total_expected_frames=30000,source_manifest=str(source.resolve()),
        environments=catalog,same_scene_geometry_and_split=True,
        split_warning='Never random-split 30K frames; all six scene variants remain in their original split.',
        dataset_warning='30K observations, only 5K independent geometry scenes; not six independent datasets.',
        bulk_started=False))
    return output/'suite.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.manifest,a.output))
