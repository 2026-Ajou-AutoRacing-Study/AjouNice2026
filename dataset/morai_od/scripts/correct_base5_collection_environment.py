#!/usr/bin/env python3
"""Correct operator-declared environment in a NEW manifest; never edit captures.

This is an annotation correction, not a command to change MORAI weather/time.
Existing resume contracts and raw evidence must keep their original bytes.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path


def digest(data):
    return hashlib.sha256(data).hexdigest()


def correct(source,output,weather='sunny',hour=13,operator_confirmed=False):
    source=Path(source);output=Path(output)
    if not operator_confirmed:
        raise ValueError('Actual MR environment requires explicit operator confirmation')
    if weather not in ('sunny','foggy') or hour not in (11,13,15):
        raise ValueError('Unsupported competition environment')
    record_path=output.parent/'environment_correction.json'
    if source.resolve()==output.resolve() or output.exists() or record_path.exists():
        raise FileExistsError('In-place correction or overwrite refused; use a new output')
    before=source.read_bytes();original=json.loads(before);result=copy.deepcopy(original)
    previous=copy.deepcopy(result['config'].get('collection_environment'))
    ident=result.get('dataset_id',result['config']['dataset_id'])
    for old in ('sunny11','sunny13','sunny15','foggy11','foggy13','foggy15'):
        if old in ident:
            ident=ident.replace(old,weather+str(hour));break
    result['dataset_id']=ident;result['config']['dataset_id']=ident
    result['config']['collection_environment']=dict(weather=weather,hour=hour,
        policy='operator setting; verify actual MR before capture')
    proof=dict(source_manifest=str(source.resolve()),source_manifest_sha256=digest(before),
        previous_declared_environment=previous,corrected_environment=dict(weather=weather,hour=hour),
        basis='operator-confirmed actual MR setting; previous declared hour was mistaken',
        independently_measured=False,scenario_payloads_and_metadata_unchanged=True,
        policy='new annotation only; existing capture/ledger/resume contract untouched')
    provenance=result.setdefault('provenance',{})
    provenance['config_sha256']=digest(json.dumps(result['config'],sort_keys=True).encode())
    provenance['environment_correction']=proof
    assert result['scenes']==original['scenes']
    output.parent.mkdir(parents=True,exist_ok=True)
    payload=(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode()
    with output.open('xb') as f:f.write(payload)
    with record_path.open('x') as f:
        json.dump(dict(proof,corrected_manifest=str(output.resolve()),
            corrected_manifest_sha256=digest(payload),scenes=len(result['scenes'])),f,ensure_ascii=False,indent=2)
        f.write('\n')
    if source.read_bytes()!=before:raise RuntimeError('Source manifest changed during correction')
    return dict(manifest=str(output.resolve()),record=str(record_path.resolve()),
        scenes=len(result['scenes']),scenario_payloads_and_metadata_unchanged=True,
        manifest_sha256=digest(payload))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--weather',choices=('sunny','foggy'),default='sunny')
    p.add_argument('--hour',type=int,choices=(11,13,15),default=13)
    p.add_argument('--operator-confirmed',action='store_true')
    a=p.parse_args()
    print(json.dumps(correct(a.manifest,a.output,a.weather,a.hour,a.operator_confirmed),ensure_ascii=False,indent=2))
