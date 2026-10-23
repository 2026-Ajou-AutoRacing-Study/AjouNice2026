#!/usr/bin/env python3
"""Prepare single-parent RedBarrel warm repeats without relaxing capture QA.

Existing mixed failures remain untouched. This bounded deck intentionally uses
the existing single-parent probe contract; it cannot establish cold-process,
multi-actor-load, UID or cross-asset-cache causality.
"""
import argparse
import copy
import json
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
from base5_scale_capture_contract import validate_entry
import prepare_base5_scale_probe as probe
from prepare_base5_scale_qualification import fresh_output


def prepare(manifest,output,unit_reference,*,distance_m=6.,repeats=3):
    if type(repeats) is not int or not 1 <= repeats <= 3:
        raise ValueError('One to three warm repeats required')
    output=fresh_output(output,[Path(manifest).resolve().parent,Path(unit_reference).resolve().parent])
    path=probe.prepare(manifest,output/'base_single',['RedBarrel'],factors=[1.25,1.5],
                       yaws=[0.,90.],distance_m=distance_m,unit_reference=unit_reference)
    base=json.loads(path.read_text())
    entries=[]
    for repeat in range(repeats):
        # Rotate the six fixed conditions. There is no claim that this fully
        # balances first-order carryover or substitutes for cold restarts.
        conditions=base['scenes'][repeat*2:]+base['scenes'][:repeat*2]
        for ordinal,entry in enumerate(conditions):
            scene=json.loads(Path(entry['scenario']).read_text())
            meta=json.loads(Path(entry['metadata']).read_text())
            ident=f'red_warm_{repeat}_{ordinal}_'+entry['id'].split('_',2)[2]
            meta['redbarrel_warm_probe']=dict(repeat_ordinal=repeat,within_repeat_order=ordinal,
                repeated_payload_sha256=entry['scenario_sha256'],single_parent_uid=8500,
                single_actor_load=1,process_state='warm_only; no restart performed by preparer',
                scenario_geometry_identical_across_repeats=True,independent_geometry_samples=False,
                native_size_review_must_be_preserved=True,automatic_retry_authorized=False,
                mixed_scene_qualification=False,cold_uid_load_causality_claim=False)
            sp,mp=(output/'scenarios'/(ident+suffix) for suffix in ('.json','_metadata.json'))
            atomic_json(sp,scene)
            atomic_json(mp,meta)
            if sha256(sp)!=entry['scenario_sha256']:
                raise ValueError('Warm repeat scenario bytes changed')
            entries.append(dict(entry,id=ident,scenario=str(sp),metadata=str(mp),scenario_sha256=sha256(sp)))
    result=copy.deepcopy(base)
    result.update(dataset_id='base5_redbarrel_single_warm_261017',scenes=entries,count=len(entries),
        status='DIAGNOSTIC_ONLY; RedBarrel enlargement remains deferred',
        redbarrel_warm_registration=dict(source_manifest=str(Path(manifest).resolve()),
            source_manifest_sha256=sha256(manifest),base_manifest=str(path),base_manifest_sha256=sha256(path),
            repeats=repeats,independent_geometry_conditions=6,first_attempt_scenes=len(entries),
            single_parent_contract_preserved=True,process_restarted=False,mixed_load_tested=False,
            automatic_retry_authorized=False,native_size_guard_unchanged=True,
            preparation_script_sha256=sha256(Path(__file__))))
    for entry in entries:
        validate_entry(entry,result)
    atomic_json(output/'manifest.json',result)
    return output/'manifest.json'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--unit-reference',type=Path,required=True)
    parser.add_argument('--distance-m',type=float,default=6.)
    parser.add_argument('--repeats',type=int,default=3)
    args=parser.parse_args()
    print(prepare(args.manifest,args.output,args.unit_reference,distance_m=args.distance_m,repeats=args.repeats))
