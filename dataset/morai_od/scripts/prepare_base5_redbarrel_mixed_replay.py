#!/usr/bin/env python3
"""Prepare exact historical mixed RedBarrel replays with unchanged old checks.

The historical eight-asset policy is pinned solely to validate the captured
scenario contract. Replaying it is NOT reapproval of deferred RedBarrel scales.
The single-parent asset_scale_probe exception is never enabled or bypassed.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil

import base5_asset_scale as scale
from base5_capture_safety import atomic_json,sha256
from base5_scale_capture_contract import validate_entry
from prepare_base5_scale_qualification import fresh_output


IDS=('sunny13_sunny11_b5v2_02170','sunny13_sunny11_b5v2_04192')


def prepare(manifest,policy_file,historical_capture_root,output,*,repeats=3):
    if type(repeats) is not int or not 1 <= repeats <= 3:
        raise ValueError('One to three exact mixed warm repeats required')
    source,policy_path,historical,output=map(lambda p:Path(p).resolve(),
        (manifest,policy_file,historical_capture_root,output))
    output=fresh_output(output,[source.parent,policy_path.parent,historical])
    deck=json.loads(source.read_text())
    policy=json.loads(policy_path.read_text())
    if (deck['config'].get('capture',{}).get('asset_scale_probe') is True
            or deck['config'].get(scale.CONFIG_KEY)!=policy or 'RedBarrel' not in policy['assets']):
        raise ValueError('Exact historical eight-asset policy required; no probe-contract bypass allowed')
    frozen=scale.freeze_policy(policy)
    validation=copy.deepcopy(deck)
    validation['config'][scale.CONFIG_KEY]=frozen
    entries={entry['id']:entry for entry in deck['scenes']}
    if any(ident not in entries for ident in IDS):
        raise ValueError('Both exact historical mixed IDs required')
    pins={str(source):sha256(source),str(policy_path):sha256(policy_path)}
    original_metadata={}
    for ident in IDS:
        entry=entries[ident]
        fresh_output(output,[Path(entry['scenario']).parent,Path(entry['metadata']).parent])
        if sha256(entry['scenario'])!=entry['scenario_sha256']:
            raise ValueError('Historical planned scenario changed')
        validate_entry(entry,validation)
        meta=json.loads(Path(entry['metadata']).read_text())
        if sum(actor.get('name')=='RedBarrel' for actor in meta.get('runtime_actors',[]))<2:
            raise ValueError('Historical mixed replay requires multiple original RedBarrel parents')
        captures=list(historical.glob('chunk_*/capture/captures/'+ident))
        if len(captures)!=1:
            raise ValueError('One original first-attempt capture required per historical ID')
        captured=captures[0]
        if sha256(captured/'scenario.json')!=entry['scenario_sha256']:
            raise ValueError('Replay differs from original historical first-attempt raw scenario')
        captured_meta=json.loads((captured/'metadata.json').read_text())
        if any(captured_meta.get(key)!=meta.get(key) for key in ('spec','runtime_actors','scale_probe')):
            raise ValueError('Historical planned/captured identity or scale differs')
        for path in [Path(entry['scenario']),Path(entry['metadata']),captured/'scenario.json',
                     captured/'metadata.json',captured/'audit.json']:
            pins[str(path.resolve())]=sha256(path)
        original_metadata[ident]=meta
    result=[]
    for repeat in range(repeats):
        ordered=IDS if repeat%2==0 else IDS[::-1]
        for position,ident in enumerate(ordered):
            entry=entries[ident]
            new_id=f'red_mixed_warm_{repeat}_{position}_{ident}'
            meta=copy.deepcopy(original_metadata[ident])
            meta['redbarrel_mixed_replay']=dict(source_id=ident,repeat_ordinal=repeat,
                within_repeat_order=position,scenario_geometry_identical_across_repeats=True,
                historical_policy_validation_only=True,redbarrel_enlargement_reapproved=False,
                process_state='warm_only; operator records actual state',process_restarted=False,
                automatic_retry_authorized=False,original_size_failure_must_be_retained=True,
                independent_geometry_samples=False,uid_and_actor_load_preserved=True)
            meta['scale_stability']=dict(condition='scaled',pair_id=None,source_id=ident,
                independent_geometry_unit=ident,source_actor_load=len(meta['runtime_actors']),
                source_actor_uids=[actor['unique_id'] for actor in meta['runtime_actors']],
                repeat_ordinal=repeat,process_state='warm_only; no cold or UID/load attribution',
                counterfactual_distribution_claim=False,native_qa_unchanged=True)
            sp,mp=(output/'scenarios'/(new_id+suffix) for suffix in ('.json','_metadata.json'))
            sp.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(entry['scenario'],sp)
            atomic_json(mp,meta)
            if sha256(sp)!=entry['scenario_sha256']:
                raise ValueError('Exact replay scenario bytes differ')
            result.append(dict(entry,id=new_id,scenario=str(sp),metadata=str(mp),scenario_sha256=sha256(sp)))
    replay=copy.deepcopy(deck)
    # The exact historical Red layouts contain no prepared moving shadow
    # actor. Do not import an unrelated, historical ground-registry runtime
    # into a static diagnostic. All Red/scale/native/ground checks stay intact.
    from base5_shadow_ground_reuse import KEY as SHADOW_REUSE_KEY
    if any(SHADOW_REUSE_KEY in meta for meta in original_metadata.values()):
        raise ValueError('Static Red replay cannot strip an active shadow preparation proof')
    unrelated_shadow=replay['config'].pop(SHADOW_REUSE_KEY,None)
    replay.update(dataset_id='base5_redbarrel_exact_mixed_replay_261017',scenes=result,count=len(result),
        status='HISTORICAL_DIAGNOSTIC_REPLAY_ONLY; no production or RedBarrel scale reapproval',
        redbarrel_mixed_replay_registration=dict(source_files=pins,historical_policy_path=str(policy_path),
            historical_policy_sha256=sha256(policy_path),historical_policy_validation_only=True,
            source_ids=list(IDS),independent_payloads=2,repeats=repeats,first_attempt_scenes=len(result),
            asset_scale_probe_enabled=False,native_size_guard_unchanged=True,bulk_approved=False,
            removed_unrelated_shadow_registration=unrelated_shadow,
            production_profile_reapproved=False,raw_files_modified=False,
            preparation_script_sha256=sha256(Path(__file__))))
    check=dict(replay,config=dict(replay['config'],**{scale.CONFIG_KEY:frozen}))
    for entry in result:
        validate_entry(entry,check)
    if any(sha256(path)!=value for path,value in pins.items()):
        raise ValueError('Historical source changed during read-only replay preparation')
    scale.verify_frozen_policy(frozen)
    atomic_json(output/'manifest.json',replay)
    return output/'manifest.json'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--historical-policy',type=Path,required=True)
    parser.add_argument('--historical-capture-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repeats',type=int,default=3)
    args=parser.parse_args()
    print(prepare(args.manifest,args.historical_policy,args.historical_capture_root,args.output,
                  repeats=args.repeats))
