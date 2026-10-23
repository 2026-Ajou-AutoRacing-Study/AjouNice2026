#!/usr/bin/env python3
"""Revalidate saved terrain preparation and materialize it before split design.

No old raw/session is resumed, copied as new training data, or edited. The two
registered prepared *scenario inputs* are placed in a new full deck; new native
capture remains required. This makes actual future actor/path geometry visible
to the layout allocator, rather than changing it after split allocation.
"""
import argparse
import copy
import json
from pathlib import Path

import base5_shadow_ground_reuse as reuse
from base5_capture_safety import atomic_json,sha256


def prepare(manifest,historical_registry,output):
    manifest,historical_registry,output=map(lambda p:Path(p).resolve(),(manifest,historical_registry,output))
    if output.exists() or manifest.parent==output or manifest.parent in output.parents:
        raise FileExistsError('Fresh output outside preserved source required')
    source=json.loads(manifest.read_text())
    historical=json.loads(historical_registry.read_text())
    if (source.get('count')!=5000 or len(source.get('scenes',[]))!=5000
            or reuse.KEY in source['config'] or source.get('split_overlay')
            or source['config'].get('native_unit_geometry_reference')):
        raise ValueError('Full source before ground/unit/layout overlays required')
    pins={str(manifest):sha256(manifest),str(historical_registry):sha256(historical_registry)}
    pins.update(historical['input_sha256'])
    if any(sha256(path)!=checksum for path,checksum in pins.items()):
        raise ValueError('Saved ground preparation evidence changed')
    selections=[dict(source_id=row['source_id'],mode=row['mode'],
        prepared_manifest=row['prepared_manifest']['path'],selected_ledger=row['selected_ledger']['path'])
        for row in historical['entries']]
    if len(selections)!=2 or len({row['source_id'] for row in selections})!=2:
        raise ValueError('Exactly the two explicitly registered shadow source cells required')
    registered=reuse.register(manifest,selections,output/'registered')
    deck=json.loads(registered.read_text())
    originals={entry['id']:entry for entry in deck['scenes']}
    replacements={}
    for selection in selections:
        ident=selection['source_id']
        subset=copy.deepcopy(deck)
        subset.update(count=1,scenes=[originals[ident]])
        one=output/'single_sources'/ident/'manifest.json'
        atomic_json(one,subset)
        adopted=reuse.adopt(one,output/'prepared_scenarios'/ident)
        if adopted is None:
            raise ValueError('Registered shadow scenario was not adopted')
        prepared=json.loads(adopted.read_text())
        replacements[ident]=prepared['scenes'][0]
    result=copy.deepcopy(deck)
    result['scenes']=[replacements.get(entry['id'],entry) for entry in deck['scenes']]
    result.update(status='PREPARED_SHADOW_GEOMETRY_MATERIALIZED_CPU_ONLY; new native capture required',bulk_ready=False)
    result.setdefault('provenance',{})['shadow_ground_materialization']=dict(
        source_manifest=str(manifest),source_manifest_sha256=pins[str(manifest)],
        historical_registry=dict(path=str(historical_registry),sha256=pins[str(historical_registry)]),
        registered_manifest=dict(path=str(registered),sha256=sha256(registered)),
        helper_sha256=sha256(__file__),materialized_ids=sorted(replacements),
        old_raw_is_not_new_training_data=True,new_native_capture_required=True)
    reuse.validate_manifest(result)
    for path,checksum in pins.items():
        if sha256(path)!=checksum:raise ValueError('Preserved ground source changed')
    final=output/'manifest.json'
    atomic_json(final,result)
    atomic_json(output/'materialization.json',dict(manifest=str(final),manifest_sha256=sha256(final),
        source_pins=pins,materialized_ids=sorted(replacements),unchanged_entries=4998,
        native_capture_started=False,training_started=False))
    return final


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--historical-registry',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(prepare(args.manifest,args.historical_registry,args.output))
