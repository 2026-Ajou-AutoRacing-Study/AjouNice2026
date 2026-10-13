#!/usr/bin/env python3
"""CPU-only source gait/minimum mover check after native finite completion."""
import argparse
import json
from pathlib import Path
from base5_capture_safety import atomic_json,load_ledger,sha256
from run_base5_motion_contract_repair import motion_matches


def check(manifest,completed):
    deck=json.loads(manifest.read_text());refs={e['id']:json.loads(Path(e['metadata']).read_text()) for e in deck['scenes']}
    status=json.loads((completed/'status.json').read_text());rows=load_ledger(completed/'selected_ledger.jsonl')
    if status['stage']!='complete' or status['review'] or len(rows)!=len(refs):raise ValueError('Moving completion incomplete')
    if {r['source_cell_id'] for r in rows}!=set(refs):raise ValueError('Moving completion identities differ')
    mismatches=[]
    for row in rows:
        actual=json.loads((Path(row['capture_root'])/'metadata.json').read_text())
        audit=json.loads((Path(row['capture_root'])/'audit.json').read_text())
        if row['status']!='accepted_candidate' or audit['review_reasons'] or not motion_matches(refs[row['source_cell_id']],actual):
            mismatches.append(row['source_cell_id'])
    roundtrip=json.loads((completed/'live_validation.json').read_text())
    proof=dict(qualified=not mismatches,accepted=len(rows),review=0,primary_gait_and_minimum_mover_failures=mismatches,
        source_manifest_sha256=sha256(manifest),selected_ledger_sha256=sha256(completed/'selected_ledger.jsonl'),
        native_roundtrip=roundtrip,parent_preserved=roundtrip['parent_preserved'],
        completed_resume_additional_raw=roundtrip['additional_completed_resume_raw'],
        actual_environment='sunny13',geometry_seed_environment='sunny11',training_data=False,
        interpretation='8 actual animated geometries; initial7 accepted plus1 first-proposal native moving recovery. No gait/ground/GT waiver.')
    atomic_json(completed/'completed_motion_validation.json',proof)
    if mismatches:raise ValueError('Source motion mismatch')
    print(json.dumps(proof,ensure_ascii=False,indent=2));return proof


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--completed',type=Path,required=True);a=p.parse_args();check(a.manifest,a.completed)
