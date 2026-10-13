#!/usr/bin/env python3
"""Re-evaluate raw captures with current policy; preserve original audit and ledger."""
import argparse
import json
from pathlib import Path
import hashlib

import base5_capture_audit
from run_base5_capture import atomic_json


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists() and any(a.output.iterdir()):raise FileExistsError('Fresh audit version required')
    a.output.mkdir(parents=True,exist_ok=True);latest={}
    for line in a.ledger.read_text().splitlines():
        row=json.loads(line);latest[row['id']]=row
    rows=[]
    for row in latest.values():
        row=dict(row)
        if row['status']=='failed':rows.append(row);continue
        root=Path(row['capture_root']);meta=json.loads((root/'metadata.json').read_text())
        audit=base5_capture_audit.audit_capture(root,meta)
        telemetry=json.loads((root/'telemetry.json').read_text())
        if telemetry.get('capture_drift_m',0)>.08:
            audit['review_reasons'].append('capture_ego_drift');audit['accepted_candidate']=False
        audit['audit_source_sha256']=hashlib.sha256(Path(base5_capture_audit.__file__).read_bytes()).hexdigest()
        destination=a.output/(row['id']+'_audit.json');atomic_json(destination,audit)
        row.update(audit=audit,audit_path=str(destination.resolve()),
                   status='accepted_candidate' if audit['accepted_candidate'] else 'review')
        rows.append(row)
    with (a.output/'ledger.jsonl').open('w') as f:
        for row in rows:f.write(json.dumps(row)+'\n')
    print(a.output/'ledger.jsonl')
