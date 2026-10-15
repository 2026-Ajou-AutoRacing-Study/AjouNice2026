#!/usr/bin/env python3
"""Select canonical accepted cells from safely terminated sessions; preserve all originals."""
import argparse
import json
from pathlib import Path
from base5_capture_safety import atomic_json,load_ledger,sha256,motion_capture_review


def select(ledgers,output):
    if output.exists():raise FileExistsError('Selected ledger overwrite refused')
    latest={};sources=[]
    for path in ledgers:
        shutdown=json.loads((path.parent/'shutdown_safety.json').read_text())
        if not shutdown['park_verified']:raise ValueError('Unsafe source session '+str(path))
        sources.append(dict(path=str(path.resolve()),sha256=sha256(path)))
        for row in load_ledger(path):
            if row['status']!='accepted_candidate':continue
            root=Path(row['capture_root']);meta=json.loads((root/'metadata.json').read_text())
            telemetry=json.loads((root/'telemetry.json').read_text());audit=json.loads((root/'audit.json').read_text())
            if audit['review_reasons'] or not audit['capture_safety']['collision']['no_collision_verified']:
                raise ValueError('Candidate contradicts audit '+row['id'])
            for actor in meta['runtime_actors']:
                if motion_capture_review(actor,telemetry['objects_before'],telemetry['objects_after'],
                        telemetry.get('object_position_samples',[]),telemetry.get('capture_requested_at_monotonic',0.))['review_reasons']:
                    raise ValueError('All-actor motion supplement not passed '+row['id'])
            latest[row.get('source_cell_id',row['id'])]=row
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as f:
        for key,row in sorted(latest.items()):f.write(json.dumps(row,ensure_ascii=False)+'\n')
    atomic_json(output.with_suffix('.provenance.json'),dict(sources=sources,selected_cells=len(latest),
        selected_ids={k:v['id'] for k,v in latest.items()},selected_sha256=sha256(output),
        policy='Last safely terminated accepted replacement per source cell; all-actor motion replay required; original raw/ledgers untouched'))
    return len(latest)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ledgers',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print('Selected cells:',select(a.ledgers,a.output))
