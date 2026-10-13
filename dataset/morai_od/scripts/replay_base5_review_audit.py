#!/usr/bin/env python3
"""Re-audit historical MR asset captures into the new schema, without touching raw.

This is a parser/dataloader regression fixture, NOT new v2 live validation.
"""
import argparse
import json
from pathlib import Path

from base5_capture_audit import audit_capture
from run_base5_capture import atomic_json


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists() and any(a.output.iterdir()):raise FileExistsError('Fresh regression directory required')
    scenes={r['id']:r for r in json.loads(a.manifest.read_text())['scenes']}
    a.output.mkdir(parents=True,exist_ok=True);rows=[]
    for line in (a.source/'captures.jsonl').read_text().splitlines():
        frame=json.loads(line);scene=scenes[frame['scene_id']];meta=json.loads(Path(scene['metadata']).read_text())
        for actor in meta['runtime_actors']:
            actor['training_class']='animal' if actor['name'] in ('Dog1','Dog2','Deer1') else 'component_annotation'
        meta.update(split='train',historical_split=frame['split'],spec=dict(family='pedestrian_related',distance_bounds=[0.,100.]),
                    evidence_scope='HISTORICAL MR parser/dataloader regression; not v2 live validation')
        root=a.output/'captures'/('historical_'+frame['scene_id'])
        for folder in a.source.iterdir():
            if not folder.is_dir():continue
            for f in folder.glob(frame['capture_id']+'*'):
                destination=root/folder.name/f.name;destination.parent.mkdir(parents=True,exist_ok=True)
                destination.symlink_to(f.resolve())
        result=audit_capture(root,meta)
        if any(actor['name']=='OBJ_G80_Open_Door' for actor in meta['runtime_actors']):
            result['accepted_candidate']=False;result['review_reasons'].append('historical_known_floating_asset')
        atomic_json(root/'metadata.json',meta);atomic_json(root/'audit.json',result)
        rows.append(dict(id='historical_'+frame['scene_id'],capture_root=str(root.resolve()),
            status='accepted_candidate' if result['accepted_candidate'] else 'review',audit=result,
            scope=meta['evidence_scope']))
    with (a.output/'ledger.jsonl').open('w') as f:
        for r in rows:f.write(json.dumps(r)+'\n')
    atomic_json(a.output/'regression_summary.json',dict(frames=len(rows),
        accepted=sum(r['status']=='accepted_candidate' for r in rows),
        components=sum(len(r['audit']['components']) for r in rows),
        policy_counts={k:sum(c['positive_'+k] for r in rows for c in r['audit']['components']) for k in ('lidar5','lidar1','lidar1_camera0')},
        limitation='No new MR load/capture or generator validation in this replay.'))
    print((a.output/'regression_summary.json').read_text())


if __name__=='__main__':main()
