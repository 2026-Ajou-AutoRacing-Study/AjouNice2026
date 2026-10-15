#!/usr/bin/env python3
"""Report requested populations separately from actually visible captured parents."""
import argparse
import json
from collections import Counter
from pathlib import Path


def report(manifests, ledger=None):
    placements=[]; visible=[]; counts=Counter(); roles=Counter(); modes=Counter()
    for path in manifests:
        manifest=json.loads(Path(path).read_text())
        for entry in manifest['scenes']:
            meta=json.loads(Path(entry['metadata']).read_text());actors=meta['runtime_actors']
            pop=meta.get('population_budget',{})
            modes[pop.get('mode','background')]+=1
            key=json.dumps([entry['family'],entry['mission'],len(actors)])
            counts[key]+=1
            for a in actors:roles[a['actual_ego_role']]+=1
            placements.append(dict(id=entry['id'],family=entry['family'],mission=entry['mission'],
                population=pop,retry=meta['retry'],
                estimated_geometry_actors=sum(not a['geometry_reference_known'] for a in actors)))
    if ledger:
        for line in Path(ledger).read_text().splitlines():
            row=json.loads(line)
            if row['status']!='accepted_candidate':continue
            root=Path(row['capture_root'])
            meta=json.loads((root/'metadata.json').read_text())
            audit=json.loads(Path(row.get('audit_path',root/'audit.json')).read_text())
            visible.append(dict(id=row['id'],source_cell_id=row.get('source_cell_id',row['id']),
                population=meta.get('population_budget'),visibility=audit.get('population_visibility'),
                review_reasons=audit['review_reasons']))
    return dict(offline_scenes=len(placements),modes=dict(modes),actor_count_by_family_mission=dict(counts),
        actual_planned_roles=dict(roles),max_resamples=max((r['retry'] for r in placements),default=0),
        mean_resamples=sum(r['retry'] for r in placements)/max(1,len(placements)),
        placements=placements,accepted_capture_population=visible,
        limitations=['Offline poses are not visibility/ground/collider certification.',
            'Parents and GT components are separate; visible includes positive ROI evidence.',
            'Fallback geometry cost is conservative, not certified asset dimensions.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,nargs='+',required=True)
    p.add_argument('--ledger',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=report(a.manifest,a.ledger)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('placements','accepted_capture_population')},indent=2))
