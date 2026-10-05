#!/usr/bin/env python3
"""Summarize latest per-scene ledger results without rewriting any capture."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics


def summarize(ledger):
    latest={}
    for line in Path(ledger).read_text().splitlines():
        row=json.loads(line);latest[row['id']]=row
    rows=list(latest.values());captures=[r for r in rows if r['status']!='failed']
    durations=[r['duration_sec'] for r in captures]
    components=[c for r in captures for c in r.get('audit',{}).get('components',[])]
    return dict(unique_scenes=len(rows),status_counts=dict(Counter(r['status'] for r in rows)),
        family_status=dict(Counter(r['family']+'/'+r['status'] for r in rows)),
        review_reasons=dict(Counter(x for r in rows for x in r.get('audit',{}).get('review_reasons',[]))),
        failures={r['id']:r.get('error') for r in rows if r['status']=='failed'},
        capture_bytes=sum(r.get('bytes',0) for r in captures),
        duration_mean_sec=statistics.mean(durations) if durations else None,
        duration_p50_sec=statistics.median(durations) if durations else None,
        components=len(components),gt_classes=dict(Counter(c['training_class'] for c in components)),
        positive_counts={p:sum(c['positive_'+p] for c in components) for p in ('lidar5','lidar1','lidar1_camera0')},
        zero_point_camera_candidates=sum(c['lidar_points']==0 and c['positive_lidar1_camera0'] for c in components),
        scope='Pilot audit, not OD accuracy, collision certification or full 10K acceptance.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=summarize(a.ledger)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
