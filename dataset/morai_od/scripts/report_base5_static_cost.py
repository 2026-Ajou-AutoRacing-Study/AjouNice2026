#!/usr/bin/env python3
"""All-in static pilot cost, including rejected cells and bounded recovery."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import numpy as np
from base5_capture_safety import atomic_json
from report_base5_batch_cost import report as batch_report, rows


def report(root,manifest):
    deck=json.loads(manifest.read_text());summary=batch_report(root,manifest)
    if summary['stage'] not in ('complete','complete_with_deficit') or summary['unsafe_sessions']:
        raise ValueError('Only completed safe static pilot can estimate cost')
    population=deck['provenance']['static_population_family_counts']
    actual={};kinds=Counter();warnings=Counter();stages=defaultdict(list)
    for path in root.rglob('ledger.jsonl'):
        for row in rows(path):
            if not row.get('capture_root'):continue
            meta=json.loads((Path(row['capture_root'])/'metadata.json').read_text())
            if meta.get('calibration_only') or any(a['motion'] not in ('static','standing') for a in meta['runtime_actors']):
                raise ValueError('Static-only pilot unexpectedly contains calibration/movement')
            actual[(str(path.parent.resolve()),row['id'])]=row
    attempts=list(actual.values())
    for row in attempts:
        for warning in row.get('audit',{}).get('ground_warnings',[]):warnings[warning['reason']]+=1
    for path in root.rglob('report.json'):
        for attempt in json.loads(path.read_text()).get('attempts',[]):kinds[attempt['kind']]+=1
    for path in root.rglob('timing.json'):
        timing=json.loads(path.read_text())
        for stage,value in timing.get('stages_sec',{}).items():stages[stage].append(value)
    approved=rows(root/'selected_ledger.jsonl');counts=Counter(r['family'] for r in approved)
    planned=Counter(r['family'] for r in deck['scenes'])
    # All real scene durations by family, including failure/recovery; remaining
    # parent wall time allocated by fixed attempted family population, not success.
    raw_cost=Counter()
    for row in attempts:raw_cost[row['family']]+=row.get('duration_sec',0.)
    overhead=summary['elapsed_sec']-sum(raw_cost.values())
    if overhead<-.1:raise ValueError('Child duration exceeds parent wall accounting')
    cost={f:(raw_cost[f]+max(0,overhead)*planned[f]/len(deck['scenes']))/counts[f]
          if counts[f] else None for f in population}
    weighted=sum(population[f]*cost[f] for f in population)/3600 if all(cost[f] is not None for f in population) else None
    duration=[r['duration_sec'] for r in attempts if r.get('duration_sec') is not None]
    forbidden={k:n for k,n in kinds.items() if k in ('static_near_ground_reference','empty_terrain',
        'input_z_retry','near_measured_input_z_retry','independent_ground_retry','static_rigid_ground_retry')}
    return dict(schema=1,policy='static-single-capture-advisory-v115',quota=summary['quota'],
        elapsed_sec=summary['elapsed_sec'],accepted=len(approved),actual_captures=len(attempts),
        seconds_per_accepted=summary['elapsed_sec']/len(approved) if approved else None,
        scene_duration_p50_sec=float(np.median(duration)) if duration else None,
        scene_duration_p95_sec=float(np.percentile(duration,95)) if duration else None,
        family_planned=dict(planned),family_accepted=dict(counts),family_seconds_per_accepted=cost,
        static_population_family_counts=population,static_population=sum(population.values()),
        extrapolated_static4171_hours_family_weighted=weighted,
        extrapolated_static4171_hours_unweighted=summary['elapsed_sec']/max(1,len(approved))*sum(population.values())/3600,
        static_ground_extra_capture_kinds=forbidden,ground_warning_counts=dict(warnings),
        capture_stage_totals_sec={k:sum(v) for k,v in stages.items()},
        event_errors=summary['event_errors'],unsafe_sessions=summary['unsafe_sessions'],
        limitations=['40-slot fixed-seed family sample; not all-asset/terrain certification or guaranteed bulk ETA.',
            'Rejected cells and placement retries included; completed-resume verification is not capture time.',
            'No new animated cost measurement. Remaining 829 animated cells are not covered by this static ETA.',
            'Family weighting allocates non-scene wall cost proportionally; rare hard conditions can alter long-run rate.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=report(a.capture_root,a.manifest);atomic_json(a.output,result)
    print(json.dumps(result,ensure_ascii=False,indent=2))
