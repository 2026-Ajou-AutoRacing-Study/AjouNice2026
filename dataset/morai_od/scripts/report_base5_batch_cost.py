#!/usr/bin/env python3
"""Read-only batch cost accounting: count real ledgers, never reused seed evidence."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
from base5_capture_safety import atomic_json


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def report(root, manifest, elapsed_sec=None):
    status=json.loads((root/'status.json').read_text())
    quota=json.loads((root/'quota.json').read_text())
    deck=json.loads(manifest.read_text())
    selected=rows(root/'selected_ledger.jsonl')
    families=defaultdict(lambda:dict(selected=0,captures=0,scene_sec=0.0,raw_bytes=0))
    for row in selected:families[row['family']]['selected']+=1
    actual={}
    for path in root.rglob('ledger.jsonl'):
        # Retries and resumed rows are identified by real session + scene,
        # not by quality-report references (which can point to a reused batch).
        for row in rows(path):actual[(str(path.parent.resolve()),row['id'])]=row
    unsafe=[]
    for path in root.rglob('session_contract.json'):
        shutdown=path.parent/'shutdown_safety.json'
        if not shutdown.exists() or not json.loads(shutdown.read_text()).get('park_verified'):
            unsafe.append(str(path.parent))
    for row in actual.values():
        family=families[row['family']]
        family['captures']+=1
        family['scene_sec']+=row.get('duration_sec',0.0)
        family['raw_bytes']+=row.get('bytes',0)
        boundary=row.get('between_scene_park')
        if boundary is not None and not boundary.get('park_verified'):
            unsafe.append(row['id']+' boundary')
    events=[row for path in root.rglob('event_cmd_requests.jsonl') for row in rows(path)]
    errors=Counter(row.get('error') for row in events if row.get('error'))
    scene_sec=sum(f['scene_sec'] for f in families.values())
    elapsed=status['elapsed_sec'] if elapsed_sec is None else float(elapsed_sec)
    if not math.isfinite(elapsed) or elapsed<0:raise ValueError('Invalid preserved wall time')
    overhead=elapsed-scene_sec
    counts=deck['config']['families']
    complete=(status['stage']=='complete' and len(selected)==quota['target']==quota['accepted']
              and len({r['source_cell_id'] for r in selected})==len(selected) and not unsafe)
    weighted=None
    if complete and scene_sec>0 and all(f in families and families[f]['selected'] for f in counts):
        weighted=sum(counts[f]*families[f]['scene_sec']/families[f]['selected'] for f in counts)
        weighted*=elapsed/scene_sec
    kinds=Counter()
    for path in root.rglob('report.json'):
        for attempt in json.loads(path.read_text()).get('attempts',[]):
            kinds[attempt['kind']]+=1
    return dict(schema=1,stage=status['stage'],quota=quota,elapsed_sec=elapsed,
        elapsed_source='status' if elapsed_sec is None else 'explicit preserved wall observation',
        selected=len(selected),actual_capture_attempts=len(actual),attempt_kinds=dict(kinds),
        family_cost=dict(families),scene_duration_sec=scene_sec,
        parent_startup_preflight_boundary_shutdown_and_other_sec=overhead,
        unsafe_sessions=unsafe,event_requests=len(events),event_errors=dict(errors),
        event_timeouts=sum('timeout' in str(r.get('error','')).lower() for r in events),
        event_workers_alive=sum(bool(r.get('worker_still_alive')) for r in events),
        all_quota_and_shutdown_verified=complete,
        extrapolated_10k_hours_family_weighted=weighted/3600 if weighted is not None else None,
        limitations=[
            'Coverage-biased bounded sample; not a long-run certificate or guaranteed 10K ETA.',
            'Scene duration includes motion/waits/audit; overhead is not pure Python overhead.',
            'Family weighting allocates non-scene wall cost globally, not per family.',
            'Preserve partial-resume wall segments or supply their measured sum. Legacy completed resume may reset elapsed; new completed verification preserves it. Session timing.json may cover only its latest segment.',
            'Seed reuse references do not count as new capture attempts.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--elapsed-sec',type=float,help='Previously measured wall time, e.g. preserved partial-resume segment sum')
    a=p.parse_args();result=report(a.capture_root,a.manifest,a.elapsed_sec)
    atomic_json(a.output,result);print(json.dumps(result,ensure_ascii=False,indent=2))
