#!/usr/bin/env python3
"""Summarize live safety pilot and optionally verify a completed-session resume is a no-op."""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from base5_capture_safety import atomic_json,sha256,load_ledger


def snapshot(root):
    return {str(p.relative_to(root)):(sha256(p),p.stat().st_mtime_ns) for p in root.rglob('*') if p.is_file()}


def report(root,manifest,resume_check=False):
    rows=load_ledger(root/'ledger.jsonl');latest={r['id']:r for r in rows};result=[]
    for entry in latest.values():
        if entry['status']=='failed':
            result.append(dict(id=entry['id'],family=entry['family'],status='failed',
                error=entry.get('error'),duration_sec=entry['duration_sec'],bytes=0))
            continue
        capture=Path(entry['capture_root']);meta=json.loads((capture/'metadata.json').read_text());audit=json.loads((capture/'audit.json').read_text())
        uid=meta['runtime_actors'][0]['unique_id'] if meta['runtime_actors'] else None
        components=[r for r in audit['components'] if r['parent_uid']==uid]
        clearances=[r for r in audit.get('actual_annotation_clearances',[]) if r['parent_uid']==uid]
        result.append(dict(id=entry['id'],family=entry['family'],status=entry['status'],review_reasons=audit['review_reasons'],
            primary_asset=meta['runtime_actors'][0]['name'] if meta['runtime_actors'] else None,
            avoidance_phase=meta.get('avoidance_observation',{}).get('phase'),
            requested_gap_m=meta.get('avoidance_observation',{}).get('requested_surface_gap_m'),
            actual_clearances=clearances,primary_ranges_m=[r.get('range_ego_xy_m') for r in components],
            lidar_points=[r['lidar_points'] for r in components],collision=audit['capture_safety']['collision'],
            support=audit['capture_safety']['support'],duration_sec=entry['duration_sec'],bytes=entry['bytes']))
    archived=list((root/'incomplete_attempts').glob('*'))
    verification=dict(completed_ids_unique=len(rows)==len(latest),archived_attempts=[str(p) for p in archived],
        archived_raw_lidar_files=sum(len(list(p.glob('LIDAR_*/*_Intensity.bin'))) for p in archived))
    if resume_check:
        before=snapshot(root/'captures');lines_before=(root/'ledger.jsonl').read_bytes()
        status_before=json.loads((root/'status.json').read_text())
        entries=json.loads(manifest.read_text())['scenes']
        completed={k:v for k,v in latest.items() if v['status'] in ('accepted_candidate','review')}
        indices=[str(i) for i,e in enumerate(entries) if e['id'] in completed]
        if len(indices)!=len(completed):raise ValueError('Resume report IDs not in manifest')
        cmd=[sys.executable,str(Path(__file__).with_name('run_base5_capture.py')),'--manifest',str(manifest),'--output',str(root),
            '--indices',','.join(indices),'--limit',str(len(completed))]
        run=subprocess.run(cmd,capture_output=True,text=True,timeout=35)
        verification.update(resume_returncode=run.returncode,resume_stdout=run.stdout,resume_stderr=run.stderr,
            completed_files_hash_and_mtime_unchanged=before==snapshot(root/'captures'),ledger_unchanged=lines_before==(root/'ledger.jsonl').read_bytes())
        if run.returncode or not verification['completed_files_hash_and_mtime_unchanged'] or not verification['ledger_unchanged']:
            raise RuntimeError('Completed-session resume verification failed')
        # Do not hide an excluded failed cell in the full-pilot dashboard.
        atomic_json(root/'status.json',status_before)
        verification['completed_cells_checked']=len(completed)
    times=[r['duration_sec'] for r in result]
    return dict(status_counts=dict(Counter(r['status'] for r in result)),scenes=len(result),rows=result,
        duration_mean_sec=float(np.mean(times)),duration_p50_sec=float(np.median(times)),
        final_capture_bytes=sum(r['bytes'] for r in result),resume_verification=verification,
        limitations=[f'Representative {len(result)} scenes, not all assets or 10K certification.',
            'Support is measured vertical stability, not pavement polygon or physical mesh proof.',
            'No detector training or 10K bulk collection performed.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--run-resume-check',action='store_true');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=report(a.capture_root,a.manifest,a.run_resume_check);atomic_json(a.output,result)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
