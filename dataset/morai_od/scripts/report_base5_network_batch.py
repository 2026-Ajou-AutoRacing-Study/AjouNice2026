#!/usr/bin/env python3
"""Read-only comparison of independent, ROS-shared, and network-shared pilots."""
import argparse
from collections import Counter
import json
from pathlib import Path
from base5_capture_safety import atomic_json


def collect(root, wall_sec):
    campaign=(root/'campaign_contract.json').exists()
    ledger=root/('selected_ledger.jsonl' if campaign else 'ledger.jsonl')
    rows=[json.loads(line) for line in ledger.read_text().splitlines() if line]
    stages=Counter();unsafe=[];network=[]
    if campaign:
        timings=[]
        for path in root.glob('*/round_*/report.json'):
            for attempt in json.loads(path.read_text())['attempts']:
                if 'capture_root' not in attempt:continue
                child=Path(attempt['capture_root']);timings.append(child/'timing.json')
                if not attempt['shutdown']['park_verified']:unsafe.append(str(child))
    else:
        timings=[root/'timing.json']
        if not json.loads((root/'shutdown_safety.json').read_text())['park_verified']:
            unsafe.append(str(root))
    for path in timings:
        stages.update(json.loads(path.read_text())['stages_sec'])
    for row in rows:
        if 'capture_root' not in row:continue
        meta=json.loads((Path(row['capture_root'])/'metadata.json').read_text())
        network.append(meta.get('capture_network_policy'))
        policy=meta.get('capture_network_policy',{})
        if (policy.get('batch_reuse_network') or policy.get('shared_ros_batch')) and not row.get('between_scene_park',{}).get('park_verified'):
            unsafe.append(row['id'])
    return dict(root=str(root),wall_sec=wall_sec,
        status=json.loads((root/'status.json').read_text())['stage'],
        frames=sum('capture_root' in r for r in rows),accepted=sum(r['status']=='accepted_candidate' for r in rows),
        failed_attempts=[dict(id=r['id'],error=r.get('error')) for r in rows if r['status']=='failed'],
        selections=sorted((r.get('source_cell_id',r['id']),r['scenario_sha256']) for r in rows if r['status']=='accepted_candidate'),
        stages_sec=dict(stages),unprofiled_wall_sec=wall_sec-sum(stages.values()),
        network_loads=sum(bool(r and r['network_reloaded']) for r in network),
        unsafe_children=unsafe,network_policy=network)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--roots',type=Path,nargs=3,required=True)
    parser.add_argument('--observed-wall-sec',type=float,nargs=3,required=True,
        help='Actual external time real values for the three runs; never infer from ROS rate')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Use new output; preserve old evidence')
    runs=[collect(root,wall) for root,wall in zip(args.roots,args.observed_wall_sec)]
    result=dict(independent=runs[0],ros_shared=runs[1],network_shared=runs[2],
        same_payloads=all(r['selections']==runs[0]['selections'] for r in runs),
        completed_ab_same_payloads=runs[0]['selections']==runs[1]['selections'],
        limitation='Small sequential live A/B/C; external real time supplied from recorded time output. Not a long-run throughput guarantee or proof of identical sensor frames.')
    atomic_json(args.output,result)
    print(json.dumps(result,indent=2))
