#!/usr/bin/env python3
"""Compare actual stage-accounted pilot campaigns without changing their status."""
import argparse
from collections import Counter
import json
from pathlib import Path
from base5_capture_safety import atomic_json


def summarize(root):
    status=json.loads((root/'status.json').read_text())
    quota=json.loads((root/'quota.json').read_text())
    stages=Counter();children=0;unsafe=[];selections=[]
    for path in sorted(root.glob('*/round_*/report.json')):
        report=json.loads(path.read_text())
        selections.extend((r.get('source_cell_id',r['id']),r['scenario_sha256']) for r in report['selected'])
        for attempt in report['attempts']:
            if 'capture_root' not in attempt:continue
            child=Path(attempt['capture_root'])
            timing=json.loads((child/'timing.json').read_text())
            if abs(sum(timing['stages_sec'].values())-timing['total_sec'])>1e-6:
                raise ValueError('Stage accounting does not partition elapsed time')
            stages.update(timing['stages_sec']);children+=1
            if not attempt['shutdown']['park_verified']:unsafe.append(str(child))
    return dict(root=str(root),stage=status['stage'],quota=quota,children=children,
        elapsed_sec=status['elapsed_sec'],stages_sec=dict(stages),
        unprofiled_campaign_sec=status['elapsed_sec']-sum(stages.values()),
        unsafe_children=unsafe,selected_scenarios=sorted(selections))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before',type=Path,required=True)
    parser.add_argument('--after',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Use a new output to retain evidence')
    before,after=summarize(args.before),summarize(args.after)
    result=dict(before=before,after=after,
        same_selected_scenario_payloads=before['selected_scenarios']==after['selected_scenarios'],
        reduction_fraction=1-after['elapsed_sec']/before['elapsed_sec'],
        limitation='Sequential small-sample A/B; simulator timing and motion pose variation remain. Not a guaranteed 10K speedup.')
    atomic_json(args.output,result)
    print(json.dumps(result,indent=2))
