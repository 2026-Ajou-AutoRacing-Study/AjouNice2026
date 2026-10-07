#!/usr/bin/env python3
"""Estimate collection cost from bounded batch campaigns, including QA/deficits.
Hardlinked QA copies are counted once. This report never approves collection.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
from base5_capture_safety import atomic_json
from base5_motion_ground import moving_actors


def unique_storage(root):
    seen=set();logical=allocated=0
    for path in Path(root).rglob('*'):
        if not path.is_file():continue
        stat=path.stat();key=(stat.st_dev,stat.st_ino)
        if key in seen:continue
        seen.add(key);logical+=stat.st_size;allocated+=stat.st_blocks*512
    return dict(unique_files=len(seen),logical_bytes=logical,allocated_bytes=allocated)


def report(manifest,campaigns):
    deck=json.loads(Path(manifest).read_text());source={};counts=Counter()
    for row in deck['scenes']:
        meta=json.loads(Path(row['metadata']).read_text())
        group='animated' if moving_actors(meta) else 'stationary'
        source[row['id']]=(group,row['scenario_sha256']);counts[group]+=1
    buckets={k:dict(attempted_cells=0,accepted_cells=0,allocated_elapsed_sec=0.) for k in ('stationary','animated')}
    samples=[];warnings=[];elapsed=logical=allocated=accepted_total=0
    for root in map(Path,campaigns):
        status=json.loads((root/'status.json').read_text());quota=json.loads((root/'quota.json').read_text())
        if status['stage'] not in ('complete','complete_with_deficit'):
            raise ValueError('Only completed bounded campaign cost is supported: '+str(root))
        grouped=Counter();cell_costs=Counter()
        for chunk in root.glob('chunk_*'):
            scenes=json.loads((chunk/'manifest.json').read_text())['scenes']
            for row in scenes:
                if row['id'] not in source or source[row['id']][1]!=row['scenario_sha256']:
                    raise ValueError('Pilot source payload differs from final manifest')
                grouped[source[row['id']][0]]+=1
            initial=chunk/'capture/status.json'
            if initial.exists():
                child=json.loads(initial.read_text())
                # Equal allocation within a shared batch is only an estimate.
                for row in scenes:cell_costs[row['id']]+=child.get('elapsed_sec',0)/len(scenes)
            for row in scenes:
                recovery=chunk/'recovery'/row['id']/'status.json'
                if recovery.exists():cell_costs[row['id']]+=json.loads(recovery.read_text()).get('elapsed_sec',0)
        selections=[json.loads(line) for line in (root/'selected_ledger.jsonl').read_text().splitlines()]
        accepted=Counter(source[r.get('source_cell_id',r['id'])][0] for r in selections)
        overhead=status['elapsed_sec']-sum(cell_costs.values())
        if overhead < -1:raise ValueError('Child accounting exceeds campaign elapsed; do not infer ETA')
        for key,bucket in buckets.items():
            bucket['attempted_cells']+=grouped[key];bucket['accepted_cells']+=accepted[key]
            bucket['allocated_elapsed_sec']+=sum(cost for ident,cost in cell_costs.items() if source[ident][0]==key)
            bucket['allocated_elapsed_sec']+=max(0,overhead)*grouped[key]/sum(grouped.values())
        storage=unique_storage(root)
        elapsed+=status['elapsed_sec'];logical+=storage['logical_bytes'];allocated+=storage['allocated_bytes']
        accepted_total+=quota['accepted']
        samples.append(dict(root=str(root),stage=status['stage'],elapsed_sec=status['elapsed_sec'],quota=quota,storage=storage))
    for bucket in buckets.values():
        n=bucket['accepted_cells'];bucket['seconds_per_accepted_cell']=bucket['allocated_elapsed_sec']/n if n else None
    seconds=sum(counts[k]*buckets[k]['seconds_per_accepted_cell'] for k in counts) if all(buckets[k]['accepted_cells'] for k in counts) else None
    return dict(target_frames=len(source),planned_groups=dict(counts),sample_groups=buckets,samples=samples,
        sample_accepted=accepted_total,sample_elapsed_sec=elapsed,
        extrapolated_hours_group_weighted=seconds/3600 if seconds else None,
        extrapolated_gib_including_qa_and_rejected_unweighted=logical/max(1,accepted_total)*len(source)/2**30,
        allocated_sample_gib=allocated/2**30,warnings=warnings,bulk_approval=False,
        limitations=['Small deliberately difficult sample, not a throughput guarantee or confidence interval.',
            'Stationary/animated weighting is not complete family/asset/distance weighting.',
            'Shared-batch child cost and parent overhead are apportioned; use accumulated bulk rate after start.',
            'Hardlinked QA files count once within each root; copied files and failed/calibration captures remain included.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--campaign',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=report(a.manifest,a.campaign);atomic_json(a.output,result);print(json.dumps(result,ensure_ascii=False,indent=2))
