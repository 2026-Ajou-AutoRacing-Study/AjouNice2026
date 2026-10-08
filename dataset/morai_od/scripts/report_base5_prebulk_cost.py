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


def observation_group(meta,detailed=False):
    if not moving_actors(meta):return 'stationary'
    if not detailed:return 'animated'
    return 'shadow_animated' if meta['spec'].get('shadow_moving') else 'intersection_animated'


def classified_storage(folders):
    seen=set();totals=Counter()
    for group,roots in folders.items():
        for root in roots:
            for path in Path(root).rglob('*'):
                if not path.is_file():continue
                stat=path.stat();key=(stat.st_dev,stat.st_ino)
                if key in seen:continue
                seen.add(key);totals[group]+=stat.st_size
    return totals


def report(manifest,campaigns,detailed_groups=False):
    deck=json.loads(Path(manifest).read_text());source={};counts=Counter()
    for row in deck['scenes']:
        meta=json.loads(Path(row['metadata']).read_text())
        group=observation_group(meta,detailed_groups)
        source[row['id']]=(group,row['scenario_sha256']);counts[group]+=1
    keys=('stationary','intersection_animated','shadow_animated') if detailed_groups else ('stationary','animated')
    buckets={k:dict(attempted_cells=0,accepted_cells=0,allocated_elapsed_sec=0.) for k in keys}
    samples=[];warnings=[];elapsed=logical=allocated=accepted_total=0;group_storage=Counter()
    for root in map(Path,campaigns):
        status=json.loads((root/'status.json').read_text());quota=json.loads((root/'quota.json').read_text())
        if status['stage'] not in ('complete','complete_with_deficit'):
            raise ValueError('Only completed bounded campaign cost is supported: '+str(root))
        grouped=Counter();cell_costs=Counter();folders={k:[] for k in keys}
        for chunk in root.glob('chunk_*'):
            scenes=json.loads((chunk/'manifest.json').read_text())['scenes']
            for row in scenes:
                if row['id'] not in source or source[row['id']][1]!=row['scenario_sha256']:
                    raise ValueError('Pilot source payload differs from final manifest')
                grouped[source[row['id']][0]]+=1
                key=source[row['id']][0]
                folders[key].extend([chunk/'capture/captures'/row['id'],chunk/'recovery'/row['id']])
            initial=chunk/'capture/status.json'
            if initial.exists():
                child=json.loads(initial.read_text())
                # Equal allocation within a shared batch is only an estimate.
                for row in scenes:cell_costs[row['id']]+=child.get('elapsed_sec',0)/len(scenes)
            for row in scenes:
                recovery=chunk/'recovery'/row['id']/'status.json'
                if recovery.exists():cell_costs[row['id']]+=json.loads(recovery.read_text()).get('elapsed_sec',0)
        state_path=root/'replenishment/state.json'
        if state_path.exists():
            state=json.loads(state_path.read_text())
            for attempt in state['attempts']:
                if attempt.get('result') is None:continue
                if not attempt.get('child'):continue
                child=Path(attempt['child'])
                folders[source[attempt['id']][0]].append(child.parent)
                status_path=child/'status.json'
                if status_path.exists():cell_costs[attempt['id']]+=json.loads(status_path.read_text()).get('elapsed_sec',0)
        selections=[json.loads(line) for line in (root/'selected_ledger.jsonl').read_text().splitlines()]
        accepted=Counter(source[r.get('source_cell_id',r['id'])][0] for r in selections)
        overhead=status['elapsed_sec']-sum(cell_costs.values())
        if overhead < -1:raise ValueError('Child accounting exceeds campaign elapsed; do not infer ETA')
        for key,bucket in buckets.items():
            bucket['attempted_cells']+=grouped[key];bucket['accepted_cells']+=accepted[key]
            bucket['allocated_elapsed_sec']+=sum(cost for ident,cost in cell_costs.items() if source[ident][0]==key)
            bucket['allocated_elapsed_sec']+=max(0,overhead)*grouped[key]/sum(grouped.values())
        storage=unique_storage(root)
        sizes=classified_storage(folders)
        remaining=storage['logical_bytes']-sum(sizes.values())
        if remaining < 0:raise ValueError('Classified storage exceeds unique campaign storage')
        for key in keys:group_storage[key]+=sizes[key]+remaining*grouped[key]/sum(grouped.values())
        elapsed+=status['elapsed_sec'];logical+=storage['logical_bytes'];allocated+=storage['allocated_bytes']
        accepted_total+=quota['accepted']
        samples.append(dict(root=str(root),stage=status['stage'],elapsed_sec=status['elapsed_sec'],quota=quota,storage=storage))
    for bucket in buckets.values():
        n=bucket['accepted_cells'];bucket['seconds_per_accepted_cell']=bucket['allocated_elapsed_sec']/n if n else None
    seconds=sum(counts[k]*buckets[k]['seconds_per_accepted_cell'] for k in counts) if all(buckets[k]['accepted_cells'] for k in counts) else None
    storage_gib=(sum(counts[k]*group_storage[k]/buckets[k]['accepted_cells'] for k in counts)/2**30
                 if all(buckets[k]['accepted_cells'] for k in counts) else None)
    return dict(target_frames=len(source),planned_groups=dict(counts),sample_groups=buckets,samples=samples,
        sample_accepted=accepted_total,sample_elapsed_sec=elapsed,detailed_groups=detailed_groups,
        extrapolated_hours_group_weighted=seconds/3600 if seconds else None,
        extrapolated_gib_including_qa_and_rejected_unweighted=logical/max(1,accepted_total)*len(source)/2**30,
        extrapolated_gib_including_qa_and_rejected_group_weighted=storage_gib,
        group_storage_bytes=dict(group_storage),
        allocated_sample_gib=allocated/2**30,warnings=warnings,bulk_approval=False,
        limitations=['Small deliberately difficult sample, not a throughput guarantee or confidence interval.',
            'Stationary/animated weighting is not complete family/asset/distance weighting.',
            'Detailed groups separate measured-endpoint intersection animation from independent tunnel terrain preparation.',
            'Shared-batch child cost and parent overhead are apportioned; use accumulated bulk rate after start.',
            'Hardlinked QA files count once within each root; copied files and failed/calibration captures remain included.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--campaign',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--detailed-groups',action='store_true');a=p.parse_args()
    result=report(a.manifest,a.campaign,a.detailed_groups);atomic_json(a.output,result);print(json.dumps(result,ensure_ascii=False,indent=2))
