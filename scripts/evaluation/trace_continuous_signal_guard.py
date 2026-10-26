#!/usr/bin/env python3
"""Feed actual replay outputs to the production C++ pure Guard, not a vehicle.

This is a pre-line STOP-request proxy. It does not simulate GO_OUT, planning
velocity insertions, controller/braking dynamics, or assign historical causality.
"""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
GROUPS=dict(A=201351,B=201365,C=201290,D=201326)


def phase(group,lamps):
    lamps={tuple(lamp) for lamp in lamps}
    if not lamps:return 'unknown'
    if any(color=='yellow' for color,shape,left in lamps):return 'yellow'
    if group==201326:return 'green' if ('green','arrow',True) in lamps else 'red'
    return 'green' if ('green','circle',None) in lamps else 'red'


def run(source,output):
    output=output.resolve();output.relative_to(ROOT/'planner_logs/analysis')
    if output.exists():raise FileExistsError('Fresh output required')
    report=json.loads(source.read_text());output.mkdir(parents=True)
    lines=[]
    for row in report['records']:
        if row['group_id']!=GROUPS[row['section']]:continue
        for mode,result in row['outcomes'].items():
            fields=(row['t'],mode,row['section'],row['group_id'],result['output_source_stamp_ns']/1e9,
                    row['receipt_stamp_ns']/1e9,phase(row['group_id'],result['decision']['final_tuples']))
            lines.append(' '.join(map(str,fields)))
    input_path=output/'guard_input.txt';input_path.write_text('\n'.join(lines)+'\n')
    code=ROOT/'scripts/evaluation/replay_signal_guard_trace.cpp'
    with tempfile.TemporaryDirectory(prefix='signal-guard-trace-') as temp:
        binary=Path(temp)/'guard'
        subprocess.run(['g++','-std=c++17','-O2','-I'+str(ROOT/'planning_ws/src/autoware_behavior_velocity_traffic_light_module/include'),
                        str(code),'-o',str(binary)],check=True)
        result=subprocess.run([str(binary),str(input_path)],check=True,capture_output=True,text=True)
    (output/'guard_output.csv').write_text(result.stdout)
    groups=defaultdict(list)
    for row in csv.DictReader(result.stdout.splitlines()):
        row['t']=float(row['t']);row['require_stop']=int(row['require_stop'])
        groups[(row['mode'],row['section'])].append(row)
    summary={}
    for (mode,section),rows in groups.items():
        to_stop=sum(a['require_stop']==0 and b['require_stop']==1 for a,b in zip(rows,rows[1:]))
        summary[mode+':'+section]=dict(frames=len(rows),stop_request_frames=sum(r['require_stop'] for r in rows),
            go_to_stop_transitions=to_stop,first_permitted_t=next((r['t'] for r in rows if not r['require_stop']),None))
    result=dict(source=str(source),summary=summary,limitations=[__doc__,
        'Direction classification follows A/B/C straight and D protected-left contract; native lamp GT absent.'])
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.source,a.output)
