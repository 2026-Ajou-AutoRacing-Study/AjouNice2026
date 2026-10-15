#!/usr/bin/env python3
"""Paired frozen-input and counterexample audit of early direction guard.

The candidate changes only world velocity output, never filter state or
association. GT is used after replay for evaluation, never by the guard.
"""
import argparse
import gzip
import json
import math
from pathlib import Path

from audit_tracking_continuity_events import replay, sha, core_summary
from audit_tracking_continuity import stats
from audit_tracking_continuity_variants import paired_outputs


def load(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def write(path, value):
    with gzip.open(path, 'wt') as stream:
        json.dump(value, stream, allow_nan=False)


def confirmed(rows):
    return [dict(stamp=f['stamp'], objects=[dict(t,
        class_name='vehicle' if t['core_class'] in (1, 2) else
                   'pedestrian' if t['core_class'] in (3, 4) else 'obstacle',
        id=str(t['id'])) for t in f['tracks'] if t['confirmed']]) for f in rows]


def synthetic_cases():
    cases = {}
    for kind in ('forward', 'born_reverse', 'stop_reverse', 'small_uturn', 'box_center_corruption'):
        frames = []
        x = 0.0
        for i in range(50):
            t = i*.1
            if kind == 'born_reverse':
                x, y, yaw = -2*t, 0.0, 0.0
            elif kind == 'stop_reverse':
                x, y, yaw = (2*t if t <= .6 else 1.2 if t <= 1.0 else 1.2-2*(t-1.0)), 0.0, 0.0
            elif kind == 'small_uturn':
                x, y, yaw = 2*math.sin(t*1.5), 2*(1-math.cos(t*1.5)), t*1.5
            else:
                x, y, yaw = 2*t, 0.0, 0.0
                if kind == 'box_center_corruption' and i in (8, 9, 10):
                    x -= 1.5
            obj = dict(class_name='vehicle', center=[x,y,.75], yaw=yaw,
                       size=[4.0,1.8,1.5], score=.9)
            frames.append(dict(stamp=100+t, world_objects=[obj]))
        cases[kind] = frames
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--native-reference', type=Path,
                        help='Independent candidate core outputs with _candidate_core.json.gz suffix')
    parser.add_argument('--measurement-clock', action='store_true',
                        help='Separate measurement-history decision time from publication copy freshness')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh evidence directory required')
    args.output.mkdir(parents=True)
    report = dict(binary_sha256=sha(args.core), cases={}, production_changed=False,
                  measurement_time_decision=args.measurement_clock,
                  scope='Same actual native measurement-time EKF, isolated output policy. Not live ROS/planner certification.')
    sources = {p.name[:-len('_frames.json.gz')]: (load(p),p)
               for p in sorted(args.baseline.glob('*_frames.json.gz'))}
    sources.update({f'synthetic_{name}':(frames,None) for name,frames in synthetic_cases().items()})
    for name,(frames,path) in sources.items():
        clock_arg = '1' if args.measurement_clock else '0'
        off = replay(args.core,frames,extra_args=('0',clock_arg))
        on = replay(args.core,frames,extra_args=('1',clock_arg))
        native_path = (args.native_reference/(name+'_candidate_core.json.gz') if args.native_reference
                       else args.baseline/(name+'_core.json.gz'))
        native = load(native_path) if native_path.exists() else None
        # Observer-only configuration is checked against the independently
        # replayed native baseline, not merely against another guard run.
        independent_exact = None if native is None else all(
            a['stamp']==b['stamp'] and a['tracks']==b['tracks'] for a,b in zip(native,off)) and len(native)==len(off)
        if independent_exact is False:
            raise RuntimeError('Observer changes independent native core output: '+name)
        changes=[]
        for a,b in zip(off,on):
            for u,v in zip(a['tracks'],b['tracks']):
                if {k:q for k,q in u.items() if k!='velocity'} != {k:q for k,q in v.items() if k!='velocity'}:
                    raise RuntimeError('Direction-only guard changes center/lifecycle/shape state')
                if u['velocity'] != v['velocity']:
                    changes.append(dict(stamp=a['stamp'],id=u['id'],confirmed=u['confirmed'],
                        original_velocity=u['velocity'],candidate_velocity=v['velocity'],
                        delta_mps=math.dist(u['velocity'],v['velocity'])))
        gt_path = args.baseline/(name+'_gt.json.gz')
        gt = load(gt_path) if gt_path.exists() else None
        summary = dict(frames=len(frames), independent_native_exact=independent_exact,
                       output_changed=len(changes), changed_confirmed=sum(c['confirmed'] for c in changes),
                       velocity_delta_mps=stats([c['delta_mps'] for c in changes]),
                       off=core_summary(off,gt),on=core_summary(on,gt),
                       paired=paired_outputs(confirmed(off),confirmed(on))[0])
        write(args.output/(name+'_off.json.gz'),off)
        write(args.output/(name+'_on.json.gz'),on)
        write(args.output/(name+'_changes.json.gz'),changes)
        report['cases'][name]=summary
        (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(case=name,frames=len(frames),exact=independent_exact,changes=len(changes))),flush=True)


if __name__=='__main__': main()
