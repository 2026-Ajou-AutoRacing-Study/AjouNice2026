#!/usr/bin/env python3
"""Read-only setup-budget ETA sensitivity; no guaranteed completion time."""
import argparse
import json
import math
from pathlib import Path
from base5_capture_safety import atomic_json


def report(manifest, cost, extra_retry_sec=30., rates=(0., .01, .05, .1)):
    deck=json.loads(manifest.read_text());measured=json.loads(cost.read_text())
    hours=measured.get('extrapolated_10k_hours_family_weighted')
    if not measured.get('all_quota_and_shutdown_verified') or hours is None:
        raise ValueError('Complete verified family-weighted cost report required')
    if not math.isfinite(extra_retry_sec) or extra_retry_sec<0:
        raise ValueError('Finite nonnegative extra retry cost required')
    if not math.isfinite(hours) or hours<0:raise ValueError('Invalid measured ETA')
    natural=0
    for entry in deck['scenes']:
        metadata=json.loads(Path(entry['metadata']).read_text())
        natural+=any(a.get('motion_path',{}).get('policy')=='natural-target-v1'
                     for a in metadata['runtime_actors'])
    if len(deck['scenes'])!=10000:
        raise ValueError('ETA coverage denominator requires canonical 10K manifest')
    sensitivity=[]
    for rate in rates:
        if not math.isfinite(rate) or not 0<=rate<=1:raise ValueError('Retry rate must be in [0,1]')
        extra=natural*rate*extra_retry_sec/3600
        sensitivity.append(dict(natural_scene_retry_rate=rate,expected_retry_scenes=natural*rate,
            extra_hours=extra,total_hours=hours+extra))
    return dict(total_cells=len(deck['scenes']),natural_motion_cells=natural,
        natural_motion_fraction=natural/len(deck['scenes']),
        nominal_9_to_6_single_attempt_saving_hours=natural*3/3600,
        measured_family_weighted_candidate_hours=hours,
        assumed_additional_seconds_per_natural_scene_retry=extra_retry_sec,
        sensitivity=sensitivity,
        limitations=['Retry rate and incremental cost are hypothetical, not measured population statistics.',
            'Candidate baseline already includes observed quality corrections; sensitivity adds target-miss retry only.',
            'Coverage-biased pilot extrapolation; not long-run certification or guaranteed ETA.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--cost',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--extra-retry-sec',type=float,default=30.)
    a=p.parse_args();result=report(a.manifest,a.cost,a.extra_retry_sec)
    atomic_json(a.output,result);print(json.dumps(result,ensure_ascii=False,indent=2))
