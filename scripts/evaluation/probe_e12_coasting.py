#!/usr/bin/env python3
"""Offline lifecycle-only probe using saved detections and isolated EKF copies.

Only a generated diagnostic harness/source copy changes its coast policy.
No production file, installed binary, tracker threshold, or GT is changed.
"""
import argparse
from copy import deepcopy
import gzip
import json
from pathlib import Path
import shlex
import subprocess

from prepare_e12_postprocessing import ROOT, TOOLS, core_frames, sha


def configured_harness(text, seconds):
    if seconds not in (.75,1.):
        raise ValueError('Only the bounded .75/1.0 second offline candidates are allowed')
    anchor='  config.confirmed_track_max_coast_time_sec = .50;'
    if text.count(anchor)!=1:
        raise ValueError('Pinned production harness anchor changed')
    return text.replace(anchor,'  config.confirmed_track_max_coast_time_sec = %.2f;'%seconds)


def mature_vehicle_core(text, seconds):
    if seconds not in (.75,1.):
        raise ValueError('Bounded coast candidate required')
    anchor='''    const double maximum_coast_time = stationary
        ? config_.confirmed_stationary_track_max_coast_time_sec
        : config_.confirmed_track_max_coast_time_sec;'''
    if text.count(anchor)!=1:
        raise ValueError('Pinned lifecycle source anchor changed')
    replacement='''    const bool mature_vehicle_probe =
        (representative_class == mc_mot::ObjectClass::CAR ||
         representative_class == mc_mot::ObjectClass::TRUCK) &&
        measurement_time - track.initialization_time >= 1.0;
    const double maximum_coast_time = stationary
        ? config_.confirmed_stationary_track_max_coast_time_sec
        : (mature_vehicle_probe ? %.2f : config_.confirmed_track_max_coast_time_sec);'''%seconds
    return text.replace(anchor,replacement)


def run(args):
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    with gzip.open(args.pack,'rt')as handle:
        baseline=json.load(handle)
    if baseline['model']!='E12' or baseline.get('schema')!='e12_postprocessing_core_pack_v1':
        raise ValueError('Frozen E12 production-baseline core pack required')
    args.output.mkdir(parents=True)
    tracker=ROOT/'base_ws/src/ekf-multi-object-tracking'
    original=TOOLS/'replay_base5_continuous_tracker_core.cpp'
    source=tracker/'src/algorithm/ekf_multi_object_tracking.cpp'
    header=tracker/'include/algorithm/ekf_multi_object_tracking.hpp'
    pins={str(p):sha(p)for p in (original,source,header,args.pack,Path(__file__))}
    generated=args.output/'coasting_harness.cpp'
    generated.write_text(original.read_text()if args.vehicle_mature else configured_harness(original.read_text(),args.seconds))
    linked_source=source
    if args.vehicle_mature:
        linked_source=args.output/'mature_vehicle_core.cpp'
        linked_source.write_text(mature_vehicle_core(source.read_text(),args.seconds))
    binary=(args.output/'coasting_core').resolve()
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','roscpp'],text=True))
    command=['g++','-std=c++17','-O1','-I/usr/include/eigen3','-I'+str(tracker/'include'),
             str(generated),str(linked_source),'-o',str(binary),*flags]
    subprocess.run(command,check=True)
    pack=deepcopy(baseline)
    pack['provenance']['baseline_core_build']=pack['provenance'].pop('core_build')
    pack['provenance']['scope']='Generated offline lifecycle diagnostic copy; not the unchanged production configuration, live ROS or planner.'
    pack['provenance']['offline_candidate']=dict(moving_coast_sec=args.seconds,
        only_vehicle_birth_age_ge1s=args.vehicle_mature,
        stationary_coast_sec=1.,scope='diagnostic configuration only; production unmodified',
        command=command,source_sha256=pins,harness_sha256=sha(generated),
        linked_source_sha256=sha(linked_source),binary_sha256=sha(binary))
    for name,case in pack['cases'].items():
        results=core_frames(binary,case['frames'])
        if results!=core_frames(binary,case['frames']):
            raise ValueError('Same candidate input failed fresh-core repeat')
        for frame,result in zip(case['frames'],results):
            frame['tracks']=result['tracks']
        case['provenance']['offline_candidate_moving_coast_sec']=args.seconds
        case['provenance']['offline_candidate_only_mature_vehicle']=args.vehicle_mature
        case['provenance']['fresh_core_repeat_exact']=True
        print(name,flush=True)
    if pins!={p:sha(p)for p in pins}:
        raise ValueError('Production source or baseline pack changed')
    with gzip.open(args.output/'replay_pack.json.gz','xt')as handle:
        json.dump(pack,handle,allow_nan=False)
    (args.output/'PROBE.json').write_text(json.dumps(dict(complete=True,production_changed=False,
        moving_coast_sec=args.seconds,only_vehicle_birth_age_ge1s=args.vehicle_mature,source_sha256=pins,
        pack_sha256=sha(args.output/'replay_pack.json.gz')),indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pack',type=Path,required=True)
    parser.add_argument('--seconds',type=float,choices=(.75,1.),required=True)
    parser.add_argument('--vehicle-mature',action='store_true',
        help='Only a generated research copy extends moving CAR/TRUCK born>=1s; pedestrian/UNKNOWN unchanged')
    parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args())
