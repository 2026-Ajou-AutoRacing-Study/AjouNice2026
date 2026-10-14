#!/usr/bin/env python3
"""Compile unchanged visualization/track-conversion methods in an offline harness.

This reconstructs frozen confirmed tracks from saved TrackedObjects, NOT hidden
tentative EKF state or exact measurement replay. No ROS node/master/publisher is
started. Production source and install space remain untouched.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=ROOT/'base_ws/src/ekf-multi-object-tracking'


def function_source(source, signature):
    """Top-level method extraction; reject missing/ambiguous source changes."""
    pattern=re.escape(signature)+r'[^\{]*\{.*?\n\}'
    matches=re.findall(pattern,source,flags=re.S)
    if len(matches)!=1:
        raise ValueError('Expected one method definition: '+signature)
    return matches[0]


def digest(path):
    sha=hashlib.sha256()
    with path.open('rb') as src:
        for chunk in iter(lambda:src.read(1024*1024),b''):
            sha.update(chunk)
    return sha.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    source_path=PACKAGE/'src/ekf_multi_object_tracking_node.cpp'
    source=source_path.read_text()
    method_names=('VisualizeTrackObjects','BuildTrackedObjects','IsVisualizeTrack')
    signatures=('void EkfMultiObjectTrackingNode::VisualizeTrackObjects',
                'void EkfMultiObjectTrackingNode::BuildTrackedObjects',
                'bool EkfMultiObjectTrackingNode::IsVisualizeTrack')
    methods=[function_source(source,s) for s in signatures]
    helpers=[function_source(source,'uint8_t ToAutowareClass'),
             function_source(source,'unique_identifier_msgs::UUID StableTrackUuid')]
    template_path=Path(__file__).with_suffix('.cpp.in')
    generated=template_path.read_text().replace('@EXTRACTED_HELPERS@','\n\n'.join(helpers))
    generated=generated.replace('@EXTRACTED_METHODS@','\n\n'.join(methods).replace('EkfMultiObjectTrackingNode::','VisualizationHarness::'))
    generated_path=args.output/'harness.cpp'
    generated_path.write_text(generated)
    packages=['roscpp','rosbag','tf','eigen_conversions','autoware_perception_msgs','jsk_recognition_msgs']
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs',*packages],text=True))
    exe=args.output/'harness'
    command=['g++','-std=c++14','-O3','-DNDEBUG','-I'+str(PACKAGE/'include'),str(generated_path),'-o',str(exe),*flags]
    subprocess.run(command,check=True)
    subprocess.run([str(exe),str(args.output/'costs.json'),*[str(p)for p in args.bag]],check=True)
    provenance={
        'created_at_utc':datetime.now(timezone.utc).isoformat(),
        'tracker_source_sha256':digest(source_path),
        'tracker_header_sha256':digest(PACKAGE/'include/algorithm/ekf_multi_object_tracking.hpp'),
        'orientation_header_sha256':digest(PACKAGE/'include/vehicle_orientation_canonicalizer.hpp'),
        'template_sha256':digest(template_path),'generated_cpp_sha256':digest(generated_path),
        'extracted_method_sha256':{name:hashlib.sha256(method.encode()).hexdigest() for name,method in zip(method_names,methods)},
        'compiler_command':command,
        'compiler_version':subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
        'bags':[{'path':str(p),'sha256':digest(p)}for p in args.bag],
        'scope':'Actual extracted methods on frozen message-derived confirmed tracks; no live node/publish/queue latency or full hidden EKF-state reproduction claim.',
        'slot_padding':'GetTrackResults returns all MAX_TRACKS=500 slots. Additional cases pad unpublished slots as unseen ID=-1; all-retired500 synthetic case stresses DELETE marker cost. Hidden tentative/retired slot contents cannot be recovered from published-only messages.',
        'subscriber_model':'Guard tested around visualization generation only when all legacy visualization subscribers absent. Canonical track generation always runs.',
        'serialization_scope':'Serialization measured separately for hypothetical subscribed outputs; roscpp already avoids serialization when no subscribers.',
        'production_changed':False,
    }
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    costs=json.loads((args.output/'costs.json').read_text())
    for case in costs['cases']:
        print(json.dumps({k:case[k]for k in ('name','mesh','frames','objects','always_total_ms','guard_no_subscriber_total_ms')}),flush=True)


if __name__=='__main__':main()
