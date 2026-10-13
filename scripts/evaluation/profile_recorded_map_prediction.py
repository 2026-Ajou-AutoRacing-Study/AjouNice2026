#!/usr/bin/env python3
"""Exact ROS-message equivalence and CPU-only node profiling on saved tracks."""
import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
import cProfile
import gc
import gzip
import importlib.util
import io
import json
from pathlib import Path
import pstats
import sys
import time
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'perception/detection3d/bevfusion/tools'),
               str(ROOT/'base_ws/src/perception_object_pipeline/src')]
from audit_base5_predicted_path_selection import stats
from experiment_base5_map_cv_candidates import digest, evaluate, prepare_network, SPEC
from prepare_base5_map_cv_regression import prepare_case
from review_base5_map_cv_candidate_risk import summarize


def serialize(message):
    output = io.BytesIO()
    message.serialize(output)
    return output.getvalue()


def identity(message):
    return (message.header.stamp.to_nsec(), message.header.frame_id,
            tuple(bytes(o.object_id.uuid).hex() for o in message.objects))


@contextmanager
def observe_gc():
    """Diagnostic only: time collections, never disable or force GC."""
    active={}
    pauses=[]
    def callback(phase,info):
        generation=info['generation']
        if phase=='start':
            active[generation]=time.perf_counter()
        elif generation in active:
            pauses.append(dict(generation=generation,
                               ms=(time.perf_counter()-active.pop(generation))*1000))
    gc.callbacks.append(callback)
    try:
        yield pauses
    finally:
        gc.callbacks.remove(callback)


def main():
    import rosbag
    import perception_object_pipeline.map_based_prediction as core
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--skip-candidate', action='store_true', help='Repeat paired timing without repeating frozen GT research')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    bag_path, = args.run.glob('*.bag')
    params = yaml.safe_load((args.run/'ros_parameters.yaml').read_text())['map_based_prediction']
    config = core.MapPredictionConfig(**{k:v for k,v in params.items() if k in core.MapPredictionConfig.__dataclass_fields__})
    network = core.LaneletNetwork.load(ROOT/'kcity_map/lanelet_map'/Path(params['map_file']).name,
                                      ROOT/'kcity_map/lanelet_map'/Path(params['projector_file']).name,config)
    spec = importlib.util.spec_from_file_location('profiled_map_node',
        ROOT/'base_ws/src/perception_object_pipeline/scripts/map_based_prediction_node.py')
    node = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node)
    horizons = node.build_prediction_horizons(params['prediction_time_step'],params['prediction_horizon'])
    indexed = core._RoutePolyline
    class ScalarReference:
        def __init__(self,points): self.points=points; self.length=core.polyline_length(points)
        def sample(self,distance): return core.sample_polyline(self.points,distance)
        def tangent(self,distance): return core.tangent_yaw(self.points,distance)
    frames=[]
    recorded=defaultdict(list)
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/perception/object_recognition/tracking/objects',
                                                   '/perception/object_recognition/objects']):
            if topic.endswith('/tracking/objects'):
                # Retain wire bytes, not thousands of nested ROS object graphs.
                # A whole-bag object heap causes artificial gen-2 GC pauses that
                # a latest-only live callback does not normally retain.
                frames.append((type(msg),serialize(msg),len(msg.objects)))
            else:
                # Normalize only seq: assigned separately by rospy publishers.
                msg.header.seq=0
                recorded[identity(msg)].append(serialize(msg))
    costs = defaultdict(list)
    counts = Counter()
    prediction_core = network.predict
    components = defaultdict(list)
    slow_frames = []
    for index,(message_type,payload,_) in enumerate(frames):
        msg=message_type().deserialize(payload)
        outputs={}
        order=('scalar','indexed') if index%2 else ('indexed','scalar')
        for mode in order:
            core_cost=[]
            def timed_predict(*a,**kw):
                before=time.perf_counter()
                value=prediction_core(*a,**kw)
                core_cost.append((time.perf_counter()-before)*1000)
                return value
            with patch.object(core,'_RoutePolyline',ScalarReference if mode=='scalar' else indexed), \
                 patch.object(network,'predict',timed_predict), observe_gc() as gc_pauses:
                gc_before=sum(g['collections'] for g in gc.get_stats())
                cpu_start=time.process_time()
                start=time.perf_counter()
                output,mapped,fallback,paths=node.make_map_based_predicted_objects(msg,horizons,network)
                make_ms=(time.perf_counter()-start)*1000
                cpu_ms=(time.process_time()-cpu_start)*1000
                gc_count=sum(g['collections'] for g in gc.get_stats())-gc_before
            output.header.seq=0
            start=time.perf_counter();payload=serialize(output)
            ser_ms=(time.perf_counter()-start)*1000
            outputs[mode]=payload
            if index>=30 and msg.objects:
                costs[mode+'/make_message_ms'].append(make_ms)
                costs[mode+'/serialize_ms'].append(ser_ms)
                costs[mode+'/make_and_serialize_ms'].append(make_ms+ser_ms)
                components[mode+'/vehicle_map_core_ms'].append(sum(core_cost))
                components[mode+'/conversion_and_fallback_ms'].append(make_ms-sum(core_cost))
                components[mode+'/process_cpu_make_ms'].append(cpu_ms)
                if make_ms>30:
                    slow_frames.append(dict(index=index,stamp=msg.header.stamp.to_sec(),mode=mode,
                                            objects=len(msg.objects),wall_ms=make_ms,process_cpu_ms=cpu_ms,
                                            gc_collections=gc_count,gc_pauses=gc_pauses))
        if outputs['scalar'] != outputs['indexed']:
            raise ValueError('Scalar/indexed ROS payload differs at '+str(msg.header.stamp))
        counts['frames']+=1;counts['nonempty_frames']+=bool(msg.objects)
        counts['objects']+=len(msg.objects);counts['paths']+=paths
        counts['future_poses']+=sum(max(0,len(p.path)-1) for o in output.objects for p in o.kinematics.predicted_paths)
        candidates=recorded.get(identity(msg),[])
        if len(candidates)==1:
            counts['recorded_pairs']+=1
            counts['recorded_payload_equal']+=outputs['indexed']==candidates[0]
        elif candidates:counts['ambiguous_recorded_pairs']+=1
    nonempty=[f for f in frames if f[2]]
    selected=nonempty[::max(1,len(nonempty)//200)][:200]
    profiler=cProfile.Profile();profiler.enable()
    for message_type,payload,_ in selected:
        msg=message_type().deserialize(payload)
        serialize(node.make_map_based_predicted_objects(msg,horizons,network)[0])
    profiler.disable()
    stream=io.StringIO()
    pstats.Stats(profiler,stream=stream).strip_dirs().sort_stats('cumulative').print_stats(45)
    (args.output/'node_cprofile.txt').write_text(stream.getvalue())
    report=dict(bag_sha256=digest(bag_path),core_sha256=digest(core.__file__),
        node_sha256=digest(node.__file__),map_sha256=network.map_sha256,
        counts=dict(counts),nonempty_frame_cpu_ms={k:stats(v) for k,v in costs.items()},
        nonempty_frame_components_ms={k:stats(v) for k,v in components.items()},
        slow_frames_gt30ms=slow_frames,
        exact_scalar_indexed_ros_payload=True,policy_changed=False,
        limitations=['CPU on local PC, not team 4090 host; 30 initial frames excluded from timing.',
                     'Input retained as wire bytes; deserialization outside timed callback. GC remains enabled.',
                     'All track frames tested, including empty and fallback; nonempty timing reported.',
                     'Message construction plus explicit ROS serialization, no TCP/publish/queue/deserialize measurement.',
                     'Recorded-payload comparison normalizes only publisher seq; unmatched keys not guessed.',
                     'cProfile is separate diagnostic run, not timing benchmark.'])
    (args.output/'geometry_and_node_cost.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Geometry',json.dumps(report,indent=2),flush=True)
    if args.skip_candidate:
        return
    # Frozen research candidate: validate once, never tune or promote on this run.
    transform=yaml.safe_load((ROOT/'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']
    packed=prepare_case(bag_path,'team_261014',transform,
                        ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json',1)
    with gzip.open(args.output/'recorded_pack.json.gz','wt') as out:
        json.dump(dict(cases={'team_261014':packed}),out,allow_nan=False)
    gt=({('team_261014',round(r['stamp'],6)):r['objects'] for r in packed['gt_frames']},
        {tuple(r['key']):r['series'] for r in packed['gt_trajectories']})
    prepare_network(network)
    result=evaluate(network,packed['frames'],gt,'team_261014',('baseline','route_history'))
    candidate=dict(spec=SPEC,experiment_source_sha256=digest(ROOT/'perception/detection3d/bevfusion/tools/experiment_base5_map_cv_candidates.py'),
                   counts=result['counts'],metrics=result['metrics'],effects=result['effects'],risk=summarize(result),
                   provenance=packed['provenance'],production_enabled=False,
                   caveat='Frozen candidate validated on new recorded tracks, not retuned. Conditional catalog-GT '
                          'FDE, not detector accuracy or closed-loop planner certification; previous deployment veto remains.')
    with gzip.open(args.output/'candidate_rows.json.gz','wt') as out: json.dump(result,out,allow_nan=False)
    (args.output/'frozen_history_candidate.json').write_text(json.dumps(candidate,indent=2,allow_nan=False)+'\n')
    print('Candidate',json.dumps({k:candidate[k] for k in ('counts','metrics','effects')},indent=2),flush=True)


if __name__=='__main__': main()
