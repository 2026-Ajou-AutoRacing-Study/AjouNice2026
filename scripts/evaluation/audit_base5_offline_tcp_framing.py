#!/usr/bin/env python3
"""Local real-protocol transport experiment, with no NN/GPU/ROS or LAN claim.

The current client/server already encode header, JSON and blobs into ONE
sendall. Candidate senders are process-local test substitutes, never edits to
the production transport/protocol. The split-header profile is deliberately
an adverse control, not a reproduction of the current implementation.
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import socket
import sys
import threading
import time

import numpy as np

from experiment_base5_proposal_peaks import ROOT, digest

sys.path.insert(0,str(ROOT/'base_ws/src/perception/bevfusion_ros/src'))
import bevfusion_ros.transport as client
from bevfusion_ros import protocol as client_protocol

spec=importlib.util.spec_from_file_location('_actual_bevfusion_app_protocol',ROOT/'docker/bevfusion/app/protocol.py')
server_protocol=importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_protocol)

MODES=('current_one_write','current_both_nodelay','prefix_coalesced_two_writes',
       'split_header_adverse_control','split_header_both_nodelay_control')


def split_sender(protocol, mode):
    if mode not in MODES:
        raise ValueError('Explicit known research transport mode required')
    if mode.startswith('current_'):
        return protocol.send_message
    def send(sock,message_type,metadata=None,blobs=None):
        wire=protocol.encode_message(message_type,metadata,blobs)
        _,_,_,_,metadata_size,_=protocol.HEADER.unpack(wire[:protocol.HEADER.size])
        boundaries=([protocol.HEADER.size] if mode.startswith('split_header') else [])
        boundaries.append(protocol.HEADER.size+metadata_size)
        start=0
        for end in boundaries+[len(wire)]:
            if end>start:
                sock.sendall(wire[start:end])
                start=end
    return send


def statistics(values, timing=True):
    a=np.asarray(values,dtype=float)
    result=dict(n=len(a),mean=float(a.mean()),p50=float(np.median(a)),p95=float(np.quantile(a,.95)),
                p99=float(np.quantile(a,.99)),max=float(a.max()),min=float(a.min()))
    if timing:
        result['over30ms']=int(np.count_nonzero(a>30.))
    return result


class TestTransport(client.TcpTransport):
    """Actual parent request/framing/reconnect path; optional test socket flag."""
    def __init__(self,port,nodelay):
        super().__init__('127.0.0.1',port,3.,5.)
        self.nodelay=nodelay

    def _connect(self):
        initial=self._socket is None
        connection=super()._connect()
        if self.nodelay and initial:
            connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        return connection


def request_fixtures(cohort,predictions,sensor_sha):
    rows=json.loads(cohort.read_text())['rows']
    import gzip
    payload=json.load(gzip.open(predictions,'rt'))
    inferred={r['token']:r for r in payload['rows']}
    fixtures=[]
    for i,row in enumerate(rows):
        points=Path(row['point_path']).read_bytes()
        if len(points)%16 or digest(row['point_path'])!=row['point_sha256']:
            raise ValueError('Immutable XYZI input differs')
        blobs={'points':points}
        for key,path in row['images'].items():
            blobs[key]=Path(path).read_bytes()
            if digest(path)!=row['image_sha256'][key]:
                raise ValueError('Immutable JPEG differs')
        cameras=[key[len('image/'):] for key in row['images']]
        metadata=dict(operation='inference',sequence=i+1,stamp=row['stamp'],lidar_frame='velodyne',
            camera_order=cameras,sensor_config_sha256=sensor_sha,intrinsic_source='horizontal_fov',
            sync_difference_sec={camera:0. for camera in cameras},points_blob='points',point_dim=4,
            point_count=len(points)//16,point_intensity_source='native' if row['source']=='native' else 'ros255',
            model_image_final_dim=[512,1056],score_threshold=.1,max_detections=100,
            lidar_deskew_mode='off',lidar_deskew_ms=0.,lidar_deskew_span_sec=0.,
            lidar_deskew_displacement_p95_m=0.,image_encoding='compressed',pointcloud_ms=0.,host_preprocess_ms=0.)
        r=inferred[row['token']]
        indices=[j for j,score in enumerate(r['pred_scores']) if score>=.1][:100]
        detections=[dict(center=r['pred_boxes'][j][:3],size=r['pred_boxes'][j][3:6],yaw=r['pred_boxes'][j][6],
            label=r['pred_labels'][j],class_name=('vehicle','pedestrian','obstacle')[r['pred_labels'][j]],
            score=r['pred_scores'][j],velocity=[0.,0.]) for j in indices]
        response=dict(operation='inference',stamp=metadata['stamp'],sequence=metadata['sequence'],frame_id='velodyne',
            detections=detections,box_center_origin='lidar_bottom_v1',query_memory_diagnostics={},
            preprocess_ms=0.,inference_ms=0.,postprocess_ms=0.,server_ms=0.)
        fixtures.append(dict(token=row['token'],metadata=metadata,blobs=blobs,response=response,
            wire_bytes=len(client_protocol.encode_message(client_protocol.TYPE_REQUEST,metadata,blobs)),
            response_wire_bytes=len(server_protocol.encode_message(server_protocol.TYPE_RESPONSE,response))))
    if len(fixtures)!=20:
        raise ValueError('Exactly the existing authored 20-frame cohort required')
    return fixtures,payload


def trial(fixtures,mode,delay_ms,repeats):
    nodelay='nodelay' in mode
    listener=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    listener.bind(('127.0.0.1',0));listener.listen(1)
    server_times=[];errors=[];verified=Counter()
    send=split_sender(server_protocol,mode)
    def serve():
        try:
            connection,_=listener.accept()
            with connection:
                connection.settimeout(5.)
                if nodelay:
                    connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                for index in range(1+len(fixtures)*repeats):
                    expected=fixtures[0] if index==0 else fixtures[(index-1)%len(fixtures)]
                    start=time.perf_counter()
                    kind,metadata,blobs=server_protocol.recv_message(connection)
                    if kind!=server_protocol.TYPE_REQUEST or blobs!=expected['blobs']:
                        raise ValueError('Actual wire request/blob equivalence failed')
                    without_descriptors={k:v for k,v in metadata.items() if k!='blobs'}
                    if without_descriptors!=expected['metadata']:
                        raise ValueError('Actual wire metadata equivalence failed')
                    verified['requests']+=1
                    if delay_ms:
                        time.sleep(delay_ms/1000.)
                    send(connection,server_protocol.TYPE_RESPONSE,expected['response'])
                    server_times.append((time.perf_counter()-start)*1000.)
        except Exception as error:
            errors.append(repr(error))
        finally:
            listener.close()
    thread=threading.Thread(target=serve,daemon=True);thread.start()
    transport=TestTransport(listener.getsockname()[1],nodelay)
    original=client.send_message
    client.send_message=split_sender(client_protocol,mode)
    elapsed=[];receive_equivalent=True
    try:
        # Connect/first-exchange is separately warmed, not hidden in steady RTT.
        for index,expected in enumerate([fixtures[0]]+fixtures*repeats):
            start=time.perf_counter()
            response,blobs=transport.request(expected['metadata'],expected['blobs'])
            value=(time.perf_counter()-start)*1000.
            expected_response=dict(expected['response'],blobs=[])
            if response!=expected_response or blobs:
                receive_equivalent=False
                raise ValueError('Frozen response equivalence failed')
            if index:
                elapsed.append(value)
            else:
                initial=value
    finally:
        client.send_message=original
        transport.close();thread.join(timeout=6.)
    if errors or thread.is_alive():
        raise RuntimeError('Stub failed: '+str(errors))
    excess=[max(0.,value-delay_ms) for value in elapsed]
    return dict(mode=mode,stub_hold_ms=delay_ms,roundtrip_ms=statistics(elapsed),
        roundtrip_minus_declared_hold_ms=statistics(excess),initial_exchange_ms=initial,
        server_receive_verify_hold_send_ms=statistics(server_times[1:]),
        payload_response_equivalent=receive_equivalent and verified['requests']==len(elapsed)+1,
        verified_requests=verified['requests'],roundtrip_samples_ms=elapsed)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('cohort','predictions','sensor','output'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--repeats',type=int,default=3)
    args=p.parse_args()
    if args.output.exists() or args.repeats<1:
        raise ValueError('Fresh output and positive repeat count required')
    fixtures,payload=request_fixtures(args.cohort,args.predictions,digest(args.sensor))
    report=dict(schema='base5_loopback_actual_tcp_framing_audit_v1',scope=__doc__,gpu_used=False,
        production_changed=False,wire_requests='Reconstructed actual request schema with immutable authored-cohort XYZI/JPEG bytes, not a captured on-wire request trace',
        fixture_tokens=[r['token'] for r in fixtures],fixtures=len(fixtures),repeats=args.repeats,
        fixture_wire_bytes=statistics([r['wire_bytes'] for r in fixtures],timing=False),
        fixture_response_wire_bytes=statistics([r['response_wire_bytes'] for r in fixtures],timing=False),
        input_sha256={str(p):digest(p) for p in (args.cohort,args.predictions,args.sensor)},
        learned_checkpoint_sha256=payload['checkpoint_sha256'],
        source_sha256={str(path):digest(path) for path in (Path(__file__),ROOT/'docker/bevfusion/app/protocol.py',
            ROOT/'base_ws/src/perception/bevfusion_ros/src/bevfusion_ros/protocol.py',
            ROOT/'base_ws/src/perception/bevfusion_ros/src/bevfusion_ros/transport.py')},
        trials=[],limitations=['No actual neural computation, image decode, ROS scheduling or LAN network tested.',
            'server receive timing includes waiting for the next request, parsing and equivalence validation; it is not NN latency.',
            'Subtracting artificial hold is a transport/serialization/scheduling estimate, not inference speed.',
            'Mode order is fixed; small timing differences can reflect OS load/noise. Split-header control is NOT current production framing.'])
    for delay in (0.,100.):
        for mode in MODES:
            row=trial(fixtures,mode,delay,args.repeats)
            report['trials'].append(row)
            print(json.dumps({k:v for k,v in row.items() if not k.endswith('_samples_ms')}),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    main()
