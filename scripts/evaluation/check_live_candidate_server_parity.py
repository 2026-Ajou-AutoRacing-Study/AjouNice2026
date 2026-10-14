#!/usr/bin/env python3
"""Same-byte inputs through actual trial TCP server vs saved full forwards.

Twenty deterministic snapshots across all cases. Not another GT/latency test.
Makes image/vendored-dependency deployment mismatches visible before promotion.
"""
import argparse
from collections import defaultdict
import gzip
import json
from pathlib import Path
import socket
import sys

import numpy as np
from experiment_base5_proposal_peaks import ROOT,CAMERAS,digest,uniform_indices
from analyze_live_perception_candidates import paired


def main(args):
    if not 1<=args.per_case<=64:raise ValueError('Bounded 1..64 snapshots per case required')
    sys.path.insert(0,str(ROOT/'base_ws/src/perception/bevfusion_ros/src'))
    from bevfusion_ros.protocol import TYPE_PING,TYPE_PONG,recv_message,send_message
    from bevfusion_ros.transport import TcpTransport
    if args.output.exists():raise FileExistsError('Fresh parity output required')
    saved=json.load(gzip.open(args.reference,'rt'))
    with socket.create_connection(('127.0.0.1',args.port),5) as connection:
        connection.settimeout(10);send_message(connection,TYPE_PING,dict(client='frozen_live_parity'))
        kind,health,_=recv_message(connection)
    identity=health.get('runtime_identity') or {}
    if kind!=TYPE_PONG or identity.get('checkpoint_sha256')!=saved['checkpoint_sha256'] or identity.get('depth_raster_policy')!='nearest_positive' or identity.get('compile_parts')!=[]:
        raise ValueError('Actual server differs from frozen checkpoint/depth/compile contract')
    grouped=defaultdict(list)
    for row in saved['rows']:grouped[row['case']].append(row)
    selected=[rows[i] for rows in grouped.values() for i in uniform_indices(len(rows),args.per_case)]
    transport=TcpTransport('127.0.0.1',args.port,5,30)
    actual=[];repeated=[];raw=[]
    try:
        for index,row in enumerate(selected):
            point=Path(row['point_path']).read_bytes()
            if digest(row['point_path'])!=row['point_sha256']:raise ValueError('Frozen points changed')
            images={k:Path(p).read_bytes() for k,p in row['images'].items()}
            if any(digest(row['images'][k])!=row['image_sha256'][k] for k in images):raise ValueError('Frozen JPEG changed')
            metadata=dict(operation='inference',sequence=index,stamp=row['stamp'],lidar_frame='lidar1',
                camera_order=['camera_'+c for c in CAMERAS],sensor_config_sha256=(health.get('calibration') or {})['config_sha256'],
                intrinsic_source='horizontal_fov',points_blob='points',point_dim=4,point_count=len(point)//16,
                point_intensity_source='ros255',model_image_final_dim=[512,1056],score_threshold=0.,max_detections=200,image_encoding='compressed')
            response,_=transport.request(metadata,dict(images,points=point))
            if response.get('box_center_origin')!='lidar_bottom_v1':raise ValueError('Origin tag changed')
            predictions=response['detections']
            actual.append(dict(row,pred_boxes=[d['center']+d['size']+[d['yaw']] for d in predictions],
                pred_labels=[d['label'] for d in predictions],pred_scores=[d['score'] for d in predictions]))
            repeat,_=transport.request(metadata,dict(images,points=point))
            predictions=repeat['detections']
            repeated.append(dict(row,pred_boxes=[d['center']+d['size']+[d['yaw']] for d in predictions],
                pred_labels=[d['label'] for d in predictions],pred_scores=[d['score'] for d in predictions]))
            raw.append(dict(token=row['token'],response=response))
    finally:transport.close()
    result=dict(server_health=health,reference_sha256=digest(args.reference),frames=len(selected),
        comparison=paired(selected,actual),actual_server_repeat=paired(actual,repeated),raw_server_responses=raw,
        scope='Exact same point/JPEG/stamp, actual loaded Docker TCP path vs source-bootstrap saved predictions. No new GT/full-stack certification.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(frames=len(selected),comparison=result['comparison'],repeat=result['actual_server_repeat']),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=29515)
    p.add_argument('--per-case',type=int,default=2)
    main(p.parse_args())
