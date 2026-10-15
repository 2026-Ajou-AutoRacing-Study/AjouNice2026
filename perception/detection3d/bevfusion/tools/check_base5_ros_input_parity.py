#!/usr/bin/env python3
"""Compare actual bridge/TCP/server inputs and learned outputs with offline val.

Uses a private localhost listener, not a ROS master, queues or competition LAN.
No CUDA compilation, AMP, TF32, training or production-topic writes.
"""
import argparse
import copy
import json
from pathlib import Path
import pickle
import socket
import threading
from types import SimpleNamespace

import cv2
import numpy as np
import torch
from mmengine.config import Config
from mmengine.runner import Runner
from mmengine.dataset import pseudo_collate
from mmdet3d.utils import register_all_modules

from app.inference_server import InferenceServer, parse_points
from app.protocol import recv_message, TYPE_RESPONSE, TYPE_REQUEST
from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration
from compare_base5_common30 import adapt_intensity, digest


def check(args):
    if args.output.exists():
        raise ValueError('Fresh parity report required')
    torch.set_num_threads(4)
    cv2.setNumThreads(1)
    register_all_modules(init_default_scope=True)
    cfg = Config.fromfile(args.config)
    with (args.inputs / 'infos_val.pkl').open('rb') as f:
        rows = pickle.load(f)['data_list']
    requests = json.loads((args.requests / 'manifest.json').read_text())
    selected = {r['token']: r for r in requests['requests']}
    loader_cfg = copy.deepcopy(cfg.val_dataloader)
    loader_cfg.update(batch_size=1, num_workers=0, persistent_workers=False)
    loader_cfg.sampler = dict(type='DefaultSampler', shuffle=False)
    loader_cfg.dataset.update(type='MoraiBase3Dataset', data_root=str(args.inputs),
        ann_file=str(args.inputs / 'infos_val.pkl'), data_prefix=dict(pts='', img=''),
        test_mode=True, load_eval_anns=True, filter_empty_gt=False)
    loader = Runner.build_dataloader(loader_cfg)
    if args.inputs_only:
        calibration=load_sensor4_calibration(args.sensor)
        preprocessor=Base3OnlinePreprocessor(calibration,point_intensity_source='ros255',
                                            **preprocessing_options_from_config(cfg))
        server=SimpleNamespace(model=SimpleNamespace(preprocessor=preprocessor))
    else:
        server = InferenceServer(SimpleNamespace(sensor_config=args.sensor, camera_order=None,
        intrinsic_source='horizontal_fov', projection_only=False, config=args.config,
        checkpoint=args.checkpoint, device='cuda:0', amp=False, preprocess_workers=4,
        warmup_iterations=0, temporal_feature_cache=False, temporal_maximum_age_sec=.3,
        base5_runtime=True, point_intensity_source='ros255',fp32_reference=True))
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    receipts = []
    for index, source in enumerate(rows):
        token = source['token']
        if token not in selected:
            continue
        batch = pseudo_collate([loader.dataset[index]])
        request_bytes = (args.requests / selected[token]['request']).read_bytes()
        class ByteReader:
            def __init__(self, data): self.data = data
            def recv(self, size):
                chunk, self.data = self.data[:size], self.data[size:]
                return chunk
        kind, metadata, blobs = recv_message(ByteReader(request_bytes))
        if kind != TYPE_REQUEST:
            raise ValueError('Actual bridge request kind differs')
        online = server.model.preprocessor.prepare(parse_points(metadata, blobs), blobs,
                    sample_idx=token, images_preprocessed=metadata['image_encoding']=='rgb8',
                    timestamp_sec=metadata['stamp'])
        batch['inputs']['points'] = [adapt_intensity(p, 1./255.) for p in batch['inputs']['points']]
        np.testing.assert_array_equal(batch['inputs']['points'][0].numpy(), online['inputs']['points'][0].numpy())
        np.testing.assert_array_equal(batch['inputs']['img'][0].numpy(), online['inputs']['img'][0].numpy())
        for key in ('cam2img', 'lidar2cam', 'cam2lidar', 'lidar2img', 'img_aug_matrix', 'lidar_aug_matrix'):
            if key != 'lidar_aug_matrix' and key not in batch['data_samples'][0].metainfo:
                raise ValueError('Offline projection metadata missing '+key)
            np.testing.assert_allclose(batch['data_samples'][0].metainfo.get(key,np.eye(4,dtype=np.float32)),
                                       online['data_samples'][0].metainfo[key], atol=1e-5, rtol=0)
        raw_points = np.fromfile(args.inputs / source['lidar_points']['lidar_path'], dtype='<f4').reshape(-1, 4)
        compressed = {'image/'+name: (args.inputs / image['img_path']).read_bytes()
                      for name, image in source['images'].items()}
        direct = server.model.preprocessor.prepare(raw_points, compressed)
        np.testing.assert_array_equal(direct['inputs']['img'][0].numpy(), online['inputs']['img'][0].numpy())
        if args.inputs_only:
            receipts.append(dict(token=token,image_pixels_exact=True,points_exact=True,
                                 projection_atol=1e-5,learned_output_checked=False))
            continue
        with torch.inference_mode():
            pred = server.model.model.test_step(batch)[0].pred_instances_3d
        scores = pred.scores_3d.cpu().numpy()
        keep = np.flatnonzero(scores >= .1)
        keep = keep[np.argsort(-scores[keep], kind='stable')][:100]
        expected_boxes = pred.bboxes_3d.tensor.cpu().numpy()[keep, :7]
        expected_scores = scores[keep]
        expected_labels = pred.labels_3d.cpu().numpy()[keep]
        # Exercise real localhost TCP framing and the actual server handler.
        errors = []
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0)); listener.listen(1)
            def serve_one():
                try:
                    conn, _ = listener.accept()
                    with conn:
                        kind, meta, payload = recv_message(conn)
                        if kind != TYPE_REQUEST: raise ValueError('Unexpected request')
                        server._handle_request(conn, meta, payload)
                except Exception as error: errors.append(repr(error))
            thread = threading.Thread(target=serve_one)
            thread.start()
            with socket.create_connection(listener.getsockname(), timeout=60) as client:
                client.settimeout(120); client.sendall(request_bytes)
                kind, response, _ = recv_message(client)
            thread.join(timeout=10)
        if errors or thread.is_alive() or kind != TYPE_RESPONSE:
            raise ValueError('TCP parity handler failed: '+str(errors))
        detections = response['detections']
        actual_boxes = np.asarray([d['center']+d['size']+[d['yaw']] for d in detections]).reshape(-1, 7)
        actual_scores = np.asarray([d['score'] for d in detections])
        actual_labels = np.asarray([d['label'] for d in detections])
        np.testing.assert_array_equal(actual_labels, expected_labels)
        np.testing.assert_allclose(actual_boxes, expected_boxes, atol=1e-4, rtol=0)
        np.testing.assert_allclose(actual_scores, expected_scores, atol=1e-5, rtol=0)
        receipts.append(dict(token=token, detections=len(detections),
            image_pixels_exact=True, points_exact=True, projection_atol=1e-5,
            box_max_abs=float(np.max(abs(actual_boxes-expected_boxes))) if len(keep) else 0.,
            score_max_abs=float(np.max(abs(actual_scores-expected_scores))) if len(keep) else 0.))
        print(json.dumps(receipts[-1]), flush=True)
    if len(receipts) != len(selected) or (not args.inputs_only and not any(r['detections'] for r in receipts)):
        raise ValueError('Incomplete parity or all-empty vacuous output')
    report = dict(passed=True, learned_output_checked=not args.inputs_only,
        scope=('CPU-only actual bridge/preprocessor input gate; no learned output or TCP qualification.' if args.inputs_only else
               'Actual ROS message bridge preparation -> private localhost TCP -> server/model versus offline pipeline. NOT live subscription/queue/LAN/13Hz certification.'),
        checkpoint_sha256=digest(args.checkpoint), config_sha256=digest(args.config),
        bridge_manifest_sha256=digest(args.requests/'manifest.json'),
        point_intensity_source='ros255; fixed /255 on server only', frames=receipts)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    server.model.preprocessor.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--requests', type=Path, required=True)
    p.add_argument('--config', required=True)
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--sensor', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--inputs-only',action='store_true',help='CPU-only input gate; not learned-output/TCP qualification')
    check(p.parse_args())
