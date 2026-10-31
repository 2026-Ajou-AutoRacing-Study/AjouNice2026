#!/usr/bin/env python3
"""Explicit container-only tiny CUDA + selected eager model warmup.

No listening socket, ROS publisher, training, registry or cloud access is used.
Invoke through preflight_bevfusion_competition.py --offline-smoke so source and
weights are read-only mounts and this process belongs to a --rm container.
"""
import argparse
from pathlib import Path
import sys
import time
import json
import socket
from types import SimpleNamespace


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', choices=['e12', 'e24', 'e25'], required=True)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--sensor-config', type=Path, required=True)
    a = p.parse_args()
    root = a.root.resolve(strict=True)
    sys.path.insert(0, str(root/'scripts/runtime'))
    from bevfusion_production_profile import profile, digest
    selected = profile(a.model)
    if digest(a.checkpoint) != selected['sha256']:
        raise ValueError('Selected production profile checkpoint SHA differs')
    # Model imports happen only INSIDE the explicitly requested container.
    import cv2
    import torch
    from app.inference_server import InferenceServer
    from app.protocol import TYPE_PING, TYPE_PONG, send_message, recv_message
    sys.path.insert(0, str(root/'docker/bevfusion/scripts'))
    from healthcheck import validate_production_identity
    result = dict(schema='bevfusion_competition_startup_smoke_v1', model=a.model,
        checkpoint_sha256=selected['sha256'], tiny_cuda=False, model_warmed=False,
        closed=False, network='none required by launcher', ros_started=False,
        model_server_started=False, camera_tensorrt=False, pool_fused_layout=False,
        full_stack_certified=False)
    started = time.perf_counter(); runtime = None; server = None
    try:
        if not torch.cuda.is_available(): raise RuntimeError('CUDA is not available inside selected image')
        torch.set_num_threads(4); cv2.setNumThreads(1)
        tiny = torch.ones(256, device='cuda:0')
        if tiny.sum().item() != 256: raise RuntimeError('Tiny CUDA arithmetic failed')
        del tiny
        result.update(tiny_cuda=True, gpu=torch.cuda.get_device_name(0),
            gpu_memory_mib=torch.cuda.get_device_properties(0).total_memory/2**20,
            torch_version=str(torch.__version__), torch_cuda_version=torch.version.cuda)
        config_path = root/selected['model_config']
        server = InferenceServer(SimpleNamespace(
            config=str(config_path), checkpoint=str(a.checkpoint), sensor_config=str(a.sensor_config),
            camera_order=['camera_front', 'camera_front_left', 'camera_front_right', 'camera_back'],
            intrinsic_source='horizontal_fov', projection_only=False, device='cuda:0', amp=True,
            base5_runtime=selected['base5'], parallel_camera=selected['parallel'],
            point_intensity_source='ros255' if selected['base5'] else 'native',
            preprocess_workers=4, warmup_iterations=2, temporal_feature_cache=False,
            temporal_maximum_age_sec=.30, camera_tensorrt_policy='off'))
        runtime = server.model
        calibration = server.calibration
        if not runtime.warmed_up: raise RuntimeError('Selected model did not finish warmup')
        # Exercise the REAL server health/protocol boundary without binding
        # any TCP port or connecting ROS. Half-close makes the handler return
        # normally after its single PING. No background thread/service stays.
        client, connection = socket.socketpair()
        try:
            client.settimeout(5); connection.settimeout(5)
            send_message(client, TYPE_PING, {'client': 'offline-preflight-socketpair'})
            client.shutdown(socket.SHUT_WR)
            server._serve_client(connection)
            message_type, metadata, _ = recv_message(client)
            if message_type != TYPE_PONG or not metadata.get('ready'):
                raise RuntimeError('Actual server did not return ready PONG after warmup')
            validate_production_identity(metadata, SimpleNamespace(
                expected_checkpoint=selected['sha256'], expected_base5=str(selected['base5']).lower(),
                expected_config=digest(config_path), expected_sensor=digest(a.sensor_config),
                expected_intrinsic_source='horizontal_fov', expected_camera_order=server.args.camera_order,
                expected_camera_backend='eager', expected_pool_fused_layout='false',
                expected_head_attention_sdpa='false', expected_head_cache_bev_pos='false',
                require_box_origin_tag=selected['base5'],
                expected_depth_raster_policy='nearest_positive' if selected['base5'] else None))
        finally:
            connection.close(); client.close()
        result.update(model_warmed=True, camera_order=[camera['frame_id'] for camera in calibration['cameras']],
            intrinsic_source=calibration['intrinsic_source'], sensor_sha256=calibration['config_sha256'],
            model_config_sha256=digest(config_path), point_intensity_source=runtime.preprocessor.point_intensity_source,
            classes=runtime.classes, server_ready_pong=True, selected_profile_health_passed=True,
            socket_scope='Local socketpair only; no TCP port/listening server or ROS')
    finally:
        if server is not None:
            server.close()
            result['closed'] = True
        result['elapsed_sec'] = time.perf_counter()-started
        print('COMPETITION_STARTUP_JSON='+json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
