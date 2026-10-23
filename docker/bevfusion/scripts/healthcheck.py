#!/usr/bin/env python3
"""Check that the local BEVFusion TCP server accepts the versioned protocol."""

import argparse
import socket

from app.protocol import TYPE_PING, TYPE_PONG, recv_message, send_message


def validate_production_identity(metadata,args):
    identity=metadata.get('runtime_identity') or {}
    expected_base5=args.expected_base5 == 'true'
    if (not metadata.get('model_loaded') or identity.get('checkpoint_sha256') != args.expected_checkpoint
            or identity.get('base5_runtime') != expected_base5
            or not identity.get('amp') or identity.get('compile_parts') != []
            or identity.get('parallel_camera') != expected_base5):
        raise ValueError('Server runtime differs from the pinned production checkpoint/options')
    if identity.get('box_center_origin', 'lidar_bottom_v1') != 'lidar_bottom_v1':
        raise ValueError('Production raw box-origin contract differs')
    if getattr(args,'require_box_origin_tag',False) and identity.get('box_center_origin')!='lidar_bottom_v1':
        raise ValueError('Corrected ROS output requires an explicitly tagged producer')
    policy=getattr(args,'expected_depth_raster_policy',None)
    if policy is not None and identity.get('depth_raster_policy')!=policy:
        raise ValueError('Server depth-hint policy differs from the selected contract')
    if args.expected_config and identity.get('config_sha256') != args.expected_config:
        raise ValueError('Server model configuration differs')
    if args.expected_sensor and (metadata.get('calibration') or {}).get('config_sha256') != args.expected_sensor:
        raise ValueError('Server sensor configuration differs')
    calibration = metadata.get('calibration') or {}
    expected_intrinsic = getattr(args, 'expected_intrinsic_source', None)
    if expected_intrinsic and calibration.get('intrinsic_source') != expected_intrinsic:
        raise ValueError('Server intrinsic-source differs from the ROS bridge')
    expected_order = getattr(args, 'expected_camera_order', None)
    if expected_order and [camera.get('frame_id') for camera in calibration.get('cameras', [])] != list(expected_order):
        raise ValueError('Server camera order differs from the ROS bridge')
    expected_backend = getattr(args, 'expected_camera_backend', None)
    if expected_backend:
        camera = (identity.get('execution_optimization') or {}).get('camera_tensorrt') or {}
        if expected_backend == 'eager' and camera.get('enabled', False):
            raise ValueError('Selected safe/baseline profile does not use a TensorRT camera')
        if expected_backend == 'camera_tensorrt' and not (
                camera.get('enabled') and camera.get('loaded') and camera.get('healthy')):
            raise ValueError('Selected TensorRT camera is not loaded and healthy')
    expected_pool = getattr(args, 'expected_pool_fused_layout', None)
    if expected_pool is not None:
        pool = (identity.get('execution_optimization') or {}).get('pool_fused_layout') or {}
        if bool(pool.get('enabled', False)) != (expected_pool == 'true'):
            raise ValueError('Server pool layout differs from the selected execution profile')
    if expected_base5 and (metadata.get('point_intensity_source') != 'ros255'
            or metadata.get('image_final_dim') != [512,1056]
            or metadata.get('classes') != ['vehicle','pedestrian','obstacle']):
        raise ValueError('Base5 server input/class contract differs')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=29500)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--expected-checkpoint")
    parser.add_argument("--expected-base5", choices=('true','false'))
    parser.add_argument("--expected-config")
    parser.add_argument("--expected-sensor")
    parser.add_argument('--require-box-origin-tag',action='store_true')
    parser.add_argument('--expected-depth-raster-policy',choices=('nearest_positive',))
    parser.add_argument('--expected-intrinsic-source',choices=('horizontal_fov','focal_length_pixel'))
    parser.add_argument('--expected-camera-order',nargs=4)
    parser.add_argument('--expected-camera-backend',choices=('eager','camera_tensorrt'))
    parser.add_argument('--expected-pool-fused-layout', choices=('true', 'false'))
    args = parser.parse_args()
    with socket.create_connection((args.host, args.port), args.timeout) as connection:
        send_message(connection, TYPE_PING, {"client": "healthcheck"})
        message_type, metadata, _blobs = recv_message(connection)
    if message_type != TYPE_PONG or not metadata.get("ready"):
        raise SystemExit("BEVFusion server is not ready: {}".format(metadata))
    if args.expected_checkpoint:
        validate_production_identity(metadata,args)
    print(
        "BEVFusion server: ready; model_loaded={}".format(
            metadata.get("model_loaded", False)
        )
    )


if __name__ == "__main__":
    main()

