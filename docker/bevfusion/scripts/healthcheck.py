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
    if args.expected_config and identity.get('config_sha256') != args.expected_config:
        raise ValueError('Server model configuration differs')
    if args.expected_sensor and (metadata.get('calibration') or {}).get('config_sha256') != args.expected_sensor:
        raise ValueError('Server sensor configuration differs')
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

