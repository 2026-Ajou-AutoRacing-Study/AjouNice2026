#!/usr/bin/env python3
"""Own one Base5 container + ROS bridge. E25 startup is deliberately separate."""
import argparse
import json
import signal
import socket
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
sys.path.insert(0, str(ROOT / 'docker/bevfusion'))
from app.protocol import TYPE_PING, TYPE_PONG, recv_message, send_message
from planning_scenario_validator.models import CANONICAL, trial_commands, verify_contract
from planning_scenario_validator.runtime import atomic_json, stop_group


def publishers():
    import rosgraph
    return dict(rosgraph.Master('/planning_validator_base5_guard').getSystemState()[0]).get(CANONICAL, [])


def guard(allowed=None):
    owners = publishers()
    if owners and (allowed is None or owners != [allowed]):
        raise RuntimeError('중복/외부 detector publisher 금지: {}'.format(owners))


def check_ready(contract):
    with socket.create_connection(('127.0.0.1', 29515), timeout=1) as connection:
        connection.settimeout(2)
        send_message(connection, TYPE_PING)
        kind, metadata, _ = recv_message(connection)
    if kind != TYPE_PONG or not metadata.get('model_loaded'):
        raise RuntimeError('Base5 서버가 모델을 준비하지 않았습니다')
    identity = metadata.get('runtime_identity', {})
    expected = {'checkpoint_sha256': contract['checkpoint_sha256'], 'config_sha256': contract['model_config_sha256'],
                'amp': True, 'parallel_camera': True, 'compile_parts': [], 'base5_runtime': True}
    if contract.get('depth_raster_policy') is not None:
        expected['depth_raster_policy'] = contract['depth_raster_policy']
    if contract.get('box_center_output_origin') == 'gravity_center_v1':
        expected['box_center_origin'] = 'lidar_bottom_v1'
    if any(identity.get(k) != v for k, v in expected.items()):
        raise RuntimeError('Base5 서버 모델/config/runtime 불일치: {}'.format(identity))
    if (metadata.get('point_intensity_source') != 'ros255' or metadata.get('classes') != ['vehicle', 'pedestrian', 'obstacle']
            or metadata.get('image_final_dim') != [512, 1056]
            or metadata.get('calibration', {}).get('config_sha256') != contract['sensor_sha256']):
        raise RuntimeError('Base5 서버 센서/기하/intensity/클래스 계약 불일치')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', required=True)
    parser.add_argument('--publish-frame-timing', action='store_true')
    args = parser.parse_args()
    path = Path(args.contract)
    contract = json.loads(path.read_text())
    if contract['preset'] not in ('base5_e24', 'base5_e9'):
        raise ValueError('Base5 전용 실행기입니다. E25는 run_bevfusion.sh를 사용하세요')
    verify_contract(contract, {'detector': 'bevfusion', 'bevfusion_model': contract['preset']}, contract['sensor_config'], ROOT)
    guard()
    # Refuse another server rather than accidentally using its checkpoint.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 29515))
    name = 'planning-validator-base5-' + uuid.uuid4().hex[:12]
    server_command, bridge_command = trial_commands(contract, ROOT, name)
    bridge_command += ['publish_frame_timing:=' + str(args.publish_frame_timing).lower()]
    image_id = subprocess.check_output(['docker', 'image', 'inspect', '--format', '{{.Id}}', contract['image']], text=True).strip()
    if contract.get('image_id', image_id) != image_id:
        raise RuntimeError('Docker image가 실행 준비 후 바뀌었습니다. 새 실험을 시작하세요')
    receipt = {'contract_fingerprint': contract['fingerprint'], 'image_id': image_id,
               'container': name, 'server_command': server_command, 'bridge_command': bridge_command}
    atomic_json(path.parent / 'model_process_receipt.json', receipt)
    server = bridge = None
    def interrupted(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        print('Base5 server command: {}'.format(server_command), flush=True)
        server = subprocess.Popen(server_command, start_new_session=True)
        deadline = time.monotonic() + 180
        while True:
            if server.poll() is not None:
                raise RuntimeError('Base5 Docker 서버 종료: {}'.format(server.returncode))
            guard()
            try:
                ready = check_ready(contract)
                break
            except (OSError, EOFError):
                if time.monotonic() >= deadline:
                    raise RuntimeError('Base5 모델 준비 180초 초과')
                time.sleep(0.5)
        receipt['server_ready'] = ready
        atomic_json(path.parent / 'model_process_receipt.json', receipt)
        print('Base5 bridge command: {}'.format(bridge_command), flush=True)
        bridge = subprocess.Popen(bridge_command, start_new_session=True)
        while bridge.poll() is None and server.poll() is None:
            guard('/base5_trial/bevfusion_ros_bridge')
            time.sleep(0.5)
        raise RuntimeError('Base5 서버 또는 ROS bridge가 종료됐습니다')
    except KeyboardInterrupt:
        return 0
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if bridge is not None:
            stop_group(bridge, timeout=3)
        # Only the container owned by this execution; no compose down/global kill.
        subprocess.run(['docker', 'rm', '-f', name], timeout=8, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if server is not None:
            stop_group(server, timeout=2)


if __name__ == '__main__':
    sys.exit(main())
