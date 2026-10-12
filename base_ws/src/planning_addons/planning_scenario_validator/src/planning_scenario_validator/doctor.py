"""Non-mutating environment checks, distinct from YAML and driving validation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

from .config import ROOT, load_campaign
from .traffic_light import runtime_settings, runtime_mode
from .models import resolve_contract


def command(args, timeout=8):
    try:
        result = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        return result.returncode == 0, result.stdout.strip()[-1800:]
    except (OSError, subprocess.TimeoutExpired) as error:
        return False, str(error)


def bev_settings():
    """Read literal settings only; never execute arbitrary shell during inspection."""
    values = dict(os.environ)
    path = ROOT / '.env.bevfusion.local'
    if path.is_file():
        for line in path.read_text().splitlines():
            match = re.match(r'^(?:export\s+)?(BEVFUSION_[A-Z_]+)=(.*)$', line.strip())
            if match:
                tokens = shlex.split(match.group(2), comments=True)
                if len(tokens) != 1 or '$(' in tokens[0] or '`' in tokens[0] or '${' in tokens[0]:
                    raise ValueError('복잡한 셸 표현식 설정은 검사기가 실행하지 않습니다: ' + match.group(1))
                values[match.group(1)] = tokens[0]
    checkpoint = values.get('BEVFUSION_CHECKPOINT') or values.get('BEVFUSION_INFERENCE_CHECKPOINT') or str(Path.home() / 'bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth')
    checkpoint = Path(os.path.expandvars(checkpoint)).expanduser()
    checkpoint = checkpoint if checkpoint.is_absolute() else ROOT / checkpoint
    script = (ROOT / 'scripts/runtime/run_bevfusion.sh').read_text()
    checksum = re.search(r'EXPECTED_SHA256="([a-f0-9]{64})"', script)
    return values.get('BEVFUSION_IMAGE', 'ajounice/bevfusion:team-cu128-v1'), checkpoint, checksum.group(1) if checksum else None


def inspect_environment(cfg, *, live=False, gpu=False):
    rows = []
    def add(label, status, detail, remedy=''):
        rows.append({'check': label, 'status': status, 'detail': str(detail), 'remedy': remedy if status != 'PASS' else ''})
    def probe(label, args, remedy='', timeout=8):
        ok, detail = command(args, timeout)
        if ok:
            detail = ' / '.join(detail.splitlines()[:3])[:240]
        add(label, 'PASS' if ok else 'FAIL', detail or '확인 완료', remedy if not ok else '')
        return ok
    add('파일·YAML', 'PASS', '{} / 장면 {}개. 주행 성공 판정이 아닙니다'.format(cfg['name'], len(cfg['cases'])))
    bag_enabled = cfg['recording'].get('bag_enabled', True)
    add('rosbag 녹화', 'PASS', '{} · 선택 {}개 토픽. 실시간 분석·JSON·CSV는 항상 유지'.format(
        'ON' if bag_enabled else 'OFF', len(cfg['recording']['topics'])))
    if bag_enabled:
        probe('rosbag 도구', ['rosbag', '--help'], 'ROS 환경을 source한 터미널에서 실행하세요')
    add('MORAI 최초 연결', 'WARN', 'GUI에서 Sensor/Ego Network Load·Connect를 직접 확인해야 합니다',
        'Network: {}\nSensor: {}'.format(cfg['network']['ego_network_json'], cfg['network']['sensor_json']))
    probe('ROS 패키지·메시지', ['/usr/bin/python3', '-c',
        'import rospy, rosgraph; from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, CollisionData; from autoware_perception_msgs.msg import DetectedObjects; from autoware_planning_msgs.msg import LaneletRoute, Trajectory; from autoware_internal_planning_msgs.srv import SetWaypointRoute'],
        '저장소 루트에서 bash scripts/workspace/setup_ws.sh 실행 후 GUI 재시작')
    try:
        launch = ROOT / 'base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch'
        paths = sorted({str(value).replace('$(arg repo_root)', str(ROOT))
                        for node in ET.parse(launch).iter() for value in node.attrib.values()
                        if str(value).startswith('$(arg repo_root)/kcity_map/')})
        missing = [path for path in paths if not Path(path).is_file()]
        add('현재 planning 맵 파일', 'FAIL' if missing or not paths else 'PASS',
            '\n'.join(missing or paths), '설치 가이드의 맵 다운로드 스크립트로 받기')
    except (OSError, ET.ParseError) as error:
        add('planning 맵 계약', 'FAIL', error, '현재 planning launch와 맵 파일 확인')
    probe('GUI·창 배치', ['/usr/bin/python3', '-c',
        'import tkinter, gi; gi.require_version("Wnck", "3.0"); from gi.repository import Wnck'],
        'sudo apt install python3-tk python3-gi gir1.2-wnck-3.0')
    if cfg['recording'].get('video', {}).get('enabled'):
        display = cfg['recording']['video']['display']
        if os.environ.get('XDG_SESSION_TYPE') == 'wayland':
            add('녹화 세션', 'FAIL', 'Wayland 세션입니다', '로그아웃 후 Ubuntu on Xorg로 로그인')
        probe('녹화 도구', ['ffmpeg', '-version'], 'sudo apt install ffmpeg')
        probe('녹화 화면', ['xdpyinfo', '-display', str(display)], 'GUI 고급 설정의 display와 MORAI 화면 DISPLAY 확인')
    parent = cfg['output']
    while not parent.exists() and parent != parent.parent:
        parent = parent.parent
    free = shutil.disk_usage(parent).free / 1024**3
    results_only = not bag_enabled and not cfg['recording'].get('video', {}).get('enabled')
    minimum = 0.1 if results_only else float(cfg['recording'].get('minimum_free_disk_gib', 10))
    add('디스크', 'PASS' if free >= minimum else 'FAIL', '여유 {:.1f}GiB / 시작 기준 {:.1f}GiB. 전체 배치 용량 보장은 아닙니다'.format(free, minimum), '결과를 백업하고 공간 확보')
    if cfg['stack'].get('detector') == 'bevfusion':
        try:
            checkpoint_label = 'E25 가중치'
            if cfg['stack'].get('bevfusion_model', 'e25') == 'e25':
                image, checkpoint, expected = bev_settings()
            else:
                model = resolve_contract(cfg['stack'], cfg['network']['sensor_json'], ROOT)
                image, checkpoint, expected = model['image'], Path(model['checkpoint_path']), model['checkpoint_sha256']
                checkpoint_label = model['label'] + ' 가중치·config'
            probe('BEVFusion Docker 이미지', ['docker', 'image', 'inspect', image, '--format', '{{.Id}}'],
                  '설치 가이드의 공용 이미지 다운로드·docker load 및 Docker 권한 확인')
            probe('Docker Compose', ['docker', 'compose', 'version'], '설치 가이드의 Docker Compose 플러그인 설치')
            probe('NVIDIA 드라이버', ['nvidia-smi', '--query-gpu=name,driver_version,memory.free', '--format=csv,noheader'], 'NVIDIA 드라이버부터 확인')
            if checkpoint.is_file() and expected:
                sha = hashlib.sha256()
                with checkpoint.open('rb') as handle:
                    for chunk in iter(lambda: handle.read(1024**2), b''):
                        sha.update(chunk)
                add(checkpoint_label, 'PASS' if sha.hexdigest() == expected else 'FAIL', str(checkpoint), '선택 모델의 SHA256 검증된 가중치·경로 확인')
            else:
                add(checkpoint_label, 'FAIL', '파일 또는 실행기 checksum 계약 없음: ' + str(checkpoint), '설치 가이드에서 가중치·경로 확인')
            if gpu:
                probe('컨테이너 실제 GPU 연산', ['docker', 'run', '--rm', '--gpus', 'all', '--entrypoint', 'python', image, '-c',
                    'import torch; print(torch.cuda.get_device_name()); print((torch.ones(2,device="cuda")+1).cpu())'],
                    'NVIDIA Container Toolkit·Docker GPU 권한 확인', timeout=40)
            else:
                add('컨테이너 실제 GPU 연산', 'WARN', 'GPU·컨테이너를 새로 띄우지 않았습니다', '원하면 GPU 검사 버튼 사용; 실제 모델 준비는 실행기에서 확인')
        except (ValueError, OSError) as error:
            add('BEVFusion 설정', 'FAIL', error, '로컬 .env와 기존 실행기 설정 확인')
    if cfg['stack'].get('traffic_light'):
        try:
            python, device = runtime_settings(cfg['stack'], ROOT, check_exists=True)
        except (ValueError, OSError) as error:
            add('신호등 Python 경로', 'FAIL', error, 'bash scripts/runtime/setup_traffic_light_env.sh 실행 후 YAML의 Python 경로 확인')
            python = None
        if python:
            inspect_traffic_light(cfg, python, device, gpu, probe, add)
    if cfg.get('upload', {}).get('enabled'):
        remote = cfg['upload'].get('rclone_remote', '').split(':')[0]
        ok, text = command(['rclone', 'listremotes'])
        add('Drive remote', 'PASS' if ok and remote + ':' in text.splitlines() else 'FAIL',
            'remote 존재만 확인. 업로드 권한·공유 폴더 접근은 실제 전송으로 확인해야 합니다', 'rclone config')
    if live:
        rows.extend(observe_topics(cfg))
    else:
        add('센서·차량 실시간 연결', 'WARN', '실시간 수신은 검사하지 않았습니다. UDP bridge가 아직 없으면 topic도 없습니다',
            '스택 수신 중 연결 검사 사용. MORAI Connect 여부는 사용자가 확인')
    return rows


def inspect_traffic_light(cfg, python, device, gpu, probe, add):
    args = [python, str(ROOT / 'scripts/runtime/check_traffic_light_env.py'), '--device', device]
    args.append('--check-ros')
    if runtime_mode(cfg['stack']) == 'virtualenv':
        args.append('--isolate-python')
    if gpu:
        probe('신호등 실제 GPU/모델', args, 'bash scripts/runtime/setup_traffic_light_env.sh', timeout=120)
    else:
        probe('신호등 Python import', [python] + (['-s'] if runtime_mode(cfg['stack']) == 'virtualenv' else []) + ['-c', 'import torch, ultralytics, rospy, cv_bridge'],
              'bash scripts/runtime/setup_traffic_light_env.sh')
        add('신호등 실제 추론', 'WARN', 'import만 확인했습니다. 실제 CUDA/YOLO 검사는 실행 전에 수행됩니다')


def observe_topics(cfg, duration=4):
    try:
        import rosgraph
        rosgraph.Master('/validator_doctor').getPid()
    except Exception as error:
        return [{'check': 'ROS 연결', 'status': 'WARN', 'detail': str(error), 'remedy': '현재 ROS_MASTER_URI와 roscore·UDP bridge 실행 여부 확인'}]
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node('validator_doctor', anonymous=True, disable_signals=True)
    topics = ['/Ego_topic', '/sensor/gps', '/sensor/imu', '/sensor/lidar1']
    if cfg['stack'].get('detector') == 'bevfusion':
        topics.append('/sensor/camera/front/compressed')
    if cfg['stack'].get('pose_source') == 'localization':
        topics.append('/Competition_topic')
    times = {topic: [] for topic in topics}
    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, lambda msg, key=topic: times[key].append(time.monotonic()), queue_size=1) for topic in topics]
    time.sleep(duration)
    for subscription in subscriptions:
        subscription.unregister()
    return [{'check': topic, 'status': 'PASS' if len(stamps) >= 2 else 'WARN',
             'detail': '{} messages / {:.1f} Hz (수신 시각 기준, {}초 관측)'.format(len(stamps),
                 (len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else 0, duration),
             'remedy': '' if len(stamps)>=2 else '센서 설정·UDP 포트·bridge 실행 확인; 미수신 원인을 단정하지 않음'} for topic, stamps in times.items()]


def main(argv=None):
    parser = argparse.ArgumentParser(description='설치 환경 검사: 파일 검사/실주행 성공과 별개')
    parser.add_argument('yaml')
    parser.add_argument('--live', action='store_true', help='실행 중인 ROS graph에서 4초 수신 관측')
    parser.add_argument('--gpu', action='store_true', help='컨테이너 GPU 및 선택된 신호등 실제 추론 추가')
    args = parser.parse_args(argv)
    try:
        rows = inspect_environment(load_campaign(args.yaml), live=args.live, gpu=args.gpu)
        for row in rows:
            print('[{}] {}: {}'.format(row['status'], row['check'], row['detail']), flush=True)
            if row['remedy']:
                print('  다음 확인: ' + row['remedy'], flush=True)
        failed = sum(row['status'] == 'FAIL' for row in rows)
        print('환경 검사 종료: FAIL {}건. WARN과 MORAI 연결은 별도 확인해야 합니다.'.format(failed), flush=True)
        return 1 if failed else 0
    except (ValueError, OSError) as error:
        print('환경 검사 오류: ' + str(error), flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
