"""Pinned production E24 and explicit comparison/rollback detector contracts."""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import yaml

E25_SHA = '36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011'
BASE5_CONFIG = 'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_pedestrian_merged_runtime.py'
BASE5_CONFIG_GRAPH_SHA = '88ac69c228daae7b8c7a96a97e412c51d9694e2ef6207093bab9e880dea479e4'
PRESETS = {
    'base5_e24': {'label': 'Base5 E24 production',
                  'sha256': '84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746',
                  'thresholds': [0.40, 0.10, 0.10], 'epoch': 24},
    'e25': {'label': 'E25 rollback', 'sha256': E25_SHA, 'thresholds': [0.40, 0.15, 0.10]},
    'base5_e9': {'label': 'Base5 E9 val best (비교용)',
                 'sha256': '067205c3f5f5b0abc1bb775116c2b8bca2c9a7f31a0727b959db0ab1ce70b271',
                 'thresholds': [0.30, 0.20, 0.12], 'epoch': 9},
}
DEFAULT_MODEL = 'base5_e24'
LABEL_PRESETS = {v['label']: k for k, v in PRESETS.items()}
CANONICAL = '/perception/object_recognition/detection/objects'


def selected_model(stack):
    value = stack.get('bevfusion_model', DEFAULT_MODEL)
    if not isinstance(value, str) or value not in PRESETS:
        raise ValueError('stack.bevfusion_model은 e25, base5_e24, base5_e9 중 하나입니다')
    if any(key in stack for key in ('bevfusion_checkpoint', 'bevfusion_config')):
        raise ValueError('checkpoint/config 직접 교체는 금지합니다. 검증된 모델 프리셋을 선택하세요')
    return value


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def config_graph(path, root):
    """Hash all inherited Python configs without importing Torch/MMEngine."""
    result = {}

    def visit(path):
        path = Path(path).resolve()
        relative = str(path.relative_to(root))
        if relative in result:
            return
        result[relative] = sha256(path)
        for node in ast.parse(path.read_text()).body:
            if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == '_base_' for t in node.targets):
                bases = ast.literal_eval(node.value)
                for base in ([bases] if isinstance(bases, str) else bases):
                    visit(path.parent / base)
    visit(path)
    return result


def e25_environment(root):
    environment = dict(os.environ)
    local = root / '.env.bevfusion.local'
    if local.is_file():
        # Same local environment file already sourced by the protected E25 script.
        output = subprocess.check_output(['bash', '-c', 'set -a; source "$1"; env -0', 'env', str(local)])
        environment.update(dict(item.decode().split('=', 1) for item in output.split(b'\0') if b'=' in item))
    return environment


def resolve_contract(stack, sensor, root, *, require_assets=True):
    root = Path(root).resolve()
    if stack.get('detector') != 'bevfusion':
        return {'schema_version': 1, 'preset': 'od_gt'}
    name = selected_model(stack)
    preset = PRESETS[name]
    env = e25_environment(root) if name == 'e25' else os.environ
    if name == 'e25':
        checkpoint = Path(env.get('BEVFUSION_CHECKPOINT') or env.get('BEVFUSION_INFERENCE_CHECKPOINT') or
                          str(Path.home() / 'bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth'))
        configured = env.get('BEVFUSION_MODEL_CONFIG', '/workspace/AjouNice2026/perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py')
        model_config = root / configured[len('/workspace/AjouNice2026/'):] if configured.startswith('/workspace/AjouNice2026/') else Path(configured)
        runtime_config = root / 'configs/perception/bevfusion_sensor4.yaml'
        amp = env.get('BEVFUSION_USE_AMP', 'true') == 'true'
    else:
        checkpoint = (Path.home() / 'bevfusion_artifacts/checkpoints/base5_e24_inference.pth' if name == 'base5_e24'
                      else root / 'artifacts/evaluation/bevfusion/base5_rosbag_quick_261012/checkpoints/E9.pth')
        model_config = root / BASE5_CONFIG
        runtime_config = root / 'configs/perception/bevfusion_base5_trial.yaml'
        amp = True
    checkpoint = checkpoint.expanduser().resolve()
    if not model_config.is_absolute():
        model_config = root / model_config
    graph = config_graph(model_config, root)
    if name != 'e25' and fingerprint(graph) != BASE5_CONFIG_GRAPH_SHA:
        raise ValueError('Base5 checkpoint/config 계약 불일치: 상속 config hash가 검증본과 다릅니다')
    actual_sha = sha256(checkpoint) if checkpoint.is_file() else None
    if (require_assets and actual_sha is None) or (actual_sha is not None and actual_sha != preset['sha256']):
        raise ValueError('checkpoint SHA256 불일치/파일 없음: {} (expected {})'.format(checkpoint, preset['sha256']))
    runtime = yaml.safe_load(runtime_config.read_text())
    if name != 'e25':
        required = {'image_encoding': 'compressed', 'point_intensity_source': 'ros255',
                    'image_final_height': 512, 'image_final_width': 1056, 'server_port': 29515,
                    'server_host': '127.0.0.1', 'image_resize': 0.825, 'image_reference_width': 1280,
                    'intrinsic_source': 'horizontal_fov', 'max_detections': 100, 'score_threshold': 0.10,
                    'temporal_sweeps': 0, 'temporal_feature_cache': False,
                    'query_memory_context': False, 'publish_detector_velocity': False}
        if any(runtime.get(key) != value for key, value in required.items()):
            raise ValueError('Base5 bridge 입력/temporal 계약 불일치')
        if runtime.get('lidar_deskew_mode') not in ('off', False):
            raise ValueError('Base5 single-frame 시험은 deskew OFF 계약입니다')
    source_files = dict(graph)
    source_files[str(runtime_config.relative_to(root))] = sha256(runtime_config)
    # Freeze model/server/bridge implementation as well as Python config inheritance.
    for directory in ('perception/detection3d/bevfusion', 'docker/bevfusion/app',
                      'base_ws/src/perception/bevfusion_ros'):
        for extension in ('*.py', '*.launch'):
            for path in sorted((root / directory).rglob(extension)):
                source_files[str(path.relative_to(root))] = sha256(path)
    for relative in ('scripts/runtime/run_bevfusion.sh', 'docker/bevfusion/compose.yaml',
                     'base_ws/src/planning_addons/planning_scenario_validator/scripts/run_bevfusion_trial.py',
                     'base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/models.py'):
        source_files[relative] = sha256(root / relative)
    for relative in ('base_ws/src/perception_object_pipeline/launch/object_pipeline.launch',
                     'base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch'):
        source_files[relative] = sha256(root / relative)
    for directory in ('base_ws/src/perception_object_pipeline/config',
                      'base_ws/src/planning_addons/ajounice_planning_integration/config', 'configs/planning'):
        for path in sorted((root / directory).rglob('*.yaml')):
            source_files[str(path.relative_to(root))] = sha256(path)
    sensor = Path(sensor).resolve()
    result = {'schema_version': 1, 'preset': name, 'label': preset['label'],
        'checkpoint_path': str(checkpoint), 'checkpoint_sha256': preset['sha256'],
        'checkpoint_verified': actual_sha == preset['sha256'],
        'model_config': str(model_config), 'model_config_sha256': sha256(model_config),
        'config_graph_sha256': fingerprint(graph), 'source_files_sha256': source_files,
        'config_files_sha256': graph,
        'runtime_config': str(runtime_config), 'sensor_config': str(sensor), 'sensor_sha256': sha256(sensor),
        'thresholds': dict(zip(('vehicle', 'pedestrian', 'obstacle'), preset['thresholds'])),
        'nms': {'enabled': True, 'class_aware': True, 'rotated_bev_iou_threshold': 0.5},
        'amp': amp, 'parallel_camera': name != 'e25', 'compile_parts': [],
        'temporal': False, 'detector_velocity': False,
        'point_intensity_source': 'ros255' if name != 'e25' else 'production_raw',
        'image_encoding': 'compressed_native_jpeg' if name != 'e25' else 'production_rgb8',
        'base5_geometry': ({'image_final_dim': [512, 1056], 'semantic_roi': [-100, -30, -3, 100, 30, 2],
                           'lidar_xy_voxel_m': 0.05, 'camera_bev_xy_m': 0.20, 'depth_step_m': 0.25,
                           'intensity_normalization': 'server /255 exactly once'} if name != 'e25' else None),
        'server_port': 29500 if name == 'e25' else 29515,
        'cpu_threads': 4 if name != 'e25' else None,
        'image': env.get('BEVFUSION_IMAGE', 'ajounice/bevfusion:team-cu128-v1'),
        'detection_topic': CANONICAL, 'threshold_calibrated': True}
    if name == 'e25':
        result['production_environment'] = {key: value for key, value in env.items() if key.startswith('BEVFUSION_') or key in ('PROJECTION_ONLY', 'INTRINSIC_SOURCE')}
    result['fingerprint'] = fingerprint({key: value for key, value in result.items() if key != 'sensor_config'})
    return result


def verify_contract(contract, stack, sensor, root):
    if contract.get('preset') != 'od_gt' and fingerprint({key: value for key, value in contract.items()
            if key not in ('fingerprint', 'sensor_config', 'image_id')}) != contract.get('fingerprint'):
        raise ValueError('모델 계약 내용/hash가 변조되거나 잘못되었습니다')
    actual = resolve_contract(stack, sensor, root)
    if actual.get('fingerprint') != contract.get('fingerprint'):
        raise ValueError('모델/핵심 설정이 이전 실행과 다릅니다. 같은 결과에 resume할 수 없습니다. 새 실험을 시작하세요')
    return actual


def trial_commands(contract, root, container_name):
    root = Path(root)
    command = ['docker', 'run', '--rm', '--name', container_name, '--gpus', 'all', '--network', 'host', '--shm-size', '4g',
        '-v', '{}:/workspace/AjouNice2026:ro'.format(root),
        '-v', '{}:/opt/ajounice/app:ro'.format(root / 'docker/bevfusion/app'),
        '-v', '{}:/config/active_sensor.json:ro'.format(contract['sensor_config']),
        '-v', '{}:/opt/checkpoints/model.pth:ro'.format(contract['checkpoint_path']),
        '-e', 'PYTHONPATH=/opt/ajounice:/workspace/AjouNice2026:/opt/mmdetection3d',
        '-e', 'TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1', '-e', 'OMP_NUM_THREADS=4', '-e', 'MKL_NUM_THREADS=4', contract.get('image_id', contract['image']),
        'python', '/opt/ajounice/app/inference_server.py', '--bind', '127.0.0.1', '--port', '29515',
        '--sensor-config', '/config/active_sensor.json', '--checkpoint', '/opt/checkpoints/model.pth',
        '--config', '/workspace/AjouNice2026/' + str(Path(contract['model_config']).relative_to(root)),
        '--camera-order', 'camera_front', 'camera_front_left', 'camera_front_right', 'camera_back',
        '--base5-runtime', '--point-intensity-source', 'ros255', '--amp', '--parallel-camera']
    launch = ('base5_e24_calibrated_trial.launch' if contract['preset'] == 'base5_e24'
              else 'base5_e9_calibrated_trial.launch')
    bridge = ['roslaunch', str(root / 'base_ws/src/perception/bevfusion_ros/launch' / launch),
              'sensor_config:=' + contract['sensor_config'], 'config:=' + contract['runtime_config'],
              'output_objects:=' + CANONICAL]
    bridge += ['{}_score_threshold:={}'.format(name, value) for name, value in contract['thresholds'].items()]
    return command, bridge
