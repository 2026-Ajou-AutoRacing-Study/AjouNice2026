#!/usr/bin/env python3
"""Read-only, offline production BEVFusion startup diagnostics.

Default execution never starts ROS, opens a model server, pulls an image,
downloads weights, logs into a service or changes a file/container. Only the
explicit --offline-smoke option creates its own temporary Docker container.
"""
import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

# Importing our local profile must not create a __pycache__ during diagnostics.
sys.dont_write_bytecode = True
from bevfusion_production_profile import profile, resolve_checkpoint
from bevfusion_acceleration_profile import resolve_acceleration

ROOT = Path(__file__).resolve().parents[2]
SAFE_IMAGE = 'ajounice/bevfusion:team-cu128-v1'
CAMERAS = ['camera_front', 'camera_front_left', 'camera_front_right', 'camera_back']
ENV_KEYS = {'BEVFUSION_CHECKPOINT', 'BEVFUSION_INFERENCE_CHECKPOINT', 'BEVFUSION_SENSOR_CONFIG',
            'BEVFUSION_IMAGE', 'BEVFUSION_PRODUCTION_MODEL', 'BEVFUSION_CAMERA_TENSORRT_POLICY',
            'BEVFUSION_CAMERA_TENSORRT_ENGINE', 'BEVFUSION_CAMERA_TENSORRT_METADATA',
            'BEVFUSION_SAFE_MODE', 'BEVFUSION_POOL_FUSED_LAYOUT',
            'BEVFUSION_HEAD_ATTENTION_SDPA', 'BEVFUSION_HEAD_CACHE_BEV_POS',
            'BEVFUSION_ACCELERATION_RECEIPT'}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 << 20), b''): digest.update(block)
    return digest.hexdigest()


def run_readonly(command, environment=None, timeout=12):
    """No shell; callers allow only diagnostic subcommands or explicit smoke."""
    try:
        result = subprocess.run(command, env=environment, capture_output=True, text=True,
                                timeout=timeout, check=False)
        return result.returncode, result.stdout
    except (OSError, subprocess.TimeoutExpired):
        return 127, ''


def local_overrides(path, environment, root, home):
    """Match simple local assignments WITHOUT sourcing arbitrary shell code.

    Credentials and unrelated environment keys are neither returned nor logged.
    Unsupported selected assignments are reported, never executed/guessed.
    """
    values, unsupported = dict(environment), []
    if not path.is_file(): return values, unsupported
    for line in path.read_text().splitlines():
        match = re.match(r'^\s*(?:export\s+)?([A-Za-z_]\w*)=(.*)$', line)
        if line.strip() and not line.lstrip().startswith('#') and not match:
            unsupported.append('dynamic_shell_statement'); continue
        if not match or match.group(1) not in ENV_KEYS: continue
        key, raw = match.groups()
        if any(token in raw for token in ('`', '$(', ';', '&&', '||', '<(', '>(')):
            unsupported.append(key); continue
        try:
            tokens = shlex.split(raw, comments=True)
        except ValueError:
            unsupported.append(key); continue
        if len(tokens) > 1:
            unsupported.append(key); continue
        value = tokens[0] if tokens else ''
        for name, replacement in [('HOME', str(home)), ('ROOT', str(root)), ('AJOUNICE_ROOT', str(root))]:
            value = value.replace('${'+name+'}', replacement).replace('$'+name, replacement)
        if '$' in value:
            unsupported.append(key); continue
        values[key] = value
    return values, unsupported


def flat_yaml(path):
    """Parse only the flat scalar/list subset used by frozen bridge configs."""
    values, list_key = {}, None
    def scalar(value):
        value = value.strip()
        if value.lower() in ('true', 'false'): return value.lower() == 'true'
        if value.startswith('[') and value.endswith(']'):
            return [scalar(item) for item in value[1:-1].split(',') if item.strip()]
        try: return ast.literal_eval(value)
        except (ValueError, SyntaxError): return value
    for raw in path.read_text().splitlines():
        line = raw.split('#', 1)[0].rstrip()
        if not line.strip(): continue
        item = re.match(r'^\s+-\s+(.+)$', line)
        if item and list_key:
            values[list_key].append(scalar(item.group(1))); continue
        match = re.match(r'^([A-Za-z_]\w*):\s*(.*)$', line)
        if not match: raise ValueError('Unsupported bridge YAML structure')
        key, value = match.groups()
        if key in values: raise ValueError('Duplicate bridge config key: '+key)
        values[key] = scalar(value) if value else []
        list_key = key if not value else None
    return values


def config_files(path, seen=None):
    seen = set() if seen is None else seen
    path = path.resolve(strict=True)
    if path in seen: return seen
    if len(seen) >= 64: raise ValueError('Unexpected model config inheritance depth')
    seen.add(path)
    tree = ast.parse(path.read_text(), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == '_base_'
                                               for target in node.targets):
            bases = ast.literal_eval(node.value)
            for value in [bases] if isinstance(bases, str) else bases:
                config_files(path.parent/value, seen)
    return seen


def sensor_contract(path):
    data = json.loads(path.read_text())
    cameras = data.get('cameraList', [])
    order = [sensor.get('cc', {}).get('rosConfig', {}).get('frameID') for sensor in cameras]
    if order != CAMERAS: raise ValueError('Sensor camera order must match the frozen four-camera bridge')
    for sensor in cameras:
        camera = sensor['cc']
        if not (int(camera['cameraResWidth']) > 0 and int(camera['cameraResHeight']) > 0 and
                0 < float(camera['cameraFOV']) < 180 and camera['rosConfig'].get('Topic')):
            raise ValueError('Invalid camera size/FOV/topic')
        for group in ('pos', 'rot'):
            if not all(math.isfinite(float(v)) for v in sensor[group].values()):
                raise ValueError('Nonfinite camera pose')
    lidars = data.get('Lidar3DList', [])
    if len(lidars) != 1: raise ValueError('Exactly one 3D LiDAR required')
    ros = lidars[0]['lc']['rosConfig']
    if not ros.get('frameID') or not ros.get('Topic'): raise ValueError('LiDAR frame/topic missing')
    return dict(camera_order=order, cameras=len(cameras), lidar_frame=ros['frameID'],
                lidar_topic=ros['Topic'], sha256=sha256(path))


def xml_contract(path):
    def node(element):
        return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
                [node(child) for child in element])
    return node(ET.parse(path).getroot())


def inspect_startup(root, model=None, checkpoint=None, sensor=None, image=None, safe=False,
                    environment=None, home=None, runner=run_readonly):
    root = Path(root).resolve(); home = Path.home() if home is None else Path(home)
    base_environment = dict(os.environ if environment is None else environment)
    environment, unsupported = local_overrides(root/'.env.bevfusion.local', base_environment, root, home)
    model = model or environment.get('BEVFUSION_PRODUCTION_MODEL') or 'e12'
    safe = safe or environment.get('BEVFUSION_SAFE_MODE') == 'true'
    acceleration = resolve_acceleration(model, dict(environment, BEVFUSION_SAFE_MODE=str(safe).lower()), home)
    environment.update(acceleration['environment'])
    selected = profile(model, home)
    checkpoint = Path(checkpoint or resolve_checkpoint(model, environment, home)).expanduser().resolve()
    sensor = Path(sensor or environment.get('BEVFUSION_SENSOR_CONFIG') or root/selected['sensor']).expanduser().resolve()
    image = SAFE_IMAGE if safe else image or environment.get('BEVFUSION_IMAGE') or SAFE_IMAGE
    checks = []
    def add(name, level, message, **details):
        checks.append(dict(check=name, level=level, message=message, **details))
    def exists(name, path):
        good = Path(path).is_file()
        add(name, 'pass' if good else 'blocker', 'Found' if good else 'Required local file missing', path=str(path))
        return good
    if unsupported:
        add('local_environment', 'blocker', 'Selected shell assignments cannot be safely inspected; simplify them before startup',
            keys=unsupported)
    else:
        add('local_environment', 'pass', 'Selected literal overrides inspected without executing the shell file')
    local_only, _ = local_overrides(root/'.env.bevfusion.local', {}, root, home)
    if local_only.get('BEVFUSION_CHECKPOINT') and Path(local_only['BEVFUSION_CHECKPOINT']).expanduser().resolve() != checkpoint:
        add('checkpoint_startup_override', 'blocker', 'The local shell file overrides the inspected checkpoint; align that literal assignment before using the startup command')
    add('model_selection', 'pass', 'Production E12 default; E24/E25 are explicit rollback profiles', model=model)
    add('optional_acceleration_setup', 'pass' if acceleration['receipt_loaded'] or safe else 'warning',
        acceleration['reason'], same_selected_model=True)
    if exists('checkpoint', checkpoint):
        try:
            actual = sha256(checkpoint)
            add('checkpoint_sha256', 'pass' if actual == selected['sha256'] else 'blocker',
                'Selected profile checkpoint identity', expected=selected['sha256'], actual=actual)
        except OSError:
            add('checkpoint_sha256', 'blocker', 'Selected checkpoint is not readable')
    config = root/selected['model_config']
    if exists('model_config', config):
        try:
            files = config_files(config)
            add('config_inheritance', 'pass', 'All local model config parents exist; no model import/download',
                files=len(files), selected_sha256=sha256(config))
        except (OSError, ValueError, SyntaxError) as error:
            add('config_inheritance', 'blocker', str(error))
    bridge = root/selected['bridge_config']
    if exists('bridge_config', bridge):
        try:
            values = flat_yaml(bridge)
            expected = dict(intrinsic_source='horizontal_fov', camera_order=CAMERAS,
                            point_feature_contract='xyzi', server_host='127.0.0.1', server_port=29500,
                            latest_frame_only=True, drop_incomplete_frame=True)
            if selected['base5']:
                expected.update(point_intensity_source='ros255', image_encoding='compressed',
                                image_final_height=512, image_final_width=1056)
            mismatches = {key: dict(expected=value, actual=values.get(key)) for key, value in expected.items()
                          if values.get(key) != value}
            add('bridge_contract', 'blocker' if mismatches else 'pass',
                'Frozen intrinsic/camera order/intensity/transport contract', mismatches=mismatches)
        except (OSError, ValueError) as error:
            add('bridge_contract', 'blocker', str(error))
    if exists('sensor_config', sensor):
        try: add('sensor_contract', 'pass', 'Four named cameras and one LiDAR; actual simulator setup is not observable offline',
                 **sensor_contract(sensor))
        except (OSError, ValueError, KeyError, TypeError) as error:
            add('sensor_contract', 'blocker', str(error))
    launch_source = root/'base_ws/src/perception/bevfusion_ros/launch'/selected['launch']
    launch_installed = root/'base_ws/install_aw149/share/bevfusion_ros/launch'/selected['launch']
    source_ok, installed_ok = exists('source_launch', launch_source), exists('installed_launch', launch_installed)
    if source_ok and installed_ok:
        try:
            same = xml_contract(launch_source) == xml_contract(launch_installed)
            add('launch_install_contract', 'pass' if same else 'blocker',
                'Selected installed launch matches source semantically' if same else 'Rebuild/install the selected ROS package; launch semantics differ')
            launch = ET.parse(launch_source).getroot()
            args = {item.get('name'): item.get('value', item.get('default'))
                    for item in (launch.findall('./include/arg') if selected['base5'] else launch.findall('./arg'))}
            names = ('vehicle_score_threshold', 'pedestrian_score_threshold', 'obstacle_score_threshold')
            thresholds = [float(args[name]) for name in names]
            add('launch_thresholds', 'pass' if thresholds == selected['thresholds'] else 'blocker',
                'Selected profile class thresholds', expected=selected['thresholds'], actual=thresholds)
        except ET.ParseError as error: add('launch_install_contract', 'blocker', str(error))
        except (ValueError, KeyError, TypeError):
            add('launch_thresholds', 'blocker', 'Selected launch does not expose the expected frozen class thresholds')
    for name, path in [
        ('noetic_setup', Path('/opt/ros/noetic/setup.bash')),
        ('workspace_source_helper', root/'scripts/workspace/source_aw149.sh'),
        ('planning_underlay', root/'planning_ws/install_isolated/setup.bash'),
        ('base_overlay', root/'base_ws/install_aw149/setup.bash'),
        ('base_local_overlay', root/'base_ws/install_aw149/local_setup.bash'),
        ('installed_bridge', root/'base_ws/install_aw149/lib/bevfusion_ros/bevfusion_ros_bridge.py'),
        ('compose_file', root/'docker/bevfusion/compose.yaml'),
        ('startup_script', root/'scripts/runtime/run_bevfusion.sh')]: exists(name, path)
    source_bridge = root/'base_ws/src/perception/bevfusion_ros/scripts/bevfusion_ros_bridge.py'
    installed_bridge = root/'base_ws/install_aw149/lib/bevfusion_ros/bevfusion_ros_bridge.py'
    if source_bridge.is_file() and installed_bridge.is_file():
        try:
            same = ast.dump(ast.parse(source_bridge.read_text()), include_attributes=False) == ast.dump(
                ast.parse(installed_bridge.read_text()), include_attributes=False)
            add('bridge_install_contract', 'pass' if same else 'blocker',
                'Installed ROS bridge matches current source AST' if same else 'Installed ROS bridge differs; rebuild/install before startup')
        except (OSError, SyntaxError): add('bridge_install_contract', 'blocker', 'ROS bridge source/install cannot be parsed')
    add('ros_live_state', 'warning', 'ROS master, live topics, publishers, MORAI network and sensor rates were not started or certified')
    env = dict(base_environment, **environment)
    env.update(AJOUNICE_ROOT=str(root), BEVFUSION_CHECKPOINT=str(checkpoint),
               BEVFUSION_SENSOR_CONFIG=str(sensor), BEVFUSION_IMAGE=image)
    compose = root/'docker/bevfusion/compose.yaml'
    image_id, cache_volume = None, None
    for name, command in [('docker_cli_daemon', ['docker', 'version', '--format', '{{.Server.Version}}']),
                          ('docker_compose', ['docker', 'compose', 'version', '--short'])]:
        code, stdout = runner(command, environment=env)
        add(name, 'pass' if code == 0 and stdout.strip() else 'blocker',
            'Available offline' if code == 0 and stdout.strip() else 'Docker CLI/daemon or compose unavailable; no installation/pull attempted')
    code, stdout = runner(['docker', 'image', 'inspect', '--format', '{{.Id}}', image], environment=env)
    if code == 0 and re.fullmatch(r'sha256:[a-f0-9]{64}', stdout.strip()):
        image_id = stdout.strip(); add('local_image', 'pass', 'Selected image is already local', image=image, image_id=image_id)
    elif acceleration['managed_image'] and not safe and environment.get('BEVFUSION_CAMERA_TENSORRT_POLICY') != 'require':
        code, stdout = runner(['docker', 'image', 'inspect', '--format', '{{.Id}}', SAFE_IMAGE], environment=env)
        if code == 0 and re.fullmatch(r'sha256:[a-f0-9]{64}', stdout.strip()):
            image = SAFE_IMAGE; image_id = stdout.strip(); env['BEVFUSION_IMAGE'] = image
            environment.update(BEVFUSION_CAMERA_TENSORRT_POLICY='off', BEVFUSION_CAMERA_TENSORRT_ENGINE='',
                               BEVFUSION_CAMERA_TENSORRT_METADATA='')
            add('local_image', 'warning', 'Optional TRT image missing; the SAME model uses the installed baseline image', image=image)
        else:
            add('local_image', 'blocker', 'Neither prepared optional nor baseline image is locally available', image=image)
    else: add('local_image', 'blocker', 'Selected image is not locally available; no registry/pull/login attempted', image=image)
    if compose.is_file():
        code, stdout = runner(['docker', 'compose', '-f', str(compose), 'config', '--format', 'json'], environment=env)
        try:
            resolved = json.loads(stdout) if code == 0 else {}
            service = resolved['services']['inference']
            if service.get('image') != image: raise ValueError('Resolved compose image differs')
            add('compose_resolution', 'pass', 'Compose resolves offline; credentials/full environment are not printed')
            for volume in service.get('volumes', []):
                if volume.get('type') == 'volume' and volume.get('target') == '/root/.cache/torch_extensions':
                    declared = resolved.get('volumes', {}).get(volume.get('source'), {})
                    name = declared.get('name')
                    if name and runner(['docker', 'volume', 'inspect', '--format', '{{.Name}}', name], environment=env)[0] == 0:
                        cache_volume = name
        except (KeyError, TypeError, ValueError):
            add('compose_resolution', 'blocker', 'Compose configuration cannot resolve offline; inspect its paths/variables')
    code, stdout = runner(['docker', 'ps', '--format', '{{.Names}}\t{{.Image}}'], environment=env)
    if code == 0:
        running = [line.split('\t', 1)[0] for line in stdout.splitlines()
                   if 'inference' in line.split('\t', 1)[0]]
        add('existing_services', 'warning' if running else 'pass',
            'Existing inference containers were only inspected; do not start a competing detector' if running else 'No inference-named container observed',
            names=running)
    code, stdout = runner(['nvidia-smi', '--query-gpu=name,memory.total,driver_version', '--format=csv,noheader,nounits'], environment=env)
    gpus = []
    if code == 0:
        for line in stdout.splitlines():
            parts = [value.strip() for value in line.split(',')]
            if len(parts) == 3:
                try: gpus.append(dict(name=parts[0], memory_mib=int(parts[1]), driver=parts[2]))
                except ValueError: pass
    add('nvidia_gpu_driver', 'pass' if gpus else 'blocker',
        'Visible NVIDIA GPU/driver; CUDA ABI/model warmup requires optional smoke' if gpus else 'No readable NVIDIA GPU/driver; CPU fallback is not a qualified BEVFusion runtime', gpus=gpus)
    if gpus and ('4090' not in gpus[0]['name'] or gpus[0]['memory_mib'] < 23000):
        add('gpu_target', 'warning', 'Different GPU/capacity is not an automatic failure of eager fallback; this is not the 4090 24GB full-stack qualification')
    add('host_python', 'pass', 'Host Python only runs diagnostics/ROS glue; model Python lives inside the selected image', version=sys.version.split()[0])
    policy = 'off' if safe else environment.get('BEVFUSION_CAMERA_TENSORRT_POLICY') or (
        'prefer' if (environment.get('BEVFUSION_CAMERA_TENSORRT_ENGINE') or
                     environment.get('BEVFUSION_CAMERA_TENSORRT_METADATA')) else 'off')
    if policy not in ('off', 'prefer', 'require'):
        add('acceleration_policy', 'blocker', 'Unknown camera TensorRT policy; use off/prefer/require')
    elif policy != 'off':
        problem = None
        try:
            paths = []
            for key in ('BEVFUSION_CAMERA_TENSORRT_ENGINE', 'BEVFUSION_CAMERA_TENSORRT_METADATA'):
                raw = environment.get(key)
                if not raw: raise ValueError('Engine and metadata must both be configured')
                path = Path(raw).expanduser()
                if str(path).startswith('/workspace/AjouNice2026/'):
                    path = root/path.relative_to('/workspace/AjouNice2026')
                if str(path).startswith('/opt/bevfusion_tensorrt/'):
                    path = Path(acceleration['artifact_root']) / path.relative_to('/opt/bevfusion_tensorrt')
                elif not path.is_absolute(): path = root/path
                if not path.is_file(): raise ValueError('Configured engine/metadata is not locally available')
                paths.append(path)
            metadata = json.loads(paths[1].read_text())
            identity = metadata['contract']
            if (identity.get('checkpoint_sha256') != selected['sha256'] or
                    identity.get('config_sha256') != sha256(config)):
                raise ValueError('Engine metadata does not match the selected production checkpoint/config')
            if metadata['engine'].get('sha256') != sha256(paths[0]):
                raise ValueError('Engine binary SHA does not match metadata')
            if gpus and metadata['engine'].get('gpu_name') != gpus[0]['name']:
                raise ValueError('Engine target GPU differs from default CUDA GPU')
        except (OSError, ValueError, KeyError, TypeError) as error:
            problem = str(error)
        add('acceleration_static_contract', 'blocker' if problem and policy == 'require' else 'warning' if problem else 'pass',
            problem or 'Local engine identity matches; TensorRT library/runtime eligibility still belongs to startup',
            fallback_to_eager_allowed=policy == 'prefer')
    head_flags = {}
    for key in ('BEVFUSION_HEAD_ATTENTION_SDPA', 'BEVFUSION_HEAD_CACHE_BEV_POS'):
        value = 'false' if safe else environment.get(key) or 'false'
        if value not in ('true', 'false'):
            add('head_optimization_option', 'blocker', key+' must be true or false')
        head_flags[key] = value == 'true'
    if any(head_flags.values()):
        add('head_optimization', 'pass' if model == 'e12' else 'warning',
            'Inference-only head optimization; startup source eligibility may retain the SAME original head. --safe disables both options', flags=head_flags)
    if policy != 'off' or environment.get('BEVFUSION_POOL_FUSED_LAYOUT') == 'true':
        add('acceleration', 'warning', 'Optional engine/layout eligibility is checked separately by runtime; --safe disables both without changing selected weights', policy=policy)
    command = ['env', 'BEVFUSION_CHECKPOINT='+str(checkpoint), 'bash',
               str(root/'scripts/runtime/run_bevfusion.sh'), '--safe', '--model', model, str(sensor)]
    report = dict(schema='bevfusion_competition_preflight_v1', collected_unix=time.time(),
        selected=dict(model=model, checkpoint=str(checkpoint), expected_sha256=selected['sha256'],
                      model_config=str(config), bridge_config=str(bridge), sensor=str(sensor), image=image,
                      camera_tensorrt_policy=policy, base5=selected['base5'], parallel=selected['parallel']),
        checks=checks, blocker_count=sum(row['level'] == 'blocker' for row in checks),
        warning_count=sum(row['level'] == 'warning' for row in checks),
        default_read_only=True, cloud_access_required=False, production_promoted=False,
        safe_start_command=shlex.join(command),
        safe_start_note='Run only after resolving blockers and confirming no other detector owns the canonical topic; this starts ROS/server and is NOT executed by this preflight',
        explicit_e25_rollback_command=shlex.join(['bash', str(root/'scripts/runtime/run_bevfusion.sh'), '--safe', '--model', 'e25']),
        model_smoke_tested=False, full_stack_certified=False, _cache_volume=cache_volume)
    report['ready_for_operator_start'] = report['blocker_count'] == 0
    return report


def smoke_command(report, root, name=None):
    selected = report['selected']; root = Path(root).resolve()
    command = ['docker', 'run', '--rm', '--network', 'none', '--gpus', 'all', '--read-only',
        '--tmpfs', '/tmp:rw,exec,size=2g', '--tmpfs', '/root/.cache:rw,exec,size=2g',
        '-e', 'PYTHONDONTWRITEBYTECODE=1', '-e', 'OMP_NUM_THREADS=4', '-e', 'MKL_NUM_THREADS=4',
        '-e', 'TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1',
        '-e', 'PYTHONPATH=/workspace/AjouNice2026:/opt/ajounice',
        '--mount', 'type=bind,src='+str(root)+',dst=/workspace/AjouNice2026,readonly',
        '--mount', 'type=bind,src='+str(root/'docker/bevfusion/app')+',dst=/opt/ajounice/app,readonly',
        '--mount', 'type=bind,src='+str(root/'docker/bevfusion/scripts')+',dst=/opt/ajounice/scripts,readonly',
        '--mount', 'type=bind,src='+selected['checkpoint']+',dst=/opt/checkpoints/selected.pth,readonly',
        '--mount', 'type=bind,src='+selected['sensor']+',dst=/config/Sensor.json,readonly']
    if name:
        command.extend(['--name', name, '--label', 'ajounice.preflight.owner='+name])
    if report.get('_cache_volume'):
        command.extend(['--mount', 'type=volume,src='+report['_cache_volume']+',dst=/root/.cache/torch_extensions'])
    command.extend(['--entrypoint', 'python', selected['image'],
        '/workspace/AjouNice2026/docker/bevfusion/scripts/verify_competition_startup.py',
        '--model', selected['model'], '--checkpoint', '/opt/checkpoints/selected.pth',
        '--sensor-config', '/config/Sensor.json', '--root', '/workspace/AjouNice2026'])
    return command


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', choices=['e12', 'e24', 'e25'])
    p.add_argument('--root', type=Path, default=ROOT)
    p.add_argument('--checkpoint', type=Path)
    p.add_argument('--sensor-config', type=Path)
    p.add_argument('--image')
    p.add_argument('--safe', action='store_true', help='Inspect the pinned original eager image, not optional accelerators')
    p.add_argument('--offline-smoke', action='store_true', help='Explicitly run only our temporary network-disabled GPU model warmup container')
    p.add_argument('--receipt', type=Path, help='Explicit new JSON receipt; never overwrite')
    a = p.parse_args()
    if a.receipt and a.receipt.exists(): p.error('Receipt already exists; never overwrite earlier evidence')
    report = inspect_startup(a.root, a.model, a.checkpoint, a.sensor_config, a.image, a.safe)
    report['offline_smoke_command'] = shlex.join(smoke_command(report, a.root))
    if a.offline_smoke:
        if report['blocker_count']:
            report['checks'].append(dict(check='offline_smoke', level='warning', message='Skipped because static blockers remain'))
        else:
            started = time.perf_counter()
            name = 'bevfusion-preflight-'+uuid.uuid4().hex[:12]
            try:
                code, output = run_readonly(smoke_command(report, a.root, name=name), timeout=300)
            finally:
                # Timeout can kill the Docker CLIENT while its container keeps
                # running. Remove only the unique container bearing OUR label.
                check, label = run_readonly(['docker', 'container', 'inspect', '--format',
                    '{{index .Config.Labels "ajounice.preflight.owner"}}', name])
                if check == 0 and label.strip() == name:
                    run_readonly(['docker', 'rm', '--force', name])
            lines = [line for line in output.splitlines() if line.startswith('COMPETITION_STARTUP_JSON=')]
            try: result = json.loads(lines[-1].split('=', 1)[1]) if lines else {}
            except ValueError: result = {}
            good = code == 0 and result.get('model_warmed') and result.get('closed')
            report['model_smoke_tested'] = bool(good)
            report['offline_smoke'] = dict(exit_code=code, elapsed_sec=time.perf_counter()-started, result=result)
            report['checks'].append(dict(check='offline_smoke', level='pass' if good else 'blocker',
                message='Our temporary container warmed and closed the selected eager model' if good else 'Model warmup failed/timed out; no existing container was stopped'))
    report.pop('_cache_volume', None)
    report['blocker_count'] = sum(row['level'] == 'blocker' for row in report['checks'])
    report['warning_count'] = sum(row['level'] == 'warning' for row in report['checks'])
    report['ready_for_operator_start'] = report['blocker_count'] == 0
    if a.receipt:
        with a.receipt.open('x') as stream: json.dump(report, stream, indent=2); stream.write('\n')
    print(json.dumps(report, indent=2))
    return 1 if report['blocker_count'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
