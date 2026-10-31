#!/usr/bin/env python3
"""Build the frozen E12 camera engine once on this PC, before driving.

Default: read local inputs and print a plan, without Docker/GPU execution or
file creation. --execute requires the local TRT image (never pulls/logs in),
builds in an isolated --rm/network-none container, warms the complete selected
model without TCP/ROS, then publishes runtime.json last. A failed optional TRT
setup leaves any existing receipt untouched; use the same E12 eagerly/--safe.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import uuid

sys.dont_write_bytecode = True
SCHEMA = 'bevfusion_production_tensorrt_setup_v1'
IMAGE = 'ajounice/bevfusion:team-trt-cu128-261023-v1'
BASE_IMAGE = 'ajounice/bevfusion:team-cu128-v1'
E12_SHA = 'f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8'
CONFIG = 'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_nearest_depth_runtime.py'
CONFIG_SHA = 'b963b78565699b62e3570081c75330db7cf1e27dba5d49fbcb2b71d27545d978'
EXPORTER = 'perception/detection3d/bevfusion/scripts/export_base5_camera_tensorrt.py'
SELF = 'docker/bevfusion/scripts/setup_production_tensorrt.py'
CONTAINER_ROOT = Path('/workspace/AjouNice2026')
CONTAINER_ARTIFACTS = Path('/opt/bevfusion_tensorrt')
CONTAINER_CHECKPOINT = Path('/opt/checkpoints/base5_e12_inference.pth')
HEAD = dict(proposal_topk=False, attention_sdpa=True, cache_bev_pos=True)
SOURCE_FILES = (SELF, EXPORTER, CONFIG,
    'docker/bevfusion/app/camera_tensorrt.py',
    'docker/bevfusion/app/bevfusion_model.py',
    'docker/bevfusion/app/head_runtime_optimization.py',
    'docker/bevfusion/app/inference_optimization.py')


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b''):
            result.update(block)
    return result.hexdigest()


def source_pins(root):
    return {name: digest(root/name) for name in SOURCE_FILES}


def validate_inputs(root, checkpoint, sensor):
    for path in (root/CONFIG, root/EXPORTER, checkpoint, sensor):
        if not path.is_file():
            raise ValueError('Required local input is missing: ' + str(path))
    if digest(checkpoint) != E12_SHA:
        raise ValueError('Expected the frozen full E12 checkpoint; E24/E25 are not substituted')
    if digest(root/CONFIG) != CONFIG_SHA:
        raise ValueError('Frozen E12 runtime config SHA differs')
    # Detailed calibration/geometry validation runs in the complete-model smoke.
    data = json.loads(sensor.read_text())
    frames = [item.get('cc', {}).get('rosConfig', {}).get('frameID')
              for item in data.get('cameraList', [])]
    if set(frames) != set(camera_order()) or len(frames) != 4 or len(data.get('Lidar3DList', [])) != 1:
        raise ValueError('Sensor must contain the four production cameras and one LiDAR')
    return source_pins(root)


def camera_order():
    return ['camera_front', 'camera_front_left', 'camera_front_right', 'camera_back']


def _checked(command, timeout=60):
    result = subprocess.run(command, check=True, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=timeout)
    return result.stdout.strip()


def inspect_image(image):
    # Explicitly local; image inspect does not contact a registry or pull.
    value = json.loads(_checked(['docker', 'image', 'inspect', image]))
    if len(value) != 1 or not value[0].get('Id', '').startswith('sha256:'):
        raise ValueError('A unique installed image ID is required')
    return value[0]['Id']


def host_gpu(index):
    line = _checked(['nvidia-smi', '-i', str(index),
        '--query-gpu=name,uuid,compute_cap', '--format=csv,noheader,nounits'])
    fields = [item.strip() for item in line.split(',')]
    if len(fields) != 3 or '\n' in line:
        raise ValueError('Cannot identify the selected target GPU')
    capability = [int(item) for item in fields[2].split('.')]
    if len(capability) != 2:
        raise ValueError('Invalid target compute capability')
    return dict(name=fields[0], uuid=fields[1], compute_capability=capability, host_index=index)


def _under(path, parent):
    try:
        return Path(path).resolve().relative_to(Path(parent).resolve())
    except ValueError as error:
        raise ValueError('Artifact escapes the configured artifact root') from error


def container_path(path, artifact_root):
    return str(CONTAINER_ARTIFACTS/_under(path, artifact_root))


def export_command(output):
    return ['python', str(CONTAINER_ROOT/EXPORTER), '--root', str(CONTAINER_ROOT),
        '--config', str(CONTAINER_ROOT/CONFIG), '--checkpoint', str(CONTAINER_CHECKPOINT),
        '--output-dir', str(output), '--device', 'cuda:0', '--onnx-dtype', 'float32',
        '--precision', 'fp16', '--output-dtype', 'float16',
        '--fp32-layer-types', 'NORMALIZATION', 'SOFTMAX', '--build-engine']


def docker_command(args, output, image_id):
    root = args.root
    command = ['docker', 'run', '--pull=never', '--rm', '--network=none', '--read-only',
        '--name', 'bevfusion-trt-setup-'+output.name,
        '--label', 'ai.ajounice.setup.owner='+output.name,
        '--gpus', 'device='+str(args.gpu), '--shm-size=4g',
        '--tmpfs', '/tmp:rw,exec,size=4g', '--tmpfs', '/root/.cache:rw,exec,size=2g',
        '--env', 'PYTHONDONTWRITEBYTECODE=1', '--env', 'PYTHONUNBUFFERED=1',
        '--env', 'PYTHONPATH='+str(CONTAINER_ROOT)+':'+str(CONTAINER_ROOT/'docker/bevfusion')+':/opt/mmdetection3d',
        '--mount', 'type=bind,src='+str(root)+',dst='+str(CONTAINER_ROOT)+',readonly',
        '--mount', 'type=bind,src='+str(args.checkpoint)+',dst='+str(CONTAINER_CHECKPOINT)+',readonly',
        '--mount', 'type=bind,src='+str(args.sensor_config)+',dst=/config/Sensor.json,readonly',
        '--mount', 'type=bind,src='+str(args.artifact_root)+',dst='+str(CONTAINER_ARTIFACTS),
        '--workdir', str(CONTAINER_ROOT), '--entrypoint', 'python', image_id,
        str(CONTAINER_ROOT/SELF), '--container-build', '--output-dir',
        container_path(output, args.artifact_root)]
    return command


def cleanup_owned_container(output):
    """A killed Docker client can leave its container running. Touch only ours."""
    name = 'bevfusion-trt-setup-'+output.name
    try:
        labels = json.loads(_checked(['docker', 'inspect', '--format',
            '{{json .Config.Labels}}', name]))
        if labels.get('ai.ajounice.setup.owner') != output.name:
            return
        _checked(['docker', 'rm', '--force', name], timeout=30)
    except (OSError, ValueError, AttributeError, RuntimeError, subprocess.SubprocessError):
        # Already removed is normal with --rm. Never stop other services.
        pass


def validate_artifacts(directory, gpu):
    engine, meta = directory/'camera.engine', directory/'camera.engine.json'
    smoke = json.loads((directory/'SETUP_SMOKE.json').read_text())
    metadata = json.loads(meta.read_text())
    contract, built = metadata.get('contract', {}), metadata.get('engine', {})
    expected_outputs = [dict(name='camera_feature_0', shape=[4, 256, 64, 132], dtype='float16'),
                        dict(name='camera_feature_1', shape=[4, 256, 32, 66], dtype='float16')]
    if (metadata.get('schema') != 'base5_camera_tensorrt_v1' or metadata.get('stage') != 'engine'
            or contract.get('checkpoint_sha256') != E12_SHA or contract.get('config_sha256') != CONFIG_SHA
            or contract.get('input_shape') != [4, 3, 512, 1056] or contract.get('input_dtype') != 'float32'
            or contract.get('input_name') != 'images' or contract.get('input_semantics') != 'normalized_flattened_BNCHW'
            or contract.get('output_cast_policy') != 'float16_after_neck'
            or contract.get('output_kind') != 'tuple' or contract.get('outputs') != expected_outputs
            or built.get('precision') != 'fp16' or built.get('tf32') is not False
            or sorted(built.get('fp32_layer_types', [])) != ['NORMALIZATION', 'SOFTMAX']
            or built.get('tensorrt_version') != '10.13.3.9'
            or built.get('torch_version', '').split('+')[0] != '2.7.1' or built.get('torch_cuda') != '12.8'
            or built.get('hardware_compatibility') != 'none'):
        raise ValueError('Built engine does not match the approved E12 v4 camera boundary')
    if digest(engine) != built.get('sha256'):
        raise ValueError('Built engine SHA differs from metadata')
    for evidence in (built, smoke.get('gpu', {})):
        if (evidence.get('gpu_name', evidence.get('name')) != gpu['name']
                or evidence.get('compute_capability') != gpu['compute_capability']):
            raise ValueError('Built/smoked engine GPU differs from this PC')
    if (smoke.get('schema') != 'bevfusion_production_tensorrt_smoke_v1'
            or smoke.get('checkpoint_sha256') != E12_SHA or smoke.get('config_sha256') != CONFIG_SHA
            or not all(smoke.get(item) is True for item in ('tiny_cuda', 'model_warmed', 'closed'))
            or smoke.get('head_runtime') != HEAD or smoke.get('camera_backend') != 'tensorrt'):
        raise ValueError('Complete E12 TRT/head model warmup did not pass')
    return dict(engine_path=str(engine.resolve()), metadata_path=str(meta.resolve()),
                engine_sha256=digest(engine), metadata_sha256=digest(meta), smoke=smoke)


def existing_receipt(path, artifact_root, image_id, gpu, pins):
    data = json.loads(path.read_text())
    if (data.get('schema') != SCHEMA or data.get('complete') is not True or data.get('model') != 'e12'
            or data.get('checkpoint_sha256') != E12_SHA or data.get('config_sha256') != CONFIG_SHA
            or data.get('image_id') != image_id or data.get('gpu') != gpu
            or data.get('head_runtime') != HEAD or data.get('source_sha256') != pins):
        raise ValueError('Existing setup differs; inspect it or explicitly use --rebuild')
    engine, meta = Path(data['engine_path']), Path(data['metadata_path'])
    if engine.parent != meta.parent or meta.name != 'camera.engine.json' or engine.name != 'camera.engine':
        raise ValueError('Existing setup artifact filenames differ')
    _under(engine, artifact_root); _under(meta, artifact_root)
    actual = validate_artifacts(engine.parent, gpu)
    for key in ('engine_path', 'metadata_path', 'engine_sha256', 'metadata_sha256'):
        if data.get(key) != actual[key]:
            raise ValueError('Existing setup artifact identity differs: '+key)
    if (data.get('container_engine_path') != container_path(engine, artifact_root)
            or data.get('container_metadata_path') != container_path(meta, artifact_root)):
        raise ValueError('Existing setup container paths differ')
    return data


def publish_receipt(path, payload):
    # --execute owns this directory. Preserve the previous successful receipt;
    # failed builds never reach this function and cannot invalidate that setup.
    temporary = path.with_name('.runtime-'+uuid.uuid4().hex+'.tmp')
    with temporary.open('x') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True); stream.write('\n')
        stream.flush(); os.fsync(stream.fileno())
    try:
        if path.exists():
            backup = path.with_name('runtime.previous-'+uuid.uuid4().hex+'.json')
            with backup.open('xb') as stream:
                stream.write(path.read_bytes())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def container_build(output):
    # Only the explicit host --execute Docker command enters this branch. No
    # TCP server, ROS publisher or credentials are needed in this container.
    import cv2
    import torch
    import tensorrt
    import onnx
    if (str(torch.__version__).split('+')[0] != '2.7.1' or torch.version.cuda != '12.8'
            or tensorrt.__version__ != '10.13.3.9' or onnx.__version__ != '1.16.0'):
        raise ValueError('Installed image does not have the pinned production toolchain')
    subprocess.run(export_command(output), check=True)
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    report = dict(schema='bevfusion_production_tensorrt_smoke_v1',
        checkpoint_sha256=E12_SHA, config_sha256=CONFIG_SHA,
        tiny_cuda=False, model_warmed=False, closed=False,
        gpu=dict(name=torch.cuda.get_device_name(0),
                 compute_capability=list(torch.cuda.get_device_capability(0))))
    runtime = None
    try:
        torch.set_num_threads(4); cv2.setNumThreads(1)
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        torch.backends.cudnn.benchmark = True
        with torch.inference_mode():
            if torch.ones(256, device='cuda:0').sum().item() != 256:
                raise RuntimeError('Tiny CUDA arithmetic failed')
            report['tiny_cuda'] = True
        # Match normal server initialization OUTSIDE inference_mode: creating
        # bev_pos inside inference_mode removes its tensor version counter,
        # which the exact position-cache invalidation contract requires.
        runtime = BevFusionModel(str(CONTAINER_ROOT/CONFIG), str(CONTAINER_CHECKPOINT),
            load_sensor4_calibration('/config/Sensor.json', camera_order(), 'horizontal_fov'),
            device='cuda:0', use_amp=True, preprocess_workers=4, warmup_iterations=2,
            base5_runtime=True, parallel_camera=True, point_intensity_source='ros255',
            camera_tensorrt_engine=str(output/'camera.engine'),
            camera_tensorrt_metadata=str(output/'camera.engine.json'),
            head_attention_sdpa=True, head_cache_bev_pos=True)
        if not runtime.warmed_up or runtime.camera_tensorrt is None:
            raise RuntimeError('TRT complete-model warmup did not finish')
        head = runtime.head_runtime_report or {}
        if not head.get('enabled') or head.get('active_flags') != HEAD:
            raise RuntimeError('Requested SDPA/position cache was not installed')
        report.update(model_warmed=True, camera_backend='tensorrt', head_runtime=dict(HEAD),
                      sensor_sha256=digest('/config/Sensor.json'),
                      execution_status=runtime.execution_status())
    finally:
        if runtime is not None:
            runtime.close(); report['closed'] = True
        with (output/'SETUP_SMOKE.json').open('x') as stream:
            json.dump(report, stream, indent=2, sort_keys=True); stream.write('\n')
    print(json.dumps(dict(event='e12_trt_setup_smoke_pass', output=str(output))))


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[3])
    p.add_argument('--checkpoint', type=Path, default=Path.home()/'bevfusion_artifacts/checkpoints/base5_e12_inference.pth')
    p.add_argument('--sensor-config', type=Path)
    p.add_argument('--artifact-root', type=Path, default=Path.home()/'bevfusion_artifacts/tensorrt')
    p.add_argument('--image', default=IMAGE)
    p.add_argument('--gpu', type=int, default=0, help='Host GPU index; exposed as cuda:0 during setup')
    p.add_argument('--execute', action='store_true', help='Explicitly build/warm on this GPU and publish receipt')
    p.add_argument('--rebuild', action='store_true', help='Keep old artifacts; explicitly build another engine')
    p.add_argument('--timeout', type=int, default=7200)
    p.add_argument('--container-build', action='store_true', help=argparse.SUPPRESS)
    p.add_argument('--output-dir', type=Path, help=argparse.SUPPRESS)
    return p


def main(argv=None):
    a = parser().parse_args(argv)
    if a.container_build:
        if not a.output_dir or not a.output_dir.is_absolute() or not Path('/.dockerenv').is_file():
            raise ValueError('Internal container worker requires an explicit mounted output directory')
        _under(a.output_dir, CONTAINER_ARTIFACTS/'e12')
        container_build(a.output_dir); return 0
    a.root = a.root.expanduser().resolve(strict=True)
    a.checkpoint = a.checkpoint.expanduser().resolve(strict=True)
    a.sensor_config = (a.sensor_config or a.root/'configs/sensors/Sensor_competition_lan.json').expanduser().resolve(strict=True)
    a.artifact_root = a.artifact_root.expanduser().resolve()
    if a.gpu < 0 or a.timeout < 60 or not a.image or a.image.startswith('-'):
        raise ValueError('Invalid GPU, timeout or installed image name')
    if any(',' in str(path) or '\n' in str(path) for path in
           (a.root, a.checkpoint, a.sensor_config, a.artifact_root)):
        raise ValueError('Docker bind paths may not contain comma/newline')
    pins = validate_inputs(a.root, a.checkpoint, a.sensor_config)
    sensor_sha = digest(a.sensor_config)
    receipt = a.artifact_root/'e12/runtime.json'
    token = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
    output = receipt.parent/('build-'+token)
    plan = dict(schema=SCHEMA, event='setup_plan', execute=a.execute, model='e12',
        image=a.image, checkpoint_sha256=E12_SHA, config_sha256=CONFIG_SHA, sensor_sha256=sensor_sha,
        artifact_root=str(a.artifact_root), receipt_path=str(receipt),
        engine_path=str(output/'camera.engine'), metadata_path=str(output/'camera.engine.json'),
        container_engine_path=container_path(output/'camera.engine', a.artifact_root),
        container_metadata_path=container_path(output/'camera.engine.json', a.artifact_root),
        head_runtime=HEAD, source_sha256=pins, gpu_index=a.gpu,
        build_image_command=shlex.join(['docker', 'build', '--pull=false', '-f',
            'docker/bevfusion/Dockerfile.production-tensorrt', '-t', a.image, '.']),
        setup_command=shlex.join(docker_command(a, output, a.image)),
        note='No engine is built during driving. Missing optional TRT uses the SAME E12 eager/--safe; no E24 substitution.')
    if not a.execute:
        print(json.dumps(plan, indent=2, sort_keys=True)); return 0
    image_id = inspect_image(a.image)
    gpu = host_gpu(a.gpu)
    if receipt.exists() and not a.rebuild:
        value = existing_receipt(receipt, a.artifact_root, image_id, gpu, pins)
        print(json.dumps(dict(event='existing_setup_ready', receipt=str(receipt), runtime=value), sort_keys=True))
        return 0
    receipt.parent.mkdir(parents=True, exist_ok=True)
    # Exporter itself creates output, refusing to overwrite any prior attempt.
    try:
        subprocess.run(docker_command(a, output, image_id), check=True, timeout=a.timeout)
    except BaseException:
        cleanup_owned_container(output)
        raise
    artifacts = validate_artifacts(output, gpu)
    if (source_pins(a.root) != pins or digest(a.checkpoint) != E12_SHA
            or digest(a.sensor_config) != sensor_sha or artifacts['smoke'].get('sensor_sha256') != sensor_sha
            or inspect_image(a.image) != image_id):
        raise RuntimeError('Source/checkpoint/image changed during setup; receipt was not published')
    payload = dict(schema=SCHEMA, complete=True, model='e12', created_utc=datetime.now(timezone.utc).isoformat(),
        image=a.image, image_id=image_id, checkpoint_sha256=E12_SHA, config_sha256=CONFIG_SHA,
        sensor_sha256=sensor_sha,
        gpu=gpu, head_runtime=HEAD, source_sha256=pins,
        engine_path=artifacts['engine_path'], metadata_path=artifacts['metadata_path'],
        engine_sha256=artifacts['engine_sha256'], metadata_sha256=artifacts['metadata_sha256'],
        container_engine_path=container_path(output/'camera.engine', a.artifact_root),
        container_metadata_path=container_path(output/'camera.engine.json', a.artifact_root),
        smoke_path=str(output/'SETUP_SMOKE.json'), accuracy_evidence='E12 frozen v4 camera + SDPA/position cache; target-PC smoke is not a new AP/full-stack certificate')
    publish_receipt(receipt, payload)
    print(json.dumps(dict(event='production_tensorrt_setup_ready', receipt_path=str(receipt), runtime=payload), sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='production_tensorrt_setup_failed', error_type=type(error).__name__,
            error=str(error), eager_fallback='Use the same selected E12 with bevfusion --safe; do not bypass missing/corrupt checkpoint or sensor errors.')), file=sys.stderr)
        raise SystemExit(1)
