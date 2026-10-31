#!/usr/bin/env python3
"""Offline-capable checks and explicitly gated runtime for a native Base5 bundle.

This module never installs packages, downloads files, starts a ROS master, or
creates operator approval. Merely checking or printing a plan does not create
files. Native execution stays disabled until the bundle and operator explicitly
enable it; qualification on the source PC is not a portable live qualification.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import subprocess
import sys


SCHEMA = 'base5_portable_native_bundle_v1'
APPROVAL_SCHEMA = 'base5_portable_operator_approval_v1'
MANIFEST = 'portable_bundle.json'
MR_RELATIVE = Path('MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile')
SCENARIO_RELATIVE = MR_RELATIVE / 'Scenario/R_KR_PR_K-city_2025'
SENSOR_RELATIVE = MR_RELATIVE / 'SensorData'


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def inside(root, relative):
    """Only safe bundle-relative paths; reject even in-bundle symlink aliases."""
    if not isinstance(relative, str):
        raise ValueError('Bundle path must be text')
    path = PurePosixPath(relative)
    if (not relative or path.is_absolute() or '..' in path.parts
            or '\\' in relative or relative != path.as_posix() or relative == '.'):
        raise ValueError('Unsafe bundle path: ' + relative)
    target = Path(root) / relative
    for part in (target, *target.parents):
        if part == Path(root):
            break
        if part.is_symlink():
            raise ValueError('Symlink in immutable bundle input: ' + relative)
    target.resolve().relative_to(Path(root).resolve())
    return target


def load_bundle(root):
    root = Path(root).resolve(strict=True)
    path = inside(root, MANIFEST)
    doc = json.loads(path.read_text())
    if doc.get('schema') != SCHEMA or not isinstance(doc.get('files'), dict) or not doc['files']:
        raise ValueError('Nonempty native bundle inventory required')
    for relative, item in doc['files'].items():
        source = inside(root, relative)
        expected = item['sha256'] if isinstance(item, dict) else item
        if (not source.is_file() or not isinstance(expected, str)
                or len(expected) != 64 or sha256(source) != expected):
            raise ValueError('Bundle file missing or changed: ' + relative)
        if isinstance(item, dict) and 'bytes' in item and source.stat().st_size != item['bytes']:
            raise ValueError('Bundle file size differs: ' + relative)
    runtime = doc.get('runtime', {})
    for key in ('repository', 'release', 'ros_overlay', 'python_overlay'):
        if key not in runtime or not inside(root, runtime[key]).exists():
            raise ValueError('Missing bundle runtime path: ' + key)
    if runtime['release'] not in doc['files']:
        raise ValueError('Portable release must be covered by inventory')
    return root, doc


def child_environment(root, doc, inherited=None):
    """Preserve installed ROS, but never inherit a user virtualenv or Python path."""
    env = dict(os.environ if inherited is None else inherited)
    runtime = doc['runtime']
    repo = inside(root, runtime['repository'])
    overlay = inside(root, runtime['ros_overlay'])
    python = inside(root, runtime['python_overlay'])
    home = Path(root) / 'runtime-home'
    ros = Path('/opt/ros/noetic')
    system_python = [str(ros / 'lib/python3/dist-packages')]
    env.update(HOME=str(home), ROS_HOME=str(home / '.ros'),
               ROS_LOG_DIR=str(home / '.ros/log'), PYTHONDONTWRITEBYTECODE='1',
               PYTHONNOUSERSITE='1', ROS_MASTER_URI='http://127.0.0.1:11311',
               ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
               ROS_PACKAGE_PATH=str(overlay / 'share') + ':' + str(ros / 'share'),
               PYTHONPATH=os.pathsep.join([str(python), str(overlay / 'lib/python3/dist-packages'),
                   str(repo / 'dataset/morai_od/scripts'), str(repo), *system_python]),
               PATH=str(ros / 'bin') + ':/usr/local/bin:/usr/bin:/bin',
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    for key in ('VIRTUAL_ENV', 'CONDA_PREFIX', 'PYTHONHOME', 'LD_PRELOAD',
                'SSH_AUTH_SOCK', 'SSH_AGENT_PID', 'GIT_ASKPASS'):
        env.pop(key, None)
    return env


DEPENDENCY_PROBE = r'''
import importlib, json, pathlib, sys
expected = tuple(map(int, sys.argv[1].split('.')))
if sys.version_info[:2] != expected:
    raise RuntimeError('Bundle requires Python ' + sys.argv[1])
modules = ('numpy', 'PIL.Image', 'scipy.spatial', 'shapely.geometry', 'cv2', 'yaml',
           'rospy', 'roslaunch', 'genpy', 'rospkg', 'roslib.message',
           'rosbridge_library.capabilities.advertise_service', 'rosbridge_server',
           'rosbridge_server.autobahn_websocket', 'rosbridge_library.rosbridge_protocol',
           'rosbridge_msgs.msg', 'rosauth.srv', 'autobahn.twisted.websocket',
           'twisted.internet.ssl', 'tornado.websocket', 'bson', 'distutils.version')
loaded = {name: importlib.import_module(name) for name in modules}
msg = importlib.import_module('morai_msgs.msg')
srv = importlib.import_module('morai_msgs.srv')
types = {name: getattr(msg, name) for name in
         ('CtrlCmd', 'EgoVehicleStatus', 'ObjectStatusList', 'SaveSensorData', 'ScenarioLoad', 'CollisionData')}
types.update({name: getattr(srv, name) for name in ('MoraiEventCmdSrv', 'MoraiEventCmdSrvRequest')})
md5 = {}
for name, cls in types.items():
    value = getattr(cls, '_md5sum', '')
    if len(value) != 32:
        raise RuntimeError('Invalid generated ROS type: ' + name)
    md5[name] = value
module = loaded['rosbridge_library.capabilities.advertise_service']
for name in ('next_id', 'handle_request', 'graceful_shutdown'):
    if not callable(getattr(module.AdvertisedServiceHandler, name, None)):
        raise RuntimeError('Unsupported rosbridge handler API: ' + name)
if not callable(getattr(module.AdvertiseService, 'advertise_service', None)):
    raise RuntimeError('Unsupported rosbridge advertise API')
prefix = pathlib.Path(loaded['rospkg'].RosPack().get_path('rosbridge_server')).parents[1]
executable = prefix / 'lib/rosbridge_server/rosbridge_websocket'
if not executable.is_file():
    raise RuntimeError('Guarded bridge executable missing: ' + str(executable))
compile(executable.read_text(), str(executable), 'exec')
print(json.dumps(dict(event='portable_dependencies_checked', python=sys.version.split()[0],
    ros_md5=md5, modules={name: getattr(module, '__file__', None) for name, module in loaded.items()},
    rosbridge_executable=str(executable), ros_started=False, simulator_contacted=False)))
'''


def check_dependencies(root, doc, python=sys.executable):
    version = doc['runtime'].get('python_version', '3.10')
    result = subprocess.run([python, '-B', '-c', DEPENDENCY_PROBE, version],
                            env=child_environment(root, doc),
                            cwd=inside(root, doc['runtime']['repository']),
                            capture_output=True, text=True, check=True, timeout=45)
    report = json.loads(result.stdout.strip().splitlines()[-1])
    expected = doc['runtime'].get('ros_md5', {})
    required = {'CtrlCmd', 'EgoVehicleStatus', 'ObjectStatusList', 'SaveSensorData',
                'ScenarioLoad', 'CollisionData', 'MoraiEventCmdSrv', 'MoraiEventCmdSrvRequest'}
    if (set(expected) != required or set(report['ros_md5']) != required
            or any(report['ros_md5'].get(name) != value for name, value in expected.items())):
        raise ValueError('ROS wire identity differs or was not pinned')
    return report


def approved(root, doc, approval, action, confirmed_environment):
    """Approval is explicit and binds this exact manifest, environment and paths."""
    if approval is None:
        raise ValueError('Separate operator approval file required; no action performed')
    record = json.loads(Path(approval).read_text())
    if (record.get('schema') != APPROVAL_SCHEMA or record.get('approved') is not True
            or record.get('bundle_sha256') != sha256(Path(root) / MANIFEST)
            or action not in record.get('actions', [])):
        raise ValueError('Approval does not authorize this bundle/action')
    if (confirmed_environment != 'sunny13' or record.get('environment') != confirmed_environment):
        raise ValueError('MR UI Sunny13 must be explicitly confirmed')
    for key in ('scenario_directory', 'sensor_directory'):
        value = record.get(key)
        if not isinstance(value, str) or not Path(value).is_absolute():
            raise ValueError('Explicit absolute MR directory required: ' + key)
        folder = Path(value).resolve(strict=True)
        if not folder.is_dir() or not os.access(folder, os.R_OK | os.W_OK | os.X_OK):
            raise ValueError('MR directory is not an accessible writable folder: ' + key)
    return record


def activate(root, doc, approval, confirmed_environment):
    """Create only owned bundle-home links; never create or modify MR directories."""
    record = approved(root, doc, approval, 'activate', confirmed_environment)
    home = Path(root) / 'runtime-home'
    if home.is_symlink():
        raise ValueError('Runtime HOME must not be a symlink')
    pairs = ((home / SCENARIO_RELATIVE, Path(record['scenario_directory']).resolve()),
             (home / SENSOR_RELATIVE, Path(record['sensor_directory']).resolve()))
    # Validate all existing links/ancestors before creating any directories.
    for link, target in pairs:
        for parent in link.parents:
            if parent == home:
                break
            if parent.is_symlink():
                raise ValueError('Unexpected runtime parent symlink')
        if link.is_symlink():
            if link.resolve(strict=True) != target:
                raise ValueError('Existing runtime MR link differs; preserve it')
        elif link.exists():
            raise ValueError('Existing runtime MR path would be overwritten')
    for link, target in pairs:
        if not link.is_symlink():
            link.parent.mkdir(parents=True, exist_ok=True)
            link.symlink_to(target, target_is_directory=True)
    return dict(event='portable_runtime_activated', home=str(home),
                scenario_directory=str(pairs[0][1]), sensor_directory=str(pairs[1][1]),
                simulator_contacted=False)


def require_activation(root, record):
    home = Path(root) / 'runtime-home'
    if home.is_symlink():
        raise ValueError('Runtime HOME must not be a symlink')
    for relative, key in ((SCENARIO_RELATIVE, 'scenario_directory'), (SENSOR_RELATIVE, 'sensor_directory')):
        link = home / relative
        if not link.is_symlink() or link.resolve(strict=True) != Path(record[key]).resolve(strict=True):
            raise ValueError('Explicit MR path activation is missing or different')


def command_plan(root, doc, action):
    runtime = doc['runtime']
    repo = inside(root, runtime['repository'])
    if action == 'bridge':
        return [sys.executable, '-B', str(repo / 'dataset/morai_od/scripts/run_base5_rosbridge.py'),
                '_address:=127.0.0.1']
    if action == 'capture':
        return [sys.executable, '-B', str(repo / 'dataset/morai_od/scripts/launch_base5_obstacle_support.py'),
                '--release', str(inside(root, runtime['release'])), '--environment', 'sunny13',
                '--run', '--confirmed-environment', 'sunny13']
    raise ValueError('No portable live command qualified for action: ' + action)


def live_run(root, doc, action, approval, confirmed_environment, smoke_receipt=None):
    record = approved(root, doc, approval, action, confirmed_environment)
    require_activation(root, record)
    if action == 'capture':
        # This preparation-only release intentionally cannot execute collection.
        # Absolute source pins need a separately qualified logical namespace;
        # a boolean or a hand-written smoke receipt cannot supply that proof.
        raise ValueError('Portable collection is not implemented/qualified; original-path namespace and live smoke pending')
    if doc.get('native_live_execution_enabled') is not True or doc.get('can_run') is not True:
        raise ValueError('Portable native execution is not yet qualified; no live action performed')
    check_dependencies(root, doc)
    return subprocess.call(command_plan(root, doc, action), env=child_environment(root, doc),
                           cwd=inside(root, doc['runtime']['repository']))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    sub = parser.add_subparsers(dest='action', required=True)
    check = sub.add_parser('check')
    check.add_argument('--inventory-only', action='store_true',
                       help='Only file integrity; does not certify native dependencies')
    activation = sub.add_parser('activate')
    activation.add_argument('--approval', type=Path, required=True)
    activation.add_argument('--confirmed-environment', required=True)
    for name in ('bridge', 'capture'):
        command = sub.add_parser(name)
        command.add_argument('--run', action='store_true', help='Requires a separately approved qualified bundle')
        command.add_argument('--approval', type=Path)
        command.add_argument('--confirmed-environment')
        command.add_argument('--smoke-receipt', type=Path)
    args = parser.parse_args(argv)
    root, doc = load_bundle(args.bundle)
    if args.action == 'check':
        report = dict(event='portable_inventory_checked', files=len(doc['files']),
                      dependencies_checked=False, simulator_contacted=False)
        if not args.inventory_only:
            report = check_dependencies(root, doc)
        print(json.dumps(report))
        return 0
    if args.action == 'activate':
        print(json.dumps(activate(root, doc, args.approval, args.confirmed_environment)))
        return 0
    if not args.run:
        print(json.dumps(dict(event='portable_command_plan_only', command=command_plan(root, doc, args.action),
                              native_live_execution_enabled=doc.get('native_live_execution_enabled') is True,
                              can_run=False, requires_original_namespace=True,
                              qualification='Preparation only; original namespace/live smoke pending',
                              simulator_contacted=False)))
        return 0
    return live_run(root, doc, args.action, args.approval,
                    args.confirmed_environment, args.smoke_receipt)


if __name__ == '__main__':
    raise SystemExit(main())
