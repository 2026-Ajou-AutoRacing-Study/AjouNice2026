#!/usr/bin/env python3
"""Pinned, opt-in traffic-light assets. No Torch/ROS, global pip or live nodes."""
import argparse
import ctypes
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTOR = ROOT / 'configs/perception/traffic_light_v5_candidate_release.json'
SCHEMA = 'ajounice_traffic_light_candidate_v1'


def publish_no_replace(source, destination):
    """Linux atomic rename that cannot replace even a concurrently created dir."""
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise RuntimeError('Atomic no-replace publication requires Linux renameat2')
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(source), -100, os.fsencode(destination), 1) != 0:
        code = ctypes.get_errno()
        raise OSError(code, os.strerror(code), str(destination))


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def safe_member(value):
    if not isinstance(value, str) or not value or '\\' in value:
        raise ValueError('Invalid asset member')
    path = PurePosixPath(value)
    if not path.parts or path.is_absolute() or '..' in path.parts or path.as_posix() != value:
        raise ValueError('Unsafe asset member')
    return value


def load_release(path=DESCRIPTOR):
    doc = json.loads(Path(path).read_text())
    if (doc.get('schema') != SCHEMA or doc.get('status') != 'opt_in_candidate_not_production'
            or not re.fullmatch(r'v[0-9]+_[0-9a-f]{8}', doc.get('version', ''))
            or set(doc.get('models', {})) != {'yolo', 'cnn'}):
        raise ValueError('Explicit candidate release descriptor required')
    if Path(safe_member(doc['archive'])).name != doc['archive']:
        raise ValueError('Archive must be a basename')
    for item in [dict(bytes=doc['archive_bytes'], sha256=doc['archive_sha256'])] + list(doc['models'].values()):
        if type(item['bytes']) is not int or not 0 < item['bytes'] < 128 * 1024**2:
            raise ValueError('Invalid bounded asset size')
        if not re.fullmatch(r'[0-9a-f]{64}', item['sha256']):
            raise ValueError('Invalid asset SHA256')
    members = [safe_member(item['member']) for item in doc['models'].values()]
    if len(set(members)) != 2:
        raise ValueError('Distinct model members required')
    return doc


def default_models_dir(release=None):
    release = release or load_release()
    return Path.home() / '.local/share/ajounice/traffic_light_models' / release['version']


def paths(models_dir=None, release=None):
    release = release or load_release()
    directory = Path(models_dir).expanduser() if models_dir is not None else default_models_dir(release)
    directory = Path(os.path.abspath(os.path.expandvars(str(directory))))
    return {key: directory / item['member'] for key, item in release['models'].items()}


def validate_install(models_dir=None, release=None):
    release = release or load_release()
    selected = paths(models_dir, release)
    directory = next(iter(selected.values())).parents[1]
    if directory.is_symlink():
        raise ValueError('Model installation root must not be a symlink')
    for key, path in selected.items():
        expected = release['models'][key]
        if path.is_symlink() or not path.is_file() or path.stat().st_size != expected['bytes'] or digest(path) != expected['sha256']:
            raise ValueError('Missing/changed candidate model: {}. Run install_traffic_light_candidate.sh; existing files are preserved'.format(path))
    ready = directory / 'models_ready.json'
    if ready.is_symlink() or not ready.is_file():
        raise ValueError('Candidate installation receipt missing; run the installer')
    receipt = json.loads(ready.read_text())
    expected = {key: item['sha256'] for key, item in release['models'].items()}
    if receipt.get('schema') != SCHEMA or receipt.get('version') != release['version'] or receipt.get('models_sha256') != expected:
        raise ValueError('Candidate installation receipt differs from release')
    return {key: str(path) for key, path in selected.items()}


def validate_archive(archive, release):
    archive = Path(archive)
    if archive.stat().st_size != release['archive_bytes'] or digest(archive) != release['archive_sha256']:
        raise ValueError('Archive checksum/size mismatch; nothing installed')
    with tarfile.open(archive, 'r:gz') as tar:
        seen = set()
        members = tar.getmembers()
        if len(members) > 128 or sum(member.size for member in members) > 128 * 1024**2:
            raise ValueError('Archive exceeds candidate asset bounds')
        for member in members:
            safe_member(member.name)
            if member.name in seen or not member.isfile():
                raise ValueError('Duplicate/non-regular archive member')
            seen.add(member.name)
        for item in release['models'].values():
            if item['member'] not in seen or tar.getmember(item['member']).size != item['bytes']:
                raise ValueError('Frozen candidate model missing from archive')


def install(archive=None, models_dir=None, remote='ajou_drive', release=None):
    release = release or load_release()
    directory = paths(models_dir, release)['yolo'].parents[1]
    if directory.exists() or directory.is_symlink():
        return {'status': 'MODELS_VERIFIED', 'models': validate_install(directory, release),
                'downloaded': False, 'runtime_qualified': False, 'production_changed': False}
    directory.parent.mkdir(parents=True, exist_ok=True)
    with (directory.parent / (directory.name + '.install.lock')).open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if directory.exists() or directory.is_symlink():
            return {'status': 'MODELS_VERIFIED', 'models': validate_install(directory, release),
                    'downloaded': False, 'runtime_qualified': False, 'production_changed': False}
        with tempfile.TemporaryDirectory(prefix='traffic_candidate_', dir=directory.parent) as temp:
            temp = Path(temp)
            downloaded = archive is None
            if archive is None:
                candidate = Path.home() / 'Downloads' / release['archive']
                if candidate.is_file() and candidate.stat().st_size == release['archive_bytes'] and digest(candidate) == release['archive_sha256']:
                    archive, downloaded = candidate, False
                else:
                    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', remote):
                        raise ValueError('Use an existing authorized rclone remote NAME, not a path or URL')
                    archive = temp / release['archive']
                    try:
                        subprocess.run(['rclone', 'copyto', remote + ':' + release['archive'], str(archive),
                            '--drive-root-folder-id', release['drive_root_folder_id'], '--immutable',
                            '--progress', '--retries', '1', '--low-level-retries', '2',
                            '--contimeout', '10s', '--timeout', '30s'], check=True, timeout=600)
                    except (OSError, subprocess.SubprocessError) as error:
                        raise RuntimeError('Drive download unavailable. Use a read-authorized rclone remote or --archive /path/to/final_weights_bundle.tar.gz; no sharing permissions were changed') from error
            validate_archive(archive, release)
            staged = temp / 'models'; staged.mkdir()
            with tarfile.open(archive, 'r:gz') as tar:
                for key, item in release['models'].items():
                    target = staged / item['member']; target.parent.mkdir(parents=True, exist_ok=True)
                    with tar.extractfile(item['member']) as source, target.open('xb') as destination:
                        shutil.copyfileobj(source, destination)
                    if digest(target) != item['sha256']:
                        raise ValueError('Extracted model SHA mismatch: ' + key)
            receipt = {'schema': SCHEMA, 'version': release['version'],
                       'models_sha256': {key: item['sha256'] for key, item in release['models'].items()},
                       'archive_sha256': release['archive_sha256'], 'production_changed': False,
                       'runtime_qualified': False, 'training_or_nodes_started': False}
            (staged / 'models_ready.json').write_text(json.dumps(receipt, indent=2) + '\n')
            # Never replace an existing destination, including a concurrent unmanaged one.
            if directory.exists() or directory.is_symlink():
                raise FileExistsError('Candidate destination appeared; preserving it')
            publish_no_replace(staged, directory)
    return {'status': 'MODELS_VERIFIED', 'models': validate_install(directory, release),
            'downloaded': downloaded, 'runtime_qualified': False, 'production_changed': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path)
    parser.add_argument('--models-dir', type=Path)
    parser.add_argument('--remote', default='ajou_drive')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--print-paths', action='store_true')
    args = parser.parse_args()
    if args.print_paths:
        print(json.dumps(paths(args.models_dir), default=str)); return
    result = ({'status': 'MODELS_VERIFIED', 'models': validate_install(args.models_dir), 'runtime_qualified': False}
              if args.check else install(args.archive, args.models_dir, args.remote))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
