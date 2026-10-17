#!/usr/bin/env python3
"""Build a local, credential-free preparation bundle; never deploy or capture."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

from portable_paths import release_pin_plan, copy_graph, verify_graph

REPO = Path(__file__).resolve().parents[2]
SCHEMA = 'base5_portable_native_bundle_v1'
VERSION = 'v.S4.251001.MolitComp03'
DATA = 'MoraiLauncher_Lin_Data'
FORBIDDEN = {'.git', '.ssh', '.aws', '.codex', '__pycache__', 'unity.lock',
             'V2AccountInfo.byte', 'optionList.byte', 'networkInfo.txt'}
CORE_DIRS = ('Resources', 'Managed', 'MonoBleedingEdge', 'Plugins',
             'Bundle/' + VERSION, 'Project/Simulator/Simulator_' + VERSION + '_Linux')
CORE_FILES = ('MoraiLauncher_Lin.x86_64', 'UnityPlayer.so', DATA + '/boot.config',
              DATA + '/ScriptingAssemblies.json', DATA + '/data.unity3d',
              DATA + '/Version/version.txt', DATA + '/Version/NotAutoPatch.txt',
              DATA + '/Version/NotRunnerPatch.txt')
SENSOR = 'configs/sensors/Sensor_competition_local.json'
NETWORK = 'configs/network/ego_network_mr_base5_savesensordata_collision_local.json'


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, sort_keys=True, indent=2)
        stream.write('\n')


def safe_file(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError('Expected a regular nonsymlink file: ' + str(path))
    if any(part in FORBIDDEN for part in path.parts):
        raise ValueError('Forbidden source file: ' + str(path))
    low = path.name.lower()
    if (low.endswith(('.pyc', '.pyo', '.pem', '.key', '.debug'))
            or low.startswith(('client_secret', 'id_rsa', 'id_ed25519', '.env'))
            or low in ('rclone.conf', '.netrc', 'credentials.json', 'token.json')):
        raise ValueError('Excluded private/cache/debug file: ' + str(path))


def copy_file(source, destination):
    safe_file(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise ValueError('Refusing an existing destination: ' + str(destination))
    expected = sha256(source)
    shutil.copyfile(source, destination)
    os.chmod(destination, 0o755 if os.access(source, os.X_OK) else 0o644)
    if sha256(source) != expected or sha256(destination) != expected:
        raise ValueError('Source changed while packaging: ' + str(source))


def morai_files(root):
    """Allowlisted runtime, not the launcher's account/cache/SaveFile tree."""
    root = Path(root).resolve(strict=True)
    selected = [root / name for name in CORE_FILES]
    for relative in CORE_DIRS:
        directory = root / DATA / relative
        if not directory.is_dir() or directory.is_symlink():
            raise ValueError('Required MORAI runtime directory unavailable: ' + str(directory))
        for file in sorted(directory.rglob('*')):
            if file.is_dir():
                continue
            rel = file.relative_to(root)
            if any(part in FORBIDDEN or part.endswith('_DoNotShip') for part in rel.parts):
                continue
            if file.suffix.lower() in ('.debug', '.log', '.pyc', '.pyo'):
                continue
            safe_file(file)
            selected.append(file)
    for file in selected:
        safe_file(file)
    return sorted(set(selected))


def inventory(root):
    result = {}
    for path in sorted(root.rglob('*')):
        if path.is_dir():
            continue
        safe_file(path)
        relative = path.relative_to(root).as_posix()
        result[relative] = dict(sha256=sha256(path), bytes=path.stat().st_size)
    return result


def expand_wheels(wheels, destination):
    """Unpack pinned library wheels; do not run pip, setup.py or .pth hooks."""
    destination.mkdir()
    for wheel in wheels:
        if wheel.suffix != '.whl':
            raise ValueError('Only explicit offline wheels are accepted')
        with zipfile.ZipFile(wheel) as archive:
            for entry in archive.infolist():
                path = Path(entry.filename)
                if path.is_absolute() or '..' in path.parts or '\\' in entry.filename:
                    raise ValueError('Unsafe wheel member')
                if entry.external_attr >> 16 & 0o170000 == 0o120000:
                    raise ValueError('Wheel symlink is not permitted')
                if entry.is_dir():
                    continue
                # Wheel data libraries belong at the import root; executable
                # entrypoint scripts are neither required nor generated here.
                parts = path.parts
                # Transport wheels can contain example TLS private keys in
                # their upstream test suite. Tests are not runtime payload.
                if any(part in ('test', 'tests', '__pycache__') for part in parts):
                    continue
                if parts[0].endswith('.data'):
                    if len(parts) < 3 or parts[1] not in ('purelib', 'platlib'):
                        continue
                    path = Path(*parts[2:])
                target = destination / path
                if target.exists():
                    raise ValueError('Overlapping wheel files: ' + str(path))
                target.parent.mkdir(parents=True, exist_ok=True)
                with target.open('xb') as stream:
                    stream.write(archive.read(entry))


def prepare(release, morai, output, wheels=None, run=False):
    release = Path(release).resolve(strict=True)
    morai = Path(morai).resolve(strict=True)
    output = Path(output).absolute()
    if output.exists() or output.is_symlink():
        raise ValueError('A fresh local bundle directory is required')
    # The caller must not use a remote mount as a destination; no network API exists here.
    for protected in (REPO, morai, Path('/home/luv/morai_cap')):
        if output == protected or protected in output.parents and protected != REPO:
            raise ValueError('Output overlaps protected runtime/data')
    extra = {str(REPO / name): sha256(REPO / name) for name in (SENSOR, NETWORK)}
    # The conversion helper reads additional model functions by AST, without Torch.
    from finish import dependency_paths
    for path in dependency_paths(REPO):
        extra[str(path)] = sha256(path)
    plan = release_pin_plan(release, extra_pins=extra)
    core = morai_files(morai)
    report = dict(schema=SCHEMA, release_sha256=sha256(release),
                  morai_files=len(core), morai_bytes=sum(p.stat().st_size for p in core),
                  output=str(output), execution='local assembly only',
                  remote_touched=False, can_run=False)
    if not run:
        return report
    wheel_files = sorted(Path(wheels).resolve(strict=True).iterdir()) if wheels else []
    for path in wheel_files:
        safe_file(path)
        if path.suffix != '.whl':
            raise ValueError('Wheel directory must contain wheels only')
    needed = report['morai_bytes'] + plan['payload_bytes'] + 4 * sum(p.stat().st_size for p in wheel_files) + 2 * 1024**3
    if shutil.disk_usage(output.parent).free < needed:
        raise ValueError('Insufficient local assembly headroom')
    copy_graph(plan, output)
    output.chmod(0o700)
    dump(output / 'path_graph.json', plan)
    mr = output / 'morai/MoraiLauncher_Lin'
    for source in core:
        copy_file(source, mr / source.relative_to(morai))
    save = mr / DATA / 'SaveFile'
    copy_file(REPO / SENSOR, save / 'Sensor' / Path(SENSOR).name)
    copy_file(REPO / NETWORK, save / 'Network' / Path(NETWORK).name)
    from dependencies import assemble_overlay
    overlay = assemble_overlay(output / 'overlay', REPO / 'base_ws/install_aw149', Path('/opt/ros/noetic'), REPO / 'base_ws/src/morai/morai_msgs')
    dump(output / 'overlay_receipt.json', overlay)
    for source in wheel_files:
        copy_file(source, output / 'wheels' / source.name)
    expand_wheels(wheel_files, output / 'python')
    for source in sorted(Path(__file__).parent.glob('*.py')):
        copy_file(source, output / 'tools' / source.name)
    copy_file(Path(__file__).parent / 'requirements-cp310.txt', output / 'tools/requirements-cp310.txt')
    copy_file(Path('/usr/bin/bwrap'), output / 'bin/bwrap')
    copy_file(Path('/usr/share/doc/bubblewrap/copyright'), output / 'licenses/bubblewrap-copyright')
    verify_graph(plan, output)
    runtime = dict(repository='payload/home/luv/AjouNice2026',
                   release=release.relative_to('/').as_posix(),
                   ros_overlay='overlay', python_overlay='python', python_version='3.10',
                   ros_md5=overlay.get('ros_md5', {}), namespace_helper='tools/namespace.py')
    runtime['release'] = 'payload/' + runtime['release']
    files = inventory(output)
    manifest = dict(schema=SCHEMA, files=files, runtime=runtime, can_run=False,
                    requires_original_namespace=True, source_commit=subprocess.check_output(
                        ['git', '-C', str(REPO), 'rev-parse', 'HEAD'], text=True).strip(),
                    source_release_sha256=sha256(release),
                    target=dict(ubuntu='22.04', python='3.10', ros='/opt/ros/noetic'),
                    pending=['approved remote extraction', 'native dependency qualification',
                             'user namespace availability', 'MORAI login/license/UI check',
                             'fresh independent small capture and conversion', 'capture authorization'],
                    excluded=['Git metadata and credentials', 'MORAI stored account',
                              'old SaveFile content', 'locks/logs/debug symbols',
                              'CUDA/PyTorch/model weights'],
                    remote_touched=False)
    dump(output / 'portable_bundle.json', manifest)
    return dict(report, files=len(files), expanded_bytes=sum(v['bytes'] for v in files.values()),
                bundle_manifest_sha256=sha256(output / 'portable_bundle.json'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--morai', type=Path, default=Path.home() / 'MoraiLauncher_Lin')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wheels', type=Path)
    parser.add_argument('--run', action='store_true', help='Assemble new LOCAL files only; no deployment')
    args = parser.parse_args()
    print(json.dumps(prepare(args.release, args.morai, args.output, args.wheels, args.run), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
