#!/usr/bin/env python3
"""Approved-folder deployment checks only. Never starts MORAI, ROS or capture."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tarfile
import time

ARCHIVE_SHA = 'e56cb9493a18579c636c06b34171e0c34ab30c5a6424fc35703adbb6fc2282d1'
MANIFEST_SHA = 'f0f30909639e0f3123e6fc89ac64b01bb60a8b6445be4331b9e7efee2f1cb1f0'
ARCHIVE_NAME = 'base5_capture_native_v1_261017.tar.zst'
WORKSPACE_NAME = 'ajounice_base5_capture'
EXPECTED_FILES = 31182


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def scope(path):
    path = Path(path).absolute()
    if '..' in path.parts or path.name != WORKSPACE_NAME or path.parent != Path.home().resolve():
        raise ValueError('Only the explicitly approved new workspace under this user HOME is allowed')
    for item in (path, *path.parents):
        if item.is_symlink():
            raise ValueError('Workspace symlink is forbidden')
    if not path.is_dir() or path.stat().st_uid != os.getuid():
        raise ValueError('Existing user-owned approved workspace required')
    return path


def member_path(root, name):
    path = PurePosixPath(name)
    if (path.is_absolute() or not path.parts or path.parts[0] != 'capture_bundle'
            or '..' in path.parts or '\\' in name or str(path) != name):
        raise ValueError('Unsafe archive member: ' + name)
    target = root / path
    for item in (target, *target.parents):
        if item == root:
            break
        if item.is_symlink():
            raise ValueError('Symlink at extraction destination')
    return target


def extract(root):
    archive = root / 'incoming' / ARCHIVE_NAME
    if archive.is_symlink() or not archive.is_file() or digest(archive) != ARCHIVE_SHA:
        raise ValueError('Transferred archive does not match the frozen local SHA')
    destination = root / 'capture_bundle'
    if destination.exists() or destination.is_symlink():
        raise ValueError('Refusing an existing extraction; preserve it and inspect the receipt')
    if shutil.disk_usage(root).free < 9 * 1024**3:
        raise ValueError('Insufficient extraction headroom')
    process = subprocess.Popen(['zstd', '-q', '-d', '-c', str(archive)], stdout=subprocess.PIPE)
    count, total = 0, 0
    try:
        with tarfile.open(fileobj=process.stdout, mode='r|') as incoming:
            for item in incoming:
                target = member_path(root, item.name)
                if not item.isfile() or item.size < 0:
                    raise ValueError('Only regular archive files are accepted')
                total += item.size
                if total > 8 * 1024**3 or item.size > 2 * 1024**3:
                    raise ValueError('Unexpected extraction volume')
                target.parent.mkdir(parents=True, exist_ok=True)
                source = incoming.extractfile(item)
                if source is None:
                    raise ValueError('Missing archive member data')
                with source, target.open('xb') as output:
                    shutil.copyfileobj(source, output, 1024**2)
                target.chmod(item.mode & 0o777)
                count += 1
        if process.wait() != 0:
            raise ValueError('Decompression failed; partial extraction is preserved')
    finally:
        process.stdout.close()
        if process.poll() is None:
            process.terminate()
            process.wait()
    if digest(destination / 'portable_bundle.json') != MANIFEST_SHA or count != EXPECTED_FILES:
        raise ValueError('Extracted manifest/file count differs')
    return dict(archive_sha256=ARCHIVE_SHA, manifest_sha256=MANIFEST_SHA,
                files=count, bytes=total, extracted=str(destination))


def load(root):
    bundle = root / 'capture_bundle'
    if digest(bundle / 'portable_bundle.json') != MANIFEST_SHA:
        raise ValueError('Unqualified bundle manifest')
    sys.path.insert(0, str(bundle / 'tools'))
    import runtime
    return runtime, *runtime.load_bundle(bundle)


def command(args, env=None, timeout=180):
    result = subprocess.run(args, capture_output=True, text=True, env=env, timeout=timeout)
    return dict(command=args, returncode=result.returncode,
                stdout=result.stdout, stderr=result.stderr)


def restore_modes(root):
    """Restore only the frozen graph's original POSIX bits in the NEW copy.

    The v1 archive normalized group-write/read bits. No file bytes, hashes,
    release JSON or host/system files are changed by this compatibility step.
    """
    runtime, bundle, doc = load(root)
    graph = json.loads((bundle / 'portable_graph.json').read_text())
    changes, pending = [], []
    for relative, pin in graph['files'].items():
        if not relative.startswith('payload/') or relative not in doc['files']:
            raise ValueError('Mode record is outside the immutable graph')
        expected = pin['mode']
        if (type(expected) is not int or expected < 0 or expected > 0o777
                or doc['files'][relative]['sha256'] != pin['sha256']):
            raise ValueError('Untrusted mode record')
        path = runtime.inside(bundle, relative)
        before = path.stat().st_mode & 0o777
        if before != expected:
            pending.append((path, expected))
            changes.append(dict(path=relative, before=oct(before), after=oct(expected)))
    directory_changes = []
    for path in (root, *root.rglob('*')):
        if path.is_symlink():
            raise ValueError('Unexpected symlink in approved deployment workspace')
        if path.is_dir():
            if path.stat().st_uid != os.getuid():
                raise ValueError('Foreign-owned deployment directory')
            before = path.stat().st_mode & 0o777
            after = before & ~0o022
            if before != after:
                pending.append((path, after))
                directory_changes.append(str(path.relative_to(root)))
    # Validate the whole operation before making any permission changes.
    for path, mode in pending:
        path.chmod(mode)
    from portable_paths import verify_graph
    verified = verify_graph(graph, bundle)
    return dict(mode_changes=len(changes), changes=changes,
                secured_directories=len(directory_changes), directories=directory_changes,
                graph_verified=verified,
                file_contents_changed=False, host_system_changed=False)


def check(root):
    runtime, bundle, doc = load(root)
    checks = {}
    checks['native_dependencies'] = command([sys.executable, '-B', str(bundle / 'tools/runtime.py'),
                                             '--bundle', str(bundle), 'check'])
    checks['namespace'] = command([sys.executable, '-B', str(bundle / 'tools/namespace.py'),
                                   '--bundle', str(bundle), '--probe'])
    env = runtime.child_environment(bundle, doc)
    repo = bundle / doc['runtime']['repository']
    if checks['native_dependencies']['returncode'] == 0:
        for name in ('qualify_base5_training_inputs.py', 'launch_base5_obstacle_support.py'):
            checks[name] = command([sys.executable, '-B', str(repo / 'dataset/morai_od/scripts' / name), '--help'], env)
    mr = bundle / 'morai/MoraiLauncher_Lin'
    binaries = [mr / 'MoraiLauncher_Lin.x86_64', mr / 'UnityPlayer.so',
                mr / 'MoraiLauncher_Lin_Data/Project/Simulator/Simulator_v.S4.251001.MolitComp03_Linux/Simulator.x86_64',
                mr / 'MoraiLauncher_Lin_Data/Project/Simulator/Simulator_v.S4.251001.MolitComp03_Linux/UnityPlayer.so']
    for binary in binaries:
        if not binary.is_file():
            checks['ldd:' + str(binary.relative_to(mr))] = dict(returncode=1, stderr='Missing binary', stdout='')
        else:
            result = command(['ldd', str(binary)], dict(PATH='/usr/bin:/bin', LANG='C.UTF-8', HOME=str(root)))
            if 'not found' in result['stdout']:
                result['returncode'] = 1
            checks['ldd:' + str(binary.relative_to(mr))] = result
    checks['gpu_inventory'] = command(['nvidia-smi', '--query-gpu=name,driver_version,memory.total', '--format=csv,noheader'])
    return dict(checks=checks, pass_all=all(v['returncode'] == 0 for v in checks.values()),
                ros_started=False, morai_started=False, capture_started=False)


def prepare_morai(root):
    _, bundle, doc = load(root)
    source = bundle / 'morai/MoraiLauncher_Lin'
    target = root / 'work/morai/MoraiLauncher_Lin'
    if target.exists() or target.is_symlink():
        raise ValueError('Existing MORAI working copy must not be overwritten')
    if shutil.disk_usage(root).free < 6 * 1024**3:
        raise ValueError('Insufficient working-copy headroom')
    for parent in (target.parent, target.parent.parent):
        if parent.is_symlink():
            raise ValueError('Working-copy parent symlink is forbidden')
    shutil.copytree(source, target, copy_function=shutil.copy2)
    prefix = 'morai/MoraiLauncher_Lin/'
    count, total = 0, 0
    for relative, pin in doc['files'].items():
        if not relative.startswith(prefix):
            continue
        left, right = bundle / relative, target / relative[len(prefix):]
        if (digest(right) != pin['sha256'] or os.path.samefile(left, right)
                or right.stat().st_size != pin['bytes']):
            raise ValueError('Mutable MORAI copy differs or is hardlinked to immutable input')
        count, total = count + 1, total + pin['bytes']
    return dict(morai=str(target), files=count, bytes=total, hardlinked=False, morai_started=False)


def relocate_morai(root, approved=False):
    """Move only our working copy to the separately approved short HOME path."""
    if not approved:
        raise ValueError('Separate approval for HOME/MoraiLauncher_Lin is required')
    source = root / 'work/morai/MoraiLauncher_Lin'
    target = root.parent / 'MoraiLauncher_Lin'
    for path in (source, *source.parents):
        if path.is_symlink():
            raise ValueError('Source parent symlink is forbidden')
    if (not source.is_dir() or source.stat().st_uid != os.getuid()
            or target.exists() or target.is_symlink()):
        raise ValueError('Owned source and an absent destination are required; nothing overwritten')
    if source.stat().st_dev != target.parent.stat().st_dev:
        raise ValueError('Only a same-filesystem move is supported')
    active = []
    for process in Path('/proc').glob('[0-9]*'):
        try:
            executable = os.readlink(process / 'exe')
        except OSError:
            continue
        if executable.startswith(str(source) + '/'):
            active.append(process.name)
    if active:
        raise ValueError('Close the running MORAI working copy first: ' + ','.join(active))
    before = source.stat()
    log = root / 'work/morai-session/launcher.log'
    for folder in (root / 'checks', log.parent):
        for path in (folder, *folder.parents):
            if path.is_symlink():
                raise ValueError('Log/receipt parent symlink is forbidden')
            if path == root:
                break
    backup = None
    if log.is_file() and not log.is_symlink():
        backup = root / 'checks' / ('launcher-before-shortpath-' + str(time.time_ns()) + '.log')
        with log.open('rb') as original, backup.open('xb') as copied:
            shutil.copyfileobj(original, copied)
        backup.chmod(0o600)
    subprocess.run(['mv', '--no-clobber', '--no-target-directory', '--', str(source), str(target)], check=True)
    if source.exists() or not target.is_dir() or target.is_symlink():
        raise ValueError('Move did not complete; preserve both paths and inspect')
    after = target.stat()
    if (before.st_dev, before.st_ino) != (after.st_dev, after.st_ino):
        raise ValueError('Unexpected move identity')
    target.chmod(0o700)
    return dict(source=str(source), target=str(target), same_inode=True,
                existing_destination_overwritten=False, log_backup=str(backup) if backup else None,
                morai_started=False, ros_started=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('action', choices=('extract', 'restore-modes', 'check', 'prepare-morai', 'relocate-morai'))
    parser.add_argument('--allow-short-home-morai', action='store_true',
                        help='Only with separate user approval for HOME/MoraiLauncher_Lin')
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    root = scope(args.workspace)
    if not args.run:
        print(json.dumps(dict(action=args.action, workspace=str(root), execute=False, morai_started=False)))
        return
    start = time.monotonic()
    if args.action == 'relocate-morai':
        result = relocate_morai(root, args.allow_short_home_morai)
    else:
        result = {'extract': extract, 'restore-modes': restore_modes,
                  'check': check, 'prepare-morai': prepare_morai}[args.action](root)
    result.update(action=args.action, elapsed_sec=round(time.monotonic()-start,3),
                  checked_at=datetime.datetime.now(datetime.timezone.utc).isoformat(), workspace=str(root))
    receipt = root / 'checks' / (args.action + '-' + str(time.time_ns()) + '.json')
    if receipt.parent.is_symlink():
        raise ValueError('Receipt directory symlink is forbidden')
    with receipt.open('x') as stream:
        json.dump(result, stream, indent=2)
    summary = {k: v for k, v in result.items() if k not in ('changes', 'directories')}
    print(json.dumps(dict(receipt=str(receipt), **summary), ensure_ascii=False), flush=True)
    if result.get('pass_all') is False:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
