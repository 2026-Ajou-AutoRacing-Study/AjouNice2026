#!/usr/bin/env bash
# User-operated desktop launcher. Never invoke MORAISim.sh or a system installer.
set -euo pipefail
exec /usr/bin/python3 -B - "${BASH_SOURCE[0]}" "$@" <<'PY'
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys

WORKSPACE_NAME = 'ajounice_base5_capture'
VERSION = 'Simulator_v.S4.251001.MolitComp03_Linux'
CORE = ('MoraiLauncher_Lin.x86_64', 'UnityPlayer.so',
        'MoraiLauncher_Lin_Data/Project/Simulator/' + VERSION + '/Simulator.x86_64',
        'MoraiLauncher_Lin_Data/Project/Simulator/' + VERSION + '/UnityPlayer.so')


def no_symlink(path):
    path = Path(path)
    if not path.is_absolute() or '..' in path.parts:
        raise ValueError('Absolute, normalized paths are required')
    for component in (path, *path.parents):
        if component.is_symlink():
            raise ValueError('Symlink path is not allowed: ' + str(component))
    return path


def owned_directory(path):
    path = no_symlink(path)
    if not path.is_dir() or path.stat().st_uid != os.geteuid():
        raise ValueError('Expected an existing directory owned by the current user: ' + str(path))
    if stat.S_IMODE(path.stat().st_mode) & 0o022:
        raise ValueError('Group/world-writable workspace directories are not allowed: ' + str(path))
    return path


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def owned_file(path, executable=False):
    path = no_symlink(path)
    info = path.stat()
    if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid():
        raise ValueError('Expected a regular user-owned file: ' + str(path))
    if stat.S_IMODE(info.st_mode) & 0o022:
        raise ValueError('Group/world-writable executable/input is not allowed: ' + str(path))
    if executable and not os.access(path, os.X_OK):
        raise ValueError('Executable permission is missing: ' + str(path))
    return path


def discover_workspace(script):
    script = no_symlink(Path(script).absolute())
    for parent in script.parents:
        if parent.name == WORKSPACE_NAME:
            return parent
    raise ValueError('Use --workspace with the explicitly approved workspace directory')


def make_plan(workspace, inherited=None):
    inherited = dict(os.environ if inherited is None else inherited)
    workspace = owned_directory(Path(workspace).absolute())
    if workspace.name != WORKSPACE_NAME:
        raise ValueError('Workspace basename must be ' + WORKSPACE_NAME)
    bundle = owned_directory(workspace / 'capture_bundle')
    work = owned_directory(workspace / 'work')
    mr = owned_directory(work / 'morai/MoraiLauncher_Lin')
    original = owned_directory(bundle / 'morai/MoraiLauncher_Lin')
    bwrap = owned_file(bundle / 'bin/bwrap', executable=True)
    hashes = {}
    for relative in CORE:
        source = owned_file(original / relative, executable=relative.endswith('.x86_64'))
        target = owned_file(mr / relative, executable=relative.endswith('.x86_64'))
        expected = digest(source)
        same_inode = (source.stat().st_dev, source.stat().st_ino) == (target.stat().st_dev, target.stat().st_ino)
        if digest(target) != expected or same_inode:
            raise ValueError('Working MORAI core differs or is hardlinked to the package: ' + relative)
        hashes[relative] = expected
    session = no_symlink(work / 'morai-session')
    folders = dict(home=session / 'home', config=session / 'config', cache=session / 'cache',
                   data=session / 'data', state=session / 'state', tmp=session / 'tmp',
                   shader=session / 'cache/nvidia', cuda=session / 'cache/cuda',
                   vulkan=session / 'cache/vulkan', mesa=session / 'cache/mesa')
    for folder in (session, *folders.values()):
        no_symlink(folder)
        if folder.exists():
            owned_directory(folder)
    display = inherited.get('DISPLAY', '')
    match = re.fullmatch(r'(?:unix)?:(\d+)(?:\.\d+)?', display)
    x11 = Path('/tmp/.X11-unix')
    gui_ready = bool(match and x11.is_dir() and (x11 / ('X' + match.group(1))).exists())
    authority = inherited.get('XAUTHORITY')
    if not authority:
        default_authority = Path(inherited.get('HOME', '/nonexistent')) / '.Xauthority'
        if default_authority.is_file():
            authority = str(default_authority)
    if authority and (not Path(authority).is_absolute() or not Path(authority).is_file()):
        raise ValueError('XAUTHORITY must be an existing absolute file when set')
    environment = {key: value for key, value in inherited.items() if key in
                   ('DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS',
                    'LANG', 'LC_ALL', 'LC_CTYPE', 'WAYLAND_DISPLAY')}
    if authority:
        environment['XAUTHORITY'] = authority
    environment.update(PATH='/usr/local/bin:/usr/bin:/bin', HOME=str(folders['home']),
        XDG_CONFIG_HOME=str(folders['config']), XDG_CACHE_HOME=str(folders['cache']),
        XDG_DATA_HOME=str(folders['data']), XDG_STATE_HOME=str(folders['state']),
        TMPDIR=str(folders['tmp']), TMP=str(folders['tmp']), TEMP=str(folders['tmp']),
        CUDA_CACHE_PATH=str(folders['cuda']), __GL_SHADER_DISK_CACHE_PATH=str(folders['shader']),
        MESA_SHADER_CACHE_DIR=str(folders['mesa']), VK_SHADER_CACHE_PATH=str(folders['vulkan']))
    command = [str(bwrap), '--unshare-user', '--unshare-pid', '--die-with-parent', '--new-session',
        '--ro-bind', '/', '/', '--proc', '/proc', '--dev-bind', '/dev', '/dev', '--tmpfs', '/tmp',
        '--dir', '/tmp/.X11-unix', '--ro-bind', str(x11), str(x11),
        '--bind', str(workspace), str(workspace), '--chdir', str(mr), '--',
        str(mr / 'MoraiLauncher_Lin.x86_64'), '-logFile', str(session / 'launcher.log')]
    return dict(schema='base5_borrowed_morai_desktop_plan_v1', workspace=str(workspace),
        working_morai=str(mr), session=str(session), directories=[str(p) for p in folders.values()],
        core_sha256=hashes, command=command, environment=environment, gui_ready=gui_ready,
        display=display, filesystem_policy='host root read-only; approved workspace read-write; private /tmp',
        fallback_allowed=False, ros_started=False, capture_started=False, morai_started=False,
        note='Local desktop X11/XWayland is required. Device/socket access is not a general security guarantee.')


def run_plan(plan):
    if not plan['gui_ready']:
        raise ValueError('Run this command in the borrowed PC graphical desktop terminal; local DISPLAY/X11 socket is required')
    session = Path(plan['session'])
    for folder in (session, *map(Path, plan['directories'])):
        folder.mkdir(parents=True, exist_ok=True, mode=0o700)
        owned_directory(folder)
    # Failure is deliberately terminal. No unconfined retry, sudo, installer,
    # ROS process, or automatic capture is permitted here.
    os.execve(plan['command'][0], plan['command'], plan['environment'])


def main(script, argv=None):
    parser = argparse.ArgumentParser(description='Check or explicitly start the bundled MORAI launcher in the approved workspace')
    parser.add_argument('--workspace', type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='Read-only checks and plan (default); create no files')
    mode.add_argument('--plan', action='store_true', help='Same read-only behavior as --check')
    mode.add_argument('--run', action='store_true', help='User-only desktop execution; never starts ROS or capture')
    args = parser.parse_args(argv)
    plan = make_plan(args.workspace if args.workspace else discover_workspace(script))
    if args.run:
        run_plan(plan)
    else:
        print(json.dumps(plan, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main(sys.argv[1], sys.argv[2:])
    except (OSError, ValueError) as error:
        print(json.dumps(dict(event='morai_launch_blocked', error=str(error), morai_started=False,
                              fallback_allowed=False), ensure_ascii=False), file=sys.stderr)
        sys.exit(1)
PY
