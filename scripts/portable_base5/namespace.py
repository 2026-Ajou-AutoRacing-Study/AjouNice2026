#!/usr/bin/env python3
"""Read-only original-path probe; intentionally cannot start capture or MORAI."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from portable_paths import verify_graph


def command_plan(bundle, python='/usr/bin/python3'):
    bundle = Path(bundle).resolve(strict=True)
    if bundle.is_symlink() or not (bundle / 'bin/bwrap').is_file():
        raise ValueError('A real, assembled bundle with bwrap is required')
    graph = json.loads((bundle / 'portable_graph.json').read_text())
    verify_graph(graph, bundle)
    # Root remains RO; actual /home and /root are hidden, not changed on the host.
    args = [str(bundle / 'bin/bwrap'), '--unshare-user', '--unshare-pid',
            '--die-with-parent', '--new-session', '--ro-bind', '/', '/',
            '--proc', '/proc', '--dev', '/dev', '--tmpfs', '/tmp',
            '--tmpfs', '/home', '--tmpfs', '/root', '--tmpfs', '/run',
            '--dir', '/home/luv', '--ro-bind', str(bundle), '/run/base5-bundle']
    for item in graph['bindings']:
        args += ['--ro-bind', str(bundle / item['bundle_path']), item['logical_root']]
    args += ['--chdir', '/run/base5-bundle', '--setenv', 'HOME', '/home/luv',
             '--setenv', 'PYTHONDONTWRITEBYTECODE', '1',
             '--setenv', 'PYTHONNOUSERSITE', '1', '--', python, '-B', '-c',
             "import hashlib,json,pathlib; g=json.loads(pathlib.Path('/run/base5-bundle/portable_graph.json').read_text()); "
             "bad=[p for p,v in g['files'].items() if hashlib.sha256(pathlib.Path(v['source']).read_bytes()).hexdigest()!=v['sha256']]; "
             "assert not bad, bad[:3]; print(json.dumps({'event':'original_namespace_probe_pass','files':len(g['files']),"
             "'ros_started':False,'capture_started':False,'host_home_hidden':True}))"]
    return args


def clean_environment():
    return {'PATH': '/usr/local/bin:/usr/bin:/bin', 'LANG': 'C.UTF-8',
            'PYTHONDONTWRITEBYTECODE': '1', 'PYTHONNOUSERSITE': '1'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--python', default='/usr/bin/python3')
    parser.add_argument('--probe', action='store_true', help='Run ONLY the read-only hash probe, never capture')
    args = parser.parse_args()
    command = command_plan(args.bundle, args.python)
    if not args.probe:
        print(json.dumps(dict(event='readonly_namespace_plan', command=command,
                              capture_enabled=False), ensure_ascii=False, indent=2))
        return
    subprocess.run(command, env=clean_environment(), check=True, timeout=300)


if __name__ == '__main__':
    main()
