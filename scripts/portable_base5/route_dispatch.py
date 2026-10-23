#!/usr/bin/env python3
"""Explicit child-process route compatibility for the immutable v158a release.

Only cumulative arc rounding is restored. Original collectors, source paths,
native QA, bounded recovery, locks and Park cleanup remain unchanged.
No capture is started without the original launcher's explicit --run.
"""
import argparse
from contextlib import contextmanager
import hashlib
import importlib
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import time

import route_compat as compat

SCHEMA = 'base5_borrowed_route_compat_intent_v1'
RELEASE_SHA = '0a3dd5c96b4522bc9d0547bed845c97ebb02d228b30ff398e1d7ed688b726180'
LOGICAL_SCRIPTS = Path('/home/luv/AjouNice2026/dataset/morai_od/scripts')


def require(value, message):
    if not value:
        raise ValueError(message)


def within(path, root):
    try:
        Path(path).relative_to(root)
        return True
    except ValueError:
        return False


def target_paths(scripts):
    scripts = Path(scripts)
    return {
        'launcher': scripts / 'launch_base5_obstacle_support.py',
        'capture-optimized': scripts / 'run_base5_capture_optimized.py',
        'campaign-spatial': scripts / 'run_base5_capture_campaign_spatial.py',
        'capture': scripts / 'run_base5_capture.py',
    }


def validate_intent(path):
    """Recheck all new and original pins even on completed resume/leaf entry."""
    path = compat.no_symlink(path)
    doc = compat.read(path)
    require(doc.get('schema') == SCHEMA, 'Route compatibility intent required')
    workspace = compat.no_symlink(doc['workspace'])
    require(path == workspace / 'checks/route_compat_intent.json'
            and workspace.name == 'ajounice_base5_capture', 'Unexpected compatibility scope')
    require(Path(doc['scripts']) == LOGICAL_SCRIPTS
            and Path(doc['output']) == workspace / 'collections/sunny13_5k'
            and doc.get('environment') == 'sunny13'
            and doc.get('source_runtime_count') == 427
            and doc.get('new_live_smoke_executed') is False,
            'Frozen route compatibility profile changed')
    own = Path(__file__).absolute()
    helpers = {own, own.with_name('route_compat.py'), own.with_name('live_route_compat.py')}
    files = doc.get('files', {})
    require(helpers.issubset({Path(key) for key in files}), 'Compatibility executable pins missing')
    for filename, expected in files.items():
        actual = compat.no_symlink(filename)
        require(within(actual, workspace / 'checks')
                or actual in (workspace / 'work/ros-session-v2/host_contract.json',
                              workspace / 'work/ros-session-v2/qualification.json'),
                'Compatibility pin outside the approved helpers/evidence')
        require(compat.sha256(actual) == expected, 'Compatibility executable/evidence changed: ' + filename)
    require(compat.sha256(workspace / 'work/ros-session-v2/host_contract.json')
            == doc['host_contract_sha256']
            and compat.sha256(workspace / 'work/ros-session-v2/qualification.json')
            == doc['qualification_sha256'], 'Original host qualification changed')
    require(compat.sha256(doc['release']) == RELEASE_SHA
            and compat.sha256(doc['manifest']) == compat.CANONICAL_MANIFEST_SHA256,
            'Original release/source changed')
    sys.path.insert(0, str(LOGICAL_SCRIPTS))
    launcher = importlib.import_module('launch_base5_obstacle_support')
    release, env = launcher.preflight(Path(doc['release']), 'sunny13')
    require(len(release['runtime_hashes']) == 427
            and env['manifest'] == doc['manifest'], 'Original runtime/source identity changed')
    gen = importlib.import_module('generate_base5_independent')
    extent = importlib.import_module('base5_static_mission_extent')
    compat.verified_witness(doc['witness'], generator=gen, extent=extent,
        manifest=doc['manifest'], expected_witness_sha256=doc['witness_sha256'])
    return doc


def dispatcher_command(intent_path, role, arguments, python=None):
    return [python or sys.executable, '-B', str(Path(__file__).absolute()),
            '--intent', str(intent_path), '--role', role, '--', *map(str, arguments)]


@contextmanager
def routed_popen(intent_path, intent, scripts):
    """Route only the three exact frozen child scripts; preserve all kwargs."""
    original = subprocess.Popen
    targets = {str(path): role for role, path in target_paths(scripts).items()
               if role != 'launcher'}
    names = {Path(path).name for path in targets}

    def routed(command, *args, **kwargs):
        if isinstance(command, (str, bytes)):
            decoded = os.fsdecode(command)
            require(not any(name in decoded for name in names),
                    'Shell-form collector invocation is not qualified')
            return original(command, *args, **kwargs)
        argv = [os.fsdecode(value) for value in command]
        index = 2 if len(argv) > 1 and argv[1] == '-B' else 1
        candidate = argv[index] if len(argv) > index else None
        if any(Path(value).name in names for value in argv):
            require(candidate is not None and Path(candidate).name in names,
                    'Unknown collector Python argument form is not qualified')
        if candidate is not None and Path(candidate).name in names:
            require(not kwargs.get('shell') and argv[0] == sys.executable
                    and candidate in targets,
                    'Foreign Python/script collector invocation is not qualified')
            command = dispatcher_command(intent_path, targets[candidate], argv[index+1:], argv[0])
        return original(command, *args, **kwargs)

    subprocess.Popen = routed
    try:
        yield
    finally:
        changed = subprocess.Popen is not routed
        subprocess.Popen = original
        require(not changed, 'Collector subprocess router ownership changed')


def option(arguments, name):
    require(arguments.count(name) == 1, 'One explicit ' + name + ' required')
    index = arguments.index(name)
    require(index+1 < len(arguments), 'Missing argument for ' + name)
    return arguments[index+1]


def validate_arguments(role, arguments, doc):
    require(not any(a.startswith('--com') for a in arguments),
            'Separate completion is not qualified by the route compatibility dispatcher')
    if role == 'finish-support':
        require('finish' in arguments, 'Explicit CPU finish role required')
        return
    output = compat.no_symlink(option(arguments, '--output'))
    root = Path(doc['output'])
    require(output == root if role == 'launcher' else within(output, root) and output != root,
            'Collector output outside the pinned collection')
    if role == 'launcher':
        require(option(arguments, '--release') == doc['release']
                and option(arguments, '--environment') == 'sunny13', 'Launcher release/environment changed')
        if '--run' in arguments:
            require(option(arguments, '--confirmed-environment') == 'sunny13', 'Actual UI confirmation required')
    else:
        source = compat.no_symlink(option(arguments, '--manifest'))
        require(source == Path(doc['manifest']) or within(source, root),
                'Leaf manifest outside pinned lineage')


def collection_guard(doc, intent_path, create=False):
    root = compat.no_symlink(doc['output'])
    guard = root / 'route_compat_contract.json'
    expected = dict(schema=SCHEMA, intent_sha256=compat.sha256(intent_path),
        witness_sha256=doc['witness_sha256'], manifest_sha256=compat.CANONICAL_MANIFEST_SHA256,
        release_sha256=RELEASE_SHA, compatibility_files=doc['files'],
        original_native_qa=True, original_bounded_recovery=True, original_shutdown_safety=True)
    if guard.exists():
        require(compat.read(compat.no_symlink(guard)) == expected, 'Collection compatibility contract changed')
    else:
        require(create and not (root / 'batch_contract.json').exists()
                and not (root / 'status.json').exists(), 'Cannot retrofit an old collection')
        root.mkdir(parents=True, exist_ok=True)
        with guard.open('x') as stream:
            json.dump(expected, stream, indent=2);stream.write('\n')
    return guard


def record_entry(doc, intent_path, role, arguments):
    # Frozen finisher inventories the entire collection. A repeated CPU
    # finish must not add another file to that already-frozen input tree.
    folder = (Path(doc['workspace']) / 'work/route-compat-finish-invocations'
              if role == 'finish-support' else Path(doc['output']) / 'route_compat_invocations')
    folder.mkdir(exist_ok=True)
    filename = folder / ('%s_%s_%s.json' % (time.time_ns(), os.getpid(), role))
    with filename.open('x') as stream:
        json.dump(dict(role=role, pid=os.getpid(), arguments=arguments,
            intent_sha256=compat.sha256(intent_path), dispatcher_sha256=compat.sha256(__file__),
            witness_sha256=doc['witness_sha256'], context_factory_installed=True,
            original_script=str(target_paths(doc['scripts']).get(role,
                Path(__file__).with_name('live_route_compat.py')))), stream, indent=2)
        stream.write('\n')


def run_role(intent_path, role, arguments):
    doc = validate_intent(intent_path)
    validate_arguments(role, arguments, doc)
    collection_guard(doc, intent_path, create=role == 'launcher')
    gen = importlib.import_module('generate_base5_independent')
    extent = importlib.import_module('base5_static_mission_extent')
    target = (Path(__file__).with_name('live_route_compat.py') if role == 'finish-support'
              else target_paths(doc['scripts'])[role])
    previous = sys.argv
    with compat.installed(doc['witness'], generator=gen, extent=extent,
            manifest=doc['manifest'], expected_witness_sha256=doc['witness_sha256']), \
            routed_popen(intent_path, doc, doc['scripts']):
        # Validate a real full context now, including leaf jobs which do not
        # otherwise need to regenerate a scenario. This is CPU-only.
        gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
        record_entry(doc, intent_path, role, arguments)
        try:
            sys.argv = [str(target), *arguments]
            runpy.run_path(str(target), run_name='__main__')
            return 0
        except SystemExit as error:
            return error.code if isinstance(error.code, int) else (0 if error.code is None else 1)
        finally:
            sys.argv = previous


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--intent', type=Path, required=True)
    parser.add_argument('--role', required=True,
        choices=[*target_paths(LOGICAL_SCRIPTS), 'finish-support'])
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    arguments = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
    return run_role(args.intent, args.role, arguments)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps(dict(event='route_compatibility_blocked', error=str(error),
            native_qa_unchanged=True), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
