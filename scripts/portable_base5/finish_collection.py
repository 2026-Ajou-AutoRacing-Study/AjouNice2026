#!/usr/bin/env python3
"""Explicit CPU-only finish for the borrowed host's completed Sunny13 5K.

Default mode reads the plan only. No ROS/MORAI startup, install, transfer,
deletion, or automatic invocation after capture. Original bundle flags and
source PNG/native files stay unchanged. Full-host 5K finish remains a separate
qualification from the three-frame native capture smoke.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import uuid

try:
    import live_v2 as live
except ModuleNotFoundError as error:
    if error.name != 'live_v2':
        raise
    import live

SCHEMA = 'base5_borrowed_cpu_finish_intent_v1'
COUNT = 5000
LOGICAL_REPO = Path('/home/luv/AjouNice2026')


def paths(ctx):
    collection = ctx['workspace'] / 'collections/sunny13_5k'
    return dict(collection=collection, selected=collection / 'selected_ledger.jsonl',
                output=ctx['workspace'] / 'finished/sunny13_5k',
                session=ctx['workspace'] / 'work/finish-session',
                source=Path(ctx['graph']['manifest']['source']),
                sensor=LOGICAL_REPO / 'configs/sensors/Sensor_competition_local.json')


def terminal(collection):
    """No paused, incomplete, deficit-bearing or still-running parent is eligible."""
    collection = live.owned(collection)
    status = live.read(live.owned(collection / 'status.json', False))
    quota = live.read(live.owned(collection / 'quota.json', False))
    live.require(status.get('stage') == 'complete'
                 and status.get('requested') == COUNT and status.get('finished') == COUNT
                 and status.get('accepted_candidate') == COUNT
                 and status.get('review') == 0 and status.get('failed') == 0,
                 'CPU finish requires complete 5000/5000 accepted native scenes')
    live.require(quota.get('target') == COUNT and quota.get('accepted') == COUNT
                 and quota.get('deficit') == 0 and quota.get('pending') == 0
                 and not quota.get('cells'), 'Unfilled quota: do not finish or bypass recovery')
    return dict(status_sha256=live.digest(collection / 'status.json'),
                quota_sha256=live.digest(collection / 'quota.json'),
                selected_sha256=live.digest(collection / 'selected_ledger.jsonl'))


def meter_roots(ctx, exclusions=None):
    """Partition both approved folders around explicitly known owned aliases.

    The legacy Budget rejects symlinks inside a scanned root. Partition only
    branches containing two runtime HOME aliases and ROS log/latest pointers.
    Never follow an arbitrary symlink. Actual MORAI/log bytes are scanned once
    through real directories. Allocated symlink blocks are separately reserved
    against the same 100 GiB limit, not silently omitted from the disk budget.
    """
    workspace, mr = ctx['workspace'], ctx['mr']
    mr_aliases = {workspace / 'work' / name / 'MoraiLauncher_Lin'
                  for name in ('ros-home', 'ros-home-v2')}
    log_aliases = {workspace / 'work' / name / 'log/latest'
                   for name in ('ros-session', 'ros-session-v2')}
    aliases = mr_aliases | log_aliases
    exclusions = [] if exclusions is None else exclusions
    result = []

    def visit(path):
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode):
            live.require(path in aliases and info.st_uid == os.getuid(),
                         'Unexpected symlink in finish budget: ' + str(path))
            target = path.resolve(strict=True)
            if path in mr_aliases:
                live.require(target == mr, 'Unexpected MORAI budget alias target')
                kind = 'owned_runtime_morai_alias'
            else:
                try:
                    canonical = str(uuid.UUID(target.name)) == target.name
                except ValueError:
                    canonical = False
                live.require(target.parent == path.parent and canonical,
                             'ROS latest must target a canonical UUID directory in the same log folder')
                kind = 'owned_ros_latest_uuid_pointer'
            live.owned(target)
            exclusions.append(dict(path=str(path), target=str(target), kind=kind,
                                   allocated_bytes=info.st_blocks * 512))
            return
        live.require(stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode),
                     'Non-regular finish budget path: ' + str(path))
        if path.is_dir() and any(path == a or path in a.parents for a in aliases):
            for child in sorted(path.iterdir()):
                visit(child)
        else:
            result.append(path)

    visit(workspace)
    result.append(mr)
    return result


def intent(ctx):
    p = paths(ctx)
    helper, finisher = Path(__file__).resolve(), Path(__file__).with_name('finish.py').resolve()
    live.owned(helper, False)
    live.owned(finisher, False)
    exclusions = []
    meter_roots(ctx, exclusions)
    return dict(schema=SCHEMA, workspace=str(ctx['workspace']), morai=str(ctx['mr']),
                manifest_sha256=live.DECK_SHA, release_sha256=live.RELEASE_SHA,
                host_contract=live.contract(ctx), helper_sha256=live.digest(helper),
                live_helper_sha256=live.digest(live.__file__),
                finisher_sha256=live.digest(finisher), expected_scenes=COUNT,
                paths={key: str(value) for key, value in p.items()},
                native_parent=terminal(p['collection']), max_usage_gib=100., min_free_gib=10.,
                budget_alias_exclusions=exclusions,
                original_png_native_preserved=True, remote_delete=False, network_transfer=False)


def save_unchanged(path, document):
    """New derived file or exact-byte JSON retry; never overwrite existing data."""
    if path.exists():
        live.require(live.read(live.owned(path, False)) == document,
                     'Existing CPU finish input differs; preserve it and inspect')
    else:
        live.write_new(path, document)


def load_finisher():
    spec = importlib.util.spec_from_file_location('borrowed_frozen_finisher',
                                                Path(__file__).with_name('finish.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def finish_inside(document, intent_path):
    """Runs only in the original-path namespace selected by the outer wrapper."""
    workspace, mr = Path(document['workspace']), Path(document['morai'])
    live.require(document.get('schema') == SCHEMA and document.get('expected_scenes') == COUNT
                 and document.get('manifest_sha256') == live.DECK_SHA
                 and document.get('release_sha256') == live.RELEASE_SHA,
                 'Unexpected borrowed finish contract')
    ctx = dict(workspace=workspace, mr=mr, graph=dict(manifest=dict(
        source=str(LOGICAL_REPO / 'artifacts/evaluation/base5_unit_release_261017/'
                   'close_primary_fullsource_v1/candidate_v158a/manifest.json'))))
    p = paths(ctx)
    live.require(document['paths'] == {key: str(value) for key, value in p.items()},
                 'CPU finish scope or input paths differ')
    live.require(Path(intent_path) == p['session'] / 'intent.json', 'Unexpected finish intent location')
    live.require(document['helper_sha256'] == live.digest(__file__)
                 and document['finisher_sha256'] == live.digest(Path(__file__).with_name('finish.py'))
                 and document['live_helper_sha256'] == live.digest(live.__file__),
                 'CPU finish helper changed')
    live.require(terminal(p['collection']) == document['native_parent'], 'Native parent changed')
    live.require(live.digest(p['source']) == live.DECK_SHA, 'Frozen source manifest changed')
    finisher = load_finisher()
    finisher.ROOT, finisher.SCRIPTS = LOGICAL_REPO, LOGICAL_REPO / 'dataset/morai_od/scripts'
    sys.path.insert(0, str(finisher.SCRIPTS))
    from run_base5_fresh_completion import verify_parent, adopted_entry
    from run_base5_capture_campaign import subset_deck
    rows = finisher.selected_rows(p['selected'], COUNT)
    verified, missing, elapsed = verify_parent(p['source'], p['collection'])
    live.require(verified == rows and not missing, 'Native parent reconstruction differs')
    source = live.read(p['source'])
    lookup = {entry['id']: entry for entry in source['scenes']}
    adopted = subset_deck(source, [adopted_entry(row, lookup) for row in rows])
    live.require(len(adopted['scenes']) == adopted['count'] == COUNT,
                 'Incomplete native adopted manifest')
    adopted_path = p['session'] / 'adopted_manifest.json'
    save_unchanged(adopted_path, adopted)
    # Keep the budget's root partition stable across first execution/resume.
    # The finisher creates descendants here, never a new top-level meter root.
    live.checked_folder(p['output'].parent)
    exclusions = []
    budget_roots = meter_roots(ctx, exclusions)
    live.require(exclusions == document['budget_alias_exclusions'], 'Budget alias mapping changed')
    reserved_alias_bytes = sum(item['allocated_bytes'] for item in exclusions)
    effective_maximum = 100. - reserved_alias_bytes / 1024**3
    live.require(effective_maximum > 0, 'Budget exhausted by allocated alias blocks')
    arguments = ['finish', '--repo', str(LOGICAL_REPO), '--source', str(p['source']),
                 '--adopted', str(adopted_path), '--selected', str(p['selected']),
                 '--sensor', str(p['sensor']), '--output', str(p['output']),
                 '--expected-scenes', str(COUNT), '--max-usage-gib', repr(effective_maximum),
                 '--min-free-gib', '10']
    for root in budget_roots:
        arguments += ['--usage-root', str(root)]
    result = finisher.finish(finisher.parser().parse_args(arguments + ['--run']))
    live.require(terminal(p['collection']) == document['native_parent'], 'Native parent mutated')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home() / live.WORKSPACE)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--inner-intent', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--intent-sha256', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.inner_intent is not None:
        live.require(args.run and args.intent_sha256
                     and os.environ.get('BASE5_FINISH_INTENT_SHA256') == args.intent_sha256
                     and live.digest(args.inner_intent) == args.intent_sha256
                     and live.digest('/run/base5-bundle/portable_bundle.json') == live.MANIFEST_SHA,
                     'Explicit namespaced CPU finish invocation required')
        print(json.dumps(finish_inside(live.read(args.inner_intent), args.inner_intent),
                         ensure_ascii=False, indent=2))
        return 0
    ctx = live.load(args.workspace)
    live.ready(ctx)
    live.verify_qualification(ctx)
    p = paths(ctx)
    if not args.run:
        exclusions = []
        meter_roots(ctx, exclusions)
        print(json.dumps(dict(event='borrowed_cpu_finish_plan_only',
              collection=str(p['collection']), output=str(p['output']), expected_scenes=COUNT,
              requires='complete; native reconstruction; 5000 accepted; zero deficit; no active capture',
              max_usage_gib=100, min_free_gib=10, source_png_native_preserved=True,
              budget_alias_exclusions=exclusions,
              budget_reserved_alias_bytes=sum(item['allocated_bytes'] for item in exclusions),
              files_written=False, capture_started=False, network_transfer=False,
              automatic_invocation=False, full_5k_finish_live_qualified=False),
              ensure_ascii=False, indent=2))
        return 0
    # Reuse the collector's own lock: no postprocessing of a still-running
    # parent, and no concurrent new capture while the selected inputs are read.
    finisher = load_finisher()
    capture_lock = live.no_symlinks(ctx['session'] / 'capture.lock')
    if capture_lock.exists():
        live.owned(capture_lock, False)
    with finisher.exclusive(capture_lock):
        expected = intent(ctx)
        live.checked_folder(p['session'])
        record = p['session'] / 'intent.json'
        save_unchanged(record, expected)
        sha = live.digest(record)
        command = ['/usr/bin/python3', '-B', str(Path(__file__).resolve()),
                   '--inner-intent', str(record), '--intent-sha256', sha, '--run']
        env = live.environment(ctx)
        env['BASE5_FINISH_INTENT_SHA256'] = sha
        result = subprocess.run(live.namespace(ctx, command), env=env, check=False)
        return result.returncode


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='borrowed_cpu_finish_blocked', error=str(error),
                              source_preserved=True, network_transfer=False), ensure_ascii=False),
              file=sys.stderr)
        raise SystemExit(1)
