#!/usr/bin/env python3
"""69장 원본은 보존하고 별도 output에서만 검증된 복구 실행을 제공한다.

원형 runtime/QA/서비스 신원은 그대로다. 새 후보 생성 guard와 명시적 인계
receipt만 추가한다. 기본 명령은 계획이며 실제 촬영에는 --run이 필요하다.
"""
import argparse
from contextlib import contextmanager
import importlib
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import time

import live_route_compat as historical
import route_dispatch as route
import route_compat as compat

SCHEMA = 'base5_borrowed_population_recovery_v1'
OUTPUT_NAME = 'sunny13_5k_recovery_v2'
INTENT_NAME = 'population_recovery_intent.json'
HELPERS = ('recovery_runtime.py', 'recovery_population.py', 'recovery_import.py',
           'recovery_qualification.py')


def require(value, message):
    if not value:
        raise ValueError(message)


def validate_profile(workspace, parent):
    """Outer-host checks use real paths; inner dispatch additionally checks the source."""
    workspace = compat.no_symlink(workspace)
    path = workspace/'checks'/INTENT_NAME
    doc = compat.read(compat.no_symlink(path))
    import recovery_population
    require(doc.get('recovery_policy') == recovery_population.POLICY,
        'The explicitly qualified roster-preserving population policy is required')
    require(doc.get('schema') == SCHEMA and doc.get('workspace') == str(workspace)
        and doc.get('output') == str(workspace/'collections'/OUTPUT_NAME)
        and doc.get('previous_output') == str(workspace/'collections/sunny13_5k')
        and doc.get('environment') == 'sunny13'
        and doc.get('manifest') == parent['manifest'] and doc.get('release') == parent['release']
        and doc.get('scripts') == parent['scripts'] and doc.get('witness') == parent['witness']
        and doc.get('host_contract_sha256') == parent['host_contract_sha256']
        and doc.get('qualification_sha256') == parent['qualification_sha256']
        and doc.get('source_runtime_count') == 427 and doc.get('qa_changed') is False
        and doc.get('new_live_smoke_executed') is False
        and doc.get('witness_sha256') == parent['witness_sha256']
        and doc.get('parent_intent_sha256') == compat.sha256(workspace/'checks/route_compat_intent.json'),
        'Explicit unchanged-source recovery profile required')
    files = doc.get('files', {})
    expected = {workspace/'checks'/name for name in HELPERS}
    require(expected <= {Path(name) for name in files}, 'Every recovery helper must be pinned')
    for filename, digest in files.items():
        p = compat.no_symlink(filename)
        require(p.parent == workspace/'checks' and p.is_file()
            and p.stat().st_uid == os.getuid() and not p.stat().st_mode & 0o022
            and compat.sha256(p) == digest, 'Recovery helper changed: '+filename)
    receipt = compat.no_symlink(doc['import_receipt'])
    require(receipt == Path(doc['output'])/'imported_prefix.json'
        and compat.sha256(receipt) == doc.get('import_receipt_sha256'), 'Imported prefix changed')
    imported = compat.read(receipt)
    require(imported.get('schema') == 1 and imported.get('manifest_sha256') == compat.CANONICAL_MANIFEST_SHA256
        and len(imported.get('selected', [])) == 69 and not imported.get('deficits')
        and len({r['source_cell_id'] for r in imported['selected']}) == 69,
        'Exactly 69 unique accepted sources must be inherited once')
    return doc


def qualification(doc):
    p = compat.no_symlink(doc['cpu_qualification'])
    workspace = Path(doc['workspace'])
    require(p == workspace/'work/recovery-qualification/v1/qualification.json'
        and compat.sha256(p) == doc.get('cpu_qualification_sha256'), 'Recovery CPU qualification changed')
    report = compat.read(p)
    require(report.get('schema') == 'base5_portable_recovery_cpu_qualification_v1'
        and report.get('passed') is True and report.get('captures_requested') == 0
        and report.get('native_capture_executed') is False
        and report.get('population_policy') == doc['recovery_policy']
        and report.get('manifest_sha256') == compat.CANONICAL_MANIFEST_SHA256,
        'Actual zero-capture recovery CPU qualification is required')
    require(report.get('problem_00082_candidate_pass') is True
        and report.get('unexpected_contract_stops') == 0 and report.get('bounded_exhaustions') == 0
        and report.get('source_inputs_unchanged') is True and report.get('native_qa_unchanged') is True
        and report.get('parent_intent_sha256') == doc['parent_intent_sha256']
        and report.get('matrix', {}).get('all_branches_selected') is True
        and report.get('source_inventory', {}).get('passed') is True
        and report.get('source_inventory', {}).get('source_scenes') == 5000
        and type(report.get('replacement_count')) is int and 2 <= report['replacement_count'] <= 24,
        'Actual stopped cell and complete preselected branch matrix must pass')
    require(report.get('helper_hashes', {}).get(str(workspace/'checks/recovery_population.py'))
        == doc['files'][str(workspace/'checks/recovery_population.py')],
        'CPU qualification must cover the actual population guard')
    require(report.get('helper_hashes', {}).get(str(workspace/'checks/recovery_qualification.py'))
        == doc['files'][str(workspace/'checks/recovery_qualification.py')],
        'CPU qualification must pin its actual verifier')
    return report


def validate_intent(path):
    path = compat.no_symlink(path)
    workspace = path.parent.parent
    require(path == workspace/'checks'/INTENT_NAME and workspace.name == 'ajounice_base5_capture',
        'Unexpected recovery intent location')
    parent = route.validate_intent(workspace/'checks/route_compat_intent.json')
    doc = validate_profile(workspace, parent)
    qualification(doc)
    return doc


def dispatcher_command(intent, role, arguments, python=None):
    return [python or sys.executable, '-B', str(Path(__file__).absolute()),
            '--intent', str(intent), '--role', role, '--', *map(str, arguments)]


@contextmanager
def routed_children(intent, doc):
    previous = route.dispatcher_command
    route.dispatcher_command = dispatcher_command
    try:
        with route.routed_popen(intent, doc, doc['scripts']):
            yield
    finally:
        changed = route.dispatcher_command is not dispatcher_command
        route.dispatcher_command = previous
        require(not changed, 'Recovery child router ownership changed')


def collection_guard(doc, intent, create=False):
    root = compat.no_symlink(doc['output'])
    path = root/'population_recovery_contract.json'
    expected = dict(schema=SCHEMA, intent_sha256=compat.sha256(intent),
        imported_prefix_sha256=doc['import_receipt_sha256'], helper_hashes=doc['files'],
        original_native_qa=True, original_bounded_recovery=True, original_shutdown_safety=True,
        previous_output_preserved=doc['previous_output'], original_source_runtime_count=427)
    if path.exists():
        require(compat.read(compat.no_symlink(path)) == expected, 'Recovery collection contract changed')
    else:
        require(create and root.is_dir() and {p.name for p in root.iterdir()} == {'imported_prefix.json'},
            'Only a fresh explicit imported prefix may gain a recovery contract')
        with path.open('x') as stream:
            json.dump(expected, stream, indent=2);stream.write('\n')


def record_entry(doc, intent, role, arguments):
    folder = (Path(doc['workspace'])/'work/recovery-finish-invocations' if role == 'finish-support'
              else Path(doc['output'])/'population_recovery_invocations')
    folder.mkdir(exist_ok=True)
    with (folder/('%s_%s_%s.json' % (time.time_ns(), os.getpid(), role))).open('x') as stream:
        json.dump(dict(role=role, pid=os.getpid(), arguments=arguments,
            intent_sha256=compat.sha256(intent), executor_sha256=compat.sha256(__file__),
            population_guard_sha256=doc['files'][str(Path(doc['workspace'])/'checks/recovery_population.py')],
            source_runtime_unchanged=True, native_qa_unchanged=True), stream, indent=2)
        stream.write('\n')


def run_role(intent, role, arguments):
    doc = validate_intent(intent)
    route.validate_arguments(role, arguments, doc)
    collection_guard(doc, intent, create=role == 'launcher')
    gen = importlib.import_module('generate_base5_independent')
    extent = importlib.import_module('base5_static_mission_extent')
    import recovery_population
    target = Path(__file__).absolute() if role == 'finish-support' else route.target_paths(doc['scripts'])[role]
    previous = sys.argv
    with compat.installed(doc['witness'], generator=gen, extent=extent,
            manifest=doc['manifest'], expected_witness_sha256=doc['witness_sha256']), \
            recovery_population.installed(), routed_children(intent, doc):
        gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
        record_entry(doc, intent, role, arguments)
        try:
            sys.argv = [str(target), *arguments]
            runpy.run_path(str(target), run_name='__main__')
            return 0
        except SystemExit as error:
            return error.code if isinstance(error.code, int) else (0 if error.code is None else 1)
        finally:
            sys.argv = previous


def protected_namespace(command, old_root):
    """The preserved halted collection remains kernel-level read-only in every child."""
    command = list(command)
    split = command.index('--chdir')
    return command[:split]+['--ro-bind', str(old_root), str(old_root)]+command[split:]


class Facade(historical.Facade):
    def __init__(self):
        super().__init__()
        self.parent = historical.Facade()
        self.__file__ = str(Path(__file__).resolve())
        self.directory = Path(self.__file__).parent
        namespace = self.old.namespace
        self.old.namespace = lambda ctx, command: protected_namespace(namespace(ctx, command),
            ctx['workspace']/'collections/sunny13_5k')

    def intent_path(self, ctx):
        return ctx['workspace']/'checks'/INTENT_NAME

    def intent(self, ctx):
        doc = validate_profile(ctx['workspace'], self.parent.intent(ctx))
        qualification(doc)
        return doc

    def verify_cpu_qualification(self, ctx, doc=None, required=True):
        return qualification(self.intent(ctx) if doc is None else doc)

    def output(self, ctx):
        return self.old.owned(self.intent(ctx)['output'])

    def contract(self, ctx):
        doc = self.intent(ctx)
        return dict(schema=SCHEMA, historical_host_contract=self.old.contract(ctx),
            executor_sha256=self.old.digest(self.__file__), intent_sha256=self.old.digest(self.intent_path(ctx)),
            helper_evidence_hashes=doc['files'], original_qualification_sha256=doc['qualification_sha256'],
            import_receipt_sha256=doc['import_receipt_sha256'], new_live_smoke_executed=False)

    def capture_command(self, ctx):
        doc = self.intent(ctx)
        original = self.old.action_command(ctx, 'capture')
        require(original[2] == str(route.LOGICAL_SCRIPTS/'launch_base5_obstacle_support.py'),
            'Unexpected original launcher')
        arguments = list(original[3:])
        index = arguments.index('--output')+1
        arguments[index] = doc['output']
        return dispatcher_command(self.intent_path(ctx), 'launcher', arguments, '/usr/bin/python3')

    def request_pause(self, ctx):
        marker = self.old.no_symlinks(self.output(ctx)/'PAUSE')
        if marker.exists():self.old.owned(marker, False)
        else:self.old.write_new(marker, dict(reason='operator_graceful_pause', requested_at=time.time(),
            semantics='finish current bounded chunk and native Park cleanup'))
        return marker

    def supervise_capture(self, ctx, command):
        previous = self.old.request_pause
        self.old.request_pause = self.request_pause
        try:return super().supervise_capture(ctx, command)
        finally:self.old.request_pause = previous

    def helper(self, name):
        module = super().helper(name)
        if name == 'control':module.output_root = self.output
        elif name == 'status':module.snapshot = lambda ctx, now=None: snapshot(ctx, self, module, now)
        elif name == 'finish_collection':
            old_paths = module.paths
            def paths(ctx):
                result = old_paths(ctx)
                collection = ctx['workspace']/'collections'/OUTPUT_NAME
                result.update(collection=collection, selected=collection/'selected_ledger.jsonl',
                    output=ctx['workspace']/'finished'/OUTPUT_NAME,
                    session=ctx['workspace']/'work/finish-session-recovery-v2')
                return result
            module.paths = paths
        return module

    def finish(self, args, ctx=None):
        helper = self.helper('finish_collection')
        if args.inner_intent is not None:
            self.old.require(args.run and args.intent_sha256
                and os.environ.get('BASE5_FINISH_INTENT_SHA256') == args.intent_sha256
                and self.old.digest(args.inner_intent) == args.intent_sha256
                and self.old.digest('/run/base5-bundle/portable_bundle.json') == self.old.MANIFEST_SHA,
                'Explicit namespaced CPU finish intent required')
            document = self.old.read(args.inner_intent)
            workspace = self.old.owned(document['workspace'])
            require(workspace == Path(args.workspace).resolve(), 'Finish workspace differs')
            self.intent(dict(workspace=workspace))
            print(json.dumps(helper.finish_inside(document, args.inner_intent), ensure_ascii=False, indent=2))
            return 0
        self.ready(ctx);self.verify_qualification(ctx)
        if not args.run:return helper.main(['--workspace', str(ctx['workspace'])])
        finisher = helper.load_finisher();lock = ctx['session']/'capture.lock'
        if lock.exists():self.old.owned(lock, False)
        with finisher.exclusive(lock):
            expected = helper.intent(ctx);p = helper.paths(ctx)
            self.old.checked_folder(p['session']);record = p['session']/'intent.json'
            helper.save_unchanged(record, expected);digest = self.old.digest(record)
            command = dispatcher_command(self.intent_path(ctx), 'finish-support',
                ['--workspace', str(ctx['workspace']), 'finish', '--inner-intent', str(record),
                 '--intent-sha256', digest, '--run'], '/usr/bin/python3')
            env = self.old.environment(ctx);env['BASE5_FINISH_INTENT_SHA256'] = digest
            return subprocess.run(self.old.namespace(ctx, command), env=env, check=False).returncode


def snapshot(ctx, facade, module, now=None):
    """Same read-only display rules, but bound to the explicitly imported output."""
    now = time.time() if now is None else now
    root = facade.output(ctx)
    view = dict(stage='waiting', accepted=0, review=0, target=5000, elapsed_sec=0,
        eta_sec=None, current=None, updated_age_sec=None, checkpoint_only=True,
        output=str(root), files_written=False, capture_started=False)
    if not (root/'status.json').exists():return view
    state = module.record(root/'status.json')
    require(state.get('requested') == 5000, 'The explicit 5K campaign is required')
    accepted = module.number(state.get('accepted_candidate'), '승인 수', True)
    review = module.number(state.get('review'), '검토 수', True)
    require(accepted+review <= 5000, 'Campaign counts exceed 5000')
    elapsed = module.number(state.get('elapsed_sec'), '경과 시간')
    updated = module.number(state.get('updated_at'), '갱신 시간')
    eta = module.number(state['eta_sec'], '예상 잔여 시간') if state.get('eta_sec') is not None else None
    require(isinstance(state.get('stage'), str), 'Status stage is missing')
    if (root/'quota.json').exists():
        quota = module.record(root/'quota.json')
        require(quota.get('target') == 5000 and sum(module.number(quota.get(k), k, True)
            for k in ('accepted', 'deficit', 'pending')) == 5000, 'Quota sum differs')
    view.update(stage=state['stage'], accepted=accepted, review=review, elapsed_sec=elapsed,
        eta_sec=eta, current=state.get('current'), updated_age_sec=max(0, now-updated),
        checkpoint_only=state.get('checkpoint_only') is True)
    return view


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ['--intent']:
        parser = argparse.ArgumentParser(description='Explicit recovery child dispatch')
        parser.add_argument('--intent', type=Path, required=True)
        parser.add_argument('--role', required=True, choices=[*route.target_paths(route.LOGICAL_SCRIPTS), 'finish-support'])
        parser.add_argument('arguments', nargs=argparse.REMAINDER)
        a = parser.parse_args(argv)
        arguments = a.arguments[1:] if a.arguments[:1] == ['--'] else a.arguments
        return run_role(a.intent, a.role, arguments)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home()/'ajounice_base5_capture')
    parser.add_argument('action', choices=['capture', 'pause', 'resume-paused', 'status', 'finish'])
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--confirmed-environment', choices=['sunny13'])
    parser.add_argument('--watch', action='store_true')
    parser.add_argument('--inner-intent', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--intent-sha256', help=argparse.SUPPRESS)
    args = parser.parse_args(argv);facade = Facade()
    if args.inner_intent is not None:
        require(args.action == 'finish', 'Inner invocation is only for CPU finish')
        return facade.finish(args)
    require(args.intent_sha256 is None, 'Inner SHA requires inner finish intent')
    ctx = facade.load(args.workspace, verify=args.action != 'status');facade.ready(ctx)
    if args.action == 'status':
        require(not args.run, 'Status is read-only')
        return facade.helper('status').main(['--workspace', str(ctx['workspace']), *(['--watch'] if args.watch else [])])
    require(not args.watch, 'Only status supports watch')
    if args.action == 'finish':return facade.finish(args, ctx)
    if not args.run:
        print(json.dumps(dict(event='population_recovery_plan_only', action=args.action,
            output=str(facade.output(ctx)), imported_accepted=69,
            command=facade.capture_command(ctx) if args.action == 'capture' else None,
            files_written=False, capture_started=False, signals_sent=False), ensure_ascii=False, indent=2))
        return 0
    if args.action in ('pause', 'resume-paused'):
        helper = facade.helper('control')
        result = helper.pause(ctx) if args.action == 'pause' else helper.resume_paused(ctx)
        if args.action == 'resume-paused':result['next_manual_command'] = [
            'bash', str(ctx['workspace']/'start_base5_collection.sh'), '--run', '--confirmed-environment', 'sunny13']
        print(json.dumps(result, ensure_ascii=False, indent=2));return 0
    require(args.confirmed_environment == 'sunny13', 'Confirm actual Sunny13 UI before capture')
    require(facade.old.port_open(11311) and facade.service_owned(ctx, 'ros')
        and facade.old.port_open(9090) and facade.service_owned(ctx, 'bridge'),
        'Keep the existing approved ROS master and bridge running')
    facade.verify_qualification(ctx)
    return facade.supervise_capture(ctx, facade.capture_command(ctx))


if __name__ == '__main__':
    try:raise SystemExit(main())
    except KeyboardInterrupt:
        print('현재 명령만 종료했다. ROS·브리지는 종료하지 않았다.')
        raise SystemExit(130)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='population_recovery_blocked', error=str(error),
            fallback_allowed=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
