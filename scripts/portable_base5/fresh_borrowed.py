#!/usr/bin/env python3
"""빌린 PC의 기존 ROS 플랫폼 위에서 독립 새 환경을 격리 실행한다.

기존 Sunny13 코드·source·출력은 수정하지 않는다. source-only load/probe는
새 환경 인증이 아니다. 원형 소량 검증 → 새 release → 공통 runtime 자격을
각각 분리하고, 실제 촬영에는 해당 환경의 명시 확인을 요구한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import signal
import stat
import subprocess
import time

SCHEMA = 'base5_fresh_borrowed_deployment_v1'
WORKSPACE = 'ajounice_base5_capture'
LIVE_SHA = '7850aed3a4a2881393af75ed2da96752e8fe292606c837abd5790281e481fd00'
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
PROJECT = Path('/home/luv/AjouNice2026')
RAW = Path('/home/luv/morai_cap')
ASSETS = Path('/home/luv/MoraiLauncher_Lin')
SCRIPTS = PROJECT/'dataset/morai_od/scripts'
REQUIRED_HELPERS = {'fresh_borrowed.py', 'fresh_route_compat.py', 'fresh_route_runner.py',
    'fresh_environment_qualification.py',
    'unified_capture.py', 'session_input_boundary.py', 'unified_paused_import.py',
    'recovery_import.py', 'recovery_population.py', 'route_compat.py'}
REQUIRED_RUNTIME = {'run_base5_batch_static_attitude.py', 'run_base5_capture_static_attitude.py',
    'run_base5_capture_campaign_direction_reference.py', 'run_base5_capture.py', 'generate_base5_independent.py'}


def require(value, message):
    if not value:raise ValueError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink physical path required: '+str(path))
    return path


def logical(value):
    require(type(value) is str and value.startswith('/') and PurePosixPath(value).as_posix() == value
        and '..' not in PurePosixPath(value).parts, 'Exact absolute logical path required')
    return Path(value)


def inside(path, root):return path == root or root in path.parents


def owned(value, directory=False):
    path = physical(value); info = path.stat()
    require(info.st_uid == os.getuid() and not info.st_mode & 0o022
        and (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode)),
        'Private owned regular deployment input required: '+str(path))
    return path


def sha(value):
    result = hashlib.sha256()
    with physical(value).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024), b''):result.update(block)
    return result.hexdigest()


def read(value):return json.loads(owned(value).read_text())


def load_original(workspace):
    filename = owned(workspace/'checks/live_v2.py')
    require(sha(filename) == LIVE_SHA, 'Historical ROS platform helper bytes changed')
    spec = importlib.util.spec_from_file_location('_base5_original_platform_for_fresh', filename)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def inventory(root, pins, label):
    root = owned(root, True)
    require(type(pins) is dict and pins and {str(p) for p in root.iterdir()} == set(pins),
            'Exact flat '+label+' inventory required')
    for name, digest in pins.items():
        filename = owned(name)
        require(filename.parent == root and filename.suffix == '.py' and type(digest) is str
            and len(digest) == 64 and sha(filename) == digest, label+' SHA changed: '+name)


def base_bindings(ctx):
    return [(physical(ctx['bundle']/row['bundle_path']), logical(row['logical_root']))
            for row in ctx['graph']['bindings']]


def mapped(ctx, filename, *, include_new=True):
    target = logical(filename)
    mappings = base_bindings(ctx)
    if include_new:
        mappings += [(physical(row['source']), logical(row['target'])) for row in ctx['deployment']['bindings']]
        mappings += [(ctx['paths']['scripts'], SCRIPTS)]
    # Later explicit bindings shadow the platform's read-only source view.
    matches = [(index, source, root) for index,(source, root) in enumerate(mappings) if inside(target, root)]
    if matches:
        _, source, root = max(matches, key=lambda row:(len(row[2].parts), row[0]))
        return physical(source/target.relative_to(root))
    if inside(target, ctx['workspace']):return physical(target)
    # Native adoption records the simulator's actual asset JSON location.
    # Permit only the already approved simulator tree; verify_pins still
    # checks the exact file SHA, and namespace does not add any new mount.
    if inside(target, ctx['mr']):return physical(target)
    raise ValueError('Logical source escaped the isolated input roots: '+filename)


def verify_pins(ctx, pins):
    require(type(pins) is dict and pins, 'Nonempty new-source input pins required')
    for filename, expected in pins.items():
        require(type(expected) is str and len(expected) == 64 and sha(mapped(ctx, filename)) == expected,
                'Fresh logical input changed: '+filename)


def load(deployment, environment, *, verify=True):
    deployment = owned(deployment)
    root = owned(deployment.parent, True); document = read(deployment)
    workspace = owned(document['workspace'], True)
    require(workspace.name == WORKSPACE and workspace.parent == Path.home().resolve()
        and inside(root, workspace/'work') and root != workspace/'work', 'Approved HOME work deployment required')
    require(document.get('schema') == SCHEMA and environment in ENVIRONMENTS
        and document.get('environment') == environment, 'Exact independent environment deployment required')
    p = dict(root=root, checks=root/'checks', scripts=root/'scripts', deployment=deployment,
             profile=root/'profile.json', output=physical(document['output']))
    require(physical(__file__) == p['checks']/'fresh_borrowed.py', 'Run the separately deployed fresh helper')
    require(p['output'].parent == workspace/'collections'
        and p['output'].name.startswith(environment+'_'), 'New output must belong to its explicit environment')
    inventory(p['checks'], document.get('helper_hashes'), 'fresh helpers')
    inventory(p['scripts'], document.get('runtime_hashes'), 'fresh scripts')
    require(REQUIRED_HELPERS <= {Path(name).name for name in document['helper_hashes']}, 'Required fresh helpers missing')
    bindings = document.get('bindings')
    require(type(bindings) is list and bindings and len(bindings) <= 16, 'Explicit bounded new-source bindings required')
    targets = []
    for row in bindings:
        require(type(row) is dict and set(row) == {'source','target'}, 'Exact read-only source binding required')
        source, target = owned(row['source'], True), logical(row['target'])
        require(inside(source, root) and source not in (root,p['checks'],p['scripts'])
            and any(inside(target,allowed) for allowed in (PROJECT,RAW,ASSETS)) and target != SCRIPTS,
            'New data binding escaped the separately deployed source directories')
        require(not any(inside(target, prior) or inside(prior,target) for prior in targets), 'Source bindings overlap')
        targets.append(target)
    old = load_original(workspace)
    ctx = old.load(workspace, verify=verify);old.ready(ctx)
    # These are unchanged platform/ROS service identities, not a Sunny11 native certificate.
    ctx = dict(ctx, old_live=old, deployment=document, environment=environment, paths=p)
    source_runtime = document.get('source_runtime_hashes')
    require(type(source_runtime) is dict and source_runtime, 'Actual new runtime closure required')
    expected_scripts = {str(p['scripts']/Path(name).name):digest for name,digest in source_runtime.items()
                        if logical(name).parent == SCRIPTS}
    require(document['runtime_hashes'] == expected_scripts
        and REQUIRED_RUNTIME <= {Path(name).name for name in source_runtime}, 'Runtime overlay differs from new source closure')
    if verify:
        verify_pins(ctx, document.get('input_hashes'))
        verify_pins(ctx, source_runtime)
    manifest = logical(document['manifest'])
    require(document.get('input_hashes', {}).get(str(manifest)) == document.get('manifest_sha256')
        and sha(mapped(ctx,str(manifest))) == document['manifest_sha256'], 'Exact new manifest pin required')
    deck = read(mapped(ctx,str(manifest)))
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000
        and deck.get('config', {}).get('environment_tag') == environment, 'Fresh source must be its own complete 5K environment')
    witness = document.get('route_witness')
    require(type(witness) is dict and set(witness) == {'path','sha256'}
        and sha(mapped(ctx,witness['path'])) == witness['sha256'], 'Explicit new-environment route witness required')
    witness_doc = read(mapped(ctx,witness['path']))
    require(witness_doc.get('schema') == 'base5_fresh_environment_route_witness_v1'
        and witness_doc.get('manifest_sha256') == document['manifest_sha256']
        and witness_doc.get('environment') == environment, 'Route witness belongs to another environment/source')
    if document.get('release') is not None:
        require(sha(mapped(ctx,document['release'])) == document.get('release_sha256'), 'Fresh release SHA changed')
    return ctx


def environment(ctx):
    result = ctx['old_live'].environment(ctx)
    result['PYTHONPATH'] = str(ctx['paths']['checks'])+':'+result['PYTHONPATH']
    return result


def merged_mounts(ctx):
    """Missing nested mountpoints live on a namespace tmpfs, not in the old bundle."""
    pivots = {}
    for row in ctx['deployment']['bindings']:
        target = logical(row['target'])
        original = mapped(ctx,str(target),include_new=False)
        if original.exists():continue
        pivot = target.parent
        while not mapped(ctx,str(pivot),include_new=False).is_dir():pivot = pivot.parent
        pivots[pivot] = mapped(ctx,str(pivot),include_new=False)
    selected = [(target,source) for target,source in sorted(pivots.items(),key=lambda row:len(row[0].parts))
                if not any(prior != target and inside(target,prior) for prior in pivots)]
    mounts = []
    for target, source in selected:
        mounts += ['--tmpfs',str(target)]
        for entry in sorted(source.iterdir()):
            physical(entry)
            mounts += ['--ro-bind',str(entry),str(target/entry.name)]
    return mounts


def namespace(ctx, command):
    argv = ctx['old_live'].namespace(ctx,command); index = argv.index('--chdir')
    p = ctx['paths'];mounts = merged_mounts(ctx)
    for row in ctx['deployment']['bindings']:
        mounts += ['--ro-bind',row['source'],row['target'], '--ro-bind',row['source'],row['source']]
    mounts += ['--ro-bind',str(p['scripts']),str(SCRIPTS), '--ro-bind',str(p['scripts']),str(p['scripts']),
        '--ro-bind',str(p['checks']),str(p['checks']), '--ro-bind',str(p['deployment']),str(p['deployment']),
        '--ro-bind',str(ctx['workspace']/'checks'),str(ctx['workspace']/'checks')]
    # All earlier collections, including completed Sunny13, remain read-only.
    for folder in sorted((ctx['workspace']/'collections').iterdir()):
        physical(folder)
        require(folder.is_dir() or folder.is_file(), 'Unexpected collection entry type')
        if folder != p['output']:mounts += ['--ro-bind',str(folder),str(folder)]
    for name in ('host_contract.json','qualification.json'):
        path = ctx['session']/name;mounts += ['--ro-bind',str(path),str(path)]
    for value in ctx['deployment'].get('preserved_roots', []):
        path = owned(value, True)
        require(inside(path, ctx['workspace']/'work') and not inside(p['root'],path)
            and not inside(path,p['root']), 'Preserved work evidence must be a separate prior root')
        mounts += ['--ro-bind',str(path),str(path)]
    return argv[:index]+mounts+argv[index:]


def validate_profile(ctx):
    doc = read(ctx['paths']['profile']); deployment = ctx['deployment']
    require(doc.get('fresh_start') is True and doc.get('environment') == ctx['environment']
        and doc.get('manifest') == deployment['manifest'] and doc.get('manifest_sha256') == deployment['manifest_sha256']
        and doc.get('release') == deployment.get('release') and doc.get('release_sha256') == deployment.get('release_sha256')
        and doc.get('output') == str(ctx['paths']['output']) and doc.get('route_witness') == deployment['route_witness'],
        'Prepared profile escaped its fresh dataset/environment/source binding')
    return doc


def indices(value):
    require(type(value) is str and value, 'Explicit bounded qualification indices required')
    words=value.split(',')
    require(all(w.isascii() and w.isdecimal() for w in words), 'Canonical decimal source indices required')
    values=list(map(int,words))
    require(2 <= len(values) <= 8 and len(values) == len(set(values))
        and all(0 <= n < 5000 for n in values), 'Common qualification requires 2..8 unique source indices')
    return ','.join(map(str,values))


def command(ctx, action, selected=None):
    p, doc = ctx['paths'],ctx['deployment']
    argv=['/usr/bin/python3','-B',str(p['checks']/'unified_capture.py'),'--profile',str(p['profile'])]
    if action == 'probe':
        return ['/usr/bin/python3','-B','-c',
            'import json,unified_capture as c;c.modules();print(json.dumps(dict(event="fresh_namespace_import_pass",source_qualified=False,captures_requested=0)))']
    if action == 'prepare':
        require(doc.get('release') and not p['profile'].exists() and not p['output'].exists(),
                'Fresh separately qualified release and unused output/profile required')
        return argv+['--prepare-fresh','--manifest',doc['manifest'],'--release',doc['release'],
            '--output',str(p['output']),'--route-witness',doc['route_witness']['path']]
    require(action in ('plan','qualify','run'), 'Unknown fresh action')
    validate_profile(ctx)
    if action == 'plan':return argv
    if action == 'qualify':return argv+['--qualify-prototype','--indices',indices(selected),
        '--confirmed-environment',ctx['environment']]
    require(selected is None, 'Bulk cannot select prototype indices')
    return argv+['--run','--confirmed-environment',ctx['environment']]


def require_services(ctx):
    old=ctx['old_live']
    for role,port in (('ros',11311),('bridge',9090)):
        require(old.port_open(port) and old.service_owned(ctx,role), 'Existing approved '+role+' service required')


def build_prototype_intent(ctx, subset, subset_sha256, output, *, limit, chunk_size=16):
    """정식 새 환경 인증 전의 원형 <=48장 시험만 명시적으로 묶는다. 쓰기는 없다."""
    doc=ctx['deployment'];subset=logical(str(subset));output=physical(output)
    require(type(limit) is int and 1 <= limit <= 48 and type(chunk_size) is int and 1 <= chunk_size <= 16,
            'Prequalification is bounded to 1..48 native scenes')
    require(inside(output,ctx['paths']['root']/'qualification') and output != ctx['paths']['output'],
            'Prototype raw must stay in its separate qualification directory')
    require(sha(mapped(ctx,str(subset))) == subset_sha256, 'Prototype subset SHA changed')
    deck=read(mapped(ctx,str(subset)));canonical=read(mapped(ctx,doc['manifest']))
    require(deck.get('count') == len(deck.get('scenes',[])) == limit
        and deck.get('config',{}).get('environment_tag') == ctx['environment'], 'Exact bounded source subset required')
    rows={r['id']:r for r in canonical['scenes']};ids=[r['id'] for r in deck['scenes']]
    require(len(ids)==len(set(ids)) and all(r['id'] in rows and r==rows[r['id']] for r in deck['scenes']),
            'Prototype must use unchanged entries from the actual new canonical 5K')
    return dict(schema='base5_fresh_route_prototype_intent_v1',environment=ctx['environment'],
        canonical_manifest=doc['manifest'],canonical_manifest_sha256=doc['manifest_sha256'],
        manifest=str(subset),manifest_sha256=subset_sha256,output=str(output),limit=limit,chunk_size=chunk_size,
        route_witness=doc['route_witness'],input_hashes=dict(doc['input_hashes'],**{str(subset):subset_sha256}),
        runtime_hashes=doc['source_runtime_hashes'],helper_hashes=doc['helper_hashes'],
        scripts=str(SCRIPTS),runner=str(ctx['paths']['checks']/'fresh_route_runner.py'),
        prototype_only=True,bulk_authorized=False,source_qualified=False)


def prototype_command(ctx, intent, intent_sha256, *, resume=False, stop_after_chunks=0):
    intent=owned(intent)
    require(inside(intent,ctx['paths']['root']) and sha(intent)==intent_sha256,
            'Explicit private prototype intent SHA required')
    doc=read(intent)
    require(doc.get('environment')==ctx['environment'] and doc.get('canonical_manifest')==ctx['deployment']['manifest']
        and doc.get('canonical_manifest_sha256')==ctx['deployment']['manifest_sha256']
        and doc.get('helper_hashes')==ctx['deployment']['helper_hashes']
        and doc.get('route_witness')==ctx['deployment']['route_witness'], 'Prototype intent escaped its deployment')
    require(type(stop_after_chunks) is int and 0 <= stop_after_chunks <= 48, 'Bounded prototype stop limit required')
    values=['--manifest',doc['manifest'],'--output',doc['output'],'--environment',ctx['environment'],
        '--confirmed-environment',ctx['environment'],'--limit',str(doc['limit']),
        '--chunk-size',str(doc['chunk_size']),'--stop-after-chunks',str(stop_after_chunks),'--run']
    if resume:values.append('--resume')
    return ['/usr/bin/python3','-B',doc['runner'],'--intent',str(intent),'--intent-sha256',intent_sha256,
        '--role','prototype','--',*values]


def request_pause(ctx):
    path=physical(ctx['paths']['output']/'PAUSE')
    if path.exists():owned(path)
    else:ctx['old_live'].write_new(path,dict(reason='operator_graceful_pause',requested_at=time.time()))
    return path


def supervise(ctx, action, argv):
    old=ctx['old_live'];lock=physical(ctx['session']/'capture.lock')
    if lock.exists():owned(lock)
    fd=os.open(lock,os.O_RDWR|os.O_CREAT|os.O_APPEND|os.O_NOFOLLOW,0o600)
    with os.fdopen(fd,'a') as stream:
        fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if action in ('run','qualify','prototype'):require_services(ctx)
        previous={};child=None
        def interrupt(signum,frame):
            if child is not None and child.poll() is None:
                if action=='run':request_pause(ctx)
                else:old.signal_inner_app(child,argv,signal.SIGINT)
        for signum in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):previous[signum]=signal.signal(signum,interrupt)
        try:
            child=subprocess.Popen(namespace(ctx,argv),env=environment(ctx),start_new_session=True)
            return child.wait()
        finally:
            if child is not None and child.poll() is None:interrupt(signal.SIGINT,None);child.wait()
            for signum,handler in previous.items():signal.signal(signum,handler)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--deployment',type=Path,required=True)
    parser.add_argument('--environment',choices=ENVIRONMENTS,required=True)
    parser.add_argument('action',choices=('source-plan','plan','probe','prepare','qualify','run','status'))
    parser.add_argument('--confirmed-environment',choices=ENVIRONMENTS)
    parser.add_argument('--indices')
    args=parser.parse_args(argv)
    require((args.confirmed_environment is not None)==(args.action in ('run','qualify'))
        and (args.confirmed_environment is None or args.confirmed_environment==args.environment),
        'Capture requires its exact actual UI environment confirmation')
    require((args.indices is not None)==(args.action=='qualify'), 'Indices belong only to bounded qualification')
    ctx=load(args.deployment,args.environment,verify=args.action!='status')
    if args.action in ('source-plan','status'):
        state=ctx['paths']['output']/'status.json'
        print(json.dumps(dict(event='fresh_source_plan' if args.action=='source-plan' else 'fresh_status',
            environment=args.environment,source_qualified=False,output=str(ctx['paths']['output']),
            state=read(state) if state.is_file() else None,files_written=False,captures_started=False)))
        return 0
    argv=command(ctx,args.action,args.indices)
    if args.action=='plan':
        print(json.dumps(dict(event='fresh_capture_plan',command=argv,files_written=False,captures_started=False)))
        return 0
    if args.action=='probe':return subprocess.run(namespace(ctx,argv),env=environment(ctx),check=False).returncode
    return supervise(ctx,args.action,argv)


if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='fresh_borrowed_blocked',error=str(error),fallback_allowed=False)),file=sys.stderr)
        raise SystemExit(1)
