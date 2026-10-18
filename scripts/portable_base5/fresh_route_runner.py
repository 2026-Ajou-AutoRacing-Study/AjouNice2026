#!/usr/bin/env python3
"""새 환경 <=48장 원형 인증 시험의 명시적 자식 route 호환 실행기.

원형 parent/collector는 변경하지 않는다. 모든 프로세스에서 같은 새 환경
witness를 검산하고 arc 호환만 설치한다. bulk·추가 보충·다른 source는 금지다.
고정 입력 반복 검사는 작은 인증 시험에서 의도적으로 유지한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
import importlib
import json
from pathlib import Path
import runpy
import subprocess

import fresh_borrowed as borrowed
import fresh_route_compat as route
import recovery_population

SCHEMA='base5_fresh_route_prototype_intent_v1'
TARGETS=dict(prototype='run_base5_batch_static_attitude.py',
    static='run_base5_capture_static_attitude.py',direction='run_base5_capture_campaign_direction_reference.py',
    capture='run_base5_capture.py')


def require(value,message):
    if not value:raise RuntimeError(message)


def validate_intent(path, expected):
    path=borrowed.owned(path)
    require(borrowed.sha(path)==expected, 'Prototype intent SHA changed')
    doc=borrowed.read(path)
    require(doc.get('schema')==SCHEMA and doc.get('environment') in borrowed.ENVIRONMENTS
        and doc.get('prototype_only') is True and doc.get('bulk_authorized') is False
        and doc.get('source_qualified') is False and doc.get('runner')==str(Path(__file__).absolute())
        and doc.get('scripts')==str(borrowed.SCRIPTS)
        and type(doc.get('limit')) is int and 1 <= doc['limit'] <= 48
        and type(doc.get('chunk_size')) is int and 1 <= doc['chunk_size'] <= 16,
        'Exact bounded fresh-source prototype scope required')
    root=borrowed.owned(Path(__file__).absolute().parent.parent,True)
    output=borrowed.physical(doc['output'])
    require(borrowed.inside(path,root) and borrowed.inside(output,root/'qualification')
        and output != root/'qualification', 'Prototype intent/output escaped the private qualification tree')
    for group in ('input_hashes','runtime_hashes','helper_hashes'):
        require(type(doc.get(group)) is dict and doc[group], 'Complete prototype executable/source pins required')
        for filename,digest in doc[group].items():
            require(borrowed.sha(filename)==digest, 'Prototype input/code changed: '+filename)
    require(doc['helper_hashes'].get(str(Path(__file__).absolute()))==borrowed.sha(__file__),
            'Actual prototype dispatcher must be pinned')
    canonical,subset=map(borrowed.read,(doc['canonical_manifest'],doc['manifest']))
    require(borrowed.sha(doc['canonical_manifest'])==doc['canonical_manifest_sha256']
        and canonical.get('count')==len(canonical.get('scenes',[]))==5000
        and canonical.get('config',{}).get('environment_tag')==doc['environment']
        and borrowed.sha(doc['manifest'])==doc['manifest_sha256']
        and subset.get('count')==len(subset.get('scenes',[]))==doc['limit']
        and subset.get('config',{}).get('environment_tag')==doc['environment'], 'Canonical/subset binding changed')
    originals={r['id']:r for r in canonical['scenes']};ids=[r['id'] for r in subset['scenes']]
    require(len(ids)==len(set(ids)) and all(row==originals.get(row['id']) for row in subset['scenes']),
            'Prototype entries differ from the new canonical source')
    return doc


def validate_arguments(role,values,doc):
    require(role in TARGETS, 'Unknown prototype worker')
    if role!='prototype':
        import unified_capture
        unified_capture.validate_worker_arguments(borrowed.SCRIPTS/TARGETS[role],values,
            dict(manifest=doc['manifest'],output=doc['output'],chunk_size=doc['chunk_size']))
        return
    parser=argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    for name in ('manifest','output','environment','confirmed-environment'):parser.add_argument('--'+name,required=True)
    parser.add_argument('--limit',type=int,required=True);parser.add_argument('--chunk-size',type=int,required=True)
    parser.add_argument('--stop-after-chunks',type=int,required=True)
    parser.add_argument('--run',action='store_true',required=True);parser.add_argument('--resume',action='store_true')
    try:args=parser.parse_args(values)
    except SystemExit as error:raise RuntimeError('Unknown prototype authority/profile flags') from error
    flags=[v for v in values if v.startswith('--')]
    require(len(flags)==len(set(flags)) and args.manifest==doc['manifest'] and args.output==doc['output']
        and args.environment==args.confirmed_environment==doc['environment']
        and args.limit==doc['limit'] and args.chunk_size==doc['chunk_size']
        and 0 <= args.stop_after_chunks <= 48, 'Prototype arguments changed the explicit bounded source')


def dispatch(intent,digest,role,values):
    return [sys.executable,'-B',str(Path(__file__).absolute()),'--intent',str(intent),
            '--intent-sha256',digest,'--role',role,'--',*values]


@contextmanager
def routed(intent,digest,doc):
    original=subprocess.Popen
    targets={str(borrowed.SCRIPTS/name):role for role,name in TARGETS.items() if role!='prototype'}
    names={Path(value).name for value in targets}
    def checked(command,*args,**kwargs):
        if isinstance(command,(str,bytes)):
            require(not any(name in str(command) for name in names), 'Shell-form prototype worker forbidden')
            return original(command,*args,**kwargs)
        argv=list(map(str,command));index=2 if len(argv)>1 and argv[1]=='-B' else 1
        target=argv[index] if len(argv)>index else None
        if any(Path(value).name in names for value in argv):
            require(argv[0]==sys.executable and target in targets and not kwargs.get('shell'),
                    'Foreign prototype interpreter/script invocation')
            role=targets[target];validate_arguments(role,argv[index+1:],doc)
            command=dispatch(intent,digest,role,argv[index+1:])
        return original(command,*args,**kwargs)
    subprocess.Popen=checked
    try:yield
    finally:
        changed=subprocess.Popen is not checked;subprocess.Popen=original
        require(not changed,'Prototype child router ownership changed')


def run_role(intent,digest,role,values):
    doc=validate_intent(intent,digest);validate_arguments(role,values,doc)
    if str(borrowed.SCRIPTS) not in sys.path:sys.path.insert(0,str(borrowed.SCRIPTS))
    gen=importlib.import_module('generate_base5_independent');extent=importlib.import_module('base5_static_mission_extent')
    witness=doc['route_witness'];target=borrowed.SCRIPTS/TARGETS[role];previous=sys.argv
    with route.installed(witness['path'],generator=gen,extent=extent,manifest=doc['canonical_manifest'],
            expected_witness_sha256=witness['sha256']), recovery_population.installed(), routed(intent,digest,doc):
        try:
            sys.argv=[str(target),*values]
            runpy.run_path(str(target),run_name='__main__')
            return 0
        except SystemExit as error:return error.code if type(error.code) is int else (0 if error.code is None else 1)
        finally:sys.argv=previous


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--intent',type=Path,required=True);parser.add_argument('--intent-sha256',required=True)
    parser.add_argument('--role',choices=tuple(TARGETS),required=True)
    parser.add_argument('arguments',nargs=argparse.REMAINDER);args=parser.parse_args(argv)
    values=args.arguments[1:] if args.arguments[:1]==['--'] else args.arguments
    return run_role(args.intent,args.intent_sha256,args.role,values)


if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,ValueError,RuntimeError) as error:
        print(json.dumps(dict(event='fresh_route_prototype_blocked',error=str(error),bulk_authorized=False)),file=sys.stderr)
        raise SystemExit(1)
