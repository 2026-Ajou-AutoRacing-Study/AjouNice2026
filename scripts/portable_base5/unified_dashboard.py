#!/usr/bin/env python3
"""공통 수집 profile의 새 출력을 관측하는 별도 localhost 대시보드.

기본은 읽기 전용 계획이다. --run도 웹 관측기만 시작한다. 수집기·ROS·MORAI,
PAUSE·잠금·기존 서버를 조작하지 않는다. 기존 포트가 사용 중이면 실행하지 않는다.
빌린 PC에서는 동결 helper 바깥에 배포하고 --borrowed로 실행한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import stat
import subprocess


SCHEMA = 'base5_unified_capture_v1'
POLICY = 'fixed-input-boundaries-and-common-native-profiles-v1'
ENVIRONMENTS = ('sunny11','sunny13','sunny15','foggy11','foggy13','foggy15')
LOCAL_SCRIPTS = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'
HTML_SHA = '7e364fd1e2ac25ebd0ceeebc7f8328c395cb88d2d550706f204c65e3e1969c58'


def require(value,message):
    if not value:
        raise ValueError(message)


def physical(value):
    path=Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path,*path.parents)),
            'Exact nonsymlink observer path required: '+str(path))
    return path


def regular(value):
    path=physical(value);info=path.stat()
    require(stat.S_ISREG(info.st_mode) and info.st_uid==os.getuid(),
            'Owned regular observer input required: '+str(path))
    return path


def sha(value):
    h=hashlib.sha256()
    with regular(value).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def read(value):
    return json.loads(regular(value).read_text())


def inside(value,root):
    return value==root or root in value.parents


def load_borrowed():
    workspace=physical(Path.home()/'ajounice_base5_capture')
    runtime=workspace/'work/unified-runtime-v1'
    require(physical(__file__)==runtime/'unified_dashboard.py',
            'Borrowed observer must be deployed outside the frozen checks directory')
    helpers=runtime/'checks'
    if str(helpers) not in sys.path:
        sys.path.insert(0,str(helpers))
    path=regular(helpers/'unified_borrowed.py')
    spec=importlib.util.spec_from_file_location('_base5_readonly_unified_borrowed',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    ctx=module.load(verify=False)
    module.validate_profile(ctx)
    return module,ctx


def configuration(profile=None,port=8075,borrowed=False):
    require(type(port) is int and 1024<=port<=65535,'Unprivileged observer port required')
    module,ctx=None,None
    if borrowed:
        module,ctx=load_borrowed()
        expected=ctx['paths']['profile']
        require(profile is None or physical(profile)==expected,'Borrowed profile path is fixed')
        profile=expected
        scripts=module.LOGICAL_SCRIPTS
        # 공통 overlay는 .py만 포함한다. 기존 검증된 원형 UI의 HTML까지
        # 함께 있는 별도 observer 자산을 재사용해 frozen overlay는 수정하지 않는다.
        script=ctx['workspace']/'work/web-dashboard-sunny13-v1/base5_capture_dashboard.py'
        state=ctx['workspace']/'work'/('unified-dashboard-state-v1-port'+str(port))
    else:
        require(profile is not None,'Local observer requires --profile')
        profile=physical(profile)
        scripts=LOCAL_SCRIPTS
        script=scripts/'base5_capture_dashboard.py'
        state=profile.parent/('unified-dashboard-state-port'+str(port))
    doc=read(profile)
    require(type(doc) is dict and doc.get('schema')==SCHEMA and doc.get('policy')==POLICY
            and doc.get('environment') in ENVIRONMENTS,'Prepared common capture profile required')
    root,manifest,old=map(physical,(doc['output'],doc['manifest'],doc['old_output']))
    require(root.is_dir() and old.is_dir() and root!=old,'Separate existing capture roots required')
    state=physical(state)
    require(not any(inside(state,source) or inside(source,state) for source in (root,old))
            and not inside(state,manifest.parent), 'Observer state must be outside raw/source directories')
    # borrowed의 logical manifest/script는 namespace 안에서만 존재한다.
    if not borrowed:
        require(sha(manifest)==doc['manifest_sha256'],'Frozen manifest changed')
    # runtime_hashes는 parent entrypoint closure(현재 82개)이며 UI가 없을 수 있다.
    # 전체 동결 소스의 authoritative pin은 fixed_pins에 보존된다.
    fixed=doc.get('fixed_pins',{})
    runtime=doc.get('runtime_hashes',{})
    require(type(fixed) is dict and type(runtime) is dict,'Explicit frozen input mappings required')
    key=str(scripts/'base5_capture_dashboard.py')
    require(key not in fixed or key not in runtime or fixed[key]==runtime[key],
            'Conflicting frozen dashboard source pins')
    expected=fixed.get(key,runtime.get(key))
    require(expected is not None and sha(script)==expected, 'Frozen dashboard source changed')
    require(sha(script.with_name('base5_capture_dashboard.html'))==HTML_SHA,
            'Frozen dashboard HTML changed')
    return dict(profile=physical(profile),doc=doc,root=root,manifest=manifest,old=old,
        state=state,history=state/'history.jsonl',scripts=scripts,script=script,port=port,
        borrowed=borrowed,borrowed_module=module,ctx=ctx)


def check_port(port):
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as stream:
        try:
            stream.bind(('127.0.0.1',port))
        except OSError as error:
            raise ValueError('포트 '+str(port)+'가 사용 중이어서 실행하지 않았어. '
                '기존 대시보드 터미널에서 Ctrl+C로 종료한 뒤 다시 실행하거나 다른 포트를 지정해 줘. '
                '기존 서버와 수집 프로세스는 건드리지 않았어.') from error


def state_directory(config,create=False):
    state=physical(config['state'])
    if create:
        state.mkdir(mode=0o700,exist_ok=True)
    if state.exists():
        require(state.is_dir() and state.stat().st_uid==os.getuid() and not state.stat().st_mode&0o077,
                'Private observer state directory required')
        for path in state.rglob('*'):
            path=physical(path);info=path.stat()
            require(info.st_uid==os.getuid() and not info.st_mode&0o022
                    and (path.is_dir() or stat.S_ISREG(info.st_mode) and info.st_nlink==1),
                    'Observer state cannot contain aliases to protected data')
    return state


def dashboard_command(config):
    return ['/usr/bin/python3','-B',str(config['script']),
        '--capture-root',str(config['root']),'--manifest',str(config['manifest']),
        '--history-file',str(config['history']),'--storage-root',str(config['old']),
        '--port',str(config['port'])]


def namespace_command(config):
    command=dashboard_command(config)
    if not config['borrowed']:
        return command
    ctx=config['ctx'];module=config['borrowed_module']
    argv=module.namespace(ctx,command)
    index=argv.index('--chdir')
    # 원래 capture namespace의 workspace/MORAI RW를 이 observer에서만 RO로 덮는다.
    return argv[:index]+['--ro-bind',str(ctx['workspace']),str(ctx['workspace']),
        '--ro-bind',str(ctx['mr']),str(ctx['mr']),
        '--bind',str(config['state']),str(config['state'])]+argv[index:]


def environment(config):
    if config['borrowed']:
        result=config['borrowed_module'].environment(config['ctx'])
        result={key:value for key,value in result.items() if not key.startswith('ROS_')}
    else:
        result=dict(PATH='/usr/bin:/bin',LANG='C.UTF-8',LC_ALL='C.UTF-8')
    state=str(config['state'])
    result.update(HOME=state,TMPDIR=state,TMP=state,TEMP=state,
        XDG_CACHE_HOME=state,XDG_CONFIG_HOME=state,XDG_DATA_HOME=state,
        PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--profile',type=Path)
    parser.add_argument('--borrowed',action='store_true')
    parser.add_argument('--port',type=int,default=8075)
    parser.add_argument('--run',action='store_true',help='Start only the separate read-only HTTP observer')
    args=parser.parse_args(argv)
    config=configuration(args.profile,args.port,args.borrowed)
    state_directory(config)
    check_port(config['port'])
    command=namespace_command(config)
    if not args.run:
        print(json.dumps(dict(event='unified_dashboard_plan',url='http://127.0.0.1:'+str(config['port']),
            capture_root=str(config['root']),profile=str(config['profile']),
            history_file=str(config['history']),collection_read_only=True,
            writable_state_only=str(config['state']),files_written=False,process_started=False,
            capture_controls=False,command=command),ensure_ascii=False,indent=2))
        return 0
    state_directory(config,create=True)
    previous=os.umask(0o077)
    try:
        child=subprocess.Popen(command,env=environment(config))
    finally:
        os.umask(previous)
    return child.wait()


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (OSError,ValueError,RuntimeError) as error:
        print(json.dumps(dict(event='unified_dashboard_blocked',error=str(error),
            existing_processes_untouched=True,capture_controls=False),ensure_ascii=False),file=sys.stderr)
        raise SystemExit(1)
