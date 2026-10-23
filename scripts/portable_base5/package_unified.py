#!/usr/bin/env python3
"""공통 438개 runtime과 7개 helper의 로컬 배포본을 만든다.

기본은 읽기 전용 계획이다. --run도 새 로컬 디렉터리에 검증된 파일을
복사할 뿐 SSH·설치·ROS·MORAI·수집·기존 출력 수정은 수행하지 않는다.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat

import unified_borrowed as borrowed


SCRIPTS = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'
MODEL_ROOT = Path(__file__).resolve().parents[2]/'perception/detection3d/bevfusion'
HELPERS = Path(__file__).resolve().parent
SCHEMA = 'base5_unified_local_package_v1'
HEX = re.compile(r'^[0-9a-f]{64}$')


def require(value, message):
    if not value:
        raise ValueError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Nonsymlink normalized local path required: '+str(path))
    return path


def regular(value):
    path = physical(value)
    require(stat.S_ISREG(path.stat().st_mode), 'Regular package input required: '+str(path))
    return path


def sha(path):
    value = hashlib.sha256()
    with regular(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def remote_workspace(value):
    require(type(value) is str and '\\' not in value and '\x00' not in value,
            'Explicit canonical remote HOME workspace required')
    path = PurePosixPath(value)
    require(str(path) == value and len(path.parts) == 4 and path.parts[:2] == ('/', 'home')
            and path.name == borrowed.WORKSPACE and path.parts[2] not in ('.','..')
            and '..' not in path.parts, 'Remote target must be /home/USER/'+borrowed.WORKSPACE)
    return path


def plan(release, workspace, recovery_intent_sha256, output):
    release, output = regular(release), physical(output)
    workspace = remote_workspace(workspace)
    require(type(recovery_intent_sha256) is str and HEX.fullmatch(recovery_intent_sha256),
            'Historical recovery intent requires an explicit lowercase SHA256')
    require(not output.exists() and output.parent.is_dir(), 'Fresh local staging output with existing parent required')
    for protected in (SCRIPTS, HELPERS, release.parent):
        protected = physical(protected)
        require(output != protected and protected not in output.parents,
                'Staging output overlaps immutable input directory')
    release_sha = sha(release)
    doc = json.loads(release.read_text())
    pins = doc.get('runtime_hashes')
    require(type(pins) is dict and len(pins) == 438, 'Qualified complete 438-file runtime required')
    require(doc.get('bulk_ready') is True and doc.get('environment') == 'sunny15',
            'Immutable qualified v159 Sunny15 reference required; this is not remote capture qualification')
    actual = {str(physical(path)) for path in SCRIPTS.glob('*.py')}
    dataset_pins = {name:digest for name,digest in pins.items() if Path(name).parent == physical(SCRIPTS)}
    external_pins = {name:digest for name,digest in pins.items() if name not in dataset_pins}
    require(len(dataset_pins) == 371 and len(external_pins) == 67 and actual == set(dataset_pins),
            'Runtime inventory must match 371 dataset scripts and 67 preserved model files')
    sources = {}; scripts = {}; helpers = {}
    root = workspace/borrowed.RUNTIME_REL
    for filename, expected in sorted(pins.items()):
        path = regular(filename)
        require((path.parent == physical(SCRIPTS) or physical(MODEL_ROOT) in path.parents) and path.suffix == '.py'
                and type(expected) is str and HEX.fullmatch(expected) and sha(path) == expected,
                'Frozen runtime input differs: '+filename)
        sources[str(path)] = expected
        if filename in dataset_pins:
            scripts[str(root/'scripts'/path.name)] = expected
    require(borrowed.EXTRA_SCRIPTS <= {Path(name).name for name in pins},
            'The complete 11-file shared native-profile extension is required')
    for name in sorted(borrowed.REQUIRED_HELPERS):
        path = regular(HELPERS/name)
        sources[str(path)] = sha(path)
        helpers[str(root/'checks'/name)] = sources[str(path)]
    require(sha(release) == release_sha, 'Release changed during package planning')
    sources[str(release)] = release_sha
    deployment = dict(schema=borrowed.SCHEMA, workspace=str(workspace), environment='sunny13',
        runtime_hashes=scripts, source_runtime_hashes=pins, helper_hashes=helpers,
        recovery_intent_sha256=recovery_intent_sha256,
        source_release_sha256=release_sha, source_runtime_count=438,
        automatic_capture_started=False, historical_bundle_rewritten=False)
    return dict(schema=SCHEMA, release=str(release), release_sha256=release_sha,
        output=str(output), workspace=str(workspace), source_pins=sources,
        deployment=deployment, runtime_files=438, overlay_scripts=371,
        preserved_external_runtime_files=67, helper_files=len(helpers),
        files_written=False, remote_access=False, ros_started=False, capture_started=False)


def copy_verified(source, destination, expected):
    source, destination = regular(source), physical(destination)
    require(sha(source) == expected, 'Input changed before package copy: '+str(source))
    fd = os.open(destination, os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd,'wb') as target, source.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            target.write(block)
    require(sha(destination) == expected and sha(source) == expected,
            'Source or derived package changed during copying: '+str(source))


def prepare(release, workspace, recovery_intent_sha256, output, run=False):
    request = plan(release, workspace, recovery_intent_sha256, output)
    if not run:
        return request
    output = Path(request['output'])
    # mkdir는 독점 생성이다. 계획 이후 생긴 부분 출력·symlink도 덮어쓰지 않는다.
    output.mkdir(mode=0o700)
    for directory in ('scripts','checks'):
        (output/directory).mkdir(mode=0o700)
    for filename, expected in request['source_pins'].items():
        source = Path(filename)
        if source == Path(request['release']):
            continue
        if source.parent == physical(SCRIPTS):
            kind = 'scripts'
        elif source.parent == physical(HELPERS):
            kind = 'checks'
        else:
            continue  # 67개 model runtime은 기존 원격 bundle에서 그대로 보존한다.
        copy_verified(source, output/kind/source.name, expected)
    for filename, expected in request['source_pins'].items():
        require(sha(filename) == expected, 'Package input changed before completion: '+filename)
    target = physical(output/'deployment.json')
    fd = os.open(target, os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd,'w') as stream:
        json.dump(request['deployment'],stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
    return dict(request, files_written=True, deployment_sha256=sha(target),
        directories_mode='0700', files_mode='0600')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--release',type=Path,required=True)
    parser.add_argument('--workspace',required=True)
    parser.add_argument('--recovery-intent-sha256',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--run',action='store_true',help='Create the fresh local package only; never deploy')
    parser.add_argument('--verbose',action='store_true',help='Explicitly print complete input/deployment hash inventories')
    args = parser.parse_args(argv)
    result = prepare(args.release,args.workspace,args.recovery_intent_sha256,args.output,args.run)
    summary = dict(event='unified_package_created' if result['files_written'] else 'unified_package_plan',
        output=result['output'], runtime_files=result['runtime_files'],
        overlay_scripts=result['overlay_scripts'],
        preserved_external_runtime_files=result['preserved_external_runtime_files'],
        helper_files=result['helper_files'], deployment_sha256=result.get('deployment_sha256'),
        files_written=result['files_written'], remote_access=False, native_captures=0)
    print(json.dumps(result if args.verbose else summary,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
