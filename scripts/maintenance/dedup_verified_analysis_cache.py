#!/usr/bin/env python3
"""SHA가 같은 분석용 data 캐시만 hardlink로 합친다. 기본은 검사 전용이다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import uuid


def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(4*1024*1024),b''):
            value.update(chunk)
    return value.hexdigest()


def state(path):
    value=path.lstat()
    if not stat.S_ISREG(value.st_mode):
        raise ValueError('일반 파일만 허용한다: '+str(path))
    return dict(device=value.st_dev,inode=value.st_ino,size=value.st_size,mtime_ns=value.st_mtime_ns)


def run(plan_path,output,apply=False,repository=None):
    repository=Path(repository or Path(__file__).resolve().parents[2]).resolve()
    boundary=repository/'planner_logs/analysis'
    plan_path,output=Path(plan_path).resolve(),Path(output).resolve()
    if output.exists() or repository not in output.parents:
        raise ValueError('저장소 안의 새 receipt 경로가 필요하다')
    document=json.loads(plan_path.read_text())
    if document.get('schema')!='verified_cache_dedup_plan_v1' or not document.get('files'):
        raise ValueError('검산된 캐시 계획이 필요하다')
    roots=[]
    for name in ('source_roots','peer_roots'):
        values=[Path(p).resolve() for p in document[name]]
        if any(boundary not in p.parents or p.name!='data' for p in values):
            raise ValueError('분석용 data 캐시 밖은 허용하지 않는다')
        roots.append(values)
    records=[];seen=set()
    for row in document['files']:
        path,peer=Path(row['path']),Path(row['peer'])
        if (path.resolve()!=path or peer.resolve()!=peer or path==peer or path in seen
                or not any(root in path.parents for root in roots[0])
                or not any(root in peer.parents for root in roots[1])):
            raise ValueError('중복·별칭 또는 허용 경로 밖의 캐시')
        seen.add(path)
        left,right=state(path),state(peer)
        if (left!=row['source_stat'] or right!=row['peer_stat']
                or left['device']!=right['device'] or left['size']!=row['bytes']
                or right['size']!=row['bytes'] or left['inode']==right['inode']
                or digest(path)!=row['sha256'] or digest(peer)!=row['sha256']):
            raise ValueError('계획 이후 파일·해시·inode가 달라졌다: '+str(path))
        records.append((row,path,peer))
    receipt=dict(schema='verified_analysis_cache_dedup_receipt_v1',plan=str(plan_path),
        plan_sha256=digest(plan_path),applied=apply,files=len(records),
        logical_duplicate_bytes=sum(row['bytes'] for row,_,_ in records),
        completed=0,content_modified=False,path_names_preserved=True,
        disk_free_before_bytes=shutil.disk_usage(repository).free)
    output.parent.mkdir(parents=True,exist_ok=True)
    try:
        if apply:
            for row,path,peer in records:
                if state(path)!=row['source_stat'] or state(peer)!=row['peer_stat']:
                    raise ValueError('작업 중 파일이 변경됐다: '+str(path))
                temporary=path.with_name('.dedup-'+uuid.uuid4().hex)
                try:
                    os.link(peer,temporary)
                    os.replace(temporary,path)
                    descriptor=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
                    try:os.fsync(descriptor)
                    finally:os.close(descriptor)
                finally:
                    if temporary.exists():temporary.unlink()
                if not os.path.samefile(path,peer) or digest(path)!=row['sha256']:
                    raise ValueError('hardlink 후 검산 실패: '+str(path))
                receipt['completed']+=1
        receipt['passed']=True
    except BaseException as error:
        receipt.update(passed=False,error_type=type(error).__name__,error=str(error))
        raise
    finally:
        receipt['disk_free_after_bytes']=shutil.disk_usage(repository).free
        with output.open('x') as stream:
            json.dump(receipt,stream,ensure_ascii=False,indent=2)
    print(json.dumps(receipt,ensure_ascii=False))
    return receipt


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--apply',action='store_true')
    args=parser.parse_args()
    run(args.plan,args.output,args.apply)
