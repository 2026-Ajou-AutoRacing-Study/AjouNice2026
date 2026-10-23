#!/usr/bin/env python3
"""승인된 Base3 RGB PNG만 Q90 분석 입력으로 바꾸고 검증 후 선택 삭제한다.

학습 infos/과거 계약은 고치지 않는다. LiDAR·Semantic·Instance·Depth·메타데이터는
그대로 보존하며 새 상대 경로 index로 분석한다. 기본 실행은 읽기 전용이다.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import stat
import time

import cv2
import numpy as np
from PIL import Image

ROOT = Path('/home/luv/morai_cap/260829_034057_base3_static_10k')
SCHEMA = 'base3_q90_analysis_rgb_migration_v1'
CAMERAS = {'CAMERA_'+str(i) for i in range(1,5)}


def require(value, message):
    if not value:
        raise ValueError(message)


def sha_bytes(value):
    return hashlib.sha256(value).hexdigest()


def sha_file(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(4*2**20),b''):
            h.update(block)
    return h.hexdigest()


def checked(path, directory=False):
    path = Path(path).absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path,*path.parents)),
            'No symlink or unnormalized path is permitted')
    s = path.stat()
    require(s.st_uid == os.getuid() and (stat.S_ISDIR(s.st_mode) if directory else stat.S_ISREG(s.st_mode)),
            'Owned regular source required: '+str(path))
    require(not directory or not os.path.ismount(path), 'No nested mount allowed')
    return path


def atomic_new(path, value):
    payload=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW,0o600)
    with os.fdopen(fd,'wb') as stream:
        stream.write(payload);stream.flush();os.fsync(stream.fileno())


def rgb_source(path):
    return path.parent.name in CAMERAS and path.name.endswith('_Intensity.png')


def inventory(root):
    rgb=[];retained=[];device=root.stat().st_dev
    for directory,folders,files in os.walk(root,followlinks=False):
        for name in folders:
            q=checked(Path(directory)/name,True)
            require(q.stat().st_dev==device,'Foreign source filesystem')
        for name in files:
            p=checked(Path(directory)/name)
            require(p.stat().st_dev==device,'Foreign source filesystem')
            if rgb_source(p):
                rgb.append(p)
            else:
                retained.append(p)
    return sorted(rgb),sorted(retained)


def unchanged_record(root,path):
    s=path.stat()
    return dict(path=str(path.relative_to(root)),bytes=s.st_size,sha256=sha_file(path),
                inode=s.st_ino,mtime_ns=s.st_mtime_ns)


def verify_retained(root,records):
    for row in records:
        p=checked(root/row['path']);s=p.stat()
        require(s.st_ino==row['inode'] and s.st_size==row['bytes']
                and s.st_mtime_ns==row['mtime_ns'] and sha_file(p)==row['sha256'],
                'Preserved LiDAR/GT/metadata changed: '+str(p))


def verify_destination(root,row):
    p=checked(root/row['jpeg_path'])
    require(p.stat().st_size==row['jpeg_bytes'] and sha_file(p)==row['jpeg_sha256'],
            'JPEG bytes changed before PNG deletion')
    image=cv2.imread(str(p),cv2.IMREAD_COLOR)
    require(image is not None and list(image.shape)==[row['height'],row['width'],3],
            'JPEG decode/native shape verification failed')


def convert_one(root,path):
    before=path.stat();raw=path.read_bytes()
    with Image.open(path) as image:
        require(image.format=='PNG','Only PNG camera input may be converted')
        rgb=np.array(image.convert('RGB'))
    ok,blob=cv2.imencode('.jpg',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,90])
    require(ok,'JPEG Q90 encoding failed')
    data=blob.tobytes();destination=path.with_suffix('.jpg')
    require(not destination.is_symlink(),'JPEG destination must not be a symlink')
    if destination.exists():
        require(destination.read_bytes()==data,'Existing destination differs; never overwrite')
    else:
        temporary=destination.with_name(destination.name+'.migration-tmp')
        fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
        with os.fdopen(fd,'wb') as stream:
            stream.write(data);stream.flush();os.fsync(stream.fileno())
        # No overwrite even if another process created the destination.
        os.link(temporary,destination);temporary.unlink()
    after=path.stat()
    require((before.st_ino,before.st_size,before.st_mtime_ns)==
            (after.st_ino,after.st_size,after.st_mtime_ns),'Source PNG changed while encoding')
    row=dict(source_png=str(path.relative_to(root)),source_bytes=len(raw),source_sha256=sha_bytes(raw),
        source_inode=before.st_ino,source_mtime_ns=before.st_mtime_ns,source_nlink=before.st_nlink,
        source_allocated_bytes=before.st_blocks*512,jpeg_path=str(destination.relative_to(root)),
        jpeg_bytes=len(data),jpeg_sha256=sha_bytes(data),height=int(rgb.shape[0]),width=int(rgb.shape[1]),
        camera=path.parent.name,quality=90,source_deleted=False)
    verify_destination(root,row)
    return row


def write_frames(root,rows,path):
    groups={}
    for row in rows:
        source=Path(row['source_png']);run=source.parent.parent
        frame=source.name[:-len('_Intensity.png')]
        key=(str(run),frame)
        record=groups.setdefault(key,dict(run=str(run),frame_id=frame,images={},
            historical_rejected_run='rejected' in str(run)))
        record['images'][row['camera']]=dict(path=row['jpeg_path'],width=row['width'],height=row['height'])
    with path.open('x') as stream:
        for (run,frame),record in sorted(groups.items()):
            lidar=Path(run)/'LIDAR_5'/(frame+'_Intensity.bin')
            corners=Path(run)/'LIDAR_5'/(frame+'_Instance_8Points.txt')
            record.update(lidar=str(lidar) if (root/lidar).is_file() else None,
                native_corners=str(corners) if (root/corners).is_file() else None,
                complete_four_camera_lidar=set(record['images'])==CAMERAS and (root/lidar).is_file(),
                metadata_directory=str(Path(run)/'dynamic_metadata'),training_split_not_reassigned=True)
            stream.write(json.dumps(record,ensure_ascii=False)+'\n')
    return dict(frames=len(groups),rejected_run_frames=sum(r['historical_rejected_run'] for r in groups.values()),
                complete_four_camera_lidar_frames=sum(r['complete_four_camera_lidar'] for r in groups.values()))


def protected_paths(releases):
    result=set()
    for release in releases:
        doc=json.loads(Path(release).read_text())
        for key in ('runtime_hashes','input_hashes'):
            result.update(Path(p).absolute() for p in doc.get(key,{}))
        result.update(Path(g['path']).absolute() for g in doc.get('gate_evidence',{}).values())
    return result


def run(root,output,*,workers=4,execute=False,delete_png=False,releases=()):
    root=checked(root,True)
    require(root==ROOT,'Only the explicitly approved Base3 source is supported')
    require(1<=workers<=4,'Use at most four CPU workers; no GPU or ROS')
    output=Path(output).absolute()
    require(not output.exists() and root not in output.parents and output!=root,'Fresh external analysis index required')
    rgb,retained=inventory(root)
    require(rgb,'No camera RGB _Intensity.png sources found')
    protected=protected_paths(releases)
    require(not (set(rgb)&protected),'A current qualified release pins a target RGB PNG')
    summary=dict(schema=SCHEMA,data_root=str(root),output=str(output),rgb_images=len(rgb),
        source_rgb_bytes=sum(p.stat().st_size for p in rgb),preserved_files=len(retained),quality=90,
        native_resolution=True,analysis_only=True,training_infos_modified=False,
        protected_release_inputs=[dict(path=str(p),sha256=sha_file(p)) for p in releases],
        approved_png_deletion=delete_png,execute=execute)
    if not execute:return summary
    output.mkdir(parents=True)
    atomic_new(output/'operation.json',summary)
    cv2.setNumThreads(1)
    retained_records=[]
    print(json.dumps(dict(event='preserved_inventory_started',files=len(retained))),flush=True)
    with (output/'preserved_files.jsonl').open('x') as stream:
        for p in retained:
            row=unchanged_record(root,p);retained_records.append(row)
            stream.write(json.dumps(row)+'\n')
    rows=[];started=time.monotonic()
    with ThreadPoolExecutor(max_workers=workers) as pool, (output/'rgb_mapping.jsonl').open('x') as stream:
        for row in pool.map(lambda p:convert_one(root,p),rgb):
            rows.append(row);stream.write(json.dumps(row)+'\n')
            if len(rows)%500==0:
                stream.flush()
                elapsed=time.monotonic()-started
                print(json.dumps(dict(event='jpeg_progress',done=len(rows),total=len(rgb),
                    elapsed_sec=elapsed,eta_sec=elapsed/len(rows)*(len(rgb)-len(rows)))),flush=True)
    # Verify every retained native payload and every finished JPEG before any
    # destructive step. A bad conversion leaves all original PNG untouched.
    print(json.dumps(dict(event='full_predelete_verification',images=len(rows))),flush=True)
    for row in rows:
        verify_destination(root,row)
        png=checked(root/row['source_png']);s=png.stat()
        require(s.st_ino==row['source_inode'] and s.st_mtime_ns==row['source_mtime_ns']
            and sha_file(png)==row['source_sha256'],'PNG changed before explicit deletion')
    verify_retained(root,retained_records)
    frames=write_frames(root,rows,output/'frames.jsonl')
    manifest=dict(summary,complete=True,**frames,mapping='rgb_mapping.jsonl',frames_index='frames.jsonl',
        preserved_inventory='preserved_files.jsonl',mapping_sha256=sha_file(output/'rgb_mapping.jsonl'),
        preserved_inventory_sha256=sha_file(output/'preserved_files.jsonl'),
        frames_sha256=sha_file(output/'frames.jsonl'),jpeg_bytes=sum(r['jpeg_bytes'] for r in rows),
        policy='native Q90 RGB; masks/depth/LiDAR/GT/metadata unchanged; original PNG hashes retained',
        past_capture_contract_not_rewritten=True,new_capture_executed=False,delete_started=False)
    atomic_new(output/'manifest.json',manifest)
    deleted=0
    if delete_png:
        # Check destination and source identity again immediately before each
        # unlink; never glob-remove PNG or touch Semantic/Instance/Depth.
        with (output/'png_deletions.jsonl').open('x') as stream:
            for row in rows:
                p=checked(root/row['source_png']);s=p.stat()
                require(rgb_source(p) and p not in protected and s.st_ino==row['source_inode']
                    and s.st_size==row['source_bytes'] and s.st_mtime_ns==row['source_mtime_ns'],
                    'Source scope/identity changed during deletion')
                require(sha_file(root/row['jpeg_path'])==row['jpeg_sha256'],'JPEG changed during deletion')
                p.unlink();deleted+=1
                stream.write(json.dumps(dict(source_png=row['source_png'],jpeg_path=row['jpeg_path'],
                    source_sha256=row['source_sha256'],jpeg_sha256=row['jpeg_sha256']))+'\n')
        require(not any(root.rglob('*_Intensity.png')),'An unexpected camera PNG remains')
    verify_retained(root,retained_records)
    for row in rows:verify_destination(root,row)
    completion=dict(schema=SCHEMA,passed=True,images=len(rows),deleted_png=deleted,
        source_rgb_bytes=summary['source_rgb_bytes'],jpeg_bytes=manifest['jpeg_bytes'],
        rgb_logical_saving_bytes=summary['source_rgb_bytes']-manifest['jpeg_bytes'] if delete_png else 0,
        semantic_instance_depth_lidar_metadata_unchanged=True,elapsed_sec=time.monotonic()-started,
        manifest_sha256=sha_file(output/'manifest.json'),analysis_only=True,new_capture_executed=False,**frames)
    atomic_new(output/'completion.json',completion)
    return completion


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=ROOT)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--workers',type=int,default=4)
    p.add_argument('--release',type=Path,action='append',default=[])
    p.add_argument('--run',action='store_true')
    p.add_argument('--delete-verified-rgb-png',action='store_true')
    a=p.parse_args()
    print(json.dumps(run(a.root,a.output,workers=a.workers,execute=a.run,
        delete_png=a.delete_verified_rgb_png,releases=a.release),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
