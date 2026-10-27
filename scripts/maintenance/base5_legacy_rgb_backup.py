#!/usr/bin/env python3
"""Approved 10/8 legacy RGB backup: prepare Q90, verify Drive, then prune RGB only.

Native GT/masks/depth/LiDAR and frozen collector records remain local, unchanged.
This is an analysis RGB archive, not a new training dataset or lossless PNG backup.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess

import base3_analysis_jpeg as core

SOURCE = Path('/home/luv/morai_cap/base5_v2_261008')
OUTPUT_BASE = Path(__file__).resolve().parents[2]/'artifacts/maintenance'
REMOTE = 'ajou_drive_private:dataset/Base5/legacy_base5_v2_261008_rgb_q90_261028'
SCHEMA = 'base5_legacy_261008_rgb_q90_backup_v1'


def checked_output(output, *, fresh=False):
    output = Path(output).absolute()
    core.require(output.resolve() == output, 'No symlink output')
    output.relative_to(OUTPUT_BASE)
    core.require(output != OUTPUT_BASE and (not fresh or not output.exists()), 'Fresh child output required')
    return output


def stat_record(root, path):
    s = path.stat()
    return dict(path=str(path.relative_to(root)), size=s.st_size, inode=s.st_ino,
                device=s.st_dev, mtime_ns=s.st_mtime_ns)


def verify_retained(root, records):
    # These payloads are never written by this tool. Preserve file identity too.
    for row in records:
        core.require(stat_record(root, root/row['path']) == row,
                     'Retained GT/metadata identity changed: '+row['path'])


def md5_file(path):
    h = hashlib.md5()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4*2**20), b''):
            h.update(chunk)
    return h.hexdigest()


def require_remote_md5(text, expected):
    lines = [line.split() for line in text.splitlines() if line.strip()]
    core.require(len(lines) == 1 and len(lines[0]) >= 2 and
                 len(expected) == 32 and lines[0][0].lower() == expected.lower(),
                 'Drive archive MD5 mismatch; PNG deletion prohibited')


def verify_predecoded_jpeg(root,row):
    # prepare already fully decoded every JPEG; unchanged bytes retain that result.
    path=core.checked(root/row['jpeg_path'])
    core.require(path.stat().st_size==row['jpeg_bytes'] and
        core.sha_file(path)==row['jpeg_sha256'],'Predecoded JPEG bytes changed')


def rclone(*args):
    return subprocess.run(['rclone', *map(str,args), '--contimeout','10s',
        '--timeout','90s','--retries','2'], check=True, text=True,
        stdout=subprocess.PIPE, stderr=None).stdout


def prepare(output, workers):
    core.require(1 <= workers <= 4, 'At most four workers')
    root = core.checked(SOURCE, True)
    output = checked_output(output, fresh=True)
    rgb, retained = core.inventory(root)
    core.require(rgb, 'No RGB Intensity PNG; already-JPEG data is not migrated')
    output.mkdir(parents=True)
    meta = output/'metadata'; meta.mkdir()
    core.atomic_new(meta/'operation.json', dict(schema=SCHEMA, source=str(root),
        quality=90, native_resolution=True, rgb_images=len(rgb), retained_files=len(retained),
        rgb_source_bytes=sum(p.stat().st_size for p in rgb),
        archive_scope='RGB JPEG and audit metadata only; native GT/non-RGB stay local',
        png_is_lossless_jpeg_is_lossy=True, frozen_records_not_rewritten=True))
    print(json.dumps(dict(event='inventory',rgb=len(rgb),retained=len(retained))),flush=True)
    records = []
    with (meta/'retained_files.jsonl').open('x') as stream:
        for path in retained:
            row=stat_record(root,path); records.append(row)
            stream.write(json.dumps(row)+'\n')
    core.cv2.setNumThreads(1)
    rows=[]
    with ThreadPoolExecutor(max_workers=workers) as pool, (meta/'rgb_mapping.jsonl').open('x') as stream:
        for row in pool.map(lambda path:core.convert_one(root,path),rgb):
            rows.append(row); stream.write(json.dumps(row)+'\n')
            if len(rows)%500 == 0:
                stream.flush()
                print(json.dumps(dict(event='jpeg',done=len(rows),total=len(rgb))),flush=True)
    verify_retained(root,records)
    for row in rows:
        core.verify_destination(root,row)
    frames=core.write_frames(root,rows,meta/'frames.jsonl')
    core.atomic_new(meta/'manifest.json', dict(schema=SCHEMA,source=str(root),images=len(rows),
        mapping_sha256=core.sha_file(meta/'rgb_mapping.jsonl'),
        retained_sha256=core.sha_file(meta/'retained_files.jsonl'),
        jpeg_bytes=sum(row['jpeg_bytes'] for row in rows), **frames))
    (output/'jpeg_paths.nul').write_bytes(b''.join(os.fsencode(row['jpeg_path'])+b'\0' for row in rows))
    archive=output/'rgb_q90.tar.zst'
    subprocess.run(['tar','-I','zstd -T2 -3','-cf',str(archive),'-C',str(root),
        '--null','-T',str(output/'jpeg_paths.nul'),'-C',str(output),'metadata'],check=True)
    mapping_sha=core.sha_file(meta/'rgb_mapping.jsonl')
    receipt=dict(schema=SCHEMA,archive=str(archive),bytes=archive.stat().st_size,
        sha256=core.sha_file(archive),md5=md5_file(archive),mapping_sha256=mapping_sha,
        remote=REMOTE+'/'+mapping_sha[:16]+'/rgb_q90.tar.zst')
    core.atomic_new(output/'pack.json',receipt)
    print(json.dumps(dict(event='prepared',**receipt)),flush=True)


def upload_and_prune(output):
    root=core.checked(SOURCE,True); output=checked_output(output)
    receipt=json.loads((output/'pack.json').read_text()); meta=output/'metadata'
    core.require(receipt['schema']==SCHEMA,'Unexpected backup schema')
    mapping_sha=core.sha_file(meta/'rgb_mapping.jsonl')
    core.require(mapping_sha==receipt['mapping_sha256'],'RGB mapping changed')
    remote=REMOTE+'/'+mapping_sha[:16]+'/rgb_q90.tar.zst'
    core.require(receipt['remote']==remote,'Unexpected Drive scope')
    archive=core.checked(output/'rgb_q90.tar.zst')
    core.require(str(archive)==receipt['archive'] and archive.stat().st_size==receipt['bytes']
        and core.sha_file(archive)==receipt['sha256'] and md5_file(archive)==receipt['md5'],
        'Local archive changed')
    # Immutable+checksum skips a byte-identical prior upload; never overwrites it.
    print(json.dumps(dict(event='upload',remote=remote)),flush=True)
    rclone('copyto',archive,remote,'--immutable','--checksum','--stats','30s')
    require_remote_md5(rclone('md5sum',remote),receipt['md5'])
    rclone('copyto',output/'pack.json',remote.rsplit('/',1)[0]+'/pack.json','--immutable','--checksum')
    core.atomic_new(output/'drive_verified.json',dict(remote=remote,md5=receipt['md5'],bytes=receipt['bytes']))
    manifest=json.loads((meta/'manifest.json').read_text())
    core.require(manifest['source']==str(root),'Unexpected source root')
    core.require(core.sha_file(meta/'retained_files.jsonl')==manifest['retained_sha256'],
                 'Preserved inventory changed')
    rows=[json.loads(line) for line in (meta/'rgb_mapping.jsonl').open()]
    retained=[json.loads(line) for line in (meta/'retained_files.jsonl').open()]
    core.require(len(rows)==manifest['images'],'RGB count mismatch')
    verify_retained(root,retained)
    # Full pre-delete pass: a bad image leaves all remaining PNGs untouched.
    for row in rows:
        path=root/row['source_png']
        path.resolve().relative_to(root)
        core.require(core.rgb_source(path), 'Only pinned RGB sources')
        verify_predecoded_jpeg(root,row)
        s=core.checked(path).stat()
        core.require((s.st_ino,s.st_size,s.st_mtime_ns)==
            (row['source_inode'],row['source_bytes'],row['source_mtime_ns']) and
            core.sha_file(path)==row['source_sha256'],'Source PNG changed')
    with (output/'png_deletions.jsonl').open('x') as stream:
        for row in rows:
            path=core.checked(root/row['source_png']); s=path.stat()
            core.require((s.st_ino,s.st_size,s.st_mtime_ns)==
                (row['source_inode'],row['source_bytes'],row['source_mtime_ns']), 'PNG identity changed')
            core.require(core.sha_file(root/row['jpeg_path'])==row['jpeg_sha256'],'JPEG changed')
            path.unlink(); stream.write(json.dumps(dict(path=row['source_png'],
                source_sha256=row['source_sha256'],jpeg_sha256=row['jpeg_sha256']))+'\n')
    verify_retained(root,retained)
    core.atomic_new(output/'completion.json',dict(schema=SCHEMA,deleted_rgb_png=len(rows),
        source=str(root),remote=remote,quality=90,native_gt_and_non_rgb_retained=True,
        source_rgb_bytes=sum(row['source_bytes'] for row in rows),jpeg_bytes=manifest['jpeg_bytes']))
    rclone('copyto',output/'completion.json',remote.rsplit('/',1)[0]+'/completion.json','--immutable','--checksum')
    print((output/'completion.json').read_text(),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','upload-and-prune'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.output,args.workers)
    else:upload_and_prune(args.output)


if __name__=='__main__':main()
