#!/usr/bin/env python3
"""Package and optionally publish a verified all-JPEG camera portable dataset.

Lossless tar.zst packaging of already Q90 RGB; masks/depth/GT remain lossless.
Never uploads credentials/source/model weights. No Vast access or GPU required.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import time

from prepare_base5_portable_dataset import sha256, validate


def pack(data, output, name):
    data = Path(data).resolve(strict=True)
    output = Path(output).absolute()
    if output.exists():
        raise FileExistsError('Use fresh archive output; upload-only supports existing verified parts')
    if Path(name).name != name or not name.endswith('.tar.zst'):
        raise ValueError('Expected simple .tar.zst basename')
    source = validate(data, verify_hashes=True)
    if source.get('image_policy') != dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'):
        raise ValueError('Current Drive share requires native Q90 RGB for every split')
    output.mkdir(parents=True)
    archive = output / name
    print('Packing verified payload; auxiliary images stay lossless.', flush=True)
    with archive.open('wb') as target:
        tar = subprocess.Popen(['tar', '--hard-dereference', '-C', str(data.parent),
                                '-cf', '-', data.name], stdout=subprocess.PIPE)
        try:
            subprocess.run(['zstd', '-6', '-T4', '-c'], stdin=tar.stdout, stdout=target, check=True)
        finally:
            tar.stdout.close()
        if tar.wait():
            raise RuntimeError('tar failed')
    # Read every archive member and hash it, not merely a tar directory listing.
    expected = dict(source['files'])
    expected['manifest.json'] = dict(bytes=(data/'manifest.json').stat().st_size,
                                    sha256=sha256(data/'manifest.json'))
    seen = set()
    decoder = subprocess.Popen(['zstd', '-dc', str(archive)], stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=decoder.stdout, mode='r|') as stream:
            for member in stream:
                if member.isdir():
                    continue
                if not member.isfile():
                    raise ValueError('Archive must contain independent regular files')
                prefix = data.name + '/'
                if not member.name.startswith(prefix):
                    raise ValueError('Archive root differs')
                relative = member.name[len(prefix):]
                if relative not in expected or relative in seen:
                    raise ValueError('Unexpected/duplicate archive payload: ' + relative)
                digest = hashlib.sha256()
                handle = stream.extractfile(member)
                for block in iter(lambda: handle.read(4*1024*1024), b''):
                    digest.update(block)
                item = expected[relative]
                if member.size != item['bytes'] or digest.hexdigest() != item['sha256']:
                    raise ValueError('Archive payload differs: ' + relative)
                seen.add(relative)
    finally:
        decoder.stdout.close()
    if decoder.wait() or seen != set(expected):
        raise ValueError('Incomplete archive')
    parts = output/'parts'; parts.mkdir()
    inventory = []
    whole = hashlib.sha256()
    with archive.open('rb') as handle:
        index = 0
        while True:
            block = handle.read(1024*1024*1024)
            if not block:
                break
            filename = name + '.part%03d' % index
            (parts/filename).write_bytes(block)
            whole.update(block)
            inventory.append(dict(name=filename, bytes=len(block),
                sha256=hashlib.sha256(block).hexdigest(), md5=hashlib.md5(block).hexdigest()))
            index += 1
    if whole.hexdigest() != sha256(archive):
        raise ValueError('Split/concatenation differs')
    manifest = dict(version=1, created_at=datetime.datetime.now().astimezone().isoformat(),
        scenes=source['scenes'], split_counts=source['split_counts'], image_policy=source['image_policy'],
        source_manifest_sha256=sha256(data/'manifest.json'), archive_root=data.name,
        archive_name=name, archive_bytes=archive.stat().st_size, archive_sha256=whole.hexdigest(),
        payload_bytes=source['bytes'], verified_archive_files=len(seen), parts=inventory,
        compression='Lossless tar.zst level6 threads4; camera JPEG Q90 encoded exactly once upstream',
        auxiliary_contract=source['auxiliary_contract'])
    (parts/'backup_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (parts/'SHA256SUMS').write_text(''.join('%s  %s\n' % (p['sha256'], p['name']) for p in inventory))
    (parts/'RESTORE.txt').write_text(
        'Camera RGB: native Q90 JPEG for train/val/test/analysis. Auxiliary PNG is lossless GT, not RGB.\n'
        'No original camera PNG or cache dependency. No extra --jpeg-cache or JPEG augmentation.\n'
        'sha256sum -c SHA256SUMS\n'
        'cat '+name+'.part* > '+name+'\n'
        'echo "'+whole.hexdigest()+'  '+name+'" | sha256sum -c -\n'
        'tar --zstd -xf '+name+'\n'
        'python perception/detection3d/bevfusion/tools/prepare_base5_portable_dataset.py --verify '+data.name+'\n'
        'Selected config: bevfusion_base5_dense_512_native_jpeg90.py\n')
    print(json.dumps({k:v for k,v in manifest.items() if k!='parts'},indent=2), flush=True)
    return manifest


def publish(parts, client, remote, root_id):
    parts = Path(parts).resolve(strict=True)
    manifest = json.loads((parts/'backup_manifest.json').read_text())
    for item in manifest['parts']:
        if sha256(parts/item['name']) != item['sha256']:
            raise ValueError('Local upload part changed')
    flags = ['--drive-root-folder-id', root_id, '--retries', '2', '--low-level-retries', '3']
    def run(*args, capture=False):
        result = subprocess.run([str(client), *args, *flags], check=True,
            capture_output=capture, text=True, timeout=1800)
        return result.stdout if capture else None
    started = time.monotonic()
    # COMPLETE is the last operation, never copied before remote verification.
    run('copy', str(parts), remote, '--exclude', 'COMPLETE.json', '--transfers', '2',
        '--checkers', '4', '--drive-chunk-size', '8M', '--stats', '30s', '--stats-one-line',
        '--stats-log-level', 'NOTICE')
    run('check', str(parts), remote, '--one-way', '--exclude', 'COMPLETE.json')
    entries = json.loads(run('lsjson', remote, '--hash', capture=True))
    by_name = {e['Name']:e for e in entries}
    for item in manifest['parts']:
        entry = by_name[item['name']]
        hashes = {k.lower():v.lower() for k,v in entry['Hashes'].items()}
        if entry['Size'] != item['bytes'] or hashes.get('md5') != item['md5']:
            raise ValueError('Remote archive size/hash differs')
        if hashes.get('sha256') and hashes['sha256'] != item['sha256']:
            raise ValueError('Remote SHA256 differs')
    folders = json.loads(run('lsjson', remote.rsplit('/',1)[0], '--dirs-only', capture=True))
    folder = next(e for e in folders if e['Name']==remote.rsplit('/',1)[1])
    result = dict(verified=True, verified_at=datetime.datetime.now().astimezone().isoformat(),
        archive_sha256=manifest['archive_sha256'], archive_bytes=manifest['archive_bytes'],
        remote_parts=len(manifest['parts']), remote_size_md5_verified=True,
        remote_path=remote, folder_url='https://drive.google.com/drive/folders/'+folder['ID'],
        upload_seconds=time.monotonic()-started)
    (parts/'COMPLETE.json').write_text(json.dumps(result,indent=2)+'\n')
    run('copyto', str(parts/'COMPLETE.json'), remote+'/COMPLETE.json')
    run('check', str(parts), remote, '--one-way')
    print(json.dumps(result,indent=2), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--archive-name', default='base5_sunny13_5k_jpeg90_261011.tar.zst')
    parser.add_argument('--upload', action='store_true')
    parser.add_argument('--upload-only', action='store_true')
    parser.add_argument('--rclone', type=Path)
    parser.add_argument('--remote')
    parser.add_argument('--drive-root-id')
    args = parser.parse_args()
    if not args.upload_only:
        if not args.data:
            parser.error('--data required to package')
        pack(args.data, args.output, args.archive_name)
    if args.upload or args.upload_only:
        if not (args.rclone and args.remote and args.drive_root_id):
            parser.error('Explicit rclone, remote and root required to upload')
        publish(args.output/'parts', args.rclone, args.remote, args.drive_root_id)


if __name__=='__main__':
    main()
