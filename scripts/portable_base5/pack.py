#!/usr/bin/env python3
"""Create a verified single local bundle archive, preserving executable bits."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

from runtime import load_bundle, sha256
from finish import verify_archive


def pack(bundle, destination, run=False):
    bundle, doc = load_bundle(bundle)
    destination = Path(destination).absolute()
    for item in (destination, *destination.parents):
        if item.is_symlink():
            raise ValueError('Symlink in archive output path')
    if '..' in destination.parts:
        raise ValueError('Noncanonical archive output path')
    destination.parent.resolve(strict=True)
    if bundle == destination or bundle in destination.parents:
        raise ValueError('Archive must be outside immutable bundle')
    partial = destination.with_suffix(destination.suffix + '.partial')
    sidecar = destination.with_name(destination.name + '.sha256')
    if any(p.exists() or p.is_symlink() for p in (destination, partial, sidecar)):
        raise ValueError('Fresh archive output is required')
    paths = {bundle.name + '/' + relative: bundle / relative for relative in doc['files']}
    paths[bundle.name + '/portable_bundle.json'] = bundle / 'portable_bundle.json'
    manifest_path = bundle / 'portable_bundle.json'
    manifest_sha = sha256(manifest_path)
    if json.loads(manifest_path.read_text()) != doc:
        raise ValueError('Bundle manifest changed after verification')
    expected = {}
    for relative, item in doc['files'].items():
        path = bundle / relative
        pin = item['sha256'] if isinstance(item, dict) else item
        expected[bundle.name + '/' + relative] = dict(
            bytes=item.get('bytes', path.stat().st_size) if isinstance(item, dict) else path.stat().st_size,
            sha256=pin)
        if path.stat().st_size != expected[bundle.name + '/' + relative]['bytes'] or sha256(path) != pin:
            raise ValueError('Bundle input changed after initial verification: ' + relative)
    expected[bundle.name + '/portable_bundle.json'] = dict(bytes=manifest_path.stat().st_size, sha256=manifest_sha)
    report = dict(action='pack-local-only', files=len(paths), destination=str(destination),
                  source_bytes=sum(v['bytes'] for v in expected.values()),
                  can_run=False, remote_touched=False)
    if not run:
        return report
    if not shutil.which('zstd'):
        raise ValueError('zstd must already be available')
    if shutil.disk_usage(destination.parent).free < report['source_bytes'] + 1024**3:
        raise ValueError('Insufficient local compression headroom')
    with partial.open('xb') as target:
        process = subprocess.Popen(['zstd', '-q', '-6', '-T2', '-c'], stdin=subprocess.PIPE, stdout=target)
        try:
            with tarfile.open(mode='w|', fileobj=process.stdin, format=tarfile.PAX_FORMAT) as archive:
                for name, path in sorted(paths.items()):
                    if path.is_symlink():
                        raise ValueError('Archive symlink refused')
                    info = tarfile.TarInfo(name)
                    info.size = expected[name]['bytes']
                    info.mode = 0o755 if path.stat().st_mode & 0o111 else 0o644
                    with path.open('rb') as source:
                        archive.addfile(info, source)
            process.stdin.close()
            if process.wait() != 0:
                raise ValueError('Compression failed; partial preserved')
        finally:
            if process.stdin and not process.stdin.closed:
                process.stdin.close()
            if process.poll() is None:
                process.terminate()
                process.wait()
    verify_archive(partial, expected)
    load_bundle(bundle)
    if sha256(manifest_path) != manifest_sha:
        raise ValueError('Bundle manifest changed during packing')
    # Publish without replacing a file created during the compression window.
    os.link(partial, destination)
    partial.unlink()
    checksum = sha256(destination)
    with sidecar.open('x') as stream:
        stream.write(checksum + '  ' + destination.name + '\n')
    return dict(report, bytes=destination.stat().st_size, sha256=checksum,
                verified_members=True, executable_modes_preserved=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    print(json.dumps(pack(args.bundle, args.output, args.run), indent=2))
