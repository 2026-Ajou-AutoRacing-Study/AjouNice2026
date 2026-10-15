#!/usr/bin/env python3
"""Lossless checkpoint parts and atomic, SHA-verified offline restoration."""
import argparse
import hashlib
import json
import re
from pathlib import Path


def split_checkpoint(source, folder, part_bytes):
    if part_bytes < 1:
        raise ValueError('Positive checkpoint part size required')
    folder.mkdir()
    digest = hashlib.sha256(); parts = []; total = 0
    with source.open('rb') as handle:
        while True:
            block = handle.read(part_bytes)
            if not block: break
            name = f'part{len(parts):04d}.bin'
            (folder/name).write_bytes(block)
            digest.update(block); total += len(block)
            parts.append(dict(name=name, bytes=len(block), sha256=hashlib.sha256(block).hexdigest()))
    manifest = dict(format='base5_checkpoint_parts_v1', name=source.name,
                    bytes=total, sha256=digest.hexdigest(), parts=parts, parts_dir=folder.name)
    (folder.parent/'PARTS.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def verify_parts(folder, manifest, output=None):
    if manifest['format'] != 'base5_checkpoint_parts_v1' or not manifest['parts']:
        raise ValueError('Unsupported or empty checkpoint parts')
    output = Path(output) if output else None
    pending = output.with_name(output.name+'.partial') if output else None
    if output and (output.exists() or pending.exists()):
        raise FileExistsError('Restore requires a fresh output path')
    full = hashlib.sha256(); total = 0
    handle = pending.open('xb') if pending else None
    try:
        for index, part in enumerate(manifest['parts']):
            if part['name'] != f'part{index:04d}.bin':
                raise ValueError('Unsafe or unordered part name')
            digest = hashlib.sha256(); size = 0
            with (folder/part['name']).open('rb') as source:
                for block in iter(lambda: source.read(4*1024*1024), b''):
                    digest.update(block); full.update(block); size += len(block)
                    if handle: handle.write(block)
            if size != part['bytes'] or digest.hexdigest() != part['sha256']:
                raise ValueError('Checkpoint part checksum failed')
            total += size
        if total != manifest['bytes'] or full.hexdigest() != manifest['sha256']:
            raise ValueError('Full checkpoint checksum failed')
        if handle:
            import os
            handle.flush(); os.fsync(handle.fileno()); handle.close(); handle = None
            pending.replace(output)
        return full.hexdigest()
    finally:
        if handle: handle.close()
        if pending and pending.exists(): pending.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True, help='Downloaded hash folder containing PARTS.json and parts/')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.folder/'PARTS.json').read_text())
    directory = manifest.get('parts_dir', 'parts')
    if not re.fullmatch(r'parts(?:_[0-9]+)?', directory):
        raise ValueError('Unsafe parts directory')
    checksum = verify_parts(args.folder/directory, manifest, args.output)
    print(json.dumps(dict(restored=str(args.output), sha256=checksum)))


if __name__ == '__main__': main()
