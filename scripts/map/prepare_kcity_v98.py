#!/usr/bin/env python3
"""Provision v98 from pinned v97 bytes (or a supplied v98); never overwrite assets."""
import argparse
import hashlib
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = 'configs/map/kcity_v98_centerline_connection.yaml'


def verify(path, expected):
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError(f'v98 map SHA mismatch or unreadable asset: {path}')
    metadata = next(e for _, e in ET.iterparse(path, events=('start',)) if e.tag == 'MetaInfo')
    if metadata.get('map_version') != '98':
        raise ValueError('Expected OSM MetaInfo map_version=98')


def derive(data, expected_source_sha):
    """Preserve all bytes except the approved endpoint reference and MetaInfo version."""
    if hashlib.sha256(data).hexdigest() != expected_source_sha:
        raise ValueError('v97 source SHA mismatch; refusing to transform another asset')
    blocks = re.findall(rb'  <way id="105522"[^>]*>.*?</way>', data, re.S)
    if len(blocks) != 1 or blocks[0].count(b'<nd ref="229565"/>') != 1:
        raise ValueError('Expected exactly one approved centerline endpoint')
    if data.count(b'map_version="97"') != 1:
        raise ValueError('Expected exactly one v97 MetaInfo version')
    updated = blocks[0].replace(b'<nd ref="229565"/>', b'<nd ref="233927"/>')
    return data.replace(blocks[0], updated, 1).replace(b'map_version="97"', b'map_version="98"', 1)


def prepare(*, root=ROOT, source=None, check=False):
    root = Path(root)
    manifest = yaml.safe_load((root / MANIFEST).read_text())
    if (manifest['map_version'] != 98 or
            manifest['map_file'] != 'kcity_map/lanelet_map/lanelet2_map_v98.osm'):
        raise ValueError('Unexpected v98 manifest version or path')
    target = root / manifest['map_file']
    if os.path.lexists(target):
        verify(target, manifest['map_sha256'])
        return target
    if check:
        raise FileNotFoundError(f'Provision v98 first: python3 scripts/map/prepare_kcity_v98.py ({target})')
    source = Path(source) if source is not None else root / manifest['source']['map_file']
    if not source.is_file():
        raise FileNotFoundError('Supply the separately distributed v97 (or v98) asset using --source PATH')
    data = source.read_bytes()
    if hashlib.sha256(data).hexdigest() != manifest['map_sha256']:
        data = derive(data, manifest['source']['sha256'])
    if hashlib.sha256(data).hexdigest() != manifest['map_sha256']:
        raise ValueError('Derived v98 SHA mismatch')
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix='.prepare-v98-', dir=target.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(data)
        verify(temporary, manifest['map_sha256'])
        os.link(temporary, target)  # Atomic creation; a concurrent writer is never overwritten.
    finally:
        temporary.unlink()
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--source', type=Path)
    args = parser.parse_args()
    print(f'PASS: v98 SHA and metadata verified: {prepare(source=args.source, check=args.check)}')


if __name__ == '__main__':
    main()
