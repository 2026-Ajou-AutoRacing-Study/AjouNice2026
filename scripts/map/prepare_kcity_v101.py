#!/usr/bin/env python3
"""Verify or provision the separately supplied v101 asset; never overwrite a map."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = 'configs/map/kcity_v101_shadow_shoulders.yaml'


def verify(path, expected):
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError(f'v101 map SHA mismatch or unreadable asset: {path}')
    metadata = next(element for _, element in ET.iterparse(path, events=('start',))
                    if element.tag == 'MetaInfo')
    if metadata.get('map_version') != '101':
        raise ValueError('Expected OSM MetaInfo map_version=101')


def prepare(*, root=ROOT, source=None):
    root = Path(root)
    manifest = yaml.safe_load((root / MANIFEST).read_text())
    if (manifest['map_version'] != 101 or
            manifest['map_file'] != 'kcity_map/lanelet_map/lanelet2_map_v101.osm'):
        raise ValueError('Unexpected v101 manifest version or path')
    target = root / manifest['map_file']
    if os.path.lexists(target):
        verify(target, manifest['map_sha256'])
        return target
    if source is None:
        raise FileNotFoundError(
            'v101 map is distributed separately and is not in Git. Supply it with '
            'python3 scripts/map/prepare_kcity_v101.py --source /path/to/lanelet2_map_v101.osm')
    verify(Path(source), manifest['map_sha256'])
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix='.prepare-v101-', dir=target.parent)
    os.close(descriptor)
    temporary = Path(name)
    try:
        shutil.copyfile(source, temporary)
        verify(temporary, manifest['map_sha256'])
        os.link(temporary, target)  # Atomic no-clobber creation, including concurrent calls.
    finally:
        temporary.unlink()
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--source', type=Path)
    args = parser.parse_args()
    print(f'PASS: v101 SHA and metadata verified: {prepare(source=args.source)}')


if __name__ == '__main__':
    main()
