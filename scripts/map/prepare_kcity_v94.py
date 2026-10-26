#!/usr/bin/env python3
"""Prepare only the manifest-pinned v94 OSM; never launch ROS or replace a file."""

import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = 'configs/map/kcity_v94_merged_asset.yaml'
MAP_FILE = 'kcity_map/lanelet_map/lanelet2_map_v94.osm'


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def verified(path, expected):
    if not path.is_file() or sha256(path) != expected:
        raise ValueError(f'SHA-256 mismatch or unreadable map: {path}; existing files are never overwritten')


def load_manifest(root):
    manifest = yaml.safe_load((root / MANIFEST).read_text())
    if not isinstance(manifest, dict) or manifest.get('map_version') != 94:
        raise ValueError('Expected the v94 merged-asset manifest')
    if manifest.get('map_file') != MAP_FILE or manifest.get('drive_path') != MAP_FILE:
        raise ValueError('Manifest must identify only the pinned v94 OSM path')
    if not re.fullmatch(r'[0-9a-f]{64}', str(manifest.get('map_sha256', ''))):
        raise ValueError('Manifest lacks a valid v94 SHA-256')
    for key in ('drive_root_folder_id', 'drive_file_id'):
        value = manifest.get(key)
        if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', value):
            raise ValueError(f'Manifest lacks a valid {key}')
    return manifest


def prepare(*, root=ROOT, check=False, source=None, remote=None, run=None):
    root = Path(root).resolve()
    manifest = load_manifest(root)
    target = root / manifest['map_file']
    expected = manifest['map_sha256']
    # Includes dangling symlinks: no existing directory entry may be replaced.
    if os.path.lexists(target):
        verified(target, expected)
        return target
    if check:
        raise FileNotFoundError(f'v94 map missing: {target}; run this command without --check')

    selected_remote = None
    if source is None:
        selected_remote = remote if remote is not None else os.environ.get(
            'KCITY_MAP_RCLONE_REMOTE', 'ajou_drive_private')
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', selected_remote):
            raise ValueError('Remote must be an explicit rclone configuration name, without a colon')
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix='.prepare-v94-', dir=target.parent)
    os.close(descriptor)
    temporary = Path(name)
    try:
        if source is not None:
            shutil.copyfile(Path(source), temporary)
        else:
            command = ['rclone', 'copyto', f'{selected_remote}:{manifest["drive_path"]}',
                       str(temporary), '--drive-root-folder-id', manifest['drive_root_folder_id']]
            try:
                result = (run or subprocess.run)(command, stdout=subprocess.PIPE,
                                                stderr=subprocess.PIPE, text=True, check=False)
            except FileNotFoundError as error:
                raise RuntimeError('rclone is not installed; install rclone or use --source FILE') from error
            if result.returncode:
                # Do not echo rclone output: it may include account/token details.
                raise RuntimeError(
                    f'v94 download failed (Drive file {manifest["drive_file_id"]}). '
                    f'If authentication expired, run: rclone config reconnect {selected_remote}: '
                    '(no public/shared-client fallback is attempted)')
        verified(temporary, expected)
        try:
            # Same-directory hardlink is atomic and cannot clobber a racing writer.
            os.link(temporary, target)
        except FileExistsError:
            verified(target, expected)
        return target
    finally:
        temporary.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='read-only local SHA check; no network or mkdir')
    mode.add_argument('--source', type=Path, metavar='FILE', help='copy an offline v94 asset instead of downloading')
    parser.add_argument('--remote', metavar='NAME', help='rclone config name; env KCITY_MAP_RCLONE_REMOTE or ajou_drive_private')
    args = parser.parse_args(argv)
    try:
        target = prepare(check=args.check, source=args.source, remote=args.remote)
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1
    print(f'PASS: v94 SHA-256 verified: {target}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
