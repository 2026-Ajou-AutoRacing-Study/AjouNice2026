#!/usr/bin/env python3
"""Prepare a credential/data-free build context and optionally build. Never push."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
BASE = 'ajounice/bevfusion:team-cu128-v1'
BASE_ID = 'sha256:13488ccac149f41b070e618db5db91bfa3bf2d3d1a6de3dac80aa2e2c948ec46'
SUFFIXES = {'.py', '.pyi', '.yaml', '.yml', '.json', '.toml', '.ini', '.cfg',
            '.txt', '.md', '.cpp', '.cu', '.h', '.hpp', '.cuh', '.sh', '.xml'}


def tracked(repo):
    return subprocess.check_output(['git', '-C', str(repo), 'ls-files', '-z']).decode().split('\0')[:-1]


def selected():
    paths = []
    for name in tracked(ROOT):
        p = Path(name)
        allowed = (name.startswith(('perception/detection3d/bevfusion/',
                                   'perception/detection3d/common/',
                                   'docker/bevfusion/app/', 'configs/sensors/'))
                   or (name.startswith('dataset/morai_od/scripts/') and p.suffix == '.py')
                   or name in ('perception/__init__.py', 'perception/detection3d/__init__.py'))
        if allowed and p.suffix in SUFFIXES:
            paths.append((ROOT / p, p))
    fork = ROOT / 'third_party/mmdetection3d'
    for name in tracked(fork):
        p = Path(name)
        if p.suffix in SUFFIXES or p.name in ('LICENSE', 'MANIFEST.in'):
            paths.append((fork / p, Path('third_party/mmdetection3d') / p))
    return paths


def prepare(context):
    if context.exists():
        raise ValueError('Use a new empty context path; existing contexts are never overwritten')
    context.mkdir(parents=True)
    inventory = {}
    for source, relative in selected():
        if source.is_symlink() or not source.is_file():
            raise ValueError('Only regular tracked source files allowed: ' + str(relative))
        target = context / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        inventory[str(relative)] = hashlib.sha256(target.read_bytes()).hexdigest()
    revision = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip()
    receipt = dict(revision=revision, base_image=BASE, required_base_image_id=BASE_ID,
                   files=inventory, scope='Tracked source allowlist only; no dataset, weights or credentials')
    (context / 'source_inventory.json').write_text(json.dumps(receipt, indent=2) + '\n')
    shutil.copyfile(ROOT / 'docker/bevfusion/Dockerfile.vast-training', context / 'Dockerfile')
    shutil.copyfile(ROOT / 'docker/bevfusion/scripts/verify_vast_training_image.py',
                    context / 'verify_vast_training_image.py')
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context', type=Path, required=True)
    parser.add_argument('--image', default='fluffyluv/ajounice2026-bevfusion-train:base5-cu128-261018-v1')
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args()
    receipt = prepare(args.context)
    print(json.dumps(dict(context=str(args.context), source_files=len(receipt['files']),
                          bytes=sum(p.stat().st_size for p in args.context.rglob('*') if p.is_file()))), flush=True)
    if args.build:
        actual = subprocess.check_output(['docker', 'image', 'inspect', BASE, '--format', '{{.Id}}'], text=True).strip()
        if actual != BASE_ID:
            raise ValueError('Base image differs from inspected local runtime; qualification required')
        subprocess.run(['docker', 'build', '--build-arg', 'SOURCE_REVISION=' + receipt['revision'],
                        '--build-arg', 'BASE_IMAGE_ID=' + actual, '-t', args.image,
                        str(args.context)], check=True)


if __name__ == '__main__':
    main()
