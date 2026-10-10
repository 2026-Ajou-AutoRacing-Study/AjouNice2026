#!/usr/bin/env python3
"""Export selected Base5 captures and crop infos without local-path dependencies.

Only load trusted local pickle files. Source captures/ledgers are never modified.
The manifest includes native annotation and provenance, not just model inputs.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import pickle
import re
import shutil


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def safe_relative(value):
    path = Path(value)
    if path.is_absolute() or '..' in path.parts or not path.parts:
        raise ValueError('Unsafe relative path: ' + str(value))
    return path


def rewrite_record(record, capture_root, destination):
    record = deepcopy(record)
    fields = [(record['lidar_points'], 'lidar_path')]
    fields += [(camera, 'img_path') for camera in record['images'].values()]
    if len(record['images']) != 4 or record.get('lidar_sweeps'):
        raise ValueError('Expected four-camera single-frame inputs')
    for owner, key in fields:
        source = Path(owner[key]).resolve(strict=True)
        relative = source.relative_to(capture_root)
        if not source.is_file():
            raise ValueError('Input is not a file: ' + str(source))
        owner[key] = str(destination / relative)
    return record


def build(ledger, profiles, output, expected_scenes=5000, copy_mode='copy'):
    ledger = Path(ledger).resolve(strict=True)
    output = Path(output).absolute()
    if output.exists():
        raise FileExistsError('Use a fresh output: ' + str(output))
    rows = [json.loads(line) for line in ledger.read_text().splitlines() if line]
    roots = {}
    for row in rows:
        token = row['id']
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', token) or token in roots:
            raise ValueError('Invalid/duplicate capture token: ' + token)
        roots[token] = Path(row['capture_root']).resolve(strict=True)
    if len(roots) != expected_scenes:
        raise ValueError('Scene count mismatch')
    exports = {}; source_infos = {}; reference_split = None
    for profile, folder in profiles.items():
        if not re.fullmatch(r'[A-Za-z0-9_-]+', profile):
            raise ValueError('Invalid profile')
        folder = Path(folder).resolve(strict=True)
        exports[profile] = {}; scene_splits = {}; groups = {}; source_infos[profile] = {}
        for split in ('train', 'val', 'test'):
            source = folder / ('morai_base5_infos_' + split + '.pkl')
            info = pickle.loads(source.read_bytes())
            source_infos[profile][split] = sha256(source)
            if tuple(info['metainfo']['classes']) != ('vehicle', 'pedestrian', 'obstacle', 'animal'):
                raise ValueError('Unexpected class contract')
            data = []
            for record in info['data_list']:
                token = record['token']
                if token not in roots:
                    raise ValueError('Info references unselected capture: ' + token)
                if split != 'test':
                    if token in scene_splits:
                        raise ValueError('Duplicate/train-val overlapping token')
                    scene_splits[token] = split
                    group = record['scene_group_id']
                    if group in groups and groups[group] != split:
                        raise ValueError('Train/val group leakage')
                    groups[group] = split
                data.append(rewrite_record(record, roots[token], Path('data') / token))
            exports[profile][split] = dict(info, data_list=data)
        if set(scene_splits) != set(roots):
            raise ValueError('Train/val does not cover the selected ledger')
        if reference_split is not None and reference_split != scene_splits:
            raise ValueError('Crop profiles disagree on scene split')
        reference_split = scene_splits
        # No synthetic held-out test is required: preserve empty test infos or
        # an explicit val alias, never silently create independent test data.
        if exports[profile]['test']['data_list'] and exports[profile]['test']['data_list'] != exports[profile]['val']['data_list']:
            raise ValueError('Unexpected test contract; review before exporting')
    output.mkdir(parents=True)
    inventory = {}; provenance = {}
    def transfer(source, relative):
        relative = safe_relative(relative)
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if copy_mode == 'hardlink':
            os.link(source, target)
        else:
            shutil.copy2(source, target)
        inventory[str(relative)] = dict(bytes=target.stat().st_size, sha256=sha256(target))
    for index, (token, root) in enumerate(roots.items(), 1):
        count = 0
        for source in sorted(root.rglob('*')):
            if source.is_symlink():
                raise ValueError('Native capture contains symlink: ' + str(source))
            if source.is_file():
                transfer(source, Path('data') / token / source.relative_to(root)); count += 1
        if not count:
            raise ValueError('Empty capture root')
        provenance[token] = dict(original_capture_root=str(root), files=count)
        if index % 250 == 0:
            print('Exported %d/%d captures' % (index, expected_scenes), flush=True)
    transfer(ledger, Path('provenance') / 'original_selected_ledger.jsonl')
    for name in ('adopted_manifest.json', 'replacement_mapping.json', 'validation_report.json'):
        source = ledger.parent / name
        if source.is_file():
            transfer(source, Path('provenance') / name)
    for profile, splits in exports.items():
        for split, info in splits.items():
            path = output / 'infos' / profile / ('morai_base5_infos_' + split + '.pkl')
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(pickle.dumps(info, protocol=4))
            inventory[str(path.relative_to(output))] = dict(bytes=path.stat().st_size, sha256=sha256(path))
        source = Path(profiles[profile]) / 'calibration.json'
        transfer(source, Path('infos') / profile / 'calibration.json')
    manifest = dict(version=1, source_ledger_sha256=sha256(ledger), scenes=expected_scenes,
        split_counts=dict(Counter(reference_split.values())), profiles=list(profiles),
        source_info_sha256=source_infos, copy_mode=copy_mode,
        bytes=sum(item['bytes'] for item in inventory.values()), files=inventory,
        provenance=provenance, training_paths='relative to the package root',
        test_contract='synthetic test infos preserved (empty or val alias); mission held-out bags are separate',
        hardlink_warning='Do not mutate exported inputs; local hardlinks share source inodes.')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    validate(output)
    return manifest


def validate(root, verify_hashes=False):
    root = Path(root).resolve(strict=True)
    manifest = json.loads((root / 'manifest.json').read_text())
    policy = manifest.get('image_policy')
    if policy and policy != dict(train='native_jpeg90', val='original_png', test='original_png'):
        raise ValueError('Unsupported portable image policy')
    for relative, item in manifest['files'].items():
        path = root / safe_relative(relative)
        path.resolve(strict=True).relative_to(root)
        if not path.is_file() or path.stat().st_size != item['bytes']:
            raise ValueError('Missing/changed payload: ' + relative)
        if verify_hashes and sha256(path) != item['sha256']:
            raise ValueError('Payload checksum mismatch: ' + relative)
    for profile in manifest['profiles']:
        for split in ('train', 'val', 'test'):
            info = pickle.loads((root / 'infos' / profile / ('morai_base5_infos_' + split + '.pkl')).read_bytes())
            for record in info['data_list']:
                if policy:
                    expected = '.jpg' if split == 'train' else '.png'
                    if any(Path(c['img_path']).suffix.lower() != expected for c in record['images'].values()):
                        raise ValueError('Portable image encoding/split policy differs')
                paths = [record['lidar_points']['lidar_path']]
                paths += [camera['img_path'] for camera in record['images'].values()]
                for value in paths:
                    relative = str(safe_relative(value))
                    if relative not in manifest['files']:
                        raise ValueError('Info points outside payload inventory: ' + value)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ledger', type=Path)
    parser.add_argument('--profile', action='append', default=[], help='name=/path/to/infos')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--expected-scenes', type=int, default=5000)
    parser.add_argument('--hardlink', action='store_true')
    parser.add_argument('--verify', type=Path)
    args = parser.parse_args()
    if args.verify:
        manifest = validate(args.verify, verify_hashes=True)
    else:
        if not args.ledger or not args.output or not args.profile:
            parser.error('Provide ledger, output and at least one profile')
        profiles = dict(item.split('=', 1) for item in args.profile)
        if len(profiles) != len(args.profile):
            parser.error('Duplicate profile')
        manifest = build(args.ledger, profiles, args.output, args.expected_scenes,
                         'hardlink' if args.hardlink else 'copy')
    print(json.dumps({key: manifest[key] for key in ('scenes', 'split_counts', 'profiles', 'bytes')}, indent=2))


if __name__ == '__main__':
    main()
