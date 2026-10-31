#!/usr/bin/env python3
"""Restore only our trusted, qualified training archive into a fresh directory.

This is not an untrusted pickle/archive importer. The archive and its packing
manifest must both come from our qualified exporter. No network, source deletion,
training, capture or installation is performed. The default is a read-only plan.
Partial failures remain at their new output path for inspection, never resume.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path, PurePosixPath
import pickle
import re
import shutil
import subprocess
import tarfile
import time


SPEC = importlib.util.spec_from_file_location('retrieved_training_finish',
                                             Path(__file__).with_name('finish.py'))
finish = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(finish)

REQUIRED_PROVENANCE = {'provenance/selected_ledger.jsonl',
                       'provenance/split_overlay.json',
                       'provenance/qualification_report.json'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    def unique(pairs):
        value = {}
        for key, item in pairs:
            require(key not in value, 'Duplicate JSON key: ' + key)
            value[key] = item
        return value
    return json.loads(Path(path).read_text(), object_pairs_hook=unique)


def local_path(value, exists=True):
    raw = Path(value)
    require('..' not in raw.parts, 'Normalized local path required')
    path = raw.absolute()
    for parent in (path, *path.parents):
        require(not parent.is_symlink(), 'Symlink local path forbidden')
    if exists:
        require(path.is_file(), 'Trusted regular local file required')
    return path


def member_name(name):
    require(isinstance(name, str) and name and '\\' not in name and '\0' not in name,
            'Unsafe archive member name')
    path = PurePosixPath(name)
    require(not path.is_absolute() and len(path.parts) >= 2
            and path.parts[0] in ('dataset', 'provenance')
            and all(part not in ('', '.', '..') for part in name.split('/'))
            and path.as_posix() == name, 'Unsafe archive member path: ' + name)
    return name


def make_plan(args):
    require(args.trusted_generated_archive is True,
            'Only our trusted generated archive/pickles are supported')
    require(args.expected_scenes > 0 and args.min_free_gib >= 0, 'Invalid count/disk budget')
    archive = local_path(args.archive)
    manifest_path = local_path(args.archive_manifest)
    output = local_path(args.output, exists=False)
    require(not output.exists(), 'Fresh output required; partial outputs are preserved')
    require(not finish.within(archive, output) and not finish.within(manifest_path, output),
            'Output overlaps protected inputs')
    document = read_json(manifest_path)
    require(type(document.get('bytes')) is int and document['bytes'] > 0
            and re.fullmatch('[a-f0-9]{64}', document.get('sha256', '')),
            'Trusted archive size/SHA binding required')
    require(archive.stat().st_size == document['bytes']
            and finish.digest(archive) == document['sha256'], 'Archive size/SHA256 mismatch')
    members = document.get('members')
    require(isinstance(members, dict) and members, 'Full trusted archive member inventory required')
    for name, item in members.items():
        member_name(name)
        require(isinstance(item, dict) and type(item.get('bytes')) is int and item['bytes'] >= 0
                and re.fullmatch('[a-f0-9]{64}', item.get('sha256', '')),
                'Invalid archive member size/SHA binding')
    require('dataset/manifest.json' in members and REQUIRED_PROVENANCE <= set(members),
            'Qualified dataset and final provenance are required')
    # A regular file must never also be a parent directory of another member.
    require(not any(str(parent) in members for name in members
                    for parent in PurePosixPath(name).parents), 'Archive file/directory prefix conflict')
    anchor = output.parent
    while not anchor.exists():
        anchor = anchor.parent
    required_bytes = sum(item['bytes'] for item in members.values())
    require(shutil.disk_usage(anchor).free >= required_bytes + args.min_free_gib * finish.GIB,
            'Insufficient free disk for verified extraction')
    return dict(schema='base5_received_training_restore_v1', archive=str(archive),
        archive_manifest=str(manifest_path), archive_sha256=document['sha256'],
        archive_manifest_sha256=finish.digest(manifest_path), archive_bytes=document['bytes'],
        output=str(output), expected_scenes=args.expected_scenes,
        members=len(members), extracted_bytes=required_bytes,
        trusted_generated_archive=True, network=False, training_started=False), members


def extract_verified(archive, output, expected):
    seen = set()
    process = subprocess.Popen(['zstd', '-q', '-d', '-c', str(archive)], stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=process.stdout, mode='r|') as stream:
            for item in stream:
                name = member_name(item.name)
                require(item.isreg() and not item.sparse and name in expected and name not in seen,
                        'Unexpected/non-regular/duplicate archive member')
                require(item.size == expected[name]['bytes'], 'Archive member declared size mismatch')
                target = output / name
                target.parent.mkdir(parents=True, exist_ok=True)
                # No archive extraction APIs: no ownership/mode/link metadata is applied.
                for parent in (target, *target.parents):
                    require(not parent.is_symlink(), 'Symlink appeared at extraction destination')
                digest, size = hashlib.sha256(), 0
                with stream.extractfile(item) as source, target.open('xb') as destination:
                    for block in iter(lambda: source.read(4 * 1024 ** 2), b''):
                        destination.write(block)
                        digest.update(block)
                        size += len(block)
                require(dict(bytes=size, sha256=digest.hexdigest()) == expected[name],
                        'Archive member bytes/SHA256 mismatch: ' + name)
                seen.add(name)
        require(process.wait() == 0 and seen == set(expected),
                'Decompression failed or archive member inventory differs')
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait()
        if process.stdout is not None:
            process.stdout.close()


def verify_package(output, expected, count):
    package = output / 'dataset'
    # Byte verification above happens before any trusted generated pickle loads.
    manifest = finish.validate_training_package(package, count)
    dataset_members = {'dataset/' + name for name in manifest['files']} | {'dataset/manifest.json'}
    require(dataset_members == {name for name in expected if name.startswith('dataset/')},
            'Archive dataset contains missing/unregistered payloads')
    require(all(expected['dataset/' + name] == item for name, item in manifest['files'].items()),
            'Packing inventory differs from qualified dataset manifest')
    counts, cameras = Counter(), set()
    for split in ('train', 'val', 'test'):
        info = pickle.loads((package / 'infos/512x1056' /
                             ('morai_base5_infos_' + split + '.pkl')).read_bytes())
        for row in info['data_list']:
            counts[split] += 1
            require(len(row['images']) == 4, 'Four camera views per frame required')
            cameras.update(camera['img_path'] for camera in row['images'].values())
    require(dict(counts) == manifest['split_counts'] and len(cameras) == count * 4,
            'Split counts/unique camera image count differs')
    return dict(package=str(package), frames=count, split_counts=dict(counts),
                unique_camera_images=len(cameras), manifest_sha256=finish.digest(package / 'manifest.json'))


def restore(args):
    plan, members = make_plan(args)
    result = dict(plan, execute=bool(args.run), complete=False)
    if not args.run:
        return result
    output = Path(plan['output'])
    output.mkdir(parents=True, exist_ok=False)
    finish.write_new(output / 'RESTORE_INTENT.json', dict(plan, started_at=time.time()))
    try:
        extract_verified(Path(plan['archive']), output, members)
        package = verify_package(output, members, args.expected_scenes)
        require(finish.digest(Path(plan['archive_manifest'])) == plan['archive_manifest_sha256']
                and Path(plan['archive']).stat().st_size == plan['archive_bytes']
                and finish.digest(Path(plan['archive'])) == plan['archive_sha256'],
                'Trusted archive/manifest changed during restoration')
        result.update(package, complete=True, verified_decompressed_members=True,
                      qualified_dataset_verified=True, completed_at=time.time())
        finish.write_new(output / 'COMPLETE.json', result)
        return result
    except BaseException as error:
        finish.write_new(output / 'INCOMPLETE.json', dict(complete=False,
            error_type=type(error).__name__, error=str(error), partial_products_preserved=True,
            source_unchanged=True, note='Use a fresh output after diagnosing; no overwrite/resume.'))
        raise


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    for name in ('archive', 'archive-manifest', 'output'):
        result.add_argument('--' + name, type=Path, required=True)
    result.add_argument('--expected-scenes', type=int, default=5000)
    result.add_argument('--min-free-gib', type=float, default=10.)
    result.add_argument('--trusted-generated-archive', action='store_true')
    result.add_argument('--run', action='store_true')
    return result


if __name__ == '__main__':
    print(json.dumps(restore(parser().parse_args()), indent=2))
