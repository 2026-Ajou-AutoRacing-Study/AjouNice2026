#!/usr/bin/env python3
"""Explicit, receipt-bound deletion of selected camera RGB PNG names only.

``plan`` is read-only unless --write-plan is supplied. ``delete`` requires that
plan's SHA and an explicit acknowledgement that Q90 JPEG is lossy. Native GT,
masks, depth, rejected captures and historical integrity proofs are never edited.
Only trusted local pickle packages produced by the existing CPU exporter apply.
"""
import argparse
from collections import Counter
from copy import deepcopy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pickle
import re
import stat
import sys
import time

from PIL import Image

SCHEMA = 'base5_selected_rgb_png_prune_v1'
CAMERAS = {'CAMERA_' + str(n) for n in range(1, 5)}
POLICY = ('User-authorized irreversible removal of selected RGB PNG names; Q90 JPEG '
          'is lossy, not a lossless source backup. Historical native contracts/proofs '
          'stay unchanged and may no longer replay source-RGB checks after cleanup. '
          'Camera Intensity PNG may be RGB or RGBA; the existing model/exporter '
          'uses PIL.convert("RGB"), discarding alpha without compositing. '
          'Instance/semantic/depth alpha is not in this deletion scope.')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 ** 2), b''):
            h.update(block)
    return h.hexdigest()


def canonical(path, directory=False):
    path = Path(os.path.abspath(os.fspath(path)))
    require('..' not in Path(path).parts, 'Non-normalized path')
    for ancestor in (path, *path.parents):
        require(not ancestor.is_symlink(), 'Symlink forbidden: ' + str(ancestor))
    info = path.lstat()
    require(stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode),
            'Wrong path type: ' + str(path))
    require(info.st_uid == os.getuid(), 'Foreign-owned path: ' + str(path))
    return path


def relative(value):
    p = Path(value)
    require(not p.is_absolute() and '..' not in p.parts and p.parts,
            'Unsafe package relative path: ' + str(value))
    return p


def within(path, root):
    return path != root and root in path.parents


def read(path):
    return json.loads(canonical(path).read_text())


def snapshot(path):
    p = canonical(path)
    item = p.stat()
    return dict(bytes=item.st_size, sha256=digest(p), device=item.st_dev,
                inode=item.st_ino, mtime_ns=item.st_mtime_ns)


class ReadCache:
    """One inspection only; aliases of an unchanged inode share disk reads.

    Every access still validates the named path and freshly lstat()s it. Never
    persist these results, share them between inspect() calls, or use them for
    the immediate pre-unlink check. Changing inode/size/mtime creates a new key.
    """
    def __init__(self):
        self.hashes = {}
        self.decoded = {}

    @staticmethod
    def identity(path):
        path = canonical(path)
        item = path.lstat()
        return path, (item.st_dev, item.st_ino, item.st_size, item.st_mtime_ns)

    def snapshot(self, path):
        path, key = self.identity(path)
        if key not in self.hashes:
            value = digest(path)
            require(self.identity(path)[1] == key, 'Input changed during checksum: ' + str(path))
            self.hashes[key] = dict(device=key[0], inode=key[1], bytes=key[2],
                                    mtime_ns=key[3], sha256=value)
        return dict(self.hashes[key])

    def digest(self, path):
        return self.snapshot(path)['sha256']

    def image_shape(self, path, encoding, width, height):
        path, key = self.identity(path)
        signature = (key, encoding, width, height)
        if signature not in self.decoded:
            mode = image_shape(path, encoding, width, height)
            require(self.identity(path)[1] == key, 'Input changed during image decode: ' + str(path))
            self.decoded[signature] = mode
        return self.decoded[signature]


def checked_payload(root, name, expected, reads=None):
    p = canonical(root / relative(name))
    require(within(p, root), 'Payload escaped package')
    actual = reads.snapshot(p) if reads is not None else snapshot(p)
    require(actual['bytes'] == expected['bytes'] and actual['sha256'] == expected['sha256'],
            'Payload differs from immutable manifest: ' + str(p))
    return p


def rows(path, count):
    payload = canonical(path).read_bytes()
    require(payload.endswith(b'\n'), 'Strict newline-terminated selection required')
    result = [json.loads(line) for line in payload.splitlines()]
    require(len(result) == count and count > 0, 'Selection count differs')
    for key in ('id', 'capture_root', 'scenario_sha256'):
        require(len({r[key] for r in result}) == count, 'Duplicate selected ' + key)
    require(len({r.get('source_cell_id', r['id']) for r in result}) == count,
            'Duplicate canonical quota slot')
    require(all(r.get('status') == 'accepted_candidate' for r in result),
            'Unapproved input is not deletable')
    return result


def identities(records):
    return {(r['id'], r['capture_root'], r['scenario_sha256']) for r in records}


def image_shape(path, encoding, width, height):
    with Image.open(path) as image:
        require(image.format == encoding and image.size == (width, height),
                'Image encoding/native dimensions differ: ' + str(path))
        image.load()  # A header-only or truncated JPEG must not authorize deletion.
        modes = ('RGB', 'RGBA') if encoding == 'PNG' else ('RGB',)
        require(image.mode in modes, 'Expected native RGB/RGBA PNG or RGB JPEG image: ' + str(path))
        return image.mode


def merge_record(record):
    result = deepcopy(record)
    classes = ('vehicle', 'pedestrian', 'obstacle', 'animal')
    for instance in result['instances']:
        label = instance['bbox_label_3d']
        require(type(label) is int and label in range(4), 'Invalid original GT label')
        require('original_bbox_label_3d' not in instance and
                'original_semantic_class' not in instance, 'Already merged PNG source')
        instance['original_bbox_label_3d'] = label
        instance['original_semantic_class'] = classes[label]
        instance['bbox_label_3d'] = 1 if label == 3 else label
    return result


def inspect(selected, png_package, jpeg_cache, jpeg_package, raw_allow_roots,
            expected_scenes=5000, delete_png_package_rgb=False, progress=False):
    reads = ReadCache()  # New independent cache on every dry-run/pre-delete inspection.
    selected = canonical(selected)
    png, cache, jpeg = [canonical(p, True) for p in (png_package, jpeg_cache, jpeg_package)]
    require(len({png, cache, jpeg}) == 3, 'Distinct source/cache/final packages required')
    allow = sorted({canonical(p, True) for p in raw_allow_roots})
    require(allow and all(p != Path('/') and len(p.parts) >= 4 for p in allow),
            'Narrow explicit raw allow roots are required')
    for root in allow:
        require(not any(within(pkg, root) or within(root, pkg) or root == pkg
                        for pkg in (png, cache, jpeg)), 'Raw/package scopes overlap')
    chosen = rows(selected, expected_scenes)
    by_token = {r['id']: r for r in chosen}
    pm, cm, jm = [read(root / 'manifest.json') for root in (png, cache, jpeg)]
    require(pm['scenes'] == jm['scenes'] == expected_scenes and
            pm['profiles'] == jm['profiles'] == ['512x1056'], 'Package scope differs')
    require(not pm.get('image_policy') and not pm.get('training_label_policy'),
            'Original four-class PNG portable package required')
    require(jm.get('image_policy') == dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90')
            and jm.get('training_label_policy') == 'animal_to_pedestrian_v1',
            'Current all-split native Q90 three-class package required')
    require(cm.get('complete') is True and cm.get('quality') == 90 and
            cm.get('native_resolution') is True and cm.get('source_color_order') == 'rgb'
            and cm.get('source_manifest_sha256') == reads.digest(png / 'manifest.json'),
            'JPEG cache is not source-bound complete native Q90')
    provenance_ledger = checked_payload(png, 'provenance/original_selected_ledger.jsonl',
                                        pm['files']['provenance/original_selected_ledger.jsonl'], reads)
    require(pm['source_ledger_sha256'] == reads.digest(provenance_ledger) and
            identities(rows(provenance_ledger, expected_scenes)) == identities(chosen),
            'Portable source does not select exactly the final captures')
    require(set(pm['provenance']) == set(by_token), 'Portable provenance differs from selection')
    binding = {}
    for split in ('train', 'val', 'test'):
        name = 'infos/512x1056/morai_base5_infos_' + split + '.pkl'
        left = pickle.loads(checked_payload(png, name, pm['files'][name], reads).read_bytes())
        right = pickle.loads(checked_payload(jpeg, name, jm['files'][name], reads).read_bytes())
        require(tuple(left['metainfo']['classes']) == ('vehicle', 'pedestrian', 'obstacle', 'animal')
                and tuple(right['metainfo']['classes']) == ('vehicle', 'pedestrian', 'obstacle'),
                'GT class contract differs')
        require(left['metainfo'].get('box_geometry_policy') ==
                right['metainfo'].get('box_geometry_policy'), 'Upright box policy changed')
        require(len(left['data_list']) == len(right['data_list']), 'Split row count differs')
        require(split != 'test' or not left['data_list'], 'Independent empty test contract required')
        for a, b in zip(left['data_list'], right['data_list']):
            token = a['token']
            require(token == b['token'] and token in by_token and token not in binding,
                    'Duplicate/missing/mismatched GT row')
            expected = merge_record(a)
            require(set(a['images']) == set(b['images']) and len(a['images']) == 4,
                    'Four camera identities required')
            raw = canonical(by_token[token]['capture_root'], True)
            require(any(within(raw, root) for root in allow), 'Selected raw root outside deletion allowlist')
            require(pm['provenance'][token]['original_capture_root'] == str(raw),
                    'Capture provenance root differs')
            require(reads.digest(canonical(raw / 'scenario.json')) == by_token[token]['scenario_sha256'],
                    'Selected raw scenario changed')
            cameras = []
            native_camera_directories = set()
            for camera, original in a['images'].items():
                source_relative = relative(original['img_path'])
                require(source_relative.parts[:2] == ('data', token) and
                        len(source_relative.parts) >= 4 and source_relative.parts[2] in CAMERAS
                        and re.fullmatch(r'.+_Intensity\.png', source_relative.name),
                        'Only selected CAMERA_1..4 native Intensity PNG is deletable')
                native_relative = Path(*source_relative.parts[2:])
                native_camera_directories.add(native_relative.parts[0])
                native = canonical(raw / native_relative)
                portable = checked_payload(png, str(source_relative), pm['files'][str(source_relative)], reads)
                source_info = reads.snapshot(native)
                require(source_info['bytes'] == pm['files'][str(source_relative)]['bytes'] and
                        source_info['sha256'] == pm['files'][str(source_relative)]['sha256'],
                        'Native PNG differs from portable source')
                item = cm['files'][str(source_relative)]
                require(item['source_sha256'] == source_info['sha256'] and
                        item['source_bytes'] == source_info['bytes'] and
                        (item['height'], item['width']) == (original['height'], original['width']),
                        'Native PNG to JPEG mapping differs')
                cached = checked_payload(cache, item['path'], item, reads)
                target_relative = str(relative(b['images'][camera]['img_path']))
                require(target_relative == str(Path('jpeg_rgb') / Path(item['path']).name),
                        'Final JPEG path is not the exact cache mapping')
                target = checked_payload(jpeg, target_relative, jm['files'][target_relative], reads)
                require(reads.digest(target) == item['sha256'] and target.suffix == '.jpg',
                        'Final JPEG payload differs from Q90 cache')
                decoded_modes = {}
                for path, encoding in ((native, 'PNG'), (portable, 'PNG'),
                                       (cached, 'JPEG'), (target, 'JPEG')):
                    decoded_modes[str(path)] = reads.image_shape(path, encoding, original['width'], original['height'])
                expected['images'][camera]['img_path'] = target_relative
                cameras.append(dict(camera=camera, raw_png=str(native), png_package=str(portable),
                    png_mode=decoded_modes[str(native)],
                    png_color_policy='PIL.convert("RGB"); alpha discarded without compositing',
                    png_alpha_discarded=decoded_modes[str(native)] == 'RGBA',
                    png=source_info, portable_png=reads.snapshot(portable), jpeg=str(target),
                    jpeg_info=reads.snapshot(target), cached_jpeg=str(cached), cached_info=reads.snapshot(cached)))
            require(native_camera_directories == CAMERAS,
                    'Exactly one RGB image from each CAMERA_1..4 is required')
            require(expected == b, 'GT boxes, split/group, calibration or non-image fields changed')
            binding[token] = dict(split=split, scene_group_id=a['scene_group_id'],
                                  capture_root=str(raw), cameras=cameras)
            if progress and len(binding) % 250 == 0:
                print(json.dumps(dict(event='rgb_prune_verified', scenes=len(binding),
                                      total=expected_scenes)), flush=True)
    require(set(binding) == set(by_token), 'Not every selected frame has four verified JPEGs')
    groups = {}
    for token, row in binding.items():
        group = row['scene_group_id']
        require(group not in groups or groups[group] == row['split'], 'Train/val group leakage')
        groups[group] = row['split']
    rgb = {str(relative(c['img_path'])) for split in ('train', 'val') for r in
           pickle.loads((png / ('infos/512x1056/morai_base5_infos_' + split + '.pkl')).read_bytes())['data_list']
           for c in r['images'].values()}
    require(len(rgb) == expected_scenes * 4 and set(cm['files']) == rgb and
            cm.get('image_count') == len(rgb), 'RGB/cache inventory is not exactly four per selection')
    # Native GT/masks/depth/audits remain byte-identical in the training package.
    # Original infos are intentionally replaced only by checked three-class infos.
    auxiliary = {}
    for name, expected in pm['files'].items():
        if name in rgb or name.startswith('infos/'):
            continue
        require(jm['files'].get(name) == expected, 'Lossless auxiliary inventory changed: ' + name)
        src = checked_payload(png, name, expected, reads)
        dst = checked_payload(jpeg, name, expected, reads)
        auxiliary[str(dst)] = reads.snapshot(dst)
        if name.startswith('data/'):
            parts = relative(name).parts
            raw_file = canonical(Path(by_token[parts[1]]['capture_root']) / Path(*parts[2:]))
            require(reads.digest(raw_file) == expected['sha256'], 'Native auxiliary source changed')
            auxiliary[str(raw_file)] = reads.snapshot(raw_file)
    candidates = {}
    for row in binding.values():
        for camera in row['cameras']:
            for key, info_key in [('raw_png', 'png')] + ([('png_package', 'portable_png')]
                                                     if delete_png_package_rgb else []):
                path = camera[key]
                require(path not in candidates, 'Duplicate deletion path')
                candidates[path] = camera[info_key]
    control_paths = [selected, png / 'manifest.json', cache / 'manifest.json', jpeg / 'manifest.json']
    inodes = {}
    for path, item in candidates.items():
        key = (item['device'], item['inode'])
        group = inodes.setdefault(key, dict(bytes=item['bytes'], names=0,
                                            hard_links=canonical(path).stat().st_nlink))
        group['names'] += 1
    reclaim = sum(group['bytes'] for group in inodes.values()
                  if group['names'] == group['hard_links'])
    return dict(schema=SCHEMA, policy=POLICY, selected=str(selected),
        png_package=str(png), jpeg_cache=str(cache), jpeg_package=str(jpeg),
        raw_allow_roots=list(map(str, allow)), expected_scenes=expected_scenes,
        delete_png_package_rgb=bool(delete_png_package_rgb),
        helper_sha256=reads.digest(__file__), controls={str(p): reads.snapshot(p) for p in control_paths},
        split_counts=dict(Counter(r['split'] for r in binding.values())),
        scenes=binding, candidates=candidates, preserved_auxiliary=auxiliary,
        logical_bytes=sum(r['bytes'] for r in candidates.values()),
        unique_inode_bytes=sum(group['bytes'] for group in inodes.values()),
        reclaimable_inode_bytes_estimate=reclaim,
        externally_retained_hardlink_inodes=sum(group['names'] < group['hard_links']
                                               for group in inodes.values()),
        warning='Removing portable RGB invalidates historical PNG portable validation; its manifest is preserved, not rewritten.')


def write_new(path, document):
    path = Path(path)
    require(not path.exists() and not path.is_symlink(), 'Fresh receipt output required')
    path.parent.mkdir(parents=True, exist_ok=True)
    canonical(path.parent, True)
    with path.open('x') as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def verify_bound(path, expected):
    require(snapshot(path) == expected, 'Pinned file changed before deletion: ' + str(path))


def delete(plan_path, plan_sha256, receipt_dir, ack_lossy=False):
    # Lock the immutable plan inode, not a particular receipt directory, so two
    # accidental invocations with different receipt destinations cannot race.
    plan_path = canonical(plan_path)
    with plan_path.open('rb') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return _delete_locked(plan_path, plan_sha256, receipt_dir, ack_lossy)


def _delete_locked(plan_path, plan_sha256, receipt_dir, ack_lossy=False):
    require(ack_lossy, 'Explicit acknowledgement of lossy irreversible source deletion required')
    plan_path = canonical(plan_path)
    require(digest(plan_path) == plan_sha256, 'Deletion plan SHA differs')
    plan = read(plan_path)
    require(plan['schema'] == SCHEMA and plan['policy'] == POLICY and
            plan['helper_sha256'] == digest(__file__), 'Prune policy/helper changed')
    receipt = Path(os.path.abspath(os.fspath(receipt_dir)))
    if receipt.exists():
        canonical(receipt, True)
    else:
        receipt.mkdir(parents=True)
        canonical(receipt, True)
    for control, expected in plan['controls'].items():
        verify_bound(control, expected)
    # Rebuild the entire plan before the first unlink; edited plan dictionaries,
    # moved inputs, corrupt JPEG and changed GT cannot authorize even one delete.
    current = inspect(plan['selected'], plan['png_package'], plan['jpeg_cache'],
        plan['jpeg_package'], plan['raw_allow_roots'], plan['expected_scenes'],
        plan['delete_png_package_rgb'], progress=True)
    require(current == plan, 'Fresh complete verification differs from approved dry-run plan')
    require(not any(receipt.iterdir()), 'Fresh deletion receipt directory required; interrupted receipts need review')
    lock = receipt / '.delete.lock'
    with lock.open('x') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        write_new(receipt / 'intent.json', dict(plan=str(plan_path), plan_sha256=plan_sha256,
                  policy=POLICY, started_at=time.time(), expected_files=len(plan['candidates'])))
        journal = receipt / 'deleted.jsonl'
        completed = 0
        try:
            with journal.open('x') as stream:
                for path, expected in plan['candidates'].items():
                    verify_bound(path, expected)
                    p = Path(path)
                    # Recheck both inode/hash and parent inode immediately around
                    # dir_fd-relative unlink. Do not follow a replacement symlink.
                    parent = canonical(p.parent, True)
                    fd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
                    try:
                        require(os.fstat(fd).st_ino == parent.stat().st_ino,
                                'Parent directory changed')
                        leaf = os.stat(p.name, dir_fd=fd, follow_symlinks=False)
                        require(stat.S_ISREG(leaf.st_mode) and leaf.st_ino == expected['inode']
                                and leaf.st_dev == expected['device'] and leaf.st_size == expected['bytes']
                                and leaf.st_mtime_ns == expected['mtime_ns'], 'Deletion inode changed')
                        os.unlink(p.name, dir_fd=fd)
                        os.fsync(fd)
                    finally:
                        os.close(fd)
                    stream.write(json.dumps(dict(path=path, before=expected, deleted_at=time.time())) + '\n')
                    stream.flush(); os.fsync(stream.fileno())
                    completed += 1
                    if completed % 1000 == 0:
                        print(json.dumps(dict(event='rgb_prune_deleted', files=completed,
                                              total=len(plan['candidates']))), flush=True)
            for path, expected in plan['preserved_auxiliary'].items():
                verify_bound(path, expected)
            for path, expected in plan['controls'].items():
                verify_bound(path, expected)
            require(not any(Path(p).exists() or Path(p).is_symlink() for p in plan['candidates']),
                    'Deleted PNG path reappeared')
            result = dict(schema=SCHEMA, complete=True, removed_files=completed,
                          logical_bytes_unlinked=plan['logical_bytes'], plan_sha256=plan_sha256,
                          reclaimable_inode_bytes_estimate=plan['reclaimable_inode_bytes_estimate'],
                          preserved_auxiliary_files=len(plan['preserved_auxiliary']),
                          historical_integrity_proofs_modified=False, policy=POLICY,
                          completed_at=time.time())
            write_new(receipt / 'COMPLETE.json', result)
            return result
        except BaseException as error:
            write_new(receipt / 'INCOMPLETE.json', dict(complete=False, removed_files=completed,
                error=str(error), error_type=type(error).__name__, plan_sha256=plan_sha256,
                note='Partial deletion journal is authoritative; do not replay old PNG/native verification.'))
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('plan')
    p.add_argument('--selected', required=True, type=Path)
    p.add_argument('--png-package', required=True, type=Path)
    p.add_argument('--jpeg-cache', required=True, type=Path)
    p.add_argument('--jpeg-package', required=True, type=Path)
    p.add_argument('--raw-allow-root', required=True, action='append', type=Path)
    p.add_argument('--expected-scenes', type=int, default=5000)
    p.add_argument('--delete-png-package-rgb', action='store_true')
    p.add_argument('--write-plan', type=Path)
    p = sub.add_parser('delete')
    p.add_argument('--plan', required=True, type=Path)
    p.add_argument('--plan-sha256', required=True)
    p.add_argument('--receipt-dir', required=True, type=Path)
    p.add_argument('--ack-lossy-source-deletion', action='store_true')
    args = parser.parse_args(argv)
    if args.command == 'plan':
        result = inspect(args.selected, args.png_package, args.jpeg_cache, args.jpeg_package,
                         args.raw_allow_root, args.expected_scenes, args.delete_png_package_rgb, True)
        if args.write_plan:
            write_new(args.write_plan, result)
        print(json.dumps(dict(event='rgb_prune_dry_run', files_deleted=0,
            scenes=len(result['scenes']), images=sum(len(r['cameras']) for r in result['scenes'].values()),
            planned_file_names=len(result['candidates']), logical_bytes=result['logical_bytes'],
            reclaimable_inode_bytes_estimate=result['reclaimable_inode_bytes_estimate'],
            externally_retained_hardlink_inodes=result['externally_retained_hardlink_inodes'],
            plan=str(args.write_plan) if args.write_plan else None,
            plan_sha256=digest(args.write_plan) if args.write_plan else None, policy=POLICY), indent=2))
    else:
        print(json.dumps(delete(args.plan, args.plan_sha256, args.receipt_dir,
                                args.ack_lossy_source_deletion), indent=2))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, pickle.UnpicklingError) as error:
        print(json.dumps(dict(event='rgb_prune_blocked', error=str(error))), file=sys.stderr)
        raise SystemExit(2)
