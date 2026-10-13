#!/usr/bin/env python3
"""Preserve a copied package; build a fresh verified local-disk mirror.

Only the four explicitly supplied 512-profile metadata files may be repaired.
Every linked/restored payload must match the existing manifest's content hash.
Useful when host-side instance copying leaves a live overlay stale directory.
No source file, manifest, class, split, label or sensor is rewritten.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import time

from prepare_base5_portable_dataset import safe_relative, sha256, validate

REPAIR_FILES = {'infos/512x1056/' + name for name in (
    'calibration.json', 'morai_base5_infos_train.pkl',
    'morai_base5_infos_val.pkl', 'morai_base5_infos_test.pkl')}


def mirror(source, repair, output, copy_mode='hardlink'):
    source, repair, output = Path(source), Path(repair), Path(output)
    if copy_mode not in ('hardlink', 'copy'):
        raise ValueError('Unknown copy mode')
    if output.exists():
        raise FileExistsError('Fresh mirror required; partial attempts are retained')
    manifest_path = source / 'manifest.json'
    before = sha256(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('box_geometry_policy') != 'native_rpy_fixed_yaw_enclosure_v1':
        raise ValueError('Current Base5 label policy required')
    repaired, linked = [], 0
    output.mkdir(parents=True)
    started = time.monotonic()
    for relative, expected in manifest['files'].items():
        relative = str(safe_relative(relative))
        src = repair / relative if relative in REPAIR_FILES else source / relative
        if src.stat().st_size != expected['bytes'] or sha256(src) != expected['sha256']:
            raise ValueError('Replacement/source payload does not match manifest: ' + relative)
        dst = output / relative
        dst.parent.mkdir(parents=True, exist_ok=True)
        if copy_mode == 'hardlink':
            os.link(src, dst)
        else:
            shutil.copyfile(src, dst)
            if sha256(dst) != expected['sha256']:
                raise ValueError('Copied payload hash differs: ' + relative)
        if relative in REPAIR_FILES:
            repaired.append(relative)
        linked += 1
        if linked % 25000 == 0:
            print('VERIFIED MIRROR %d/%d' % (linked, len(manifest['files'])), flush=True)
    shutil.copy2(manifest_path, output / 'manifest.json')
    if sha256(manifest_path) != before or sha256(output / 'manifest.json') != before:
        raise ValueError('Manifest changed')
    return dict(valid=True, files=linked, repaired=sorted(repaired),
                manifest_sha256=before, bytes=manifest['bytes'], seconds=time.monotonic() - started,
                source_preserved=True, sensor_retransmission=False,
                copy_mode=copy_mode,
                note='Immutable verified payload mirror; no in-place source edits')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--repair', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--copy-mode', choices=('hardlink', 'copy'), default='hardlink',
                   help='Use copy if live-overlay copy-up/hardlinks return stale handles')
    args = p.parse_args()
    if args.report.exists():
        p.error('Fresh report required')
    result = mirror(args.source, args.repair, args.output, args.copy_mode)
    manifest = validate(args.output, verify_hashes=True)
    result.update(split_counts=manifest['split_counts'], profiles=manifest['profiles'])
    args.report.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
