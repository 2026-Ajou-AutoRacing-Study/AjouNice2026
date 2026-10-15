#!/usr/bin/env python3
"""Derive three-class Base5 infos; preserve immutable raw animal annotations.

Only trusted local pickle inputs are supported. No simulator or training is
started. Hardlinked payload must never be edited in place.
"""
import argparse
from collections import Counter
from copy import deepcopy
import json
import os
from pathlib import Path
import pickle
import shutil

from prepare_base5_portable_dataset import safe_relative, sha256, validate

SOURCE_CLASSES = ('vehicle', 'pedestrian', 'obstacle', 'animal')
TARGET_CLASSES = SOURCE_CLASSES[:3]
LABEL_POLICY = 'animal_to_pedestrian_v1'


def merge_infos(source):
    if tuple(source['metainfo']['classes']) != SOURCE_CLASSES:
        raise ValueError('Require original four-class infos in the audited order')
    if source['metainfo'].get('training_label_policy'):
        raise ValueError('Already transformed labels; use the original source')
    result = deepcopy(source)
    counts = Counter()
    assets = Counter()
    for row in result['data_list']:
        for instance in row['instances']:
            label = instance['bbox_label_3d']
            if isinstance(label, bool) or label not in range(4):
                raise ValueError('Invalid source class label')
            if 'original_bbox_label_3d' in instance or 'original_semantic_class' in instance:
                raise ValueError('Original semantic metadata already exists')
            instance['original_bbox_label_3d'] = label
            instance['original_semantic_class'] = SOURCE_CLASSES[label]
            instance['bbox_label_3d'] = 1 if label == 3 else label
            counts[SOURCE_CLASSES[label]] += 1
            if label == 3:
                assets[instance['asset']] += 1
    result['metainfo'].update(classes=TARGET_CLASSES, source_classes=SOURCE_CLASSES,
                              training_label_policy=LABEL_POLICY)
    return result, dict(original_gt=dict(counts), animal_assets=dict(assets),
                        merged_gt={name: sum(counts[old] for old in SOURCE_CLASSES
                            if ('pedestrian' if old == 'animal' else old) == name)
                            for name in TARGET_CLASSES}, frames=len(result['data_list']))


def build(data, output, hardlink=False):
    data = Path(data).resolve(strict=True)
    output = Path(output).absolute()
    if output.exists():
        raise FileExistsError('Use a fresh output; never overwrite labels or raw data')
    source = validate(data, verify_hashes=True)
    if source.get('training_label_policy'):
        raise ValueError('Source already has a training label policy')
    source_hash = sha256(data / 'manifest.json')
    rewritten = {}
    stats = {}
    for profile in source['profiles']:
        for split in ('train', 'val', 'test'):
            relative = f'infos/{profile}/morai_base5_infos_{split}.pkl'
            info, stats[profile + ':' + split] = merge_infos(
                pickle.loads((data / relative).read_bytes()))
            rewritten[relative] = pickle.dumps(info, protocol=4)
    report = dict(training_label_policy=LABEL_POLICY, source_manifest_sha256=source_hash,
                  source_classes=SOURCE_CLASSES, classes=TARGET_CLASSES, stats=stats,
                  unchanged='sensors, raw audits, frames, splits, boxes, component IDs, visibility, asset names',
                  evaluation='Use original_semantic_class/asset for human and animal subgroup diagnostics; '
                             'three-class mean AP is not directly comparable to four-class mean AP')
    provenance = {
        'provenance/pre_pedestrian_merge_manifest.json': (data / 'manifest.json').read_bytes(),
        'provenance/pedestrian_merge.json': (json.dumps(report, indent=2) + '\n').encode(),
    }
    if set(provenance) & set(source['files']):
        raise ValueError('Merge provenance already exists')
    output.mkdir(parents=True)
    files = deepcopy(source['files'])
    for relative in source['files']:
        target = output / safe_relative(relative)
        target.parent.mkdir(parents=True, exist_ok=True)
        if relative in rewritten:
            target.write_bytes(rewritten[relative])
            files[relative] = dict(bytes=target.stat().st_size, sha256=sha256(target))
        elif hardlink:
            os.link(data / relative, target)
        else:
            shutil.copy2(data / relative, target)
    for relative, payload in provenance.items():
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
        files[relative] = dict(bytes=len(payload), sha256=sha256(target))
    manifest = deepcopy(source)
    manifest.update(files=files, bytes=sum(v['bytes'] for v in files.values()),
                    classes=TARGET_CLASSES, training_label_policy=LABEL_POLICY,
                    source_pre_pedestrian_merge_manifest_sha256=source_hash,
                    copy_mode='hardlink' if hardlink else 'copy')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    validate(output, verify_hashes=True)
    if sha256(data / 'manifest.json') != source_hash:
        raise ValueError('Source manifest changed during conversion')
    print(json.dumps(report, indent=2), flush=True)
    return manifest, report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hardlink', action='store_true')
    args = parser.parse_args()
    build(args.data, args.output, args.hardlink)
