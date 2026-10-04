#!/usr/bin/env python3
"""Build a small +/-30m Base3 replay + Base5 addon deck using hard links.

Far-range labels belong to a separate ROI experiment, not this baseline pilot.
Original source metadata is retained in the manifest; no source files are edited.
"""
import argparse
import copy
import json
import os
import pickle
import random
from pathlib import Path

from merge_morai_bevfusion_infos import rewrite_paths


def prepare(base_info, base_root, addon_info, output, replay_train=600, replay_val=100):
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    manifest = dict(schema_version=1, purpose='short learnability pilot, not production replacement',
                    range=[-30., -30., -3., 30., 30., 2.], sources=[], splits={})
    sources = [(base_info, base_root, '/datasets/base3', 'base3')]
    for index, info in enumerate(addon_info if isinstance(addon_info, list) else [addon_info]):
        sources.append((info, info, None, 'addon'+str(index)))
    combined = {split: [] for split in ('train', 'val', 'test')}
    for info, root, old_root, tag in sources:
        source_record = dict(info_root=str(info), data_root=str(root), metainfo={}, selected={})
        for split in combined:
            with (info/('morai_base3_infos_'+split+'.pkl')).open('rb') as stream:
                payload = pickle.load(stream)
            if tuple(payload['metainfo']['classes']) != ('vehicle', 'pedestrian', 'obstacle'):
                raise ValueError('class order mismatch')
            source_record['metainfo'][split] = payload['metainfo']
            rows = copy.deepcopy(payload['data_list']) if split != 'test' else []
            if tag == 'base3':
                random.Random(20261004).shuffle(rows)
                rows = rows[:replay_train if split == 'train' else replay_val]
            else:
                # Canonical converter root is recorded explicitly, never guessed.
                audit = json.loads((info/'conversion_audit.json').read_text())
                old_root = audit['serialized_root']
                for row in rows:
                    row['instances'] = [obj for obj in row['instances']
                                        if -30. <= obj['bbox_3d'][0] < 30.
                                        and -30. <= obj['bbox_3d'][1] < 30.]
                rows = [row for row in rows if row['instances']]
            for row in rows:
                if any(len(obj['bbox_3d']) != 7 for obj in row['instances']):
                    raise ValueError('pilot requires 7D spatial boxes, no velocity targets')
                # Copy only files referenced by selected samples; hardlinks do not
                # allocate another raw dataset. Container mounts them read-only.
                paths = [row['lidar_points']['lidar_path']]
                paths += [image['img_path'] for image in row['images'].values()]
                for value in paths:
                    relative = Path(value).relative_to(old_root)
                    src, dst = root/relative, output/tag/relative
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    if not dst.exists():
                        os.link(src, dst)
                row = rewrite_paths(row, old_root, '/datasets/base5_replay/'+tag)
                row['source_dataset'] = tag
                combined[split].append(row)
            source_record['selected'][split] = len(rows)
        manifest['sources'].append(source_record)
    seen = set()
    for split, rows in combined.items():
        for row in rows:
            identifier = str(row['sample_idx'])
            if identifier in seen:
                raise ValueError('duplicate/leaked sample: '+identifier)
            seen.add(identifier)
        metadata = dict(dataset='base5_replay_pilot', classes=('vehicle', 'pedestrian', 'obstacle'),
                        box_dim=7, coordinate_frame='lidar', source_contracts_in_manifest=True)
        with (output/('morai_base3_infos_'+split+'.pkl')).open('wb') as stream:
            pickle.dump(dict(metainfo=metadata, data_list=rows), stream, protocol=4)
        manifest['splits'][split] = len(rows)
    (output/'replay_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest['splits']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('base-info', 'base-root', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--addon-info', type=Path, nargs='+', required=True)
    args = parser.parse_args()
    prepare(args.base_info, args.base_root, args.addon_info, args.output)
