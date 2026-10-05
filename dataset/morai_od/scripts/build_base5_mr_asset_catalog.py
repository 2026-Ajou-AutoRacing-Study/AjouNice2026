#!/usr/bin/env python3
"""Versioned Base5 asset policy; preserve historical catalogs and actor containers."""
import argparse
import copy
import json
from pathlib import Path

import extract_morai_asset_catalog as extract

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CATALOG = ROOT / 'dataset/morai_od/configs/morai_asset_catalog_mr_base5_261005.json'
DEFAULT_RUNTIME = Path.home() / 'MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025'


def build(runtime, option_list, historical):
    names = extract.load_names(option_list)
    old = {int(r['data_id']): r for key in ('vehicles', 'pedestrians_and_related',
                                           'obstacles_and_related') for r in historical[key]}
    groups = [('new_vehicle_list.json', 'vehicleList'),
              ('new_pedestrian_list.json', 'pedestrianList'),
              ('new_pedestrian_list.json', 'aiPedstrianList'),
              ('new_obstacle_list.json', 'objectList')]
    rows, seen, sources = [], set(), {}
    for filename, container in groups:
        path = runtime / filename
        sources[filename] = extract.sha256(path)
        for actor in json.loads(path.read_text()).get(container, []):
            ident = int(actor['DataID'])
            if ident in seen:
                continue
            seen.add(ident)
            name = names.get(ident)
            if name is None:
                raise ValueError('Unresolved DataID: {}'.format(ident))
            row = copy.deepcopy(old.get(ident, {}))
            row.update(data_id=ident, name=name, source_file=filename,
                       source_list=container, semantic_class=None, collection_role='review',
                       generation_policy='review_only', reason='GT/mesh review required')
            lower = name.lower()
            if container == 'vehicleList' or name.startswith('OBJ_'):
                row.update(semantic_class='vehicle', ontology_class='Vehicle',
                           collection_role='positive', generation_policy='enabled',
                           reason='Vehicle appearance independent of stationary state')
            elif row.get('ontology_class') == 'Pedestrian':
                row.update(semantic_class='pedestrian', collection_role='positive',
                           generation_policy='enabled', reason='Existing person/dummy policy')
            elif container != 'objectList':
                row.update(reason='Animal/rider/composite: no unlabeled hard-negative training')
            elif lower.startswith('tree') or lower.startswith('effect_') or name in ('Speed_Bump', 'hole-cover'):
                row.update(collection_role='excluded', generation_policy='excluded_dataset_scope',
                           reason='Outside current positive reinforcement scope')
            elif 'sign' in lower or 'streetlight' in lower:
                row.update(collection_role='roadside_context', generation_policy='context_only',
                           reason='Roadside only; never random lane-center positive')
            elif ident in old and old[ident].get('ontology_class') == 'Obstacle':
                row.update(semantic_class='obstacle', collection_role='positive',
                           generation_policy='enabled', reason='Existing physical obstacle')
            else:
                row.update(reason='New stroller/two-wheeler: occupant and annotation review first')
            if name == 'OBJ_G80_Open_Door':
                row.update(collection_role='review', generation_policy='review_only',
                           reason='261005 live: floating mesh/GT; requested z -0.55m did not change observed height. Retain Vehicle semantics, block training until ground support resolved.')
            rows.append(row)
    return dict(schema_version=1, policy_version='mr_base5_261005',
                source_sha256=sources, option_list_sha256=extract.sha256(option_list),
                policy=dict(runtime='MR only', deduplicate_by_data_id=True,
                            scale=1, unknown_motion_policy='stationary; do not map animals blindly',
                            production_catalog_changed=False), assets=sorted(rows, key=lambda r: r['data_id']))


def load_pools(catalog_path=DEFAULT_CATALOG, runtime=DEFAULT_RUNTIME):
    catalog = json.loads(Path(catalog_path).read_text())
    templates = {}
    for filename in catalog['source_sha256']:
        doc = json.loads((Path(runtime) / filename).read_text())
        for container in ('vehicleList', 'pedestrianList', 'aiPedstrianList', 'objectList'):
            for actor in doc.get(container, []):
                templates.setdefault((filename, container, int(actor['DataID'])), actor)
    pools = {key: [] for key in ('near_obstacle', 'far_vehicle', 'pedestrian')}
    family = {'obstacle': 'near_obstacle', 'vehicle': 'far_vehicle', 'pedestrian': 'pedestrian'}
    for row in catalog['assets']:
        if row['collection_role'] != 'positive':
            continue
        key = (row['source_file'], row['source_list'], row['data_id'])
        if key not in templates:
            raise ValueError('Missing runtime template: {}'.format(key))
        pools[family[row['semantic_class']]].append(dict(row, template=copy.deepcopy(templates[key])))
    return pools


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-dir', type=Path, default=DEFAULT_RUNTIME)
    parser.add_argument('--option-list', type=Path, default=Path.home() / 'MoraiLauncher_Lin/MoraiLauncher_Lin_Data/Version/optionList.byte')
    parser.add_argument('--output', type=Path, default=DEFAULT_CATALOG)
    args = parser.parse_args()
    old = json.loads((ROOT / 'dataset/morai_od/configs/morai_asset_catalog_260807.json').read_text())
    catalog = build(args.runtime_dir, args.option_list, old)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
    print('Inventory:', len(catalog['assets']))
    print('Positive pools:', {k: len(v) for k, v in load_pools(args.output, args.runtime_dir).items()})


if __name__ == '__main__':
    main()
