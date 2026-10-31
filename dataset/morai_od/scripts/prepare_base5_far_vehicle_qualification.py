#!/usr/bin/env python3
"""Register a hash-fixed 32-scene native qualification from a full candidate.

No generation, scene edits, ROS, simulator, training, or bulk capture occurs.
The selected entries still reference the exact candidate scenario/metadata and
retain its config, seed, primary, split and secondary target contracts.
"""
import argparse
from collections import Counter, defaultdict
import copy
import hashlib
from itertools import combinations, product
import json
from pathlib import Path

from base5_capture_safety import atomic_json, sha256
from base5_front_release_inputs import ENTRYPOINTS, collect_runtime


POLICY = 'secondary-far-vehicle-native-qualification-v153'
BANDS = ((60, 80), (80, 90), (90, 95), (95, 99))
TAIL_WINDOWS = ((95, 97), (97, 99))
ROLES = ('front', 'rear')
LANES = ('same', 'adjacent')
QUALIFICATION_COUNT = len(BANDS) * len(ROLES) * len(LANES) * 2
SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parents[2]
PASSENGER_NAMES = frozenset((
    'DEFAULT_SEDAN', 'DEFAULT_WAGON', '2014_Kia_K7', '2015_Kia_K5',
    '2015_Toyota_Prius(HEV)', '2016_Hyundai_Genesis_DH', '2016_Hyundai_Ioniq',
    '2016_Hyundai_Sonata', '2017_Hyundai_Grandeur', '2019_Toyota_Prius',
    '2019_hyundai_genesis_g90', '2020_Kia_K5', '2020_Kia_Stinger',
    '2020_hyundai_sonata', '2021_Hyundai_G80', '2021_Volkswagen_Golf_GTI'))
SUV_NAMES = frozenset((
    'DEFAULT_SUV', '2016_Hyundai_Santafe', '2016_Kia_Soul(EV)',
    '2017_Kia_Niro(HEV)', '2017_Kia_Sorento', '2019_Hyundai_Kona(EV)',
    '2019_Hyundai_Nexo', '2020_Kia_Niro(EV)', '2020_Kia_Soul(EV)',
    '2020_volvo_xc90', '2021_Hyundai_GV70', '2021_Hyundai_Genesis_GV80',
    '2023_Hyundai_Ioniq5', '2023_Kia_EV6'))


def read(path):
    return json.loads(Path(path).read_text())


def order_key(ident):
    return hashlib.sha256((POLICY + ':' + ident).encode()).hexdigest(), ident


def body_group(name):
    # Explicit audited ordinary-NPC names; unknown/light vans are not silently
    # relabeled SUV. This is selection bookkeeping, not a training class.
    return 'passenger' if name in PASSENGER_NAMES else 'suv_crossover' if name in SUV_NAMES else 'other'


def candidate_row(entry, meta):
    plan = meta.get('spec', {}).get('secondary_far_vehicle')
    if plan is None:
        return None
    bounds = tuple(plan['distance_bounds_m'])
    if bounds not in BANDS or plan['role'] not in ROLES or plan['lane_relation'] not in LANES:
        raise ValueError('Unsupported frozen far stratum: ' + entry['id'])
    ordinal = plan['target_ordinal']
    if type(ordinal) is not int or ordinal < 1 or ordinal >= len(meta['runtime_actors']):
        raise ValueError('Far target must be an existing secondary ordinal')
    actor = meta['runtime_actors'][ordinal]
    sampling = tuple(plan.get('sampling_bounds_m', ()))
    if (sampling not in (TAIL_WINDOWS if bounds == (95, 99) else (bounds,))
            or not sampling[0] <= actor['planned_center_distance_m'] < sampling[1]):
        raise ValueError('Frozen far sampling window mismatch: ' + entry['id'])
    if (actor['unique_id'] != plan['target_uid'] or actor['data_id'] != plan['target_data_id']
            or actor['name'] != plan['target_name'] or actor['source_list'] != 'vehicleList'
            or actor['motion'] != 'static' or actor['actual_ego_role'] != plan['role']
            or not bounds[0] <= actor['planned_center_distance_m'] < bounds[1]
            or meta['spec']['family'] != 'highway' or meta['spec']['mission']
            or not meta['spec'].get('vehicle_only_population')
            or any(a['source_list'] != 'vehicleList' or a['motion'] != 'static'
                   for a in meta['runtime_actors'])):
        raise ValueError('Far target/vehicle-only/static/range contract differs: ' + entry['id'])
    return dict(id=entry['id'], stratum=(bounds, plan['role'], plan['lane_relation']),
        asset=actor['name'], data_id=actor['data_id'], body_group=body_group(actor['name']),
        target_uid=actor['unique_id'], target_ordinal=ordinal, plan=copy.deepcopy(plan),
        sampling_bounds_m=list(sampling),
        planned_center_distance_ego_xy_m=actor['planned_center_distance_m'],
        seed=meta['spec']['seed'], split=meta['split'])


def select(candidates):
    """Lexicographic hash selection subject to all predeclared coverage gates."""
    index = {row['id']: row for row in candidates}
    if len(index) != len(candidates):
        raise ValueError('Duplicate far candidate ID')
    groups = defaultdict(list)
    for row in candidates:
        groups[row['stratum']].append(row)
    selected = []
    for band in BANDS:
        for role in ROLES:
            pools = []
            for lane in LANES:
                values = sorted(groups[(band, role, lane)], key=lambda r: order_key(r['id']))
                if len(values) < 2:
                    raise ValueError(f'Far stratum shortage: {band}/{role}/{lane}: {len(values)} < 2')
                pools.append(list(combinations(values, 2)))
            best = None
            for same, adjacent in product(*pools):
                combination = same + adjacent
                assets = {r['asset'] for r in combination}
                bodies = {r['body_group'] for r in combination}
                if len(assets) < 3 or not {'passenger', 'suv_crossover'} <= bodies:
                    continue
                upper_lanes = set()
                if band == (95, 99):
                    windows = {tuple(r['sampling_bounds_m']) for r in combination}
                    if windows != set(TAIL_WINDOWS):
                        continue
                    upper = [r for r in combination if tuple(r['sampling_bounds_m']) == (97, 99)]
                    if role == 'front' and not any(r['body_group'] == 'passenger' for r in upper):
                        continue
                    upper_lanes = {r['stratum'][2] for r in upper}
                # Tail upper-window same+adjacent coverage is a pre-outcome
                # preference, never a native-success-based reselection.
                key = (-len(upper_lanes), tuple(sorted(order_key(r['id']) for r in combination)))
                if best is None or key < best[0]:
                    best = (key, combination)
            if best is None:
                raise ValueError(f'Far asset coverage unavailable: {band}/{role}; '
                                 'need >=3 assets including ordinary passenger and SUV/crossover, same2+adjacent2; '
                                 'tail needs both sampling windows and a >=97m front passenger')
            selected.extend(sorted(best[1], key=lambda r: (LANES.index(r['stratum'][2]), order_key(r['id']))))
    if (len(selected) != QUALIFICATION_COUNT
            or len({r['id'] for r in selected}) != QUALIFICATION_COUNT):
        raise ValueError(f'Expected exactly {QUALIFICATION_COUNT} unique training-candidate qualification entries')
    return selected


def config_references(config, base):
    references = {}
    def add(path, digest):
        path = Path(path)
        if not path.is_absolute():
            path = base / path
        if sha256(path) != digest:
            raise ValueError('Frozen config dependency changed: ' + str(path))
        references[str(path.resolve())] = digest
    def visit(value):
        if isinstance(value, dict):
            if isinstance(value.get('path'), str) and isinstance(value.get('sha256'), str):
                add(value['path'], value['sha256'])
            for key, item in value.items():
                if isinstance(item, str) and isinstance(value.get(key + '_sha256'), str):
                    add(item, value[key + '_sha256'])
                if key.endswith('_path') and isinstance(item, str):
                    digest = value.get(key[:-5] + '_sha256')
                    if isinstance(digest, str):
                        add(item, digest)
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)
    visit(config)
    return references


def generation_dependency_pins():
    """Pin the data read by gen.context, without constructing context or ROS."""
    import generate_base5_independent as gen
    inventory = read(gen.catalog.DEFAULT_CATALOG)
    paths = {gen.catalog.DEFAULT_CATALOG, gen.GEOMETRY, gen.APPEARANCE_POLICY,
             gen.HIGHWAY_SAMPLE, gen.geo.DEFAULT_TEMPLATE, gen.geo.DEFAULT_ROUTE}
    if gen.LIVE_GEOMETRY.exists():
        paths.add(gen.LIVE_GEOMETRY)
    paths.update(gen.geo.DEFAULT_RUNTIME_ASSET_DIR / name for name in inventory['source_sha256'])
    paths.update(gen.geo.PROJECT_ROOT / row['geometric_gt_reference'] for row in inventory['assets']
                 if row.get('geometric_gt_reference'))
    paths.update(gen.geo.DEFAULT_MGEO.glob('*.json'))
    mission_root = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions'
    for mission in ('static_obstacle', 'dynamic_obstacle'):
        paths.update((mission_root / mission / 'base/scenarios').glob('*.json'))
    return {str(path.resolve()): sha256(path) for path in sorted(paths)}


def source_audit_pins(manifest):
    """Fail closed on an incomplete/failed full-candidate offline audit."""
    manifest = Path(manifest).resolve()
    preservation_path = manifest.parent / 'preservation_audit.json'
    scenario_path = manifest.parent / 'scenario_audit.json'
    preservation, scenario = read(preservation_path), read(scenario_path)
    if (preservation.get('passed') is not True or preservation.get('errors')
            or preservation.get('frames') != 5000
            or preservation.get('candidate_manifest_sha256') != sha256(manifest)):
        raise ValueError('Full candidate preservation audit is missing, failed, or stale')
    if (scenario.get('status') != 'OFFLINE_PASS' or scenario.get('errors')
            or scenario.get('scenes') != 5000 or scenario.get('unique_payloads') != 5000):
        raise ValueError('Full candidate scenario audit is missing, failed, or incomplete')
    return {str(path): sha256(path) for path in (preservation_path, scenario_path)}


def prepare(manifest, output):
    manifest, output = Path(manifest).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('A new far qualification registration directory is required')
    if output == manifest.parent or manifest.parent in output.parents:
        raise ValueError('Qualification output must be separate from source candidate')
    source_hash = sha256(manifest)
    deck = read(manifest)
    if deck.get('count') != 5000 or len(deck.get('scenes', [])) != 5000:
        raise ValueError('Complete 5K training candidate is required')
    if not deck['config'].get('secondary_far_vehicle_policy'):
        raise ValueError('Frozen secondary far policy is required')
    audit_pins = source_audit_pins(manifest)
    import base5_secondary_far_vehicle as far
    frozen_path = Path(deck['config']['frozen_specs_path'])
    if not frozen_path.is_absolute():
        frozen_path = manifest.parent / frozen_path
    if sha256(frozen_path) != deck['config']['frozen_specs_sha256']:
        raise ValueError('Frozen full candidate specs changed')
    frozen = read(frozen_path)
    if len(frozen) != 5000:
        raise ValueError('Frozen spec count differs')
    entries, metadata, candidates, full_digest = {}, {}, [], hashlib.sha256()
    for source_index, (entry, spec) in enumerate(zip(deck['scenes'], frozen)):
        ident = entry['id']
        if ident in entries:
            raise ValueError('Duplicate source candidate ID')
        scenario, sidecar = Path(entry['scenario']), Path(entry['metadata'])
        if not scenario.is_absolute() or not sidecar.is_absolute():
            raise ValueError('Absolute source paths required to preserve subset entries without rewriting them')
        if sha256(scenario) != entry['scenario_sha256']:
            raise ValueError('Source scenario checksum differs: ' + ident)
        meta_hash = sha256(sidecar)
        meta = read(sidecar)
        if meta['spec'] != spec or meta['split'] != entry['split']:
            raise ValueError('Frozen source spec/split differs: ' + ident)
        plan = far.validate_plan(meta['spec'], deck['config'])
        if plan:
            far.verify_preserved(meta, meta)
            if meta['runtime_actors'][plan['target_ordinal']]['secondary_far_vehicle']['plan_sha256'] != far.digest(plan):
                raise ValueError('Frozen target plan SHA differs')
            row = candidate_row(entry, meta)
            row.update(source_index=source_index, scenario_sha256=entry['scenario_sha256'],
                       metadata_sha256=meta_hash)
            candidates.append(row)
        full_digest.update(ident.encode() + b'\0' + entry['scenario_sha256'].encode() + meta_hash.encode())
        entries[ident], metadata[ident] = entry, meta
    selected = select(candidates)
    closure = collect_runtime(entrypoints=ENTRYPOINTS + (
        Path(__file__).name, 'prepare_base5_secondary_far_vehicle_candidate.py'))
    dependencies = config_references(deck['config'], manifest.parent)
    dependencies.update(audit_pins)
    dependencies.update(generation_dependency_pins())
    for selected_row in selected:
        dependencies.update(config_references(selected_row['plan'], manifest.parent))
    from generate_dynamic_object_capture_set import DEFAULT_MGEO
    for path in (ROOT / 'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
                 ROOT / 'configs/sensors/Sensor_competition_local.json',
                 DEFAULT_MGEO / 'link_set.json', DEFAULT_MGEO / 'lane_boundary_set.json'):
        dependencies[str(path.resolve())] = sha256(path)
    # Recheck selected files and every frozen reference immediately before write.
    if sha256(manifest) != source_hash or sha256(frozen_path) != deck['config']['frozen_specs_sha256']:
        raise ValueError('Candidate changed during qualification registration')
    for row in selected:
        entry = entries[row['id']]
        if sha256(entry['scenario']) != row['scenario_sha256'] or sha256(entry['metadata']) != row['metadata_sha256']:
            raise ValueError('Selected payload changed during registration')
    if any(sha256(path) != digest for path, digest in {**closure, **dependencies}.items()):
        raise ValueError('Source dependency changed during qualification registration')
    subset = copy.deepcopy(deck)
    subset.update(scenes=[copy.deepcopy(entries[r['id']]) for r in selected], count=QUALIFICATION_COUNT,
        status='TRAINING_CANDIDATE_QUALIFICATION_PENDING_NATIVE; no bulk or model improvement claim')
    registration = dict(policy=POLICY, source_candidate=str(manifest), source_candidate_sha256=source_hash,
        source_count=5000, full_source_payload_metadata_digest=full_digest.hexdigest(),
        source_far_candidates=len(candidates), frozen_specs_sha256=deck['config']['frozen_specs_sha256'],
        full_source_offline_audit_sha256=audit_pins,
        helper_sha256=sha256(__file__), core_local_import_closure_sha256=closure,
        native_qa_sha256={name: closure[str((SCRIPTS / name).resolve())] for name in
            ('base5_far_vehicle_native.py', 'base5_capture_audit.py', 'base5_capture_safety.py',
             'base5_ground_support.py', 'base5_static_ground_policy.py', 'base5_capture_timing.py')},
        frozen_source_dependency_sha256=dependencies,
        selection='Pre-outcome ID SHA256; per band/role same2+adjacent2; >=3 assets incl passenger+SUV; tail both windows and >=97m front passenger; prefer upper same+adjacent before hash',
        selection_contract_version=2,
        tail_window_contract=dict(distance_bounds_m=[95, 99],
            declared_sampling_windows=[list(w) for w in TAIL_WINDOWS], minimum_each_window_per_role=1,
            minimum_upper_front_passenger=1,
            upper_lane_coverage='Prefer both same and adjacent when a hard-gate-compliant combination exists',
            outcome_filtering=False, basis='Frozen planned Ego center distance; report native Ego and LiDAR distances separately'),
        selected=[dict(r, stratum=dict(distance_bounds_m=list(r['stratum'][0]), role=r['stratum'][1],
                                       lane_relation=r['stratum'][2])) for r in selected],
        stratum_counts={f'{a}_{b}/{role}/{lane}': sum(r['stratum']==((a,b),role,lane) for r in selected)
                        for a,b in BANDS for role in ROLES for lane in LANES},
        source_sampling_window_counts={f'{a}_{b}/{role}': sum(r['sampling_bounds_m']==[a,b]
            and r['stratum'][1]==role for r in candidates) for a,b in TAIL_WINDOWS for role in ROLES},
        selected_sampling_window_counts={f'{a}_{b}/{role}': sum(r['sampling_bounds_m']==[a,b]
            and r['stratum'][1]==role for r in selected) for a,b in TAIL_WINDOWS for role in ROLES},
        native_comparison_contract=dict(
            allocated_target='Use frozen target_uid/ordinal, not whichever visible primary happens to pass.',
            support='Native LiDAR>=1 OR actual 512x1056 instance-mask>=16 pixels and short side>=4px; camera-only kept.',
            geometry='Compare calibrated native Ego-frame range and role with frozen target; report planned sampling window and actual native window separately; LiDAR-frame labels reported separately.',
            placement='Keep frozen lane proof/source identities; Park, CollisionData, stationary/ground QA unchanged.',
            preservation='Check primary and all non-target actors remain exactly frozen; no threshold relaxation.',
            metrics='Per-band/direction/asset target support, raw missing/excluded, ground measured/unmeasured, runtime Z/position, bounded recovery cost.',
            limits='Native support qualification only; no AP, temporal bootstrap, final 5K completion or all-weather certification.'),
        live_qualified=False, bulk_started=False, original_config_preserved=True)
    atomic_json(output / 'manifest.json', subset)
    registration['subset_manifest_sha256'] = sha256(output / 'manifest.json')
    atomic_json(output / 'registration.json', registration)
    return output / 'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
