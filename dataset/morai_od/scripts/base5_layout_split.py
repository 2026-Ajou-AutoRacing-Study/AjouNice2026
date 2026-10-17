"""Versioned known-map, layout-group split. No raw or simulator writes.

Map/background overlap is intentional. Actual repeated layouts and explicitly
declared clip groups remain indivisible; logical quota slots are not groups.
Training coverage is a hard constraint, while the validation ratio is soft.
"""
from collections import Counter, defaultdict
import hashlib
import itertools
import json
import math
from pathlib import Path
from functools import lru_cache

POLICY = 'known-map-layout-group-v156'
SCHEMA = 'base5_layout_split_overlay_v1'
CONFIG_KEY = 'layout_split_policy'


@lru_cache(maxsize=128)
def _overlay_file_hash(path, inode, size, mtime_ns):
    from base5_capture_safety import sha256
    return sha256(path)


def overlay_file_hash(path):
    path=Path(path).resolve();stat=path.stat()
    return _overlay_file_hash(str(path),stat.st_ino,stat.st_size,stat.st_mtime_ns)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def configuration(val_fraction=.15, train_asset_floor=5):
    result = dict(policy=POLICY, val_fraction=val_fraction, train_asset_floor=train_asset_floor,
        coverage_cell_m=10., near_position_m=.05, near_yaw_deg=.25)
    validate_config(result)
    return result


def validate_config(value):
    if (not isinstance(value, dict) or value.get('policy') != POLICY
            or not isinstance(value.get('val_fraction'), (int, float))
            or not 0 < value['val_fraction'] < .5
            or type(value.get('train_asset_floor')) is not int or value['train_asset_floor'] < 1
            or any(not isinstance(value.get(k), (int, float)) or not math.isfinite(value[k])
                or value[k] <= 0 for k in ('coverage_cell_m', 'near_position_m', 'near_yaw_deg'))):
        raise ValueError('Invalid known-map layout split policy')
    return value


def enabled(config):
    value = config.get(CONFIG_KEY)
    if value is None:
        return False
    validate_config(value)
    return True


def _number(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('Nonfinite layout geometry')
    return round(result, 6)


def _position(value):
    return [_number(value[k]) for k in ('x', 'y', 'z')]


def _yaw(value):
    return round((float(value) + 180.) % 360. - 180., 6)


def geometry(meta):
    """Exclude weather, seed, slot IDs and annotation/support bookkeeping.

    Include actual initializer XYZ, yaw, physical size/scale and the full motion
    path, not just base-link/anchor XY. Actor identity uses DataID, not instance
    UID (UID renumbering does not make an independent layout).
    """
    ego = meta['ego']
    actors = []
    for actor in meta['runtime_actors']:
        scale = actor.get('asset_scale', {})
        if not isinstance(scale, dict):
            scale = dict(value=scale)
        actors.append(dict(data_id=actor['data_id'], source_list=actor['source_list'],
            position=_position(actor['position']), yaw=_yaw(actor['placement_yaw_deg']),
            size=[_number(v) for v in actor['estimated_size']], motion=actor['motion'],
            motion_path=actor.get('motion_path'), scale=scale.get('scale_xyz', scale.get('scale'))))
    actors.sort(key=lambda a: (str(a['data_id']), a['source_list'], a['position'], a['yaw'], digest(a)))
    return dict(ego=dict(position=_position(ego), yaw=_yaw(ego['yaw'])), actors=actors)


def provisional_split(meta, config):
    """Generator-only split; the complete deck still needs coverage allocation."""
    validate_config(config[CONFIG_KEY])
    plan = meta.get('spec', {}).get('static_mission_extent', {})
    if plan.get('split_policy') == POLICY:
        return plan['split']
    declared = meta.get('spec', {}).get('layout_split')
    if declared is not None:
        if declared not in ('train', 'val'):
            raise ValueError('Invalid explicit layout split')
        return declared
    return 'val' if int(digest(geometry(meta))[:12], 16) / 16**12 < config[CONFIG_KEY]['val_fraction'] else 'train'


def _yaw_distance(a, b):
    return abs((a-b+180.) % 360. - 180.)


def nearby(a, b, config):
    """Conservative near-duplicate layout check, not a geographic holdout."""
    if len(a['actors']) != len(b['actors']):
        return False
    for left, right in [(a['ego'], b['ego'])] + list(zip(a['actors'], b['actors'])):
        if any(left.get(k) != right.get(k) for k in ('data_id', 'source_list', 'size', 'motion', 'motion_path', 'scale')):
            return False
        if (math.dist(left['position'], right['position']) > config['near_position_m']
                or _yaw_distance(left['yaw'], right['yaw']) > config['near_yaw_deg']):
            return False
    return True


def groups(records, config):
    """Union exact/near geometry and real clip groups, never split_group links."""
    validate_config(config)
    ids = [r['id'] for r in records]
    if len(ids) != len(set(ids)):
        raise ValueError('Duplicate scene ID; use unique environment scene IDs')
    parent = list(range(len(records)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        a, b = find(a), find(b)
        parent[max(a, b)] = min(a, b)

    descriptors = [geometry(r['meta']) for r in records]
    hashes = [digest(g) for g in descriptors]
    exact, clips, grid = {}, {}, defaultdict(list)
    near_pairs = []
    for i, (record, descriptor, fingerprint) in enumerate(zip(records, descriptors, hashes)):
        if fingerprint in exact:
            union(i, exact[fingerprint])
        exact[fingerprint] = i
        # This is a layout/clip identity, not logical_quota_slot_id, source ID,
        # or legacy whole-link split_group. Fresh environments have unique IDs.
        clip = record['meta'].get('scene_group_id')
        if clip:
            if clip in clips:
                union(i, clips[clip])
            clips[clip] = i
        xyz = descriptor['ego']['position']
        cell = tuple(math.floor(v/config['near_position_m']) for v in xyz)
        for delta in itertools.product((-1, 0, 1), repeat=3):
            for j in grid[tuple(c+d for c, d in zip(cell, delta))]:
                if hashes[j] != fingerprint and nearby(descriptor, descriptors[j], config):
                    union(i, j)
                    near_pairs.append([records[j]['id'], record['id']])
        grid[cell].append(i)
    members = defaultdict(list)
    for i, record in enumerate(records):
        members[find(i)].append(record['id'])
    group_ids = {i: 'layout:'+digest(sorted(value))[:24] for i, value in members.items()}
    return ({record['id']: group_ids[find(i)] for i, record in enumerate(records)},
        dict(zip(ids, hashes)), near_pairs)


def stratum(meta):
    spec = meta['spec']
    return (meta.get('static_mission_extent', {}).get('stratum')
        or spec.get('static_mission_extent', {}).get('stratum')
        or spec.get('avoidance_phase') or spec.get('dynamic_observation_phase')
        or spec.get('roundabout_phase') or spec.get('shadow_observation_stage') or 'generic')


def features(meta, config):
    spec, ego = meta['spec'], meta['ego']
    family = spec['family']
    cell = tuple(math.floor(ego[k]/config['coverage_cell_m']) for k in ('x', 'y'))
    link = meta.get('ego_ground_initializer', {}).get('center', {}).get('link_id') or ego.get('link_idx')
    phase = stratum(meta)
    hard = {'cell:'+str(cell), 'link:'+str(link), 'mission:'+str((family, bool(spec.get('mission')), phase))}
    for actor in meta['runtime_actors']:
        hard.add('asset:'+str(actor['data_id']))
        assignment = actor.get('asset_scale')
        if assignment:
            hard.add('profile:'+str((actor['data_id'], assignment.get('profile_id'))))
    soft = {'family:'+family, 'role:'+str((family, spec.get('role'))),
        'distance:'+str((family, spec.get('distance_bin'))),
        'mission:'+str((family, bool(spec.get('mission')), phase)),
        'yaw:'+str((family, spec.get('yaw_limit'))),
        'topology:'+str(spec.get('topology'))}
    return hard, soft


def allocate(records, config=None):
    config = configuration() if config is None else validate_config(config)
    if not records:
        raise ValueError('Empty split deck')
    records = sorted(records, key=lambda r: r['id'])
    group_of, fingerprints, near_pairs = groups(records, config)
    members, hard_by, soft_by = defaultdict(list), defaultdict(set), defaultdict(Counter)
    for record in records:
        group = group_of[record['id']]
        members[group].append(record)
        hard, soft = features(record['meta'], config)
        hard_by[group].update(hard)
        soft_by[group].update(soft)
    remaining, totals = Counter(), Counter()
    for group in members:
        remaining.update(hard_by[group])
        totals.update(soft_by[group])
    floors = {key: min(value, config['train_asset_floor']) if key.startswith(('asset:', 'profile:')) else 1
        for key, value in remaining.items()}
    selected, val_features, val_count = set(), Counter(), 0
    target = round(len(records)*config['val_fraction'])
    ordered = sorted(members, key=lambda group: digest([POLICY, group]))
    while val_count < target:
        best, best_score = None, -math.inf
        for group in ordered:
            if group in selected or val_count+len(members[group]) > target:
                continue
            if any(remaining[key]-1 < floors[key] for key in hard_by[group]):
                continue
            # Weighted reduction in squared stratification error. Rare cells
            # have useful validation representatives only after train floors.
            score = sum((2*(totals[key]*config['val_fraction']-val_features[key])*n-n*n)
                / max(1., totals[key]*config['val_fraction'])
                for key, n in soft_by[group].items()) / len(members[group])
            if score > best_score:
                best, best_score = group, score
        if best is None:
            break
        selected.add(best)
        val_count += len(members[best])
        val_features.update(soft_by[best])
        remaining.subtract(hard_by[best])
    assignments = {}
    for record in records:
        ident = record['id']
        assignments[ident] = dict(split='val' if group_of[ident] in selected else 'train',
            group_id=group_of[ident], source_split=record['meta']['split'],
            scenario_sha256=record['scenario_sha256'], geometry_sha256=fingerprints[ident])
    audit = audit_assignments(records, assignments, config)
    audit.update(target_val_frames=target, validation_shortfall=target-val_count,
        near_duplicate_pairs=near_pairs, independent_layout_groups=len(members),
        train_floors=floors, train_floor_violations=[k for k, v in floors.items() if remaining[k] < v])
    audit['status'] = 'CPU_PASS' if not audit['errors'] and not audit['train_floor_violations'] else 'CPU_FAIL'
    return dict(schema=SCHEMA, policy=config, assignments=assignments, audit=audit)


def audit_assignments(records, assignments, config):
    group_of, fingerprints, near_pairs = groups(records, config)
    errors, split_groups, exact_splits = [], defaultdict(set), defaultdict(set)
    split_counts, families, missions, assets, links, cells = Counter(), defaultdict(Counter), defaultdict(Counter), defaultdict(Counter), defaultdict(set), defaultdict(set)
    asset_groups, profiles, stratification = defaultdict(lambda: defaultdict(set)), defaultdict(lambda: defaultdict(set)), defaultdict(Counter)
    for record in records:
        ident, meta = record['id'], record['meta']
        assignment = assignments.get(ident, {})
        split = assignment.get('split')
        if (split not in ('train', 'val') or assignment.get('scenario_sha256') != record['scenario_sha256']
                or assignment.get('geometry_sha256') != fingerprints[ident]
                or assignment.get('group_id') != group_of[ident]):
            errors.append([ident, 'split/source geometry binding mismatch'])
            continue
        split_groups[group_of[ident]].add(split)
        exact_splits[fingerprints[ident]].add(split)
        split_counts[split] += 1
        families[meta['spec']['family']][split] += 1
        missions[str((meta['spec']['family'], bool(meta['spec'].get('mission')), stratum(meta)))][split] += 1
        for actor in meta['runtime_actors']:
            assets[str(actor['data_id'])][split] += 1
            asset_groups[str(actor['data_id'])][split].add(group_of[ident])
            assignment_scale = actor.get('asset_scale')
            if assignment_scale:
                profiles[str((actor['data_id'], assignment_scale['profile_id']))][split].add(group_of[ident])
        _, soft = features(meta, config)
        soft.add('actual_ego_yaw_5deg:'+str(math.floor(float(meta.get('ego_road_yaw_offset_deg', 0.))/5)))
        if meta['runtime_actors']:
            distance = meta['runtime_actors'][0].get('planned_center_distance_m')
            if distance is not None:
                soft.add('actual_primary_distance_10m:'+str(math.floor(float(distance)/10)))
        for feature in soft:
            stratification[feature][split] += 1
        hard, _ = features(meta, config)
        links[split].update(k for k in hard if k.startswith('link:'))
        cells[split].update(k for k in hard if k.startswith('cell:'))
    leaking = [group for group, splits in split_groups.items() if len(splits) > 1]
    exact_leak = sum(len(splits) > 1 for splits in exact_splits.values())
    if leaking:
        errors.append(['all', 'Repeated actual layout/clip crosses train/val'])
    if len(assignments) != len(records) or set(assignments) != {r['id'] for r in records}:
        errors.append(['all', 'Overlay frame set differs'])
    all_links, all_cells = links['train'] | links['val'], cells['train'] | cells['val']
    if all_links-links['train'] or all_cells-cells['train']:
        errors.append(['all', 'Train lost observed final Ego link/cell coverage'])
    return dict(errors=errors, frames=len(records), splits=dict(split_counts),
        family_split={k: dict(v) for k, v in sorted(families.items())},
        mission_stratum_split={k: dict(v) for k, v in sorted(missions.items())},
        asset_parent_counts={k: dict(v) for k, v in sorted(assets.items())},
        asset_independent_group_counts={k: {split: len(ids) for split, ids in sorted(v.items())}
            for k, v in sorted(asset_groups.items())},
        scale_profile_independent_group_counts={k: {split: len(ids) for split, ids in sorted(v.items())}
            for k, v in sorted(profiles.items())},
        stratification={k: dict(v) for k, v in sorted(stratification.items())},
        train_missing_assets=[k for k, v in assets.items() if not v['train']],
        final_ego_coverage=dict(cell_width_m=config['coverage_cell_m'], observed_links=len(all_links),
            train_links=len(links['train']), val_links=len(links['val']),
            observed_cells=len(all_cells), train_cells=len(cells['train']), val_cells=len(cells['val']),
            train_missing_links=sorted(all_links-links['train']), train_missing_cells=sorted(all_cells-cells['train']),
            limitation='Observed-deck final Ego coverage, not every physically allowed road or 100m visibility independence'),
        actual_geometry_cross_split_groups=leaking, exact_geometry_cross_split=exact_leak,
        near_geometry_cross_split=sum(assignments[a]['split'] != assignments[b]['split'] for a, b in near_pairs),
        planned_parents_only=True, native_retained_gt_audited=False,
        limitation='Known-map backgrounds may overlap. Retained native GT and independent rosbag accuracy remain separate gates.')


def route_coverage(records, assignments, poses, sample_spacing_m=10.):
    """Nearest final Ego distance to allowed route samples; no visibility claim."""
    import numpy as np
    query = np.asarray([[p['x'], p['y']] for p in poses], dtype=float)
    if not len(query):
        raise ValueError('No allowed route audit samples')
    output = dict(sample_spacing_m=sample_spacing_m, allowed_route_samples=len(query), splits={})
    for split in ('train', 'val'):
        xy = np.asarray([[r['meta']['ego']['x'], r['meta']['ego']['y']] for r in records
            if assignments[r['id']]['split'] == split], dtype=float)
        distances = []
        for start in range(0, len(query), 64):
            batch = query[start:start+64]
            nearest = np.sqrt(((batch[:, None, :]-xy[None, :, :])**2).sum(axis=2)).min(axis=1) if len(xy) else np.full(len(batch), np.inf)
            distances.extend(map(float, nearest))
        covered = {str(radius)+'m': sum(d <= radius for d in distances) for radius in (10, 25, 50, 100)}
        output['splits'][split] = dict(samples_with_final_ego_within=covered,
            maximum_nearest_final_ego_distance_m=max(distances) if len(xy) else None,
            samples_without_final_ego_within_25m=[dict(route_arc_m=p.get('route_arc_m'),
                link_idx=p.get('link_idx'), nearest_distance_m=d if math.isfinite(d) else None)
                for p, d in zip(poses, distances) if d > 25])
    output['limitation'] = ('Allowed MGeo/support-screened route sample proximity only, not exhaustive road-surface '
        'coverage or camera/LiDAR visibility. Existing unobserved geometry is not created by a split overlay.')
    return output


def read_overlay(path):
    value = json.loads(Path(path).read_text())
    if value.get('schema') != SCHEMA or value.get('audit', {}).get('status') != 'CPU_PASS':
        raise ValueError('Unqualified split overlay')
    validate_config(value['policy'])
    checksum = value.get('overlay_sha256')
    if checksum != digest({k: v for k, v in value.items() if k != 'overlay_sha256'}):
        raise ValueError('Split overlay checksum mismatch')
    groups_seen = defaultdict(set)
    for row in value['assignments'].values():
        if row.get('split') not in ('train', 'val'):
            raise ValueError('Invalid overlay split')
        groups_seen[row['group_id']].add(row['split'])
    if any(len(splits) > 1 for splits in groups_seen.values()):
        raise ValueError('Overlay group leakage')
    for alias, canonical in value.get('recovery_aliases', {}).items():
        if alias in value['assignments'] or canonical not in value['assignments']:
            raise ValueError('Overlay recovery alias namespace/target collision')
    return value


def verify_source_pins(overlay):
    """Once-per-plan audit, never a per-actor whole-deck hash scan."""
    from base5_capture_safety import sha256
    for path, checksum in overlay.get('source_hashes', {}).items():
        if sha256(path) != checksum:
            raise ValueError('Split overlay frozen source changed: '+path)
    for path, checksum in overlay.get('finalization_source_hashes', {}).items():
        if sha256(path)!=checksum:
            raise ValueError('Finalized split native/source lineage changed: '+path)
    for binding in overlay.get('recovery_bindings', {}).values():
        if sha256(binding['scenario']) != binding['scenario_sha256']:
            raise ValueError('Recovery binding scenario changed')
        if binding.get('metadata_content_sha256'):
            meta = json.loads(Path(binding['metadata']).read_text())
            if digest({k:v for k,v in meta.items() if k != 'training_split_overlay'}) != binding['metadata_content_sha256']:
                raise ValueError('Recovery binding non-overlay metadata changed')


def resolve(overlay, scene_id, entry, meta):
    aliases = overlay.get('recovery_aliases', {})
    candidates = [aliases.get(key, key) for key in (scene_id, entry.get('source_cell_id'))
        if aliases.get(key, key) in overlay['assignments']]
    if not candidates:
        raise ValueError('Scene missing from split overlay: '+scene_id)
    rows = [overlay['assignments'][key] for key in candidates]
    if any(row != rows[0] for row in rows[1:]):
        raise ValueError('Ambiguous split overlay scene identity')
    row = rows[0]
    if (row['scenario_sha256'] != entry['scenario_sha256']
            or row['geometry_sha256'] != digest(geometry(meta)) or row['source_split'] != meta['split']):
        raise ValueError('Split overlay source/payload/geometry differs: '+scene_id)
    return row


def overlay_for_metadata(meta, explicit=None):
    """New candidate tags survive capture; forgetting a CLI flag cannot revert split."""
    reference = meta.get('training_split_overlay')
    if explicit is None and reference is None:
        return None
    if reference is not None and overlay_file_hash(reference['path']) != reference['sha256']:
        raise ValueError('Captured split overlay reference checksum differs')
    if reference is not None and reference.get('source_metadata_path') is not None:
        from base5_capture_safety import sha256
        if sha256(reference['source_metadata_path']) != reference.get('source_metadata_sha256'):
            raise ValueError('Split overlay source metadata provenance changed')
    if isinstance(explicit, dict):
        if reference is not None and explicit['overlay_sha256'] != reference['overlay_sha256']:
            authorized = explicit.get('authorized_source_overlays', {}).get(reference['overlay_sha256'])
            from base5_capture_safety import sha256
            if (not authorized or reference['sha256'] != authorized['sha256']
                    or overlay_file_hash(reference['path']) != authorized['sha256']):
                raise ValueError('Captured split overlay content differs')
        return explicit
    path = Path(explicit) if explicit is not None else Path(reference['path'])
    if reference is not None:
        from base5_capture_safety import sha256
        if sha256(path) != reference['sha256']:
            raise ValueError('Captured split overlay reference checksum differs')
    overlay = read_overlay(path)
    if reference is not None and overlay['overlay_sha256'] != reference['overlay_sha256']:
        raise ValueError('Captured split overlay content differs')
    return overlay


def validate_manifest_overlay(document, allow_subset=False):
    """Read-only launcher/preflight gate; capture split and frozen specs stay intact."""
    reference = document.get('split_overlay')
    if reference is None:
        return None
    from base5_capture_safety import sha256
    path = Path(reference['path'])
    if sha256(path) != reference['sha256']:
        raise ValueError('Manifest split overlay checksum differs')
    overlay = read_overlay(path)
    verify_source_pins(overlay)
    records = []
    for entry in document['scenes']:
        meta = json.loads(Path(entry['metadata']).read_text())
        if entry['split'] != meta['split']:
            raise ValueError('Capture/source split differs; overlay must not rewrite it')
        row = resolve(overlay, entry['id'], entry, meta)
        if entry.get('derived_training_split') != row['split']:
            raise ValueError('Candidate effective split differs')
        if entry.get('derived_layout_group_id') != row['group_id']:
            raise ValueError('Candidate effective layout group differs')
        if not meta.get('training_split_overlay') or overlay_for_metadata(meta, overlay) is not overlay:
            raise ValueError('Candidate metadata lacks its matching training overlay')
        tag = meta['training_split_overlay']
        if not tag.get('source_metadata_path') or not tag.get('source_metadata_sha256'):
            raise ValueError('Candidate metadata has no pinned original metadata')
        source_meta = json.loads(Path(tag['source_metadata_path']).read_text())
        if ({k:v for k,v in source_meta.items() if k != 'training_split_overlay'}
                != {k:v for k,v in meta.items() if k != 'training_split_overlay'}):
            raise ValueError('Candidate changed non-overlay metadata fields')
        records.append(dict(id=entry['id'], meta=meta, scenario_sha256=entry['scenario_sha256']))
    ids = {r['id'] for r in records}
    if allow_subset and ids != set(overlay['assignments']):
        # A smoke/child manifest is not a new independent split. Verify every
        # selected payload against the qualified complete overlay, without
        # demanding train coverage inside a deliberately val-only subset.
        return dict(errors=[], frames=len(records), scope='bound_subset_of_qualified_full_deck',
            full_overlay_frames=len(overlay['assignments']),
            splits=dict(Counter(resolve(overlay, entry['id'], entry,
                json.loads(Path(entry['metadata']).read_text()))['split'] for entry in document['scenes'])))
    audit = audit_assignments(records, overlay['assignments'], overlay['policy'])
    if audit['errors']:
        raise ValueError('Manifest layout split audit failed: '+str(audit['errors']))
    return audit


def register_recovery_overlay(source_entry, original, candidate, proposed, output):
    """Strict pre-capture rebind for a bounded replacement, never raw rewriting.

    A proposal must retain its parent's effective split/layout group, actor
    roster, motion, range/phase/yaw contract and observed train coverage floors.
    New geometry is checked against the complete original deck and previous
    rebindings. This is CPU split approval only, not native/capture approval.
    """
    import copy
    from base5_capture_safety import atomic_json, sha256
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh recovery split overlay output required')
    overlay = overlay_for_metadata(original)
    if overlay is None:
        raise ValueError('Recovery source has no versioned layout overlay')
    parent_id = source_entry['id']
    binding = resolve(overlay, parent_id, source_entry, original)
    verify_source_pins(overlay)
    parent_meta = json.loads(Path(source_entry['metadata']).read_text())
    tag = original.get('training_split_overlay', {})
    upstream = json.loads(Path(tag['source_metadata_path']).read_text()) if tag.get('source_metadata_path') else parent_meta
    if (parent_meta != original or {k:v for k,v in upstream.items() if k != 'training_split_overlay'}
            != {k:v for k,v in original.items() if k != 'training_split_overlay'}):
        raise ValueError('Recovery parent non-overlay metadata differs from its pinned source')
    declaration = proposed.get('fresh_completion_source', {})
    if (declaration.get('source_cell_id') != parent_id
            or declaration.get('source_scenario_sha256') != source_entry['scenario_sha256']
            or type(declaration.get('ordinal')) is not int or declaration['ordinal'] < 1
            or candidate['id'] == parent_id or candidate['id'] in overlay['assignments']
            or candidate['id'] in overlay.get('recovery_aliases', {})
            or parent_id in overlay.get('recovery_aliases', {})
            or candidate.get('source_cell_id', candidate['id']) != candidate['id']
            or proposed['split'] != original['split']
            or proposed.get('scene_group_id') != original.get('scene_group_id')):
        raise ValueError('Recovery mapping/source split/group is not explicitly preserved')
    protected = ('family', 'mission', 'role', 'distance_bin', 'distance_bounds', 'yaw_limit', 'motion',
        'topology', 'avoidance_phase', 'avoidance_side', 'avoidance_gap_bounds_m',
        'dynamic_observation_phase', 'roundabout_phase', 'shadow_phase', 'shadow_observation_stage')
    if any(original['spec'].get(k) != proposed['spec'].get(k) for k in protected):
        raise ValueError('Recovery changed frozen family/range/phase/yaw contract')
    roster = lambda meta: [(a['data_id'], a['source_list'], a['motion'],
        (a.get('asset_scale') or {}).get('profile_id')) for a in meta['runtime_actors']]
    if roster(original) != roster(proposed):
        raise ValueError('Recovery changed actor identity/motion/scale roster')
    if sha256(candidate['scenario']) != candidate['scenario_sha256']:
        raise ValueError('Recovery payload checksum differs')
    fingerprint = digest(geometry(proposed))
    if candidate['scenario_sha256'] == binding['scenario_sha256'] or fingerprint == binding['geometry_sha256']:
        raise ValueError('Recovery must have genuinely new geometry, not renamed payload')
    document = json.loads(Path(overlay['source_manifest']).read_text())
    if sha256(overlay['source_manifest']) != overlay['source_manifest_sha256']:
        raise ValueError('Recovery source manifest changed')
    records = []
    for entry in document['scenes']:
        prior = overlay.get('recovery_bindings', {}).get(entry['id'], entry)
        meta = json.loads(Path(prior['metadata']).read_text())
        if entry['id'] == parent_id:
            meta, payload = proposed, candidate['scenario_sha256']
        else:
            resolve(overlay, entry['id'], dict(entry, scenario_sha256=prior['scenario_sha256']), meta)
            if nearby(geometry(meta), geometry(proposed), overlay['policy']):
                raise ValueError('Recovery duplicates another actual layout; propose distinct geometry')
            payload = prior['scenario_sha256']
        records.append(dict(id=entry['id'], meta=meta, scenario_sha256=payload))
    rebound = copy.deepcopy(overlay)
    rebound.pop('overlay_sha256', None)
    rebound['assignments'][parent_id] = dict(binding, scenario_sha256=candidate['scenario_sha256'],
        geometry_sha256=fingerprint, source_split=proposed['split'])
    rebound.setdefault('recovery_aliases', {})[candidate['id']] = parent_id
    rebound.setdefault('recovery_bindings', {})[parent_id] = dict(candidate,
        metadata_content_sha256=digest({k:v for k,v in proposed.items() if k != 'training_split_overlay'}))
    audit = audit_assignments(records, rebound['assignments'], rebound['policy'])
    hard_train = Counter()
    groups_train = defaultdict(set)
    for record in records:
        row = rebound['assignments'][record['id']]
        if row['split'] == 'train':
            groups_train[row['group_id']].update(features(record['meta'], rebound['policy'])[0])
    for values in groups_train.values():
        hard_train.update(values)
    floors = overlay['audit'].get('train_floors', {})
    violated = [key for key, floor in floors.items() if hard_train[key] < floor]
    if audit['errors'] or violated:
        raise ValueError('Recovery layout leakage/coverage changed: '+str(audit['errors']+violated))
    audit.update(status='CPU_PASS', train_floors=floors, train_floor_violations=[],
        recovery_mapping=dict(source_id=parent_id, candidate_id=candidate['id'],
            ordinal=declaration['ordinal'], old_scenario_sha256=binding['scenario_sha256'],
            new_scenario_sha256=candidate['scenario_sha256'], effective_split=binding['split'],
            preserved_layout_group=binding['group_id']), native_recovery_qualified=False)
    rebound['audit'] = audit
    rebound['parent_overlay_sha256'] = overlay['overlay_sha256']
    rebound['recovery_implementation_sha256'] = sha256(__file__)
    rebound['overlay_sha256'] = digest(rebound)
    output.mkdir(parents=True)
    source_metadata_path = output/'source_metadata.json'
    atomic_json(source_metadata_path, {k:v for k,v in proposed.items() if k != 'training_split_overlay'})
    path = output/'split_overlay.json'
    atomic_json(path, rebound)
    reference = dict(path=str(path), sha256=sha256(path), overlay_sha256=rebound['overlay_sha256'],
        source_metadata_path=str(source_metadata_path), source_metadata_sha256=sha256(source_metadata_path))
    return reference
