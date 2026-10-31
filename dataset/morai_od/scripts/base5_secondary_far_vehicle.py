"""Opt-in far secondary NPC observations with all other source geometry frozen.

This never changes the finished v152 source, native QA or a primary quota. A
declared source vehicle is moved along its original legal lane family only.
"""
import copy
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import random

import generate_base5_independent as gen
import base5_far_vehicle_visibility as visibility
from base5_map_surface import configure, initial_pose
from base5_reference_frames import references

POLICY = 'frozen-source-secondary-far-vehicle-v153d'
PLAN_KEY = 'secondary_far_vehicle'
CONFIG_KEY = 'secondary_far_vehicle_policy'
MAX_PROPOSALS = 512
RANKED_CANDIDATES = 8
RETRY_BLOCK = 4096
PROPOSAL_SEARCH = 'first-draw-order; distinct legal road points; bounded512/rank8'
ALLOWED_BOUNDS = ((60, 80), (80, 90), (90, 95), (95, 99))


class FarContractError(RuntimeError):
    """Frozen source/config/helper violation; never an ordinary deficit."""


class FarGeometryExhausted(ValueError):
    """Bounded target-only legal geometry search was exhausted."""


ContractError = FarContractError
GeometryExhausted = FarGeometryExhausted


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    ensure_ascii=False, allow_nan=False).encode()).hexdigest()


plan_sha256 = digest


def has_plan(meta):
    return PLAN_KEY in meta.get('spec', {})


def configuration(environment, source_manifest):
    return dict(policy=POLICY, source_sha256=sha256(__file__), environment_tag=environment,
        source_manifest=str(Path(source_manifest).resolve()),
        source_manifest_sha256=sha256(source_manifest), bulk_started=False,
        proposal_search=PROPOSAL_SEARCH,
        proposal_visibility=visibility.configuration())


def lane_token(link):
    for lane, path in gen.geo.HIGHWAY_LANE_LINK_PATHS.items():
        if link in path:
            return 'mainline_' + str(lane)
    if link in ('A2256W000846', 'A2256W000411'):
        return 'merge'
    if link in ('A2256W000153', 'A2256W000451', 'A2256W000446'):
        return 'post_right'
    if link in ('A2256W000432', 'A2256W000427', 'A2256W000444'):
        return 'post_left'
    return None


def lane_relation(ego_link, target_link):
    a, b = lane_token(ego_link), lane_token(target_link)
    if a is None or b is None:
        return None
    if a == b:
        return 'same'
    adjacent = {frozenset(pair) for pair in (('mainline_6', 'mainline_8'),
        ('mainline_8', 'mainline_10'), ('merge', 'mainline_6'), ('post_right', 'post_left'))}
    return 'adjacent' if frozenset((a, b)) in adjacent else None


def lane_path(link):
    token = lane_token(link)
    if token and token.startswith('mainline_'):
        return list(gen.geo.HIGHWAY_LANE_LINK_PATHS[int(token.split('_')[1])])
    return dict(merge=['A2256W000846', 'A2256W000411'],
        post_right=['A2256W000153', 'A2256W000451', 'A2256W000446'],
        post_left=['A2256W000432', 'A2256W000427', 'A2256W000444']).get(token)


def _read_source(plan):
    manifest = plan['source_manifest']
    if sha256(manifest['path']) != manifest['sha256']:
        raise FarContractError('Far secondary original manifest changed')
    document = _frozen_json(manifest['path'], manifest['sha256'])
    if document['config']['environment_tag'] != plan['environment_tag']:
        raise FarContractError('Far secondary cannot reuse another environment source geometry')
    docs = []
    for key in ('scenario', 'metadata'):
        pin = plan['source'][key]
        path = Path(pin['path'])
        if not path.is_absolute() or sha256(path) != pin['sha256']:
            raise FarContractError('Far secondary source ' + key + ' changed')
        docs.append(_frozen_json(str(path), pin['sha256']))
    return docs


@lru_cache(maxsize=512)
def _frozen_json(path, expected):
    # Callers hash the current file on every use. Cache parsing, not integrity.
    return json.loads(Path(path).read_text())


def attach(entry, meta, scene, ordinal, bounds, policy, sampling_bounds_m=None):
    """Return one explicit frozen source plan; no source object is mutated."""
    row = copy.deepcopy(meta['spec'])
    actor = meta['runtime_actors'][ordinal]
    npc = next(a for a in scene['vehicleList'] if a['UNIQUEID'] == actor['unique_id'])
    target_link = actor.get('map_ground_initializer', {}).get('link_id')
    ego_link = meta['base']['link_idx']
    relation = lane_relation(ego_link, target_link)
    path = lane_path(target_link)
    sampling = list(bounds if sampling_bounds_m is None else sampling_bounds_m)
    if (tuple(sampling) != tuple(bounds) and not (
            tuple(bounds) == (95, 99) and tuple(sampling) in ((95, 97), (97, 99)))):
        raise FarContractError('Far secondary sampling window is not an explicitly supported allocation')
    if (row.get(PLAN_KEY) or row['family'] != 'highway' or row['mission']
            or row.get('shadow_phase') or ordinal < 1 or actor['motion'] != 'static'
            or actor['source_list'] != 'vehicleList' or relation is None or not path
            or npc['initPosition']['initLink'] != target_link
            or not 30 <= actor['planned_center_distance_m'] < 50
            or tuple(bounds) not in ALLOWED_BOUNDS
            or actor['actual_ego_role'] not in ('front', 'rear')):
        raise FarGeometryExhausted('Source is not a supported same/adjacent-lane far donor')
    plan = dict(policy=POLICY, implementation_sha256=policy['source_sha256'],
        proposal_search=policy['proposal_search'],
        proposal_visibility=copy.deepcopy(policy['proposal_visibility']),
        environment_tag=policy['environment_tag'], logical_slot_id=entry['id'],
        source={key: dict(path=str(Path(entry[key]).resolve()), sha256=sha256(entry[key]))
                for key in ('scenario', 'metadata')},
        source_manifest_sha256=policy['source_manifest_sha256'],
        source_manifest=dict(path=policy['source_manifest'], sha256=policy['source_manifest_sha256']),
        source_spec_sha256=digest(row), frozen_seed=row['seed'], split=meta['split'],
        target_ordinal=ordinal, target_uid=actor['unique_id'], target_data_id=actor['data_id'],
        target_name=actor['name'], distance_bounds_m=list(bounds), role=actor['actual_ego_role'],
        sampling_bounds_m=sampling,
        original_distance_m=actor['planned_center_distance_m'],
        source_ego_link=ego_link, source_target_link=target_link,
        lane_identity=lane_token(target_link), lane_relation=relation, route_links=path,
        actor_identity=[{k: a.get(k) for k in ('unique_id', 'data_id', 'name', 'source_list',
            'population_pool', 'motion')} for a in meta['runtime_actors']],
        all_non_target_payload_and_metadata_preserved=True, primary_preserved=True,
        native_qa_unchanged=True)
    row[PLAN_KEY] = plan
    return row


def validate_plan(row, config=None):
    if PLAN_KEY not in row:
        return None
    try:
        plan = row[PLAN_KEY]
        policy = config[CONFIG_KEY] if config is not None else None
        if (not policy or policy['policy'] != POLICY or plan['policy'] != POLICY
                or policy['source_sha256'] != sha256(__file__)
                or plan['implementation_sha256'] != policy['source_sha256']
                or plan['environment_tag'] != config['environment_tag']
                or policy['environment_tag'] != config['environment_tag']
                or plan['proposal_search'] != policy['proposal_search']
                or policy['proposal_search'] != PROPOSAL_SEARCH
                or plan['proposal_visibility'] != policy['proposal_visibility']
                or plan['source_manifest_sha256'] != policy['source_manifest_sha256']
                or sha256(policy['source_manifest']) != policy['source_manifest_sha256']):
            raise FarContractError('Far secondary policy/helper/environment/source manifest changed')
        _visibility_calibration(plan)
        regional = config.get('regional_policy')
        if regional:
            from base5_region_contract import POLICY as REGION_POLICY
            if regional.get('policy') != REGION_POLICY:
                raise FarContractError('Far secondary regional policy changed')
        scene, source = _read_source(plan)
        expected = copy.deepcopy(source['spec'])
        if (PLAN_KEY in expected or digest(expected) != plan['source_spec_sha256']
                or expected['seed'] != plan['frozen_seed'] or source['split'] != plan['split']):
            raise FarContractError('Far secondary source spec/seed/split changed')
        expected[PLAN_KEY] = copy.deepcopy(plan)
        if row != expected:
            raise FarContractError('Far secondary changed source spec outside its added plan')
        index = plan['target_ordinal']
        if isinstance(index, bool) or not isinstance(index, int) or not 1 <= index < len(source['runtime_actors']):
            raise FarContractError('Far secondary target ordinal must be a secondary')
        actor = source['runtime_actors'][index]
        identity = [{k: a.get(k) for k in plan['actor_identity'][i]} for i, a in enumerate(source['runtime_actors'])]
        if (identity != plan['actor_identity'] or plan['target_uid'] != 8500 + index
                or actor['unique_id'] != plan['target_uid'] or actor['data_id'] != plan['target_data_id']
                or actor['name'] != plan['target_name'] or actor['motion'] != 'static'
                or actor['source_list'] != 'vehicleList' or source['spec']['family'] != 'highway'
                or source['spec']['mission'] or source['spec'].get('shadow_phase')
                or tuple(plan['distance_bounds_m']) not in ALLOWED_BOUNDS
                or plan['role'] != actor['actual_ego_role'] or plan['role'] not in ('front', 'rear')
                or not 30 <= actor['planned_center_distance_m'] < 50):
            raise FarContractError('Far secondary frozen actor/scope/role/bounds changed')
        target = next(a for a in scene['vehicleList'] if a['UNIQUEID'] == plan['target_uid'])
        link = actor['map_ground_initializer']['link_id']
        if (link != plan['source_target_link'] or target['initPosition']['initLink'] != link
                or source['base']['link_idx'] != plan['source_ego_link']
                or lane_token(link) != plan['lane_identity'] or lane_path(link) != plan['route_links']
                or lane_relation(plan['source_ego_link'], link) != plan['lane_relation']):
            raise FarContractError('Far secondary source legal lane contract changed')
        _validate_source_only(row)
        return plan
    except FarContractError:
        raise
    except (OSError, ValueError, KeyError, TypeError, IndexError, StopIteration) as error:
        raise FarContractError('Malformed far secondary source/plan: ' + str(error)) from error


def _validate_source_only(row):
    """Hash/source checks available to native audit without a config object."""
    plan = row[PLAN_KEY]
    if (plan['implementation_sha256'] != sha256(__file__) or plan['policy'] != POLICY
            or plan['proposal_search'] != PROPOSAL_SEARCH):
        raise FarContractError('Far secondary helper changed')
    _visibility_calibration(plan)
    index = plan['target_ordinal']
    if isinstance(index, bool) or not isinstance(index, int) or index < 1:
        raise FarContractError('Far secondary audit requires an integer secondary ordinal')
    scene, source = _read_source(plan)
    expected = copy.deepcopy(source['spec'])
    expected[PLAN_KEY] = copy.deepcopy(plan)
    if row != expected or digest(source['spec']) != plan['source_spec_sha256']:
        raise FarContractError('Far secondary source spec changed')
    entry = dict(id=plan['logical_slot_id'], **{key: plan['source'][key]['path']
        for key in ('scenario', 'metadata')})
    configuration = dict(source_sha256=plan['implementation_sha256'],
        source_manifest=plan['source_manifest']['path'],
        source_manifest_sha256=plan['source_manifest_sha256'], environment_tag=plan['environment_tag'],
        proposal_search=plan['proposal_search'],
        proposal_visibility=copy.deepcopy(plan['proposal_visibility']))
    reconstructed = attach(entry, source, scene, plan['target_ordinal'],
                           plan['distance_bounds_m'], configuration, plan['sampling_bounds_m'])[PLAN_KEY]
    if reconstructed != plan:
        raise FarContractError('Far secondary declared source assignment changed')
    return scene, source, plan


def _visibility_calibration(plan, calibration=None):
    try:
        return visibility.validate_policy(plan['proposal_visibility'], calibration)
    except (KeyError, RuntimeError) as error:
        raise FarContractError('Far secondary proposal visibility contract changed: '+str(error)) from error


def score_proposal(meta, calibration, kind, score_fn):
    """Same frozen 3-D guard for initial allocation and source-only recovery."""
    plan = meta['spec'][PLAN_KEY]
    _visibility_calibration(plan, calibration)
    return visibility.score(meta, calibration, kind, score_fn,
                            plan['proposal_visibility']['geometry_reference']['path'])


def verify_preserved(original_meta, new_meta):
    """Check source hashes and exact non-target metadata, including native audit."""
    if not has_plan(original_meta):
        if has_plan(new_meta):
            raise FarContractError('Foreign far-secondary assignment introduced')
        return None
    try:
        _, canonical, plan = _validate_source_only(original_meta['spec'])
        if new_meta['spec'] != original_meta['spec']:
            raise FarContractError('Far secondary frozen spec changed')
        old, new = canonical['runtime_actors'], new_meta['runtime_actors']
        index = plan['target_ordinal']
        if len(old) != len(new):
            raise FarContractError('Far secondary actor count changed')
        for i, (before, after) in enumerate(zip(old, new)):
            if i != index and before != after:
                raise FarContractError('Far secondary non-target metadata changed')
            if i == index:
                identity = plan['actor_identity'][i]
                if any(after.get(k) != v for k, v in identity.items()):
                    raise FarContractError('Far secondary target identity/motion changed')
                mutable = {'position', 'geometric_center_estimate', 'planned_center_distance_m',
                    'planned_surface_clearance_m', 'longitudinal_m', 'lateral_m', 'actual_ego_role',
                    'placement_yaw_deg', 'reference_frames', 'static_height_initializer',
                    'map_ground_initializer', 'secondary_far_vehicle'}
                if {k: v for k, v in before.items() if k not in mutable} != {
                        k: v for k, v in after.items() if k not in mutable}:
                    raise FarContractError('Far secondary changed target non-pose metadata')
        mutable = {'spec', 'runtime_actors', 'retry', 'secondary_far_vehicle_generation',
            'flexible_completion', 'fresh_completion_source', 'environment_split_contract'}
        for key, value in canonical.items():
            if key not in mutable and new_meta.get(key) != value:
                raise FarContractError('Far secondary changed source ' + key)
        target = new[index]
        distance = float(target['planned_center_distance_m'])
        lower, upper = plan['distance_bounds_m']
        window_lower, window_upper = plan['sampling_bounds_m']
        if (not math.isfinite(distance) or not lower <= distance < upper
                or not window_lower <= distance < window_upper
                or target['actual_ego_role'] != plan['role']
                or lane_token(target['map_ground_initializer']['link_id']) != plan['lane_identity']
                or target['map_ground_initializer']['link_id'] not in plan['route_links']
                or target.get('secondary_far_vehicle', {}).get('plan_sha256') != digest(plan)):
            raise FarContractError('Far secondary actual distance/role/lane/proof changed')
        proof = visibility.evaluate(new_meta, _visibility_calibration(plan),
                                   plan['proposal_visibility']['geometry_reference']['path'])
        if (proof['rejected'] or new_meta.get('secondary_far_vehicle_generation', {}).get(
                'proposal_visibility') != proof):
            raise FarContractError('Far secondary proposal visibility proof missing/changed/blocked')
        return plan
    except FarContractError:
        raise
    except (OSError, ValueError, KeyError, TypeError, IndexError, StopIteration) as error:
        raise FarContractError('Invalid preserved far-secondary metadata: ' + str(error)) from error


def verify_payload(scene, source_scene, plan, metadata=None):
    expected = copy.deepcopy(source_scene)
    uid = plan['target_uid']
    targets = [a for a in scene['vehicleList'] if a['UNIQUEID'] == uid]
    if len(targets) != 1:
        raise FarContractError('Far secondary serialized target missing/duplicated')
    previous = next(a for a in source_scene['vehicleList'] if a['UNIQUEID'] == uid)
    initial = targets[0]['initPosition']
    if (initial['initLink'] not in plan['route_links']
            or lane_token(initial['initLink']) != plan['lane_identity']):
        raise FarContractError('Far secondary serialized target left its frozen legal lane path')
    try:
        xyz = [float(initial['pos'][k]) for k in ('x', 'y', 'z')]
        yaw = float(initial['rot']['yaw'])
        ratio = float(initial['initLinkRatio'])
        if not all(math.isfinite(v) for v in xyz+[yaw, ratio]) or not 0 <= ratio <= 1:
            raise FarContractError('Far secondary serialized pose is nonfinite/outside link ratio')
        if metadata is not None:
            actor = metadata['runtime_actors'][plan['target_ordinal']]
            expected_xyz = [float(actor['position'][k]) for k in ('x', 'y', 'z')]
            expected_xyz[2] += .36  # Existing MORAI static-NPC serializer convention.
            delta_yaw = (yaw-float(actor['placement_yaw_deg'])+180) % 360-180
            if (any(abs(a-b) > 1e-5 for a, b in zip(xyz, expected_xyz))
                    or abs(delta_yaw) > .0011
                    or actor['map_ground_initializer']['link_id'] != initial['initLink']):
                raise FarContractError('Far secondary serialized target pose disagrees with metadata')
    except (KeyError, TypeError, ValueError, IndexError) as error:
        raise FarContractError('Far secondary malformed serialized target pose') from error
    old_target, new_target = copy.deepcopy(previous), copy.deepcopy(targets[0])
    for target in (old_target, new_target):
        for key in ('pos', 'initLink', 'initLinkRatio'):
            target['initPosition'].pop(key, None)
        target['initPosition']['rot'].pop('yaw', None)
    if old_target != new_target:
        raise FarContractError('Far secondary changed target state/gear/speed/behavior outside pose/link')
    for i, actor in enumerate(expected['vehicleList']):
        if actor['UNIQUEID'] == uid:
            expected['vehicleList'][i] = copy.deepcopy(targets[0])
            break
    if expected != scene or targets[0]['DataID'] != plan['target_data_id']:
        raise FarContractError('Far secondary changed another serialized actor/Ego/context')


def _candidates(source, plan, ctx):
    """Fixed source Ego, fixed legal lane; cache pure route/clearance geometry."""
    key = digest(plan)
    cache = ctx.setdefault('_secondary_far_candidates', {})
    if key in cache:
        return cache[key]
    for left, right in zip(plan['route_links'], plan['route_links'][1:]):
        if ctx['links'][left]['to_node_idx'] != ctx['links'][right]['from_node_idx']:
            raise FarContractError('Far secondary route disconnected')
    route, arcs = gen.geo.route_from_link_path(ctx['links'], plan['route_links'])
    actor = source['runtime_actors'][plan['target_ordinal']]
    size = actor['estimated_size']
    angle = math.radians(actor['placement_yaw_deg'])
    dx, dy = [actor['geometric_center_estimate'][i] - actor['position'][axis] for i, axis in enumerate('xy')]
    offset = [math.cos(angle) * dx + math.sin(angle) * dy,
              -math.sin(angle) * dx + math.cos(angle) * dy, size[2] / 2]
    ego = source['ego']; ea = math.radians(ego['yaw']); c, s = math.cos(ea), math.sin(ea)
    ego_box = gen.rectangle([ego['x'] + 1.35*c, ego['y'] + 1.35*s], [4.8, 2.], ego['yaw'])
    other_boxes = [gen.rectangle(a['geometric_center_estimate'], a['estimated_size'], a['placement_yaw_deg'])
        for i, a in enumerate(source['runtime_actors']) if i != plan['target_ordinal']]
    candidates = []
    for arc in arcs:
        pose = gen.geo.continuous_route_pose_at_arc(route, arcs, float(arc))
        pose['yaw'] = round(pose['yaw'], 3)
        pa = math.radians(pose['yaw']); ca, sa = math.cos(pa), math.sin(pa)
        center = [pose['x'] + ca*offset[0] - sa*offset[1], pose['y'] + sa*offset[0] + ca*offset[1]]
        dx, dy = center[0] - ego['x'], center[1] - ego['y']
        lon, lat = c*dx+s*dy, -s*dx+c*dy
        distance = math.hypot(dx, dy)
        lower, upper = plan['sampling_bounds_m']
        role = ('front' if lon >= 0 else 'rear') if abs(lon) >= abs(lat) else ('left' if lat >= 0 else 'right')
        if (not lower <= distance < upper or role != plan['role'] or abs(lat) >= 29.5
                or abs(lon) >= 99.5 or lane_token(pose['link_idx']) != plan['lane_identity']):
            continue
        box = gen.rectangle(center, size, pose['yaw'])
        if box.distance(ego_box) < .3 or any(box.distance(other) < .2 for other in other_boxes):
            continue
        candidates.append((pose, center, lon, lat, distance, box.distance(ego_box)))
    cache[key] = candidates, offset
    return cache[key]


def _draw_index(plan, attempt, count):
    seed = int(digest([POLICY, plan['frozen_seed'], plan['logical_slot_id'], attempt])[:16], 16)
    # choice(sequence) and randrange(len(sequence)) use the same _randbelow draw.
    return random.Random(seed).randrange(count)


def proposal_index(row, ctx, attempt):
    """Pure same-RNG road point identity; does not replace actual build guards."""
    if isinstance(attempt, bool) or not isinstance(attempt, int) or attempt < 0:
        raise FarContractError('Nonnegative deterministic far-secondary attempt required')
    plan = row[PLAN_KEY]
    _, source = _read_source(plan)
    candidates, _ = _candidates(source, plan, ctx)
    if not candidates:
        raise FarGeometryExhausted('No same-lane far secondary road point within frozen source geometry')
    return _draw_index(plan, attempt, len(candidates)), len(candidates)


def build(row, config, ctx, attempt=0):
    plan = validate_plan(row, config)
    if plan is None:
        raise FarContractError('Far secondary build requires its explicit plan')
    if isinstance(attempt, bool) or not isinstance(attempt, int) or attempt < 0:
        raise FarContractError('Nonnegative deterministic far-secondary attempt required')
    source_scene, source = _read_source(plan)
    candidates, offset = _candidates(source, plan, ctx)
    if not candidates:
        raise FarGeometryExhausted('No same-lane far secondary road point within frozen source geometry')
    index = _draw_index(plan, attempt, len(candidates))
    pose, center, lon, lat, distance, gap = copy.deepcopy(candidates[index])
    actor = source['runtime_actors'][plan['target_ordinal']]
    asset = next((a for a in ctx['pools']['vehicle'] if a['data_id'] == plan['target_data_id']), None)
    if asset is None or asset['name'] != actor['name']:
        raise FarContractError('Frozen far-secondary catalog identity unavailable')
    configure(ctx, config.get('placement_ground'))
    runtime, height, ground = initial_pose(pose, asset, actor['estimated_size'], offset, ctx)
    if (not ground or not ground['usable'] or ground['confidence'] not in ('road_strip', 'near_centerline')
            or not ground['footprint']['all_supported'] or lane_token(ground['link_id']) != plan['lane_identity']):
        raise FarGeometryExhausted('Far secondary road-height/footprint support unavailable')
    scene, meta = copy.deepcopy(source_scene), copy.deepcopy(source)
    npc = next(a for a in source_scene['vehicleList'] if a['UNIQUEID'] == plan['target_uid'])
    temporary = dict(vehicleList=[])
    gen.set_pose(copy.deepcopy(npc), runtime, plan['target_uid'], asset, temporary, 'static',
                 stationary_vehicle_policy=config['capture'].get('static_vehicle_policy'))
    for i, value in enumerate(scene['vehicleList']):
        if value['UNIQUEID'] == plan['target_uid']:
            scene['vehicleList'][i] = temporary['vehicleList'][0]
    changed = meta['runtime_actors'][plan['target_ordinal']]
    changed.update(position={k: runtime[k] for k in ('x', 'y', 'z')},
        geometric_center_estimate=center, planned_center_distance_m=distance,
        planned_surface_clearance_m=gap, longitudinal_m=lon, lateral_m=lat,
        actual_ego_role=plan['role'], placement_yaw_deg=pose['yaw'],
        reference_frames=references(center, meta['ego'], meta['base']['yaw']),
        static_height_initializer=height, map_ground_initializer=ground,
        secondary_far_vehicle=dict(policy=POLICY, plan_sha256=digest(plan),
            lane_identity=plan['lane_identity'], lane_relation=plan['lane_relation'],
            actual_center_distance_m=distance, legal_link=pose['link_idx'],
            source_target_preserved_except_pose=True, native_qa_unchanged=True))
    meta['spec'] = copy.deepcopy(row)
    meta['retry'] = attempt
    meta['secondary_far_vehicle_generation'] = dict(policy=POLICY, attempt=attempt,
        source_scenario_sha256=plan['source']['scenario']['sha256'],
        source_metadata_sha256=plan['source']['metadata']['sha256'],
        frozen_ego_and_all_non_target=True, candidate_road_points=len(candidates))
    # Reject before the remaining regional work; this never approves a raw or
    # waives those gates for the partial/uncertain proposals that remain.
    calibration = _visibility_calibration(plan)
    proof = visibility.evaluate(meta, calibration,
                               plan['proposal_visibility']['geometry_reference']['path'])
    if proof['rejected']:
        raise FarGeometryExhausted('Far secondary complete single-foreground 3-D proposal blockage')
    meta['secondary_far_vehicle_generation']['proposal_visibility'] = proof
    # The early generator hook must retain the existing end-of-build regional
    # gates, including protection of the unchanged primary. This is the old
    # placement contract, not a new 2-D claim of rendered visibility.
    from base5_region_contract import validate as validate_region
    try:
        region_proof = validate_region(gen, ctx, meta, config)
    except ValueError as error:
        raise FarGeometryExhausted(str(error)) from error
    if region_proof != source.get('regional_placement'):
        raise FarGeometryExhausted('Far secondary would change frozen regional placement proof')
    from base5_static_vehicle import validate_serialized
    try:
        validate_serialized(meta, scene, config['capture'].get('static_vehicle_policy'))
    except (ValueError, KeyError, StopIteration) as error:
        raise FarContractError('Far secondary static-state serialization changed') from error
    verify_preserved(meta, meta)
    verify_payload(scene, source_scene, plan, meta)
    return scene, meta


def choose(source, config, ctx, calibration, ordinal, ident, known_shas,
           build, proposal_score, novelty):
    """Source-only bounded completion; novelty measures the moved secondary."""
    if isinstance(ordinal, bool) or not isinstance(ordinal, int) or not 1 <= ordinal <= 5:
        raise FarContractError('Bounded far-secondary completion ordinal required')
    plan = validate_plan(source['spec'], config)
    if plan is None:
        raise FarContractError('Far secondary completion requires a frozen source plan')
    verify_preserved(source, source)
    _visibility_calibration(plan, calibration)
    first = source.get('retry', 0) + 1 + (ordinal-1)*RETRY_BLOCK
    ranked, visited, point_count, draws, built_count = [], set(), None, 0, 0
    for retry in range(first, first+MAX_PROPOSALS):
        if point_count is not None and len(visited) == point_count:
            break
        draws += 1
        validate_plan(source['spec'], config)
        try:
            index, point_count = proposal_index(source['spec'], ctx, retry)
            if index in visited:
                continue
            visited.add(index)
            built_count += 1
            scene, meta = build(copy.deepcopy(source['spec']), config, ctx, retry)
        except FarContractError:
            raise
        except FarGeometryExhausted:
            continue
        verify_preserved(source, meta)
        payload = hashlib.sha256((json.dumps(scene, ensure_ascii=False, indent=2)+'\n').encode()).hexdigest()
        if payload in known_shas:
            continue
        index = plan['target_ordinal']
        a, b = source['runtime_actors'][index], meta['runtime_actors'][index]
        displacement = math.dist(a['geometric_center_estimate'], b['geometric_center_estimate'])
        fresh = min(2., displacement / max(1., min(10., a['planned_center_distance_m']*.1)))
        if fresh < .25:
            continue
        # The moved vehicle, not the unchanged primary, is the ranking target.
        view = copy.deepcopy(meta)
        actors = view['runtime_actors']
        view['runtime_actors'] = [actors[index]] + actors[:index] + actors[index+1:]
        proxy = score_proposal(view, calibration, 'all_actor_visibility', proposal_score)
        if not math.isfinite(proxy['score']):
            continue
        ranked.append((proxy['score']+min(1., fresh), -retry, scene, meta, proxy, fresh, displacement))
        if len(ranked) >= min(RANKED_CANDIDATES, point_count):
            break
    if not ranked:
        raise FarGeometryExhausted('No novel far-secondary layout within 512 offline proposals')
    _, negative, scene, meta, proxy, fresh, displacement = max(ranked, key=lambda a: a[:2])
    meta['flexible_completion'] = dict(policy=POLICY, source_cell_id=ident, attempt=ordinal,
        generation_retry=-negative, retry_block_start=first, proposals_tried=-negative-first+1,
        random_draws_considered=draws, distinct_geometry_proposals=built_count,
        available_legal_road_points=point_count,
        valid_candidates=len(ranked), maximum_proposals=MAX_PROPOSALS,
        ranked_candidate_cap=RANKED_CANDIDATES, source_seed=plan['frozen_seed'],
        donor_selection='disabled; frozen source geometry and target lane/band',
        target_displacement_m=displacement, relative_view_novelty=fresh, proxy=proxy,
        native_qa_unchanged=True)
    return scene, meta
