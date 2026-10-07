"""Bounded editor waypoint-height lookup, NOT a universal mesh/ground certificate."""
import bisect
import copy
import math
from pathlib import Path

from base5_capture_safety import sha256

POLICY = 'editor-intersection-path-v116'
QA_POLICY = 'manual-ground-single-capture-advisory-v116'


def profile_from_scene(scene, row):
    from generate_dynamic_object_capture_set import waypoint_to_world
    actors = scene.get(row['source_list'], [])
    if len(actors) != 1 or int(actors[0]['DataID']) != row['data_id']:
        raise ValueError('Manual scene must retain exactly the intended actor')
    actor = actors[0]
    paths = [p for p in scene.get('waypointDataList', [])
             if p['pathName'].endswith(' - ' + str(actor['UNIQUEID']))]
    if len(paths) != 1:
        raise ValueError('Expected one matching manually saved path')
    center = row['crosswalk']['center']; axis = row['axis']
    knots = []
    for node in paths[0]['waypointData']:
        p = waypoint_to_world(node['pos'])
        if not all(math.isfinite(p[k]) for k in ('x', 'y', 'z')):
            raise ValueError('Nonfinite manual waypoint')
        dx, dy = p['x'] - center['x'], p['y'] - center['y']
        knots.append(dict(p, s=dx*axis[0]+dy*axis[1], l=-dx*axis[1]+dy*axis[0]))
    if len(knots) < 2 or any(b['s']-a['s'] <= .02 for a, b in zip(knots, knots[1:])):
        raise ValueError('Manual path must progress along reference axis; no repeated or reversed knots')
    if max(abs(p['l']) for p in knots) > 2.:
        raise ValueError('Manual path moved too far from the intended crosswalk')
    if max(p['z'] for p in knots)-min(p['z'] for p in knots) > .5:
        raise ValueError('Large manual height discontinuity needs inspection')
    return dict(asset=row['asset'], data_id=row['data_id'], crosswalk_id=row['crosswalk']['crosswalk_id'],
                center=copy.deepcopy(center), axis=list(axis), knots=knots,
                s_bounds_m=[knots[0]['s'], knots[-1]['s']], maximum_path_offset_m=1.,
                manual_actor_position=actor['pos'],
                initial_actor_xy_displacement_m=math.hypot(actor['pos']['x']-knots[0]['x'], actor['pos']['y']-knots[0]['y']),
                first_knot_is_unchanged_seed=abs(knots[0]['z']-row['provisional_constant_z_m']) < .0011,
                pivot_semantics='Saved waypoint height; asset-specific motion pivot not independently separated',
                contact_verified=False)


def height(profile, x, y):
    center, axis, knots = profile['center'], profile['axis'], profile['knots']
    dx, dy = x-center['x'], y-center['y']
    s, l = dx*axis[0]+dy*axis[1], -dx*axis[1]+dy*axis[0]
    if not all(math.isfinite(v) for v in (s, l)) or s < knots[0]['s']-1e-8 or s > knots[-1]['s']+1e-8:
        raise ValueError('Outside measured manual path length')
    i = min(max(bisect.bisect_right([p['s'] for p in knots], s)-1, 0), len(knots)-2)
    a, b = knots[i:i+2]; t = (s-a['s'])/(b['s']-a['s'])
    path_l = a['l']+t*(b['l']-a['l'])
    distance = abs(l-path_l)
    if distance > profile['maximum_path_offset_m']:
        raise ValueError('Outside bounded manual path strip')
    return dict(x=x, y=y, z=a['z']+t*(b['z']-a['z']), s=s, l=l,
                path_l=path_l, lateral_distance_m=distance,
                contact_verified=False, interpolation='actual-XY-projected-piecewise-linear')


def load_lookup(path, expected_hash=None):
    import json
    path = Path(path)
    if expected_hash and sha256(path) != expected_hash:
        raise ValueError('Manual lookup hash mismatch')
    data = json.loads(path.read_text())
    if data.get('policy') != POLICY:
        raise ValueError('Unknown manual lookup policy')
    for item in data['sources']:
        if sha256(Path(item['path'])) != item['sha256']:
            raise ValueError('Manual source hash mismatch')
    return data


def validate_actor(actor, metadata, scene):
    """Verify every applied height and raw waypoint; pending lookups are diagnostic only."""
    import generate_dynamic_object_capture_set as geo
    path = actor['motion_path']; proof = path['ground_path_evidence']
    data = load_lookup(proof['lookup_file'], proof['lookup_sha256'])
    profile = data['profiles'][proof['profile_key']]
    if profile['asset'] != actor['name'] or profile['data_id'] != actor['data_id']:
        raise ValueError('Manual profile asset mismatch')
    if data.get('status') != 'LIVE_VALIDATED' and not metadata.get('calibration_only'):
        raise ValueError('Pending manual lookup is diagnostic only')
    if data.get('status') == 'LIVE_VALIDATED':
        evidence = data['live_evidence']
        if sha256(Path(evidence['path'])) != evidence['sha256']:
            raise ValueError('Manual live evidence hash mismatch')
        if proof['profile_key'] not in evidence['validated_profiles']:
            raise ValueError('Manual profile lacks live validation')
    for p in path['points'] + [path['target_position']]:
        q = height(profile, p['x'], p['y'])
        if abs(p['z']-q['z']) > 1e-8:
            raise ValueError('Manual applied height differs from lookup')
    if actor['position'] != path['points'][0]:
        raise ValueError('Manual departure metadata mismatch')
    raw_actor = next(a for a in scene[actor['source_list']] if a['UNIQUEID']==actor['unique_id'])
    if any(abs(raw_actor['pos'][k]-actor['position'][k]) > 1e-8 for k in ('x','y','z')):
        raise ValueError('Manual raw departure mismatch')
    raw = next(p for p in scene['waypointDataList'] if p['pathName'].endswith(' - '+str(actor['unique_id'])))
    if [p['pos'] for p in raw['waypointData']] != [geo.world_to_morai_waypoint(p) for p in path['points']]:
        raise ValueError('Manual raw waypoint mismatch')


def explicit_manual_mode(metadata):
    mode = metadata.get('ground_support_check', {}).get('manual_policy')
    if mode is None:
        return False
    if mode != QA_POLICY:
        raise ValueError('Unknown manual ground QA policy')
    moving = [a for a in metadata.get('runtime_actors',[]) if a['motion'] not in ('static','standing')]
    if not moving or any(a.get('motion_path',{}).get('ground_path_evidence',{}).get('policy') != POLICY for a in moving):
        raise ValueError('Manual QA requires explicit editor-prepared movers')
    return True


def ground_decision(ground):
    from base5_static_ground_policy import static_decision
    result = static_decision(ground)
    result['policy'] = QA_POLICY
    result['review_reasons'] = [r.replace('static_ground_', 'manual_ground_') for r in result['review_reasons']]
    result['limitation'] = 'Original cloud inspection; uncertainty is warning, not ground certification. Severe +/-40cm errors remain review.'
    return result


def vertical_review(actor, samples, window_start):
    """Compare actual moving positions with their local path, not a flat scalar Z."""
    from base5_capture_safety import vertical_motion_review
    result = vertical_motion_review(actor, samples, window_start)
    proof = actor['motion_path']['ground_path_evidence']
    data = load_lookup(proof['lookup_file'], proof['lookup_sha256'])
    profile = data['profiles'][proof['profile_key']]
    errors, outside = [], 0
    for p in samples:
        if p['uid'] != actor['unique_id'] or p['t'] < window_start:
            continue
        try:
            expected = height(profile, p['x'], p['y'])['z']
        except ValueError:
            outside += 1; continue
        errors.append(abs(p['z']-expected))
    if outside:
        result['review_reasons'].append('motion_outside_manual_height_domain')
    if not errors:
        result['review_reasons'].append('motion_manual_height_trace_unverified')
    elif max(errors) > .40:
        result['review_reasons'].append('motion_manual_height_severe_mismatch')
    result.update(manual_height_policy=POLICY, maximum_height_error_m=max(errors) if errors else None,
                  height_limit_m=.40, outside_domain_samples=outside,
                  minor_height_warning=bool(errors and max(errors)>.10))
    return result
