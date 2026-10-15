"""Opt-in secondary support redistribution, with immutable asset/count contracts.

No primary, mission, NPC, moving actor, yaw or acceptance policy is changed.
The independent RNG leaves the original builder stream intact. Old manifests
without this policy retain their exact generation behavior.
"""
import copy
import hashlib
import random

POLICY = 'secondary-obstacle-support-retain-near-v144'
VARIANTS = ('near50', 'near70', 'richfloor')


def eligible(row):
    return (not row['mission'] and not row.get('shadow_phase')
            and row['family'] not in ('highway','roundabout','background')
            and row.get('population_mode') in ('generic_regular','generic_dense'))


def attach(row, actors, variant):
    if variant not in VARIANTS: raise ValueError('Unknown obstacle support variant')
    if variant=='richfloor':
        from base5_obstacle_rich_floor import attach as rich_attach
        return rich_attach(row,actors)
    result=copy.deepcopy(row)
    if not eligible(result): return result
    if any(a['motion'] not in ('static','standing') for a in actors):return result
    physical=[i for i,a in enumerate(actors) if i and a['population_pool'].startswith('obstacle')
              and a['source_list']=='objectList']
    targets=[i for i in physical if actors[i]['planned_center_distance_m']>=20]
    if targets:
        result['frozen_population_data_ids']=[a['data_id'] for a in actors]
        result['actors']=len(actors)
        result['secondary_obstacle_support']=dict(policy=POLICY,variant=variant,ordinals=targets,
            protected_near_ordinals=[i for i in physical if i not in targets],
            primary_and_count_preserved=True,mission_and_moving_excluded=True)
    return result


def sample(row, ordinal, asset, size, role, attempt):
    plan=row.get('secondary_obstacle_support')
    if not plan: return None
    from base5_obstacle_rich_floor import POLICY as RICH_POLICY, sample as rich_sample
    if plan.get('policy')==RICH_POLICY:return rich_sample(row,ordinal,asset,size,role,attempt)
    if plan.get('policy')!=POLICY or plan.get('variant') not in ('near50','near70'):
        raise ValueError('Unknown secondary obstacle support policy')
    if not eligible(row) or 0 in plan['ordinals']:
        raise ValueError('Support policy cannot modify primary/mission/traffic/background')
    if ordinal not in plan['ordinals']: return None
    if asset['source_list']!='objectList' or asset.get('semantic_class')!='obstacle':
        raise ValueError('Support policy restricted to physical static obstacle assets')
    key=f"{POLICY}:{row['seed']}:{ordinal}:{asset['data_id']}"
    # Band identity survives quality retries, as does the frozen asset list.
    rank=int(hashlib.sha256(key.encode()).hexdigest()[:16],16)/2**64
    rng=random.Random(int(hashlib.sha256((key+':'+str(attempt)).encode()).hexdigest()[:16],16))
    near=rank < (.5 if plan['variant']=='near50' else .7)
    large=max(size[:2])>3 or size[0]*size[1]>3
    bounds=[8.,20.] if near and large else [6.,18.] if near else [20.,29.]
    distance=rng.uniform(bounds[0]+.1,bounds[1]-.1)
    return dict(policy=POLICY,variant=plan['variant'],ordinal=ordinal,role=role,
                band='near' if near else 'mid',distance_bounds_m=bounds,
                target_center_distance_m=distance,size_aware_large=large,
                primary_changed=False,count_changed=False,band_fixed_across_retry=True)


def verify_actual(proof, actual):
    if proof and not proof['distance_bounds_m'][0] <= actual < proof['distance_bounds_m'][1]:
        raise ValueError('Secondary support center escaped frozen distance band')


def geometry_summary(metadata):
    actors=metadata['runtime_actors']
    obstacles=[a for a in actors if a.get('population_pool','').startswith('obstacle')]
    return dict(actors=len(actors),physical_obstacles=len(obstacles),
        obstacle_near20=sum(a['planned_center_distance_m']<20 for a in obstacles),
        obstacle_mid20_30=sum(20<=a['planned_center_distance_m']<30 for a in obstacles),
        obstacle_far30=sum(a['planned_center_distance_m']>=30 for a in obstacles),
        redistributed=sum(bool(a.get('secondary_obstacle_support')) and
                          not a['secondary_obstacle_support'].get('protected_resample') for a in actors))


def verify_protected_near(row, ordinal, actual):
    plan=row.get('secondary_obstacle_support',{})
    if ordinal in plan.get('protected_near_ordinals',[]) and actual>=20:
        raise ValueError('Protected near secondary escaped under placement retry')
    from base5_obstacle_rich_floor import verify_mid
    verify_mid(row,ordinal,actual)
