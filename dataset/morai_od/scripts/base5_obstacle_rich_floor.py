"""Preserve existing near/mid support and add a bounded generic obstacle floor.

This is a new, explicit population contract; no old manifest is reinterpreted.
One extra actor is allowed only in selected static generic mixtures with space
below the original eight-actor limit. Primary/missions/other assets remain.
"""
import copy
import hashlib
import random

POLICY='generic-obstacle-rich-floor-v145'


def extra_selected(row,count):
    if count>=row.get('population_max_actors',8):return False
    value=int(hashlib.sha256(f"extra-obstacle:{row['seed']}".encode()).hexdigest()[:16],16)/2**64
    return value<.6


def attach(row,actors,extra=None,charge=0):
    from base5_obstacle_support_policy import eligible
    result=copy.deepcopy(row)
    if not eligible(row) or any(a['motion'] not in ('static','standing') for a in actors):return result
    physical=[i for i,a in enumerate(actors) if i and a['population_pool'].startswith('obstacle')
              and a['source_list']=='objectList']
    far=[i for i in physical if actors[i]['planned_center_distance_m']>=30]
    extra=extra if extra is not None and extra_selected(row,len(actors)) else None
    if not far and extra is None:return result
    ids=[a['data_id'] for a in actors]
    added=[]
    if extra is not None:
        if extra['source_list']!='objectList' or extra.get('semantic_class')!='obstacle' or not 2<=charge<=6:
            raise ValueError('Rich floor extra must be a physical obstacle with bounded size cost')
        added=[len(ids)];ids.append(extra['data_id'])
        lo,hi=row['population_budget_bounds']
        result['population_rng_reference']=dict(actors=len(actors),population_budget_bounds=[lo,hi])
        result['population_budget_bounds']=[lo+charge,hi+charge]
        result['population_support_delta']=dict(added_data_id=extra['data_id'],added_cost=charge,
            original_actor_count=len(actors),maximum_actor_count=row['population_max_actors'],
            original_bounds=[lo,hi],new_bounds=result['population_budget_bounds'],scope='one extra generic obstacle; not mission clutter')
    result['frozen_population_data_ids']=ids;result['actors']=len(ids)
    result['secondary_obstacle_support']=dict(policy=POLICY,variant='richfloor',ordinals=far+added,
        new_actor_ordinals=added,protected_near_ordinals=[i for i in physical if actors[i]['planned_center_distance_m']<20],
        protected_mid_ordinals=[i for i in physical if 20<=actors[i]['planned_center_distance_m']<30],
        primary_preserved=True,mission_and_moving_excluded=True,far_to_near_fraction=.7)
    return result


def sample(row,ordinal,asset,size,role,attempt):
    from base5_obstacle_support_policy import eligible
    plan=row['secondary_obstacle_support']
    if plan['policy']!=POLICY or not eligible(row) or 0 in plan['ordinals']:
        raise ValueError('Invalid rich floor scope or primary injection')
    protected_near=ordinal in plan.get('protected_near_ordinals',[])
    protected_mid=ordinal in plan.get('protected_mid_ordinals',[])
    protected=protected_near or protected_mid
    if ordinal not in plan['ordinals'] and not (protected and attempt>plan.get('original_generation_attempt',0)):
        return None
    if asset['source_list']!='objectList' or asset.get('semantic_class')!='obstacle':
        raise ValueError('Rich floor restricted to physical obstacles')
    key=f"{POLICY}:{row['seed']}:{ordinal}:{asset['data_id']}"
    rank=int(hashlib.sha256(key.encode()).hexdigest()[:16],16)/2**64
    rng=random.Random(int(hashlib.sha256((key+':'+str(attempt)).encode()).hexdigest()[:16],16))
    added=ordinal in plan['new_actor_ordinals']
    near=protected_near if protected else added or rank<.7
    if added:
        role=('front','left','rear','right')[int(hashlib.sha256(key.encode()).hexdigest()[16:18],16)%4]
    large=max(size[:2])>3 or size[0]*size[1]>3
    bounds=[8.,20.] if near and large else [6.,18.] if near else [20.,29.]
    if added and not large and size[2]<.35:bounds=[6.,10.]
    elif added and not large and size[2]<.75:bounds=[6.,14.]
    return dict(policy=POLICY,variant='richfloor',ordinal=ordinal,role=role,
        band='near' if near else 'mid',distance_bounds_m=bounds,
        target_center_distance_m=rng.uniform(bounds[0]+.1,bounds[1]-.1),
        size_aware_large=large,primary_changed=False,count_changed=added,
        added_actor=added,protected_resample=protected,band_fixed_across_retry=True)


def verify_mid(row,ordinal,actual):
    if ordinal in row.get('secondary_obstacle_support',{}).get('protected_mid_ordinals',[]) and not 20<=actual<30:
        raise ValueError('Protected mid-distance obstacle escaped under placement retry')
