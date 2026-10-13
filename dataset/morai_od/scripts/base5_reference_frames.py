"""Road semantics and actual Ego observation frames; never rotates raw sensor GT."""
import math


def project_xy(center,ego,yaw_deg):
    a=math.radians(yaw_deg);dx,dy=center[0]-ego['x'],center[1]-ego['y']
    return math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy


def role_xy(x,y):
    return ('front' if x>=0 else 'rear') if abs(x)>=abs(y) else ('left' if y>=0 else 'right')


def references(center,ego,road_yaw_deg):
    x,y=project_xy(center,ego,ego['yaw']);rx,ry=project_xy(center,ego,road_yaw_deg)
    return dict(road_longitudinal_m=rx,road_lateral_m=ry,road_role=role_xy(rx,ry),
        ego_longitudinal_m=x,ego_lateral_m=y,ego_role=role_xy(x,y),
        reference_road_yaw_deg=road_yaw_deg,reference_ego_yaw_deg=ego['yaw'])


def center_in_roi(center,roi=(-100,-30,-3,100,30,2)):
    return all(roi[i]<=center[i]<=roi[i+3] for i in range(3))


def center_in_bev(center,roi=(-100,-30,-3,100,30,2)):
    """MMDet3D ObjectRangeFilter uses strict XY center bounds, not box corners/Z."""
    return roi[0]<center[0]<roi[3] and roi[1]<center[1]<roi[4]


def yaw_runtime_review(meta,before,after=None):
    """Position-only frozen-world gate; no assumption about velocity units."""
    reasons=[]
    def actors(message):
        return {a['unique_id']:a['position'] for key in ('npc_list','pedestrian_list','obstacle_list')
            for a in (message or {}).get(key,[])}
    b=actors(before);a=actors(after) if after is not None else b
    for actor in meta['runtime_actors']:
        uid=actor['unique_id'];expected=actor['position']
        if uid not in b or uid not in a:
            reasons.append('yaw_probe_missing_actor');continue
        if max(math.hypot(p['x']-expected['x'],p['y']-expected['y']) for p in (b[uid],a[uid]))>.25:
            reasons.append('yaw_probe_actor_not_at_fixed_pose')
        if math.hypot(a[uid]['x']-b[uid]['x'],a[uid]['y']-b[uid]['y'])>.10:
            reasons.append('yaw_probe_actor_moved')
    return sorted(set(reasons))
