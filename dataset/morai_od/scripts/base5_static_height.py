"""Conservative F2 saved-pivot initializer; not a verified ground calibration."""
import numpy as np


def saved_height_candidate(asset, ctx):
    key=(asset.get('source_file'),asset['source_list'],asset['data_id'])
    cache=ctx.setdefault('static_height_candidates',{})
    if key in cache:return cache[key]
    template=asset['template']
    position=(template.get('initPosition',{}).get('pos') if asset['source_list']=='vehicleList'
        else template.get('pos'))
    result=dict(usable=False,policy='saved-pivot-near-map-initializer-v1',
        data_id=asset['data_id'],source_file=asset.get('source_file'),
        source_scenario_sha256=ctx.get('static_height_reference_sources',{}).get('scenarios',{}).get(asset.get('source_file')),
        source_map_link_sha256=ctx.get('static_height_reference_sources',{}).get('map_link_sha256'),
        limitation='F2 saved pivot minus nearby MGeo height is an initializer, not measured mesh contact')
    if not position or not all(k in position for k in ('x','y','z')):
        result['reason']='saved_position_missing';cache[key]=result;return result
    points=ctx.get('static_height_map_points')
    if points is None:
        points=np.asarray([p for link in ctx['links'].values() for p in link.get('points',[])],float)
        ctx['static_height_map_points']=points
    if points.size==0:result['reason']='map_points_missing'
    else:
        distance=np.linalg.norm(points[:,:2]-[position['x'],position['y']],axis=1)
        nearest=int(np.argmin(distance));near=points[distance<=1.]
        result.update(saved_position=dict(position),nearest_map_xy_distance_m=float(distance[nearest]))
        if distance[nearest]>1.:result['reason']='saved_pivot_not_near_mapped_road'
        elif float(np.ptp(near[:,2]))>.10:result['reason']='source_map_height_ambiguous'
        else:
            offset=float(position['z']-points[nearest,2])
            result.update(source_map_z_m=float(points[nearest,2]),pivot_height_candidate_m=offset)
            if not -.5<=offset<=1.:result['reason']='saved_pivot_height_out_of_bounds'
            else:result.update(usable=True,reason=None)
    cache[key]=result
    return result
