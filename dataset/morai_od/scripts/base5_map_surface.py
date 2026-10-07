"""Indexed MGeo initial-height estimate, not a ground-contact certificate.

Project on original road segments; only associated lane boundaries supply lateral
height. No global triangulation, invented terrain samples or annotation edits.
"""
import math
import numpy as np
from scipy.spatial import cKDTree


class MapSurface:
    def __init__(self,links,boundaries=None):
        self.links=links;self.boundaries=boundaries or {}
        start=[];end=[];owners=[];point_indices=[];samples=[];sample_owners=[]
        for key,link in sorted(links.items()):
            # Lane-change connectors are routing shortcuts, not surveyed road
            # surfaces (a long diagonal can cross entirely different levels).
            if link.get('lazy_init') or link.get('lane_ch_link_path'):continue
            points=np.asarray(link.get('points',[]),float)
            if len(points)<2:continue
            for j,(a,b) in enumerate(zip(points[:-1],points[1:])):
                length=np.linalg.norm(b[:2]-a[:2])
                if not np.isfinite(length) or length<1e-6:continue
                index=len(start);start.append(a);end.append(b);owners.append(key);point_indices.append(j)
                # Sampling is a search index only; projection still uses the
                # original segment's XYZ, not interpolated pseudo evidence.
                n=max(1,int(math.ceil(length/2.)))
                for t in np.linspace(0,1,n+1):
                    samples.append(a[:2]+t*(b[:2]-a[:2]));sample_owners.append(index)
        self.start=np.asarray(start,float);self.end=np.asarray(end,float)
        self.owners=owners;self.point_indices=point_indices
        self.sample_owners=np.asarray(sample_owners,int)
        self.tree=cKDTree(np.asarray(samples)) if samples else None
        self.boundary_segments={}
        for key,row in self.boundaries.items():
            points=np.asarray(row.get('points',[]),float)
            if len(points)>=2:self.boundary_segments[key]=(points[:-1],points[1:])

    @staticmethod
    def project(xy,start,end):
        direction=end[:,:2]-start[:,:2];norm=np.sum(direction*direction,axis=1)
        t=np.clip(np.sum((xy-start[:,:2])*direction,axis=1)/np.maximum(norm,1e-12),0,1)
        p=start+t[:,None]*(end-start)
        distance=np.linalg.norm(p[:,:2]-xy,axis=1)
        return p,distance,t

    def estimate(self,xy,preferred_link=None,maximum_distance=8.):
        xy=np.asarray(xy,float)
        result=dict(policy='mgeo-segment-associated-boundary-v1',usable=False,confidence='unsupported',
            query_xy_m=xy.tolist(),contact_verified=False)
        if xy.shape!=(2,) or not np.isfinite(xy).all() or self.tree is None:
            return dict(result,reason='invalid_query_or_empty_map')
        _,near=self.tree.query(xy,k=min(32,len(self.sample_owners)))
        indices=np.unique(self.sample_owners[np.atleast_1d(near)])
        projected,distance,t=self.project(xy,self.start[indices],self.end[indices])
        best=int(np.argmin(distance));minimum=float(distance[best])
        result.update(nearest_road_distance_m=minimum)
        if minimum>maximum_distance:return dict(result,reason='target_outside_map_height_support')
        close=np.where(distance<=minimum+.35)[0]
        if len(close)>1 and np.ptp(projected[close,2])>.25:
            return dict(result,reason='ambiguous_road_levels')
        preferred=[i for i in close if self.owners[indices[i]]==preferred_link]
        if preferred:best=min(preferred,key=lambda i:distance[i])
        idx=int(indices[best]);p=projected[best];owner=self.owners[idx]
        direction=self.end[idx,:2]-self.start[idx,:2];direction/=np.linalg.norm(direction)
        normal=np.array([-direction[1],direction[0]])
        lateral=float(np.dot(xy-p[:2],normal));longitudinal=float(np.dot(xy-p[:2],direction))
        z=float(p[2]);boundary_proof=None
        link=self.links[owner]
        # Topologically associated boundaries only. Far longitudinal endpoints,
        # incompatible levels or crossing-direction lines are not a road patch.
        candidates=[]
        for key in sorted(set(link.get('lane_mark_left',[])+link.get('lane_mark_right',[]))):
            if key not in self.boundary_segments:continue
            a,b=self.boundary_segments[key];bp,bd,bt=self.project(p[:2],a,b)
            j=int(np.argmin(bd));vec=b[j,:2]-a[j,:2];length=np.linalg.norm(vec)
            if length<1e-6 or abs(np.dot(vec/length,direction))<.8:continue
            offset=bp[j,:2]-p[:2];lat=float(np.dot(offset,normal));lon=float(np.dot(offset,direction))
            if abs(lon)>.75 or not .3<=abs(lat)<=6. or abs(bp[j,2]-z)>.35:continue
            if lat*lateral>0 and abs(lateral)<=abs(lat)+.35:
                candidates.append((abs(lat),key,j,lat,bp[j],float(bt[j])))
        if candidates:
            _,key,j,lat,bp,ratio=min(candidates,key=lambda row:row[0])
            alpha=lateral/lat
            z=float(z+alpha*(bp[2]-z))
            boundary_proof=dict(boundary_id=key,segment_index=j,segment_ratio=ratio,
                boundary_projection_xyz_m=bp.tolist(),lateral_ratio=alpha)
        width=max(float(link.get('width_start',3.5)),float(link.get('width_end',3.5)))
        confidence=('road_strip' if boundary_proof and 0<=boundary_proof['lateral_ratio']<=1 else
            'near_centerline' if abs(lateral)<=min(width/2,1.75) and abs(longitudinal)<.5 else 'roadside_estimate')
        result.update(usable=True,confidence=confidence,reason=None,height_m=z,
            link_id=owner,segment_index=self.point_indices[idx],segment_ratio=float(t[best]),
            road_projection_xyz_m=p.tolist(),lateral_distance_m=lateral,boundary=boundary_proof,
            limitation='Road/associated-line heights only; curb, sidewalk, off-road mesh and collider are not certified')
        return result


def configure(ctx,policy):
    """Share index/cache across builds, retain legacy behavior without opt-in."""
    if not policy or not policy.get('enabled'):
        ctx.pop('placement_ground_policy',None);return ctx
    if 'map_surface_index' not in ctx:
        ctx['map_surface_index']=MapSurface(ctx['links'],ctx.get('lane_boundaries'))
    ctx.setdefault('static_height_candidates',{})
    ctx['placement_ground_policy']=policy
    return ctx


def footprint_profile(surface,pose,size,offset):
    a=math.radians(pose['yaw']);c,s=math.cos(a),math.sin(a)
    points=[]
    for x,y in ((offset[0],offset[1]),(offset[0]-size[0]/2,offset[1]-size[1]/2),
            (offset[0]-size[0]/2,offset[1]+size[1]/2),(offset[0]+size[0]/2,offset[1]-size[1]/2),
            (offset[0]+size[0]/2,offset[1]+size[1]/2)):
        points.append(surface.estimate([pose['x']+c*x-s*y,pose['y']+s*x+c*y],pose.get('link_idx')))
    supported=[p['height_m'] for p in points if p['usable']]
    return dict(samples=points,all_supported=len(supported)==len(points),
        map_height_span_m=float(max(supported)-min(supported)) if supported else None,
        contact_verified=False,limitation='Map footprint profile, not measured mesh contact; actor yaw/pitch and GT unchanged')


def initial_pose(pose,asset,size,offset,ctx):
    """Final XY lookup then asset pivot; no blanket positive Z margin."""
    from base5_static_height import saved_height_candidate
    result=dict(pose);ground=None;legacy=.36 if asset['source_list']=='vehicleList' else 0.
    plan=ctx.get('placement_ground_policy')
    if plan and plan.get('enabled'):
        surface=ctx['map_surface_index']
        # Runtime scene rotations are three-decimal degree strings. Profile the
        # actual serialized yaw so offline source checks reproduce the proof.
        profile_pose=dict(pose,yaw=float(f"{pose['yaw']:.3f}"))
        ground=surface.estimate([pose['x'],pose['y']],pose.get('link_idx'),plan.get('maximum_road_distance_m',8.))
        ground.update(previous_reference_z_m=float(pose['z']),
            preferred_link=pose.get('link_idx'),
            source_map_link_sha256=ctx['static_height_reference_sources'].get('map_link_sha256'),
            source_map_boundary_sha256=ctx['static_height_reference_sources'].get('map_boundary_sha256'),
            footprint=footprint_profile(surface,profile_pose,size,offset))
        if ground['usable']:result['z']=ground['height_m']
    initializer=saved_height_candidate(asset,ctx)
    if initializer['usable']:result['z']+=initializer['pivot_height_candidate_m']-legacy
    if ground:
        ground['input_z_delta_m']=float(result['z']-pose['z'])
        ground['asset_offset_usable']=initializer['usable']
        ground['literal_contact_verified']=False
    return result,initializer,ground
