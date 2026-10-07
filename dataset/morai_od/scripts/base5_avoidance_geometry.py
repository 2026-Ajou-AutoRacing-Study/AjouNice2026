"""Independent stationary observations of a virtual bypass, not a controller path."""
import math


def smoothstep(t):
    t=max(0.,min(1.,t))
    return t*t*(3.-2.*t),6.*t*(1.-t)


def road_supported_pair(ego,target,support,surface):
    """Rigidly translate a synthetic pair, never snap Ego alone.

    Preserve the view/footprint gap. Place Ego's geometric center over the road
    center while retaining its tangent yaw; unsupported yaw still retries.
    This is not a driven-path or object-ground-contact certificate.
    """
    from base5_map_surface import ego_road_support
    before=ego_road_support(surface,ego)
    if before['road_supported']:return ego,target,None
    a=math.radians(ego['yaw']);road=math.radians(support['yaw'])
    fixed=dict(ego,x=support['x']+1.35*(math.cos(road)-math.cos(a)),
        y=support['y']+1.35*(math.sin(road)-math.sin(a)),z=support['z'])
    after=ego_road_support(surface,fixed)
    if not after['road_supported']:
        raise ValueError('Final Ego footprint outside MGeo road support')
    dx,dy=fixed['x']-ego['x'],fixed['y']-ego['y']
    moved=dict(target,x=target['x']+dx,y=target['y']+dy)
    proof=dict(policy='stationary-road-supported-pair-v114',translation_xy_m=[dx,dy],
        original_ego=ego,original_target=target,road_anchor=support,
        before=before,after=after,relative_xy_and_yaw_preserved=True,
        object_ground_contact_verified=False,virtual_path_driven=False)
    return fixed,moved,proof


def observation(gen,ctx,route,arc,base,asset,row,rng):
    size,off,known=gen.geometry(asset,ctx)
    road_yaw=base['yaw']
    actor_yaw=road_yaw
    if row['family']=='obstacle':
        actor_yaw+=rng.choice([0.,90.,-90.])+rng.uniform(-8.,8.)
    angle=math.radians(actor_yaw)
    center=[base['x']+math.cos(angle)*off[0]-math.sin(angle)*off[1],
            base['y']+math.sin(angle)*off[0]+math.cos(angle)*off[1]]
    road=math.radians(road_yaw);c,s=math.cos(road),math.sin(road)
    delta=math.radians(actor_yaw-road_yaw)
    projected_length=size[0]*abs(math.cos(delta))+size[1]*abs(math.sin(delta))
    projected_width=size[0]*abs(math.sin(delta))+size[1]*abs(math.cos(delta))
    halfspan=(projected_length+4.8)/2
    phase=row['avoidance_phase'];sign=1 if row['avoidance_side']=='left' else -1
    low,high=row['avoidance_gap_bounds_m']
    requested_gap=rng.uniform(low+.003,high-.003)
    # A generic 1x1m placeholder is not a collider/mesh measurement. Keep the
    # same asset and phase, but do not synthesize centimetre passes from it.
    gap=max(requested_gap,.30 if known else 1.0)
    if phase=='approach':
        longitudinal=-rng.uniform(halfspan+8.,halfspan+60.);lateral=0.;yaw=road_yaw
    else:
        if phase=='entry': longitudinal=-halfspan-rng.uniform(1.,8.)
        elif phase=='alongside':
            # Ego reference is rear axle, not its geometric center. Keep the
            # object genuinely sideward of that reference even for long trucks.
            longitudinal=1.35+rng.uniform(-min(.7,projected_length*.35),min(.7,projected_length*.35))
        else: longitudinal=halfspan+rng.uniform(.2,6.)
        # Smooth lateral transition: entry/clearing are tangent views, not random yaw.
        if phase=='entry': fraction,derivative=smoothstep((longitudinal+halfspan+14.)/14.)
        elif phase=='clearing':
            fraction,d=smoothstep((halfspan+14.-longitudinal)/14.);derivative=-d
        else: fraction,derivative=1.,0.
        peak=(projected_width+2.)/2+gap
        yaw=road_yaw+math.degrees(math.atan(sign*peak*derivative/14.))
        # Solve requested footprint gap with the actual oriented rectangles.
        # This is a static observation near a virtual path, not a driven-safe path certificate.
        actor_poly=gen.rectangle(center,size,actor_yaw)
        def distance(lat):
            return gen.rectangle([center[0]+c*longitudinal-s*sign*lat,
                                  center[1]+s*longitudinal+c*sign*lat],[4.8,2.],yaw).distance(actor_poly)
        if phase=='alongside':
            low,high=0.,peak+3.
            for _ in range(32):
                mid=(low+high)/2
                if distance(mid)<gap:low=mid
                else:high=mid
            lateral=sign*(low+high)/2
        else:lateral=sign*peak*fraction
    target_arc=base['route_arc_m']+longitudinal
    if not 0<=target_arc<=arc[-1]:raise ValueError('Avoidance Ego beyond reference route')
    support=gen.geo.continuous_route_pose_at_arc(route,arc,target_arc)
    yaw_rad=math.radians(yaw)
    ego=dict(support,x=center[0]+c*longitudinal-s*lateral-1.35*math.cos(yaw_rad),
             y=center[1]+s*longitudinal+c*lateral-1.35*math.sin(yaw_rad),yaw=yaw)
    target_pose=dict(base,yaw=actor_yaw)
    if row.get('avoidance_origin')=='generic_close':
        # Generic scenes are not virtual driven bypasses. Keep Ego on the road
        # anchor and translate the object instead of putting Ego on a curb/verge.
        dx,dy=base['x']-ego['x'],base['y']-ego['y']
        ego=dict(base,yaw=yaw)
        target_pose=dict(target_pose,x=target_pose['x']+dx,y=target_pose['y']+dy)
    road_relocation=None
    if row.get('mission') and ctx.get('map_surface_index') is not None:
        ego,target_pose,road_relocation=road_supported_pair(ego,target_pose,support,ctx['map_surface_index'])
    reference=dict(phase=phase,side=row['avoidance_side'],
        requested_surface_gap_m=gap if phase=='alongside' else None,
        longitudinal_center_offset_m=longitudinal,lateral_center_offset_m=lateral,
        yaw_relative_to_object_road_deg=yaw-road_yaw,
        geometry_reference_known=known,ground_support_verified=False,
        original_gap_request_m=requested_gap,
        clearance_policy='measured-geometry-min30cm; unknown-geometry-min1m-v46',
        collider_clearance_verified=False,virtual_path_driven=False,
        contract='stationary-bypass-observation-v1')
    if row.get('avoidance_origin')=='generic_close':
        reference.update(contract='stationary-road-ego-side-object-v97',ego_road_anchor_preserved=True,
            object_ground_support_verified=False)
    if road_relocation:
        reference.update(contract='stationary-road-supported-pair-v114',road_relocation=road_relocation)
    return ego,target_pose,reference
