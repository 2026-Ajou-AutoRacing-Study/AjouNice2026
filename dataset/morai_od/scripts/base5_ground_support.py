"""Local LiDAR ground-plane witness; no raw/GT edits, no collider certification."""
import numpy as np


SURFACE_ASSOCIATION_POLICY = 'far-static-sparse-multilevel-surface-association-v157'


def plane_observation(patch, plane, corners):
    """Retain support-level evidence without changing the fitted plane or gap."""
    patch, corners = np.asarray(patch, float), np.asarray(corners, float)
    coef = np.asarray(plane['coefficients'], float)
    residual = patch[:, 2] - np.c_[patch[:, :2], np.ones(len(patch))] @ coef
    return dict(patch_point_count=len(patch), actor_center_ego_xy_m=corners[:, :2].mean(0).tolist(),
        native_parent_component_count=len(corners)//8 if len(corners) % 8 == 0 else None,
        native_lowest_corner_ego_z_m=float(corners[:, 2].min()),
        patch_ego_z_range_m=[float(patch[:, 2].min()), float(patch[:, 2].max())],
        lower_by_one_m_count=int((residual <= -1.).sum()),
        minimum_signed_plane_residual_m=float(residual.min()),
        limitation='Raw surrounding returns after Ego/actor footprint exclusion; lower returns do not certify a road plane.')


def static_surface_association(row, ground, metadata):
    """A narrow uncertainty classification, never replacement ground/contact GT.

    Only a far, sparse, upward competing level can be ambiguous here. Genuine
    above-ground gaps and below-ground gaps on a road-associated plane retain
    their old severe result. Missing/untrusted inputs never grant a waiver.
    """
    from base5_static_ground_policy import lightweight_static
    if (not metadata or not lightweight_static(metadata)
            or metadata.get('ground_probe_references') or metadata.get('static_ground_references')
            or row.get('reason') != 'actor_annotation_deep_below_ground'
            or row.get('reference') or row.get('used_independent_reference')):
        return None
    try:
        finite = lambda x: type(x) in (int, float) and np.isfinite(x)
        observation, plane = row['surface_observation'], row['plane']
        xy = np.asarray(observation['actor_center_ego_xy_m'], float)
        coef = np.asarray(plane['coefficients'], float)
        maximum = ground['max_range_m']
        count, lower = observation['patch_point_count'], observation['lower_by_one_m_count']
        if (xy.shape != (2,) or coef.shape != (3,) or not np.isfinite(np.r_[xy, coef]).all()
                or type(observation.get('native_parent_component_count')) is not int
                or observation['native_parent_component_count'] != 1
                or not finite(maximum) or maximum <= 0 or np.linalg.norm(xy) <= maximum
                or type(count) is not int or not 18 <= count <= 64
                or type(lower) is not int or not 2 <= lower <= count
                or row.get('verified') is not False
                or not finite(row.get('minimum_annotation_ground_gap_m'))
                or row['minimum_annotation_ground_gap_m'] >= -.40
                or not finite(observation['minimum_signed_plane_residual_m'])
                or observation['minimum_signed_plane_residual_m'] > -1.
                or type(plane.get('inliers')) is not int or not 18 <= plane['inliers'] <= count
                or not finite(plane.get('residual_rms_m')) or not 0 <= plane['residual_rms_m'] <= .025):
            return None
        matches = [actor for actor in metadata['runtime_actors'] if actor.get('unique_id') == row['uid']]
        if len(matches) != 1:
            return None
        actor = matches[0]
        if (actor.get('source_list') != 'objectList' or actor.get('is_compound') or actor.get('compound')
                or actor.get('component_count', 1) != 1 or actor.get('annotation_component_count', 1) != 1):
            return None
        road = actor['map_ground_initializer'];footprint = road['footprint']
        height, planned_z = road['height_m'], actor['position']['z']
        if (road.get('policy') != 'mgeo-segment-associated-boundary-v1'
                or road.get('usable') is not True or road.get('confidence') != 'road_strip'
                or road.get('reason') is not None or not finite(height) or not finite(planned_z)
                or not finite(road['nearest_road_distance_m']) or not 0 <= road['nearest_road_distance_m'] <= 2.
                or abs(planned_z-height) > .25 or footprint.get('all_supported') is not True
                or not finite(footprint['map_height_span_m']) or not 0 <= footprint['map_height_span_m'] <= .15
                or not isinstance(footprint.get('samples'), list) or len(footprint['samples']) < 3
                or any(sample.get('usable') is not True or sample.get('confidence') != 'road_strip'
                    or not finite(sample.get('height_m')) or abs(sample['height_m']-height) > .15
                    for sample in footprint['samples'])
                or any(not isinstance(road.get(key), str) or len(road[key]) != 64
                    or any(c not in '0123456789abcdef' for c in road[key])
                    for key in ('source_map_link_sha256', 'source_map_boundary_sha256'))):
            return None
        ego = metadata['capture_runtime_ego']
        if not all(finite(ego.get(key)) for key in ('x', 'y', 'z', 'heading')):
            return None
        planned_xy = np.asarray([actor['position'][key] for key in ('x', 'y')], float)
        query_xy = np.asarray(road['query_xy_m'], float)
        expected_center = np.asarray(actor['geometric_center_estimate'][:2], float)
        from base5_ground_probe import ego_to_world
        actual_center = ego_to_world([np.r_[xy, 0.]], ego)[0, :2]
        if (query_xy.shape != (2,) or expected_center.shape != (2,)
                or not np.isfinite(np.r_[planned_xy, query_xy, expected_center, actual_center]).all()
                or np.linalg.norm(query_xy-planned_xy) > .05
                or np.linalg.norm(actual_center-expected_center) > .50):
            return None
        low = observation['native_lowest_corner_ego_z_m']
        if not finite(low):
            return None
        # Z is unchanged by the existing Ego->world yaw transform. MGeo is
        # corroborating level evidence only; never promote it to mesh contact.
        fitted_world_z = float(np.r_[xy, 1.] @ coef + ego['z'])
        native_world_low = low + ego['z']
        if fitted_world_z-height < 2. or abs(native_world_low-height) > .30:
            return None
        return dict(policy=SURFACE_ASSOCIATION_POLICY, status='unverified_multilevel_surface_association',
            original_reason=row['reason'], original_gap_m=row['minimum_annotation_ground_gap_m'],
            actor_range_ego_xy_m=float(np.linalg.norm(xy)), max_local_probe_range_m=maximum,
            fitted_surface_world_z_m=fitted_world_z, road_initializer_world_z_m=height,
            fitted_minus_road_level_m=fitted_world_z-height,
            native_lowest_corner_minus_road_level_m=native_world_low-height,
            native_to_planned_center_xy_delta_m=float(np.linalg.norm(actual_center-expected_center)),
            patch_point_count=count, lower_by_one_m_count=lower,
            native_parent_component_count=observation['native_parent_component_count'],
            map_source_sha256={key:road[key] for key in ('source_map_link_sha256','source_map_boundary_sha256')},
            raw_plane_gap_and_native_preserved=True, mesh_ground_or_contact_certified=False,
            limitation='Far sparse competing surface level is not a verified supporting road; original measurement remains evidence.')
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def height_response_review(ground, metadata):
    """Detect measured input-Z changes that did not fix the observed height gap."""
    changes={}
    for row in metadata.get('ground_retry_provenance',{}).get('measured_corrections',[]):
        changes[row['uid']]=(row.get('minimum_annotation_ground_gap_m'),row.get('suggested_z_shift_m'))
    for row in metadata.get('probed_retry_provenance',{}).get('input_z_corrections',[]):
        changes[row['uid']]=(row.get('before_ground_gap_m'),row.get('delta_z_m'))
    results=[]
    for row in ground.get('actors',[]):
        before,delta=changes.get(row['uid'],(None,None));after=row.get('minimum_annotation_ground_gap_m')
        if before is None or delta is None or after is None:continue
        ineffective=(delta<-.10 and after>ground['maximum_gap_m'] and before-after<abs(delta)*.2)
        results.append(dict(uid=row['uid'],before_gap_m=before,after_gap_m=after,
            requested_delta_z_m=delta,observed_gap_reduction_m=before-after,
            ineffective=ineffective,limitation='Observed annotation/terrain response; not proof of a simulator bug'))
    return dict(actors=results,review_reasons=['actor_input_z_correction_ineffective']
        if any(r['ineffective'] for r in results) else [])


def fit_plane(points,minimum_points=18):
    points=np.asarray(points,float)
    if len(points)<minimum_points:return None
    # Prefer low returns; exclude meshes before calling. Curbs/multiple levels can still fail.
    low=points[points[:,2]<=np.quantile(points[:,2],.55)+.04]
    if len(low)<minimum_points:return None
    xy=low[:,:2].mean(axis=0);p=low.copy();p[:,:2]-=xy
    rng=np.random.RandomState(261006);best=None
    for _ in range(96):
        sample=p[rng.choice(len(p),3,replace=False)];A=np.c_[sample[:,:2],np.ones(3)]
        if abs(np.linalg.det(A))<1e-6:continue
        coef=np.linalg.solve(A,sample[:,2])
        if np.linalg.norm(coef[:2])>.25:continue
        mask=abs(np.c_[p[:,:2],np.ones(len(p))]@coef-p[:,2])<.035
        if best is None or mask.sum()>best.sum():best=mask
    if best is None or best.sum()<minimum_points or best.mean()<.60:return None
    inliers=p[best]
    if np.min(np.linalg.eigvalsh(np.cov(inliers[:,:2].T)))<.04:return None
    coef=np.linalg.lstsq(np.c_[inliers[:,:2],np.ones(len(inliers))],inliers[:,2],rcond=None)[0]
    residual=inliers[:,2]-np.c_[inliers[:,:2],np.ones(len(inliers))]@coef
    if np.linalg.norm(coef[:2])>.25 or np.sqrt(np.mean(residual**2))>.025:return None
    coef[2]-=coef[:2]@xy
    from shapely.geometry import MultiPoint
    hull=MultiPoint(inliers[:,:2]+xy).convex_hull
    return dict(coefficients=coef.tolist(),inliers=int(best.sum()),candidates=len(low),
                residual_rms_m=float(np.sqrt(np.mean(residual**2))),support_polygon_xy=list(hull.exterior.coords))


def ground_review(points,boxes,transform,max_range=20.,maximum_gap=.15,references=None,runtime_ego=None,required_uids=None):
    from base5_capture_audit import rotation_matrix
    points=np.asarray(points,float);transform=np.asarray(transform,float)
    # MORAI synchronous scans contain zero-XYZ no-return placeholders. They
    # become thousands of identical elevated Ego-frame points after extrinsics.
    # Keep raw files intact, but never use those placeholders as ground evidence.
    points=points[np.isfinite(points).all(axis=1)&(points!=0).any(axis=1)]
    ego_points=(np.c_[points,np.ones(len(points))]@transform.T)[:,:3]
    signs=np.asarray([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    # Near patches can include returns from Ego itself. A very flat bonnet/roof
    # is not road ground even if RANSAC residual and XY support look excellent.
    outside=~((ego_points[:,0]>=-1.3)&(ego_points[:,0]<=4.)&(abs(ego_points[:,1])<=1.3))
    corners=[]
    for b in boxes:
        local=(points-b['center'])@rotation_matrix(b['rotation_rpy'])
        outside&=~(abs(local[:,:2])<=b['size'][:2]/2+.20).all(axis=1)
        c=signs*b['size']/2@rotation_matrix(b['rotation_rpy']).T+b['center']
        c=(np.c_[c,np.ones(8)]@transform.T)[:,:3];corners.append((b['unique_id'],c))
    results=[];reasons=[]
    for uid in sorted({u for u,_ in corners}):
        cs=np.concatenate([c for u,c in corners if u==uid]);xy=cs[:,:2].mean(axis=0)
        if np.linalg.norm(xy)>max_range and uid not in (required_uids or []):
            results.append(dict(uid=uid,required=False,verified=False,reason='outside_local_ground_probe_range'));continue
        radius=max(5.,float(np.linalg.norm(cs[:,:2]-xy,axis=1).max())+2.)
        plane=None
        from shapely.geometry import Point,Polygon
        for patch_radius in (radius,max(radius,8.)):
            patch=ego_points[outside & (np.linalg.norm(ego_points[:,:2]-xy,axis=1)<patch_radius)]
            candidate=fit_plane(patch)
            # Refuse long extrapolation from a separate patch/level.
            if candidate is not None and all(Polygon(candidate['support_polygon_xy']).distance(Point(p))<=.75 for p in cs[:,:2]):
                plane=candidate;break
        row=dict(uid=uid,required=True,verified=False,plane=plane,patch_radius_m=patch_radius)
        if plane is not None:
            row['surface_observation'] = plane_observation(patch, plane, cs)
        reference=next((r for r in (references or []) if r['uid']==uid),None)
        if reference and runtime_ego:
            from base5_ground_probe import ego_to_world,plane_height,supported
            world=ego_to_world(cs,runtime_ego);independent=reference['plane_world']
            if all(supported(independent,c[:2]) for c in world):
                external_gap=world[:,2]-np.array([plane_height(independent,c[:2]) for c in world])
                if plane:
                    a,b,c=plane['coefficients'];local_gap=cs[:,2]-(a*cs[:,0]+b*cs[:,1]+c)
                    if np.max(abs(external_gap-local_gap))>.10:
                        row.update(reason='actor_ground_reference_disagreement',reference=reference,
                            maximum_reference_disagreement_m=float(np.max(abs(external_gap-local_gap))))
                        results.append(row);reasons.append(row['reason']);continue
                gap=float(external_gap.min());row.update(reference=reference,used_independent_reference=True,
                    minimum_annotation_ground_gap_m=gap,suggested_z_shift_m=-gap-.03 if gap>maximum_gap else 0.)
                if gap>maximum_gap:row['reason']='actor_annotation_above_ground';reasons.append(row['reason'])
                elif gap<-.30:row['reason']='actor_annotation_deep_below_ground';reasons.append(row['reason'])
                else:row.update(verified=True,reason=None)
                results.append(row);continue
        if plane is None:row['reason']='actor_ground_plane_unverified';reasons.append(row['reason'])
        else:
            a,b,c=plane['coefficients'];clearance=cs[:,2]-(a*cs[:,0]+b*cs[:,1]+c)
            gap=float(clearance.min());row.update(minimum_annotation_ground_gap_m=gap,
                suggested_z_shift_m=-gap-.03 if gap>maximum_gap else 0.)
            if gap>maximum_gap:row['reason']='actor_annotation_above_ground';reasons.append(row['reason'])
            elif gap<-.30:row['reason']='actor_annotation_deep_below_ground';reasons.append(row['reason'])
            else:row.update(verified=True,reason=None)
        results.append(row)
    for uid in sorted(set(required_uids or [])-{u for u,_ in corners}):
        results.append(dict(uid=uid,required=True,verified=False,reason='required_actor_ground_annotation_missing'))
        reasons.append('required_actor_ground_annotation_missing')
    return dict(actors=results,review_reasons=sorted(set(reasons)),max_range_m=max_range,
        maximum_gap_m=maximum_gap,limitation='Local low-return plane and lowest parent component OBB corner. Margins/articulation may extend below ground; not mesh contact or physics collider proof. Sparse/multilevel terrain stays review.')
