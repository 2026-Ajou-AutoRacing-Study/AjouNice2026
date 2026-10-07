"""Local LiDAR ground-plane witness; no raw/GT edits, no collider certification."""
import numpy as np


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
            if candidate is not None and Polygon(candidate['support_polygon_xy']).distance(Point(xy))<=.75:
                plane=candidate;break
        row=dict(uid=uid,required=True,verified=False,plane=plane,patch_radius_m=patch_radius)
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
    return dict(actors=results,review_reasons=sorted(set(reasons)),max_range_m=max_range,
        maximum_gap_m=maximum_gap,limitation='Local low-return plane and lowest parent component OBB corner. Margins/articulation may extend below ground; not mesh contact or physics collider proof. Sparse/multilevel terrain stays review.')
