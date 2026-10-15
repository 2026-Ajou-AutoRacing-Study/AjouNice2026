"""Independent empty-scene surface reference for near LiDAR blind spots."""
import math
from pathlib import Path
import numpy as np
from base5_ground_support import fit_plane
from shapely.geometry import Point,Polygon


def ego_to_world(points,ego,heading_correction=-1.347):
    a=math.radians(ego['heading']+heading_correction)
    R=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
    return np.asarray(points)@R.T+np.array([ego[k] for k in ('x','y','z')])


def surface_at(points,xy,support_xy=None):
    points=np.asarray(points);xy=np.asarray(xy)
    for radius in (3.,5.,8.):
        local=points[np.linalg.norm(points[:,:2]-xy,axis=1)<radius]
        plane=fit_plane(local)
        targets=[xy] if support_xy is None else support_xy
        if plane and all(Polygon(plane['support_polygon_xy']).distance(Point(p))<=.75 for p in targets):
            plane.update(patch_radius_m=radius,reference_xy=xy.tolist())
            return plane
    return None


def world_cloud(root,metadata,telemetry):
    paths=list(Path(root).glob('LIDAR_*/*_Intensity.bin'))
    if len(paths)!=1:raise ValueError('Ground probe requires one complete LiDAR')
    xyz=np.fromfile(paths[0],np.float32).reshape(-1,4)[:,:3]
    xyz=xyz[np.isfinite(xyz).all(axis=1)&(xyz!=0).any(axis=1)]
    ego=(np.c_[xyz,np.ones(len(xyz))]@np.asarray(metadata['lidar_to_ego']).T)[:,:3]
    # Ego mesh is not independent road support. Same footprint mask used by
    # ground review; raw sensor files are left untouched.
    ego=ego[~((ego[:,0]>=-1.3)&(ego[:,0]<=4.)&(np.abs(ego[:,1])<=1.3))]
    return ego_to_world(ego,telemetry['ego_before'])


def plane_height(plane,xy):
    a,b,c=plane['coefficients'];return a*xy[0]+b*xy[1]+c


def supported(plane,xy):
    return Polygon(plane['support_polygon_xy']).distance(Point(xy))<=.75
