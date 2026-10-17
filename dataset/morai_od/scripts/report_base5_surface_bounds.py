#!/usr/bin/env python3
"""Read-only silhouette/surface checks independent of box containment.

An enclosing box can contain all mask pixels and still be too large. Compare
the *visible* native mask bounds with projected physical hypotheses instead.
Drum hypotheses are elliptical cylinders, not an exact mesh/collider model.
Depth.png is deliberately not decoded without a verified metric encoding.
"""
import argparse
import ast
import itertools
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from audit_morai_lidar_gt_visibility import parse_instances
from base5_asset_scale import WHITELIST
from base5_capture_audit import rotation_matrix
from base5_capture_safety import atomic_json, sha256
from base5_native_box_corners import derive_geometry, load_matching_corners
from dynamic_capture_metadata import normalized_sensor_calibration
from report_base5_compound_evidence import camera_parent
from report_base5_camera_crop import CAMERAS


SCHEMA = 'base5_visible_surface_bounds_diagnostic_v1'
DRUMS = {'RedBarrel', 'YellowBarrel', 'TrafficBarrel', 'PE_Drum'}


def local_surface(size, cylinder=False):
    size = np.asarray(size, float)
    if size.shape != (3,) or not np.isfinite(size).all() or (size <= 0).any():
        raise ValueError('Finite positive L/W/H required')
    if not cylinder:
        return np.asarray(list(itertools.product((-1., 1.), repeat=3))) * size / 2
    angle = np.arange(360) * np.pi / 180
    xy = np.c_[np.cos(angle)*size[0]/2, np.sin(angle)*size[1]/2]
    return np.vstack([np.c_[xy, np.full(len(xy), z)] for z in (-size[2]/2, size[2]/2)])


def project(points, camera):
    optical = (np.c_[points, np.ones(len(points))] @ np.asarray(camera['lidar_to_optical']).T)[:, :3]
    good = optical[:, 2] > .05
    pixels = optical @ np.asarray(camera['intrinsic']).T
    pixels = pixels[:, :2] / np.where(good, pixels[:, 2], np.nan)[:, None]
    return pixels, good


def bounds(points):
    points = np.asarray(points, float)
    return [float(points[:, 0].min()), float(points[:, 1].min()),
            float(points[:, 0].max()), float(points[:, 1].max())]


def compare_bounds(observed, predicted):
    observed, predicted = np.asarray(observed, float), np.asarray(predicted, float)
    if observed.shape != (4,) or predicted.shape != (4,) or not np.isfinite(np.r_[observed, predicted]).all():
        raise ValueError('Finite image bounds required')
    oe, pe = observed[2:]-observed[:2], predicted[2:]-predicted[:2]
    if (oe <= 0).any() or (pe <= 0).any():
        raise ValueError('Positive image extents required')
    return dict(observed_to_projected_extent_ratio=(oe/pe).tolist(),
                absolute_edge_residual_px=np.abs(observed-predicted).tolist(),
                mean_absolute_edge_residual_px=float(np.abs(observed-predicted).mean()))


def inspect(root, asset, output, index):
    if asset not in WHITELIST:
        raise ValueError('Only explicitly whitelisted physical obstacle assets supported')
    root = Path(root).resolve(strict=True)
    metadata = json.loads((root/'metadata.json').read_text())
    scenario = json.loads((root/'scenario.json').read_text())
    sensor = root/'sensor_reference.json'
    calibration = normalized_sensor_calibration(sensor)
    if metadata['sensor_source_sha256'] != sha256(sensor):
        raise ValueError('Saved sensor bytes differ from capture calibration')
    annotations = list(root.glob('LIDAR_*/*_Instance.txt'))
    if len(annotations) != 1:
        raise ValueError('One exact annotation timestamp required')
    annotation = annotations[0]
    corners = load_matching_corners(annotation)
    actors = {a['unique_id']: a for a in metadata['runtime_actors'] if a['name'] == asset}
    payloads = {a['UNIQUEID']: a for a in scenario.get('objectList', [])}
    selected = [b for b in parse_instances(annotation) if b['unique_id'] in actors and b['raw_class'] == 'Obstacle']
    if len(selected) != len(actors) or not actors:
        raise ValueError('Only exact single-component physical obstacle parents supported')
    pins = {str(p): sha256(p) for p in (root/'metadata.json', root/'scenario.json', sensor, annotation,
        annotation.with_name(annotation.name.replace('_Instance.txt', '_Instance_8Points.txt')))}
    lidar_path = annotation.with_name(annotation.name.replace('_Instance.txt', '_Intensity.bin'))
    pins[str(lidar_path)] = sha256(lidar_path)
    cloud = np.fromfile(lidar_path, np.float32).reshape(-1, 4)[:, :3]
    cloud = cloud[np.isfinite(cloud).all(1) & (cloud != 0).any(1)]
    rows = []
    for box in selected:
        uid = box['unique_id']
        if (uid not in payloads or actors[uid].get('source_list') != 'objectList'
                or actors[uid].get('data_id') != WHITELIST[asset][0]
                or payloads[uid].get('DataID') != WHITELIST[asset][0]):
            raise ValueError('Asset/DataID/container differs between native and scenario evidence')
        measured = derive_geometry(box, corners[(uid, 'Obstacle', 0)], allow_dimension_change=True)
        rotation = rotation_matrix(measured['rotation_rpy'])
        size = np.asarray(measured['size'])
        hypotheses = dict(native_corners=size, native_div125=size/1.25)
        actor = actors[uid]
        if actor.get('estimated_size'):
            hypotheses['placement_expected'] = np.asarray(actor['estimated_size'], float)
        scale = [float(payloads[uid]['scale'][k]) for k in 'xyz']
        row = dict(parent_uid=uid, asset=asset, scenario_scale_xyz=scale, native_size_lwh_m=size.tolist(),
            summary_size_lwh_m=box['size'].tolist(), cameras={}, native_center_lidar_m=measured['center'],
            hypothesis_shape='elliptical-cylinder-proxy' if asset in DRUMS else 'OBB-envelope-not-mesh',
            annotation_identity_verified=True)
        for name in CAMERAS:
            folder = root/name
            stamp = annotation.name[:-len('_Instance.txt')]
            paths = {key: folder/(stamp+suffix) for key, suffix in
                [('mask','_Instance.png'), ('bbox','_BBox2D.txt'), ('rgb','_Intensity.png')]}
            if not all(p.is_file() for p in paths.values()):
                raise ValueError('Exact synchronous camera companions required')
            pins.update({str(p): sha256(p) for p in paths.values()})
            colours = []
            for line in paths['bbox'].read_text().splitlines():
                f = line.split(maxsplit=6)
                if len(f) != 7:
                    raise ValueError('Malformed BBox2D')
                if camera_parent(int(f[5])) == uid and f[0] == 'Obstacle':
                    colours.append(ast.literal_eval(f[6]))
            with Image.open(paths['mask']) as im:
                pixels = np.asarray(im.convert('RGBA'))
            mask = np.zeros(pixels.shape[:2], bool)
            for colour in colours:
                mask |= (pixels == np.asarray(colour)).all(2)
            yy, xx = np.where(mask)
            if not len(xx):
                continue
            observed = [int(xx.min()), int(yy.min()), int(xx.max())+1, int(yy.max())+1]
            matches = [record for record in calibration['cameras'].values()
                       if record['sensor_unique_id'] == int(name.rsplit('_', 1)[1])]
            if len(matches) != 1:
                raise ValueError('Unique camera calibration by sensor UID required')
            record = matches[0]
            camera = dict(intrinsic=record['intrinsic'], lidar_to_optical=(
                np.asarray(record['ego_to_sensor']) @ np.asarray(calibration['lidar']['lidar_to_ego'])).tolist())
            value = dict(mask_pixels=int(mask.sum()), visible_bounds_xyxy=observed,
                touches_image_edge=bool(mask[0].any() or mask[-1].any() or mask[:,0].any() or mask[:,-1].any()),
                occlusion_not_resolved=True, hypotheses={})
            for label, shape_size in hypotheses.items():
                pts = local_surface(shape_size, asset in DRUMS) @ rotation.T + np.asarray(measured['center'])
                uv, good = project(pts, camera)
                if not good.all():
                    value['hypotheses'][label] = dict(available=False, reason='surface_near_or_behind_camera')
                    continue
                predicted = bounds(uv)
                value['hypotheses'][label] = dict(available=True, projected_bounds_xyxy=predicted,
                    **compare_bounds(observed, predicted))
            # This selection uses visible instance pixels, never native box membership.
            uv, good = project(cloud, camera)
            ij = np.rint(np.nan_to_num(uv, nan=-1e6)).astype(np.int64)
            good &= (ij[:,0] >= 0) & (ij[:,0] < mask.shape[1]) & (ij[:,1] >= 0) & (ij[:,1] < mask.shape[0])
            selected_ids = np.flatnonzero(good)
            selected_ids = selected_ids[mask[ij[selected_ids,1], ij[selected_ids,0]]]
            values = (cloud[selected_ids]-measured['center']) @ rotation
            value['lidar_projecting_into_visible_mask'] = dict(count=len(selected_ids),
                local_min_m=values.min(0).tolist() if len(values) else None,
                local_max_m=values.max(0).tolist() if len(values) else None,
                not_semantic_instance_returns=True, cross_sensor_parallax_and_occlusion_unresolved=True)
            row['cameras'][name] = value
            with Image.open(paths['rgb']) as im:
                rgb = im.convert('RGB')
            draw = ImageDraw.Draw(rgb)
            draw.rectangle(observed, outline='#00ff00', width=2)
            for label, colour in [('native_corners','#ff0000'), ('native_div125','#00ffff'), ('placement_expected','#ffff00')]:
                h = value['hypotheses'].get(label, {})
                if h.get('available'):
                    draw.rectangle(h['projected_bounds_xyxy'], outline=colour, width=2)
            x1,y1,x2,y2 = observed
            pad = max(40, (x2-x1)//2, (y2-y1)//3)
            rgb = rgb.crop((max(0,x1-pad), max(0,y1-pad), min(rgb.width,x2+pad), min(rgb.height,y2+pad)))
            rgb.save(output/f'{index:03d}_{uid}_{name}.png')
        rows.append(row)
    if any(sha256(p) != checksum for p, checksum in pins.items()):
        raise RuntimeError('Immutable native source changed')
    return dict(capture_root=str(root), actors=rows, sources=pins, source_unchanged=True)


def run(roots, asset, output):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh diagnostic output required')
    if any(Path(root).resolve() == output or Path(root).resolve() in output.parents for root in roots):
        raise ValueError('Never write diagnostic inside raw capture')
    output.mkdir(parents=True)
    result = dict(schema=SCHEMA, asset=asset, captures=[inspect(root,asset,output,i) for i,root in enumerate(roots)],
        implementation_sha256={str(Path(__file__).resolve()): sha256(__file__),
          **{str(Path(__file__).with_name(name).resolve()): sha256(Path(__file__).with_name(name))
             for name in ('base5_capture_audit.py','base5_native_box_corners.py',
                          'dynamic_capture_metadata.py','report_base5_compound_evidence.py')}},
        limitations=['Diagnostic, not automatic mesh/collider/GT approval.',
          'Silhouette extent can shrink under occlusion, field-of-view clipping or transparent materials.',
          'Cylinder is a drum shape hypothesis, not an exact mesh reconstruction.',
          'Projected-mask LiDAR association has parallax/occlusion ambiguity; no metric depth decoding assumed.',
          'Raw annotations, acceptance and training labels remain unchanged.'])
    atomic_json(output/'report.json', result)
    return output/'report.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-root', type=Path, action='append', required=True)
    parser.add_argument('--asset', default='RedBarrel')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(run(args.capture_root,args.asset,args.output))
