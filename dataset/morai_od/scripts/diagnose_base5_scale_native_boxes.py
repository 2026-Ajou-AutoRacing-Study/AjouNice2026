#!/usr/bin/env python3
"""Diagnostic-only scale hypotheses around unchanged native centers/rotations.

This writes independent evidence, NOT replacement GT, training infos, a capture
approval, or a production scale policy. Native source bytes remain untouched.
"""
import argparse
import itertools
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

from base5_capture_audit import rotation_matrix
from base5_capture_safety import atomic_json, sha256
from dynamic_capture_metadata import normalized_sensor_calibration
from report_base5_camera_crop import CAMERAS
from report_base5_scale_probe import inventory, require, runtime_sources
from report_base5_compound_evidence import point_memberships


REPO = Path(__file__).resolve().parents[3]
SIGNS = np.asarray(list(itertools.product((-1., 1.), repeat=3)))


def corners(component, size):
    return SIGNS * np.asarray(size) / 2 @ rotation_matrix(component['native_rotation_rpy']).T + component['native_center_lidar_m']


def project(component, size, camera):
    transform = np.asarray(camera['lidar_to_optical'])
    optical = (np.c_[corners(component, size), np.ones(8)] @ transform.T)[:, :3]
    if (optical[:, 2] <= .05).any():
        return dict(available=False, reason='Near/behind-camera corner; edge clipping not implemented')
    pixels = optical @ np.asarray(camera['intrinsic']).T
    xy = pixels[:, :2] / pixels[:, 2:3]
    hull = cv2.convexHull(xy.astype(np.float32)).reshape(-1, 2)
    return dict(available=True, projected_corners_xy=xy.tolist(), projected_hull_xy=hull.tolist(),
        projected_bounds_xyxy=[float(xy[:, 0].min()), float(xy[:, 1].min()),
                               float(xy[:, 0].max()), float(xy[:, 1].max())])


def containment(projection, mask):
    if not projection['available']:
        return dict(projection, native_instance_pixels=int(mask.sum()))
    hull = np.asarray(projection['projected_hull_xy'])
    bounded = np.clip(hull, -100000., 100000.)
    polygon = np.zeros_like(mask, np.uint8)
    cv2.fillConvexPoly(polygon, np.round(bounded).astype(np.int32), 1)
    count, inside = int(mask.sum()), int((mask & (polygon > 0)).sum())
    return dict(projection, native_instance_pixels=count,
        instance_pixels_inside_projected_hull=inside, instance_pixels_outside_projected_hull=count - inside,
        instance_fraction_inside_projected_hull=inside / count if count else None,
        projected_hull_in_image_pixels=int(polygon.sum()))


def diagnostic(component, size, points, cameras, masks, transform, ground):
    raw = dict(center_lidar_m=component['native_center_lidar_m'], size_m=list(size),
               rotation_rpy=component['native_rotation_rpy'])
    selected = point_memberships(points, [raw])[0]
    result = dict(hypothetical_size_lwh_m=list(size), lidar_bbox_points=int(selected.sum()),
        native_center_and_rotation_unchanged=True,
        cameras={name: containment(project(component, size, cameras[name]), mask)
                 for name, mask in masks.items()})
    if transform is not None:
        ego = (np.c_[corners(component, size), np.ones(8)] @ np.asarray(transform).T)[:, :3]
        values = []
        for check in ground:
            plane = check.get('plane')
            if not plane:
                continue
            a, b, c = plane['coefficients']
            gaps = ego[:, 2] - (a * ego[:, 0] + b * ego[:, 1] + c)
            values.append(dict(parent_uid=check['uid'],
                minimum_annotation_corner_gap_over_same_observed_plane_m=float(gaps.min()),
                source_plane=plane, not_mesh_contact=True))
        result['hypothetical_ground_plane_gaps'] = values
    return result


def overlay(root, component, native, assumed, output, measured_corners=False):
    canvas = Image.new('RGB', (1280, 800), '#e5e7eb')
    draw = ImageDraw.Draw(canvas)
    label = 'measured native eight-corner box' if measured_corners else 'hypothetical scaled 3D'
    draw.text((8, 6), 'RED summary 3D; CYAN ' + label + '. Diagnostic only, not replacement labels.', fill='black')
    for i, name in enumerate(CAMERAS):
        files = list((root / name).glob('*_Intensity.png'))
        require(len(files) == 1, 'One native RGB required')
        with Image.open(files[0]) as image:
            raw = image.convert('RGB')
        factor = min(630. / raw.width, 350. / raw.height)
        thumb = raw.resize((round(raw.width * factor), round(raw.height * factor)), Image.BILINEAR)
        x, y = i % 2 * 640 + 5, i // 2 * 380 + 35
        canvas.paste(thumb, (x, y))
        draw.text((x, y - 15), name, fill='black')
        for profile, color in ((native, '#ef4444'), (assumed, '#06b6d4')):
            value = profile.get('cameras', {}).get(name)
            if not value or not value['available']:
                continue
            hull = [(x + p[0] * factor, y + p[1] * factor) for p in value['projected_hull_xy']]
            draw.line(hull + hull[:1], fill=color, width=2)
    canvas.save(output)


def run(report_path, output, sensor):
    report_path, output, sensor = Path(report_path), Path(output), Path(sensor)
    require(not output.exists(), 'Fresh standalone diagnostic output required')
    source = json.loads(report_path.read_text())
    require(source['schema'] == 'base5_scale_probe_native_v1', 'Native scale report required')
    calibration = normalized_sensor_calibration(sensor)
    inputs_before = {str(report_path.resolve()): sha256(report_path), str(sensor.resolve()): sha256(sensor)}
    rows, source_rows = [], []
    for row in source['captures']:
        root = Path(row['capture_root'])
        require(root.resolve() not in output.resolve().parents and root.resolve() != output.resolve(),
                'Cannot write diagnostic inside immutable native capture')
        for path, info in source['source_inventory'].items():
            path = Path(path)
            if root.resolve() in path.resolve().parents:
                require(sha256(path) == info['sha256'], 'Native evidence changed since source report')
        metadata = json.loads((root / 'metadata.json').read_text())
        require(metadata['sensor_source_sha256'] == calibration['source_sha256'],
                'Projection calibration differs from the captured sensor SHA')
        require(np.allclose(metadata['lidar_to_ego'], calibration['lidar']['lidar_to_ego'], rtol=0., atol=1e-10),
                'Projection LiDAR-to-Ego differs from captured calibration')
        source_rows.append(dict(id=row['id'], capture_root=str(root)))
    native_before = inventory([], source_rows)
    helpers_before = runtime_sources()
    output.mkdir(parents=True)
    for row in source['captures']:
        root = Path(row['capture_root'])
        metadata = json.loads((root / 'metadata.json').read_text())
        lidar = list(root.glob('LIDAR_*/*_Intensity.bin'))
        require(len(lidar) == 1, 'One native XYZI required')
        raw = np.fromfile(lidar[0], np.float32)
        require(raw.size and raw.size % 4 == 0 and np.isfinite(raw).all(), 'Invalid native XYZI')
        points = raw.reshape(-1, 4)
        points = points[(points[:, :3] != 0).any(axis=1)]
        for component in row['components']:
            cameras, masks = {}, {}
            for record in calibration['cameras'].values():
                name = 'CAMERA_' + str(record['sensor_unique_id'])
                cameras[name] = dict(intrinsic=record['intrinsic'],
                    lidar_to_optical=(np.asarray(record['ego_to_sensor']) @
                                      np.asarray(metadata['lidar_to_ego'])).tolist())
            for observation in component['camera_observations']:
                name = observation['camera']
                if name not in masks:
                    mask_files = list((root / name).glob('*_Instance.png'))
                    require(len(mask_files) == 1, 'One native instance mask required')
                    with Image.open(mask_files[0]) as image:
                        rgba = np.asarray(image.convert('RGBA'))
                    masks[name] = (rgba == observation['instance_colour']).all(axis=2)
            size = component.get('native_summary_size_lwh_m', component['native_size_lwh_m'])
            scale = row['probe']['actual_scenario_scale_xyz']
            native = diagnostic(component, size, points, cameras, masks, metadata['lidar_to_ego'], row['ground_support'])
            corner_geometry = component.get('native_eight_corner_geometry', {})
            measured_corners = (diagnostic(component, corner_geometry['size_lwh_m'], points, cameras, masks,
                metadata['lidar_to_ego'], row['ground_support']) if corner_geometry.get('available') else None)
            hypotheses = []
            for axes in itertools.permutations(range(3)):
                hypothetical = (np.asarray(size) * np.asarray(scale)[list(axes)]).tolist()
                value = diagnostic(component, hypothetical, points, cameras, masks, metadata['lidar_to_ego'], row['ground_support'])
                value['native_lwh_from_unity_xyz'] = ['xyz'[i] for i in axes]
                value['axis_permutation'] = list(axes)
                hypotheses.append(value)
            assumption = row['probe'].get('geometry_axis_hypothesis', [0, 2, 1])
            assumed = next(h for h in hypotheses if h['axis_permutation'] == assumption)
            ident = ''.join(c if c.isalnum() or c in '_-' else '_' for c in row['id'])
            image_path = output / (ident + '_projection.png')
            overlay(root, component, native, measured_corners or assumed, image_path, measured_corners is not None)
            rows.append(dict(id=row['id'], asset=row['probe']['asset'], requested_scale_xyz=scale,
                native_component_id=component['component_id'], native=native, hypothetical_axis_variants=hypotheses,
                measured_native_eight_corner=measured_corners,
                preferred_geometry_source='native_eight_corners' if measured_corners is not None else 'unqualified_hypothesis_only',
                declared_hypothesis=assumption, overlay=str(image_path.resolve()), production_gt_changed=False,
                production_approval=False, naturalness_approved=False))
    require(native_before == inventory([], source_rows), 'Native evidence changed during hypothesis diagnosis')
    require(inputs_before == {str(report_path.resolve()): sha256(report_path), str(sensor.resolve()): sha256(sensor)},
            'Source report or projection sensor changed during diagnosis')
    require(helpers_before == runtime_sources(), 'Runtime source changed during hypothesis diagnosis')
    result = dict(schema='base5_scale_native_box_hypothesis_v1', source_report=str(report_path.resolve()),
        input_sha256=inputs_before, captures=rows, native_source_inventory=native_before,
        native_unchanged=True, runtime_sources=helpers_before,
        limitations=['Hypothetical boxes only, never rewritten raw GT or training labels.',
            'Center/rotation are fixed to actual native annotation; each size hypothesis is a deliberately unqualified transform.',
            'More inside-box returns may include road/background and do not independently prove correct GT.',
            'Image containment tests native foreground against projected box hull, not semantic 3D correspondence.',
            'Plane gaps reuse the observed ground fit; zero gap is not certified mesh contact.',
            'Axis selection, GT repair, production use and semantic naturalness are not automatically approved.'])
    atomic_json(output / 'report.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sensor', type=Path, default=REPO / 'configs/sensors/Sensor_competition_local.json')
    args = parser.parse_args()
    result = run(args.report, args.output, args.sensor)
    print(json.dumps(dict(captures=len(result['captures']), native_unchanged=result['native_unchanged'])))
