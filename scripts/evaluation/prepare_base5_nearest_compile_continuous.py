#!/usr/bin/env python3
"""Freeze every original eligible four-bag frame for a research-only A/B.

No new raw decoding, GT edits, interpolation, resampling or production changes.
The original reconstructed GT and its frame exclusions remain in force. A
large original timestamp gap is preserved rather than silently filled.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import pickle
import sys

import numpy as np

from experiment_base5_proposal_peaks import CAMERAS, ROOT, author_row, digest


def calibration_comparison(reference, other):
    """File identities may differ while numeric camera geometry is identical."""
    rows = []
    if len(reference['cameras']) != len(other['cameras']):
        raise ValueError('Camera count differs')
    for a, b in zip(reference['cameras'], other['cameras']):
        if any(a[key] != b[key] for key in ('frame_id', 'height', 'width')):
            raise ValueError('Camera identity/resolution differs')
        error = {key: float(np.max(np.abs(np.asarray(a[key])-np.asarray(b[key]))))
                 for key in ('cam2img', 'lidar2cam', 'cam2lidar', 'lidar2img')}
        if any(value > 1.e-7 for value in error.values()):
            raise ValueError('Camera geometry differs')
        rows.append(dict(camera=a['frame_id'], max_abs_error=error))
    base_error = float(np.max(np.abs(reference['base_from_lidar']-other['base_from_lidar'])))
    if base_error > 1.e-7:
        raise ValueError('LiDAR mounting differs')
    return dict(reference_sha256=reference['config_sha256'], other_sha256=other['config_sha256'],
                tolerance=1.e-7, lidar_mount_max_abs_error=base_error, cameras=rows,
                numeric_geometry_equal=True, file_bytes_equal=reference['config_sha256']==other['config_sha256'])


def check_authored_camera_geometry(info, calibration):
    for camera in calibration['cameras']:
        row = info['images'][camera['frame_id']]
        if any(row[key] != camera[key] for key in ('height', 'width')):
            raise ValueError('Authored camera size differs from original sensor')
        for key in ('cam2img', 'lidar2cam', 'cam2lidar', 'lidar2img'):
            if not np.allclose(row[key], camera[key], rtol=0, atol=1.e-7):
                raise ValueError('Authored calibration differs: '+key)


def chronological_indices(raw_stamps, selected_stamps):
    """Recover raw indices from timestamps, never from eligible token suffixes."""
    # The original preparer sorts/deduplicates message timestamps. Preserve
    # that definition and record the raw read index separately for provenance.
    raw_read = {float(stamp): i for i, stamp in enumerate(raw_stamps)}
    chronological = {stamp: i for i, stamp in enumerate(sorted(raw_read))}
    if any(float(t) not in chronological for t in selected_stamps):
        raise ValueError('Prepared timestamp is absent from original raw LiDAR')
    return [(chronological[float(t)], raw_read[float(t)]) for t in selected_stamps]


def prepare(args):
    if args.output.exists():
        raise FileExistsError('Fresh research output required')
    sys.path.insert(0, str(ROOT/'docker/bevfusion'))
    from app.sensor4_calibration import load_sensor4_calibration
    import rosbag
    info_path, manifest_path = args.source/'infos_val.pkl', args.source/'manifest.json'
    infos = pickle.loads(info_path.read_bytes())
    original = json.loads(manifest_path.read_text())
    if tuple(infos['metainfo']['classes']) != ('vehicle', 'pedestrian', 'obstacle'):
        raise ValueError('Original class identity differs')
    calibration = load_sensor4_calibration(args.sensor, ['camera_'+key for key in CAMERAS])
    if original['sensor_sha256'] != calibration['config_sha256']:
        raise ValueError('Use the original manifest sensor, not a different file')
    comparison = calibration_comparison(calibration, load_sensor4_calibration(
        args.comparison_sensor, ['camera_'+key for key in CAMERAS]))
    source_hashes = {str(path.resolve()): digest(path) for path in (info_path, manifest_path, args.sensor, args.comparison_sensor)}
    authored = infos['data_list']
    if len(authored) != original['frames'] or len({r['token'] for r in authored}) != len(authored):
        raise ValueError('Original frame count/token contract differs')
    rows, case_reports = [], {}
    for case, details in original['cases'].items():
        subset = [r for r in authored if r['scene_id']==case]
        payloads = {r['token']: r for r in details['input_payloads']}
        if len(subset) != details['selected'] or {r['token'] for r in subset} != set(payloads):
            raise ValueError('Original selected frame contract differs')
        raw_bag = Path(details['raw_bag'])
        bag_sha256 = digest(raw_bag)
        raw_stamps = []
        with rosbag.Bag(str(raw_bag), 'r') as bag:
            for _, message, receipt in bag.read_messages(topics=['/sensor/lidar1']):
                stamp = message.header.stamp.to_sec() if message.header.stamp.to_sec()>0 else receipt.to_sec()
                raw_stamps.append(stamp)
        indices = chronological_indices(raw_stamps, [r['timestamp'] for r in subset])
        stamps = np.asarray([r['timestamp'] for r in subset])
        if len(stamps)>1 and not np.all(np.diff(stamps)>0):
            raise ValueError('Original case timestamps not strictly increasing')
        for source, (raw_index, raw_read_index) in zip(subset, indices):
            check_authored_camera_geometry(source, calibration)
            row = author_row(source, args.source.resolve(), 'dev')
            row['raw_lidar_index'], row['raw_lidar_read_index'] = raw_index, raw_read_index
            row['gt_track_keys'] = [r['track_key'] for r in source['instances']]
            row['gt_instance_ids'] = [r['instance_id'] for r in source['instances']]
            row['gt_validity'] = [r['bbox_3d_isvalid'] for r in source['instances']]
            row['box_center_convention'] = 'Export boxes: bottom-center; original infos: catalog geometric/gravity center; z -= height/2 only, original infos unchanged'
            row['point_sha256'] = digest(row['point_path'])
            row['image_sha256'] = {key:digest(path) for key,path in row['images'].items()}
            fingerprint = payloads[row['token']]
            if row['stamp'] != fingerprint['stamp'] or row['point_sha256'] != fingerprint['point_sha256']:
                raise ValueError('Original point/stamp fingerprint changed')
            if any(row['image_sha256']['image/'+camera] != value for camera,value in fingerprint['image_sha256'].items()):
                raise ValueError('Original JPEG fingerprint changed')
            frame_path = args.source/'data'/row['token']/'frame.json'
            frame = json.loads(frame_path.read_text())
            if frame['stamp'] != row['stamp'] or frame['case'] != case:
                raise ValueError('Original frame metadata differs')
            row['authored_source'].update(info_sha256=source_hashes[str(info_path.resolve())],
                frame_sha256=digest(frame_path), frame_path=str(frame_path.resolve()),
                camera_offsets_sec=frame['camera_offsets_sec'], ego=frame['ego'])
            rows.append(row)
        dt = np.diff(stamps)
        case_reports[case] = dict(raw_bag=str(raw_bag), raw_bag_sha256=bag_sha256,
            frames=len(subset), raw_lidar_messages=len(raw_stamps), retained_raw_indices=[a for a,_ in indices],
            stamp_start=float(stamps[0]), stamp_end=float(stamps[-1]),
            dt_sec=dict(min=float(np.min(dt)), median=float(np.median(dt)), p95=float(np.quantile(dt,.95)), max=float(np.max(dt))),
            original_catalog_resolution=details['catalog_resolution'],
            original_frame_resolution=details['frame_resolution'],
            original_skipped=details['skipped'], original_contract_sha256=details['contract_sha256'])
        print(json.dumps(dict(case=case, frames=len(subset), raw_messages=len(raw_stamps), preserved_max_gap_sec=float(np.max(dt)))),flush=True)
    by_token = {r['token']:r for r in rows}
    rows = [by_token[r['token']] for r in authored]
    if source_hashes != {path:digest(path) for path in source_hashes}:
        raise ValueError('Source contract changed while preparing research manifest')
    result = dict(rows=rows, input_sha256=source_hashes, cases=dict(Counter(r['case'] for r in rows)),
        source_lineage=dict(original_inputs=str(args.source.resolve()), original_infos_sha256=digest(info_path),
            original_manifest_sha256=digest(manifest_path), sensor_sha256=calibration['config_sha256'],
            catalog_sha256=original['catalog_sha256'], original_gt_reference=original['gt_reference'],
            cases=case_reports, calibration_comparison=comparison),
        scope='All original 1570 eligible four-bag frames, no additional subsampling or temporal resampling; original exclusion gaps retained.',
        replay_scope='Contiguous retained sensor inputs with original stamps and original exclusion gaps. Suitable for independent offline EKF-core replay, not actual 30Hz ROS/full-stack/closed-loop latency certification.',
        source_contracts=dict(dev='Original ROS XYZI intensity0..255 and JPEG bytes; original reconstructed/interpolated GT, not synchronous native annotation; GT center conversion in exports only.'))
    args.output.mkdir(parents=True)
    (args.output/'inputs.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),frames=len(rows),sensor_geometry_equal=True)),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sensor',type=Path,required=True)
    p.add_argument('--comparison-sensor',type=Path,required=True)
    prepare(p.parse_args())
