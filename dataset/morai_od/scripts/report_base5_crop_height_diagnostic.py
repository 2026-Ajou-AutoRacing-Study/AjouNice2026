#!/usr/bin/env python3
"""CPU-only 512/576 height diagnostic; never change raw, labels or production.

Both profiles keep width 1056 and its existing native-relative resize. A taller
crop adds upper field of view, not object sampling density. Actual instance
masks are read again; the legacy full-5K 512 cache must match exactly.
"""
import argparse
import ast
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image

import analyze_base5_crop_candidates as crop


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ANALYSIS = ROOT/'artifacts/evaluation/bevfusion/base5_crop_final5k_261010'
ONLINE_SOURCE = ROOT/'docker/bevfusion/app/online_preprocessor.py'
SENSOR = ROOT/'configs/sensors/Sensor_competition_local.json'
PROFILES = (dict(id='h512', h=512, w=1056, side_start=None),
            dict(id='h576', h=576, w=1056, side_start=None))
CLASSES = ('vehicle', 'pedestrian', 'obstacle', 'animal')
LIMITATIONS = [
    'Foreground retention and native sensor gates, not trained AP, BBox accuracy or latency.',
    'Only pixels inside the original sensor can be restored; native border contact is not a certified clipping measurement.',
    'Instance masks use nearest resize; RGB runtime uses cubic interpolation.',
    'Near observations and unique unambiguous components have different denominators.',
    'Ambiguous shared-camera components do not receive invented per-component camera evidence.',
    'Far campaigns use initial raw once per source, including review rows; they are not a new accepted training dataset.',
    'CPU geometry checks do not load a model/checkpoint, run backward, or certify 576 GPU/runtime performance.',
    '576 is not enabled in the GT converter or production; a future change needs crop-specific infos and runtime parity qualification.',
]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def pin(pins, path):
    path = Path(path).resolve()
    digest = sha256(path)
    previous = pins.setdefault(str(path), digest)
    if previous != digest:
        raise RuntimeError('Input changed during diagnostic: '+str(path))
    return path


def verify_pins(pins):
    if any(sha256(path) != expected for path, expected in pins.items()):
        raise RuntimeError('Diagnostic input/source hash changed')


def runtime_geometry_functions(path=ONLINE_SOURCE):
    """Execute only existing pure geometry methods, not torch/ROS/model imports."""
    tree = ast.parse(Path(path).read_text())
    option = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                  and n.name == 'preprocessing_options_from_config')
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == 'Base3OnlinePreprocessor')
    methods = [n for n in original.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_resize_crop', '_augmentation_matrix')]
    if len(methods) != 2:
        raise RuntimeError('Online pure geometry API changed')
    wrapper = ast.ClassDef(name='CPUGeometry', bases=[], keywords=[], body=methods, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[option, wrapper], type_ignores=[]))
    namespace = dict(np=np)
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace['CPUGeometry'], namespace['preprocessing_options_from_config']


def shape_contract():
    geometry_class, options = runtime_geometry_functions()
    sensor = read(SENSOR)
    dimensions = [(int(c['cc']['cameraResWidth']), int(c['cc']['cameraResHeight']))
                  for c in sensor['cameraList']]
    expected = [(crop.CAMERAS[c][1], crop.CAMERAS[c][2]) for c in crop.CAMERAS]
    if dimensions != expected:
        raise RuntimeError('Current native sensor dimensions differ from crop analysis')
    report = dict(passed=True, model_or_checkpoint_loaded=False,
                  gpu_used=False, production_modified=False, candidates={})
    prepared = {}
    for height in (512, 576, 640, 704):
        profile = dict(id='h'+str(height), h=height, w=1056, side_start=None)
        config = dict(test_dataloader=dict(dataset=dict(pipeline=[dict(type='MoraiImageAug3D',
            final_dim=[height, 1056], resize_lim=[.825, .825], bot_pct_lim=[0., 0.],
            rot_lim=[0., 0.], rand_flip=False, is_train=False)])), point_cloud_range=[-100,-30,-3,100,30,2],
            model=dict(view_transform=dict(image_size=[height,1056], feature_size=[height//8,132])))
        options(config)
        instance = geometry_class()
        instance.final_dim, instance.resize, instance.reference_width = (height,1056), .825, 1280
        cameras, errors = {}, []
        for name, (width, native_height) in zip(crop.CAMERAS, dimensions):
            camera = dict(width=width, height=native_height, frame_id=name)
            try:
                resize, rectangle = instance._resize_crop(camera)
                matrix = instance._augmentation_matrix(camera)
                expected_geometry = crop.geometry(name, profile)
                expected_matrix = np.eye(4)
                expected_matrix[:2,:2] = np.asarray(expected_geometry['augmentation_matrix'])[:2,:2]
                expected_matrix[:2,3] = np.asarray(expected_geometry['augmentation_matrix'])[:2,2]
                np.testing.assert_allclose(matrix, expected_matrix, atol=1e-6)
                point = np.array([123.,234.,0.,1.])
                np.testing.assert_allclose(np.linalg.inv(matrix)@(matrix@point), point, atol=1e-4)
                x, y = np.meshgrid(np.arange(width), np.arange(native_height))
                rgb = np.stack((x%251, y%251, (x+y)%251), axis=-1).astype(np.uint8)
                resized = cv2.resize(rgb, resize, interpolation=cv2.INTER_CUBIC)
                left,top,right,bottom = rectangle
                prepared[(height,name)] = resized[top:bottom,left:right]
                if prepared[(height,name)].shape != (height,1056,3):
                    raise RuntimeError('Unexpected CPU prepared RGB shape')
                cameras[name] = dict(resize_wh=list(resize), crop_xyxy=list(rectangle),
                    native_window_xyxy=expected_geometry['native_window_xyxy'],
                    augmentation_matrix=matrix.tolist())
            except ValueError as error:
                errors.append(dict(camera=name, reason=str(error)))
        report['candidates'][str(height)] = dict(common_crop_valid=not errors,
            image_size=[height,1056], feature_size=[height//8,132], cameras=cameras, errors=errors)
    for name in crop.CAMERAS:
        if not np.array_equal(prepared[(512,name)], prepared[(576,name)][64:]):
            raise RuntimeError('576 changed the existing bottom512 RGB pixels')
    report.update(existing512_rgb_identical_to_bottom512_of576=True,
        added_upper_rows=64, image_and_feature_pixel_ratio=576/512,
        current_gt_converter_supports576=False, checkpoint_compatibility='not assessed')
    if not all(report['candidates'][str(h)]['common_crop_valid'] == (h <= 576)
               for h in (512,576,640,704)):
        raise RuntimeError('Current shape boundary changed')
    return report


def measure(rows, pins, cached512=False):
    grouped = defaultdict(list)
    for index, row in enumerate(rows):
        grouped[(row['capture_root'], row['camera'])].append(index)
    metrics, file_count, cache_matches = {}, 0, 0
    for (root, camera), indices in sorted(grouped.items()):
        folder = Path(root)/camera
        masks, boxes = list(folder.glob('*_Instance.png')), list(folder.glob('*_BBox2D.txt'))
        if len(masks) != 1 or len(boxes) != 1:
            raise RuntimeError('Expected one raw mask and BBox2D: '+str(folder))
        mask_path, box_path = pin(pins,masks[0]), pin(pins,boxes[0])
        with Image.open(mask_path) as image:
            pixels = np.asarray(image.convert('RGBA'))
        _, width, height = crop.CAMERAS[camera]
        if pixels.shape != (height,width,4):
            raise RuntimeError('Raw native camera shape changed')
        annotations = {}
        for line in box_path.read_text().splitlines():
            fields = line.split(maxsplit=6)
            if len(fields) == 7:
                annotations[(int(fields[5]),fields[0],tuple(float(v) for v in fields[1:5]))] = ast.literal_eval(fields[6])
        for index in indices:
            row = rows[index]
            key = (row['camera_instance_id'],row['raw_class'],tuple(row['native_rect']))
            if key not in annotations:
                raise RuntimeError('Missing exact camera annotation identity')
            colour = np.asarray(annotations[key],dtype=np.uint8)
            mask = (pixels == colour).all(axis=2).astype(np.uint8)
            prefix = np.r_[0,np.cumsum(mask.sum(axis=1),dtype=np.int64)]
            if int(prefix[-1]) != row['native_pixels']:
                raise RuntimeError('Actual native instance pixel count changed')
            metrics[index] = {p['id']:crop.mask_statistics(mask,prefix,row['native_rect'],
                crop.geometry(camera,p)) for p in PROFILES}
            if cached512:
                if metrics[index]['h512'] != row['candidates']['bottom512w1056']:
                    raise RuntimeError('Actual512 does not exactly match saved full5K cache')
                cache_matches += 1
        file_count += 1
    return metrics, dict(raw_camera_files=file_count, observation_count=len(rows),
                         cached512_exact_matches=cache_matches if cached512 else None)


def observation_key(row):
    return (row['scene'],row['camera'],row['camera_instance_id'],row['raw_class'],tuple(row['native_rect']))


def summaries(rows, metrics):
    result = []
    for class_name in sorted({r['training_class'] for r in rows}):
        for camera in crop.CAMERAS:
            indices = [i for i,r in enumerate(rows) if r['training_class']==class_name
                       and r['camera']==camera and r['native_pixels']>=16]
            if not indices:
                continue
            result.append(dict(training_class=class_name,camera=camera,n=len(indices),
                profiles={p['id']:crop.summarize(indices,metrics,p['id']) for p in PROFILES},
                improved_observations=sum(metrics[i]['h576']['native_kept']>metrics[i]['h512']['native_kept'] for i in indices),
                native_border_contact=sum(rows[i]['native_rect'][0]<=0 or rows[i]['native_rect'][1]<=0
                    or rows[i]['native_rect'][2]>=crop.CAMERAS[camera][1]-1
                    or rows[i]['native_rect'][3]>=crop.CAMERAS[camera][2]-1 for i in indices)))
    return result


def component_summary(components, rows, metrics):
    lookup = {observation_key(row):i for i,row in enumerate(rows)}
    joined, ambiguous = [], 0
    for scene, component in components:
        if component.get('camera_component_ambiguous'):
            ambiguous += 1
            continue
        indices = []
        for evidence in component['cameras']:
            key = crop.observation_key(scene,evidence['camera'],evidence)
            if key in lookup:
                indices.append(lookup[key])
            elif (evidence['native_instance_pixels']>0
                    and evidence['native_rect'][3]>evidence['native_rect'][1]
                    and evidence['native_rect'][2]>evidence['native_rect'][0]):
                raise RuntimeError('Cannot claim unique-component gate from partial camera views')
        if not indices:
            continue
        gates = {p['id']:any(metrics[i][p['id']]['candidate_camera_gate'] for i in indices) for p in PROFILES}
        joined.append(dict(scene=scene,component_id=component['component_id'],
            training_class=component['training_class'],lidar_points=component['lidar_points'],
            inside_roi=crop.inside_analysis_roi(component['center_lidar_m']),**gates))
    by_class = {}
    for name in sorted({r['training_class'] for r in joined}):
        values = [r for r in joined if r['training_class']==name]
        by_class[name] = dict(camera_observed_unambiguous_components=len(values),
            zero_lidar=sum(r['lidar_points']==0 for r in values),
            profiles={p['id']:dict(camera_gate=sum(r[p['id']] for r in values),
                zero_lidar_camera_gate=sum(r['lidar_points']==0 and r[p['id']] for r in values),
                sensor_positive=sum(r['lidar_points']>=1 or r[p['id']] for r in values),
                roi_sensor_positive=sum(r['inside_roi'] and (r['lidar_points']>=1 or r[p['id']]) for r in values)) for p in PROFILES},
            changed_camera_gates=[r for r in values if r['h512']!=r['h576']],
            gained_roi_sensor_positive=sum(r['inside_roi'] and r['lidar_points']==0 and not r['h512'] and r['h576'] for r in values))
    return dict(by_class=by_class,ambiguous_components_excluded_from_join=ambiguous)


def near_diagnostic(analysis, pins, classes=CLASSES, distance_max=5.):
    report_path = pin(pins,analysis/'report.json')
    observations_path = pin(pins,analysis/'observations.json')
    ledger_path = pin(pins,analysis/'source_ledger_snapshot.jsonl')
    report = read(report_path)
    if (not report.get('full_snapshot') or report['accepted_scenes']!=5000
            or report['source_sha256']!=sha256(ledger_path)):
        raise RuntimeError('Expected exact full5K frozen cache/ledger')
    observations = read(observations_path)
    if len(observations)!=report['measured_observations']:
        raise RuntimeError('Full cache observation count changed')
    rows = [r for r in observations if r['training_class'] in classes and 0<=r['range_m']<distance_max]
    metrics, counts = measure(rows,pins,cached512=True)
    counts.update(cached512_exact_matches_by_class=dict(Counter(r['training_class'] for r in rows)),
        native_at_least16_observations_by_class=dict(Counter(r['training_class'] for r in rows
                                                            if r['native_pixels']>=16)))
    components = []
    for line in ledger_path.read_text().splitlines():
        entry = json.loads(line)
        if entry['status']!='accepted_candidate':
            raise RuntimeError('Full5K canonical snapshot contains a nonaccepted row')
        audit = entry['audit'] if isinstance(entry['audit'],dict) else read(pin(pins,entry['audit']))
        for component in audit['components']:
            distance = component.get('range_ego_xy_m',component['range_xy_m'])
            if component['training_class'] in classes and 0<=distance<distance_max:
                components.append((entry['id'],component))
    return dict(source_analysis=str(analysis),source_ledger=str(ledger_path),
        source_ledger_sha256=report['source_sha256'],distance_bounds_m=[0,distance_max],
        classes=list(classes),counts=counts,by_camera_class=summaries(rows,metrics),
        components=component_summary(components,rows,metrics)), rows


def far_diagnostic(campaign, pins):
    rows, targets, seen, raw_roots = [], [], set(), set()
    ledgers = sorted(campaign.glob('chunk_*/capture/ledger.jsonl'))
    if not ledgers:
        raise RuntimeError('No initial far capture ledgers')
    for ledger in ledgers:
        for line in pin(pins,ledger).read_text().splitlines():
            entry = json.loads(line)
            if entry['id'] in seen:
                raise RuntimeError('Repeated initial far source; do not count retries as frames')
            seen.add(entry['id'])
            root = Path(entry['capture_root'])
            if root.resolve() in raw_roots:
                raise RuntimeError('Same raw capture root counted as several far sources')
            raw_roots.add(root.resolve())
            metadata = read(pin(pins,root/'metadata.json'))
            plan = metadata['spec']['secondary_far_vehicle']
            audit = entry['audit'] if isinstance(entry['audit'],dict) else read(pin(pins,entry['audit']))
            if audit != read(pin(pins,root/'audit.json')):
                raise RuntimeError('Far raw audit differs from its closed initial ledger')
            for filename in ('scenario.json','sensor_reference.json'):
                pin(pins,root/filename)
            selected = [c for c in audit['components'] if c['parent_uid']==plan['target_uid']]
            if len(selected)!=1 or selected[0]['training_class']!='vehicle':
                raise RuntimeError('Far target is not one native Vehicle component')
            component = selected[0]
            if (component['data_id']!=plan['target_data_id'] or component['asset']!=plan['target_name']):
                raise RuntimeError('Far native target identity differs from recorded plan')
            indices = []
            for evidence in component['cameras']:
                rect = evidence['native_rect']
                if evidence['native_instance_pixels']<=0 or rect[3]<=rect[1] or rect[2]<=rect[0]:
                    continue
                indices.append(len(rows))
                rows.append(dict(scene=entry['id'],capture_root=str(root),camera=evidence['camera'],
                    asset=component['asset'],training_class='vehicle',native_rect=rect,
                    camera_instance_id=evidence['camera_instance_id'],raw_class=evidence['annotation_class'],
                    native_pixels=evidence['native_instance_pixels']))
            targets.append(dict(scene=entry['id'],capture_root=str(root),source_status=entry['status'],
                lidar_points=component['lidar_points'],target_uid=plan['target_uid'],
                distance_m=component['range_ego_xy_m'],indices=indices))
    if len(targets)!=32:
        raise RuntimeError('Expected a complete initial far32 cohort, not a running/partial campaign')
    metrics, counts = measure(rows,pins)
    for target in targets:
        indices = target.pop('indices')
        for p in PROFILES:
            target[p['id']] = any(metrics[i][p['id']]['candidate_camera_gate'] for i in indices)
    return dict(source_campaign=str(campaign),initial_raw_only=True,retry_frames_counted=False,
        initial_sources=len(targets),source_status_counts=dict(Counter(t['source_status'] for t in targets)),
        counts=counts,by_camera_class=summaries(rows,metrics),zero_lidar=sum(t['lidar_points']==0 for t in targets),
        profiles={p['id']:dict(camera_gate=sum(t[p['id']] for t in targets),
            sensor_positive=sum(t['lidar_points']>=1 or t[p['id']] for t in targets),
            partial_camera_observations=sum(0<metrics[i][p['id']]['native_fraction']<1 for i in metrics)) for p in PROFILES},
        improved_observations=sum(metrics[i]['h576']['native_kept']>metrics[i]['h512']['native_kept'] for i in metrics),
        changed_camera_gates=[t for t in targets if t['h512']!=t['h576']],targets=targets), rows


def run(analysis, campaigns=(), classes=CLASSES, distance_max=5.):
    started = time.monotonic()
    analysis = Path(analysis).resolve()
    if not 0<distance_max<=5 or not classes or not set(classes)<=set(CLASSES):
        raise ValueError('Diagnostic scope is declared classes and at most the 0--5m cohort')
    pins = {}
    for path in (__file__,crop.__file__,ROOT/'dataset/morai_od/scripts/report_base5_camera_crop.py',ONLINE_SOURCE,SENSOR):
        pin(pins,path)
    shapes = shape_contract()
    near, source_rows = near_diagnostic(analysis,pins,classes,distance_max)
    far = []
    for campaign in campaigns:
        result, rows = far_diagnostic(Path(campaign).resolve(),pins)
        source_rows.extend(rows)
        far.append(result)
    verify_pins(pins)
    result = dict(schema='base5_crop_height_cpu_v1',passed=True,
        created_at_utc=datetime.now(timezone.utc).isoformat(),elapsed_sec=time.monotonic()-started,
        raw_and_labels_unchanged=True,production_unchanged=True,gpu_used=False,
        shape_contract=shapes,near=near,far=far,input_sha256=pins,limitations=LIMITATIONS)
    return result, {Path(r['capture_root']).resolve() for r in source_rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis',type=Path,default=DEFAULT_ANALYSIS)
    parser.add_argument('--far-campaign',type=Path,action='append',default=[])
    parser.add_argument('--classes',nargs='+',choices=CLASSES,default=list(CLASSES))
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError('Use a new diagnostic output; never overwrite evidence')
    for source in [args.analysis.resolve()]+[p.resolve() for p in args.far_campaign]:
        if output==source or source in output.parents:
            raise ValueError('Output must be separate from source analysis/raw campaign')
    report, raw_roots = run(args.analysis,args.far_campaign,args.classes)
    if any(output==root or root in output.parents for root in raw_roots):
        raise ValueError('Never write diagnostic output inside raw')
    output.mkdir(parents=True)
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'],output=str(output/'report.json'),
        near_counts=report['near']['counts'],far_cohorts=len(report['far']),
        elapsed_sec=report['elapsed_sec']),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
