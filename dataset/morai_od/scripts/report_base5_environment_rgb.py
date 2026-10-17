#!/usr/bin/env python3
"""Read-only, exact-payload environment RGB diagnosis; never a training export.

Only approved one-to-one static captures are compared. Source files are hashed
before and after analysis. New JSON and contact sheets are written only under a
fresh output directory. No ROS, simulator, network, GPU or training imports.
"""
import argparse
import ast
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw


SCHEMA = 'base5_environment_rgb_exact_static_v1'
CAMERAS = tuple('CAMERA_' + str(i) for i in range(1, 5))
LIMITATIONS = [
    'Diagnostic paired observations, not independent train/val samples; keep each geometry in one split.',
    'Declared environment and saved sensor reference do not measure actual UI time, weather or graphics settings.',
    'No renderer/exposure/shadow ground truth: shadow area is not inferred or certified.',
    'Small runtime pose, animation, renderer or graphics-setting changes can confound image differences.',
    'Instance masks establish visible annotation pixels, not detector accuracy or semantic correctness.',
    'Parent/raw-class camera matches may be ambiguous for multi-component assets; no invented 3D child association.',
    'Surrounding-ring contrast is a local brightness proxy, not perceptual detectability or AP.',
    'Q90 uses local OpenCV encoding; codec-version changes may produce small decoder differences.',
]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def ledger(path):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError('Empty selected ledger')
    for row in rows:
        if row.get('status') != 'accepted_candidate':
            raise ValueError('Only authoritative accepted_candidate selections are supported')
        for key in ('id', 'scenario_sha256', 'capture_root'):
            if not row.get(key):
                raise ValueError('Missing selected ' + key)
    for key in ('id', 'scenario_sha256', 'capture_root'):
        if len({str(row[key]) for row in rows}) != len(rows):
            raise ValueError('Ambiguous one-to-many ledger ' + key)
    return rows


def snapshot(paths):
    result = {}
    for path in sorted(set(map(Path, paths))):
        stat = path.stat()
        result[str(path.resolve())] = dict(sha256=sha256(path), size=stat.st_size,
            mtime_ns=stat.st_mtime_ns, inode=stat.st_ino)
    return result


def source_inventory(roots, ledgers):
    paths = set(ledgers)
    for root in roots:
        paths.update(path for path in root.rglob('*') if path.is_file())
    return paths


def only(root, pattern):
    paths = sorted(Path(root).glob(pattern))
    if len(paths) != 1:
        raise ValueError('Expected one synchronous ' + str(Path(root) / pattern))
    return paths[0]


def crop_model(image, mask=False):
    """Current deterministic per-camera width-relative cubic/nearest crop."""
    height, width = image.shape[:2]
    scale = .825 * 1280. / width
    dims = (int(width * scale), int(height * scale))
    if dims[0] != 1056 or dims[1] < 512:
        raise ValueError('Native image cannot produce current 512x1056 bottom crop')
    resized = cv2.resize(image.astype(np.uint8), dims,
        interpolation=cv2.INTER_NEAREST if mask else cv2.INTER_CUBIC)
    return resized[-512:]


def q90(image):
    ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise ValueError('JPEG Q90 encoding failed')
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if decoded is None:
        raise ValueError('JPEG Q90 decoding failed')
    return cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)


def luminance(image):
    return image.astype(np.float32) @ np.asarray([.2126, .7152, .0722], np.float32)


def distribution(values):
    values = np.asarray(values, dtype=np.float32)
    if values.size == 0:
        return None
    return dict(mean=float(values.mean()), p05=float(np.percentile(values, 5)),
        p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)))


def brightness(image, mask=None):
    y = luminance(image)
    selected = y.ravel() if mask is None else y[mask]
    pixels = image.reshape(-1, 3) if mask is None else image[mask]
    return dict(pixels=int(selected.size), luminance_0_255=distribution(selected),
        dark_fraction=float((selected <= 10).mean()) if selected.size else None,
        bright_fraction=float((selected >= 245).mean()) if selected.size else None,
        any_channel_clipped_fraction=float(((pixels <= 1) | (pixels >= 254)).any(axis=-1).mean())
            if selected.size else None)


def difference(before, after, mask=None):
    if before.shape != after.shape:
        raise ValueError('RGB shape mismatch; never resize native images to conceal it')
    delta = np.abs(before.astype(np.float32) - after.astype(np.float32))
    selected = delta.reshape(-1, 3) if mask is None else delta[mask]
    return dict(channel_absolute_error_0_255=distribution(selected),
        per_channel_mean=selected.mean(axis=0).tolist() if selected.size else None)


def local_contrast(image, mask, occupied):
    radius = max(2, min(24, int(round(math.sqrt(max(1, int(mask.sum()))) / 10))))
    ring = cv2.dilate(mask.astype(np.uint8), np.ones((radius * 2 + 1,) * 2, np.uint8)).astype(bool)
    ring &= ~occupied
    foreground = luminance(image)[mask]
    background = luminance(image)[ring]
    return dict(foreground=brightness(image, mask), ring_pixels=int(ring.sum()),
        ring_luminance_0_255=distribution(background), ring_radius_px=radius,
        foreground_minus_ring_mean=float(foreground.mean() - background.mean())
            if foreground.size and background.size else None)


def motion(meta, telemetry, row):
    actors = meta.get('runtime_actors', [])
    declared = [a.get('unique_id') for a in actors
        if a.get('motion') not in (None, 'static', 'stationary', 'standing')]
    marked = bool(row.get('audit', {}).get('motion_capture', {}).get('required'))
    velocities = []
    for key in ('ego_before', 'ego_after'):
        record = telemetry.get(key, meta.get('capture_runtime_ego', {}))
        if all(k in record for k in ('vx', 'vy', 'vz')):
            velocities.append(math.sqrt(sum(float(record[k]) ** 2 for k in ('vx', 'vy', 'vz'))))
    object_records = []
    for key in ('objects_before', 'objects_after'):
        value = telemetry.get(key, {})
        for group in ('npc_list', 'pedestrian_list', 'obstacle_list'):
            object_records.extend(value.get(group, []))
    for record in object_records:
        velocity = record.get('velocity', {})
        if all(k in velocity for k in ('x', 'y', 'z')):
            velocities.append(math.sqrt(sum(float(velocity[k]) ** 2 for k in ('x', 'y', 'z'))))
    return dict(declared_moving_uids=declared, audit_requires_motion=marked,
        maximum_observed_speed_mps=max(velocities) if velocities else None,
        runtime_velocity_observed=bool(velocities),
        static=not declared and not marked and (not velocities or max(velocities) <= .1))


def pose_delta(first, second):
    if not all(key in first and key in second for key in ('x', 'y', 'z', 'heading')):
        return dict(observed=False)
    return dict(observed=True,
        xy_m=math.hypot(float(first['x']) - float(second['x']), float(first['y']) - float(second['y'])),
        z_m=abs(float(first['z']) - float(second['z'])),
        heading_deg=abs((float(first['heading']) - float(second['heading']) + 180.) % 360. - 180.))


def object_pose_audit(first_meta, second_meta, first_telemetry, second_telemetry):
    def objects(meta, telemetry):
        records = telemetry.get('objects_before')
        if records is None:
            history = meta.get('capture_runtime_objects', [])
            records = history[0] if history else {}
        result = {}
        for group in ('npc_list', 'pedestrian_list', 'obstacle_list'):
            for record in records.get(group, []):
                uid = int(record['unique_id'])
                if uid in result:
                    raise ValueError('Duplicate runtime object UID')
                result[uid] = record
        return result
    first, second = objects(first_meta, first_telemetry), objects(second_meta, second_telemetry)
    result = []
    for uid in sorted(set(first) | set(second)):
        rows = [mapping.get(uid) for mapping in (first, second)]
        observed = all(row is not None for row in rows)
        item = dict(parent_uid=uid, observed_in_both=observed)
        if observed:
            poses = [dict(row.get('position', {}), heading=row.get('heading')) for row in rows]
            if all(all(value.get(k) is not None for k in ('x', 'y', 'z', 'heading')) for value in poses):
                item['pose_delta'] = pose_delta(*poses)
            item['name_equal'] = rows[0].get('name') == rows[1].get('name')
            item['size_equal'] = rows[0].get('size') == rows[1].get('size')
        result.append(item)
    return dict(objects=result, runtime_object_records_observed=bool(first or second),
        limitation='Receive-time Object_topic witnesses, not exactly synchronized SaveSensorData geometry.')


def sensor_geometry(sensor):
    result = []
    for group in ('cameraList', 'Lidar3DList', 'Lidar2DList', 'lidarList'):
        for item in sensor.get(group, []):
            cc = item.get('cc', {})
            result.append(dict(group=group, uid=item.get('m_SensorUniqueID'), pos=item.get('pos'),
                rot=item.get('rot'), parameters={key:value for key, value in cc.items()
                    if key not in ('udpConfig', 'rosConfig', 'serialConfig', 'canConfig', 'bboxROSConfig')}))
    return result


def annotations(folder, meta, row):
    instance_path = only(folder, '*_Instance.png')
    with Image.open(instance_path) as image:
        rgba = np.asarray(image.convert('RGBA'))
    annotation_paths = sorted(Path(folder).glob('*_BBox2D.txt'))
    if len(annotation_paths) > 1:
        raise ValueError('Multiple synchronous BBox2D files')
    if not annotation_paths:
        # Native MORAI legitimately omits BBox text when a view has no actor.
        # This is not a blanket missing-GT waiver: opaque black Ego/background
        # and opaque white unassigned background are the only allowed colours.
        black = (rgba == np.asarray([0, 0, 0, 255])).all(axis=2)
        white = (rgba == np.asarray([255, 255, 255, 255])).all(axis=2)
        camera = Path(folder).name
        positive_witness = any(c.get('camera') == camera and c.get('native_instance_pixels', 0) > 0
            for component in row.get('audit', {}).get('components', []) for c in component.get('cameras', []))
        if not (black | white).all() or positive_witness:
            raise ValueError('BBox2D missing despite foreground Instance/audit evidence')
        return [], np.zeros(rgba.shape[:2], dtype=bool)
    annotation_path = annotation_paths[0]
    actor_uids = {int(a['unique_id']) for a in meta.get('runtime_actors', [])}
    counts = Counter((int(c['parent_uid']), c['raw_class']) for c in row.get('audit', {}).get('components', []))
    entries = [];occurrences = Counter();colours = defaultdict(set)
    for line in annotation_path.read_text().splitlines():
        if not line.strip():
            continue
        fields = line.split(maxsplit=6)
        if len(fields) != 7:
            raise ValueError('Malformed native BBox2D record')
        candidate = int(fields[5]);parent = candidate if candidate >= 0 else abs(candidate) // 100
        if parent not in actor_uids:
            continue
        colour = tuple(ast.literal_eval(fields[6]))
        if len(colour) != 4 or any(not isinstance(v, int) or not 0 <= v <= 255 for v in colour):
            raise ValueError('Invalid native instance RGBA')
        base = (fields[0], candidate);ordinal = occurrences[base];occurrences[base] += 1
        key = '|'.join(map(str, (*base, ordinal)))
        colours[colour].add(key)
        entries.append(dict(key=key, parent_uid=parent, raw_class=fields[0], camera_instance_id=candidate,
            record_ordinal=ordinal, native_rect=list(map(float, fields[1:5])), rgba=list(colour),
            same_parent_raw_class_3d_components=counts[(parent, fields[0])],
            camera_component_ambiguous=candidate >= 0 and counts[(parent, fields[0])] > 1,
            mask=(rgba == np.asarray(colour)).all(axis=2)))
    occupied = np.zeros(rgba.shape[:2], dtype=bool)
    for entry in entries:
        entry['rgba_collision'] = len(colours[tuple(entry['rgba'])]) > 1
        occupied |= entry['mask']
    return entries, occupied


def camera_report(first_root, second_root, camera, first_meta, second_meta, first_row, second_row):
    roots = (first_root / camera, second_root / camera)
    paths = [only(root, '*_Intensity.png') for root in roots]
    for root, path in zip(roots, paths):
        stamp = path.name[:-len('_Intensity.png')]
        for suffix in ('_Instance.png', '_Depth.png'):
            if only(root, '*' + suffix).name != stamp + suffix:
                raise ValueError('Asynchronous native camera annotation filename')
        boxes = sorted(root.glob('*_BBox2D.txt'))
        if len(boxes) > 1 or (boxes and boxes[0].name != stamp + '_BBox2D.txt'):
            raise ValueError('Asynchronous native camera annotation filename')
    images = []
    for path in paths:
        with Image.open(path) as image:
            images.append(np.asarray(image.convert('RGB')))
    if images[0].shape != images[1].shape:
        raise ValueError('Native camera dimensions changed: ' + camera)
    jpeg = [q90(image) for image in images]
    model = [crop_model(image) for image in jpeg]
    states = [annotations(roots[i], meta, row) for i, (meta, row) in enumerate(
        ((first_meta, first_row), (second_meta, second_row)))]
    entry_maps = [{entry['key']:entry for entry in state[0]} for state in states]
    foreground = []
    for key in sorted(set(entry_maps[0]) | set(entry_maps[1])):
        entries = [mapping.get(key) for mapping in entry_maps]
        record = dict(key=key, matched=all(entry is not None for entry in entries), observations=[])
        for i, entry in enumerate(entries):
            if entry is None:
                record['observations'].append(None);continue
            mask = entry['mask'];cropped_mask = crop_model(mask, mask=True).astype(bool)
            item = {k:v for k,v in entry.items() if k != 'mask'}
            item.update(native_pixels=int(mask.sum()), model_pixels=int(cropped_mask.sum()),
                native=local_contrast(images[i], mask, states[i][1]),
                jpeg90=local_contrast(jpeg[i], mask, states[i][1]),
                model512x1056=local_contrast(model[i], cropped_mask,
                    crop_model(states[i][1], mask=True).astype(bool)))
            record['observations'].append(item)
        if record['matched']:
            union = entries[0]['mask'] | entries[1]['mask']
            intersection = entries[0]['mask'] & entries[1]['mask']
            record.update(mask_iou=float(intersection.sum() / union.sum()) if union.any() else None,
                native_foreground_intersection_difference=difference(images[0], images[1], intersection),
                identity_warning='Camera annotation identity preserved; ambiguous records are not joined to invented 3D children.')
        foreground.append(record)
    result = dict(camera=camera, native_shape=list(images[0].shape), native_paths=list(map(str, paths)),
        native_png_byte_identical=sha256(paths[0]) == sha256(paths[1]), domains={}, foreground=foreground,
        bbox2d_files_present=[bool(list(root.glob('*_BBox2D.txt'))) for root in roots],
        missing_bbox_policy='Allowed only for foreground-free opaque black/white Instance and no positive camera-audit witness.')
    for domain, arrays in (('native_png', images), ('jpeg90_native', jpeg), ('jpeg90_model512x1056', model)):
        result['domains'][domain] = dict(baseline=brightness(arrays[0]), paired=brightness(arrays[1]),
            difference=difference(*arrays))
    result['jpeg90_recompression_error'] = [difference(images[i], jpeg[i]) for i in (0, 1)]
    return result, images, model


def native_hash_comparison(first_root, second_root):
    def files(root):
        result = {}
        for folder in (*CAMERAS, 'LIDAR_5'):
            directory = root / folder
            for path in sorted(directory.glob('*')):
                if path.is_file():
                    name = path.name.split('_', 1)[-1]
                    key = folder + '/' + name
                    if key in result:
                        raise ValueError('Multiple synchronous native files for ' + key)
                    result[key] = sha256(path)
        return result
    first, second = files(first_root), files(second_root)
    return [dict(native_kind=key, baseline_sha256=first.get(key), paired_sha256=second.get(key),
        byte_identical=key in first and key in second and first[key] == second[key])
        for key in sorted(set(first) | set(second))]


def contact_page(observations, output, suffix, page_number):
    tile_width = 240;tile_height = 160
    canvas = Image.new('RGB', (tile_width * 8, 34 + len(observations) * tile_height), '#17202b')
    draw = ImageDraw.Draw(canvas)
    draw.text((8, 8), 'Exact static RGB pairs / DIAGNOSTIC ONLY / ' + suffix, fill='white')
    for row, (ident, before, after) in enumerate(observations):
        for camera in range(4):
            for side, images in enumerate((before, after)):
                x = (camera * 2 + side) * tile_width;y = 34 + row * tile_height
                draw.text((x + 4, y + 2), ident[-16:] + ' ' + ('B' if side == 0 else 'P') + ' C' + str(camera + 1), fill='white')
                image = Image.fromarray(images[camera]);image.thumbnail((tile_width, tile_height - 22))
                canvas.paste(image, (x, y + 20))
    name = 'contact_sheet_' + suffix + ('' if page_number == 1 else '_' + str(page_number).zfill(3)) + '.png'
    path = output / name;canvas.save(path)
    return str(path)


def aggregate(pairs):
    results = {}
    for camera in CAMERAS:
        records = [next(c for c in pair['cameras'] if c['camera'] == camera) for pair in pairs]
        domains = {}
        for domain in ('native_png', 'jpeg90_native', 'jpeg90_model512x1056'):
            domains[domain] = dict(
                per_pair_rgb_mae_0_255=distribution([r['domains'][domain]['difference']['channel_absolute_error_0_255']['mean'] for r in records]),
                per_pair_paired_minus_baseline_luminance=distribution([
                    r['domains'][domain]['paired']['luminance_0_255']['mean'] -
                    r['domains'][domain]['baseline']['luminance_0_255']['mean'] for r in records]))
        foreground = [f for record in records for f in record['foreground'] if f['matched']]
        results[camera] = dict(pairs=len(records), domains=domains,
            matched_camera_records=len(foreground),
            any_ambiguous_camera_records=sum(any(o['camera_component_ambiguous'] or o['rgba_collision'] for o in f['observations']) for f in foreground),
            mask_iou=distribution([f['mask_iou'] for f in foreground if f['mask_iou'] is not None]),
            meaning='Equal per-pair statistics; component records are not independent layout samples.')
    return results


def build(baseline, paired, output):
    baseline = Path(baseline).resolve(strict=True);paired = Path(paired).resolve(strict=True)
    # Resolve existing parent symlinks before checking the immutable raw roots.
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh report output required')
    before_rows, after_rows = ledger(baseline), ledger(paired)
    before = {r['scenario_sha256']:r for r in before_rows}
    after = {r['scenario_sha256']:r for r in after_rows}
    common = sorted(set(before) & set(after))
    if not common:
        raise ValueError('No exact scenario_sha256 pairs')
    roots = {Path(row['capture_root']).resolve(strict=True) for row in before_rows + after_rows}
    if any(root == output or root in output.parents for root in roots):
        raise ValueError('Output must not be inside a source capture')
    sources = source_inventory(roots, (baseline, paired))
    original = snapshot(sources)
    pairs = [];excluded = [];pages = [];native_tiles = [];model_tiles = []
    # Output creation occurs after all immutable input paths have been recorded.
    output.mkdir(parents=True)
    for digest in common:
        rows = (before[digest], after[digest]);capture_roots = tuple(Path(r['capture_root']).resolve() for r in rows)
        if capture_roots[0] == capture_roots[1]:
            raise ValueError('A capture cannot be its own independent paired observation')
        for root in capture_roots:
            if sha256(root / 'scenario.json') != digest:
                raise ValueError('Saved exact scenario payload hash mismatch')
        metas = [read(root / 'metadata.json') for root in capture_roots]
        telemetry = [read(root / 'telemetry.json') if (root / 'telemetry.json').is_file() else {} for root in capture_roots]
        motions = [motion(meta, telem, row) for meta, telem, row in zip(metas, telemetry, rows)]
        if not all(state['static'] for state in motions):
            excluded.append(dict(scenario_sha256=digest, ids=[r['id'] for r in rows], reason='not_static', motion=motions));continue
        sensors = [read(root / 'sensor_reference.json') if (root / 'sensor_reference.json').is_file() else None for root in capture_roots]
        ego = [telem.get('ego_before', meta.get('capture_runtime_ego', {})) for meta, telem in zip(metas, telemetry)]
        sensor_equal = all(s is not None for s in sensors) and sensor_geometry(sensors[0]) == sensor_geometry(sensors[1])
        delta = pose_delta(*ego)
        pair = dict(scenario_sha256=digest, ids=[r['id'] for r in rows], capture_roots=list(map(str, capture_roots)),
            exact_payload=True, motion=motions, sensor_reference_observed=all(s is not None for s in sensors),
            sensor_geometry_equal=sensor_equal, runtime_ego_pose_delta=delta,
            runtime_object_pose_audit=object_pose_audit(*metas, *telemetry),
            source_geometry_environment=[meta.get('fresh_environment_provenance', {}).get('environment') for meta in metas],
            environment_label_contract='Source geometry metadata is not the physical capture time; paired UI environment is operator-attested externally.',
            native_file_comparison=native_hash_comparison(*capture_roots), cameras=[],
            photometric_attribution_qualified=False,
            photometric_attribution_limit='Actual UI/graphics not independently measured; inspect pose/sensor/mask differences.')
        images = ([], []);models = ([], [])
        for camera in CAMERAS:
            report, native, model = camera_report(*capture_roots, camera, *metas, *rows)
            pair['cameras'].append(report)
            for i in (0, 1):
                images[i].append(native[i]);models[i].append(model[i])
        pairs.append(pair)
        native_tiles.append((rows[1]['id'], *images));model_tiles.append((rows[1]['id'], *models))
        if len(native_tiles) == 4:
            number = len(pages) // 2 + 1
            pages.extend((contact_page(native_tiles, output, 'native', number), contact_page(model_tiles, output, 'model512', number)))
            native_tiles = [];model_tiles = []
    if native_tiles:
        number = len(pages) // 2 + 1
        pages.extend((contact_page(native_tiles, output, 'native', number), contact_page(model_tiles, output, 'model512', number)))
    if sources != source_inventory(roots, (baseline, paired)):
        raise RuntimeError('Source inventory changed during report; report is not qualified')
    final = snapshot(sources)
    if original != final:
        raise RuntimeError('Source files changed during report; report is not qualified')
    if sources != source_inventory(roots, (baseline, paired)):
        raise RuntimeError('Source inventory changed during report; report is not qualified')
    result = dict(schema=SCHEMA, report_script_sha256=sha256(Path(__file__)),
        baseline_ledger=str(baseline), paired_ledger=str(paired),
        exact_common_payloads=len(common), compared_static_pairs=len(pairs), excluded=excluded,
        baseline_only_payloads=sorted(set(before) - set(after)), paired_only_payloads=sorted(set(after) - set(before)),
        paired_observations_are_independent_training_samples=False, raw_modified=False,
        source_snapshot_unchanged=True, source_inventory_unchanged=True,
        sources=original, contact_sheets=pages, pairs=pairs,
        aggregate_by_camera=aggregate(pairs), limitations=LIMITATIONS, model_transform=dict(width=1056, height=512, resize='OpenCV cubic; width-relative .825*1280/native_width',
            crop='deterministic bottom, no training random augmentation', mask_resize='nearest', jpeg_quality=90))
    (output / 'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--paired', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = build(args.baseline, args.paired, args.output)
    print(json.dumps({k:result[k] for k in ('compared_static_pairs', 'exact_common_payloads', 'source_snapshot_unchanged', 'contact_sheets')}, indent=2))


if __name__ == '__main__':
    main()
