#!/usr/bin/env python3
"""CPU-only image-domain evidence; no model, ROS master, or locked bags.

Different-scene image statistics cannot identify motion blur/exposure/noise.
JPEG tables are compared exactly against this OpenCV encoder, not guessed
from the compressed file size. Only developer-selected bags may be supplied.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
from io import BytesIO
import json
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image, JpegImagePlugin


def quantization_signature(image):
    return json.dumps({str(k): list(v) for k, v in
                       getattr(image, 'quantization', {}).items()}, sort_keys=True)


def quality_reference():
    result = defaultdict(list)
    for quality in range(1, 101):
        ok, encoded = cv2.imencode('.jpg', np.zeros((16, 16, 3), np.uint8),
                                   [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError('JPEG encoder failed')
        result[quantization_signature(Image.open(BytesIO(encoded.tobytes())))].append(quality)
    return dict(result)


def codec_metadata(payload, reference):
    with Image.open(BytesIO(payload)) as image:
        signature = quantization_signature(image)
        return dict(codec=image.format, width=image.width, height=image.height,
                    quantization_sha256=hashlib.sha256(signature.encode()).hexdigest()
                    if image.format == 'JPEG' else None,
                    opencv_quality_matches=reference.get(signature, [])
                    if image.format == 'JPEG' else [],
                    sampling=JpegImagePlugin.get_sampling(image)
                    if image.format == 'JPEG' else None)


def model_crop(image):
    height, width = image.shape[:2]
    scale = 1056.0 / width
    resized = cv2.resize(image, (int(width * scale), int(height * scale)),
                         interpolation=cv2.INTER_CUBIC)
    if resized.shape[0] < 512:
        raise ValueError('Unexpected source aspect ratio')
    return resized[-512:, :1056]


def image_statistics(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # The bottom half excludes much of the sky, but is NOT a semantic road mask.
    lower = gray[gray.shape[0] // 2:]
    return dict(gray_mean=float(gray.mean()), lower_gray_mean=float(lower.mean()),
                dark_fraction=float((gray <= 5).mean()),
                bright_fraction=float((gray >= 250).mean()),
                laplacian_variance=float(cv2.Laplacian(lower, cv2.CV_32F).var()))


def difference(before, after):
    delta = before.astype(np.float32) - after.astype(np.float32)
    mse = float(np.mean(delta * delta))
    return dict(mae=float(np.mean(np.abs(delta))),
                psnr_db=float(10 * np.log10(255 ** 2 / mse)) if mse else None,
                lower_laplacian_ratio=(image_statistics(after)['laplacian_variance'] /
                    max(1.e-8, image_statistics(before)['laplacian_variance'])))


def summarize(rows):
    keys = sorted({k for row in rows for k in row})
    return {key: dict(zip(('p10', 'p50', 'p90', 'mean'),
                          map(float, [*np.percentile(values, [10, 50, 90]), np.mean(values)])))
            for key in keys if (values := [row[key] for row in rows if row.get(key) is not None])}


def audit_bag(path, count, reference):
    if 'evaluation_locked' in path.parts:
        raise ValueError('Locked evaluation bags cannot tune augmentation')
    import rosbag
    with rosbag.Bag(str(path)) as bag:
        info = bag.get_type_and_topic_info().topics
        topics = {k: v for k, v in info.items()
                  if v.msg_type == 'sensor_msgs/CompressedImage' and '/sensor/camera/' in k}
        if len(topics) != 4:
            raise ValueError('Expected exactly four raw camera topics: ' + str(path))
        indices = {k: set(np.linspace(0, v.message_count - 1,
                       min(count, v.message_count), dtype=int).tolist()) for k, v in topics.items()}
        counters = Counter()
        records = []
        for topic, message, timestamp in bag.read_messages(topics=list(topics)):
            index = counters[topic]
            counters[topic] += 1
            if index not in indices[topic]:
                continue
            payload = bytes(message.data)
            image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Invalid camera payload')
            records.append(dict(topic=topic, message_index=index, bag_time=timestamp.to_sec(),
                payload_sha256=hashlib.sha256(payload).hexdigest(),
                codec=codec_metadata(payload, reference), statistics=image_statistics(model_crop(image))))
    return dict(path=str(path.resolve()), bytes=path.stat().st_size,
                mtime_ns=path.stat().st_mtime_ns, samples=records)


def audit_native(ledger, count, reference):
    payload = ledger.read_bytes()
    by_family = defaultdict(list)
    for line in payload.splitlines():
        row = json.loads(line)
        by_family[row['family']].append(row)
    rng = np.random.RandomState(261010)
    records = []
    for family, rows in sorted(by_family.items()):
        rows.sort(key=lambda r: hashlib.sha256(r['id'].encode()).hexdigest())
        for row in rows[:count]:
            for camera in range(1, 5):
                files = list((Path(row['capture_root']) / ('CAMERA_' + str(camera))).glob('*_Intensity.png'))
                if len(files) != 1:
                    raise ValueError('Native camera file mismatch')
                path = files[0]
                raw = path.read_bytes()
                image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                before = model_crop(image)
                start = time.perf_counter()
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not ok:
                    raise RuntimeError('JPEG encode failed')
                decoded = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
                jpeg_ms = (time.perf_counter() - start) * 1000
                after = model_crop(decoded)
                # Legacy placement: compression AFTER resizing/cropping.
                ok, late = cv2.imencode('.jpg', before, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not ok:
                    raise RuntimeError('JPEG encode failed')
                late = cv2.imdecode(late, cv2.IMREAD_COLOR)
                blur = model_crop(cv2.GaussianBlur(image, (3, 3), 0))
                gain = model_crop(np.clip(image.astype(float) * 1.03, 0, 255).astype(np.uint8))
                noise = model_crop(np.clip(image.astype(float) + rng.normal(0, 1, image.shape), 0, 255).astype(np.uint8))
                records.append(dict(id=row['id'], family=family, camera=camera,
                    path=str(path), payload_sha256=hashlib.sha256(raw).hexdigest(),
                    codec=codec_metadata(raw, reference), statistics=image_statistics(before),
                    jpeg90_native=difference(before, after), jpeg90_late=difference(before, late),
                    native_vs_late=difference(after, late), blur3=difference(before, blur),
                    gain103=difference(before, gain), noise_std1=difference(before, noise),
                    jpeg_encode_decode_ms=jpeg_ms))
    return dict(ledger=str(ledger.resolve()), ledger_sha256=hashlib.sha256(payload).hexdigest(), samples=records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, action='append', required=True)
    parser.add_argument('--ledger', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bag-samples-per-camera', type=int, default=32)
    parser.add_argument('--native-samples-per-family', type=int, default=24)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Use a fresh evidence output')
    if min(args.bag_samples_per_camera, args.native_samples_per_family) < 1:
        raise ValueError('Positive sample counts required')
    cv2.setNumThreads(1)
    reference = quality_reference()
    bags = []
    for path in args.bag:
        bags.append(audit_bag(path, args.bag_samples_per_camera, reference))
        print(json.dumps(dict(bag=str(path), samples=len(bags[-1]['samples']))), flush=True)
    native = audit_native(args.ledger, args.native_samples_per_family, reference)
    bag_samples = [r for b in bags for r in b['samples']]
    native_samples = native['samples']
    summary = dict(bag_count=len(bags), bag_image_samples=len(bag_samples),
        native_image_samples=len(native_samples), native_scenes=len({r['id'] for r in native_samples}),
        bag_codecs=dict(Counter(r['codec']['codec'] for r in bag_samples)),
        bag_quality_matches=dict(Counter(str(r['codec']['opencv_quality_matches']) for r in bag_samples)),
        bag_sampling=dict(Counter(str(r['codec']['sampling']) for r in bag_samples)),
        native_codecs=dict(Counter(r['codec']['codec'] for r in native_samples)),
        perturbations={key: summarize([r[key] for r in native_samples]) for key in
                       ('jpeg90_native', 'jpeg90_late', 'native_vs_late', 'blur3', 'gain103', 'noise_std1')},
        jpeg_cpu_ms=summarize([dict(milliseconds=r['jpeg_encode_decode_ms']) for r in native_samples]),
        statistics_by_domain_camera={domain: {str(camera): summarize([r['statistics'] for r in rows
            if (r.get('camera') == camera if domain == 'native' else r['topic'].split('/')[-2] == camera)])
            for camera in (range(1, 5) if domain == 'native' else ('front','front_left','front_right','back'))}
            for domain, rows in (('native', native_samples), ('bag', bag_samples))})
    result = dict(version=1, cv2_version=cv2.__version__, bags=bags, native=native, summary=summary,
        limitation='Non-paired scenes: histograms/edge statistics do not identify physical blur, noise, or exposure; no model accuracy measurement.',
        sampling='Time-uniform per bag camera; hash-ranked fixed quota per native family. No locked evaluation bags.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
