#!/usr/bin/env python3
"""Read-only model ablation on a known-failed-pose development matrix.

Native identity/state must bracket each unchanged JPEG. The supplied manually
reviewed housing rectangle is a development correspondence reference, not an
independent human annotation. Does not publish ROS, train, or change defaults.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys
import time

from ablate_traffic_light_near_detector_scales import ROOT, digest, distribution, predicted_boxes
from traffic_light_yolo_dataset import iou

ENVIRONMENTS = ('sunny13', 'sunny11', 'sunny15', 'foggy13', 'foggy11', 'foggy15')
BIT_VALUES = (1, 4, 16, 32)


def native_truth(frame):
    gt = frame['native_gt']
    if not frame.get('native_GT_verified') or not gt.get('jpeg_receipt_bracketed'):
        raise ValueError('Native truth must bracket the JPEG')
    head = gt['observed_native_id']
    state = gt['observed_state']
    for when in ('before', 'after'):
        check = gt[when]
        observations = check['readback']['observations']
        if not check['verified'] or not check['readback']['readback_success']:
            raise ValueError('Native bracket is not verified')
        if not any(row['tl_id'] == head and row['tl_color'] == state
                   and row['identity_matches_request'] and row['color_usable_for_gt']
                   for row in observations):
            raise ValueError('Native brackets disagree with observed truth')
    if state & ~sum(BIT_VALUES):
        raise ValueError('Unsupported native bitmask')
    return head, state, [int(bool(state & bit)) for bit in BIT_VALUES]


def runtime_crop(image, box, margin=12):
    h, w = image.shape[:2]
    x1, y1, x2, y2 = box
    bounds = [max(0, int(x1)-margin), max(0, int(y1)-margin),
              min(w, int(x2)+margin), min(h, int(y2)+margin)]
    a, b, c, d = bounds
    return image[b:d, a:c], bounds


def aggregate(rows):
    output = {'frames': len(rows)}
    for mode in ('960', '640', 'manual_reference_crop'):
        samples = [row['outcomes'][mode] for row in rows]
        accepted = [sample for sample in samples if sample['accepted']]
        known = [sample for sample in accepted if sample['cnn']['known']]
        output[mode] = dict(accepted=len(accepted), cnn_known=len(known),
                            cnn_exact_native=sum(sample['cnn_exact_native'] for sample in known),
                            score=distribution([sample['confidence'] for sample in samples]),
                            known_wrong=sum(not sample['cnn_exact_native'] for sample in known))
    return output


def run(matrix, output, weights, cnn, bbox, scope='known_failed_pose_development', frozen_plan=None):
    import cv2
    import torch
    from ultralytics import YOLO
    from train_traffic_light_yolo import offline_runtime
    sys.path.insert(0, str(ROOT/'base_ws/src/perception/traffic_light_detection/scripts'))
    from traffic_light_cnn_classifier import TrafficLightCnnClassifier

    output = output.resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists():
        raise FileExistsError('Fresh evidence directory required')
    if scope == 'prospective_new_pose_holdout' and frozen_plan is None:
        raise ValueError('New-pose holdout requires the prospectively frozen plan')
    plan_metadata = None
    if frozen_plan is not None:
        plan = json.loads(frozen_plan.read_text())
        if (plan['primary']['imgsz'], plan['primary']['confidence'],
                plan['retry']['imgsz'], plan['retry']['confidence'],
                plan['classifier']['crop_margin_px'], plan['classifier']['min_bit_confidence']) != (
                    960, .25, 640, .25, 12, .6):
            raise ValueError('Frozen plan differs from unchanged evaluation settings')
        plan_metadata = dict(path=str(frozen_plan.resolve()), sha256=digest(frozen_plan),
                             contents=plan)
    sources, frames, excluded = [], [], []
    for environment in ENVIRONMENTS:
        source = matrix/environment/'manifest.json'
        document = json.loads(source.read_text())
        sources.append(dict(path=str(source.resolve()), sha256=digest(source)))
        for frame in document['frames']:
            if not frame.get('accepted_for_sample_quota'):
                excluded.append(dict(source=str(source.resolve()), image_path=frame['image_path'],
                                     reason='source collector rejected attempt; never used as GT'))
                continue
            head, state, bits = native_truth(frame)
            image_path = source.parent/frame['image_path']
            if digest(image_path) != frame['sha256']:
                raise ValueError('Original JPEG digest differs')
            frames.append(dict(environment=environment, image_path=str(image_path.resolve()),
                               image_sha256=frame['sha256'], observed_native_id=head,
                               observed_native_state=state, expected_bits=bits))
    output.mkdir(parents=True)
    rows, timings = [], defaultdict(list)
    with offline_runtime():
        model = YOLO(str(weights))
        classifier = TrafficLightCnnClassifier(cnn, device='cuda:0', min_bit_confidence=.6,
                                                expected_sha256=digest(cnn))
        first = cv2.imread(frames[0]['image_path'])
        for size in (960, 640):
            model.predict(first, imgsz=size, conf=.001, device=0, verbose=False)
        with (output/'predictions.jsonl').open('x') as stream:
            for index, frame in enumerate(frames):
                image = cv2.imread(frame['image_path'])
                outcomes, raw_predictions = {}, {}
                for size in ((960, 640) if index % 2 == 0 else (640, 960)):
                    torch.cuda.synchronize()
                    started = time.monotonic()
                    result = model.predict(image, imgsz=size, conf=.001, device=0, verbose=False)[0]
                    torch.cuda.synchronize()
                    timings[str(size)].append((time.monotonic()-started)*1000)
                    predictions = predicted_boxes(result)
                    raw_predictions[str(size)] = predictions
                    matches = [pred for pred in predictions if iou(pred['xyxy'], bbox) >= .5]
                    target = max(matches, key=lambda pred: pred['confidence'], default=None)
                    accepted = target is not None and target['confidence'] >= .25
                    item = dict(accepted=accepted, confidence=target['confidence'] if target else 0.,
                                matched_bbox=target['xyxy'] if target else None,
                                matched_IoU=iou(target['xyxy'], bbox) if target else None, cnn=None)
                    if accepted:
                        crop, bounds = runtime_crop(image, target['xyxy'])
                        observation = classifier.classify_bgr(crop)
                        item.update(cnn=observation.to_dict(), crop_xyxy=bounds,
                                    cnn_exact_native=list(observation.bits) == frame['expected_bits'])
                    outcomes[str(size)] = item
                crop, bounds = runtime_crop(image, bbox)
                observation = classifier.classify_bgr(crop)
                outcomes['manual_reference_crop'] = dict(
                    accepted=True, confidence=1., crop_xyxy=bounds, cnn=observation.to_dict(),
                    cnn_exact_native=list(observation.bits) == frame['expected_bits'])
                row = dict(frame, predictions=raw_predictions, outcomes=outcomes)
                rows.append(row)
                stream.write(json.dumps(row, allow_nan=False)+'\n')
                print('processed %d/%d' % (index+1, len(frames)), flush=True)
    scope_limit = ('Known failed pose used to develop policy: NOT an independent test or new-pose holdout.'
                   if scope == 'known_failed_pose_development' else
                   'Prospective new-pose component holdout after policy freeze; six environments and phases '
                   'share each pose and are correlated. The planned two poses are not a broad independent route test.')
    report = dict(schema='traffic_light_near_scale_and_cnn_v2', evaluation_scope=scope,
                  frozen_plan=plan_metadata,
                  source_manifests=sources, excluded_source_attempts=excluded,
                  detector_sha256=digest(weights), cnn_sha256=digest(cnn),
                  script_sha256=digest(__file__), manual_reference_bbox_xyxy=bbox,
                  manual_bbox_provenance='root agent independent visual inspection; not detector-derived',
                  confidence_threshold=.25, target_correspondence_IoU=.5,
                  crop_margin_px=12, cnn_min_bit_confidence=.6,
                  overall=aggregate(rows),
                  by_environment={env: aggregate([row for row in rows if row['environment'] == env])
                                  for env in ENVIRONMENTS},
                  by_native_state={str(state): aggregate([row for row in rows if row['observed_native_state'] == state])
                                   for state in sorted({row['observed_native_state'] for row in rows})},
                  timings_decoded_BGR_detector_ms={mode: distribution(values) for mode, values in timings.items()},
                  limitations=[
                      scope_limit,
                      'Only target housing correspondence is annotated: no full-image false-positive precision.',
                      'Native lamp bitmask is bracketed; manually supplied target box is not independent human GT.',
                      'CNN bit accuracy does not certify mission association, signal policy or real driving.',
                      'Component comparison does not evaluate the map width/depth gate or ambiguity veto implementation.',
                      'Decoded BGR detector timing is not full ROS pipeline latency.'])
    (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report['overall'], indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--cnn', type=Path, required=True)
    parser.add_argument('--bbox', type=float, nargs=4, required=True)
    parser.add_argument('--scope', choices=('known_failed_pose_development', 'prospective_new_pose_holdout'),
                        default='known_failed_pose_development')
    parser.add_argument('--frozen-plan', type=Path)
    run(**vars(parser.parse_args()))
