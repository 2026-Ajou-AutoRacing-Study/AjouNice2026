"""nuScenes-style detection metrics for MORAI ego-frame 3D boxes.

This module intentionally has no dependency on the nuScenes database or
devkit.  It follows the nuScenes detection matching/AP interpolation rules,
but evaluates the three MORAI classes directly in the LiDAR/ego frame.  When
both GT and predictions contain planar velocity, the same TP matches also
report AVE and scalar speed MAE. NDS remains unavailable because MORAI has no
compatible attribute ground truth.
"""

from dataclasses import dataclass

import numpy as np


DEFAULT_CENTER_THRESHOLDS = (0.5, 1.0, 2.0, 4.0)
DEFAULT_DISTANCE_BINS = ((0.0, 10.0), (10.0, 20.0),
                         (20.0, 30.0), (30.0, 50.0))
DEFAULT_IOU_THRESHOLDS = (0.25, 0.30, 0.50)


@dataclass
class MetricData:
    recall: np.ndarray
    precision: np.ndarray
    confidence: np.ndarray
    translation_error: np.ndarray
    scale_error: np.ndarray
    orientation_error: np.ndarray
    velocity_error: np.ndarray
    speed_error: np.ndarray
    velocity_available: bool = False

    @property
    def max_recall_index(self):
        nonzero = np.nonzero(self.confidence)[0]
        return int(nonzero[-1]) if len(nonzero) else 0


def _empty_metric_data(velocity_available=False):
    recall = np.linspace(0.0, 1.0, 101)
    return MetricData(
        recall=recall,
        precision=np.zeros(101),
        confidence=np.zeros(101),
        translation_error=np.ones(101),
        scale_error=np.ones(101),
        orientation_error=np.ones(101),
        velocity_error=np.ones(101),
        speed_error=np.ones(101),
        velocity_available=bool(velocity_available),
    )


def _cummean(values):
    values = np.asarray(values, dtype=np.float64)
    if len(values) == 0:
        return values
    return np.cumsum(values) / np.arange(1, len(values) + 1)


def _scale_error(gt_size, pred_size):
    """nuScenes ASE primitive: 1 - aligned 3D size IoU."""
    gt_size = np.maximum(np.asarray(gt_size, dtype=np.float64), 1e-8)
    pred_size = np.maximum(np.asarray(pred_size, dtype=np.float64), 1e-8)
    intersection = np.prod(np.minimum(gt_size, pred_size))
    union = np.prod(gt_size) + np.prod(pred_size) - intersection
    return float(1.0 - intersection / max(union, 1e-8))


def _orientation_error(gt_yaw, pred_yaw, period=2.0 * np.pi):
    difference = (pred_yaw - gt_yaw + period / 2.0) % period - period / 2.0
    return float(abs(difference))


def _normalize_result(result):
    if "pts_bbox" in result:
        result = result["pts_bbox"]
    elif "img_bbox" in result:
        result = result["img_bbox"]
    return {
        "boxes": result["boxes_3d"].tensor.detach().cpu().numpy(),
        "scores": result["scores_3d"].detach().cpu().numpy(),
        "labels": result["labels_3d"].detach().cpu().numpy(),
    }


def _extract_samples(data_infos, results, class_names, use_valid_flag):
    samples = []
    for info, result in zip(data_infos, results):
        prediction = _normalize_result(result)
        if use_valid_flag:
            valid = np.asarray(info["valid_flag"], dtype=bool)
        else:
            valid = np.asarray(info["num_lidar_pts"]) > 0
        gt_boxes = np.asarray(info["gt_boxes"], dtype=np.float64)[valid]
        gt_names = np.asarray(info["gt_names"])[valid]
        gt_labels = np.asarray([
            class_names.index(name) if name in class_names else -1
            for name in gt_names
        ], dtype=np.int64)
        known = gt_labels >= 0
        samples.append({
            "gt_boxes": gt_boxes[known],
            "gt_labels": gt_labels[known],
            "pred_boxes": np.asarray(prediction["boxes"], dtype=np.float64),
            "pred_labels": np.asarray(prediction["labels"], dtype=np.int64),
            "pred_scores": np.asarray(prediction["scores"], dtype=np.float64),
        })
    return samples


def _filter_range(samples, distance_range):
    if distance_range is None:
        return samples
    lower, upper = distance_range
    filtered = []
    for sample in samples:
        gt_distance = np.linalg.norm(sample["gt_boxes"][:, :2], axis=1)
        pred_distance = np.linalg.norm(sample["pred_boxes"][:, :2], axis=1)
        gt_mask = (gt_distance >= lower) & (gt_distance < upper)
        pred_mask = (pred_distance >= lower) & (pred_distance < upper)
        filtered.append({
            "gt_boxes": sample["gt_boxes"][gt_mask],
            "gt_labels": sample["gt_labels"][gt_mask],
            "pred_boxes": sample["pred_boxes"][pred_mask],
            "pred_labels": sample["pred_labels"][pred_mask],
            "pred_scores": sample["pred_scores"][pred_mask],
        })
    return filtered


def _filter_center_box_range(samples, point_cloud_range):
    """Keep GT and predictions whose centers lie in the model box range.

    ``ObjectRangeFilter`` removes out-of-range GT during training, while the
    detector head applies the same contract to predictions.  Evaluation must
    therefore filter both sides with the identical axis-aligned range; a
    radial distance cut is not equivalent near the corners of the BEV grid.
    """
    if point_cloud_range is None:
        return samples

    point_cloud_range = np.asarray(
        point_cloud_range, dtype=np.float64).reshape(-1)
    if point_cloud_range.size != 6:
        raise ValueError(
            "point_cloud_range must contain [xmin, ymin, zmin, xmax, ymax, zmax]"
        )
    minimum = point_cloud_range[:3]
    maximum = point_cloud_range[3:]

    filtered = []
    for sample in samples:
        gt_centers = sample["gt_boxes"][:, :3]
        pred_centers = sample["pred_boxes"][:, :3]
        gt_mask = np.all(
            (gt_centers >= minimum) & (gt_centers <= maximum), axis=1)
        pred_mask = np.all(
            (pred_centers >= minimum) & (pred_centers <= maximum), axis=1)
        filtered.append({
            "gt_boxes": sample["gt_boxes"][gt_mask],
            "gt_labels": sample["gt_labels"][gt_mask],
            "pred_boxes": sample["pred_boxes"][pred_mask],
            "pred_labels": sample["pred_labels"][pred_mask],
            "pred_scores": sample["pred_scores"][pred_mask],
        })
    return filtered


def accumulate(samples, class_index, distance_threshold):
    """Score-ranked, per-sample, one-to-one center-distance matching."""
    positives = sum(int(np.count_nonzero(sample["gt_labels"] == class_index))
                    for sample in samples)
    velocity_available = bool(samples) and all(
        sample["gt_boxes"].shape[1] >= 9
        and sample["pred_boxes"].shape[1] >= 9
        for sample in samples
    )
    if positives == 0:
        return _empty_metric_data(velocity_available), 0, 0

    ranked = []
    for sample_index, sample in enumerate(samples):
        indices = np.nonzero(sample["pred_labels"] == class_index)[0]
        ranked.extend((float(sample["pred_scores"][index]), sample_index, int(index))
                      for index in indices)
    ranked.sort(key=lambda item: item[0], reverse=True)

    taken = set()
    tp, fp, confidence = [], [], []
    match_confidence = []
    translation_error, scale_error, orientation_error = [], [], []
    velocity_error, speed_error = [], []
    for score, sample_index, pred_index in ranked:
        sample = samples[sample_index]
        candidates = np.nonzero(sample["gt_labels"] == class_index)[0]
        best_distance, best_gt = np.inf, None
        for gt_index in candidates:
            if (sample_index, int(gt_index)) in taken:
                continue
            distance = np.linalg.norm(
                sample["gt_boxes"][gt_index, :2] -
                sample["pred_boxes"][pred_index, :2])
            if distance < best_distance:
                best_distance, best_gt = float(distance), int(gt_index)

        if best_gt is not None and best_distance < distance_threshold:
            taken.add((sample_index, best_gt))
            tp.append(1.0)
            fp.append(0.0)
            gt_box = sample["gt_boxes"][best_gt]
            pred_box = sample["pred_boxes"][pred_index]
            translation_error.append(best_distance)
            scale_error.append(_scale_error(gt_box[3:6], pred_box[3:6]))
            orientation_error.append(_orientation_error(gt_box[6], pred_box[6]))
            if velocity_available:
                gt_velocity = gt_box[7:9]
                pred_velocity = pred_box[7:9]
                velocity_error.append(float(np.linalg.norm(
                    pred_velocity - gt_velocity)))
                speed_error.append(float(abs(
                    np.linalg.norm(pred_velocity) -
                    np.linalg.norm(gt_velocity))))
            match_confidence.append(score)
        else:
            tp.append(0.0)
            fp.append(1.0)
        confidence.append(score)

    if not match_confidence:
        return _empty_metric_data(velocity_available), positives, len(ranked)

    cumulative_tp = np.cumsum(tp, dtype=np.float64)
    cumulative_fp = np.cumsum(fp, dtype=np.float64)
    raw_recall = cumulative_tp / float(positives)
    raw_precision = cumulative_tp / np.maximum(cumulative_tp + cumulative_fp, 1e-12)
    recall = np.linspace(0.0, 1.0, 101)
    precision = np.interp(recall, raw_recall, raw_precision, right=0.0)
    interpolated_confidence = np.interp(
        recall, raw_recall, np.asarray(confidence), right=0.0)

    errors = []
    reference_confidence = np.asarray(match_confidence)[::-1]
    error_values = [translation_error, scale_error, orientation_error]
    if velocity_available:
        error_values.extend([velocity_error, speed_error])
    for values in error_values:
        cumulative = _cummean(values)
        errors.append(np.interp(
            interpolated_confidence[::-1],
            reference_confidence,
            cumulative[::-1],
        )[::-1])

    if not velocity_available:
        errors.extend([np.ones(101), np.ones(101)])
    return MetricData(
        recall=recall,
        precision=precision,
        confidence=interpolated_confidence,
        translation_error=errors[0],
        scale_error=errors[1],
        orientation_error=errors[2],
        velocity_error=errors[3],
        speed_error=errors[4],
        velocity_available=velocity_available,
    ), positives, len(ranked)


def calculate_ap(metric_data, min_recall=0.1, min_precision=0.1):
    first = round(100 * min_recall) + 1
    precision = metric_data.precision[first:].copy() - min_precision
    precision[precision < 0.0] = 0.0
    return float(np.mean(precision) / (1.0 - min_precision))


def calculate_tp(metric_data, name, min_recall=0.1):
    first = round(100 * min_recall) + 1
    last = metric_data.max_recall_index
    if last < first:
        return 1.0
    return float(np.mean(getattr(metric_data, name)[first:last + 1]))


def best_f1_operating_point(metric_data):
    denominator = metric_data.precision + metric_data.recall
    f1 = np.divide(
        2.0 * metric_data.precision * metric_data.recall,
        denominator,
        out=np.zeros_like(denominator),
        where=denominator > 0.0,
    )
    index = int(np.argmax(f1))
    return (float(f1[index]), float(metric_data.precision[index]),
            float(metric_data.recall[index]),
            float(metric_data.confidence[index]))


def _rectangle_corners(box):
    """Return one LiDAR box's four counter-clockwise BEV corners."""
    x, y, length, width, yaw = (
        float(box[0]), float(box[1]), float(box[3]),
        float(box[4]), float(box[6]))
    local = np.asarray([
        [length / 2.0, width / 2.0],
        [-length / 2.0, width / 2.0],
        [-length / 2.0, -width / 2.0],
        [length / 2.0, -width / 2.0],
    ], dtype=np.float64)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + np.asarray([x, y])


def _polygon_area(polygon):
    if len(polygon) < 3:
        return 0.0
    values = np.asarray(polygon, dtype=np.float64)
    return float(abs(np.dot(values[:, 0], np.roll(values[:, 1], -1)) -
                     np.dot(values[:, 1], np.roll(values[:, 0], -1))) * 0.5)


def _clip_polygon(subject, clip):
    """Sutherland-Hodgman clipping for two convex CCW polygons."""
    output = [np.asarray(point, dtype=np.float64) for point in subject]
    epsilon = 1.0e-10

    def inside(point, edge_start, edge_end):
        edge = edge_end - edge_start
        relative = point - edge_start
        return edge[0] * relative[1] - edge[1] * relative[0] >= -epsilon

    def intersection(first, second, edge_start, edge_end):
        direction = second - first
        edge = edge_end - edge_start
        denominator = direction[0] * edge[1] - direction[1] * edge[0]
        if abs(denominator) <= epsilon:
            return second
        offset = edge_start - first
        scale = (offset[0] * edge[1] - offset[1] * edge[0]) / denominator
        return first + scale * direction

    for index, edge_start in enumerate(clip):
        edge_start = np.asarray(edge_start, dtype=np.float64)
        edge_end = np.asarray(clip[(index + 1) % len(clip)], dtype=np.float64)
        input_polygon, output = output, []
        if not input_polygon:
            break
        previous = input_polygon[-1]
        for current in input_polygon:
            current_inside = inside(current, edge_start, edge_end)
            previous_inside = inside(previous, edge_start, edge_end)
            if current_inside:
                if not previous_inside:
                    output.append(intersection(
                        previous, current, edge_start, edge_end))
                output.append(current)
            elif previous_inside:
                output.append(intersection(
                    previous, current, edge_start, edge_end))
            previous = current
    return output


def bev_iou(first, second):
    """Oriented BEV IoU used by the PointPainting-compatible panels."""
    if min(float(first[3]), float(first[4]),
           float(second[3]), float(second[4])) <= 0.0:
        return 0.0
    first_corners = _rectangle_corners(first)
    second_corners = _rectangle_corners(second)
    # Almost all dense head proposals are nowhere near a given GT box.  This
    # inexpensive rejection keeps the pure-NumPy metric practical on the full
    # 1,635-frame validation split.
    if (np.max(first_corners[:, 0]) < np.min(second_corners[:, 0]) or
            np.max(second_corners[:, 0]) < np.min(first_corners[:, 0]) or
            np.max(first_corners[:, 1]) < np.min(second_corners[:, 1]) or
            np.max(second_corners[:, 1]) < np.min(first_corners[:, 1])):
        return 0.0
    intersection = _polygon_area(
        _clip_polygon(first_corners, second_corners))
    first_area = float(first[3]) * float(first[4])
    second_area = float(second[3]) * float(second[4])
    union = first_area + second_area - intersection
    return float(intersection / union) if union > 0.0 else 0.0


def _precision_envelope_ap(recall, precision):
    if len(recall) == 0:
        return 0.0
    mrec = np.concatenate(([0.0], recall, [1.0]))
    mpre = np.concatenate(([0.0], precision, [0.0]))
    for index in range(len(mpre) - 2, -1, -1):
        mpre[index] = max(mpre[index], mpre[index + 1])
    changed = np.nonzero(mrec[1:] != mrec[:-1])[0]
    return float(np.sum(
        (mrec[changed + 1] - mrec[changed]) * mpre[changed + 1]))


def _prepare_iou_matching(samples, class_index):
    """Cache pairwise overlaps shared by all configured IoU thresholds."""
    positives = sum(int(np.count_nonzero(sample["gt_labels"] == class_index))
                    for sample in samples)
    ranked = []
    overlaps = {}
    candidates = {}
    for sample_index, sample in enumerate(samples):
        gt_indices = np.nonzero(sample["gt_labels"] == class_index)[0]
        pred_indices = np.nonzero(sample["pred_labels"] == class_index)[0]
        candidates[sample_index] = gt_indices
        for pred_index in pred_indices:
            pred_index = int(pred_index)
            ranked.append((float(sample["pred_scores"][pred_index]),
                           sample_index, pred_index))
            for gt_index in gt_indices:
                gt_index = int(gt_index)
                overlaps[(sample_index, pred_index, gt_index)] = bev_iou(
                    sample["pred_boxes"][pred_index],
                    sample["gt_boxes"][gt_index])
    ranked.sort(key=lambda item: item[0], reverse=True)
    return positives, ranked, candidates, overlaps


def accumulate_iou(samples, class_index, iou_threshold, prepared=None):
    """PointPainting-compatible global-score, one-to-one BEV-IoU AP."""
    if prepared is None:
        prepared = _prepare_iou_matching(samples, class_index)
    positives, ranked, candidates_by_sample, overlaps = prepared
    if positives == 0 or not ranked:
        return 0.0, 0.0

    taken = set()
    tp = np.zeros(len(ranked), dtype=np.float64)
    fp = np.zeros(len(ranked), dtype=np.float64)
    for rank, (_score, sample_index, pred_index) in enumerate(ranked):
        candidates = candidates_by_sample[sample_index]
        best_iou, best_gt = -1.0, None
        for gt_index in candidates:
            key = (sample_index, int(gt_index))
            if key in taken:
                continue
            overlap = overlaps[(sample_index, pred_index, int(gt_index))]
            if overlap > best_iou:
                best_iou, best_gt = overlap, int(gt_index)
        if best_gt is not None and best_iou >= float(iou_threshold):
            taken.add((sample_index, best_gt))
            tp[rank] = 1.0
        else:
            fp[rank] = 1.0

    cumulative_tp = np.cumsum(tp)
    cumulative_fp = np.cumsum(fp)
    recall = cumulative_tp / float(positives)
    precision = cumulative_tp / np.maximum(cumulative_tp + cumulative_fp, 1e-12)
    return _precision_envelope_ap(recall, precision), float(recall[-1])


def _evaluate_center(samples, class_names, center_thresholds,
                     tp_threshold, min_recall, min_precision, prefix,
                     include_tp=True, include_operating_points=True):
    metrics = {}
    class_mean_aps = []
    tp_names = ("translation_error", "scale_error", "orientation_error")
    tp_aliases = ("mATE", "mASE", "mAOE")
    for class_index, class_name in enumerate(class_names):
        class_aps = []
        for threshold in center_thresholds:
            metric_data, positives, predictions = accumulate(
                samples, class_index, threshold)
            ap = calculate_ap(metric_data, min_recall, min_precision)
            metrics[f"{prefix}/{class_name}_AP_{threshold:g}m"] = ap
            metrics[f"{prefix}/{class_name}_GT"] = float(positives)
            metrics[f"{prefix}/{class_name}_predictions"] = float(predictions)
            if include_operating_points:
                best_f1, precision, recall, score = best_f1_operating_point(
                    metric_data)
                operating_prefix = (
                    f"{prefix}/{class_name}_{threshold:g}m_best")
                metrics[f"{operating_prefix}_F1"] = best_f1
                metrics[f"{operating_prefix}_precision"] = precision
                metrics[f"{operating_prefix}_recall"] = recall
                metrics[f"{operating_prefix}_score"] = score
            class_aps.append(ap)
        mean_ap = float(np.mean(class_aps))
        # This is the mean over distance thresholds for one class, not a
        # mean over classes.  Spell that distinction out in dashboards and
        # exported JSON instead of overloading the conventional ``mAP`` name.
        metrics[f"{prefix}/{class_name}_mean_AP"] = mean_ap
        class_mean_aps.append(mean_ap)

        if include_tp:
            metric_data, positives, _ = accumulate(
                samples, class_index, tp_threshold)
            for name, alias in zip(tp_names, tp_aliases):
                metrics[f"tp/{class_name}_{alias}"] = calculate_tp(
                    metric_data, name, min_recall)
            if positives > 0 and metric_data.velocity_available:
                metrics[f"tp/{class_name}_mAVE"] = calculate_tp(
                    metric_data, "velocity_error", min_recall)
                metrics[f"tp/{class_name}_speed_MAE"] = calculate_tp(
                    metric_data, "speed_error", min_recall)
    metrics[f"{prefix}/mAP"] = float(np.mean(class_mean_aps))
    return metrics


def morai_detection_eval(data_infos,
                         results,
                         class_names,
                         use_valid_flag=False,
                         center_thresholds=DEFAULT_CENTER_THRESHOLDS,
                         tp_threshold=2.0,
                         min_recall=0.1,
                         min_precision=0.1,
                         distance_bins=DEFAULT_DISTANCE_BINS,
                         iou_thresholds=DEFAULT_IOU_THRESHOLDS,
                         include_tp=True,
                         include_operating_points=True,
                         point_cloud_range=None):
    """Evaluate MORAI predictions and return flat scalar metric keys."""
    if len(data_infos) != len(results):
        raise AssertionError(
            f"Result count {len(results)} != dataset count {len(data_infos)}")
    center_thresholds = tuple(float(value) for value in center_thresholds)
    if float(tp_threshold) not in center_thresholds:
        raise ValueError("tp_threshold must be included in center_thresholds")
    samples = _extract_samples(
        data_infos, results, list(class_names), use_valid_flag)
    samples = _filter_center_box_range(samples, point_cloud_range)
    metrics = _evaluate_center(
        samples, list(class_names), center_thresholds, float(tp_threshold),
        float(min_recall), float(min_precision), "center",
        include_tp=include_tp,
        include_operating_points=include_operating_points)
    # Preserve both spellings because older PointPainting panels used the
    # underscore alias while newer model-neutral panels use center/mAP.
    metrics["center_mAP"] = metrics["center/mAP"]
    if 1.0 in center_thresholds:
        for class_name in class_names:
            source = f"center/{class_name}_1m_best_"
            target = f"operating/{class_name}_1m_"
            for suffix in ("F1", "precision", "recall", "score"):
                metrics[target + suffix] = metrics[source + suffix]
    for lower, upper in distance_bins or ():
        ranged = _filter_range(samples, (float(lower), float(upper)))
        ranged_metrics = _evaluate_center(
            ranged, list(class_names), center_thresholds,
            float(tp_threshold), float(min_recall), float(min_precision),
            f"range/{lower:g}_{upper:g}m",
            include_tp=False,
            include_operating_points=include_operating_points)
        # Range diagnostics only need AP; avoid duplicating TP keys computed on
        # the complete evaluation range.
        metrics.update({key: value for key, value in ranged_metrics.items()
                        if key.startswith("range/")})
    iou_aps = {float(threshold): [] for threshold in iou_thresholds}
    iou_recalls = {float(threshold): [] for threshold in iou_thresholds}
    for class_index, class_name in enumerate(class_names):
        prepared = _prepare_iou_matching(samples, class_index)
        for threshold in iou_thresholds:
            ap, recall = accumulate_iou(
                samples, class_index, float(threshold), prepared=prepared)
            key = f"{float(threshold):.2f}"
            metrics[f"iou/{class_name}_AP_{key}"] = ap
            metrics[f"iou/{class_name}_rec_{key}"] = recall
            iou_aps[float(threshold)].append(ap)
            iou_recalls[float(threshold)].append(recall)
    for threshold in iou_thresholds:
        key = f"{float(threshold):.2f}"
        metrics[f"iou/mAP_{key}"] = float(np.mean(iou_aps[float(threshold)]))
        metrics[f"iou/mAR_{key}"] = float(
            np.mean(iou_recalls[float(threshold)]))
    metrics["meta/official_NDS_available"] = 0.0
    return metrics
