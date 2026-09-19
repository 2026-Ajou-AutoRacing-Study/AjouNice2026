"""Model-neutral MORAI center-distance metrics used by Fast-BEV dashboards."""

from dataclasses import dataclass

import numpy as np


DEFAULT_CENTER_THRESHOLDS = (0.5, 1.0, 2.0, 4.0)
DEFAULT_DISTANCE_BINS = ((0.0, 10.0), (10.0, 20.0),
                         (20.0, 30.0), (30.0, 50.0))


@dataclass
class MetricData:
    recall: np.ndarray
    precision: np.ndarray
    confidence: np.ndarray
    translation_error: np.ndarray
    scale_error: np.ndarray
    orientation_error: np.ndarray

    @property
    def max_recall_index(self):
        nonzero = np.nonzero(self.confidence)[0]
        return int(nonzero[-1]) if len(nonzero) else 0


def _empty_metric_data():
    recall = np.linspace(0.0, 1.0, 101)
    return MetricData(
        recall=recall,
        precision=np.zeros(101),
        confidence=np.zeros(101),
        translation_error=np.ones(101),
        scale_error=np.ones(101),
        orientation_error=np.ones(101),
    )


def _cummean(values):
    values = np.asarray(values, dtype=np.float64)
    if len(values) == 0:
        return values
    return np.cumsum(values) / np.arange(1, len(values) + 1)


def _scale_error(gt_size, pred_size):
    gt_size = np.maximum(np.asarray(gt_size, dtype=np.float64), 1e-8)
    pred_size = np.maximum(np.asarray(pred_size, dtype=np.float64), 1e-8)
    intersection = np.prod(np.minimum(gt_size, pred_size))
    union = np.prod(gt_size) + np.prod(pred_size) - intersection
    return float(1.0 - intersection / max(union, 1e-8))


def _orientation_error(gt_yaw, pred_yaw, period=2.0 * np.pi):
    difference = (pred_yaw - gt_yaw + period / 2.0) % period - period / 2.0
    return float(abs(difference))


def _filter_range(samples, distance_range):
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


def _accumulate(samples, class_index, distance_threshold):
    positives = sum(int(np.count_nonzero(sample["gt_labels"] == class_index))
                    for sample in samples)
    if positives == 0:
        return _empty_metric_data(), 0, 0

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
            match_confidence.append(score)
        else:
            tp.append(0.0)
            fp.append(1.0)
        confidence.append(score)

    if not match_confidence:
        return _empty_metric_data(), positives, len(ranked)

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
    for values in (translation_error, scale_error, orientation_error):
        cumulative = _cummean(values)
        errors.append(np.interp(
            interpolated_confidence[::-1],
            reference_confidence,
            cumulative[::-1],
        )[::-1])

    return MetricData(
        recall=recall,
        precision=precision,
        confidence=interpolated_confidence,
        translation_error=errors[0],
        scale_error=errors[1],
        orientation_error=errors[2],
    ), positives, len(ranked)


def _calculate_ap(metric_data, min_recall=0.1, min_precision=0.1):
    first = round(100 * min_recall) + 1
    precision = metric_data.precision[first:].copy() - min_precision
    precision[precision < 0.0] = 0.0
    return float(np.mean(precision) / (1.0 - min_precision))


def _calculate_tp(metric_data, name, min_recall=0.1):
    first = round(100 * min_recall) + 1
    last = metric_data.max_recall_index
    if last < first:
        return 1.0
    return float(np.mean(getattr(metric_data, name)[first:last + 1]))


def _best_f1_operating_point(metric_data):
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
            metric_data, positives, predictions = _accumulate(
                samples, class_index, threshold)
            ap = _calculate_ap(metric_data, min_recall, min_precision)
            metrics[f"{prefix}/{class_name}_AP_{threshold:g}m"] = ap
            metrics[f"{prefix}/{class_name}_GT"] = float(positives)
            metrics[f"{prefix}/{class_name}_predictions"] = float(predictions)
            if include_operating_points:
                best_f1, precision, recall, score = _best_f1_operating_point(
                    metric_data)
                operating_prefix = f"{prefix}/{class_name}_{threshold:g}m_best"
                metrics[f"{operating_prefix}_F1"] = best_f1
                metrics[f"{operating_prefix}_precision"] = precision
                metrics[f"{operating_prefix}_recall"] = recall
                metrics[f"{operating_prefix}_score"] = score
            class_aps.append(ap)
        mean_ap = float(np.mean(class_aps))
        metrics[f"{prefix}/{class_name}_mean_AP"] = mean_ap
        class_mean_aps.append(mean_ap)

        if include_tp:
            metric_data, _, _ = _accumulate(samples, class_index, tp_threshold)
            for name, alias in zip(tp_names, tp_aliases):
                metrics[f"tp/{class_name}_{alias}"] = _calculate_tp(
                    metric_data, name, min_recall)
    metrics[f"{prefix}/mAP"] = float(np.mean(class_mean_aps))
    return metrics


def evaluate_center_samples(
        samples, class_names, center_thresholds=DEFAULT_CENTER_THRESHOLDS,
        tp_threshold=2.0, min_recall=0.1, min_precision=0.1,
        distance_bins=DEFAULT_DISTANCE_BINS):
    """Return the same flat canonical metric keys used by Fast-BEV."""
    center_thresholds = tuple(float(value) for value in center_thresholds)
    metrics = _evaluate_center(
        samples, class_names, center_thresholds, float(tp_threshold),
        float(min_recall), float(min_precision), "center",
        include_tp=True, include_operating_points=True)
    metrics["center_mAP"] = metrics["center/mAP"]

    for class_name in class_names:
        source = f"center/{class_name}_1m_best_"
        target = f"operating/{class_name}_1m_"
        for suffix in ("F1", "precision", "recall", "score"):
            metrics[target + suffix] = metrics[source + suffix]

    for lower, upper in distance_bins:
        ranged = _filter_range(samples, (float(lower), float(upper)))
        ranged_metrics = _evaluate_center(
            ranged, class_names, center_thresholds, float(tp_threshold),
            float(min_recall), float(min_precision),
            f"range/{lower:g}_{upper:g}m",
            include_tp=False, include_operating_points=False)
        metrics.update(ranged_metrics)

    metrics["meta/official_NDS_available"] = 0.0
    return metrics
