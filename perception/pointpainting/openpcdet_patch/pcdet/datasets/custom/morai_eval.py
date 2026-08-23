import numpy as np

from pcdet.ops.iou3d_nms import iou3d_nms_utils
from .morai_center_metrics import evaluate_center_samples


def _empty_boxes():
    return np.zeros((0, 7), dtype=np.float32)


def _filter_by_center_range(boxes, point_cloud_range):
    if point_cloud_range is None or boxes.size == 0:
        return boxes

    point_cloud_range = np.asarray(point_cloud_range, dtype=np.float32).reshape(-1)
    if point_cloud_range.size != 6:
        raise RuntimeError(
            f"point_cloud_range는 6개 값이어야 합니다: {point_cloud_range}"
        )

    mask = (
        (boxes[:, 0] >= point_cloud_range[0])
        & (boxes[:, 1] >= point_cloud_range[1])
        & (boxes[:, 2] >= point_cloud_range[2])
        & (boxes[:, 0] <= point_cloud_range[3])
        & (boxes[:, 1] <= point_cloud_range[4])
        & (boxes[:, 2] <= point_cloud_range[5])
    )

    return np.ascontiguousarray(boxes[mask], dtype=np.float32)


def _extract_boxes(anno, box_key, class_name, point_cloud_range=None):
    names = np.asarray(
        anno.get("name", np.empty((0,), dtype=str))
    )

    raw_boxes = anno.get(box_key, None)

    if raw_boxes is None:
        return _empty_boxes()

    boxes = np.asarray(raw_boxes, dtype=np.float32)

    if boxes.size == 0:
        return _empty_boxes()

    boxes = boxes.reshape(-1, boxes.shape[-1])

    if names.shape[0] != boxes.shape[0]:
        raise RuntimeError(
            f"name/box 개수가 다릅니다: "
            f"{names.shape[0]} vs {boxes.shape[0]}"
        )

    class_mask = names == class_name
    boxes = boxes[class_mask]

    if boxes.size == 0:
        return _empty_boxes()

    valid = (
        np.isfinite(boxes).all(axis=1)
        & (boxes[:, 3] > 0)
        & (boxes[:, 4] > 0)
        & (boxes[:, 5] > 0)
    )

    boxes = np.ascontiguousarray(boxes[valid], dtype=np.float32)
    return _filter_by_center_range(boxes, point_cloud_range)


def _extract_predictions(anno, class_name, point_cloud_range=None):
    names = np.asarray(
        anno.get("name", np.empty((0,), dtype=str))
    )

    raw_boxes = anno.get("boxes_lidar", None)
    raw_scores = anno.get("score", None)

    if raw_boxes is None or raw_scores is None:
        return _empty_boxes(), np.zeros((0,), dtype=np.float32)

    boxes = np.asarray(raw_boxes, dtype=np.float32)
    scores = np.asarray(raw_scores, dtype=np.float32).reshape(-1)

    if boxes.size == 0:
        return _empty_boxes(), np.zeros((0,), dtype=np.float32)

    boxes = boxes.reshape(-1, boxes.shape[-1])

    if not (
        names.shape[0]
        == boxes.shape[0]
        == scores.shape[0]
    ):
        raise RuntimeError(
            "prediction name/box/score 개수가 다릅니다: "
            f"{names.shape[0]}, {boxes.shape[0]}, "
            f"{scores.shape[0]}"
        )

    class_mask = names == class_name
    boxes = boxes[class_mask]
    scores = scores[class_mask]

    if boxes.size == 0:
        return _empty_boxes(), np.zeros((0,), dtype=np.float32)

    valid = (
        np.isfinite(boxes).all(axis=1)
        & np.isfinite(scores)
        & (boxes[:, 3] > 0)
        & (boxes[:, 4] > 0)
        & (boxes[:, 5] > 0)
    )

    boxes = np.ascontiguousarray(boxes[valid], dtype=np.float32)
    scores = np.ascontiguousarray(scores[valid], dtype=np.float32)

    if point_cloud_range is not None and boxes.size:
        point_cloud_range = np.asarray(
            point_cloud_range, dtype=np.float32
        ).reshape(-1)

        mask = (
            (boxes[:, 0] >= point_cloud_range[0])
            & (boxes[:, 1] >= point_cloud_range[1])
            & (boxes[:, 2] >= point_cloud_range[2])
            & (boxes[:, 0] <= point_cloud_range[3])
            & (boxes[:, 1] <= point_cloud_range[4])
            & (boxes[:, 2] <= point_cloud_range[5])
        )
        boxes = np.ascontiguousarray(boxes[mask], dtype=np.float32)
        scores = np.ascontiguousarray(scores[mask], dtype=np.float32)

    return boxes, scores


def _precision_envelope_ap(recall, precision):
    if recall.size == 0:
        return 0.0

    mrec = np.concatenate(
        [np.array([0.0]), recall, np.array([1.0])]
    )
    mpre = np.concatenate(
        [np.array([0.0]), precision, np.array([0.0])]
    )

    for index in range(mpre.size - 2, -1, -1):
        mpre[index] = max(mpre[index], mpre[index + 1])

    changed = np.where(mrec[1:] != mrec[:-1])[0]

    return float(
        np.sum(
            (mrec[changed + 1] - mrec[changed])
            * mpre[changed + 1]
        )
    )


def _weighted_mean(values, weights):
    values = np.asarray(values, dtype=np.float64)
    weights = np.asarray(weights, dtype=np.float64)
    denominator = float(np.sum(weights))
    if values.size == 0 or denominator <= 0.0:
        return 0.0
    return float(np.sum(values * weights) / denominator)


def _evaluate_velocity(frame_data, center_threshold=2.0):
    """Score-ranked 1:1 center matches with quality-weighted velocity error."""
    residuals = []
    speed_errors = []
    yaws = []
    weights = []

    for frame in frame_data:
        gt_boxes = frame["gt_boxes"]
        pred_boxes = frame["pred_boxes"]
        if gt_boxes.shape[1] < 9 or pred_boxes.shape[1] < 9:
            continue

        order = np.argsort(-frame["scores"])
        matched_gt = set()
        for pred_index in order:
            best_gt = None
            best_distance = np.inf
            for gt_index in range(gt_boxes.shape[0]):
                if gt_index in matched_gt:
                    continue
                quality = float(gt_boxes[gt_index, 9]) if gt_boxes.shape[1] >= 10 else 1.0
                if not np.isfinite(quality) or quality <= 0.0:
                    continue
                distance = float(
                    np.linalg.norm(
                        pred_boxes[pred_index, :2] - gt_boxes[gt_index, :2]
                    )
                )
                if distance < best_distance:
                    best_gt = gt_index
                    best_distance = distance

            if best_gt is None or best_distance >= center_threshold:
                continue

            matched_gt.add(best_gt)
            residuals.append(pred_boxes[pred_index, 7:9] - gt_boxes[best_gt, 7:9])
            speed_errors.append(
                abs(
                    np.linalg.norm(pred_boxes[pred_index, 7:9])
                    - np.linalg.norm(gt_boxes[best_gt, 7:9])
                )
            )
            yaws.append(gt_boxes[best_gt, 6])
            weights.append(
                float(gt_boxes[best_gt, 9]) if gt_boxes.shape[1] >= 10 else 1.0
            )

    if not residuals:
        return None

    residuals = np.asarray(residuals, dtype=np.float64)
    yaws = np.asarray(yaws, dtype=np.float64)
    weights = np.asarray(weights, dtype=np.float64)
    vector_error = np.linalg.norm(residuals, axis=1)
    speed_errors = np.asarray(speed_errors, dtype=np.float64)
    longitudinal = np.cos(yaws) * residuals[:, 0] + np.sin(yaws) * residuals[:, 1]
    lateral = -np.sin(yaws) * residuals[:, 0] + np.cos(yaws) * residuals[:, 1]

    return {
        "matches": int(len(residuals)),
        "weight_sum": float(np.sum(weights)),
        "mave": _weighted_mean(vector_error, weights),
        "speed_mae": _weighted_mean(speed_errors, weights),
        "longitudinal_mae": _weighted_mean(np.abs(longitudinal), weights),
        "lateral_mae": _weighted_mean(np.abs(lateral), weights),
    }


def _build_canonical_samples(
        gt_annos, det_annos, class_names, point_cloud_range):
    samples = []
    for gt_anno, det_anno in zip(gt_annos, det_annos):
        gt_chunks, gt_labels = [], []
        pred_chunks, pred_labels, pred_scores = [], [], []
        for class_index, class_name in enumerate(class_names):
            gt_boxes = _extract_boxes(
                gt_anno, "gt_boxes_lidar", class_name, point_cloud_range)
            boxes, scores = _extract_predictions(
                det_anno, class_name, point_cloud_range)
            if len(gt_boxes):
                gt_chunks.append(gt_boxes)
                gt_labels.append(np.full(len(gt_boxes), class_index, dtype=np.int64))
            if len(boxes):
                pred_chunks.append(boxes)
                pred_labels.append(np.full(len(boxes), class_index, dtype=np.int64))
                pred_scores.append(scores)

        samples.append({
            "gt_boxes": (
                np.concatenate(gt_chunks) if gt_chunks else _empty_boxes()
            ),
            "gt_labels": (
                np.concatenate(gt_labels) if gt_labels
                else np.zeros((0,), dtype=np.int64)
            ),
            "pred_boxes": (
                np.concatenate(pred_chunks) if pred_chunks else _empty_boxes()
            ),
            "pred_labels": (
                np.concatenate(pred_labels) if pred_labels
                else np.zeros((0,), dtype=np.int64)
            ),
            "pred_scores": (
                np.concatenate(pred_scores) if pred_scores
                else np.zeros((0,), dtype=np.float32)
            ),
        })
    return samples


def _evaluate_one_threshold(frame_data, num_gt, iou_threshold):
    predictions = []

    for frame_index, frame in enumerate(frame_data):
        for prediction_index, score in enumerate(frame["scores"]):
            predictions.append(
                (
                    float(score),
                    frame_index,
                    prediction_index,
                )
            )

    predictions.sort(key=lambda item: item[0], reverse=True)

    if not predictions:
        return {
            "ap": 0.0,
            "precision": 0.0,
            "recall": 0.0,
            "best_f1": 0.0,
            "best_score": 0.0,
            "tp": 0,
            "fp": 0,
        }

    matched = [
        np.zeros(frame["gt_boxes"].shape[0], dtype=bool)
        for frame in frame_data
    ]

    tp = np.zeros(len(predictions), dtype=np.float64)
    fp = np.zeros(len(predictions), dtype=np.float64)
    sorted_scores = np.zeros(len(predictions), dtype=np.float64)

    for rank, (score, frame_index, prediction_index) in enumerate(
        predictions
    ):
        sorted_scores[rank] = score
        frame = frame_data[frame_index]

        if frame["gt_boxes"].shape[0] == 0:
            fp[rank] = 1.0
            continue

        ious = frame["bev_iou"][prediction_index].copy()
        ious[matched[frame_index]] = -1.0

        best_gt = int(np.argmax(ious))
        best_iou = float(ious[best_gt])

        if best_iou >= iou_threshold:
            matched[frame_index][best_gt] = True
            tp[rank] = 1.0
        else:
            fp[rank] = 1.0

    cumulative_tp = np.cumsum(tp)
    cumulative_fp = np.cumsum(fp)

    recall = cumulative_tp / max(num_gt, 1)
    precision = cumulative_tp / np.maximum(
        cumulative_tp + cumulative_fp,
        1e-12,
    )

    ap = _precision_envelope_ap(recall, precision)

    f1 = (
        2.0
        * precision
        * recall
        / np.maximum(precision + recall, 1e-12)
    )

    best_index = int(np.argmax(f1))

    return {
        "ap": ap,
        "precision": float(precision[-1]),
        "recall": float(recall[-1]),
        "best_f1": float(f1[best_index]),
        "best_score": float(sorted_scores[best_index]),
        "tp": int(cumulative_tp[-1]),
        "fp": int(cumulative_fp[-1]),
    }


def get_morai_eval_result(
    gt_annos,
    det_annos,
    class_names,
    iou_thresholds=(0.25, 0.30, 0.50),
    point_cloud_range=None,
):
    if len(gt_annos) != len(det_annos):
        raise RuntimeError(
            "GT/prediction 프레임 수가 다릅니다: "
            f"{len(gt_annos)} vs {len(det_annos)}"
        )

    result_lines = [
        "MORAI LiDAR-coordinate BEV evaluation",
        "AP: global score ranking + one-to-one greedy matching",
    ]
    result_dict = {}
    canonical_class_names = [str(name).lower() for name in class_names]
    canonical_samples = _build_canonical_samples(
        gt_annos, det_annos, class_names, point_cloud_range)
    result_dict.update(evaluate_center_samples(
        canonical_samples, canonical_class_names))
    iou_ap_by_threshold = {float(value): [] for value in iou_thresholds}
    iou_recall_by_threshold = {float(value): [] for value in iou_thresholds}
    velocity_maves = []

    for class_name in class_names:
        frame_data = []
        total_gt = 0
        total_predictions = 0

        for gt_anno, det_anno in zip(gt_annos, det_annos):
            gt_boxes = _extract_boxes(
                gt_anno,
                "gt_boxes_lidar",
                class_name,
                point_cloud_range=point_cloud_range,
            )

            pred_boxes, pred_scores = _extract_predictions(
                det_anno,
                class_name,
                point_cloud_range=point_cloud_range,
            )

            if pred_boxes.shape[0] and gt_boxes.shape[0]:
                bev_iou = iou3d_nms_utils.boxes_bev_iou_cpu(
                    pred_boxes[:, :7],
                    gt_boxes[:, :7],
                ).astype(np.float32, copy=False)
            else:
                bev_iou = np.zeros(
                    (
                        pred_boxes.shape[0],
                        gt_boxes.shape[0],
                    ),
                    dtype=np.float32,
                )

            frame_data.append(
                {
                    "gt_boxes": gt_boxes,
                    "pred_boxes": pred_boxes,
                    "scores": pred_scores,
                    "bev_iou": bev_iou,
                }
            )

            total_gt += int(gt_boxes.shape[0])
            total_predictions += int(pred_boxes.shape[0])

        result_lines.append("")
        result_lines.append(
            f"{class_name}: "
            f"GT={total_gt}, predictions={total_predictions}"
        )

        result_dict[f"morai/{class_name}_gt"] = total_gt
        result_dict[
            f"morai/{class_name}_predictions"
        ] = total_predictions

        for iou_threshold in iou_thresholds:
            metrics = _evaluate_one_threshold(
                frame_data,
                total_gt,
                float(iou_threshold),
            )

            key = f"{iou_threshold:.2f}"

            result_lines.append(
                f"BEV IoU {key}: "
                f"AP={metrics['ap']:.6f}, "
                f"precision={metrics['precision']:.6f}, "
                f"recall={metrics['recall']:.6f}, "
                f"best_F1={metrics['best_f1']:.6f}, "
                f"best_score={metrics['best_score']:.6f}, "
                f"TP={metrics['tp']}, FP={metrics['fp']}"
            )

            result_dict[
                f"morai/{class_name}_bev_ap_{key}"
            ] = metrics["ap"]
            result_dict[
                f"morai/{class_name}_bev_precision_{key}"
            ] = metrics["precision"]
            result_dict[
                f"morai/{class_name}_bev_recall_{key}"
            ] = metrics["recall"]
            result_dict[
                f"morai/{class_name}_bev_best_f1_{key}"
            ] = metrics["best_f1"]
            result_dict[
                f"morai/{class_name}_bev_best_score_{key}"
            ] = metrics["best_score"]
            canonical_class = str(class_name).lower()
            result_dict[f"iou/{canonical_class}_AP_{key}"] = metrics["ap"]
            result_dict[f"iou/{canonical_class}_rec_{key}"] = metrics["recall"]
            iou_ap_by_threshold[float(iou_threshold)].append(metrics["ap"])
            iou_recall_by_threshold[float(iou_threshold)].append(metrics["recall"])

        velocity = _evaluate_velocity(frame_data)
        if velocity is not None:
            result_lines.append(
                "Velocity @2m center: "
                f"matches={velocity['matches']}, "
                f"weighted_mAVE={velocity['mave']:.6f}m/s, "
                f"speed_MAE={velocity['speed_mae']:.6f}m/s, "
                f"longitudinal_MAE={velocity['longitudinal_mae']:.6f}m/s, "
                f"lateral_MAE={velocity['lateral_mae']:.6f}m/s"
            )
            result_dict[f"morai/{class_name}_velocity_matches"] = velocity["matches"]
            result_dict[f"morai/{class_name}_velocity_weight_sum"] = velocity["weight_sum"]
            result_dict[f"morai/{class_name}_velocity_mAVE"] = velocity["mave"]
            result_dict[
                f"morai/{class_name}_velocity_speed_MAE"
            ] = velocity["speed_mae"]
            result_dict[
                f"morai/{class_name}_velocity_longitudinal_MAE"
            ] = velocity["longitudinal_mae"]
            result_dict[
                f"morai/{class_name}_velocity_lateral_MAE"
            ] = velocity["lateral_mae"]
            canonical_class = str(class_name).lower()
            result_dict[f"velocity/{canonical_class}_matches"] = velocity["matches"]
            result_dict[
                f"velocity/{canonical_class}_weight_sum"
            ] = velocity["weight_sum"]
            result_dict[f"velocity/{canonical_class}_mAVE"] = velocity["mave"]
            result_dict[
                f"velocity/{canonical_class}_speed_MAE"
            ] = velocity["speed_mae"]
            result_dict[
                f"velocity/{canonical_class}_longitudinal_MAE"
            ] = velocity["longitudinal_mae"]
            result_dict[
                f"velocity/{canonical_class}_lateral_MAE"
            ] = velocity["lateral_mae"]
            velocity_maves.append(velocity["mave"])

    for threshold in iou_thresholds:
        key = f"{float(threshold):.2f}"
        result_dict[f"iou/mAP_{key}"] = float(
            np.mean(iou_ap_by_threshold[float(threshold)]))
        result_dict[f"iou/mAR_{key}"] = float(
            np.mean(iou_recall_by_threshold[float(threshold)]))
    if velocity_maves:
        result_dict["velocity/mAVE"] = float(np.mean(velocity_maves))

    return "\n".join(result_lines), result_dict
