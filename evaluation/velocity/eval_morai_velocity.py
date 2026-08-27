import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np
import torch

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import build_dataloader
from pcdet.models import build_network, load_data_to_gpu
from pcdet.ops.iou3d_nms import iou3d_nms_utils
from pcdet.utils import common_utils


MOTION_THRESHOLD = 0.5


def make_bucket():
    return {
        "gt": 0,
        "matched": 0,
        "vec": [],
        "vx": [],
        "vy": [],
        "speed": [],
        "zero": [],
    }


def groups_for(class_name, speed, distance):
    groups = [
        "All",
        f"Class/{class_name}",
        (
            "Motion/Static"
            if speed < MOTION_THRESHOLD
            else "Motion/Moving"
        ),
    ]

    if distance < 10.0:
        groups.append("Distance/0-10")
    elif distance < 20.0:
        groups.append("Distance/10-20")
    elif distance < 30.0:
        groups.append("Distance/20-30")
    else:
        # MORAI evaluator uses rectangular x/y range,
        # so corner GT can have radial distance > 30 m.
        groups.append("Distance/30+")

    return groups


def register_gt(agg, groups):
    for g in groups:
        agg[g]["gt"] += 1


def register_match(agg, groups, pv, gv):
    pv = np.asarray(pv, dtype=np.float32)
    gv = np.asarray(gv, dtype=np.float32)

    diff = pv - gv

    vx_err = abs(float(diff[0]))
    vy_err = abs(float(diff[1]))
    vec_err = float(np.linalg.norm(diff))

    pred_speed = float(np.linalg.norm(pv))
    gt_speed = float(np.linalg.norm(gv))
    speed_err = abs(pred_speed - gt_speed)

    for g in groups:
        b = agg[g]

        b["matched"] += 1
        b["vec"].append(vec_err)
        b["vx"].append(vx_err)
        b["vy"].append(vy_err)
        b["speed"].append(speed_err)

        # all-zero velocity baseline on the SAME matched GTs
        b["zero"].append(gt_speed)


def mean_or_nan(values):
    if not values:
        return float("nan")

    return float(np.mean(values))


def range_mask(boxes, point_cloud_range):
    """
    Exact center-range logic used by morai_eval.py.
    """
    if len(boxes) == 0:
        return np.zeros((0,), dtype=bool)

    r = np.asarray(
        point_cloud_range,
        dtype=np.float32,
    ).reshape(-1)

    return (
        (boxes[:, 0] >= r[0])
        & (boxes[:, 1] >= r[1])
        & (boxes[:, 2] >= r[2])
        & (boxes[:, 0] <= r[3])
        & (boxes[:, 1] <= r[4])
        & (boxes[:, 2] <= r[5])
    )


def extract_gt(
    info,
    velocity_path,
    class_name,
    point_cloud_range,
):
    annos = info["annos"]

    names = np.asarray(annos["name"])

    boxes = np.asarray(
        annos["gt_boxes_lidar"],
        dtype=np.float32,
    )

    if boxes.size == 0:
        boxes = np.zeros(
            (0, 7),
            dtype=np.float32,
        )
    else:
        boxes = boxes.reshape(
            -1,
            boxes.shape[-1],
        )[:, :7]

    velocities = np.load(
        velocity_path
    ).astype(np.float32)

    if not (
        len(names)
        == len(boxes)
        == len(velocities)
    ):
        raise RuntimeError(
            "GT name/box/velocity count mismatch: "
            f"{len(names)}, {len(boxes)}, "
            f"{len(velocities)}"
        )

    class_mask = names == class_name

    boxes = boxes[class_mask]
    velocities = velocities[class_mask]

    if len(boxes) == 0:
        return (
            np.zeros((0, 7), dtype=np.float32),
            np.zeros((0, 3), dtype=np.float32),
        )

    # Exact geometric validity filtering from morai_eval.py
    valid = (
        np.isfinite(boxes).all(axis=1)
        & (boxes[:, 3] > 0)
        & (boxes[:, 4] > 0)
        & (boxes[:, 5] > 0)
    )

    boxes = boxes[valid]
    velocities = velocities[valid]

    rm = range_mask(
        boxes,
        point_cloud_range,
    )

    boxes = np.ascontiguousarray(
        boxes[rm],
        dtype=np.float32,
    )

    velocities = np.ascontiguousarray(
        velocities[rm],
        dtype=np.float32,
    )

    return boxes, velocities


def extract_predictions(
    pred,
    class_id,
    point_cloud_range,
):
    boxes = (
        pred["pred_boxes"]
        .detach()
        .cpu()
        .numpy()
        .astype(np.float32)
    )

    scores = (
        pred["pred_scores"]
        .detach()
        .cpu()
        .numpy()
        .astype(np.float32)
    )

    labels = (
        pred["pred_labels"]
        .detach()
        .cpu()
        .numpy()
    )

    velocities = (
        pred["pred_velocities"]
        .detach()
        .cpu()
        .numpy()
        .astype(np.float32)
    )

    if not (
        len(boxes)
        == len(scores)
        == len(labels)
        == len(velocities)
    ):
        raise RuntimeError(
            "prediction box/score/label/velocity "
            "count mismatch"
        )

    class_mask = labels == class_id

    boxes = boxes[class_mask]
    scores = scores[class_mask]
    velocities = velocities[class_mask]

    if len(boxes) == 0:
        return (
            np.zeros((0, 7), dtype=np.float32),
            np.zeros((0,), dtype=np.float32),
            np.zeros((0, 2), dtype=np.float32),
        )

    # Exact prediction filtering used by morai_eval.py.
    # Velocity validity must NOT affect detection matching.
    valid = (
        np.isfinite(boxes).all(axis=1)
        & np.isfinite(scores)
        & (boxes[:, 3] > 0)
        & (boxes[:, 4] > 0)
        & (boxes[:, 5] > 0)
    )

    boxes = boxes[valid]
    scores = scores[valid]
    velocities = velocities[valid]

    rm = range_mask(
        boxes,
        point_cloud_range,
    )

    boxes = np.ascontiguousarray(
        boxes[rm],
        dtype=np.float32,
    )

    scores = np.ascontiguousarray(
        scores[rm],
        dtype=np.float32,
    )

    velocities = np.ascontiguousarray(
        velocities[rm],
        dtype=np.float32,
    )

    return boxes, scores, velocities


def evaluate_threshold(
    frame_data,
    num_gt,
    iou_threshold,
    agg,
    class_name,
):
    """
    Exact matching order of morai_eval._evaluate_one_threshold():

      1. collect predictions from ALL frames
      2. global descending score sort
      3. one-to-one greedy matching per frame
      4. highest unmatched BEV IoU GT
    """

    predictions = []

    for frame_index, frame in enumerate(
        frame_data
    ):
        for prediction_index, score in enumerate(
            frame["scores"]
        ):
            predictions.append(
                (
                    float(score),
                    frame_index,
                    prediction_index,
                )
            )

    predictions.sort(
        key=lambda item: item[0],
        reverse=True,
    )

    matched = [
        np.zeros(
            frame["gt_boxes"].shape[0],
            dtype=bool,
        )
        for frame in frame_data
    ]

    tp = 0
    fp = 0
    velocity_matches = 0
    csv_rows = []

    for (
        score,
        frame_index,
        prediction_index,
    ) in predictions:

        frame = frame_data[frame_index]

        if frame["gt_boxes"].shape[0] == 0:
            fp += 1
            continue

        ious = (
            frame["bev_iou"][
                prediction_index
            ].copy()
        )

        ious[
            matched[frame_index]
        ] = -1.0

        best_gt = int(
            np.argmax(ious)
        )

        best_iou = float(
            ious[best_gt]
        )

        if best_iou < iou_threshold:
            fp += 1
            continue

        matched[
            frame_index
        ][best_gt] = True

        tp += 1

        gt_vel = frame["gt_vel"][best_gt]

        velocity_valid = (
            gt_vel[2] > 0.5
        )

        # Detection TP is still counted even if
        # velocity supervision is unavailable.
        if not velocity_valid:
            continue

        pred_vel = (
            frame["pred_vel"][
                prediction_index
            ]
        )

        if not np.isfinite(
            pred_vel
        ).all():
            raise RuntimeError(
                "NaN/Inf predicted velocity"
            )

        gv = gt_vel[:2]
        pv = pred_vel[:2]

        speed = float(
            np.linalg.norm(gv)
        )

        box = frame["gt_boxes"][best_gt]

        distance = float(
            np.hypot(
                box[0],
                box[1],
            )
        )

        groups = groups_for(
            class_name,
            speed,
            distance,
        )

        register_match(
            agg,
            groups,
            pv,
            gv,
        )

        velocity_matches += 1

        csv_rows.append({
            "frame_id": frame["frame_id"],
            "class": class_name,
            "score": score,
            "bev_iou": best_iou,
            "distance_m": distance,
            "gt_vx": float(gv[0]),
            "gt_vy": float(gv[1]),
            "gt_speed": speed,
            "pred_vx": float(pv[0]),
            "pred_vy": float(pv[1]),
            "pred_speed": float(
                np.linalg.norm(pv)
            ),
            "vector_error": float(
                np.linalg.norm(pv - gv)
            ),
            "motion": (
                "static"
                if speed < MOTION_THRESHOLD
                else "moving"
            ),
        })

    return {
        "tp": tp,
        "fp": fp,
        "recall": (
            float(tp / num_gt)
            if num_gt > 0
            else 0.0
        ),
        "velocity_matches": velocity_matches,
        "csv_rows": csv_rows,
    }


def print_velocity_report(
    agg,
    threshold,
):
    groups = [
        "All",
        "Motion/Static",
        "Motion/Moving",
        "Class/Pedestrian",
        "Class/Vehicle",
        "Class/Obstacle",
        "Distance/0-10",
        "Distance/10-20",
        "Distance/20-30",
        "Distance/30+",
    ]

    print()
    print("=" * 118)
    print(
        "VELOCITY METRICS ON MORAI DETECTION TP "
        f"(BEV IoU >= {threshold:.2f})"
    )
    print("=" * 118)

    print(
        f"{'Group':22s} "
        f"{'VelGT':>7s} "
        f"{'Match':>7s} "
        f"{'Rate':>8s} "
        f"{'mAVE':>9s} "
        f"{'MAE_vx':>9s} "
        f"{'MAE_vy':>9s} "
        f"{'SpeedMAE':>10s} "
        f"{'ZeroBase':>10s}"
    )

    for name in groups:
        b = agg[name]

        rate = (
            100.0
            * b["matched"]
            / b["gt"]
            if b["gt"]
            else float("nan")
        )

        print(
            f"{name:22s} "
            f"{b['gt']:7d} "
            f"{b['matched']:7d} "
            f"{rate:7.2f}% "
            f"{mean_or_nan(b['vec']):9.4f} "
            f"{mean_or_nan(b['vx']):9.4f} "
            f"{mean_or_nan(b['vy']):9.4f} "
            f"{mean_or_nan(b['speed']):10.4f} "
            f"{mean_or_nan(b['zero']):10.4f}"
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--cfg",
        required=True,
    )
    parser.add_argument(
        "--ckpt",
        required=True,
    )
    parser.add_argument(
        "--data",
        required=True,
    )
    parser.add_argument(
        "--out-dir",
        required=True,
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=2,
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=4,
    )

    args = parser.parse_args()

    out_dir = Path(args.out_dir)

    out_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    cfg_from_yaml_file(
        args.cfg,
        cfg,
    )

    cfg.DATA_CONFIG.DATA_PATH = args.data

    point_cloud_range = np.asarray(
        cfg.DATA_CONFIG.POINT_CLOUD_RANGE,
        dtype=np.float32,
    )

    logger = common_utils.create_logger()

    dataset, loader, _ = build_dataloader(
        dataset_cfg=cfg.DATA_CONFIG,
        class_names=cfg.CLASS_NAMES,
        batch_size=args.batch_size,
        dist=False,
        workers=args.workers,
        logger=logger,
        training=False,
    )

    info_map = {
        str(
            info["point_cloud"]["lidar_idx"]
        ): info
        for info in dataset.custom_infos
    }

    print()
    print("Val samples:", len(dataset))
    print(
        "POINT_CLOUD_RANGE:",
        point_cloud_range.tolist(),
    )

    model = build_network(
        model_cfg=cfg.MODEL,
        num_class=len(cfg.CLASS_NAMES),
        dataset=dataset,
    )

    model.load_params_from_file(
        filename=args.ckpt,
        logger=logger,
        to_cpu=False,
    )

    model.cuda()
    model.eval()

    # Save raw inference results first.
    raw_frames = []

    for batch_idx, batch in enumerate(loader):

        frame_ids = [
            str(x)
            for x in batch["frame_id"]
        ]

        load_data_to_gpu(batch)

        with torch.no_grad():
            pred_dicts, _ = model(batch)

        for bi, sid in enumerate(frame_ids):

            pred = pred_dicts[bi]

            if "pred_velocities" not in pred:
                raise RuntimeError(
                    "pred_velocities missing"
                )

            raw_frames.append({
                "frame_id": sid,
                "info": info_map[sid],
                "pred": {
                    "pred_boxes": (
                        pred["pred_boxes"]
                        .detach()
                        .cpu()
                    ),
                    "pred_scores": (
                        pred["pred_scores"]
                        .detach()
                        .cpu()
                    ),
                    "pred_labels": (
                        pred["pred_labels"]
                        .detach()
                        .cpu()
                    ),
                    "pred_velocities": (
                        pred["pred_velocities"]
                        .detach()
                        .cpu()
                    ),
                },
            })

        if (batch_idx + 1) % 100 == 0:
            print(
                "processed",
                min(
                    (batch_idx + 1)
                    * args.batch_size,
                    len(dataset),
                ),
                "/",
                len(dataset),
            )

    assert len(raw_frames) == len(dataset)

    thresholds = [0.30, 0.50]

    aggs = {
        t: defaultdict(make_bucket)
        for t in thresholds
    }

    csv_rows = {
        t: []
        for t in thresholds
    }

    print()
    print("=" * 80)
    print("MORAI-EXACT DETECTION MATCHING CHECK")
    print("=" * 80)

    for class_id, class_name in enumerate(
        cfg.CLASS_NAMES,
        start=1,
    ):

        frame_data = []

        total_gt = 0
        total_predictions = 0
        total_valid_velocity_gt = 0

        for raw in raw_frames:

            sid = raw["frame_id"]

            gt_boxes, gt_vel = extract_gt(
                raw["info"],
                Path(args.data)
                / "velocities"
                / f"{sid}.npy",
                class_name,
                point_cloud_range,
            )

            pred_boxes, pred_scores, pred_vel = (
                extract_predictions(
                    raw["pred"],
                    class_id,
                    point_cloud_range,
                )
            )

            if (
                len(pred_boxes)
                and len(gt_boxes)
            ):
                bev_iou = (
                    iou3d_nms_utils
                    .boxes_bev_iou_cpu(
                        pred_boxes,
                        gt_boxes,
                    )
                    .astype(
                        np.float32,
                        copy=False,
                    )
                )
            else:
                bev_iou = np.zeros(
                    (
                        len(pred_boxes),
                        len(gt_boxes),
                    ),
                    dtype=np.float32,
                )

            frame_data.append({
                "frame_id": sid,
                "gt_boxes": gt_boxes,
                "gt_vel": gt_vel,
                "pred_boxes": pred_boxes,
                "pred_vel": pred_vel,
                "scores": pred_scores,
                "bev_iou": bev_iou,
            })

            total_gt += len(gt_boxes)
            total_predictions += len(
                pred_boxes
            )

            # Register denominator for velocity groups.
            for gi in range(len(gt_boxes)):

                if gt_vel[gi, 2] <= 0.5:
                    continue

                total_valid_velocity_gt += 1

                gv = gt_vel[gi, :2]

                speed = float(
                    np.linalg.norm(gv)
                )

                distance = float(
                    np.hypot(
                        gt_boxes[gi, 0],
                        gt_boxes[gi, 1],
                    )
                )

                groups = groups_for(
                    class_name,
                    speed,
                    distance,
                )

                for t in thresholds:
                    register_gt(
                        aggs[t],
                        groups,
                    )

        print()
        print(
            f"{class_name}: "
            f"GT={total_gt}, "
            f"predictions={total_predictions}, "
            f"valid_velocity_GT="
            f"{total_valid_velocity_gt}"
        )

        for t in thresholds:

            result = evaluate_threshold(
                frame_data,
                total_gt,
                t,
                aggs[t],
                class_name,
            )

            csv_rows[t].extend(
                result["csv_rows"]
            )

            print(
                f"  IoU {t:.2f}: "
                f"TP={result['tp']}, "
                f"FP={result['fp']}, "
                f"recall="
                f"{result['recall']:.6f}, "
                f"velocity_matches="
                f"{result['velocity_matches']}"
            )

    for t in thresholds:

        print_velocity_report(
            aggs[t],
            t,
        )

        path = (
            out_dir
            / f"velocity_matches_iou{t:.2f}.csv"
        )

        fields = [
            "frame_id",
            "class",
            "score",
            "bev_iou",
            "distance_m",
            "gt_vx",
            "gt_vy",
            "gt_speed",
            "pred_vx",
            "pred_vy",
            "pred_speed",
            "vector_error",
            "motion",
        ]

        with open(
            path,
            "w",
            newline="",
        ) as f:

            writer = csv.DictWriter(
                f,
                fieldnames=fields,
            )

            writer.writeheader()
            writer.writerows(
                csv_rows[t]
            )

        print()
        print(
            f"CSV IoU {t:.2f}:",
            path,
        )

    print()
    print(
        "MORAI-EXACT VELOCITY EVALUATION: PASS"
    )


if __name__ == "__main__":
    main()
