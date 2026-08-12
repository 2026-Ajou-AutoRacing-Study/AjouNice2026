"""Fast-BEV dataset adapter that accepts MORAI's four-camera rig."""

import numpy as np

from mmdet.datasets import DATASETS
from mmdet3d.datasets.dataset_wrappers import MultiViewMixin
from mmdet3d.datasets.nuscenes_dataset import NuScenesDataset


def _float32(value):
    return value.astype(np.float32) if value is not None else None


@DATASETS.register_module()
class MoraiMultiViewDataset(MultiViewMixin, NuScenesDataset):
    """NuScenes-info-compatible dataset without the upstream 6-camera assert."""

    def get_data_info(self, index):
        data_info = NuScenesDataset.get_data_info(self, index)
        camera_count = len(data_info["img_filename"])
        if camera_count != 4:
            raise AssertionError("MORAI Sensor_6 requires exactly four cameras")
        result = {
            "sample_idx": data_info["sample_idx"],
            "img_prefix": [None] * camera_count,
            "img_info": [dict(filename=value) for value in data_info["img_filename"]],
            "lidar2img": {
                "extrinsic": [_float32(value) for value in data_info["lidar2img"]],
                "intrinsic": np.eye(4, dtype=np.float32),
                "lidar2img_aug": data_info["lidar2img_aug"],
                "lidar2img_extra": data_info["lidar2img_extra"],
            },
        }
        if "ann_info" in data_info:
            boxes = data_info["ann_info"]["gt_bboxes_3d"]
            labels = data_info["ann_info"]["gt_labels_3d"].copy()
            mask = labels >= 0
            result["ann_info"] = {
                "gt_bboxes_3d": boxes[mask],
                "gt_names": data_info["ann_info"]["gt_names"][mask],
                "gt_labels_3d": labels[mask],
            }
        return result

    def evaluate(self,
                 results,
                 metric=('center', 'tp', 'iou'),
                 iou_thr=(0.25, 0.5),
                 center_thresholds=(0.5, 1.0, 2.0, 4.0),
                 tp_threshold=2.0,
                 min_recall=0.1,
                 min_precision=0.1,
                 distance_bins=((0.0, 10.0), (10.0, 20.0),
                                (20.0, 30.0), (30.0, 50.0)),
                 compact=False,
                 logger=None,
                 **_kwargs):
        """Evaluate MORAI boxes without requiring a nuScenes installation.

        The parent dataset's evaluator serializes nuScenes tokens and opens a
        real nuScenes database.  MORAI only borrows its info-dict layout, so
        use the generic 3D IoU AP/recall evaluator on native LiDAR boxes.
        """
        from mmdet3d.core.evaluation import indoor_eval
        from perception.detection3d.common.evaluation import (
            morai_detection_eval,
        )

        if len(results) != len(self.data_infos):
            raise AssertionError(
                f"Result count {len(results)} != dataset count "
                f"{len(self.data_infos)}")

        if metric is None:
            requested = {'center', 'tp', 'iou'}
        elif isinstance(metric, str):
            requested = {value.strip() for value in metric.split(',')}
        else:
            requested = set(metric)
        unknown = requested - {'center', 'tp', 'iou'}
        if unknown:
            raise ValueError(f"Unsupported MORAI metrics: {sorted(unknown)}")

        metrics = {}
        if requested & {'center', 'tp'}:
            center_metrics = morai_detection_eval(
                self.data_infos,
                results,
                self.CLASSES,
                use_valid_flag=self.use_valid_flag,
                center_thresholds=center_thresholds,
                tp_threshold=tp_threshold,
                min_recall=min_recall,
                min_precision=min_precision,
                distance_bins=distance_bins if 'center' in requested else (),
                include_tp='tp' in requested,
                # Compact validation still keeps the 1 m operating point;
                # the remaining thresholds are filtered below.
                include_operating_points=True)
            if 'center' not in requested:
                center_metrics = {
                    key: value for key, value in center_metrics.items()
                    if key.startswith('tp/') or key.startswith('meta/')}
            if 'tp' not in requested:
                center_metrics = {
                    key: value for key, value in center_metrics.items()
                    if not key.startswith('tp/')}
            if 'center/mAP' in center_metrics:
                # MMCV includes save_best verbatim in the checkpoint name.
                # A slash would accidentally create a path component.
                center_metrics['center_mAP'] = center_metrics['center/mAP']
            if compact:
                compact_metrics = {}
                for key, value in center_metrics.items():
                    if (key.endswith('_GT') or
                            key.endswith('_predictions') or
                            key.startswith('meta/')):
                        continue
                    if key.startswith('range/'):
                        # Retain only one summary per class and range instead
                        # of all four thresholds and operating points.
                        if key.endswith('_mean_AP') or key.endswith('/mAP'):
                            compact_metrics[key] = value
                        continue
                    if '_best_' in key:
                        # One practical operating point per class is enough
                        # for ROS score-threshold calibration.
                        if '_1m_best_' in key and key.startswith('center/'):
                            renamed = key.replace(
                                'center/', 'operating/', 1).replace(
                                    '_1m_best_', '_1m_')
                            compact_metrics[renamed] = value
                        continue
                    compact_metrics[key] = value
                center_metrics = compact_metrics
            metrics.update(center_metrics)

        if 'iou' not in requested:
            return metrics

        normalized_results = []
        for result in results:
            if "pts_bbox" in result:
                result = result["pts_bbox"]
            elif "img_bbox" in result:
                result = result["img_bbox"]
            # Fast-BEV predicts a 9D box including (vx, vy), whereas the
            # generic 3D IoU evaluator accepts geometric 7D boxes only.
            boxes_3d = result["boxes_3d"]
            boxes_3d = type(boxes_3d)(
                boxes_3d.tensor[:, :7],
                box_dim=7,
                with_yaw=boxes_3d.with_yaw).to("cpu")
            normalized_results.append({
                "boxes_3d": boxes_3d,
                "scores_3d": result["scores_3d"].detach().cpu(),
                "labels_3d": result["labels_3d"].detach().cpu(),
            })

        gt_annos = []
        for info in self.data_infos:
            if self.use_valid_flag:
                mask = np.asarray(info["valid_flag"], dtype=bool)
            else:
                mask = np.asarray(info["num_lidar_pts"]) > 0

            names = np.asarray(info["gt_names"])[mask]
            labels = np.asarray([
                self.CLASSES.index(name) if name in self.CLASSES else -1
                for name in names
            ], dtype=np.int64)
            known = labels >= 0
            boxes = np.asarray(info["gt_boxes"], dtype=np.float32)[mask][known]
            gt_annos.append({
                "gt_num": int(known.sum()),
                "gt_boxes_upright_depth": boxes[:, :7],
                "class": labels[known],
            })

        label2cat = {index: name for index, name in enumerate(self.CLASSES)}
        iou_metrics = indoor_eval(
            gt_annos,
            normalized_results,
            tuple(iou_thr),
            label2cat,
            logger=logger,
            box_type_3d=self.box_type_3d,
            box_mode_3d=self.box_mode_3d)
        metrics.update({f"iou/{key}": value
                        for key, value in iou_metrics.items()})
        return metrics
