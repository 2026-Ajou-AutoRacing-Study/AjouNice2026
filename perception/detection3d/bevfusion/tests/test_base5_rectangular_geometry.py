"""Run in the team image (compiled BEVFusion ops required)."""
from pathlib import Path
import unittest
from unittest.mock import patch
import torch
from mmengine.config import Config
from projects.BEVFusion.bevfusion.depth_lss import DepthLSSTransform
from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import (
    MoraiRectangularDepthLSSTransform,MoraiRectangularTransFusionHead)


class RectangularGeometryTest(unittest.TestCase):
    def test_grid_is_yx_flatten_with_xy_coordinates(self):
        grid=MoraiRectangularTransFusionHead.create_2D_grid(None,3,2)
        self.assertEqual(grid.tolist(),[[[.5,.5],[1.5,.5],[2.5,.5],[.5,1.5],[1.5,1.5],[2.5,1.5]]])
    def test_camera_pool_transposes_values_not_metric_bounds(self):
        model=object.__new__(MoraiRectangularDepthLSSTransform)
        camera=torch.zeros(1,1,3,2);camera[0,0,2,1]=7
        with patch.object(DepthLSSTransform,'bev_pool',return_value=camera):
            result=model.bev_pool(None,None)
        self.assertEqual(tuple(result.shape),(1,1,2,3))
        self.assertEqual(float(result[0,0,1,2]),7)
    def test_padding_roi_and_fresh_contract(self):
        cfg=Config.fromfile(str(Path(__file__).resolve().parents[1]/'configs/bevfusion_base5_independent_100m.py'))
        self.assertIsNone(cfg.load_from)
        self.assertEqual(cfg.model.bbox_head.num_classes,4)
        self.assertNotIn('vel',cfg.model.bbox_head.common_heads)
        gx,gy,_=cfg.model.bbox_head.train_cfg.grid_size
        self.assertEqual(gx%16,0);self.assertEqual(gy%16,0)
        self.assertEqual(cfg.model.bbox_head.bbox_coder.pc_range,cfg.model.bbox_head.train_cfg.point_cloud_range[:2])
        self.assertEqual(cfg.model.view_transform.xbound[2]*2,cfg.model.bbox_head.test_cfg.voxel_size[0]*8)


if __name__=='__main__':unittest.main()
