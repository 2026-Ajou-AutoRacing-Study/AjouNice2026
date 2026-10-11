"""Run in the team image (compiled BEVFusion ops required)."""
from pathlib import Path
import unittest
from unittest.mock import patch
import torch
from mmengine.config import Config
from projects.BEVFusion.bevfusion.depth_lss import DepthLSSTransform
from projects.BEVFusion.bevfusion.sparse_encoder import BEVFusionSparseEncoder
from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import (
    MoraiRectangularDepthLSSTransform,MoraiRectangularTransFusionHead,MoraiRectangularSparseEncoder)
from perception.detection3d.bevfusion.plugin.morai_separated_depth_lss import MoraiSeparatedRectangularDepthLSSTransform


class RectangularGeometryTest(unittest.TestCase):
    def test_sparse_encoder_maps_xyz_voxels_to_yxz_shape(self):
        encoder=object.__new__(MoraiRectangularSparseEncoder)
        coordinates=torch.tensor([[0,2000,400,25],[0,0,863,0]])
        with patch.object(BEVFusionSparseEncoder,'forward',return_value='sentinel') as forward:
            self.assertEqual(encoder.forward(None,coordinates,1),'sentinel')
        torch.testing.assert_close(forward.call_args.args[1],torch.tensor([[0,400,2000,25],[0,863,0,0]]))
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
        self.assertEqual(cfg.model.pts_middle_encoder.type,'MoraiRectangularSparseEncoder')
        self.assertNotIn('vel',cfg.model.bbox_head.common_heads)
        gx,gy,_=cfg.model.bbox_head.train_cfg.grid_size
        self.assertEqual(gx%16,0);self.assertEqual(gy%16,0)
        self.assertEqual(cfg.model.bbox_head.bbox_coder.pc_range,cfg.model.bbox_head.train_cfg.point_cloud_range[:2])
        self.assertEqual(cfg.model.view_transform.xbound[2]*2,cfg.model.bbox_head.test_cfg.voxel_size[0]*8)

    def test_opt_in_type_adds_no_checkpoint_keys(self):
        args=(8,4,(32,32),(4,4),(-8,8,1),(-6,6,1),(-8,8,16),(1,4,1))
        original=MoraiRectangularDepthLSSTransform(*args)
        optimized=MoraiSeparatedRectangularDepthLSSTransform(*args)
        self.assertEqual(set(original.state_dict()),set(optimized.state_dict()))
        self.assertEqual(optimized._pool_optimization['mode'],'separated')

    def test_dense_candidate_contract(self):
        cfg=Config.fromfile(str(Path(__file__).resolve().parents[1]/'configs/bevfusion_base5_dense_448_separated_candidate.py'))
        self.assertEqual(cfg.model.view_transform.type,'MoraiSeparatedRectangularDepthLSSTransform')
        self.assertEqual(cfg.model.view_transform.image_size,[448,896])
        self.assertEqual(cfg.model.view_transform.dbound,[1.,100.,.5])
        self.assertEqual(cfg.model.data_preprocessor.voxelize_cfg.voxel_size,[.075,.075,.2])
        self.assertEqual(cfg.model.pts_middle_encoder.sparse_shape,[864,2688,41])
        for pipeline in (cfg.train_dataloader.dataset.pipeline,cfg.val_dataloader.dataset.pipeline):
            aug=next(v for v in pipeline if v['type']=='MoraiImageAug3D')
            self.assertEqual(aug['final_dim'],[448,896]);self.assertEqual(aug['resize_lim'],[.7,.7])

    def test_depth25_runtime_is_explicit_candidate(self):
        root=Path(__file__).resolve().parents[1]/'configs'
        original=Config.fromfile(str(root/'bevfusion_base5_dense_512_native_jpeg90.py'))
        candidate=Config.fromfile(str(root/'bevfusion_base5_dense_512_depth25_runtime_candidate.py'))
        self.assertEqual(original.model.view_transform.dbound,[1.,100.,.5])
        self.assertEqual(candidate.model.view_transform.dbound,[1.,100.,.25])
        self.assertEqual(candidate.model.view_transform.pool_cache_limit_bytes,512*2**20)
        self.assertEqual(candidate.model.type,'MoraiTrainingRuntimeBEVFusion')
        self.assertFalse(candidate.model.get('compile_camera',False))
        self.assertEqual(candidate.model.view_transform.pool_geometry_mode,'batched_checked')
        self.assertEqual(candidate.train_pipeline,original.train_pipeline)
        self.assertEqual(candidate.model.bbox_head,original.model.bbox_head)
        self.assertEqual(candidate.required_box_geometry_policy,original.required_box_geometry_policy)


if __name__=='__main__':unittest.main()
