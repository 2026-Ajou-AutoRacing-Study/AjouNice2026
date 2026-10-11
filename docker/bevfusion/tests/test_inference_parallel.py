import pytest
import torch
from torch import nn

from app.inference_optimization import InferenceOptimization
from app.inference_parallel import ParallelBranches


class AddFusion(nn.Module):
    def forward(self, features):
        return features[0]+features[1]


class BranchToy(nn.Module):
    def __init__(self):
        super().__init__()
        self.img_backbone = nn.Conv2d(1, 1, 1)
        self.img_neck = nn.Identity()
        self.fusion_layer = AddFusion()
        self.seen_streams = []
        self.batch_sizes = []

    def voxelize(self, points):
        return points[0], torch.zeros(1,4,device=points[0].device,dtype=torch.int32), []

    def pts_middle_encoder(self, feats, coords, batch_size):
        self.seen_streams.append(torch.cuda.current_stream())
        self.batch_sizes.append(batch_size)
        return feats * 3

    def extract_pts_feat(self, inputs):
        feats, coords, _ = self.voxelize(inputs['points'])
        return self.pts_middle_encoder(feats,coords,len(inputs['points']))

    def extract_img_feat(self, images, points):
        return self.img_backbone(images).flatten()

    def extract_feat(self, inputs, metas):
        camera = self.extract_img_feat(inputs['imgs'],inputs['points'])
        lidar = self.extract_pts_feat(inputs)
        # Like BEVFusion, computation starts INSIDE fusion, after its join.
        return self.fusion_layer([camera,lidar])


def test_guards_and_idempotent_restore():
    model = BranchToy()
    with pytest.raises(ValueError):
        ParallelBranches(model)
    model.eval()
    with pytest.raises(ValueError):
        ParallelBranches(model, 'unknown')
    keys = tuple(model.state_dict())
    original = [model.extract_feat, model.extract_pts_feat, model.extract_img_feat, model.fusion_layer.forward]
    candidate = ParallelBranches(model,'camera_side')
    with pytest.raises(RuntimeError,match='inference-only'):
        model.extract_feat({}, [])
    candidate.disable()
    candidate.disable()
    assert original == [model.extract_feat, model.extract_pts_feat, model.extract_img_feat, model.fusion_layer.forward]
    assert keys == tuple(model.state_dict())
    with pytest.raises(ValueError,match='not qualified'):
        InferenceOptimization(model,parallel_camera=True,camera_cuda_graph=True)


def test_production_runtime_rejects_parallel_before_loading(monkeypatch):
    from app import bevfusion_model
    from mmengine.config import Config
    config = Config(dict(metainfo=dict(classes=['vehicle','pedestrian','obstacle']),
                         model=dict(img_backbone=dict(init_cfg=None),bbox_head=dict(type='fake'))))
    monkeypatch.setattr(bevfusion_model.Config,'fromfile',lambda path:config)
    monkeypatch.setattr(bevfusion_model,'init_model',lambda *args,**kwargs:pytest.fail('Loaded unsafe contract'))
    with pytest.raises(ValueError,match='Base5-only'):
        bevfusion_model.BevFusionModel('config','checkpoint',{},parallel_camera=True)


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA required')
def test_camera_side_matches_keeps_sparse_default_and_restores():
    model = BranchToy().cuda().eval()
    points = torch.arange(8,device='cuda',dtype=torch.float32)
    inputs = dict(points=[points],imgs=torch.ones(1,1,2,4,device='cuda'))
    with torch.inference_mode():
        expected = model.extract_feat(inputs,[])
        optimization = InferenceOptimization(model,parallel_camera=True)
        for _ in range(3):
            actual = model.extract_feat(inputs,[])
            torch.cuda.synchronize()
            assert torch.equal(actual,expected)
        assert all(s == torch.cuda.default_stream() for s in model.seen_streams)
        assert model.batch_sizes == [1]*4
        with pytest.raises(RuntimeError,match='Disable'):
            model.train()
        with torch.cuda.stream(torch.cuda.Stream()):
            with pytest.raises(RuntimeError,match='default CUDA stream'):
                model.extract_feat(inputs,[])
        optimization.disable()
        assert torch.equal(model.extract_feat(inputs,[]),expected)


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA required')
def test_exception_drains_side_and_next_call_works():
    model = BranchToy().cuda().eval()
    inputs = dict(points=[torch.ones(8,device='cuda')],imgs=torch.ones(1,1,2,4,device='cuda'))
    candidate = ParallelBranches(model,'camera_side')
    original = candidate.fusion
    candidate.fusion = lambda *args,**kwargs: (_ for _ in ()).throw(RuntimeError('test failure'))
    with torch.inference_mode():
        with pytest.raises(RuntimeError,match='test failure'):
            model.extract_feat(inputs,[])
        assert candidate.pending is None
        candidate.fusion = original
        assert torch.isfinite(model.extract_feat(inputs,[])).all()
    candidate.disable()
