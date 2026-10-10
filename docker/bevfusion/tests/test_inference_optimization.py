import pytest
import torch
from torch import nn

from app.inference_optimization import InferenceOptimization, validate_base5_runtime


class Toy(nn.Module):
    def __init__(self):
        super().__init__()
        self.img_backbone = nn.Identity()
        self.img_neck = nn.Sequential(nn.Conv2d(2, 3, 1), nn.BatchNorm2d(3), nn.ReLU())
        self.fusion_layer = nn.Identity()
        self.pts_backbone = nn.Identity()
        self.pts_neck = nn.Identity()
        self.view_transform = nn.Identity()


def test_fold_reversible_and_training_guard():
    torch.manual_seed(17)
    model = Toy().eval(); x = torch.randn(2, 2, 3, 3)
    original = {k:v.clone() for k,v in model.state_dict().items()}
    before = model.img_neck(x)
    optimization = InferenceOptimization(model, fuse_dense_bn=True)
    assert optimization.report['folded_dense_bn'] == 1
    torch.testing.assert_close(before, model.img_neck(x), rtol=1e-5, atol=1e-6)
    with pytest.raises(RuntimeError, match='Disable'):
        model.train()
    optimization.disable(); optimization.disable()
    for key, value in model.state_dict().items():
        assert torch.equal(value, original[key])
    model.train(); assert model.training


def test_compile_callable_preserves_keys(monkeypatch):
    model = Toy().eval(); keys = tuple(model.state_dict()); calls = []
    def compile_mock(fn, **kwargs):
        calls.append(kwargs)
        return lambda *args, **kw: fn(*args, **kw)
    monkeypatch.setattr(torch, 'compile', compile_mock)
    old = model.img_backbone.forward
    optimization = InferenceOptimization(model, ('camera',))
    assert tuple(model.state_dict()) == keys
    assert calls == [dict(dynamic=False, options={'emulate_precision_casts': True})]
    optimization.disable(); assert model.img_backbone.forward == old


def test_invalid_compile_and_training_rejected():
    with pytest.raises(ValueError, match='eval'):
        InferenceOptimization(Toy())
    with pytest.raises(ValueError, match='Unknown'):
        InferenceOptimization(Toy().eval(), ('sparse',))
    with pytest.raises(ValueError, match='OR CUDA Graph'):
        InferenceOptimization(Toy().eval(), ('camera',), camera_cuda_graph=True)
    with pytest.raises(ValueError, match='requires camera'):
        InferenceOptimization(Toy().eval(), eager_camera_numerics=True)
    with pytest.raises(ValueError, match='requires a compiled'):
        InferenceOptimization(Toy().eval(), reference_layout=True)


def test_base5_rejects_default_and_partial_contract_before_loading(monkeypatch):
    from app import bevfusion_model
    from mmengine.config import Config
    config = Config(dict(metainfo=dict(classes=['vehicle', 'pedestrian', 'obstacle', 'animal']),
        model=dict(img_backbone=dict(init_cfg=None), bbox_head=dict(type='fake'))))
    monkeypatch.setattr(bevfusion_model.Config, 'fromfile', lambda path: config)
    monkeypatch.setattr(bevfusion_model, 'init_model', lambda *a, **kw: pytest.fail('Loaded invalid contract'))
    with pytest.raises(ValueError, match='explicit --base5-runtime'):
        bevfusion_model.BevFusionModel('config', 'checkpoint', {})


def test_base5_config_contract():
    from mmengine.config import Config
    config = Config(dict(metainfo=dict(classes=['vehicle', 'pedestrian', 'obstacle', 'animal']),
        required_camera_input_policy='native_jpeg90',
        required_box_geometry_policy='native_rpy_fixed_yaw_enclosure_v1',
        model=dict(type='BEVFusion', pts_voxel_encoder=dict(num_features=4),
                   bbox_head=dict(bbox_coder=dict(code_size=8), common_heads=dict(rot=(2, 2))))))
    validate_base5_runtime(config)
    config.model.bbox_head.common_heads.vel = (2, 2)
    with pytest.raises(ValueError, match='velocity'): validate_base5_runtime(config)
    del config.model.bbox_head.common_heads.vel
    config.metainfo.classes = config.metainfo.classes[:3]
    with pytest.raises(ValueError, match='four-class'): validate_base5_runtime(config)


def test_mmcv_convmodule_fold():
    from mmcv.cnn import ConvModule
    model = Toy().eval()
    model.img_neck = ConvModule(2, 3, 1, norm_cfg=dict(type='BN')).eval()
    x = torch.randn(1, 2, 3, 3); before = model.img_neck(x)
    optimization = InferenceOptimization(model, fuse_dense_bn=True)
    assert optimization.report['folded_dense_bn'] == 1
    torch.testing.assert_close(before, model.img_neck(x), rtol=1e-5, atol=1e-6)
    optimization.disable(); assert isinstance(model.img_neck.norm, nn.BatchNorm2d)


@pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA required')
@pytest.mark.parametrize('amp', [False, True])
def test_camera_graph_replay_and_restore(amp):
    model = Toy().cuda().eval()
    model.img_backbone = nn.Sequential(nn.Conv2d(2, 4, 1), nn.GELU(), nn.Conv2d(4, 2, 1)).cuda().eval()
    original = model.img_backbone.forward
    keys = tuple(model.state_dict())
    x, y = torch.randn(1, 2, 8, 8, device='cuda'), torch.randn(1, 2, 8, 8, device='cuda')
    with torch.inference_mode(), torch.autocast('cuda', enabled=amp, dtype=torch.float16):
        expected_x, expected_y = original(x), original(y)
        optimization = InferenceOptimization(model, camera_cuda_graph=True)
        actual_x = model.img_backbone(x).clone()
        actual_y = model.img_backbone(y).clone()
        torch.cuda.synchronize()
        assert torch.equal(expected_x, actual_x)
        assert torch.equal(expected_y, actual_y)
        assert tuple(model.state_dict()) == keys
        optimization.disable()
        assert model.img_backbone.forward == original
