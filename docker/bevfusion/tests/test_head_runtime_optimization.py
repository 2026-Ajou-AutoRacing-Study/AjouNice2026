"""CPU regressions; small synthetic activations do not certify detector AP."""
import ast
import copy
import hashlib
from pathlib import Path
import sys

import pytest
import torch
from torch import nn
from mmcv.cnn.bricks.transformer import MultiheadAttention

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'docker/bevfusion'))
sys.path.insert(0, str(ROOT))
from app.head_runtime_optimization import (
    FORWARD_AST_SHA256, _tree, install_head_runtime_optimization)
from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import (
    MoraiRectangularTransFusionHead)


class SmallDecoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.self_attn = MultiheadAttention(8, 2, batch_first=True)
        self.cross_attn = MultiheadAttention(8, 2, batch_first=True)

    def forward(self, query, key, query_pos, key_pos):
        query, key = query.transpose(1, 2), key.transpose(1, 2)
        return self.cross_attn(self.self_attn(query), key=key, value=key).transpose(1, 2)


class SmallPrediction(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers = nn.ModuleDict({name: nn.Conv1d(8, channels, 1)
            for name, channels in [('center', 2), ('height', 1), ('dim', 3), ('rot', 2), ('heatmap', 3)]})

    def forward(self, value):
        return {name: module(value) for name, module in self.layers.items()}


def head():
    # Use the REAL inherited forward and class but avoid constructing a full
    # detector/loss/voxelizer. Learned modules and 16x16 inputs are CPU fixtures.
    torch.manual_seed(917)
    result = MoraiRectangularTransFusionHead.__new__(MoraiRectangularTransFusionHead)
    nn.Module.__init__(result)
    result.num_classes, result.num_proposals = 3, 200
    result.num_decoder_layers, result.nms_kernel_size, result.auxiliary = 1, 3, True
    result.test_cfg = {'dataset': 'Waymo'}
    result.shared_conv = nn.Conv2d(8, 8, 1)
    result.heatmap_head = nn.Conv2d(8, 3, 1)
    result.class_encoding = nn.Conv1d(3, 8, 1)
    result.decoder = nn.ModuleList([SmallDecoder()])
    result.prediction_heads = nn.ModuleList([SmallPrediction()])
    result.bev_pos = result.create_2D_grid(16, 16)
    return result.eval()


def outputs(value):
    with torch.inference_mode():
        return value.forward_single(torch.randn(1, 8, 16, 16, generator=torch.Generator().manual_seed(19)), [{}])[0]


def test_frozen_actual_upstream_ast_and_instance_only_topk_restore():
    value, untouched = head(), head()
    original = value.forward_single
    digest = hashlib.sha256(ast.dump(_tree(original).body[0], include_attributes=False).encode()).hexdigest()
    assert digest == FORWARD_AST_SHA256
    before, control = outputs(value), outputs(untouched)
    states = copy.deepcopy(value.state_dict())
    installed = install_head_runtime_optimization(value, proposal_topk=True)
    assert installed.report['proposal_topk'] and not installed.report['validated_accuracy']
    actual = outputs(value)
    for name in before:
        torch.testing.assert_close(actual[name], before[name], rtol=0, atol=0)
        torch.testing.assert_close(outputs(untouched)[name], control[name], rtol=0, atol=0)
    assert states.keys() == value.state_dict().keys()
    assert all(torch.equal(tensor, value.state_dict()[name]) for name, tensor in states.items())
    installed.close(); installed.close()
    assert value.forward_single == original and 'forward_single' not in value.__dict__


def test_sdpa_tuple_contract_weights_rejection_and_reversal():
    value = head()
    before = outputs(value)
    installed = install_head_runtime_optimization(value, attention_sdpa=True)
    assert installed.report['attention_modules'] == 2
    actual = outputs(value)
    for name in before:
        torch.testing.assert_close(actual[name], before[name], rtol=1e-5, atol=1e-5)
    with torch.inference_mode():
        tensor = torch.randn(4, 1, 8)
        output, weights = value.decoder[0].cross_attn.attn(tensor, tensor, tensor)
        assert output.shape == tensor.shape and weights is None
        with pytest.raises(ValueError, match='does not return'):
            value.decoder[0].cross_attn.attn(tensor, tensor, tensor, need_weights=True)
        with pytest.raises(ValueError, match='does not return'):
            value.decoder[0].cross_attn.attn(tensor, tensor, tensor, None, True)
    installed.close()
    assert 'forward' not in value.decoder[0].cross_attn.attn.__dict__


def test_position_cache_is_exact_invalidates_mutation_and_has_no_state_keys():
    value = head()
    before = outputs(value)
    states = set(value.state_dict())
    installed = install_head_runtime_optimization(value, cache_bev_pos=True)
    actual = outputs(value)
    assert set(value.state_dict()) == states
    for name in before:
        torch.testing.assert_close(actual[name], before[name], rtol=0, atol=0)
    feature = torch.zeros(1, 8, 16, 16)
    first = installed._position(feature, 1)
    assert first is installed._position(feature, 1)
    value.bev_pos.add_(.125)
    second = installed._position(feature, 1)
    assert first is not second
    assert torch.equal(second, value.bev_pos.repeat(1, 1, 1))
    installed.close()
    assert not installed.positions


@pytest.mark.parametrize('flags', [dict(proposal_topk=True), dict(attention_sdpa=True),
                                  dict(cache_bev_pos=True)])
def test_no_grad_training_and_batch_guards(flags):
    value = head()
    installed = install_head_runtime_optimization(value, **flags)
    with pytest.raises(RuntimeError, match='inference_mode'):
        value.forward_single(torch.zeros(1, 8, 16, 16), [{}])
    value.train()
    with torch.inference_mode(), pytest.raises(RuntimeError, match='eval'):
        value.forward_single(torch.zeros(1, 8, 16, 16), [{}])
    value.eval()
    with torch.inference_mode(), pytest.raises(ValueError, match='batch size one'):
        value.forward_single(torch.zeros(2, 8, 16, 16), [{}, {}])
    installed.close()


def test_duplicate_changed_binding_class_and_training_rejected_before_mutation():
    value = head()
    with pytest.raises(ValueError, match='at least one'):
        install_head_runtime_optimization(value)
    value.train()
    with pytest.raises(ValueError, match='eval'):
        install_head_runtime_optimization(value, proposal_topk=True)
    value.eval()
    installed = install_head_runtime_optimization(value, proposal_topk=True)
    with pytest.raises(ValueError, match='already installed'):
        install_head_runtime_optimization(value, proposal_topk=True)
    installed.close()
    value.forward_single = lambda *_: None
    with pytest.raises(ValueError, match='inherited upstream'):
        install_head_runtime_optimization(value, proposal_topk=True)
    value = head(); value.num_classes = 4
    with pytest.raises(ValueError, match='three-class single-frame'):
        install_head_runtime_optimization(value, attention_sdpa=True)


def test_modified_attention_is_rejected_atomically():
    value = head()
    original = value.forward_single
    value.decoder[0].cross_attn.attn.forward = lambda *_: None
    with pytest.raises(ValueError, match='unmodified MMCV'):
        install_head_runtime_optimization(value, attention_sdpa=True, proposal_topk=True)
    assert value.forward_single == original
    assert 'forward' not in value.decoder[0].self_attn.attn.__dict__


def test_close_refuses_to_overwrite_another_agent_wrapper():
    value = head()
    installed = install_head_runtime_optimization(value, proposal_topk=True)
    ours = value.forward_single
    value.forward_single = lambda *_: 'another wrapper'
    with pytest.raises(RuntimeError, match='ownership changed'):
        installed.close()
    value.forward_single = ours
    installed.close()
