import pytest
import torch
from torch import nn

from app import pool_runtime_layout as candidate


def legacy_layout(output):
    value = output.permute(0, 4, 1, 2, 3).contiguous()
    return torch.cat(value.unbind(dim=2), dim=1).transpose(-1, -2).contiguous()


@pytest.mark.parametrize('batch', [1, 2])
@pytest.mark.parametrize('z_cells', [1, 2, 3])
@pytest.mark.parametrize('channels', [1, 4, 80])
def test_rectangular_layout_exact(batch, z_cells, channels):
    value = torch.arange(batch*z_cells*3*5*channels, dtype=torch.float32).reshape(
        batch, z_cells, 3, 5, channels)
    actual = candidate.fused_runtime_layout(value)
    assert torch.equal(actual, legacy_layout(value))
    assert actual.is_contiguous()


class MoraiRectangularDepthLSSTransform(nn.Module):
    def __init__(self):
        super().__init__()
        self.weight = nn.Parameter(torch.ones(1))
        self._pool_optimization = dict(mode='separated', backward_mode='grouped',
            plan=dict(input_shape=(1, 4, 2, 3, 5), shape=(1, 2, 3, 5)))

    def bev_pool(self, geom, features):
        return 'original'


def test_install_restore_kernel_inputs_and_state_keys(monkeypatch):
    view = MoraiRectangularDepthLSSTransform().eval()
    original, keys = view.bev_pool, tuple(view.state_dict())
    depth = torch.ones(1, 4, 2, 3, 5)
    feat = torch.ones(1, 4, 3, 5, 7)
    output = torch.arange(1*2*3*5*7).reshape(1, 2, 3, 5, 7).float()
    calls = []

    def same_kernel(d, f, plan, mode):
        calls.append((d, f, plan, mode))
        return output

    monkeypatch.setattr(candidate, '_separated_pool', same_kernel)
    handle = candidate.install_fused_pool_layout(view)
    with torch.inference_mode():
        assert torch.equal(view.bev_pool(None, (depth, feat)), legacy_layout(output))
    assert calls[0][0] is depth and calls[0][1] is feat
    assert calls[0][2] is view._pool_optimization['plan']
    assert calls[0][3] == 'grouped'
    assert tuple(view.state_dict()) == keys
    with pytest.raises(ValueError, match='already installed'):
        candidate.install_fused_pool_layout(view)
    handle.close()
    handle.close()
    assert view.bev_pool == original
    assert not handle.report['enabled']


def test_install_guards():
    view = MoraiRectangularDepthLSSTransform()
    with pytest.raises(ValueError, match='eval'):
        candidate.install_fused_pool_layout(view)
    with pytest.raises(ValueError, match='rectangular'):
        candidate.install_fused_pool_layout(nn.Identity().eval())
    view.eval()._pool_optimization['mode'] = 'cache'
    with pytest.raises(ValueError, match='separated'):
        candidate.install_fused_pool_layout(view)


def test_runtime_guards_and_wrapper_ownership():
    view = MoraiRectangularDepthLSSTransform().eval()
    handle = candidate.install_fused_pool_layout(view)
    depth, feat = torch.ones(1, 4, 2, 3, 5), torch.ones(1, 4, 3, 5, 7)
    with pytest.raises(RuntimeError, match='eval inference'):
        view.bev_pool(None, (depth, feat))
    with torch.inference_mode():
        with pytest.raises(ValueError, match='dimensions'):
            view.bev_pool(None, (depth[:, :3], feat))
        with pytest.raises(ValueError, match='pair'):
            view.bev_pool(None, depth)
        view._pool_optimization['plan'] = None
        with pytest.raises(RuntimeError, match='Geometry'):
            view.bev_pool(None, (depth, feat))
    view.bev_pool = lambda *args: None
    with pytest.raises(RuntimeError, match='Another owner'):
        handle.close()
    view.bev_pool = handle.pool
    handle.close()
