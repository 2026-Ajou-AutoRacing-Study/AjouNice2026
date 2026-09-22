#!/usr/bin/env python3
"""Small deterministic checks for the Base4 T2 temporal BEV gate."""

import torch

from perception.detection3d.bevfusion.plugin.morai_temporal_bevfusion import (
    MoraiTemporalBEVGate,
    MoraiTemporalBEVResidual,
)


def main():
    gate = MoraiTemporalBEVGate(channels=4, num_sweeps=3)
    current = torch.randn(2, 4, 5, 6, requires_grad=True)
    past1 = torch.randn(2, 4, 5, 6, requires_grad=True)
    past2 = torch.randn(2, 4, 5, 6, requires_grad=True)
    output = gate([current, past1, past2])
    assert output.shape == current.shape
    weights = gate.last_mean_weights
    assert weights is not None
    assert float(weights[0]) > 0.99
    assert torch.all(weights[1:] > 0.0)
    output.square().mean().backward()
    assert gate.gate.weight.grad is not None
    assert float(gate.gate.weight.grad.abs().sum()) > 0.0
    assert past1.grad is not None and float(past1.grad.abs().sum()) > 0.0
    assert past2.grad is not None and float(past2.grad.abs().sum()) > 0.0
    print("weights:", weights.tolist())

    residual = MoraiTemporalBEVResidual(channels=4, num_sweeps=3)
    residual_output = residual([current.detach(), past1.detach(), past2.detach()])
    assert torch.equal(residual_output, current.detach())
    residual_output.square().mean().backward()
    final_weight_grad = residual.motion_encoder[-1].weight.grad
    assert final_weight_grad is not None
    assert float(final_weight_grad.abs().sum()) > 0.0
    assert float(residual.last_residual_rms) == 0.0
    print("TEMPORAL_BEV_GATE_OK")


if __name__ == "__main__":
    main()
