#!/usr/bin/env python3
"""Deterministic CPU checks for the Base4 Q1-P0 memory primitives."""

import math

import torch

from perception.detection3d.bevfusion.plugin.morai_query_memory import (
    CausalQueryMemoryBank,
    MoraiQueryMemoryAttention,
    planar_source_to_target_matrix,
    transform_query_centers,
)


def main():
    centers = torch.tensor([[1.0, 0.0], [0.0, 2.0]])
    transform = planar_source_to_target_matrix(
        (0.0, 0.0, 0.0, math.pi / 2.0),
        (0.0, 0.0, 0.0, 0.0),
    )
    rotated = transform_query_centers(centers, transform)
    assert torch.allclose(
        rotated, torch.tensor([[0.0, 1.0], [-2.0, 0.0]]), atol=1.0e-6)

    module = MoraiQueryMemoryAttention(
        channels=8, num_heads=2, maximum_association_distance_m=3.0)
    current = torch.randn(1, 8, 3, requires_grad=True)
    memory = torch.randn(1, 8, 4, requires_grad=True)
    output = module(
        current,
        torch.tensor([[[0.0, 0.0], [10.0, 0.0], [1.0, 0.0]]]),
        torch.tensor([[0, 0, 1]]),
        memory,
        torch.tensor([[[0.5, 0.0], [8.0, 0.0], [1.5, 0.0], [0.0, 0.0]]]),
        torch.tensor([[0.9, 0.9, 0.1, 0.9]]),
        torch.tensor([[0, 0, 0, 1]]),
        torch.tensor([[0.1, 0.1, 0.1, 0.1]]),
    )
    assert output.shape == current.shape
    assert 0.0 < float(module.last_active_query_fraction) < 1.0
    output.square().mean().backward()
    assert current.grad is not None and float(current.grad.abs().sum()) > 0.0
    assert memory.grad is not None and float(memory.grad.abs().sum()) > 0.0
    assert module.residual_logit.grad is not None

    motion_module = MoraiQueryMemoryAttention(
        channels=8, num_heads=2, maximum_association_distance_m=30.0,
        use_motion_value=True)
    motion_output = motion_module(
        current.detach().clone().requires_grad_(True),
        torch.tensor([[[2.0, 0.0], [10.0, 0.0], [1.0, 0.0]]]),
        torch.tensor([[0, 0, 1]]), memory.detach(),
        torch.tensor([[[0.0, 0.0], [8.0, 0.0], [1.5, 0.0], [0.0, 0.0]]]),
        torch.tensor([[0.9, 0.9, 0.1, 0.9]]),
        torch.tensor([[0, 0, 0, 1]]),
        torch.tensor([[0.2, 0.2, 0.2, 0.2]]))
    motion_output.square().mean().backward()
    assert motion_module.last_motion_context.shape == current.shape
    assert float(motion_module.motion_value[0].weight.grad.abs().sum()) > 0.0

    bank = CausalQueryMemoryBank(
        max_frames=2, max_queries_per_frame=2, maximum_age_sec=0.6,
        reset_gap_sec=0.3)
    bank.push(
        "scene-a", 1.0, (0.0, 0.0, 0.0, 0.0),
        torch.arange(24.0).reshape(8, 3),
        torch.tensor([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]]),
        torch.tensor([0.3, 0.9, 0.8]), torch.tensor([0, 0, 1]))
    prepared = bank.prepare(
        "scene-a", 1.1, (1.0, 0.0, 0.0, 0.0), channels=8,
        device=torch.device("cpu"), dtype=torch.float32)
    assert prepared["features"].shape == (1, 8, 2)
    assert torch.allclose(
        prepared["centers_xy"][0],
        torch.tensor([[1.0, 0.0], [0.0, 0.0]]), atol=1.0e-6)
    assert torch.allclose(prepared["ages_sec"], torch.full((1, 2), 0.1))

    reset = bank.prepare(
        "scene-b", 1.2, (0.0, 0.0, 0.0, 0.0), channels=8,
        device=torch.device("cpu"), dtype=torch.float32)
    assert reset["features"].shape[-1] == 0
    bank.push(
        "scene-b", 2.0, (0.0, 0.0, 0.0, 0.0),
        torch.ones(8, 1), torch.zeros(1, 2), torch.ones(1),
        torch.zeros(1, dtype=torch.long))
    gap_reset = bank.prepare(
        "scene-b", 2.5, (0.0, 0.0, 0.0, 0.0), channels=8,
        device=torch.device("cpu"), dtype=torch.float32)
    assert gap_reset["features"].shape[-1] == 0

    disabled = CausalQueryMemoryBank(max_frames=0)
    disabled.push(
        "scene-a", 1.0, (0.0, 0.0, 0.0, 0.0),
        torch.ones(8, 1), torch.zeros(1, 2), torch.ones(1),
        torch.zeros(1, dtype=torch.long))
    no_memory = disabled.prepare(
        "scene-a", 1.1, (0.0, 0.0, 0.0, 0.0), channels=8,
        device=torch.device("cpu"), dtype=torch.float32)
    assert no_memory["features"].shape[-1] == 0
    print("QUERY_MEMORY_P0_OK")


if __name__ == "__main__":
    main()
