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
    assert motion_module.last_direct_velocity.shape == (1, 2, 3)
    assert torch.isfinite(motion_module.last_direct_velocity).all()
    assert float(motion_module.motion_value[0].weight.grad.abs().sum()) > 0.0

    robust_module = MoraiQueryMemoryAttention(
        channels=8,
        num_heads=2,
        maximum_association_distance_m=30.0,
        robust_direct_velocity=True,
        robust_min_consensus_frames=2,
        robust_consensus_threshold_mps=2.0,
    )
    robust_current = torch.randn(1, 8, 2)
    robust_memory = torch.randn(1, 8, 6)
    robust_module(
        robust_current,
        torch.tensor([[[3.0, 0.0], [20.0, 0.0]]]),
        torch.tensor([[0, 0]]),
        robust_memory,
        # Query zero has pair velocities 1, 1, and 10 m/s.  Median consensus
        # must reject the third frame.  Query one has no mutual match.
        torch.tensor([[[2.9, 0.0], [50.0, 0.0],
                       [2.8, 0.0], [50.0, 1.0],
                       [0.0, 0.0], [50.0, 2.0]]]),
        torch.tensor([[0.9, 0.8, 0.9, 0.8, 0.9, 0.8]]),
        torch.zeros(1, 6, dtype=torch.long),
        torch.tensor([[0.1, 0.1, 0.2, 0.2, 0.3, 0.3]]),
        torch.tensor([[0, 0, 1, 1, 2, 2]]),
    )
    assert robust_module.last_direct_velocity_active_mask.tolist() == [
        [True, False]]
    assert torch.allclose(
        robust_module.last_direct_velocity[0, :, 0],
        torch.tensor([1.0, 0.0]), atol=1.0e-5)
    assert float(robust_module.last_consensus_pair_count_mean) == 2.0
    assert float(robust_module.last_query_switch_fraction) > 0.0
    assert torch.allclose(
        robust_module.last_consensus_query_fraction,
        torch.tensor(0.5))

    ambiguous_module = MoraiQueryMemoryAttention(
        channels=8,
        num_heads=2,
        maximum_association_distance_m=30.0,
        robust_direct_velocity=True,
        robust_min_consensus_frames=2,
        robust_consensus_threshold_mps=2.0,
    )
    ambiguous_module(
        torch.randn(1, 8, 1),
        torch.tensor([[[2.0, 0.0]]]),
        torch.tensor([[0]]),
        torch.randn(1, 8, 2),
        torch.tensor([[[1.9, 0.0], [0.0, 0.0]]]),
        torch.tensor([[0.9, 0.9]]),
        torch.zeros(1, 2, dtype=torch.long),
        torch.tensor([[0.1, 0.2]]),
        torch.tensor([[0, 1]]),
    )
    assert not ambiguous_module.last_direct_velocity_active_mask.any()
    assert float(ambiguous_module.last_fallback_query_fraction) == 1.0

    single_pair_module = MoraiQueryMemoryAttention(
        channels=8,
        num_heads=2,
        maximum_association_distance_m=30.0,
        robust_direct_velocity=True,
        robust_min_consensus_frames=2,
        robust_consensus_threshold_mps=2.0,
        robust_allow_single_mutual_pair=True,
        robust_single_pair_max_speed_mps=35.0,
    )
    single_pair_module(
        torch.randn(1, 8, 2),
        torch.tensor([[[2.0, 0.0], [10.0, 0.0]]]),
        torch.tensor([[0, 0]]),
        torch.randn(1, 8, 2),
        # 10 m/s is accepted; 40 m/s is rejected by the physical cap.
        torch.tensor([[[1.0, 0.0], [6.0, 0.0]]]),
        torch.tensor([[0.9, 0.9]]),
        torch.zeros(1, 2, dtype=torch.long),
        torch.tensor([[0.1, 0.1]]),
        torch.tensor([[0, 0]]),
    )
    assert single_pair_module.last_direct_velocity_active_mask.tolist() == [
        [True, False]]
    assert torch.allclose(
        single_pair_module.last_direct_velocity[0, :, 0],
        torch.tensor([10.0, 0.0]), atol=1.0e-5)
    assert torch.allclose(
        single_pair_module.last_single_pair_query_fraction,
        torch.tensor(0.5))

    # Shadow mode must be exactly P3-compatible when its blend is zero.  It
    # still computes robust diagnostics, but neither attention nor direct
    # velocity may be hard-gated.
    shadow_baseline = MoraiQueryMemoryAttention(
        channels=8, num_heads=2, maximum_association_distance_m=30.0)
    shadow_module = MoraiQueryMemoryAttention(
        channels=8,
        num_heads=2,
        maximum_association_distance_m=30.0,
        shadow_robust_association=True,
        robust_velocity_blend=0.0,
        robust_allow_single_mutual_pair=True,
    )
    shadow_module.load_state_dict(shadow_baseline.state_dict())
    shadow_current_features = torch.randn(1, 8, 1)
    shadow_memory_features = torch.randn(1, 8, 3)
    shadow_args = (
        shadow_current_features,
        torch.tensor([[[3.0, 0.0]]]),
        torch.tensor([[0]]),
        shadow_memory_features,
        torch.tensor([[[2.9, 0.0], [2.8, 0.0], [0.0, 0.0]]]),
        torch.tensor([[0.9, 0.9, 0.9]]),
        torch.zeros(1, 3, dtype=torch.long),
        torch.tensor([[0.1, 0.2, 0.3]]),
    )
    baseline_output = shadow_baseline(*shadow_args)
    shadow_output = shadow_module(
        *shadow_args, torch.tensor([[0, 1, 2]]))
    assert torch.allclose(shadow_output, baseline_output, atol=1.0e-6)
    assert torch.allclose(
        shadow_module.last_direct_velocity,
        shadow_baseline.last_direct_velocity,
        atol=1.0e-6)
    assert shadow_module.last_direct_velocity_active_mask.tolist() == [[True]]
    assert float(shadow_module.last_consensus_query_fraction) == 1.0

    # P6 fixed-intercept WLS must suppress a noisy short-dt displacement in
    # favour of the longer baseline. Zero logits make the two pair weights
    # identical before the dt^2 reweighting.
    pair_mean_module = MoraiQueryMemoryAttention(
        channels=8, num_heads=2, maximum_association_distance_m=30.0)
    age_wls_module = MoraiQueryMemoryAttention(
        channels=8, num_heads=2, maximum_association_distance_m=30.0,
        direct_velocity_age_power=2.0)
    age_wls_module.load_state_dict(pair_mean_module.state_dict())
    for candidate in (pair_mean_module, age_wls_module):
        for parameter in candidate.parameters():
            parameter.data.zero_()
    wls_args = (
        torch.zeros(1, 8, 1),
        torch.tensor([[[3.0, 0.0]]]),
        torch.tensor([[0]]),
        torch.zeros(1, 8, 2),
        # Pair velocities are 3 m/s at 0.1 s and 2 m/s at 0.5 s.
        torch.tensor([[[2.7, 0.0], [2.0, 0.0]]]),
        torch.tensor([[0.9, 0.9]]),
        torch.zeros(1, 2, dtype=torch.long),
        torch.tensor([[0.1, 0.5]]),
    )
    pair_mean_module(*wls_args)
    age_wls_module(*wls_args)
    pair_mean_speed = pair_mean_module.last_direct_velocity[0, 0, 0]
    age_wls_speed = age_wls_module.last_direct_velocity[0, 0, 0]
    assert torch.allclose(pair_mean_speed, torch.tensor(2.5), atol=1.0e-5)
    assert abs(float(age_wls_speed) - 2.0) < abs(float(pair_mean_speed) - 2.0)

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
    assert torch.equal(prepared["frame_indices"], torch.zeros(
        1, 2, dtype=torch.long))

    reset = bank.prepare(
        "scene-b", 1.2, (0.0, 0.0, 0.0, 0.0), channels=8,
        device=torch.device("cpu"), dtype=torch.float32)
    assert reset["features"].shape[-1] == 0
    assert reset["frame_indices"].shape[-1] == 0
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
