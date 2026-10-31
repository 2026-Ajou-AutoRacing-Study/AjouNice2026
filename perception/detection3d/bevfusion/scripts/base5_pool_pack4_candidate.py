#!/usr/bin/env python3
"""Opt-in adapter for benchmark_base5_pool_variants.py; no production imports.

FP32 input/output and FP32 accumulation, identical sorted-ray reduction order,
and --fmad=false. This is channel grouping, NOT FP16/INT8 quantization. It has
no backward. A saved plan must pass the benchmark's CPU index validation first.
Compilation and one-time initialization must be outside steady-state timing.
"""
import argparse
import hashlib
import math
from pathlib import Path


_EXTENSION = None


def source_paths():
    """Let the A/B report bind the compiled CUDA source as well as this adapter."""
    return [Path(__file__).resolve().parents[1]/"plugin/experimental_pool_pack4.cu"]


def extension():
    global _EXTENSION
    if _EXTENSION is None:
        from torch.utils.cpp_extension import load
        source = source_paths()[0]
        digest = hashlib.sha256(source.read_bytes()).hexdigest()[:12]
        _EXTENSION = load(name="morai_experimental_pool_pack4_"+digest,
                          sources=[str(source)], extra_cuda_cflags=["-O3", "--fmad=false"],
                          verbose=False)
    return _EXTENSION


def forward(depth, feat, plan):
    import torch
    if torch.is_grad_enabled() or depth.requires_grad or feat.requires_grad:
        raise ValueError("Pack4 candidate is inference-only; use torch.inference_mode()")
    if depth.dtype != torch.float32 or feat.dtype != torch.float32:
        raise ValueError("FP32 pack4 candidate never casts original inputs to half/float")
    if tuple(depth.shape) != tuple(plan["input_shape"]) or tuple(feat.shape[:-1]) != (
            *plan["input_shape"][:2], *plan["input_shape"][3:]):
        raise ValueError("Activation/plan shape mismatch")
    return extension().forward(depth, feat, *(plan[key] for key in
        ("rd", "rf", "rb", "starts", "lengths")), math.prod(plan["shape"])).reshape(
            *plan["shape"], feat.shape[-1])


def self_test():
    """CPU scalar-order oracle, channel tails and empty-plan arithmetic only.

    No CUDA import/build/run. This does not qualify compiled device code; the
    parent benchmark must compare actual CUDA output against the old kernel.
    """
    import torch
    torch.manual_seed(261023)
    checks = 0
    for channels in (1,3,4,5,31,32,80,81):
        depth = torch.rand(17)
        feat = torch.randn(5, channels)
        rd = list(range(17))
        rf = [index%5 for index in rd]
        rb = [0]*7+[2]*10
        for intervals in ([], [(0,7),(7,10)]):
            scalar = torch.zeros(3,channels)
            packed = scalar.clone()
            for start,length in intervals:
                for channel in range(channels):
                    total = torch.zeros((),dtype=torch.float32)
                    for j in range(start,start+length):
                        total += depth[rd[j]]*feat[rf[j],channel]
                    scalar[rb[start],channel] = total
                for base in range(0,channels,4):
                    total = torch.zeros(min(4,channels-base),dtype=torch.float32)
                    for j in range(start,start+length):
                        total += depth[rd[j]]*feat[rf[j],base:base+len(total)]
                    packed[rb[start],base:base+len(total)] = total
            torch.testing.assert_close(packed,scalar,rtol=0,atol=0)
            checks += 1
    print({"passed":True,"cpu_only":True,"checks":checks,"cuda_compilation_tested":False})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test",action="store_true",required=True)
    parser.parse_args()
    self_test()
