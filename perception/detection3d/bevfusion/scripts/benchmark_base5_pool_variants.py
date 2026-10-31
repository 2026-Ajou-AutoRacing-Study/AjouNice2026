#!/usr/bin/env python3
"""Bounded, opt-in inference pooling A/B on a trusted saved activation.

This does not load a detector/checkpoint, train, access ROS/the network, replace a
production method, or provide whole-model latency. CUDA runs require --execute.
The unchanged separated_bev_pool.cu reference retains --fmad=false and launches
on the current PyTorch stream. A candidate must return FP32 [B,Z,X,Y,C].

Activation schema (torch.save of CPU tensors and JSON-compatible metadata):
  schema: "base5_pool_activation_v1"
  depth: [B,N,D,H,W], feat: [B,N,H,W,C]
  plan: rd/rf/rb/starts/lengths contiguous int32 tensors, shape: (B,Z,X,Y),
        input_shape: (B,N,D,H,W)
  metadata: optional checkpoint/config/sample/precision identifiers

Save actual inputs before SeparatedPool.apply, not synthetic features and not a
post-pooling tensor. Do not silently quantize FP32 activations to half. FP16-load
+ FP32-accumulate can preserve the original AMP half input values; casting a
genuinely FP32 input to half is a separate numerical change.

Optional trusted local candidate adapter (--candidate path.py:function):
  def function(depth, feat, plan): -> FP32 CUDA tensor [B,Z,X,Y,C]
It must preserve indices, ray reduction order, output zeroing and current-stream
semantics. Timed candidate calls include their own casts/allocations. Compilation
and first-use setup are warm-up, not steady state. This CLI makes no assertion
that a candidate is safe for production merely because activation parity passes.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time


SCHEMA = "base5_pool_activation_v1"
PLAN_KEYS = ("rd", "rf", "rb", "starts", "lengths")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 2**20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_payload(payload, torch):
    """Validate on CPU before an extension can dereference any device index."""
    if payload.get("schema") != SCHEMA:
        raise ValueError(f"Expected activation schema {SCHEMA}")
    depth, feat, plan = (payload[key] for key in ("depth", "feat", "plan"))
    for name, tensor in (("depth", depth), ("feat", feat)):
        if not isinstance(tensor, torch.Tensor) or tensor.device.type != "cpu":
            raise ValueError(f"{name}: expected CPU tensor")
        if tensor.dtype not in (torch.float16, torch.float32):
            raise ValueError(f"{name}: expected original FP16 or FP32 activation")
        if tensor.requires_grad or not tensor.is_contiguous():
            raise ValueError(f"{name}: expected detached contiguous activation")
        if tensor.ndim != 5 or not all(tensor.shape):
            raise ValueError(f"{name}: expected nonempty five-dimensional tensor")
        if not bool(torch.isfinite(tensor).all()):
            raise ValueError(f"{name}: nonfinite activation")
    shape = tuple(plan["shape"])
    input_shape = tuple(plan["input_shape"])
    if len(shape) != 4 or any(type(n) is not int or n <= 0 for n in shape):
        raise ValueError("plan.shape: expected positive integer B,Z,X,Y")
    if input_shape != tuple(depth.shape) or shape[0] != depth.shape[0]:
        raise ValueError("Depth/plan shape mismatch")
    if tuple(feat.shape[:-1]) != (*input_shape[:2], *input_shape[3:]):
        raise ValueError("Feature/depth shape mismatch")
    for key in PLAN_KEYS:
        tensor = plan[key]
        if not isinstance(tensor, torch.Tensor) or tensor.device.type != "cpu":
            raise ValueError(f"plan.{key}: expected CPU tensor")
        if tensor.dtype != torch.int32 or tensor.ndim != 1 or not tensor.is_contiguous():
            raise ValueError(f"plan.{key}: expected contiguous 1D int32")
    rd, rf, rb, starts, lengths = (plan[key] for key in PLAN_KEYS)
    if len(rd) != len(rf) or len(rd) != len(rb) or len(starts) != len(lengths):
        raise ValueError("Index shape mismatch")
    if len(rd):
        for key, upper in (("rd", depth.numel()),
                           ("rf", feat.numel() // feat.shape[-1]),
                           ("rb", math.prod(shape))):
            tensor = plan[key]
            if int(tensor.min()) < 0 or int(tensor.max()) >= upper:
                raise ValueError(f"plan.{key}: out-of-range index")
        if len(torch.unique(rd)) != len(rd):
            raise ValueError("Repeated depth index; not an original frustum plan")
        if not len(starts) or int(starts[0]) != 0 or bool((lengths <= 0).any()):
            raise ValueError("Invalid nonempty interval coverage")
        ends = starts.to(torch.int64) + lengths
        if int(ends[-1]) != len(rd) or not torch.equal(ends[:-1], starts[1:].long()):
            raise ValueError("Intervals must cover sorted valid rays exactly once")
        interval_cells = rb[starts.long()]
        if len(torch.unique(interval_cells)) != len(interval_cells):
            raise ValueError("Repeated cell interval would race in reference CUDA")
        # Each interval has one output writer; all rays must belong to its cell.
        expected_cells = torch.repeat_interleave(interval_cells, lengths.long())
        if not torch.equal(rb, expected_cells):
            raise ValueError("Mixed output cells inside an interval")
        # rf is derived from rd by the production plan builder, not arbitrary.
        _, _, depth_bins, height, width = input_shape
        expected_rf = (rd.long() // (depth_bins * height * width)) * height * width
        expected_rf += rd.long() % (height * width)
        if not torch.equal(rf.long(), expected_rf):
            raise ValueError("Feature index does not match original depth ray")
    elif len(starts) or len(lengths):
        raise ValueError("Empty rays must have empty intervals")
    metadata = payload.get("metadata", {})
    json.dumps(metadata, allow_nan=False)
    return dict(input_shape=list(input_shape), output_shape=[*shape, feat.shape[-1]],
                depth_dtype=str(depth.dtype), feature_dtype=str(feat.dtype),
                valid_rays=len(rd), intervals=len(starts),
                input_bytes=depth.numel()*depth.element_size()+feat.numel()*feat.element_size(),
                expanded_fp32_input_bytes=4*(depth.numel()+feat.numel()),
                output_fp32_bytes=4*math.prod(shape)*feat.shape[-1], metadata=metadata)


def runtime_layout(output):
    """Exact current pool wrapper layout, including rectangular final transpose."""
    import torch
    value = output.permute(0, 4, 1, 2, 3).contiguous()
    value = torch.cat(value.unbind(dim=2), dim=1)
    return value.transpose(-1, -2).contiguous()


def fused_runtime_layout(output):
    """Opt-in one-copy equivalent; Z-major then C order matches unbind+cat."""
    batch, z_cells, x_cells, y_cells, channels = output.shape
    return output.permute(0,1,4,3,2).contiguous().reshape(
        batch,z_cells*channels,y_cells,x_cells)


def cpu_reference(depth, feat, plan, torch):
    """Tiny self-test oracle: same scalar multiply/add order, no parallel scatter."""
    out = torch.zeros((*plan["shape"], feat.shape[-1]), dtype=torch.float32)
    flat = out.reshape(-1, feat.shape[-1])
    depth, feat = depth.float().reshape(-1), feat.float().reshape(-1, feat.shape[-1])
    for start, length in zip(plan["starts"].tolist(), plan["lengths"].tolist()):
        for index in range(start, start + length):
            flat[int(plan["rb"][index])] += depth[int(plan["rd"][index])] * feat[int(plan["rf"][index])]
    return out


def self_test(torch):
    payload = dict(schema=SCHEMA, depth=torch.tensor([.5, .25, .75, .5]).reshape(1,1,2,1,2),
                   feat=torch.tensor([1.,2.,3.,4.,5.,6.]).reshape(1,1,1,2,3),
                   plan=dict(shape=(1,1,2,2), input_shape=(1,1,2,1,2),
                             rd=torch.tensor([0,1,2,3], dtype=torch.int32),
                             rf=torch.tensor([0,1,0,1], dtype=torch.int32),
                             rb=torch.tensor([0,0,3,3], dtype=torch.int32),
                             starts=torch.tensor([0,2], dtype=torch.int32),
                             lengths=torch.tensor([2,2], dtype=torch.int32)))
    validate_payload(payload, torch)
    expected = torch.tensor([[1.5,2.25,3.], [0.,0.,0.], [0.,0.,0.], [2.75,4.,5.25]])
    actual = cpu_reference(payload["depth"], payload["feat"], payload["plan"], torch)
    torch.testing.assert_close(actual.reshape(-1,3), expected, atol=0, rtol=0)
    assert tuple(runtime_layout(actual).shape) == (1,3,2,2)
    count = 2
    for shape in ((1,1,3,5,7), (2,3,5,7,4)):
        values = torch.arange(math.prod(shape),dtype=torch.float32).reshape(shape)
        torch.testing.assert_close(fused_runtime_layout(values),runtime_layout(values),rtol=0,atol=0)
        count += 1
    for key, index, value in (("rd",0,-1), ("rf",0,1), ("rb",1,1),
                              ("starts",1,1), ("lengths",1,1), ("rd",1,0)):
        bad = copy.deepcopy(payload)
        bad["plan"][key][index] = value
        try:
            validate_payload(bad, torch)
        except ValueError:
            count += 1
        else:
            raise AssertionError(f"Invalid {key} was accepted")
    empty = copy.deepcopy(payload)
    for key in PLAN_KEYS:
        empty["plan"][key] = torch.empty(0, dtype=torch.int32)
    validate_payload(empty, torch)
    assert not bool(cpu_reference(empty["depth"], empty["feat"], empty["plan"], torch).any())
    count += 1
    print(json.dumps(dict(passed=True, cpu_only=True, checks=count)))


def statistics(values, np):
    array = np.asarray(values, dtype=np.float64)
    return dict(n=len(values), mean=float(array.mean()), minimum=float(array.min()),
                **{f"p{p}":float(np.percentile(array,p)) for p in (50,90,95,99)},
                maximum=float(array.max()))


def compare(actual, expected, atol, rtol, torch, np):
    if actual.shape != expected.shape or actual.dtype != torch.float32:
        raise ValueError("Candidate must return FP32 [B,Z,X,Y,C], unchanged shape")
    if not bool(torch.isfinite(actual).all()):
        raise ValueError("Nonfinite candidate output")
    a = actual.detach().cpu().numpy()
    b = expected.detach().cpu().numpy()
    difference = np.abs(a - b)
    active = b != 0
    delta = difference[active]
    return dict(passed=bool(np.all(difference <= atol + rtol*np.abs(b))),
                bitwise_equal=bool(np.array_equal(a,b)), atol=atol, rtol=rtol,
                max_abs=float(difference.max()),
                changed_elements=int(np.count_nonzero(difference)), total_elements=int(a.size),
                reference_nonzero=int(np.count_nonzero(active)),
                abs_error_nonzero_reference=(statistics(delta, np) if delta.size else None),
                warning="Activation-only engineering parity is not AP/ROS/tracker qualification")


def load_candidate(spec):
    path_text, separator, symbol = spec.rpartition(":")
    if not separator or not symbol.isidentifier():
        raise ValueError("--candidate requires trusted_local.py:function")
    path = Path(path_text).resolve(strict=True)
    module_spec = importlib.util.spec_from_file_location("base5_pool_ab_candidate", path)
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    function = getattr(module, symbol)
    if not callable(function):
        raise ValueError("Candidate is not callable")
    dependency_paths = getattr(module, "source_paths", lambda:[])()
    return path, function, [Path(item).resolve(strict=True) for item in dependency_paths]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", type=Path, help="Trusted CPU activation .pt; loaded with weights_only=True")
    parser.add_argument("--output", type=Path, help="Fresh JSON report; never overwritten")
    parser.add_argument("--execute", action="store_true", help="Opt in to CUDA extension build and bounded GPU calls")
    parser.add_argument("--self-test", action="store_true", help="Tiny CPU-only schema/oracle tests")
    parser.add_argument("--candidate", help="Optional trusted local adapter path.py:function")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--iterations", type=int, default=50)
    parser.add_argument("--atol", type=float, default=0.)
    parser.add_argument("--rtol", type=float, default=0.)
    args = parser.parse_args()
    import torch
    if args.self_test:
        if args.execute or args.input or args.output or args.candidate:
            parser.error("--self-test cannot run CUDA, read activations or write reports")
        self_test(torch)
        return
    if not args.input or not args.output:
        parser.error("--input and --output are required")
    if args.output.exists():
        raise FileExistsError("Use a fresh --output; prior evidence is immutable")
    if not 0 <= args.warmup <= 100 or not 1 <= args.iterations <= 1000:
        parser.error("Bounded warmup 0..100 and iterations 1..1000 required")
    if args.atol < 0 or args.rtol < 0 or not math.isfinite(args.atol+args.rtol):
        parser.error("Finite nonnegative parity tolerances required")
    if args.candidate and not args.execute:
        parser.error("Candidate source is never imported without --execute")
    import numpy as np
    root = Path(__file__).resolve().parents[4]
    sys.path.insert(0, str(root))
    plugin_path = root/"perception/detection3d/bevfusion/plugin/morai_pool_optimization.py"
    kernel_path = plugin_path.with_name("separated_bev_pool.cu")
    paths = [Path(__file__).resolve(), args.input.resolve(strict=True), plugin_path, kernel_path]
    adapter = None
    if args.candidate:
        adapter_path = Path(args.candidate.rpartition(":")[0]).resolve(strict=True)
        adapter_before = sha256(adapter_path)
        _, adapter, dependencies = load_candidate(args.candidate)
        if sha256(adapter_path) != adapter_before:
            raise RuntimeError("Candidate source changed during import")
        paths.extend([adapter_path, *dependencies])
    pins = {str(path):sha256(path) for path in paths}
    payload = torch.load(args.input, map_location="cpu", weights_only=True)
    report = dict(schema="base5_pool_ab_report_v1", activation=validate_payload(payload, torch),
                  execute=args.execute, source_sha256=pins, completed=False,
                  contract=dict(inference_only=True, production_changed=False, backward_tested=False,
                                indices_and_geometry_changed=False, fma_reference="--fmad=false",
                                default_parity="exact", includes_model_preprocessing_ros=False))
    if args.execute:
        if not torch.cuda.is_available() or not args.device.startswith("cuda:"):
            raise ValueError("Explicit available CUDA device required")
        from perception.detection3d.bevfusion.plugin.morai_pool_optimization import extension
        torch.cuda.set_device(args.device)
        device = torch.device(args.device)
        depth, feat = (payload[key].to(device) for key in ("depth", "feat"))
        plan = {key:(value.to(device) if torch.is_tensor(value) else value)
                for key,value in payload["plan"].items() if key in (*PLAN_KEYS,"shape","input_shape")}
        cells = math.prod(plan["shape"])
        index_args = tuple(plan[key] for key in PLAN_KEYS)
        engine = extension()  # Unchanged reference: CUDA O3 and --fmad=false.
        def cast():
            return depth.contiguous().float(), feat.contiguous().float()
        d32, f32 = cast()
        def kernel():
            return engine.forward(d32, f32, *index_args, cells).reshape(*plan["shape"],feat.shape[-1])
        def reference():
            d, f = cast()
            return engine.forward(d, f, *index_args, cells).reshape(*plan["shape"],feat.shape[-1])
        candidate = None
        if args.candidate:
            candidate = lambda: adapter(depth, feat, plan)
        report["environment"] = dict(torch=torch.__version__, cuda=torch.version.cuda,
            gpu=torch.cuda.get_device_name(device), capability=list(torch.cuda.get_device_capability(device)),
            stream="new nondefault current PyTorch stream", stream_tested_default=False)
        # Producer copies happen on default; establish dependency before side-stream work.
        stream = torch.cuda.Stream(device=device)
        stream.wait_stream(torch.cuda.current_stream(device))
        rows = {}
        with torch.inference_mode(), torch.cuda.stream(stream), torch.autocast("cuda", enabled=False):
            expected = reference()
            stream.synchronize()
            layout_input = expected
            functions = dict(cast_only=cast, reference_kernel_and_zeroing=kernel,
                             layout_only=lambda:runtime_layout(layout_input),
                             fused_layout_only=lambda:fused_runtime_layout(layout_input),
                             reference_full=lambda:runtime_layout(reference()),
                             reference_fused_layout=lambda:fused_runtime_layout(reference()))
            fused = fused_runtime_layout(expected)
            if not torch.equal(fused,runtime_layout(expected)):
                raise AssertionError("One-copy layout changed tensor values/order")
            report["fused_layout_parity"] = dict(bitwise_equal=True,
                input_shape=list(expected.shape), output_shape=list(fused.shape))
            del fused
            if candidate:
                report["candidate_parity"] = compare(candidate(), expected, args.atol, args.rtol, torch, np)
                functions["candidate_full"] = lambda:runtime_layout(candidate())
                functions["candidate_fused_layout"] = lambda:fused_runtime_layout(candidate())
            for function in functions.values():
                for _ in range(args.warmup):
                    value = function()
                    del value
                stream.synchronize()
            samples = {name:dict(cuda_event_ms=[], synchronized_wall_ms=[]) for name in functions}
            names = list(functions)
            for iteration in range(args.iterations):
                # Alternate order so a consistently later variant cannot exploit warm-up drift.
                for name in (names if iteration%2==0 else names[::-1]):
                    begin, end = (torch.cuda.Event(enable_timing=True) for _ in range(2))
                    wall = time.perf_counter()
                    begin.record(stream)
                    value = functions[name]()
                    end.record(stream)
                    end.synchronize()
                    samples[name]["synchronized_wall_ms"].append((time.perf_counter()-wall)*1000)
                    samples[name]["cuda_event_ms"].append(begin.elapsed_time(end))
                    del value
            for name,function in functions.items():
                torch.cuda.reset_peak_memory_stats(device)
                before = torch.cuda.memory_allocated(device)
                value = function()
                stream.synchronize()
                rows[name] = {key:statistics(values,np) for key,values in samples[name].items()}
                rows[name]["additional_peak_allocated_bytes"] = torch.cuda.max_memory_allocated(device)-before
                del value
        torch.cuda.current_stream(device).wait_stream(stream)
        report["timing"] = rows
        report["timing_scope"] = "Isolated saved activation; precomputed geometry/indices; kernel row includes output zeros/allocation; not full-model/overlap latency"
        report["warmup"] = args.warmup
        report["iterations"] = args.iterations
    report["sources_unchanged"] = all(sha256(Path(path)) == digest for path,digest in pins.items())
    if not report["sources_unchanged"]:
        raise RuntimeError("Activation or source changed during benchmark; no completed report written")
    report["completed"] = True
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps(dict(completed=True, output=str(args.output), executed=args.execute,
                          candidate_parity=report.get("candidate_parity"))))
    if report.get("candidate_parity",{}).get("passed") is False:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
