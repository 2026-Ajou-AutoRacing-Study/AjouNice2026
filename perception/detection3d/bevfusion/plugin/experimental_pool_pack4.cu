// Inference-only FP32 pack4 experiment. Not a production/training kernel.
// Preserve every channel's original sorted-ray multiply/add order.
// Build with --fmad=false, as for separated_bev_pool.cu; no half conversion.
#include <torch/extension.h>
#include <ATen/cuda/CUDAContext.h>
#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAException.h>
#include <limits>

__global__ void pool_pack4(const float* depth, const float* feat,
    const int* rd, const int* rf, const int* rb, const int* starts,
    const int* lengths, float* out, int channels, int intervals) {
  const int tiles = (channels + 3) / 4;
  const int64_t task = static_cast<int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const int64_t interval64 = task / tiles;
  if (interval64 >= intervals) return;
  const int interval = static_cast<int>(interval64);
  const int channel = (task % tiles) * 4;
  const int start = starts[interval];
  const int end = start + lengths[interval];
  float sum[4] = {0.f, 0.f, 0.f, 0.f};
  for (int j = start; j < end; ++j) {
    const float weight = depth[rd[j]];
    const int64_t offset = static_cast<int64_t>(rf[j]) * channels + channel;
#pragma unroll
    for (int k = 0; k < 4; ++k) {
      if (channel + k < channels) sum[k] += weight * feat[offset + k];
    }
  }
  const int64_t output = static_cast<int64_t>(rb[start]) * channels + channel;
#pragma unroll
  for (int k = 0; k < 4; ++k) {
    if (channel + k < channels) out[output + k] = sum[k];
  }
}

torch::Tensor forward(torch::Tensor depth, torch::Tensor feat,
    torch::Tensor rd, torch::Tensor rf, torch::Tensor rb,
    torch::Tensor starts, torch::Tensor lengths, int64_t cells) {
  for (const auto& t : {depth, feat}) {
    TORCH_CHECK(t.is_cuda() && t.is_contiguous() && t.scalar_type() == torch::kFloat32,
                "Pack4 requires contiguous CUDA FP32 activations; no implicit quantization");
    TORCH_CHECK(t.device() == depth.device() && !t.requires_grad(),
                "Pack4 is inference-only and requires one device");
  }
  TORCH_CHECK(feat.dim() > 0 && feat.size(-1) > 0 &&
              feat.size(-1) <= std::numeric_limits<int>::max() - 3,
              "Invalid channel count");
  for (const auto& t : {rd, rf, rb, starts, lengths}) {
    TORCH_CHECK(t.is_cuda() && t.is_contiguous() && t.scalar_type() == torch::kInt32 &&
                t.dim() == 1 && t.device() == depth.device(),
                "Expected same-device contiguous one-dimensional int32 indices");
  }
  TORCH_CHECK(rd.numel() == rf.numel() && rd.numel() == rb.numel() &&
              starts.numel() == lengths.numel(), "Index shape mismatch");
  TORCH_CHECK(rd.numel() <= std::numeric_limits<int>::max() &&
              starts.numel() <= std::numeric_limits<int>::max(), "Index count overflow");
  TORCH_CHECK(cells > 0 && cells <= std::numeric_limits<int64_t>::max()/feat.size(-1),
              "Invalid output size");
  TORCH_CHECK((rd.numel() == 0) == (starts.numel() == 0),
              "Empty rays and intervals must agree");
  // Index VALUES/coverage are validated once on saved CPU activations by the
  // benchmark CLI. Re-reading GPU indices on the CPU here would hide a sync in
  // the measured hot path. This low-level experimental entry is not public input.
  c10::cuda::CUDAGuard guard(depth.device());
  auto out = torch::zeros({cells, feat.size(-1)}, feat.options());
  if (starts.numel()) {
    const int64_t tasks = starts.numel() * ((feat.size(-1)+3)/4);
    TORCH_CHECK((tasks+255)/256 <= std::numeric_limits<int>::max(), "Grid size overflow");
    pool_pack4<<<(tasks+255)/256,256,0,at::cuda::getCurrentCUDAStream()>>>(
        depth.data_ptr<float>(), feat.data_ptr<float>(), rd.data_ptr<int>(),
        rf.data_ptr<int>(), rb.data_ptr<int>(), starts.data_ptr<int>(),
        lengths.data_ptr<int>(), out.data_ptr<float>(), feat.size(-1), starts.numel());
    C10_CUDA_KERNEL_LAUNCH_CHECK();
  }
  return out;
}

PYBIND11_MODULE(TORCH_EXTENSION_NAME, m) {
  m.def("forward", &forward);
}
