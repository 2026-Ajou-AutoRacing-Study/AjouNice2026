// Experimental depth/feature-separated pooling. Not the NVIDIA BEVPoolv3 kernel.
// Interval reduction follows the BEVPoolv2 formulation; implementation is local.
#include <torch/extension.h>
#include <ATen/cuda/CUDAContext.h>
#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAException.h>

__global__ void pool_forward(const float* depth, const float* feat,
 const int* rd, const int* rf, const int* rb, const int* starts,
 const int* lengths, float* out, int channels, int intervals) {
 int index=blockIdx.x*blockDim.x+threadIdx.x;
 int interval=index/channels, channel=index%channels;
 if(interval>=intervals) return;
 int start=starts[interval]; float sum=0;
 for(int j=start;j<start+lengths[interval];++j)
   sum += depth[rd[j]] * feat[rf[j]*channels+channel];
 out[rb[start]*channels+channel]=sum;
}

// Unique rank_depth per valid frustum sample. Feature indices recur over depth.
// FP32 atomics implement the feature gradient; reduction-order tolerance is tested.
__global__ void pool_backward(const float* grad, const float* depth,
 const float* feat, const int* rd, const int* rf, const int* rb,
 float* gd, float* gf, int channels, int count) {
 int j=blockIdx.x*blockDim.x+threadIdx.x;
 if(j>=count) return;
 float sum=0;
 for(int c=0;c<channels;++c) {
   float g=grad[rb[j]*channels+c];
   sum += g*feat[rf[j]*channels+c];
   atomicAdd(gf+rf[j]*channels+c,g*depth[rd[j]]);
 }
 gd[rd[j]]=sum;
}

void check_float(const torch::Tensor& t) {
 TORCH_CHECK(t.is_cuda() && t.is_contiguous() && t.scalar_type()==torch::kFloat32,
             "Expected contiguous CUDA FP32");
}
void check_int(const torch::Tensor& t, const torch::Tensor& ref) {
 TORCH_CHECK(t.is_cuda() && t.is_contiguous() && t.scalar_type()==torch::kInt32
             && t.device()==ref.device(), "Expected same-device CUDA int32");
}

torch::Tensor forward(torch::Tensor depth, torch::Tensor feat,
 torch::Tensor rd, torch::Tensor rf, torch::Tensor rb,
 torch::Tensor starts, torch::Tensor lengths, int64_t cells) {
 check_float(depth);check_float(feat);
 TORCH_CHECK(depth.device()==feat.device(), "Device mismatch");
 for(auto t:{rd,rf,rb,starts,lengths}) check_int(t,depth);
 TORCH_CHECK(rd.numel()==rf.numel() && rd.numel()==rb.numel()
             && starts.numel()==lengths.numel(), "Index shape mismatch");
 c10::cuda::CUDAGuard guard(depth.device());
 auto out=torch::zeros({cells,feat.size(-1)},feat.options());
 if(starts.numel()) {
   pool_forward<<<(starts.numel()*feat.size(-1)+255)/256,256,0,
      at::cuda::getCurrentCUDAStream()>>>(depth.data_ptr<float>(),feat.data_ptr<float>(),
      rd.data_ptr<int>(),rf.data_ptr<int>(),rb.data_ptr<int>(),starts.data_ptr<int>(),
      lengths.data_ptr<int>(),out.data_ptr<float>(),feat.size(-1),starts.numel());
   C10_CUDA_KERNEL_LAUNCH_CHECK();
 }
 return out;
}

std::vector<torch::Tensor> backward(torch::Tensor grad, torch::Tensor depth,
 torch::Tensor feat, torch::Tensor rd, torch::Tensor rf, torch::Tensor rb) {
 check_float(grad);check_float(depth);check_float(feat);
 for(auto t:{rd,rf,rb}) check_int(t,depth);
 TORCH_CHECK(grad.device()==depth.device() && feat.device()==depth.device(), "Device mismatch");
 c10::cuda::CUDAGuard guard(depth.device());
 auto gd=torch::zeros_like(depth),gf=torch::zeros_like(feat);
 if(rd.numel()) {
   pool_backward<<<(rd.numel()+255)/256,256,0,at::cuda::getCurrentCUDAStream()>>>(
     grad.data_ptr<float>(),depth.data_ptr<float>(),feat.data_ptr<float>(),
     rd.data_ptr<int>(),rf.data_ptr<int>(),rb.data_ptr<int>(),gd.data_ptr<float>(),
     gf.data_ptr<float>(),feat.size(-1),rd.numel());
   C10_CUDA_KERNEL_LAUNCH_CHECK();
 }
 return {gd,gf};
}
PYBIND11_MODULE(TORCH_EXTENSION_NAME,m) {
 m.def("forward", &forward); m.def("backward", &backward);
}
