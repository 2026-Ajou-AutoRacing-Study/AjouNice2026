# CUDA-BEVFusion과 BEVPool 배포 방식 검토

2026-10-09 KST 조사. 코드 확인과 기존 실측을 바탕으로 한 CURRENT 추천이며,
이번에 kernel·TensorRT·모델을 구현하거나 GPU 시험을 실행한 것은 아니다.
Sunny13 수집, raw/GT, production E25 및 tracker 계약은 유지한다.

로컬 참조 코드는 `third_party/lidar_ai_solution/CUDA-BEVFusion`에 있다.
2026-10-09에 아래 조사 revision을 clone했으며 관련 sparse library도 펼쳤다.
clone은 Git ignore 상태이고 학습·runtime dependency로 연결하지 않는다.
[위치와 clone 명령](../../third_party/README.md#cuda-bevfusion-로컬-참조).

## 결론

학습은 현재 PyTorch/MMDetection3D 체계를 유지하고 **고정 기하 cache와 depth/feature
분리 pooling을 먼저 이식**한다. CUDA-BEVFusion은 유용한 배포 참조지만 그대로
교체할 수 있는 우리용 학습 baseline은 아니다. v3라는 이름보다 재현 가능한 코드,
학습 backward, 같은 입력·정밀도의 4090 실측과 출력 정합으로 선택한다.
최적화 완료를 Base5 본학습 시작의 필수조건으로 만들지 않는다.

## 외부 주장과 확인 범위

- “최신 MMDetection3D에 기본적으로 v2가 있다”는 일반화는 부정확하다.
  조사한 공식 main의 BEVFusion은 ranks 정렬과 기존 QuickCumsumCuda를 사용한다.
  확인 revision은 fe25f7a51d36e3702f961e198894580d83c4387b이며,
  우리 fork revision 19422fb451504c6c8ff8dbab0fd48f3e4d73a113과도 구분한다.
  [공식 pooling 코드](https://github.com/open-mmlab/mmdetection3d/blob/fe25f7a51d36e3702f961e198894580d83c4387b/projects/BEVFusion/bevfusion/ops/bev_pool/bev_pool.py).
- BEVNeXt가 v2를 사용하는 것은 특정 BEVDet 계열의 사례다. 모든 fusion 모델의
  기본 연산이나 baseline 교체 필요성을 증명하지 않는다.
  [CVPR 2024 논문](https://openaccess.thecvf.com/content/CVPR2024/papers/Li_BEVNeXt_Reviving_Dense_BEV_Frameworks_for_3D_Object_Detection_CVPR_2024_paper.pdf).
- Autoware tensorrt_bevdet은 6-camera camera-only temporal BEVDet의 ROS2 배포 사례다.
  우리 4-camera+LiDAR single-frame ROS1 detector의 대체 모델은 아니다.
  [공식 모델 카드](https://huggingface.co/AutowareFoundation/tensorrt_bevdet).

NVIDIA의 2026-06-24 글은 v2, CUDA-BEVFusion의 depth-outer 구현(V2+DO), v3를
구분한다. v3는 명시적 인덱스·중복 load 감소·GPU별 메모리 최적화를 제시한다.
RTX PRO 6000 Blackwell Max-Q의 같은 FP16 pooling 측정은 V2+DO 37.8µs,
v3 17.3µs다. 전체 모델이나 4090 측정이 아니다. 이번 조사에서는 본문과 연결된
공식 저장소에서 해당 v3 benchmark kernel/plugin 및 backward의 직접 배포 경로를
확인하지 못했다. 공개되지 않았다고 단정하지 않으며, 직접 구현하면 공식 v3 이식이
아닌 v3 아이디어 적용으로 기록한다.
[NVIDIA 설명과 benchmark](https://developer.nvidia.com/blog/accelerating-bev-pooling-on-nvidia-gpus-for-physical-ai-applications/).

## 우리 구현과 CUDA-BEVFusion 비교

CUDA-BEVFusion 참조 revision은 NVIDIA Lidar_AI_Solution의
7c1623f234d5a9794fd862c9456f68f87ae7b900이다. 배포 README는 CUDA·TensorRT 추론을
목적으로 하며 Swin-Tiny와 ResNet50 예제를 모두 제공한다. ResNet으로 바꿔야만
사용 가능한 체계라고 해석하지 않는다. ORIN/3090의 서로 다른 모델 수치는 우리
4090의 처리시간·정확도·VRAM 보장으로 사용하지 않는다.
[공식 README](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/README.md).

| 단계 | 현재 우리 경로 | CUDA-BEVFusion 참조 경로 | 판단 |
|---|---|---|---|
| JPEG decode | CPU OpenCV, 카메라별 병렬 준비 | 예제의 CPU 이미지 로딩 | 이름에 CUDA가 있어도 decode 전체가 GPU인 것은 아님 |
| resize/crop/normalize | CPU cubic resize/crop, uint8 전송 후 GPU 정규화 | CUDA resize/crop/normalize/layout 변환, nearest/bilinear | 픽셀·원본 전송량·학습 정합 확인 후 후보 |
| Camera 신경망 | PyTorch CUDA/cuDNN, AMP | TensorRT engine | CUDA 미사용→사용의 차이가 아니라 실행 최적화 차이 |
| LiDAR 경로 | CUDA voxelization + spconv CUDA sparse encoder | CUDA voxelization + 별도 sparse inference library | 현재 spconv checkpoint/export 호환 확인 필요 |
| 기하·인덱스 | GPU에서 forward마다 geometry/ranks/정렬 | update에서 계산 후 재사용 | 현재 병목과 직접 연결되는 우선 이식 후보 |
| pooling | CUDA FP32 interval reduction, depth×feature tensor 생성 | 분리 depth/feature, FP16 입출력·FP32 누적 | 큰 중간 tensor 생략이 핵심 |
| fusion/head | PyTorch CUDA, 우리 class/직사각형 head | TensorRT + CUDA decode | export와 7D/class/output 계약 수정 필요 |
| 학습 | autograd/backward 지원 | 조사한 배포 pooling은 inference forward | TensorRT로 본학습이 빨라지는 것은 아님 |

우리도 이미 CUDA를 핵심 계산에 사용한다. cuDNN autotune, TF32 허용, AMP,
warmup, CPU 카메라 병렬 처리와 latest-only 입력 처리가 있다. 아직 없는 것은
기하 cache, 분리 pooling, TensorRT engine 및 GPU resize/crop이다.
view transform/pooling은 FP32 경로이므로 모델 AMP가 모든 연산의 FP16을 뜻하지 않는다.

로컬 근거: [Docker](../../docker/bevfusion/Dockerfile),
[runtime](../../docker/bevfusion/app/bevfusion_model.py),
[전처리](../../docker/bevfusion/app/online_preprocessor.py),
[DepthLSS](../../third_party/mmdetection3d/projects/BEVFusion/bevfusion/depth_lss.py),
[pooling CUDA](../../third_party/mmdetection3d/projects/BEVFusion/bevfusion/ops/bev_pool/src/bev_pool_cuda.cu).
외부 근거: [core](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/src/bevfusion/bevfusion.cpp),
[pool](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/src/bevfusion/camera-bevpool.cu),
[normalization](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/src/bevfusion/camera-normalization.cu),
[sparse library](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/libraries/3DSparseConvolution/README.md).

## 전체 교체보다 부분 이식을 추천하는 이유

| 확인한 차이 | 우리에게 필요한 작업 |
|---|---|
| 원본 export는 과거 BEVFusion model object/API에 의존 | 현재 MMEngine state_dict/module 구조에 맞게 exporter 작성 |
| 예제 nuScenes 클래스·정사각형 dummy shape | 4-class/7D·직사각형 BEV·선택한 crop/depth/grid로 export 및 engine 재생성 |
| core depth 호출에 LiDAR feature 수 5가 직접 지정됨 | 우리 XYZI 4채널 경로를 끝까지 수정·검증; 설정만 변경하면 충분하지 않음 |
| normalization은 카메라 공통 원본 크기·resize 설정을 사용 | 서로 다른 앞뒤/측방 native 크기와 카메라별 img_aug_matrix 지원 |
| geometry kernel의 img_aug 역변환은 공통 matrix 주소 사용 | 카메라별 matrix indexing 및 우리 augmentation 정합 확인 |
| head decode는 vel binding을 요구하고 z를 bottom center로 변환 | velocity 없는 head, 기존 BBox 중심·ROS shape·class 계약 보존 |
| pack10 pooling은 C/10 묶음에 의존 | 현재 C=80은 맞지만 임의 channel 수에는 tail 처리 필요 |
| core forward는 기본적으로 같은 stream에 단계 순차 실행 | 가져오기만 하면 camera/LiDAR가 자동 병렬화된다는 가정 금지 |

근거: [camera export](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/qat/export-camera.py),
[fusion/head export](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/qat/export-transfuser.py),
[geometry](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/src/bevfusion/camera-geometry.cu),
[head decode](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/src/bevfusion/head-transbbox.cu).

CUDA-BEVFusion timer의 Total에는 CopyLidar/ImageNorm이 빠져 있으며 CPU decode와
ROS 전달도 들어가지 않는다. 우리 CPU 준비 포함 latency와 직접 비교하지 않는다.
MIT 표시가 있는 kernel을 차용하면 원 저작권·라이선스를 보존한다. 현재 Docker의
CUDA 12.8/PyTorch/spconv와 별도 sparse library·TensorRT의 ABI 및 GPU arch도 확인한다.
ground removal, INT8/FP8, interpolation 변경을 성능 최적화에 묶어 무검증 도입하지 않는다.

## 우리가 노릴 수 있는 이득과 한계

[기존 4090 실측](261009_RTX4090_CAPACITY_RESULTS.md)의 320×896/±100m 비용 대조는
모델 45.86ms, CPU 준비 포함 50.93ms다. geometry 6.81ms와 pooling 10.52ms는
최적화 우선순위를 정할 충분한 근거다. 두 구간 합 17.33ms는 모델 시간의 약38%지만
CPU 대기를 포함할 수 있는 단계 span이며 순수 kernel 시간이 아니다. 17.33ms를
그대로 없애거나 전체 모델을 pooling speedup 배수만큼 가속한다고 예상하지 않는다.
이 측정은 3-class/depth0.5m 비용 대조이며 최종 Base5의 정확도·처리시간 인증도 아니다.

현재 lift 중간 tensor의 논리적 크기는 B×N×D×Hf×Wf×C×dtype bytes다.
B=1/N=4/C=80/feature stride8/FP32일 때 다음과 같다.

| 이미지 | depth99bin | depth198bin |
|---|---:|---:|
| 320×896 | 541MiB | 1,083MiB |
| 448×896 | 758MiB | 1,516MiB |
| 512×1056 | 1,021MiB | 2,042MiB |

단일 tensor의 계산값이며 peak VRAM 절감 실측은 아니다. 분리 pooling도 output,
index, allocator, 다른 activation과 학습 gradient를 사용한다. 그래도 고해상도에서
중간 tensor 생성·메모리 traffic을 줄이는 방향은 검토 가치가 크다.

## 실행 순서와 합격 기준

1. A: 기존 kernel을 유지한 inference geometry/valid index/rank/interval cache.
   feature shape·camera 순서·calibration·augmentation·grid·device/dtype 변경 시 무효화한다.
2. B: 공개 BEVPoolv2를 backward 포함 기준 구현으로 삼고 분리 pooling을 검증한다.
   CUDA-BEVFusion depth-outer/FP16 load·FP32 누적은 별도 inference 후보로 비교한다.
   기존 kernel도 interval-owned 방식이므로 atomic 제거를 새 이득으로 계산하지 않는다.
3. 같은 입력·dtype에서 CPU 기준/기존 출력, depth와 feature gradient, 빈 입력,
   마지막 interval·경계·직사각형 XY·카메라별 crop·batch를 검사한다.
   non-default CUDA stream과 launch stream도 확인한 뒤 병렬화한다.
4. 수집 중에는 합의한 작은 bounded CUDA 연산 시험만 가능하다. 공식 속도·VRAM·짧은
   학습 step은 별도4090/수집 종료 후 baseline/A/A+B/선택 후보를 나눠 측정한다.
5. 동일 checkpoint와 mission bag에서 class별 AP·근접 누락·원거리 첫 전달·ID 연속성·
   Tracking/Map-CV를 회귀 검증한다. 모델 평균뿐 아니라 CPU 준비/ROS p95와 drop을 본다.
6. 병목이 남으면 camera backbone/neck/depthnet의 부분 TensorRT를 검토한다.
   Torch/TRT tensor·stream 연결 및 전송 비용까지 측정한다. v3 아이디어는 같은 조건의
   추가 이득이 있을 때 반영하며, 전체 CUDA-BEVFusion 이주는 이 경로가 부족할 때의 후순위다.

pooling이 같은 수학 연산과 입출력 계약을 보존하면 기존 학습 가중치를 나중에 옮기는 것이
가능하며 재학습이 필수는 아니다. 다만 dtype/합산 순서에 따른 수치 차이는 회귀 검사한다.
crop·grid·depth·모델 구조 변경 및 quantization은 이 보장과 별개다.
세부 초기화/검증 계획은 [실행 계획](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md)을 따른다.
