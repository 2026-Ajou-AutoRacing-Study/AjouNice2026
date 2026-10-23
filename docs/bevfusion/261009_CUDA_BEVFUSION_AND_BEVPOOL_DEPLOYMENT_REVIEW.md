# CUDA-BEVFusion과 BEVPool 배포 방식 검토

2026-10-09 KST 조사. 아래 초기 본문은 **당시의 코드·상태·추천**을 보존한다.
현재 geometry cache·분리 pooling·병렬화는 이미 적용돼 있으며, 다음 순서는
[2026-10-23 후속 검토](#e12-partial-tensorrt-priority-261023)를 따른다.
초기 조사에서 kernel·TensorRT·모델을 구현하거나 GPU 시험을 실행한 것은 아니다.
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
- Autoware tensorrt_bevdet은 6-camera camera-only temporal BEVDet의 ROS2배포 사례다.
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
최적화 우선순위를 정할 충분한 근거다. 두 구간 합 17.33ms는 모델 시간의 약 38%지만
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
   학습 step은 별도 4090/수집 종료 후 baseline/A/A+B/선택 후보를 나눠 측정한다.
5. 동일 checkpoint와 mission bag에서 class별 AP·근접 누락·원거리 첫 전달·ID 연속성·
   Tracking/Map-CV를 회귀 검증한다. 모델 평균뿐 아니라 CPU 준비/ROS p95와 drop을 본다.
6. 병목이 남으면 camera backbone/neck/depthnet의 부분 TensorRT를 검토한다.
   Torch/TRT tensor·stream 연결 및 전송 비용까지 측정한다. v3 아이디어는 같은 조건의
   추가 이득이 있을 때 반영하며, 전체 CUDA-BEVFusion 이주는 이 경로가 부족할 때의 후순위다.

pooling이 같은 수학 연산과 입출력 계약을 보존하면 기존 학습 가중치를 나중에 옮기는 것이
가능하며 재학습이 필수는 아니다. 다만 dtype/합산 순서에 따른 수치 차이는 회귀 검사한다.
crop·grid·depth·모델 구조 변경 및 quantization은 이 보장과 별개다.
세부 초기화/검증 계획은 [실행 계획](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md)을 따른다.

<a id="e12-partial-tensorrt-priority-261023"></a>

## 2026-10-23 후속: E12의 경량화·후처리·부분 TensorRT 우선순위

사용자는 모델 경량화·후처리 중 어디부터 볼지, TensorRT를 시도할 시점인지 검토하도록 요청했다.
현재는 **재학습 없는 실행 최적화부터 검토할 시점**이다. 구조 축소·해상도/ROI 변경·INT8을
동시에 도입하지 않는다. 이번에는 코드·기존 실측·공식 문서를 읽고 기록 출력의 CPU 후처리만
측정했으며 TensorRT 설치·export·engine build·production 교체는 실행하지 않았다.

### 지금 확인된 비용과 확인하지 않은 비용

- [학습된 E24 4090 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-learned-runtime-vast-261012)의
  localhost AMP 병렬화 RTT는 평균 72.22/p95 73.30ms였다. 전처리·모델·반환 비용은
  baseline에서 약 18.50/58.65/0.19ms였다. 같은 가중치의 당시 결과이지 새 E12·실제 LAN/ROS
  풀스택 계측은 아니다. 별도 CUDA-event stage는 중첩·호스트 idle을 포함하므로 단순 합산하지 않는다.
- 새 E12의 저장 bag 출력 **5,986프레임**에 현재 floor 0.1/top100·class threshold·회전 NMS를
  3회 적용했다. 현재 V/P/O 0.4/0.1/0.1에서 CPU 선택+원형 NMS는 평균 **0.0495ms**, p50
  **0.0442ms**, p95 **0.0937ms**, p99 **0.1194ms**였다. AP용 세 class 0.1에서도 평균
  0.0500ms였다. 반복 결과의 출력 index는 정확히 같았다.
- 위 CPU 측정은 GPU→CPU 전송·JPEG/PointCloud 준비·직렬화·ROS 메시지·tracker/planner를
  제외한다. 다른 native FP32 평가가 동시에 실행 중인 로컬 replay이며 대회 PC의 지연 인증이 아니다.
  그럼에도 **현재 표본의 NMS 자체를 먼저 GPU/C++로 바꿔 큰 이득을 기대할 근거는 약하다.**
- 이번 전체 native val의 느린 FP32 offline export 시간을 AMP 병렬화 ROS runtime 시간으로
  전용하지 않는다. 최신 E12의 실제 단계별 비용은 다음 별도 측정에서 확인해야 한다.

### 추천 순서 — 아직 구현 계획

1. E12·현재 Q90/512×1056/5cm·20cm/depth0.25m·nearest·AMP·병렬화·compile OFF를
   고정하고 decode/resize·입력 전송·camera·depth/pool·sparse·dense/head·출력/ROS를 측정한다.
   stage CUDA event와 실제 wall/입력 age·p95를 함께 보되 GPU 부하를 겹쳐 비용을 측정하지 않는다.
2. 출력이 같은 범위에서 불필요한 host/device 동기화·작은 전송·반복 준비·메모리 할당을 찾는다.
   GPU에서 score/topK 뒤 CPU로 전달하는 후보도 동점 순서·NMS 결과·원점을 보존할 때만 채택한다.
   latest-only 큐를 순서형 backlog로 바꾸거나 센서 header 지연을 GPU 속도로 해결했다고 쓰지 않는다.
3. **첫 TensorRT 후보는 고정 shape의 camera backbone+neck**, 이후 측정 결과에 따라 depthnet이나
   dense fusion/head를 추가한다. 현재 custom pooling·voxelization·spconv는 처음에는 유지한다.
   Torch-TensorRT 부분 변환 또는 ONNX+TensorRT 중 현재 Torch/CUDA와 맞는 경로를 사전검사한다.
   직접 CUDA tensor/stream 연결 비용까지 포함하고 CPU 왕복·과도한 graph partition이 생기면 기각한다.
4. 후보별 원형 tensor·최종 BBox/score/yaw·전체 AP·거리/class·0점/sparse·근거리/후방과
   연속 bag의 중복·ID/잔류를 비교한다. 과거 compile에서 작은 수치 차이가 query 선택과 추적 차이로
   이어졌으므로 평균 mAP가 같다는 이유만으로 통과시키지 않는다. 기존 eager OFF 복귀를 남긴다.
5. 실제 목표 4090 환경에서 engine 생성·입력 준비 포함 비용·동시 신호등/스택·24GB 메모리를
   검증한 뒤 배포한다. 기본 engine을 로컬 5060 Ti에서 만들면 4090에서 그대로 동작한다고 가정하지 않는다.
   compatibility mode는 별도 성능/호환 trade-off이므로 목표 GPU 빌드가 우선이다.

모델 채널/해상도 축소·pruning/distillation은 성능 손실 또는 재학습이 필요한 별도 후보이며,
초장거리 저신뢰도 문제가 남은 현재 첫 선택은 아니다. INT8/FP8도 대표 calibration/정확도
검증이 필요해 뒤로 둔다. TensorRT는 실행 최적화이고 LiDAR 0점의 관측 단서를 새로 만들지 않는다.
운영 threshold/NMS 의미 정책 개선은 latency 최적화와 별도 축으로 평가한다.

공식 근거: [NVIDIA import 경로](https://github.com/NVIDIA/TensorRT/blob/main/documents/import_workflows.md),
[Torch-TensorRT 미지원 연산·부분 fallback](https://docs.pytorch.org/TensorRT/user_guide/compilation/unsupported_ops.html),
[engine GPU/버전 호환](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/engine-compatibility.html),
[CUDA-BEVFusion](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/master/CUDA-BEVFusion).
현재 버전·호환성을 확인한 뒤 고정하며 기존 학습/실행 이미지를 최신 패키지로 즉시 업그레이드하지 않는다.

기록 후처리 산출물은 `artifacts/evaluation/bevfusion/e12_runtime_optimization_review_261023_v1/`의
`profile_recorded_postprocess.py`, `RECORDED_POSTPROCESS.json`이다. 원형 NMS source·예측 SHA
전후 불변과 반복 출력 동일성을 확인했다. 실제 TensorRT 속도 향상률은 아직 측정하지 않았다.

### 외부 AI의 TensorRT 주장과 실제 공개 근거 대조

사용자가 전달한 Gemini 답변은 검토 출발점이지 성능 근거로 채택하지 않았다.

| 주장 | 공식 자료에서 확인한 사실 | 현재 Base5에 대한 판단 |
|---|---|---|
| Orin에서 25 FPS 이상 | NVIDIA README의 ResNet50 FP16은 18 FPS, FP16+INT8은 25 FPS다. PyTorch 8.4 FPS는 SwinTiny·RTX 3090 조건이다 | 25 FPS는 실제 공개 수치지만 같은 모델·GPU의 8.4→25 가속 대조가 아니다. 우리 4090의 예상 FPS로 전용하지 않는다 |
| 양자화해도 정확도 하락이 거의 없다 | 해당 nuScenes ResNet50 mAP는 PyTorch 67.93, FP16 67.89, PTQ 67.66이다 | 해당 조건의 작은 하락은 확인된다. 우리 obstacle·0점/희소·원거리 score·yaw·추적의 보존은 따로 검증해야 한다 |
| pooling 수십 배 가속 | 2026 NVIDIA 글의 Blackwell V2 FP16 274µs→V3 FP16 17.3µs는 약 15.8배다. 이미 depth-outer가 들어간 V2+DO 37.8µs와 비교하면 약 2.18배다 | kernel·baseline·GPU에 따라 이득이 다르다. 우리 geometry cache·분리 pooling의 기존 이득을 다시 더하거나 전체 모델 가속률로 쓰지 않는다 |
| sparse convolution도 가속 가능 | NVIDIA에는 별도 `libspconv` 추론 엔진이 있고, Autoware에는 sparse backend·custom plugin 연동 경로가 있다 | 가능성이 있는 실제 후보이며 단순 ONNX export로 자동 변환되는 것은 아니다. 현재 spconv와 같은 입출력·가중치·좌표/stride를 검증해야 한다 |
| 실시간 배포에는 TensorRT가 필수 | 이런 보편적 필수 조건은 공개 결과에서 도출되지 않는다 | 현재 PyTorch AMP 병렬화도 과거 4090에서 약 72ms였다. 필요한 것은 목표 주기·p95·age·동시 스택 검증이며 제품명 자체가 아니다 |

근거는 [CUDA-BEVFusion README](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/master/CUDA-BEVFusion/README.md),
[NVIDIA pooling 글](https://developer.nvidia.com/blog/accelerating-bev-pooling-on-nvidia-gpus-for-physical-ai-applications/),
[Autoware 모델 설명](https://huggingface.co/AutowareFoundation/bevfusion/blob/main/README.md)이다.
pooling 글의 Blackwell·A6000 측정은 RTX 4090 실측이 아니며 FP8 경로도 별도 후보로 둔다.

### 공개 구현을 그대로 이식할 수 없는 이유와 재사용 후보

로컬 NVIDIA 참조 revision `7c1623f234d5a9794fd862c9456f68f87ae7b900`의 camera/head export,
`bevfusion.cpp`, `lidar-scn.cpp`, `camera-bevpool.cu`와 sparse library 설명을 대조했다.

- export는 구 MIT `encoders.camera` API와 nuScenes head 계약을 사용한다. 현재 MMEngine
  모듈·3 class·7D box·4 camera·512×1056·depth396bin을 그대로 처리하는 export가 아니다.
  공개 예제의 6 camera·256×704·118bin·velocity 포함 head를 우리 모델로 바꾸어야 한다.
- camera depth 호출에는 point feature 수 5가 고정되어 있다. 우리 XYZI 4열 입력을 5열로
  오독하지 않도록 바꾸어야 한다. voxel 좌표 순서·sparse weight layout·BN folding도 대조 대상이다.
- camera normalization의 보간·카메라별 입력 크기와 crop 행렬도 우리 CPU cubic 전처리와
  동등하다고 가정하지 않는다. 공개 기본 입력 크기나 grid에 맞추려고 재학습 조건을 바꾸지 않는다.
- 참조 forward는 LiDAR와 camera를 같은 전달 stream에서 순서대로 호출한다. 현재의
  두 branch 병렬화를 유지하지 못하면 개별 kernel이 빨라도 전체 이득이 줄 수 있다.
- 참조 timer의 `Total`은 CopyLidar와 ImageNorm을 합산하지 않는다. 입력 준비·반환을 포함한
  우리 RTT나 실제 센서 header→ROS 수신 age와 직접 비교하지 않는다. 기존 localhost RTT도
  실제 LAN/ROS 수신·tracker/planner를 포함한 측정은 아니다.
- 재사용 후보는 feature/depth 분리 pooling·packed/vectorized load·FP32 누적,
  GPU voxelization·재사용 buffer·sparse 추론 backend·부분 engine export다. 이미 구현한
  cache/분리 pooling을 신규 성과로 세지 않고 동일 입력의 추가 이득만 측정한다.

우리 `morai_pool_optimization.py`와 `separated_bev_pool.cu`에도 구체적 대조 지점이 남아 있다.
현재 forward는 depth/feature를 FP32로 변환하고 scalar channel별 interval 합산을 수행한다.
따라서 FP16 load+FP32 누적·channel vectorization은 새 inference 전용 후보가 될 수 있다.
하지만 현 빌드는 `--fmad=false`이며 원형의 곱·합 순서를 지키려는 계약이 있다. half2 누적이나
FMA 활성화를 출력 동일 개선이라고 부르지 않는다. 또한 calibration signature는 cache hit에서도
작은 GPU→CPU 값 복사를 수행하므로, 값 변경·카메라별 crop·stream 무효화 보호를 유지하면서
동기화 비용을 줄일 여지가 있는지 계측한다. 현재 코드가 틀렸다고 판정하거나 즉시 생략하지 않는다.

[Autoware 구현](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_bevfusion)은
별도 image-backbone engine·사전 geometry·GPU buffer·sparse/custom plugin 경로의 참고가 된다.
하지만 ROS2·6 camera·17cm voxel·500 proposals·다른 class/센서 계약이므로 노드/가중치를
통째로 교체하지 않는다. 모델 카드의 plugin 명칭만 믿지 않고 CMake의 cumm/spconv 연결과
실제 engine plugin 경로를 함께 확인한다. 최신
[spconv_cpp](https://github.com/autowarefoundation/spconv_cpp)는 CUDA 13 기준이므로 기존
Torch 2.7.1/CUDA 12.8 환경에 그대로 덮어쓰지 않는다.

따라서 위 추천 순서의 **camera FP16 부분 TRT는 첫 구현 후보**로 유지하되 sparse/CUDA를
검토 대상에서 제외하지 않는다. E12 단계 측정에서 sparse 또는 view transform이 임계 경로에
남으면 같은 저장 activation으로 해당 backend/kernel만 A/B한다. custom backend가 필요한
전체 이주는 첫 부분 후보의 이득·이식 비용·정확도 검증 후 판단한다. 아직 새 engine의 성능,
전체 이식 성공, 기존 checkpoint의 자동 호환을 확인한 상태는 아니다.

<a id="e12-partial-trt-implementation-261023"></a>

## 10/23 후속: E12 camera 부분 TRT 시험 구현

앞 절의 ‘아직 구현 계획’은 작성 당시 상태다. 이후 실제 4090에서 camera backbone+neck
FP16 engine·현재 stream의 zero-copy adapter·원형 복귀·비용/출력 비교·전체 native val
export/scorer를 구현했다. [실측과 한계는 60절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-partial-trt-cuda-experiment-261023)을 따른다.
운영 서버·launch를 바꾸거나 E12를 자동 production 승격한 것은 아니다. 작은 raw query 차이의
엄격 gate와 실제 정확도 손실은 구분하며, 측정한 AP/검출/추적 손실을 지연 이득과 함께 판단한다.

### 환경과 실행 방법

실측은 기존 고정 학습 환경 위 별도 venv의 `--system-site-packages`와
`docker/bevfusion/requirements/tensorrt-inference-experiment.txt`를 사용했다.
`--no-deps`로 ONNX/TRT만 추가하고 Torch·CUDA·기존 model dependency를 업그레이드하지 않는다.
`Dockerfile.tensorrt-experiment`는 같은 의존성을 담는 **미빌드 recipe**다. 필요할 때 저장소
루트에서 다음과 같이 build하며 실제 build/pull·팀장님 PC 시험은 별도다.

```bash
docker build -f docker/bevfusion/Dockerfile.tensorrt-experiment \
  -t ajounice-bevfusion-trt-experiment:10.13.3 .
```

아래는 필요한 Python dependency·custom op·repo import 경로가 준비된 목표 GPU 환경의 예시다.
`E12_CHECKPOINT`, `INPUT_MANIFEST`, `RUN_ROOT`는 실제 파일과 **새 출력 디렉터리**로 지정한다.
manifest는 `base5_camera_trt_inputs_v1` 형식이며 원본 JPEG/XYZI 파일·SHA·강도 출처를 명시한다.
경로만 다른 학습 가중치나 임의 이미지를 첫 자격 시험에 대신 넣지 않는다.

```bash
CONFIG=perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_nearest_depth_runtime.py
SCRIPTS=perception/detection3d/bevfusion/scripts
python "$SCRIPTS/export_base5_camera_tensorrt.py" \
  --config "$CONFIG" --checkpoint "$E12_CHECKPOINT" \
  --output-dir "$RUN_ROOT/engine" --precision fp16 --output-dtype float16 \
  --fp32-layer-types NORMALIZATION SOFTMAX --build-engine
python "$SCRIPTS/qualify_base5_camera_tensorrt.py" \
  --inputs-manifest "$INPUT_MANIFEST" --config "$CONFIG" \
  --checkpoint "$E12_CHECKPOINT" --sensor-config configs/sensors/Sensor.json \
  --engine "$RUN_ROOT/engine/camera.engine" \
  --metadata "$RUN_ROOT/engine/camera.engine.json" \
  --output "$RUN_ROOT/qualify" --samples 64 --timing-repeats 3
```

engine은 build한 GPU·compute capability·TRT 버전·고정 shape와 camera parameter에 묶인다.
4090 48GB에서 통과한 비용 시험을 24GB 카드·ROS/TCP·다른 GPU의 검증으로 대체하지 않는다.
adapter는 `install_camera_tensorrt(model, ...)`의 반환 handle을 `close()`하면 원래
backbone/neck으로 복귀한다. 학습 모델에 설치하지 않으며 암묵적 CPU fallback도 하지 않는다.

전체 val은 `infer_base5_execution_candidate.py --backend eager`와
`--backend camera_trt`를 같은 infos·가중치·AMP/TF32에서 각각 새 폴더로 실행한다.
`score_base5_execution_pair.py --reference EAGER_DIR --candidate TRT_DIR --info INFOS.pkl
--output FRESH_RESULT.json`으로 class/10m/전후방 원거리 AP를 계산한다. 센서·source·GT
정합을 확인한 COMPLETE/receipt/STARTED/predictions가 모두 필요하며 GT 0점·빈 프레임도 남긴다.
과거 FP32 평가와 새 AMP 평가를 비교해 TRT 차이라고 부르지 않는다.

CUDA pooling은 `benchmark_base5_pool_variants.py`로 실제 저장 activation을 재사용한다.
이번 새 pack4 kernel은 exact지만 기존 kernel보다 빠르지 않았으므로 기본 채택하지 않는다.
반복 layout 복사를 한 번으로 줄이는 후보와 새 kernel의 효과도 분리한다. backward·재학습이나
새 sparse backend 이식은 이번 시험의 완료 범위가 아니다.

<a id="e12-orin-scope-safe-startup-261023"></a>

## 10/23 추가 검토: Orin 25 FPS의 비교 범위와 실대회 시작 안전성

이 절은 앞선 구현·실험 기록 이후의 설명이다. 과거 수치나 당시 미구현 상태를 수정하지 않는다.
현재 production 선택은 [CURRENT](CURRENT.md#runtime)를 따르며 **E24를 유지한다.**
E12의 TensorRT 시험 결과가 좋아도 E12를 자동 선택하거나 E24용 engine으로 간주하지 않는다.
연속 bag·후속 최적화·시작 시험의 실제 결과는 [근거 문서](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md)에 별도로 남긴다.

### Orin의 25 FPS를 우리 서비스 시간에 그대로 대입할 수 없는 이유

NVIDIA 공식 README의 25 FPS는 **ResNet50·FP16+INT8** 조건이며 FP16-only는 18 FPS다.
Swin-Tiny PyTorch의 8.4 FPS는 RTX 3090 측정이므로 같은 모델·같은 GPU의 전후 대조가 아니다.
공개 예제와 현재 E12의 크기를 비교하면 다음과 같다.

| 항목 | NVIDIA CUDA-BEVFusion 공개 예제 | 현재 E12 입력·격자 |
|---|---|---|
| Camera backbone | ResNet50의 Orin 결과 | Swin-Tiny |
| 카메라 입력 H×W | 6×256×704 | 4×512×1056, 총 입력 픽셀 2배 |
| Depth 구간·간격 | 1–60m / 0.5m, 118 bin | 1–100m / 0.25m, 396 bin |
| 투영 후보 수 N×D×Hf×Wf | 1,993,728 | 13,381,632, 약 6.71배 |
| Camera BEV 격자 X×Y | 360×360, 30cm | 1008×336, 20cm, cell 수 약 2.61배 |
| 검출 head의 격자 X×Y | 180×180 | 504×168, cell 수 약 2.61배 |
| LiDAR XY voxel | 7.5cm | 5cm |
| 배포 경로 | Camera/depth·BEV downsample·fusion/decoder/head engine과 별도 sparse 추론 엔진 | 먼저 camera backbone+neck만 부분 TensorRT로 교체 |

배수는 배열·후보 수의 비교이며 전체 latency 예측 배수가 아니다. 현재 경로는 이미 geometry
cache와 분리 pooling으로 큰 중간 tensor 생성을 줄였다. Sparse convolution 비용도 전체 grid
크기뿐 아니라 실제 활성 voxel·포인트 분포·kernel·호스트 제출에 좌우된다. 전방 범위가 길다는
이유만으로 전체 BEV 면적도 같은 비율로 커졌다고 해석하지 않는다. 직사각형 ROI와 내부 padding을 구분한다.

NVIDIA의 공개 `Total` timer는 모델 단계의 합이며 CopyLidar와 ImageNorm을 합산하지 않는다.
JPEG decode는 예제의 forward 바깥에서 수행하고 ROS·통신도 포함하지 않는다. 우리
`service_ms`는 CPU 준비·모델·결과 변환을 포함한다. 따라서 같은 파일에 기록된 `model_ms`와
`service_ms`를 먼저 구분해야 하며, 모델 구간의 역수를 ROS 처리 Hz나 센서 age로 부르지 않는다.
공개 결과와 우리 모델은 조건이 달라 **‘Orin도 25 FPS인데 4090은 더 느리니 최적화가 끝났다’나
‘같은 코드를 가져오면 곧바로 25 FPS 이상이다’라는 결론은 모두 성립하지 않는다.**

공식 근거는 고정 revision의 [성능 표](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/README.md),
[입력·voxel·depth 설정](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/src/main.cpp#L132),
[실제 timer](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/src/bevfusion/bevfusion.cpp#L121),
[camera/depth exporter](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/qat/export-camera.py#L47),
[fusion/head exporter](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/qat/export-transfuser.py#L207)다.

### 지금 Swin을 다른 backbone으로 바꾸지 않는 이유

MIT의 공개 detection checkpoint·학습 예시는 Swin과 그 사전학습 가중치를 사용한다.
현재 E12도 해당 계열의 backbone·neck feature 채널과 호환되는 가중치를 재사용했다.
ResNet50으로 바꾸는 것은 TensorRT export 옵션 변경이 아니라 backbone과 neck 입력 채널·특징
분포를 바꾸는 작업이다. 기존 E12의 카메라 가중치를 그대로 적용할 수 없으며 재학습·재정합 평가가 필요하다.
[MIT 학습·사전학습 경로](https://github.com/mit-han-lab/bevfusion/blob/main/README.md#training)를 따른다.

부분 TRT 뒤 카메라 특징 추출은 이미 약 5ms대다. 카메라 구간을 완전히 없애는 비현실적 가정조차
그 구간 이상의 절감을 보장하지 못하고, 병렬 실행에서는 임계 경로 때문에 실제 이득이 더 작을 수 있다.
현재는 backbone 교체보다 CPU 입력 준비와 검출 head의 불필요한 정렬·attention weight 계산을
먼저 조사한다. `topk`와 SDPA 후보는 재학습 없이 비교할 수 있지만 동점 후보·수치 차이가 생길 수 있으므로
전체 검출·거리별 AP·연속 tracker와 실제 지연을 함께 판단한다. 작은 수치 차이 자체를 실패로 취급하지도,
전체 mAP가 비슷하다는 이유만으로 근거리 누락·ID 변화까지 통과시켰다고 쓰지도 않는다.
공개 head의 [topk 선택](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/qat/export-transfuser.py#L99)과
[PyTorch의 need_weights=False·SDPA 안내](https://docs.pytorch.org/docs/2.7/generated/torch.nn.MultiheadAttention.html)를 참고했다.

### 선택 가속기 때문에 원형 실행까지 거부하지 않는 시작 정책

선택 가속기 설치 조건과 필수 모델·센서 계약을 분리했다. 실제 구현은
[startup policy](../../docker/bevfusion/app/acceleration_startup.py)와
[서버](../../docker/bevfusion/app/inference_server.py)를 따른다.

| 시작 정책 | 동작 | 허용하지 않는 우회 |
|---|---|---|
| `off` | 명시한 동일 checkpoint/config의 기존 eager·AMP 경로를 사용한다 | 다른 가중치·다른 detector·GT로 교체하지 않는다 |
| `prefer` | 선택 engine을 요청했으면 먼저 준비한다. 준비 단계에서 분류된 선택 가속기 가용성 오류만 같은 모델의 eager 경로로 한 번 복귀한다 | checkpoint·센서·클래스·입력 오류나 일반 모델 load 실패를 가속기 오류로 숨기지 않는다 |
| `require` | 선택 engine을 반드시 사용해야 하는 명시적인 자격 시험이다. 준비에 실패하면 시작을 거부한다 | 원형 복귀를 성공적인 TRT 실행으로 보고하지 않는다 |

engine/metadata를 지정하지 않은 기본은 `off`이며, 지정하고 정책을 생략하면 `prefer`다.
복귀 여부·이유 코드·실제 backend를 logging/status에 구분한다. 복귀 시에도 사용자가 선택한
checkpoint/config·AMP·병렬화·threshold는 유지한다. E12 engine과 E24 가중치가 맞지 않는다고
E12를 자동 선택하는 구조가 아니다. 직접 model/adapter API의 엄격한 검사도 이 정책 때문에 완화하지 않는다.

`run_bevfusion.sh --safe`는 선택한 E24 또는 명시적인 E25를 그대로 유지하면서 기존 팀 CUDA
12.8 이미지와 eager 경로를 선택하고, 선택 TRT·시험용 pool layout·head SDPA/좌표 cache를 끈다. 정확한 production
시작·설치 명령은 [E24 handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)와
[E25 복귀 handoff](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md)를 따른다.
`--safe`는 checkpoint SHA 불일치, 필수 sensor/config 누락, bridge의 camera order·horizontal FOV·
강도 계약 불일치, 이미 canonical topic을 소유한 다른 detector를 무시하는 옵션이 아니다.

이 복귀는 **시작 시점에만** 적용한다. engine enqueue 이후 실행 오류, illegal memory access,
device-side assert, context 손상·GPU 소실 같은 CUDA 오류는 같은 프로세스에서 eager 재시도로
덮지 않는다. 해당 실행의 readiness를 유지한 채 결과를 계속 내보내는 방식도 허용하지 않는다.
새 프로세스로 원형 경로를 다시 시작하고 GPU/driver 상태를 확인해야 한다. 하드웨어·driver 자체가
정상 CUDA 실행을 못 하는 상황은 `--safe`도 해결하지 못한다. 이 처리는 제어기의 물리 비상정지 보장이 아니다.

### 오프라인 시작 점검과 이미지 배포 범위

[competition preflight](../../scripts/runtime/preflight_bevfusion_competition.py)의 기본 동작은 읽기 전용이다.
선택 production profile의 가중치 SHA·config 상속·sensor/bridge 계약·ROS 설치본·로컬 Docker
이미지·GPU/driver를 확인하며, W&B·Drive·registry 로그인이나 인터넷 연결을 요구하지 않는다.
호스트 Python과 컨테이너 Python 버전이 다르거나 GPU가 4090이 아니라는 이유만으로 정상적인
원형 실행을 막지 않는다. 정적 통과, 모델 warmup, 실제 ROS/LAN 입력, 24GB 풀스택 검증은 구분한다.

`--offline-smoke`만 명시적으로 요청하면 도구가 소유한 임시 `--rm --network none` 컨테이너에서
작은 CUDA 연산과 선택 모델 warmup·종료를 확인한다. source·가중치·sensor는 read-only이며 서버·
ROS publisher·주행 제어를 시작하지 않는다. timeout 정리도 고유 owner label이 일치하는 자체
컨테이너만 대상으로 한다. 실제 실행 결과와 제한은 근거 문서에 남기며 이 문서의 구현 설명을
실대회 시작·전체 스택 성능 인증으로 사용하지 않는다.

`Dockerfile.tensorrt-experiment`는 여전히 **미빌드·미푸시 recipe**다. 실제 부분 TRT 검증에 사용한
원격 venv와 구분한다. Docker Hub에 새 팀 배포 이미지가 올라갔거나 팀원이 `git pull`만으로
TensorRT engine을 자동 확보했다고 간주하지 않는다. 필요한 경우 image build·목표 GPU engine
생성·동일 가중치 검증을 별도로 수행한다. 선택 TRT가 준비되지 않은 팀 PC에도 기존 이미지와
E24 가중치가 있으면 원형 경로를 사용할 수 있도록 유지한다.

같은 시작 감사에서 기존 production ZIP 설치기가 **가중치 archive 검증과 현재 checkout의
코드 바이트 정합을 결합**한 문제도 확인했다. 10/15에 동결한 ZIP의 `reference/`는 당시 코드
감사본인데, 새 app 파일을 추가한 현재 checkout까지 그 목록·SHA와 같아야 한다고 검사하면
가중치가 정상이어도 install/check-only가 거부될 수 있었다. 현재 설치기는 동결 archive·
checkpoint SHA와 경로 안전성 검사를 유지하면서, 가중치 설치와 과거 코드 snapshot 감사를
분리하도록 수정했다. `reference/`를 현재 checkout 위에 자동 덮어쓰거나 현재 코드를 옛 ZIP에
맞추어 되돌리는 방식으로 해결하지 않는다. **코드가 바뀌었다는 이유만으로 기존 E24 가중치·ZIP을
다시 Drive에 업로드할 필요는 없다.** 설치기·실제 GPU 기동·회귀 결과는 [61절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#continuous-trt-competition-startup-261023)을 따르며,
최신 코드로 시작하기 전에는 위 preflight로 현재 설정·설치본을 검사한다.

### Head 후속: 전체 AP 확인 뒤 선택 runtime으로 연결

[62절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#head-native-ap-runtime-261023)의 새 paired export는
camera TRT를 고정하고 SDPA+BEV 위치 cache만 바꿨다. E12 native val 전체 4,614장 mAP는
94.066058→94.065589%, 장애물 AP 변화는 −0.001278%p다. 장애물 20~30/30~40m도 각각
−0.00198/−0.00433%p였다. 작은 수치 차이보다 앞선 연속 입력의 서비스 약 5.63ms 절감이
실용적이라고 판단했다. 원형↔후보의 byte 동일성을 배포 조건으로 삼지 않았다.

실제 model/server·Compose·shell·health에 두 opt-in을 연결했다. `topk`는 계속 OFF다.
소스/AST 미지원은 첫 실행 전 같은 head로 복귀하며, GPU 실행 이후 오류는 hot fallback하지 않는다.
`--safe`는 두 옵션을 모두 끈다. E24 전체 AP 또는 팀 4090 24GB 풀스택을 인증한 것은 아니므로
production 기본 OFF와 기존 E24를 유지한다. 재학습은 필요 없으며 이 변경은 새 TRT head engine이나
새 backbone의 도입이 아니다. 실행 방법은 [E24 handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md#competition-startup)를 따른다.
