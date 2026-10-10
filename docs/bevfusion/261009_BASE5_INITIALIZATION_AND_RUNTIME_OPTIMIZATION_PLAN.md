# Base5 학습 초기화와 BEV 연산 최적화 계획

2026-10-09 KST 계획과 당시 미구현 상태를 보존한다. **현재 실행은 아래 10-10 갱신이 우선**한다.
Sunny13 수집, 원본 annotation, collector runtime과 production E25는 변경하지 않는다.
설명용 원리와 개발 계약을 구분하며, 실측은 연결된 EVIDENCE 문서를 따른다.

## 2026-10-10 상태 갱신

[실제 입력·직사각형 축·cache A/분리 B 검증](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md)을 완료했다.
[Vast 준비 결과](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-로컬-준비-결과)에
상대경로 기반 5K 전달본, 직접 실행 학습, 실제 4-class AMP 저장·재개,
로컬 백업 검증과 원격 실행 순서를 정리했다. 4090 연산 비용 재측정, 최종 입력 선정,
장기 학습과 체크포인트의 Drive 외부 백업 검증은 아직 수행하지 않았다.
데이터셋 자체의 Drive 백업은 완료했으며 체크포인트 백업과 구분한다.

초기화 대안으로 nuScenes 카메라 backbone/neck의 가중치를 실제로 로딩하고 동일성을 확인했다.
기존 정사각형 체크포인트의 LiDAR/BEV 방향은 새 [Y,X] 계약과 달라 텐서 크기만 맞는다고
전체 가중치를 로드하지 않는다. 이 대안의 준비가 전체 사전학습 가중치 적용을 완료했다는
뜻은 아니다. 원격 학습 전에 전체 축 변환의 검증 결과 또는 대안 선택을 명시한다.
이 대안을 사용하면 depth/class/head와 LiDAR/BEV는 새로 초기화한다.
아래의 'cache/adapter/번들 미구현·GPU 미실행'은 10-09 시점의 이력이며 최신 상태가 아니다.

## Sunny13 수집 종료 후 묶음 실험 합의

2026-10-09 사용자 합의. 이번 Sunny13 5K 수집을 먼저 마치고 MORAI를 끈 뒤,
입력·pooling·초기화 실험을 묶어서 진행한다. 20:49 KST 대화에서 남은 약 1시간 30분은
사용자 예상이며 완료 시각이나 현재 진행률을 인증하는 값이 아니다. 이 합의는 실험 순서의 결정이지
지금 GPU 작업·본학습·다음 환경 수집을 자동 시작하라는 요청이 아니다.

다음 작업은 이 절부터 읽고 아래 입력/crop·4090 비용·CUDA-BEVFusion 조사 링크로
세부 context를 복구한다. 수집 종료와 MORAI 종료를 확인한 뒤 사용자의 실행 요청에 따라
진행한다. Sunny13 5K 완료와 6 환경 30K 완료를 혼동하지 않는다.

원격 학습을 선택하면 [Vast.ai 단일4090 실행 계획](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md)을
함께 따른다. 로컬 검증으로 후보를 좁힌 뒤 임대 4090에서 추론·학습 비용을 재확인하고 최종 설정을
동결한다. SSH/direct runner·전달 용량·외부 checkpoint 백업 및 재개 검증이 선행 조건이며,
임대·업로드·장기 학습은 아직 시작하지 않았다.

| 순서 | 수행할 일 | 완료 조건 |
|---|---|---|
| 1 | 성공/실패·raw/annotation/metadata 무결성, 자산·거리·근접 coverage 및 geometry group split 감사 | 사용 가능한 데이터와 부족분을 분리; 원본 보존 |
| 2 | 학습/Docker/ROS 전처리와 투영 정합, 384×896/448×896/512×1056 crop 비교 | 잘림·가시성·픽셀 밀도로 후보 축소; 기존 320 기준도 보존 |
| 3 | geometry/index cache A → depth/feature 분리 pooling B 구현·검증 | 기존 출력과 gradient 정합, cache 무효화·경계·직사각형 축·stream 검사; 기존 경로 fallback |
| 4 | 동일 조건의 baseline/A/A+B latency·VRAM 및 짧은 학습 step 측정 | 추천 입력·격자·depth·연산 구성 하나와 fallback 하나 선정; 학습시간 실측 갱신 |
| 5 | 공식 nuScenes 호환 초기화 adapter·로딩 감사, 4-class single-frame 학습 smoke 및 resume 확인 | 본학습 준비 결과를 사용자에게 보고; 장기 학습은 별도 요청에 따라 시작 |

해상도 비교에서는 voxel/camera BEV/depth 간격과 class/query·입력 장면·정밀도 등을
맞추고 변경 요인을 분리한다. 새 해상도·depth shape에 필요한 초기화는 명시하며
비용 측정을 학습 완료 모델의 정확도 평가로 해석하지 않는다. CPU 준비 포함 시간과
모델 시간, ROS 전달 지연을 구분하고 p95·drop·peak VRAM을 확인한다.

### 선택과 종료 기준

- 모든 기술의 적용이 목표가 아니다. 출력 정합 실패·이득 부족·과도한 구현 비용은
  근거를 남기고 기존 경로로 돌아간다. 최적화 미완료 때문에 본학습을 무기한 미루지 않는다.
- GPU 전처리·v3 추가 최적화·부분 TensorRT는 앞 단계의 잔여 병목과 일정에 따라 선택한다.
  전체 CUDA-BEVFusion 이주는 후순위이며 이번 묶음 실험의 필수 성공 조건이 아니다.
- 학습 전에 잘림·좌표·속도·메모리 문제는 검증할 수 있지만 해상도별 탐지 성능 우열은
  학습 및 mission rosbag 평가가 필요하다. 본학습 전에 정확도 최적 구성을 확정했다고 주장하지 않는다.
- 원본 이미지·LiDAR·annotation을 보존하면 crop/해상도 및 visibility/info를 다시 만들 수 있다.
  같은 연산을 보존하는 pooling/cache 교체 자체 때문에 재수집할 필요는 없다.
  누락 센서·잘못된 원본 GT·부족한 배치 분포는 별도 감사 후 해당 조건을 보강한다.
- 보고는 수행 내용·측정 결과·불확실성·추천 구성·fallback·남은 과제로 묶는다.
  본학습 시작 직전에 사용자에게 검토받으며, production E25와 기존 tracker는 유지한다.
- 이후 5 환경 수집 전에는 기존 [collector 반복 비용 A/B](../../dataset/morai_od/docs/261009_base5_collection_cost_and_next_experiments.md)
  계획도 유지한다. 이번 GPU 실험과 별개이며 완료된 Sunny13 raw를 바꾸지 않는다.

## 결정과 남은 선택

아래 표는 초기 계획 당시의 후보 상태를 보존한다. 이후 검증한 입력·정밀도·연산 구성과 초기화 준비 상태는
상단의 상태 갱신 및 연결된 최신 EVIDENCE 문서를 따른다.

| 항목 | 현재 상태 | 다음 기준 |
|---|---|---|
| 데이터 | Sunny13 5K 보존, 나머지 5 환경은 각각 새 배치 5K로 총 30K | Base3를 기본 학습본에 섞지 않음; 구성표·주요자산 할당 유지 |
| 모델 | 4-camera + LiDAR single-frame, 7D BBox | Temporal/query memory/OD velocity 회귀 미사용 |
| 이미지 | 코드 후보 320×896, 최종 입력 미확정 | 448×896 아래 정렬 우선, 384×896 비용 대안, 512×1056 원거리 밀도 대안 |
| 범위 | label 중심 ROI 종방향 ±100m, 횡방향 ±30m | encoder padding 범위와 label ROI를 구분 |
| 공간/depth 간격 | 코드 후보 voxel 0.10m / camera BEV 0.40m / depth 1.0m | 최종 선택 전 비용 대조의 0.075/0.30/0.50m와 혼동 금지 |
| 초기화 | 현재 Base5 config는 load_from=None | 호환되는 공식 nuScenes 가중치 재사용을 추천, 새 adapter 검증 후 반영 |
| 연산 최적화 | 기존 CUDA pooling 사용 | A: 고정 기하/인덱스 cache → B: BEVPoolv2 방식, 독립 검증 |
| threshold | production E25는 0.40/0.15/0.10 | 새 모델은 rosbag dev-val에서 별도 선택 |

crop 결과는 [입력 후보 분석](261009_BASE5_CAMERA_CROP_INVESTIGATION.md),
비용과 전달 번들은 [4090 시험 계획](261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md)을 따른다.
최적화 성공을 본학습의 필수조건으로 만들지 않는다. 일정상 필요하면 검증된 기존 pooling으로 학습한다.

2026-10-09 [CUDA-BEVFusion·v2/v3 배포 조사](261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md)를
반영했다. PyTorch 학습 체계는 유지하고 필요한 연산을 선별 이식한다. CUDA-BEVFusion의
depth-outer pooling과 NVIDIA가 소개한 v3는 다르며, 전체 저장소 교체나 v3 이름만으로
속도·정확도·학습 호환을 보장하지 않는다. 아래 B와 후속 TensorRT는 아직 미구현이다.

## Base3/E25 초기화의 실제 이력

Base3 production E25를 만든 `bf-sw-fresh-e40-rtx5060ti-260918-005536`은 완전한
랜덤 초기화가 아니다. `fresh`는 MORAI 호환 nuScenes checkpoint를 로드하고
optimizer/scheduler를 새로 시작했다는 뜻이다. E40 실행 중 E25를 선정했다.

- 실행 설정: [resolved_experiment.yaml](../../artifacts/detection3d/bevfusion/team_training/sw/bf-sw-fresh-e40-rtx5060ti-260918-005536/resolved_experiment.yaml).
  checkpoint는 `bevfusion_nuscenes_morai3_compatible.pth`, lr 1e-4, 물리 batch 4였다.
- [run_manifest.json](../../artifacts/detection3d/bevfusion/team_training/sw/bf-sw-fresh-e40-rtx5060ti-260918-005536/run_manifest.json)의
  checkpoint SHA256은 `2a9ce3e01a723581f8e76b247887a704c4751f2ebcc9b36884c266466700a247`이다.
- [Base3 adapter](../../perception/detection3d/bevfusion/tools/adapt_nuscenes_checkpoint.py)는
  nuScenes 10-class를 Vehicle/Pedestrian/Obstacle로 평균 집계했다. 분류 가중치를 전부
  랜덤으로 초기화한 것이 아니다. LiDAR 입력은 5→4채널, velocity head와 grid buffer는 제외했다.
- [현재 Base5 후보](../../perception/detection3d/bevfusion/configs/bevfusion_base5_independent_100m.py)는
  load_from=None이며 상속된 camera init_cfg도 None이다. 별도 override 없는 후보는
  checkpoint-free 초기화다. 이 문서의 추천 초기화가 코드에 이미 반영됐다고 해석하지 않는다.

공식 BEVFusion의 [학습 절차](https://github.com/mit-han-lab/bevfusion#training)도
camera nuImages pretrained와 LiDAR checkpoint를 사용하는 fusion 학습을 안내한다.
Base5-only 학습은 데이터 독립성의 결정이지 가중치까지 랜덤이어야 한다는 뜻은 아니다.

## Base5 초기화 추천과 로딩 검증

기본 추천은 **공식 nuScenes BEVFusion 가중치에서 호환 부분을 가져와 Base5 전체를 학습**하는 것이다.
E25/Base4/Q1 가중치를 기본 초기값으로 쓰지 않는다. 나머지 5 환경은 새 geometry로 생성할
계획이지만 완전 랜덤 초기화가 더 유리하다고 가정하지 않는다. backbone freeze는 기본 계획에 넣지 않는다.

| 부분 | 추천 처리 | 검증할 사항 |
|---|---|---|
| Camera backbone/neck | shape 호환 가중치 로드 | 색상·정규화·채널·stage 구성, 위치 관련 buffer |
| LiDAR sparse encoder | 호환 가중치 로드, XYZI 4채널 적용 | spconv layout와 state_dict의 _metadata 보존, 실제 로딩률 |
| Fusion/BEV decoder | 호환 learned weight 로드 | 직사각형 축 순서, stride와 channel 호환 |
| 분류 heatmap/class encoding | Vehicle/Pedestrian/Obstacle/Animal 4-class로 새 초기화 우선 | 3-class adapter 그대로 사용 금지, 클래스 순서와 loss |
| BBox 회귀/공통 head | 호환 부분만 재사용 검토 | center/size/yaw 정의, 새 rectangular head의 key/shape |
| Depth 출력 | depth bin 변경으로 불일치하는 부분 새 초기화 | depth와 image feature가 합쳐진 출력층은 명시적으로 분리 검사 |
| Geometry/frustum/grid | 새 config/calibration으로 재생성 | dx/bx/nx뿐 아니라 모든 좌표 buffer, nuScenes 배치 재사용 금지 |
| Velocity/temporal | 로드/활성화하지 않음 | 7D loss와 single-frame 실행 확인 |
| Optimizer/scheduler | 새 시작 | 과거 epoch/step/optimizer state를 초기 학습에 resume하지 않음 |

로드 전에 CPU에서 source/target key와 shape를 대조하고 loaded/reset/dropped/unexpected
목록과 모듈별 tensor/parameter 로딩률을 저장한다. strict=False의 경고만 보고 성공 처리하지 않는다.
source checkpoint 경로·SHA256·adapter revision·config hash·클래스 순서·전처리 계약을 기록한다.
4-class adapter는 아직 없으며 Base3 adapter를 수정 없이 적용하지 않는다.

GPU가 분리되거나 수집이 끝난 뒤 실제 forward/backward/loss 및 AMP step을 확인한다.
pretrained/fresh 모듈의 LR 차등은 후보이며 수치 미확정이다. epoch 수와 batch도 짧은 실제 학습
비용 측정 후 동결한다. 20epoch와 과거 학습시간 외삽을 필수 수렴 시간으로 사용하지 않는다.

## BEVPoolv2 검토 근거와 한계

[BEVFusion](https://arxiv.org/abs/2205.13542)은 2022-05-26,
[BEVPoolv2](https://arxiv.org/abs/2211.17111)는 2022-11-30 최초 제출됐다.
BEVFusion은 처음부터 자체 최적화 pooling을 사용했고
[공식 구현](https://github.com/mit-han-lab/bevfusion/blob/main/mmdet3d/ops/bev_pool/bev_pool.py)도
CUDA interval reduction이다. 이후 미채택 이유나 저자의 인지 여부는 공식 근거 없이 추측하지 않는다.

BEVPoolv2의 핵심은 depth 확률과 image feature를 분리 입력하여 거대한 frustum feature의
중간 생성/저장을 생략하는 것이다. 새 탐지 모델이나 학습된 정확도 개선 기법이 아니다.
논문의 특정 pooling latency를 우리 전체 모델 latency로 사용하지 않는다.

- [참조 구현](https://github.com/HuangJunJie2017/BEVDet/blob/dev3.0/mmdet3d/ops/bev_pool_v2/bev_pool.py)은
  depth/feature를 FP32로 변환하며 backward에서도 ranks_feat 정렬을 한다. AMP의 dtype와
  학습 비용을 추론 cache 효과로 대신 인증하지 않는다.
- CUDA extension/배포 plugin 호환, XYZ/B/C 축·z collapse·정수 인덱스 정합이 필요하다.
  합산 순서 변화에 따른 수치 오차와 depth/feature 양쪽 gradient를 검증해야 한다.
- 고정 인덱스 cache와 v2 kernel은 별도 변경이다. calibration/crop/augmentation이 변하면
  cache를 재생성한다. 동적 calibration이 v2 자체를 사용할 수 없다는 뜻은 아니다.
- 입력·격자·장면 밀도에 따라 효과가 다르며 backbone/sparse encoder/head 비용은 남는다.
  구현이 정확할 경우 탐지 능력을 희생하는 것이 목표는 아니지만 속도와 정확도 모두 검증 전이다.

## 최적화 A/B 구현 순서

현재 [DepthLSS](../../third_party/mmdetection3d/projects/BEVFusion/bevfusion/depth_lss.py)는
매 forward geometry를 계산하고 depth×feature 중간 tensor를 만든다.
[pooling](../../third_party/mmdetection3d/projects/BEVFusion/bevfusion/ops/bev_pool/bev_pool.py)은
ranks 정렬 후 CUDA로 합산한다. 따라서 CPU pooling을 처음 CUDA로 옮기는 작업이 아니다.

| 단계 | 구현 범위 | 승격 조건 |
|---|---|---|
| A | inference-only 고정 geometry/valid index/rank/interval cache, 기존 pooling 유지 | 기존과 출력 정합, cache 무효화와 메모리 상한, 실제 단계 비용 감소 |
| B | 공개 v2를 backward 기준으로 삼은 depth/feature 분리 pooling, 기존 경로 fallback | CPU 기준/기존 CUDA와 forward·gradient 정합, A와 A+B 분리 측정 |
| B 추론 후보 | CUDA-BEVFusion depth-outer, FP16 load/FP32 누적 및 명시적 index 개선 | 같은 dtype/shape의 추가 이득, channel/stream/경계/직사각형 정합; 공식 v3 이식이라고 혼칭하지 않음 |
| 후속 | GPU 이미지 전처리, camera 부분 compile/TensorRT/CUDA graph | 앞 단계 실측 후 병목이 남을 때, 배포와 수치 회귀 검증; 전체 CUDA-BEVFusion 이주는 후순위 |
| 별도 모델 변경 | backbone/channel 축소·distillation 등 | 재학습 필요, 이번 초기 최적화와 분리 |

A/B 모두 opt-in/default OFF로 시작한다. 최종 옵션명은 구현 시 정하며 현재 존재하는 옵션인 것처럼
명령어에 쓰지 않는다. cache는 camera 순서, K/extrinsic/img_aug/lidar_aug, feature shape,
depth/grid bounds, batch, device/dtype을 포함해 무효화하며 엔트리와 메모리를 제한한다.
고정 camera→LiDAR 상대기하와 변화하는 world pose를 구분한다. 학습의 랜덤 augmentation에
추론 cache를 그대로 사용하지 않는다. 현재 `.long()`의 음수 경계 처리도 비교 중 임의 변경하지 않는다.

포인트 deepcopy는 DepthLSS의 좌표 변형과 연관되어 무조건 제거하지 않는다.
CUDA synchronize를 지워 측정 숫자만 줄이는 것도 최적화로 취급하지 않는다.
현재 online은 CPU OpenCV decode/resize/crop, 카메라 병렬 처리, uint8 전송과 GPU 정규화,
FP16 inference를 이미 사용한다. OpenCV CUDA 도입은 host 재다운로드/재업로드 없이 실제
pipeline 비용이 줄어드는지 확인한다. camera/LiDAR branch 병렬화도 GPU 경합 때문에 자동 이득은 아니다.

## 수집 중 가능한 검증과 종료 후 비용 시험

이전 합의에서는 작은 연산 단위 GPU 검증을 제한적으로 허용했다. 최신 합의는
위의 묶음 실험 순서대로 이번 수집 종료/MORAI 종료 후 GPU 검증을 진행하는 것이다.
아래 항목은 추후 사용자가 수집 중 단위 시험을 다시 요청할 때의 안전 조건으로 보존한다.
전체 모델/학습/latency benchmark를 수집 중 임의 실행하지 않는다. 아직 GPU 시험은 미실행이다.

1. CPU 먼저: 수동 합산 기준, 중복 rank, 빈/범위 밖 입력, 직사각형 XYZ와 batch/camera 순서,
   z 축 결합, 경계값, depth/feature gradient와 cache hit/invalidation/메모리 상한을 검사한다.
2. CUDA 최소 시험: 작은 tensor의 forward/backward, 제한된 반복부터 시작한다.
   추가 VRAM 1~2GiB 이내를 초기 예산으로 잡되 실제 context/peak를 확인하고 확대 여부를 판단한다.
3. 캡처 진행률·timeout·재시도와 GPU 여유를 전후/중간 확인한다. 영향/OOM 징후가 있으면
   **시험 프로세스만 종료**하고 collector는 건드리지 않는다. 예상보다 큰 모델을 임의 로드하지 않는다.
4. 동시 실행의 시간은 공식 latency가 아니다. 전체 모델, 학습 step과 열 안정성 측정은
   수집 종료 후 또는 별도 4090에서 실시한다.

과거 17:43 GPU snapshot의 총 사용 약 6.7GiB는 여유/peak 인증이 아니다.
4090 기존 확대 모델 warmup peak allocated 약 11.0GiB도 같은 GPU 병행의 안전 근거로 쓰지 않는다.
허용 범위가 실제 구현된 watchdog이나 수집 무영향 인증을 뜻하지는 않는다.

팀원 딸깍 번들은 미구현이다. 같은 raw·checkpoint·crop·BEV/depth에 baseline/A/A+B를
비교하고 correctness, cold/warm, 모델 단독/CPU 준비 포함 p50/p95/p99, steady/peak VRAM,
짧은 backward/optimizer step을 기록한다. 조합 전체를 본학습하는 실험은 아니다.
13Hz≈76.9ms이며 모델 평균 70ms만으로 ROS/full-stack 실시간성을 인증하지 않는다.

## 수집 종료부터 학습·배포까지

1. 성공 raw/annotation/metadata와 부족분을 확정하고 자산·거리·근접·환경 coverage를 감사한다.
2. 전처리 세 경로의 crop/resize/img_aug_matrix 정합 및 비용으로 입력·격자·depth를 동결한다.
   새 crop 기준으로 visibility/info를 재생성한다. 기존 320 camera positive를 재사용하거나 3D GT를 자르지 않는다.
3. Base5 adapter와 로딩 manifest를 검증하고 4-class single-frame 학습 smoke를 통과한다.
4. [split 계약](../../dataset/morai_od/docs/261009_base5_training_split_visibility_and_temporal_decisions.md)의
   Sunny13 train 4255/val 745를 유지한다. 나머지 25K는 새 geometry의 공간 group 일관성과
   누출을 검사해 split을 재계산한다. 4255/745를 6배한 값을 최종 30K 분할로 단정하지 않는다.
   학습시간은 실측 step으로 갱신한다.
5. offline val로 후보를 줄이고 동일 mission rosbag의 detection/tracking/Map-CV, 첫 전달거리,
   근접 누락과 ID 연속성을 평가한다. 새 threshold를 선택하고 독립 미사용 bag으로 최종 확인한다.
6. 기존 E25 fallback을 유지하며 production 변경 시 CURRENT/EVIDENCE와 runtime 전처리도 함께 반영한다.

로컬/외부 단일 GPU 학습 모두 주기적인 resumable checkpoint를 사용한다. model뿐 아니라
optimizer/scheduler/AMP scaler/epoch/global step 및 가능한 RNG/sampler 상태를 보존하고
중단→재개 smoke로 이어지는 LR와 step을 확인한다. 정확한 mid-epoch batch 재현 여부는
실행기 지원을 검증하며 무조건 보장하지 않는다. 외부 GPU를 선택하면 checkpoint 완료 후
비동기 Drive 업로드·checksum·latest 완료 파일과 실패 기록을 추가 검증한다. 아직 구현/업로드된 상태가 아니다.

## TensorRT 후속 시험의 범위와 작업 예산

2026-10-09 합의는 기존 최적화의 효과를 먼저 측정한 뒤 부분 TensorRT를 선택적으로 검토하는 것이다.
ONNX는 모델 교환 형식이며 내보내기만으로 가속을 보장하지 않는다. CUDA-BEVFusion은 주로
CUDA/TensorRT 추론 배포 참조이며 현재 PyTorch 학습 체계를 대체하는 기본 학습기로 채택하지 않는다.
호환 구조·전처리·출력 계약을 맞춘 후 학습 가중치를 내보내는 경로를 검증한다.

| 범위 | 개략 작업량 추정 | 종료 판단 |
|---|---|---|
| Camera backbone/neck 부분 export와 비용 탐색 | 반나절~하루 | 먼저 export·수치 정합·실제 이득 확인 |
| 부분 engine 연결과 실제 추론·rosbag 회귀 | 추가 1~2일 | 이득이 충분할 때 진행, PyTorch fallback 유지 |
| sparse encoder/pooling/head를 포함한 전체 전환 | 수일~1주 이상 가능 | 이번 본학습의 필수 조건으로 두지 않음 |

직접 이식한 실측 작업 시간이 아닌 일정 추정이다. 현재 spconv와 custom pooling,
카메라별 crop·4-camera·4-class·7D·직사각형 BEV export 및 CUDA/TensorRT 호환 때문에
전체 전환은 불확실성이 크다. 카메라 부분 탐색에서 이득이 작거나 호환 문제가 크면
기존 경로로 학습을 시작한다. GPU 임대 비용보다 구현과 수치·mission 회귀 검증 시간이 주요 부담이다.

학습 해상도는 [13Hz 처리 예산과 후보 예상](261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md#수집-종료-후-해상도-선택-기준)을
따른다. 학습 전에 입력·crop·grid/depth·class를 동결하되 동등한 출력의 추론 최적화는
학습 중/후에도 검증해 적용할 수 있다. TensorRT는 추론 최적화이며 본학습 시간 단축으로 계산하지 않는다.
