# Base5 입력·격자 정합·pooling 최적화 — 2026-10-10

## CURRENT / 다음 판단

Sunny13 **완료된 실제 5K 전수** crop 분석과 5060 Ti 실제 raw 비용 검증을 완료했다.
우선 학습 입력 후보는 **448×896 아래 정렬**, 원거리 픽셀 밀도 대안은 **512×1056**이다.
384×896은 비용 fallback, 576×1152는 비용 탐색 후보다. 아직 본학습 해상도를 확정하거나
production E25를 교체하지 않았다. 다음은 임대한 4090에서 동일 번들을 실행하고 최종 입력을 고르는 것이다.
원본 Sunny13/나머지 환경 수집기/runtime/hash 계약과 tracker/planning/localization은 변경하지 않았다.

추론 opt-in cache A와 **depth/feature 분리 pooling B**를 구현했다. B는 학습 역전파도 지원한다.
공식 BEVPoolv2의 분리 표현을 참고한 로컬 CUDA 구현이며 NVIDIA v3나 TensorRT 전환이 아니다.
기존 기본 모델은 그대로이며 `MoraiSeparatedRectangularDepthLSSTransform`을 명시한 후보만 사용한다.

## 입력과 원본 보존

- canonical: `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_sunny13_flexible_v140`.
- ledger SHA256: `026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43`.
- 완료 5,000장/부족 0, train4,255/val745. 원본 재수집·삭제·라벨 덮어쓰기를 하지 않았다.
- mask 분석: 카메라 관측14,353개, instance PNG9,141개, 카메라 관측 자산121종.
  수집 자산122종과 카메라 관측121종은 다른 집계다.
- 비용 입력14장은 전체5K의 nonzero finite ROI return 밀도 quantile/최대3장/패밀리별
  deterministic hash로 선택했다. AP나 성공 사례를 보고 고르지 않았다.
  각19200개 원본 beam 기록을 그대로 유지했고 점을 추가로 subsampling하지 않았다.

ignored 근거 root:
`artifacts/evaluation/bevfusion/base5_crop_final5k_261010`,
`artifacts/evaluation/bevfusion/base5_input_runtime_261010`.

## 1. Crop와 GT 재선별

native mask16px 이상 관측 중 **원본 foreground의 90% 이상 유지** 비율이다.
센서 자체에서 이미 잘린 객체의 완전 복원이나 탐지 정확도를 뜻하지 않는다.

| 아래 정렬 입력 | 전방 | 좌전방 | 우전방 | 후방 |
|---|---:|---:|---:|---:|
| 320×896 | 98.25% | 8.03% | 9.21% | 98.29% |
| 384×896 | 99.48% | 94.63% | 95.36% | 99.00% |
| 448×896 | 99.90% | 98.80% | 99.23% | 99.74% |
| 512×1056 | 99.86% | 98.51% | 99.09% | 99.64% |

512는 폭도 늘려 픽셀 밀도를 높이므로 448보다 crop 보존율이 무조건 높지는 않다.
0–5m 좌/우 Obstacle의 90% 보존은384/448에서100%였으나,
320측면 crop만 위로 옮기면48.84/47.46%로 떨어졌다. crop 위치 이동만으로 해결하지 않는다.

unambiguous native-camera-visible, zero-LiDAR component785개의 recomputed gate:
320은583개, 448은639개(+56), 512는679개(+96)다.
**이 차이는 camera-only 단서 보존이지 AP 개선값이 아니다.**

`build_base5_crop_infos.py`는 전수 분석/ledgerSHA를 검증하고 새 derived audit/infos를 만든다.
원본3D center/size/rotation/LiDAR 수와 collection 승인은 바꾸지 않는다.
camera mask16px/최소extent4px, LiDAR≥1 또는 camera 조건을 유지한다.
복합체에서 camera component 대응이 모호하면 자식 GT의 camera support를 임의로 만들지 않는다.

| 변환본 | 전체 GT | Vehicle | Pedestrian | Obstacle | Animal | 기존 대비 추가/제외 |
|---|---:|---:|---:|---:|---:|---:|
| 기존320 임시본 | 10,959 | 4,513 | 2,937 | 3,239 | 270 | — |
| 448 derived | 11,012 | 4,518 | 2,952 | 3,271 | 271 | +59/−6 |
| 512 derived | 11,052 | 4,519 | 2,963 | 3,289 | 281 | +95/−2 |

이전 native audit의 Pillow integer boundary와 현재 OpenCV nearest-mask 판정 차이로
threshold 경계의 소수 GT가 제외될 수 있다. 추가56/96의 대조 cohort와 실제59/95는
다른 집계이며 둘을 동일 수치라고 쓰지 않는다. 최종 학습 infos는 선택한 crop 기준으로 확정한다.

`artifacts/datasets/base5_crop_448x896_261010_v2/infos`,
`artifacts/datasets/base5_crop_512x1056_261010/infos`에 새 변환본이 있다.
train/val split4255/745 유지. 512 비용/역전파 probe 일부는448 GT subset을 사용했으므로
512 본학습은512 derived infos를 사용해야 한다.

실제 MMDetection3D dataset loader→crop→pack→GPU normalization과 Docker raw online 경로를
448/512 각각14장×4camera에서 비교했다. PNG pixels/정규화된 pixels/points는 exact,
calibration/augmentation matrix는 tolerance 내 일치했다.
추가로 E25 실제 host 순수 전처리 함수, 새 후보의 RGB8 prepared roundtrip/투영을 검사했다.
**호스트 ROS prepared 경로는 여전히 E25256×704 고정이다. 새 shape ROS 실행은 아직 연결/검증 전이다.**

## 2. 새 직사각형 sparse 축 문제

다양한 완료5K raw를 넣자 기존 확대 후보의 일부 입력이 spconv에서 실패했다.
custom voxelizer 실제 coordinate 순서는 `(b,x,y,z)`이나 확대 config sparse_shape는
`[Y,X,Z]`였다. shape만 맞추는 기존 square 기반 시험으로는 이를 충분히 검증하지 못했다.

새 `MoraiRectangularSparseEncoder`에서 coordinate를 `(b,y,x,z)`로 바꿔
LiDAR feature `[B,C,Y,X]`와 camera pool transpose/head index `y*X+x`를 일치시켰다.
기본 E25 square encoder는 바꾸지 않는다. 수집 raw/calibration/GT 오류가 아니므로 재수집하지 않는다.
이전에 반환된4090 확대 cost는 이 보정 전의 **역사적 입력 비용**이며 새 모델 비용을 다시 재야 한다.
누락 상태에서 실패한 JSON `input_pool_matrix.json`도 삭제하지 않고 진단으로 보존했다.
기존 E25 ±30m 비용과 새 ±100m geometry를 혼동하지 않는다.

새 축 계약으로 pretrained LiDAR/BEV kernel을 로드할 경우 단순 shape 호환만으로
방향 정합을 보장하지 못한다. 초기화 adapter에서 방향/버퍼/depth/class 감사를 별도로 해야 한다.

## 3. 같은 정밀도에서 A/B 비용

5060Ti16GB, team-cu128 이미지, torch2.7.1, batch1 AMP,
fresh4-class/200query/7D, label X±100/Y±30m.
내부 padding X±100.8/Y±32.4m, **voxel.075m/cameraBEV.30m/depth.50m(198bins)**를 모두 유지했다.
기존 Base5 coarse 후보의.10/.40/1.0 비용과 섞지 않는다.
모든 mode는 같은 shape의 같은 가중치다. 새 학습 성능/실제4090/ROS·풀스택 시간은 아니다.

JPEG90 controlled input,14raw×3회/8warmup, reverse-order 반복:

| 입력 | baseline 모델 평균 | cache A | cache+분리 B | B 서비스 평균 | B 서비스 p95 |
|---|---:|---:|---:|---:|---:|
| 320×896 | 125.39ms | 77.74ms | 63.79ms | 71.33ms | 73.39ms |
| 384×896 | 144.61ms | 87.45ms | 70.11ms | 78.02ms | 80.82ms |
| 448×896 | 162.67ms | 96.36ms | 74.91ms | 82.83ms | 84.98ms |
| 512×1056 | 216.75ms | 126.95ms | 93.52ms | 102.98ms | 104.96ms |
| 576×1152 탐색 | 263.20ms | 미측정 | 109.83ms | 120.44ms | 122.57ms |

서비스는 CPU decode/resize/pack + model + prediction CPU 복사다.
JPEG90은 실제 MORAI packet 품질 확정값이 아니라 비용 대조다. 원본PNG 입력에서는 CPU가
약22–25ms로 증가했다. 이는 online network JPEG 성능과 학습PNG loader 시간을 구분해야 하는 이유다.
순서 반전/PNG 반복에서도 모델 비용은 유사했다.

448의 모델54.0%, 512의56.9% 단축을 확인했다. 주로 **반복 기하/정렬 제거와 큰 중간텐서 제거** 효과다.

| 448 단계 stream span | baseline | A | B |
|---|---:|---:|---:|
| Camera backbone | 29.98ms | 30.03ms | 29.99ms |
| Geometry | 52.32ms | 0.20ms | 0.21ms |
| Depth/feature | 9.87ms | 9.90ms | 5.88ms |
| Pool | 33.66ms | 19.35ms | 1.92ms |
| 전체 view transform | 100.16ms | 33.79ms | 12.44ms |
| LiDAR sparse encoder | 11.01ms | 10.99ms | 10.87ms |
| Head/decode | 8.09ms | 8.07ms | 7.97ms |

stream span은 host idle을 포함한다. parent와 substage를 더하지 않으며 FLOPs/순수kernel 시간이라고 쓰지 않는다.
backbone이나 모델 폭을 줄인 결과가 아니다. B 후에는 camera backbone 비중이 커졌다.
448 PyTorch steady allocated는baseline4229.7MiB→B812.2MiB,
512는5625.4→949.4MiB였다. CUDA context/allocator/reserved/desktop 포함 전체 VRAM이 아니다.

## 4. Correctness / 학습 smoke

- CPU7tests PASS(CUDA 전용3skip), CUDA7tests PASS. empty/batch/negative boundary,
  reference forward/gradient, 실제depthnet gradient, cache value invalidation,
  training bypass/zero budget/current stream/shape guard를 검사했다.
- rectangular/config tests6개와 crop infos tests3개 PASS.
- full model448/512 각각14실제장면에서 A/B의 view 최대차이≤1.51e−5,
  decoded boxes/labels/scores는 설정tolerance 내 일치했다.
- 학습 상태에서는 기하 cache를 재사용하지 않는다. FP32 feature-gradient atomics로
  reduction 순서 차이가 가능하며 bitwise deterministic 학습을 인증하지 않는다.
- 실제 GT, AdamW1e−4, AMP init_scale32, clipping35, 각3step:
  448/512 baseline batch1 PASS, B batch1/2 PASS. baseline batch2는16GB에서OOM이었다.
  loss/gradient 유한, scaler step skip 없음. 짧은 loss 감소는 수렴/AP 인증이 아니다.
- 초기 smoke의 PyTorch 기본scale65536에서 baseline/B 모두 gradient overflow가 발생했다.
  테스트를 실제 config의scale32로 맞춘 뒤 통과했다. 실패 결과도 보존한다.
- 처음 포함 autotune/allocator 비용 때문에 warmup peak를 해상도별 정밀 메모리 비교로 쓰지 않는다.
  본학습 시간은 실제 loader/val/checkpoint 포함 원격 짧은 측정으로 다시 추정한다.
- Cache A는 upstream pool의 default CUDA stream만 허용한다. B는 current stream을 사용한다.
  single worker instance용이며 concurrent 호출/다중worker를 인증하지 않는다. double backward는 미지원이다.

## 실행 코드와 전달

새 opt-in config: `perception/detection3d/bevfusion/configs/bevfusion_base5_dense_448_separated_candidate.py`.
현재 load_from=None은 비용/wiring 용도다. 이를 최종 초기화 선택으로 해석하지 않는다.

`build_portable_input_pool_bundle.py`가14샘플/GT/코드만 묶는다. pretrained/full dataset/ROS는 필요 없다.
`portable_input_pool/run.sh`는 host팀이미지 또는 이미컨테이너인Vast에서 직접 실행한다.
Vast 컨테이너 안에서 다시 docker run을 중첩하지 않는다. checksum 검증/로그/JSON/최종ZIP을 남긴다.
첫 JIT compile은 해당 GPU에서 수행하고 sm120 binary를4090으로 복사하지 않는다.

남은 순서:

1. 4090 동일 조건 비용/학습메모리 재측정 →448/512(필요시576) 최종 선택.
2. 선택shape의 infos/초기화 adapter·checkpoint resume/외부 백업 감사.
3. 최종 offline 학습. 최신 mission bag은 dev-val/독립test로 유지.
4. 새shape ROS raw/prepared 경로와4-class postprocess/NMS/threshold 연계.
5. detection/tracking/Map-CV 및 판단 전달 지연 회귀 후 production 승격.

이번 작업은 TensorRT/ONNX나 CUDA-BEVFusion 저장소 전체 이주를 수행하지 않았다.
현저한 pooling 개선 이후의 잔여 병목을4090에서 보고 선택한다.

참고: [BEVDet BEVPoolv2 Python](https://github.com/HuangJunJie2017/BEVDet/blob/dev3.0/mmdet3d/ops/bev_pool_v2/bev_pool.py),
[CUDA interval reduction](https://github.com/HuangJunJie2017/BEVDet/blob/dev3.0/mmdet3d/ops/bev_pool_v2/src/bev_pool_cuda.cu).
