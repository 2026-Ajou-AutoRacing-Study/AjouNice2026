# Base5 입력·격자 정합·pooling 최적화 — 2026-10-10

## CURRENT / 다음 판단

현재 실행 순서는 [단일 계획 허브](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub)를 따른다.
최신 로컬 학습 비용 개선은 [21절](#base5-training-runtime-local-261011), 원격 이전·비용은
19절, 실제 외부 백업 완료는 20절이다. 아래 날짜별 기록의 당시 ‘미완료’를 최신 상태로
재사용하지 않는다. 본학습·AP 검증은 아직이며, 21절 소스는 아직 4090에 반영하지 않았다.
raw·센서·클래스·GT 수·split·production E25는 보존한다. 최신 입력은 Q90 JPEG와
`native_rpy_fixed_yaw_enclosure_v1` 라벨이다. 공간 7.5/30cm·ROI X±100/Y±30m를 유지한다.
depth 25cm는 우선 후보이고, 기존 원격 준비 계약은 50cm다. 재수집하지 않는다.

## 초기 기록의 요약 — 최신 실행 상태가 아닌 이력

Sunny13 **수집이 완료된 실제 5K 전체**의 crop 분석과 5060 Ti에서의 원본 입력 비용 검증을 완료했다.
추가 원격 검증 이후 추천은 **512×1056 아래 정렬**, 더 빠른 대안은 **448×896**이다.
384×896은 비용 fallback, 576×1152는 비용 탐색 후보다. 아직 본학습을 시작하거나
production E25를 교체하지 않았다. 2026-10-10 17:31~17:34 KST에 임대한 4090에서
동일 번들의 비용·동등성·짧은 역전파 검사를 완료했다. 아래 원격 결과가 최신 비용 근거다.
20:23 이후에는 서로 다른 300장 비용 비교, 64장 동등성, 실제 5K 학습 로더와
745장 전체 validation, 원격 저장·재개를 추가 검증했다. 최신 결과는 아래 6절이다.
512×1056·물리 batch 4를 추천하며 본학습은 시작하지 않았다.
현재 남은 시작 조건은 임대 머신의 외부 네트워크 복구와 실제 체크포인트 외부 백업이다.
2026-10-10 22:08 KST에는 좌우 ±40m 비용 후보를 별도로 측정했다. 대표 표본의 서비스
평균은 ±30m 69.77ms / ±40m 71.61ms다. 아래 7절의 동일 조건 비교이며,
현재 학습·라벨 ROI를 ±40m로 승격하거나 본학습 시간 추정을 갱신한 것은 아니다.
후속 미션 경로·개발 bag 확인과 사용자 합의에 따라 **이번 Base5 학습은 전후방 ±100m,
좌우 ±30m를 유지하고 ±40m 확대를 기각한다.** 아래 8절이 현재 범위 선택 기준이다.
원본 Sunny13/나머지 환경 수집기/runtime/hash 계약과 tracker/planning/localization은 변경하지 않았다.
2026-10-11 현재는 사용자 승인으로 **train·val·test·분석 모두 native Q90 JPEG**를 사용한다.
선택 config는 `bevfusion_base5_dense_512_native_jpeg90.py`이며 원본 RGB PNG는 로컬
보존·변환용이다. 아래 12절의 all-JPEG 전달본·Drive 교체가 현재 실행 기준이다.
mask·depth·LiDAR·GT·보정은 무손실로 유지하고 blur·노출·noise는 OFF다.
9절의 80%, 10절의 train 캐시, 11절의 train JPEG / val PNG는 비교 이력이다.
11절의 PNG/JPEG 짧은 학습 비용은 보존한다. 새 입력의 AP는 미검증이다.
2026-10-11 사용자 재개 요청으로 Vast에 접속해 전체 JPEG 변환·전수 검증을 완료했다.
아래 13절이 최신 원격 입력 기준이며, 외부 checkpoint 백업과 본학습은 아직 미완료다.
사용자가 다시 Vast를 중단한 뒤 **로컬에서만** Base5 유효 반사점 필터와 uint8 이미지
운반 경로를 검증했다. 아래 14절이 최신 로컬 입력 기준이다. 13절의 원격 소스와 준비 계약에는
아직 이 변경이 없으므로 재개 시 새 소스 전달·새 work-dir 준비·짧은 재검증이 필요하다.

추론 opt-in cache A와 **depth/feature 분리 pooling B**를 구현했다. B는 학습 역전파도 지원한다.
공식 BEVPoolv2의 분리 표현을 참고한 로컬 CUDA 구현이며 NVIDIA v3나 TensorRT 전환이 아니다.
기존 기본 모델은 그대로이며 `MoraiSeparatedRectangularDepthLSSTransform`을 명시한 후보만 사용한다.

## 입력과 원본 보존

- canonical: `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_sunny13_flexible_v140`.
- ledger SHA256: `026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43`.
- 완료 5,000장/부족 0, train 4,255/val 745. 원본 재수집·삭제·라벨 덮어쓰기를 하지 않았다.
- mask 분석: 카메라 관측 14,353개, instance PNG 9,141개, 카메라 관측 자산 121종.
  수집 자산 122종과 카메라 관측 121종은 다른 집계다.
- 비용 측정 입력 14장은 전체 5K의 ROI 내 유효 LiDAR 반사점 밀도 분위수, 밀도 최대 사례 3장, 패밀리별
  결정적 해시 선택을 조합해 골랐다. AP나 성공 사례를 보고 고르지 않았다.
  각 입력의 원본 beam 기록 19200개를 그대로 유지했고, 점을 추가로 줄이지 않았다.

Git 추적에서 제외한 근거 경로:
`artifacts/evaluation/bevfusion/base5_crop_final5k_261010`,
`artifacts/evaluation/bevfusion/base5_input_runtime_261010`.

## 1. Crop와 GT 재선별

native mask가 16px 이상인 관측 중 **원본 객체 픽셀의 90% 이상을 유지한** 비율이다.
센서 자체에서 이미 잘린 객체의 완전 복원이나 탐지 정확도를 뜻하지 않는다.

| 아래 정렬 입력 | 전방 | 좌전방 | 우전방 | 후방 |
|---|---:|---:|---:|---:|
| 320×896 | 98.25% | 8.03% | 9.21% | 98.29% |
| 384×896 | 99.48% | 94.63% | 95.36% | 99.00% |
| 448×896 | 99.90% | 98.80% | 99.23% | 99.74% |
| 512×1056 | 99.86% | 98.51% | 99.09% | 99.64% |

512는 폭도 늘려 픽셀 밀도를 높이므로 448보다 crop 보존율이 무조건 높지는 않다.
0–5m 좌/우 Obstacle의 90% 보존은 384/448에서 100%였으나,
320의 측면 crop만 위로 옮기면 48.84/47.46%로 떨어졌다. crop 위치 이동만으로 해결하지 않는다.

native 카메라에서 보이고 구성요소 대응이 명확하지만 LiDAR 점은 없는 component 785개의 가시성 조건을 재계산했다.
조건을 통과한 수는 320에서 583개, 448에서 639개(+56), 512에서 679개(+96)다.
**이 차이는 camera-only 단서 보존이지 AP 개선값이 아니다.**

`build_base5_crop_infos.py`는 전수 분석 결과와 ledger SHA를 검증하고 새 파생 audit/infos를 만든다.
원본 3D 중심·크기·회전·LiDAR 점 수와 수집 승인 여부는 바꾸지 않는다.
camera mask 16px/최소 extent 4px, LiDAR≥1 또는 camera 조건을 유지한다.
복합체에서 카메라 구성요소 대응이 모호하면 자식 GT의 카메라 관측 근거를 임의로 만들지 않는다.

| 변환본 | 전체 GT | Vehicle | Pedestrian | Obstacle | Animal | 기존 대비 추가/제외 |
|---|---:|---:|---:|---:|---:|---:|
| 기존 320 임시본 | 10,959 | 4,513 | 2,937 | 3,239 | 270 | — |
| 448 derived | 11,012 | 4,518 | 2,952 | 3,271 | 271 | +59/−6 |
| 512 derived | 11,052 | 4,519 | 2,963 | 3,289 | 281 | +95/−2 |

이전 native audit의 Pillow integer boundary와 현재 OpenCV nearest-mask 판정 차이로
threshold 경계의 소수 GT가 제외될 수 있다. 추가 56/96의 대조 cohort와 실제 59/95는
다른 집계이며 둘을 동일 수치라고 쓰지 않는다. 최종 학습 infos는 선택한 crop 기준으로 확정한다.

`artifacts/datasets/base5_crop_448x896_261010_v2/infos`,
`artifacts/datasets/base5_crop_512x1056_261010/infos`에 새 변환본이 있다.
train/val split 4255/745는 유지한다. 512 비용·역전파 시험 일부는 448 GT의 부분 집합을 사용했으므로
512 본학습에는 512 전용 파생 infos를 사용해야 한다.

실제 MMDetection3D dataset loader→crop→pack→GPU normalization과 Docker raw online 경로를
448/512 각각 14장×4카메라에서 비교했다. PNG 픽셀·정규화된 픽셀·포인트는 정확히 일치했고,
보정·증강 행렬은 허용 오차 내에서 일치했다.
추가로 E25 실제 host 순수 전처리 함수, 새 후보의 RGB8 prepared roundtrip/투영을 검사했다.
**호스트 ROS prepared 경로는 여전히 E25의 256×704 입력에 고정돼 있다. 새 입력 크기의 ROS 실행은 아직 연결·검증 전이다.**

## 2. 새 직사각형 sparse 축 문제

다양한 완료 5K raw를 넣자 기존 확대 후보의 일부 입력이 spconv에서 실패했다.
custom voxelizer 실제 coordinate 순서는 `(b,x,y,z)`이나 확대 config sparse_shape는
`[Y,X,Z]`였다. shape만 맞추는 기존 square 기반 시험으로는 이를 충분히 검증하지 못했다.

새 `MoraiRectangularSparseEncoder`에서 coordinate를 `(b,y,x,z)`로 바꿔
LiDAR feature `[B,C,Y,X]`와 camera pool transpose/head index `y*X+x`를 일치시켰다.
기본 E25 square encoder는 바꾸지 않는다. 수집 raw/calibration/GT 오류가 아니므로 재수집하지 않는다.
이전에 반환된 4090 확대 cost는 이 보정 전의 **역사적 입력 비용**이며 새 모델 비용을 다시 재야 한다.
누락 상태에서 실패한 JSON `input_pool_matrix.json`도 삭제하지 않고 진단으로 보존했다.
기존 E25 ±30m 비용과 새 ±100m geometry를 혼동하지 않는다.

새 축 계약으로 pretrained LiDAR/BEV kernel을 로드할 경우 단순 shape 호환만으로
방향 정합을 보장하지 못한다. 초기화 adapter에서 방향/버퍼/depth/class 감사를 별도로 해야 한다.

## 3. 같은 정밀도에서 A/B 비용

측정 환경은 5060 Ti 16GB, team-cu128 이미지, torch 2.7.1, batch 1 AMP다.
새 4-class/200 query/7D 후보이며 label 범위는 X±100/Y±30m다.
내부 padding X±100.8/Y±32.4m, **voxel .075m/camera BEV .30m/depth .50m(198 bins)**를 모두 유지했다.
기존 Base5의 성긴 격자 후보 .10/.40/1.0 비용과 섞지 않는다.
각 입력 크기 안에서 모든 mode는 같은 가중치를 사용한다. 새 학습 성능이나 실제 4090·ROS·풀스택 시간은 아니다.

JPEG90으로 통제한 입력에서 raw 14장×3회, warmup 8회를 수행했고 실행 순서를 뒤집어 반복했다.

| 입력 | baseline 모델 평균 | cache A | cache+분리 B | B 서비스 평균 | B 서비스 p95 |
|---|---:|---:|---:|---:|---:|
| 320×896 | 125.39ms | 77.74ms | 63.79ms | 71.33ms | 73.39ms |
| 384×896 | 144.61ms | 87.45ms | 70.11ms | 78.02ms | 80.82ms |
| 448×896 | 162.67ms | 96.36ms | 74.91ms | 82.83ms | 84.98ms |
| 512×1056 | 216.75ms | 126.95ms | 93.52ms | 102.98ms | 104.96ms |
| 576×1152 탐색 | 263.20ms | 미측정 | 109.83ms | 120.44ms | 122.57ms |

서비스는 CPU decode/resize/pack + model + prediction CPU 복사다.
당시 JPEG90은 실제 MORAI packet 품질을 확인하지 않은 비용 대조였다. 후속 개발 bag 표본의
Q90 양자화 표 확인은 9절에 기록하며 모든 MORAI 환경의 보장으로 일반화하지 않는다. 원본 PNG 입력에서는 CPU가
약 22–25ms로 증가했다. 이는 online network JPEG 성능과 학습 PNG loader 시간을 구분해야 하는 이유다.
순서 반전/PNG 반복에서도 모델 비용은 유사했다.

448의 모델 54.0%, 512의 56.9% 단축을 확인했다. 주로 **반복 기하/정렬 제거와 큰 중간텐서 제거** 효과다.

| 448 단계 stream span | baseline | A | B |
|---|---:|---:|---:|
| Camera backbone | 29.98ms | 30.03ms | 29.99ms |
| Geometry | 52.32ms | 0.20ms | 0.21ms |
| Depth/feature | 9.87ms | 9.90ms | 5.88ms |
| Pool | 33.66ms | 19.35ms | 1.92ms |
| 전체 view transform | 100.16ms | 33.79ms | 12.44ms |
| LiDAR sparse encoder | 11.01ms | 10.99ms | 10.87ms |
| Head/decode | 8.09ms | 8.07ms | 7.97ms |

stream span은 host idle을 포함한다. parent와 substage를 더하지 않으며 FLOPs/순수 kernel 시간이라고 쓰지 않는다.
backbone이나 모델 폭을 줄인 결과가 아니다. B 후에는 camera backbone 비중이 커졌다.
448 PyTorch의 정상 상태 할당 메모리는 baseline 4229.7MiB→B 812.2MiB,
512는 5625.4→949.4MiB였다. CUDA context/allocator/reserved/desktop 포함 전체 VRAM이 아니다.

## 4. 정확성 검증 / 학습 smoke

- CPU 검사 7개 통과(CUDA 전용 3개 제외), CUDA 검사 7개 통과. empty/batch/negative boundary,
  reference forward/gradient, 실제 depthnet gradient, cache value invalidation,
  training bypass/zero budget/current stream/shape guard를 검사했다.
- 직사각형 격자·config 검사 6개와 crop infos 검사 3개를 통과했다.
- 전체 모델 448/512 각각의 실제 장면 14개에서 A/B의 view 최대 차이는 ≤1.51e−5였고,
  복호화한 boxes/labels/scores는 설정한 허용 오차 내에서 일치했다.
- 학습 상태에서는 기하 cache를 재사용하지 않는다. FP32 feature-gradient atomics로
  reduction 순서 차이가 가능하며 bitwise deterministic 학습을 인증하지 않는다.
- 실제 GT, AdamW 1e−4, AMP init_scale 32, clipping 35로 각각 3 step을 수행했다.
  448/512 baseline batch 1과 B batch 1/2가 통과했다. baseline batch 2는 16GB에서 OOM이었다.
  loss/gradient 유한, scaler step skip 없음. 짧은 loss 감소는 수렴/AP 인증이 아니다.
- 초기 smoke의 PyTorch 기본 scale 65536에서 baseline/B 모두 gradient overflow가 발생했다.
  테스트를 실제 config의 scale 32로 맞춘 뒤 통과했다. 실패 결과도 보존한다.
- 최초 실행의 autotune·메모리 할당 비용 때문에 warmup peak를 해상도별 정밀 메모리 비교에 쓰지 않는다.
  본학습 시간은 실제 loader/val/checkpoint 포함 원격 짧은 측정으로 다시 추정한다.
- Cache A는 upstream pool의 default CUDA stream만 허용한다. B는 current stream을 사용한다.
  single worker instance용이며 concurrent 호출/다중 worker를 인증하지 않는다. double backward는 미지원이다.

별도 self-contained bundle의 448/512·14회 비용/3-step 전체 실행도 통과했다.
재실행의 batch 1 마지막 2step 시간은 448 baseline 413.1ms→B303.6ms,
512 baseline 544.0ms→B380.7ms였다. 본학습 epoch 추정으로 바로 외삽하지 않는다.
정상 상태 할당 메모리는 448 batch 1에서 baseline 7759.4MiB→B 4678.7MiB,
512 batch 1에서 baseline 10377.2MiB→B 5730.0MiB다. B batch 2는 각각 8761.0/10871.8MiB였다.
optimizer step을 건너뛰지 않았으며 실제 depthnet weight 변경도 확인했다.

## 실행 코드와 전달

새 opt-in config: `perception/detection3d/bevfusion/configs/bevfusion_base5_dense_448_separated_candidate.py`.
현재 load_from=None 설정은 비용·연결 검증 용도다. 이를 최종 초기화 선택으로 해석하지 않는다.

`build_portable_input_pool_bundle.py`는 샘플 14개·GT·코드만 묶는다. 사전 학습 가중치·전체 데이터셋·ROS는 필요 없다.
`portable_input_pool/run.sh`는 호스트의 팀 이미지 또는 이미 컨테이너로 실행 중인 Vast에서 직접 실행한다.
Vast 컨테이너 안에서 다시 docker run을 중첩하지 않는다. checksum 검증/로그/JSON/최종 ZIP을 남긴다.
첫 JIT compile은 해당 GPU에서 수행하고 sm120 binary를 4090으로 복사하지 않는다.
실주행에 연결할 때는 최초 JIT compile/모델 warmup을 출발 전에 끝내야 한다.
측정된 74/93ms에 최초 CUDA 컴파일 시간은 포함되지 않는다. 현재 opt-in 타입만 바꿔
첫 프레임부터 곧바로 그 지연이 나온다고 보장하지 않는다.

최종 local-qualified 묶음:
`artifacts/handoff/input_pool_gpu_261010_final_v2/ajounice_input_pool_gpu.zip`
(52,592,101 bytes, SHA256 `9bd2c750a4831a77ecf38214f82e9516025fd6e0d58b4225d93f48c31eef5d62`).
payload 417개의 checksum이 일치한다. 별도 폴더·마운트에서 다음 명령을 끝까지 검증했다.

```bash
bash artifacts/handoff/input_pool_gpu_261010_final_v2/ajounice_input_pool_gpu/run.sh \
  --iterations 14 --shapes 448x896 512x1056
```

CPU/CUDA 단위 검사 → 비용 case 6개 → 전체 모델 동등성 검사 4개 → 3-step 학습 성공 6개와 baseline batch 2 OOM 2개 기록을 완료했다.
runner 3개가 exit 0으로 종료했고 결과 ZIP 생성도 통과했다. 기본 실행은 384를 포함한 42회 시험이며 4090에서 실행할 예정이다.
초기 preview/final(최소 calibration helper 누락으로 실패)은 진단용으로 보존하고 최종 v2만 전달한다.
이 번들을 검증할 당시에는 GPU 임대·registry 이미지 업로드·번들의 Drive 업로드를 수행하지 않았다.
이후 별도로 진행한 데이터셋 백업 상태는 Vast 계획 문서를 따른다. Vast에 임의의 기본 PyTorch 이미지만
있으면 바로 실행되는 것은 아니며, 팀 이미지에 맞는 컴파일 의존성 환경이 필요하다.

## 5. Vast 4090 실제 검증 — 2026-10-10 17:31~17:34 KST

실제 임대 인스턴스 55165763의 단일 RTX 4090, 표시 VRAM 49140MiB,
torch 2.7.1+cu128 / driver 570.144에서 최종 v2 묶음을 직접 실행했다.
MORAI·학습·다른 GPU 추론과 동시에 측정하지 않았다. JPEG90·batch 1·AMP,
실제 raw 14장 순환·warmup 8회·측정 42회, 범위/격자/depth 계약은 위 로컬 측정과 같다.
새 4-class 모델의 랜덤 초기화 비용 시험이며 E25 정확도·새 학습 정확도 결과가 아니다.

| 입력 | baseline 모델 평균 | cache A 모델 평균 | 분리 B 모델 평균 | B 전처리 평균 | B 서비스 평균 | B 서비스 p95 |
|---|---:|---:|---:|---:|---:|---:|
| 384×896 | 68.54ms | 56.83ms | 48.16ms | 16.22ms | 64.40ms | 65.36ms |
| 448×896 | 72.77ms | 60.02ms | 48.84ms | 16.93ms | 65.79ms | 70.02ms |
| 512×1056 | 84.48ms | 64.78ms | 51.50ms | 18.19ms | 69.71ms | 70.18ms |

서비스는 CPU JPEG decode/resize/pack + model + prediction CPU 복사다. ROS 수신·입력 동기화 대기·
발행·풀스택을 포함하지 않는다. 여기서의 CPU 준비가 로컬보다 느려 GPU 모델 시간만으로
온라인 지연을 판단하면 안 된다. 13Hz≈76.9ms에 대해 448/512 모두 이 제한된 서비스 표본은
들어오지만, 전체 시스템 여유나 512 최종 채택을 보장하지 않는다. 최종 shape는 아직 미확정이다.

CPU 7개(CUDA 전용 3개 제외), CUDA 7개, 비용 9개 case를 통과했다. 전체 모델 동등성은
3개 shape × cache/분리 2개 mode × 실제 14장에서 통과했고, view 최대 절대 차이는
1.17e−5 미만이다. boxes/labels/scores는 지정 허용 오차 내에서 일치했다.

실제 GT의 3-step AdamW/AMP 역전파는 3개 shape × baseline/분리 × batch 1/2의
12개 case에서 모두 통과했다. loss·depth gradient가 유한하고 scaler step skip 없이
depth weight가 실제 변경됐다. 학습 중 기하 cache bypass도 확인했다. 아래 시간은 최초 step을
제외한 나머지 2 step 평균으로, 전체 데이터 로더·val·checkpoint가 포함된 epoch 측정이 아니다.

| 입력 | B batch 1 step | B batch 2 step | B batch 1 정상 할당 | B batch 2 정상 할당 |
|---|---:|---:|---:|---:|
| 384×896 | 186.89ms | 301.98ms | 4365.0MiB | 8130.9MiB |
| 448×896 | 190.20ms | 316.55ms | 4675.1MiB | 8759.7MiB |
| 512×1056 | 208.80ms | 358.12ms | 5725.5MiB | 10857.5MiB |

512 baseline 최초 autotune 단계에서는 할당 peak가 약 45GB였다. 정상 상태 약 10/20GB와
혼동하지 않으며 24GB 카드에서 baseline 초기 실행도 동일하게 통과한다고 보장하지 않는다.
분리 B의 짧은 batch 2 검증은 이 임대 카드에서 통과했다. batch 4/장기 안정성은 미검증이다.
512 smoke 일부 GT는 448 변환본의 부분 집합이다. 512 본학습에는 반드시 전용 infos를 쓴다.

runner 3개 exit 0, 완료 ZIP의 원격·로컬 SHA256이 일치한다.
근거: `artifacts/evaluation/bevfusion/base5_vast_input_pool_261010/SEND_THIS_20261010_083125.zip`,
SHA256 `a196bf2b37f2da832e31bd980fa446240b0e01e4b850852e50ee4ba7b3d8041b`.
ZIP의 시각은 원격 UTC이며 이 절은 KST로 기록한다. 산출물은 Git ignore를 유지한다.
끝난 벤치마크 프로세스는 남아 있지 않고 사용자 `ssh_tmux` 세션은 보존했다.

전체 5K 전송은 Drive API 제한·최신 속도 기준 미달로 보류했다.
[전송 재검사 기록](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#호출-제한-완화-재검사--2026-10-10-17301732-kst)을
따른다. 이번 유한한 역전파는 checkpoint를 저장하지 않으며 원격 저장/재개·외부 백업 검증 완료와 다르다.

남은 순서:

1. 아래 6절이 최신이다. 이전 14장 비용·batch 1/2 역전파 결과만으로 최종 설정을 고르지 않는다.
2. 선택 shape의 infos/초기화 adapter·checkpoint resume/외부 백업 감사.
3. 최종 offline 학습. 최신 mission bag은 dev-val/독립 test로 유지.

## 6. 4090 확대 표본·학습 전 검증 — 2026-10-10 20:23 이후

임대 55165763의 같은 4090 48GB에서 수행했다. E25, 수집 원본, 기존 train/val
분할과 planning/tracker는 변경하지 않았다. **본학습은 시작하지 않았다.**

### 표본과 추론 비용

5K 전체의 실제 ROI 반사점과 GT 거리·방향·패밀리를 읽고 서로 다른 300장을 고정했다.
결정적 해시 무작위 200장은 일반 비용 표본이며, 밀집·근접·원거리·후방·Animal·패밀리
조건을 더한 100장은 스트레스 표본이다. 두 집단의 평균을 분리한다.
같은 14장 반복 횟수를 늘린 시험이 아니다. 새 4클래스 랜덤 가중치이므로 정확도 개선을 뜻하지 않는다.

JPEG90·batch 1·AMP, voxel 0.075m / camera BEV 0.30m / depth 0.50m,
semantic ROI X±100/Y±30m를 유지했다. 아래는 **단계별 계측을 끈** 300장 재측정 중
일반 200장 집계다. 서비스는 CPU decode/resize/pack + 모델 + 결과 CPU 복사다.

| 입력 | 모델 평균 | 서비스 평균 | 서비스 p95 | 서비스 p99 | 76.92ms 초과 |
|---|---:|---:|---:|---:|---:|
| 448×896 | 48.41ms | 65.13ms | 67.56ms | 96.80ms | 8/200 |
| 512×1056 | 51.43ms | 69.96ms | 70.33ms | 91.60ms | 4/200 |

스트레스 100장의 서비스 평균/p95는 448에서 65.72/71.01ms,
512에서 70.02/69.56ms다. 512의 최대 지연은 일반/스트레스에서 175.35/175.65ms였다.
**평균·p95가 예산 이내라는 것과 모든 프레임·ROS 풀스택 13Hz를 보장한다는 것은 다르다.**
입력별 실행 시점이 달라 드문 지연의 차이를 해상도 자체의 안정성 차이로 단정하지 않는다.

앞선 단계별 계측 실행에서는 일반 표본 서비스 p95가 448/512 각각 87.34/89.58ms였다.
이후 계측 없이 다시 확인했으며, 실행 시점·부하도 달라 차이 전부를 계측 오버헤드라고
설명하지 않는다. 단계별 계측 결과와 실제 서비스 비용 선택 근거를 구분한다.
14장 기존 평균과 새 평균은 대체로 비슷하지만, 300장에서 꼬리 지연을 추가 관측했다.

64장(일반 32 + 스트레스 32) × 448/512 × cache/분리의 전체 모델 동등성은 통과했다.
BEV 최대 절대 오차는 2.561×10⁻⁵ 이내이며 decoded prediction도 허용 오차 내에서 일치했다.
처음에는 보충 촬영 token을 부모 `scene_id`로 조회해 3장의 GT를 찾지 못했다.
시험 코드가 실제 `token`을 우선하도록 수정하고 같은 64장을 다시 검증했다.
원본 GT·분할·수집 데이터 오류나 임의 제외가 아니다. 최초 실패 로그도 보존한다.

### 실제 5K 학습 로더 비용

각 후보에 실제 4,255장 train 로더·PNG decode·학습 증강·AMP AdamW를 연결했다.
각 batch case는 80 iteration만 실행하고, warmup 16회를 제외한 64회를 집계했다.
full epoch나 본학습이 아니다. 각 case의 optimizer는 80회 전진했다.

| 입력 | 물리 batch | 로더 포함 step 평균 | 처리량 | 정상 최대 할당 | train 1 epoch 추정 |
|---|---:|---:|---:|---:|---:|
| 448×896 | 1 | 158.49ms | 6.31장/s | 4.60GiB | 11.24분 |
| 448×896 | 2 | 254.38ms | 7.86장/s | 8.58GiB | 9.02분 |
| 448×896 | 4 | 471.42ms | 8.48장/s | 16.58GiB | 8.36분 |
| 448×896 | 8 | 945.74ms | 8.46장/s | 32.57GiB | 8.39분 |
| 512×1056 | 1 | 186.69ms | 5.36장/s | 5.63GiB | 13.24분 |
| 512×1056 | 2 | 316.91ms | 6.31장/s | 10.63GiB | 11.24분 |
| 512×1056 | 4 | 609.57ms | 6.56장/s | 20.64GiB | 10.81분 |
| 512×1056 | 8 | 1187.06ms | 6.74장/s | 40.67GiB | 10.53분 |

512 batch 8은 batch 4보다 처리량 이득이 약 2.7%뿐인데 정상 할당은 거의 두 배다.
**현재 48GB 임대에서는 batch 4를 추천한다.** batch 확대가 정확도 향상을 보장하지 않는다.
메모리 표는 LoggerHook의 CUDA peak 초기화보다 먼저 측정하고 iteration별 최대를 누적한 값이다.
초기 448 batch 1/2/4의 잘못된 peak 집계는 재측정으로 대체했다.
warmup의 cuDNN 탐색에서는 일부 case가 44~45GiB까지 사용했다. 이 결과만으로
24GB 카드의 학습을 보장하지 않는다. 추론 메모리와 학습 메모리도 구분한다.

양쪽 입력 모두 validation 745장 전체를 실제 로더·metric까지 통과했다.
검증 정답 클래스는 448에서 Vehicle/Pedestrian/Obstacle/Animal 341/647/669/33개,
512에서 341/653/674/34개다. 원래 validation 설정인 FP32를 유지했다.
validation 1회의 실측은 448에서 58.17초, 512에서 70.54초다.
4-iteration smoke 가중치를 사용한 **평가 경로 검사**이며 AP를 학습 성능으로 제시하지 않는다.
4,255장의 batch 4 마지막 묶음은 3장이므로, 512 batch 3도 별도 32-iteration 시험으로 통과했다.
마지막 batch 크기의 모델 경로 검사이며 전체 epoch의 마지막 장면 재생이라고 주장하지 않는다.

### 추천과 시간 추정

원거리 픽셀 밀도와 추가 camera-only 단서를 고려해 **512×1056·batch 4·누적 1·worker 4**를
추천한다. 448은 더 빠른 fallback이다. 512의 폭 방향 픽셀 밀도는 448 대비 약 17.9% 높고,
측정된 batch 4 학습 시간은 약 29% 길다. 정확도 이득은 본학습·mission bag으로 검증해야 한다.

초기화는 감사한 nuScenes **camera backbone/neck만** 사용한다. LiDAR/BEV/depth/fusion/head는
새로 초기화하며 E25나 미검증한 square-BEV 가중치를 자동으로 로드하지 않는다.
AdamW LR 1e-4 / weight decay 0.01 / grad clip 35 / AMP 초기 scale 32,
warmup 500 update 후 cosine, 24 epoch는 임시 5K의 **제안 실행안**이다. 최적 epoch의 실증은 아니다.

| 범위·조건 | 448·batch 4 | 512·batch 4 |
|---|---:|---:|
| 현재 5K train+val, 1 epoch | 약 9.3분 | 약 12.0분 |
| 현재 5K train+val, 24 epoch | 약 3.7시간 | 약 4.8시간 |
| 이후 30K, train 25,500/val 4,500 가정, 24 epoch | 약 22.4시간 | 약 28.8시간 |

80-step 비용과 현재 validation 처리율의 외삽이다. 저장·백업·장기 부하·데이터 확장에 따른
오버헤드와 성능 변화는 별도이며, 계획 시간에는 추가 여유가 필요하다.
30K의 실제 split은 후속 공간 group 감사 결과로 결정하며 현재 5K의 resume으로 붙이지 않는다.

### 저장·재개와 남은 시작 조건

448/512 각각 실제 2장·4클래스 모델에서 epoch 1(iter 2) 저장 후 중단하고
epoch 2(iter 4)로 재개했다. optimizer step 2, LR·scheduler·AMP scaler·메인 RNG가 이어졌고
미완료 `.partial` 파일은 없었다. 작은 CUDA 모델의 연속 실행과 중단/재개 가중치는 정확히 일치했다.
이 증명은 epoch 경계까지이며 worker/prefetch·mid-epoch 완전 재현은 주장하지 않는다.

외부 Drive 백업은 **미완료**다. 실제 전체 체크포인트 493,471,045 bytes의 업로드는 실패했다.
20:51경 작은 `about`/25-byte 쓰기 진단에서 원격 `www.googleapis.com` DNS lookup timeout을
확인했다. 공용 DNS 1.1.1.1/8.8.8.8 UDP와 일부 IP 직접 TLS 검사도 timeout이었다.
로컬에서는 같은 전용 OAuth의 Drive 전체 scope, 약 76GiB 여유, 대상 폴더의 쓰기 권한을 확인했다.
API 요금·quota 초과로 단정하지 않는다. DNS뿐 아니라 외부 연결 자체의 복구가 필요할 수 있다.

Direct SSH 8MiB 회수 검사도 25초 제한에 7,716,864 bytes만 수신했다.
Proxy는 SSH banner 교환 timeout이었다. 따라서 큰 체크포인트를 느린 경로로 자동 회수하지 않았다.
체크포인트와 5K는 원격에 보존하며 instance stop/destroy, Cloud Sync나 새 임대는 수행하지 않았다.

`backup_base5_vast_access_only.py`는 갱신 토큰/client secret을 로컬에만 두고 매 poll마다
짧은 접근 토큰을 SSH stdin/원격 anonymous RAM으로 전달하도록 준비했다.
네트워크 preflight는 최대 15초이며 실패하면 큰 업로드를 시작하지 않는다.
로컬 watcher가 살아 있어야 자동 갱신/백업이 가능하다. **실제 업로드·재읽기 SHA 검증과
연속 watcher 검증은 네트워크 복구 후 남은 gate다. 구현됐다는 이유로 성공 판정하지 않는다.**

Git 제외 근거는 `artifacts/evaluation/bevfusion/base5_vast_qualification_261010/`에 있다.
원격의 실행 로그·JSON·config를 로컬에 회수했고 원본 14장 시험과 신규 300장 시험을 구분한다.
추론·역전파·validation·재개·백업 preflight의 결과를 각각 기록하며, 학습 개시 명령은 실행하지 않았다.

offline 학습 준비와 production 배포를 구분한다. 학습 후에는 기존 E25 크기에 고정된
ROS prepared 입력의 새 크기 연결, Animal의 runtime/planning 전달 정책, 새 score threshold,
mission bag과 ROS 풀스택 지연 검증이 남는다. 이번 비용 검증으로 이를 통과했다고 표현하지 않는다.
4. 새 shape ROS raw/prepared 경로와 4-class postprocess/NMS/threshold 연계.
5. detection/tracking/Map-CV 및 판단 전달 지연 회귀 후 production 승격.

이번 작업은 TensorRT/ONNX나 CUDA-BEVFusion 저장소 전체 이주를 수행하지 않았다.
현저한 pooling 개선 이후의 잔여 병목을 4090에서 보고 선택한다.

## 7. 좌우 ±40m 추론 비용 대조 — 2026-10-10 22:08 KST

사용자 요청에 따라 이미지 512×1056, 앞뒤 ±100m, voxel 0.075m, camera BEV 0.30m,
depth 0.50m와 4-class/200 query/AMP/batch 1/분리 pooling을 유지하고 횡방향 범위만 넓혔다.
동일 임대 4090 48GB에서 MORAI·학습 없이 수행했다. 학습된 AP나 ROS 풀스택 측정은 아니다.

새 비용 전용 config는 `bevfusion_base5_dense_512_lateral40_separated_candidate.py`다.
semantic Y ±30→±40m, stride에 정합한 내부 Y ±32.4→±40.8m,
sparse grid Y 864→1088, camera BEV Y 216→272로 변경했다. 내부 격자 수는 약 25.9% 증가한다.
격자를 성기게 하거나 depth bin을 줄이지 않았다. default config와 production은 그대로다.

기존 300개 실제 장면(대표 200 + 스트레스 100)을 같은 순서로 넣었다.
±30→±40→±40→±30의 순서로 각각 300회, warmup 8회 제외 총 1,200회를 측정했다.
장면 수는 300개이며 반복 측정을 새로운 장면으로 세지 않는다. 단계별 hook 계측을 끄고
JPEG90 decode/resize/pack + model + prediction CPU 복사를 포함했다.
공유 random parameter state에서 범위와 무관한 가중치 SHA가 네 실행 모두 같음을 검증했다.

| 대표 200장 × 2회 | 좌우 ±30m | 좌우 ±40m | 변화 |
|---|---:|---:|---:|
| 모델 평균 | 51.13ms | 52.75ms | +1.63ms, 약 3.2% |
| 전처리 평균 | 18.63ms | 18.78ms | +0.16ms |
| 서비스 평균 | 69.77ms | 71.61ms | +1.84ms, 약 2.6% |
| 서비스 p95 | 73.45ms | 76.03ms | +2.58ms |
| 서비스 p99 | 75.58ms | 81.21ms | +5.63ms |
| 서비스 최대 | 81.17ms | 94.64ms | 최악 지연 보장은 아니다 |
| 13Hz 예산 76.92ms 초과 | 2/400, 0.50% | 15/400, 3.75% | 실제 관측 빈도 |
| 정상 최대 할당 | 949.40MiB | 979.38MiB | +29.98MiB |

스트레스 100장 × 2회의 서비스 평균/p95/p99/최대는 ±30m에서
69.91/71.73/72.39/74.35ms, ±40m에서 71.82/76.17/85.50/150.50ms다.
76.92ms 초과는 각각 0/200과 8/200이다. 특정 실행의 시간적 부하·CPU 전처리 변동과
희귀 outlier를 범위 확대의 순수 GPU 비용으로 모두 귀속하지 않는다.
메모리 수치는 PyTorch allocation이며 전체 VRAM, 학습 메모리와 다르다.

평균 추론 비용만으로 ±40m를 배제할 근거는 약하다. 다만 대표 p95의 13Hz 여유는
약 0.90ms에 불과하므로 ROS·동기화·발행·풀스택까지 실시간성을 인증한 것은 아니다.
이미지 backbone과 depth 입력 크기는 그대로라는 구조는 작은 평균 증가와 부합하지만,
이번에는 단계별 계측을 껐으므로 추가 시간을 각 layer에 배분해 단정하지 않는다.
±40m를 학습 계약으로 채택하려면 native GT의 신규 횡방향 영역·가시성 재감사와 파생 infos,
학습 비용 검증이 별도 필요하다. 기존 ±30m 파생 라벨을 그대로 확대 학습하지 않는다.

### 실행 중 진단과 검증

- 첫 실행은 원격 repo의 `configs/sensors/Sensor.json` 부재로 추론 전에 실패했다.
  기존 시험 payload의 동일 보정 파일을 연결했다. 로컬과 원격 파일 SHA는
  `35615d2a3c0c65f833412f0bc6da387d745abe1abcb49441eea815c790ee45dd`로 같다.
- upstream의 `dx/bx/nx/frustum`은 buffer가 아니라 비학습 `nn.Parameter`다.
  최초 전체 parameter hash 차이는 ROI 메타 변화와 섞여 있었다.
  이를 가중치로 복사한 보조 시험은 camera/LiDAR Y 108/136 불일치로 실패했다.
  해당 로그를 보존하고 결과에서 제외했다. 공유·해시에서 이 네 기하 항목을 제외하고,
  config 기준 기하를 복원한 `controlled_v2` 네 실행만 최종 수치로 사용했다.
- 각 모델의 `nx`는 ±30m `[672,216,1]`, ±40m `[672,272,1]`로 확인했다.
  네 실행 모두 해당 범위의 원본 pooling 대비 view/prediction 일치 검사를 통과했다.
- 비교기 신규 단위 검사 4개와 기존 표본·통계 검사 3개, Python compile 및 diff 공백 검사를 통과했다.
  서로 다른 가중치, 격자 간격 변화, 잘못 복사된 이전 기하를 비교기가 차단한다.

로컬 근거는 Git에서 제외한 `artifacts/evaluation/bevfusion/base5_lateral40_cost_261010/`이다.
요약은 `summary.json`, 실제 사용한 네 결과는 추출 폴더의 `controlled_v2/`에 있다.
가중치를 제외한 원격 로그·JSON archive SHA는
`df0c628b1fb6f82ccc5dc686b929434753539f18c8d956b080172a11b5b806e6`다.
범위 외 기하 parameter를 제외한 공통 가중치 SHA는
`5cb485a60d6d7111265004ea9120b876d03444aff6858c59fe27dfb9ee4e1117`이다.
native portable manifest SHA는
`2e03c54c3db6c56168f985e88b45b5e1e23abae7c166f2a5ecdfdfbfd2284753`이다.
GPU 작업은 종료했다. instance stop/destroy나 본학습 실행은 하지 않았다.

재현 시 기존 출력 파일을 덮어쓰지 않고 새 output/state 경로를 사용한다.
아래 명령을 같은 shared state로 `--lateral-range 30/40/40/30` 순서로 실행한다.

```bash
python -u perception/detection3d/bevfusion/scripts/benchmark_base5_input_optimization.py \
  --root /root/AjouNice2026 \
  --samples /root/base5_qualification_261010_2023/samples \
  --output /root/base5_lateral40_cost_261010/controlled_v2/r2_y40.json \
  --iterations 300 --shapes 512x1056 --modes separated \
  --codec jpeg90 --no-stages --skip-preprocess --lateral-range 40 \
  --shared-parameter-state /root/base5_lateral40_cost_261010/controlled_v2/random_parameters.pth
```

<a id="lateral-range-decision-261010"></a>

## 8. 좌우 ±30m 유지·±40m 확대 기각 — 2026-10-10 22:43 KST

### 결정과 적용 범위

사용자와 미션별 필요성을 검토한 뒤 이번 Base5 학습의 semantic ROI를
**X ±100m / Y ±30m / Z −3~2m**로 유지한다. 모델 기준은 Ego에 고정된 LiDAR 좌표계이며,
도로 접선 좌표계로 입력이나 GT를 회전하는 변경은 하지 않는다.
stride 정합용 내부 범위 X ±100.8m / Y ±32.4m와 semantic ROI를 혼동하지 않는다.
이미지 512×1056·batch 4 추천은 유지한다. production E25의 기존 범위·threshold는 바꾸지 않는다.

±40m 비용 config와 7절의 측정 결과는 비교·재현용으로 보존하되 본학습 후보로 채택하지 않는다.
이번 결정은 문서의 실행 기준을 확정한 것이며 코드/config/파생 infos나 수집 원본을 수정한 것이 아니다.
Vast 인스턴스는 사용자가 중단했다고 보고했으며 이번 로컬 조사에서 접속·재개·GPU 실행을 하지 않았다.

### 기존 Sunny13 5K 원본 감사

crop 분석에 동결한 `source_ledger_snapshot.jsonl`을 전수 확인했다.
원본 ledger SHA는 위의 `026e6244…df74a43`이며, 분석 입력 경로는
`artifacts/evaluation/bevfusion/base5_crop_final5k_261010/source_ledger_snapshot.jsonl`이다.

- native annotation component는 11,372개이며 |X|≤100m인 것은 11,371개다.
- 이 11,371개 중 중심이 30<|Y|≤40m 또는 |Y|>40m인 것은 각각 0개다.
- 중심은 |Y|≤30m이지만 회전된 BEV footprint 일부가 경계를 넘는 것은 16개다.
  XY footprint는 `(|sin(yaw)|·length + |cos(yaw)|·width)/2`의 횡 반폭으로 계산했다.
  이는 중심 기반 GT가 모두 삭제된다는 뜻이 아니라 BBox/입력 일부가 잘릴 수 있다는 뜻이다.
- 위 수치는 가시성 선별 전 component 집계다. 512 변환본의 최종 GT 11,052개와 다른 모집단이다.
  audit가 ROI 밖 항목을 미리 숨긴 결과는 아니지만, 생성기가 ±30m를 고려해 배치했으므로
  0개라는 결과만으로 실대회 ±30m 충분성을 증명하지 않는다.

### 실제 개발 bag의 범위 밖 관측

`260923_teammate_manual`의 H01/R01 **development** bag만 읽었다.
평가 잠금 bag·모델 추론·ROS master는 사용하지 않았다. 기존 catalog의 BBox 중심 변환과
MR heading 보정 −1.347°를 적용하고, Object 시각으로 Ego를 보간했다.
LiDAR의 Ego 기준 X offset 0.855m를 반영해 XY 반경 100m와 |X|≤100m 안에서 집계했다.
각 객체 궤적을 약 0.5초 간격으로 표본화한 반복 관측이며 고유 차량 수·프레임 누락률이 아니다.

| 개발 bag | 기하학적 차량 관측 | 측방 30~40m 관측 | 확인된 위치와 해석 |
|---|---:|---:|---|
| H01 |209|4, 약 1.9%|X 약 76~93m, Y 약 −31~−37m다. 전부 **합류 이전 A2256W000846 진입로**에서 보였으며 합류 왼쪽 차량 사례가 아니다.|
| R01 |132|6, 약 4.5%|Y 약 +30~+35m 차량이다. 이 수치만으로 진입에 필요한 차량이 누락됐다고 판단하지 않는다.|

H01의 geometry 미식별 segment 2개는 제외했다. H01에서 Ego가 합류 차로
`A2256W000411`에 있었던 유효 BBox 중심 관측 68개는 모두 ±30m 안이었다.
카메라/LiDAR 가시성·진로 충돌 가능성·planning 소비 여부는 이 집계에서 검증하지 않았다.
따라서 위 비율을 실제 검출 누락률이나 위험 차량 비율로 사용하지 않는다.

### 고주로 합류 경로 확인

샘플 NPC spawn 6/8/10의 실제 start/end로 잘린 MGeo 경로와 Ego 합류 차로
`A2256W000411`을 비교했다. 약 167.41m의 Ego 차로를 양 끝 포함 169개 위치,
약 1m 간격으로 검사했다. 반경 100m 안 NPC 경로 점의 측방 범위다.
차량 기준점과 도로 기하 계산이며 BBox 전체·가시성·안전한 합류를 보장하지 않는다.

| Ego yaw 조건 | NPC 기준점 Y 최소~최대 | ±30m 초과 |
|---|---:|---|
| 도로 접선 그대로 |−5.39~+15.60m|없음|
| 접선 대비 −3/0/+3° |−10.59~+20.73m|없음|
| 접선 대비 −6/0/+6° |−15.75~+25.79m|없음|
| 접선 대비 −10/0/+10° |−22.57~+32.44m|일부 있음|
| 접선 대비 −15/0/+15° |−30.93~+40.53m|일부 있음|

앞서 H01의 범위 밖 4관측을 일반적인 합류 왼쪽 차량의 약점처럼 설명한 해석은 수정한다.
통상적인 합류 경로의 왼쪽 차량을 위해 ±40m가 필수라는 근거는 확인하지 못했다.
큰 Ego yaw나 합류 이전 곡선에서 먼 본선 차량을 조기에 보는 여유는 ±30m 선택의 한계로 남긴다.

### 로터리 접근·진입 경로 확인

Ego의 기존 미션 경로와 샘플 spawn 4의 약 92.81m 열린 NPC 경로를 비교했다.
아래 arc는 기존 전체 Ego 경로의 누적 거리이며 횡거리나 로터리 반지름이 아니다.
접선 yaw·기준점 계산이므로 실제 Ego yaw/차량 크기가 변하면 결과도 달라질 수 있다.

| Ego 경로 arc | 단계의 대표 위치 | NPC 전체 경로 Y 최소~최대 |
|---|---|---:|
|870m|접근|−16.51~+19.52m|
|880m|접근|−15.13~+20.92m|
|885m|진입 대기|−14.31~+21.75m|
|887m|진입 직전|−8.80~+27.18m|
|890m|진입 중 방향 전환|−4.66~+30.88m|
|900m|원형 구간|+0.08~+35.28m|

접근·대기 대표 위치에서는 NPC 경로 전체가 ±30m 안이었다. 900m 위치에서 범위 밖인
NPC 경로 점은 Ego의 NPC 경로 투영점보다 경로를 따라 약 40.79~58.04m 뒤쪽이었다.
이는 바로 앞뒤에 밀집한 차량을 놓친다는 증거가 아니다. 단, 대표 위치 조사만으로
로터리 전 구간·본선 속도·큰 yaw에서 모든 위험 차량이 포함된다고 보장하지 않는다.

### 기각 이유·남는 한계·재검토 조건

- **주된 이유는 미션 필요성과 학습 우선순위다.** 일반적인 고주로 합류·로터리 대기에서
  추가 횡방향 영역의 필요성이 입증되지 않았다. 근접 통과·전후방 원거리·로터리 밀집
  객체의 인지와 tracking 안정성에 데이터와 검증 시간을 우선 사용한다.
- 기존 5K에는 신규 띠의 중심 GT가 없다. ±40m config만으로 해당 영역의 정확도가
  좋아진다고 보장할 수 없으며, 채택하려면 추가 분포·GT·학습 비용 검증이 필요하다.
  한정된 데이터 수에서 신규 띠를 별도 quota로 채우면 핵심 사례의 예산과 경쟁한다.
  **범위 확대 자체가 반드시 학습 성능을 떨어뜨린다는 실험 결과는 아니다.**
- **느려서 불가능하다는 이유로 기각한 것은 아니다.** 서비스 평균 증가 +1.84ms는 작았으나
  p95·13Hz 예산 초과 관측이 늘었으며 ±40m 학습 시간과 정확도는 측정하지 않았다.
- ±30m는 곡선 위 원거리 차량, 합류 이전 진입로, 큰 Ego yaw에서 첫 관측을 늦출 수 있다.
  기존 yaw stress의 차량 2대는 Y 약 −33.63/+34.54m에서 LiDAR 5점씩 있었지만
  ±30m 필터 뒤 0점이었다. 해당 [이력](../../dataset/morai_od/docs/BASE5_V2_HISTORY_THROUGH_V100.md)은
  보존하며, 이를 불가능한 검출이나 전체 미션 누락률로 해석하지 않는다.
- 기존 5K와 나머지 환경의 전후방 ±100m·좌우 ±30m 계약을 유지한다.
  확대를 위해 도로 밖 30~40m에 일반 객체를 임의 추가하거나 기존 5K를 재수집하지 않는다.
- 재검토는 **센서에 보이고 실제 Ego 진로와 관련된 차량이 Y 범위 때문에 잘려,
  판단에 필요한 시점까지 첫 검출·속도 추정이 늦는 사례**가 개발 주행에서 확인될 때 한다.
  객체의 현재 Y만 보지 않고 경로상 충돌 지점까지 시간·ROI 진입 시점·판단 여유를 확인한다.
  확인 시 해당 경로의 표적 보강과 범위 확대를 함께 비교한다. 새 모델의 원거리 정확도가
  낮을 것이라는 추측이나 단순 범위 밖 객체 수만으로 재검토를 막지 않는다.

참고: [BEVDet BEVPoolv2 Python](https://github.com/HuangJunJie2017/BEVDet/blob/dev3.0/mmdet3d/ops/bev_pool_v2/bev_pool.py),
[CUDA interval reduction](https://github.com/HuangJunJie2017/BEVDet/blob/dev3.0/mmdet3d/ops/bev_pool_v2/src/bev_pool_cuda.cu).

<a id="image-domain-decision-261010"></a>

## 9. JPEG·blur·노출·noise 증강 결정 — 2026-10-10 23:05 KST

이 절은 당시 80% 선택의 근거다. 현재 확률 100%와 저장 방식은 아래 10절을 따른다.

### 결정과 근거의 범위

SaveSensorData PNG와 실제 주행 JPEG 사이에 **관측된 압축 차이만 학습에서 재현**한다.
개발용 bag을 사용했으며 잠긴 test bag은 사용하지 않았다. 원본 데이터·GT·split·가시성
정책·생산 모델은 변경하지 않는다. 이번 검사는 CPU 이미지 분석과 실제 로더 검증이며,
새 모델의 AP 향상이나 증강 확률의 최적값을 확인한 실험은 아니다.

| 증강 | 최종 학습 설정 | 판단 이유 |
|---|---|---|
| JPEG | ON, 카메라별 확률 0.8, Q90 고정 | 실제 bag의 양자화 표·색차 서브샘플링과 맞춘다. 20%는 원본 PNG 단서를 유지한다. |
| Blur | OFF | 실제 물리 blur의 차이는 입증되지 않았다. 임의 Gaussian blur로 작은·먼 객체의 경계 단서를 약화시키지 않는다. |
| 노출 gain/bias | OFF | 다른 장면의 밝기 통계만으로 노출 오차라고 단정할 수 없다. 이후 실제 6개 환경의 조명·안개 분포를 우선한다. |
| Gaussian noise | OFF | 임의 센서 noise를 추가해야 할 근거가 없다. JPEG 압축 차이를 noise로 대체하지 않는다. |

80%는 clean/JPEG를 섞는 보수적인 설계값이다. Q90이 관측됐다는 이유만으로 확률 80%가
최적이라는 뜻은 아니다. 전체 장면의 20%가 clean이라는 뜻도 아니며 **각 카메라에 독립 적용**한다.
고정 Q90을 사용하고 근거 없이 Q72~96처럼 압축 품질 범위를 넓히지 않는다. 실제 fog를
gain이나 blur로 대신하지 않는다. 기존 이미지 기하·BEV 회전/scale/translation 증강은 유지한다.

### 표본과 측정

- 기존 raw sensor bag 7개와 09-23 팀원 bag 5개를 포함한 12개를 읽었다. 고주로·로터리·
  보행자·장애물과 전체 저속 주행을 포함하며 각 bag·카메라에서 시간/메시지 순서에 걸쳐
  균등하게 32장을 선택했다. **4카메라 × 32장 × 12개 = 1,536장**이다.
- 1,536장 모두 JPEG였고, 양자화 표가 OpenCV Q90 기준과 정확히 일치했다.
  색차 서브샘플링은 모두 4:2:0이었다. 파일 크기에서 품질을 추측한 것이 아니다.
  이는 해당 표본의 특성이며 MORAI 모든 버전·설정의 공통 보장은 아니다.
- canonical 5K의 고정 ledger에서 7개 패밀리별 24장면을 해시로 선택했다.
  **168장면 × 4카메라 = 672장** 모두 PNG였다. 전후방 1280×720과 측방 640×480의
  원본 해상도를 사용했다. ledger SHA256은 문서 상단의 기존 5K와 같다.
- `audit_base5_image_domain.py`는 bag 이름·크기·수정 시각, 표본 메시지 인덱스·시각·payload
  SHA256, PNG 경로·SHA256을 기록한다. 전체 bag 파일의 SHA를 새로 계산한 것은 아니다.
  `evaluation_locked` 경로는 읽기 전에 거부한다.

같은 native PNG에 변환을 적용하고 512×1056 아래 정렬 입력으로 비교했다.
아래 값은 672개 이미지의 중앙값이며 MAE는 8-bit 픽셀 값의 평균 절대 차이다.
Laplacian variance 비율은 아래쪽 절반의 경계 에너지 proxy이지 탐지 정확도나 객체 가시율이 아니다.

| 비교 | 픽셀 MAE p50 | PSNR p50 | 경계 에너지 비율 p50 |
|---|---:|---:|---:|
| 원본 해상도에서 JPEG Q90 후 resize/crop | 1.48 | 39.93dB | 1.00 |
| resize/crop 후 JPEG Q90 — 이전 배치 순서 | 1.38 | 40.68dB | 1.02 |
| native Gaussian 3×3 후 resize/crop | 1.94 | 33.49dB | 0.18 |
| native gain ×1.03 후 resize/crop | 2.58 | 37.96dB | 1.06 |
| native noise σ=1 후 resize/crop | 0.81 | 47.37dB | 1.02 |

약한 JPEG와 달리 3×3 blur는 이 경계 지표를 크게 낮췄다. 이 결과만으로 motion blur가
없다거나 blur 증강이 반드시 AP를 낮춘다고 단정하지 않는다. native와 bag은 **같은 위치·
객체·날씨를 맞춘 쌍이 아니므로** 밝기·경계 통계의 차이를 센서 물리 효과로 해석하지 않는다.
확인되지 않은 교란을 추가하기보다 직접 확인한 압축 차이를 우선 반영한다.

### 구현·검증과 비용

- 선택 config: `perception/detection3d/bevfusion/configs/bevfusion_base5_dense_512_jpeg90_train.py`.
  기존 `bevfusion_base5_dense_512_separated_candidate.py`는 all-OFF 비용 기준으로 유지한다.
  새 config는 학습 pipeline만 덮어쓰며 512×1056·±100×30m·4-class·7D·분리 pooling은 동일하다.
- `MoraiOnlineImageDomainAug3D`를 `MoraiImageAug3D` **앞**에 한 번만 배치한다.
  원본 카메라 크기로 padding을 제외한 뒤 RGB→BGR→JPEG→RGB를 수행한다. 이후 기존
  OpenCV resize/crop을 사용한다. 검은 padding을 압축에 섞거나 GT/투영 행렬을 바꾸지 않는다.
  과거 실험 인터페이스의 기본값은 유지하고 새 config가 RGB/native 모드를 명시한다.
- UDP 수신기는 이미 JPEG packet을 `CompressedImage`로 전달하며 다시 압축하지 않는다.
  이번 수정은 학습 입력에만 적용된다. 추론에 JPEG를 한 번 더 넣거나 production E25를 바꾸지 않는다.
- CPU 단위 검사 **19개 통과**: JPEG/RGB/native padding/확률·seed/config 9개,
  직사각형 geometry 회귀 6개, audit/locked bag 차단 4개다.
- 실제 train 24장면·val 24장면의 paired CPU pipeline에서 GT 112개와 LiDAR·보정 행렬이
  정확히 일치했다. train 카메라 96개 중 76개에 JPEG가 적용됐고 val 카메라 96개는 변화가 없다.
  비교에서만 BEV 증강·point shuffle을 고정해 이미지 변화와 기하 RNG 차이를 분리했다.
- 별도로 **정상 BEV 랜덤 증강을 켠** batch 4·worker 4 실제 train loader를 8 batch,
  32장면 실행했다. 이미지 shape·픽셀·GT·행렬의 유한값 검사를 통과했다.
  최초 로컬 Docker 기본 공유 메모리에서는 worker bus error가 났고, `--shm-size=2g`로
  재검증해 통과했다. 이 오류는 JPEG나 원본 손상으로 판정하지 않는다. 기존 Vast의 공유
  메모리 약 47GB와 로컬 기본 Docker 제한을 혼동하지 않는다.
- 호스트 OpenCV 4.2·단일 CPU thread에서 JPEG encode/decode는 이미지당 평균 4.19ms였다.
  4카메라×확률 0.8로 계산하면 **장면당 codec만 약 13.4ms CPU 작업**이다. padding 복사·
  로더 전체 비용은 별도이며 worker/prefetch와 GPU가 겹치므로 학습 step에 13.4ms가 그대로
  더해진다고 계산하지 않는다. **추론 경로의 추가 비용은 없으며 새 학습 시간은 미측정**이다.

근거 JSON은 ignored 경로 `artifacts/evaluation/bevfusion/base5_image_domain_261010/`의
`audit.json`, `pipeline.json`, `loader.json`에 보존했다. 원본 PNG를 JPEG로 덮어쓰지 않는다.
val/test는 원본 pipeline을 유지하며 실제 JPEG bag에서 후속 dev-val을 수행한다.

사용자가 Vast 재개를 요청하면 새 코드/config를 전달하고 **새 준비 경로·해시 계약**으로
설정을 확정한다. 기존 all-OFF 설정으로 생성한 학습 준비 계약을 그대로 resume하지 않는다.
6절의 약 4.8시간은 JPEG 학습 증강 이전 비용으로, 짧은 실제 학습 step에서 worker 병목을
확인한 뒤 갱신한다. 외부 체크포인트 백업 gate도 아직 남아 있다. 이번에는 원격 접속·
GPU 실행·본학습·추론 AP 평가를 수행하지 않았다.

기술 참고: [OpenCV JPEG encode 옵션](https://docs.opencv.org/4.x/d8/d6a/group__imgcodecs__flags.html),
[Pillow JPEG quantization·subsampling](https://pillow.readthedocs.io/en/stable/handbook/image-file-formats.html#jpeg).

<a id="jpeg-cache-decision-261010"></a>

## 10. Q90 100%와 사전 생성 캐시 선택 — 2026-10-10 23:55 KST

### 결정과 비교 범위

사용자 요청으로 Q90을 네 카메라 모두 100% 적용한다. **별도 native JPEG 캐시를 학습
머신에서 한 번 만드는 경로를 추천**하고 PNG→Q90→decode를 매번 수행하는 경로를 대안으로
유지한다. 캐시 선택은 다른 모델이나 압축 정책이 아니라 같은 입력을 준비하는 방법의 차이다.
학습 기회가 제한된 상황에서 저장 방식 때문에 두 모델을 본학습할 필요는 없다.
Q90 자체의 AP 개선은 별도 rosbag 평가 전까지 미검증이다.

로컬 team-cu128 이미지의 OpenCV 4.11.0으로 실제 train의 7개 패밀리에서 256장면을
hash 순서로 균등 추출했다. 패밀리별 36~37장이므로 모집단 비율을 그대로 반영한 표본은 아니다.
모델·GPU·원격 접속 없이 실제 MMDetection3D 로딩, resize/crop, 정규화, GT 변환과
batch packing을 측정했다. JPEG는 원본 PNG의 RGB를 명시적으로 BGR로 바꿔 Q90으로
한 번 encode하며 원본 크기를 유지한다. resize/crop 이전에 적용하고 재압축하지 않는다.

### 동등성과 구현 검증

- 256장면·1,024개 카메라의 최종 입력 픽셀이 정확히 같았다. GT 483개
  (Vehicle 238/Pedestrian 116/Obstacle 117/Animal 12), LiDAR와 투영·증강 행렬도 정확히 같았다.
- 실제 등록된 캐시 transform과 실행 config로 train 32장면·val 32장면을 별도로 검사했다.
  train 픽셀 동등성, GT 143개·LiDAR·투영 동등성, val 입력 불변을 통과했다.
- paired 검사에서만 BEV 증강과 PointShuffle을 고정했다. 비용 검사에서는 정상 train 증강을
  유지했다. JPEG transform 제거로 난수 소비 순서가 달라지므로 동일 seed의 전체 학습 궤적까지
  같다고 주장하지 않는다. 증강 분포는 유지하며 기존 준비 계약을 그대로 resume하지 않는다.
- CPU 단위 검사 27개를 통과했다. 캐시 누락·손상·다른 원본 hash·해상도 불일치와 이중 압축을
  거부한다. 캐시 생성은 fresh 경로만 허용하고 완료 manifest 이전에 실패한 캐시는 사용하지 않는다.
- `prepare_base5_jpeg_cache.py`가 train 전체 카메라를 생성·검증하고, direct runner의
  `--jpeg-cache`가 train에만 연결한다. val/test는 원본 pipeline을 유지한다.
  cache manifest SHA256은 실행·resume 계약에 포함하며 `--run`에서 실제 파일 hash도 확인한다.

### 로더 비용

batch 4, 두 실행 순서를 뒤집어 반복했고 각 경로에서 첫 epoch를 warmup으로 제외한
두 epoch를 측정했다. OS 파일 캐시는 warm 상태이며 cold disk·네트워크 성능 시험이 아니다.
아래 CPU 단독 수치는 동일한 캐시 파일 연결 원리를 쓰는 benchmark adapter로 측정했으며,
최종 등록 transform의 동등성은 위 64장면 검사로 따로 확인했다.

| CPU 로더 조건 | 매번 PNG→Q90 | 사전 JPEG 캐시 | 해석 |
|---|---:|---:|---|
| worker 0, 준비 처리량 | 10.77장/s | 30.42장/s | CPU 준비 약 2.82배 |
| worker 4, 준비 처리량 | 29.27장/s | 58.64장/s | CPU 준비 약 2.00배 |
| worker 4, batch 수신 대기 | 136.71ms | 68.20ms | GPU 대기 없이 최대한 소비한 조건 |
| worker 4, 소비 대기 모사 시 처리량 | 6.02장/s | 6.14장/s | 실제 CUDA 학습이 아닌 CPU 모사 |
| 같은 모사의 정상 공급 중 수신 대기 | 15.64ms | 15.92ms | 초반 두 batch 제외, 개선 확인 안 됨 |

모사는 과거 4090 batch 4 학습 step 609.57ms를 단순 sleep으로 넣어 prefetch 겹침을
관찰했다. 초기 준비 대기를 포함한 처리량은 캐시가 약 2% 높았지만, 정상 공급 중에는
두 방식 모두 충분히 따라갔다. **CPU 준비 두 배 개선을 전체 학습 두 배 개선으로 환산하지
않는다.** 캐시의 주된 이점은 반복 PNG decode/encode와 읽기 용량·CPU 사용을 줄이는 것이다.
실제 4090 전체 학습 시간은 이번에 다시 측정하지 않았으며 기존 약 4.8시간 추정을 바꾸지 않는다.

### 생성 시간과 추가 용량

| 항목 | 측정 또는 추정 |
|---|---|
| 표본 캐시 생성 | 256장면·1,024개 이미지, 13.52초 실측 |
| 표본 이미지 용량 | 원본 PNG 915,924,530 bytes → JPEG 83,703,564 bytes |
| 전체 train 원본 이미지 | 4,255장면·17,020개 이미지, 14.13GiB 실제 manifest 합계 |
| 전체 train JPEG 캐시 | 약 1.28GiB, 패밀리별 표본 크기로 가중 추정 |
| 전체 train 생성 시간 | 로컬 약 225초(약 4분), 표본 시간을 선형 외삽 |

원본 PNG를 보존하므로 **디스크를 줄이는 작업이 아니라 약 1.28GiB를 추가하는 작업**이다.
전체 train 캐시는 아직 생성하지 않았다. 시간은 Vast 실측이 아니고 다른 날씨 25K의 용량도
보장하지 않는다. 원격에 이미 있는 PNG에서 생성할 수 있어 별도 대용량 업로드는 필요 없다.
native 해상도를 유지하므로 이후 입력 crop 크기를 바꿔도 캐시를 다시 만들 필요는 없다.

측정 파일은 ignored 경로 `artifacts/evaluation/bevfusion/base5_jpeg_cache_261010/report.json`,
`base5_jpeg_cache_261010_steady/report.json`, `base5_jpeg_cache_261010/production_pipeline.json`에
보존했다. 첫 report는 CPU 단독 및 초기 모사, 두 번째는 정상 공급 대기를 분리한 반복 측정이다.
본학습·Vast 재개·GPU·원본 수정·production 변경은 하지 않았다.
다음은 사용자 요청으로 Vast를 재개한 뒤 소스 전달 → 원격 train 캐시 생성·검증 →
새 준비 계약 확정 → 외부 checkpoint 백업 gate 확인 → 승인 후 본학습이다.

<a id="jpeg-full-build-261011"></a>

## 11. 전체 JPEG 생성과 PNG 없는 train 전달본 — 2026-10-11 초기 기록

아래 train JPEG / val PNG는 당시 검증 이력이다. 이후 사용자 승인으로 12절의 all-JPEG
계약으로 변경했다. 수치와 원본은 보존하지만 아래 mixed 전달본을 현재 기본값으로 사용하지 않는다.

사용자 요청으로 로컬 Sunny13 5K의 native 카메라 이미지 **20,000개를 모두 Q90 JPEG로
생성하고 전수 SHA256 검증**했다. 원본 PNG·GT·split은 보존했다. val JPEG도 참고용으로
생성했지만 현재 학습/검증 계약은 **train JPEG / val PNG**로 유지한다. production E25는 불변이다.

| 산출물 | 수량·용량·시간 |
|---|---|
| 전체 native JPEG 캐시 | 5,000장면·20,000개 이미지, 1,619,725,032 bytes(1.508GiB) |
| 대응 원본 PNG | 17,827,527,772 bytes(16.603GiB), 보존 |
| 실제 변환 시간 | 269.72초(약 4.5분), 파일 검증·전달본 포장 시간은 별도 |
| 실제 train JPEG | 4,255장면·17,020개 이미지, 1,374,346,013 bytes(1.280GiB) |
| 독립 전달본 전체 payload | 12,838,551,616 bytes(11.957GiB), 현재 manifest 파일 자체는 별도 |

로컬 ignored 경로는 다음과 같다.

```text
# 전체 5K의 참고용 native JPEG, 원본 PNG를 덮어쓰지 않음
artifacts/datasets/base5_sunny13_native_jpeg90_261011/

# 독립 전달·학습용: train JPEG / val PNG / LiDAR / native annotation / metadata
artifacts/handoff/base5_sunny13_jpeg90_train_portable_261011/
```

`prepare_base5_jpeg_portable.py`가 원본 portable과 검증된 캐시에서 새 전달본을 만든다.
train infos의 이미지 경로만 상대경로 JPEG로 바꾸고 train PNG는 전달본에서 제외한다.
LiDAR·3D GT·보정·native annotation·provenance는 보존하며 val/test infos는 byte 그대로다.
현재 두 crop profile과 train 4,255 / val 745 분할도 유지한다.
원본 전달본 payload 24.765GiB에서 새 전달본 11.957GiB로 줄었지만 JPEG 이미지 1.280GiB만
보내서는 학습할 수 없다. **LiDAR·라벨·보정·메타데이터와 val PNG도 함께 전달해야 한다.**

새 manifest의 `image_policy`를 실행기가 확인해 Q90 재변환을 제거한다. 추가 `--jpeg-cache`는
사용하지 않으며, 원본 PNG 폴더를 옮겨도 새 전달본 자체만으로 준비·학습할 수 있다.
단위 검사에서는 원본/캐시 폴더를 숨기고 새 전달본을 다시 옮긴 후 검증과 실제 실행 준비를
통과했다. 자동 fallback이나 기존 PNG manifest 덮어쓰기는 하지 않는다.
로컬 전달본은 용량 중복을 줄이려고 hardlink를 사용했으므로 immutable로 취급한다.
수정이 필요하면 새 파생본으로 만들며 모델 체크포인트를 데이터 폴더에 쓰지 않는다.

Vast에는 아직 새 소스·JPEG 전달본을 전송하지 않았다. 기존 Vast에는 PNG가 이미 있으므로
재개 후 캐시를 원격에서 생성하는 경로가 가장 간단하다. 향후 새 환경 데이터는 로컬에서
JPEG 전달본을 먼저 만들면 원격으로 train PNG를 보낼 필요가 없다.
Vast 재개·본학습은 여전히 사용자 요청과 외부 checkpoint 백업 gate 확인 뒤에 진행한다.

### 원본 PNG vs 사전 JPEG의 아주 짧은 실제 학습 비용

이번에는 10절의 '매번 PNG→JPEG 변환' 비교가 아니라 **JPEG 증강 없는 원본 PNG vs
사전 생성 Q90 JPEG**를 직접 비교했다. 로컬 RTX 5060 Ti에서 512×1056·batch 2·worker 4,
분리 pooling·AMP AdamW·동일 camera-only 초기 가중치와 seed로 수행했다.
PNG→JPEG→JPEG→PNG 순서로 네 번 새로 시작해 각 32 step 중 처음 8 step을 제외했다.
각 조건은 48개의 측정 step이며 같은 64장면을 반복했다. scene 순서와 증강 후 GT·BEV
행렬 hash가 네 실행에서 정확히 같았다. 픽셀은 JPEG 손실 압축으로 의도적으로 다르다.
GPU 비용 측정 중 CPU 정합 검사는 일시 대기시켜 부하가 겹치지 않도록 했다.

| batch 2 측정 평균 | 원본 PNG | 사전 Q90 JPEG |
|---|---:|---:|
| 데이터 수신·반복 관리 대기 | 11.08ms | 10.47ms |
| 학습 step: 전처리·forward/backward·optimizer | 740.67ms | 737.75ms |
| 위 비용을 포함한 반복 전체 | 751.75ms | 748.22ms |

반복 전체 차이는 **약 0.47%**로, 이 짧은 시험에서는 사실상 같은 수준이다.
GPU에 전달되는 이미지 텐서 크기가 같고 PNG도 이미 충분히 공급돼 JPEG 저장 용량 감소가
큰 학습 가속으로 이어지지 않았다. 약 3ms의 학습 step 차이를 JPEG가 GPU 연산량을
줄인 근거로 해석하지 않는다. 단기 반복 편차와 비결정적 CUDA 연산도 포함한 비용이다.
모든 loss는 유한했고 각 실행에서 optimizer가 32회 정상 진행했다. peak allocation은
두 경로 모두 약 10.63GiB였다. 모델 수렴·AP·전체 epoch·validation·checkpoint 비용은
측정하지 않았으며 이 수치로 기존 4090 약 4.8시간 추정을 갱신하지 않는다.

반복 가능한 짧은 실행기는 `run_base5_png_jpeg_cost_probe.py`다. `--png-data`,
`--jpeg-data`, `--init-checkpoint`, 새 `--output`을 지정하며 기본은 batch 2·32 step이다.
4090 재개 후 필요하면 같은 도구에 `--batch 4`를 명시해 다시 측정할 수 있다.
현재 결과는 `artifacts/evaluation/bevfusion/base5_png_vs_jpeg_train_261011/report.json`과
네 실행의 `cost.json`·log·resolved config에 보존했다. 비교용 가중치는 저장하지 않았다.
JPEG 선택의 주된 이득은 **전송·저장·CPU 준비 비용 감소와 실제 JPEG 입력 분포에 맞추기**다.

### 추가 정합·무결성 근거

- 원본 portable 전수 SHA256, 생성 JPEG 20,000개 전수 SHA256, 독립 전달본 173,292개
  payload 전수 SHA256을 통과했다. 원본과 새 manifest를 각각 provenance로 보존했다.
- 두 crop profile의 전체 10,000개 record를 재귀 비교해 **train 이미지 경로 외의 필드가
  모두 정확히 같음**을 확인했다. 원본 3D GT·calibration·LiDAR·split 변경은 없다.
- 독립 전달본 manifest SHA256은
  `6ed3eb535fa6400136c636f68884053fede85e9085764193ddc8b7e1a614e6b0`이다.
- 실제 등록 pipeline으로 train 64 / val 64장면을 확인해 통과했다. 결과는
  `artifacts/evaluation/bevfusion/base5_jpeg_portable_261011/pipeline.json`에 보존한다.
  원본+online Q90와 독립 JPEG의 최종 픽셀·GT·LiDAR·투영 동등성과 val 불변을 확인했다.
- JPEG portable/원본 portable/캐시/runner/이미지 증강/직사각형 기하 CPU 검사 33개를 통과했다.
  데이터는 ignored 상태이며 본학습·Vast 접속·업로드는 실행하지 않았다.

<a id="all-jpeg-policy-261011"></a>

## 12. 카메라 RGB 입력 통일 — 2026-10-11 현재 기준

사용자 승인으로 **train·val·test·분석·실험의 카메라 RGB는 native Q90 JPEG**를 사용한다.
PNG는 로컬 원본·JPEG 변환용으로 보존하며 재수집은 필요 없다. instance mask·depth·native
annotation·LiDAR·보정·메타데이터는 무손실로 그대로 유지한다. 'JPEG only'는 RGB만 뜻한다.
MORAI capture의 원본 형식이나 production E25의 ROS 입력 계약을 바꾼 것은 아니다.

| 항목 | 현재 결과 |
|---|---|
| 선택 장면·분할 | 5,000장면, train 4,255 / val 745, 두 crop profile 유지 |
| native RGB JPEG | 20,000개, 1,619,725,032 bytes(1.508GiB) |
| LiDAR·GT·보조 파일을 포함한 전체 payload | 10,424,732,034 bytes(약 9.71GiB), 173,292개 파일 |
| Drive 전달용 tar.zst | 3,333,160,311 bytes(약 3.10GiB), 1GiB 이하 네 조각 |
| 기존 PNG tar.zst | 19,438,853,713 bytes(약 18.10GiB), 로컬 보존 |
| 변하지 않은 lossless 파일 | 153,284개, 원본 파일별 size·SHA256과 동일 |

현재 전달본은 `artifacts/handoff/base5_sunny13_jpeg90_portable_261011/`이며 manifest SHA256은
`c7df4c47babb1197b871ccd877d5bd95b5a5eb8833781906c5dd6ff36d61a91e`다.
train/val/test infos의 RGB 경로만 상대경로 JPEG로 바꿨다. 두 profile의 전체 10,000개 record를
재귀 비교해 **이미지 경로 외 모든 필드가 정확히 같음**을 확인했다. 원본 RGB PNG 20,000개는
새 payload에서 제외됐으며 auxiliary PNG는 남아 있다. 로컬 hardlink 전달본은 수정하지 않는다.

실제 등록 pipeline의 train 64 / val 64장면에서 PNG→native Q90→resize/crop reference와
전달본 JPEG의 최종 픽셀·3D GT·LiDAR·투영 행렬이 정확히 일치했다. 이는 **Q90 reference와의
동등성**이지 PNG val 픽셀이 불변이라는 뜻이 아니다. 기존 PNG val metric과 신규 JPEG val metric은
입력 조건을 명시해 구분한다. 근거는 `artifacts/evaluation/bevfusion/base5_all_jpeg_261011/`의
`inventory.json`과 `pipeline.json`이다. JPEG 정책의 AP 이득은 아직 검증하지 않았다.

### 실행·분석 경로

- 현재 config는 `bevfusion_base5_dense_512_native_jpeg90.py`다. train의 재인코딩 transform은
  제거하고 기존 image/BEV 기하 증강은 유지했다. val/test는 JPEG를 직접 decode한다.
- direct runner는 현재 config에 PNG/mixed manifest가 들어오면 거부한다. all-JPEG 전달본에는
  추가 `--jpeg-cache`를 금지하며 source/config/dataset hash로 새 실행 계약을 만든다.
- `prepare_base5_jpeg_cache.py`는 기본으로 train/val/test 전체를 생성한다. exporter도 기본으로
  전체 RGB를 바꾸며, 과거 mixed 재현만 명시적 `--keep-val-png`를 사용한다.
- 새 비용·분석 표본은 `prepare_base5_qualification_samples.py --data "$JPEG_PORTABLE"`로 만든다.
  PNG 표본은 명시적 `--legacy-png-probe` 비교에만 허용한다. 비용 실행기의 Q90 기본 경로는
  이미 JPEG인 bytes를 다시 encode하지 않는다. 과거 raw/PNG 비교 도구는 원본 증거용으로 보존한다.
- 원본·캐시를 숨기고 전달본을 이동한 단위 검사, mixed 거부·val 캐시 누락·hash 변조·lossless mask
  보존을 포함한 CPU 검사 35개와 archive 검사 2개를 통과했다. GPU·본학습은 이번에 실행하지 않았다.

### Drive 전달 정책

새 기본 경로는 `AjouNice2026/dataset/Base5/sunny13_5k_jpeg90_261011/`이다.
`share_base5_jpeg_portable.py`는 manifest 전수 SHA256 → 무손실 tar.zst → 모든 archive member의
size·SHA256 재읽기 → 분할·연결 SHA256 → Drive 조각별 size·MD5/SHA256 metadata 검증을 수행한다.
원격 검증 후에만 `COMPLETE.json`을 업로드한다. 인증정보·모델·소스·원본 RGB PNG는 포함하지 않는다.
archive SHA256은 `8094e24386e2d15c77558896a652e6fc44fd08338f6e5fe2e29fc4c1eb4b7fcd`다.
압축과 분할은 이미 생성된 JPEG bytes를 변경하지 않는다. 손실 압축은 PNG→Q90 변환 한 번뿐이다.

새 공유본 확인 뒤 기존 PNG 공유 조각만 제거하며 로컬 원본·기존 PNG archive는 그대로 보존한다.
이전 PNG archive SHA256 `ab24307ee17007c4170dacabb00e6c97130e090ddbdb45f0e99c38b244a8374f`를
다시 계산해 확인했다. 실제 Drive 완료·교체 근거는 아래 후속 검증 기록을 따른다.
Vast는 재개하거나 접속하지 않았다. 사용자가 재개하면 이미 원격에 있는 PNG로 전체 JPEG를
생성하고 같은 exporter·새 config·새 hash 계약을 적용한다. 대용량 데이터 재전송은 필수가 아니다.

### Drive 교체 완료 — 2026-10-11 00:42:26 KST

- [현재 Sunny13 5K JPEG 전달본](https://drive.google.com/drive/folders/1Yso9P7f21RWhe7mgU0SMXXp-B824uIhA)을
  업로드하고 네 조각의 원격 size·MD5·SHA256 metadata를 확인했다. 복원 안내·manifest·체크섬과
  `COMPLETE.json`을 포함한 전체 8개 파일의 rclone check는 차이 0이었다.
- 업로드·원격 검증은 127.81초였다. 향후 Vast 다운로드 시간이나 다른 날씨 전송 속도를 보장하지 않는다.
- 기존 PNG 공유본은 원격 파일별 SHA256이 로컬 백업 목록과 같음을 먼저 확인했다.
  기존 19개 archive 조각과 6개 부속 파일만 **Drive 휴지통으로 이동**했으며 영구 삭제하지 않았다.
  원래 폴더에는 `SUPERSEDED_README.txt`로 새 공유본을 안내한다. 알 수 없는 파일은 없었다.
- `dataset/Base5/CURRENT_BASE5.json`을 새 전달본으로 갱신하고 원격에서 다시 읽어 확인했다.
  로컬 원본 PNG·PNG portable·18.10GiB PNG archive는 그대로 보존했다.
- 근거는 `artifacts/backups/base5_sunny13_5k_jpeg90_261011/parts/COMPLETE.json`,
  `artifacts/evaluation/bevfusion/base5_all_jpeg_261011/drive_replacement.json`이다.
- 새 JPEG 기반 300장 비용 표본(대표 200 / stress 100)을 생성했다. 512×1056·batch 4·24 epoch
  설정의 direct runner **prepare-only**도 통과했다. 실학습·새 GPU 비용 측정은 하지 않았다.
  이 준비 경로는 연결 검사 근거이며 최종 source hash가 바뀌면 새 work-dir에서 다시 준비한다.

<a id="vast-jpeg-migration-261011"></a>

## 13. Vast 재개와 전체 JPEG 입력 전환 — 2026-10-11 01:08 KST

사용자가 기존 인스턴스를 다시 켠 뒤 같은 Direct SSH 주소로 접속했다. 기존 소스·원본
PNG 5K·초기화 가중치·설치가 남아 있었으며, PyTorch 2.7.1+cu128 / CUDA 12.8 /
OpenCV 4.11.0과 RTX 4090·49140MiB를 확인했다. 새 이미지 설치나 대용량 데이터 재전송은
하지 않았다. 이전 소스는 별도 snapshot으로 보존하고 최신 인지 소스만 전달했다.

| 원격 전환 검사 | 실제 결과 |
|---|---|
| 전체 native Q90 생성 | 20,000개, 441.07초, 1,619,725,032 bytes |
| 로컬과 원격 JPEG 내용 | 20,000개 이미지의 경로·SHA256·크기·해상도 aggregate 일치 |
| 장면·분할 | 5,000장면, train 4,255 / val 745 유지 |
| 두 profile 전체 infos | 10,000개 record를 재귀 비교, RGB 경로 외 모든 필드 동일 |
| 무손실 데이터 | 153,284개 파일의 size·SHA256 원본과 동일 |
| 독립 전달본 | 173,292개 payload 파일 전수 SHA256 검증 통과 |
| 실제 등록 pipeline | train 64 / val 64장면, Q90 reference와 픽셀·GT·점·투영 정확히 일치 |
| 새 경로 CPU 검사 | 로컬·원격 각각 36개 통과 |

JPEG 내용 aggregate SHA256은
`c381d8612d7e42e5bc756bb0c7443eefa3ece6e45ec479c322fd13272f978e6a`다.
원격 portable manifest SHA256은
`79a0ccfb8b04049cf4892a32d6e54d69246378ff7e0c68a74cc0f5c719cd82b0`이며,
로컬 manifest와 다르다. 생성 시간 등 cache provenance가 다르기 때문이며 이미지·GT 차이가 아니다.
원격 payload는 10,424,732,037 bytes로 로컬보다 3 bytes 크다. 새 manifest를 원래 PNG 또는
로컬 manifest hash로 강제 변경하지 않는다.

```text
/root/base5_sunny13_portable_261010/                       # 원본 PNG 보존
/root/base5_sunny13_native_jpeg90_261011/                  # 원격 생성 캐시
/root/base5_sunny13_jpeg90_portable_261011/                # 현재 원격 입력
/root/base5_training_sunny13_512_b4_e24_jpeg90_261011/      # prepare-only
/root/base5_jpeg_qualification_261011/                    # 변환·검증 근거
```

512×1056·batch 4·24 epoch·worker 4의 새 실행 계약을 준비했으며 `--run`은 주지 않았다.
기존 PNG 준비 디렉터리를 재사용하지 않는다. 본학습 시간 약 4.8시간은 이전 원격 실측 기반
추정이며 이번 변환·CPU 검사로 다시 측정한 값이 아니다. JPEG 정책의 정확도도 미검증이다.

### JPEG 실제 모델의 제한된 저장·재개 검사

기존 smoke exporter는 RGB가 `data/<token>` 안에 있는 PNG 구조를 가정해, 현재 JPEG
portable의 `jpeg_rgb/` 경로를 그대로 쓸 수 없었다. 새 smoke subset은 선택한 두 장면의
JPEG·LiDAR·보정·무손실 보조 파일을 원래 상대경로와 정책으로 보존하고 각 파일 SHA256을
확인한다. 기존 PNG smoke 경로는 별도 함수로 유지했으며 본 5K나 분할을 변경하지 않는다.
원본·캐시·전체 portable 없이도 독립 subset을 읽는 CPU 검사도 추가했다.

실제 512×1056·4클래스 모델에서 AMP 2 iteration 후 epoch 1 checkpoint를 저장하고 의도적으로
중단했다. 같은 실행 계약으로 재개해 epoch 2 / 누적 4 iteration을 통과했다. 재개 시작의
optimizer step은 2였고, optimizer·scheduler·AMP scaler·메인 RNG 상태를 확인했다.
최종 weight의 유한값 검사와 잔여 partial 파일 0개도 통과했다.
`report.json`과 `smoke.log`가 근거다. worker·prefetch의 완전 재현이나 정확도·수렴 결과는 아니다.

별도의 본학습 준비 경로에는 `run_manifest.json` / `resolved_train.py`만 있고 학습 checkpoint나
시작 marker는 없다. 본학습 계약 SHA256은
`bdfaeac31408403a6f7be1a981abfb09df39b4bc482f512dd65a9d503f179d0f`다.
이 manifest의 인지 소스 파일 137개 hash가 최종 로컬 코드와 같음을 확인했다.
본학습은 시작하지 않았으며 검사 종료 후 원격 GPU는 사용률 0%, 할당 약 1MiB였다.

### 외부 checkpoint 백업의 제한 검사

이전 DNS 실패와 달리 이번 Google API 읽기 전용 preflight는 성공했다. 기존 검증용 full
checkpoint 498,641,285 bytes를 대상으로 180초까지만 검사했다. Drive에 파일이 존재하고
size 및 MD5 `60d1952705a8ac50d82256aa03f46f18`가 원격 원본과 일치함을 확인했다.
그러나 전체 SHA256 재읽기·완료 marker·최종 pointer 검증을 끝내기 전에 시간 제한으로
중단했다. **파일 업로드 관측을 자동 백업 완료로 취급하지 않는다.**

`COMPLETE.json` / `LATEST.json`이 없는 이 qualification 경로를 복구 기준으로 자동 선택하지
않는다. 원격 원본 checkpoint와 5K는 보존했고, 장시간 전송을 자동 반복하지 않았다.
재개 후 Google API 연결 성공만으로 외부 네트워크 문제가 영구 해결됐다고 단정하지 않는다.
외부 백업 경로의 완료·반복 watcher 검증과 사용자 본학습 승인은 여전히 남아 있다.

로컬 근거는 `artifacts/evaluation/bevfusion/base5_vast_jpeg_261011/`의
`inventory.json`, `pipeline.json`, `unit_final.log`, `prepare.log`, `external_backup_probe.jsonl`이다.
원격 인증에는 단기 access token만 anonymous RAM으로 전달했으며 갱신 토큰·client secret은
로컬에 보존했다. 인스턴스 중단·삭제, production·tracker·planning 변경은 하지 않았다.

<a id="base5-valid-returns-uint8-261011"></a>

## 14. Base5 유효 반사점·uint8 입력과 증강 검토 — 2026-10-11

Vast 중단 후 사용자 승인 범위에서 로컬 CPU와 5060 Ti만 사용했다. 현재 config는
`bevfusion_base5_dense_512_native_jpeg90.py`이며 수집 데이터·GT·split·production E25는
유지한다. 본학습·Vast 접속·대용량 전송은 수행하지 않았다.

### 증강 강도는 유지한다

실제 train pipeline은 native Q90 RGB 읽기, 고정 resize/crop, 포인트·GT의 동일 BEV 변환,
범위 필터와 포인트 shuffle이다. BEV 회전은 ±0.10rad(약 ±5.73°), scale은 0.95–1.05,
translation은 각 XYZ 축의 표준편차 0.2m인 정규분포다. ±0.2m 상한이 아니다.
이미지 zoom·회전·flip은 고정 또는 OFF이며 blur·노출·noise·재압축도 추가하지 않는다.
이미지 고정 변환과 BEV 변환은 투영 행렬에 반영되므로 서로 다른 좌표계를 임의로 섞지 않는다.

기존 검토의 4,255 train 장면·9,350 GT·장면당 64회 변환에서 중심 기준 XY ROI 이탈은
전체 0.206%, 종방향 80–100m 객체 4.021%, 횡방향 25–30m 객체 4.088%였다.
원거리 경계의 소수 손실은 남지만 현재 강도를 바꿀 근거는 부족하다. 강한 ±45° 회전은
직사각형 ROI에서 원거리 GT를 많이 떨어뜨려 도입하지 않는다.
추론 geometry cache는 학습 중 사용되지 않는다. 증강만 끄면 자동으로 학습 cache가
켜지는 구조가 아니므로 속도를 이유로 증강을 제거하지 않는다.

### 0 기록을 실제 반사점과 구분한다

5K raw 전수의 96,000,000개 XYZI 기록 중 XYZ가 모두 0인 기록은 44,877,300개,
**46.7472%**다. 이 기록의 intensity도 모두 0이었다. nonfinite 기록은 0개였고,
필터 후 빈 클라우드가 된 장면도 0개였다. 이는 객체 미검출률이나 GT 제외율이 아니다.

수집 audit는 이미 0 기록을 빼고 GT의 LiDAR 점 수를 계산했지만 기존 학습 loader는
beam 슬롯을 모두 읽었다. 그 뒤 이동 증강이 0 기록들을 같은 위치로 옮겨 가짜 밀집
클러스터가 생길 수 있었다. `MoraiFilterInvalidPoints`를 **포인트 읽기 직후·증강 전**에
삽입해 exact XYZ-zero 및 nonfinite 행만 제거한다. intensity=0인 실제 점과 근접 점은
유지하며 거리 cutoff·점 수 sampling·GT 삭제를 추가하지 않는다. 빈 클라우드에 가짜 점을
넣지 않는다. 온라인 추론은 기존처럼 ROI 내 포인트가 모두 없으면 명시적 오류를 반환한다.

Base5 train/val/test와 config 기반 온라인 전처리가 같은 정책을 따른다. 온라인 범위
경계도 MMDetection3D의 열린 구간과 맞췄다. E25에는 필터와 경계 변경을 적용하지 않는다.
raw LiDAR 파일·infos·GT·카탈로그·가시성 조건은 변경하지 않으므로 재수집하지 않는다.
점 기록이 약 47% 줄어도 sparse convolution 비용이 약 47% 줄어드는 것은 아니다.
중복 0 기록은 매우 적은 voxel에 모이므로 이 변경의 주목적은 입력 정합이다.

### 픽셀을 바꾸지 않고 float32 변환 시점을 늦춘다

기존 경로는 JPEG uint8 decode → padded float32 → resize용 uint8 → float32 이미지 →
float32 pack이었다. Base5는 loader와 crop 출력을 uint8으로 유지한다. 실제
`Det3DDataPreprocessor`가 device 전달 후 float32 변환·mean/std 정규화를 수행한다.
기존 transform의 기본 output은 float32로 남겨 Base3 설정을 보존한다.
지원하지 않는 resize/crop이 목표 크기를 만들지 못하면 잘린 작은 tensor를 내보내지 않고
명시적으로 실패한다. 현재 고정 crop의 픽셀·위치·투영은 변하지 않는다.

같은 64 train + 64 val 장면을 3회 비교했다. 각 split/repeat의 첫 8장면은 비용 집계에서
제외했고 비교 순서는 순환했다. GPU 학습과 겹치지 않은 CPU 비용이다. GPU 정규화와
모델 forward/backward 시간은 아래 수치에 포함되지 않는다.

| train CPU 단계 | 기존 float32·0 기록 유지 | 유효점 필터·float32 | 유효점 필터·uint8 |
|---|---:|---:|---:|
| JPEG 읽기·decode·padding | 13.73ms | 13.91ms | 10.71ms |
| 고정 resize/crop | 9.75ms | 9.70ms | 6.93ms |
| 유효 반사점 필터 | 없음 | 0.46ms | 0.44ms |
| BEV 회전·이동·scale | 0.62ms | 0.59ms | 0.53ms |
| 입력 pack | 6.19ms | 6.21ms | 2.11ms |
| 모든 transform 합계 | **31.39ms** | **31.75ms** | **21.47ms** |
| val 모든 transform 합계 | 29.81ms | 30.36ms | 20.50ms |

4개 512×1056 이미지의 운반량은 25,952,256 → 6,488,064 bytes로 75% 줄었다.
이는 로더/IPC/CPU→GPU의 이미지 운반량이며 원본 JPEG 파일 크기나 최종 모델 float feature
메모리가 75% 줄었다는 뜻은 아니다. 순차 CPU 준비 비용 감소도 전체 학습 시간 감소와 다르다.

### 실제 짧은 학습 비용은 거의 같다

5060 Ti에서 512×1056·batch 2·worker 4·AMP AdamW로 각 32 iteration을 실행했다.
첫 8 iteration은 제외했다. 같은 64장면을 읽었고, 측정 24 iteration의 48장면과 증강 후
GT geometry hash·클래스 개수를 대조했다. 두 비교 모두 유효점 필터를 적용했으며 이미지
float32/uint8 운반만 달랐다. 모델 단독 추론 비용이나 전체 epoch 측정이 아니다.

| 로컬 실제 학습 평균 / batch 2 | 유효점·float32 운반 | 유효점·uint8 운반 |
|---|---:|---:|
| 로더 대기 | 10.69ms | 4.94ms |
| device 전처리·forward·backward·optimizer | 737.80ms | 742.78ms |
| 전체 iteration | **748.49ms** | **747.72ms** |
| 측정 구간 peak allocated | 10,879MiB | 10,886MiB |

전체 평균 차이는 약 0.10%로 실질적인 전체 학습 가속을 입증하지 못했다. 로더 대기는
줄었지만 대부분의 시간은 GPU 학습이며 작은 변동이 그 차이를 상쇄했다. uint8 운반은
모델 활성값 크기를 바꾸지 않으므로 peak VRAM도 거의 같다. 기존 4090 학습 ETA를
이 수치나 CPU 비용 감소율로 줄이지 않는다. 단일 짧은 비교의 tail 차이로 우열을 확정하지 않는다.
근거는 `float_gpu_probe_v2/cost.json`과 `uint8_gpu_probe/cost.json`이다.

### 검증과 재개 조건

관련 단위 검사 41개를 통과했다. actual pipeline 384회 비교에서 픽셀·정규화 결과·GT·
유효 포인트·변환 행렬이 정확히 일치했다. 별도로 실제 JPEG와 raw를 온라인 메모리 경로에
넣어 128장면의 픽셀·포인트·정규화 결과가 정확히 같고 보정 행렬은 1e-5 허용 오차 내임을
확인했다. 수집 manifest SHA256은 기존
`c7df4c47babb1197b871ccd877d5bd95b5a5eb8833781906c5dd6ff36d61a91e`를 유지한다.
AP·미션 주행 향상·새 입력 크기의 실제 ROS 풀스택 완료를 의미하지 않는다.

최종 코드에서 실제 2장면·4클래스의 full-model AMP 4 iteration을 별도로 수행했다.
epoch 1 / iteration 2의 원자적 checkpoint 저장 후 의도적으로 중단하고 같은 계약으로
epoch 2 / 누적 iteration 4까지 재개했다. optimizer step 2, scheduler·AMP scaler·메인
RNG 상태의 복구, 최종 weight 유한값, 잔여 partial 파일 0개를 확인했다.
`gpu_resume_smoke/report.json`이 근거다. worker 0의 제한 검사이며 전체 worker/prefetch
또는 중간 batch의 정확한 replay, 본학습 수렴을 보장하지 않는다.

재현 도구는 `perception/detection3d/bevfusion/scripts/qualify_base5_input_pipeline.py`다.
`--online-only`는 비용 측정 없이 파일 기반 입력과 온라인 전처리를 대조한다.
ignored 근거는 `artifacts/evaluation/bevfusion/base5_input_pipeline_261011/`의
`cpu_isolated_report.json`, `online_report.json` 및 제한 GPU 실험 결과다.
처음 GPU 비용 실행은 실험용 Docker의 기본 64MiB shared memory 부족으로 worker가
실패했다. 새 컨테이너의 `--shm-size=2g`로 수정했다. 실제 Vast의 shared memory는
접속 후 별도로 확인하며 중첩 Docker 명령을 실행하지 않는다.

다음 Vast 재개 시 기존 JPEG 데이터는 유지하고 최신 소스만 전달한다. 변경 전 prepare-only
work-dir과 hash를 덮어쓰지 않고 새 경로에서 준비·짧은 학습/재개·외부 백업 검증을 수행한다.
13절의 원격 입력 내용은 유효하지만 그때의 소스/학습 계약은 현재 코드의 실행 계약이 아니다.

<a id="base5-heatmap-target-alignment-261011"></a>

## 15. Base5 학습 heatmap 정답 축 수정 — 2026-10-11

### 발견과 수정 범위

현재 Base5는 `[Y,X]` feature, `(x,y)` query 위치, `y*X+x` flatten index를 사용한다.
그러나 상속받는 `TransFusionHead.get_targets_single`은 Gaussian 정답 생성 시
`center_int[[1,0]]`를 전달하고 있었다. Gaussian 함수는 `(x,y)`를 받아 `[y,x]`에 기록하므로
이미 정합된 Base5에서는 이 교환이 잘못된 정답 위치를 만든다. 전방 40m·Y=0의 올바른
cell은 `[54,234]`이나 교환 후 행 234는 높이 108의 heatmap 밖으로 나가 peak가 사라졌다.
후방 40m의 정답은 `[54,101]`이어야 하나 `[101,54]`에 생성됐다.

이전 sparse/camera/query 축 검사와 loss 유한값·짧은 역전파 검사는 **실제 학습 정답 peak의
위치까지 검증하지 못했다.** 이를 기존 검증 완료 범위의 누락으로 기록한다. 이 문제로
수집 GT 파일이 삭제됐다는 뜻은 아니며, production E25의 성능·영향 범위로 일반화하지 않는다.

`MoraiRectangularTransFusionHead`에 Base5 전용 `build_dense_heatmap_targets`와
`get_targets_single` override를 추가했다. 부모의 matching·sampling·분류·box 회귀 정답은
유지하고 반환되는 마지막 dense heatmap만 `[classes,Y,X]` 정답으로 교체한다.
부모의 기존 heatmap 계산은 실행되지만 그 결과는 사용하지 않는다. 새로운 target 생성은
학습 경로에만 추가되며 prediction/encoder/voxel/pooling/이미지 입력에는 연산을 추가하지 않는다.
Gaussian 크기 정책은 유지한다. 파라미터·state-dict key·E25 head·third-party 소스는 바꾸지 않는다.
범위 밖 중심을 경계로 clamp하지 않고, 잘못된 클래스·크기·nonfinite·격자는 명시적으로 거부한다.

### 실제 5K와 학습 증강 검사

CPU에서 현재 native JPEG portable의 train/val infos를 사용했다. 증강 검사는 현재
`BEVFusionGlobalRotScaleTrans`, `ObjectRangeFilter`, `ObjectNameFilter`를 그대로 실행했다.
GT 변환만 검사하기 위해 포인트 입력은 빈 `LiDARPoints`를 사용했으며, 이미지 decode나
전체 학습 로더 비용 검사가 아니다. train 4,255장에 결정적인 seed로 두 번의 증강을 적용했다.

| 검사 조건 | 장면 검사 수 | 검사한 GT | 올바른 위치의 peak | 증강 범위 필터 제외 |
|---|---:|---:|---:|---:|
| 원본 train | 4,255 | 9,350 | 9,350 | 0 |
| 원본 val | 745 | 1,702 | 1,702 | 0 |
| train 증강 1 | 4,255 | 9,333 | 9,333 | 17 |
| train 증강 2 | 4,255 | 9,329 | 9,329 | 21 |
| **검사 합계 — 반복 포함** | **13,510** | **29,714** | **29,714** | **38** |

고유 5K의 11,052개 GT와 반복 증강을 포함한 29,714회 검사를 구분한다.
원본에서 교환된 중심이 heatmap 밖으로 나간 GT는 10,260개(92.83%)였다. 수정 후
실제 peak와 query flatten 위치가 모두 일치했다. 이는 detector recall/AP 수치가 아니다.
증강으로 semantic ROI를 벗어난 17/21개는 기존 범위 필터의 정상 결과이며 새 누락이 아니다.
빈 장면도 0 heatmap으로 통과했다. 클래스별 고유 GT는 Vehicle 4,519 / Pedestrian 2,963 /
Obstacle 3,289 / Animal 281개다.

Box coder encode/decode의 XYZ·크기는 최대 절대 오차 `2.288818359375e-05 m`였다.
yaw는 주기성을 고려해 sin/cos로 비교했다. FP32 100m 좌표의 수 ULP 오차를 허용하려고
roundtrip 허용치를 0.05mm로 두었으며 cell index와 peak=1 검사는 정확 일치를 요구한다.
최초 `cpu_v1`은 0.02mm 허용치에서 중단됐고, 최종 `cpu_v2`를 통과 근거로 사용한다.
입력 manifest·train/val infos SHA256와 GT tensor는 검사 전후 동일하다.

### 회귀 검사와 실제 전체 모델 smoke

- CPU 관련 단위 검사 **35개 통과**, CUDA 전용 resume 검사 1개는 CPU 실행에서 제외했다.
  새 target 검사 8개는 전후좌우·네 클래스·semantic/padding 경계·빈 GT·오류 입력,
  기존 회귀 정답 불변·E25 method 불변·loss gradient 방향을 포함한다.
- CPU integration 검사에서는 CUDA IoU/Hungarian만 고정 assigner로 대체하고 실제 부모의
  decode·PseudoSampler·encode·반환 값을 비교했다. 마지막 heatmap 이외의 값은 동일하다.
- 별도로 5060 Ti에서 실제 JPEG 캡처 2장과 4개 클래스를 사용하는 전체 모델 AMP
  **4 iteration**을 실행했다. 이 경로의 matching·loss·backward·optimizer는 실제 구현이다.
  target 생성 4회에서 총 8개의 증강 후 정답 peak를 직접 확인했다.
- epoch 1 / iteration 2에서 원자적 저장·의도적 중단 후 epoch 2 / 누적 iteration 4로
  재개했다. optimizer step 2·scheduler·AMP scaler·메인 RNG 복구, 최종 weight 유한값과
  잔여 partial 파일 0개를 확인했다. worker 0의 제한 검사이지 전체 학습 수렴·AP 검증은 아니다.
- 본학습·Vast 접속/재개·미션 rosbag 성능 평가·격자/depth 비용 비교는 수행하지 않았다.

재현 도구는 `perception/detection3d/bevfusion/tools/audit_base5_heatmap_targets.py`다.
team 이미지에서 `--data artifacts/handoff/base5_sunny13_jpeg90_portable_261011`,
`--config perception/detection3d/bevfusion/configs/bevfusion_base5_dense_512_native_jpeg90.py`,
`--augmentation-repeats 2`, 신규 `--output`으로 실행한다. ignored 근거는
`artifacts/evaluation/bevfusion/base5_heatmap_targets_261011/`의 `cpu_v2/report.json`,
`cpu_v2/before_after_targets.png`, `gpu_resume_smoke/report.json`,
`gpu_resume_smoke/target_checks.json`이다. GPU smoke는 기존
`smoke_base5_direct_training.py`로 재현하며, 이번 직접 target 관측은 런타임 검사 wrapper로 수행했다.

### 후속 공간 해상도·depth 실험 순서

1. **완료:** Base5 정답 축 수정과 전수·증강·역전파·재개 검사.
2. **다음:** 로컬에서 LiDAR/Camera BEV `5/20cm`, `7.5/30cm`, `10/40cm`의
   단계별 추론·전체 서비스 비용·VRAM·짧은 학습 처리량을 비교한다. 이미지 512×1056,
   depth `[1,100,0.5]`, semantic ROI X±100/Y±30m와 입력·AMP·batch 조건을 고정한다.
   각 후보의 padding·원점·head metadata를 함께 맞춘다. 현재 stride는 LiDAR 8 / Camera 2이므로
   fusion/head 간격은 각각 40/60/80cm다. Gaussian/local-max의 물리적 범위 변화도 기록한다.
3. 실제 5K에서 작은/밀집 객체의 feature cell 점유와 후보 충돌 위험을 분석하고 학습 후보를 좁힌다.
   속도와 점유 분석만으로 탐지 성능 우위를 확정하지 않는다.
4. 선택 격자에서 depth 범위 1–100m를 유지한 채 간격 25/50/100cm(396/198/99 bins)를
   비교한다. 전체 조합 sweep을 먼저 수행하지 않는다. 현재 50cm의 최적성도 미입증이다.
5. Vast 재개 요청 후 최신 소스·새 work-dir로 준비하고 실제 4090 비용·짧은 학습·외부
   checkpoint 백업을 검증한다. 이후 승인한 본학습과 rosbag 평가, class별 threshold 선택으로 이어간다.

기존 raw/GT는 격자·depth 후보에서도 재사용할 수 있다. 이전 원격 소스·prepare-only 계약은
이번 정답 수정 이전이므로 덮어쓰거나 그대로 resume하지 않는다. 데이터 재전송은 필요하지 않다.

<a id="base5-upright-native-rpy-261011"></a>

## 16. 기울어진 native BBox의 upright 7D 변환 — 2026-10-11

### 발견과 정책

native `Instance.txt`는 중심·크기·roll/pitch/yaw를 기록하지만 기존 infos 변환은
roll/pitch를 버리고 크기는 그대로 사용했다. 실제 넘어진 드럼통에도 서 있는 드럼통의
박스가 생기는 문제다. raw → 기존 infos의 값 복사 자체는 일치했으나 **9D 회전 박스를
7D 박스로 근사하는 정책이 부족했다.** 카메라로 확인한 드럼통·철제 바리케이드도 실제로
기울어져 있었다. 지면 배치 버그나 모든 원본 annotation의 정확성을 인증하는 검사는 아니다.

정책 `native_rpy_fixed_yaw_enclosure_v1`은 중심과 native yaw를 유지하고, native 회전 박스의
8개 모서리를 모두 포함하도록 upright 박스의 길이·폭·높이를 계산한다.
`R = Rz(yaw) Ry(pitch) Rx(roll)`일 때 크기는 `abs(Ry Rx) @ native_size`다.
이는 고정 yaw에서의 enclosure이지 최소 부피 박스나 실제 mesh segmentation이 아니다.
새 공간에는 배경이 더 포함될 수 있다. 9D detector·새 클래스·재촬영은 도입하지 않는다.

5° 같은 임계값으로 일부만 바꾸지 않고 **모든 GT에 동일한 변환**을 적용한다. native R/P가
정확히 0이면 크기도 정확히 유지한다. 실제 5K에는 미세한 기울기도 있어 모든 GT 크기에
작은 변화가 생겼다. ‘큰 기울기 55개’와 ‘변환 대상 전체 11,052개’를 혼동하지 않는다.
가시성 판정과 `num_lidar_pts`는 기존 native full-RPY 박스의 센서 근거 그대로이며,
확장 박스 안의 새 배경 포인트를 정답 근거로 추가하지 않는다.

### 전수·실제 포인트·증강 결과

현재 512×1056 profile의 고유 GT 기준이다. 기울기는 `acos(abs(R[2,2]))`로 계산했다.
180° 뒤집힘을 단순히 ‘큰 경사’로 집계하지 않는 측정이다.

| 항목 | 결과 |
|---|---:|
| 장면 / train / val | 5,000 / 4,255 / 745 |
| GT / 클래스·중심·yaw·split 변경 | 11,052 / 없음 |
| 5° 초과 | 55 GT, 54장면: 장애물 51 / 차량 4 |
| 20° 초과 | 17 GT: train 16 / val 1 |
| camera-only / 카메라 component 대응 모호·LiDAR 근거 유지 | 679 / 223 GT, 변동 없음 |
| 큰 기울기 GT의 native 박스 내부 실제 LiDAR 점 | 8,525개 |
| 위 점 중 파생 박스 밖으로 빠진 수 | **기존 633 → 수정 0** |
| 원본 + train 증강 2회, native 모서리 포함 검사 | **29,752회 통과** |
| ROI 필터 후 실제 학습 정답 peak 검사 | **29,714회 통과** |
| box coder 왕복 최대 XYZ·크기 절대 오차 | 0.00002289m |

전체 GT의 크기 변화 절대값 중앙값은 길이 6.12mm / 폭 4.46mm / 높이 10.15mm다.
p95는 28.07 / 21.78 / 79.94mm다. 큰 기울기 사례의 최대 변화는 각각
0.546 / 0.468 / 1.184m로, 평균적으로 작은 변화라고 넘어진 객체까지 작은 변화인 것은 아니다.

`b5v2_04541`의 PE_Drum은 약 89.86° 기울어졌다. native 박스에 들어 있던 실제 점
17개 중 기존 파생 박스 밖의 7개가 새 박스에는 모두 포함된다. 이 사례를 실제 좌표로
그린 `native_and_aug_v1/sideways_barrel_boxes.png`에 native/기존/새 박스를 함께 기록했다.

검사는 actual `MoraiBase5Dataset`, native `Instance.txt`, 8개 모서리, actual
`BEVFusionGlobalRotScaleTrans`를 사용했다. 기하 증강 검사에서는 모서리 자체를 포인트로
변환해 GT와 함께 움직이는지 확인했다. 55개 큰 기울기 사례는 실제 LiDAR도 검사했다.
모든 카메라의 물리적 정합·모든 GT 의미·탐지 AP를 전수 인증했다는 뜻은 아니다.

- 관련 단위·회귀 검사: host 27개 + team 이미지 32개, **총 59개 통과**.
- 실제 5060 Ti 전체 모델: animal 장면과 넘어진 드럼통이 있는 혼합 장면, 네 클래스,
  AMP 4 iteration 통과. iteration 2 저장 → iteration 4 재개를 확인했다.
- optimizer·scheduler·AMP scaler 복구와 유한 weight, 잔여 partial 파일 0개를 확인했다.
  이는 아주 짧은 학습 경로 검사이며 수렴·성능·학습 처리량 실험은 아니다.

### 현재 입력과 재현·원격 반영

**최신 로컬 학습 입력**은 다음 경로다. 이전 전달본을 덮어쓰지 않았다.

```text
artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release/
```

manifest SHA256: `47f208bba79cb4866f64bf6133b06905e246822fc4fa24ccdafb7f90622df039`.
원본 JPEG portable의 SHA256은 계속
`c7df4c47babb1197b871ccd877d5bd95b5a5eb8833781906c5dd6ff36d61a91e`다.
512×1056와 448×896 profile을 모두 변환했고 센서·native annotation·audit는 동일한
내용의 hardlink로 보존했다. provenance에 이전 manifest와 모든 박스 변경·native RPY를 남겼다.
`base5_sunny13_jpeg90_upright_v1_261011/`은 host hardlink 권한 문제로 중단된 진단 경로이며
최신 입력이 아니다. root 권한의 기존 team Docker로 완성한 `_release`만 사용한다.

신규 raw 변환기의 기본값도 같은 정책이다. 이전 방식은 명시적인
`--box-geometry-policy native_size_yaw_only_legacy`일 때만 재현한다.
현재 native-JPEG config와 direct runner는 manifest 및 모든 split infos의 정책을 검사해
old yaw-only 라벨을 실수로 사용하지 못하게 한다. portable exporter와 smoke subset도 정책을 유지한다.

기존 JPEG portable에서의 변환은 CPU-only이며 다음 도구를 사용한다. output은 새 경로여야 한다.
로컬의 root 소유 payload는 team Docker 안에서, 기존 Vast payload는 root 셸에서 실행한다.

```bash
python perception/detection3d/bevfusion/tools/prepare_base5_upright_portable.py \
  --data /path/to/existing/jpeg_portable \
  --output /path/to/new/jpeg_upright_portable \
  --hardlink
```

`--hardlink`는 같은 파일시스템의 불변 payload에만 사용한다. 다른 디스크라면 생략한다.
전체 SHA 검증은 수행하지만 JPEG/LiDAR 재인코딩·클라우드 전송은 하지 않는다.
원격 재개 승인을 받으면 최신 소스 전달 → 기존 원격 JPEG에서 이 변환 → 새 준비 계약과
work-dir → 4090 재검증 순서다. 옛 work-dir의 resume로 라벨을 중간 교체하지 않는다.
**이번에는 Vast에 접속하지 않았고 Drive 전달본도 수정하지 않았다.** Drive의 기존 JPEG/raw
자료에는 재변환에 필요한 native RPY가 보존돼 있으므로 재촬영·센서 재전송은 필요 없다.

ignored 검증 근거는 `artifacts/evaluation/bevfusion/base5_upright_targets_261011/`의
`native_and_aug_v1/report.json`, `heatmap_v1/report.json`, `gpu_resume_smoke/report.json`이다.
재현 도구는 `audit_base5_upright_targets.py`, `audit_base5_heatmap_targets.py`,
`smoke_base5_direct_training.py`이며 smoke의 `--capture` 2개로 위 실제 장면을 지정할 수 있다.

다음 순서는 15절의 공간 격자 세 후보 비교 → 선택 격자의 depth 간격 비교 → 승인 후
4090 최신 소스·새 라벨·비용·외부 백업 검증 → 본학습·rosbag 성능 평가다.

<a id="base5-grid-depth-local-261011"></a>

## 17. 공간 격자·depth 간격의 로컬 비교 — 2026-10-11

### 비교 계약과 내부 padding

16절 upright portable와 현재 native-JPEG config를 사용한다. 센서/GT·이미지 512×1056·
전후방 ±100m/좌우 ±30m semantic ROI·4클래스·200 query·분리 pooling·batch 1 AMP는 고정한다.
미학습 공유 파라미터로 연산 비용을 비교하며 AP·ROS 네트워크·실제 4090 성능을 측정하는 작업이 아니다.
공간 비교에서는 비기하 파라미터 390개의 값과 SHA256이 모든 후보에서 동일하다.

| 후보 | LiDAR voxel XY | Camera pooling 간격 | Fusion/head 간격 | 실제 feature Y×X |
|---|---:|---:|---:|---:|
| 현재 그대로 대조군 | 7.5cm | 30cm | 60cm | 108×336 |
| 공통 padding · 정밀 | 5cm | 20cm | 40cm | 168×504 |
| 공통 padding · 기준 | 7.5cm | 30cm | 60cm | 112×336 |
| 공통 padding · 경량 | 10cm | 40cm | 80cm | 84×252 |

Camera의 20/30/40cm는 fusion 직전의 최종 간격이 아니다. LiDAR stride 8과 Camera
downsample 2를 통과하면 양쪽 feature의 간격은 각각 40/60/80cm로 같아진다.
각 후보의 원점·범위·sparse shape·head train/test/coder metadata를 함께 계산한다.

공통 비교의 내부 범위는 X±100.8/Y±33.6m다. 세 후보 모두 XY stride 16과 나누어떨어지고
동일한 원점을 사용한다. 현재 내부 X±100.8/Y±32.4m 대조군을 별도로 두어 padding 변경과
해상도 변경의 영향을 분리한다. **센서/GT의 승인된 semantic ROI를 확대하지 않는다.**
이 공통 padding은 비교 조건이며 현재 기본 학습 config를 변경하는 승격이 아니다.

### 새 후보에서 발견한 정수 격자 반올림 누락

`201.6/.2`와 `201.6/.4`는 Python float로 각각 `1007.999…`, `503.999…`다.
분리 pooling의 plan builder가 `int`로 잘라 마지막 padding 열을 하나 잃을 수 있었다.
stride 2 이후 shape가 같을 수 있어 fusion tensor shape 검사만으로는 발견하지 못한다.
30cm의 현재 설정에서는 이 X열 누락이 발생하지 않았다.

opt-in pooling plan은 정수 격자로 반올림하고, 실제로 나누어떨어지지 않는 bounds는 거부한다.
Base5 rectangular view의 새 학습 인스턴스도 config-owned geometry를 즉시 복원해, 추론에서만
복원하고 학습의 `nx` metadata는 잘린 채 남는 일을 방지한다. E25/third-party 기본 클래스는
수정하지 않는다. 모서리 최종 열의 point 유지와 CPU/CUDA gradient·cache·stream을 회귀 검사한다.
기존 `.long()`의 하한 경계 양자화 정책은 이번에 변경하지 않는다.

### 실제 5K 정보량과 밀집 객체

모든 후보에서 고유 11,052개 GT의 target peak·query 위치·coder 왕복이 통과했다.
현재 실제 BEV 증강 1회까지 후보별 20,385회, 네 후보 합계 81,540회 peak 검사를 통과했다.
증강마다 train 17개 GT가 semantic ROI 밖으로 나가 제외되는 기존 정책은 동일하다.
‘peak 검사 횟수’는 중복 셀의 독립 peak 수를 뜻하지 않는다.

| 공통 비교 후보 | 같은 클래스·같은 셀의 중복 GT 초과 수 | 같은 클래스 3×3 이웃 pair 수 | 전체 GT의 native 근거 voxel 합계 |
|---|---:|---:|---:|
| 5/20cm | 0 | 25 | 751,907 |
| 7.5/30cm | 0 | 82 | 619,897 |
| 10/40cm | 8 | 109 | 529,648 |

이웃 pair는 local-max 후보 간 경쟁 위험이지 반드시 검출 실패한다는 뜻이 아니다.
동일 높이의 인접 peak는 함께 남을 수 있고, head는 연속 center offset을 회귀한다.
객체 폭이 한 셀보다 작다고 탐지가 불가능한 것도 아니다. Camera feature·voxel XYZ 평균·
여러 channel과 query attention이 존재한다. 이 표를 recall/AP로 해석하지 않는다.

native full-RPY 박스 내부이면서 실제 모델 semantic ROI에 들어오는 점만 voxel 통계에 쓴다.
upright enclosure에 새로 포함된 배경 점을 객체 근거로 추가하지 않는다. 합계는 GT별 점유 합산으로,
서로 겹치는 component의 물리 포인트 중복을 제거한 dataset 전체 포인트 수는 아니다.
원본 포인트 수를 변경하지 않았고, camera-only 679개는 세 후보 모두 usable LiDAR 0점이다.

근거리 10m 미만의 native 근거 voxel 중앙값은 다음과 같다.

| 객체군 | 5cm | 7.5cm | 10cm |
|---|---:|---:|---:|
| 차량 | 349 | 266 | 215 |
| 보행자 | 46 | 36 | 30 |
| 장애물 | 34 | 27.5 | 24 |
| 동물 | 10.5 | 9 | 8.5 |

반면 60m 이상 차량의 중앙값은 세 후보 모두 **5점 / 5 voxel**이었다.
16채널의 원거리 희소성은 더 작은 voxel만으로 해소되지 않는다. 정밀 후보의 직접적인
정보 보존 이점은 원거리 새 반사점 생성이 아니라 근거리 포인트 평균화 감소·BEV 공간 분리에 있다.

### 전체 모델 비용과 병목 — 5060 Ti, 공간 비교

서로 다른 300장(대표 200 + stress 100)을 후보마다 입력했다. 순방향과 역방향 후보 순서를
바꾼 두 실행에서도 결론은 같았다. 다음 표는 두 실행의 평균이다. 같은 300장을 두 번
측정했으므로 600개의 독립 장면이라고 하지 않는다. `model`은 test_step이고 `service`는
CPU JPEG decode/crop + test_step + decoded prediction CPU 복사다. 디스크·ROS·네트워크는 제외한다.

| 후보 | model 평균 | service 평균 | 공통 기준 대비 service |
|---|---:|---:|---:|
| 현재 그대로 7.5/30cm | 94.73ms | 104.16ms | 별도 padding 대조군 |
| 공통 5/20cm | 128.51ms | 138.02ms | +32.2% |
| 공통 7.5/30cm | 94.80ms | 104.38ms | 기준 |
| 공통 10/40cm | 84.98ms | 94.44ms | −9.5% |

별도 CUDA event 계측 32장 평균은 다음과 같다. 표 전체를 단순 합산해 service와 비교하지
않는다. 비계측 비용과 다른 반복이며 view의 하위 span·host 대기·동기화 구간이 존재한다.

| 구간 | 5/20cm | 7.5/30cm 공통 | 10/40cm |
|---|---:|---:|---:|
| Camera backbone | 44.94ms | 44.80ms | 44.85ms |
| Depth/feature 생성 | 8.78ms | 8.67ms | 8.69ms |
| 분리 pooling | 7.19ms | 2.41ms | 1.35ms |
| Camera BEV downsample | 8.02ms | 3.41ms | 1.82ms |
| LiDAR sparse backbone | 13.80ms | 11.59ms | 10.80ms |
| Fusion | 5.17ms | 2.10ms | 1.06ms |
| BEV backbone | 9.69ms | 4.00ms | 2.37ms |
| BEV neck | 2.67ms | 0.81ms | 0.36ms |
| Detection head/decode | 20.04ms | 8.63ms | 5.49ms |

정밀 격자의 셀 수는 공통 기준의 2.25배지만, 전체 모델을 2.25배로 환산할 수 없다.
이미지 backbone과 depth 생성은 입력 픽셀·depth 수가 고정돼 거의 같다. 비용 증가는
dense BEV와 head에 집중되고, sparse 연산은 빈 공간 전체가 아니라 활성 voxel 중심이다.
컨볼루션 커널 크기·head의 min_radius와 local-max 이웃 수는 셀 단위로 유지했으므로
실제 미터 단위 receptive field와 최소 Gaussian 크기도 변한다. 이것까지 통제한 순수
양자화 효과나 정확도 A/B라고 주장하지 않는다.

최종 재실행은 case 사이에 계측용 bound-method 참조를 GC한 뒤 CUDA peak를 초기화한다.
이전 case의 모델 참조가 남아 VRAM 통계에 합산되는 benchmark 오염을 막는 조치이며,
production inference를 더 빠르게 만드는 수정은 아니다. `spatial_v4_final`에서도 같은
속도 결론을 확인했다. 순방향/역방향 실행의 수 ms 변동과 padding 대조를 보존하므로
1% 내외 차이를 확실한 가속으로 주장하지 않는다.

실제 학습 비용은 camera-only 초기화·같은 24개 장면과 증강·AdamW AMP·batch 1/workers 4다.
첫 8 step을 제외한 16 step을 측정했고, 네 후보의 scene ID와 증강 후 GT 해시가 모두 같다.
모든 loss는 유한하며 optimizer는 24회 모두 갱신됐다. 모델 객체를 별도 프로세스에서
새로 만들어 rounded training geometry를 검사했다. validation·checkpoint·전체 epoch 비용은 제외한다.

| 후보 | 실제 학습 1 step 평균, loader 포함 | steady 학습 allocated peak | steady 추론 allocated peak |
|---|---:|---:|---:|
| 현재 그대로 7.5/30cm | 373.39ms | 5,754MiB | 1,022MiB |
| 공통 5/20cm | 447.03ms | 7,646MiB | 1,427MiB |
| 공통 7.5/30cm | 365.08ms | 5,800MiB | 1,022MiB |
| 공통 10/40cm | 346.98ms | 5,161MiB | 1,021MiB |

이 짧은 측정에서 정밀 후보의 학습 비용은 공통 기준보다 약 22%, 경량 후보는 약 5%
차이였다. loader 대기는 평균 약 3ms로 GPU 역전파보다 작았다. 4090·batch 4·전체 epoch에
같은 비율/시간을 보장하지 않는다. 초기 cuDNN 알고리즘 탐색의 임시 peak는 추론 약
3.63–8.60GiB, 학습 약 7.59–12.35GiB로 기록돼 steady 값과 구분해야 한다.

### Depth 후보에서 구분해야 하는 캐시 비용

현재 7.5/30cm와 optical depth 범위 `[1,100)`m를 유지하면 25/50/100cm 간격은
396/198/99 planes다. 이는 실제 depth 오차가 25/50/100cm라는 뜻이 아니다.
학습된 soft depth 분포로 feature를 여러 깊이에 보내는 discretization 설정이다.

기본 256MiB 캐시에서 50cm는 plan 약 155MiB, 100cm는 약 78MiB로 들어갔다.
25cm는 예산을 넘어서 **348회 모두 miss, hit 0회**였다. model 평균은 각각
94.79/91.75/259.21ms(50/100/25cm)였다. 25cm의 큰 지연은 depth 연산 증가만이 아니라
geometry 정렬·plan 재계산을 포함한다. 512MiB 대조 실험으로 이 비용을 분리하며,
기본 모델의 cache 예산과 depth 간격을 자동 변경하지 않는다.

512MiB 대조에서는 세 후보 모두 **hit 347 / miss 1**로 정상 캐시를 사용했다.
같은 300장 결과는 다음과 같다. depth head 출력의 weight/bias 2개는 bin 수가 바뀌어
shape를 공유하지 못하며 나머지 388개 비기하 파라미터는 같은 값이다. 학습된 depth의
정확도 비교가 아니다.

| depth 간격 | planes | model 평균 | service 평균 | 실제 plan payload |
|---|---:|---:|---:|---:|
| 25cm | 396 | 93.57ms | 102.92ms | 308.8MiB |
| 50cm | 198 | 93.44ms | 102.93ms | 154.9MiB |
| 100cm | 99 | 93.93ms | 103.23ms | 78.0MiB |

이 실행의 전체 비용 차이는 반복 측정 변동 수준이다. 세부 CUDA event에서 25cm의
depth/feature는 9.44ms, pooling은 2.82ms이고, 50cm는 8.62ms / 2.15ms다.
하위 연산은 늘지만 전체 비용을 지배하는 image backbone은 같다. ‘25cm는 무료’라고
일반화하지 않고, 반대로 캐시 없는 259ms를 depth bin 자체의 필연적인 비용으로 쓰지도 않는다.
추론 cache는 GPU payload 비용이다. 학습은 증강으로 geometry가 바뀌므로 plan cache를
우회한다. 캐시 hit의 추론 비용을 학습 비용으로 환산하지 않는다.

같은 24개 장면·같은 증강으로 짧은 실제 학습도 비교했다. 기준 50cm는 위
`train_v2/current075`, 나머지는 `train_depth_v1`이며 모두 24회 optimizer 갱신을 통과했다.

| depth 간격 | 실제 학습 1 step 평균, loader 포함 | 50cm 대비 | steady 학습 allocated peak |
|---|---:|---:|---:|
| 25cm | 471.79ms | +26.4% | 5,867MiB |
| 50cm | 373.39ms | 기준 | 5,754MiB |
| 100cm | 315.53ms | −15.5% | 5,690MiB |

따라서 이번에는 50cm를 기본 유지한다. 25cm는 추론 지연 때문에 버릴 후보는 아니지만
학습 비용을 더 들일 만한 AP 근거가 아직 없다. 100cm도 시간이 빠르다는 것만으로
정확도 손실이 없다고 가정하지 않는다. 이는 5060 Ti·batch 1·16개 측정 step의 짧은
비용 검사이며 장시간 학습·4090 시간·평가지표를 보장하지 않는다.

### 선택과 다음 실행 조건

1. **기본 유지:** 7.5cm voxel / 30cm Camera pooling / 60cm fusion, depth 50cm다.
   현재 semantic ROI·512×1056·Q90·BEV 증강·4클래스와 upright GT 정책을 유지한다.
   기존 값이라는 이유가 아니라 실제 점 보존·밀집 GT·전체 비용을 확인한 뒤 내린 선택이다.
2. **정밀 후보 보류:** 5/20cm는 근거리 사람·작은 장애물의 정보 보존 측면에서 의미 있는
   후보다. 단순한 ‘16채널이므로 더 정밀해도 쓸모없다’는 결론은 내리지 않는다.
   다만 metric receptive field가 작아지고 비용이 늘므로 AP 향상을 보장하지 않는다.
   사용자 승인으로 Vast를 재개한 뒤 최신 소스·새 라벨로 실제 4090 비용을 확인한다.
3. **경량 후보 미승격:** 10/40cm의 약 10% 가속만으로 작은 객체·근접 밀집 위험을
   감수하지 않는다. 실제 시간 예산이 부족할 때 정확도 검증과 함께 재검토할 fallback이다.
4. **Depth 유지:** 더 작은 간격이 정확한 depth를 자동으로 만들지 않고, 100cm의
   작은 비용 절감만으로 원거리 feature 표현을 성기게 하지 않는다. 25cm는 512MiB
   cache를 사용하면 추론 시간 때문에 기각할 후보가 아니다. 다만 본학습 AP는 미검증이고
   학습 초기화의 depth 출력 shape·메모리·재개 계약도 달라져 기본값을 자동 변경하지 않는다.
5. **본학습 전 원격 gate:** 최신 소스 전달 → 원격 기존 JPEG/native RPY로 upright infos
   재변환 → 새 work-dir와 준비 계약 → 실제 4090 비용·짧은 학습·저장/재개 → 외부
   checkpoint 백업 완료 → 사용자 승인 후 본학습이다. 기존 준비 계약에 라벨을 교체해
   이어 붙이거나 대규모 센서 재전송을 하지 않는다.

이번 작업에서 수집기·생성기·원본 5K·Drive·Vast·production E25는 변경하지 않았다.
최종 성능은 학습 후 dev rosbag과 별도 test bag으로 확인한다. 미학습 파라미터의 latency나
24-step 초기 loss를 탐지 성능으로 해석하지 않는다.

### 최종 검증 근거와 재현

ignored 산출물의 공통 경로는 `artifacts/evaluation/bevfusion/base5_grid_depth_261011/`다.

- `information_v1/report.json`, `objects.json`: 실제 5K·11,052 GT의 target/coder/
  raw native-RPY 근거 voxel·클래스·거리별 통계다. candidate config 네 개도 함께 보존한다.
- `augmented_targets_v2/{current075,common050,common075,common100}/report.json`:
  원본 + 실제 BEV 증강의 정답 peak·coder·query grid와 수정된 physical plot extent다.
- `spatial_v2`, `spatial_v3_reverse`: 같은 300장·공유 가중치·반대 측정 순서의 비용과
  CUDA event 하위 span이다. VRAM 최종 판단은 case 간 참조를 정리한 `spatial_v4_final`을 따른다.
- `depth_v1`: 기본 256MiB cache의 25/50/100cm 결과다. `depth_v2_cache512`는
  동일 조건에서 cache budget만 512MiB로 바꾼 대조다. 최종 추론 report의 source guard가
  현재 소스와 일치하고 선택된 300개 센서 파일 SHA256도 전달본 manifest와 같다.
- `train_v2/{case}/cost.json`, `train_depth_v1/depth{0.25,1.0}/cost.json`:
  fresh-process 비용 검사 6개다. 같은 24개 scene ID·증강 GT 해시이며 **144회 모두
  optimizer 갱신**, 모든 loss 유한, 전체 epoch/validation/checkpoint 미실행을 확인했다.
- `pooling_tests_final.json`: CPU 9개 중 CUDA 전용 3개 skip, 나머지 6개 통과;
  CUDA 9개 모두 통과했다. fractional 마지막 열·fresh constructor·forward/gradient·
  cache 무효화·training bypass·nondefault stream을 포함한다.
- `regression_tests_final.json`: rectangular/heatmap/input/image-domain/격자 계약
  33개 모두 통과했다. 총 test 실행 51개 중 48개 통과, 정상 skip 3개다.
- `completion_checks.json`: 최종 소스 guard·같은 학습 scene/증강 GT·144회 optimizer
  갱신·회귀 결과를 다시 확인한 묶음 검사다. 두 실행의 전체 로그도 같은 경로에 보존한다.

`spatial_v1`은 float 격자 문제를 발견해 중단한 진단 이력이다. `train_v1`은 같은
Python 프로세스가 이미 import한 constructor를 쓰던 초기 비용 이력이며, fresh
constructor 검증은 `train_v2`만 따른다. 첫 묶음 실행의 마지막 pool test는 CLI의
필수 `--output` 누락으로 종료됐고, 위 최종 pool/regression 검사를 올바른 인자로 다시
실행해 통과했다. 진단 로그와 실패 실행도 숨기거나 최종 성공 결과로 합산하지 않는다.

재현은 team CUDA128 image의 기존 PYTHONPATH 설정에서 다음 도구를 사용한다.
출력은 매번 새 경로여야 하고, GPU 비교 중 MORAI/다른 학습·추론은 중지한다.

```bash
python perception/detection3d/bevfusion/tools/audit_base5_grid_information.py \
  --data "$UPRIGHT_JPEG_PORTABLE" \
  --config perception/detection3d/bevfusion/configs/bevfusion_base5_dense_512_native_jpeg90.py \
  --output "$NEW_INFORMATION_OUTPUT"

python perception/detection3d/bevfusion/scripts/benchmark_base5_grid_depth.py \
  --data "$UPRIGHT_JPEG_PORTABLE" \
  --selection artifacts/evaluation/bevfusion/base5_all_jpeg_261011/qualification_samples/manifest.json \
  --config perception/detection3d/bevfusion/configs/bevfusion_base5_dense_512_native_jpeg90.py \
  --output "$NEW_COST_OUTPUT" --iterations 300 --stage-iterations 32

python perception/detection3d/bevfusion/scripts/test_pool_optimization.py \
  --output "$NEW_POOL_TEST_REPORT"
```

Depth 대조는 cost 명령에 `--cases current075 --depth-steps .5 .25 1.0`을 붙인다.
512MiB 대조에는 `--pool-cache-mib 512`를 추가한다. 실제 학습 비용은 기존
`benchmark_base5_real_loader.py`에 audit가 출력한 candidate config를 지정하고
`--batch 1 --workers 4 --steps 24 --warmup 8`로 제한한다. full epoch 명령이 아니다.

<a id="base5-vast-grid-qualification-261011"></a>
## 18. 최신 소스·upright 라벨의 실제 4090 검증 — 2026-10-11 KST

사용자가 기존 Vast를 재개한 뒤 Direct SSH로 접속했다. RTX 4090 48GB, Torch
2.7.1+cu128과 기존 5K JPEG·초기화 파일이 보존되어 있었다. 센서는 재전송하지 않았다.
`271a2a6c`의 tracked 실행 소스와 기존 300장 선택 명세만 약 332KiB로 전달했다.
전달 archive SHA256은 `94485abadea5d5d7f8f77f52256ff61284779eac1f1d70d9cf3191b848c1d8ab`다.
원격의 이전 소스를 별도 tar로 보존하고 최신 파일을 반영했다. 원격 `.git`의 revision을
새 checkout으로 바꾼 것은 아니다. 실행 근거는 전달 소스와 각 report/manifest의 파일 해시다.
문서와 측정 산출물의 회수·정리는 로컬에서 수행한다. 본학습은 시작하지 않았다.

### 입력·라벨과 실행 계약

- 현재 원격 입력: `/root/base5_sunny13_jpeg90_upright_v1_261011`이다. 이전 원본과
  yaw-only JPEG 전달본은 유지하고, 센서 hardlink와 새 infos로 별도 파생본을 만들었다.
- 원격 기존 native RPY에서 `native_rpy_fixed_yaw_enclosure_v1`로 전수 변환하고 입력과
  출력의 모든 manifest 파일 SHA256을 검사했다. 512 프로필은 train 4,255 / val 745장,
  GT 9,350 / 1,702개, camera-only 535 / 144개다. 클래스·중심·yaw·분할·센서는 유지했다.
- 로컬 현재 release와 원격의 **train/val infos 내용 SHA256이 각각 완전히 같다**.
  train은 `4e16333f4766b67f06651e2d7a9ae2734175869141ec9d1ecf6b870f9208634e`,
  val은 `93b505edd177d29430b059d71abd73fe83897288530cbd186aec54b5142d3c36`이다.
- 초기화 파일 SHA256은 `d06ae54d380a7c0ec245936ff394a51e0ff2c36fd1288eb2d7879954c682107b`다.
  감사한 nuScenes camera-only 초기화이며 새 LiDAR·depth·4-class head는 학습해야 한다.
  미학습 부분의 missing-key 경고를 전체 모델 가중치 로딩 실패로 해석하지 않는다.
- 유효 반사점 필터·uint8 운반·heatmap `[Y,X]` 정답·분리 pooling·정수 격자 보완을
  모두 반영했다. Q90, 512×1056, 4클래스, single-frame 7D, temporal/OD velocity OFF,
  depth 50cm와 semantic ROI X±100/Y±30m는 유지했다.

### 실제 300장 추론 비용과 반복 확인

200장 대표 표본 + 100장 stress의 동일한 고유 300장을 사용했다. 선택된 센서 SHA256을
확인하고 warmup 16회 뒤 batch 1 AMP의 비계측 latency를 측정했다. 별도의 32회에서
CUDA event 하위 span을 수집했다. 비교용 390개 호환 파라미터는 동일하며 비용 검사용
가중치다. 학습된 정확도나 실제 ROS/full-stack 전달 시간을 측정한 것이 아니다.
service는 메모리에 있는 센서의 CPU 전처리 + 모델 + decoded CPU transfer이고,
디스크 읽기·네트워크·ROS 큐 대기는 제외한다.

| 공간 격자 | 모델 평균 | service 평균 | service p95 | steady allocated peak |
|---|---:|---:|---:|---:|
| 기본 7.5/30cm, 기존 padding | 49.07ms | 68.20ms | 70.64ms | 1,022MiB |
| 7.5/30cm, 비교 공통 padding | 49.56ms | 68.57ms | 72.50ms | 1,022MiB |
| 정밀 5/20cm, 비교 공통 padding | 58.47ms | 78.42ms | 83.57ms | 1,427MiB |

순서를 뒤집어 같은 300장씩 다시 측정했다. 정밀은 모델 59.26 / service 78.74ms,
기본은 모델 50.60 / service 70.09ms였다. 이 반복의 service p95는 각각 88.46 /
77.68ms, p99는 116.26 / 99.47ms다. 첫 실행의 낮은 p95만으로 모든 프레임이
13Hz 주기 76.92ms를 만족한다고 보장하지 않는다. 평균 격자 비용 차이는 반복에서도 유지됐다.
원격 CPU 전처리는 대략 19~20ms로, 로컬의 CPU 비용과 같은 값이라고 가정하지 않는다.

정밀 격자의 dense fusion cell 수는 공통 기준 대비 2.25배다. 하지만 이미지 입력과
image backbone은 같고 sparse LiDAR는 빈 모든 voxel을 dense 연산하지 않는다.
별도 계측에서 camera backbone은 약 14.1ms로 같고, sparse encoder도 약 16ms였다.
주된 증가 부분은 camera BEV downsample 0.94→2.25ms, fusion 0.63→1.56ms,
BEV backbone 1.32→2.77ms, neck 0.23→0.69ms, head/decode 5.14→8.06ms다.
event span은 host idle을 포함할 수 있고 view 내부 span이 중첩되므로 단순 합산하지 않는다.
추론 warmup의 cuDNN workspace 탐색 peak는 32.9~36.1GiB였다. 이를 steady 모델
메모리나 24GB GPU에서의 필수 VRAM으로 해석하지 않으며, 실제 대회 GPU의 cold start는 별도 gate다.

### 실제 5K 로더·증강·역전파 비용

각 후보는 fresh process로 batch 4 / workers 4 / AMP AdamW 64회만 갱신했다.
warmup 16회를 제외한 48회, 실제 192개 고유 장면의 비용이다. 전체 사용 장면은
각 256개이며 수집한 세 report의 64개 batch 장면 순서·증강 GT 해시는 모두 정확히
같았다. 전체 epoch나 수렴 성능을 측정한 것이 아니고, batch 8의 이점을 새로 주장하지 않는다.

| 공간 격자 | loader 포함 평균 / step | steady allocated peak | optimizer 갱신 |
|---|---:|---:|---:|
| 기본 7.5/30cm | 583.82ms | 21,130MiB | 64/64 |
| 공통 padding 7.5/30cm | 579.89ms | 21,340MiB | 64/64 |
| 공통 padding 5/20cm | 703.02ms | 28,679MiB | 64/64 |

정밀 격자는 공통 기준 대비 약 21.2% 느리고 메모리는 약 34.4% 늘었다. 두 후보 모두
현재 임대한 48GB GPU에서 batch 4가 가능하다. 기본 결과를 24GB에 그대로 적용하거나,
정밀 batch 4가 24GB에도 들어간다고 가정하지 않는다. 모든 loss는 유한하고 AMP skip 없이
optimizer가 갱신됐다. 초기 loss 크기를 서로 비교해 최종 탐지 성능을 판정하지 않는다.

기본 설정의 745장 전체 validation은 FP32로 63.14초였다. 1,702 GT의 클래스별 개수는
vehicle 341 / pedestrian 653 / obstacle 674 / animal 34다. 출력과 metric의 유한성·
4클래스 범위를 검사했다. 4-iteration smoke checkpoint의 AP는 품질 평가가 아니다.
기본 24 epoch는 1,064 updates/epoch × 24이며, 짧은 train 평균을 외삽하면 순수
train 약 4.14시간 + 매 epoch validation 약 0.42시간이다. 저장·초기화·긴 실행 변동을
포함한 운영 예산은 약 4.7~5.3시간으로 잡는다. 정밀은 train만 약 4.99시간이다.
정밀 745장 full validation은 FP32로 72.65초였고 기본과 같은 GT 개수·출력/metric 검사를
통과했다. 이를 매 epoch 수행하면 validation 약 0.48시간, train과 합쳐 약 5.47시간이다.
전체 운영 예산은 약 5.5~6.2시간으로 잡되 장시간 실측 전에는 확정 시간으로 제시하지 않는다.

### 저장·재개와 외부 백업은 별개의 gate다

기본 설정은 실제 두 train 장면의 네 클래스로 AMP 2회 → atomic epoch 저장 →
의도적 중단 → epoch 경계 재개 → 누적 4회 갱신을 통과했다. optimizer·scheduler·
scaler·주 프로세스 RNG와 계약을 복원하고 partial checkpoint가 없음을 검사했다.
정확한 mid-epoch worker/prefetch replay를 보장하는 기능은 아니다.
정밀 격자도 같은 두 장면·4클래스로 별도의 2→4 iteration 저장·재개를 통과했다.
세 비용 실행의 192회와 두 smoke의 8회, 총 200회 optimizer 갱신을 확인했다.

최신 기본 smoke의 full checkpoint 498,641,221 bytes를 access-only 메모리 인증으로
Drive에 백업하는 검사를 **180초만** 수행했다. preflight는 성공했지만 본 백업 과정이
기한 내 완료되지 않아 자식 전송을 종료했다. SHA256 read-back·COMPLETE·LATEST·
중복 재실행 검사를 통과한 report는 없으며 **외부 백업 gate는 미완료**다. 원격 원본은
보존하고 반복 재전송하지 않았다. 이후 로컬의 metadata-only 목록에는 이전
`9fb2bc...` checkpoint만 보였고 최신 `860009...` content-addressed payload는 없었다.
이를 최신 checkpoint 업로드 성공으로 해석하지 않는다. timeout만으로 API quota나
정확한 네트워크 병목을 확정하지도 않는다.
종료 후 인증 없는 읽기 요청으로 API 연결만 검사했을 때 Drive 조회 endpoint는 3.17초에
401, upload endpoint의 GET은 1.50초에 405, OAuth endpoint는 1.64초에 404를 반환했다.
인증 없이 확인한 정상 HTTP 오류 응답이며 인증·업로드 성공을 뜻하지 않는다. 당시
DNS/TCP/TLS 전체 단절은 아니었다. 대용량 업로드의 지속 속도나 재시도 원인은 미확정이다.

기본 최신 checkpoint SHA256은
`860009380c0abbf20bc6e3ad6cfdfa129e3ca98fc929609c03dfabf3257ebdbe`다.
원격 원본은 `/root/base5_grid_qualification_271a2a6c_261011/smoke/training/epoch_2.pth`에 있다.
큰 재전송·장시간 재시도·인스턴스 교체를 자동 수행하지 않는다. 본학습은 별도 승인을 받고,
외부 백업 성공 또는 사용자가 명시적으로 선택한 대체 보존 조건을 확인한 뒤 시작한다.

### 선택과 본학습 준비

이번 단계의 추천은 **512×1056 / 7.5cm voxel / 30cm Camera BEV / depth 50cm /
batch 4**를 기본으로 유지하는 것이다. 정밀 5/20cm는 충분히 실행 가능한 후보다.
다만 현재 비용은 더 들고 AP 우위는 아직 없으므로, 학습 기회가 제한된 첫 임시 모델을
비용 측정만으로 정밀 격자에 고정하지 않는다. 정밀 후보를 성능 한계 때문에 영구 기각하지 않는다.
공통 padding은 비교용이며 기본 semantic ROI나 기본 config를 자동 확대하지 않았다.

새 main prepare-only 폴더는
`/root/base5_training_sunny13_512_b4_e24_upright_271a2a6c_261011`이다.
계약 SHA256은 `22db7bc4afe39205e63181e7733eb9e25f6fa2561bc6b6dd58dba7ca88b3c357`이며,
24 epoch / batch 4 / workers 4 / LR 1e-4 / warmup 500 updates / cosine / 매 epoch
val·전체 재개 checkpoint / 최근 3개 보존을 준비했다. `fresh_start.json`, `last_checkpoint`,
학습 checkpoint가 없는 prepare-only 상태를 확인했다. 예전 실행 계약을 덮어쓰지 않았다.

재개 명령의 데이터·config·초기화·schedule·source hash를 바꾸지 않는다. 단일 Vast
컨테이너의 Python을 직접 사용하며 Docker를 중첩하지 않는다. 본학습 개시 전 백업 watcher의
인증/지속 실행을 확인한다. 현 access-only 방식은 로컬 refresh watcher가 살아 있어야 하며,
로컬 PC가 꺼져도 자동 백업된다고 주장하지 않는다. 이번 검사의 watcher는 종료했다.

로컬 ignored 근거 경로는 `artifacts/evaluation/bevfusion/base5_vast_grid_261011/`이고,
원격은 `/root/base5_grid_qualification_271a2a6c_261011/`이다. `inference`와
`inference_reverse` report, `train_{current075,common075,common050}_b4/cost.json`,
`smoke/report.json`, `validation/validation_cost.json`, `pool_tests.json`,
`grid_tests.log`, `regression_tests.log`, `external_backup.jsonl`과 실행 로그를 보존한다.
`smoke_fine/report.json`, `validation_fine/validation_cost.json`,
`main_preparation/{run_manifest.json,resolved_train.py}`도 회수했다.
최종 검사에서 원격 main manifest의 실행 소스 **980개가 로컬 내용 SHA256과 모두 같고**,
두 추론 report의 source guard도 현재 소스와 일치했다. 순서 역전 추론·세 비용 실행의
같은 장면/증강·두 저장/재개·두 전체 val 검사를 완료했다. 격자 계약 3개, pipeline/
heatmap/rectangular/image-domain 회귀 30개, pool CPU 9개 중 6개 통과·CUDA 전용 3개 skip,
pool CUDA 9개 모두 통과했다. 총 51개 실행 중 48개 통과, 정상 skip 3개다.
2026-10-11 KST 최종 확인에서 GPU 0%·1MiB, 학습/추론/rclone 자식 없음과 main
 prepare-only 상태를 확인했다. 인스턴스를 자동 중단·destroy하지 않았다.

<a id="base5-vast-depth-z-qualification-261011"></a>

## 19. 새 4090 이전·depth 간격과 LiDAR 높이 검증 — 2026-10-11 KST

사용자가 기존 인스턴스 55165763을 중단한 뒤 GPU가 다른 사용자에게 할당되어 재개하지
못했다. 사용자가 새 인스턴스 55272391을 임대하고 Vast Copy로 5K와 초기화 가중치를
복사했다. 새 환경은 host 124072 / machine 38442의 RTX 4090 48GB다. 기존 인스턴스의
데이터를 삭제하거나 새 인스턴스를 자동 중단하지 않았다. 아래 기록은 이전 18절의
`/root` 경로와 새 `/workspace` 경로를 구분한다. 본학습은 시작하지 않았다.

### 복사 검증과 읽기 오류 복구

호스트 측 복사가 진행되는 동안 일부 파일을 읽을 수 있었지만 `infos/512x1056`는
`Stale file handle` 또는 `ENOENT`였다. 복사 완료처럼 보인다는 것만으로 실행하지 않았다.
실행 중 overlay와 호스트 측 복사의 상호작용이 의심되나 정확한 원인은 확정하지 않았다.
원본 복사 폴더는 보존하고 컨테이너 내부에서 별도 검증본을 만들었다. 센서를 인터넷으로
다시 전송하지 않았으며, 동일 해시의 512 프로필 metadata 네 파일만 로컬에서 보충했다.
hardlink 시도도 실패하여 해당 부분 결과를 보존하고 일반 파일 복사 방식으로 완료했다.

- 실행 입력: `/workspace/base5_sunny13_jpeg90_verified_v2_261011`.
- 파일 173,294개 / payload 10,471,712,458 bytes를 **전체 SHA256 검증**했다.
- 원격 portable manifest SHA256은
  `34f3560de16370a763607d68180d35268b159f3f87d8d88f1f217a250da26ce1`이다.
  로컬 release의 manifest SHA256 `47f208b...`와는 provenance가 다르다.
  train/val infos 해시는 18절의 로컬 release와 각각 같고 클래스·센서·split은 바꾸지 않았다.
  provenance를 제외한 전체 파일명·bytes·SHA256 목록의 정규화 digest는 양쪽 모두
  `a6dc7d0dd4d239bf5ca20292b574fecbde5c13fa9ded7f86f5eab63244315766`으로 같다.
- train 4,255 / val 745, all-JPEG Q90, upright native-RPY 정책을 유지했다.
- 초기화 파일 `/workspace/base5_nuscenes_camera_only_261010.pth`의 SHA256도
  `d06ae54d380a7c0ec245936ff394a51e0ff2c36fd1288eb2d7879954c682107b`로 확인했다.
- `repair_base5_portable_mirror.py`는 fresh output만 허용하고 지정한 metadata 외의
  replacement를 사용하지 않는다. 잘못된 replacement 해시는 실패한다. 원본을 수정하지 않는다.

### 실제 5K의 높이·voxel 제한·Camera depth 점검

로컬 현재 release의 train+val 5,000장, GT 11,052개를 읽기 전용 검사했다. 센서 유효점
필터와 semantic XY 범위를 적용하고, upright enclosure가 아니라 native full-RPY 박스
내부 점을 계산했다. 지면 접지 인증이나 학습 AP 검사가 아니다.

| 검사 | 관측 | 판단 |
|---|---|---|
| 현재 LiDAR Z −3~2m 필터 | 객체 내부 점 1,066,389개 중 667개 제거, 약 0.0625%; 46개 차량에 영향 | 명목 배치에서 전체 점 또는 절반 이상을 잃은 객체 없음 |
| 내부 voxel 공간 Z −5~3m까지 입력 확대 | 위 667점을 복구하지만 전체 입력점도 584,789개 추가 | 대부분 배경 증가이므로 필터 확대 근거 부족 |
| 보행자·장애물·동물의 명목 Z 손실 | 세 클래스 모두 0점 | 기존 높이 필터 유지 |
| Z-only 증강 16회/프레임 | 176,832 객체 관측 중 전체 점 손실 5회, 모두 점 4~6개의 큰 차량 | 현재 증강을 전면 축소할 근거 부족; 전체 pipeline/AP 결과는 아님 |
| Z voxel 10/20/40cm 양자화 | 객체 occupied voxel 647,471 / 619,897 / 567,768 | 10cm가 새 센서 정보를 만드는 것은 아님; backbone 변경 비용 없이 정보량만 비교 |
| voxel/point cap | 프레임 최대 원시점 19,200, occupied voxel 10,720 | train/test voxel cap 120,000/160,000에 여유 있음 |
| voxel당 10점 제한 | 넘치는 점 25,585 / 필터 후 45,291,797개, 약 0.0565% | cap 증설 필요성이 입증되지 않음 |
| 현재 crop에 중심이 투영되는 optical depth | 13,769 관측, 최대 98.27m; camera-only 중 60m 이상 29개, 80m 이상 21개 | depth 상한을 60/80m로 줄이지 않고 100m 유지 |

위 증강 검사는 실제 RTS 순서 `(z + translation_z) * scale`을 사용하되 native XY
지지는 고정한 Z-only probe다. 실제 증강 전체의 XY/crop/GT keep을 재현한 결과처럼
해석하지 않는다. 명목 camera-only 679개는 원래 native 박스 내부 LiDAR 점이 없는
객체이며, Z 필터가 새로 camera-only로 만든 것이 아니다.

camera-only 중 모든 투영 가능 중심이 optical depth 1m 미만인 객체 62개가 있으나,
해당 시점의 native 8모서리를 검사하면 **박스 전체가 1m 아래인 사례는 0개**다.
중심이 가까운 것만으로 카메라 feature를 전혀 받지 못한다고 결론 내리지 않는다.
이는 기하학적 투영 검사이며 실제 가림·픽셀 가시성·학습된 depth 정밀도를 보장하지 않는다.
현재 depth `[1,100)`m, Z voxel 20cm, 입력 Z 필터 −3~2m와 내부 grid −5~3m를 유지한다.
Z voxel 10/40cm는 이번에 실제 네트워크 비용 후보로 승격하지 않았다. 현재 sparse encoder는
마지막 Z 방향 convolution 뒤 `C*D`로 높이를 채널에 합친다. Z 간격만 바꾸면 마지막
높이 차원과 fusion 입력 채널까지 달라질 수 있으므로 단순 config 한 줄의 동등 비교가
아니다. 정보량 점검에서 변경의 시급성이 입증되지 않은 상태에서 backbone까지 재설계하고
초기 loss로 우열을 정하지 않는다. 필요하면 본학습 후 별도 정확도 비교로 검토한다.

### 원격 비용 및 본학습 준비 상태

Torch 2.7.1+cu128 / numpy 1.26.4 / MMCV 2.1.0 / MMEngine 0.10.7 / MMDet 3.2.0 /
MMDet3D 1.4.0 / spconv-cu120 2.3.6으로 실행했다. 새 GPU용 MMCV·native ops를 빌드하고
CUDA NMS와 모델 import를 통과했다. 설치 다운로드·Matplotlib 빌드·첫 pooling 커널 빌드
시간은 steady latency에 합치지 않는다. 선택한 headless 모델 경로에서 사용하지 않는
open3d·plyfile·scikit-image·tensorboard의 미설치 경고는 별도로 남았다. GUI 시각화까지
완비한 환경이라고 주장하지 않으며 실제 train/val 경로 검증을 우선한다.

동일한 200장 대표 + 100장 stress 표본, 이미지 512×1056, 7.5/30cm 공간 격자와
depth 범위 `[1,100)`m를 고정했다. 아래 추론은 각 300장 AMP batch 1의 첫 측정이며,
service는 CPU 전처리·모델·CPU 예측 회수이고 disk/network/ROS queue는 제외한다.
호환 파라미터를 공유한 비용 검사이며 미학습 정확도를 비교한 것이 아니다.

| depth 간격 | bin 수 | 모델 평균 | service 평균 / p95 | steady allocated | pooling plan |
|---|---:|---:|---:|---:|---:|
| 25cm | 396 | 49.45ms | 68.71 / 70.33ms | 1,195MiB | 308.8MiB |
| 50cm | 198 | 48.64ms | 67.49 / 68.85ms | 1,022MiB | 154.9MiB |
| 100cm | 99 | 48.95ms | 68.04 / 69.43ms | 935MiB | 78.0MiB |

비교용 cache 예산을 모두 512MiB로 맞춰 각 후보 1회 miss / 347회 hit / 0 bypass를
확인했다. 25cm가 기존 256MiB에 들어가지 않는 비용을 depth 연산 자체의 증가로
해석하지 않는다. 기본 50cm의 기존 256MiB 예산은 유지하며 비교 예산을 기본값으로
승격하지 않았다. 별도 32회 CUDA event span에서 depth feature는 50→25cm일 때
2.53→2.90ms, pooling은 0.69→0.88ms였다. camera backbone 약 14.1ms와 sparse encoder
약 15.7~15.9ms는 그대로다. 중첩 span을 합산해 전체 모델 시간으로 해석하지 않는다.

각 후보의 실제 5K train 로더는 fresh process로 batch 4 / workers 4 / AMP AdamW
64회만 갱신했다. 16회 warmup 뒤 48회, 고유 192장 측정이며 전체 사용 장면은 256장이다.
세 실행의 64개 batch 장면 순서와 증강 GT 해시, 실행 소스 985개가 각각 동일했다.
loss 유한성·optimizer 64/64 갱신을 검사했다. 64-step loss로 AP 우열을 정하지 않는다.

| depth 간격 | loader 포함 평균 / step | 50cm 대비 | steady allocated peak |
|---|---:|---:|---:|
| 25cm | 684.77ms | +20.1% | 21,563MiB |
| 50cm | 570.10ms | 기준 | 21,131MiB |
| 100cm | 522.77ms | −8.3% | 20,913MiB |

50cm를 기본으로 유지한다. 25cm는 더 정밀한 depth 확률 격자일 뿐 실제 거리 정확도가
25cm가 되는 것이 아니며, 학습 비용 증가 대비 AP 이점이 아직 없다. 100cm의 비용 절감은
관측했지만 근거리·원거리 위치 정확도에 미치는 영향을 검증하지 않아 자동 채택하지 않는다.
입력 range·공간 격자·증강·label을 동시에 바꾸지 않는다.

100→25→50cm 순서로 같은 300장씩 다시 측정했다. 모델/service 평균은 각각
49.06/67.74ms, 50.01/68.75ms, 49.54/68.34ms였다. 약 1ms 안팎의 변동보다 작은
차이로 100cm가 반드시 더 빠르다고 단정하지 않는다. 세 후보의 AP는 아직 미측정이다.

기본 50cm는 추가 fresh process에서 CUDA allocator를 GPU 총 메모리의 42.5%로
제한했다. 48GB 카드에서 약 20.4GiB 예산으로, 일반 24GB 카드의 85% 예산을 흉내 낸
검사이지 실제 다른 카드 실측이 아니다. 300장과 32회 계측을 OOM 없이 통과했고,
모델/service/p95는 49.16/68.34/69.71ms였다. cold warmup peak 18,756MiB,
steady allocated 1,022MiB였다. 큰 예산 실행에서 관측한 33~36GiB cuDNN 탐색 peak를
기본 추론에 반드시 필요한 메모리라고 해석하지 않는다. 실제 대회 24GB 카드·ROS/full-stack
gate는 별도로 남긴다. `benchmark_base5_grid_depth.py`의 새 `--memory-fraction`은
이 검사만 제어하며 기본값 0.85는 유지한다. 세 짧은 train 실행을 모두 종료한 뒤
해당 CLI를 추가했으며 모델·학습 경로는 바꾸지 않았다.

새 환경의 pooling 검사는 CPU 9개 중 6개 통과·CUDA 전용 3개 정상 skip, CUDA 9개 모두
통과했다. 입력/정답 축/image-domain 회귀 30개와 격자 계약 3개도 통과했다. 새 높이 감사·
파일 복구·access-only 경로/인증 redaction CPU 검사 9개를 별도로 통과했다.

기본 설정의 두 실제 train 장면·4클래스로 AMP 2 iteration → atomic epoch 저장 →
의도적 중단 → epoch 경계 resume → 누적 4 iteration을 통과했다. optimizer·scheduler·
scaler·주 프로세스 RNG를 복원하고 partial 파일 없음과 유한 가중치를 확인했다.
정확한 mid-epoch worker/prefetch replay 보장은 아니다. 최종 smoke checkpoint는
498,641,221 bytes이며 SHA256은
`cc150a22861a1603abe2fe6aedd22f73552750eb4cb7f22fc0ff091b8b440973`이다.
원본은 `/workspace/base5_depth_qualification_55272391_261011/smoke/training/epoch_2.pth`에
보존했다. 745장 전체 FP32 validation은 62.67초였고 1,702 GT와 출력/metric 유한성을
검사했다. smoke 모델의 AP를 학습 완료 성능으로 제시하지 않는다.

새 main prepare-only 폴더는
`/workspace/base5_training_sunny13_512_b4_e24_55272391_261011`이다. 계약 SHA256은
`ebb21d252f6f8d9e987210684f0a26f4623a4983ed2783a2f23797eb1cd86a94`다.
512×1056 / 7.5·30cm / depth 50cm / 24 epoch / batch 4 / workers 4 / LR 1e-4 /
warmup 500 updates / cosine / 매 epoch val·full checkpoint / 최근 3개 보존을 준비했다.
본학습 실행은 하지 않았다. 순수 train 약 4.04시간 + 매 epoch val 약 0.42시간이며,
초기화·저장·장기 변동까지 포함한 운영 예산은 **약 4.6~5.2시간**이다. 단기 측정 외삽이지
24 epoch 완료 실측이 아니다. 이전 호스트 수치를 새 호스트 실측으로 재사용하지 않는다.

최종 main 계약의 실행 소스 985개를 로컬 내용 SHA256과 대조해 모두 일치함을 확인했다.
`fresh_start.json`, `last_checkpoint`, main checkpoint가 없는 prepare-only 상태를 확인했다.

새 호스트의 full checkpoint 외부 백업도 access-only 메모리 인증으로 **180초만** 확인했다.
읽기 전용 preflight는 통과했지만 기한 안에 백업·SHA256 재읽기·marker 완료 증거가 나오지
않아 전송 자식에 SIGTERM을 보냈다. `backup_failed`, returncode −15이며 `errors: []`는
전송 성공이 아니라 종료 시점에 완료 status가 없었다는 뜻이다. checkpoint 원본은
보존했고 장시간 반복 전송이나 원격 renewable 인증 저장은 하지 않았다. 정확한 업로드/
재읽기 병목은 미확정이며 외부 백업 gate는 미완료다. 성공한 백업 또는 사용자가 명시적으로
선택한 대체 보존 조건과 본학습 승인을 확인한 뒤 시작한다.
종료 후 로컬의 읽기 전용 metadata 목록 검사에서는 위 checkpoint hash 폴더가 빈 목록을
반환했다. 완료된 payload가 확인되지 않았으므로 이번 timeout을 SHA 재읽기만의 병목으로
확정하지 않는다. 최종 GPU는 0%·1MiB였고 학습/추론/rclone 자식과 자동 watcher가 없음을
확인했다. 인스턴스 Stop/destroy는 수행하지 않았다.
근거 경로는 로컬 `artifacts/evaluation/bevfusion/base5_vast_depth_z_261011/`이며,
높이 감사의 최신 결과는 `vertical_caps_depth_final_v2/`다.

<a id="base5-checkpoint-parallel-backup-261011"></a>

## 20. 체크포인트 병렬 백업과 실제 전송 관측 — 2026-10-11 KST

사용자가 Drive의 checkpoint hash 폴더가 비어 있음을 확인하고 백업 업로드 해결을 요청했다.
19절의 180초 제한 로그에는 경과 시간만 있었으므로 업로드와 SHA 재읽기를 구분할 수 없었다.
이를 실제 전송량·속도·오류·단계가 보이는 JSON 이벤트로 보완했다. rclone의 원문 URL과
인증 정보를 출력하지 않고, 자식 출력 PIPE를 실행 중에도 비워 로그 누적으로 인한 정지를 방지한다.

새 단일 파일 시험은 8MiB upload chunk에서도 약 100초에 8MB 미만 전송했다.
관측 속도는 약 0.1MB/s였으며 직접 중단했다. Vast→로컬 Direct SSH의 별도 8MiB 표본도
30초에 458,752 bytes만 받아 종료됐다. 두 경로 모두 느렸다는 관측이지, 집 Wi-Fi나
Vast 호스트의 정확한 제한 원인을 확정한 것은 아니다. 원본 checkpoint는 변경하지 않았다.

### 병렬 저장 및 복원 계약

- `--part-mib 32 --transfers 16`으로 checkpoint를 **무손실 조각**으로 저장한다. 수치 정밀도,
  optimizer, scheduler, AMP scaler, RNG를 버리는 압축이나 모델 경량화가 아니다.
- 먼저 조각을 병렬 업로드하고 다시 병렬 다운로드한다. 각 조각 SHA256과 순서대로 결합한
  전체 SHA256이 원본과 같아야 성공한다. 최종 구현은 실제 `.pth` 복원·metadata 재검사도 한다.
- `PARTS.json`에 순서·크기·해시와 조각 디렉터리를 기록하고 `RESTORE.py`를 함께 올린다.
  기존 파일 덮어쓰기는 금지하며 손상·누락·경로 이탈은 복원 실패로 처리한다.
- 후속 저장은 `parts_<조각 bytes>` 디렉터리를 사용한다. 조각 크기를 바꿔 재시도해도
  이전에 완료된 layout의 조각을 업로드 도중 덮어쓰지 않는다.
- payload·복원 정보·학습 metadata 검증 후에만 `COMPLETE.json`, 마지막에 `LATEST.json`을 쓴다.
  빈 hash 폴더나 일부 조각만 존재하는 것은 백업 완료가 아니다.
- 원격에는 단기 access token만 anonymous RAM으로 전달한다. 갱신 토큰과 client secret은
  로컬에 남으며, 자동 watcher는 로컬 PC와 프로세스가 살아 있어야 한다.

Drive 폴더를 내려받은 뒤 원본 checkpoint를 복원하는 명령은 다음과 같다.

```bash
python RESTORE.py --folder /path/to/downloaded/hash_folder --output /path/to/fresh/epoch_2.pth
```

이번 시험 폴더는
`model/bevfusion/base5/checkpoint_backups/external_backup_probe_parts_261011/cc150a22861a1603abe2fe6aedd22f73552750eb4cb7f22fc0ff091b8b440973`이다.
업로드 도중 약 1.2~1.7MB/s를 관측했으며, 최종 완료 여부와 시간은 아래 실제 결과로 확정한다.
이 시험에서 사용한 조각 폴더 이름은 최초 구현의 `parts/`이며 복원기는 해당 형식도 지원한다.
본학습은 시작하지 않았다. 최종 로그는 `external_backup_parts.jsonl`에 보존한다.

### 실제 payload와 복원 검증

32MiB 조각 15개, 총 498,641,221 bytes를 업로드했다. payload 업로드 완료는 약
362.72초, 다시 내려받는 과정은 약 316.51초였다. 전송 오류는 두 단계 모두 0건이었다.
다운로드된 조각을 순서대로 읽은 전체 SHA256은 원본과 동일한
`cc150a22861a1603abe2fe6aedd22f73552750eb4cb7f22fc0ff091b8b440973`이다.

별도 복원 검사기는 uploader가 임시 read-back 조각을 삭제하기 전에 해당 조각을 보존하고
실제 `epoch_2.pth`를 원자적으로 복원했다. 원본 `.pth`를 복사해 복원으로 가장한 검사가 아니다.
복원 파일은 다시 SHA256과 `torch.load(map_location='cpu')`를 통과했고 optimizer,
scheduler, AMP scaler, RNG 및 epoch 2 / iteration 4를 확인했다.
복원 checkpoint의 원래 smoke contract는
`0390bedcc8e6307504ba9a696f6685ddd8e52391bcea4fb4746166ff8e48e9d3`이다.
복원 경로는 `/workspace/base5_depth_qualification_55272391_261011/smoke/backup_restore_qualification_261011/epoch_2.pth`,
로컬 근거는 `backup_restore_report.json`이다. 이것은 4-update 기능 검사용 checkpoint이며
본학습 완료 모델이 아니다. 별도 CUDA 추론·학습을 추가 실행하지 않았다.

백업 helper 변경도 training source hash 범위에 들어가므로 기존 준비 경로를 덮어쓰지 않고
`/workspace/base5_training_sunny13_512_b4_e24_55272391_backup_v3_261011`에 다시 **준비만** 했다.
새 contract는 `f968a33da5d2454afd4d8f17b875a091d7faad5b4984c583dfc39770f4d27e47`이다.
987개 소스 hash가 로컬과 일치하고, 이전 identity에서 바뀐 key는 `source_hashes`뿐이다.
입력·모델·초기화·split·epoch·batch·증강·학습률은 동일하며 본학습은 여전히 미시작이다.

### 최종 완료 상태

전체 처리 834.05초, 약 **13분 54초**에 외부 백업을 완료했다. `COMPLETE.json`과
부모의 `LATEST.json`까지 업로드·SHA 재읽기를 통과했고, 같은 상태로 다시 검사해
중복 업로드가 없음을 확인했다. `external_backup_report.json`은 `valid=true`,
`duplicate_recheck_passed=true`, `uploaded=1`, `errors=[]`다.
로컬의 별도 읽기 전용 Drive 목록에서도 조각 폴더·`PARTS.json`·`RESTORE.py`·학습
metadata·`COMPLETE.json`을 확인했다. [완료된 hash 폴더](https://drive.google.com/drive/folders/1XGPBc0bWYPD_VgFUQOnxjDY5XWnOEKGE)를
사용하며, 이전 실패한 빈 hash 폴더는 다른 부모 아래의 이력이다.
19절의 외부 백업 미완료 gate는 이 실제 결과로 해소한다. 본학습 시작은 여전히 별도
사용자 승인이 필요하다. 인스턴스 Stop/destroy나 상주 자동 watcher 시작은 수행하지 않는다.
실제 `--watch --max-polls 1` 경로도 약 3.87초에 재검사했고 추가 payload 업로드 0건,
`valid=true`, 오류 0건이었다. watch를 무한 실행하지 않았으며 종료 후 GPU 학습·추론과
백업 자식 프로세스가 남아 있지 않음을 확인했다. backup/parts/credential 검사 11개를 통과했다.

<a id="base5-training-runtime-local-261011"></a>

## 21. 로컬 학습 실행 비용 개선 — 2026-10-11 KST

사용자는 Vast를 중단하고 로컬에서 학습 비용의 병목과 개선 후보를 구현·실험하도록
요청했다. 이 작업에서는 Vast 접속·상태 변경·본학습·이미지 게시를 수행하지 않는다.
실험·최종 기록 구간은 2026-10-11 약 06:49~07:54 KST다.
기존 `ajounice/bevfusion:team-cu128-v1` 환경과 RTX 5060 Ti 16GB를 사용한다.
CUDA·의존성을 다시 설치하지 않는다. 컨테이너 로그 시각은 UTC이고 이 문서는 KST다.

### 조건과 비교 방법

- 실제 upright native-JPEG Sunny13 train 4,255장의 로더를 사용한다. 512×1056,
  공간 7.5/30cm, 원래 증강·AMP·AdamW·clip·스케줄·초기화 정책을 유지한다.
- 로컬 비교는 물리 batch 2, workers 4다. 각 실행은 같은 camera-only 초기화에서 시작하며
  64 step 중 앞 16 step을 제외한다. 한 실행의 입력은 서로 다른 128장이다.
- depth 25cm 최종 대조는 `baseline → geometry → geometry → baseline` 순서다.
  실제 초기 파라미터 SHA256, 첫 이미지·점군 입력 SHA256, 모든 step의 장면 순서와
  증강 GT·LiDAR 변환 SHA256이 같다. 짧은 loss를 AP 우위의 근거로 사용하지 않는다.
- 표의 step은 로더 대기와 GPU 완료 확인을 포함한 벽시계 시간이다. epoch validation,
  checkpoint·외부 업로드 비용과 첫 컴파일 비용은 정상 step 평균에 포함하지 않는다.
  CUDA profiler의 CUPTI가 이 Blackwell 장치에서 kernel trace를 제공하지 못하므로
  별도 CPU trace·CUDA event 구간을 보조 근거로 사용한다. 중첩된 event 구간을 합산하거나
  CPU의 대기 시간을 해당 연산의 GPU 자체 비용으로 해석하지 않는다.

### 발견과 선택

| 후보 | 발견·검증 결과 | 선택 |
|---|---|---|
| 데이터 읽기·전처리 | 첫 진단의 로더 대기 평균 약 5.25ms, 전체 약 735.92ms다. 기존 uint8·JPEG·workers 경로가 주병목은 아니다. | 증강이나 데이터를 줄이지 않는다. |
| pooling 역전파 warp 병렬화 | 한 thread가 80채널을 순회하던 부분을 warp의 인접 lane에 나눠 coalesced 접근을 사용한다. 동일 목적함수이며 FP32 합산 순서만 달라질 수 있다. | opt-in 후보에 포함한다. |
| int32 좌표·정렬 키 | 기존 truncation·rank 순서를 유지하고, int32 범위 초과는 거부한다. | opt-in 후보에 포함한다. |
| loss 로그 일괄 읽기·fused AdamW | scalar 로그의 CPU 읽기를 합치고 optimizer 실행을 융합한다. loss tensor·gradient·학습률·weight decay는 유지한다. 단독 fused 이득은 작다. | 단일 GPU 후보에 포함한다. DDP 로그는 기존 구현으로 돌아간다. |
| feature별 grouping 역전파 | atomics는 없앨 수 있지만 추가 정렬 비용이 든다. 50cm 비교에서 warp 묶음 697.02ms보다 grouping 묶음 703.62ms가 느렸다. | 기본 후보에서는 제외하고 실험 옵션만 보존한다. |
| cuDNN autotune | grouping 묶음 703.62→703.77ms로 이득이 없었다. | 이번 후보의 설정을 바꾸지 않는다. |
| 기하 계산 BMM | 원래는 작은 3×3 행렬을 수많은 깊이 샘플에 broadcast하여 복제한다. 카메라별 큰 점 행렬에 BMM을 적용하면 반복 복제 비용이 줄어든다. | 아래 경계 보완을 통과한 `batched_checked`만 사용한다. |

### 빠른 기하 계산의 경계 보완

보완 없는 BMM은 FP32 계산 순서 차이로 XYZ 최대 약 0.061mm 차이가 났고, 32장면에서
유효 BEV 셀 23개가 달라졌다. 기하 계산만 141.70→16.81ms로 줄었지만 그대로 채택하지 않았다.

셀 경계 1mm 이내이며 유효 격자에 들어올 가능성이 있는 ray만 원래 3×3 matrix-vector
방식으로 재계산한다. depth 25cm의 실제 증강 32장면, frustum sample 428,212,224개에서
유효 셀 배정 차이는 0개다. 재계산은 전체 ray의 약 0.63%이며 기하 계산은
282.72→52.00ms다. 이는 검사한 센서·증강 범위의 실측 정합이며 임의의 calibration에 대한
수학적 오차 상한 증명이 아니다. 학습 기하·정렬은 매 batch 다시 계산하며 추론 cache를
랜덤 증강에 재사용하지 않는다.

실제 학습에는 float32 autocast 문맥이 있다. Torch 2.7에서 `is_autocast_enabled=true`라도
dtype이 float32일 수 있다. 첫 prototype은 이 조건을 FP16으로 오인해 기존 경로로 돌아갔고,
단독 계산이 빨라도 전체 step에는 이득이 없었다. dtype까지 검사하도록 보완했으며 최종 비교는
개선 실행 각각의 `fast_geometry_calls=64`를 확인한다. FP16/BF16·TF32, calibration gradient,
eval에서는 기존 기하 경로를 유지한다. 기존 warm 추론 cache의 좌표·조건도 바꾸지 않는다.

### 확정된 로컬 전체 학습 비용

| 동일 설정 비교 | 기존 평균 | 개선 평균 | 단축 |
|---|---:|---:|---:|
| depth 25cm, batch 2, 각 64 step × 두 번 | 940.69ms | 632.00ms | 32.82% |
| depth 50cm, batch 2, 각 64 step | 733.32ms | 576.47ms | 21.39% |
| 정밀 5/20cm, depth 50cm, batch 1, 각 48 step | 450.79ms | 371.33ms | 17.63% |

개선 두 실행은 631.32 / 632.68ms이고 기존 두 실행은 936.62 / 944.75ms다.
peak allocated는 약 11,110~11,115MiB로 큰 차이가 없다. 이것은 4090의 가속률이나
epoch 전체 단축률을 측정한 것이 아니다. 검증·저장 비용은 그대로 남는다.
50cm·정밀 격자 비교도 초기 파라미터·첫 센서 입력·장면 순서·증강 GT가 같다.
정밀 격자는 로컬 VRAM 제한 때문에 batch 1이며 batch 2 결과와 절대값으로 비교하지 않는다.
정밀 5/20cm에서도 32장면의 유효 셀 배정 차이 0개, full-frustum forward/gradient 검사를
통과했다. 기하 계산은 140.44→26.29ms다. 유효 격자 밖의 미세한 셀 차이는 유지된다.

### 카메라 컴파일 후보의 별도 검토

동일 Swin backbone의 bound `forward` callable만 `torch.compile(dynamic=False)`로 감싼다.
모듈 자체를 교체하지 않아 `state_dict`에 `_orig_mod` 접두사가 생기지 않는다. 실제 학습용
opt-in config는 초기화·resume 이후 첫 camera forward에서 컴파일한다.
TF32 변경·해상도 축소·증강 제거·모델 채널 축소나 TensorRT 적용은 하지 않는다.

depth 50cm·batch 2의 64 step 비용은 기하 개선 묶음 576.47ms에서 컴파일 묶음 457.32ms로
추가 약 20.67% 줄었다. 기존 733.32ms 대비 전체 약 37.64%다. 실제 optimizer 64 update와
유한 loss를 확인했고 peak allocated는 10,111.6MiB로 비컴파일 후보 10,893.6MiB보다
약 782MiB 낮았다. 첫 컴파일을 포함한 해당 비용 실행 전체 시간은 95.86초다.
정상 step 평균과 cold compile 비용은 구분한다. 4090의 compiler cache는 별도로 생성한다.

depth 25cm에서도 같은 64-step 입력으로 비컴파일 후보 632.73ms와 컴파일 후보 509.52ms를
비교했다. 추가 약 19.47% 단축이며 기존 depth 25cm 두 실행 평균 940.69ms 대비 약
45.84%다. 기존 실행과도 초기 parameter·첫 센서 입력·전체 64 step의 증강 GT와 장면
순서가 일치함을 별도로 확인했다. peak allocated는 10,339.2MiB이고 optimizer 64 update,
fast geometry 64회, 유한 loss를 확인했다. 컴파일 후보는 각 depth에서 한 실행이므로
비컴파일 25cm의 두 번 반복 검증과 동일한 반복 수로 기록하지 않는다.

최초 수치 비교는 LayerNorm 출력 제곱 평균을 인위적인 loss로 사용했다. forward 상대 L2는
약 0.1%였지만 전체 parameter gradient 상대 L2 약 4.13%로 사전 기준 0.3%를 통과하지
못했다. LayerNorm 출력의 제곱 에너지는 거의 상수여서 작은 상쇄 gradient의 상대 오차가
커질 수 있다. 이 실패를 삭제하거나 tolerance를 넓혀 통과로 바꾸지 않는다.
별도 고정 upstream sensitivity를 사용한 Jacobian-vector 비교에서는 전체 parameter gradient
상대 L2 0.1876%로 같은 0.3% 기준을 통과했다. 각 출력 상대 L2도 0.5% 미만이고 missing
gradient는 없다. 개별 parameter의 최대 상대 L2는 0.6295%이며 이 값도 숨기지 않는다.
이는 한 실제 장면의 4-camera 입력에 대한 AMP 수치 검사이며 모든 loss 방향·AP 우위나
eager/compiled DropPath RNG의 bitwise 동일성을 보장하지 않는다. 컴파일은 별도 실험 후보이며
원래 config의 기본값으로 승격하지 않는다.

동일 검사 입력의 warmed camera backbone eval 비용은 44.63→24.63ms다. pooling·LiDAR·head·
ROS 전처리가 빠져 있는 **부분 추론**이므로 서비스 전체 70ms 달성으로 해석하지 않는다.
학습 기하 fast path는 eval에서 사용하지 않으며 pooling backward 개선도 추론에서 실행되지 않는다.

### 저장·재개와 회귀 검사

- pooling·격자·경계·정밀도 guard·stream·역전파 검사 21개는 CUDA 전체 통과,
  CPU는 CUDA 전용 8개 skip 외 통과했다. 별도 loss/gradient·fused AdamW 상태 검사 3개,
  기존 직접 학습 prepare/resume guard 검사 5개를 통과했다.
- 실제 4-class JPEG 두 장면을 사용해 depth 25cm의 비컴파일 후보와 컴파일 후보 각각
  AMP 2 update → 원자적 epoch 1 저장 → 새 Runner 재개 → iteration 4 완료를 통과했다.
  optimizer step 2, LR 7.75e-06, AMP scaler 32의 재개 상태와 유한 최종 parameter,
  partial checkpoint 0개를 확인했다. full optimizer/scheduler/scaler·main-process RNG를
  보존하며 worker/prefetch의 중간 batch bitwise replay를 보장한다고 기록하지 않는다.
- smoke 계약은 비컴파일 `95746b3605292f825c3d90eef6202ea2fc0d17fecd9463ca8f986dda12ca2b9f`,
  컴파일 `05acf8de0e7b05136eee36e49c76ad9df6e00ab7867b5423c638ba7547b75925`다.
  이 두 계약은 각각 두 장면 검사용이며 본학습의 계약이 아니다.
- 두 최종 full checkpoint의 `state_dict` key 574개가 같고 `_orig_mod` key는 0개다.
  실제 저장된 optimizer param group의 `fused=True`와 iteration 4를 별도로 확인했다.

### 코드와 재현 경로

- 후보 config: `perception/detection3d/bevfusion/configs/bevfusion_base5_dense_512_training_runtime_candidate.py`.
  현재 후보 파일 자체의 depth는 50cm다. 비용 시험의 `--depth-step .25`와 구분한다.
  production E25·기존 native-JPEG 기본 config는 바꾸지 않는다.
- 실제 로더 비용: `benchmark_base5_real_loader.py`의 opt-in 옵션이다. `--profile-steps`의
  계측 step은 정상 비용 통계에서 제외한다. `--compile-camera`는 opt-in 비용 실험 옵션이다.
- 컴파일 후보 config: `bevfusion_base5_dense_512_training_compiled_candidate.py`.
  비컴파일 후보와 별도 비교하며 체크포인트 key·재개·수치 검사를 통과해야 한다.
- 순차 대조: `benchmark_base5_training_matrix.py`. 새 output만 받으며 반복 2회째에는
  순서를 뒤집는다. 입력·초기화 계약이 다르면 비교를 거부한다. full epoch를 실행하지 않는다.
- 기하·큰 frustum gradient: `qualify_base5_batched_geometry.py`.
- 근거: `artifacts/evaluation/bevfusion/base5_training_runtime_261011/`의 `cost.json`,
  `matrix.json`, 기하 검사와 CPU/CUDA 단위 검사다. ignored 산출물을 Git에 강제로 추가하지 않는다.

다음은 채택한 소스만 4090에 전달해 batch 4의 50/25cm 비용·메모리·저장/재개와 기존
warm 추론 비용을 재확인하고 새 main config/source hash/work-dir을 준비하는 것이다.
구계약 resume이나 본학습 시작으로 대체하지 않는다. 학습 전용 기하·역전파 개선을
warm 추론의 같은 비율 가속으로 환산하지 않는다.

### 4090 재개 후 순서

1. 기존 verified JPEG 5K와 camera-only 초기화 파일을 재사용한다. 새 코드의 SHA와
   `torch/CUDA/spconv`를 확인하며 데이터 재전송이나 의존성 재설치부터 시작하지 않는다.
2. 먼저 batch 4·depth 25cm의 `baseline/geometry/compiled`를 같은 입력으로 대조한다.
   `benchmark_base5_training_matrix.py --steps 64 --warmup 16 --repeats 2`를 사용한다.
   기본 단일 case 제한은 600초이며 실패를 기록하고 후보 채택을 거부한다.
3. 컴파일의 4090 수치·cold 비용·메모리, 짧은 저장·재개를 확인한다. 전체 추론 비용은
   현재 warm cache 경로와 별도로 비교한다. 이득이 없거나 검사가 실패하면 비컴파일
   `batched_checked/warp/int32/log/fused` 후보를 대안으로 사용한다.
4. 최종 depth·runtime 플래그와 optimizer wrapper를 명시해 **새** main config/source hash/
   work-dir을 준비하고 ETA를 보고한다. 기존 50cm 준비 계약의 resume는 하지 않는다.
   사용자가 승인하면 기존 완료된 외부 백업 경로와 연결해 본학습을 시작한다.

<a id="base5-inference-runtime-local-261011"></a>

## 22. 로컬 전체 추론 서비스 비교 — 2026-10-11 KST

사용자는 학습 실행 비용 개선에 이어 추론 개선을 요청했다. 약 07:55~08:35 KST에
기존 팀 이미지와 RTX 5060 Ti 16GB에서 수행했다. Vast 접속·재개·설치, MORAI 제어,
본학습·TensorRT·production E25 배포는 수행하지 않았다. 아래 가속 수치는 **비용 후보의
실측이며 기본값 승격이나 학습 완료 정확도 검증을 뜻하지 않는다.**

### 입력·비교 계약

- upright native-JPEG Sunny13 release와 기존의 고유 300장 선택을 재사용했다.
  200장 hash-random 대표 표본과 100장 stress 표본이며 선택과 센서 payload SHA를 확인했다.
  입력 512×1056, batch 1 AMP, depth 25cm, semantic ROI X±100/Y±30m를 고정했다.
- 모든 후보는 21절의 **4-update smoke full checkpoint** 하나를 strict load한다.
  이 가중치는 수렴한 탐지 모델이 아니다. camera-only 초기화 파일을 full checkpoint처럼
  로드하지 않는다. 체크포인트 SHA256은
  `e02c5dd9cc498d3ceea08b0e8824a3995fcdf941422a8b356f7fa9d935fd7f38`이다.
- 기존 7.5/30cm는 내부 X±100.8/Y±32.4m, 정밀 5/20cm는 공통 비교 padding
  X±100.8/Y±33.6m를 사용한다. 각 격자 안의 기존/개선 비교만 가속률로 계산한다.
  원격 과거의 depth 50cm 비용과 이 로컬 25cm 결과를 직접 비교하지 않는다.
- 실제 지속 TCP 연결에서 JPEG·점군 요청 → 전처리 → 모델 → CPU 박스 필터·목록 →
  JSON 응답 수신을 측정했다. localhost RTT이며 ROS callback·LAN·planner/controller가
  포함된 풀스택 비용은 아니다. 요청은 직렬이고 디스크 읽기는 측정 전에 끝낸다.
- CPU threads 4, OpenCV threads 1, camera workers 4다. synthetic startup warm-up과
  실제 입력 16회 warm-up은 정상 300회 통계에서 제외한다. 첫 컴파일 비용은 별도다.
  CUDA event 구간은 별도 16회 측정하며 host idle이 포함된 실행 경로다.
  중첩된 view 구간을 더하거나 kernel self-time으로 해석하지 않는다.

### 전체 비용과 병목

| 300장 동일 격자 비교 | GPU 완료 포함 모델 평균 | JPEG부터 응답까지 평균 | RTT p95 | peak allocated |
|---|---:|---:|---:|---:|
| 7.5/30cm 기존, 첫 실행 | 94.97ms | 105.62ms | 108.25ms | 3,902MiB |
| 같은 격자 camera compile | 74.68ms | 85.26ms | 88.22ms | 3,869MiB |
| 같은 격자 camera + neck compile | 73.84ms | 84.41ms | 87.09ms | 3,840MiB |
| 같은 격자 기존, 마지막 재실행 | 95.31ms | 105.81ms | 108.23ms | 3,900MiB |
| 5/20cm 기존 | 129.90ms | 140.34ms | 143.03ms | 8,276MiB |
| 같은 정밀 격자 camera + neck compile | 108.57ms | 118.87ms | 121.99ms | 8,239MiB |

기존 두 실행 평균 대비 camera+neck 후보의 전체 응답 단축은 약 20.1%다.
정밀 격자 안에서는 약 15.3%다. **4090의 가속률이나 70ms 달성으로 환산하지 않는다.**
해상도·depth·head query 수·threshold를 낮춰 얻은 수치가 아니다.

| 별도 계측 구간, 7.5/30cm | 기존 | camera+neck 후보 |
|---|---:|---:|
| 카메라 backbone | 44.87ms | 25.64ms |
| 카메라 neck | 3.61ms | 2.40ms |
| view transform 전체 | 17.17ms | 17.56ms |
| voxelization | 1.95ms | 2.04ms |
| LiDAR sparse encoder | 10.97ms | 11.35ms |
| fusion / BEV backbone / BEV neck | 2.00 / 3.66 / 0.85ms | 2.11 / 3.75 / 0.90ms |
| head·decode | 8.24ms | 8.44ms |

가속의 주원인은 고정 camera backbone·neck의 실행 방식이다. 분리 pooling·추론 geometry
cache는 이미 적용된 상태이며 이번 새 가속으로 다시 계산하지 않는다. 21절의 학습 전용
기하 계산·pool backward 개선을 추론 가속으로 옮겨 계산하지도 않는다.

### 출력 정합과 미채택 후보

컴파일은 모듈이 아닌 bound forward callable에 적용한다. 원래 파라미터와 checkpoint key를
유지하며 Torch 2.7.1의 `emulate_precision_casts=True`를 사용했다. 그러나 FP16의 연산
융합·실행 알고리즘 변화까지 bitwise 같아지는 것은 아니다.

사전 출력 gate는 class별 일대일 대응, 개수 유지, 중심·크기 5cm 이내, yaw 0.02rad 이내,
score 0.002 이내다. 통과시키려고 이 기준을 사후 완화하지 않았다.

- 일반 300장 비교의 camera/camera+neck 후보는 각각 64/63장의 출력 gate가 실패했다.
  다만 **변경 없는 모델을 새로 로드한 마지막 기존 실행도 69장 실패**했으므로 이 숫자를
  전부 컴파일 탓으로 해석하지 않는다. RGB 순서 변경 실행과 마지막 기존 실행도 같은
  69장 실패했다. 픽셀 동일성 검사는 별도로 아래와 같이 통과했다.
- 같은 모델에서 LiDAR→image raster를 동일하게 고정한 **수치 검사 전용 통제**를 수행했다.
  32장 eager 반복은 출력이 동일했지만 camera-only compile은 4장, camera+neck은 7장
  gate가 실패했다. camera-only의 최대 중심 차이는 약 1.72cm, 크기 차이는 7.23cm,
  yaw 차이는 0.0322rad였다. 이는 4-update smoke 모델의 실행 차이이며 AP 하락률이 아니다.
- LayerNorm·GELU를 eager로 남긴 후보도 64장 중 9장, layout optimization·shape padding을
  끈 후보도 32장 중 5장 실패했다. 현재 **compiler 후보는 기본값으로 승격하지 않는다.**
  가속 가능성과 출력·학습 완료 정확도 검증을 구분한다.
- 처음에는 sparse depth의 같은 픽셀 중복 쓰기를 원인 후보로 의심했다. 그러나 실제 한
  장면의 8회 반복에서는 depth pixel 변화가 0개였다. 이 작업만으로 재로드 차이의 원인을
  확정하지 않으며 nearest-depth/last-point 정책이나 학습 입력을 임의로 바꾸지 않았다.
  frozen raster는 수치 통제일 뿐, 서로 다른 추론 프레임에 재사용하는 cache가 아니다.
- Dense Conv·BatchNorm 26개 folding은 FP32 toy·복원 검사를 통과했지만 실제 출력 gate가
  실패했고 전체 비용 이득도 작았다. 기본값으로 채택하지 않는다. sparse conv/head는 접지 않는다.
- Camera CUDA Graph는 예비 48장 출력 gate를 통과했지만 카메라 구간은 약 44.8→44.4ms로
  큰 이득이 없었다. 직렬 고정 shape의 실험 옵션으로만 보존하며 기본값은 OFF다.
- 분리 CUDA stream으로 LiDAR 가지를 먼저 실행하는 prototype은 sparse encoder에서
  `Your points vanished here` 오류를 재현했다. 설치된 voxel CUDA의 global device sync도
  확인했다. **stream prototype·서버 옵션은 활성 소스에서 제거했다.** 모든 병렬화가
  불가능하다고 결론내리지는 않지만 이 환경에서 그대로 채택하지 않는다.
  rejected source snapshot과 전체 오류 로그는 ignored 평가 폴더에 보존한다.
  로그상 오류는 실제 장면이 아닌 prototype의 startup warm-up에서 발생했다.
  spconv는 입력 index가 8,190개 있었지만 stride 2 sparse convolution의 유효 출력
  index 생성에 실패해 이 오류를 냈다. 이 문구의 일반적인 padding 예시는 실제 원인
  확정이 아니다. 같은 실험의 기존 경로는 24장 추론을 완료했다. 별도 stream의
  voxel/spconv 호환성·동기화가 원인 후보이며, 근본 원인은 아직 확정하지 않았다.
  `cudaDeviceSynchronize()` 발견 자체가 점군 소실의 원인 증명도 아니다.
  기각 범위는 camera/LiDAR 동시 scheduling prototype이며 sparse encoder·학습 compile·
  데이터셋·GT를 기각하거나 변경한 것이 아니다. 기존 직렬 경로는 이후 23절의
  4090 300장 비용 검사를 완료했다. 성능 개선률은 실패한 병렬 실행에서 주장하지 않는다.

### 전처리와 서버 계약

색 변환을 전체 원본 이미지가 아닌 resize·crop 이후에 수행하는 opt-in을 구현했다.
실제 300장 × 4camera의 **1,200개 입력 픽셀과 점군이 완전히 같았다.** crop·투영은 유지된다.
CPU 단독 AB/BA 300회씩의 평균은 기존 8.413ms, 변경 8.334ms로 약 0.079ms 절약이다.
전체 서비스 측정에서는 확실한 단축을 확인하지 못했으므로 기본값으로 바꾸지 않는다.

기존 서버는 E25의 3-class만 허용했다. `--base5-runtime`을 명시한 시험 경로를 추가하고,
4-class 순서·single-frame·7D·native-JPEG·upright 계약과 trusted local full checkpoint의
strict load를 검사한다. training config의 lazy camera compile을 중복 활성화하지 않는다.
animal은 label 3 / class_name animal로 보존하며 pedestrian으로 자동 변환하지 않는다.
실제 ROS adapter·planning 배포 완료를 뜻하지 않는다. E25 기본 경로·입력·증강·가중치는 유지한다.

서버 응답에 기존 모델 비용과 구분되는 `postprocess_ms`를 추가했다. `--cpu-threads`,
`--opencv-threads`도 명시적 옵션이며 기존 기본값은 바꾸지 않는다. 모든 새 최적화는 OFF가
기본이고 source checkpoint를 덮어쓰지 않는다. inference-only 최적화는 disable 후 복원된다.

### 코드·재현·다음 실행

- `docker/bevfusion/app/inference_optimization.py`: opt-in compile·fold·camera graph와 복원/guard.
- `benchmark_base5_inference_runtime.py`: 동일 full checkpoint·300장 localhost 서비스 비교,
  별도 stage 계측·입력/source identity·실패한 출력 gate 보존. `--require-parity`는 실패 시
  exit 2다. cost 완료의 exit 0을 numerical qualification 통과로 읽지 않는다.
- `qualify_base5_inference_numerics.py`: 같은 모델의 eager 반복과 frozen raster 통제.
  `qualify_base5_late_rgb.py`: CPU 픽셀 정합과 AB/BA 전처리 비용.
- 근거: `artifacts/evaluation/bevfusion/base5_inference_runtime_261011/` 아래
  `final_300/report.json`, `fine_300/report.json`, `numerics*`, `late_rgb_v2/report.json`,
  `parallel_failure.log`, `rejected_parallel_source.py`, `unit_tests.log`다.
  ignored 입력·체크포인트·산출물은 Git에 강제로 추가하지 않는다.
- Docker runtime 검사 24개(CUDA graph FP32/AMP 포함)와 별도 sensor calibration 1개를
  통과했다. 이는 실제 학습 완료 AP나 ROS/LAN 폐루프 검증의 대체가 아니다.

다음은 사용자 재개 확인 후 기존 4090 데이터·설치 환경을 재사용해 최신 학습 후보와 추론
후보를 재확인하는 것이다. `benchmark_base5_inference_runtime.py`의 `--depth-step .25`,
`--grid current075/common050`, `--variants baseline camera camera_neck baseline`,
`--iterations 300 --stage-iterations 16`을 사용한다. **해당 depth와 맞는 full checkpoint가
필요하다.** 현재 smoke 비용 시험만으로 compile·정밀 격자를 채택하지 않는다.
학습된 checkpoint에서 same-instance 출력 차이·val/rosbag 정확도를 추가 확인하고,
가속 이득이 없거나 gate가 실패하면 검증된 비컴파일 추론을 사용한다.
최종 소스·depth·학습 플래그·새 work-dir 계약과 사용자 승인 후 본학습을 시작한다.

<a id="base5-inference-runtime-vast-261011"></a>

## 23. 4090 재확인 — 학습 제외 검증, 2026-10-11 14:32 KST

사용자는 4090을 재개하고 **학습을 제외한 필요한 작업**을 요청했다. 기존 인스턴스
55272391의 설치·5K 입력을 재사용했다. 이번 작업의 optimizer update는 0회다.
본학습·짧은 학습 benchmark·백업 watcher·ROS 배포·TensorRT는 실행하지 않았고,
인스턴스를 Stop/destroy하거나 production E25·센서·GT·split을 변경하지 않았다.
다음 내용은 22절의 로컬 추론 결과와 구분되는 실제 원격 실행이다.

### 조건과 비용용 checkpoint

- RTX 4090 48GB 개조 모델, Torch 2.7.1+cu128, OpenCV 4.11.0, AMP·batch 1이다.
  512×1056 Q90 JPEG, semantic ROI X±100/Y±30m, 4-class·7D를 유지한다.
  camera workers 4 / OpenCV threads 1 / Torch threads 4가 기본 대조군이다.
- 동일한 서로 다른 300장: 대표 200장 + stress 100장이다. selection SHA256은
  `76443de45c3252c6e3dc7809802832e9948546519371e0a416ce152a00cd0b42`다.
  실행 입력 manifest SHA256은 `34f3560de16370a763607d68180d35268b159f3f87d8d88f1f217a250da26ce1`다.
  실제 입력 16회 warm-up과 cold compilation은 정상 300회 통계에서 제외한다.
  전체 응답은 JPEG decode·전처리·모델·후처리·localhost TCP 반환까지이며 ROS/LAN/판단 비용은 아니다.
- 로컬의 200MB inference checkpoint 전송은 약 0.2MB/s여서 제한 시간 안에 중단했다.
  불완전 파일은 `.incomplete`로 보존하고 로딩하지 않았다. 5K 센서를 다시 전송하지 않았다.
- 원격에 있는 D50 4-update smoke full checkpoint를 비용 시험에 재사용했다.
  source SHA256은 `cc150a22861a1603abe2fe6aedd22f73552750eb4cb7f22fc0ff091b8b440973`다.
  D25는 depth logit 198→396개를 쌍으로 복제하고 config-owned frustum을 복원해
  **COST-only shape fixture**로 만들었다. 다른 state key·shape 변경은 거부하며 574개를 strict load한다.
  결과 SHA256은 `1a0c411e087a6d52ed9a666b84ed32f8525efe66c735f9cf68f4fe45130788f7`다.
  이는 학습된 D25 모델도, 본학습 초기화 정책도 아니다. 정확도/AP/속도 MAE는 평가하지 않았다.

### 전체 추론 비용과 메모리

아래는 allocator 예산을 20GiB로 제한한 같은 호스트의 실행이다. **48GB GPU의 예산 제한은
실제 24GB 카드 검증이 아니다.** 기본 격자는 7.5/30cm, depth 25cm다.

| 실행, 300장씩 | 모델 평균 | 전체 응답 평균 | 전체 응답 p95 | steady peak allocated | 출력 정합 |
|---|---:|---:|---:|---:|---|
| 비컴파일, 첫 실행 | 50.95ms | 71.83ms | 73.04ms | 1,130MiB | 비교 기준, 매 장면 31개 출력 |
| camera+neck compile 후보 | 45.23ms | 65.94ms | 67.12ms | 1,072MiB | 300/300 gate 실패, 기본 OFF |
| 비컴파일, 마지막 재실행 | 50.75ms | 71.76ms | 73.69ms | 1,132MiB | 기준 대비 5/300 gate 실패 |

두 비컴파일 평균 71.80ms 대비 compiler 후보의 비용 단축은 약 **8.2%**다.
5060 Ti의 20.1%를 그대로 적용하지 않는다. 13Hz 주기 76.92ms 이내의 대조군도
p95 기준 여유가 작으며 실제 ROS·네트워크까지 실시간성을 보장한 결과는 아니다.
20GiB 제한에서 startup peak는 약 **17.1GiB**였고 OOM은 없었다.
제한 없는 48GB 실행은 CuDNN warm-up 중 약 33.5GiB를 할당했다. 이를 정상 steady VRAM과
섞거나 ‘24GB에서는 반드시 불가능’으로 결론내리지 않는다.

별도 D50 smoke 대조군은 비컴파일 두 실행 평균 약 70.63ms, camera+neck 66.20ms였다.
stage 계측에서 camera backbone 15.47→9.41ms, neck 1.44→0.86ms가 주요 단축이며
전처리 약 18.8~19.0ms, LiDAR 약 16.0ms, view transform 약 7.1ms는 거의 그대로였다.
이 stage는 별도 16회 CUDA event 측정이며 중첩 구간을 더하지 않는다.
D50 source와 D25 COST fixture가 달라 이 차이를 학습된 두 모델의 성능 비교로 사용하지 않는다.

**정밀 5/20cm + D25는 연산 비용 참고만 남긴다.** 같은 300장에서 비컴파일
79.60ms / 재실행 82.55ms, camera+neck 75.06ms(p95 79.53ms)였지만 **모든 실행의
decoded 출력이 0개**였다. 이 fixture의 정밀 격자 적응·객체 반환 비용·수치 정합은 검증되지 않았다.
빈 리스트끼리 같다는 이유로 통과 처리하지 않도록 `prediction_counts`와 `nonvacuous`를
보고하고 `--require-parity`에서는 빈 출력도 실패한다. 정확도 열세를 의미하지 않으며,
비어 있는 비용 시험조차 13Hz에 충분한 여유가 없는 참고 결과로만 사용한다.
7.5/30cm와 정밀 격자의 계산 padding Y±32.4/±33.6 차이도 있어 순수 단일 변수 비교가 아니다.

### compiler 출력 차이의 원인 통제

기존 gate는 중심·크기 5cm, yaw 0.02rad, score 0.002 및 class·개수 유지다.
통과시키기 위해 기준이나 decode threshold를 완화하지 않았다.

- 같은 모델·LiDAR depth raster 통제의 64장에서 eager 반복은 64/64 일치했다.
  camera+neck compile과 dense folding은 각각 64/64 gate 실패했다.
  한 장면의 depth raster 8회 반복도 바뀐 픽셀이 0개였다.
- camera feature relative L2 차이는 한 장면에서 약 0.06~0.11%, dense heatmap 최대 차이는
  약 `1.5e-5`였다. 그러나 실제 decoder가 선택한 **200개 query 중 12개**가 달라졌다.
  거의 학습되지 않은 heatmap의 가까운 순위가 작은 연산 오차로 뒤바뀌면, 서로 다른 위치의
  박스 목록을 비교하게 된다. 목록 매칭의 수십 m 차이는 실제 차량 한 대의 위치가
  그만큼 움직였다는 뜻이 아니다. 재로드 대조군의 일부 차이도 따로 보존한다.
- 원인 진단용 32장 검사에서 baseline heatmap으로 query 선택만 고정하자,
  compiled feature를 사용한 regression의 gate 실패는 **0/32**, 최대 중심 차이는
  약 **1.95mm**였다. 원래 compile은 32/32 실패했다. 후보 선택의 증폭이 주요 원인이라는
  근거이며, 전체 AP 저하 또는 학습 후 안전성을 확정한 결과는 아니다.
- **query 고정은 원인 진단 전용**이다. 실제 서버에서 과거 heatmap/query를 재사용하지 않는다.
  compile·dense folding·camera graph는 기본 OFF로 유지하고 학습된 checkpoint에서 재검증한다.

### CPU 전처리와 CUDA·geometry 검증

CPU-only로 128장의 camera worker/OpenCV thread 조합 8개를 정방향·역방향으로 비교했다.
일반 thread 조합들은 이미지·점군·투영 행렬이 완전히 같았다. 단독 측정에서 빨라 보였던
camera workers 1 / OpenCV 4도 실제 300장 서비스에서는 두 실행 평균 **73.26ms**로
기존 71.80ms보다 느렸다. 기본 4/1/4 설정은 바꾸지 않는다.

RGB 변환을 crop 뒤로 미루는 후보는 원격 OpenCV에서 `b5v2_01161`의 이미지 값
**1개가 1만큼** 달랐다. 이전 로컬 1,200개 카메라 입력의 동일성 결과를 모든 환경에
일반화하지 않는다. 이는 thread 수 자체의 픽셀 차이가 아니며 확실한 전체 비용 이득도 없어 OFF다.

| 검사 | 실제 결과 | 해석 범위 |
|---|---|---|
| Docker runtime 단위 검사 | 24개 통과 | inference 옵션·복원 계약 |
| rectangular config·geometry | 7개 통과 | 새 D25 opt-in과 기존 기본값 분리 |
| 센서 보정 CLI | 1개 통과 | 기존 센서 투영 계약 유지 |
| pooling CPU / CUDA | CPU 21개 중 8개 skip·실패 0, CUDA 21개 통과 | 연산 forward/backward 정합 |
| 실제 증강 geometry, D25 32장 | relevant cell mismatch 0, accepted | 기하 계산·grouped forward·gradient 검사 |

32장 증강 geometry의 계산 평균은 **52.21→18.76ms**였다. backward 검사는 수행했지만
optimizer update는 없으며 이것을 전체 batch 4 학습 step 단축률로 환산하지 않는다.
기존 환경을 재사용했고 변경한 CUDA 커널의 첫 JIT build만 필요했다.

### 새 prepare-only와 다음 실행

현재 후보는 `bevfusion_base5_dense_512_depth25_runtime_candidate.py`다. depth 25cm·
512MiB cache, 검증된 batched geometry/warp pooling/int32 plan/fused AdamW의 opt-in을
명시하며 **camera compile은 OFF**다. 기본 50cm config·증강·GT 정책은 유지한다.

`run_base5_direct_training.py`를 **`--run` 없이** 실행해 다음 준비 계약만 만들었다.

- 원격 source: `/workspace/base5_runtime_source_18ddf1eb_261011`.
- 새 work-dir: `/workspace/base5_training_sunny13_d25_runtime_prepared_55272391_261011`.
- contract: `4fa4015b88d0b4114deebd1145df74f85da19208d6deb12efa233c45c663570b`.
- 1,000개 source hash가 로컬과 일치한다. train 4,255 / val 745, batch 4, workers 4,
  24 epoch, lr `1e-4`, warm-up 500 update, seed 261010, epoch 경계 resume 계약이다.
- 초기화는 감사한 camera-only nuScenes checkpoint다. SHA256은
  `d06ae54d380a7c0ec245936ff394a51e0ff2c36fd1288eb2d7879954c682107b`다.
  위 COST-only fixture는 초기화에 사용하지 않는다. prepare-only는 학습 완료·시작 승인이 아니다.

학습 허용 후 최신 소스·D25 runtime 후보에서 실제 batch 4 비용·AMP 갱신·저장/재개를
짧게 확인한다. 이전 소스의 검사나 geometry 단독 속도를 최신 전체 step 시간으로 바꾸지 않는다.
그 결과와 시간 예산을 보고한 뒤 승인 후 백업 watcher·본학습을 시작한다.
학습된 모델에서 compile의 실제 val/rosbag 정확도와 출력 안정성을 다시 확인하며,
통과하지 않으면 비컴파일 추론을 유지한다. 정밀 격자는 이번 비용만으로 채택하지 않는다.

### 근거 보존

`artifacts/evaluation/bevfusion/base5_inference_remote_261011/reports/`에 실패·중단 로그도
함께 보존했다. 핵심은 `d25_current_cap20_300`, `d25_fine_cap20_300`,
`numerics_cap20_64`, `numerics_query_control_32`, `preprocessing_threads_v2`,
`d25_cpu1_cv4_cap20_300`, `pool_tests.json` 및 geometry/runtime 검사 로그다.
source checkpoint나 불완전 업로드는 보고서 압축에 포함하지 않았다.
로컬 `prepared_run_manifest.json`과 `prepared_resolved_train.py`도 보존했다.
보고서 압축 SHA256은 `25df7bd3bb3513c9db482435c974b20caf2368b76fc6b8eb84b56e38d6468346`다.
ignored 산출물·가중치·센서는 Git에 강제로 추가하지 않는다.

<a id="base5-parallel-stream-repair-261011"></a>

## 24. Sparse encoder 분리 stream 오류 재현과 실행 순서 보완 — 2026-10-11 15:13 KST

사용자 요청에 따라 이전에 제거했던 병렬 prototype의 원인을 추가 조사했다.
학습·optimizer update는 실행하지 않았으며 원본 데이터·GT·센서·E25·학습 기본값은 유지했다.
단순히 padding을 늘리거나 빈 LiDAR를 통과시키는 보완이 아니다.

### 원인: 실제 점 소실이 아닌 활성 voxel 수의 조기 읽기

설치된 `spconv 2.3.6 / cumm 0.4.11`의 regular sparse convolution은 GPU에서
활성 voxel 개수를 계산한 뒤 `unique_hash`에서 CPU로 읽는다. spconv는 stream context를
전달하지만 cumm의 `Tensor.cpu(Context ctx)`는 내부에서 `res.copy_(*this)`를 호출하여
그 context를 복사에 전달하지 않는다. 별도 non-blocking stream에서 GPU 계산이
완료되기 전 기본 stream의 복사로 수치를 읽을 가능성이 있다.
근거는 설치된 코드와 [cumm v0.4.11 원본](https://github.com/FindDefinition/cumm/blob/v0.4.11/include/tensorview/tensor.h#L1376),
[spconv v2.3.6 unique_hash](https://github.com/traveller59/spconv/blob/v2.3.6/spconv/csrc/sparse/indices.py#L1427)다.

- 5060 Ti의 기존 side-LiDAR 전체 경로에서 `Your points vanished here`를 다시 재현했다.
  실패는 synthetic startup 단계이며 실제 장면에 point가 없다는 근거가 아니다.
- 원래 count를 읽은 직후 해당 stream을 동기화해 **같은 GPU count**를 다시 읽는
  진단 통제를 구현했다. 8장 A/B/A/B에서 count 불일치 45회, `0→양수` 26회를 확인했다.
  예를 들어 원래 읽기는 0, 동기화 후 읽기는 5,454였다. 다른 예는 27,722→39,488이다.
  이 통제에서는 cold startup과 모든 8장 비교를 통과했다.
- count 재읽기는 **진단 도구에만** 있으며 설치 라이브러리를 전역 패치하거나 기본 모델에
  동기화를 추가하지 않았다. 이 통제의 전체 비용은 약 108ms로 가속 이득이 없었다.
- 4090에서는 side-LiDAR의 단독·cold·실제 8장 검사 출력이 정확히 같았고 기존 전체
  64장 경로도 실패하지 않았다. GPU·초기화 순서·실행 타이밍에 따라 감춰질 수 있는
  stream 문제를 4090의 한 번 통과만으로 해결됐다고 보지 않는다.

### 수정: LiDAR는 기본 stream, 카메라를 별도 stream으로

`inference_parallel.py`의 `camera_side`는 다음 순서로 실행한다.

1. 전송·정규화와 voxel화를 기본 stream에서 끝낸다. 설치된 hard voxel kernel의
   device-wide synchronize를 실제 overlap 구간 전에 둔다.
2. 카메라 backbone·neck·view transform을 별도 stream에 실행한다.
3. sparse encoder는 기본 stream에서 실행한다. batch size는 알려진 입력 list 길이의
   Python 정수이며 GPU scalar를 다시 읽지 않는다.
4. fusion 전에 camera stream 완료를 기다린다. 전달 tensor의 `record_stream`과
   예외 시 stream join, instance-local 직렬 lock, 원래 메서드 복원 경로를 둔다.

가중치·checkpoint key·모델 구조·semantic ROI·격자·depth·증강·threshold·head query 수는
바꾸지 않는다. `--base5-runtime --parallel-camera`의 **명시적 opt-in**으로만 제공한다.
학습과 non-default caller stream을 거부하며 기존 E25 서버와 기본 추론은 바뀌지 않는다.
`legacy`·`voxel_main` 및 count 재읽기는 재현·진단용이다.

### 실제 4090 300장 전체 응답 비교

23절과 같은 native JPEG 300장(대표 200 + stress 100), D25 full COST fixture,
7.5/30cm·512×1056, allocator cap 20GiB를 사용했다. 직렬 localhost 요청이며 GPU 완료·
CPU decode·응답 전송을 포함한다. 디스크 읽기·cold/JIT·계측 구간은 통계에서 제외했다.

| 실행 | 모델 평균 | 전체 RTT 평균 | RTT p95 |
|---|---:|---:|---:|
| 비컴파일 기존, 첫 실행 | 50.65ms | 71.39ms | 72.71ms |
| **비컴파일 camera-side 병렬화** | **45.09ms** | **65.81ms** | **67.92ms** |
| 비컴파일 기존, 재실행 | 50.50ms | 71.26ms | 72.73ms |
| camera+neck compile | 45.53ms | 66.31ms | 69.02ms |
| camera+neck compile + camera-side | 38.42ms | 59.16ms | 60.56ms |
| camera+neck compile, 재실행 | 45.49ms | 66.07ms | 67.83ms |

비컴파일 두 실행 평균 대비 병렬화 단축은 약 7.7%, compiled 두 실행 평균 대비
추가 단축은 약 10.6%다. 기존 비컴파일 대비 결합 후보는 약 17.1% 단축이다.
해상도나 depth를 낮추어 얻은 이득이 아니며 ROS/LAN/풀스택 보장으로 해석하지 않는다.
병렬 실행의 CUDA event span은 겹치므로 합산하지 않는다. 모든 case는 300장 비어 있지 않은
출력 9,300개를 유지했고 stream/encoder 예외 없이 완료했다.

5/20cm도 같은 300장으로 별도 검사했다. 기존 전체 RTT는 첫 실행 79.25ms(p95 80.68),
마지막 재실행 82.69ms(p95 85.17)였고, 비컴파일 병렬화는 72.87ms(p95 74.33),
camera+neck compile 결합은 66.61ms(p95 68.40)였다. **이 정밀 격자의 fixture는
모든 실행에서 decoded 출력이 0개**다. 따라서 비용 참고용이며, 70ms 아래의 실제
학습 완료 검출기 또는 정합 통과를 확보했다고 주장하지 않는다. baseline의 변동도 보존한다.

### 출력 정합과 채택 범위

- 4090 같은 모델 64장 A/B/A/B는 기존의 5cm·0.02rad·score 0.002 기준을 모두 통과했다.
  이것을 bitwise 동일성이나 AP 통과로 표현하지 않는다. 최대 중심 차이는 약 1.04mm다.
- 새로 로드하는 실제 300장 비교는 병렬화 단독 296/300, 비변경 재실행 298/300 통과였다.
  원래 compiler 후보와 결합 후보는 decoded gate가 실패했다. 기준·query를 완화하거나
  출력 하나를 숨겨서 통과시키지 않았다.
- 같은 4090 모델을 유지한 300장 A/B/A/B도 완료했다. 병렬화 297/300, 기존 반복
  298/300, 병렬 반복 296/300이었다. 예외 해결을 전체 decoded gate 통과로 바꾸지 않는다.
- 마지막 4090 **같은 프레임의 sparse-depth raster 고정 통제 300장 A/B/A/B**는
  모든 비교가 통과했고 중심·크기·yaw·score 차이가 모두 0이었다. 실제 원본 입력의
  gate 실패와 이 통제의 통과를 구분한다. 통제 때문에 추론 입력 정책을 변경하지 않는다.
- 5060 Ti 실제 64장 일반 비교에는 기존 반복과 병렬 반복 모두 1장 실패가 있었다.
  sparse depth raster만 같은 프레임의 기준값으로 고정한 **진단 통제**에서는
  64장 A/B/A/B가 모두 통과했다. 이는 서로 다른 프레임 사이의 depth 재사용 정책이 아니다.
  이 통제의 예측 중심·크기·yaw·score 차이는 모두 0이었다.
- COST fixture의 불안정한 query 선택과 학습된 모델의 정확도를 구분한다.
  오류를 우회하고 가속 가능성을 확인했지만 **병렬화·compile 기본값은 계속 OFF**다.
  학습된 모델의 val·rosbag 정합과 실제 서비스 비용을 확인한 뒤 선택한다.

마지막 로컬 Docker 회귀 검사는 28개 통과, 설정을 인자로 받는 sensor calibration은
별도 1개 통과다. CUDA toy는 실제 BEVFusion처럼 tensor list를 fusion에 전달하여
계산이 stream join 이후에 시작하도록 구성했다. 중간 검사에서 누락된 PYTHONPATH·
calibration 인자를 바로잡았으며, toy가 join 전에 덧셈하던 잘못된 모사도 수정했다.
출력 허용 오차를 늘리지 않았다. 학습 차단·기본 stream 유지·예외 복구·idempotent 복원·
production E25에서 새 옵션 거부·checkpoint key 유지 검사를 포함한다.

### 실행 근거와 재현 경로

- 원격 isolated source: `/workspace/base5_parallel_source_151cefab_261011`.
- 원격 산출물: `/workspace/base5_parallel_qualification_151cefab_261011`.
- 로컬 ignored 근거: `artifacts/evaluation/bevfusion/base5_parallel_261011/`.
- 도구: `diagnose_base5_lidar_streams.py`, `qualify_base5_parallel_inference.py`,
  `benchmark_base5_inference_runtime.py`의 `parallel_camera` / `camera_neck_parallel` variant.

`remote_reports.tar.gz`에는 config·보고서·실패/성공 로그만 보존하며 가중치·센서 데이터는
포함하지 않는다. 마지막 frozen300 보고서는 실행 전후 주요 소스 SHA256 일치도 검사했다.
보고서 archive SHA256은 `a29de9b2271e1dd0aaa8fba0cf6fa610b19df9bdfb942c026831f7648002c707`다.

23절의 원격 source·준비 계약은 덮어쓰지 않았다. 새 inference 소스는 별도 경로에 두며,
이 변경을 포함한 다음 실행에는 source/config 계약을 다시 확인한다. 본학습 초기화에
COST fixture를 사용하지 않으며 다음 학습 작업은 허브에 합의한 짧은 compile OFF/ON
검사와 새 계약부터다. 인스턴스는 자동 Stop하지 않는다.

<a id="base5-fine-training-compile-vast-261011"></a>

## 25. 정밀 5/20cm의 4090 학습 compile OFF/ON 검사 — 2026-10-11 KST

사용자는 5cm LiDAR voxel / 20cm camera BEV를 우선 후보로 선택하고 **짧은 학습
compile 비교만** 승인했다. Sunny13 전체 본학습·다른 환경 수집·production E25·원본 GT는
변경하지 않는다. 추론의 camera-side 병렬화와 camera+neck compile은 별개이며 기본 OFF다.

### 실제 비교 조건

- 인스턴스 55272391, RTX 4090의 관측 VRAM 49,140MiB, 기존 Torch 2.7.1+cu128 환경.
  의존성을 다시 설치하거나 GPU 임대·Stop/destroy를 수행하지 않았다.
- 원본 해상도 Q90 JPEG, 입력 512×1056, depth `[1,100,.25]`의 396개 bin,
  single-frame·4-class·7D, train 4,255 / val 745, 실제 로더 batch 4 / workers 4.
- semantic ROI X±100/Y±30m는 유지한다. 정수 내부 padding은 X±100.8/Y±33.6m이며
  sparse grid `[4032,1344,41]`, raw camera BEV `[336,1008]`, 최종 fusion `[168,504]`다.
  최종 fusion/head 간격은 40cm다. 20cm가 최종 head 간격이라는 뜻은 아니다.
- camera-only nuScenes 초기화 SHA256은
  `d06ae54d380a7c0ec245936ff394a51e0ff2c36fd1288eb2d7879954c682107b`다.
  LiDAR/depth/head는 새 초기화다. 23·24절의 shape 변환 COST checkpoint를 학습에 사용하지 않았다.
- 양쪽 모두 warp pooling backward·compact int32·batched checked geometry·일괄 loss log·
  fused AdamW, AMP scale 32, LR 1e-4, seed 261010을 사용한다. 카메라 **backbone forward만**
  compile한다. neck·sparse encoder·head를 compile하거나 증강을 끄지 않는다.
- 64 update씩 OFF→ON으로 실행한다. 첫 16회 warm-up과 profiler 4회는 정상 비용에서
  제외하고 44회씩 측정한다. 실제 장면은 각 256개이며 64회 전체 장면 순서·증강 GT,
  첫 센서 입력·초기 전체 parameter SHA가 양쪽에서 같다. 이후 ON→OFF 24회씩을 별도로
  재확인했다. 작은 mini-dataset의 재개 검사는 아래와 분리한다.

### 실제 전체 step 비용

| 실제 64 update 비교 | compile OFF | compile ON |
|---|---:|---:|
| 로더·증강·로그를 포함한 정상 step 평균 | 659.59ms | 575.03ms |
| 정상 step p95 | 673.18ms | 594.76ms |
| GPU 전송부터 갱신까지의 평균 | 643.13ms | 558.79ms |
| 다음 입력 준비·로더 대기 평균 | 16.47ms | 16.24ms |
| 정상 peak allocated | 29,118.90MiB | 27,595.08MiB |
| 정상 peak reserved | 30,784MiB | 29,116MiB |
| 첫 준비 구간 peak allocated | 29,107.33MiB | 46,808.75MiB |
| 첫 update 시간 | 60.11초 | 98.85초 |
| 전체 bounded train-loop 시간 | 120.52초 | 150.68초 |

정상 step은 **약 12.82% 단축**됐다. ON→OFF의 별도 24회에서는 567.03ms와
659.89ms로 약 14.07% 단축을 재확인했다. 후속 비교는 warm-up 8회·profile 4회·
별도 gradient audit 구간을 제외한 각 10회이므로 64회 검사와 같은 표본 수로 읽지 않는다.
첫 update에는 JIT·compiler·라이브러리 준비가 섞여 있다. 두 시간 차이를 순수 compile
비용으로 단정하지 않으며 cold 비용을 정상 step에 섞지 않는다.

정상 allocated는 ON 약 26.95GiB / OFF 약 28.44GiB지만 ON의 첫 준비 구간은
약 **45.71GiB**까지 올라갔다. 이번 48GB 인스턴스에서는 OOM 없이 통과했으나 여유가
넉넉한 시작은 아니다. 일반 24GB 4090에서 같은 batch 4 학습이 된다는 뜻도 아니다.
추론용 VRAM·13Hz 전체 서비스 비용과 학습 메모리를 혼동하지 않는다.

### 무엇이 빨라졌는가

아래는 각 실행의 별도 계측 4회에서 얻은 CUDA event span이다. host idle·계측 영향을
포함하며 정상 step 표와 섞거나 모든 항목을 합산하지 않는다. backward는 전체 모델의
backward이고 카메라 backward만 따로 측정한 값은 아니다.

| 계측 항목 | OFF | ON |
|---|---:|---:|
| 카메라 backbone forward | 89.00ms | 75.96ms |
| 전체 backward | 330.84ms | 264.39ms |
| geometry / plan | 62.83ms | 61.99ms |
| sparse LiDAR | 32.44ms | 33.82ms |

카메라 compile의 주요 추가 이득은 forward뿐 아니라 backward에도 나타났다. LiDAR,
depth bin·BEV 격자·head를 줄인 결과가 아니다. 큰 정밀 BEV에서 다른 연산 비용은 그대로
남으므로 21절의 로컬 7.5/30cm 가속률을 이번 전체 학습에 그대로 적용하지 않는다.

### 갱신·gradient·저장/재개

- 64회와 추가 24회 각각 optimizer가 모든 update에서 진행했고 AMP overflow/skip은
  없었다. scale 32·growth tracker 64 또는 24와 유한 loss·최종 parameter를 확인했다.
- 첫 64회 도구에서는 profiler의 method 복원이 gradient audit wrapper까지 원복해
  실제 audit는 0·16회에서만 실행됐다. 이 진단 도구 문제를 숨기거나 4회로 집계하지 않는다.
  audit hook을 profiler보다 먼저 설치하도록 순서를 수정하고 후속 24회에서 0·8·12·23회의
  **네 번 모두 전체 390 trainable parameter**를 검사했다. OFF/ON 모두 gradient 누락 0,
  nonfinite 0이다. 진단 구간은 정상 비용에서 제외한다.
- compiled opt-in config로 실제 7개 train capture를 사용해 batch **4→3**을 실행하고,
  2 update checkpoint에서 중단 후 4 update까지 재개했다. 실제 5K의 마지막 batch 3을
  고려한 검사다. 네 class를 포함하고 checkpoint의 `_orig_mod` 접두사·partial 파일은 없다.
- 재개 직후 모델 모든 tensor·optimizer와 AMP scaler·scheduler·Python/NumPy/Torch/CUDA
  메인 RNG를 저장된 payload와 정확히 비교했다. 9개 확인 항목이 모두 true다.
  mini-dataset resume contract는
  `3e938b7fb877aea5c66b331b6a77d204cf59018a4cf2e0706a1cf26d26c57b68`다.
  이는 workers 0의 epoch 경계 wiring 검사이며 workers 4의 prefetch/중간 epoch를
  bitwise replay한다는 인증이 아니다. 짧은 loss 감소도 장기 수렴·AP 동등성을 보장하지 않는다.

### 별도 수치 통제·validation·회귀 검사

실제 장면 `b5v2_02165`의 4-camera 입력에서 DropPath를 끈 camera backbone의 AMP
forward와 고정 upstream sensitivity backward를 비교했다. 21절과 같은 사전 기준
(전체 gradient 상대 L2 <0.3%, 각 feature 상대 L2 <0.5%)을 바꾸지 않았다.

- feature 상대 L2는 0.0754% / 0.1047% / 0.0581%다.
- 전체 parameter gradient 상대 L2는 **0.1758%**로 통과했다. 개별 parameter 최대값
  0.4210%는 별도로 기록한다. 모든 parameter가 0.3% 이내라는 뜻은 아니다.
- gradient 누락 0, 모든 gradient 유한. 이 단일 입력의 고정 sensitivity 검사는 모든
  실제 loss 방향이나 stochastic DropPath RNG의 bitwise 동일성을 인증하지 않는다.
- 설치 MMEngine `BaseModel.train_step`와 `AmpOptimWrapper`를 확인했다. forward는
  AMP context 안, `update_params`의 backward는 밖에서 실행된다. 수치 통제도 같은
  forward AMP / backward context 밖의 배치를 사용한다.
- 같은 4-update fine checkpoint로 **FP32 val batch 1의 32개 서로 다른 장면**과 실제
  metric 경로를 검사했다. 모두 유한 출력·4-class 범위·유한 metric, 누락 0으로 통과했다.
  745장 전체 val/AP 검사로 보고하지 않는다. source infos는 변경하지 않고 dataset
  indices로 균등하게 선택했다. 비용 도구 기본값은 계속 745장이다.
- 마지막 CPU 회귀 17개 통과 / CUDA-only 1개 skip, GPU runtime·정확한 toy resume
  회귀 4개 통과다. CPU 격자 단위 검사는 CUDA custom-import를 실행하지 않도록 분리했다.
  실제 원격 모델 import·forward/backward는 CUDA 확장이 있는 환경에서 별도로 검증했다.
  최초 로컬 bare fork 검사에서 extension import 오류가 있었으며 모델 오류로 해석하지 않는다.

### 선택·예산과 준비 계약

**이번 48GB 4090의 5/20cm·depth 25cm 학습에는 camera backbone compile ON을 추천한다.**
실제 초기화·증강·loss·optimizer를 유지하며 학습 속도와 gradient·저장/재개를 확인했다.
이는 긴 학습의 수렴/AP 우위를 증명하거나 추론 compiler/병렬화 gate를 통과했다는 뜻이 아니다.
모든 원래 config를 변경하지 않고 새 opt-in config 둘을 추가했다.

- OFF: `bevfusion_base5_fine_512_depth25_runtime_candidate.py`.
- ON: `bevfusion_base5_fine_512_depth25_training_compiled_candidate.py`.
- 최종 준비 소스: `/workspace/base5_training_compile_ready_source_66414829_261011`.
- 새 prepare-only work-dir:
  `/workspace/base5_training_sunny13_fine_d25_compiled_prepared_55272391_261011`.
- 계약: `9e2b7b85e2a3d2d0d51272e1b5b8ae09e7b28b6c0659b991505d2065958a2324`.
- 전체 detector/common/fork의 **1,005개 source SHA가 현재 로컬과 모두 일치**한다.
  디렉터리 이름의 `66414829`는 기반 revision이며 이후 검사 변경을 포함한 frozen snapshot이다.
  실행 contents는 이름이 아니라 `run_manifest.json`의 source hashes로 검증한다.
- prepare-only는 batch 4 / workers 4 / 24 epoch / LR 1e-4 / warm-up 500 / seed 261010,
  원래 4,255/745 split·증강·camera-only 초기화를 사용한다. 비용 probe의 warm-up LR 50회,
  mini-dataset smoke의 1회와 본학습 schedule을 혼동하지 않는다.

24 epoch 학습 갱신만 외삽하면 OFF 약 4.68시간 / ON 약 **4.08시간**이다.
32장 val의 cold 구간을 제외한 마지막 24장 평균은 85.67ms이며 이를 단순 외삽한
매 epoch 745장 val 비용은 약 63.8초다. train+val 중심값은 약 4.50시간이고,
첫 compile·batch 3/val용 graph 준비·checkpoint 저장·장기 부하 여유를 포함한
**운영 예산은 약 4.7~5.3시간**으로 둔다. 32장 외삽·짧은 train probe 기반 예상이며
전체 fine val·장기 학습 시간의 실측이 아니다.

### 근거 보존과 다음 단계

로컬 ignored 근거는 `artifacts/evaluation/bevfusion/base5_training_compile_261011/`다.
원격 64회/역순 24회/partial batch resume/val32 보고서·trace·로그와 새 준비 config를
`base5_training_compile_reports_66414829_261011.tar.gz`로 모았다. 센서·가중치는 포함하지 않는다.
archive SHA256은 `7fc15b6d7267725744acd81638a89c76570c0d5b7bd729831232322517f2fb42`다.
실제 4-update checkpoint SHA256은
`f1eede3469ff25e2d8ffb4ba3bcbca4f8b557f96a4710347d5bf53ce9a763562`이며
원격 resume 폴더에 보존했다. 본학습 초기화 checkpoint로 승격하지 않는다.

본학습·상주 백업 watcher는 시작하지 않았다. 새 준비 폴더에 `fresh_start.json`,
`last_checkpoint`, 학습 checkpoint가 없음을 확인했다. 기존 준비/실험 폴더와 데이터는
보존한다. 다음은 사용자 승인 후 **새 계약의 실행과 해당 폴더를 가리키는 백업 watcher**다.
실행 직전 source/config 일치를 다시 확인하며 이후 code 변경을 기존 계약에 강제 섞지 않는다.
이번 인스턴스는 자동 Stop하지 않고 GPU 작업이 끝난 유휴 상태로 둔다.

<a id="base5-memory-batch-decision-261011"></a>

## 26. 48GB 학습과 24GB 추론의 메모리·배치 판단 — 2026-10-11 16:03 KST

25절 이후 대화에서 기존 보고서를 다시 확인했다. **새 GPU 실행이나 batch 확대 검사는
수행하지 않았다.** 사용자는 학습 48GB·batch 4·누적 1 유지 방향에 동의했다.
현재 결정과 후속 순서는 [계획 허브](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-training-inference-memory-decision-261011)에
둔다. 아래는 결정의 수치 근거이며 학습/추론 compile을 혼동하지 않는다.

### 추론 compile ON도 20GiB 예산에서 실행한 근거가 있다

24절의 5/20cm·D25·512×1056·batch 1, 300장 비용 보고서를 재확인했다.
48GB 4090에서 CUDA allocator 예산을 **20GiB**로 제한했고 다음 case가 모두 OOM 없이
완료됐다. allocated peak는 GPU 전체 사용량·reserved memory와 다른 값이다.

| 추론 case | 정상 peak allocated | 시작 peak allocated |
|---|---:|---:|
| 비컴파일 기존, 첫 실행 | 1.58GiB | 17.11GiB |
| 비컴파일 camera-side 병렬화 | 1.59GiB | 17.13GiB |
| camera+neck compile와 병렬화 결합 | 1.55GiB | 17.08GiB |
| 비컴파일 기존, 재실행 | 1.61GiB | 17.14GiB |

ignored 원자료:
`artifacts/evaluation/bevfusion/base5_parallel_261011/base5_parallel_qualification_151cefab_261011/fine_tcp300_v1/report.json`.
`allocator_budget_gib`, 각 case의 `peak_allocated_mib`, `startup_peak_allocated_mib`를
사용해 GiB로 환산했다. 보고서의 cost fixture·source hashes와 제한 조건은 그대로다.

**48GB GPU 예산 제한은 실제 24GB GPU 검증이 아니다.** 이 fine fixture는 모든 case의
decoded 출력이 0개였으므로, 300장 실행 완료가 검출 정확도·출력 정합 인증은 아니다.
실제 완성 가중치·출력 전송·장면별 활성 voxel·ROS 풀스택 부하의 확인은 남아 있다.
24GB에서 추론 compile이 불가능하다는 결론은 현재 근거와 맞지 않으며, 실행 가능성이
충분한 후보로 유지하되 실제 카드의 cold start와 연속 입력으로 최종 확인한다.

25절의 **45.71GiB는 batch 4 학습 최초 준비 피크**다. 이를 추론 compile의 필수 메모리로
옮겨 해석하지 않는다. 학습에는 역전파용 중간값·gradient·optimizer 상태가 필요하지만
추론 batch 1에서는 필요하지 않다. compile 없이 읽을 수 있는 checkpoint key와 실제
저장/재개를 확인했으므로 학습 ON 가중치로 추론 OFF/ON을 따로 선택할 수 있다.

### batch를 늘리지 않은 이유와 시간 추정의 경계

6절의 과거 512×1056·실제 train 로더·AMP AdamW 비용은 다음과 같다. 현재 fine·D25·
JPEG·compile ON 설정과 다른 조건이므로 최신 batch 8의 성능·메모리로 재사용하지 않는다.

| 과거 물리 batch | step 평균 | 처리량 | 정상 peak allocated |
|---|---:|---:|---:|
| 4 | 609.57ms | 6.56장/s | 20.64GiB |
| 8 | 1,187.06ms | 6.74장/s | 40.67GiB |

처리량 이득은 약 2.7%였고 메모리는 거의 두 배였다. 현재 fine ON의 batch 4 정상
allocated는 26.95GiB이며 cold peak는 45.71GiB다. 최신 batch 6/8의 OOM 여부·처리량은
미검증이므로 숫자를 선형 외삽해 가능/불가능 또는 시간 단축률을 확정하지 않는다.
이번에는 25절의 검증된 batch 4와 운영 예산 **4.7~5.3시간**을 유지한다.

batch 확대는 모델 파라미터 개수를 바꾸지 않지만 optimizer 갱신 빈도·gradient 통계와
학습 설정의 의미를 바꾼다. train 4,255장, 마지막 묶음을 버리지 않는 기준에서 batch 4는
epoch당 1,064 update, batch 8은 532 update다. warm-up을 500 update로 그대로 두면
노출되는 장면 수와 epoch 비율이 달라진다. 학습률을 무조건 두 배로 하거나 큰 batch의
정확도 개선을 보장하지 않는다. 배치를 변경하려면 별도 계약·스케줄 검토가 필요하다.

향후 24GB에서 추가 학습할 필요가 생기면 작은 물리 batch와 gradient accumulation을
검토한다. [MMEngine 공식 OptimWrapper 설명](https://mmengine.readthedocs.io/en/latest/tutorials/optim_wrapper.html)의
누적 기능을 사용할 수 있지만 BatchNorm·난수·연산 순서 때문에 물리 batch 4와 완전히
같은 학습이라고 보장하지 않는다. 현재 원격 준비 계약·소스·학습률·warm-up은 변경하지 않았다.

현재는 추가 최적화를 반복하기보다 본학습 결과를 확보할 단계다. TensorRT는 후속 후보이며
학습된 모델의 실제 24GB 메모리·추론 정합·13Hz 비용을 확인하기 전에는 배포를 인증하지 않는다.

<a id="base5-cpu-contention-local-261011"></a>

## 27. 로컬 ROS 스택과 인지의 CPU 경합 검사 — 2026-10-11 16:45 KST

### 결론과 검사 범위

Ryzen 5 7500F의 6코어/12스레드와 RTX 5060 Ti 16GB에서 **측위·tracking·prediction·
planning·controller를 실제 실행하고 인지 비용과 함께 측정했다.** 정상 장면의 전체 CPU
평균은 약 33~37%였으며 제어·측위 약 50Hz, 최종 planning 약 10Hz를 유지했다.
추가 CPU 점유 작업 4개로 평균 약 76%까지 올려도 같은 주기는 유지했다.
현재의 CPU 스레드 제한 조건에서는 심각한 CPU 포화가 관측되지 않았다.

이는 4090의 13Hz 처리, 실제 LAN 왕복, 모든 trajectory 검사의 통과나 폐루프 안전 인증이
아니다. 5060 Ti의 인지 처리율은 약 6.2~7.0장/s였으므로 4090에서 13Hz로 더 자주
호출되는 CPU 비용은 다시 확인해야 한다. 추가 점유 시험은 여유를 보는 보조 근거이지
실제 13Hz 스택과 동일한 부하가 아니다. 본학습·Vast 작업·production 설정은 변경하지 않았다.

### 입력·실행 계약

- raw DEVELOPMENT bag은 `260923_161315_H01` 고주로, `260923_163859_O01` 장애물,
  `260923_163034_R01` 로터리를 사용했다. 각 실제 경로는 산출물 `report.json`에 기록했다.
  평가 잠금 bag이나 원본 수집 데이터를 변경하지 않았다.
- 새로운 loopback ROS master에서 1배속·단방향으로 재생했다. `/sensor/*`, Ego,
  Competition, Object만 재생하고 기록된 TF·측위·객체 인식 출력·경로·제어 명령은
  재생하지 않았다. UDP 송신, MORAI, RViz와 폐루프 제어는 실행하지 않았다.
- production 측위 launch, CTRV tracking·Map-CV prediction, 실제 AW149 planning
  전체 경로와 Smooth state controller를 실행했다. planner의 전방 경로 길이는 400m다.
  객체 입력은 `/Object_topic`을 13Hz 상한으로 제한한 GT adapter다. 실제 관측된
  detection 빈도는 약 9.9~11.3Hz이며 prediction은 약 28Hz다. nominal 설정 주기와
  관측 주기를 동일시하지 않는다.
- 실제 신호등 YOLO·route group helper를 추가로 실행했다. GPU device 0, 기존 신호등
  판단 설정과 10Hz 상한을 유지했다. debug 이미지 송출은 껐고 관측된 송출 빈도는
  약 8.6~8.9Hz다. 고주로에서 신호등 검출 결과가 없어도 YOLO 연산은 실행됐다.
- 인지 비용은 ROS의 GT와 독립적인 Base5 JPEG/TCP 시험이다. 공간 5/20cm·depth 25cm·
  512×1056·batch 1 AMP, compile OFF·별도 stream OFF, camera workers 4·Torch 4·
  OpenCV 1을 사용했다. GT 기반 planning을 사용하여 비용용 가중치의 탐지 내용 때문에
  planning 부하가 비어버리지 않도록 했다. **실제 ROS detector bridge·LAN 처리나 AP
  측정은 포함되지 않았다.**
- 인지 입력은 기존 300장 비용 표본에서 대표 64장·stress 32장을 교차 선택한 동일한
  96장이다. selection SHA는 `67bba553d8383532403238f28a0816d5104aa51c829ab32fada149b805690b55`다.
  인지 단독을 앞뒤에서 각각 실행한 192 request는 **96장씩 두 번**이며 192개 독립
  장면이 아니다. 주요 비교는 bag 재생 25초 뒤 시작했으며 cold/JIT·warm-up·별도
  CUDA event 계측은 정상 응답 통계에서 제외했다.
- 팀 이미지 `ajounice/bevfusion:team-cu128-v1`를 사용했다. allocator는 12GiB로 제한했고
  이 비컴파일 비용 시험의 인지 정상 peak allocated는 약 1.575GiB, 초기 peak는
  약 8.08GiB였다. 26절의 원격 compile 후보 메모리 측정과 다른 조건이다.
  비용용 `d25_smoke_inference.pth`를 본학습 초기화나 탐지 성능 근거로 사용하지 않는다.

### 실제 측정값

CPU는 모든 논리 코어의 사용률 평균이며 기존 데스크톱 작업도 포함한다. 응답은 localhost
TCP·decode/crop·모델·decode/postprocess를 포함하고 디스크 읽기는 미리 끝냈다.

| 조건 | 전체 CPU 평균 | 인지 응답 평균 | 인지 응답 p95 | 제어 / 최종 planning |
|---|---:|---:|---:|---:|
| 인지 단독, 앞쪽 대조 | 15.33% | 142.70ms | 145.26ms | 미실행 |
| 고주로 실제 CPU 스택 + 인지 | 34.15% | 146.21ms | 148.80ms | 49.98 / 9.97Hz |
| 위 조건 + 실제 신호등 YOLO | 36.23% | 149.37ms | 153.52ms | 49.97 / 10.04Hz |
| 위 조건 + CPU 점유 작업 4개 | 76.20% | 156.06ms | 163.00ms | 50.03 / 10.01Hz |
| 인지 단독, 뒤쪽 대조 | 20.26% | 146.59ms | 150.24ms | 미실행 |
| 장애물 실제 스택 + 신호등 + 인지 | 33.34% | 151.83ms | 155.63ms | 50.05 / 10.01Hz |
| 로터리 실제 스택 + 신호등 + 인지 | 37.43% | 149.69ms | 155.81ms | 50.00 / 10.01Hz |

인지 단독 두 실행을 합친 응답 평균은 144.65ms, p95는 149.31ms다. 고주로 CPU 스택의
차이는 약 +1.56ms, 신호등 포함 약 +4.72ms다. 단독 반복 자체의 평균 차이가 약 3.89ms
이므로 작은 차이를 정밀한 확정 가속률·감속률로 해석하지 않는다. 장애물·로터리는
다른 bag의 별도 부하 확인이며 고주로와 동일한 장면이라고 합산하지 않는다.

고주로의 preprocessing 평균은 단독 9.64ms, CPU 스택 9.63ms, 신호등 포함 9.61ms,
추가 점유 포함 11.29ms다. 대부분의 응답 시간은 GPU를 포함한 모델 실행 구간에 있고
CPU 영상 준비가 새 주 병목이 된 것은 아니다. 신호등도 GPU를 사용하므로 전체 스택을
‘BEVFusion만 GPU, 나머지는 모두 CPU’로 가정하지 않는다.

고주로 CPU 스택의 프로세스군 평균은 인지 약 1.34 논리 코어, planning 약 0.77,
tracking/prediction 약 0.66, 측위 약 0.26, controller 약 0.034다. 프로세스군의 100%는
논리 코어 하나이며 전체 CPU 표의 100%와 분모가 다르다. 작업자·OS thread 개수와 실제
사용 중인 코어 수를 혼동하지 않는다. 학습 DataLoader workers 설정은 ROS 추론 작업자가 아니다.

### 경고와 실패를 별도로 보존한 이유

- 정상 고주로의 control 간격 p95는 약 21.8~22.2ms, 최대 약 46~50ms다. 추가 점유
  조건에서는 p95 25.46ms, 최대 52.24ms였다. 평균 50Hz 유지가 매번 정확히 20ms에
  실행됐다는 뜻은 아니다. 장애물·로터리의 최대 control 간격은 약 35.17/34.00ms다.
- 약 0.5초의 객체·신호등 입력 공백을 발견하여 원본 bag을 확인했다. 재생 34.2초 부근에
  이미 Object 518.15ms, 전방 카메라 517.15ms, LiDAR 583.66ms의 기록 간격이 있었다.
  이를 새로운 CPU 경합으로 발생한 입력 누락이라고 판정하지 않는다. 원인 자체가
  시뮬레이터인지 당시 기록기인지까지 이번 시험으로 확정하지 않는다.
- H01의 `Invalid Trajectory`·planning latency 경고는 GPU 추론을 실행하지 않은 실제
  CPU 스택 대조에서도 발생했다. 따라서 인지 경합만으로 설명하지 않는다. `report.valid`
  는 실제 비어 있지 않은 prediction·trajectory와 측위·제어 출력이 활성화됐다는 뜻이며
  trajectory validator의 모든 flag가 통과했다는 뜻이 아니다. 경고는 보존하고 planning
  안전 정책을 완화하지 않았다.
- 최초 로터리 목표인 마지막 Ego pose는 기존 planner에서 `Goal is not valid`로
  거부됐다. 그 실패는 `roundabout_probe/failure.json`에 남겼다. 같은 bag의 80% 시점에
  기록된 실제 Ego pose를 목표로 재실행한 `roundabout_probe_v2`는 정상 경로와 측정
  출력을 만들었다. 목표 유효성 검사·map·각도 허용 기준은 수정하지 않았다.
- 초기 harness의 긴 측정 창은 bag 길이 guard로 중단됐고, source overlay의 미포함
  CUDA extension은 import 실패로 중단됐다. 모두 실패로 기록하고 최종 표에서 제외했다.
  현재 시험은 임시 source 복사본에 팀 이미지의 ABI 호환 extension을 사용한다.
  원본 third-party source나 production container는 덮어쓰지 않았다.

### 재현 도구와 남은 실행

산출물 root는 `artifacts/evaluation/bevfusion/base5_cpu_contention_261011/`다.
`summary.json`은 matched selection 검증·원본 입력 공백·실패 제외 근거를 포함한다.
각 case의 `report.json`, `resources.json`, `gpu/report.json`, 실제 노드 로그와
`route_result.json`을 보존했다. 새 probe는 topic event 원자료도 저장한다.

```bash
source scripts/workspace/source_aw149.sh

python3 scripts/runtime/test_benchmark_ros_cpu_contention.py

python3 scripts/runtime/benchmark_ros_cpu_contention.py \
  --bag rosbags/eval/perception/260923_teammate_manual/highway/260923_161315_H01_development/260923_161315_H01_ajounice_eval_highway_balanced_0.bag \
  --output artifacts/evaluation/bevfusion/base5_cpu_contention_261011/new_stack_probe \
  --mode stack --gpu-probe --traffic-light --iterations 96 --duration 60

python3 scripts/runtime/analyze_ros_cpu_contention.py \
  --root artifacts/evaluation/bevfusion/base5_cpu_contention_261011
```

기존 output을 재사용하지 않는다. 신호등 시험은 provision된 `traffic_light_gpu` Python과
가중치가 필요하다. `--stress-cores 4`는 90초 상한의 별도 CPU 점유 시험이며 실제
planning 모듈 4개를 뜻하지 않는다. 격리 프로세스군·새 시험 container만 종료하고 기존
데스크톱 작업·production 설정·Vast 인스턴스는 건드리지 않았다. host 회귀 8개,
Python syntax 검사와 실제 8개 정상 case를 통과했고 생성한 ROS/container의 종료도 확인했다.

normal server의 camera workers 기본값은 4지만 Torch·OpenCV의 스레드 제한은 자동으로
보장되지 않는다. 이후 Base5 배포 시험에서 `--cpu-threads 4 --opencv-threads 1`을
명시하는 것을 시작점으로 삼는다. 이번에 production launcher 기본값을 변경한 것은 아니다.

본학습 전에 원격 인공 CPU 부하를 추가 선행 조건으로 만들지 않는다. 다음은 승인 후
Sunny13 본학습·백업이며, 완성 가중치로 실제 4090 24GB에서 compile/병렬화 정합,
ROS bridge·13Hz 입력·신호등·필요한 RViz·실제 LAN·대회 제어 주기와 최대 지연을 확인한다.
그 결과에 따라 팀장님 PC에서 최종 스택 시험을 요청한다.

<a id="base5-training-health-repair-261011"></a>

## 28. 본학습 수치 오류·초기 AP 진단과 제한 복구 — 2026-10-11

18:42 KST 중간 기록이다. 사용자가 낮은 validation 성능을 의심해 전체 점검·수정과
필요한 1~2 epoch 검증 학습을 승인했다. 다른 agent의 MORAI/planning_validator 작업은
건드리지 않으며, 수집·센서·native GT·현재 production E25는 보존한다.

### 실제 중단과 비교 조건

원본 `/workspace/base5_training_sunny13_fine_d25_compiled_prepared_55272391_261011`은
18:14 KST경 E2 중반에 `ValueError: matrix contains invalid numeric entries`로 종료됐다.
오류는 AMP 역전파/스케일러 전에 수행하는 BBox Hungarian matching에서 발생했다.
OOM·Drive 업로드·W&B 통신 오류가 이번 학습 중단의 직접 원인은 아니다.
원본 E1의 1,675개 floating 모델·optimizer tensor는 모두 유한값이며 optimizer step은
1,064, AMP scale은 32다. 원본 E1과 실패 로그를 덮어쓰지 않는다.

| E1 실제 validation | Vehicle | Pedestrian | Obstacle | Animal | 4-class mean |
|---|---:|---:|---:|---:|---:|
| Base5 / 745장 / ±100×±30m | 0.76% | 18.20% | 0% | 0% | 4.74% |

Base3 production 계열의 원래 학습 E1 scalar는 mean AP 93.11%, Vehicle 86.63%,
Pedestrian 98.80%, Obstacle 93.91%다. 이는 nuScenes MORAI3 adapter의 LiDAR·BEV·
fusion·head까지 사용한 3-class/좁은 범위 학습이다. Base5는 카메라 backbone/neck만
초기화하고 LiDAR·depth·BEV·head를 새로 학습한다. 같은 ‘E1’이라는 이유로 직접적인
성능 우열로 해석할 수 없지만, 낮은 Base5 AP를 이유 없이 정상 수렴이라고 단정하지도 않는다.
과거 별도 `corrected_range30_full_e1_e40/e1/metrics.json`은 사후 range 통제 평가로,
원래 학습 E1 scalar와 혼동하지 않는다.

### 모델·데이터·검증 경로 점검

원본 E1에서 validation의 균등 index 48장을 eager FP32로 읽어 query/GT 위치와
BatchNorm running-stat / batch-stat 대조를 수행했다. parameter·optimizer 갱신은 없다.
batch-stat 대조는 진단용이며 validation 점수로 채택하거나 배포하지 않는다.

| 표본 클래스 | GT 수 | 같은 클래스 예측이 XY 1m 안에 있는 GT / running stats | batch-stat 대조 |
|---|---:|---:|---:|
| Vehicle | 20 | 13 | 14 |
| Pedestrian | 36 | 31 | 31 |
| Obstacle | 41 | 25 | 26 |
| Animal | 3 | 0 | 0 |

이는 score-ranked one-to-one AP/recall이 아니라 후보 생성 여부 진단이다. 위치를
근처에 반환해도 높은 점수의 오탐이 많으면 AP가 낮거나 0일 수 있다. 현 metric은
nuScenes-style min precision/recall 0.1을 사용한다. 표본에서 후보 위치가 전부 엉뚱한
축으로 이동하거나 BatchNorm 대조만으로 검출이 회복되는 현상은 관측하지 않았다.
표본 animal query는 running-stat 8/9,600으로 매우 적다. 희귀 class의 초기 학습
부족을 전체 30K 성능으로 일반화하지 않는다.

### 수치 보완과 엄격한 재개

실제 설치된 `FocalLossCost._focal_loss_cost`에서 finite logit 10을 입력하면 FP16은
`-Inf`, FP32는 `-7.499008`을 반환했다. FP16 sigmoid 포화와 eps=1e-12의 표현 범위,
BBox dimension decode의 FP16 exponential overflow가 위험 경로다. 실패 batch의
중간 tensor가 원래 저장되지 않았으므로 정확히 어느 component였는지는 확정하지 않는다.

`MoraiRectangularTransFusionHead.get_targets_single`에서 detached 예측을 **decode 전에
FP32로 변환하고 autocast를 끈 상태에서 매칭**한다. 원래 network 출력·gradient를
수정하지 않으며 NaN/Inf를 0으로 덮거나 GT/실패 batch를 제외하지 않는다. 비유한
network 예측은 장면 index·GT·출력 범위가 남는 오류로 중단한다. legacy E25 head는 유지한다.

`migrate_base5_numerical_resume.py`는 ordinary resume의 hash guard를 풀지 않는다.
부모/자식 identity에서 데이터·모델·초기화·증강·batch·optimizer·LR/schedule이 같은지
검사하고, 열거한 수치/진단/백업 source 변경만 허용한다. tuple→JSON list 표현은
정규화하여 비교한다. parent checkpoint SHA·계약·변경 파일을 migration receipt에 남기고
원본 폴더를 보존한다. exact worker RNG/prefetch replay는 보장하지 않는다.

실제 복구 work-dir는 `/workspace/base5_numerical_validation_e2_v2_261011`이며,
source는 `/workspace/base5_numerical_repair_source_261011`이다. E1의 full state에서
24 epoch schedule을 유지하고 **E2 train+val 후 정상 종료**하도록 제한했다. 18:42 KST에는
560/1,064 batch까지 finite loss/gradient·약 0.58초 step을 확인했다. 전체 epoch/AP gate는 진행 중이다.
최초 migration 시도는 JSON tuple/list 표현 차이를 엄격하게 거부했고 GPU 학습 전 종료했다.
원본과 이 준비 실패 기록도 유지한다.

### 외부 백업의 별도 보완

best checkpoint는 weights-only이므로 resumable epoch checkpoint와 구분하여 저장한다.
`COMPLETE.json`의 `resume_capable=false`를 유지하고 LATEST 재개 포인터는 full epoch만
가리킨다. multipart 복원 임시 이름에서도 이 구분을 명시적으로 전달한다.
live watch에서 새 epoch/validation rewrite를 60초 idempotency 시험에 넣지 않는다.
다음 poll에서 갱신된 fingerprint를 다시 처리하고 qualification-only 재검사는 유지한다.
업로더는 `/workspace/base5_backup_repair_261011/`로 분리해 원래 학습 source를 수정하지 않는다.
완료 marker·full SHA readback·최신 fingerprint가 일치하기 전 외부 백업 완료로 보고하지 않는다.

CPU 회귀는 target 10개, checkpoint backup 10개, migration identity 2개,
local oneclick 13개를 통과했다. 로컬 CPU container의 첫 target 실행은 native pool 확장
경로가 없어 import 단계에서 실패했으며 실제 설치가 있는 remote CPU에서 다시 검사했다.
이 테스트는 full validation AP나 장기 수렴을 보장하지 않는다. 산출물은 로컬
`artifacts/evaluation/bevfusion/base5_oneclick_261011/`과 위 원격 work-dir/log에 보존한다.

### E2 완료와 데이터·평가 대조

18:48 KST경 복구 E2의 1,064 batch와 validation 745장을 모두 완료하고 정상 종료했다.
24 epoch LR 스케줄은 축소하지 않았으며 누적 iteration은 2,128이다.

| 동일 Base5 validation | Vehicle | Pedestrian | Obstacle | Animal | 4-class mean |
|---|---:|---:|---:|---:|---:|
| 원본 E1 | 0.76% | 18.20% | 0% | 0% | 4.74% |
| FP32 matching 복구 E2 | 14.39% | 25.36% | 0% | 0% | 9.94% |

E2의 floating 모델·optimizer tensor 1,675개는 모두 유한값이다. 모든 optimizer state의
step은 2,128이며 scaler는 64, growth tracker는 128이다. 초기 scale 32에서 2,000회의
성공 업데이트 후 정상 성장한 상태와 일치한다. 이 실행에서 AMP skip으로 업데이트가
사라진 근거는 없다. 정상 학습 step은 약 0.58초이며 compile 준비와 validation/AP 비용을
포함한 제한 실행은 796.26초다. 짧은 실행 성공을 장기 수렴의 보장으로 기록하지 않는다.

E2의 Obstacle center AP가 0이어도 위치 예측이 전부 없는 것은 아니다. 동일 validation의
1m 최적 F1 작동점은 precision 7.39% / recall 22% / F1 11.06%, TP mATE는 0.526m다.
0~10m Obstacle mean AP는 1.50%, IoU 0.25 AP는 0.85%다. center AP는 recall 0.1 이후
precision에서 기준 0.1을 빼고 음수를 0으로 자르는 평가라, 많은 배경 오탐과 낮은 점수
순위 때문에 전체 AP가 0으로 나올 수 있다. metric 기준이나 배포 threshold를 바꿔
초기 점수를 올리지 않는다. 정확한 박스·foreground score의 학습이 아직 부족하다는
의미이며 장시간 학습의 성공을 보장하지 않는다.

파생 train/val GT 11,052개를 전수 검사해 비유한 좌표·0 이하 크기는 발견하지 않았다.
train의 BBox bottom Z 한 개는 −3.09m이며 나머지 중대 높이 붕괴의 근거는 없다.
train은 Vehicle 4,178 / Pedestrian 2,310 / Obstacle 2,615 / Animal 247개,
val은 각각 341 / 653 / 674 / 34개다. 두 split의 원거리·클래스 구성도 E25와 다르다.
validation 745장의 정답 박스를 동일 score 1의 예측으로 돌린 oracle 대조에서는
네 클래스와 전체 mean AP 모두 수치 허용 오차 안에서 100%다. 이는 parser·class·metric의
기본 계약을 확인한 것이며 실제 모델의 센서 투영이나 정확도까지 인증하지 않는다.

Vehicle/Pedestrian의 개선에도 Obstacle/Animal이 0이므로 추가 한 epoch를 제한 검증한다.
E3 stopper 변경은 W&B의 기존 run config와 충돌해 첫 두 준비가 학습 전 종료됐다.
기존 E2 run의 config를 강제로 덮지 않고 별도 `-e3validation` run ID로 진단을 분리했다.
training identity·checkpoint·optimizer·24 epoch schedule은 유지한다.

추가 E3 진단 재개에서 `fused AdamW`의 parameter/gradient/moment 형식 불일치를 발견했다.
원인은 **진단 실행기가 저장용 resolved config를 다시 읽은 것**이다. config pretty-print가
`common_heads`를 정렬하면서 height/dim 모듈 등록 순서가 달라지고, optimizer의 위치 기반
state 연결이 다른 branch로 이동했다. 모델 state의 이름 기반 로드가 통과해도 optimizer
moment는 안전하지 않다. 실제 본 실행기는 원본 source config의 `prepare`로 모델을
구성하므로 같은 결함으로 확정하지 않는다. E3도 그 실제 경로로 재검증한다.
E2 checkpoint 자체는 유한값이며 보존한다. 새 checkpoint에는 optimizer parameter name
순서를 저장하고, 재개 전에 이름·순서와 moment shape를 검사한다. 기존 single-group
Base5는 ordered state_dict에서 원래 순서를 복원해 검사하며 이를 임의 재배열하지 않는다.
PyTorch의 [optimizer 입력 검사 구현](https://github.com/pytorch/pytorch/blob/v2.7.1/aten/src/ATen/native/ForeachUtils.h)은
dtype/device뿐 아니라 tensor 크기·stride와 non-overlapping dense 조건도 확인한다.
온라인 W&B 접속이 불안정하여
진단 재시작의 한 번은 API token 검증 서버에 연결하지 못했다. 자격 증명 불량으로
단정하지 않으며, 제한 진단은 로컬 로그/오프라인 W&B 기록을 남겨 통신과 분리한다.

원본 E1 full checkpoint는 SHA `9f6b2e7d9f4da25364e65ee598c63ba8914b89a3592e0cc21e3b411b77479cde`,
best weights는 SHA `e76b22e8f1815a2941f10a5d9a963abaa237f7b417ccde52643f5707fb654013`로
Drive 업로드·전체 SHA 재읽기를 완료했다. validation이 atomically 다시 쓴 E1의 최신
fingerprint까지 반영했다. 이전 content-addressed snapshot은 보존한다.

### E3 전체 검증과 최신 full checkpoint 보존

19:20 KST에 원본 source config의 native prepare 경로로 추가 E3 train/745장 val을
완료했다. 총 iteration은 3,192이며 장시간 본학습을 자동 재개하지 않았다.

| 동일 Base5 validation | Vehicle | Pedestrian | Obstacle | Animal | 4-class mean |
|---|---:|---:|---:|---:|---:|
| 원본 E1 | 0.76% | 18.20% | 0% | 0% | 4.74% |
| FP32 matching E2 | 14.39% | 25.36% | 0% | 0% | 9.94% |
| FP32 matching E3 | 14.43% | 28.33% | 2.49% | 0% | 11.31% |

E3에서도 floating 모델·optimizer tensor 1,675개 모두 유한값, optimizer step 3,192,
AMP scale 64 / growth tracker 1,192를 확인했다. 1,064개의 추가 갱신이 정상 반영됐으며
AMP skip의 근거는 없다. 정상 step 약 0.568초, train+val 제한 실행 약 720.39초다.
loss는 마지막 기록에서 약 4.50이며 matched IoU 약 0.39다. 수치 중단은 복구됐지만
이 AP를 학습 완료·정확도 문제 해결·E25 대체 인증으로 해석하지 않는다.

별도 eager FP32 진단은 validation **745장 전수**로 확대했다. decode가 입력 tensor를
수정하므로 진단에서 복제한 뒤 decode하여 원래 logits/log-dimension을 보존한다.
반환 box 비유한값은 0장, query 총수 149,000개 중 클래스별 11,183 / 28,014 / 103,902 /
5,901개였다. 다음 값은 같은 class 후보의 XY 1m 근접 여부이지 score-ranked one-to-one
recall 또는 AP가 아니다. 이 표만으로 classifier/size/yaw의 품질을 인정하지 않는다.

| 클래스 | GT | 1m 안에 같은 class 후보가 있는 GT | 해당 후보의 score 중앙값 | GT 주변 dense 확률 중앙값 |
|---|---:|---:|---:|---:|
| Vehicle | 341 | 221 | 0.0120 | 0.2605 |
| Pedestrian | 653 | 609 | 0.0205 | 0.3395 |
| Obstacle | 674 | 499 | 0.0117 | 0.1586 |
| Animal | 34 | 24 | 0.0056 | 0.1165 |

근접 후보는 존재하지만 foreground score 순위·박스 품질이 부족하다. Animal AP 0도
‘animal query가 완전히 없다’는 의미는 아니며 train 247 / val 34개의 희귀 class라는
조건을 유지한다. 모든 class를 높은 AP처럼 보이게 하려고 평가 min precision/recall이나
threshold를 완화하지 않는다. 최초 E1의 작은 표본 진단과 이번 전수 결과를 혼동하지 않는다.

새 재개용 준비 폴더는 `/workspace/base5_training_sunny13_health_fixed_55272391_261011`,
동결 source는 `/workspace/base5_training_health_fixed_source_261011`이다. 기존 E3의
full state를 명시적 migration으로 보존했으며 contract는
`de543769fbcddad1838eb1c31f29f2f35a4c6c573b006d0a19656a6610cc3199`다.
원본 E3 SHA `4367c2da6612bedcd95e19c38c37815c98be5818cd11589e591d02a5a513f882`에서
모델·optimizer·24 epoch schedule을 유지하고 새 source 계약·receipt만 기록했다.
이 폴더는 prepare-only이며 실제 E4~24 본학습은 미시작이다.

새 full checkpoint의 SHA는
`2e865f2056faeb7707027a45baf63583c2b436b69bf9b9db0b7a64a814b8d381`,
크기는 520,112,581 bytes다. 실제 Drive 16개 part와 결합 전체 SHA를 로컬에서 재읽어
확인하고 `COMPLETE.json`·`LATEST.json`·run manifest까지 쓰기/재읽기를 완료했다.
`resume_capable=true`, pending 0, errors 0이며 실제 1회 백업은 약 462.88초다.

직전 uploader는 원격으로 full bytes를 다시 받아 약 0.15MB/s에 묶여 1,202.6초 제한에
종료됐다. 동일 SSH 전송에 대용량 backup과 W&B를 함께 넣으면 W&B probe가 10초에
시간 초과되는 현상을 관측했다. 별도 SSH 연결에서는 같은 endpoint가 약 0.63초에
응답했다. 현재 실행기는 W&B(원격 18991)와 backup(18993)을 **독립 SSH 연결**로
분리한다. Drive refresh token/client secret은 로컬에만 둔다. full bytes readback은
로컬 credential 소유 PC에서 수행하고 nonce·part SHA·결합 SHA·byte count가 모두 맞아야
완료를 승인한다. full SHA 기준을 생략하거나 업로드 byte count만으로 완료 처리하지 않는다.
최신 full epoch부터 백업하고 파일 하나가 확인될 때마다 상태를 atomic 저장한다.

실제 회귀는 remote CPU 31개, local oneclick 14개, 실제 CUDA AMP interruption/resume
1개를 통과했다. CUDA toy의 연속/재개 가중치·scheduler·scaler는 정확히 일치했다.
이는 전체 detector 장기 수렴을 보장하는 검사가 아니다.

### 추가 초기화 통제 — camera + sparse LiDAR 후보

E3 AP가 여전히 낮으므로 camera-only 초기화를 정상 수렴이라고 단정하지 않는다.
공식 nuScenes에서 파생한 기존 MORAI3 adapter는 E25 학습 결과가 아니며 원본 SHA는
`2a9ce3e01a723581f8e76b247887a704c4751f2ebcc9b36884c266466700a247`이다.
여기서 sparse encoder 126개 tensor / 10,786,472 bytes만 추출하고 head/class/fusion/BEV/
depth/geometry/optimizer를 가져오지 않는다. 기존 감사한 camera backbone/neck은 유지한다.

Base5는 legacy encoder의 [X,Y,Z] sparse 좌표를 [Y,X,Z]로 바꿨으므로 shape만 같은
kernel을 그대로 로드하지 않는다. 알려진 spconv v2 [O,X,Y,Z,I] metadata에서 공간 kernel의
첫 두 축을 교환하고 `_metadata`를 유지한다. 실제 CUDA sparse encoder의 동일 포인트/
feature 대조에서 출력 [1,256,10,12], XY transpose 최대 절대 오차 0.0000041723으로
rtol/atol 1e-4 검사를 통과했다. CPU 범위/layout 거부와 실제 CUDA 대조 3개를 통과했다.

후보 initializer는 `/workspace/base5_nuscenes_camera_lidar_xy_261011.pth`, SHA
`a08b1beb4e74771ba63806b8b3ce129f7ceb32037f2a7a0fe8ae7ff01e310b9b`다.
기존 원본·복구 E3와 독립된 `/workspace/base5_lidar_initializer_validation_e1_261011`에서
**1 epoch train/전체 val 후 종료**하는 검증을 시작했다. seed·dataset·augmentation·batch·
LR와 24 epoch LR schedule을 유지한다. 일반 본학습은 아직 시작하지 않는다.
원본 E1과는 FP32 matching 보완도 달라 순수한 initializer-only A/B라고 보고하지 않는다.
결과를 확인하기 전 initializer를 production 기본값이나 24 epoch 실행기로 승격하지 않는다.

### Sparse 초기화 단독 실패와 query 분류 prior 누락

19:43 KST에 sparse 초기화 단독 trial이 첫 epoch 중간에 **실제 network 예측 NaN**으로
중단됐다. dense heatmap과 query heatmap score는 유한값이었지만 decoder 이후 center/
height/dim/rot/heatmap이 NaN이었다. 장면 3454 / 3433 / 2894 / 1542와 수치 진단을 보존했다.
이는 FP16 cost만의 문제와 다르며 FP32 matching guard가 유효하게 거부한 사례다.
실패 batch를 버리고 진행하거나 NaN을 0으로 덮지 않는다. 이 trial에는 E1 완료 checkpoint/
전체 validation AP가 없으며 **성공한 사전학습 후보로 승격하지 않는다.**

실제 fresh head를 CPU로 구성해 dense classifier bias는 −2.19였지만 query classifier
bias는 약 −0.084 / −0.010 / 0.008 / −0.015이고, 다시 `init_weights()`를 호출해도
그대로임을 확인했다. TransFusion의 override가 SeparateHead 자식의 초기화를 수행하지
않아 `SeparateHead.init_bias=-2.19`가 적용되지 않는 경로다. 과거 E25는 head까지
pretrained checkpoint로 덮었기 때문에 Base5의 fresh head 조건과 다르다.

Base5-only rectangular head에서 query classifier의 최종 bias에도 SeparateHead의
의도한 prior를 적용한다. 기존 regression/decoder weight, legacy E25, 로드한 checkpoint의
학습된 bias를 변경하지 않는다. fresh/loaded bias 회귀를 추가해 target 검사 총 11개를
통과했다. sigmoid(−2.19)는 약 0.10065, sigmoid(0)은 0.5다. zero-feature 음성 logit
800개, focal α=.25/γ=2의 설명용 합은 103.97→0.645다. 실제 batch loss 실측이나
개선율이 아니라 sparse scene의 초기 background 부담을 설명하는 대조다.

새 source `/workspace/base5_query_prior_fixed_source_261011`, 독립 work-dir
`/workspace/base5_query_prior_validation_e1_261011`에서 동일 camera+sparse initializer·seed·
데이터·optimizer·24 epoch LR schedule로 **1 epoch 후 정상 종료**하는 대조를 시작했다.
직전 sparse-only trial과의 핵심 training 변경은 query bias prior다. 결과 확인 전에는
NaN 원인을 prior 하나로 확정하거나 장시간 수렴을 보장하지 않는다.

진단 E3 offline W&B 파일의 온라인 sync 시도는 180초 상한에서 완료되지 않았다.
로그/결과 파일은 보존하며 업로드 완료로 보고하지 않는다. 별도 실제 온라인 qualification
run `base5-health-online-probe-261011`은 `wandb.init/log/finish`와 서버 summary 재읽기를
통과했다. 따라서 offline bulk sync 실패를 본학습 온라인 logging 전체 불능으로 바꾸어
해석하지 않는다. 작은 probe를 대용량 artifact 업로드 속도 인증으로도 사용하지 않는다.

### Query prior 보완의 첫 전체 epoch 결과 — 2026-10-11 20:01 KST

동일 4,255 train / 745 val을 모두 수행하고 의도한 E1 경계에서 정상 종료했다.
camera+sparse 초기화와 query prior를 함께 적용한 결과이며 원래 camera-only E1과의
차이를 prior 하나의 순수 효과로 보고하지 않는다. 데이터·GT·AP 기준·LR·batch는 유지했다.

| 완료 실행 | Vehicle AP | Pedestrian AP | Obstacle AP | Animal AP | 4-class mean AP |
|---|---:|---:|---:|---:|---:|
| 원래 camera-only E1 | 0.76% | 18.20% | 0.00% | 0.00% | 4.74% |
| FP32 matching 복구 E3 | 14.43% | 28.33% | 2.49% | 0.00% | 11.31% |
| camera+sparse + query prior E1 | 61.69% | 56.78% | 22.17% | 0.00% | 35.16% |

E1 checkpoint의 모델·optimizer floating tensor 1,675개는 모두 유한값이다.
1,064 batch 중 optimizer update는 1,062회이며 초기 AMP overflow로 2회 skip됐다.
scaler는 32→8, growth tracker 1,059다. 이를 ‘AMP skip 0’으로 기록하지 않는다.
후반 정상 step은 약 0.57초, 전체 train+val은 약 788.66초였다.
희귀 Animal(train 247 / val 34)은 AP 0이므로 해결 완료 또는 4-class 최종 품질로 인정하지 않는다.

20:06 KST부터 원래 source config로 E1을 재개해 **E2 전체 train+val 뒤 종료**하도록
제한했다. 24 epoch LR schedule은 그대로이며 본학습 E3~24는 자동 실행하지 않는다.
최종 source의 추가 차이는 Animal W&B 지표를 숨기지 않는 logger·검사·migration 도구다.
모델·optimizer 상태를 보존하고 원본 E1/E3 및 실패 sparse-only trial을 덮어쓰지 않는다.

### Query prior 보완 E2 완료·본학습 재개 선택 — 2026-10-11 20:20 KST

동일 조건의 추가 E2 train 1,064 batch / val 745장을 모두 완료하고 정상 종료했다.
source config의 원래 module 순서로 E1을 재개했으며 optimizer·LR·scaler·RNG를 유지했다.

| 수정된 실행 | Vehicle AP | Pedestrian AP | Obstacle AP | Animal AP | 4-class mean AP |
|---|---:|---:|---:|---:|---:|
| E1 | 61.69% | 56.78% | 22.17% | 0.00% | 35.16% |
| E2 | 72.67% | 72.40% | 37.20% | 0.00% | 45.57% |

E2의 floating model/optimizer tensor 1,675개는 모두 유한값, iteration 2,128 /
optimizer update 2,126이다. E2에서는 추가 AMP skip이 없었고 scale 16 / growth tracker
123이었다. 정상 step 약 0.57초, cold resume·train·val 포함 약 777.51초였다.
mean IoU AP .25는 45.26%, .50은 21.10%다. AP는 검증셋의 전체 예측으로 계산했고
실제 rosbag·실대회 성능이나 학습 완료 수준의 인증으로 바꾸어 해석하지 않는다.

Animal은 E1 1m best-F1 지점에서 recall 52% / precision 2.97%, E2에서는 recall 14% /
precision 6.36%다. 같은 score operating point나 고정 threshold의 직접 비교가 아니다.
동물 예측이 전혀 없는 것은 아니지만 foreground 순위·오탐 구분은 아직 부족하다.
nuScenes-style AP의 min precision/recall 0.1 기준에서 AP가 0이며 평가 조건을 낮추지 않는다.
희귀 class train 247 / val 34라는 조건과 본학습 추세 확인 과제를 남긴다.
E25는 3-class, 이번 평균은 4-class이므로 평균을 같은 분모로 오해하지 않는다.

결정: camera+sparse의 검증된 XY 변환 initializer와 query prior 보완을 채택하고,
**이 실행의 E2 full state에서 E3~24를 사용자가 직접 재개**하도록 준비한다. 새로운
head/fusion/BEV/depth는 이번 Base5 데이터로 학습한다. E25 학습 가중치·새 LR·GT 제외·
metric 완화는 사용하지 않았다. 수정 원인을 prior 하나의 독립 AP 효과로 확정하지 않는다.
FP32 matching 복구만으로는 E3 mean AP 11.31%였고, sparse-only trial은 NaN으로
실패했다는 부정 결과도 보존한다.

최종 source는 `/workspace/base5_training_final_health_source_261011`, work는
`/workspace/base5_training_sunny13_query_prior_fixed_55272391_261011`, 계약은
`a72aaec76e4c3cd8de73f4f6ebd0dc3945780db7951da671938d0e6dcc96aad8`이다.
최종 CPU 회귀 37개 중 CUDA 전용 1개는 건너뛰었고, sparse 축 변환의 별도 실제 CUDA
대조와 AMP interruption/resume는 이미 통과했다. 로컬 실행기 14개·문법·diff 검사도 통과했다.
현재 본학습은 미시작이며 22개 잔여 epoch의 보수적 예상은 약 4.4~4.9시간이다.
초기 compiler ETA는 cold start를 포함해 일시적으로 커지므로 후반 step과 epoch 실측으로
추정한다. 최종 Drive catch-up 시간은 이 학습 예상에서 별도다.

### 최종 재개 검사와 새 실행의 백업 제한 — 2026-10-11 20:28 KST

최종 E2 full checkpoint는 519,926,149 bytes, SHA
`e98bb33d5bce6b9b5b7a3c0db9d6258e3c4b4b6f5a77d3221e6d1f7982e90e9e`이며
`last_checkpoint`는 이 파일을 가리킨다. E2 종료 뒤 remote resume preflight의 GPU 유휴·
source/config/data/init 계약·W&B 서버 summary 재읽기·원래 config prepare-only를 통과했다.
같은 중계에서 Drive 작은 파일의 쓰기/재읽기도 통과했다.

새 실행의 E1 backup 시도는 multipart 전송 재시도 후 **1,201.04초 상한에서 SIGTERM으로
종료**했다. 현재 원인은 대용량 전송 지연이며 로그의 unclassified copy error만으로
quota·인증·특정 네트워크 원인을 확정하지 않는다. rclone 누적 bytes에는 재시도 byte가
섞여 원본 크기를 넘었고 ETA 0도 표시됐지만 완료 증거가 아니었다. 새 실행의 E1/E2/best
외부 백업은 모두 미확정이며 `COMPLETE/LATEST` 검증 완료로 보고하지 않는다.
원격 checkpoint는 삭제하거나 변경하지 않았다. 앞서 확인한 원본 E1 및 복구 E3의 실제
Drive full SHA 검증은 유효하지만 새 E2의 복원 가능성을 대신 증명하지 않는다.

현재 실행기의 백업 watcher는 실패를 노출하고 120초 뒤 재시도하며 최신 full epoch부터
선택한다. 학습 실패/완료 뒤에도 남은 파일이 확인될 때까지 관측한다. 사용자의
`--resume`은 GPU 학습과 별도 SSH 통신/백업 worker를 함께 시작한다. agent는 이 요청에서
상주 watcher나 E3~24 본학습을 시작하지 않았다. 새 E2의 외부 백업 완료를 확인하기 전
Vast 인스턴스를 Destroy하지 않는다. 백업 지연을 학습 정상성·최종 정확도 문제와 구분한다.

최종 로컬 `bash scripts/runtime/start_base5_vast_training.sh --check --resume`도
실제 통과했다(`ready_without_training`). 영수증은 로컬 ignored artifact의
`base5_oneclick_261011/preflight.json`에 기록했다. GPU 학습·상주 watcher를 만들지 않고
임시 SSH 중계는 검사 종료 시 해제했다. 사용자는 같은 명령에서 `--check`만 빼고 재개한다.

<a id="base5-animal-pedestrian-merge-261011"></a>

## 29. Animal → Pedestrian 학습 통합 — 2026-10-11

사용자가 이번 학습에서는 동물을 Pedestrian으로 합치고, 학습 완료 후 성능을 보고
추가 수집 여부를 결정했다. 자산별 사례 수가 적다는 이유만으로 증량하지 않는다.
이 결정은 이전 4클래스 E2 재개보다 우선하며 통합의 성능 개선을 검증한 결과는 아니다.

### 파생 라벨과 보존

`prepare_base5_pedestrian_merged.py`는 신뢰하는 4클래스 portable의 infos를 새 출력으로
변환한다. Animal 3을 Pedestrian 1로 바꾸고 각 instance에 `original_bbox_label_3d`와
`original_semantic_class`를 추가한다. 사람·반려동물·유모차 등의 개별 BBox를 합치지 않는다.
raw audit·자산명·component ID·BBox·LiDAR 점 수·가시성·장면 순서·train/val split과
센서 파일은 유지한다. source/target 전체 payload SHA 검사를 수행하고 새 manifest와
변환 전 manifest·통계 provenance를 기록한다. 공유 payload는 hardlink이며 편집하지 않는다.
새 환경도 canonical raw/4클래스 infos를 보존한 후 같은 도구로 파생하면 된다.

512×1056 기준 GT는 다음과 같다. 5,000장 / train 4,255 / val 745 / test 0은 유지한다.

| 구분 | Vehicle | Pedestrian | Obstacle | 원래 Animal, Pedestrian에 포함 |
|---|---:|---:|---:|---:|
| Train | 4,178 | 2,557 | 2,615 | 247 |
| Val | 341 | 687 | 674 | 34 |

원래 Animal 247/34에는 Dog1 45/3, Dog2 40/7, Deer1 42/6,
Ped_Pet1의 동물 component 120/18이 있다. 이는 클래스 instance 수이며 독립 장면 수가 아니다.
로컬 파생본은 `artifacts/handoff/base5_sunny13_jpeg90_pedestrian_merged_v1_261011`이다.
최초 일반 사용자 hardlink 시도는 root 소유 JPEG의 OS hardlink 보호로 실패했다.
원본은 보존하고 컨테이너 root에서 새 v1 출력으로 완료했다. 실패한 부분 출력은 학습에 쓰지 않는다.

### 학습·평가·추론 계약

- 새 config는 `bevfusion_base5_fine_512_depth25_pedestrian_merged.py`다. head 3클래스,
  dataset metainfo, ObjectNameFilter, train/val/test 라벨을 일치시킨다. 기존 4클래스 후보는 보존한다.
- direct runner는 명시적 `animal_to_pedestrian_v1` 정책, 원래 클래스와 변환 라벨의 일치,
  infos/manifest/config/head의 일치를 검사한다. 4클래스 데이터에서 animal을 조용히
  버리거나 4클래스 checkpoint를 새 클래스 실행으로 resume하지 않는다.
- MMDet3D 기본 parser는 이름에 `label`이 있는 모든 필드를 remap한다. 원래 label 3을
  provenance로 남긴 필드는 기본 parser에 전달하지 않고, 학습 label 파싱 후 원본 값을
  복원하도록 새 dataset adapter에서 보완했다. 단위 검사와 실제 전체 val 파싱을 통과했다.
- `MoraiBase5MergedCenterMetric`은 기존 3클래스 AP 지표를 유지하고 원래 Pedestrian과
  Animal의 1m one-to-one recall을 score 0.05/0.10/0.30에서 추가한다. 진단용 cutoff이며
  배포 threshold를 확정한 값이 아니다. 모든 human/animal GT에 함께 매칭하므로 같은
  예측을 사람·동물 모두의 성공으로 중복 계산하지 않는다. `subgroup/original_animal/...`
  및 asset별 지표를 W&B에 기록한다. asset별 Ped_Pet1은 사람·동물 component를 함께
  포함하므로 동물만 분석할 때는 original semantic 메타데이터를 함께 사용한다.
- 3클래스 mean AP와 4클래스 mean AP는 분모와 분류 과제가 다르므로 직접 비교하지 않는다.
  recall만으로 false positive가 해결됐다고 보지 않고 통합 Pedestrian AP·precision도 확인한다.
- 새 inference runtime config는 동일 이름의 `_runtime.py`다. camera training compile은
  OFF로 바꾸고 별도 추론 최적화는 기본 OFF다. Base5 runtime은 명시적 3클래스 정책을
  추가 지원하지만 4클래스 가중치를 로드하며 label만 바꾸지는 않는다. production E25는 유지한다.

### 실제 검증 범위

로컬 단위·회귀 검사 18개, 사용자 실행기 검사 15개, 추론 최적화·계약 회귀 검사 8개를
통과했다. 실제 5K의 두 crop
profile 전체 split을 역변환 비교해 라벨·추가 provenance 외 데이터가 같은 것을 확인했다.
745장 val의 실제 dataset adapter 및 metric에 perfect-GT 예측을 넣었을 때 center mean AP와
사람·동물·asset subgroup recall은 모두 100%다. 이는 라벨·metric oracle이며 모델 정확도가 아니다.
로컬·원격 새 512×1056 infos의 SHA256은 train/val/test 모두 일치한다.

4090에서는 원래 Animal을 포함한 7장으로 batch 4→3, 네 번의 실제 AMP iteration을
실행하고 첫 두 번 후 checkpoint에서 재개했다. model/optimizer·scaler/scheduler와
Python/NumPy/Torch/CUDA RNG·iteration의 복원을 모두 확인했다. 최종 가중치는 유한값이며
partial checkpoint 파일은 없다. 최종 report는 원격
`/workspace/base5_pedestrian_merge_smoke_v2_261011/report.json`, 로컬
`artifacts/evaluation/bevfusion/base5_pedestrian_merge_261011/smoke_report.json`이다.
이 짧은 subset은 본학습에 섞지 않으며 1~2개 전체 epoch의 성능 시험으로 보고하지 않는다.
첫 smoke의 report에 고정 4개 클래스 목록이 남은 표시 오류를 수정하고 새 v2에서 재검증했다.

새 본학습의 원격 source/data/work·계약·명령은 [Vast 최신 실행 절](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-user-oneclick-training-261011)을
따른다. camera+sparse XY 초기화와 fresh query prior·FP32 matching 보완을 유지한다.
4클래스 E2의 optimizer나 검출 head를 자동 이식하지 않는다. agent는 24 epoch 본학습과
상주 backup watcher를 시작하지 않았다. 다음은 사용자가 새 실행을 시작하고 완료 후
사람·동물별 검출을 확인하는 것이다. 추가 수집과 정책 재변경은 그 결과를 보고 결정한다.

최종 로컬 `bash scripts/runtime/start_base5_vast_training.sh --check`는 실제 원격 source/
data/init/3클래스 계약, 유휴 48GB GPU, W&B 온라인 probe 재읽기, Drive 소량 쓰기·재읽기를
통과했다(`ready_without_training`). 새 실행기를 최상위 단일 파일로 배포했을 때의
부모 디렉터리 깊이 오류는 별도 실행기 디렉터리로 배포해 해결했다. 임시 통신 중계는
검사 종료 후 해제했으며 본학습·상주 watcher는 만들지 않았다. 다음 명령은
`--resume` 없는 새 시작이다. 기존 4클래스 E2의 외부 백업 미확정 상태는 유지하며
이번 작은 Drive 연결 검사를 full checkpoint 백업 완료로 해석하지 않는다.

<a id="base5-full-initializer-trial-261011"></a>

## 30. 호환 nuScenes 가중치 추가 재사용의 제한 비교 — 2026-10-11

사용자가 현재 초기화와 추가 재사용 후보의 짧은 비교를 승인했다. 각각 1 epoch 후
전체 validation을 비교하고 애매할 때만 2 epoch까지 확인한다. 24 epoch 본학습은
승인 범위가 아니며 기존 체크포인트·데이터·production E25는 보존한다. 비교 종료 후
사용자 실행기의 대상만 채택한 후보 E1로 전환하며 본학습은 자동 시작하지 않는다.

### 후보와 사전 검사

공식 nuScenes MORAI3 adapter SHA `2a9ce3e01a723581f8e76b247887a704c4751f2ebcc9b36884c266466700a247`에서
추가 233개 tensor를 추출한다. E25 학습 결과는 쓰지 않는다. 기존 camera+sparse 337개
tensor는 그대로 유지하고 전체 후보는 570개 tensor다.

- `view_transform.downsample`, SECOND/FPN, fusion, head의 공간 Conv2D/ConvTranspose2D
  커널 21개에서 마지막 X/Y 축을 교환한다. 영상 평면의 depth context convolution은 교환하지 않는다.
- learned self/cross position encoding의 첫 1×1 Conv1D를 물리 좌표가 같도록 affine 변환한다.
  원래 origin −54/−54m·pitch 0.6m에서 현재 origin −100.8/−33.6m·pitch 0.4m로 변경한다.
  center 최종 출력의 weight/bias는 1.5배로 변환해 같은 미터 단위 offset을 유지한다.
  dim/rot/height의 물리 의미는 교환하지 않는다.
- depth 118→396 bins의 logits는 seed 261010의 새 출력층 값으로 초기화한다.
  마지막 80개 context 출력 channel은 복사하고 depth 공통층을 재사용한다.
  depth logits를 100m까지 임의 외삽하지 않는다.
- dx/bx/nx/frustum은 현재 config에서 생성한다. optimizer·scheduler를 가져오지 않는다.
  이는 동일 함수 보장이나 AP 인증이 아니라 감사한 초기화 후보이며 해상도·센서 분포 차이는 남는다.

추가 변환/거부/보존 단위 검사 5개와 기존 sparse CPU 검사 2개를 통과했다.
CUDA sparse 검사는 기존 28절의 실제 qualification을 재사용하며 이번 CPU 실행에서는 1개 skip이다.
실제 pretrained camera BEV downsample·SECOND/FPN·fusion·dense head의 rectangular transpose
대조 최대 절대 오차는 각각 5.46e−7 / 5.36e−7 / 4.53e−6 / 1.91e−5다.
원격 실제 모델은 570개 tensor를 로드했고 missing은 위 4개 geometry뿐, unexpected는 없다.

추가 delta는 32,326,557 bytes로 한 번만 전송했다. 원격 후보는
`/workspace/base5_nuscenes_full_xy_metric_v1_261011.pth`, SHA
`a786b70ec7e639587da9c075fa7db9d3e96bf522b56590162c3134be7197b977`이다.
로컬 provenance는 `artifacts/evaluation/bevfusion/base5_full_initializer_261011/`에 보존한다.

### 통제 조건과 진행 상태

source는 기존 사용자 실행기를 보존한 별도
`/workspace/base5_initializer_ab_source_261011`이다. 데이터는 29절의 merged JPEG90이며
4,255 train / 745 val, 512×1056·공간 5/20cm·depth 25cm·batch 4·LR 1e−4·warm-up 500·
seed 261010·24 epoch schedule·학습 camera compile ON을 고정한다.
`run_base5_initializer_trial.py`는 원본 source config를 사용하고 epoch 경계의 full val 후 종료한다.
resolved config를 재개 입력으로 사용하지 않는다. 제한 검증의 W&B는 offline으로 보존하며
온라인 logging이나 대용량 Drive 백업 완료로 보고하지 않는다.

### 완료 결과와 결정 — 2026-10-11 KST

두 실행 모두 train 1,064 iteration과 745장 전체 val을 완료했다. 실제 실행 계약의
identity 비교에서 달라진 항목은 `initial_checkpoint_sha256`뿐이다. val GT는 차량 341개,
보행자 687개, 장애물 674개로 같으며 단위 검사 2개가 조건 변경·부분 실행을 거부한다.

| E1 지표 | camera+sparse 기준 | 추가 재사용 후보 | 차이 |
|---|---:|---:|---:|
| 차량 center AP | 59.69% | 84.09% | +24.39%p |
| 보행자 center AP | 58.39% | 82.72% | +24.32%p |
| 장애물 center AP | 28.93% | 43.78% | +14.84%p |
| 3클래스 평균 center AP | 49.01% | 70.19% | +21.19%p |
| 평균 IoU AP @0.25 | 54.87% | 73.35% | +18.48%p |
| 평균 IoU AP @0.50 | 25.49% | 49.89% | +24.41%p |

장애물 0~10/10~20/20~30/30~50m AP는 후보에서 59.32/57.05/36.23/12.27%다.
장애물 mATE는 0.298→0.149m, mASE는 0.629→0.464로 개선됐지만 mAOE는 약
1.50rad로 개선되지 않았다. 원래 Animal 34개 GT의 score 0.10·1m recall은
17.65→32.35%이며 score 0.30에서는 둘 다 0이다. 동물 정확도 문제 해결로 보고하지 않는다.

**추가 재사용 후보를 채택하며 2 epoch 비교는 생략한다.** 이는 동일 조건의 단일 seed
초기 수렴 비교다. 최종 성능·실주행·rosbag 평가를 인증하지 않으며 이전 Base3의 E1과
동일 평가 조건도 아니다. 초기화 영향은 확인했지만 남은 데이터·센서·구조 차이는 유지된다.

총 소요 시간은 기준 약 12분 53초, 후보 약 12분 3초다. 정상 train step은 둘 다
약 0.57초이며 준비·compile 시간 차이를 초기화의 학습 속도 최적화로 해석하지 않는다.
모델·optimizer의 floating tensor 1,675개는 각 실행에서 모두 유한하다. 두 실행 모두
AMP 초기 1회 update를 건너뛰어 실제 optimizer update는 1,063회다. full checkpoint에
optimizer·scheduler·메인 RNG·parameter names가 보존돼 있다. 이 후보의 장시간 재개는
아직 실행하지 않았으며 이전 29절의 짧은 실제 저장·재개 검증과 구분한다.

- 기준 work: `/workspace/base5_init_baseline_3class_e1_261011`.
  E1 SHA: `cc5dc832f34c0d9feadb6b8673b4d5b3d2f293055b59bec4fad80858ba037cb6`.
- 채택 work: `/workspace/base5_init_full_3class_e1_261011`.
  E1 SHA: `b81957a311ff020f8d46d0e96530f35e2a3dcb3b7632d999df750a584cb2c862`.
- 채택 계약: `ecddcbad9c2a5a8347a134c0af848fa46a6bf97261b39b605379825075713ca9`.
- 비교·상태·전체 지표·checkpoint health는 로컬 provenance의 `comparison.json`,
  `baseline/`, `candidate/`에 보존한다. trial W&B는 offline이며 새 E1 full Drive 백업은 미확정이다.

실행기 기본 대상은 위 후보로 전환했다. 원격 helper는 동결 source 바깥의 별도 폴더에
배포해 source hash와 비교 계약을 유지한다. `--check --resume`에서 원격 계약·W&B 온라인
연결·Drive 소량 쓰기/재읽기를 통과했다. 전체 24 epoch를 유지한 E1에서 사용자가 E2~24를
재개한다. work 이름의 `e1`은 비교 출처이며 본학습 종료 epoch를 제한하지 않는다.
실행기·비교 검사까지 합계 단위 검사 24개 통과, CPU 전용 CUDA 검사 1개 skip이다.
본학습·상주 watcher는 시작하지 않았다. 명령과 최신 경로는
[사용자 실행 절](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-user-oneclick-training-261011)을 따른다.

<a id="base5-far-distance-resume-261012"></a>

## 31. 원거리 거리별 평가 확장과 같은 W&B run 재개 — 2026-10-12 KST

사용자가 본학습을 시작한 뒤 원거리 지표 누락을 확인했다. 기존 거리 진단은
0~10/10~20/20~30/30~50m까지만 정의돼 있었지만 **전체 AP에는 ROI의 원거리 GT와
예측이 이미 포함돼 있다.** 전체 AP를 거리별 AP 평균으로 계산하는 것이 아니다.
기존 지표를 보존하고 새 원거리 진단을 두 구간으로 추가하기로 승인했다.

### 유지하는 계약과 평가 전용 확장

- Source/data/work는 30절의 실행 경로와 같다. 동결 source 파일은 수정하지 않는다.
- 훈련 계약은 `ecddcbad9c2a5a8347a134c0af848fa46a6bf97261b39b605379825075713ca9`로 유지한다.
- 모델·optimizer·24 epoch LR 스케줄·초기화·4255/745 split·증강·batch·RNG 재개 경로는 같다.
- `scripts/runtime/base5_distance_diagnostics.py`를 동결 source 바깥에 배포한다.
  실행기가 확장 SHA256을 별도로 확인한 뒤 기존 훈련 entrypoint를 호출한다. 기존 source
  hash guard와 checkpoint 계약 검사를 완화하거나 우회하지 않는다.
- 오직 Base5 merged metric에 진단을 덧붙인다. 기존 global/default metric을 바꾸지 않는다.
  단위·통합 검사에서 기존 모든 AP·IoU·recall scalar와 ROI 경계 처리가 동일함을 확인했다.
- 실행 확장 receipt와 resolved config를 별도 파일에 보존하고, 이후 checkpoint metadata에
  확장 version/SHA256을 남긴다. 이전 체크포인트를 변환하거나 덮어쓰지 않는다.

### 범위·키·지원 정답 수

거리 기준은 ego/LiDAR XY의 `sqrt(x²+y²)`이며 모든 bin은 `[하한, 상한)`이다.
새 핵심 키는 `val/range/50_80m/{vehicle,pedestrian,obstacle}_mean_AP`와
`val/range/80_100m/{vehicle,pedestrian,obstacle}_mean_AP`다. 0.5/1/2/4m 중심 매칭 허용
거리에 대한 클래스 AP 평균으로 기존 bin과 같다. 1은 100%다.

구간별 `{class}_GT`, `{class}_available`, `{class}_small_sample`도 기록한다.
GT 0이면 새 AP 키를 생략한다. GT가 있지만 검출하지 못하면 AP 0이다.
`small_sample=1`은 GT 1~19개로, 성능 실패나 수집 불합격을 의미하지 않는다.
기존 가까운 구간은 AP를 다시 계산하지 않고 기존 GT 수와 일치하는 지원 정보를 덧붙인다.

차량 전방/후방은 x≥0/x<0으로 나누고 같은 두 원거리 bin의
`val/range_direction/{50_80,80_100}m/{front,rear}/vehicle_mean_AP`와 `vehicle_GT`를 기록한다.
동적 속도 지표나 tracker 평가를 추가하는 것은 아니다.
직사각형 ROI의 방사 거리 100m 이상 모서리 GT는 전체 AP에 그대로 포함하고
`val/range_support/radial_ge100m/{class}_GT`에 개수만 기록한다. 세 번째 AP bin은 만들지 않는다.

### W&B 패널 구성

기존 W&B run ID `base5_init_full_3class_e1_261011-ecddcbad9c2a`를 유지한다.
기존 기록을 지우거나 epoch를 1부터 다시 시작하지 않는다. 새 지표는 적용 뒤 validation부터
나타나며 이전 epoch의 누락한 원거리 지표를 소급 추정하지 않는다.

| 섹션 | 제목 | X축 | Y축·선택 키 |
|---|---|---|---|
| `Val — Distance section` | `50–80m 클래스별 AP` | `epoch` | `val/range/50_80m/vehicle_mean_AP`, `pedestrian_mean_AP`, `obstacle_mean_AP`의 세 곡선 |
| `Val — Distance section` | `80–100m 클래스별 AP` | `epoch` | 위와 같되 `80_100m` |
| 같은 섹션, 선택 사항 | `원거리 차량 — 전방·후방 AP` | `epoch` | `val/range_direction/{50_80,80_100}m/{front,rear}/vehicle_mean_AP`의 네 곡선 |
| 같은 섹션, 선택 사항 | `원거리 AP 지원 GT 수` | `epoch` | `val/range/{50_80,80_100}m/{class}_GT`의 여섯 곡선 |

핵심 AP 패널은 두 개만 추가한다. AP의 Y축은 0~1로 고정하거나 표시 수식 `100*y`로
0~100%로 표시한다. GT 수 패널의 Y축은 개수다. 자동 노출되는 AP 키를 정의했지만
사용자 workspace의 기존 수동 섹션 배치를 API로 수정한 것으로 보고하지 않는다.

### 구현 검사와 실제 재개

로컬 CPU 컨테이너에서 단위·통합 검사 28개를 통과했다. 범위 경계·전후방·GT 없음과
미검출 구분·소표본·기존 전체 scalar 보존·ROI 모서리·동결 source와 확장의 별도 hash
guard·중복 실행 방지·stop 로그 형식을 검사했다. 원격에서 확장 import·중복 설치 방지와
기존 source/config 계약의 prepare-only 검사를 수행한다. 실제 에폭 종료·재개·새 validation
온라인 반영 여부는 아래 실행 확인에 추가한다.

**00:31 KST 중단 확인:** 10 epoch의 1,064 train iteration과 745장 전체 validation을
완료한 뒤 PID를 확인한 SIGINT로 중단했다. 종료 코드 −2는 사용자 요청의 중단 신호다.
원격 `epoch_10.pth` SHA256은
`1b5c57c61f059057927df751116c692ebeeeb6d101760698b5ce863c51c805a0`이다.
epoch 10 / iter 10,640·훈련 계약·optimizer·scheduler·메인 RNG·parameter order 정보를
확인했으며 모델 floating tensor 505개는 모두 유한하다. 저장된 LR은
`6.50525026138152e-05`다. 중간 iteration부터 정확히 재생한다는 주장은 하지 않는다.

첫 재개 시도는 W&B의 기존 config `load_from=epoch_1.pth`를 `epoch_10.pth`로 바꾸는
기록을 거부해 Runner 생성 단계에서 종료됐다. optimizer update 이전의 로깅 문제이며
checkpoint와 모델은 변경하지 않았다. MMEngine W&B backend의
`init_kwargs.allow_val_change=True`로 **재개용 로깅 metadata** 갱신을 허용했다.
훈련 source/config/checkpoint 계약 guard는 유지한다. 진단 훅과 이 W&B flag를 제외한
전체 resolved config의 동일성을 다시 확인하고 같은 run ID로 재시도한다.
확장 SHA256은 `e692c2d3147404d3f454bb4c15d2d8eae4685d04308d19b3113beac1ea25f8d0`이며
로컬 증거는 `artifacts/evaluation/bevfusion/base5_far_distance_resume_261012/`에 보존한다.

**00:39 KST 실제 재개 확인:** 원본 source config와 `epoch_10.pth`에서
`resumed epoch: 10, iter: 10640`을 확인하고 11 epoch 학습으로 진입했다.
`resume_start.json`의 LR은 위 저장 값과 같고 optimizer step 10,638, AMP scaler 256,
메인 RNG·scheduler·parameter order guard가 복원됐다. MMEngine iteration 수와 실제 Adam
update 수가 다른 것은 AMP가 비유한 gradient의 update를 건너뛴 기록이며 epoch 재설정이 아니다.
정상 학습 step은 약 0.567초로 유지된다. W&B API에서 같은 run ID의 `running`, epoch 11,
기존 history 유지와 `load_from=epoch_10.pth` 갱신을 확인했다. 기존 온라인 기록은 E2부터이며
E1 offline trial을 소급 업로드한 것은 아니다.

**00:51~00:54 KST 전체 validation·온라인 반영 확인:** 11에폭의 전체 745장 validation과
체크포인트를 완료했고 12에폭으로 자동 진행했다. 진단 receipt는 epoch 11 / iter 11,704이며
11에폭 체크포인트의 확장 version/SHA256·기존 훈련 계약 보존을 확인했다.
전체 GT는 차량 341 / 보행자 687 / 장애물 674개로 기존과 같다. 전체 center mAP는 83.29%다.

| 새 거리 진단 | 차량 AP / GT | 보행자 AP / GT | 장애물 AP / GT |
|---|---:|---:|---:|
| 50~80m | 61.65% / 51 | 17.50% / 46 | 36.89% / 40 |
| 80~100m | 26.94% / 6 | 0.00% / 3 | 0.00% / 4 |

차량 50~80m 전방/후방 AP는 59.20/97.15%이고 GT는 47/4개다.
80~100m 전방/후방 AP는 0.18/98.89%이고 GT는 4/2개다. 매우 적은 후방·초원거리 표본으로
일반화 성능이나 안전을 인증하지 않는다. ROI 안의 방사 거리 100m 이상 GT는 세 클래스 모두 0이다.
이는 11에폭의 중간 학습 결과이며 마지막 수렴 성능으로 해석하지 않는다.

W&B API의 실제 history에서 `epoch=11`과 새 AP 값의 일치를 확인했다. `summary='max'`를
정의한 핵심 AP의 summary 저장 키는 `..._mean_AP.max`지만 **차트의 Y축은 history의
`..._mean_AP`**를 사용한다. 매 epoch 성능 그래프에 `.max`를 고르면 안 된다.
GT·방향별 AP도 온라인 summary에서 확인했다. 사용자 수동 workspace의 패널 위치는
수정하지 않았으며 위 표의 제목·키로 두 핵심 패널을 추가하면 된다.
전체 학습·watcher는 그대로 진행하고 인스턴스를 Stop/Destroy하지 않았다.

### 후속 보완: 구간 전체 mAP 누락 — 2026-10-12 KST

사용자가 `val/range/50_80m/mAP`, `val/range/80_100m/mAP`가 보이지 않는다고 확인했다.
v1 확장은 클래스별 AP만 가져오고 세 클래스 평균인 구간 mAP를 누락했다. 기존 0~50m의
구간 mAP는 계산·기록됐지만 자동 노출 정의도 빠져 있었다. 학습·전체 AP 계산 오류가
아니며 앞선 ‘지표 추가 완료’ 설명에 이 누락을 명시하지 못했다.

v2 평가 전용 확장은 다음을 보완한다. 모델·훈련 source·데이터·LR 스케줄은 바꾸지 않는다.

- 50~80/80~100m의 mAP를 차량·보행자·장애물 `mean_AP`의 동일 비중 평균으로 기록한다.
  각 클래스의 AP 자체는 기존 0.5/1/2/4m 중심 매칭 AP 평균 그대로다. GT 수 가중 평균이나
  두 거리 구간의 평균은 아니다.
- 세 클래스 모두 GT가 있을 때만 새 구간 mAP를 기록한다. 클래스가 하나라도 GT 0이면
  mAP 키를 생략하고 `mAP_available=0`, `mAP_GT_classes`에 지원 클래스 수를 남긴다.
  지원 클래스만 평균해 같은 이름의 지표 의미가 바뀌거나 GT 없는 클래스를 0점으로 처리하지 않는다.
  GT가 있으나 예측이 없는 클래스의 AP 0은 평균에 그대로 포함한다.
- 기존 네 구간과 새 두 구간, 총 여섯 구간의 `/mAP`를 W&B에서 `epoch` 축·visible로 정의한다.
  기존 0~50m mAP 값과 계산 방식은 바꾸지 않는다.

| 섹션 | 제목 | X축 | Y축 |
|---|---|---|---|
| `Val — Distance section` | `50–80m 전체 mAP` | `epoch` | `val/range/50_80m/mAP` |
| `Val — Distance section` | `80–100m 전체 mAP` | `epoch` | `val/range/80_100m/mAP` |
| 같은 섹션, 기존 네 구간 | `0–50m 구간별 mAP` | `epoch` | `val/range/{0_10,10_20,20_30,30_50}m/mAP` |

Y축은 0~1이며 1이 100%다. `.max`는 summary용 이름으로 epoch 그래프에는 사용하지 않는다.
기존 클래스별 AP와 지원 GT 수 패널도 유지한다. 사용자 수동 workspace의 섹션 배치를
직접 수정한 것은 아니므로 필요하면 위 키로 패널을 추가한다.

`scripts/runtime/backfill_base5_distance_maps.py`는 실제 저장된 validation receipt에서만
세 클래스 평균을 복원한다. 훈련 worker와 같은 파일 lock을 잡아 W&B 동시 writer를 금지한다.
기존 run에서 `resume=must`로 원래 epoch 좌표에 **새 history 행을 추가**하며 기존 행이나
W&B 내부 `_step`을 덮어쓰지 않는다. 이미 같은 값이 있으면 생략하고 다른 값이면 중단한다.
원거리 클래스 AP가 저장되기 전인 E1~10을 추정하거나 추가 validation하지 않는다.
기존 receipt·checkpoint도 수정하지 않는다. mAP만 복원하므로 학습을 처음부터 다시 하지 않는다.

로컬 CPU 단위·통합 검사 33개를 통과했다. 클래스 동일 비중 평균·GT 없음과 미검출 구분·
반올림 오차·복원 중복/충돌 거부·여섯 visible mAP 정의·기존 scalar 보존·별도 hash guard를 검사했다.
v2 확장 SHA256은 `7248640fccdfb5538a4fa32ccca282b8b5682c9d1761d2914e384e5cb3e93818`이다.
위 v1 해시는 최초 적용 당시의 기록으로 보존한다.

**00:37~00:38 KST 실제 복원 확인:** E15의 train 1,064 iteration과 745장 전체 validation을
완료한 뒤 PID 10,897을 SIGINT로 중단했다. E15 체크포인트 SHA256은
`5ee3436cad2662230cae80435fdb43e3d293dd938e6ea652d1cc799e2af1904f`이며 epoch 15 / iter 15,960이다.
E11~15의 원거리 mAP 열이 W&B history에서 비어 있음을 확인한 뒤 동일 run에 새 행 5개를
추가했다. API의 exact history 재읽기로 두 mAP 키 모두 원래 epoch·계산값과 일치함을 확인했다.
W&B 내부 history step 770~774에 추가한 것이며 기존 train/val 행을 교체하지 않았다.

| epoch | 50~80m mAP | 80~100m mAP |
|---|---:|---:|
| 11 | 38.68% | 8.98% |
| 12 | 30.30% | 10.17% |
| 13 | 36.00% | 10.00% |
| 14 | 31.76% | 10.48% |
| 15 | 34.23% | 9.67% |

80~100m의 GT는 차량 6 / 보행자 3 / 장애물 4개로 소표본이다. 이 중간 결과와 평균을
일반화 성능 또는 안전 인증으로 해석하지 않는다. 원본 receipt와 추가 복원 근거는 로컬
`artifacts/evaluation/bevfusion/base5_distance_map_repair_261012/`에 따로 보존한다.

**00:40~00:42 KST 재개 확인:** v2 확장과 별도 SHA guard를 배포하고 같은 사용자 실행기로
재개했다. 훈련·W&B·Drive 사전 검사를 통과했으며 E15 / iter 15,960에서 E16으로 진입했다.
`resume_start.json`의 LR은 `3.263829998845258e-05`, optimizer update 수는 15,956,
AMP scaler는 256이다. 같은 계약과 메인 RNG·optimizer·scheduler 복원 경로를 유지한다.
E16의 80 iteration에서 정상 step 약 0.566초와 유한 loss를 확인했다. 새 console은
`console_resume_1791736792.log`다. 재개 초기의 ETA 증가는 warm-up 평균이 반영된 것이며
새 로깅을 위한 추가 모델 연산이나 추가 validation은 없다. E16 전체 validation의 새 자동
mAP 기록은 이후 정상 에폭 종료 때 확인할 수 있다. 이번 실제 온라인 검증은 E11~15 복원분이다.
본학습·Drive watcher는 계속 진행하며 인스턴스를 Stop/Destroy하지 않는다.

<a id="base5-range-evidence-local-261012"></a>

## 32. 원거리 GT 정보량·다음 환경 배치의 로컬 조사 — 2026-10-12

### 조사 범위와 보존

사용자가 원격 학습 중 로컬에서 원거리 성능과 다음 수집 방향을 조사하도록 요청했다.
이번에는 Vast에 접속하거나 학습·W&B·수집기·생성기·GT·threshold를 변경하지 않았다.
현재 Sunny13 JPEG90/3클래스 파생 입력 5,000장과 GT 11,052개를 전수 대조했다.
train 4,255장/9,350 GT, val 745장/1,702 GT다. GT가 있는 장면은 4,750장이고
나머지 250장도 전체 입력에 포함된 정상 배경 장면이다.

`tools/audit_base5_range_evidence.py`는 현재 infos·native audit·이전 전수 native 점 검사·
실제 instance mask의 512×1056 crop 검사를 읽기 전용으로 결합한다. 기존 crop 검사의
source ledger SHA256과 현재 manifest의 SHA256이 일치하는지 검사한다. 해시는
`026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43`다.
native 점 캐시는 token/component/자산/중심/원본 점 수로 대조하며 중심 밖 ROI의 점을
새로 포함하지 않는다. 센서를 재스캔하거나 BBox를 변형한 것은 아니다.
E15 validation receipt의 18개 클래스·거리 GT 분모도 현재 val과 모두 일치했다.

최종 산출물은 `artifacts/evaluation/bevfusion/base5_range_evidence_release_v2_261012/`다.
`report.json`, `objects.json`, `range_support.png`, `far_input_examples.png`,
`review_samples.json`, `rosbag_inventory.json`을 사용한다. 앞선 파일럿 디렉터리는
진단 이력이다. 이미지 예시는 실제 Q90 JPEG를 OpenCV cubic resize와 현재 bottom crop으로
읽어 만든 비교 그림이다. 확대는 센서 정보를 새로 만드는 것이 아니다.

### 정보량 기준과 전체 결과

- 약한 단서: 현재 필터 후 LiDAR 0~4점이고, 실제 카메라 instance 픽셀 ≥16 및
  clipped 2D BBox의 최소 변 ≥4px 조건도 충족하지 않은 GT다.
- 풍부한 단서의 **진단용 proxy**: LiDAR ≥20점 또는 한 카메라의 실제 instance 픽셀
  ≥64 및 최소 변 ≥8px다. 이 숫자는 학습 라벨 필터나 새로운 캡처 승인 기준이 아니다.
- 같은 부모·같은 클래스의 복합체 카메라 대응이 모호하면 `unknown`으로 둔다.
  모호함을 안 보인다는 뜻으로 해석하지 않는다. 픽셀 수는 실제 보이는 foreground이며
  투영 BBox 면적과 다르다. 안개 속 대비·분류 가능성·depth 정확도까지 보장하지 않는다.

| 현재 5K의 GT 구분 | 수량 | 전체 GT 대비 |
|---|---:|---:|
| 풍부한 단서 proxy | 9,650 | 87.31% |
| 약한 단서 proxy | 117 | 1.06% |
| 카메라 대응 unknown | 223 | 2.02% |
| LiDAR 0점, camera-only GT | 679 | 6.14% |

네 행은 서로 배타적인 합계가 아니다. camera-only 중에도 충분한 카메라 단서가 있다.
unknown 중 LiDAR 0~4점인 97개는 약한 단서로 단정하지 않았다. 모두 약하다고 가정한
보수 상한은 214/11,052=1.94%지만 실제로 확인한 비율은 1.06%다.
train만의 약한 단서는 98/9,350=1.05%, 풍부한 proxy는 8,257/9,350=88.31%다.
0~30m 전체의 풍부한 proxy는 7,389/8,206=90.04%다. 선명한 사례가 전반적으로 없다는
설명은 이 자료와 맞지 않지만, 이것만으로 검출 성능이나 학습 충분성을 인증하지 않는다.

현재 학습 GT에서 제외된 native component는 320개다. 318개는 저장된 positive 조건
미충족, 2개는 ROI 등 나머지 조건이다. 과거 320 crop의 413개 제외와 다른 집계이며
현재 crop·파생 infos 기준이다. 이번 분석으로 제외 정책을 새로 바꾸지는 않았다.

### 거리별 정보량과 E15 중간 성능

아래 중앙값은 해당 클래스·거리의 **train+val 전체** GT 기준이다. 카메라는 대응이
확실한 GT의 최다 foreground 관측을 사용한다. AP는 E15의 **val** 중간 결과이며
점 수나 픽셀 수와 같은 모집단의 per-object 오검출 분석은 아니다.

| 거리 | 클래스 | train GT | val GT | LiDAR 점 중앙값 | 보이는 카메라 픽셀 중앙값 | E15 center AP |
|---|---|---:|---:|---:|---:|---:|
| 30~50m | Vehicle | 1,046 | 97 | 12 | 662 | 91.13% |
| 30~50m | Pedestrian | 311 | 145 | 2 | 102 | 76.86% |
| 30~50m | Obstacle | 324 | 91 | 1 | 128 | 27.62% |
| 50~80m | Vehicle | 377 | 51 | 6 | 251 | 60.58% |
| 50~80m | Pedestrian | 25 | 46 | 1 | 49 | 12.93% |
| 50~80m | Obstacle | 55 | 40 | 1 | 54 | 29.18% |
| 80~100m | Vehicle | 213 | 6 | 4 | 104 | 29.00% |
| 80~100m | Pedestrian | 3 | 3 | 0 | 26.5 | 0% |
| 80~100m | Obstacle | 9 | 4 | 0 | 34 | 0% |

확인한 해석은 다음과 같다.

1. 50~80m 사람 71개는 모두 LiDAR 0~4점이며, 80~100m 사람 6개는 모두 0점이다.
   후자의 카메라 foreground는 중앙값 26.5px다. 이 범위의 낮은 AP는 센서 정보량과
   작은 학습 표본이라는 설명에 부합하지만 원인을 분리한 실험은 아니다.
2. 30~50m 장애물은 val 91개 중 89개가 카메라 조건을 통과하고, 69개가 풍부한 proxy다.
   그런데 AP는 27.62%다. **안 보이는 라벨 때문이라고만 설명할 수 없다.**
   1m AP 23.80%/4m AP 34.36%라 위치 허용 오차를 늘려도 전부 회복되지는 않는다.
   검출 누락·FP·score 순서·크기/자산·depth/위치 문제를 실제 prediction과 대조해야 한다.
3. 원거리 50m 이상 사람·장애물은 전체 185개, 그중 127개가 음영 family다.
   대부분 전방 관측이므로 일반 후방 사람 분포를 과도하게 늘린 결과도 아니다.
   80~100m의 val은 클래스마다 3~6개에 불과해 0% 또는 높은 AP를 안정된 성능으로
   단정하지 않는다. 차량 50~80m 후방 val도 4개뿐이다.
4. train의 모든 클래스 50m 이상 GT는 682/9,350=7.29%이고, 사람·장애물만은
   92/9,350=0.98%다. 원거리 사람·장애물 때문에 전체 학습이 망가졌다는 증거는 없다.
   반대로 음의 전이가 전혀 없음을 증명한 제거 ablation도 수행하지 않았다.
5. 실제 이미지에는 앞차에 가려져 일부만 보이는 원거리 차와, 작은 사람·얇은 장애물,
   LiDAR 0점이지만 RGB에서 형태가 남아 있는 바리게이트가 각각 존재했다.
   일부 어려운 사례를 남기는 것과 모든 자산의 100m 인식을 기대하는 것은 구분한다.

### 아직 촬영하지 않은 25K 명세도 확인

`~/morai_cap/base5_postcollection_261010/fresh_suite_final_v142`의 5환경·25,000개
metadata를 읽었다. 각 manifest SHA256과 metadata 집합 digest를 결과에 보존했다.
현재 Sunny13에서 Pedestrian 성분만 관측된 자산을 primary로 배치하고,
**계획 Ego 거리**가 50m 이상인 train parent 수는 다음과 같다.

| 예정 환경 | 50m 이상 사람 계열 train primary parent |
|---|---:|
| Sunny11 | 14 |
| Sunny15 | 11 |
| Foggy11 | 11 |
| Foggy13 | 9 |
| Foggy15 | 9 |

이는 **계획 parent 수이지 실제 유효 GT 수가 아니다.** 보조 객체·복합체 여러 성분,
실제 포즈·LiDAR 중심 거리·가시성·보충에 따라 최종 GT 수가 달라진다.
다만 연속 Ego 위치와 환경별 새 배치를 주어도 원거리 사람의 train 희소성이
자동 해소되지는 않는다는 경고로 사용할 수 있다. 장면 train/val=85/15가 각 클래스·
각 거리의 GT도 85/15라는 뜻은 아니다. 현재 사람 50~80m는 train보다 val GT가 많다.

### 수집 방향의 현재 추천과 남은 검증

- **완료 Sunny13 5K·진행 중 학습은 유지한다.** 이번 자료 때문에 처음부터 재수집하거나
  원거리 GT를 무조건 삭제할 근거는 없다. visible GT만 제거하고 RGB를 남기면 잘못된
  background supervision이 될 수 있으므로 ignore/loss 처리도 별도 설계 대상이다.
- **나머지 25K의 현재 전체 패밀리·자산·거리 계약을 즉시 뒤집지는 않는다.**
  먼 차량은 유지하고, 사람·장애물 전체를 80~100m로 늘리는 변경은 추천하지 않는다.
- 보강 우선순위는 **장애물 20~50m의 큰/작은 자산·가림 차이**, 다음으로 **실제 식별
  가능한 사람·장애물 30~60m train 사례**다. 모든 자산을 최대 거리까지 강제하지 않는다.
  숫자를 늘릴 경우 최종 모델의 clear/weak 분리 결과를 본 뒤 별도 소규모 보강 manifest로
  진행한다. 기존 frozen suite를 덮어쓰거나 같은 output에 강제 resume하지 않는다.
- 첫 Foggy 환경은 작은 pilot에서 JPEG의 대비·가림·camera-only 사례를 확인한 뒤 본
  5K로 진행하는 것이 좋다. instance mask 픽셀은 안개의 RGB 대비를 평가하지 않는다.
- 최종 checkpoint에서 동일 val의 clear/weak·자산·거리별 **고정 score P/R, TP/FN/FP,
  matched 위치 오차**를 확인한다. 중간 AP에서 tracker score threshold를 정하지 않는다.
- rosbag에서는 raw timestamp 기준 첫 전달 거리, 연속 누락 시간, 위치 흔들림,
  track 생성/유지/유령 잔존과 속도 MAE/2D AVE를 별도로 본다. AP는 프레임 검출 확률이
  아니므로 60% AP에서 40% 시간의 누락을 계산하지 않는다.

### rosbag 평가 준비 조사

`tools/inventory_base5_rosbag_cases.py`로 기존 계약이 있는 bag 22개를 실제 indexed
header로 확인했다. highway 9 / roundabout 4 / pedestrian 5 / obstacle 4이며 약
11.19GiB다. 전부 raw bag·asset-name map과 네 camera/compressed·LiDAR·Ego/Object
topic이 존재한다. MORAI·ROS 노드 재생·GPU 추론은 실행하지 않았다.

다음 제한을 확인했으므로 기존 명령을 그대로 Base5 100m 평가라고 부르지 않는다.

1. `detection_evaluator.py`는 기본 최대 거리 30m와 세 구간 0~10/10~20/20~30m다.
   `evaluate_rosbag_detection_visibility.py`도 기본 radial 30m·관측점 5개다.
   후속 Base5 평가 adapter에서 직사각형 ROI·여섯 구간·점/카메라 정보량을 명시해야 한다.
   이번에는 production/역사적 평가기의 기본값을 변경하지 않았다.
2. 기존 Base3 온라인 infos 변환기는 marker 기반이다. 이번 22개 미션 bag에는 수집
   marker가 없으므로 그대로 입력하면 유효 장면을 만들 수 없다. marker 없는 연속 bag용
   선택 구간·LiDAR 시각 동기화·미해결 GT mask를 별도로 설계해야 한다.
3. bag별 heading 0°/-1.347° 계약과 status-size v2 geometric BBox catalog를 유지한다.
   이름 미해결 객체를 임의 class/BBox로 추측하지 않는다. bag에는 synchronous native
   annotation이 없어 완벽한 센서 GT라고 주장하지 않는다.
4. 이 bag들은 이전 비교에 사용한 dev-val이다. `evaluation_locked` 이름만으로 독립
   미사용 test가 되지는 않는다. 최종 test는 사용 이력이 다른 bag 또는 새 주행으로 정한다.

이번에 완료한 것은 전수 정보량 조사·다음 25K 명세 조사·bag 준비 상태 확인이다.
최종 모델의 실제 누락 원인, 원거리 제거 ablation, 100m bag 변환·추론·tracking 평가는
아직이다. 학습 완료를 기다리는 동안 수집 준비와 CPU 변환기를 구현할 수 있지만,
그 작업이 이미 완료된 것처럼 기록하지 않는다.

### 재현과 검사

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 perception/detection3d/bevfusion/tools/audit_base5_range_evidence.py \
  --data artifacts/handoff/base5_sunny13_jpeg90_pedestrian_merged_v1_261011 \
  --vertical artifacts/evaluation/bevfusion/base5_vast_depth_z_261011/vertical_caps_depth_final_v2 \
  --crops artifacts/evaluation/bevfusion/base5_crop_final5k_261010 \
  --metrics artifacts/evaluation/bevfusion/base5_distance_map_repair_261012/epoch_15.json \
  --fresh-suite "$HOME/morai_cap/base5_postcollection_261010/fresh_suite_final_v142" \
  --output artifacts/evaluation/bevfusion/base5_range_evidence_new

/usr/bin/python3 perception/detection3d/bevfusion/tools/inventory_base5_rosbag_cases.py \
  --contract-root artifacts/evaluation/bevfusion/old_mission_current_contract_261004/e25 \
  --contract-root artifacts/evaluation/bevfusion/teammate_manual_261004/e25 \
  --output artifacts/evaluation/bevfusion/base5_range_evidence_new/rosbag_inventory.json

python3 -m unittest discover -s perception/detection3d/bevfusion/tools \
  -p test_base5_range_evidence.py -v
```

CPU 단위 검사 12개를 통과했다. 실제 픽셀과 BBox 면적 구분, 복합체 unknown,
camera-only의 정상 지원, 거리 경계, 약한/풍부한 proxy의 구분, 기존 AP 분모 대조와
불일치 거부, source 아래 output 거부를 검사했다. 전수 실행에서 infos·manifest·
점/픽셀 cache·E15 receipt의 분석 전후 SHA256도 일치했다. 산출물은 ignored로 유지한다.
