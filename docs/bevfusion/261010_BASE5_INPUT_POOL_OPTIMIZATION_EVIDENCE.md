# Base5 입력·격자 정합·pooling 최적화 — 2026-10-10

<a id="current--다음-판단"></a>

## 문서 역할과 읽는 방법

문서 상태: EVIDENCE. 현재 모델·완료 상태·실행 선택은 [CURRENT](CURRENT.md)를 따른다.
[주제 인덱스](README.md#evaluation)에서 관련 anchor의 절만 선택한다.
아래 각 절의 ‘현재’, ‘최신’, ‘미완료’는 해당 검증 날짜의 상태다. 과거 7.5/30cm·
4클래스·학습 미시작 기록을 지금의 실행 계약으로 복구하지 않는다.
실측·실패·표본과 기존 anchor는 보존하며 이 문서 앞에 현재 상태를 다시 복사하지 않는다.

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

<a id="base5-common30-e25-comparison-261012"></a>

## 33. 본학습 완료와 E25·Base5의 같은 30m 비교 — 2026-10-12 04:30 KST

### 완료 상태와 보존

사용자 완료 알림 후 원격 `oneclick_status.json`의 `finished / exit_code=0`과
E24 체크포인트를 확인했다. 학습은 24 epoch를 완료했으며 새 학습은 실행하지 않았다.
전체 ROI validation의 best는 E9다. E9·E24는 모두 보존한다.
Drive 백업 보고서는 `valid=true`, `pending_checkpoints=[]`, `errors=[]`이며,
두 파일 모두 분할 업로드와 로컬 credential owner의 readback을 기록했다.

| 파일 | SHA256 | 성격 |
|---|---|---|
| `best_val_center_mAP_epoch_9.pth` | `067205c3f5f5b0abc1bb775116c2b8bca2c9a7f31a0727b959db0ab1ce70b271` | 검출 best, optimizer 재개용 아님 |
| `epoch_24.pth` | `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746` | 마지막 epoch, optimizer 포함 재개 가능 |

원격 work는 `/workspace/base5_init_full_3class_e1_261011`이다. production E25,
threshold, 원본 5K, frozen 훈련 소스, 다음 25K 명세와 다른 agent의 planning_validator
작업은 변경하지 않았다. 인스턴스 Stop/Destroy도 실행하지 않았다.

### 동일 평가 계약

`tools/compare_base5_common30.py`로 학습된 세 체크포인트를 실제 추론했다.
E25는 로컬 5060 Ti, Base5 E9/E24는 원격 4090을 사용했다. 모두 strict state load,
FP32 경로이며 AMP·TF32·camera compile·camera/LiDAR 병렬 추론은 OFF다.
각 모델의 원래 카메라 전처리·공간 격자·depth·전체 추론 ROI는 유지한다.
Base5 모델의 구조를 30m 모델로 다시 만들거나 GT에 맞춰 prediction을 수정하지 않았다.

- 같은 Sunny13 Q90 validation **745장 전체**를 사용한다. ROI 내 객체가 없는 프레임도
  남겨 FP를 평가한다. Animal → Pedestrian의 현재 3클래스 라벨을 세 모델에 동일 적용한다.
- 주 비교 범위는 **LiDAR 좌표계 XY 중심 거리 <30m**와 E25 공통 box
  `[-30,-30,-3,30,30,2]`의 교집합이다. GT·prediction 모두 같은 범위로 제한한다.
  기존 evaluator와 같은 bottom-center BBox 표현을 사용한다.
- ±30m 정사각형은 별도 secondary 결과다. 모서리의 30m 초과 객체를 radial 30m라고
  부르지 않는다. 세 모델의 center AP는 같은 0.5/1/2/4m 매칭 기준의 평균이다.
- 공통 GT는 Vehicle **187**, Pedestrian **493**, Obstacle **539**, 총 **1,219개**다.
  세 prediction export의 ordered scene/GT/점 수/asset fingerprint는 모두
  `86ac7a02dc4542d362fef09422688a2efc116fdc38e7573fb144e0d1e82bf619`이다.
- LiDAR 0~4점 GT도 현재 Base5 라벨 정책대로 유지한다. 지원점별 recall을 낼 때는
  전체 GT에 먼저 one-to-one 매칭한 뒤 분류한다. 약한 GT만 지워 올바른 prediction을
  FP로 만드는 평가를 하지 않는다. native 점 수는 Z 필터 후 모델 입력점 수와 구분한다.

### 먼저 잡은 입력 스케일 불일치

처음 E25를 Base5 입력 그대로 평가하면 장애물 AP가 **1.03%**까지 떨어졌다.
이를 모델 성능으로 채택하지 않고 native 입력과 원래 검증셋을 확인했다.

1. Base3 native `.bin`의 intensity는 **0~255**, Base5 native `.bin`은 **0~1**이었다.
   Base5 validation 745개 파일 전수 검사에서 1 초과 프레임은 0개이며, intensity×255는
   전부 정확한 정수였다. infos 변환기와 portable exporter는 원본 bin을 참조·복사할 뿐
   강도를 정규화하지 않는다. 왜 native 저장 형식이 달라졌는지는 이번 조사에서 확정하지 않았다.
2. E25를 원래 Base3 validation 1,635장에 돌린 control의 기존 square AP는
   Vehicle **97.4705%**, Pedestrian **98.8025%**, Obstacle **97.9689%**였다.
   저장된 과거 장애물 AP **97.9755%**와 차이는 0.0066pp로, 평가 실행 경로가 E25를
   크게 훼손한 것은 아니다. control은 별도 데이터셋이며 아래 같은-data 표에 섞지 않는다.
3. 최종 E25 비교만 **입력 tensor의 intensity×255**로 원래 학습 스케일을 복원했다.
   XYZ·GT·이미지·원본 파일·가중치는 불변이다. Base5 E9/E24에는 변환하지 않는다.
   `--intensity-scale 255`는 명시적 opt-in이고, 입력이 0~1 범위를 벗어나면 거부한다.
4. 기존 dev bag 22개의 첫 LiDAR 메시지 각 2개, 총 44개도 읽었다. 모두 intensity 최대값이
   1을 초과했고 최대값은 135~255였다. 현재 ROS bridge와 online preprocessor는 이 값을
   그대로 전달한다. 따라서 **Base5를 ROS에 붙이기 전 고정 255로 나누는 입력 계약과
   offline/online 동등성 검증이 필요하다.** 프레임 최대값으로 나누는 정규화가 아니다.
   이 처리는 이번에 production에 적용하지 않았다. 현재 live MORAI 버전의 출력은 후속 확인한다.

### 같은 데이터·radial 30m의 최종 결과

단위는 %, center AP다. E25는 위 강도 adapter를 적용한 결과만 비교한다.

| 모델 | Vehicle | Pedestrian | Obstacle | 3클래스 mAP |
|---|---:|---:|---:|---:|
| E25 | 89.93 | 77.82 | 60.76 | 76.17 |
| Base5 best E9 | 97.66 | 95.60 | 81.95 | 91.74 |
| Base5 final E24 | 97.25 | 94.87 | 83.49 | 91.87 |

| 장애물 거리 | GT 수 | E25 | Base5 E9 | Base5 E24 |
|---|---:|---:|---:|---:|
| 0~10m | 206 | 64.68 | 83.09 | 85.34 |
| 10~20m | 190 | 72.73 | 81.85 | 86.13 |
| 20~30m | 143 | 28.77 | 75.82 | 76.74 |

**이 검증셋에서는 가까운 장애물까지 E25보다 못 배웠다는 증거가 아니다.** E24 장애물
전체는 +22.73pp, 20~30m는 +47.97pp다. E25의 옛 Base3 val 약 98%와 Base5 전체
±100m AP를 그대로 대조한 우려는 데이터·라벨 정책·거리 분모가 다른 비교였다.
다만 이번 데이터는 Base5 학습 분포이며 E25에는 다른 분포다. 이 표만으로 모든 실제
주행에서 우위, 새 자산 일반화, 정밀 격자나 depth 변경의 개별 효과를 증명하지 않는다.

### 기존 threshold를 그대로 쓰는 것은 별도 문제

같은 30m에서 score를 현재 production 값에 고정하고 중심 오차 <1m로 매칭한 P/R다.
AP와 달리 **실제 전달 threshold의 TP/FN/FP trade-off**이며 새 threshold의 최적값을 뜻하지 않는다.

| 클래스·score | E25 P / R | Base5 E9 P / R | Base5 E24 P / R |
|---|---:|---:|---:|
| Vehicle 0.40 | 100.00 / 81.82 | 98.79 / 87.17 | 97.74 / 92.51 |
| Pedestrian 0.15 | 93.37 / 65.72 | 92.46 / 92.09 | 94.82 / 89.05 |
| Obstacle 0.10 | 90.21 / 54.73 | 68.65 / 83.67 | 74.23 / 80.71 |

Base5 장애물은 많이 잡지만 0.1에서 FP도 늘었다. AP 향상이 기존 threshold의 즉시
배포 적합성을 보증하지 않는다. E24의 1m·score 0.1 장애물 recall은 native ≥5점
GT에서 **84.48% (332/393)**, <5점에서 **70.55% (103/146)**다.
지원점 많은 사례도 완벽하지 않으며 Bicycle/Scooter 같은 자산별 잔여 약점이 있다.
예를 들어 Scooter_01의 현재 obstacle recall은 E9 15/22, E24 4/22다. 복합 자산 전체
검출 실패율로 해석하지 않고 성분 단위 진단으로 보존한다. 특정 epoch 한 개를 전체
평균만으로 자동 production 승격하지 않는다.

### 산출물·재현·다음 순서

ignored 결과 루트는 `artifacts/evaluation/bevfusion/base5_common30_261012`다.

- `comparison_final_validated/comparison.json`: 최종 세 모델의 radial/square AP,
  거리·자산·native 지원점별 고정 score recall과 각 입력·checkpoint receipt.
- `control_base3/comparison.json`: E25의 원래 데이터 control. 최종 비교와 분모가 다르다.
- `comparison_unadapted/comparison.json`: 강도 스케일 미적용 진단. 최종 성능표로 사용 금지.
- `intensity_inventory.json`, `bag_intensity_samples.json`: 전수 native 입력과 bag 표본 근거.
- `training_completion.json`, `training_backup_report.json`: 완료·Drive readback·E9/E24 SHA 근거.
- `E25_intensity255`, `E9_v3`, `E24_v3`: prediction export와 receipt. 초기 wrapper import/
  token 오류 로그도 보존하되 완성 결과와 섞지 않는다.

```bash
python3 perception/detection3d/bevfusion/tools/compare_base5_common30.py compare \
  --predictions \
  artifacts/evaluation/bevfusion/base5_common30_261012/E25_intensity255/predictions.json.gz \
  artifacts/evaluation/bevfusion/base5_common30_261012/E9_v3/predictions.json.gz \
  artifacts/evaluation/bevfusion/base5_common30_261012/E24_v3/predictions.json.gz \
  --output artifacts/evaluation/bevfusion/base5_common30_repeat

python3 -m unittest discover -s perception/detection3d/bevfusion/tools \
  -p test_base5_common30.py
```

실제 추론은 준비된 BEVFusion Docker/원격 의존성 환경에서 도구의 `infer`를 사용한다.
각 receipt의 config·checkpoint·info SHA 및 pipeline을 유지한다. E25는 원래
`bevfusion_base3_b0_4cam_lidar.py`와 release inference checkpoint, Base5는 frozen
source와 `resolved_distance_diagnostics.py`를 사용한다. 초기화 전 Swin의 scratch
경고는 full strict checkpoint load 이전의 메시지이며 랜덤 가중치 추론이라는 뜻이 아니다.
CPU 단위 검사 10개에서 강도 adapter의 XYZ/원본 불변·비정상 입력 거부, radius/square,
희소 GT 보존, one-to-one, score·거리 경계, GT fingerprint·class·token·score guard와
빈 프레임을 검사했다. 비교 결과는 추론 latency 측정이나 24GB 실전 인증이 아니다.

다음 우선순위는 **Base5 ROS 입력 강도 계약·동등성 → 공통 주행 bag에서 E25/E9/E24의
검출·threshold·연속 누락 비교 → 학습된 실제 출력으로 추론 최적화·24GB 비용 재검증**이다.
100m GT adapter와 camera clear/weak 분석은 32절 제한을 유지한다. 재수집이나 장시간
재학습을 이번 비교만으로 결정하지 않는다.

<a id="base5-rosbag-quick-common30-261012"></a>

## 34. 대표 rosbag의 E25/E9/E24 빠른 공통 30m 비교 — 2026-10-12

04:53 KST 완료. 사용자가 대표 rosbag의 빠른 실제 평가를 요청했다. 기존 teammate
dev 기록에서 장애물 O02, 횡단 보행자 P01, 추월 고주로 H03, 로터리 R01을 선택했다.
**원본 bag 센서로 실제 모델 추론을 수행했다.** SaveSensorData val 재평가가 아니며,
ROS playback·tracker·planning을 실행한 실시간 폐루프 평가도 아니다.

### 입력과 GT 계약

- 각 bag의 전체 시간에 걸친 적격 LiDAR 프레임에서 결정론적 균등 index로 64장씩,
  총 **256장**을 선택했다. 객체가 없는 프레임도 FP 분모에 남긴다. E25/E9/E24의
  token·GT·점 수·asset fingerprint는 모두
  `0b68bc2afd5a496a49ba123469728fef3d1f5b6b6032342fb40d764a86f3ed14`다.
- 카메라는 nearest header stamp의 차이가 ≤50ms인 원본 JPEG를 사용했다. 실제
  선택된 최대 차이는 31.755ms다. Ego/Object 상태는 기존 선형 보간 계약의 ≤150ms
  bracket만 허용하고 외삽하지 않는다. Ego/Object heading 보정은 모두 −1.347°다.
- GT는 기존 `morai_asset_bbox_offsets_status_size_v2.json`과 각 case의
  `asset_name_map.json`으로 `/Object_topic`을 BBox 중심으로 변환한 것이다.
  **동기 native LiDAR annotation이 아니며**, 시간 보간·카탈로그 근사는 남는다.
  미해결 객체 근처 프레임은 기존 unresolved-frame 정책으로 제외한다. 지원되는
  GT는 LiDAR 점이 적거나 보이지 않아도 임의로 삭제하지 않는다.
- Sensor_4의 네 카메라 calibration은 Base5 native infos의 K/T와 대조했다.
  GT·prediction 범위는 LiDAR XY 중심 거리 <30m와 E25 공통 box의 교집합으로
  33절과 같다. 동일 객체의 반복 관측 수이며 아래 GT 수가 독립 자산 수는 아니다.
- raw XYZI와 JPEG bytes는 그대로 저장했다. ROS 강도 0~255를 E25에는 그대로,
  Base5 E9/E24에는 **입력 tensor만 고정 ÷255**로 전달한다. 프레임 max 정규화가
  아니며 원본·XYZ·GT는 불변이다. 실제 production bridge의 adapter는 아직 적용하지 않았다.

| bag | 내용 | 적격 프레임 → 선택 | 공통 30m 대상 GT |
|---|---|---:|---:|
| O02_164012 | CargoBox·드럼·배럴·콘크리트/PE 바리케이드 등 6자산 회피 | 463 → 64 | Obstacle 122 |
| P01_163434 | Man1 횡단 | 269 → 64 | Pedestrian 27 |
| H03_162301 | 고주로 혼합 차량 추월 | 512 → 64 | Vehicle 15 |
| R01_163034 | 로터리 차량 주행 | 326 → 64 | Vehicle 65 |

raw bag 경로·source contract SHA·각 이미지/점 payload SHA와 제외 원인은 산출물
`inputs/manifest.json`에 보존했다. 기존에 평가한 개발 bag이며 untouched test라고
부르지 않는다. 시간 축 subsampling이므로 연속 누락 지속 시간과 tracker 안정성은 평가하지 않았다.

### 실제 추론과 공통 후처리

세 모델 모두 로컬 5060 Ti에서 strict state load·FP32로 실행했다. AMP·TF32·camera
compile·camera/LiDAR 병렬화는 OFF다. 각 모델의 원래 입력 해상도·공간 격자·depth를
유지하며 Base5를 30m 구조로 바꾸지 않았다. 각 256장 export 완료를 확인했다.

비교 중 원격 SSH가 direct/proxy 모두 Connection refused로 바뀌었다. 중단 원인을
추정하거나 인스턴스를 시작하지 않고 Drive 백업에서 E9/E24를 로컬로 복원했다.
조각별 SHA와 full SHA가 모두 일치했다. 두 복원본은 후속 실험에 재사용할 수 있다.

- E9 full SHA256: `067205c3f5f5b0abc1bb775116c2b8bca2c9a7f31a0727b959db0ab1ce70b271`
- E24 full SHA256: `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746`

**주 결과는 ROS와 같은 class-aware rotated BEV NMS IoU 0.5 이후**다. 원래 ROI에서
server score floor 0.1·top100을 적용한 뒤 NMS, 공통 30m 제한 순서다. AP는 floor
이상의 score 곡선과 0.5/1/2/4m 중심 매칭 평균이며 고정 class threshold의 검출 확률이
아니다. 고정 P/R만 현재 V/P/O 0.40/0.15/0.10과 중심 오차 <1m를 사용한다.
`comparison_final.json`에 raw decoder AP도 secondary로 보존했다. NMS가 없는
`comparison_preliminary.json`을 아래 주 결과와 섞지 않는다. 33절의 static-val raw
decoder AP와도 후처리가 다르므로 숫자 차이를 데이터 효과 하나로 해석하지 않는다.

### 대표 bag 결과

단위는 %, 각 bag의 주 대상 클래스 center AP다. 4행의 평균을 전체 클래스 mAP라고
부르지 않는다. H03의 GT 15개처럼 작은 분모의 1~2pp 차이를 의미 있게 단정하지 않는다.

| bag·대상 | E25 | Base5 best E9 | Base5 final E24 |
|---|---:|---:|---:|
| O02 장애물 | 65.86 | 76.61 | 79.03 |
| P01 보행자 | 55.04 | 81.70 | 81.91 |
| H03 고주로 차량 | 80.28 | 81.39 | 78.98 |
| R01 로터리 차량 | 92.28 | 93.86 | 92.47 |

| bag·현재 score | E25 P / R | Base5 E9 P / R | Base5 E24 P / R |
|---|---:|---:|---:|
| O02 장애물 0.10 | 95.74 / 73.77 | 88.24 / 86.07 | 90.68 / 87.70 |
| P01 보행자 0.15 | 100.00 / 70.37 | 96.43 / 100.00 | 96.43 / 100.00 |
| H03 차량 0.40 | 86.67 / 86.67 | 86.67 / 86.67 | 86.67 / 86.67 |
| R01 차량 0.40 | 96.97 / 98.46 | 96.97 / 98.46 | 96.97 / 98.46 |

이번 장애물 bag에서 E24는 E25보다 TP 90→107/122, FP 4→11이다. 검출 증가와
오검출 증가가 함께 있으므로 기존 score 0.1을 새 최적 threshold로 채택하지 않는다.
장애물 0~10m recall은 26/38→33/38, 20~30m는 24/43→38/43이다. 단, 10~20m는
40/41→36/41로 떨어졌다. 가까운 장애물이 전반적으로 망가졌다는 증거는 아니지만
모든 거리·자산에서 일방적으로 개선된 것도 아니다. 차량은 이 표본에서 비슷하고,
보행자 recall 100%도 27개 반복 관측 표본에 한정한다.

**결론:** Base5의 장애물·보행자 개선은 같은 생성기 val에서만 보이는 현상은 아니다.
다만 4개 dev bag·30m 빠른 표본 결과이며 모든 실전 조건, 100m 원거리, 새로운 자산의
일반화나 정밀 격자/depth의 개별 효과를 증명하지 않는다. E9/E24 모두 보존하고 새
threshold·연속 누락·학습된 출력 runtime gate 전에 production을 자동 교체하지 않는다.

### 산출물과 검증

ignored 루트는 `artifacts/evaluation/bevfusion/base5_rosbag_quick_261012`다.
`inputs`는 원본 센서 payload·GT·manifest, `E25/E9/E24`는 실제 prediction과 receipt,
`checkpoints`는 검증된 Drive 복원본, `comparison_final.json`은 NMS 주 결과다.
새 `prepare_base5_rosbag_quick.py`는 CPU bag 추출, `summarize_base5_rosbag_quick.py`는
공통 NMS·GT identity guard·case별 AP/P/R를 담당한다. 모델 추론은 기존
`compare_base5_common30.py infer`의 명시적 `--intensity-scale 0.00392156862745098`을
사용했다. 결과를 정리하기 위해 ROS나 MORAI를 켜지 않았다.

```bash
python3 perception/detection3d/bevfusion/tools/summarize_base5_rosbag_quick.py \
  --inputs artifacts/evaluation/bevfusion/base5_rosbag_quick_261012/inputs \
  --predictions \
  artifacts/evaluation/bevfusion/base5_rosbag_quick_261012/E25/predictions.json.gz \
  artifacts/evaluation/bevfusion/base5_rosbag_quick_261012/E9/predictions.json.gz \
  artifacts/evaluation/bevfusion/base5_rosbag_quick_261012/E24/predictions.json.gz \
  --output artifacts/evaluation/bevfusion/base5_rosbag_quick_261012/comparison_repeat.json
```

CPU 단위 검사 16개가 통과했다. 강도 ÷255의 원본/XYZ 불변·역변환·범위 거부,
organized point cloud row padding, 시간 오차·높이 외삽 거부, world→LiDAR 중심 변환,
희소 GT 지원점, NMS 중복 제거·클래스 분리 등을 검사했다. 다른 agent의
planning_validator 변경은 커밋에 포함하지 않는다. 다음은 실제 ROS 입력 동등성과
연속 전체 bag의 threshold·누락 분석, 학습된 출력의 최적화/24GB 검증이다.

<a id="base5-ros-input-continuity-261012"></a>

## 35. Base5 ROS 입력 정합과 연속 bag 평가 — 2026-10-12 05:50 KST

사용자 승인 범위는 Base5 ROS 입력 동등성 및 대표 bag의 연속 threshold·누락 검사다.
실제 학습 완료 E9/E24와 production E25를 로컬 5060 Ti에서 비교했다. 원격 학습,
MORAI·ROS master·차량 제어·production detector는 실행하거나 교체하지 않았다.
다른 agent의 planning_validator 변경도 이 작업 및 커밋에서 제외한다.

### ROS 입력 계약 보완과 실제 TCP 검사

E25 기본 `256×704 / resize 0.55 / host RGB8 / native intensity`는 유지한다.
새 `base5_trial.launch`와 `configs/perception/bevfusion_base5_trial.yaml`만
`Sensor_4 / 512×1056 / resize 0.825 / compressed / ros255`를 선택한다.
서버는 별도 port 29515, 출력은 `/perception/base5_trial/detection/objects`다.
status·latency·marker·projection도 trial topic으로 remap해 production 관측을 덮지 않는다.
원본 ROS XYZI 강도는 0~255 그대로 전달하고 Base5 서버가 모델 입력 복사본에만
고정 ÷255를 한 번 적용한다. XYZ와 원본 파일은 변하지 않는다. native Base5 파일을
직접 시험할 때는 서버의 기본 `native`를 사용하며 다시 나누지 않는다.
요청에 명시된 intensity source·최종 이미지 크기가 서버와 다르면 추론 전에 거부한다.

처음에는 host가 Base5 크기로 RGB를 만들어 전달했으나 오프라인과 이미지 성분
2.67%가 최대 1(uint8) 달랐다. host OpenCV 4.2.0과 Docker 4.11.0의 환경 차이가
있으며, decoder/resize 중 정확한 발생 지점까지 분리한 결과는 아니다. 큰 성능
저하의 증거로 단정하지 않지만 exact input gate는 실패했다. **Base5는 원본 JPEG
bytes를 재압축 없이 전달하고 오프라인과 동일한 Docker에서 decode·resize·crop**하도록
수정했다. production E25의 host RGB 경로는 바꾸지 않는다. PointCloud2의 organized
row padding 파서도 보완했고 기존 비패딩·big-endian 입력을 검사했다.

4개 bag에서 실제 ROS 메시지 3장씩, 총 12장을 골라 다음 경로를 대조했다.

`실제 ROS 메시지 → 실제 bridge _prepare_frame → binary packet → private localhost TCP → 실제 server handler/model`

비교 기준은 같은 E24·동결 val pipeline이다. 이미지 픽셀과 모델 입력 포인트는 12장
모두 정확히 일치했고 투영 행렬은 atol 1e-5를 통과했다. 서버 반환 검출은 총 31개로
전체 빈 출력만 비교하는 검사가 아니다. label·개수는 동일하고 7D 박스 최대 절대 차이
6.074e-5, score 최대 차이 5.246e-6으로 각각 atol 1e-4/1e-5를 통과했다. 박스 수치는
위치·크기·yaw를 포함하므로 모든 차이를 미터 단위로 읽지 않는다. 12장 중 1장은 양쪽
출력 모두 0개였다. AMP·TF32·추론 compile·병렬 stream은 OFF인 기준 검사다.
동결 학습 설정의 compile ON을 추론에서 자동 활성화하지 않도록 보완했다.

이는 **실제 메시지 파싱·전처리·TCP·모델의 동등성 검사**다. live subscription,
latest-only queue, ROS 후처리·tracker, LAN, 13Hz 또는 실제 24GB 배포 인증은 아니다.
ROS package만 증분 설치하고 launch 해석을 확인했으며 노드를 실행하지 않았다.

### 연속 비교 범위와 GT

34절의 같은 4개 dev bag을 사용하되 균등 256장 표본 대신 모든 적격 프레임을 추론했다.
원본 camera JPEG와 LiDAR XYZI는 그대로이며 Base5 모델 입력만 ÷255다. 각 모델의
native 구조·해상도·결정론적 전처리를 유지하고 strict checkpoint loading을 사용했다.
모델 export의 공통 GT SHA256은 아래와 같다.

`89b4e420450bab2a6056bc3917aa7765dbb275a2a88c10f5e45e6c2ded44e33a`

| bag | 적격 연속 프레임 | 공통 30m 주 대상 GT 관측 |
|---|---:|---:|
| O02_164012 장애물 | 463 | 897 |
| P01_163434 보행자 | 269 | 113 |
| H03_162301 고주로 추월 | 512 | 122 |
| R01_163034 로터리 | 326 | 336 |
| 합계 | 1,570 | 1,468 |

GT 수는 반복 프레임 관측이며 독립 객체 수가 아니다. Object/Ego status의 보간과
기존 asset catalog·BBox offset을 사용한 근사 GT이지 SaveSensorData 동기 annotation은
아니다. camera header 시간차 ≤50ms, status 보간 bracket ≤150ms, 높이 외삽 거부와
기존 unresolved-frame 제외를 유지한다. Ego/Object heading 보정 −1.347°도 동일하다.
공통 범위는 XY 방사 거리 <30m와 E25 박스 `[-30,-30,-3,30,30,2]`의 교집합이며
박스 Z는 bottom-center다. 약하거나 가려진 지원 GT를 점수 때문에 제거하지 않는다.

### AP 결과

server floor 0.1·top100 이후 ROS와 같은 class-aware rotated BEV NMS IoU 0.5,
공통 30m 제한을 적용한다. 아래 AP는 0.5/1/2/4m center AP 평균(%)이며 각 bag의
주 대상 클래스다. 4행의 평균을 전체 mAP나 독립 검출 확률이라고 부르지 않는다.

| bag·대상 | E25 | Base5 best E9 | Base5 final E24 |
|---|---:|---:|---:|
| O02 장애물 | 65.65 | 79.24 | 81.41 |
| P01 보행자 | 59.81 | 83.24 | 80.92 |
| H03 고주로 차량 | 76.67 | 76.87 | 76.34 |
| R01 로터리 차량 | 90.51 | 93.21 | 91.16 |

장애물·보행자 개선은 연속 raw dev bag에서도 유지됐다. E24는 장애물, E9는 보행자·
로터리에서 우세하므로 둘 다 보존한다. 차량은 이 범위에서 크게 달라지지 않았다.
학습 격자/depth 한 변수만의 효과나 모든 자산의 개선을 증명하는 비교는 아니다.

### 고정 threshold와 연속 매칭 실패

0.10/0.15/0.20/0.25/0.30/0.40/0.50/0.60/0.70/0.80을 모든 bag·클래스에 검사했다.
주 매칭은 중심 오차 <1m다. 실제 검출 부재와 위치 오차를 구분하려고 <2m도 별도
계산했다. 누락 구간은 동일 GT track key 안에서만 이어 붙이며 GT/ROI 제외,
**원본 LiDAR frame index의 끊김**, sensor 시간 간격 >0.25s에서 분리한다. H03에는
원본 index break가 2개 있었다. 구간 시간은 `마지막−첫 sensor stamp + 중앙 주기`
추정이다. 정확한 비가시 시간이나 tracker 삭제 시간은 아니다. GT track key 일부는
재구성된 trajectory association으로 simulator unique_id의 독립 인증도 아니다.

| bag·현재 score | E25 P / R (%) | E9 P / R (%) | E24 P / R (%) |
|---|---:|---:|---:|
| 장애물 0.10 | 98.07 / 73.58 | 88.51 / 87.63 | 90.88 / 89.97 |
| 보행자 0.15 | 100.00 / 73.45 | 96.58 / 100.00 | 98.23 / 98.23 |
| 고주로 차량 0.40 | 82.79 / 82.79 | 82.93 / 83.61 | 82.93 / 83.61 |
| 로터리 차량 0.40 | 97.92 / 98.21 | 97.65 / 98.81 | 97.35 / 98.51 |

| bag·현재 score | E25 최대 1m 매칭 실패 (s) | E9 | E24 |
|---|---:|---:|---:|
| 장애물 0.10 | 2.085 | 1.260 | 0.708 |
| 보행자 0.15 | 1.487 | 0.000 | 0.087 |
| 고주로 차량 0.40 | 1.045 | 1.045 | 1.045 |
| 로터리 차량 0.40 | 0.250 | 0.167 | 0.250 |

고주로의 1.045s를 ‘객체 출력이 1초 사라졌다’고 읽으면 안 된다. **2m 기준 차량
recall은 E25 98.36%, E9/E24 99.18%, 최대 실패는 모두 0.083s**다. 1~2m 중심 오차를
검출 부재와 구분해야 한다. GT 보간·시각 정합·검출 위치 중 무엇이 주원인인지는
이 결과만으로 확정하지 않으며 다음 실제 위치 오차 조사에서 확인한다.

| E24 장애물 score | TP / FP | Precision / Recall (%) | 최대 1m 실패 (s) |
|---|---:|---:|---:|
| 0.10 | 807 / 81 | 90.88 / 89.97 | 0.708 |
| 0.15 | 771 / 42 | 94.83 / 85.95 | 0.802 |
| 0.20 | 726 / 21 | 97.19 / 80.94 | 0.973 |
| 0.40 | 539 / 4 | 99.26 / 60.09 | 1.406 |

후속 tracker 시험의 장애물 후보는 0.15다. 0.10 대비 주 대상 FP는 48% 줄지만 TP도
36개 줄어 안전 trade-off가 있다. 보행자 0.15·차량 0.40은 우선 유지할 후보이며
**production 최적 threshold를 인증하거나 기본값을 승격한 것은 아니다.** 별도 trial도
비교 재현을 위해 기존 V/P/O 0.40/0.15/0.10으로 시작한다. 표는 주 대상 클래스 FP이며
다른 클래스의 background FP도 전체 JSON에 별도 남겼다. ≥5 LiDAR points 조건은
추가 진단으로만 계산하며 전체 GT matching이나 학습 라벨을 대체하지 않는다.

### 실행·산출물·검증 및 다음 순서

ignored 루트는 `artifacts/evaluation/bevfusion/base5_rosbag_continuous_261012`다.
`inputs/manifest.json`은 원본 경로·시간·catalog·payload hash,
`E25/E9/E24/predictions.json.gz`와 receipt는 실제 추론 결과,
`comparison_final.json`은 AP, `continuity_final.json`은 threshold·연속 실패 결과다.
`parity_requests_compressed_v3`는 실제 bridge packet,
`parity_E24_compressed.json`은 학습된 출력까지의 TCP gate다.
실패한 host RGB gate도 `parity_console.log`와 당시 requests로 보존한다.

새 도구는 `prepare_base5_ros_parity_requests.py`, `check_base5_ros_input_parity.py`,
`analyze_base5_bag_continuity.py`다. 기존 bag 준비기는 이후 생성 시 원본 LiDAR index를
저장하도록 보완했다. 이번 infos에는 없는 필드여서 analyzer가 raw bag header에서
복원했으며 기존 센서·GT를 재작성하지 않았다.

```bash
# ROS Noetic + 기존 AW149 overlay의 host Python에서 실행한다.
python3 perception/detection3d/bevfusion/tools/analyze_base5_bag_continuity.py \
  --inputs artifacts/evaluation/bevfusion/base5_rosbag_continuous_261012/inputs \
  --predictions \
  artifacts/evaluation/bevfusion/base5_rosbag_continuous_261012/E25/predictions.json.gz \
  artifacts/evaluation/bevfusion/base5_rosbag_continuous_261012/E9/predictions.json.gz \
  artifacts/evaluation/bevfusion/base5_rosbag_continuous_261012/E24/predictions.json.gz \
  --output artifacts/evaluation/bevfusion/base5_rosbag_continuous_261012/continuity_repeat.json
```

단위/회귀 검사 **47개 통과·GPU optional compile 검사 2개 skip**다. Docker 전처리·
runtime·최적화 19개, ROS 입력 3개, 기존 threshold/NMS 7개, 공통30m 11개,
bag/연속 구간 7개다. 추가로 실제 입력/학습 출력 12장 TCP gate, ROS package 증분
설치·launch 해석, py_compile·diff whitespace 검사를 완료했다. skip을 통과로 세지 않는다.

**남은 순서:** 학습된 출력이 있는 모델로 compile/병렬화 정합과 4090·실제 24GB 비용
확인 → live ROS queue·LAN·tracker의 threshold/누락 시험 → 100m·미사용 bag·다른
환경 확인이다. 고주로 중심 오차 1~2m의 원인 분해도 필요하다. 이번 검사는 원거리,
foggy·동물·복합체와 실제 full-stack 안전성을 인증하지 않으며 production E25는 유지한다.

<a id="base5-learned-runtime-vast-261012"></a>

## 36. 학습된 E9/E24의 4090 추론 최적화·출력·메모리 검증 — 2026-10-12

### 실행 범위와 재현 계약

사용자가 Vast를 켠 뒤 인스턴스 `55272391`의 SSH·4090 48GB·유휴 GPU를 확인했다.
학습·MORAI·production E25·threshold·frozen 훈련 소스·원본 센서는 변경하지 않았다.
인스턴스 Stop/Destroy도 실행하지 않는다. 33절의 E9 best와 E24 마지막 체크포인트를
strict load하며 그 절의 SHA256을 유지한다. 빈 출력의 COST fixture가 아니라 실제 학습된
검출 출력을 비교한다.

- 공간 격자 LiDAR XY 5cm·Z 20cm, camera BEV 20cm, depth `[1,100,0.25]`,
  입력 `512×1056` Q90 JPEG와 현재 3클래스를 유지한다. 내부 padding은 X±100.8/Y±33.6m,
  의미 ROI는 `[-100,-30,-3,100,30,2]`다. 학습 compile은 추론 설정으로 자동 상속하지 않는다.
- `benchmark_base5_inference_runtime.py --preserve-config`는 학습된 공간/depth 설정을
그대로 사용한다. 기존 COST 후보 생성 경로와 명시적으로 구분한다.
- private localhost TCP·batch 1이며 decoder floor 0.1/top100이다. 네 카메라 OpenCV 병렬
  전처리, Torch CPU 4 threads·OpenCV 1 thread를 사용한다. 이미지와 점 payload를 RAM에
  읽은 뒤 측정하므로 disk loader·ROS 메시지 대기·LAN·bridge NMS·tracker 비용은 포함하지 않는다.
- AMP/TF32 runtime과 별도 FP32/TF32 OFF 기준을 비교한다. stage instrumentation은
  실제 RTT 측정 뒤 별도 수행하며 stream span을 단순 합산하지 않는다.
- 48GB GPU에서 PyTorch allocator를 **20GiB로 제한**한다. 실제 24GB 카드나 신호등·
  전체 스택 동시 실행을 인증하는 검사는 아니다.
- 모든 variant는 동일한 순서·입력·가중치다. 처음/마지막 기본 모드를 반복하며 class별
  one-to-one 출력 정합의 고정 허용치는 중심/크기 5cm, yaw 0.02rad, score 0.002다.
  출력 개수 차이도 실패로 남기며 AP가 비슷하다고 이 gate를 완화하지 않는다.

원격 실행 소스는 `/workspace/base5_learned_runtime_source_8e2f86f5_261012`의 별도 snapshot이다.
기존 built CUDA ops를 재사용하며 의존성·CUDA를 다시 설치하지 않았다. frozen work의
`resolved_train.py`와 가중치는 읽기 전용이다. 745장 실행의 `_next.py`는 bounded iteration을
1000으로 확장한 benchmark 사본이며 각 report의 source SHA256이 실제 실행 파일을 식별한다.

### 300장 비용 비교와 전체 validation의 구분

기존 대표/스트레스 300장은 **train 261 / val 39**다. 비용·출력 차이 진단에만 사용하고
그 AP를 validation 점수나 일반화 성능으로 주장하지 않는다. E24는 672개 출력/273개 nonempty,
E9는 687개 출력/277개 nonempty여서 빈 출력의 형식적 통과가 아니다.

단위는 ms이며 각 행은 300개 서로 다른 입력의 warmed localhost RTT다.

| 모델·실행 | 평균 RTT | p95 RTT | 고정 출력 gate 통과 |
|---|---:|---:|---:|
| E24 기본 | 78.45 | 79.99 | 비교 기준 |
| E24 병렬화 | 72.16 | 73.56 | 299/300 |
| E24 camera+neck compile | 72.43 | 74.58 | 292/300 |
| E24 compile+병렬화 | 66.87 | 68.61 | 292/300 |
| E24 기본 재실행 | 78.84 | 80.51 | 300/300 |
| E9 기본 | 79.16 | 80.85 | 비교 기준 |
| E9 병렬화 | 72.59 | 74.54 | 300/300 |
| E9 camera+neck compile | 72.94 | 74.37 | 293/300 |
| E9 compile+병렬화 | 66.47 | 68.16 | 293/300 |
| E9 기본 재실행 | 79.25 | 81.34 | 300/300 |

E24의 전처리/모델/반환은 기본 약 **18.50/58.65/0.19ms**, compile+병렬화는
**18.83/46.62/0.19ms**다. 이번 추가 단축은 전처리를 생략하거나 검출 결과를 안 돌려준
효과가 아니라 모델 실행 경로의 단축이다. RTT에는 별도 protocol/호스트 비용이 더 있다.

후속은 기존 validation **745장 전체만** 사용한다. train·stress 재샘플링이 없으며
val infos SHA256은 `24a6d6527c046bbee65324867fe38327b9bb40726668abc7f351d1543f7bacaf`다.
`prepare_base5_runtime_validation_selection.py`가 unique token·upright 정책·정확히 745장을
검사한다. `summarize_base5_runtime_accuracy.py`는 같은 native GT를 bottom-center로 변환하고
동일 ROS NMS IoU 0.5를 적용한다. 기존 val이지 독립 test set은 아니다.

### 후처리가 다른 AP를 섞지 않는다

전체 val 745장의 완료 결과는 다음과 같다. AMP 각 실행은 E24 약 1,625개/E9 약 1,817개
출력을 반환했다. 기본 재실행은 처음 기본 모드와 비교한 결과이며 비교 기준 행의 0개
정합 검사를 통과 증거로 세지 않는다.

| 모델·실행 | 평균 RTT (ms) | p95 RTT (ms) | 고정 출력 gate 통과 |
|---|---:|---:|---:|
| E24 FP32 기준 | 110.24 | 111.73 | 별도 precision 기준 |
| E24 AMP 기본 | 78.93 | 80.01 | 비교 기준 |
| E24 AMP 병렬화 | 72.22 | 73.30 | 743/745 |
| E24 AMP compile+병렬화 | 66.09 | 67.29 | 715/745 |
| E24 AMP 기본 재실행 | 78.89 | 80.73 | 742/745 |
| E9 FP32 기준 | 109.54 | 110.82 | 별도 precision 기준 |
| E9 AMP 기본 | 78.49 | 79.35 | 비교 기준 |
| E9 AMP 병렬화 | 72.42 | 74.28 | 742/745 |
| E9 AMP compile+병렬화 | 65.86 | 66.99 | 723/745 |
| E9 AMP 기본 재실행 | 78.90 | 79.86 | 741/745 |

기본 AMP 대비 병렬화는 약 **7.7~8.5%**, compile+병렬화는 약 **16.1~16.3%** 단축이다.
compile 결합의 모델 부분은 E24 46.08/E9 45.97ms다. 원격 localhost warmed 비용이므로
13Hz live ROS/LAN·전체 스택의 지연 상한을 인증하지 않는다.

아래는 전체 의미 ROI·동일 NMS의 center AP(%)다. 300장 mixed selection AP와 섞지 않는다.

| 모델·실행 | Vehicle | Pedestrian | Obstacle | mAP |
|---|---:|---:|---:|---:|
| E24 FP32 기준 | 85.463 | 80.214 | 65.606 | 77.094 |
| E24 AMP 기본 | 85.457 | 80.993 | 65.649 | 77.367 |
| E24 AMP 병렬화 | 85.457 | 80.993 | 65.648 | 77.366 |
| E24 AMP compile+병렬화 | 85.721 | 80.992 | 65.651 | 77.455 |
| E9 FP32 기준 | 84.836 | 85.740 | 67.094 | 79.223 |
| E9 AMP 기본 | 85.348 | 85.748 | 67.102 | 79.399 |
| E9 AMP 병렬화 | 85.348 | 85.748 | 67.105 | 79.400 |
| E9 AMP compile+병렬화 | 85.349 | 85.747 | 67.103 | 79.400 |

AMP 기본은 FP32보다 E24 +0.272pp/E9 +0.176pp이며 성능 하락은 관측되지 않았다.
compile 추가의 mAP 변화는 E24 +0.088pp/E9 약 +0.001pp다. 이 미세 차이를 학습 개선이나
일반화 이득으로 주장하지 않는다. AP는 중심 매칭이므로 방향각 안정성 gate를 대체하지 못한다.

기본 재실행도 E24 3장/E9 4장에서 고정 수치 gate를 벗어났다. 병렬화의 E24 2장/E9 3장
실패를 전부 병렬화의 결함으로 단정하지 않는다. 모든 출력의 중심 차이는 병렬화에서 약
2mm 이내, compile 결합에서 약 8mm 이내였다. 그러나 방향각 최대 변화는 각각
E24 약 1.90°/6.83°, E9 약 3.86°/5.37°였고 score 허용치를 벗어난 사례도 있다.
개수 불일치 장면은 geometry 최대치 계산에 포함할 수 없어 별도 실패로 남긴다.
compile 결합은 두 모델 모두 한 장에서 출력이 하나 추가됐다. E24 `b5v2_02650`의 추가
Vehicle score는 **0.100046**으로 decoder floor 경계다. 현재 Vehicle 0.40이면 전달되지
않지만, 이 사실로 모든 추가/누락이나 미래 threshold를 안전하다고 일반화하지 않는다.

33절의 static-val AP는 raw decoder 결과다. 이번 표의 AP는 ROS와 같은 floor 0.1/top100·
NMS 뒤 결과다. 이전 E24 FP32 export에 동일 후처리를 적용하면 radial 30m AP는
V/P/O **95.214/89.481/76.394%**, mAP **87.030%**로 이번 FP32 TCP 결과와 일치한다.
33절의 raw 97.25/94.87/83.49%에서 낮아진 차이를 모델 손상이나 최적화의 영향으로
해석하지 않는다. 낮은 score 삭제와 중복 억제 조건이 달라진 것이다.

구간 AP도 같은 후처리로 계산하며 GT 수를 함께 기록한다. 빈 클래스의 관례상 AP 0을
‘실제 원거리 성능 0’이라고 읽지 않는다. 이 진단 JSON은 W&B의 세 클래스 GT가 모두
있을 때만 정의하는 구간 mAP 패널을 수정하거나 기존 history에 덮어쓰지 않는다.

### 메모리·산출물과 남은 검사

300장 AMP 추론의 steady allocated peak는 약 **1.53~1.61GiB**, 초기 로드/준비 peak는
약 **17.07~17.15GiB**다. FP32 745장 기준 초기 peak는 약 **18.90GiB**, steady peak는
약 **1.95GiB**다. report의 `steady_reserved_mib`는 stage probe 이후 allocator reservation이므로
계측 전의 실제 steady allocated peak와 혼동하지 않는다. 20GiB cap 성공은 24GB 배포의
유력한 근거지만 실제 카드·다른 GPU 노드와의 동시 사용 검사는 여전히 남는다.

ignored 산출물 루트는 `artifacts/evaluation/bevfusion/base5_learned_runtime_261012`이며
같은 이름의 원격 `/workspace` 폴더를 유지한다. 각 실행의 `report.json`, variant별
`*_predictions.json`, `accuracy.json`, resolved runtime config와 validation selection을 보존한다.
출력·가중치·config·최적화 source의 SHA256은 report에 기록된다. 작은 결과 파일만 로컬로
회수하며 센서 데이터나 가중치를 다시 전송하지 않는다.
회수된 JSON/config 43개는 원격·로컬 SHA256이 모두 일치했다.

최초 CPU accuracy 실행은 snapshot에 ROS NMS package가 빠져 import 실패했다.
해당 package만 별도 snapshot에 추가한 뒤 재실행했다. GPU benchmark나 frozen training을
재시작·변경한 문제가 아니며 실패 로그도 보존한다.

### 같은 모델의 수치 통제와 적용 결론

E24 val의 균등 선택 32장·기준 출력 79개를 사용한 별도 진단을 완료했다. LiDAR의 카메라
깊이 raster를 각 프레임의 기준값으로 고정한 **진단 전용** 통제이며 실제 추론에서 프레임 간
depth·query를 캐시하지 않는다. 같은 모델을 유지하여 가중치 재로드 차이도 제거한다.

| 고정 raster 진단 | 고정 출력 gate 실패 / 32 |
|---|---:|
| 기본 재실행 | 0 |
| 병렬화 | 0 |
| camera+neck compile | 2 |
| compile+병렬화 | 2 |
| dense Conv/BN folding | 3 |
| compile + 기준 query 선택 고정 | 1 |

병렬화의 camera feature·dense heatmap은 이 통제에서 정확히 같았다. compile의 최대
camera feature relative L2 차이는 약 **0.00099**였지만 dense heatmap 최대 절대 차이는
약 **0.0312**였고, 실패 두 장에서 query 200개 중 하나가 바뀌었다. query를 기준으로
고정해도 한 장은 남아 query 선택 변화만으로 모든 수치 차이를 설명할 수 없다.
첫 입력 8회 반복에서는 raster 자체의 변화가 없었다. 따라서 이번 32장 결과만으로
전체 val 기본 재실행의 흔들림 원인을 모두 raster race라고 확정하지 않는다.

추가로 기존 실패 장면을 의도적으로 고른 **8장·기준 출력 31개**를 조사했다. 이 집합은
실패 원인 진단이지 일반적인 실패율/AP 추정용이 아니다. 첫 입력 `b5v2_00180`의 8회
반복 중 5회에서 depth **1픽셀·약 1.679m** 차이와 출력 gate 실패를 함께 관측했다.
기존 DepthLSS의 여러 점이 같은 픽셀에 쓸 수 있는 인덱스 assignment와 일치하는 현상이다.
그 raster를 고정하면 같은 모델의 기본/병렬화는 8/8 통과했지만 compile/compile+병렬화는
각 7/8 실패했다. 기준 query 통제에서도 2/8은 남았다. 따라서 **기본 모드의 일부 입력
raster 흔들림과 compiler 산술/query 선택 차이는 별개**다. 다른 모든 반복 실패의 원인을
이 한 장으로 확정하지 않으며 deterministic pixel 선택 정책을 production에 적용하지 않았다.
후속 raster 정책 보완은 같은 val에서 AP·출력·비용을 다시 확인한 뒤 판단한다.

**결론:** 학습된 출력이 있는 모델에서도 약 66ms의 최적화 후보를 확인했고 전체 val AP는
거의 유지된다. 병렬화는 우선 실전 시험 후보, compile은 추가적인 선택지로 유지한다.
고정 수치 gate를 전부 통과하지 않았으므로 추론 최적화의 production 기본값은 변경하지 않는다.
이는 ‘재학습해야 한다’나 ‘모델이 망가졌다’는 뜻이 아니며 같은 가중치로 OFF/ON 비교가 가능하다.
folding도 이 진단만으로 채택하지 않는다. 실제 24GB·ROS/LAN·tracker에서 관측된 변화가
판단에 영향을 주는지 확인한 뒤 적용 정책을 정한다.

검사는 Docker CPU 회귀 **38개 통과·CUDA optional 4개 skip**, 추가 host CPU 10개 통과,
실제 4090의 learned 출력/전체 val/수치 통제, py_compile·diff whitespace다. 같은 CPU
테스트를 실행 환경별로 반복한 수를 독립 테스트 개수로 더하지 않는다. source 변경은
비용 도구·선택기·정확도 진단기·실험용 수치 통제에 한정하며 production 모델은 바꾸지 않았다.

**다음:** 실제 4090 24GB에서 Base5 trial의 ROS queue·LAN·최신 프레임 처리·tracker와
threshold 후보를 확인한다. 100m bag GT·새 환경/자산별 잔여 누락 확인도 남는다.
33절 raw AP, 이번 ROS 후처리 AP, 35절 dev bag AP는 서로 다른 평가 조건이다.
이번 검사를 live full-stack·30K 학습·실대회 검출률 인증으로 확대 해석하지 않는다.
검사 종료 뒤 원격 benchmark/진단/학습 프로세스가 없고 GPU 0%·1MiB를 확인했다.
과금되는 인스턴스 자체는 사용자 결정 없이 중단하지 않았다.

<a id="base5-compile-delivery-tracking-review-261012"></a>

## 37. 컴파일 오차의 전달·연속 검출·실제 EKF 코어 영향 — 2026-10-12

### 요청과 실행 범위

사용자가 추가로 컴파일의 yaw·score 차이를 검토하도록 요청했다. 고정 수치 gate를
완화하지 않고 실제 전달·BBox 경계·tracker의 비선형 조건을 별도 조사했다. E25 기본값,
훈련·가중치·원본·MORAI·다른 agent의 planning_validator는 변경하지 않았다.
실제 24GB 카드는 없으므로 원격 48GB/20GiB cap과 로컬 5060 Ti 검사를 구분한다.

원격 source는 `/workspace/base5_compile_review_source_b645211c_261012`, ignored 결과는
`artifacts/evaluation/bevfusion/base5_compile_review_261012`다. 기존 CUDA 의존성을 재사용한다.
로컬 연속 추론 결과는 `base5_compile_bag_261012`이며 모든 가중치는 33절 E24 SHA를 유지한다.

### 전달·경계와 분기 조건

`analyze_base5_runtime_effects.py`는 실제 floor 0.1/top100·class-aware NMS 0.5 뒤
V/P/O 0.40/0.15/0.10과 별도 O0.15 후보를 검사한다. 클래스 보존 XY Hungarian
<0.5m 매칭과 물리적인 BBox 모서리 이동을 기록한다. π 회전의 같은 사각형을
물리적 경계 이동으로 잘못 세지 않는다. 이 도구는 시계열 tracker 시험을 대체하지 않는다.

36절의 E9/E24 전체 745장 export에서는 현재 threshold 뒤 전달 집합 변화·score0.5
분기 전환이 모두 0이었다. compile 결합의 최대 모서리 이동은 E24 V/P/O
**3.36/4.23/6.84cm**, E9 **2.25/3.07/3.22cm**였다. 하지만 새로운 E24 반복에서는
`b5v2_03402`의 Obstacle score가 **0.500058→0.499552**로 바뀌었다.
실제 EKF `UpdateTrack`은 score<0.5에서 측정 R을 10배로 바꾼다. 따라서 작은 score
차이를 항상 무해하다고 일반화할 수 없다. tracker R·association은 이 작업에서 수정하지 않는다.

### 더 보수적인 컴파일 후보의 비용과 한계

같은 E24·val 745장·20GiB cap·현재 geometry로 새로 측정했다. ‘연산 유지’는
LayerNorm/GELU를 eager로 남기는 opt-in이고 ‘레이아웃 유지’는 compiler layout 변경을
제한한다. 시간은 전처리 포함 warmed localhost RTT다.

| 실행 | 평균 / p95 (ms) | 고정 출력 gate 실패 / 745 |
|---|---:|---:|
| AMP 기본 | 78.26 / 79.45 | 비교 기준 |
| 병렬화 | 72.28 / 73.09 | 1 |
| 일반 compile+병렬화 | 66.10 / 67.27 | 29 |
| 연산 유지 compile+병렬화 | 71.57 / 72.96 | 25 |
| 레이아웃 유지 compile+병렬화 | 65.94 / 66.88 | 28 |
| 연산·레이아웃 유지 compile+병렬화 | 71.22 / 73.07 | 22 |
| AMP 기본 재실행 | 79.23 / 80.46 | 1 |

전체 mAP는 기본 77.364%, 후보 77.364~77.452%로 거의 같았다. 하지만 마지막 후보도
완전한 수치 동일성을 확보하지 못했고 `b5v2_04089`에서 NMS 뒤 Obstacle 하나가 추가됐다.
더 느리면서 안정성이 확정되지 않아 이 후보들을 기본값으로 채택하지 않는다.

### 실제 연속 bag 1,570프레임의 검출 비교

35절과 **같은 전체 적격 프레임·GT·원본 JPEG·raw XYZI**를 사용했다. 로컬 5060 Ti에서
AMP 기본/병렬화/일반 compile 결합/Camera CUDA Graph 결합을 각각 실제 추론했다.
입력 tensor의 강도만 고정 ÷255로 변환한다. 원본·GT·geometry를 바꾸지 않는다.
로컬은 TF32 OFF·cuDNN benchmark OFF이며 원격 비용 preset과 같지 않다. 로컬 시간은
4090 속도 근거로 사용하지 않는다. 각 export의 GT identity는
`89b4e420450bab2a6056bc3917aa7765dbb275a2a88c10f5e45e6c2ded44e33a`로 일치한다.

`compare_base5_common30.py infer`에 명시적 Base5-only opt-in을 추가했다. 기본 FP32
비교와 E25 경로는 유지한다. `inference_optimization`을 receipt에 남기며 CUDA Graph
probe와 camera compile 동시 요청은 거부한다. stage 시간과 처음 JIT 준비 비용을 섞어
배포 latency라고 부르지 않는다.

아래 AP(%)는 같은 radial30·ROS 후처리·각 bag 주 대상 클래스다.

| bag·대상 | AMP 기본 | 병렬화 | compile+병렬화 | Graph+병렬 prototype |
|---|---:|---:|---:|---:|
| O02 장애물 | 81.41 | 81.41 | 81.42 | 81.41 |
| P01 사람 | 80.89 | 80.92 | 80.92 | 80.89 |
| H03 차량 | 76.33 | 76.34 | 76.35 | 76.32 |
| R01 차량 | 91.15 | 91.15 | 91.15 | 91.15 |

현재 전달 threshold 뒤에는 O02 일반 compile의 1프레임에서 Obstacle 관측이 추가됐다.
`O02_164012_00197`, score **0.100328**, LiDAR 중심 X 약 **56.63m**다. 이 bag의 검증 GT는
30m 계약이므로 이 관측을 곧바로 FP나 TP로 판정하지 않는다. 다른 세 bag의 전달 집합
변화는 없었다. **비슷한 AP가 동일한 연속 처리나 안전성을 뜻하지 않는다.**

### 실제 EKF 코어 재생과 장애물의 예외

새 `replay_base5_tracker_core.cpp`는 production의 **기존 실제 C++ EKF 소스를 그대로 링크**한다.
CTRV·XY noise0.40m·yaw noise10°·association5m·OD velocity fusion OFF로 실행한다.
`replay_base5_compiled_tracking.py`가 같은 기록 Ego·heading−1.347°·Sensor_4 extrinsic으로
두 경로의 detection을 동일 map 좌표로 옮기고 원래 센서 stamp 순서로 재생한다.
각 bag에서 코어를 새로 만들며 같은 입력의 독립 반복 control은 출력 차이 0이었다.

이는 **오프라인 EKF 코어**다. ROS TF·node-side orientation canonicalization·30Hz 발행
스케줄러·UNKNOWN stationary 후처리·Map-CV·planning·LAN은 실행하지 않는다.
GT 속도 MAE/AVE나 실제 ID switch 인증도 아니다. 서로 다른 실행의 숫자 ID가 일정하게
바뀌는 것은 ID switch로 세지 않고, 대응 관계가 바뀐 관측을 별도 진단으로 기록한다.

35절과 같은 30m 제한의 matched confirmed-track 비교다. 수치는 **기본 대비 차이**이지
GT 오차가 아니다. confirmed 집합 변화는 추가/누락이 관측된 frame-class 건수다.

| 일반 compile 결합 | 위치 차이 p95 / 최대 (m) | 2D 속도 차이 p95 / 최대 (m/s) | confirmed 집합 변화 |
|---|---:|---:|---:|
| O02 | 0.00917 / 0.48766 | 0.03055 / 0.95512 | 1 |
| P01 | 0.000147 / 0.000215 | 0.000376 / 0.000559 | 0 |
| H03 | 0.000794 / 0.003983 | 0.006404 / 0.014170 | 0 |
| R01 | 0.000153 / 0.000279 | 0.000753 / 0.003753 | 0 |

O02에서는 가까운 UNKNOWN track 1/4의 association·coasting 대응이 달라지는 구간을
관측했다. 같은 시간의 실제 코어 출력에서 baseline track 4는 약 `[-61.237,-177.786]`,
compile track 4는 약 `[-60.586,-177.334]`였으며 양쪽 모두 미관측 coasting이었다.
단순히 서로 다른 숫자 ID를 오류로 센 현상만은 아니다. 대응 진단 10건은 GT ID switch
10회를 뜻하지 않으며, 어떤 경로가 실제 GT에 더 정확한지는 별도 평가가 필요하다.
전체 model ROI 재생에서도 이 예외가 남았다. O threshold0.15 후보도 해결하지 못했다.
UNKNOWN 실제 속도는 후속 stationary 정책에서 0이 되지만 위치·대응 차이까지
사라지는 것은 아니다. 작은 수치 차이가 이산적인 association/분기에 증폭될 수 있다.

병렬화만 사용한 30m 재생은 네 bag 모두 confirmed 집합·ID 대응 변화 0이었다.
O02 위치 차이 p95/최대는 0.000654/0.04354m, 다른 세 bag의 최대는 0.000156~0.00824m다.
이 결과도 full ROS나 다른 환경의 안정성을 인증하지 않는다.

### CUDA Graph 대안과 결론

학습이나 모델 구조를 바꾸지 않고 원래 camera 연산을 capture/replay하는 별도 prototype을
조사했다. benchmark의 `graph_parallel_probe`와 offline tool의 `--camera-cuda-graph-probe`
만 사용한다. production constructor의 미검증 Graph+parallel 조합 거부는 그대로 둔다.

4090 전체 val 745장은 평균/p95 **72.36/73.61ms**, startup/steady allocated peak 약
**17.21/1.67GiB**였다. 고정 gate 실패는 1장이며 동일 장면은 기본 반복에서도 실패했다.
기본 mAP77.366%와 Graph77.366%가 같았다. 1,570프레임의 30m 코어 재생은 confirmed
집합·ID 대응 변화가 0이고 O02 위치 차이 p95/최대는 **0.000175/0.001874m**였다.
전체 model ROI 재생에서도 네 bag 모두 집합·대응 변화 0이었다. 하지만 **단독 병렬화보다
빠르지 않으므로 이번 기본 배포 후보로 추가하지 않는다.**

**현재 권고:** 병렬화 우선 시험을 유지한다. 일반 compile은 66ms 이득은 분명하지만
장애물의 연속 코어 예외가 있어 **기본 ON으로 올리지 않는다**. 연산/레이아웃 유지도
이를 확실히 해결하지 않았으며 Graph는 추가 속도 이득이 없었다. AP만으로 ON을 승인하거나
고정 gate를 느슨하게 바꾸지 않는다. E25 production·기존 threshold·tracker는 미변경이다.
다음은 실제 4090 24GB의 별도 Base5 ROS trial에서 queue·LAN·tracker·UNKNOWN 정책까지
확인하는 것이다. 여기서 관측한 코어 차이를 실주행 충돌이나 모든 compile 실행의 실패로
확대 해석하지 않는다.

### 산출물과 재현

- `E24_guarded_val745_v1/report.json`, `accuracy_v1.json`, `effects_v1.json`: 원격 후보·전달 진단.
- `E24_graph_parallel_val745_v1`: 별도 원격 Graph prototype.
- `base5_compile_bag_261012/E24_amp_*/predictions.json.gz`, receipt: 실제 연속 입력의 출력.
- `bag_all_accuracy_v2.json`, `tracker_full_v2.json`, `tracker_radial30_v2.json`,
  `tracker_obstacle015_v2.json`, `tracker_graph_*`: 검출/코어 비교와 범위·policy 분리.
- `tracker_repeat_control_v2.json`: 별도 실제 코어 반복 control. 초기 relative info 경로
  실패와 import/JSON serialization 실패도 당시 로그/빈 폴더로 보존한다.

코어 재생은 ROS Python 환경에서 다음처럼 실행한다. 다른 master에 발행하지 않는다.

```bash
python3 perception/detection3d/bevfusion/tools/replay_base5_compiled_tracking.py \
  --inputs artifacts/evaluation/bevfusion/base5_rosbag_continuous_261012/inputs \
  --binary artifacts/evaluation/bevfusion/base5_compile_review_261012/tracker_core \
  --sensor-config configs/sensors/Sensor_4.json --maximum-range-m 30 \
  --predictions \
  artifacts/evaluation/bevfusion/base5_compile_bag_261012/E24_amp_baseline_v2/predictions.json.gz \
  artifacts/evaluation/bevfusion/base5_compile_bag_261012/E24_amp_compiled_v1/predictions.json.gz \
  --output artifacts/evaluation/bevfusion/base5_compile_review_261012/tracker_repeat_new.json
```

검증은 Docker CPU **48개 통과·CUDA optional 4개 skip**, host의 새 효과/코어 단위 검사,
기존 C++ 소스의 실제 링크·독립 반복 control·네 bag 코어 실행, 실제 4090 후보 비교와
로컬 네 경로의 1,570프레임 전수 추론이다. py_compile·whitespace·문서 언어 검사도 통과했다.
원격·로컬 GPU 작업을 종료하고 결과를 보존했으며 Vast 인스턴스 자체는 자동 중단하지 않는다.

<a id="base5-compile-population-distributions-261012"></a>

## 38. 컴파일 OFF/ON의 전체 객체 분포 재분석 — 2026-10-12

사용자가 최대값이나 특정 객체만으로 판단하지 않고 평균·중앙값·p90·p95 등을
확인하도록 요청했다. 37절의 저장된 E24 출력과 같은 실제 C++ EKF 코어를 사용해
CPU 재분석을 완료했다. 새 GPU 추론·학습·Vast 접속·MORAI 조작은 하지 않았다.
데이터·체크포인트·production·기존 threshold는 변경하지 않았다.

### 비교 조건과 분모

- 연속 dev bag 4개 전체 적격 **1,570프레임**을 사용한다. 장애물뿐 아니라 차량·
  보행자를 모두 집계하며, 출력의 class를 기준으로 구분한다. GT와의 정확도 오차가 아니다.
- 주요 비교는 **병렬화 ON + compile OFF** 대 **병렬화 ON + camera backbone 및
  neck compile ON**이다. 가중치·입력·AMP·TF32 OFF·cuDNN benchmark OFF·기타 옵션이
  동일한지 검사한다. 37절의 기본 실행 대 compile+병렬화 비교도 별도로 유지한다.
- 각 객체·프레임의 대응된 출력 하나에 동일한 가중치를 준다. 반복 관측 수를 서로
  다른 물리적 객체 수로 해석하지 않는다. p50은 중앙값이며 평균은 절대 차이의 평균이다.
- 클래스별 XY Hungarian 대응은 기본 **0.5m 미만**이다. 미대응 수를 따로 보존하며
  분포에서 빠진 상태 차이를 조사하기 위해 **2m 대응 민감도**도 계산한다. 특히
  가까운 track들의 대응 변경은 실제 GT identity가 보장된 비교가 아니다.
- tracker는 confirmed 상태만 비교한다. 30m는 입력 detection을 radial 30m로 제한한
  코어 재생이며, full ROI는 학습된 X±100/Y±30m 모델의 전체 전달 출력을 사용한다.
  30m GT의 정확도 평가를 100m까지 확장한 것이 아니다.

### 검출 중심 차이: 전체 모델 범위

단위는 **mm**다. score floor 0.1·상위 100개·class-aware rotated NMS 0.5와
Vehicle/Pedestrian/Obstacle threshold 0.4/0.15/0.1을 동일하게 적용한다.

| 출력 클래스 | 대응 관측 수 | 평균 | 중앙값 | p90 | p95 | p99 | 최대 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 차량 | 1,349 | 0.147 | 0.110 | 0.275 | 0.343 | 0.678 | 3.059 |
| 보행자 | 201 | 0.087 | 0.073 | 0.197 | 0.217 | 0.303 | 0.442 |
| 장애물 | 1,240 | 0.107 | 0.099 | 0.215 | 0.246 | 0.402 | 1.099 |
| 전체 | 2,790 | 0.125 | 0.099 | 0.222 | 0.303 | 0.499 | 3.059 |

양쪽 전달 수는 각각 2,791개다. 차량·보행자는 추가/누락 0이고 장애물은 서로 다른
두 프레임에서 추가 1·누락 1이다. radial 30m에서는 양쪽 1,479개가 모두 대응되고
전달 추가/누락은 0이다. 30m 검출 중심의 전체 평균/p95는 **0.103/0.222mm**다.

중심 차이가 작다고 yaw·BBox 경계까지 같다는 뜻은 아니다. 30m 장애물의 yaw 차이는
평균/중앙값/p90/p95/p99 **0.137/0.028/0.280/0.448/1.802°**다.
전체 범위에서는 다음과 같다. 모서리 차이는 π/정사각형 대칭을 고려한 대응이다.

| 출력 클래스 | 모서리 차이 평균 / 중앙값 / p90 / p95 / p99 / 최대 (cm) | yaw 차이 p95 / 최대 (°) |
|---|---|---|
| 차량 | 0.178 / 0.071 / 0.290 / 0.424 / 0.779 / 32.188 | 0.056 / 6.218 |
| 보행자 | 0.095 / 0.051 / 0.192 / 0.350 / 0.622 / 0.943 | 0.448 / 1.588 |
| 장애물 | 0.226 / 0.059 / 0.320 / 0.492 / 1.871 / 81.194 | 0.477 / 28.088 |

장애물의 최대 모서리 차이는 `O02_164012_00178`, 차량은 `H03_162301_00043`이다.
이 값은 두 실행의 footprint 차이이며 실제 물체와의 오차가 아니다. 30m에서는
장애물 모서리 차이 p95/p99/최대가 **0.490/1.986/7.847cm**다.

### Tracker 위치 차이: radial 30m 코어 재생

단위는 **cm**다. 대응된 객체·프레임 수이며 실제 물리적 객체 수가 아니다.

| 출력 클래스 | 대응 관측 수 | 평균 | 중앙값 | p90 | p95 | p99 | 최대 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 차량 | 493 | 0.016 | 0.008 | 0.023 | 0.036 | 0.182 | 0.768 |
| 보행자 | 124 | 0.006 | 0.005 | 0.012 | 0.013 | 0.017 | 0.021 |
| 장애물 | 1,309 | 0.699 | 0.009 | 0.089 | 1.288 | 22.055 | 48.760 |
| 전체 | 1,926 | 0.480 | 0.008 | 0.046 | 0.411 | 16.263 | 48.760 |

30m 장애물의 평균 0.699cm만으로는 소수 구간의 영향을 알 수 없다.
장애물의 10cm 초과는 **30/1,309 = 2.29%**, 20cm 초과는 **14/1,309 = 1.07%**다.
10cm 초과는 `O02_164012`의 reference track 1/4에 집중된다. 이는 실제 두 개의
GT 물체나 '전체 장애물의 2.29%가 오검출'이라는 뜻은 아니다.

참고로 장애물의 대응된 21개 reference 추적 생애 구간별 평균 위치 차이에 각각
같은 가중치를 주면, 평균/중앙값/p90/p95는 **0.421/0.014/0.176/3.803cm**다.
ID 재사용 시 사라진 뒤 다시 나타난 구간을 분리해, 오래 관측한 track만으로 평균이
결정되지 않도록 했다. 클래스가 변하는 구간은 클래스별 개수의 합이 전체와 다를 수 있다.
어느 쪽도 GT identity를 인증한 통계는 아니다.

장애물의 association별로 보면 양쪽 관측 업데이트 837건의 위치 차이 평균/p95는
**0.144/0.183cm**, 양쪽 coasting 460건은 **1.020/3.086cm**, 관측 업데이트 여부가
다른 12건은 **27.144/47.226cm**다. 평균·중앙값이 작아도 대응이 다른 소수 구간은
별도로 평가해야 한다. 이산적인 대응 차이는 관측했지만 모든 수치 차이의 인과는 미확정이다.

### Tracker 위치 차이: 전체 모델 범위

같은 0.5m 대응의 집계이며 단위는 **cm**다.

| 출력 클래스 | 대응 관측 수 | 평균 | 중앙값 | p90 | p95 | p99 | 최대 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 차량 | 1,530 | 0.091 | 0.011 | 0.050 | 0.136 | 0.777 | 11.877 |
| 보행자 | 233 | 0.007 | 0.006 | 0.012 | 0.014 | 0.018 | 0.025 |
| 장애물 | 1,975 | 0.639 | 0.009 | 0.113 | 1.295 | 19.341 | 48.760 |
| 전체 | 3,738 | 0.375 | 0.010 | 0.065 | 0.324 | 11.654 | 48.760 |

### 대응 gate에 따른 최대값 한계와 속도 차이

**37절의 약49cm를 전체 상태 차이의 절대 최대처럼 읽으면 안 된다.** 0.5m gate 밖은
미대응으로 빠진다. radial 30m에서 양쪽confirmed 수는 각각1,927건이고, 이 중
1프레임·UNKNOWN의 1건씩이 기본gate로 대응되지 않는다. 이는 양쪽track 수가 다르거나
객체를 실제로 삭제했다는 증거는 아니다.

2m 민감도에서는 장애물 대응 수가1,310건으로 늘고, 위치 차이 평균/중앙값/p90/p95/p99가
**0.787/0.009/0.092/1.340/22.385cm**, 최대는 **113.718cm**다. 넓은 gate의 대응은
가까운 track 사이에서 바뀔 수 있으므로 이 값을 특정 GT 객체의 이동 오차로 확정하지 않는다.
49cm가 보장된 상한이 아니라는 한계를 확인한 것이다. full ROI도 같은 민감도 예외가 남는다.

30m tracker의 2D 속도 벡터 차이 평균/중앙값/p90/p95/p99/최대는 다음과 같다.

| 출력 클래스 | 2D 속도 차이 분포 (m/s) |
|---|---|
| 차량 | 0.000845 / 0.000324 / 0.001481 / 0.002758 / 0.011923 / 0.026457 |
| 보행자 | 0.000193 / 0.000147 / 0.000355 / 0.000423 / 0.000703 / 0.001254 |
| 장애물 | 0.017690 / 0.000191 / 0.003179 / 0.038399 / 0.630706 / 0.955125 |

GT 속도 MAE/AVE가 아니라 UNKNOWN stationary 후처리 전의 코어 상태 차이다.

### Native val 745장 보충과 결론

원격에서 이미 저장한 E9/E24의 745장 출력을 같은 병렬화 ON·compile OFF/ON 조건으로
재분석했다. E24의 1,624개 대응 관측에서 중심 차이 평균/p95는 **0.110/0.288mm**,
E9의 1,817개 관측은 **0.102/0.222mm**다. E24는 전달 차량 추가 1·장애물 누락 1,
E9는 장애물 누락 1이며 다른 클래스의 추가/누락은 0이다. 순서 없는 val을 연속
tracker로 재생하지 않았다. 단일 bag 이외의 클래스 관측도 보충했으며 37절의 AP를
새로 측정한 것은 아니다.

결론은 **전반적인 검출 차이는 매우 작고, 차량·보행자 tracker도 대부분 거의 같다**는 것이다.
컴파일을 전반적인 정확도 저하나 학습 실패로 판단할 근거는 없다. 다만 장애물의 소수
연속 구간에서 대응/coasting 차이가 남으므로 평균만으로 기본 ON을 승인하지 않는다.
병렬화 우선과 compile opt-in 비교를 유지하며, 다음은 실제 ROS의 orientation/UNKNOWN
후처리까지 포함한 해당 구간 ON/OFF 비교다. 전체 미사용 bag·실제24GB·LAN 검증은 남았다.

### 산출물·검증

재분석 도구는 `perception/detection3d/bevfusion/tools/summarize_base5_compile_distributions.py`다.
ignored 결과는 `artifacts/evaluation/bevfusion/base5_compile_distributions_261012_v3`에 둔다.
`summary.json`과 전체 대응 row·세 실행의 실제 core frame을 gzip으로 보존한다. 각 분포의
분모·미대응·tail 비율·association별·track별 평균 분포·원본 SHA256을 기록했다.
초기 v1은 클래스 enum 확인 후 중단했고, v2는 초기 분포 집계다. v3의 비교 조건 검사·
ID 생애 구간 분리까지 완료한 결과가 최신이다. production C++ tracker는 수정하지 않았다.

분포·대응·gate·재사용 ID·미대응·클래스 구분 단위 검사와 기존 runtime 검사 **29개 통과**,
실측 분모 보존 대조 72건·`py_compile`·`git diff --check`·신규 절 언어 검사를 통과했다.
전체 GPU 실행을 새로 했다는 주장은 하지 않는다.

<a id="base5-compile-actual-ros-tracker-261012"></a>

## 39. 실제 ROS tracker·TF·UNKNOWN 후처리의 컴파일 비교 — 2026-10-12

38절 다음 단계에 대한 사용자 승인으로 실제 ROS 노드를 실행했다. 저장된 E24의
병렬화 ON·compile OFF/ON 출력을 사용하고, 같은 OFF 입력을 별도 노드에 한 번 더
넣어 독립 반복 통제로 삼았다. 모델 재추론·학습·Vast 접속·MORAI 조작은 하지 않았다.
다른 agent의 planning_validator 변경과 production 설정도 그대로 보존했다.

### 실행 범위와 분리

기존 C++ tracker의 node/core 소스를 수정하지 않고 별도 ignored 위치에 직접 빌드했다.
private ROS master에서 세 tracker와 세 실제 `tracked_object_motion_policy_node.py`를
실행했다. Sensor_4 extrinsic을 `/tf_static`에, 기록된 Ego 위치와 heading−1.347°로
계산한 같은 pose를 `/tf`에 발행했다. 실제 tracker callback의 TF 변환과 quaternion
yaw 정규화, 차량 orientation canonicalization, BBox 크기 필터, confirmed lifecycle,
UNKNOWN stationary policy를 모두 통과한 `TrackedObjects`를 수집했다.

이는 **저장된 검출 출력 이후의 실제 ROS 경로 시험**이다. ROS 센서→GPU 추론→LAN→
queue 전체 시험이나 실제 localization·planning·Map-CV·GT 속도 오차 시험은 아니다.
`map` TF는 두 실행의 비교를 위한 같은 기록 Ego 기반이며 localization 추정 오차를
재현한 것이 아니다. 물리적 4090 24GB의 배포 검증도 아직 아니다.

| 실행 | 입력 | 세 실행 각각의 raw/policy 출력 수 | 목적 |
|---|---|---:|---|
| `sync_full_v1` | 네 bag·1,570프레임·전체 모델 범위 | 각 1,570프레임 | 검출 시각별 노드·후처리 동등성 |
| `sync_radial30_v1` | 장애물 O02·463프레임·radial 30m | 각 463프레임 | 앞선 근거리 예외 재현 |
| `fixed30_full_v1` | 네 bag·1,570프레임·전체 모델 범위 | 각 3,771프레임 | 내부 30Hz 설정의 예측 복사 출력 포함 |

내부 scheduler는 운영 alias와 같은 30Hz·최대 prediction age 0.50s, 외부 resampler
OFF로 설정했다. 기록 시각의 `/clock`을 통제해 비교했으며 물리적인 30Hz 처리 성능을
측정한 것이 아니다. scheduler는 늦은 슬롯을 건너뛸 수 있고 H03의 선택된 입력에는
10.322s·2.237s의 긴 공백도 있다. 출력 수가 `30 × bag 길이`와 같아야 하는 시험이 아니다.
네 bag의 출력 수는 각각 O02 1,147·P01 646·H03 1,226·R01 752이며 세 실행에서
출력 timestamp가 모두 일치했다. 이 실행의 wall time은 약 198초다.

초기 `fixed_smoke_v1`에서는 한 노드가 시작할 때 static TF를 받기 전 첫 입력이
도착했다. 이를 컴파일 실패로 해석하지 않고 비교 대상에서 제외했다. harness에서
static/dynamic TF 연결 대기·static TF 재발행·첫 출력 확인을 보강했다.
`fixed_smoke_v2`의 각 99프레임과 최종 세 실행에는 TF 실패·비증가 stamp 경고가 없다.
production의 TF 코드나 검사 기준을 바꾼 것은 아니다.

### 실제 30Hz 설정·후처리 뒤의 위치 차이

양쪽 병렬화 ON·같은 AMP 조건이며 아래는 **두 실행 사이의 위치 차이**다. GT 오차가
아니다. 단위는 **cm**이고 0.5m 미만 class-preserving XY 대응을 사용했다. 관측 수는
같은 객체의 여러 시각 출력을 포함한다.

| 출력 클래스 | 대응 관측 수 | 평균 | 중앙값 | p90 | p95 | p99 | 최대 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 차량 | 3,596 | 0.117 | 0.011 | 0.045 | 0.108 | 4.090 | 12.453 |
| 보행자 | 560 | 0.007 | 0.006 | 0.014 | 0.015 | 0.020 | 0.029 |
| 장애물 | 4,901 | 0.631 | 0.010 | 0.119 | 1.408 | 18.919 | 49.983 |
| 전체 | 9,057 | 0.388 | 0.010 | 0.062 | 0.429 | 11.006 | 49.983 |

장애물의 10cm 초과는 **93/4,901 = 1.90%**, 20cm 초과는 **47/4,901 = 0.96%**다.
차량의 10cm 초과는 **8/3,596 = 0.22%**, 보행자는 0건이다. 전반적인 평균 차이는
작지만 장애물의 소수 대응 구간은 후처리 뒤에도 남는다.

서로 다른 실행의 numeric ID 대응 변화는 O02에서 16건이며 GT ID switch 16회가 아니다.
나머지 bag은 대응 변화 0이다. 양쪽 관측 수는 각각 9,062건으로 같고, 0.5m gate의
미대응은 장애물의 5프레임에서 reference/candidate 각 5건이다. 이는 객체를 실제로
추가하거나 삭제했다는 의미와 구분한다. 2m 민감도에서는 모두 대응되며 장애물 위치
차이 평균/p95/최대가 **0.745/1.442/127.240cm**다. 넓은 gate도 실제 GT identity를
보장하지 않으며, 기존 약49cm가 절대 상한이 아님을 다시 확인한 것이다.

### 속도 0 후처리가 해결하는 것과 남기는 것

UNKNOWN 후처리는 **출력 속도·yaw rate를 0으로 만들지만 pose·yaw·크기·ID는 보존**한다.
tracker 내부 상태를 다시 정지 상태로 바꾸거나 위치를 고정하는 기능이 아니다.

- 실제 실행에서 장애물의 후처리 뒤 2D 속도 차이는 평균·중앙값·p95·최대 모두 **0**이다.
  후처리 전 속도 벡터 차이는 평균 **0.01446m/s**, p95 **0.02811m/s**였다.
- 각 실행의 raw/policy 출력에 대해 timestamp·ID·pose·shape·yaw·confidence가
  보존되는지 검사했다. 모든 UNKNOWN의 출력 속도·yaw rate 0과 stationary=true를
  확인했고, Vehicle/Pedestrian은 후처리로 바뀌지 않았다.
- 따라서 출력의 가짜 속도 차이는 제거되지만, 다른 association·coasting 이력을
  따른 위치와 BBox 경계 차이는 제거되지 않는다.

| 출력 클래스 | BBox 모서리 차이 평균 / 중앙값 / p90 / p95 / p99 / 최대 (cm) |
|---|---|
| 차량 | 0.240 / 0.069 / 0.205 / 0.409 / 5.828 / 13.021 |
| 보행자 | 0.032 / 0.023 / 0.070 / 0.096 / 0.126 / 0.138 |
| 장애물 | 1.296 / 0.068 / 1.388 / 7.936 / 30.505 / 102.520 |

모서리는 π·정사각형 대칭을 고려한 대응이다. 근처 track의 대응 변경으로 다른 이력의
shape를 비교할 수 있으므로 이 수치도 실제 물체 경계에 대한 오차로 단정하지 않는다.
UNKNOWN의 속도를 0으로 만드는 것만으로 컴파일 차이가 모두 흡수된다는 가설은 기각한다.

### 검출 동기 모드·근거리 통제

전체 1,570프레임 검출 동기 모드의 후처리 위치 차이 평균/p95는 차량
**0.102/0.084cm**, 보행자 **0.007/0.014cm**, 장애물 **0.639/1.294cm**다.
네 bag의 모든 입력 stamp가 raw/policy 출력에 빠짐없이 대응됐다.

O02 radial 30m에서는 장애물의 1,306개 대응 관측에서 위치 차이 평균/중앙값/p90/p95/p99가
**0.701/0.009/0.090/1.304/22.133cm**, 최대 **48.760cm**다.
BBox 모서리 차이 p95는 **10.436cm**다. 출력 속도 차이는 모두 0이지만 위치·경계의
예외는 남았다. 이 근거리 실행은 장애물 bag 하나이며 다른 클래스의 전수 통계는 위의
네 bag 실행을 따른다.

같은 OFF 입력을 받은 별도 노드의 반복 통제는 검출 동기·radial 30m·30Hz 설정 모두
timestamp와 관측 집합이 같고, 위치·크기·yaw·confidence·모서리·속도 차이가 **0**이다.
이번 차이는 이 harness에서 관측한 ROS 스케줄링 잡음으로만 설명되지 않는다.

### 결론·다음 단계·산출물

평균적인 검출·차량·보행자 동등성은 유지하지만, **실제 ROS 후처리도 장애물의 소수
위치·BBox 차이를 없애지는 못한다.** 일반 compile은 선택 가능한 opt-in 시험으로
남기고 기본 ON으로 승격하지 않는다. 약72ms의 병렬화 우선 후보를 유지한다. 학습
가중치를 폐기하거나 재학습할 근거가 생긴 것은 아니다.

다음은 실제 4090 24GB의 별도 Base5 ROS 입력→GPU→queue/LAN 전체 비용·누락 검사다.
이번 시험에는 물리적 24GB GPU가 없었고 Vast의 48GB cap 시험으로 대체 인증하지 않는다.
planning·실주행 안전성·GT AVE·100m 정확도와 새로운 환경 검증도 별도 과제다.

도구는 `replay_base5_ros_tracking.py`, `analyze_base5_ros_tracking.py`이며 ignored 루트는
`artifacts/evaluation/bevfusion/base5_compile_ros_tracking_261012`다. 원본 소스에서 빌드한
`tracker_node`·세 실제 실행의 `messages.json.gz`·`receipt.json`·`analysis_v2.json`·
전체 대응 row·node log를 보존했다. `analysis.json`은 최초 집계이고, pooled 미대응
개수까지 추가한 `analysis_v2.json`이 최신이다. source/binary/input SHA256을 기록했다.

관련 단위 검사 **36개 통과**, 실제 세 실행의 분모 보존 대조 **288건 통과**, 같은
timestamp·반복 통제 0·후처리 보존·로그 오류 검사와 `py_compile`을 통과했다.
자신이 시작한 private ROS master·노드만 종료했으며 다른 agent와 기존 GUI는 유지했다.

<a id="base5-local-threshold-calibration-261012"></a>

## 40. 로컬 threshold 선정·실제 tracker 후보 비교 — 2026-10-12

사용자는 원거리 bag 평가·풀스택 영상 공유·tracker 개선을 후속 후보로 남기고,
먼저 로컬 threshold를 정하도록 요청했다. 대여한 4090을 잠시 중단하겠다고 했으므로
원격 접속·GPU 추론·학습·Drive 작업은 하지 않았다. 저장된 예측을 CPU로 분석하고
private ROS master에서 실제 tracker·UNKNOWN policy를 재생했다.

### 현재 채택: E24 로컬 시험값

| 클래스 | 앞선 비교 기준 | 이번 로컬 시험값 | 판단 |
|---|---:|---:|---|
| Vehicle | 0.40 | **0.40** | 낮추면 native val recall은 개선되지만 현재 mission tracker에서는 recall 증가 없이 미대응 출력이 늘었다. 원거리 bag에서 다시 판단한다. |
| Pedestrian | 0.15 | **0.10** | E24의 근접 사람·복합체를 일부 더 유지하며 현재 bag의 tracker 미대응·끊김 증가는 없었다. |
| Obstacle | 0.10 | **0.10** | 0.12/0.15는 중복성 출력을 줄이지만 선명한 근접 GT도 잃는다. threshold만으로 tracker 중복을 해결하지 않는다. |

이는 **2026-10-12 Sunny13 5K 학습의 E24·AMP·병렬화 ON·추론 compile OFF**를 위한
개발 시험값이다. E24 SHA256은 33절과 같은
`84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746`이다.
E9나 향후 30K 재학습의 최적값, 다른 환경·실전 안전성을 인증한 값이 아니다.
기존 E25 production과 기본 비교용 `base5_trial.launch`는 유지한다.

### 분석 범위·평가 계약

- 같은 4개 dev bag의 모든 적격 **1,570프레임**에 대해 E9 FP32·E24 FP32·E24 AMP
  병렬화 출력을 사용했다. score **0.10~0.80, 간격 0.01의 71개 후보**를 각 클래스에
  비교했다. 현재 server floor 0.1·상위100개·class-aware rotated NMS 0.5를 유지한다.
  0.1 미만이 최적이 아니라는 결론은 내리지 않는다.
- bag GT와 정확도 평가는 기존 **radial 30m 및 공통 XYZ 범위**에 한정한다.
  30m 밖의 출력은 threshold별 개수만 따로 기록하며 FP로 세지 않는다.
- primary TP는 score 순서의 동일 클래스 중심 거리 <1m greedy 대응이다. <2m도
  별도로 계산하고, 미대응을 같은 클래스 GT 1m 이내·1~2m·다른 클래스 GT 2m 이내·
  나머지로 분해했다. 이는 중복·위치 오차·클래스 혼동의 진단이지 실제 원인 확정이 아니다.
- 누락 구간·hit/miss 전환은 GT key 안에서만 계산한다. 원본 LiDAR index가 끊기거나
  GT/ROI에서 빠지거나 sensor gap>0.25s면 연결하지 않는다. 반복 프레임과 재구성된
  GT key를 독립 표본·authoritative simulator ID로 해석하지 않는다.
- ≥5 LiDAR points는 추가 진단이며 전체 GT를 삭제하는 기준이 아니다. 강한 point
  지원이 있어도 낮은 score가 나오는 자산을 threshold 선정에서 확인했다.
- 원격에서 이미 저장한 E9/E24의 **전체 native val 745장·AMP 병렬화**도 전수
  분석했다. 현지 train 표본이 섞이지 않았는지 검사하고, native GT로 full ROI와
  radial30을 구분했다. native 원거리 통계는 100m bag 평가를 대신하지 않는다.

### E24 native val의 threshold 영향

아래는 radial 30m의 **1m 중심 대응**이며 AP가 아니다. GT 관측 분모는 Vehicle 187,
Pedestrian 493, Obstacle 539다. 표의 변화는 각 클래스 threshold만 바꾼 결과다.

| 클래스·후보 | TP / FP | Precision / Recall (%) | 판단 |
|---|---:|---:|---|
| Vehicle 0.40 | 173 / 4 | 97.74 / 92.51 | mission 초기값 유지 |
| Vehicle 0.20 | 179 / 5 | 97.28 / 95.72 | 범용 recall 대안, mission tracker 결과와 함께 보류 |
| Vehicle 0.15 | 180 / 5 | 97.30 / 96.26 | 낮은 값이 항상 나쁘지는 않지만 아직 실전 기본으로 채택하지 않음 |
| Pedestrian 0.15 | 439 / 14 | 96.91 / 89.05 | 앞선 비교 기준 |
| Pedestrian 0.10 | 450 / 22 | 95.34 / 91.28 | 11개 GT 관측 회복·FP 8개 증가의 trade-off |
| Obstacle 0.10 | 435 / 49 | 89.88 / 80.71 | 누락 억제 우선 |
| Obstacle 0.12 | 427 / 42 | 91.04 / 79.22 | FP 7개 감소와 TP 8개 감소 |
| Obstacle 0.15 | 407 / 30 | 93.14 / 75.51 | 근접 recall 손실이 큼 |

Pedestrian 0.10으로 회복한 GT에는 Woman·police·유모차·반려동물 동반 사람과 Bike의
구성 GT가 포함된다. ≥5 point 지원의 GT도 8개 회복됐다. full ROI의 전체 Pedestrian
recall은 **81.80→84.13%**, precision은 **95.90→94.60%**다.

Vehicle 0.20의 근접 회복 6건은 일반 차량 6대가 아니다. ElectricScooter1·NCAP_EBT·
Bike2 등 해당 분류의 구성 GT와 Hyundai Genesis가 포함된다. 따라서 이를 고주로 NPC
6대의 회복으로 설명하지 않는다. full ROI recall은 82.40→86.22%이나 실제 mission
입력의 추가 미대응과 원거리 GT를 확인한 뒤 낮은 threshold를 채택해야 한다.

Obstacle 0.12에서 잃은 8개 근접 GT 중 6개는 ≥5 point 지원이 있었다.
예를 들어 Scooter_01은 175 points·약7.3m, Bicycle_city는 58 points·약7.8m였다.
저점수라고 모두 보이지 않는 GT는 아니다. 0.12는 별도 정밀도 대안으로 기록하되
이번 기본값으로 선택하지 않았다.

### Bag의 검출과 실제 tracker 후보 검증

세 클래스의 threshold를 함께 바꾼 5개 프로필을 각각 같은 E24 AMP 병렬화 출력으로
재생했다. 각 실행은 기존값·후보·기존값의 독립 반복을 실제 C++ tracker/TF와 UNKNOWN
policy에 넣는다. 모든 후보에서 **각 노드의 1,570개 input stamp와 raw/policy 출력이
빠짐없이 대응**했고, 기존값의 독립 반복 차이는 0이었다. TF 실패·비증가 stamp·node
오류가 없었으며 소유한 master·노드만 종료했다.

| 프로필 V/P/O | 목적 |
|---|---|
| 0.15 / 0.10 / 0.10 | 낮은 차량 threshold의 recall 후보 |
| 0.20 / 0.10 / 0.12 | 범용 recall·중복 억제 조합 |
| 0.40 / 0.10 / 0.15 | 장애물 precision 우선 조합 |
| 0.20 / 0.10 / 0.10 | 낮은 차량값·장애물 recall 유지 조합 |
| **0.40 / 0.10 / 0.10** | **이번에 선택한 로컬 mission 초기 시험값** |

모든 bag의 30m 내 해당 클래스 출력에 대해 primary 1m 중심 대응으로 집계한다.
아래 FP는 중복·coasting·위치 차이·경계 효과 등을 포함한 평가 미대응이지, 모두
실제로 없는 물체가 생겼다는 의미가 아니다. GT는 기존 보간·catalog 근사다.

| 실제 tracker 후보 | Vehicle TP / FP | Pedestrian TP / FP | Obstacle TP / FP |
|---|---:|---:|---:|
| 기존 0.40 / 0.15 / 0.10 | 427 / 44 | 113 / 6 | 883 / 385 |
| 0.15 / 0.10 / 0.10 | 427 / 63 | 113 / 6 | 883 / 385 |
| 0.20 / 0.10 / 0.10 | 427 / 59 | 113 / 6 | 883 / 385 |
| 0.20 / 0.10 / 0.12 | 427 / 59 | 113 / 6 | 882 / 340 |
| 0.40 / 0.10 / 0.15 | 427 / 44 | 113 / 6 | 879 / 311 |
| **이번 0.40 / 0.10 / 0.10** | **427 / 44** | **113 / 6** | **883 / 385** |

낮은 Vehicle 후보는 native val 개선과 달리 현재 bag의 tracker recall을 늘리지 않고
미대응 관측을 15~19건 늘렸다. 기존의 차량 1m 미대응 상당수는 GT 근처 1~2m 위치
차이이며 높은 score도 포함한다. threshold를 높이거나 낮추는 것만으로 해결되지 않는다.
이 때문에 0.40을 유일한 최적값으로 선언하지 않으면서, 현재 mission 초기값은 유지한다.

Obstacle 0.12/0.15는 미대응을 줄였으나 0.15는 O02의 0~10m tracker TP도
292→291로 줄었다. native 자산별 손실과 함께 보면 aggressive threshold 상승보다
중복 birth·association·coasting을 별도 조사하는 편이 낫다. 이번에는 tracker를 수정하지
않았다. 같은 GT key의 대응 track 변화 진단도 실제 simulator ID switch로 주장하지 않는다.

이번 채택값은 현재 bag에서 기존값 대비 새 ≥0.5s 누락 구간이나 GT-key 대응 변화
증가가 없었다. Pedestrian의 추가 검출은 기존 tracker coasting이 이미 채우던 일부
프레임이어서 이 bag의 tracker recall 자체를 더 올리지는 않았다. native의 더 다양한
사람·복합체 관측을 유지하는 이득과 구분한다.

### 적용 경로와 다음 순서

새 opt-in launch는 `base5_e24_calibrated_trial.launch`다. 내부에서 기존 Base5 trial을
사용해 별도 topic/port 계약을 유지하고 V/P/O 0.40/0.10/0.10만 전달한다. 소스 파일을
직접 진입점으로 쓸 수 있으며 E24용 서버를 준비한 뒤 실행한다.

```bash
roslaunch "$AJOUNICE_ROOT/base_ws/src/perception/bevfusion_ros/launch/base5_e24_calibrated_trial.launch"
```

launch 자체가 모델 서버를 시작하거나 E24 hash를 강제로 검증하는 것은 아니다.
동일 E24·입력 계약·AMP 병렬화·compile OFF 서버를 별도로 준비해야 한다. 파일 이름과
위 설명이 현재 적용 범위를 명시한다. production E25 및 baseline Base5 trial은 미변경이다.
서버 score floor는 여전히 0.1이므로 모델의 forward 비용을 바꾸는 조정도 아니다.

다음은 **30~50/50~80/80~100m bag GT 확장과 차량 평가**다. 그 뒤 필요하면 낮은 차량
threshold·원거리 전달 안정성을 다시 판단하고, 팀장님의 실제 대회용 4090 24GB에서
ROS 입력→GPU→queue/LAN·풀스택 및 영상을 확인한다. tracker 개선은 확인된 failure를
근거로 별도 수행한다. 대여 인스턴스에서 실제 대회 PC의 검증을 대신하지 않는다.

### 산출물·검사

- 최신 fine sweep: `artifacts/evaluation/bevfusion/base5_threshold_calibration_261012_validated_v3/report.json`.
- 실제 5개 ROS 후보·quality·독립 반복: `base5_threshold_calibration_261012_v2/ros_*`,
  `tracking_quality.json`, `tracking_precision_quality.json`, `tracking_selected_quality.json`,
  **`tracking_adopted_quality.json`**. `ros_selected_candidate`는 0.20 후보의 당시 이름이며
  최종 채택은 **`ros_adopted_local_policy`**다.
- 새 CPU 도구: `calibrate_base5_thresholds.py`, `evaluate_base5_threshold_tracking.py`.
  기존 ROS replay는 동일 prediction file의 threshold-only 비교를 명시적으로 지원하고,
  compile 비교와 threshold 비교의 입력·옵션 계약을 구분하도록 보완했다.

초기 v1은 NumPy 정수의 JSON 저장 오류로 보고서가 완성되지 않았다. 보완한 v2와
native payload 검사를 포함한 v3의 fine 결과는 완전히 같았다. 이전 coarse sweep의
GT/TP/FP/FN **960개 항목 및 누락 구간 전체가 정확히 일치**했다. 캐시된 prefix matching은
기존 scorer와 무작위·동점·빈 GT·범위·지원점·시간 구간·실제 ROS 좌표 역변환으로 검사했다.
관련 CPU 단위 검사·launch 계약 검사 **49개 통과**, Python 구문·launch 해석·whitespace·
문서 언어 검사를 완료했다. GPU·원격 비용·실전 30Hz 처리 성능을 새로 측정한 것은 아니다.

<a id="base5-far-bag-coasting-261012"></a>

## 41. 원거리 rosbag 평가와 짧은 기억·잔류 비용 — 2026-10-12

사용자는 오검출이 적으면 객체 기억이 유용하지만, 오검출을 오래 기억하면 planning에
해가 될 수 있다고 지적했다. 30~50/50~80/80~100m 평가와 함께 이 trade-off를 조사했다.
꺼 둔 Vast에 접속하거나 GPU 추론·학습·MORAI·운영 설정을 변경하지 않았다.
E24 AMP·병렬화 ON·추론 compile OFF의 저장된 전 범위 검출을 그대로 재사용했다.

### 평가 범위 확장과 분모

기존 네 dev bag의 동일 1,570개 센서 token에서 `/Object_topic`과 감사한 asset catalog로
GT를 다시 구성했다. 영상·point cloud를 복사하거나 재추론하지 않았다. 이전 35m 내 GT의
위치·크기는 모두 그대로이며 **1,570개 prediction 배열·score·label은 정확히 불변**이다.
새 GT는 native SaveSensorData annotation이 아니라 기록 status의 보간·catalog 근사다.

- 모델 ROI는 LiDAR X±100/Y±30m, **bottom Z −3~2m**다. 거리 표는 XY 방사 거리다.
  방사 100m 밖 ROI 모서리는 전체 ROI 통계에만 포함하고 80~100m 표에 섞지 않는다.
- 미확정 차량이 ROI에 들어올 수 있는 프레임은 정확도 집계 전체에서 제외한다.
  ROI 반지름보다 넓은 105m와 기존 raw pivot guard 5m를 사용했다. 고주로의 미확정
  차량 두 segment 때문에 기존 적격 512장 중 157장을 추가 제외했다.
- 정확도 분모는 **1,413장**이다: O02 463, P01 269, H03 355, R01 326장.
  tracker에는 불확실한 157장도 포함한 원래 **1,570장 전부**를 넣어 이력 단절을 피했다.
- 재구성 trajectory의 최소 길이는 1로 설정했다. 짧은 GT를 삭제해 FP를 늘리지 않으며,
  실제 보간은 0.15초 이내 bracket만 사용하고 외삽하지 않는다. Object status 시각 지원도
  0.075초 이내여야 한다. 이 네 bag의 raw segment 수·자산 해결 결과는 이전과 같다.
- LiDAR 0점 GT도 삭제하지 않는다. ≥5점은 정보량 진단이다. 알려진 actor를 115m까지
  보존해 ROI 경계 밖 실제 객체를 근처 미대응 진단에서 확인할 수 있도록 했다.

이전 30m 표와 고주로 분모가 달라졌으므로 수치 상승을 모델 개선으로 해석하지 않는다.
이전에 사용한 dev bag의 추가 분석이며 미사용 test나 전체 자산·환경 인증이 아니다.
Object status에 없는 지도 배경도 있어, 미대응을 모두 실제로 없는 객체라고 부르지 않는다.

### 검출 AP와 현재 threshold의 원거리 recall

동일 ROS floor 0.1/top100/class-aware rotated NMS 0.5 뒤의 검출 AP다.
AP는 기존 0.5/1/2/4m 중심 gate의 평균이며 아래 고정 threshold recall과 다르다.

| 방사 거리 | Vehicle AP (%) / GT 관측 | Pedestrian AP (%) / GT 관측 | Obstacle AP (%) / GT 관측 |
|---|---:|---:|---:|
| 0~30m | 91.35 / 350 | 79.98 / 113 | 81.35 / 897 |
| 30~50m | 70.49 / 306 | 61.61 / 109 | 13.91 / 590 |
| 50~80m | 51.28 / 276 | 0.00 / 46 | 10.81 / 702 |
| 80~100m | 9.84 / 91 | 0.00 / **1: 소표본** | 0.00 / 286 |

AP 0은 검출이 한 번도 없었다는 뜻이 아니다. 현재 P0.10에서 50~80m 보행자는
4/46, O0.10에서 80~100m 장애물은 1/286개의 GT 관측을 2m 이내로 대응했다.
AP의 최소 recall 기준 때문에 이처럼 낮은 recall은 AP 0이 될 수 있다.
P01은 Man1 한 사람이고, O02는 six-asset 장면이다. 이 수치를 전체 pedestrian/122종
자산의 독립 검출 확률로 해석하지 않는다.

현재 V/P/O 0.40/0.10/0.10의 검출 recall은 모든 ROI GT에 먼저 대응한 뒤 거리별로
집계한다. AP의 구간별 양쪽 필터에서 발생할 수 있는 거리 경계 효과와 구분한다.

| 거리 | Vehicle recall@2m (%) | Pedestrian recall@2m (%) | Obstacle recall@2m (%) |
|---|---:|---:|---:|
| 30~50m | 84.31 | 71.56 | 25.76 |
| 50~80m | 69.57 | 8.70 | 21.65 |
| 80~100m | 13.19 | 0.00: GT 1 | 0.35 |

raw 검출 전체 ROI의 2m 미대응은 Vehicle 23/835, Pedestrian 18/212, Obstacle 124/1,241
관측이다. 이 숫자도 의미상 오검출률 인증이 아니다. 위치 오차·중복·다른 클래스 GT·배경을
분해해야 하며, 특히 원거리에서 못 찾는 GT 수와 가짜 검출 수를 혼동하지 않는다.

### 실제 ROS tracker의 기억 A/B

같은 detector 배열과 **V/P/O 0.40/0.10/0.10**을 TF·실제 C++ tracker·UNKNOWN policy에
넣었다. 두 독립 실행에서 기존값/후보/기존값 반복의 세 노드를 사용했다. 운영 config는
불변이며 private parameter만 바꿨다.

| 프로필 | 이동·보행자 coast | UNKNOWN·저속 차량 coast |
|---|---:|---:|
| 거의 즉시 삭제 | 0.001초 | 0.001초 |
| 현재 | 0.5초 | 1.0초 |
| 2배 유지 후보 | 1.0초 | 2.0초 |

confirmed/tentative·birth·association·위치 noise·UNKNOWN 후처리는 동일하다.
0.001초 후보는 단순 검출 출력이 아니라, 누락 뒤 재확인도 필요해지는 실제 tracker다.
실제 planner는 실행하지 않았고, 5초의 planner target 기억을 직접 A/B한 것도 아니다.
내부 30Hz 비용·live queue·LAN·조향 인증이 아니라 측정 stamp 동기 출력 비교다.
구간 길이는 마지막−첫 센서 stamp에 nominal period를 더한 표본 노출 추정이며,
GT/ROI 제외·원본 frame index 공백·0.25초 초과 sensor gap에서 분리한다.

아래는 전체 ROI에서 **중심 2m 이내** 대응한 GT 관측과 미대응 출력이다.
평균 상태가 가까운 것을 보기 위한 gate이며 세밀한 회피 경계 정확도를 인증하지 않는다.

| 클래스 | 거의 즉시 삭제 TP / 미대응 | 현재 TP / 미대응 | 2배 유지 TP / 미대응 |
|---|---:|---:|---:|
| Vehicle | 748 / 7 | **897 / 78** | 947 / 143 |
| Pedestrian | 170 / 1 | **233 / 13** | 246 / 45 |
| Obstacle | 967 / 21 | **1,444 / 532** | 1,526 / 874 |

50~80m Vehicle tracker recall은 **59.78→92.03→95.65%**다. 현재 유지가 실제 누락을
크게 메우지만, 추가 0.5~1초를 더 유지하는 이득에 비해 미대응이 많이 증가한다.
Obstacle은 현재→2배에서 TP +82에 미대응 +342다. 확인된 객체를 무조건 더 오래
남기는 후보는 채택하지 않는다.

현재 tracker의 Vehicle recall@1m/@2m은 30~50m **77.78/87.91%**, 50~80m
**65.94/92.03%**, 80~100m **8.79/30.77%**다. 2m recall이 높아도 위치·BBox의
정밀도가 충분하다는 뜻은 아니다. 고주로 50~80m 대응의 중심 오차 p50/p95는
**0.467/1.903m**다. <4m 대응만 남긴 조건부 통계이며 미대응 오차는 제외되어 있다.
GT 자체의 보간·pivot 근사도 포함한다. 속도 GT MAE나 planner 궤적 오차로 읽지 않는다.

### 미대응이 무엇인지: 중복·잔류·원본 이미지 확인

현재 profile의 2m 미대응을 모든 알려진 actor와 다시 비교했다.

- Vehicle 78 중 **41개**는 4m 내 다른 클래스 GT, **8개**는 같은 클래스 GT와 2~4m
  차이, **29개**는 4m 내 catalog actor가 없다. O02의 장애물 근처를 차량으로 분류한
  35개도 포함하므로 모든 vehicle 미대응을 고주로의 가짜 차량으로 설명하지 않는다.
- Pedestrian 13개는 근접 같은 GT 중복/위치 차이 6개와 다른 클래스 GT 근처 7개다.
  4m 내 catalog actor가 없는 관측은 이번 분모에서 0개였다. 일반화된 오검출 0 인증은 아니다.
- Obstacle 532개 중 **445개**는 같은 GT 2m 내에 있으나 이미 다른 출력이 대응한
  관측이다. 2~4m 위치 차이 58개, 다른 클래스 근처 2개, catalog actor 없는 관측 27개다.
  주된 비용은 독립적인 가짜 물체 수백 개라기보다 중복·잔류·geometry 차이다.

catalog actor가 없는 Vehicle 관측 비중은 현재 29/975=**2.97%**, 2배 후보는
72/1,090=**6.61%**다. Obstacle은 27/1,976=**1.37%**에서 86/2,400=**3.58%**로
늘었다. 이것도 진짜 hallucination 비율이 아니라 더 보수적인 spatial diagnostic이다.
Ego 직전방 X0~100/|Y|<2m의 catalog actor 없는 출력은 현재 Vehicle/Pedestrian 0,
Obstacle 1개 관측이었다. 이 좁은 corridor 밖 객체도 차선 변경·회피에 영향을 줄 수 있어
실주행 안전성으로 일반화하지 않는다.

원본 네 카메라에 GT와 track을 투영한 검토 이미지 6장을 만들고 모두 확인했다.
H03 track11, R01 track0/2의 대표 미대응은 앞선 정상 catalog 대응 이후 **GT 기록이
끝난 다음 frame부터** 시작했다. 최대 미대응 구간은 약 0.50/0.50/0.38초였다.
해당 이미지에도 차량 위치에 비어 있는 공간이 보여, 처음부터 생긴 가짜 검출보다는
actor가 기록/장면에서 빠진 뒤 남은 coasting 꼬리라는 해석을 지지한다. H03 track14도
전체 입력 이력에서 같은 양상이지만, 직전 정상 대응 프레임이 정확도 제외 구간이라 최신
scored provenance에는 선행 대응이 없다. 그 차이를 숨기지 않는다.

O02 track14는 이전 GT 대응 뒤 위치가 5m 이상 벗어난 잔류였고, track37은 4m 내 actor
없는 출력이 약1.23초 이어졌다. 양쪽 대표 장면은 후방이었다. 배경·ghost·잘못된 association을
이 이미지 하나로 확정하지 않는다. 미대응 구간이 TTL보다 길 수 있는 것은 계속 들어오는
잘못된 측정이 track을 갱신할 수 있기 때문이다. 구간 전체를 미관측 coasting으로 부르지 않는다.

### 현재 결정과 다음 우선순위

1. **tracker의 현재 0.5/1.0초 bounded coasting은 유지한다.** 원거리 누락을 메우는
   근거가 있으며 거의 즉시 삭제 후보는 recall과 재확인 비용이 나빴다.
2. **유지시간을 1/2초로 늘리지는 않는다.** 추가 recall보다 중복·미대응 증가가 크다.
   static planner의 5초 기억도 별도 기능이며, 이번 결과로 모든 잘못된 target이 안전하게
   걸러진다고 인증하지 않는다. 해당 모듈에 등록되는 사례를 풀스택에서 확인해야 한다.
3. **E24 로컬 V/P/O 0.40/0.10/0.10 유지.** 원거리 Vehicle 0.20 후보의 actual tracker는
   30~50/50~80/80~100m TP가 269/254/28→279/257/36으로 증가하지만 각 band의
   미대응도 3/47/6→8/58/8로 늘었다. 원거리 이득은 있지만 추가 유지와 혼동하지 않으며
   이번에 보수적인 차량 초기값을 자동 변경하지 않는다.
4. 다음 인지 보완은 **장애물 중복 birth·coasting 잔류·잘못된 association의 실제 구간
   조사**가 우선이다. 단순히 threshold를 올리거나 기억을 늘려 해결하지 않는다. 실제
   팀장님 4090 24GB의 별도 trial에서 detector→tracker→planner와 궤적/조향·영상도
   확인한다. E25 production·데이터 생성·학습·planner 코드는 이번 작업에서 유지했다.

### 산출물과 검사

- fresh GT·원본 token·scoring mask: `artifacts/evaluation/bevfusion/base5_far_bag_retention_261012/inputs_v1`.
- 최신 report: **`report_validated_v6.json`**. GT geometry guard 추가 전 `report_v5.json`과
  통계가 같으며 v1~v4는 진단·provenance 보강 전 이력이다.
- actual ROS: `ros_near_zero_coast_v1`, `ros_double_coast_v1`의 messages/receipt/analysis.
  0.20 차량 후보는 40절의 `ros_selected_candidate`를 동일 prediction SHA로 연결했다.
- 검토 이미지: `review_v1/00~05_*.jpg`, 설명·run·좌표는 `index.json`에 보존했다.
- CPU 도구: `prepare_base5_far_bag_gt.py`, `evaluate_base5_far_retention.py`,
  `render_base5_far_review.py`. 기존 replay는 threshold와 lifecycle을 동시에 바꾸는
  비교를 거부하며 독립 반복을 유지하도록 확장했다. production 소스는 수정하지 않았다.

관련 CPU·launch 계약 **64개 통과**. 예측 불변 1,570행, 이전 근접 GT 기하 보존,
두 실제 실행의 모든 input/output stamp·독립 반복 delta0/IoU1을 확인했다. Python
구문·언어·whitespace와 owned process 종료도 확인했다. 전체 tools discover는 system
Python에 Torch/mmengine이 없어 학습 등 11개 모듈의 import가 실패했다. 별도 CPU 관련
검사는 통과했지만 전체 GPU·학습 suite를 통과한 것으로 기록하지 않는다.

<a id="base5-stable-whole-route-view-261012"></a>

## 42. 2026-10-12: 안정적인 이전 완주 입력의 E9/E25 화면 비교

### 입력 선택과 한-PC 주행 문제의 분리

사용자는 최근 한-PC full-stack 주행의 Ego 주기가 약 20 Hz여서 제어가 이상했다고
설명했고, 이전의 안정적인 완주 기록을 수동주행까지 포함해 비교하도록 요청했다.
10-12 17:15 E25 충돌 종료 기록을 완주로 대체하거나 최근 완주라고 부르지 않는다.
그 기록의 controller 시작~충돌 구간에서는 Ego 평균 24.03 Hz, 수신 간격 p95
106.12 ms인 반면 control state·steering·`/ctrl_cmd`는 50 Hz였다. 이동 구간의
위치 변화 약 1,201 m와 wall-time 속도 적분 약 1,643 m도 일치하지 않았다.
과거 localization의 비실시간율/receipt-time 문제와 일치하는 단서이며, 이번 측정만으로
원인을 단일 확정하거나 production 제어·속도 bias를 수정하지 않는다.

이전 두 완주 후보의 원본 Ego와 센서·Object 메시지를 직접 읽었다.

| 이전 원본 | 경로 길이 | Ego 평균 주기 | 간격 p95 / 최대 | 100 ms 초과 공백 | 속도 적분 / 위치 거리 |
|---|---:|---:|---:|---:|---:|
| `raw_sensor_w_cam/sensor_cam_raw_full_slow.bag` | 2,192.99 m | 45.78 Hz | 48.12 / 126.79 ms | 4 | 1.00284 |
| `raw_sensor/competition_route_full_highspeed.bag` | 2,249.38 m | 48.51 Hz | 38.39 / 63.51 ms | 0 | 1.00034 |

속도 비율은 1 m/s 초과 이동, 250 ms 미만 간격, 5 m 미만 위치 변화의 동일 구간에서
계산했다. 예전 slow bag에는 명시적 km/h→m/s 변환 1/3.6을 적용했으며, highspeed는
m/s 그대로다. 원본 메시지는 변경하지 않았다. 이 비율 자체는 독립적인 simulator
real-time factor 측정이 아니다. 파일명·보존 날짜가 아닌 bag의 원본 stamp로 보면 slow는
2026-07-25 12:21 KST의 기록이다. 최근 local odom 변경 이후의 폐루프 기록과 혼동하지 않는다.

highspeed의 Object 메시지 7,551개는 전부 빈 목록이다. 이것만으로 map 배경을 포함한
모든 실제 객체의 부재나 완전한 negative GT를 확정하지 않는다. 객체 장면 확인에 더
적합한 slow를 주 비교 입력으로 선택했다. slow의 Object 12,881개 중 12,024개가
비어 있지 않았고, 한 메시지의 최대 객체 수는 17개였다.

### 같은 센서 프레임과 실행 계약

- slow의 동기화 가능한 고유 LiDAR **3,369장 전체**를 사용한다. 예산에 따른 표본
  축소는 하지 않았다. 네 카메라의 가까운 JPEG를 ±50 ms에서 선택하며 원본 bytes와
  XYZI를 보존했다. 카메라 절대 시간차 p50/p95/최대는 25.75/31.36/48.34 ms다.
- `inputs_v1`의 모든 JPEG·point SHA256, token·timestamp 순서, Sensor_4 해시·카메라
  순서·투영 행렬·native 해상도를 재확인했다. info SHA256은
  `fad6d9fc29a3c2509e60bc027ce55b825f4a335e2734a1d5b41d60daa27b66a7`이다.
- 전·후방 1280×720, 측방 640×480의 역사적인 Sensor_4 입력이다. 기존
  `MoraiImageAug3D`가 padding을 제거하고 각 카메라의 원래 크기에 맞게 resize 및
  homography를 계산한다. bag에 tf_static이 없어 기록 당시 mounting을 독립적으로
  인증한 것은 아니다. 현재 대회 센서·ROS/LAN의 실시간 검증과 분리한다.
- Base5 best는 현재 native validation 기준 **E9**다. E24는 마지막 epoch이며
  모든 미션에서 E9가 우월하다는 뜻은 아니다. E9는 학습된 512×1056·5/20 cm·
  depth 25 cm·X±100/Y±30 m 구조와 강도 ÷255를 유지한다.
- 로컬 RTX 5060 Ti에서 추론했다. E9: AMP·camera 병렬화 ON, inference compile OFF.
  E25: 원래 config·원래 강도,
  이 오프라인 기준 실행은 FP32다. 두 모델의 실행 정밀도가 달라 순수 수치 통제나
  공정한 latency benchmark로 해석하지 않는다. checkpoint는 strict load한다.
- 두 모델의 관측 threshold를 **V/P/O 0.40/0.15/0.10**, rotated BEV NMS IoU 0.50으로
  맞춘다. E24 별도 시험값 0.40/0.10/0.10과 production 기본값은 변경하지 않는다.
- 비교 화면은 공통 **LiDAR-frame 중심 XY ±30 m, tensor bottom-Z −3~2 m 범위**다. radial 30 m와
  다른 계약이다. E9의 전체 범위는 별도 기본 OFF 표시로 분리한다.
- 정확도 정답은 만들지 않았다. `instances=[]`는 정답을 모르는 시각화용 입력 표시이며
  장면에 객체가 없다는 GT가 아니다. 이 package로 AP·recall·FP 비율을 계산하지 않는다.

### 실행 상태와 재현 경로

산출물은 `artifacts/evaluation/bevfusion/base5_best_vs_e25_full_route_261012/`에
분리한다. `bag_timing_audit.json`, `inputs_v1`, `e9_full_amp_parallel_v1`,
`e25_full_fp32_v1`의 원본 입력 해시·checkpoint 해시·정밀도 receipt를 유지한다.
E9/E25 모두 전체 3,369장 추론을 완료했다. token·info SHA256·빈 GT identity는
두 실행에서 같다. 공통 범위 marker bag, 네 카메라 원본 투영 12장과 private-master
RViz를 확인했다. 검토 이미지는 시간 균등 6장과 최소 8초 간격의 출력 개수 차이가
큰 6장으로 구성하므로, 무작위 정확도 표본으로 해석하지 않는다.

| 관측 범위·모델 | Vehicle 출력 관측 | Pedestrian 출력 관측 | Obstacle 출력 관측 | 출력이 있는 프레임 |
|---|---:|---:|---:|---:|
| 공통 범위 E25 | 811 | 88 | 58 | 696 |
| 공통 범위 E9 | 888 | 281 | 205 | 994 |
| E9 학습 전체 범위 | 2,250 | 388 | 305 | 1,452 |

출력 관측 수는 frame마다 반환된 박스를 더한 값이며 고유 객체 수나 TP가 아니다.
E9의 추가 검출을 곧바로 성능 향상으로 해석하지 않는다. 대표 투영에서는 두 모델이
같은 차량·전방 장애물에 대응하는 사례와 E9만 사람·작은 장애물 주변에 반응하는
사례를 확인했다. 특히 **`full_route_01451`** 등의 추가 박스는 annotation 없는
건물·도로 배경과의 구분도 검토해야 한다. 이번 작업은
그 구간을 보존하며 완전한 FP 판정을 하지 않는다.

최신 재생 세션은 **`view_v2`**, 0.75배속 반복이다. 공통 범위 E25는 초록색, E9는
하늘색이며 E9 전체 범위는 주황색의 기본 OFF display다. `/clock`·두 marker stream·
LiDAR와 네 JPEG subscriber를 private master에서 확인했다. Sensor_4의 base→LiDAR
정적 TF만 private master에 발행한다. 재생 중에는 detector GPU 추론·planning·controller·live Ego
재생은 없으며 전역 ROS master와 MORAI는 변경하지 않았다. RViz를 닫으면 해당
도구가 자신이 시작한 playback·TF·master만 정리한다. ROS1 EOL popup은 그 세션의
환경변수만으로 숨겼다. compressed transport의 중복 dynamic-reconfigure service와
기존 inotify watch 부족 경고는 남지만 네 영상·LiDAR 렌더링은 확인했다. 시스템
설정이나 다른 프로세스를 수정해 경고를 없애지는 않았다.

재생을 다시 시작할 때는 이미 있는 세션 경로를 덮어쓰지 않고 새 이름을 사용한다.

```bash
cd "$HOME/AjouNice2026"
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
OUT="$PWD/artifacts/evaluation/bevfusion/base5_best_vs_e25_full_route_261012"
/usr/bin/python3 perception/detection3d/bevfusion/tools/view_base5_full_route_comparison.py view \
  --bag "$PWD/rosbags/raw_sensor_w_cam/sensor_cam_raw_full_slow.bag" \
  --markers "$OUT/comparison_markers_v1.bag" \
  --output "$OUT/view_$(date +%Y%m%d_%H%M%S)" \
  --rate 0.75
```

다음은 사용자가 화면을 검토한 뒤 차이가 큰 구간의 실제 객체·오검출·tracker 전달을
분리해 평가하는 것이다. 이 수동 기록은 최근 Smooth controller 폐루프, 실제
4090 24GB의 ROS/LAN latency 또는 현장 주행 안전성을 인증하지 않는다.

도구는 `view_base5_full_route_comparison.py`의 `prepare / verify / audit / export /
review / view`다. 준비·재생 결과는 fresh 경로만 허용하고 raw를 덮어쓰지 않는다.
8장 correctness pilot에서 기존 timing helper가 10장 warmup을 모두 제외해 NaN을
쓰려던 문제를 발견해 소량 표본의 median fallback을 보완했다. 모델 추론 오류나
학습 변경은 아니며, 불완전 pilot 출력은 성공 근거로 사용하지 않는다. 관련 CPU 검사
18개와 Python 구문 검사는 통과했다.

<a id="base5-e9-e24-selection-261013"></a>

## 43. E9 threshold 선정과 E9/E24 최종 시험 선택 — 2026-10-13

사용자는 E9도 threshold를 정한 뒤 E24와 최종 비교하도록 요청했다.
**다음 Base5 실주행 시험 모델은 E24로 선택한다.** Native val best E9를 폐기하거나
E24가 모든 객체·거리에서 우월하다고 선언하는 결정은 아니다. Production E25의
canonical 실행 경로는 유지하며 실제 대회 PC·풀스택 검증 후 교체를 별도로 판단한다.

### 실행·평가 계약

- 로컬 5060 Ti에서 **E9/E24 각각 같은 1,570장**을 새로 추론했다. 같은 config·입력
  token·GT·강도 ÷255·strict state load·AMP·camera 병렬화 ON·compile/TF32 OFF다.
  모델의 512×1056·5/20cm·depth 0.25m·X±100/Y±30m 설정과 가중치는 바꾸지 않았다.
- 옛 E9 bag 출력은 FP32였고 옛 E24 출력의 config 파일 SHA가 현재 wrapper와 달랐다.
  비교 guard를 풀지 않고 **E24도 동일 실행기로 재추론**했다. 예전 FP32와 이번 AMP
  결과를 같은 조건의 model A/B로 섞지 않는다. 설치 경로·상대 ann 경로·native 전용
  dataset loader로 실패한 준비 시도는 정확도 근거에서 제외했다. bag loader 이름의
  `MoraiBase3Dataset`은 기존 bag metadata 형식 adapter이며 Base3 학습 데이터를 쓴 것이 아니다.
- 41절에서 감사한 far GT·scoring mask를 그대로 재사용했다. 두 예측의 box·score·label은
  GT 연결 과정에서 불변이고 GT identity는 같다. **1,413장만 정확도에 사용**한다.
  미확정 차량 때문에 제외한 157장도 tracker에는 넣어 전체 1,570장의 이력을 유지한다.
- 새 AMP 출력에서 각 클래스 score **0.10~0.80의 71개 후보**를 near/full ROI에 전수
  계산했다. 기존 native val 745장의 AMP 병렬화 sweep·AP도 함께 사용했다. Native val과
  bag의 관측을 하나의 분모에 섞지 않는다. 0.10 미만은 현재 server floor 때문에 평가하지 않는다.
- 실제 TF·C++ tracker·UNKNOWN policy를 private ROS master에서 재생했다. 40절과 같은
  동결 tracker binary SHA `7dda240b5e3331ea2b3322eb6e2403c2e8e1616d8dd665b95d31c7baf80ca3d1`
  및 source/config를 사용한다. 현재 devel binary가 다르므로 이름만 보고 사용하지 않았다.
  각 후보는 reference/candidate/reference 독립 반복이며 raw/policy 각각 1,570 stamp가
  빠짐없이 대응하고 reference 반복은 정확히 같다. Coasting 0.5/1.0초는 유지한다.
- Score floor 0.10·top100·class-aware rotated BEV NMS 0.50을 유지한다. AP는 score 곡선,
  고정 threshold 대응은 중심 거리 gate다. 표의 미대응은 중복·위치 오차·잔류·배경을
  포함하며 모두 의미상 오검출이라고 부르지 않는다. GT는 status 보간·catalog 근사다.
  아래 native 고정 threshold 수치는 별도 표기가 없으면 중심 1m 대응이며 tracker 표의
  2m 대응과 구분한다.

### 모델별 threshold 결정

| 모델 | Vehicle | Pedestrian | Obstacle | 역할 |
|---|---:|---:|---:|---|
| E9 | **0.30** | **0.20** | **0.12** | 별도 선정한 균형 비교 후보 |
| E24 | **0.40** | **0.10** | **0.10** | 최종 Base5 실주행 시험 선택 |

E9의 선정 근거는 다음과 같다.

- Vehicle 0.40→0.30: tracker의 전체 ROI 2m 대응 **828→876**, 미대응 **93→104**다.
  50~80m 대응은 224→250이며 1m 기준 1초 이상 누락 episode는 8→5다. Native 근접 1m 대응은
  163→169, FP 2→3이다. 0.20 조합은 대응 901까지 늘지만 미대응 127 및 0.5초 이상
  2m 미대응 episode 7개로 커져 균형값 0.30을 선택한다.
- Pedestrian 0.10은 bag raw 2m 대응을 0.15 대비 165→174로 늘리지만 미대응도
  30→131로 늘린다. 0.20은 native 근접 대응 454→452에 FP 28→16이고, P01 근접
  tracker 대응 113/113을 유지한다. 원거리 recall 일부 손실은 인정하며 E24와 비교한다.
- Obstacle 0.10→0.12: Vehicle 0.40을 고정한 tracker 비교에서 근접 대응 **875/897로
  동일**하고 전체 미대응은 555→348로 줄었다. Native 근접 TP 451→438·FP 71→50이며
  ≥5점 지원 GT도 9건 잃는다. 단순 무손실 개선이 아니다. 0.15는 native 근접 TP 412까지
  줄어 더 올리지 않는다. E9 0.12와 E24 0.10의 native 근접 precision은 89.75/89.88%로
  비슷해진다. 두 모델에 같은 숫자를 강제하는 대신 실제 operating point를 비교했다.

실제 tracker는 클래스가 다른 관측끼리도 association에 영향을 줄 수 있다. 예를 들어
E9 Vehicle만 0.40→0.30으로 바꾼 joint profile에서 Obstacle 전체 2m 대응도 1443→1436으로
변했다. Detector의 독립 클래스 sweep을 tracker에서도 완전히 독립인 것으로 해석하지 않는다.

### 최종 후보 비교

아래 tracker 통계는 각 모델의 선정값이며 동일한 scoring 분모를 사용한다.
프레임 반복 관측 수와 그 recall이지, 독립 객체 수나 현장 검출 확률이 아니다.

| 지표 | E9 0.30/0.20/0.12 | E24 0.40/0.10/0.10 | 해석 |
|---|---:|---:|---|
| Native val 745장 full ROI mAP | **79.40%** | 77.37% | E9의 val best 지위는 유지 |
| Bag 0~30m Obstacle AP | 79.20% | **81.34%** | bag의 near 장애물 ranking은 E24 우세 |
| Tracker 0~30m Obstacle 대응@2m | 875/897, 97.55% | **887/897, 98.89%** | 미션 회피 근접 recall은 E24 우세 |
| Tracker 30~50m Pedestrian 대응@2m | 51/109, 46.79% | **100/109, 91.74%** | 현재 Man1 횡단 bag에서 큰 차이 |
| Tracker 50~80m Vehicle 대응@2m | 250/276, 90.58% | **254/276, 92.03%** | E24가 조금 더 유지 |
| Tracker 80~100m Vehicle 대응@2m | 12/91, 13.19% | **28/91, 30.77%** | 양쪽 모두 부족하나 E24가 더 유지 |
| Tracker 전체 Vehicle 대응/미대응@2m | 876/104 | **897/78** | E24가 대응도 많고 미대응도 적음 |
| Tracker 전체 Pedestrian 대응/미대응@2m | 176/49 | **233/13** | E24의 연속 전달이 더 유리 |
| Tracker 전체 Obstacle 대응/미대응@2m | 1436/**357** | **1443**/533 | E9가 중복성 미대응 억제에 유리 |
| Obstacle 4m 내 catalog actor 없는 관측 | 44 | **27** | 이 진단만으로 실제 hallucination 확정 불가 |

E9가 나은 부분을 숨기지 않는다. 50~80m Obstacle AP는 **17.83 vs 10.81%**이고,
근접 Obstacle의 tracker 조건부 위치 오차 p95는 **0.737 vs 0.949m**로 E9가 작다.
전체 matched TP의 native Obstacle mASE는 반대로 E24 **0.238 vs E9 0.289**다.
각 지표의 대응 집합·조건이 다르며 어느 하나로 완벽한 회피 geometry를 인증하지 않는다.

### 위치 오차와 잔류 확인

아래는 **중심 4m gate 안에서 대응된 관측만**의 GT 오차다. 누락·미대응을 제외했고
GT의 pivot·보간 오차도 포함한다. 크기·속도·planner 오차나 두 모델 사이의 수치 drift가 아니다.

| 구간·클래스 | 모델 / 표본 | 평균 / p50 / p90 / p95 / 최대 (m) |
|---|---|---|
| O02 0~30m Obstacle | E9 / 875 | 0.267 / 0.156 / 0.607 / 0.737 / 2.582 |
| 같은 구간 | E24 / 887 | 0.302 / 0.154 / 0.678 / 0.949 / 3.705 |
| H03 50~80m Vehicle | E9 / 124 | 0.658 / 0.408 / 1.550 / 1.787 / 2.112 |
| 같은 구간 | E24 / 123 | 0.807 / 0.467 / 1.769 / 1.903 / 3.772 |

E24의 recall 이득을 무조건 더 정밀한 위치라는 뜻으로 읽지 않는다. 실제 회피·추월
거리 margin은 팀장님 PC의 tracker→planner 폐루프에서 확인한다.
원본 네 카메라 overlay 38장을 생성했고 P01·O02의 대표 4장을 직접 확인했다.
두 모델 모두 실제 actor 주변의 중복/geometry 차이가 미대응으로 잡히는 사례가 있었다.
선택 이미지는 duration 순의 진단이며 무작위 표본·동일 frame의 정량 비교가 아니다.
E24에도 장애물 중복·잔류가 남아 있고 좁은 전방 corridor의 catalog 미지원 관측 1건도
있으므로 오검출 없는 모델 또는 안전성 인증으로 선언하지 않는다.

### E24 Obstacle 0.12 재검토와 최종 결론

E24도 0.12 후보를 같은 조건으로 다시 재생했다. 근접 대응 887/897은 유지됐으나 전체
Obstacle 대응 **1443→1403**, 미대응 **533→442**, catalog actor 없는 관측 **27→36**이다.
Native 근접 대응도 435→427로 줄고 이 중 6건은 ≥5점 지원이 있었다. 미대응 총량 감소가
원인이 확인된 ghost 감소와 같지 않으므로 이번에는 기존 **E24 O0.10을 유지**한다.
더 높은 threshold로 중복 문제를 모두 해결하려 하지 않는다.

정리하면 E9는 더 넓은 native asset 분포의 val ranking과 일부 geometry·중복 억제에
강점이 있다. **현재 연속 미션에서 near 장애물, 30~50m 횡단 보행자, 고주로/로터리 차량을
함께 전달할 단일 모델은 E24가 더 균형 잡혔다.** E24를 선택하고 E9는 비교·후속 학습 판단용으로
보존한다. Class별 두 모델 ensemble이나 학습 변경은 수행하지 않는다.
근거는 six-asset O02·Man1 P01·H03·R01 dev bag과 Sunny13 val이며 모든 122종·Foggy·
역방향 미션에 대한 독립 test가 아니다. 실주행에서 결론이 뒤집힐 가능성은 남긴다.

### 적용·산출물

- E9 opt-in: `base5_e9_calibrated_trial.launch`, V/P/O 0.30/0.20/0.12.
- 최종 Base5 시험: 기존 `base5_e24_calibrated_trial.launch`, V/P/O 0.40/0.10/0.10.
  각각 같은 모델의 AMP·병렬화 서버를 별도 준비한다. Launch는 가중치 선택·hash 검증을
  자동 수행하지 않는다. E25 production·baseline trial·수집·학습·planner 설정은 불변이다.
- 산출물 root: `artifacts/evaluation/bevfusion/base5_e9_e24_selection_261013`.
  `E9_amp_parallel_v4`, `E24_amp_parallel_v1`, `frozen_gt_v2/receipt.json`에 두 실행 계약을
  보존했다. E9의 ROS 재생은 그 앞서 만든 `e9_frozen_gt_v1`의 정확한 gzip SHA를 사용한다.
  같은 detector 배열이라도 gzip metadata가 다르면 hash guard를 임의로 우회하지 않는다.
- 최종 집계 `summary_v2.json`: SHA256
  `49ce48191037aa223fc8de67e5f56c3e2f057c0cd8a35eab780dd2e1f11485bf`.
  상세는 `e9_tracking_v2.json`, `e24_tracking_v2.json`의 case·range·miss·persistence를 따른다.
- AMP fine sweep `fine_curves_v1.json`: SHA256
  `6c3328f731c8ce78309c9e82534753c7c77776f9a5f576b33be17dc0b782122f`.
  Native sweep은 40절의 `base5_threshold_calibration_261012_validated_v3/report.json`,
  AP/geometry는 36절의 `E9_amp_val745_cap20_v1/accuracy.json`과 E24 대응본이다.
- `ros_e9_precision_v2`, `ros_e9_balanced_v1`, `ros_e9_recall_v1`, `ros_e24_adopted_v1`,
  `ros_e24_precision_v1`에 messages·receipt와 독립 reference 반복을 보존했다.
  `review_e9_*`, `review_e24_*`는 원본 이미지의 actor/track overlay다.
- 도구 `compare_base5_checkpoint_selection.py`의 attach/curves/summary는 fresh 산출물만
  만들고 sensor·GT 원본을 보존한다. Far evaluator에는 명시적 score policy, renderer에는
  명시적 replay/mode/diagnostic 선택을 추가했고 기존 E24 명령의 기본 동작은 유지했다.

다음은 선택한 E24로 실제 4090 24GB에서 ROS/LAN·queue·tracker→planner·주행 영상을
확인하는 것이다. 이번 로컬 저장 검출 재생은 GPU부터 planner까지의 풀스택 인증이 아니다.

관련 CPU·ROS 입력/launch 계약·context 검사 **52개 통과**. 두 opt-in launch를 실제
roslaunch parser로 해석했고 새 Python 구문·문서 링크·whitespace 검사도 통과했다.
5개 private ROS 재생의 source/binary/coasting/GT 계약과 독립 반복을 검사했으며,
이번 작업이 만든 GPU container·ROS master·tracker·policy 프로세스는 모두 종료했다.

<a id="base5-on-base3-val-261013"></a>

## 44. 원본 Base3 val 교차 평가와 자차 회신점 대조 — 2026-10-13

### 목적과 동결 조건

사용자가 제안한 E9/E24의 Base3 val 평가를 수행했다. 새로운 학습·수집·production 변경은
하지 않았다. 원래 Base3 portable val **1,635장, 545개 장면**에 대해 E9/E24를 로컬
5060Ti에서 각각 전체 추론했다. E25는 같은 infos·checkpoint의 기존 전체 추론을 재사용했다.
같은 장면의 세 캡처가 있으므로 1,635개의 독립 배치 또는 현장 검출 확률로 읽지 않는다.

- Infos: `artifacts/datasets/morai_base3_bevfusion_portable_v1/morai_base3_infos_val.pkl`.
  SHA256 `ff4f97c02565784ccb21ada41c3d7360e04c214b087176849890daebc1358470`.
- 세 모델 GT fingerprint:
  `2289a908689e177ad40c3656c56971d173ff89d89a259fb9285b2ecfedd2d0c1`.
  원본 native annotation과 infos의 retained GT **4,415개**를 위치·크기·yaw·클래스로 대조했다.
- 원본 PNG 네 카메라·XYZ·GT를 유지했다. PNG를 JPEG로 조용히 바꾸거나 기존 Base3의
  최소 5점 GT 정책을 바꾸지 않았다. Base5와 이미지 압축·가시성 정책이 다른 점은 남는다.
- 두 모델 모두 FP32, AMP·TF32·compile·camera 병렬화 OFF다. 모델별 학습된 구조·crop은
  그대로 사용했다. E25의 intensity 0–255와 Base5의 고정 ÷255 계약을 각각 유지했다.
- 공통 의미 범위는 중심 XY±30m, bottom Z −3~2m이며 주 표는 **중심 반경 30m 미만**이다.
  사각형 ±30m 결과도 별도 저장했다. 원래 decoder center AP와 ROS floor 0.1·top100·
  class-aware rotated NMS 0.5 이후 AP를 구분했다. 표의 AP는 center 기준이지 IoU AP가 아니다.
- 세 checkpoint SHA는 CURRENT 및 33절의 동결 E25/E9/E24와 같다. Base5를 E25에서
  이어 학습한 것은 아니므로 단순히 ‘catastrophic forgetting’으로 명명하지 않는다.

### 같은 원본 Base3 val에서의 결과

아래는 원래 decoder 출력의 center AP다. 과거 E25와 비교할 때 후처리 AP를 섞지 않는다.

| 모델 | Vehicle | Pedestrian | Obstacle | mAP |
|---|---:|---:|---:|---:|
| E25 | 97.47% | 98.80% | **97.95%** | **98.07%** |
| E9 | **98.56%** | 98.16% | 87.55% | 94.76% |
| E24 | 96.45% | 96.74% | 87.97% | 93.72% |

사각형 ±30m의 Obstacle AP도 E25/E9/E24 **97.97/87.59/88.05%**로 같은 방향이다.
ROS 후처리 뒤 반경 30m Obstacle AP는 **95.09/85.35/85.57%**다.
단일 threshold와 AP ranking을 혼동하지 않으며 특히 아래 자차 반응을 별도 확인한다.

| Obstacle 거리 | E25 | E9 | E24 |
|---|---:|---:|---:|
| 0~10m | **97.27%** | 88.91% | 88.87% |
| 10~20m | **97.57%** | 88.27% | 88.38% |
| 20~30m | **93.66%** | 67.87% | 67.59% |

33절의 Base5 val과 함께 보면 다음과 같다. 행 안에서는 동일 GT지만 서로 다른 행은
같은 난이도나 같은 GT가 아니다. Base3 평가를 새 threshold 선정용 독립 test로 주장하지 않는다.

| 평가 분포의 Obstacle AP | E25 | E9 | E24 |
|---|---:|---:|---:|
| 원본 Base3 val | **97.95%** | 87.55% | 87.97% |
| 현재 Base5 val, 동일 공통 30m | 60.76% | 81.95% | **83.49%** |

따라서 ‘Base5 val이 더 어려워서 수치만 낮다’로 전부 설명할 수 없다. 구 분포에서도
장애물 격차가 실제로 남으며, 반대로 Base5 분포에서는 새 모델이 E25보다 낫다.
초기화·공간/depth·학습량·데이터·입력 분포가 함께 달라진 비교라 어느 하나의 원인으로
확정하지 않는다. 이 평가만으로 5cm/20cm나 depth 25cm를 되돌리거나 재학습하지 않는다.

### 과거 초기 성능과 학습 노출

기존 Fresh E40의 `bf-sw-fresh-e40-rtx5060ti-260918-005536` 기록도 확인했다.
`20260917_155542/vis_data/scalars.json`의 첫 두 epoch Obstacle AP는 **93.91%, 96.35%**다.
사용자의 ‘초기부터 약 95%’ 기억은 근거가 있다. 단, 당시 학습/평가 계약의 숫자이며
현재 보완된 평가와 동일한 조건의 초기화 ablation으로 사용하지 않는다.

원본 train infos와 scenario metadata를 실제로 대조한 장애물 학습 GT 관측 수는 아래와 같다.
배치 객체 수, 122종 전체 asset 수, 독립 객체 수, optimizer가 본 유효 gradient 횟수와 다르다.

| 항목 | Base3 train | 이번 Base5 train |
|---|---:|---:|
| 프레임 | 7,632 | 4,255 |
| Obstacle GT 관측 | **10,696** | **2,615** |
| 해당 클래스 asset 종류 | 10 | 22 |
| CargoBox GT 관측 | 1,523 | 152 |
| Concrete_Barricade01 GT 관측 | 1,012 | 154 |
| Steel_Barricade GT 관측 | 880 | 166 |

Base3의 반복 캡처와 학습 epoch 수도 달라 관측 수 배율을 성능의 인과 비율로 읽지 않는다.
다만 기존 열 종류에 대한 학습 노출은 상당히 줄었고 분포는 넓어졌다. 후속 환경의 유효
GT/asset coverage와 부족 asset 보강을 확인할 가치가 있다. 필요하면 **Base3 train만**
별도 replay 후보로 검토하며 Base3 val/test를 학습에 섞지 않는다. 아직 보강·재학습은 하지 않았다.

### 희소 GT 제외와 실제 장애물 누락을 분리

원래 Base3의 최소 5점 조건 때문에 반경 30m native annotation 중 Obstacle 217,
Pedestrian 33, Vehicle 1개 관측이 학습용 GT에 없었다. 실제 원본 point 수를 다시 계산해
모두 5점 미만임을 확인했다. 위치 근접은 진단 근거일 뿐 새 TP·가시성 인증으로 바꾸지 않는다.

같은 V/P/O 0.40/0.15/0.10, 중심 1m 대응에서 Obstacle FP는 E25/E9/E24 **36/248/225**다.
E24 225개 중 92개는 제외된 희소 native 객체의 1m 안, 114개는 이미 retained GT의 1m 안,
5개는 이번 annotation 근접 분류로 설명되지 않았다. 나머지는 1~2m나 다른 클래스 근접이다.
이를 225개의 실제 독립 유령 장애물이라고 해석하지 않는다. 그렇다고 모두 정답 처리하지도 않는다.

실제 retained GT의 누락도 남는다. CargoBox 314개 중 TP는 E25/E9/E24 **302/270/270**,
Concrete_Barricade01 201개 중 **198/161/162**다. 희소 GT 정책 차이만으로 장애물 격차를
전부 설명할 수 없으며 위치 대응·중복·score·형상 오차를 추가로 분리해야 한다.

### 구 입력의 자차 회신점과 E24 자차 위치 검출

E24는 구 Base3 입력에서 자차 위치에 score 약 0.67~0.77의 Vehicle 박스를 반복 반환했다.
공통 V0.40 이후 Vehicle TP/FP는 **1,081/1,536**, precision 41.31%다. 높은 Vehicle AP가
해당 threshold의 정상 동작을 보증하지 않는다. E9도 V0.30으로 낮추면 이 반응이 늘어난다.

유효·nonzero XYZ만 세어 LiDAR 근처 부피 X[−1.2,1.0], Y[−0.75,0.75], Z[−0.35,0.10]m를
확인했다. 이는 측정한 진단 부피이며 일반적인 안전 자차 제거 영역이 아니다.

| 입력 집합 | 프레임 | 이 부피에 회신점이 있는 프레임 | point 수 중앙값 |
|---|---:|---:|---:|
| 원본 Base3 val | 1,635 | 1,635 | 10,523 |
| 현재 Base5 val | 745 | 0 | 0 |
| 현재 네 개발 bag 입력 | 1,570 | 0 | 0 |

Base5의 NoReturn zero placeholder를 자차 회신점으로 센 결과가 아니다. 현재 Base5 train·val에는
근원점 Vehicle GT도 없었다. 어느 simulator 변경이 이 차이를 만들었는지는 확정하지 않았다.
센서 설정 이름만으로 mounting/extrinsic 오류라고 주장하지 않는다.

점의 역할을 분리하려고 dominant GT 클래스 및 empty cohort별로 8장씩 **균등 선택한 32장**에
E24 FP32를 다시 실행했다. score가 높은 사례만 고르지 않았다. 해당 부피의 점만 파생 사본에서
제외하고 모든 원래 GT의 point 수가 전후 동일함을 검사했다. PNG·GT·calibration·가중치는 그대로다.

| 32장 입력 대조 | 원본 | 진단 부피의 점만 제외 |
|---|---:|---:|
| 자차 위치 Vehicle 관측, score≥0.40·중심<2m | **30** | **0** |
| Vehicle raw AP | 96.20% | 98.98% |
| Pedestrian raw AP | 93.75% | 93.75% |
| Obstacle raw AP | 91.72% | 91.04% |

자차 위치 검출의 입력 원인은 대조로 지지되지만 **장애물 격차 전체의 원인은 아니다**.
32장 GT를 훼손하지 않았다는 사실은 다른 pose의 실제 근접 객체까지 안전하게 제거할 수 있다는
뜻이 아니다. production·수집기·ROS에 이 mask를 적용하지 않았다. 실시간 실제 입력에서
자차 회신점과 출력 존재 여부를 반드시 확인하며 기존 threshold를 구 입력에 무조건 이식하지 않는다.

### 산출물·검증·현재 선택

- Root: `artifacts/evaluation/bevfusion/base5_on_base3_val_261013`.
  `E9_fp32_v1`, `E24_fp32_v1`에 전체 predictions·receipt를 보존했다.
- 전체 집계: `cross_validation_v2.json`, SHA256
  `33e68d69e41628a34c3c9bde0038272ea8f5ae8e4d387a1289b043d0a8248c60`.
  구 v1의 주 평가·threshold 통계와 완전히 같으며 population·probe 근거만 추가했다.
- 자차 대조: `roof_probe32_v1`의 원본 subset·masked infos·파생 points·manifest,
  `E24_roof_probe_fp32_v1`의 predictions·receipt. 모든 원본 point 파일은 유지했다.
- 재현 도구: `summarize_base3_cross_validation.py`, `prepare_base3_self_return_probe.py`.
  동결 hash·GT·강도·FP32·같은 프레임·원본 GT support 보존을 검사하며 새 output만 생성한다.
  산출물과 raw는 git에 강제로 넣지 않는다. GPU 정확도 시험은 latency/4090 성능 인증이 아니다.

현재 Base5·Sensor4 개발 입력을 기준으로 선정했던 E24 trial은 유지하되, 자차 출력 검사와
구 분포 장애물 보강 필요성을 후속 gate에 추가한다. E9 val best도 유지하고 E25 production은
변경하지 않았다. 이번 결과만으로 model ensemble·threshold 변경·재학습을 수행하지 않는다.

교차 평가·기존 공통 범위·context 단위 검사 **29개**를 통과했다. 32장 파생 cloud가
현재 유효·nonzero mask로 원본에서 만든 배열과 정확히 같음을 다시 확인했다.
문서 링크·기존 anchor·legacy 원문 SHA·Python 구문·whitespace 검사도 통과했다.
이 작업의 추론 container는 모두 종료했고 다른 agent의 planning·MORAI 세션은 변경하지 않았다.

<a id="base5-e24-production-261013"></a>

## 45. 사용자 요청에 따른 E24 production 승격 — 2026-10-13

E24를 production 기본값으로 승격하고 E25는 명시적 복귀로 보존했다.
실행·weights 준비·팀원 증분 빌드는 [E24 handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)를 따른다.
판단팀 브랜치 머지, 로컬 무거운 풀스택·MORAI 실행, 실제 대회 PC 검증은 수행하지 않았다.

- `bevfusion`은 profile e24의 512×1056·5cm/20cm·depth 0.25m 모델과 원본 JPEG·ros255,
  AMP·camera 병렬화 ON/compile OFF, 0.40/0.10/0.10, canonical topic을 함께 선택한다.
- E25 `base3_online.launch`·config·weights는 유지하고 `bevfusion --model e25`로 복귀한다.
  기존 opt-in trial의 namespace·port·운영값은 유지한다.
- `.env.bevfusion.local`의 옛 `BEVFUSION_INFERENCE_CHECKPOINT`는 E25 호환값으로만 사용한다.
  명시적 `BEVFUSION_CHECKPOINT`는 선택 모델의 SHA가 맞아야 한다. 원래 .env는 수정하지 않았다.
- 준비기는 기존 정상 weights를 다시 받지 않으며 다른 SHA 파일을 덮어쓰지 않는다.
  서버 readiness는 checkpoint·model config·sensor SHA와 클래스·입력·옵션을 검사한다.
  모델을 로드하지 않은 projection 서버나 다른 모델의 ready 응답을 정상으로 처리하지 않는다.
- `~/bevfusion_artifacts/checkpoints/base5_e24_inference.pth` 설치 SHA는 E24 원본과 동일하다.
  팀 Drive `model/bevfusion/production/base5_e24_261013/E24.pth`에 약 501MiB 원본을 올렸다.
  공용 client의 quota 오류 후 전용 client를 사용했으며 원격 native MD5와 로컬 MD5
  `caaf2a5583ff90a169ae969aea927980`가 같았다. 원격 전체 SHA readback을 수행한 것은 아니다.
  팀원 준비기가 실제 다운로드 파일의 SHA256을 다시 검사한다.
- 현재 Docker 이미지의 upstream `depth_lss.py`, `bevfusion.py`, `sparse_encoder.py` SHA는
  checkout과 같았다. 다른 PC 이미지의 동일성이나 GPU 실행 인증으로 확대하지 않는다.
- profile·weights 보존·readiness·기존 bridge 입력·context 단위 검사 19개, 실제 roslaunch
  parser의 새 canonical launch 해석, Compose 해석·shell/Python 구문·문서 링크 검사가 통과했다.

로컬 설치 tracker에는 최신 소스의 일부 옵션이 빠져 있었다. 팀원 handoff는 package 증분
빌드를 요구하며 이후 정책 비교에는 기존 검증된 별도 tracker binary를 사용한다.
이번 승격에서 tracker·planning의 기본 안전 정책을 자동으로 완화하지 않는다.

<a id="base5-tracker-planning-policy-review-261013"></a>

## 46. 저장 bag 기반 tracker와 planning 정책 검토 — 2026-10-13

E24를 기본값으로 바꾼 뒤, 기존 tracker 정책이 새 모델의 원거리·근접 검출을 막는지 검토했다.
MORAI·Vast·GPU detector·무거운 planning 풀스택은 실행하지 않았다. 저장된 검출을 private ROS
master의 실제 C++ tracker·UNKNOWN policy에 공급했고 planning은 현재 소스와 launch/설정을 읽었다.
판단팀 브랜치·planning_validator·planning/control 소스와 파라미터는 변경하지 않았다.

### Bag 출처와 실제 기록

`planner_logs/validation`, `rosbags/eval`, 기존 일곱 raw mission bag 경로에서 bag 250개를
read-only index 검사했다. 인지 결과 토픽이 있는 것은 179개였고, model sidecar에 E24 preset과
인지 토픽이 함께 있는 것은 7개였다. E25 preset은 6개, 나머지 166개는 이 조사에서 모델을
식별하지 않았다. 파일명만으로 E24나 동일 초기화 결과로 분류하지 않는다. Sidecar의 선택 모델·
파일 SHA는 기록된 선택 근거이며 실제 GPU 서버 가중치의 독립적인 런타임 증명이 아니다.

E24 7개와 과거 정적 회귀 16개 bag의 실제 perception 메시지도 읽었다. E24 두 초기 full-course
bag은 detection/tracking이 모두 비어 있었다. 다른 full-course bag과 정적 네 bag에는 비어 있지
않은 인지 출력이 있었다. 정적 네 bag은 모두 Ego 변위 0·최대 속도 0의 출발 실패/timeout이다.
주로 20~30m 장애물을 반복 관측한 기록이므로 초근접 회피 성공이나 tracker 때문에 출발이
막혔다는 근거로 사용하지 않는다. CollisionData 0도 차량이 실제 지나가며 안전했다는 뜻이 아니다.

정적 네 bag의 detector 메시지 수는 각각 488/493/492/494, 비어 있지 않은 메시지는
481/493/492/494다. 동일 순서의 tracking nonempty는 1671/1888/1837/1748이다.
기록된 장애물 class는 UNKNOWN으로 유지돼 네 bag 안의 track class switch는 0이었다.
모델 성능이나 위치 정확도를 이 전달량으로 인증하지 않는다.
기록 토픽의 거리 집계는 detection의 LiDAR 원점과 tracking의 기록 Ego 원점이 다르다.
같은 원점의 정량 recall/오차 비교는 아래 동결 GT 재생 결과만 사용한다.

기록된 detection 수신 시각−header stamp의 중앙값은 약 0.367~0.383초,
p95는 0.435~0.458초이고 header 간격 중앙값은 약 0.20초다. Tracking 출력은 약 30Hz이며
수신−출력 stamp는 약 8ms였다. 후자는 measurement가 8ms 전의 관측이라는 뜻이 아니다.
fixed scheduler가 오래된 measurement state의 prediction copy에 현재 stamp를 붙이므로
입력 관측 age와 출력 age를 반드시 구분한다. 해당 구 PC 비용을 대회 4090의 비용으로 외삽하지 않는다.

### 소스에서 확인한 정책

현재 EKF·표준 detection 변환·motion policy에서 MORAI 구역이나 30/50/80m별로
Vehicle을 UNKNOWN으로 강제 바꾸거나 초근접 객체를 일괄 제외하는 분기는 찾지 못했다.
다음 일반적인 필터가 실제로 있으며, 없애기보다 영향과 물리적 목적을 따로 확인한다.

| 정책 | 현재 동작과 검토 |
|---|---|
| 클래스 smoothing | alpha 0.2. UNKNOWN에서 일관된 Vehicle 관측 네 번이면 대표 클래스가 바뀔 수 있다. 실제 지연은 입력 Hz·miss에 따라 다름 |
| 낮은 confidence | score<0.5에서 measurement covariance를 10배로 키움. E24 class별 score의 보정 상태와 matched residual을 후속 확인할 가치가 있음 |
| Association | 비보행자 5m, 보행자 2m gate. Vehicle/UNKNOWN 간 association은 가능하며 큰 중복 box의 birth는 별도로 억제 |
| 중복 birth 억제 | mature track이 해당 frame에 관측됐을 때 0.75m 내 유사한 큰 unmatched box의 새 ID를 억제. 원거리/구역 전용 clip이 아님 |
| Coasting | moving 0.5초, UNKNOWN/저속 차량 1초. 41절의 확대 A/B에서 잔류·중복 때문에 기본 확대를 채택하지 않았음 |
| UNKNOWN motion | pose·shape·ID·class는 유지하고 출력 속도·가속도·yaw rate를 0으로 만듦. 차량 오분류 시에는 부작용이 있으나 정적 장애물의 가짜 운동을 막는 목적 |

과거 정책이 존재한다는 사실만으로 과적합이 입증되지는 않는다. 낮은 confidence의 covariance나
클래스 smoothing은 기존 EKF의 일반 정책이며, 새 모델에서 부적절할 가능성과 실제 회귀를 구분한다.

### E24 고정 실제 ROS 비교

O02/P01/H03/R01 네 개발 bag의 전체 1,570 입력, catalog GT가 해결된 scoring 1,413프레임을
사용했다. weights·검출 배열·GT·0.40/0.10/0.10·NMS·0.5/1초 coasting은 고정하고 후보의
한 정책만 바꿨다. 모델 추론을 다시 실행하거나 score와 lifecycle을 함께 바꾸지 않았다.
입력은 43절의 `frozen_gt_v2/E24_amp_parallel.json.gz`, full-range GT는 41절의 `inputs_v1`이다.

| 비교 | 기본 정책 | 후보 정책 | 결론 |
|---|---:|---:|---|
| O02 0~30m 장애물 대응@2m, 중복 birth guard OFF | 887/897 | 887/897 | 근거리 recall 이득 없음 |
| 같은 구간의 미대응 출력@2m | 376 | 523 | +147개. 실제 유령 수가 아니라 GT 미대응 관측 수 |
| O02 전체 장애물 대응/미대응@2m | 1443/531 | 1447/748 | 대응 +4보다 미대응 +217이 큼 |
| O02 전체 1초 이상 미대응 episode | 15 | 23 | 더 긴 중복·잔류 위험. 독립 실제 객체 수가 아님 |
| H03 Vehicle 대응@2m, 빠른 클래스 승격 ON | 228 | 228 | 고주로 이득 없음 |
| R01 Vehicle 대응@2m, 빠른 클래스 승격 ON | 669 | 670 | 로터리 30~50m 관측 1건 개선 |

따라서 **중복 birth guard는 유지**, `fast_confirmed_vehicle_promotion`은 기존 기본 OFF를
유지한다. 단 한 개발 관측의 이득으로 대회 클래스 정책을 자동 변경하지 않는다.
이 시험은 six-asset O02·Man1·두 차량 미션에 한정돼 희귀/복합/초근접 두 실제 객체의 안전성을
인증하지 않는다. 실제 distinct actor가 0.75m 근처에 있는 조건은 별도 MR 검증 대상으로 남긴다.

두 A/B의 독립 reference repeat는 모든 case에서 timestamp·관측 집합·위치 차이 0이었다.
별도 검증 binary SHA는 `7dda240b5e3331ea2b3322eb6e2403c2e8e1616d8dd665b95d31c7baf80ca3d1`이다.
로컬 install binary `ef67431d3c12987e5d8fd3e9384332b338db9e86157c2e66bf46d4c157101f5c`에는
promotion option이 없었다. 이를 사용한 `promotion_ros_v1`은 정책 비교 근거에서 제외하고
현재 소스를 반영한 `promotion_ros_v2`를 사용했다. Harness에 미지원 옵션의 binary를 거부하는 검사를 추가했다.
로컬 install과 다른 agent 세션은 재빌드·교체하지 않았다.

### Planning과 연결되는 제약

실제 통합 launch는 `autoware_launch/config/ros1/planning/behavior_path_modules.yaml`을
로드한다. 개별 module의 sample YAML을 실제 운영값으로 오인하지 않는다.

- SOA는 UNKNOWN을 허용하며 동적 전방 검토 범위를 min 50m/max 150m로 clamp한다.
  실제 range는 lateral jerk·shift·Ego speed·prepare distance에 따라 결정된다. 이미 30m로
  고정된 구조가 아니므로 더 이른 신뢰할 만한 검출을 활용할 경로는 있다. 더 이른 조향을 보장하지는 않는다.
- 통합 base launch는 UNKNOWN 안정화 대기를 2초 sample 값에서 **0초**로 override한다.
  최신 통합 경로를 사용한다면 이 대기를 detector 개선 후 새로 풀어야 한다고 가정하지 않는다.
- LC는 `target_object/unknown=false`이며 실제 소스가 클래스 필터를 먼저 적용한다.
  움직이는 차량을 UNKNOWN으로 오래 전달하면 차선 변경의 해당 객체 검사에서 빠질 수 있다.
  모든 UNKNOWN을 Vehicle로 강제 바꾸거나 모든 UNKNOWN을 무조건 LC 대상으로 넣지는 않는다.
- SOA의 `avoidance_for_close_vehicle.policy=ignore`와 정지/주차/차선 조건은 차량 검출 개선과 별개다.
  CAR로 올바르게 전달돼도 어떤 모듈이 회피·정지를 담당할지는 planning팀이 폐루프에서 확인해야 한다.
- 더 안정적인 중심·크기·속도·클래스와 낮은 입력 age는 planning의 검토 범위·관측 신뢰 gate를
  재검토할 근거가 된다. 그러나 RSS 거리·반응시간·jerk·collision margin을 모델 AP만으로 줄이지 않는다.

### 후속과 산출물

Tracker production 파라미터는 이번 검토에서 변경하지 않았다. 다음은 실제 대회 PC에서
관측 age·queue·class 전환·Bbox 여유를 기록하고, E24로 움직이며 초근접 통과·후방 추월을
검증하는 것이다. Detector status/latency, detection, measurement tracking, policy tracking,
prediction, odometry/TF, planner debug/trajectory, CollisionData를 함께 기록한다.
Validator의 녹화 설정 변경은 해당 담당 작업으로 전달하며 이번 검토에서 수정하지 않았다.

생성기 pilot에는 이동 중 가림 전후 관측, 회피 후 측방·측후방 부분 관측, 실제 distinct actor가
가까운 혼합 조건을 넣는다. Stationary Ego 20~30m 반복 출력만으로 근접 전달이 검증됐다고 하지 않는다.
크기·yaw·가림별 GT와 detector box 중복을 함께 보며 tracker 필터 제거로 학습 부족을 대체하지 않는다.

- Root: `artifacts/evaluation/bevfusion/base5_production_tracker_review_261013`.
- `inventory_v1.json`, `recorded_delivery_v1.json`은 모델 출처와 실제 토픽 관측을 분리한다.
- `promotion_ros_v2`, `birth_guard_ros_v1`의 messages·receipt와 대응 metrics를 보존했다.
- `summary_v1.json` SHA256:
  `22408fd5d051df62fe8b057bdc60631062cf47e706a328b3a924fd4d81a28523`.
- 재현 도구는 inventory·recorded audit·policy summary와 단일 정책 actual ROS replay다.
  원본 bag은 재녹화·재색인·삭제하지 않았고 자신이 시작한 private ROS master/노드만 종료했다.

production profile·bridge 입력·tracker replay·bag audit·거리 평가·checkpoint 선택·context 관련
CPU 단위 검사 **46개**가 통과했다. 문서 링크·원문 보존·Python 구문·whitespace 검사도 통과했다.

<a id="base5-range-tracking-pivot-261013"></a>

## 47. 원거리 차량과 다음 수집의 개선 기점 — 2026-10-13

팀장님 논의는 새로운 가설이며 실제 실행 정책으로 간주하지 않는다. 사용자는 팀장님이 E24
실제 풀스택 검증을 수행할 예정이라고 확인했고, 로컬에서는 문서화와 후속 문제 해결을 요청했다.
[공통 개선 계획](261013_BASE5_RANGE_AND_TRACKING_IMPROVEMENT_PLAN.md)에 담당·순서·상태·채택 기준을 기록했다.
E24 production·MORAI·학습 source·v150 배치·기존 데이터와 원본 bag은 변경하지 않았다.

### 실제 train GT 노출

32절의 `base5_range_evidence_release_261012/objects.json`에서 split=train인 실제 파생 라벨을
객체 중심 XY 방사 거리로 집계했다. 4,255프레임의 9,350 GT이며 고유 자산 수나 실제
BEV 증강 뒤 gradient 노출량이 아니다. Animal은 현재 학습 class인 Pedestrian에 포함한다.

| 거리 | Vehicle | Pedestrian | Obstacle |
|---|---:|---:|---:|
| 0~30m | 2,542 | 2,218 | 2,227 |
| 30~50m | 1,046 | 311 | 324 |
| 50~100m | 590 | 28 | 64 |
| 합계 | 4,178 | 2,557 | 2,615 |

40m 이상 P/O는 299개(전체 GT의 3.20%), 50m 이상 92개(0.98%), 60m 이상 51개(0.55%)다.
50m 이상 Vehicle 590개 중 native LiDAR 20점 이상은 13개였다. 현재 crop의 카메라 정보량
proxy는 실제 학습 성능이나 depth 정확도를 보증하지 않는다. 적은 원거리 비차량이 전체
차량 학습을 크게 방해했다는 인과 근거는 없으며, 객체 수만으로 영향이 없다고 증명하지도 않는다.
새 수집에서 먼 작은 비차량의 의도적 quota를 줄이는 후보와 기존 양성 라벨 삭제는 구분한다.

현재 TransFusion target은 GT가 없는 heatmap을 0으로 만들고 unmatched query에도 classification
loss를 준다. 실제 센서에 남은 객체의 GT만 제거하면 중립적인 ignore가 되지 않는다.
종류별·거리별 무시 정책을 적용하려면 assignment·heatmap/query loss를 별도로 검증해야 한다.

### 저장 검출의 Vehicle score 비교

새 `analyze_base5_far_vehicle_scores.py`는 CPU에서 41절의 full-ROI 개발용 입력과
동일 E24 저장 검출을 사용한다. 서버 floor 0.1/top100·class-aware NMS 0.5 뒤 Vehicle
score만 0.40/0.35/0.30/0.20/0.10으로 바꿨다. P/O score는 0.10으로 고정했다.
새 GPU 추론·ROS replay·지도 필터·tracker·planner 실행이나 production threshold 변경은 없다.

입력 1,570장 중 기존 기준과 같은 1,413장만 정확도에 사용했다. 미확정 GT 157장은 제외한다.
원래 LiDAR index와 case별 시각 공백을 보존하고, 제외된 frame을 넘어 누락 구간을 연결하지 않는다.
ROI 전체에서 먼저 대응한 뒤 GT 거리로 recall, prediction 거리로 미대응을 집계한다.
0.40의 각 case·거리·1/2/4m gate의 GT/TP/prediction/미대응 count는 기존 validated_v6와 정확히 같다.
누락 시간은 이번 도구의 case별 적격 시각 median으로 추정하며 기존 시간 통계와의 완전 동일성을 주장하지 않는다.

아래 recall은 2m 중심 gate이며 분모는 30~50m 306, 50~80m 276, 80~100m 91개의 GT 관측이다.
미대응은 같은 gate에서 50~80m 위치에 발행될 Vehicle 박스의 관측 수다. 의미상 오검출 확정 수가 아니다.

| Vehicle score | 30~50m recall | 50~80m recall | 80~100m recall | 50~80m 미대응 | 전체 ROI 미대응 |
|---|---:|---:|---:|---:|---:|
| 0.40 | 84.31% | 69.57% | 13.19% | 12 | 23 |
| 0.35 | 85.95% | 71.01% | 14.29% | 14 | 25 |
| 0.30 | 87.58% | 72.46% | 16.48% | 14 | 28 |
| 0.20 | 90.20% | 74.28% | 18.68% | 20 | 43 |
| 0.10 | 91.83% | 77.17% | 28.57% | 26 | 61 |

0~30m Vehicle recall@2m는 모두 350/350이나 해당 prediction 거리의 미대응은 8→22로 증가했다.
이는 four-bag 개발용 분모의 결과이며 전체 근거리 클래스·안전성이나 보편적 threshold 인증이 아니다.
0.30은 50~80m에서 TP +8, 0.10은 +21로, threshold만으로 큰 누락이 해소되지 않았다.
80~100m는 0.10에서도 recall 28.57%이므로 안정적인 전달 범위로 격상하지 않는다.

전체 ROI의 미대응 위치 진단은 다음과 같다. 같은 클래스 GT와 1~2m인 박스도 이미 다른
박스가 대응했거나 ROI 밖 알려진 actor 근처일 수 있어, 단독으로 ID 중복을 확정하지 않는다.

| score | 다른 클래스 catalog actor 4m 이내 | 같은 클래스 1~2m | 같은 클래스 2~4m | catalog actor 4m 이내 없음 |
|---|---:|---:|---:|---:|
| 0.40 | 17 | 1 | 3 | 2 |
| 0.30 | 19 | 2 | 4 | 3 |
| 0.10 | 41 | 2 | 8 | 10 |

0.10의 O02 미대응 중 21개는 다른 클래스 actor 근처다. 도로 polygon 필터만으로
도로 위 장애물의 Vehicle 오분류까지 해결된다고 가정하지 않는다. catalog가 지도 배경을
모두 포함하지 않으므로 마지막 열을 모두 존재하지 않는 물체라고 해석하지 않는다.
각 cell의 최대 5개 예시는 시간순 진단 후보이며 대표 오검출 표본이 아니다.

### 후단과 수집 정책의 판단

Production 0.40을 유지한다. 다음 후보는 높은 score·반복 관측으로 새 차량을 확인하고
낮은 score는 기존 확인 차량의 갱신에만 활용하는 opt-in association이다. 검출 recall뿐 아니라
실제 tracker의 ID·오차·잔류와 planner 반응을 확인한다. 현행 기억 시간을 일괄 늘리지 않는다.

현재 Map-CV는 매칭 실패 시 Cartesian CV로 fallback하며 검출 삭제 필터가 아니다.
현재 Lane Change의 `use_all_predicted_paths=false`는 최고 confidence 동점을 모두 남긴다.
`objects_filtering.hpp`와 `safety_core.cpp`에서 동일 정책을 확인했다. 현재 Obstacle Cruise는
주요 경로와 outside-cruise 설정에서 한 경로를 사용한다. 실제 소비 module·동점·잘못된
Top-1·관측 age/속도/BBox·안전거리 조건을 구분하기 전에 Ego geofence로 전역 Top-1을 강제하지 않는다.

다음 배치의 비차량 0~40m 집중·40~50m 소량 유지·50m 이상 의도적 quota 축소와
차량 50~80m 보강은 다음 후보다. 기존 v150 payload나 실행 정책은 그대로이며 완료 5K를
폐기하지 않는다. JPEG 자동 전달과 업로드 검증 후 RGB PNG 정리는 향후 구현 항목이고,
현재 원본 해시 기반 collector resume 중 파일을 삭제하지 않는다.

### 산출물과 검증

- 출력: `artifacts/evaluation/bevfusion/base5_range_tracking_improvement_261013/score_sweep_v1.json`.
- 출력 SHA256: `9fc8ee46ccb91e50b56f37e746583ba414b28b169949be71ea36cefa501ef139`.
- GT infos SHA256: `64ef1a1dacb95e3e00b1f3b4e0556bee4f18538dd712dcca7ba5c9a51665b866`.
- 입력 prediction SHA256: `c2f89327a3105143b460032000a24de0707294989dc619870da88e345ff192fa`.
- 기준 report SHA256: `ac0a70f32168f6942e43cba6d82668f8a50b6298144b98693e61c4c3b417ca84`.
- 시작/끝 input SHA를 재확인해 원본 불변을 검사했다. GT geometry·label·token·적격 분모를 대조했다.
- 거리 경계 정상 대응·원거리 무점 GT·다른 클래스·중복/미대응·frame 제외·JSON·기존 matching 관련 CPU 검사 23개가 통과했다.

관련 참고는 [ByteTrackV2](https://arxiv.org/abs/2303.15334)의 계층적 low-score association과
[Autoware map prediction](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_map_based_prediction/)의
lanelet/차선 변경 이력이다. 현재 custom tracker/predictor에 이 기능을 구현했다거나 논문 성능을 재현한 것은 아니다.

<a id="base5-low-score-core-probe-261013"></a>

## 48. 저신뢰도 갱신의 실제 EKF 코어 비교와 한계 — 2026-10-13

사용자는 초기 threshold 실패, 확인 후 누락, 위치/속도 변화, 유지한 장점과 새 부작용을
다각도로 분석한 뒤 판단할 것을 요청했다. 새 GPU 추론·MORAI·원격 학습·ROS master를
실행하지 않고 47절의 동일 E24 검출을 실제 C++ EKF 코어에 재생했다.
설치된 tracker·submodule·production config·checkpoint와 원본 bag은 변경하지 않았다.

### 독립 원형과 비교 조건

`build_base5_low_score_probe.py`는 현재 헤더/CPP를 읽고 엄격한 anchor를 검사한 뒤
artifact 안에 원형을 생성한다. 원본 source와 수정 복사본을 각각 별도 바이너리로 컴파일한다.
비활성 원형과 원본 코어, 원본 독립 반복은 네 bag의 모든 state에서 정확히 같았다.
기존 EKF prediction/update/lifecycle을 재사용하며 매 frame에 한 번만 lifecycle을 진행한다.
현재 MatchPairs는 Mahalanobis cost 정렬 후 greedy 대응이고 Hungarian이 아니다.

CTRV·association 5m·XY noise 0.40m·dimension alpha 0.3·yaw-rate filtering ON·
OD velocity/center-motion fusion OFF와 원래 0.5/1초 coast를 사용했다. Recorded Ego와
Sensor_4로 모든 관측을 동일 map 좌표로 옮겼다. 1,570장 전부를 이력에 넣고
기존 GT 적격 1,413장만 집계했다. TF node·orientation canonicalizer·30Hz scheduler·
Map-CV·planner·실시간 queue는 실행하지 않았다. Z는 이 core probe의 bottom 기준이다.

12개 프로필은 baseline/반복, 약한 floor 0.30/0.20/0.10의 XY 갱신, 0.20 전체 갱신,
약한 허용 창 1초, 클래스 평활 score 0.80/0.90, 이동 조건, 단순 0.20 새 birth 허용이다.
변경 요인을 분리하며 개발 bag으로 후보를 정제했다. 이를 untouched test로 부르지 않는다.

높은 관측을 먼저 대응하고 남은 confirmed CAR/TRUCK에만 약한 Vehicle을 연결한다.
30m 이상·XY innovation 2m 이내·pi 대칭 yaw 60도 이내·길이/너비 ratio 2 이내를 요구한다.
낮은 후보의 새 birth나 tentative confirmation은 금지한다. XY 전용은 약한 관측의 크기·Z·
클래스 score를 반영하지 않고 yaw 측정을 비정보적으로 만든다. XY와 state covariance의
연결 때문에 EKF 속도/yaw state가 전혀 변하지 않는다는 뜻은 아니다.

### 원거리 누락과 threshold의 관계

GT/ROI/원래 frame index/0.25초 공백마다 연속 구간을 분리했다. 다음은 50~80m GT 276개에
대한 같은 floor 0.10/NMS 뒤의 진단이다. GT는 tracker 입력이나 association에 사용하지 않았다.

| 관측 상태 | GT 관측 수 |
|---|---:|
| 0.40 이상 Vehicle, 중심 2m 이내 대응 | 192 |
| 이전 높은 대응 이후 0.10~0.40 Vehicle 대응 | 21 |
| Vehicle 후보가 있지만 중심 차이 2~4m | 4 |
| 다른 클래스 후보가 GT 2m 이내 | 2 |
| 2m 대응도 없고 Vehicle 4m 이내 후보도 없음 | 57 |

0.40→0.10으로 살릴 수 있는 raw 대응은 21/276이다. 나머지를 단순 threshold 문제라고
설명하지 않는다. 마지막 열에는 floor 미만 score·큰 위치 오차·다른 원인이 섞일 수 있어
센서나 모델에 후보가 전혀 없었다고 단정하지 않는다.
80~100m GT 91개는 높은 대응 12, 기존 높은 대응 뒤 약한 대응 14, 나머지 65개다.

이 적격 구간에서 올바르게 대응한 약한 Vehicle은 모두 이전 높은 대응 이후였다.
두 never-high 구간은 각각 3개 관측이고 거의 ROI 끝이라 초기 획득 실패의 대표로 쓰지 않는다.
H03은 주요 차량이 약 26/29/65m에서 시작해 멀어진다. R01의 주요 후방 차량도 약
30~52m에서 이미 보던 뒤 80~100m로 멀어진다. 대부분 first-high는 구간 시작부터 있어
최초 탐지 거리가 left-censored다. 이 결과로 100m에서 접근하는 미확인 차량의 bootstrap을 인증하지 않는다.

참고로 기존 native val의 독립 50~80m Vehicle 51개는 1m gate에서 score 0.40 대응 28개,
0.10 대응 32개였다. 이 역시 threshold만으로 모든 원거리 미탐을 해결한다는 근거가 아니다.
단일 장면이므로 이 숫자로 연속 track 생성 가능성을 판단하지 않는다.

### 효과와 평균 및 tail 분포

최종 시험 후보는 `weak010_moving_class090_xy`다. 약한 창 0.5초, 클래스 평활 score 0.90,
현재 정지 판정 경계인 world speed 2m/s 초과를 추가한다. 클래스 score는 실제 확률이 아니다.
정지·저속 차량은 삭제하지 않고 baseline 처리를 유지한다. 낮은 관측이 last measurement를
갱신하므로 총 유지 시간은 늘 수 있으며, 약한 창과 삭제 타이머를 같은 것으로 설명하지 않는다.

아래는 tracking의 중심 2m 이내 recall과 prediction 거리의 미대응 관측 수다.
미대응은 의미상 오검출 확정 수가 아니다. 동작을 실제 ROS가 아니라 core probe에서 비교했다.

| 거리 | baseline TP / GT | 시험 후보 TP / GT | baseline / 후보 미대응 |
|---|---:|---:|---:|
| 0~30m | 346 / 350 | 346 / 350 | 22 / 22 |
| 30~50m | 269 / 306 | 271 / 306 | 3 / 3 |
| 50~80m | 254 / 276 | 257 / 276 | 47 / 47 |
| 80~100m | 28 / 91 | 38 / 91 | 6 / 6 |

50~80m은 92.03→93.12%의 작은 증가다. 80~100m은 30.77→41.76%지만 이득이 일부
이미 보던 차량에 집중되어, 원거리 전체 성능이 해결됐다고 보지 않는다.
H03 14회/R01 28회의 실제 약한 갱신이 있었고 O02/P01은 0회다.

평균·p50·p90·p95는 양쪽 모두 4m 이내로 대응한 같은 GT 관측만 사용한다.
miss는 별도 recall로 보고하며 새 대응이 조건부 평균을 바꾸는 selection bias를 구분한다.

| 동일 관측의 중심 오차 | 관측 수 | 평균 baseline→후보 | p50 | p90 | p95 |
|---|---:|---:|---:|---:|---:|
| H03 50~80m | 123 | 0.807→0.801m | 0.467→0.466 | 1.769→1.769 | 1.903→1.900 |
| R01 50~80m | 135 | 0.667→0.657m | 0.533→0.533 | 1.238→1.196 | 1.437→1.418 |
| H03 80~100m | 14 | 1.245→1.245m | 1.148→1.148 | 1.931→1.931 | 2.026→2.026 |
| R01 80~100m | 16 | 1.326→0.570m | 1.259→0.461 | 1.890→0.906 | 1.985→1.103 |

R01 80~100m 16개 동일 관측에서 속도 벡터 오차 평균/p50/p90/p95는
2.574/2.492/3.829/3.886→0.689/0.755/0.999/1.042 m/s다.
이는 교정된 BBox-center trajectory의 ±0.2초 local fit를 기준으로 한 오차이며,
순간 UDP velocity GT나 실전 AVE 인증이 아니다. Future fit는 평가에만 사용한다.
H03의 이득은 작고 R01 한 주요 후방 궤적의 이득이 크므로 평균만으로 보편적 향상을 주장하지 않는다.

연속 같은 ID의 속도 벡터 변화와 CV 이동 residual의 평균·p50·p90·p95·최대도
association/coasting으로 나눠 저장했다. 이것은 실제 조향 jerk나 exact CTRV innovation이 아니다.
약한 갱신이 큰 새 spike를 만들었다는 근거는 이번 후보에서 없지만, 표본 구성과 조건부
분포를 구분하고 기존 수 m/s의 초기/강한 관측 변동까지 해결했다고 주장하지 않는다.

H03의 큰 속도 오차도 원인을 따로 확인했다. `H03_162301_00070/00071`은 새로운
ID 3이 age 4에서 confirmed된 직후이며 reference 대비 약 7.36~7.37m/s 차이가 있었다.
selected 후보에서도 동일하다. 이는 약한 보충이 새로 만든 spike가 아니라 기존의 재확인/
초기 속도 추정 문제이며, GT reference의 근사도 함께 고려한다. 확인 여부와 속도 성숙도를
같은 것으로 취급하지 않는다. ID 변화 지표 0도 GT 제외/누락 구간을 넘는 새 ID를 부정하지 않는다.

### 발견한 부작용과 보호 조건

단순 floor 0.20과 새 birth 허용은 전체 ROI Vehicle 미대응 78→111, R01 인접 GT-key↔
track-ID 변화 0→1이었다. 근접 Vehicle 미대응도 22→37로 증가했다. 이를 채택하지 않는다.
약한 관측의 크기/yaw/클래스까지 갱신한 0.20 후보는 50~80m Vehicle 미대응 47→52였고,
약한 창을 0.5→1초로 늘린 0.20 후보는 차량 이득 없이 O02 갱신만 추가했다.

XY만 쓰는 일반 0.10 후보에서도 CAR였던 track이 이후 UNKNOWN이 되며 O02의 11개
UNKNOWN 관측에 최대 0.2746m 위치 차이가 전파됐다. 최대 차이는 약 61m에 있었다.
CAR 클래스 평활 score 1.0과 크기 5.17×2.18m도 실제 차량임을 보증하지 않았다.
클래스 margin 0.90만으로는 이 부작용이 사라지지 않았다.

해당 false-vehicle track의 약한 갱신 때 속도는 약 0.44~0.73m/s였다. 현재 정지 경계
2m/s를 약한 보충의 이동 조건으로 사용하자 O02 갱신은 0회가 됐고 그 bag의 모든
track state가 baseline과 같아졌다. P01도 전부 같으며, H03/R01의 confirmed 비차량
출력 state도 모두 같았다. R01 후방 이득은 유지됐다. 이 조건은 약한 보충만 제한하며,
실제 정지 차량을 없애거나 저속 차량을 오검출로 단정하지 않는다. 가짜 world motion이나
새 환경의 진짜 저속 차량에서 같은 효과가 난다는 인증도 아니다.

### 검증과 다음 판단

원형은 별도 artifact 바이너리일 뿐 production node의 기능을 추가한 상태가 아니다.
보호 조건을 적용한 프로필만 실험 후보로 유지한다. 실제 ROS boundary와 새 incoming
차량 구간에서 검증하기 전에는 기본 ON으로 바꾸지 않는다. 강한/모든 관측 age,
약한 갱신 여부·측정/track confidence를 구분해 후단과 planner가 불확실성을 확인해야 한다.
새 bag은 전방 및 후방 80~100→30m의 초기 접근, 가림/가감속, 정지 차량과 도로 배경을 포함한다.
이전에 보던 차량이 멀어지는 이번 조건과 분리하며 threshold 아래 후보나 짧은 raw가 필요하다.

- Root: `artifacts/evaluation/bevfusion/base5_low_score_tracking_261013`.
- 최종 build: `build_v3/build.json`, `original`, `probe`. 최종 matrix: `replay_v6/report.json`, `frames.json.gz`.
- Report SHA256: `69faf365c6dbf24cb9c262c9a4c86af6c15b516a7eed788d9634d1c1a71e4863`.
- Frames SHA256: `8d757bf67ddc17ed9a1b357e01e8d3ce003000ef2b90334e7677785294791de7`.
- Production CPP/Header SHA: `9b413ea8b7786df408650b706f516052f27d24e117c48a548729f38d95a9827b` /
  `6f00ac4890215e9b415324edc0d08f9db7cf7df8a535b65374e61cb950b7944a`. 원본은 유지됐다.
- `qualification_v2.json`은 frozen source·원본 반복·동일 이력/stamp·비차량/근접 회귀를 확인한다. `production_enabled=false`, 새 bag/full-stack 미검증이다.
- `review_r01_gt4_v2`와 `review_h03_gt5_v2`의 SVG/PNG/trace JSON에 거리·score·중심/속도 오차를 남겼다. R01 하나의 이득만 보여주지 않고 H03의 작은 이득도 보존했다.
- 초기 분석 v1의 score 필드 누락과 ROI 이후 ID indexing을 보완했다. ROI 경계를 넘는 track의 quality index는 ID로 다시 찾는다. 이전 단계는 보존하되 최종 수치에 섞지 않는다.
- CPU/실제 compiled-core 관련 검사 40개가 통과했다. 저신뢰도 무생성·tentative 보호·high 우선·shape/innovation gate·pi 모호성·무한 유지 방지·XY 속성 보호·클래스/정지 보호·원본 반복을 포함한다.

재현은 새 artifact 경로로 build → evaluate → qualify 순서로 수행한다. ROS master나 GPU는 필요 없다.

```bash
python3 perception/detection3d/bevfusion/tools/build_base5_low_score_probe.py --output /tmp/base5_low_score_build_new
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 python3 perception/detection3d/bevfusion/tools/evaluate_base5_low_score_probe.py \
  --inputs artifacts/evaluation/bevfusion/base5_far_bag_retention_261012/inputs_v1 \
  --predictions artifacts/evaluation/bevfusion/base5_far_bag_retention_261012/inputs_v1/predictions_0.json.gz \
  --build /tmp/base5_low_score_build_new --sensor-config configs/sensors/Sensor_4.json \
  --output /tmp/base5_low_score_replay_new
python3 perception/detection3d/bevfusion/tools/qualify_base5_low_score_probe.py \
  --replay /tmp/base5_low_score_replay_new --build /tmp/base5_low_score_build_new --output /tmp/base5_low_score_qualification_new.json
```

<a id="base5-map-path-consumers-261014"></a>

## 49. 지도 보조·모듈별 경로 소비·차선 이력 진단 — 2026-10-14

팀장님 새 bag을 기다리는 동안 기존 기록과 현재 소스로 Top-1 설명을 재검토했다.
GPU·MORAI·ROS master·유료 인스턴스·planner를 실행하지 않았다. Production 코드·
설정·하위 submodule·원본 bag·동결 추적 결과는 변경하지 않았다.

### 발행·시각화·planner 소비를 구분한다

Map-CV는 현재 위치/yaw와 차선 중심선 거리로 시작 후보를 고르고 연결된 successor를
열거한다. 과거 lanelet 이력은 사용하지 않는다. 최대 6개의 구별되는 물리 경로를 발행하며,
RViz가 그 경로를 표시하는 것과 특정 planner module이 모든 경로를 검사하는 것은 다르다.

| 현재 소스의 소비자 | 현재 통합 설정과 동작 |
|---|---|
| Lane Change | `use_all_predicted_paths=false`. 최고 confidence의 **정확한 동점 전체**를 유지한다. near-tie는 동점이 아니다. |
| Obstacle Cruise inside | confidence 정렬 뒤 1개를 재표본화하여 검사한다. |
| Obstacle Cruise outside | 설정 개수의 경로를 선택한다. 통합 YAML의 `outside_obstacle/num_of_predicted_paths`는 1이다. |
| Roundabout | confidence 0.05 이상 경로를 순회한다. 유효 길이·attention 대상·시간/형상 교차 조건을 검사하고 위험 충돌이면 조기 중단한다. |

LC 근거는 `objects_filtering.hpp:157`, `safety_core.cpp:305`, LC `utils.cpp:748`이다.
Cruise는 `obstacle_cruise_module.cpp:548/607`과 통합 `motion_velocity_modules.yaml`을
대조했다. 기본 `bvp_experimental=true` 경로는 `experimental::RoundaboutModulePlugin`이며,
해당 `experimental/scene_roundabout_collision.cpp:188`을 확인했다. Normal 구현도 같은
confidence 순회 방식이다. 실제 실행의 module override와 parameter는 팀장님 bag/런타임으로
별도 확인한다. 정확한 동점에서 Cruise의 C++ 정렬 순서는 보장되지 않는다.

### 기존 실제 기록의 사용 가능 범위

기존 inventory에서 E24 선택 receipt 7개, 고주로 E25 기록 1개, E25 전체 코스 1개,
9월 전체 코스 1개를 read-only 조사했다. 10개 모두 prediction 토픽이 있었다.
E24 7개와 해당 고주로 기록은 차량 class 관측이 0개라 원거리 차량 경로 검증으로 쓰지 않는다.
빈 객체 메시지와 비차량만 있는 메시지는 구분하며 실패 원인으로 추정하지 않는다.

E25 전체 코스의 차량 관측 1,810개 중 다중 경로는 1,265개, 9월 전체 코스의 6,864개 중
다중 경로는 4,989개였다. 두 기록의 최고점 **정확한 동점은 0개**여서 현재 LC 정책을
재적용하면 모두 한 경로만 남는다. 반면 0.05 이상 복수 경로는 각각 1,053/4,552개다.
이 두 기록에는 이번 auditor가 확인한 LC/Cruise/Roundabout debug 토픽이 없었다.
과거 발행 경로의 현재 정책 재해석이지 당시 planner가 선택한 경로의 직접 기록은 아니다.
9월 기록의 detector 가중치와 실행 소스는 파일명으로 식별하지 않았다.

동일 ID·frame·0.25초 이내 관측만 연결하고 동일한 미래 절대 시각에서 Top-1 경로의
수정량을 계산했다. 3초 forecast revision의 평균/중앙값/p95는 E25 전체 코스
0.381/0.003/1.627m, 9월 전체 코스 0.416/0.002/1.770m였다. 이는 GT 오차·차선 변경 횟수·
조향 jerk가 아니라 예측 갱신 진단이며 실제 속도 변경과 지도 분기 변화도 섞여 있다.

### 현재 지도와 E24 추적 상태의 재생

46절의 실제 ROS tracker 출력 `promotion_ros_v2/messages.json.gz`의 `off/policy`를
현재 v85 지도와 현재 Map-CV 코어에 넣었다. Detector 재추론·coasting 변경 없이
고주로/로터리 및 비차량 구간의 차량으로 분류된 상태를 함께 조사했다.

이 기존 tracker A/B fixture는 `/Ego_topic`의 MGeo 위치를 world 원점으로 쓰면서 header를
`map`으로 명명한다. Lanelet 지도에 직접 넣으면 잘못된 위치가 된다. 새 도구는 명시적인
physical coordinate 계약을 요구하고 `configs/tf/morai_lanelet_static_tf.yaml`의
`morai_map -> map` 변환(+131.559130220, +545.911338186m)을 **한 번만** 적용했다.
공식 START의 알려진 변환으로 단위 검사했다. 원점 정합이 없던 `map_v1`은 채택하지 않았다.
이것은 독립 평가 fixture의 좌표 정합이며 production localization/TF 오류를 발견하거나
수정했다는 뜻이 아니다. 이전 tracker A/B의 동일 world 좌표 기반 상대 오차는 보존된다.

| E24 상태·현재 Map-CV | 고주로 H03 | 로터리 R01 |
|---|---:|---:|
| 차량 관측 | 796 | 693 |
| 지도 경로 생성 | 796 | 665 |
| 낮은 속도/지도 미매칭 fallback | 0/0 | 6/22 |
| 지도 다중 경로 | 258 | 641 |
| Top-2가 같은 시작 lanelet에서 다른 route로 분기 | 74 | 406 |
| Top-2의 시작 lanelet 자체가 다름 | 184 | 235 |
| 최고점 정확한 동점 | 0 | 0 |

로터리 다중 후보에는 **시작 차선의 모호성**과 **같은 시작 차선 이후의 실제 지도 분기**가
함께 있다. 둘을 구분하지 않고 Top-1으로 자르는 방식은 채택하지 않는다.
지도 branch가 열려 있다는 사실만으로 NPC의 실제 출제 route를 확정한 것은 아니다.

### 미래 GT 비교: 순위 개선 여지와 분기 보존

기존 catalog/보간 GT를 동일 좌표로 변환했다. 현재 중심 2m 이내 1:1 대응이 있고 미래
GT의 0.15초 이내 interpolation이 지원되는 관측만 집계했다. 매칭 누락·fallback은 위에서
별도로 보고하고 이 조건부 평균에 0오차로 넣지 않는다. 평가 시각과 ROI 지원에 따라
1/2/3초 분모가 달라진다. 미래 GT는 경로 생성이나 이력 선택에 쓰지 않는다.

| 3초 FDE·동일 조건부 관측 | n | 현재 Map Top-1 평균/중앙값/p95 | 평가용 minFDE-K 평균/중앙값/p95 | 직선 CV 평균/중앙값/p95 |
|---|---:|---:|---:|---:|
| H03 | 142 | 2.335/0.998/8.583m | 2.303/0.983/8.583m | 4.089/3.082/10.178m |
| R01 | 530 | 4.953/2.440/17.106m | 2.444/1.754/6.489m | 8.455/8.067/14.140m |

1초 Top-1/minK 평균은 H03(n=204) 0.991/0.934m, R01(n=611) 1.050/0.799m다.
2초는 H03(n=173) 1.524/1.503m, R01(n=574) 2.596/1.560m다.
3초 minK가 Top-1보다 0.5m 이상 좋은 관측은 H03 2/142, R01 113/530이다.
minK는 미래 정답을 보고 고른 **평가용 oracle**이지 실제 planner가 쓸 수 있는 정책이 아니다.
관측들은 같은 몇 차량의 연속 표본이며 독립 테스트·대회 전체 확률로 해석하지 않는다.

고주로는 현재 map Top-1이 CV보다 유리하고 순위 gap이 작다. 로터리는 map 자체의
장점을 유지하면서 순위/차선 이력을 개선할 여지가 크다. 속도·pose·초기 추정 오차가
섞인 FDE이므로 큰 tail을 모두 잘못된 분기 탓으로 돌리지 않는다.

### 차선 이력 원형의 효과와 한계

새 연구 실행기에서만 이전 시작 lanelet 또는 직접 successor를 보조했다. 현재 거리/yaw
gate 안의 후보 중 최고 score가 이력 후보의 1.25배 이하일 때만 이력 후보를 우선한다.
이력은 0.5초 이내·같은 ID에서만 쓰고 미매칭/저속 시 초기화한다. 명확한 새 후보는 전환한다.
지도에서 사라진 후보를 복원하거나 미래 분기를 삭제하지 않는다.

H03의 시작 lanelet 변화는 23→23회, 직접 successor가 아닌 변화는 2→2회다.
R01은 51→27회, 직접 successor가 아닌 변화는 39→15회였다. 그러나 후자는 실제
차선 변경·겹치는 지도 segment를 포함할 수 있어 전부 오매칭이라고 단정하지 않는다.
첫 원형은 **진단용 시작 lanelet만 선택**하며 발행 route/confidence를 바꾸지 않았다.
따라서 이 수치로 Top-1 FDE나 planner 안전성이 개선됐다고 주장하지 않는다.
새로운 lane-change/분기 GT와 route-level 재생 후에만 실제 predictor 이식 여부를 판단한다.

이 진단을 실제 예측 정확도 개선으로 오인하지 않도록 두 번째 후보도 비교했다.
현재 predictor의 독립 메모리 복사본에서만 이력으로 선택한 시작 후보의 score를 1.25배로
보조하고, 원래 lane 후보와 successor 분기는 삭제하지 않았다. 현재 clustering/6개 cap을
그대로 재사용했다. H03 Top-1 route ID는 3개 관측, R01은 60개 관측에서 달라졌다.
R01에서 route-ID 집합도 17개 관측에서 달라졌으나 동일 미래의 representative 변경이
섞일 수 있어 실제 분기 소실로 단정하지 않는다. 0.05 이상 경로 개수는 같았다.

| R01 동일 미래 GT 관측 | n | 기존 Top-1 평균/p95 | 이력 가중 Top-1 평균/p95 | 0.5m 이상 개선/악화 관측 |
|---|---:|---:|---:|---:|
| 1초 | 611 | 1.050/3.031m | 1.071/3.181m | 10/14 |
| 2초 | 574 | 2.596/8.794m | 2.721/9.819m | 14/19 |
| 3초 | 530 | 4.953/17.106m | 5.079/18.419m | 14/14 |

H03의 조건부 FDE는 그대로였다. R01의 minFDE-K 평균은 1/3초 그대로이고 2초도
1.5595815→1.5595331m의 미소 차이뿐이라 주로 후보의 순위 문제로 보인다.
**시작 lanelet 진동 감소와 더 정확한 예측은 동일하지 않다.** 이전의 잘못된 후보를
붙잡거나 실제 이동/지도 segment 변화를 늦게 따라가는 위험이 있어 이 가중 후보를 기각한다.
관측 이력의 route residual과 실제 lateral motion을 사용하는 후속은 별도 후보이며
이번에 구현·검증됐다는 뜻이 아니다. 이 시험 후보도 실제 publisher나 planner에 넣지 않았다.

O02에는 차량으로 분류된 관측 35개 중 26개가 지도 경로를 얻었다. 실제 정적 장애물
six-asset 장면이라는 기존 catalog와 함께 보면 **지도 위 차량처럼 보인다는 사실은
진짜 차량의 증거가 아니다.** Road prior만으로 Vehicle을 확정·UNKNOWN을 강제 승격하지 않는다.
미매칭을 바로 삭제하지 않고 off-lane/합류/역방향의 기존 Cartesian fallback을 유지한다.

### 산출물·검증·다음 단계

- Root: `artifacts/evaluation/bevfusion/base5_map_path_review_261014`.
- `recorded_v2.json`: 10개 bag 기록과 현재 consumer source/YAML SHA. Runtime 검증은 false다.
- `map_v6.json`: map/projector/alignment/predictor/센서/GT/추적 상태/도구 SHA, 후보 lanelet trace와 baseline/이력 가중 조건부 FDE.
- 최종 SHA256은 recorded_v2 `8a0b129d58ab62096b3e2f9dc048d66abd8a289687466a76e1505de550932443`,
  map_v6 `9670440aced4f4b9f567212191bb65aa19b91476bab1a6148aded7750639d459`다. 반복 수치와 baseline FDE 불변을 대조했다.
- 도구: `audit_base5_predicted_path_selection.py`, `probe_base5_map_lane_history.py`.
- 선택·동점·시간 정합·좌표 변환·fallback·이력·soft prior 비파괴 synthetic 검사 16개와 기존 map core 검사 5개를 통과했다.
- 다음은 시작 차선 score의 시간적 안정화와 실제 분기 의도를 분리한 route-level A/B다.
  팀장님 bag에서 module 실행값·최고점/동점/0.05 후보·관측 age·lanelet/route ID·선택된 충돌 대상과
  Ego 후보 궤적을 확인한다. Prediction 시각화만으로 Top-2가 추월 실패 원인이라고 결론내리지 않는다.
- Production 경로 개수·confidence·planner 안전 조건은 유지한다. 로터리 일괄 Top-1이나
  관측 없이 지도만으로 차량을 새로 만드는 기능은 적용하지 않았다.

<a id="base5-map-cv-candidates-261014"></a>

## 50. 관측 적합도·횡이동 후보의 회귀와 출력 동일 계산 최적화 — 2026-10-14

사용자는 기존 bag의 과적합에 주의하면서 Map-CV 개선을 이어 가도록 요청했다.
GPU·MORAI·ROS master·학습·유료 인스턴스·planning/control을 실행하거나 변경하지 않았다.
원본 bag·완료 데이터·GT catalog·검출 및 tracker 가중치/설정은 보존했다.
예측 정책 후보는 연구 실행기에만 두고 출력 동일 계산 최적화만 Map-CV 소스에 반영했다.
실행 중 node·팀장님 배포본을 재시작하거나 설치 경로를 교체하지 않았다.

### 먼저 동결한 후보와 회귀 묶음

H03/R01의 같은 E24 실제 ROS `off/policy` 상태로 baseline과 세 후보를 비교했다.
입력·GT·좌표 변환과 1/2/3초 조건부 FDE는 49절과 같고 baseline 값은 정확히 재현됐다.
새 sensor 추론이나 GT 재라벨링을 수행하지 않았다.

| 후보 | 변경 요소와 제한 |
|---|---|
| `route_history` | 최근 0.5초 track-state의 시작 차선/한 단계 predecessor 적합도로 **현재** 시작 후보 score를 보조한다. 이전에 고른 lanelet을 우대하지 않는다. 미래 route/GT는 적합도 입력이 아니다. 같은 시작 lanelet의 sibling branch 비율은 보존한다. |
| `convergence3` | 중심선 복귀 시간만 1.5→3초로 바꾼다. |
| `lateral_hermite3` | 3초 중심선 복귀에서 Frenet 횡방향 속도와 Hermite 연결을 시험한다. 최근 이력 fit가 없으면 baseline으로 돌아간다. |

이력 최소 4점·span 0.3초·gap 0.15초·fit RMS 0.4m를 먼저 고정했다.
횡속도 cap 2m/s, history blend 0.5도 사전 고정값이며 대회 동작의 정답이나 최적값이 아니다.
현재 입력은 tracker state이고 coasting을 포함할 수 있어 독립 sensor 관측 이력으로 부르지 않는다.

후보는 case/horizon별 30개 이상 정답 지원, Top-1 및 minK 평균 +3%/p95 +5% 이내의
개발 screening을 거쳤다. 통과 후보 중 case-balanced 2/3초 평균으로 하나만 선택했다.
평균 screening은 배포 승인 조건이 아니며 큰 신규 오차·actor별 회귀는 별도로 확인했다.

다음 네 bag을 후보 비교 전에 regression 용도로 지정했다. 이들은 이미 알려진 과거 기록이며,
이번 후보 선택에 쓰지 않았다는 뜻이지 전 프로젝트에서 untouched test라는 뜻은 아니다.

- `planning_rviz_0915_sensor_local_1.bag`, `_2.bag`, `_3.bag`.
- `260917_134456_full_route_mr_sigma0p10.bag`.

Recorded map/morai_map static TF를 확인하고 현재 v85 좌표로 한 번만 변환했다.
기존 measured-size asset catalog로 BBox-center GT를 구성하고 ambiguous 자산은 추측하지 않았다.
미해결 차량이 Ego 105m 이내에 있는 전체 scoring frame은 제외했다. 9/17은 stride3의
4,802개 중 504개, full cadence의 14,404개 중 1,510개가 제외됐다.
MR heading -1.347°는 catalog pivot 회전 계약으로 사용했으며 이 기록의 절대 heading
정확도를 새로 인증한 것은 아니다. 과거 detector/실행 source도 파일명으로 확정하지 않았다.

주 비교는 고정 message index stride3으로 했다. 선택한 후보를 동결한 뒤 9/17 원래
약30Hz tracking 메시지 전부도 추가 확인했다. Regression 결과에 맞춘 재튜닝을 하지 않았다.
실행기는 후보 source/spec 변경, 다른 preregistration·catalog·비등록 bag을 거부한다.

### 개발 결과와 기각한 횡방향 후보

`convergence3`는 H03 1초 minFDE-K 평균 0.934→0.980m, p95 3.040→3.241m로
screening을 통과하지 못했다. 작은 오차를 늦게 중심선에 복귀시키는 것만으로 개선되지 않았다.
`lateral_hermite3`는 H03 1초 Top-1 평균 0.991→1.124m, R01 1.050→1.150m로 악화했다.
곡선 주행·tracker/localization 잡음이 횡이동 추정과 섞일 수 있다. 실제 차선 변경으로
분리 검증하지 않은 이 벡터를 일반적인 lateral intent로 사용하지 않는다.

`route_history`만 screening을 통과해 별도 네 bag으로 보냈다.

| 3초 FDE·현재 대응/미래 GT 지원 관측 | n | 평균 baseline→후보 | p95 baseline→후보 |
|---|---:|---:|---:|
| H03 개발 | 142 | 2.335→2.335m | 8.583→8.583m |
| R01 개발 | 530 | 4.953→4.848m | 17.106→17.697m |
| 9/15 sensor1 | 1,184 | 3.212→3.119m | 16.546→16.546m |
| 9/15 sensor2 | 490 | 2.703→2.704m | 10.656→10.656m |
| 9/15 sensor3 | 551 | 3.075→3.020m | 11.447→11.140m |
| 9/17 stride3 | 1,941 | 3.625→3.462m | 13.230→12.814m |
| 9/17 full cadence | 5,810 | 3.603→3.462m | 13.185→12.869m |

네 regression의 집계 screening도 통과했다. 그러나 반복 관측은 독립 객체가 아니며,
9/15 세 주행은 같은 환경 반복이라는 제한이 있다. 원거리 recall·planner 안전·E24
실제 대회 PC 성능 인증으로 사용하지 않는다. 관측별 평균뿐 아니라 actor/episode를 함께 본다.

### 평균·p95에 가려진 신규 큰 순위 오차

R01의 3초 관측 530개 중 실제 FDE가 달라진 것은 25개였다. 전체 delta의 중앙값/p90/p95는
모두 0이지만, **변한 25개만** 보면 추가 오차 p90/p95는 20.735/21.742m였다.
5m 초과로 좋아진 관측은 15개, 5m 초과로 나빠진 관측은 7개였다. 최대 신규 추가 오차는
25.651m다. 최대 하나만으로 판단하지 않고 분포·횟수·actor별 평균을 함께 남겼다.

9/15 sensor1·sensor3·9/17 stride3도 3초 추가 오차 >5m가 각각 5/1/11개 있었다.
R01에서 10개 이상 관측이 있는 다섯 GT segment 중 평균 0.1m 이상 개선은 두 segment,
악화는 한 segment였고 해당 악화 segment 평균은 +2.529m(n=25)이었다.
9/17에서는 같은 actor 단위 기준 17개 중 개선 9개/악화 0개였지만 개별 큰 신규 오차는 남았다.
GT segment는 재구성된 key이고 simulator의 authoritative actor ID로 보증하지 않는다.

모든 `route_history` 관측의 minFDE-K는 baseline과 정확히 같았다. 즉 올바른 기하 후보가
새로 사라진 것이 아니라 일부 프레임의 **순위**가 잘못 바뀌었다. 이는 model 내부 후보의
보존이지 planner의 0.05 gate까지 동일하다는 뜻은 아니다. 실제 planner의 유효 경로
선택·attention 영역·충돌 반응을 이번 offline 검사가 인증하지 않는다.

`risk_v2/development_worse.png`와 `development_better.png`는 같은 GT segment의
악화와 개선을 모두 보여 준다. 미래 GT는 그림/평가에만 썼다. 큰 새 분기 오차는
연속 segment에서 대체로 소수 관측에 발생하므로 전체 평균·p95만으로 감추지 않는다.
관측 없는 미래 의도는 과거 0.5초의 거리 적합도만으로 확정할 수 없다.
**집계 screening 통과와 배포 가능은 다르다.** 이 순위 후보도 production에 적용하지 않는다.

### 유지한 개선: 출력 동일 경로 계산 최적화

기존 `predict`는 미래 pose마다 route length와 center/tangent를 구할 때 모든 segment를
반복 순회했다. 경로별 `_RoutePolyline`에 길이와 누적 arc index를 한 번 만들고
이진 검색으로 표본을 찾도록 바꿨다. 시간 모델·경로 생성·confidence·clustering·horizon은
그대로다. 지속적인 motion cache나 새 프레임 보정은 없다.

작은/중복 segment, 끝점, 경계점, 단일 점, 무작위 polyline에서 기존 scalar 함수와
정확히 같은 결과를 확인했다. 실제 selected 입력은 네 개발 bag의 vehicle 상태와
별도 네 bag stride3, 9/17 full cadence의 **14,546개 vehicle 관측**이다.
미해결 GT frame도 계산 동등성 검사에서는 제외하지 않았다.
비교한 **26,339개 경로·790,170개 미래 pose**의 위치·yaw·경로 순서·confidence가 정확히 같았다.

아래는 순서를 번갈아 실행한 로컬 CPU의 **차량 한 관측당 지도 예측 계산** 비용이다.
ROS callback/LAN/GPU/전체 pipeline latency가 아니고 초기 10관측은 비용 집계에서 제외했다.

| 입력 | 평균 scalar→index | 중앙값 scalar→index | p95 scalar→index |
|---|---:|---:|---:|
| H03 | 1.658→0.653ms | 0.459→0.448ms | 5.925→1.714ms |
| R01 | 4.054→1.244ms | 4.332→1.300ms | 6.738→1.920ms |
| 9/17 full cadence | 3.135→1.031ms | 2.474→0.903ms | 6.522→1.973ms |

입력에 단순한 직선 경로가 많으면 중앙값 개선은 작을 수 있다. 분기/곡선이 많은 경로에서
반복 계산 감소가 컸다. 같은 출력의 계산 최적화만 소스에 반영하고 30Hz node의 실제
callback 처리/입력 age 효과는 팀장님 환경에서 별도로 확인한다.

### 산출물·검증·후속

- Root: `artifacts/evaluation/bevfusion/base5_map_cv_candidates_261014`.
- `development_v1/preregistered.json`, `development.json`: 고정 후보·선택·기준과 개발 matrix.
- `reserved_pack_v1.json.gz`, `regression_v2.json`: 별도 네 bag의 source/GT/TF 근거와 후보 회귀.
- `reserved_pack30_v1.json.gz`, `regression30_v1.json`: 9/17 원래 cadence의 추가 확인이며 네 bag 전체 gate를 대체하지 않는다.
- `risk_v2/risk.json` 및 PNG/JSON: 전체/변경 관측 delta, actor·episode, 양방향 대표 사례.
- `geometry_v1.json`: 원본 sampler SHA·수정 source/input SHA·전 관측 정확한 동일 출력·paired 비용.
- 핵심 receipt SHA256: development `e6aa2d8fd68d851dccc9a8e0fb05b80201227e1d92f2de431a43df941a4b7f9d`,
  regression_v2 `6c1b06b7488c40763a3ad3b67095cb578eda8d944acc1f0cda6cfb91ecc2cffa`,
  geometry_v1 `fb35f6b504ee81c5ee9d9be8f05fcbfde3abadaa47d16d841ef25d58e4342abe`.
- Map core/node·기존 prediction/motion policy/launch와 새 후보/risk/선택 검사 40개를 통과했다.
  노드 메시지 검사에는 ROS1 호환 `builtin_interfaces`가 있는 planning overlay가 필요하다.
  Base overlay만으로 발생한 import 실패는 overlay를 올바른 순서로 source한 뒤 해소됐다.
  Python 구문·문서 링크·whitespace·반복 baseline/모든 FDE row의 불변도 확인했다.
- 원본 Map-CV source SHA는 `f15dda428c1b2472903f20fe2d8aea59c0923620f149fbb7ce81ec78d8aae968`이다.
  현재 기준은 scalar helper semantics를 유지하는 새 arc index 구현이다.
- 새로운 제안은 별도 가설로 기록하고 이 regression 묶음에 수치를 계속 맞추지 않는다.
  신규 cut-in/진출/재획득 bag 또는 팀장님 실제 planner 기록으로 영향과 불확실성을 확인한다.
- 다음 생성 분포 검토로 넘어갈 수 있다. Map-CV의 새로운 순위 정책·planner 안전거리·
  detector threshold·tracker lifecycle 변경은 이번 계산 최적화에 포함되지 않는다.

<a id="base5-next-training-cpu-261015"></a>

## 51. 다음 학습 compile OFF·nearest 공통 CPU 준비 — 2026-10-15

사용자가 새 학습의 사전 준비만 승인했다. 완료 E24의 compiled 학습 config·동결 Vast 실행기와
resume 계약·가중치·원본 데이터·production 소스는 변경하지 않았다. 새 `bevfusion_base5_fine_512_depth25_next_training.py` 설정과
`prepare_base5_next_training.sh`를 분리했다. train/val은 같은 nearest_positive/sorted view-transform을
사용하고 compile OFF·현재 3클래스·upright 7D·JPEG·의미 ROI·공간 격자·depth 간격을 유지한다.
이 선택은 compile OFF의 정확도 우위를 입증한 결과가 아니라 다음 실험의 명시적인 기준이다.

검증 환경은 기존 팀 Docker `ajounice/bevfusion:team-cu128-v1`, Torch 2.7.1+cu128,
MMEngine 0.10.7이다. GPU 장치를 전달하지 않고 CUDA_VISIBLE_DEVICES를 비웠으며 network none,
CPU 4개·RAM 8GiB 제한과 저장소/데이터 read-only mount를 사용했다. 결과만 별도 경로에 저장했다.

- 신규 config 8개·준비 실행기 15개, 기존 depth raster 16개·direct runner 5개,
  동결 oneclick 19개·배포 bundle 8개 등 서로 다른 71개 검사가 통과했다.
- 처음 기존 모델 등록 회귀는 호스트의 미빌드 submodule이 Docker 설치본을 가려 extension import가
  실패했다. PYTHONPATH에서 해당 경로를 제거해 기존 빌드본을 사용하자 16+5개가 통과했다.
  CUDA 설치·재컴파일·GPU forward는 수행하지 않았다.
- 입력은 `artifacts/handoff/base5_sunny13_jpeg90_pedestrian_merged_v1_261011`이다.
  train 4,255장/9,350 GT, val 745장/1,702 GT, test 0장을 대조했다. 모든 payload 해시를 확인하고
  전체 11,052 GT의 7D·유한값·양수 크기·원래 클래스/animal 매핑·장면/group split 계약을 검사했다.
- 강도 검사는 train/val에서 고르게 16개씩 뽑은 32개 cloud 표본이다. 유효 return 강도 min/max는
  0.0/1.0이었다. 학습 입력에 ROS용 ÷255를 적용하지 않았으며 invalid-return 판정도 정확한 XYZ-zero
  및 nonfinite 필터와 맞췄다. 모든 cloud의 강도 분포를 전수 인증한 결과는 아니다.
- 기존 호환 초기화 `base5_nuscenes_full_xy_metric_v1_261011.pth`의 SHA256은
  `a786b70ec7e639587da9c075fa7db9d3e96bf522b56590162c3134be7197b977`로 기록했다.
  CPU 단계에서는 checkpoint shape/성능을 새로 인증하지 않는다.

산출물은 `artifacts/evaluation/bevfusion/base5_next_training_cpu_261015/plan_v3/`의
`checked_model_config.py`, `next_training_preflight.json`이다. 최종 plan SHA256은
`988377cc55a746ec7fd8cf17ef5ec7fda123154760fb785d1307f64893b1aabc`다.
실제 direct runner의 run_manifest나 GPU 자격 검증을 대체하지 않으며
직접 학습을 시작하지 않는다. 기록한 후속 prepare 명령은 원거리 AP/W&B 진단 확장을 유지하고
`--run`은 포함하지 않는다. 재사용·실행 경계는 [새 학습 준비 런북](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#next-training-cpu)을 따른다.
GPU 초기화 strict load·forward/backward 비용·재개·W&B/Drive 백업과 최종 학습 예산 확정은 별도다.

<a id="full30750-e2-local-bags-261021"></a>

## 52. 전체 30,750장 E2와 production E24의 로컬 bag 비교 완료 — 2026-10-21

사용자는 전체 데이터 학습의 E2를 로컬로 가져와 기존 rosbag으로 비교하도록 승인했다.
두 모델 모두 새로 추론하고 동일 입력·실행 계약을 확인한 뒤 비교했다. 과거 E24 저장
추론값과 새 E2 실행값을 무조건 직접 섞지 않는다. 아래에 준비 조건과 실제 결과를 함께 남긴다.
원격 학습은 계속 유지하고 로컬의 새로운 출력 디렉터리에 E24·E2를 각각 추론한다.
이번 비교를 위해 MORAI·ROS master·detector/tracker/planner 노드를 추가로 실행하지 않는다.

### 가중치와 복원 계보

- 비교 후보: 전체 30,750장 E2 full checkpoint, **522,174,725바이트**,
  SHA256 `83005296010995b4d9a42dfb09675c1f5400c0ba4aad29582f7f47c7394f8f56`.
  계승 contract는 `97760ca4680d77451ecb0aee86269711176bc10b7f535d4f01c5e19fbf8f5998`,
  epoch 2 / iteration 13,068이다. 학습 재개 가능한 optimizer·scheduler 등의 상태가 포함돼
  있어도 추론에서는 `state_dict`만 strict load한다. full 파일을 단순 모델 파일로 착각하지 않는다.
- 기준 후보: 기존 Sunny13 production E24,
  SHA256 `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746`.
- E2는 Drive의 내용 주소형 16파트를 받아 파트별·전체 SHA로 복원한다. 기존 full 백업의
  실제 readback 근거는 [학습 런북의 E2 후속](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-speed-repair-261021)과
  `artifacts/evaluation/base5_speed_repair_261021_v1/continuation_v2/logs/drive_backup.jsonl`이다.
  원격 학습 디렉터리나 production alias를 덮어쓰지 않는다.

### 같은 입력·같은 추론 조건

기존 O02_164012 장애물, P01_163434 보행자, H03_162301 고주로 추월,
R01_163034 로터리 **네 dev bag에서 고정해 둔 연속 구간 1,570프레임 전부**를 두 모델에 넣는다.
정확도는 기존 미확정 GT 제외 규칙에 따라 **1,413프레임**만 집계한다.
분모는 O02 463 / P01 269 / H03 355 / R01 326이며, H03의 미확정 차량으로 제외한
157프레임을 모델 score에 따라 다시 선택하지 않는다.

같은 현재 nearest runtime config·512×1056·5cm/20cm·depth 0.25m와 strict full-model
loading을 사용한다. **AMP·camera 병렬화 ON / 추론 compile OFF**로 통일하며,
원본 JPEG bytes와 ROS XYZI를 유지한다. `ros255` 입력 강도는 모델 입력 복사본에서
정확히 한 번 ÷255한다. 학습 config의 compile 값으로 추론 compile을 자동 켜지 않는다.
초기화 가중치만 불러오는 비교가 아니며, 학습 optimizer를 step하거나 checkpoint를 수정하지 않는다.

팀 Docker image ID는 `13488ccac149f41b070e618db5db91bfa3bf2d3d1a6de3dac80aa2e2c948ec46`이다.
저장소는 read-only, 새 산출물 폴더만 writable로 연결하고 container network는 끈다.
로컬 MORAI가 별도로 이미 실행 중이므로 GPU/CPU 경합이 포함될 수 있다. 이번 실행 시간은
실제 대회 전용 PC의 추론 latency나 ROS 전체 입력 age를 인증하는 값이 아니다.

GT는 기록된 Ego/Object status의 보간과 감사된 asset catalog·BBox offset으로 만든
**근사 GT**다. SaveSensorData의 동기 native annotation 평가가 아니며, 기존 heading·ROI·
미확정 actor 제외 계약을 그대로 따른다. 이미 여러 개발 분석에 사용한 bag으로서 untouched
test나 실제 대회 4090 24GB 풀스택 성능 인증으로 부르지 않는다. E2 native val의 AP와
이 AMP·ROS 후처리 bag AP도 서로 다른 평가 계약으로 구분한다.

공통 입력은 유지하고 모델만 바꾸는 비교이나, 두 모델의 학습 데이터·업데이트 수·학습
compile/depth-raster 조건이 모두 같은 단일 변수 실험은 아니다. 결과를 보강 데이터 하나의
효과나 최종 E12 성능으로 해석하지 않는다. 후속 결과에서 거리별·클래스별 검출 이득과
오검출/누락을 함께 확인한 뒤 threshold·tracker 비교 여부를 결정한다.
**현재 production E24·alias·운영 threshold는 승격 또는 변경하지 않는다.**

산출물 root는 `artifacts/evaluation/bevfusion/full30750_e2_rosbag_261021_v1/`이다.
`checkpoint_parts/`에 Drive metadata·원본 계보를, `E2_full.pth`에 복원한 전체 상태를 보존한다.
새 기준 출력은 `e24_fresh_v2/`, E2 출력은 `e2_fresh/`로 분리한다. 초기 상대경로 구성 실패로
생긴 빈 출력 디렉터리는 성공 결과로 세지 않고 보존한다.
### 실제 결과와 해석

10/21 **20:37 KST**까지 두 모델의 1,570프레임 strict-load 추론과 1,413프레임 공통
평가를 완료했다. `comparison.json`의 `complete=true`와 동일 `gt_identity`를 확인했다.
ROI 안 GT 관측 수는 V/P/O **1,028 / 269 / 2,489**다. 이는 연속 영상에서 반복된 관측 수이며
서로 독립적인 객체 수가 아니다. 거리 구간은 원점으로부터 XY 방사 거리이며 전체 ROI는
직사각형이므로 모서리의 방사 거리 100m 이상 관측까지 포함할 수 있다.

아래 AP는 동일한 중심 XY 거리 gate **0.5/1/2/4m의 평균**이다. 현행 고정 threshold의
recall이나 native val의 AP와 혼동하지 않는다. 클래스 GT가 없는 구간은 `null`로 보존한다.

| 비교 항목 | GT 관측 수 | E24 AP(%) | 전체 데이터 E2 AP(%) |
|---|---:|---:|---:|
| 전체 ROI mAP | V/P/O 위 분모 | 53.65 | 51.30 |
| 전체 ROI Vehicle | 1,028 | 69.34 | 69.29 |
| 전체 ROI Pedestrian | 269 | 57.15 | 46.58 |
| 전체 ROI Obstacle | 2,489 | 34.45 | 38.03 |
| Vehicle 0~30m | 350 | 91.35 | 92.35 |
| Vehicle 50~80m | 276 | 51.27 | 54.49 |
| Vehicle 80~100m | 91 | 9.84 | 0.00 |
| Pedestrian 0~30m | 113 | 79.98 | 83.03 |
| Pedestrian 30~40m | 29 | 54.35 | 68.14 |
| Pedestrian 40~50m | 80 | 65.46 | 8.52 |
| Obstacle 0~10m | 292 | 76.27 | 79.17 |
| Obstacle 10~20m | 303 | 82.61 | 86.54 |
| Obstacle 20~30m | 302 | 79.21 | 80.99 |
| Obstacle 0~30m | 897 | 81.34 | 84.81 |
| Obstacle 30~50m | 590 | 13.91 | 12.79 |
| Obstacle 50~80m | 702 | 11.06 | 20.12 |

**근거리 세 클래스의 AP는 개선됐지만 E2가 전반적으로 production보다 낫다는 결과는 아니다.**
장애물의 센서 지원이 상대적으로 충분한 GT(5점 이상, 745회)에서는 현행 score 0.1·1m 매칭
recall이 **84.56%(630/745)→95.17%(709/745)**로 올랐다. 5점 미만 1,744회에서는
27.18→27.92%여서 희소한 원거리 객체까지 해결했다고 볼 수 없다. 30~50m 장애물도 미개선이다.

현행 V/P/O **0.40/0.10/0.10**을 유지한 전체 ROI의 고정 1m 매칭 결과는 다음과 같다.
`unmatched`는 카탈로그 GT와 대응하지 못한 출력 수이지 의미적으로 확정한 유령 객체 수가 아니다.

| 클래스 | Precision E24→E2(%) | Recall E24→E2(%) | Unmatched E24→E2 |
|---|---:|---:|---:|
| Vehicle | 88.26→90.71 | 71.69→68.39 | 98→72 |
| Pedestrian | 91.51→72.53 | 72.12→62.83 | 18→64 |
| Obstacle | 88.89→87.75 | 44.36→48.05 | 138→167 |

근거리 장애물도 recall만 보면 90.08→93.31%지만 unmatched는 84→134로 늘었다.
보행자의 추가 unmatched 46회 중 O02 장애물 장면이 27회(10→37), P01 장면이 16회(4→20)다.
따라서 보행자 문제를 원거리 누락 하나로만 설명하지 않는다. 배경 누락·클래스 혼동·중복 후보는
서로 다른 원인이며 이 수치만으로 모두 확정하지 않는다.

80~100m 차량의 E2 AP=0은 **검출 출력 0개라는 뜻이 아니다.** score floor 0.1에서
해당 band의 2m·4m 매칭은 각각 10/91회다. 최대 recall 10.989%가 현행 AP의
첫 집계 recall 격자 11%에 조금 못 미쳐 AP가 0이 된다. 동일 ROI 선매칭에서도 같은 계수여서
이번 0은 거리 경계 유실 때문이 아니다. 현행 차량 score 0.4에서는 1m 매칭이 **9→3회**,
2m 매칭이 **12→4회**다. 단순히 AP 표기만 나쁜 것이 아니라 이 구간의 검출량도 적다.
50~80m 차량은 AP가 올라도 현행 score 0.4·2m recall은 **69.57→58.33%**로 낮아졌다.
낮은 score까지 포함하는 AP와 고정 운영점은 다르며, 이 결과로 운영 threshold를 바로 낮추지 않는다.

연속 누락은 case별 실제 센서 주기와 원래 frame index로 계산했다. P01의 40~50m 구간은
score 0.1·1m 검출 **58/80→20/80회**다. 별도로 저장된 30~50m·2m 매칭 기준 최대
연속 누락은 **0.85→4.43초**다. 이 보행자 구간은 LiDAR 지원이 모두 5점 미만인 동일 actor의
반복 관측이므로, 80명의 독립 보행자나 모든 40~50m 장면의 성능으로 일반화하지 않는다.
근거리 장애물도 2m 기준 최대 누락이 0.71→1.26초여서 AP 상승만으로 안정성이 좋아졌다고
단정하지 않는다. 전체 case를 합친 단일 주기의 누락 시간은 잘못 해석하지 않도록 `null`로 둔다.
이 누락은 detector 관측의 누락이며 tracker coasting이나 planner 동작을 재현한 값은 아니다.

현재 판단은 **E24 유지, 예정된 전체 학습 지속, 후기 checkpoint에서 동일 bag 재평가**다.
E2는 아직 중간 checkpoint이며 두 학습의 데이터·총 업데이트 수 등도 다르므로 이 한 번으로
새 데이터가 실패했다거나 추가 수집이 즉시 필요하다고 결론 내리지 않는다. 반대로 native val
상승만으로 bag의 40~50m 보행자·80~100m 차량 문제를 해결했다고 주장하지 않는다.

### 재현 산출물과 검증

공통 비교기는 `perception/detection3d/bevfusion/tools/summarize_base5_frozen_bag_comparison.py`다.
동일 infos·GT·config·AMP·강도 adapter·최적화·GPU·추론 source hash를 검사하고, 모델별
고정 제외 규칙·GT 지원점 수·자산을 대조한 뒤 AP와 고정 운영점을 계산한다. 25개 CPU 회귀 검사는
통과했다. `git diff --check`도 통과했다. context 검사에서는 기존 세 current/index 문서의
줄 수 초과만 남았고, 이번 링크·anchor·과거 snapshot 변경 오류는 없었다.

산출물 root 아래 `comparison.json`에 수치·분모·runtime receipt·SHA를,
`e24_fresh_v2/receipt.json`과 `e2_fresh/receipt.json`에 strict-load 결과를 보존했다.
`visuals/`의 네 비교 화면은 **모델 출력 전에 GT 조건으로 고른 네 시점**이다. 4카메라 원본에
E24(주황)·E2(초록)·카탈로그 GT 문맥(청록)을 투영했고, 원본 JPEG는 수정하지 않았다.
장면 전체의 평균이나 최악 사례를 대표한다고 부르지 않으며, source JPEG와 결과의 SHA는
`visuals/receipt.json`에 남겼다. 이 시험 때문에 가중치·기존 입력·production·원격 학습을 변경하지 않았다.

<a id="full30750-e3-local-bags-261022"></a>

## 53. 중단 뒤 보존된 E3의 native val·동일 bag 비교 — 2026-10-22

사용자가 결제 누락으로 학습이 중단됐다고 알리고 E3 복원·val·bag 비교를 요청했다.
기존 Vast direct SSH는 연결 거부였으므로 인스턴스를 시작하거나 학습을 재개하지 않았다.
Drive에 완료된 E3 full 백업이 있어 16파트·전체 SHA를 확인해 새 로컬 경로로 복원했다.
파일은 **523,673,221바이트**, SHA256은
`96b31da0f475c2f081d133ccf26f219ffae16e9eb203f43d859f5ca8eb41f5f4`다.
CPU load로 **epoch 3 / iteration 19,602 / contract `97760ca4680d77451ecb0aee86269711176bc10b7f535d4f01c5e19fbf8f5998`**를 확인했다.
원본 checkpoint·optimizer 상태를 수정하지 않았으며 production은 E24다.

### Native val: 이미 완료된 같은 4,614장 기록 확보

W&B를 읽기 전용으로 조회해 E2·E3의 완료된 val 행과 공통 scalar 836개씩을 보존했다.
두 epoch 모두 6환경 각 750장과 보강 114장, GT V/P/O **3,972 / 2,850 / 3,685**가 같다.
기존 batch 1·FP32·compile OFF validation 결과이며 로컬에서 native val을 새로 재추론한 것은 아니다.
epoch 3 val의 W&B `_step=655`, 바로 앞 epoch-boundary 행 `_step=654`의 train global iteration은
19,602다. Val 행의 `global_step=3`은 epoch 축이지 train iteration 3이라는 뜻이 아니다.

E3 full checkpoint는 E3 val 전에 저장돼 내부 `message_hub`의 마지막 val은 **E2**다.
이를 E3 지표로 읽지 않으며 아래 수치는 W&B의 명시적인 epoch 3 history 행에서 가져왔다.

| 같은 native val AP(%) | E2 | E3 |
|---|---:|---:|
| mAP | 90.54 | 89.45 |
| Vehicle | 95.75 | 95.90 |
| Pedestrian | 94.33 | 93.31 |
| Obstacle | 81.55 | 79.13 |
| Pedestrian 40~50m | 82.62 | 84.54 |
| Vehicle 80~100m | 69.84 | 73.98 |
| Obstacle 20~30m | 80.06 | 77.43 |
| Obstacle 30~50m | 30.04 | 26.60 |

전체 native mAP 기준으로 E3는 E2보다 1.10pp 낮아 새 best가 아니다. 두 원거리 지표의 상승을
전체 모델의 일방적인 개선으로 해석하지 않는다. 기존 E24 자체 val 745장과 이 표를 직접 비교하지 않는다.
W&B는 `crashed`지만 E3 train·val 완료와 뒤따른 **E4 일부 / 마지막 기록 train iteration 22,642**는
별도로 확인됐다. 따라서 완료 E3까지 유실됐다고 보지 않는다. 결제 원인은 사용자 설명이며,
W&B 상태만으로 프로세스 종료 원인이나 현재 인스턴스 상태를 독립 인증하지 않는다.

### Rosbag: 같은 입력을 로컬에서 새로 추론

52절의 고정 입력·config·GT·후처리·AMP/병렬 ON·compile OFF·강도 한 번 ÷255를 유지했다.
E3를 strict load해 같은 **1,570프레임**을 로컬 5060 Ti에서 새로 추론했고,
같은 미확정 구간을 제외한 **1,413프레임**으로 E24/E3와 E2/E3 두 쌍을 평가했다.
공통 `gt_identity`는 `6759d2010372fc3fff11034183325e1caf993eb9d0853a8cb08538c062d1a446`이며,
기존 두 모델의 저장 예측과 runtime·source·input SHA가 모두 일치했다.

| 같은 bag AP(%) | E24 | E2 | E3 |
|---|---:|---:|---:|
| 전체 ROI mAP | 53.65 | 51.30 | 51.73 |
| 전체 ROI Vehicle | 69.34 | 69.29 | 67.91 |
| 전체 ROI Pedestrian | 57.15 | 46.58 | 47.41 |
| 전체 ROI Obstacle | 34.45 | 38.03 | 39.85 |
| Vehicle 50~80m | 51.27 | 54.49 | 55.75 |
| Vehicle 80~100m | 9.84 | 0.00 | 1.61 |
| Pedestrian 0~30m | 79.98 | 83.03 | 81.72 |
| Pedestrian 30~40m | 54.35 | 68.14 | 74.41 |
| Pedestrian 40~50m | 65.46 | 8.52 | 18.59 |
| Obstacle 0~30m | 81.34 | 84.81 | 82.64 |
| Obstacle 20~30m | 79.21 | 80.99 | 79.03 |
| Obstacle 30~50m | 13.91 | 12.79 | 15.90 |
| Obstacle 50~80m | 11.06 | 20.12 | 26.63 |

Native val과 bag은 정답·입력·정밀도·server floor/NMS 계약이 다르며 숫자의 직접 차이를 개선량으로
사용하지 않는다. AP는 중심 XY gate 0.5/1/2/4m 평균이고 고정 threshold recall과도 구분한다.
이전과 같은 카탈로그·Ego 보간 근사 GT, 연속된 소수 actor 반복 관측이라는 한계도 유지한다.

### 취약 구간의 score 회복과 운영점

같은 보행자 40~50m 80관측과 차량 80~100m 91관측을 고정하고,
저장된 raw 선택 query의 낮은 score와 server floor/NMS 뒤 결과를 따로 분석했다.
낮은 raw 후보가 정답 근처에 있다는 것 자체를 운영 TP로 세지 않는다.

| 고정 2m 매칭 조건 | E24 | E2 | E3 |
|---|---:|---:|---:|
| Pedestrian 40~50m, score≥0.1 | 58/80 | 20/80 | 33/80 |
| 그중 전방 | 50/66 | 9/66 | 20/66 |
| 그중 후방 | 8/14 | 11/14 | 13/14 |
| 초기 약 44m 전방 50관측 | 40/50 | 0/50 | 10/50 |
| Vehicle 80~100m, score≥0.1 | 26/91 | 10/91 | 14/91 |
| Vehicle 80~100m, 현행 score≥0.4 | 12/91 | 4/91 | 4/91 |

보행자 80관측의 정답 2m 이내 최고 raw score 중앙값은 **E2 0.0388→E3 0.0826**으로 올랐다.
E2에서 놓쳤던 관측 14회가 회복되고 1회가 새로 탈락해 순증 13회다. 이 band의 최대 연속
누락은 4.43→1.13초이며, 예측 중심이 이 band에 있는 catalog-relative 미대응은 13→13회다.
단일 Man1의 반복 관측이며 대부분 0~2 LiDAR 점이라는 조건을 전 거리 보행자로 일반화하지 않는다.

차량은 score≥0.1에서 5회 회복·1회 탈락으로 순증 4회지만,
현행 score≥0.4의 동일 4관측은 그대로다. 낮은 후보의 일부 상승이 운영 원거리 recall
회복까지 도달하지 못했다. K5의 score≥0.1 대응은 22→8→9/66, Niro는 4→2→5/18,
Nexo는 세 모델 모두 0/7이다. 서로 다른 독립 차량 91대나 모든 도로의 성능이라고 부르지 않는다.

전체 ROI의 고정 1m 운영 결과에서도 E2→E3 Vehicle recall은 **68.39→64.11%**,
Pedestrian은 **62.83→66.91%**, Obstacle은 **48.05→49.18%**다.
미대응 출력은 각각 **72→73 / 64→76 / 167→158**회다. 보행자의 특정 band 회복과
전체 ROI 미대응 증가는 동시에 존재하므로, 한 지표로 안전성·tracker 안정성을 인증하지 않는다.

현재 결정은 **production E24 유지, native best E2와 마지막 완료 E3를 모두 보존**이다.
이번 요청으로 학습·인스턴스·W&B run을 재개하거나 threshold·tracker를 바꾸지 않았다.
E3가 일부 취약 구간에서 회복된 사실은 확인했지만, E12 성능이나 향후 회복을 보장하지 않는다.

### 산출물과 검증

새 root는 `artifacts/evaluation/bevfusion/full30750_e3_rosbag_261022_v1/`다.
`checkpoint_parts/`·`E3_full.pth`에 SHA 복원한 원형 전체 상태를 보존하고,
`native_val_wandb.json`에 E2/E3 원본 val 행·정밀 scalar·분모·epoch/step 연결·조회 시각을 남겼다.
해당 W&B receipt SHA는 `6a987a45ba2bdbf78030949e470d731a5bd41444abf45db32878eff1d38d7b8d`다.
`e3_fresh/receipt.json`과 `predictions.json.gz`, `comparison_E2_E3.json`,
`comparison_E24_E3.json`, `focused_scores.py`·`focused_scores.json`이 새 bag 근거다.
입력·E3 checkpoint·과거 예측의 SHA 전후 불변, 동일 제외 분모와 이전 실패 집합 46/17개를 확인했다.
기존 비교·거리·quick CPU 회귀 25개와 `git diff --check`를 통과했다.
실제 ROS queue·tracker·planner·대회 4090 전용 latency를 이번 평가로 인증하지 않는다.

<a id="full30750-e10-local-bags-261023"></a>

## 54. 전체 데이터 E10 회수와 동일 bag 재비교 — 2026-10-23

사용자가 후기 가중치를 다시 가져와 비교하도록 요청했다. 선택 시각인 **08:47 KST**에
원격은 E11 학습 중이었고, 최신 완료·native 전체 best가 E10이었다. 따라서 이번 비교는
E10으로 고정했다. E11 이후를 추가 선택하거나 학습·threshold·production을 바꾸지 않았다.

### 가중치와 고정 실행 계약

- `best_val_center_mAP_epoch_10.pth`는 **213,315,455바이트**, SHA256
  `6e489870f59ad42b940b20ec7be2e0b0712406c78588f24cb1abf7af28eecae7`다.
  원격에서 stat 전후 불변·전체 SHA와 완료된 epoch 10 val receipt를 확인했다.
  Drive 7개 part·PARTS manifest·전체 SHA·ZIP CRC를 확인해 로컬 `E10_best.pth`로 복원했다.
  수령·결합 검증은 약 14.3초였다. CPU load에서 **epoch 10 / iteration 65,340**,
  574개 state tensor와 optimizer 미포함을 확인했다. **Weights-only이며 resume용 full이 아니다.**
- 원형과 같은 Docker image·5060 Ti·Torch 2.7.1+cu128·runtime config·추론 script·input SHA,
  AMP/병렬 ON·compile OFF·TF32 OFF·강도 정확히 한 번 ÷255를 유지했다. 52·53절의
  고정 4개 bag **1,570프레임**에 strict-load 추론하고 같은 **1,413프레임**을 평가했다.
  GT identity `6759d201…a446`, 제외 H03 157프레임과 V/P/O 1,028/269/2,489관측은 동일하다.
  기존 E24/E2/E3 저장 예측은 runtime·GT·source/hash gate를 통과한 뒤 재사용했다.
- 이미 3클래스로 작성한 bag infos에는 원형 `MoraiBase3Dataset` adapter를 명시했다.
  학습용 animal provenance가 없는 bag에 merged-native adapter를 강제로 적용하지 않았다.
  모듈 경로·adapter·JIT 임시 경로 준비 중 발생한 세 실패와 빈 출력은 별도 보존하고
  완료 추론으로 세지 않았다. 최종 추론의 전후 입력·가중치·기존 예측 SHA는 불변이다.

### 같은 native val 4,614장: 학습 진행에 따른 회복

원래 W&B run의 완료 E2~E10 val 행을 읽기 전용으로 수령했다. 프레임/GT 관련 76개 계수는
전 epoch에서 같았고, 아래는 FP32·compile OFF 학습 validation의 원래 기록이다.
로컬에서 native val을 재추론하거나 구 E24의 다른 745장 val과 직접 비교하지 않았다.

| Native val AP(%) | E2 | E3 | E10 |
|---|---:|---:|---:|
| mAP | 90.54 | 89.45 | 93.83 |
| Vehicle | 95.75 | 95.90 | 97.29 |
| Pedestrian | 94.33 | 93.31 | 96.49 |
| Obstacle | 81.55 | 79.13 | 87.72 |
| Obstacle 20~30m | 80.06 | 77.43 | 86.14 |
| Pedestrian 40~50m | 82.62 | 84.54 | 88.27 |
| Vehicle 80~100m | 69.84 | 73.98 | 79.22 |

E10 전체 mAP는 E8보다 약 **0.019pp** 높아 후기 전체 개선 폭은 작아졌다.
개별 클래스/구간 best epoch는 서로 다르다. 6환경 E10 mAP는 93.51~94.45%이며
각 환경 모두 E3보다 높지만 배치 차이가 있어 날씨만의 인과 효과로 부르지 않는다.
전방 80~100m 차량은 **56.17%**, 후방은 **96.11%**(각 GT 134개)로 격차가 남았다.

### 같은 개발 bag의 정확도와 운영점

아래 AP는 이전과 같은 중심 XY gate 0.5/1/2/4m 평균이다. Native val·검출 확률·tracker
성능과 혼용하지 않는다. 전체 ROI와 거리별 GT는 52·53절의 원형 근사 GT다.

| 동일 bag AP(%) | 기존 E24 | E2 | E3 | E10 |
|---|---:|---:|---:|---:|
| 전체 ROI mAP | 53.65 | 51.30 | 51.73 | 60.53 |
| Vehicle | 69.34 | 69.29 | 67.91 | 72.32 |
| Pedestrian | 57.15 | 46.58 | 47.41 | 66.93 |
| Obstacle | 34.45 | 38.03 | 39.85 | 42.35 |
| Obstacle 0~30m | 81.34 | 84.81 | 82.64 | 87.68 |
| Obstacle 20~30m | 79.21 | 80.99 | 79.03 | 82.44 |
| Obstacle 30~50m | 13.91 | 12.79 | 15.90 | 18.20 |
| Pedestrian 40~50m | 65.46 | 8.52 | 18.59 | 74.41 |
| Vehicle 50~80m | 51.27 | 54.49 | 55.75 | 57.56 |
| Vehicle 80~100m | 9.84 | 0.00 | 1.61 | 16.74 |

공통 운영 threshold V/P/O **0.40/0.10/0.10**·1m 매칭에서도 E24→E10 recall은
V 71.69→75.68 / P 72.12→82.53 / O 44.36→50.58%다.
카탈로그 상대 미대응 출력은 V **98→104**, P **18→19**, O **138→105**회다.
이를 의미적으로 확정한 유령 객체 수로 부르지 않는다. 현재 운영점을 바꾸지 않아도
전반적인 회복이 확인됐지만 모든 클래스의 미대응이 감소한 것은 아니다.

40~50m 보행자 80관측에서 score≥0.1·2m 대응은 **E24/E3/E10 58/33/71회**다.
전방 66관측에서는 50/20/59회, 초기 전방 50관측은 40/10/49회이며 raw 최고 score 중앙값은
0.291/0.083/0.377이다. 최대 연속 누락도 **0.854/1.134/0.266초**로 줄었다.
E24 대비 18관측을 복구하고 5관측을 잃었다. 단일 Man1의 반복 관측이며 LiDAR 0~2점인
이 한 장면의 회복을 모든 원거리 보행자로 일반화하지 않는다.

80~100m 차량은 score≥0.4·2m 대응이 **91관측 중 E24/E3/E10 12/4/23회**다.
그러나 후방 K5 66관측은 **9/4/22회**, 전방 Niro/Nexo 25관측은 **3/0/1회**다.
전방 E10은 score≥0.1에서도 3/25라 전방 초장거리를 해결했다고 판단하지 않는다.
두 취약 band의 후보가 NMS에서 사라진 관측은 0회였고, 가까운 raw 후보의 score가
운영 기준을 넘는지가 주된 관측 차이였다. 후보 존재를 운영 TP로 바꾸어 세지 않는다.

근거리 장애물도 평균 AP 상승이 모든 누락 꼬리의 개선을 뜻하지 않는다. O02 0~30m에서
같은 score≥0.1·2m 매칭과 원형 gap 처리를 적용해 누락 구간 전체를 다시 계산했다.

| 근거리 장애물 누락 구간 | 개수 | 평균(초) | p50 | p90 | p95 | 최대 |
|---|---:|---:|---:|---:|---:|---:|
| E24 | 35 | 0.220 | 0.159 | 0.500 | 0.618 | 0.708 |
| E10 | 21 | 0.223 | 0.094 | 0.518 | 0.521 | 0.979 |

누락 관측은 85/897→51/897로 줄었지만 평균·p90까지 단조 개선된 것은 아니다.
가장 긴 E10 누락은 **후방 CargoBox, 23.04→26.63m, LiDAR 0점인 11프레임**이다.
그 구간의 E24는 6/11회, E2/E3/E10은 0/11회 대응했다. E10은 모든 프레임에 2m 안의
동일 클래스 raw 후보가 있었으나 score가 **0.0485~0.099666**으로 floor 0.1 미만이었다.
NMS 손실이나 후보 부재로 설명하지 않으며, 한 사례만으로 threshold를 낮추지 않는다.
이 분석은 tracker coasting·ID·planner를 실행한 결과가 아니다.

### 보존·검증과 현재 판단

산출물 root는 `artifacts/evaluation/bevfusion/full30750_latest_compare_261023_v1/`다.
`E10_REMOTE_PIN.json`, `checkpoint_parts/RETRIEVED.json`, `E10_STATE_IDENTITY.json`,
`INFERENCE_COMPLETE.json`, `e10_fresh/`, `comparison_E{24,2,3}_E10.json`,
`focused_scores_four_models.json`, `near_obstacle_misses.json`과 `native_history/`에
입력·출력·분모·계보를 보존한다.
Focus 분석에서 기존 세 모델의 group/운영 계수는 저장된 원형과 정확히 일치했다.
CPU 회귀는 비교기·quick·거리·새 focus 합계 **39개 통과**했다.

Drive E10 best 폴더는 payload를 복원할 수 있었지만 **COMPLETE가 없고 일부 part 및
`resolved_resume.py` 이름이 중복**돼 자동 백업의 ValueError/pending 기록이 남았다.
이번 SHA 회수 성공을 정식 백업 완료로 덮어쓰거나 Drive 항목을 삭제하지 않았다.
가중치 비교와 별도로 백업 metadata 정리가 필요하다.

현재 판단은 **E10이 E2/E3의 초기 약점을 상당 부분 회복했고, 이 개발 bag에서는 E24보다
전반적으로 유망한 교체 후보**라는 것이다. 데이터·학습 업데이트 수 등이 달라 단일 수집
보강의 인과 효과로 단정하지 않는다. 전방 초장거리·중거리 장애물·개별 누락 꼬리는 남았다.
예정된 E12 학습은 유지하며 production E24·alias·threshold·tracker는 변경하지 않았다.
