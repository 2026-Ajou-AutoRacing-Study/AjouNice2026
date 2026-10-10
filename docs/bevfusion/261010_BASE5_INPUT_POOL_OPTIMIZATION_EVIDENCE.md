# Base5 입력·격자 정합·pooling 최적화 — 2026-10-10

## CURRENT / 다음 판단

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
2026-10-10 23:05 KST에는 개발용 rosbag과 원본 PNG를 비교해 JPEG Q90을 카메라별
80% 확률로 적용했다. 이후 사용자 합의와 로컬 비교를 거쳐 **23:55 KST 기준으로 Q90을
네 카메라 모두 100% 적용하며, 사전 생성 native JPEG 캐시를 추천**한다. 2026-10-11에는
전체 5K JPEG 생성·독립 전달본과 짧은 PNG/JPEG 실제 학습 비용 비교를 완료했다.
아래 11절이 최신 산출물과 비용 근거이고 9절의 80%는 이전 결정 이력이다. blur·노출·noise는 OFF다.
본학습에는 `bevfusion_base5_dense_512_jpeg90_train.py`를 명시한다. 기존 all-OFF 비용
config와 수치는 비교 이력으로 보존한다. Vast는 재개하지 않았다.

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

## 11. 전체 JPEG 생성과 PNG 없는 train 전달본 — 2026-10-11

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
