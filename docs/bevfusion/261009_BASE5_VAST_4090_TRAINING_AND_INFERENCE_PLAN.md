# Base5 원격 실행과 보존 안내

문서 상태: RUNBOOK + COMPATIBILITY ROUTER. 확인 기준: 2026-10-22.
기존 Sunny13 24 epoch 학습·E9/E24 외부 백업은 완료했다. 새 데이터 대조 학습을 포함한
현재 모델·다음 작업은 [CURRENT](CURRENT.md)를 따른다.
아래 기존 명령은 완료된 동결 run의 점검/복구용이며 새로운 학습 준비를 자동으로 의미하지 않는다.
10/21 새 Foggy13 정상 장면 1장을 추가한 최신 입력은 **30,750장**이다.
[새 30,750장 계약](#full-data-30750-preparation-261021)을 먼저 읽는다.
아래 30,749장 절은 추가 촬영 전 준비 이력이며, 당시 계약·데이터·검증 기록은 보존한다.
같은 날 추가한 [실제 val 로깅 통합](#full-data-val-logging-261021)은 새 runtime v5에 분리한다.
v4 준비와 더미 패널 run은 실제 추가 지표의 검증 기록으로 대신 쓰지 않는다.
이후 사용자 승인으로 시작한 현재 전체 학습은 [실제 시작 기록](#full-data-live-training-261021)을 따른다.
10/22 확인한 중단 상태와 완료 E3 복원·평가는 [후속 기록](#full-data-e3-interrupted-evaluation-261022)을 따른다.
평가 작업은 학습 재개나 인스턴스 시작 승인이 아니다.
첫 val의 파일 디스크립터 오류와 별도 계승 준비는 [10/21 병목 수정 기록](#full-data-speed-repair-261021)을 따른다. 시작 당시의 기록을 재개 성공으로 바꾸지 않는다.

## 점검과 재개 경계

- 실행 진입점은 [start_base5_vast_training.sh](../../scripts/runtime/start_base5_vast_training.sh)다.
- 인자 없는 실행은 원격 run을 시작할 수 있다. 문서 확인만을 위해 실행하지 않는다.
- 사용자가 원격 점검을 요청했을 때 `bash scripts/runtime/start_base5_vast_training.sh --check --resume`로 계약·연결·W&B·Drive를 확인한다.
- 같은 동결 run의 미완료 에폭을 재개할 때만 사용자 승인 후 `--resume`를 사용한다.
- 새 환경·클래스·초기화·스케줄 변경은 새 source/data/work 계약을 만든다. 완료 run의 hash를 덮어쓰지 않는다.
- W&B run identity·optimizer/scheduler/RNG·완료 epoch 재개 범위를 보존한다. 정확한 mid-epoch replay를 보장하지 않는다.
- 백업은 업로드 폴더 존재가 아니라 full checkpoint·metadata·COMPLETE/LATEST·SHA readback으로 확인한다.
- GPU Stop 후 재할당과 재시작 불가가 가능하다. 데이터 복사·백업 확인 없이 Destroy하지 않는다.
- SSH endpoint·현재 요금·가용 GPU는 실제 인스턴스에서 확인한다. 이력의 IP·가격을 현재값으로 쓰지 않는다.
- 2026-10-18 새 임대용 private 학습 이미지 `fluffyluv/ajounice2026-bevfusion-train:base5-cu128-261018-v1`을 게시했다.
  [빌드·고정 digest·Vast 설정](../../docker/bevfusion/README.md#vast-오프라인-학습-이미지--2026-10-18)을 따른다.
  CPU import/config/CLI 검증과 새 Vast의 실제 pull·GPU 학습/재개·백업 검증은 구분하며 후자는 아직 미완료다.

<a id="next-training-cpu"></a>

## 다음 학습의 CPU 사전검사

2026-10-15 사용자 승인으로 **새 학습만** compile OFF로 준비했다. 완료 E24의 학습 ON 설정,
동결 Vast 실행기·optimizer·W&B run·가중치·resume 계약은 수정하지 않는다.
OFF가 정확도를 높인다는 인증이나 기존 E24가 잘못 학습됐다는 뜻은 아니다.

- 설정: `bevfusion_base5_fine_512_depth25_next_training.py`. 기존 nearest runtime을 상속하며 compile OFF를 명시한다.
- train/val 공통 nearest_positive/sorted, Q90 JPEG, upright 7D·Animal→Pedestrian, 512×1056·5cm/20cm·depth 0.25m를 유지한다.
- SaveSensorData 강도는 원래 0~1이며 학습에서 ÷255하지 않는다. ROS 0~255 변환은 추론 입력에만 적용한다.
- 새 실행기는 SSH·GPU·모델 registry·학습을 시작하지 않는다. `--run`·`--resume`를 받지 않으며 새 빈 work-dir만 허용한다.

기존 BEVFusion Python 환경 안에서 실행한다. 호스트 시스템 Python에는 MMEngine이 없을 수 있으며,
사전검사를 위해 의존성을 재설치하거나 GPU를 빌릴 필요는 없다. 기존 팀 Docker를 GPU 없이 사용해도 된다.

```bash
bash scripts/runtime/prepare_base5_next_training.sh --check-config

bash scripts/runtime/prepare_base5_next_training.sh \
  --data "$BASE5_DATA" \
  --init-checkpoint "$BASE5_INITIALIZER" \
  --work-dir "$PWD/artifacts/training/base5_next_cpu_preflight" \
  --verify-payload-hashes --sample-clouds 16 --wandb
```

`BASE5_DATA`는 신뢰하는 3클래스 native-JPEG portable root, `BASE5_INITIALIZER`는 별도로 검증한
호환 초기화 파일 경로다. CPU 단계에서는 초기화 파일의 SHA만 기록하며 state shape를 검증하지 않는다.
기본 24 epoch·batch 4·누적 1·lr 1e-4는 준비 계획 값이고 최종 30K 학습 예산의 확정이 아니다.

전체 GT·split·입력 경로와 선택 시 전체 payload 해시를 검사하고, LiDAR 강도는 split마다 고르게
뽑은 표본으로 확인한다. `checked_model_config.py`와 `next_training_preflight.json`만 생성한다.
후자는 실제 direct runner의 `run_manifest.json`이나 GPU/패키지 자격 검증을 대체하지 않으며 이를 복사해 E24에 강제 resume하지 않는다.
계획에 기록된 후속 prepare 명령은 기존 거리별 AP/W&B 진단 확장을 유지하지만 `--run`은 포함하지 않는다.
실제 학습은 대상 GPU에서 초기화 strict load·forward/backward·비용·재개·W&B/Drive 백업을 검증하고
새 source/data/work 계약을 동결한 뒤 별도로 승인한다. CPU artifact의 컨테이너 경로는 새 호스트에서 다시 준비한다.
[2026-10-15 CPU 검증 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-next-training-cpu-261015)는 실제 5K·GT 전체와 강도 표본 검사의 범위를 구분한다.

<a id="full-data-12epoch-preparation-261021"></a>

## 6환경 + 장애물 보강 전체 학습 준비 — 2026-10-21

사용자는 정규 6환경 약 30K와 장애물 보강 750장의 전체 학습 준비를 승인했다.
Foggy11 최종 Q90/upright 5,000장도 검증을 완료했다. Foggy13의 지원 GT 없는 한 장은
계속 제외하므로 실제 입력은 **30,749장 / train 26,135 / val 4,614**다. 구 production용
Sunny13 5K를 추가로 중복 합산하지 않는다. 9개 cohort의 실제 infos/manifest 대조에서
전역 token/raw/payload 중복과 group 분할 충돌은 모두 0이었다. 이는 로컬 입력 준비의
완료이며, Vast 전달·학습 시작과 구분한다. [데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)를 따른다.

### 8에폭과 12에폭 비교·사용자 선택

기존 compile OFF Sunny15 일반 코사인 학습 로그의 평균 iteration은 약 0.667초다.
전체 데이터의 batch 4에서는 에폭당 6,534 update다. 아래는 그 비용을 단순 확장한 예상이며,
새 전체 데이터·대상 GPU에서 실측한 총시간이 아니다. 전송 시간은 포함하지 않는다.

| 예산 | 업데이트 | 학습 연산만 | 검증·체크포인트·백업 포함 예상 |
|---|---:|---:|---:|
| 8 epoch | 52,272 | 약 9.68시간 | 약 11~12시간 |
| 12 epoch | 78,408 | 약 14.52시간 | 약 16~18시간 |

참고로 새 Sunny13의 기존 **5K·24 epoch 고정 스케줄** 로그를 보면, 각 지표의 최고값은 다음과 같다.

| 자체 val 지표 | 8 epoch까지 최고 | 12 epoch까지 최고 |
|---|---:|---:|
| Center mAP | 89.39% | 89.94% |
| Obstacle AP | 81.06% | 82.04% |
| 20~30m Obstacle AP | 76.04% | 77.81% |
| 80~100m Vehicle AP | 70.50% | 72.94% |

각 지표의 최고 epoch는 서로 다르며 하나의 모델 성적표가 아니다. 8/12 epoch로 처음부터
코사인을 끝까지 내린 독립 실험도 아니므로 이번 전체 데이터의 예상 개선량으로 쓰지 않는다.
전체 데이터 8 epoch도 구 5K·24 epoch보다 update는 약 2.05배지만, 각 표본 노출은 8회이고
새 Foggy 환경도 있으므로 update 수만으로 수렴 완료를 보장할 수 없다.

한 번의 본학습 기회를 충분히 쓰는 방향으로 12 epoch를 제안했고 사용자는 **“12에폭으로 준비”**를
선택했다. 실제 학습 시작은 아직 승인·실행 전이다. 처음부터 12 epoch 스케줄로 새 계약을 만들며,
8 epoch 완료 run을 12로 몰래 연장하거나 과거 optimizer를 강제 resume하지 않는다.
원근거·계산은 `artifacts/evaluation/base5_full_training_budget_review_261021_v1/comparison.json`,
실제 사용자 응답은 같은 폴더 `USER_APPROVAL.json`에 기록했다.

### 고정할 입력·실행 정책과 전송 범위

- 기존 호환 nuScenes 초기화와 새 optimizer/scheduler, 일반 코사인·500 update 선형 warm-up을 사용한다.
- compile OFF·batch 4·누적 1·workers 4·AdamW lr 1e-4·512×1056·5cm/20cm·depth 25cm를 유지한다.
- 기존 W&B 지표명·거리별 AP·front/rear 진단과 full checkpoint Drive 백업·readback 계약을 유지한다.
- 각 환경의 원래 root/infos/GT/split을 보존한 `ConcatDataset`으로 연결한다. metainfo에서 달라질 수 있는
  키는 배치 overlay 출처 SHA 하나뿐이며, 장면별 원래 overlay/group binding은 그대로 검증한다.
- 기존 Vast의 새 Sunny13·Sunny15와 초기 가중치 SHA·설치를 재사용한다. 재대여·Destroy·CUDA 재설치는 하지 않는다.
- 사용자가 누락 MORAI 합성 데이터 압축본을 본인 Drive `dataset/Base5/full30749_261021/`에
  업로드하고 현재 Vast로 받는 것을 명시적으로 승인했다. 기존 파일 덮어쓰기·삭제는 하지 않는다.
- Direct 전송은 약 0.1MB/s라 자체 전송만 중지하고 불완전 파일은 보존했다. Foggy15의 Drive 업로드는
  약 29.6MiB/s, 이후 Vast 실제 수신은 약 2.7~3.8MB/s로 관측했다. 일반적인 MB/s 진행은 유지하며
  옛 8MB/s 중단 기준을 새 전송에 적용하지 않는다. 전체 SHA 확인·복원·입력 준비 전에는 완료라고 하지 않는다.

새 cohort index/현장 설정·receipt는 `artifacts/evaluation/base5_full_data_training_draft_261021_v1/`에서
관리한다. 재사용 가능한 전체 데이터 실행기는 `scripts/runtime/full_data_prepare.py`,
`full_data_oneclick.py`, `start_base5_full_vast_training.sh`로 분리했다. 모델·optimizer·scheduler는
원형의 검증된 구현을 재사용하고 전체 데이터 연결·identity·시작 권한만 별도 계약으로 확장한다.
16개 CPU 단위 테스트가 통과했다. 구 5K 전용 oneclick을 대신 실행하지 않는다.

기본 실행과 `--status`는 상태만 읽는다. `--prepare`는 설정 준비, `--check`는 온라인 사전 검사,
`--smoke`는 CPU Concat 호환 검사이며 모델 학습을 시작하지 않는다. 최종 전달 후 생성할
`launcher_e12_ready.json`이 없는 동안에는 기본 실행을 차단한다. `--run`은 epoch 준비 승인과
별개인 실제 학습 시작 승인이 있어야 한다. Production E24와 완료된 과거 run은 바꾸지 않는다.

백업은 새 전체 데이터 실행기에 한해 **Vast→Drive 직접 전송**으로 준비했다. 기존 경로의
Vast→로컬 역방향 SSH→Drive 우회는 대용량 전송 병목 가능성이 있었다. W&B 경로와 원형의
access-only 인증·전체 SHA·분할 전송·COMPLETE/LATEST·로컬 readback·1,200초 제한은 유지한다.
새 직접 경로의 작은 파일 쓰기/재읽기와 W&B 온라인 연결은 확인했지만, **새 전체 checkpoint의
전송 속도·제한시간 내 완료는 아직 실측하지 않았다. 첫 epoch의 실제 full 백업을 확인해야 한다.**
기존 자동 백업 실패나 수동 백업 이력을 성공으로 덮어쓰지 않는다. 갱신 가능한 인증정보는
원격에 저장하지 않으며, 학습 중 로컬의 인증 갱신·W&B 중계·백업 확인 프로세스는 유지해야 한다.

### 실제 전달·실행 준비 검사 완료

9개 cohort의 실제 Vast 입력 전달을 마쳤고, **원형 `--prepare`와 `--check`가 모두 exit 0**으로
통과했다. 구 5K 전용 run은 변경하지 않았다. 전체 입력은 30,749장, train 26,135 / val 4,614이며
12 epoch·batch 4·누적 1·epoch당 6,534 update·checkpoint `max_keep_ckpts=3`·compile OFF를 준비했다.
일반 코사인·기존 거리별/전후방 지표와 호환 nuScenes 초기화를 유지한다.

최종 계약 SHA는 `0cdd6b587e21147f29cdce5e471cebd24c632d663477330343d9554a8245b7c3`이다.
원격 설정은 `/workspace/base5_full_data_runtime_261021_v3/config_e12_final.json`, 새 work는
`/workspace/base5_full30749_e12_261021_v1`이며, 로컬 설정은
`artifacts/evaluation/base5_full_data_training_draft_261021_v1/launcher_e12_ready.json`이다.

준비된 `resolved_train.py`를 실제 설치된 MMEngine/MMDetection3D에서 CPU로 한 번 구성했다.
**train 26,135 / val 4,614·18개 child·첫 token 18개·원래 입력 경로 90개**가 일치했고 약 25.61초였다.
이를 위해 portable 검증기나 전체 payload/GT 검사를 다시 호출하지 않았으며 모델·학습 실행은 0이다.
별도 16개 단위 테스트와 shell 문법·오류 경로 독립 검토도 통과했다.

GPU 49,140MiB·W&B 온라인 접근·백업 qualification 재사용과 새 직접 경로의 소규모 증거·Drive
쓰기/재읽기 검사를 확인했다. Vast 여유는 게시 시 약 27.11GiB, 본인 Drive 여유는 조회 시 약
21.17GiB였다. 새 전체 checkpoint 백업 완료를 뜻하지 않으며 **첫 epoch의 실제 전송·readback은
학습 시작 뒤 확인해야 한다.** 이번 사전 검사는 원형 제한시간 안에서 통과했고 재시도나 제한 완화는 없었다.

최종 요약은 `artifacts/evaluation/base5_full_data_training_draft_261021_v1/FINAL_PREPARATION.json`,
SHA는 `bdb96a8fb4ebcbd025ed76c98c12f1b764f090aae7b9c9e7b581948db95bc50b`다.
이는 실제 준비·온라인 검사·CPU 구성 결과이며 12-epoch 학습이나 새 checkpoint 백업의 완료 기록이 아니다.

준비 상태는 저장소 루트에서 다음 명령으로 읽을 수 있다. **학습을 시작하는 명령이 아니다.**

```bash
bash scripts/runtime/start_base5_full_vast_training.sh --status
```

실제 시작 권한 파일은 만들지 않았고 학습도 시작하지 않았다. 사용자가 별도로 시작을 승인한 뒤
동일한 data/code/epoch 계약에 시작 권한을 연결한다. 학습 중 로컬 인증 갱신·W&B 중계·백업 확인을
유지해야 하므로 로컬 PC를 끄거나 해당 watcher를 종료하지 않는다.

<a id="full-data-30750-preparation-261021"></a>

## 새 정상 장면 1장으로 30,750장 마감 — 2026-10-21 후속

사용자 요청으로 Foggy13에서 **새 장면을 실제 1회 촬영**했다. 원래 제외한 Bollard04의
GT를 임의로 복구하거나 과거 30,749장 계약을 수정하지 않았다. 새 WoodBox는 전방 약
13.73m, LiDAR 52점·CAM1 Instance 6,750픽셀로 관측됐고 native 승인·충돌 없음·종료 정지/P를
통과했다. 같은 Q90/upright·3클래스 변환으로 **유효 Obstacle GT 1개·JPEG 4개**를 얻었다.
새 원본 PNG는 보존하며 촬영을 더 하거나 기존 5K를 재변환하지 않았다.
[촬영·변환 근거](../../dataset/morai_od/docs/261019_base5_foggy13_reinforcement_and_capture.md#foggy13-terminal-one-30750-261021)를 따른다.

새 `foggy13_completion`을 train 1 / val 0의 독립 cohort로 연결했다. 기존 Foggy13 지원
4,999장과 합치면 논리적으로 5,000장이지만, 원래 root·infos·제외 근거는 그대로다.
전체는 **30,750장 / train 26,136 / val 4,614 / test 0**이다. 새 장면의 token·raw·payload·group은
기존 입력과 충돌하지 않는다. train은 10개, val은 원래 9개 nonempty child를 사용한다.
val 0인 singleton을 빈 MMEngine child로 만들지 않는 것이며, 원래 dataset 안의 배경 장면이나
기존 GT를 추가로 걸러내는 변경이 아니다.

약 520KB인 새 singleton 압축본만 Direct SSH로 전달해 전체 SHA와 51개 내부 멤버·입력을
검증했다. 새 runtime은 `/workspace/base5_full_data_runtime_261021_v4`, 새 work는
`/workspace/base5_full30750_e12_261021_v1`이다. 구 runtime v3·30,749장 work·원래 9개 cohort는
변경하지 않았다. 이 단계의 로컬 설정은
`artifacts/evaluation/base5_full30750_training_261021_v1/launcher_e12_ready.json`이며 당시 shell 기본값도
이 경로였다. 이후 기본값은 아래 val 로깅 통합의 새 v5로 변경했다. 새 index SHA는
`481008024038779fd6eaf8e6bb2676461ccd201081ad0f8cb762718399d42d24`다.
원형 `--prepare`는 실제 181.76초에 exit 0으로 완료했고 새 계약 SHA는
`5c5a6db78ddb9d1ee16ecba0e90ea81490de12879d425656c7c5da9eadfcd456`이다.
원형 `--check`도 동일 계약으로 통과했다. 첫 검사는 W&B SSH relay listener 확인에서
실패했고 기록을 보존했다. 포트 경합 증거는 없었으며 원인을 확정하거나 해결됐다고
단정하지 않는다. 코드·포트·loopback 안전 기준 변경 없이 **실제 재검사 1회가 218.95초에
exit 0**으로 완료됐다. GPU 49,140MiB·W&B 온라인 접근과 새 Drive 목적지의
쓰기/재읽기·백업 자격 재사용을 확인했다. 실제 학습이나 새 full checkpoint 백업은 아니다.
준비된 `resolved_train.py`도 실제 설치된 MMEngine/MMDetection3D에서 **CPU로 한 번** 구성했다.
train 10개·val 9개의 **19개 child / 첫 token 19개 / 원래 입력 경로 95개**가 일치했고
구성 시간은 약 28.24초였다. 전체 RGB·native QA를 다시 실행하지 않았고 모델 실행도 0이다.
최종 준비 receipt는 `artifacts/evaluation/base5_full30750_training_261021_v1/FINAL_PREPARATION.json`,
SHA는 `e803d31694ec80cb2ea0d386f80b8e292463cf02f7c14468a882343c0ac19ac2`다.

**12 epoch·일반 코사인·compile OFF·batch 4·기존 호환 nuScenes 초기화**는 그대로다.
train 한 장 증가 뒤에도 올림한 epoch당 update는 6,534, 전체는 78,408로 같고 시간 예산도
기존 로그 기반 약 16~18시간 예상이다. W&B 지표명·거리별/전후방 진단·원형 모델과
optimizer/scheduler 구현은 변경하지 않았다. 새 run group은 `base5-full30750-261021`이다.

전체 데이터 준비가 구 5K의 제한시간에 걸리지 않도록 **설정 준비 명령만 900초, 원격
prepare/check/start 통신 대기만 1,200초**로 분리했다. 실제 학습 `--run`의 내부 제한,
일반 SSH/status·인증·백업의 1,200초 제한은 변경하지 않았다. setup 대기시간을 늘린 것이며
촬영 품질이나 입력 검사를 줄인 변경이 아니다. CPU 단위 테스트 20개와 shell 문법 검사를 통과했다.

새 Drive 목적지는 `model/bevfusion/base5/full30750_261021/checkpoint_backups`다.
작은 파일의 실제 쓰기·재읽기와 보존된 full-checkpoint 백업 기계적 자격의 재사용을 확인했다.
**새 경로의 full checkpoint 전송 속도·시간 내 완료는 첫 실제 epoch에서 확인해야 한다.**
갱신 가능한 인증정보를 원격에 저장하지 않으며 실제 학습 중 로컬의 인증 갱신·W&B 중계·
백업 확인 프로세스를 유지해야 한다. Production E24와 과거 run은 변경하지 않는다.

**실제 학습 시작 권한은 만들지 않았으며 학습도 시작하지 않았다.** 준비 승인과 시작 승인을
구분한다. 기본 실행과 `--status`는 상태 조회다. 재사용한 상태 이벤트의 이름이 `sunny15_status`여도
새 설정은 전체 30,750장 계약이다. 조회의 `ready: false`와 `requires_online_preflight: true`는
조회만으로 새 온라인 사전 검사를 인증하지 않는다는 뜻이며, `data_and_contract_ready`와 구분한다.

<a id="full-data-val-logging-261021"></a>

## 전체 30,750장 val 로깅 통합 — 2026-10-21 후속

사용자는 더미 run의 패널 이름·형식을 유지한 실제 로깅 연결을 승인했다.
더미 `DUMMY_BASE5_VAL_PANEL_PREVIEW_NOT_A_MODEL`의 3 epoch 값은 전부 합성값이고,
실제 모델 비교·best 선정에서 제외한다. 아래 지표를 실제 학습 경로에 연결하기 위한 새 모듈은
`scripts/runtime/base5_val_logging.py`, schema는 `base5_val_logging_v1`이다.
기존 동결 detector source와 `base5_distance_diagnostics.py`는 수정하지 않는다.

| 계열 | 실제 키와 의미 |
|---|---|
| 거리 | `val/range/{0_10m,…,90_100m}/{class}_mean_AP`, 지원될 때 `mAP`. 기존 30–50/50–80/80–100m와 전후방·100m 이상 모서리 지원 수는 유지 |
| 환경 | `val/environment/{sunny11,sunny13,sunny15,foggy11,foggy13,foggy15}/{mAP,vehicle_mean_AP,pedestrian_mean_AP,obstacle_mean_AP}`. 클래스별 `_GT`와 `frames`도 기록 |
| 보강 | `val/supplement/sunny_obstacle/obstacle_mean_AP`. Sunny 장애물 단독 val 114장을 따로 집계하고, 단일 클래스 평균을 mAP로 부르지 않음 |
| 고정 운영점 | `val/fixed/{vehicle,pedestrian,obstacle}/{precision,recall,F1,fp_per_frame,score_threshold}` |

X축은 기존과 같은 `epoch`, AP·P/R·F1은 **0~1**이다. 클래스 AP는 원형과 동일한
XY 중심거리 0.5/1/2/4m AP의 평균이고, mAP는 3클래스 동일 가중 평균이다.
새 구간에서 GT 0인 클래스 AP는 생략하고 `_GT`·`_available`·`_small_sample`로 구분한다.
기존 근거리의 GT 0 관련 scalar까지 임의로 고치지는 않는다.

거리 구간은 XY 반경의 `[하한, 상한)`이다. 경계의 작은 위치 오차에 민감할 수 있어 기존 넓은
구간도 남긴다. 이미 계산한 0~10/10~20/20~30m AP는 재사용하고, 새 7개 구간만 추가 계산한다.
GPU 재추론·추가 이미지 로딩·gradient 계산은 없으며 val 예측의 CPU 후처리만 늘어난다.

6환경 비교에는 본수집 **각 750장, 총 4,500장**만 사용한다. 보강 3개 cohort의 val 각 38장은
별도 114장으로 묶고, **전체 AP에는 원래처럼 4,614장 모두 포함**한다. 토큰 문자열의 앞부분이나
수신 순서로 날씨를 추정하지 않는다. 원래 infos에서 확인한 token→cohort 매핑을 evaluator 설정에
명시하며, `sample_idx`와 LiDAR 경로·중복·누락·cohort별 개수를 검사한다.

고정 score는 **V/P/O 0.40/0.10/0.10**이다. 클래스별 score 내림차순·동일 클래스·XY 중심거리
**<1m**의 greedy 1:1 대응이며, score는 **≥ threshold**를 사용한다. TP/FP/FN의 정수 집계로
P/R·F1을 계산하고 FP/frame 분모에는 GT가 없는 프레임도 포함한다. 이 고정 threshold는 새
진단에만 적용한다. AP 예측을 미리 잘라내거나 기존 best-F1 지표를 대체하지 않는다.
이는 raw val decoded 출력의 score 진단이며 ROS NMS·tracker까지 재현한 실전 지표는 아니다.
새 모델의 최종 배포 threshold 선정도 별도다.

환경×거리×전후방×threshold의 모든 조합은 만들지 않는다. loss·모델·optimizer·스케줄·train
pipeline·샘플 순서와 `val/center/mAP` best 기준도 유지한다. 새 schema/module SHA와 val 매핑 SHA는
새 준비 계약에 묶고, 체크포인트·에폭별 JSON receipt에도 로깅 버전을 기록한다.

새 runtime은 `/workspace/base5_full_data_runtime_261021_v5`, 새 work는
`/workspace/base5_full30750_e12_val_logging_261021_v1`이다. 이전 v4 준비·데이터·가중치·원본 source는
그대로 보존한다. 새 전달·설정·검증 근거는
`artifacts/evaluation/base5_full30750_val_logging_261021_v1/`에서 관리한다.
기존 데이터·초기화·12 epoch 예산·백업 목적지를 재사용하며 데이터 payload 재전송은 없다.
**실제 학습은 시작하지 않으며 시작 권한 파일도 만들지 않는다.**

로컬에서는 새 지표·프로토콜·실행기 테스트 **53개가 통과**했다. 실제 MMEngine에 의존하는
추가 통합 테스트도 Vast에서 **4/4 통과**해 총 57개를 확인했다. 단위 테스트는 threshold의 포함 경계와 1m의 제외 경계,
중복 FP·빈 프레임 분모·GT 0·legacy scalar 불변·더미 키 대소문자·기존 3구간 AP 재사용을 확인한다.
원형 새 `--prepare`는 236.14초에 통과했고, 계약은
`7de3d916b0080074bbdbe62b97f714c398579ad2f9bf638c60719a8b18a0960f`다.
module SHA는 `658db7bc4a2a28d947f24e9e67955c914c407aab59c1b9cf8aee337d4e088af5`,
schema SHA는 `d41d49e5a44823e36d4dd0292c6ec2504bc87954ebaed61405b1567f9ea26b69`다.

원형 `--check`는 **238.48초·첫 시도**에 통과했다. W&B 온라인·Drive 쓰기/재읽기와 기존
백업 자격 재사용을 확인했다. 실제 MMEngine 통합은 4,614개 합성 결과를 원형 평가기에 넣어
legacy scalar의 정확한 보존, 환경/보강 분리, W&B 키·epoch 축·중복 prefix 방지와 checkpoint
metadata 보존을 확인했다. GPU 모델 실행이나 실제 모델 AP 측정은 아니다. 기존 identity의
데이터·모델·초기화·optimizer·scheduler 등 **30개 항목도 모두 동일**했다.
최종 receipt는 같은 폴더의 `FINAL_PREPARATION.json`, SHA는
`24111550eec9f8f3dbbb207eb3591ad7a113bc527a4d7d0785c41a433984717b`다.
기본 shell은 이 폴더의 `launcher_e12_ready.json`을 사용하며 이전 v4와 과거 근거는 남긴다.
새 진단의 실제 전체 val 추가 시간과 첫 epoch의 full checkpoint 전송은 학습 시작 후 확인한다.

문서 검증기는 새 링크·보존 계약 오류 없이 기존 길이 예산 초과 3건을 보고했다.
`docs/bevfusion/README.md` 67줄, 모델 CURRENT 150줄, 데이터 CURRENT 209줄은 작업 전 HEAD와
같다. 이번 로깅 연결 작업에서 범위 밖 문서를 재편하거나 과거 근거를 삭제하지 않았다.

<a id="full-data-live-training-261021"></a>

## 사용자 승인 후 전체 30,750장 학습 시작 — 2026-10-21

사용자의 **“응 학습 쭉쭉 진행 ㄱㄱ”** 요청으로 준비와 별개인 실제 시작 권한을 기록했다.
앞선 v4·v5 절의 ‘미시작’은 각각 준비 당시 이력이며 해당 receipt·동결 설정은 수정하지 않았다.
실제 시작 권한은 데이터 index·adapter SHA·12 epoch와 결합했고, 원격
`/workspace/base5_full_data_runtime_261021_v5/USER_START_AUTHORITY.json`에 전달했다.
로컬 `START_APPROVAL.json` SHA는
`43264e618566b50e6e7f974734771f73371ed839feca0aa6afbd2697d70c9e66`이다.

첫 시작 요청은 SSH relay listener 확인 중 **4.19초에 실패**했으며 학습은 시작하지 않았다.
그 기록을 보존하고 코드·포트·안전 조건 변경 없이 같은 시작 명령을 재시도했다.
재시도는 원형 온라인 사전검사를 포함해 **469.47초·exit 0**으로 완료했고 원격 worker를 시작했다.
이후 실제 시작 시의 입력 payload hash 확인·원형 첫 cohort 확인·데이터 구성 때문에 첫 iter까지
약 12분이 추가로 걸렸다. 이 준비 시간을 GPU 학습 정지나 완료로 판정하지 않는다.

10/21 **15:59:50 KST의 E1 iter 20**, **16:00:04 KST의 iter 40 / 6,534**를 실제 console에서 확인했다.
loss는 각각 **18.5036 / 13.5102**였다. iter 20의 `grad_norm: inf`는 초기 관측으로 남기며,
iter 40은 **485.2875**의 유한값으로 돌아왔다. 초기 두 출력만으로 수렴·정확도를 인증하지 않는다.
iter 40의 평균 step은 **0.6623초**, data time은 **0.0139초**, console ETA는 **16:02:33**이었다.
GPU는 총 49,140MiB 중 **30,864MiB·utilization 100%·60°C**로 관측했다. 초기 ETA는 안정화와
첫 전체 val 비용에 따라 달라지며, 새 CPU 진단의 실제 추가 시간을 아직 확정하지 않았다.

`fresh_start.json`은 **resumed=false / epoch 0 / iteration 0**과 기존 v5 계약
`7de3d916b0080074bbdbe62b97f714c398579ad2f9bf638c60719a8b18a0960f`를 기록했다.
초기화는 기존 호환 nuScenes checkpoint이며 E24 resume가 아니다. 초기 load 경고의 누락 키는
현재 설정에서 구성하는 `view_transform.dx/bx/nx/frustum` 4개였다. 데이터·모델·optimizer·
일반 코사인·compile OFF·batch 4·12 epoch는 준비 조건 그대로이며 production E24는 교체하지 않는다.

시작·관측 근거는 `artifacts/evaluation/base5_full30750_val_logging_261021_v1/`의
`START_LAUNCH_1792564701659673358.json`(실패), `START_LAUNCH_1792564758783616997.json`(시작 요청 성공),
`TRAINING_OBSERVATION_1792566009315070270.json`(실제 iter·GPU·fresh start)이다.
원격 work의 `console.log`·`oneclick_status.json`과 로컬 `logs/watcher.log`·`logs/status.json`을 구분한다.
후속 **16:02:03 KST의 E1 iter 220**은 loss **3.7191**, grad norm **244.5065**, step **0.6602초**,
data time **0.0137초**였다. W&B 읽기 전용 API에서도 같은 run의 **running / epoch 1 / iter 260 /
loss 3.5307 / grad norm 259.7153**를 실제 수신했다. run은
[전체 30,750장 실제 학습](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/base5_full30750_e12_val_logging_261021_v1-7de3d916b008)이며
group은 `base5-full30750-261021`이다. 더미 run을 실제 결과로 전환한 것이 아니다.
`TRAINING_OBSERVATION_1792566126345984457.json`과 `START_VERIFIED.json`에 이 확인을 보존했다.
거리·환경·고정 운영점 지표의 실제 값은 첫 전체 val 종료 후에 나온다. 첫 CPU val 추가 시간과
**첫 epoch full checkpoint의 Drive 업로드·전체 SHA readback은 아직
완료 전**이다. 작은 Drive 사전검사 성공을 새 full 백업 완료로 표현하지 않는다. 로컬 인증 갱신·
W&B relay·백업 watcher를 유지해야 하므로 학습 중 로컬 PC를 끄거나 해당 프로세스를 종료하지 않는다.

```bash
# 기본값은 현재 v5 계약이며 인자 없는 실행은 상태 조회다.
bash scripts/runtime/start_base5_full_vast_training.sh --status

# 중단 후 완료 epoch의 last_checkpoint가 있을 때만 같은 계약으로 재개한다.
bash scripts/runtime/start_base5_full_vast_training.sh --run --resume
```

재개는 epoch 경계이며 중간 iteration·worker/prefetch의 정확한 재현을 보장하지 않는다.
이미 실행 중인 fresh run에 `--run`을 다시 보내거나 과거 5K 실행기로 재개하지 않는다.

<a id="full-data-speed-repair-261021"></a>

## 전체 학습의 val FD 오류 수정과 비용 재측정 — 2026-10-21

사용자는 train·val 병목을 바로잡기 위한 중단과 개선을 승인했다. 실제 상태를 먼저
확인하니 **중단 명령을 보내기 전에 기존 v5가 이미 exit 1로 종료**돼 있었다.
E1 train은 끝났고, 첫 val의 **4,220 / 4,614번째** 근처에서 DataLoader가
`RuntimeError: Too many open files`를 냈다. 느려 보여서 의도적으로 중단한 결과가 아니다.
당시 GPU는 유휴였고 원본 console·실패 상태·v5 manifest는 그대로 보존했다.

E1 full checkpoint는 정상 저장됐으며 Drive 상태에 전체 SHA readback과
**epoch 1 / iteration 6,534 / resume_capable=true / errors=[]**가 남아 있다.
SHA256은 `471e1e9c5a2c2c3a92330dc858570109644286bc3ea15fd74f54382b0274ba36`이다.
학습한 첫 epoch를 버리거나 E1부터 새로 학습할 필요는 없다. 실패한 첫 전체 val은
완성된 AP로 간주하지 않으며, E1 weights와 이후 계승 run의 지표를 구분한다.

### 원인과 수정 범위

기존 metric은 CPU에 있는 GT tensor를 `tensor.cpu().numpy()`로 보관했다.
이 연산은 복사가 아닐 수 있어, NumPy가 DataLoader worker의 공유 메모리 storage와
파일 디스크립터를 val 끝까지 붙잡았다. 장수가 늘수록 누적될 수 있는 구조였다.
`base5_metric_owned_storage.py`는 **보관할 작은 GT 배열을 복사하고 CPU 예측 tensor를
clone**해 입력 batch와 소유권을 분리한다. GT 값·순서·박스 표현·AP 계산·지표명은 바꾸지 않는다.
고정 source는 수정하지 않고 새 runtime extension의 SHA와 checkpoint receipt로 남긴다.

회귀 검사는 무복사 24행에서 descriptor가 최소 72개 늘어나는 반례와, 복사한
5,100행에서 증가가 제한되는 조건을 포함한다. 실제 2-worker IPC 및 실제 metric process의
5,100행 검사도 별도로 제공한다. 작은 회귀 검사·64장 비용 검사와 **전체 4,614장 검증 완료**는
구분한다. FD 한도만 높여 누적을 숨기는 방식은 채택하지 않는다. 앞선 모든 입력 지연이
이 FD 문제 때문이었다고 단정하지도 않는다.

### Val: batch 확대는 채택하지 않음

같은 E1 checkpoint·9개 val cohort에 분산한 **64장**·workers 4·prefetch 2·pin memory OFF·
FP32 조건에서 장당 동기화 wall time을 비교했다. 처음 두 batch는 warm-up으로 제외했다.

| Val batch | 평균 장당 시간 | 판단 |
|---|---:|---|
| 1 | **94.25 ms** | 유지 |
| 2 | 110.46 ms | 더 느림 |
| 4 | 107.19 ms | 더 느림 |

Batch 1의 입력 대기는 평균 **4.06 ms**, 계산은 **89.80 ms**였다. 무조건 batch를
늘리는 것이 이 sparse/BEV 구조의 장당 처리량 개선으로 이어지지는 않았다.
과거 정상 구간의 val 중앙값도 약 93.4 ms였으므로, 이 결과를 FD 수정만으로
기존 정상 추론이 여러 배 빨라졌다는 근거로 사용하지 않는다. Val AMP도 이번 운영값으로
승격하지 않으며 **batch 1·FP32를 유지**한다. 부분 표본 비용은 전체 AP·실전 latency 인증이 아니다.

### Train: 입력보다 역전파가 큰 비용

E1 전체 상태를 별도 시험 runner로 복원해 **32 step**을 측정했다. Warm-up 8회와
계측을 넣은 4회를 제외한 20회 평균은 **전체 671.37 ms / train 655.41 ms /
입력 대기 15.96 ms**였다. 입력 대기는 약 **2.4%**였다. 계측 구간의 backward stream span은
약 **331.26 ms**로 가장 컸다. CUDA event의 중첩·host 대기가 있어 각 구간을 단순 합산하지 않는다.
worker를 무작정 늘리거나 학습 batch·해상도·깊이·정확도를 바꾸는 처방은 선택하지 않았다.

발견한 작은 낭비는 rectangular head가 부모 구현의 dense heatmap target을 만든 뒤 버리고,
현재 XY 축에 맞는 올바른 target을 다시 만드는 부분이었다. 새 opt-in
`base5_skip_discarded_heatmap.py`는 **버리는 부모 target 생성만 생략**한다.
같은 실제 E1 출력 fixture 3묶음에서 사용되는 target·loss·head 출력에 대한 gradient·RNG가
모두 정확히 같았고, model parameter update와 checkpoint 저장은 하지 않았다.
이 검증은 이후 전체 학습 궤적의 bitwise 동일성이나 정확도 상승을 인증하는 것이 아니다.

동일 예산의 짧은 비용 대조에서 전체 step은 **671.37→660.19 ms(-1.67%)**, train 부분은
**655.41→644.99 ms**였다. 작은 이득이며 다른 CUDA 실행의 비결정성과 호스트 부하가
포함된 단일 짧은 대조다. 두 32-step run의 모든 loss가 완전히 같았다고 표현하지 않는다.
대형 구조 변경 대신 정확한 fixture 검사와 SHA가 결합된 이 작은 후보만 계승용으로 준비했다.

### 계승 계약과 근거

최종 계승 work는 `/workspace/base5_full30750_e12_speed_261021_v2`, runtime은
`/workspace/base5_speed_repair_261021_v2/scripts/runtime`이다. v1 준비에서는 원본 tuple과
저장 JSON list의 표현 차이를 동일성 검사가 거부했다. 실패 기록을 보존하고 v2에서
manifest의 JSON 표현만 정규화하며 부모 contract SHA 일치 검사는 유지했다.
원본 config의 삽입 순서로
모델을 구성하며, 정렬된 resolved config로 재구성하지 않는다. 부모 checkpoint SHA·계약·
E1/6,534 경계·optimizer parameter 순서·scheduler·AMP scaler·주 프로세스 RNG를 검사한다.
원본 checkpoint bytes와 30,750장·train 26,136/val 4,614·12 epoch·batch 4·일반 코사인·
compile OFF·best `val/center/mAP`·기존 로깅 이름은 유지한다.
실효 loader 정책과 새 extension SHA는 별도 계승 계약에 명시한다.

이후 학습은 E2부터 별도 W&B run에 이어 기록하며, epoch를 1로 다시 번호 매기지 않는다.
새 Drive 경로는 `backup:model/bevfusion/base5/full30750_speed_v1/checkpoint_backups`다.
새 목적지의 작은 write/readback, 실제 full payload 백업, 처리량 검증은 각각 구분한다.
기존 E24 full 백업 기법의 근거를 재사용하더라도 새 full 업로드 성공을 주장하지 않는다.
재개 준비 당시에는 첫 child checkpoint 백업이 미완료였으며, 이후 E2의 실제 완료 근거는
아래 후속 절에 기록한다.

### E1 전체 검증 복구 결과

동일 E1을 **batch 1·FP32·workers 4**로 실제 4,614장 평가해 exit 0으로 완료했다.
입력·추론·결과 수집은 **436.60초(7분 17초)**, 지표 집계까지는
**564.26초(9분 24초)**였다. 초기 준비·모델 구성 시간을 포함한 명령 전체는 689.35초다.
CPU 지표 집계 약 127.66초도 별도 비용이며 단순 GPU 추론 시간과 구분한다.
100장부터 마지막 4,614장까지 FD는 **62개로 일정**했다. 종료 PSS는 약 1.76GiB였고
E1 checkpoint SHA도 변하지 않았다. 이전 val의 4,220장·약 27분 반·실패와는 달리
이번에는 전체 지표를 생성했다. 다만 캐시·호스트 상태도 달라 모든 시간 차이를 FD 수정
하나에만 귀속하지 않으며, 과거 정상 구간 자체는 약 93ms/장이었다.

실제 `val/center/mAP`는 **0.8643879357**, 환경별 프레임은 각 750장, 보강은 114장이다.
기존 키를 포함한 834개 scalar를 JSON으로 보존했다. 이 복구 평가는 W&B에 쓰거나
optimizer를 step하지 않았다. 기존 run에 완성되지 않은 E1 val 이력을 꾸며 넣지 않으며,
복구한 E1과 E2부터 이어지는 계승 run을 함께 보존한다.
근거는 `full_e1_recovery_1792574972977419856.report.json`이다.

### 실제 재개 확인 — 2026-10-21 18:50 KST

E1/6,534 경계에서 전체 상태를 복원했으며 optimizer step은 저장된 **6,533** 그대로다.
첫 epoch의 AMP skip 1회를 임의로 보정하지 않았다. 복원 LR은 `9.854193209435755e-5`,
AMP scale은 128이며 parameter 이름·순서 검사를 통과했다. 18:50 KST에
**E2 120/6,534·time 0.6570초·data_time 0.0146초·loss 1.1532·gradient 18.0705**를 확인했다.
GPU는 100% 사용, peak 메모리 로그는 약 29,132MiB였다.
[새 W&B run](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/base5_full30750_e12_speed_261021_v2-97760ca4680d)도
epoch 2·global train iteration 6,654와 같은 loss/gradient를 실제로 수신했다.
기존 E1 run을 덮어쓰거나 epoch를 1부터 다시 시작하지 않았다.

계승 contract는 `97760ca4680d77451ecb0aee86269711176bc10b7f535d4f01c5e19fbf8f5998`이다.
`RESUME_VERIFIED_1792576218381343083.json`에 복원 상태·실제 iteration·W&B·백업 watcher를
기록했다. 백업 오류는 없으며 새 checkpoint는 아직 없어 업로드 성공으로 표시하지 않는다.
W&B·자격 갱신·Drive watcher를 위해 **로컬 PC를 켜 두어야 한다**.

현재 상태는 `bash scripts/runtime/start_base5_full_vast_training.sh`로 읽기만 확인한다.
기본 설정은 로컬 `artifacts/evaluation/base5_speed_repair_261021_v1/continuation_v2/launcher_speed_ready.json`이다.
이번 부모→계승 시작은 이미 실행됐으므로 `--run`을 중복 실행하지 않는다. 이후 child 안에
완료 checkpoint가 생긴 뒤의 명시적 재개는 `--run --resume`이다. 옛 full-data 설정은
`--config`로 명시했을 때만 기존 실행기로 분기하고, 알 수 없는 schema는 거부한다.
로컬 집중 회귀는 **91개 중 90개 통과·MMEngine 미설치 검사 1개 제외**였으며,
원격 실제 IPC/metric 회귀 13개와 전체 4,614장 검증은 따로 통과했다.
문서 링크 검사는 새 오류가 없고 기존 README/CURRENT 3파일의 줄 수 예산 초과만 남는다.

로컬 근거는 `artifacts/evaluation/base5_speed_repair_261021_v1/`의
`LIVE_DIAG_1792572916035005684.json`, `val_b124_fp32_1792573840125155564.report.json`,
`train_baseline_32_1792574434770362692.report.json`,
`train_candidate_32_1792574698628927250.report.json`,
`heatmap_fixture_gate_1792574188846766369.report.json`에 보존한다.
원격 운영은 새 `base5_performance_vast_training.py`를 사용한다. 실제 재개 전 준비와
사용자 승인·W&B·Drive·유휴 GPU·디스크·session lock 검사를 생략하지 않는다.

### E2 검증·첫 child checkpoint 백업 확인 — 2026-10-21 후속

10/21 **20:12 KST(11:12 UTC)** 관측에서 E2 train·val 완료 뒤 **E3 160 step 진행**을
확인했다. 이 관측 이후의 학습 상태나 12 epoch 완료를 미리 주장하지 않는다.
E2는 같은 4,614장·batch 1·FP32의 native validation을 완료했으며 AP는 아래와 같다.
학습 중 scalar는 0~1이고, 표에서는 읽기 쉽게 %로 표시했다.

| E2 native val 지표 | AP(%) |
|---|---:|
| `val/center/mAP` | 90.5435 |
| Vehicle / Pedestrian / Obstacle | 95.7524 / 94.3262 / 81.55194 |
| Obstacle 20~30m / 30~50m | 80.06359 / 30.0430 |
| Sunny15 Obstacle | 80.9043 |

정밀 scalar의 원본은 원격 work의 `val_logging/epoch_2.json`이다. 실제 console·E3 160 step·
첫 child 백업 상태를 읽어 보존한 로컬 receipt는
`artifacts/evaluation/base5_speed_repair_261021_v1/SUCCESSOR_OBSERVATION_1792581155370848658.json`이다.

첫 child **`epoch_2.pth` 전체 상태**의 실제 Drive 업로드와 로컬 인증 소유 PC의 전체
재읽기를 완료했다. `epoch=2`, `iteration=13068`, `resume_capable=true`, `errors=[]`다.
파일은 **522,174,725바이트 / 32MiB 단위 16파트**이며 SHA256은
`83005296010995b4d9a42dfb09675c1f5400c0ba4aad29582f7f47c7394f8f56`이다.
계약은 계승 run의 `97760ca4680d77451ecb0aee86269711176bc10b7f535d4f01c5e19fbf8f5998`이다.

Drive 위치는 다음 내용 주소형 경로다. 가중치를 받기 위해 학습 호스트에서 추가
백업·모델 실행을 하거나 현재 학습을 중단할 필요는 없다.

```text
model/bevfusion/base5/full30750_speed_v1/checkpoint_backups/
  base5_full30750_e12_speed_261021_v2/
    83005296010995b4d9a42dfb09675c1f5400c0ba4aad29582f7f47c7394f8f56/
```

완료 표식·`PARTS.json`·`parts_33554432`와 원래 manifest/resolved config를 함께 보존한다.
복원은 저장소의 `tools/base5_checkpoint_parts.py`로 각 파트와 전체 SHA를 확인한 뒤
새 출력 파일에 원자적으로 수행한다. 이 도구 경로는
`perception/detection3d/bevfusion/` 기준이다. 실제 업로드·readback 근거는
`artifacts/evaluation/base5_speed_repair_261021_v1/continuation_v2/logs/drive_backup.jsonl`의
`local_readback_verified`와 완료 status다. 이후 원격의 같은 파일명이 다시 저장되더라도
이미 검증한 SHA 폴더와 원래 바이트를 변경하지 않는다.

E2 val 상승을 데이터 보강만의 인과 효과로 부르지 않는다. 기존 Sunny13 E24와는 학습 데이터·
업데이트 수·compile/depth-raster 계약 등 차이가 있으며, 자체 val이 다른 숫자의 단순 차이는
공정한 비교가 아니다. 같은 실제 센서 입력의 로컬 비교는
[E2 대 E24 bag 검증](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-e2-local-bags-261021)으로
분리한다. 현재 production 가중치·threshold·tracker 정책은 유지한다.

<a id="full-data-e3-interrupted-evaluation-261022"></a>

### 학습 중단 뒤 완료 E3 복원·평가 — 2026-10-22 후속

사용자가 결제 누락으로 중단됐다고 알리고 E3 가중치 회수·val·bag 비교를 요청했다.
기존 direct SSH는 연결 거부였고, 인스턴스를 켜거나 학습을 재개하지 않았다.
기존 Drive 경로의 내용 주소형 폴더에서 **완료 E3 full checkpoint**를 복원했다.

```text
model/bevfusion/base5/full30750_speed_v1/checkpoint_backups/
  base5_full30750_e12_speed_261021_v2/
    96b31da0f475c2f081d133ccf26f219ffae16e9eb203f43d859f5ca8eb41f5f4/
```

16파트·전체 SHA와 **523,673,221바이트 / epoch 3 / iteration 19,602 / 계승 contract
`97760ca4680d77451ecb0aee86269711176bc10b7f535d4f01c5e19fbf8f5998`**를 확인했다.
로컬 파일은 `artifacts/evaluation/bevfusion/full30750_e3_rosbag_261022_v1/E3_full.pth`다.
Optimizer·scheduler·RNG를 포함한 원형 full이며, 실제 추론은 model state만 strict load했다.

W&B 읽기 전용 조회에서 **E3 native val 4,614장 완료**와 다음 E4 일부의 기록을 확인했다.
Val E3 행 `_step=655` 바로 앞 `_step=654`의 epoch-boundary는 train iteration 19,602다.
6환경 각 750장·보강 114장, GT V/P/O 3,972/2,850/3,685는 E2와 같다.
W&B summary의 마지막 epoch 4 / train iteration 22,642와 `crashed`는 완료 E3와 구분한다.
현재 종료 원인을 W&B만으로 확정하거나 E4 부분 진행을 E4 checkpoint 완료로 부르지 않는다.

같은 native val mAP는 **E2 90.5435→E3 89.4464%**,
V/P/O는 **95.7524/94.3262/81.5519→95.8955/93.3124/79.1312%**다.
E3의 보행자 40~50m AP 84.5401%, 차량 80~100m AP 73.9784%는 E2보다 높지만
전체 mAP best는 E2다. E3를 자동 best·production으로 승격하지 않는다.
Full checkpoint는 val 전에 저장돼 내부 `message_hub`의 마지막 val은 **E2**다.
Native E3 지표의 원본은 명시적인 W&B epoch 3 history 행이며, 내부 E2 값을 E3로 오독하지 않는다.

정밀 지표·분모·원본 행·조회 시각은 같은 로컬 root의 `native_val_wandb.json`에 보존했다.
Native val은 이미 완료된 결과를 회수한 것이며, bag 1,570프레임은 로컬에서 새로 추론했다.
동일 1,413평가 프레임의 E24/E2/E3 AP·고정 threshold·raw score·연속 누락과 한계는
[E3 비교 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-e3-local-bags-261022)를 따른다.
Production·운영 threshold·W&B run·기존 학습 source·가중치는 변경하지 않았다.

<a id="full-data-new-host-migration-261022"></a>

### 새 48GB 호스트로 최소 입력 복원 — 2026-10-22

사용자가 기존 GPU 재할당 불가와 인스턴스 간 복사 중단을 알리고 최소 입력 복원을 승인했다.
현재 대상은 **`root@99.9.4.202:33964` / proxy `root@ssh8.vast.ai:16748`**이며,
실제 GPU는 RTX 4090 **49,140MiB**다. 앞서 잠시 확인한 24GB 인스턴스와 구분한다.
이 주소를 향후에도 유효하다고 가정하지 않으며 기존 인스턴스를 Stop·Destroy하지 않았다.

새 호스트는 기본 Torch 2.7.1/CUDA 12.8 이미지였으며 MMCV·spconv 등은 없었다.
`/workspace` 복사는 설치 환경 복제가 아니다. 중단 직후 디스크 여유는 약 193GB여서
당시 용량 부족은 아니었고, 복사 중단의 원인을 데이터 개수만으로 확정하지 않았다.
전체 복사에는 과거 실험·중복·미완성 archive가 섞여 있어 현재 학습에 필요한 파일만 복원했다.

- 원래 10개 cohort 압축본은 **19,158,484,002바이트**다. 도착한 Sunny15
  **3,315,767,472바이트**의 전체 SHA가 일치해 재사용했다. 추가 입력은 약 **15.84GB**다.
  이미 Drive에 있는 8개 누락 archive를 file ID로 직접 수신하고, 마지막 1장 압축본
  519,600바이트는 로컬에서 보냈다. Sunny 보강은 동명 Drive 폴더 이력이 있어 ID를 사용한다.
  각 archive 크기·전체 SHA를 확인했으며 몇 MB/s 변동을 옛 8MB/s 기준으로 중단하지 않았다.
  최초 보강 162MB 시험은 수신·SHA 확인까지 약 3.4초, 이후 Sunny11·13은 각각 약 42초였다.
  이는 이번 호스트의 관측이며 다른 호스트나 이후 전송 속도의 보증이 아니다.
- 원래 index SHA **`481008024038779fd6eaf8e6bb2676461ccd201081ad0f8cb762718399d42d24`**와
  cohort별 manifest·infos·Q90/upright·3클래스·split을 유지한다. 새 데이터 생성이나 재분할이 아니다.
  최종 준비에서 누락됐던 2,724바이트 index도 같은 경로에 원문 그대로 배타적 복원·재읽기 검증했다.
- 동결 source **1,098개를 포함한 control/runtime 총 1,117개**, 부모 E1·manifest·E3 full을
  원래 SHA와 대조해 복원했다. 새 `last_checkpoint`는 원래 work의 `epoch_3.pth`를 가리킨다.
  E3는 epoch 3 / iteration 19,602다. 중단된 E4의 임의 지점부터 재생한다는 뜻이 아니다.
- 초기 가중치의 원래 경로에는 복사 실패 뒤 **읽기 ENOENT·생성 ESTALE**인 깨진 항목이 남았다.
  유효 파일을 지우거나 OS 마운트를 바꾸지 않고, 동일 **159,952,011바이트·SHA `a786…b977`**를
  `/workspace/base5_migration_control_261022_v1/initializer_original.pth`에 보존해 새 transport 설정만 연결했다.
  학습 identity는 이 파일의 SHA를 묶으며, 최종 ordered prepare의 부모 identity 일치도 통과했다.
- 데이터의 원래 `base5_full_data_delivery_261021_v1/cohorts`도 같은 잔존 항목 때문에
  mkdir가 ESTALE로 실패했다. 컨테이너 내 활성 writer가 없고 정상 cohort가 그 아래 게시되지 않았음을
  확인한 뒤 상위 실디렉터리 전체를 `.pre_migration_ghost_261022`에 no-overwrite rename으로
  보존하고, 동일 canonical 경로를 정상 directory로 다시 만들었다. 과거 partial·ghost를
  삭제하거나 마운트를 바꾸지 않았으며 index와 데이터 경로 계약도 변경하지 않았다.
- 원래 source tree에도 읽히지 않는 `__pycache__` 등 ghost 52개가 남아, ordered prepare의
  재귀 탐색이 실패했다. 기존 tree 전체를 동결 source 영역 밖인
  `/workspace/base5_migration_control_261022_v1/source_pre_migration_ghost_261022`에 보존하고,
  원래 canonical source 경로에
  동일 SHA의 동결 1,098개와 원래 pin의 diagnostic 지원 파일 1개만 깨끗하게 복원했다.
  원형 core의 세 폴더 재귀 수집 알고리즘으로도 1,098개 SHA 사전이 정확히 일치했다.
  기존 추가 파일 810개도 이전 tree에 그대로 남겼으며 개별 ghost 삭제나 코드 내용 수정은 없었다.
- 동일 Python/Torch/CUDA ABI의 검증 이미지에서 MMCV·BEVFusion CUDA ops만 재사용하고 필요한
  의존성을 설치했다. `/opt/conda` 전체 복제·Torch/CUDA 재설치는 하지 않았다. 두 BEV binary는
  외부 경로의 정확한 module-name hook으로 연결해 동결 source를 수정하지 않았다.
  원래 config import와 실제 CUDA rotated NMS·voxel·BEVPool forward/backward·sparse 소량 검사는 통과했다.
  선택적 open3d·plyfile·scikit-image·tensorboard가 없는 `pip check` 결과를 오류 0이라고 표현하지 않는다.
- 새 endpoint의 작은 Drive 쓰기·재읽기와 W&B 기존 health/run 읽기 검사는 통과했다.
  새 호스트용 백업 증거는 과거 전체 백업 구현과 **새 endpoint 검사**를 명시적으로 연결했다.
  새 전체 checkpoint 업로드나 처리량을 이미 검증했다는 뜻이 아니다. 갱신 가능한 Drive 인증은
  로컬에 유지하고 원격에는 RAM 단기 token만 전달했다. W&B 키도 stdin/RAM으로만 사용했다.
- E3 모델 574개 state tensor와 optimizer 394개 parameter의 이름·순서가 일치했다.
  Adam moment 780개·AMP scaler·두 scheduler·Python/NumPy/Torch/CUDA RNG를 실제 RAM에
  복원해 대조했다. checkpoint epoch 3 / iteration 19,602와 optimizer update 19,597은
  AMP의 건너뛴 update 때문에 구분하며, 검사 중 optimizer step·학습·checkpoint 저장은 없었다.
  이어 실제 Sunny13 val 1장·원래 model/pipeline·FP32/B1 forward에서 raw 후보 200개의
  7D BBox·score가 모두 유한값이었다. E3·모델 state·입력·동결 source 전후 SHA도 일치했다.
  첫 호출 약 60.8초는 cold 준비 비용을 포함하므로 실시간 latency나 전체 AP의 근거가 아니다.
  첫 진단기의 numeric `sample_idx`와 문자열 token 비교 오류는 실패 로그를 보존하고,
  원래 동결 로깅의 `_sample_token`으로만 정정했다. 데이터나 pipeline을 수정하지 않았다.
- 최초 압축 전개 뒤 검증 보조 모듈 누락으로 4개 cohort의 게시가 중단됐다. 이미 전개한 파일과
  실패 기록을 보존하고, 동결 source 밖에 원래 검증 Python 54개와 catalog 2개를 SHA 고정해 추가했다.
  원래 ordered prepare에서도 이를 찾도록 새 전용 `.pth` 하나로 외부 경로를 기존 경로 끝에
  추가했다. 새 Python import·catalog 146개 로드와 기존 `.pth` 6개·source/helper SHA 불변을 확인했다.
  중복 SHA 읽기를 줄이는 stat 캐시 후보는 파일시스템의 timestamp 해상도 예외로 변경 감지를
  놓쳐 기각했다. 실패 근거를 보존하며 원래 전체 바이트 SHA·package/native 검증을 사용한다.
  이는 데이터 손상·GT 변경이나 검증 기준 완화가 아니다.
- 최종 데이터 복원은 **10개 cohort·30,750장 / train 26,136·val 4,614**로 완료했다.
  원래 archive/member SHA·package/native 검사, Q90·3클래스·분할·원래 경로를 유지했다.
  기존 4개 전개물과 그중 먼저 완료된 2개 성공 증거는 재사용했고, 나머지 6개만 처음 전개했다.
  전체 실패 이력·ghost 격리 전후·10개 결과는 `data/receipts_261022_v1/`에 회수했다.
  완료 후 디스크 여유는 약 109GB였으며 전체 데이터 본문을 로컬로 재전송하지 않았다.

새 설정은 `artifacts/evaluation/base5_vast_migration_261022_v1/preflight/launcher_new_host_ready_v2.json`이다.
원래 동결 launcher와 과거 증거는 덮어쓰지 않았다. 위 준비 설정은
**`actual_training_start_approved=False`**이며 준비·복원 승인을 실제 학습 시작 승인으로 바꾸지 않는다.
기존 인자 없는 `start_base5_vast_training.sh --resume`가 이 새 설정을 자동 선택한다고
가정하지 않는다. 실제 재개 시에는 새 endpoint·이 설정·별도 시작 승인·RAM 인증 전달을 연결한다.
최종 원형 `remote_check(resume=True)`도 **75.96초·exit 0**으로 통과했다.
데이터·source·초기화·원래 순서의 config 재구성과 부모 identity, E3 resume binding,
W&B health·백업 gate가 통과했고 원래 계약
**`97760ca4680d77451ecb0aee86269711176bc10b7f535d4f01c5e19fbf8f5998`**를 유지했다.
최종 영수증은 `preflight/prepared_resume_1792651142824148689.json`이며,
`training_started=False`, `optimizer_steps_performed=0`이다.
별도 시작 승인 뒤 E4 처음부터 남은 9 epoch를 이어갈 준비이며, 12 epoch 완료나 실제 재개를
이미 했다는 뜻이 아니다. 초기 누락 index·helper import·source ghost로 실패한 검사 기록도 보존했다.
근거 root는 `artifacts/evaluation/base5_vast_migration_261022_v1/`의 `data/`, `environment/`,
`control/`, `preflight/`, `qualification/`, `review/`다. 데이터 전송 실행기의 독립 CPU 회귀는
**18/18**, 캐시를 제거한 전체 SHA 복구 경로의 회귀는 **5/5 통과**했다.
기각된 캐시 후보의 실패 테스트를 이 통과 수에 섞지 않는다.

<a id="full-data-e3-resumed-new-host-261022"></a>

### E3에서 E4 실제 재개와 백업 확인 — 2026-10-22

위 준비 완료 뒤 사용자가 **“응 학습 돌려줘”**라고 별도로 시작을 승인했다.
기존 준비 설정·영수증은 그대로 보존하고, 새 `start/authorized_config.json`과
`start/start_authority.json`에 총 12 epoch·원래 부모 계보·데이터·runtime 정책 및 실제
재개점 E3 SHA/epoch 3/iteration 19,602를 연결했다. 원래 계약 `97760c…f5998`을 유지한다.
모델·학습/검증 파이프라인·optimizer/scheduler·source·초기화·production은 바꾸지 않았다.

- 원격 승인 scope는 `/workspace/base5_migration_start_261022_v1`이다. 실제 원형
  `remote_start`의 중복 실행/온라인 검사, GPU lock, `remote_worker`, ordered prepare와
  full-state resume를 그대로 사용한다. 외부 bootstrap이 정확히 한 번의 tmux worker 명령만
  연결하며, 새 private `TMUX_TMPDIR=/dev/shm/base5_e3_resume_261022_v1`을 0700으로 만들었다.
  원격 세션은 `base5-full30750-e3-host99-training`이다.
- W&B 키는 SSH stdin→프로세스/새 tmux 서버의 RAM 환경으로만 전달한다. worker 진입에서
  키 존재와 config/bootstrap/E3 pin을 검사하며 argv·설정 파일·원격 `.netrc`에 쓰지 않는다.
  새 호스트에서 검증한 **직접 HTTPS**를 사용한다. 동결 코드를 고치지 않고 외부 transport의
  proxy 환경만 비운다. 모든 로컬 SSH 경로는 기존 migration host-key pin을 사용한다.
- Drive는 원래 access-only uploader·32MiB parts·16 transfers·로컬 readback을 유지한다.
  로컬 새 세션 `base5-full30750-e3-host99-backup`이 갱신 가능한 인증을 소유하고, 원격에는
  단기 access token만 RAM으로 전달한다. 이전 terminal 상태만 보고 watcher가 끝나지 않도록
  이번 worker 진입 관측을 요구한다. **Drive 자동 백업을 계속하려면 로컬 PC를 켜 둬야 한다.**
  GPU 학습과 이번 직접 HTTPS W&B logging 자체는 로컬 relay를 사용하지 않는다.
- 실제 원격 콘솔에서 **07:44:58 UTC / 16:44:58 KST**에 trusted `epoch_3.pth`를 읽고
  `resumed epoch: 3, iter: 19602`를 확인했다. 16:46:54 KST에 E4 80/6,534까지 진행했고
  loss 0.8483·grad norm 10.7802, GPU 31,194MiB/92%가 관측됐다. OOM·Traceback·CUDA 오류는 없었다.
  준비 event의 `parent_epoch=1`은 기존 성능 개선 successor의 계보이며 실제 복원 경계가 아니다.
  후속 **16:50:27 KST** 관측에서는 E4 **400/6,534**, 약 0.6648초/step·loss 0.7418까지 진행했다.
- **16:47:22 KST**의 읽기 전용 API에서 기존 같은
  [W&B run](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/base5_full30750_e12_speed_261021_v2-97760ca4680d)이
  `running`이며 오늘 새 history 5행을 확인했다. epoch 4 / global iteration
  **19,622→19,702**, LR **8.60006e-5→8.58892e-5**, loss 약 **0.83~0.91**다.
  워밍업 뒤 **0.662~0.668초/step**, data time 약 **0.015~0.016초**였고 첫 20 step 평균
  3.77초는 cold 준비 비용을 포함한다. 학습 compile OFF는 유지한다.
  과거 중단된 E4 부분 history(iteration 22,642~22,682)는 어제 timestamp로 남아 있다.
  `_step`은 계속 증가하지만 논리 train iteration은 E3 경계에서 다시 시작하며, 과거 행을 삭제하지 않는다.
- 이번 새 호스트에서 **E3 full 523,673,221바이트·16 parts**를 실제 Drive 업로드하고
  로컬에서 전체 재다운로드·SHA `96b31d…1f5f4`를 대조했다. 약 **107.33초**, COMPLETE/LATEST
  재읽기까지 완료했고 이후 poll은 pending/errors가 없다. 기존 backup 상태의 E2/best E2 기록을
  이번 새 업로드로 세지 않는다. **E4 이후 새 checkpoint의 백업 완료는 아직 확인한 것이 아니다.**

원격 콘솔은 `/workspace/base5_full30750_e12_speed_261021_v2/console_resume_1792654915.log`,
로컬 백업 로그는 `start/logs/watcher.log`와 `start/logs/drive_backup.jsonl`이다.
근거는 같은 migration root의 `start/`와
`wandb_observation/observation_1792655242889963716.json`에 보존했다.
현재 실행 요약은 `start/TRAINING_RUNNING.json`, 원격 실제 관측은
`start/remote_observation_261022_e4_running_v1.json`을 따른다.
이 절은 실제 재개 확인이며, 위 `MIGRATION_PREPARED.json`의 학습 미시작 상태는 당시 이력으로 유지한다.

## 상세 운영과 실측의 보존

## Sunny15 스케줄 대조 준비 — 2026-10-18

사용자는 기존 48GB 4090 매물을 유지하기로 했다. 새 이미지로 재대여하거나 기존
인스턴스를 Destroy하지 않는다. 새 Sunny15 본학습은 아직 시작하지 않는다.

위 문단은 준비 당시 이력이다. 이후 사용자가 일반 코사인 학습을 시작했으며,
현재 실행 관측은 [실제 학습 기록](#sunny15-live-training-261018)을 따른다.

- 별도 원격 소스: `/workspace/ajounice_sunny15_training_source_261018_v2`.
  기존 소스 복사본에 변경분을 전송하고 1,797개 파일의 SHA를 확인했다. 기존 E24 source·
  run·데이터는 변경하지 않았다. CUDA 확장은 기존 설치본에서 새 복사본으로 재사용한다.
- 일반 코사인과 재시작형은 8 epoch·batch 4·누적 1·workers 4·lr 1e-4·seed 261018을
  공통 사용한다. compile OFF·nearest·512×1056·5cm/20cm·depth 25cm·3클래스를 유지한다.
- 재시작형은 고정 4 epoch 주기 두 번, 주기마다 500 iteration warm-up,
  시작/최대/최소 LR 비율 0.1/1/0.01이다. 주기 길이 증가·최대 LR 감소·optimizer reset은 없다.
  katsura-jp 코드는 참고만 했고 외부 패키지를 설치하거나 그대로 복제하지 않았다.
  MMEngine이 유일하게 scheduler를 step하며 parameter-group LR 비율을 보존한다.
- 스케줄 목록과 W&B group을 새 run contract에 넣었다. 서로 다른 스케줄·데이터·코드의
  optimizer를 강제 resume하지 않는다. 기존 원격 E24 실행기는 그대로다.
- CPU에서 경계/비율/저장·재개와 기존 runner 회귀 9개를 통과했다. 원격에서도 scheduler
  3개가 통과했다. 원격 기존 Sunny13의 7장 표본으로 두 스케줄 각각 4회 AMP iteration과
  실제 저장·중단·재개도 통과했다. 두 보고서의 model·optimizer/scaler·scheduler·Python/
  NumPy/Torch/CUDA RNG·iteration·compiler prefix 검사는 모두 true다. 새 Sunny15의 AP나
  8 epoch 수렴·W&B/Drive 업로드 인증으로 해석하지 않는다.
  원격 근거는 `/workspace/sunny15_env_qualification_261018_v1/{cosine,restarts}/report.json`이다.
- W&B group은 `base5-sunny15-ab-261018`이다. 기존 `base5-sunny13` run을 재사용하지 않는다.
  거리별 AP 진단 wrapper를 유지한다. 자동 Drive 백업은 새 데이터와 run 계약 확정 뒤
  인증·write/readback·실제 checkpoint 업로드를 다시 검증해야 한다.
- Docker Hub `base5-cu128-261018-v1`은 이 후속 scheduler 구현 이전의 소스 snapshot이다.
  재시작형이 그 이미지에 이미 들어 있다고 가정하지 않는다. 현재 기존 Vast의 별도 소스에
  준비한 것이며 향후 이미지 갱신은 새 태그로 한다.

원격 Python 환경에서 CPU 준비하는 명령은 아래와 같다. `JPEG_PORTABLE`은 **새 Sunny15**
검증본으로 교체한다. 기본 동작은 준비뿐이다. 실제 시작 전 초기화·GPU·W&B·Drive 자격을
확인하며 구 E24 oneclick 명령으로 이 작업을 시작하지 않는다.

```bash
cd /workspace/ajounice_sunny15_training_source_261018_v2
bash scripts/runtime/prepare_sunny15_ab.sh cosine "$JPEG_PORTABLE" /workspace/sunny15_ab_261018
bash scripts/runtime/prepare_sunny15_ab.sh restarts "$JPEG_PORTABLE" /workspace/sunny15_ab_261018
```

### 코드 전송과 Drive 인증

직접 scp의 약 3MB archive는 30초 제한으로 중단했으며 완전하지 않은 archive는 사용하지
않았다. 이후 원격 소스를 재사용한 차이 전달은 완료했고 실제 파일 SHA를 검산했다.
본 데이터 전송은 시작하지 않았다. 코드/설정/추가 패키지도 전달 대상이며 데이터셋만
Drive 우회 대상이라고 해석하지 않는다.

동일 소스 archive의 SHA는
`1a47604550f2e12ffcfe523f1cc4fe579b2eea93a7150d40ed7a0c8bb2cc5697`이다.
팀 Drive의 `model/bevfusion/runtime_sources/sunny15_ab_261018/source_v1.tar.gz`에 업로드했다.
원격 직접 다운로드 완료·지속 10MB/s 충족은 아직 인증하지 않았다.

전용 `ajou_drive_private`는 `invalid_grant`로 사용할 수 없었다. 기존 `ajou_drive`로
업로드는 됐으나 후속 목록 조회에서 공유 API project의 `403 rateLimitExceeded`가 발생했다.
반복 요청을 중단했고 사용자에게 로컬에서 `rclone config reconnect ajou_drive_private:`로
전용 연결을 재인증하도록 요청했다. 장기 갱신 토큰·client secret은 원격으로 전달하지 않는다.
재인증 전 Drive 전송·백업이 준비됐다고 보고하지 않는다. 작은 코드 archive의 완료 시간은
대용량 데이터의 지속 대역폭 인증과 구분하며 10MB/s 기준 미달 전송을 장시간 유지하지 않는다.

후속 사용자 재인증으로 전용 연결은 복구됐고, 로컬에서 동일 파일의 목록/ID 조회가 성공했다.
전용 client 설정이 Downloads에 보존된 OAuth 앱과 일치함도 값 노출 없이 확인했다.
그러나 원격 rclone 다운로드는 15초 하드 제한에 걸렸고, 파일 ID 직접 Drive API 요청도
약 6.9초 후 TimeoutError로 끝났다. 추가 전송 시도는 중단했다. 이는 로컬 인증 복구와
원격 통신 성공이 별개임을 보여 주며, Drive→Vast 전송 성공·속도나 백업 완료의 근거가 아니다.
코드는 이미 별도 차이 전송으로 배포·GPU 검증됐으므로 이 실패를 코드 배포 실패와 혼동하지 않는다.

### 새 데이터 전용 시작기와 남은 차단 조건

기존 E24 시작기와 분리한 `scripts/runtime/start_sunny15_vast_training.sh`를 준비했다.
인자 없이 후보 이름만 주면 상태 조회만 한다. `--run`이 없으면 본학습을 시작하지 않는다.
로컬 설정은 `artifacts/evaluation/bevfusion/sunny15_training_readiness_261018_v5/`의
`cosine.json`·`restarts.json`, 원격 실행기는 `/workspace/sunny15_runtime_261018_v5`다.
코드·초기화 가중치·두 스케줄의 짧은 GPU/저장·재개 시험은 완료했으며, 기존 Sunny13 데이터로
새 Sunny15 학습을 잘못 시작하지 않도록 새 5K portable manifest의 SHA를 명시적으로 연결해야 한다.
원본 선택 ledger의 SHA·5,000개 고유 승인·portable provenance와 각 native metadata SHA·
실제 `fresh_environment_provenance.environment=sunny15`도 검사한다. 폴더 이름이나 설정 문자열만
바꾼 구 Sunny13 패키지는 연결하지 못한다. 48GB GPU의 실제 여유·W&B 연결·전체 checkpoint
백업 검증도 시작 전에 확인한다.
두 후보는 별도 work/W&B run을 사용하며 같은 GPU에서 동시에 시작하지 못하도록 잠근다.

```bash
# 로컬 저장소에서 실행한다. 두 명령 모두 조회뿐이다.
bash scripts/runtime/start_sunny15_vast_training.sh cosine --status
bash scripts/runtime/start_sunny15_vast_training.sh restarts --status

# 새 Sunny15 Q90/upright 5K를 전송·해시 검증한 뒤, 각 후보에 동일한 manifest를 연결한다.
python3 -B scripts/runtime/sunny15_vast_training.py \
  --config artifacts/evaluation/bevfusion/sunny15_training_readiness_261018_v5/cosine.json \
  --bind-local-manifest "$JPEG_PORTABLE/manifest.json"
# restarts.json에도 같은 명령을 적용한다.
bash scripts/runtime/start_sunny15_vast_training.sh cosine --prepare
bash scripts/runtime/start_sunny15_vast_training.sh cosine --check
# --check의 모든 조건을 통과하고 사용자가 학습을 승인한 뒤에만 --run을 사용한다.
```

최종 Sunny15 5K를 원격에 복원·전수 검증하고 두 후보에 같은 manifest SHA를 연결했다.
두 후보 모두 `--prepare`와 GPU·W&B·Drive `--check`를 통과했으며 `training_started=false`다.
실제 시작 때는 온라인 사전검사를 다시 수행한다. 상태 조회의 보수적인 `ready=false`는
온라인 자격을 상태 조회만으로 인증하지 않는다는 뜻이며, `data_and_contract_ready`와 구분한다.
조회 시점에 GPU 49,140MiB와 W&B 0.22.3·기존 온라인 probe의 인증 조회를 확인했다.
새 시작기·백업 자격 회귀 24개와 기존 oneclick 회귀 19개가 통과했다. 이는 새 5K 전체 학습이나
새 데이터의 AP 검증 결과가 아니다.

Drive는 로컬 갱신 자격증명을 유지하고 원격에는 RAM의 단기 access token만 전달하는
기존 제한 relay를 재사용한다. 작은 receipt의 업로드·동일 내용 재확인은 34.68초에 성공했다.
실제 checkpoint의 앞 32MiB 전송은 25초 제한 안에 완료하지 못해 중단했다.
API 초기 지연과 payload 전송을 분리한 시험이 아니므로 이를 0MB/s나 지속 10MB/s 미달의
측정값으로 표현하지 않는다. 약 519MB 전체 checkpoint 재전송은 시작하지 않았다.

후속 대조에서 기존 E24의 **524,552,773바이트 전체 readback·SHA·resumable 판정**과
현재 로컬 3개·원격 2개 backup helper의 SHA가 정확히 같은 것을 확인했다. 같은 backend와
Drive root, 새 destination의 소량 쓰기·동일 내용 읽기 결과를 연결해 v5의 별도
`reused_full_backup_qualification.json`으로 재사용 자격을 검증했다. 시작 전 원본 근거·도구·
원격 helper·자격증명 앱/프로필·새 destination 연결을 다시 확인한다. 작은 probe 단독으로는
통과하지 않는다. **검증된 백업 방식의 자격 재사용**이지 새 Sunny15 checkpoint 업로드 완료가
아니다. `new_checkpoint_uploaded=false`를 명시하며 첫 새 epoch의 full checkpoint·metadata·
SHA readback은 실제 생성 후 반드시 확인한다. 이 검증을 위해 같은 대용량 파일을 다시
올릴 필요는 없다. 당시 남았던 새 Sunny15 5K 전달본 연결은 이후 전수 검증·전송·연결까지 완료했다.
근거는 `artifacts/evaluation/bevfusion/sunny15_training_readiness_261018/`의
`drive_small_probe.json`·`backup_transport_probe.json`과
`sunny15_training_readiness_261018_v3/network_environment_probe.json`이다.
최종 v5의 `readiness_summary.json`·`reused_full_backup_qualification.json`에 재사용 근거와
남은 검증 범위를 기록했다. 이전 v1~v4는 학습 미시작 이력으로 보존한다.
학습 중 로컬 relay/백업을 사용하는 경우 이 PC와 네트워크를 계속 켜 둬야 한다.

### 최종 Sunny15 전달 형식과 전송 재측정

최종 5K의 입력 형식·변환 중 발견한 장면 제외 전제·압축·RGB 원본 정리는
[데이터 전달본 근거](../../dataset/morai_od/docs/261018_base5_unified_generation_capture.md#sunny15-training-package)를 따른다.
이미지를 미리 512×1056으로 잘라 압축하는 방식이 아니다. 원본 해상도 Q90 JPEG와
현재 resize/crop·upright 7D·Animal→Pedestrian·최종 split을 결합한 portable package다.
`tar.zst`는 그 전달본을 무손실 압축하며 PNG·중간 cache를 다시 싣지 않는다.

동일 Vast에서 짧게 재측정한 SSH 전송은 direct 약 0.09MB/s(중단 전 확인량 기준),
proxy 약 0.81MB/s였다. 새 데이터의 대용량 SSH 전송은 시작하지 않았다. 사용자 승인 후
단기 Drive access token만 원격 RAM으로 전달해 기존 백업의 일부를 읽었다. 단일 32MiB는
payload 약 7.65MB/s였고, 4병렬은 약 50.9MB를 받은 시점의 합산 약 9.87MB/s로 기존
10MB/s 기준에 따라 중단했다. startup과 payload 시간을 구분한 짧은 측정이며 대용량의
지속 속도를 인증하지 않는다. 장기 갱신 토큰·client secret은 원격으로 보내지 않았다.

이후 사용자가 **8MB/s 이상이면 진행하고, 그 아래로 지속되면 중단**하도록 당시 기준을
명시적으로 변경했다. 이는 당시 전송의 기준이며, 이후 10/18에 사용자가 8MB/s를 엄격하게
적용할 필요 없이 10KB/s 같은 비정상적 정체만 아니면 진행하라고 정정했다.
이전 10/8MB/s 미달 판정과 측정 이력은 고쳐 쓰지 않으며 과거 고정 guard를 새 요청에
자동 적용하지 않는다. 근거는 v5 준비 폴더의 `transfer_probe_261018.json`과
`drive_range_probe_authorized_261018.json`이다. 원격 환경·초기화·GPU 자격은 재사용하되
실제 새 데이터 연결·전체 payload 해시·온라인 사전검사 통과 전에는 ready로 표시하거나
본학습을 시작하지 않는다.

<a id="최종-사용자-실행-명령--학습-미시작"></a>

### 최종 사용자 실행 명령 — 준비 당시 기록

최종 manifest SHA는 `aea7d157f5cc646a125c6e2f603108d81990a17ee9e1f516e433be5a4be6a2c7`이다.
원격 data는 `/workspace/base5_sunny15_jpeg90_pedestrian_merged_261018`이며 train 4,250/val 750,
각 후보 8 epoch·batch 4·누적 1·workers 4·lr 1e-4·compile OFF·nearest다.
epoch당 optimizer update는 1,063회다. 준비와 `--check`만 실행했으며 본학습은 시작하지 않았다.

| 후보 | 준비한 contract SHA |
|---|---|
| cosine | `d68f0824772a2f6da30520cc4c91b28906ecd629a7a2e721da7a8ddaa156975e` |
| restarts | `7d345c6b32761b3fbd2ba5180fa1a2181325ce35003ffe11c0031111839af8f8` |

사용자가 시작할 때 로컬 저장소에서 아래 명령 중 **한 개만** 실행한다. 같은 GPU에서 두 후보를
동시에 시작하지 않는다. 첫 비교의 결과를 본 뒤 다른 후보를 같은 초기화에서 새로 시작한다.
미시작 후보에 `--resume`를 붙이거나 기존 E24 시작기를 사용하지 않는다.

```bash
cd ~/AjouNice2026
bash scripts/runtime/start_sunny15_vast_training.sh cosine --run

# 별도 비교 후보를 시작할 때만 사용한다. 위 학습과 동시에 실행하지 않는다.
# bash scripts/runtime/start_sunny15_vast_training.sh restarts --run
```

로컬 제한 relay가 W&B와 백업을 처리하므로 이 PC와 네트워크는 켜 둔다. 두 후보의 실제
새 checkpoint 업로드는 아직 없으며 첫 새 epoch의 전체 외부 백업 검증이 남아 있다.
최종 점검 근거는 v5 폴더의 `prepared_ready_261018.json`과 `readiness_summary.json`이다.

<a id="sunny15-live-training-261018"></a>

### 사용자 시작 후 실제 학습 관측 — 2026-10-18

아래 20:53~20:57 기록은 첫 epoch 당시 이력이다. 최종 E8 완료와 후속 학습은
[완료·재시작형 실행](#sunny15-cosine-complete-restarts-start-261018)을 따른다.

사용자가 일반 코사인 후보를 시작한 뒤 읽기 전용으로 확인했다. 재시작·설정 변경·추가 학습은
수행하지 않았다. Run은 `cosine-d68f0824772a`이며
[W&B](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/cosine-d68f0824772a) 수신도 확인했다.

20:53~20:56 KST 관측에서 1 epoch train 1,063회와 val 750장을 완료했다. 최근 iteration은
약 0.662초, data time은 약 0.0134초이며 GPU 약 30.2GiB/48GB를 사용했다.
첫 center mAP는 **78.90%**, Vehicle/Pedestrian/Obstacle은 각각 **88.74/84.96/63.00%**다.
이는 새 val의 초기 관측이며 기존 E24와 동일 셋 비교나 최종 모델 선정 결과가 아니다.
초기 AMP 구간에 `grad_norm: inf` 1회가 있었으나 이후 유한값으로 복귀했고, 반복 NaN·OOM은
관측하지 않았다. `epoch_1.pth`와 best checkpoint가 실제로 생성됐다. 첫 저장 snapshot은
519,510,789바이트였고 validation 후 atomic 갱신한 최종 epoch 파일은 519,738,245바이트다.

첫 epoch의 실제 외부 Drive 백업이 시작됐으나, 확인 시점에는 업로드·전체 SHA readback이
끝나지 않았다. 진행 중인 업로드의 약 49초 표본은 41.9MB/519.5MB·약 1.07MB/s·오류 0으로
느렸다. 이전 백업 방식 자격을 이번 checkpoint의 업로드 완료로 표현하지 않는다.
학습 프로세스와 checkpoint 저장, W&B 수신, 외부 백업 완료는 각각 별도로 확인한다.

20:57 KST에는 2 epoch 80/1,063·0.670초/iteration·loss 1.1207·유한 gradient로
학습이 계속됐다. 백업은 약 100초에 67.1MB·약 0.75MB/s·오류 0이며 아직 완료 전이다.
별도 watcher의 업로드는 학습을 동기적으로 막지 않는다. 현재 업로드는 첫 저장 snapshot이고,
기존 helper가 완료 후 fingerprint 재확인으로 validation 후 최종본을 후속 처리한다.
따라서 이전 snapshot의 업로드 완료만으로 최종 checkpoint 백업을 완료로 판정하지 않는다.
전송 경로는 Vast→독립 SSH 터널→로컬 제한 relay→Drive이며, 현재 로그로 어느 구간이
병목인지는 구분하지 못했다. W&B는 별도 연결을 사용한다.

첫 train 약 11분 45초·val 약 1분 25초를 단순 확대하면 8 epoch는 약 1시간 45분~2시간이다.
백업 대역폭·후속 val 비용에 따라 달라지며 완료 보장이 아니다. 학습 중에는 로컬 relay와
네트워크를 유지하고, 기존 run과 같은 후보를 중복 시작하지 않는다.

<a id="sunny15-cosine-complete-restarts-start-261018"></a>

### Sunny15 cosine E8 완료·재시작형 실행 — 2026-10-18

사용자가 일반 코사인 실험 완료를 알리고, 기존 Sunny13 E24와의 간단 비교 및 다음 실험의
즉시 시작을 요청했다. 비교를 기다리지 않고 별도 agent가 기존 restarts 후보를 시작했다.
데이터셋·소스·CUDA를 다시 전송·설치하지 않았다. 빌린 수집 PC의 원격 PNG는 사용자
재확인대로 보존하며, 이 모델 실험과 별개의 Sunny13 로컬 회수·전수 검증도 완료했다.

일반 코사인은 exit 0이며 E8이 최종이자 자체 val best다. 학습 로그는 20:42:16~22:29:13 KST로
8 epoch·validation 포함 **1시간 46분 57초**다. 마지막 train은 0.6682초/iteration,
data 0.0150초·메모리 29,441MiB였다. mAP의 E1~E8 기록은
78.90 / 82.34 / 85.80 / 86.75 / 86.70 / 88.37 / 88.43 / 88.49%다.

| 지표 | 구 Sunny13 E24 자체 val | 새 Sunny15 E8 자체 val |
|---|---:|---:|
| Center mAP | 83.47% | 88.49% |
| Vehicle | 89.90% | 94.60% |
| Pedestrian | 87.73% | 91.17% |
| Obstacle | 72.79% | 79.69% |
| Obstacle 0~10m | 85.32%, GT 206 | 86.74%, GT 209 |
| Obstacle 10~20m | 86.13%, GT 190 | 81.50%, GT 180 |
| Obstacle 20~30m | 76.74%, GT 143 | 71.06%, GT 116 |
| Vehicle 50~80m | 58.83%, GT 51 | 83.87%, GT 78 |
| Vehicle 80~100m | 29.38%, GT 6 | 73.09%, GT 42 |

이는 같은 이름의 center AP(0.5/1/2/4m 평균)지만 **같은 검증셋에서 교차 평가한 결과가 아니다**.
구 val은 745장·V/P/O GT 341/687/674, 새 val은 750장·650/454/544다. 조명·배치·지원 객체·
분할·nearest·학습 compile 및 스케줄 길이 차이도 있다. 따라서 mAP +5.01pp를 순수 모델
개선량으로 단정하지 않는다. 원거리 차량은 유망하지만 10~30m 장애물은 아직 공통 평가가
필요하다. 구 E24의 epoch 24 train median 0.564초/iteration과 새 OFF의 iteration 비용도
구분한다. 구 학습 E2~24 resume 구간 약 4시간 36분은 E1을 포함한 정확한 총시간이 아니다.
새 E8을 production으로 승격하거나 구 E24를 교체하지 않았다.

<a id="sunny15-first8-distance-review-261019"></a>

#### 첫 8 epoch와 장애물 20~30m의 실제 GT 대조 — 2026-10-19

사용자가 요청한 비교 기준은 구 실험의 최종 E24가 아니라 **첫 E1~E8 안에서의 최고 결과**다.
아래는 기존 로그·infos·변환 보고서를 읽기 전용으로 다시 집계한 결과다. 추론·재학습·설정 변경은
하지 않았으며, 별도로 진행하는 Sunny11 수집 준비와도 독립이다.

| 선정 기준·지표 | 구 Sunny13 첫 8 epoch | 새 Sunny15 cosine 첫 8 epoch |
|---|---:|---:|
| 전체 center mAP 최고 | E8, 82.57% | E8, 88.49% |
| 위 전체 최고 epoch의 Obstacle 전체 AP | 69.27% | 79.69% |
| 위 전체 최고 epoch의 Obstacle 0~10m | 83.83%, GT 206 | 86.74%, GT 209 |
| 위 전체 최고 epoch의 Obstacle 10~20m | 80.45%, GT 190 | 81.50%, GT 180 |
| 위 전체 최고 epoch의 Obstacle 20~30m | 72.90%, GT 143 | 71.06%, GT 116 |
| **20~30m 구간 자체 최고** | **E5, 74.59%** | **E6, 71.53%** |

전체 최고끼리의 20~30m 격차는 −1.84pp, 구간 최고끼리는 −3.06pp다. 앞 표의 E24 대 E8 격차
−5.68pp와 구분한다. 새 E8의 전체 Vehicle/Pedestrian AP는 94.60/91.17%로 구 E8의
88.89/89.55%보다 높지만, 모든 거리·클래스가 상승한 것은 아니다. 예를 들어 Pedestrian
20~30m는 91.33→86.68%다. 검증 장면과 자산 구성이 달라 이를 그대로 순수 성능 변화로 해석하지 않는다.

새 cosine의 Obstacle 20~30m AP는 E1~E8에서
53.69 / 58.41 / 65.74 / 67.48 / 67.20 / 71.53 / 71.01 / 71.06%다.
후반 정체가 있어 추가 학습만으로 해결된다고 보장할 수 없다. 구 실험은 24 epoch 스케줄의
E8 마지막 LR이 약 `7.7021e-5`였지만 새 실험은 8 epoch 안에 약 `1e-6`까지 내린다.
같은 8 epoch라도 최적화 단계가 다르며, 끝난 실험을 최저 LR로 단순 연장하는 것과
처음부터 긴 스케줄로 학습하는 것도 다르다. 현재 restart의 E1~E4 해당 AP는
54.75 / 64.79 / 63.53 / 65.20%다. 이 조회 시점까지 restart의 우위를 확인한 것은 아니다.

##### 배치 보강과 GT 제외를 분리한 확인

| 실제 train infos의 Obstacle GT | 구 Sunny13 | 새 Sunny15 | 변화 |
|---|---:|---:|---:|
| 0~10m | 780 | 1,268 | +62.6% |
| 10~20m | 837 | 1,187 | +41.8% |
| 20~30m | 610 | 677 | +11.0% |
| 전체 거리 | 2,615 | 3,414 | +30.6% |

이 수는 프레임 수나 고유 자산 수가 아니라 학습 infos의 instance 관측 수다.
`bbox_label_3d == 2`를 선택하고 `hypot(bbox_3d[0], bbox_3d[1])`에
`lower <= distance < upper`를 적용했다. 20~30m GT가 있는 train 프레임은 455→513장이다.
절대 GT는 늘었지만 전체 Obstacle 내 해당 구간의 비중은 23.3→19.8%로 낮아졌다.
증가율 11%만으로 충분하거나 부족하다고 판정하지 않는다.

보강이 근거리에 더 집중된 이유는 당시 [생성기 설계](../../dataset/morai_od/docs/261013_base5_obstacle_generator_experiment_plan.md#현재-근거와-해석)와
[실제 비교](../../dataset/morai_od/docs/261013_base5_obstacle_support_experiments.md#후보-변경)에 있다.
기존 0~20m 노출은 Base3보다 크게 적었지만 20~30m의 프레임당 GT 수는 비슷했고,
후자는 점 희소성·가림·형상 학습도 별도 문제로 보았다. 기존 중거리 객체까지 근거리로
이동한 초기안은 중거리 감소 때문에 기각했다. 채택한 rich-floor는 기존 20~30m 보조 객체를
보호하되 새 추가 객체는 주로 6~20m에 배치하고 일부 기존 30m 이상 보조 객체만 20~29m로
이동한다. 이후 전방 지원 보완도 주로 6~14m를 늘렸다. 따라서 20~30m를 대폭 늘린 정책이
아니며, 그 구간 AP 향상이 소량 배치 시험으로 입증됐다고 해석하지 않는다.

실제 새 Sunny15의 20~30m native component와 최종 GT는 다음과 같다.

| 최종 split | native Obstacle component | 센서 지원 없어 제외 | 최종 infos GT |
|---|---:|---:|---:|
| train | 716 | 39 | 677 |
| val | 122 | 6 | 116 |
| 합계 | 838 | 45, 5.37% | 793 |

제외 45개는 모두 LiDAR 0점·현재 512 입력의 확인된 카메라 지원 없음이며 ROI 제외가 아니다.
카메라 상태는 `below_gate` 12 / `incomplete` 29 / `no_observed_instance_pixels` 4개다.
`incomplete`는 대응 annotation 일부 없음이라는 관측 분류이며 수집 손상이나 실제 비가시를
확정한 뜻이 아니다. 지원을 새로 지어내지 않는 기존 양성 GT 규칙으로 제외했고 raw는 보존했다.
반면 LiDAR 0점이어도 카메라 지원이 있는 53개는 train 45 / val 8개 모두 유지했다.
확인된 지원 GT의 누락·추가 component는 전수 검증에서 0개다. 대량 사후 삭제로 보강이
사라진 양상은 아니지만, 이 제외가 AP에 미친 영향 자체는 별도 평가가 필요하다.

보고서의 계획 수는 선택된 5K metadata 기준 물리 장애물 parent 20~30m
819개(train 701 / val 118)다. 이는 **Ego 기준 계획 중심 거리·19종 물리 parent**이며,
위 native 수는 **LiDAR 기준 실제 component 중심 거리·유모차 Obstacle 성분 포함**이다.
두 수를 직접 빼서 유실량으로 계산하지 않는다.

새 val은 Obstacle 20~30m 116개·82프레임이고 구 val은 143개·97프레임이다. 해당 구간의
LiDAR 점 수 중앙값은 모두 4점, 평균은 5.70→6.53점이며 0점 GT는 각각 8개다.
새 val의 센서 근거가 전반적으로 더 희소해졌다고 단정할 수 없다. 자산·전후방 구성도 달라졌다.
전체 Obstacle val 중 30m 이상 비중도 20.0→7.2%로 줄어 전체 AP 상승의 분포 효과를 분리해야 한다.
AP는 고정 운영 threshold 하나의 정확도가 아니므로 threshold만 낮춰 격차가 해결된다고 보지 않는다.
현재 restart를 마친 뒤 같은 val에서 E24·새 E8을 교차 평가하고, 20~30m의 자산별 누락·오검출을
나누는 것이 다음 판단 근거다. **그 교차 평가는 아직 수행하지 않았다.**

##### 근거 파일과 집계 범위

아래 `artifacts/` 경로는 로컬 저장소 `/home/luv/AjouNice2026` 기준이다.

- 구 E2~E24 및 LR: `artifacts/evaluation/bevfusion/base5_training_history_261018/base5_init_full_3class_e1_261011-ecddcbad9c2a/history.jsonl`.
  구 E1은 Vast `/workspace/base5_init_full_3class_e1_261011/20261011_123906/vis_data/scalars.json`과
  같은 디렉터리의 `20261011_123906.log` 최종 `Epoch(val) [1]`로 별도 확인했다.
- 새 cosine: Vast `/workspace/sunny15_ab_261018/cosine/20261018_114215/vis_data/scalars.json` 및
  `/workspace/sunny15_ab_261018/cosine/distance_diagnostics/epoch_1.json`~`epoch_8.json`.
- restart 조회: Vast `/workspace/sunny15_ab_261018/restarts/20261018_144405/vis_data/scalars.json`의
  E1~E4 기록. 완료 상태나 이후 epoch를 이 조회로 인증하지 않는다.
- 구 train/val: `artifacts/handoff/base5_sunny13_jpeg90_pedestrian_merged_v1_261011/infos/512x1056/morai_base5_infos_{train,val}.pkl`.
- 새 train/val: `artifacts/evaluation/base5_sunny15_training_export_261018_v2/sunny15/training3_jpeg_portable/infos/512x1056/morai_base5_infos_{train,val}.pkl`.
- 계획·native·제외 이유: `artifacts/evaluation/base5_sunny15_training_export_261018_v2/sunny15/report.json`과
  같은 디렉터리의 `objects.json`. `training_class`, `distance_lidar_xy_m`, `split`, `retained`,
  `exclusion_reason`, `lidar_points`, `camera512_supported`를 대조했다.
- 새 스케줄: `artifacts/evaluation/bevfusion/sunny15_cosine_e8_backup_261018/cosine/run_manifest.json`의
  `identity.param_schedulers`. 기존 실행·checkpoint·양성 GT 정책은 변경하지 않았다.

#### 다음 8 epoch 대조군 실제 시작

```bash
bash scripts/runtime/start_sunny15_vast_training.sh restarts --run
```

- 새 run: [restarts-7d345c6b3276](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/restarts-7d345c6b3276).
- 원격 work/log: `/workspace/sunny15_ab_261018/restarts/console.log`.
- 23:43:16 KST fresh 시작. `resumed=false`, 시작 epoch/iteration 0, optimizer 초기 상태다.
- 실제 두 `run_manifest.identity` 전 항목 대조에서 차이는 **param_schedulers 한 항목뿐**이다.
  데이터·초기화·source·모델·seed·batch·8 epoch·평가 계약은 같다.
- 스케줄: 4 epoch×2주기, 주기별 500 iteration warm-up. 시작/최대/최소 LR은
  1e-5 / 1e-4 / 1e-6이며 주기 길이 증가·optimizer reset은 없다.
- 23:45 KST epoch 1의 40/1,063 iteration·0.6650초/iteration·loss 11.7625·유한 gradient를
  확인했다. 첫 20 iteration에 baseline처럼 `grad_norm: inf` 1회가 있었지만 40에서 회복했고
  NaN loss·OOM·traceback은 관측하지 않았다. GPU 90%, 30,960/49,140MiB였다.
- W&B API도 `running`을 확인했다. 예상 약 1시간 45분~2시간은 완료 cosine 실측에 기반한
  예산이며 완료 보장이 아니다. 기존 cosine checkpoint·relay는 수정하지 않았다.
- 시작 증거는 로컬 v5 준비 폴더의 `restarts_start_observation_261018.json`이다.

#### 새 cosine checkpoint의 외부 백업 주의

아래는 10/18 후속 학습을 시작할 당시의 실패 이력이다. 사용자 추가 요청 이후의 실제 E8
복구 상태는 [로컬 경유 수동 백업](#sunny15-e8-manual-drive-backup-261019)을 따른다.

학습·짧은 Drive write/readback 성공과 실제 새 checkpoint 전체 백업은 다르다.
조회에서 검증 완료한 새 cosine Drive 백업은 **0개**였고 `backup_status.uploaded={}`다.
로그상 8회 시작·7회 실패, 그중 6회는 약 0.27~0.42MB/s의 느린 전송이 1,200초 상한을
넘어 SIGTERM으로 끝났다. E8 시도도 약 318.8MB/521.2MB에서 끝났으며 완료·전체 readback
성공 기록은 없다. `epoch_1 unclassified_copy_failure`는 오래된 상태 기록이지 현재 실패가
계속 E1이라는 뜻이 아니다. 정확한 HTTP/TLS 원인은 기존 오류 분류의 관측 부족으로 미확정이다.

Vast의 원본 E6/E7/E8와 best E8은 보존한다. E8 full은 521,239,045바이트, best E8은
202,071,167바이트다. 남은 `.backup_stage_*`의 E2/E3/E5/E7/E8 snapshot도 지우지 않는다.
E1/E4는 현재 work/남은 stage에서 찾지 못했고 검증된 Drive 백업 근거도 없다.
다음 실험은 실제 온라인 사전검사를 통과해 시작했지만, 이것을 bulk checkpoint 백업
문제 해결로 표현하지 않는다. 별도 권장 복구는 best E8·full E8을 로컬 회수한 뒤 로컬에서
Drive 업로드·전체 SHA readback하는 것이다. 이 읽기 전용 조사에서 업로드나 relay 변경을
수행하지 않았다. 백업 확인 전 인스턴스를 Destroy하지 않는다.

비교 근거는 새 원격 cosine의 `20261018_114215/vis_data/scalars.json`,
`distance_diagnostics/epoch_8.json`, `oneclick_status.json`과 구 로컬
`artifacts/evaluation/bevfusion/base5_training_history_261018/base5_init_full_3class_e1_261011-ecddcbad9c2a/history.jsonl`이다.
새 [cosine W&B](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/cosine-d68f0824772a)와
구 [Sunny13 W&B](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/base5_init_full_3class_e1_261011-ecddcbad9c2a)를
구분한다. 같은 공통 val/bag에서의 E8 대 E24 결과는 아직 없다.

<a id="sunny15-e8-manual-drive-backup-261019"></a>

### Sunny15 cosine E8의 로컬 경유 Drive 백업 — 2026-10-19

사용자가 기존 cosine E8의 실제 외부 백업을 요청했다. 느린 Vast→Drive 자동 전송과
진행 중인 restart 학습은 수정하지 않고, Vast→로컬 회수 후 로컬 rclone으로 업로드했다.
로컬→Drive best E8 업로드는 약 14MiB/s로 완료했고, 실제 Drive 파일 전체를 다시 읽어
원격 원본과 크기·SHA가 정확히 같은 것을 확인했다. 단순 폴더 생성이나 작은 probe의
성공을 checkpoint 백업 성공으로 사용하지 않는다. **10/19 00:10:37 KST에 두 checkpoint의
payload·metadata·COMPLETE와 full용 LATEST readback까지 완료**했으며 수동 실행은 exit 0이다.

| 파일 | 바이트 | SHA256 | 현재 검증 |
|---|---:|---|---|
| `best_val_center_mAP_epoch_8.pth` | 202,071,167 | `93882eafc93ecbadbee546ae31ce1dfef0794efb7a0aa03a08ee9468ca5c63ef` | 로컬·Drive 전체 readback·metadata·COMPLETE 확인 완료. weights-only이며 optimizer resume용이 아님 |
| `epoch_8.pth` | 521,239,045 | `42e18d77c2eb598f48b178a4bb4fe6c2384a70e49d10d2e6d306be4d68be6973` | full 상태·로컬·Drive 전체 readback·metadata·COMPLETE·LATEST 확인 완료. optimizer·scheduler·AMP·RNG를 포함함 |

두 파일의 계약은 `d68f0824772a2f6da30520cc4c91b28906ecd629a7a2e721da7a8ddaa156975e`,
epoch 8·iteration 8,504다. 기존 검증 helper로 full의 optimizer·scheduler·AMP scaler·RNG·
message hub를 확인했고, 같은 SHA의 로컬 파일에 그 증거를 연결했다. 원격 rsync는 실행
권한이 없어 scp를 사용했다. full 단일 전송은 느려서 내가 시작한 scp만 종료한 뒤,
이미 받은 137,265,152바이트를 보존하고 나머지를 읽기 전용 SSH 6개 범위로 회수했다.
잔여 수신·합치기·전체 SHA 확인은 약 411.1초였으며 정식 로컬 파일로 확정했다.
full E8의 로컬→Drive 업로드는 약 53.8초, 실제 전체 Drive readback·SHA 확인은 약 28.1초였다.

로컬 파일·증거는 `artifacts/evaluation/bevfusion/sunny15_cosine_e8_backup_261018/`에 둔다.
각 파일은 해당 폴더의 `cosine/`, 수동 실행 receipt는
`manual_drive_recovery_20261018T150121_193959Z/receipt.json`과 `progress.jsonl`이다.
원격 CPU 검증 증거는 `source_verification.json`으로 보존한다.

기존 자동 watcher와 충돌하지 않는 별도 Drive 경로를 사용한다.

```text
AjouNice2026/model/bevfusion/base5/sunny15_ab_261018/checkpoint_backups/
  manual_e8_recovery_261018/cosine/<checkpoint SHA256>/
```

[실제 Drive 복구 폴더](https://drive.google.com/drive/folders/1oGQQbVCDYpQSzyll4TqFrcNfIgqktToS)에서
각 SHA 폴더를 확인한다. 각 파일의 `run_manifest.json`·`resolved_train.py`·`fresh_start.json`·
`oneclick_status.json`·원격 검증 증거도 업로드 후 전체 readback했다. payload와 metadata가
모두 일치한 뒤에만 `COMPLETE.json`을 썼고, full E8 완료 후에만 복구 폴더의 `LATEST.json`을
썼다. best는 weights-only, full은 resume-capable로 명시한다. 실제 로그에서 이 순서를 확인했다.

이 수동 백업은 기존 `backup_status.json`을 성공으로 덮어쓰지 않는다. 자동 전송 경로의
속도·1,200초 상한 문제를 해결했다고 표현하지 않으며, 후속 restart의 백업 완료도 아니다.
원격 원본 가중치·PNG·native GT·학습 source·relay는 수정하거나 삭제하지 않는다.
백업 중 10/19 00:07:44 KST에도 restart의 epoch 2·900/1,063 iteration과 실제 train 세션을
확인했다. 이후 남은 학습·후속 백업 완료 여부는 별도 관측이 필요하다.

설치·전송·OAuth·백업·학습 준비·재개 실험의 명령은 아래 해당 anchor의 당시 기록으로 연결한다.
과거 문서 개편 자체는 원격 접속·이미지 게시·학습·과금 상태를 변경하지 않았다.
위에 명시한 10/18 데이터 전달·새 설정 준비 실행과 구분한다.

<a id="sunny15-restarts-e8-result-backup-261019"></a>

### Sunny15 restart E8 완료·수동 Drive 백업 — 2026-10-19

일반 코사인과 재시작형 모두 8 epoch·exit 0으로 완료했고 각 자체 val best는 E8이다.
원격 `run_manifest.json`의 identity를 재귀 비교해 차이가 `param_schedulers`뿐임을 확인했다.
재시작형 실제 로그는 10/18 23:44:06~10/19 01:30:48 KST, 1시간 46분 42초다.
일반 코사인은 1시간 46분 57초다. 15초 차이를 유의미한 학습 처리속도 개선으로 해석하지 않는다.

| 같은 Sunny15 val, E8 | 일반 코사인 | Warm-up restart |
|---|---:|---:|
| 전체 center mAP | 88.49% | 88.05% |
| Vehicle | 94.60% | 95.12% |
| Pedestrian | 91.17% | 90.09% |
| Obstacle | 79.69% | 78.93% |
| Obstacle 0~10m | 86.74% | 87.23% |
| Obstacle 20~30m | 71.06% | 69.99% |
| Vehicle 80~100m | 73.09% | 74.43% |

재시작형은 E4 86.49% → E5 84.78%로 일시 하락한 뒤 E8 88.05%까지 회복했다.
이번 짧은 예산에서는 전체 성능·20~30m 장애물의 개선을 확인하지 못했다.
단일 seed 대조이므로 일반적인 scheduler 우열이나 30K 최종 성능을 인증하지 않는다.
현재 일반 코사인을 주 후보로 유지하며 production E24·학습 설정은 변경하지 않았다.
원본 scalar 로그는 `artifacts/evaluation/bevfusion/sunny15_ab_result_261019/`에 회수했다.
W&B 전체 동기화 완료 여부와 원본 로그 정상 완료는 별도다.

사용자 승인으로 restart best E8과 full E8을 로컬 경유 수동 백업한다.
원격 CPU 검증으로 두 파일의 계약·epoch 8·iteration 8,504와 전체 SHA를 확인했다.
full은 optimizer·scheduler·AMP scaler·RNG·message hub를 포함하며 best는 weights-only다.

| 파일 | 바이트 | SHA256 |
|---|---:|---|
| `best_val_center_mAP_epoch_8.pth` | 202,071,999 | `c231f3abf3136179146b6dbeabf66c9d33e4ec5254936cf7f543d0fbf8022114` |
| `epoch_8.pth` | 521,239,685 | `1bbb0a56305442259e0ac0d2fb9b6496eed3f21290d90fe3be8162ded3ecd8c8` |

계약은 `7d345c6b32761b3fbd2ba5180fa1a2181325ce35003ffe11c0031111839af8f8`이다.
자동 백업과 충돌하지 않는 별도 prefix를 사용한다.

```text
AjouNice2026/model/bevfusion/base5/sunny15_ab_261018/checkpoint_backups/
  manual_e8_recovery_261019/restarts/<checkpoint SHA256>/
```

로컬 실행 근거는 위 결과 폴더의 `restart_e8_backup/progress.jsonl`·
`source_verification.json`·`receipt.json`이다. 2026-10-19 03:31 KST 두 파일의
payload·설정·계약·검증 증거·COMPLETE 전체 readback을 완료했다. full용 `LATEST.json`도
재읽기 검증을 통과했다. 원격→로컬 병렬 SSH 수신 뒤 전체 SHA를 대조하고 Drive로 올렸다.
full 수신은 약 14분 15초가 걸렸으며, Drive payload 업로드·전체 재읽기는 약 45초였다.
기존 자동 백업 상태를 성공으로 덮어쓰거나 원격 가중치·Sunny11 수집을 수정하지 않는다.

<a id="sunny13-matched-e24-261019"></a>

### 새 Sunny13 데이터만 교체한 E24 조건 대조 학습 — 2026-10-19

아래 시작·E1/E6 관측·후행 평가 대기는 당시 이력이다. 실제 24 epoch 정상 종료·백업과
동일 val 교차 결과는 [완료 결과](#sunny13-matched-e24-result-261019)를 따른다.

사용자 요청으로 기존 E24의 실제 동결 소스와 실행 manifest를 대조했다. 당시 예산은
25가 아니라 **24 epoch**다. 기존 source·초기화·seed 261010·증강·batch 4·workers 4·
AdamW(lr 1e-4, weight decay 0.01)·500 iteration warm-up 이후 cosine·compile ON을 보존했다.
이 compile ON은 재현 대조군이며 다음 일반 학습의 OFF 기본값이나 production을 바꾼 것이 아니다.
E24에서 fine-tune/resume한 것이 아니라 같은 변환 nuScenes initializer로 새로 시작했다.

새 Sunny13 전달본 3,382,560,124바이트를 Drive 경유로 약 291.9초에 수신했다.
전체 SHA와 184,055개 멤버, 5K 장면·20K JPEG, portable payload·group split 검증을 통과했다.
실제 identity 차이는 dataset manifest SHA·train 4,255→4,250·epoch당 iteration 1,064→1,063의
세 항목뿐이다. 이에 따라 총 update는 25,536→25,512로 달라진다. 같은 seed가 역사적
worker/prefetch·중간 resume까지 비트 단위로 재현한다는 뜻은 아니다.

- 시작: **2026-10-19 04:11:54 KST**. 실제 E1 120 iteration까지 오류 없이 진행했고,
  W&B API에서 epoch·train loss·유한 gradient 수신을 확인했다.
- 원격 work: `/workspace/sunny13_matched_e24_261019`.
- 계약: `4b350d8b8d2a52a75322c2dc590f289b59c525028044d8aada4b09f1082c9321`.
- [W&B 실험](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/sunny13_matched_e24_261019-4b350d8b8d2a).
- 초기 compile 뒤 약 0.56초/iteration, 학습 로그 memory 약 27.6GiB다. 전체 4.5~5시간은
  초기 실측 기반 예상이며 완료 시간 인증이 아니다. 냉간 compile을 포함한 최초 ETA와 구분한다.
- Drive 백업 감시기와 온라인 W&B 중계는 독립 tmux `sunny13-matched-e24-supervisor`에서
  실행한다. **04:34:24 KST에 실제 E1 full checkpoint 519,738,181바이트의 Drive 백업을 확인했다.**
  전체 SHA·metadata 재읽기와 COMPLETE/LATEST 검사가 통과했다. SHA는
  `6696333a558db54338489753d005c2dc30e357ad4480753693af954119344f5a`다.
  초기 SSH 일부 범위의 연결 종료는 이미 받은 범위를 재사용해 복구했다. 후속 epoch도
  validation 완료·파일 안정화 뒤 개별 receipt로 확인하며, 폴더 존재만으로 완료를 주장하지 않는다.

05:25 KST 실제 원격 확인에서는 E7 학습 약 0.56초/iteration과 온라인 W&B의 E7 loss 수신을
확인했다. E6 자체 val은 mAP 88.52%, Vehicle 95.15 / Pedestrian 92.20 / Obstacle 78.22%다.
05:27:49 KST E4 full도 전체 SHA·metadata·COMPLETE·LATEST의 Drive 재읽기를 통과했다.
동기 백업 중에는 마지막 inventory 시간이 오래돼 보일 수 있어, 상태 명령에서 수신 중 바이트·
마지막 쓰기 시각·로컬 SHA·Drive 완료를 구분한다. 중계의 `relay_ready` 파일은 시작 표식이며
heartbeat가 아니다. 이 점검 때문에 학습을 재시작하거나 하이퍼파라미터를 바꾸지 않았다.
실측 근거는 같은 artifact 폴더의 `health_0528_receipt.json`이다.

첫 E1 자체 val mAP는 구/신규 70.19→78.50%, obstacle은 43.78→55.99%다.
하지만 val GT 구성이 달라 순수 개선량이 아니다. 장애물 20~30m는 GT 143개로 같고
AP 36.23→47.54%지만, 30~50m는 GT 91→29개·AP 12.27→0.12%다.
초기 한 epoch·작은 표본만으로 최종 퇴보를 확정하거나 중거리 문제를 해결했다고 주장하지 않는다.

근거는 `artifacts/evaluation/base5_sunny13_matched_e24_261019/`의
`matched_condition_comparison.json`·`first_training_observation.json`·전송/복원 receipt다.
서로 다른 자체 val AP만으로 데이터 개선량을 주장하지 않는다. 원본 val 745장과 새 val 750장
각각에서 구 E24와 새 E24를 평가하는 **2×2 공통 평가를 후행 큐에 준비했다.**
당시에는 `sunny13-matched-posteval` tmux에서 학습 24 epoch의 정상 종료와 실제 full E24
Drive 백업을 기다렸다. 네 칸 모두 같은 current-nearest/sorted·compile OFF·AMP FP16·
batch 1·증강 OFF·score 0의 평가 전용 snapshot을 사용한다. 학습의 legacy depth raster나
production 설정을 바꾸지 않는다. 기존 자체 val 수치를 이 네 칸의 결과로 재사용하지 않는다.
CPU strict 검사에서 구 E24·새 E2의 state key/shape 574개와 입력/지표 계약을 확인했다.
최종 E24의 SHA·source·데이터는 평가 시작 직전에 다시 검사하도록 했다. 당시 실제 네 칸 결과는 없었다.
평가별 600초·전체 2,500초 상한과 별도 Drive/W&B 결과를 사용하며, 이후 독립 bag 비교는 별도다.

읽기 전용 상태 명령은
`python3 -B artifacts/evaluation/base5_sunny13_matched_e24_261019/status.py`다.
`--local-only`는 원격 SSH 없이 마지막 관측만 읽는다. 로컬 PC의 supervisor를 끄면
GPU 학습이 계속되더라도 W&B 중계·Drive 자동 백업·후행 평가가 중단될 수 있으므로
학습 종료/백업 확인 전에 로컬 PC를 종료하지 않는다.

<a id="sunny13-matched-e24-result-261019"></a>

#### 24 epoch 완료·Drive 백업·동일 val 2×2 결과

새 Sunny13 **v158a 원형 5K** 학습은 10/19 04:11:54~08:48:18 KST,
약 **4시간 36분 24초·24 epoch·exit 0**으로 정상 종료했다. 이 결과는 Foggy13의 장애물
149장 거리 재분포나 Sunny 추가 보강 초안으로 학습한 결과가 아니다.
같은 초기화에서 시작한 위 대조 계약을 유지했으며 학습 도중 하이퍼파라미터를 바꾸지 않았다.

full E24와 자체 val best E20의 payload·metadata·COMPLETE에 대한 Drive 전체 readback을
완료했고 full용 LATEST도 검증했다. 10/19 후속 읽기 전용 확인에서 Vast 원본 두 파일의
전체 SHA를 다시 계산해 아래 값과 대조했다. Drive의 두 COMPLETE·LATEST와 공통 평가
SUMMARY도 다시 읽어 일치를 확인했다. 이번 확인에서 대용량 가중치를 로컬로 다시 받지는 않았다.

| 파일 | 역할 | 바이트 | SHA256 |
|---|---|---:|---|
| `epoch_24.pth` | optimizer·scheduler·AMP·RNG를 포함한 full | 524,630,021 | `6d86c4726c0f7d3abc4870999767a594d623f97b9bb85b8e48a42429bcbd6f32` |
| `best_val_center_mAP_epoch_20.pth` | 자체 val best, weights-only | 204,640,447 | `33b262bcffd2d09efadf9d6eff1f81c86b2b7c53e088588d0126bf5141080585` |

Drive 위치는 `ajou_drive_private:model/bevfusion/base5/sunny13_matched_e24_261019/` 아래
`checkpoint_backups/<checkpoint SHA256>/`이며 full용 LATEST는 `checkpoint_backups/LATEST.json`이다.

예약한 **구/신규 E24 × 기존 745장/신규 750장 val의 네 평가가 모두 완료**됐으며,
실제 평가 시간 합계는 약 6분 28초다. 네 칸 모두 current-nearest/sorted·compile OFF·
AMP FP16·batch 1·이미지/BEV 증강 OFF·score threshold 0을 사용했다.
각 val에서 두 모델의 GT 수는 같고, 개별 COMPLETE와 SUMMARY의 수치·checkpoint SHA·
평가 계약을 대조했다. 아래 값은 0.5/1/2/4m 중심 거리 AP의 평균이며 단위는 %다.

| 같은 평가 자료 | 모델 | 전체 mAP | Vehicle | Pedestrian | Obstacle |
|---|---|---:|---:|---:|---:|
| 기존 val 745장 | 구 E24 | 83.47 | 89.91 | 87.73 | 72.79 |
| 기존 val 745장 | 새 E24 | 87.63 | 93.04 | 92.05 | 77.79 |
| 신규 val 750장 | 구 E24 | 89.93 | 96.05 | 92.31 | 81.42 |
| 신규 val 750장 | 새 E24 | 91.00 | 95.73 | 93.42 | 83.84 |

같은 val에서 전체 mAP 차이는 기존 val **+4.15%p**, 신규 val **+1.07%p**다.
서로 다른 자체 val의 83.47→91.00%를 그대로 데이터 개선량으로 해석하지 않는다.
새 E24의 근거리 장애물 AP는 두 val에서 모두 높아졌지만 중거리 장애물과 원거리 차량의
개선은 일관되지 않았다. 아래 각 행의 GT는 두 모델이 공유하는 해당 거리·방향의 개수다.

| 거리·방향별 AP | 기존 val GT | 구→새 E24 | 신규 val GT | 구→새 E24 |
|---|---:|---:|---:|---:|
| Obstacle 0~10m | 206 | 85.34→92.63 | 199 | 89.78→92.14 |
| Obstacle 10~20m | 190 | 86.14→88.78 | 206 | 88.01→91.19 |
| Obstacle 20~30m | 143 | 76.73→81.23 | 143 | 78.20→79.74 |
| Obstacle 30~50m | 91 | 27.64→23.95 | 29 | 3.52→2.13 |
| Vehicle 50~80m | 51 | 58.83→74.63 | 67 | 84.43→83.47 |
| Vehicle 80~100m | 6 | 29.40→29.70 | 46 | 77.67→73.94 |
| Vehicle 전방 50~80m | 47 | 55.66→72.39 | 43 | 82.43→86.48 |
| Vehicle 후방 50~80m | 4 | 98.89→98.89 | 24 | 85.85→77.76 |
| Vehicle 전방 80~100m | 4 | 0.65→0.53 | 24 | 53.27→49.31 |
| Vehicle 후방 80~100m | 2 | 98.89→98.89 | 22 | 95.36→93.20 |

특히 30~50m 장애물은 두 val 모두에서 낮아져 해결되지 않았다. 신규 val의 80~100m 차량도
전·후방 모두 낮아졌다. 기존 val의 원거리 차량 GT 6개와 후방 50~80m GT 4개는 매우 작아
그 AP만으로 일반적인 방향별 성능을 인증하지 않는다. 점수·거리별 상세 값은 CSV를 따른다.

자체 val best는 구 모델 **E9 83.97397%**, 새 모델 **E20 91.01361%**다. 새 E24의 학습 중
자체 val은 90.99242%다. **이번 네 칸은 E24끼리 평가했으며 best E20의 교차 평가는 하지 않았다.**
학습 당시 자체 val과 현재 nearest 평가의 수치는 미세하게 달라질 수 있으므로 두 계열을
섞지 않는다. 단일 seed·같은 지도 합성 분포 대조이며 개별 생성기 변경의 인과, 두 데이터셋
간 train/val 기하 중복, rosbag·threshold·tracker·실제 24GB ROS 풀스택 성능을 새로 인증한 것은 아니다.
**Production은 기존 Sunny13 E24를 유지**하고 새 가중치를 자동 승격하지 않았다.

[학습 W&B](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/sunny13_matched_e24_261019-4b350d8b8d2a)와
[별도 공통 평가 W&B](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/sunny13-commonval-2a176b2069ac)는
API에서 모두 `finished`를 확인했다. 학습 epoch 24와 평가 네 칸의 mAP도 실제 수신값과 일치했다.
공통 평가 로그·설정·결과의 Drive 위치는 위 prefix의 `commonval_2x2/`다.

근거는 `artifacts/evaluation/base5_sunny13_matched_e24_261019/` 아래
`commonval_results/{SUMMARY,EXECUTION_CONTRACT,WANDB}.json`·개별 COMPLETE,
`commonval_drive_COMPLETE.json`·checkpoint별 backup receipt와 `final_review_v1/`이다.
`final_review_v1/comparison.json`·`commonval_comparison.csv`·`ownval_epoch_history.csv`에 전체 비교를,
`remote_observation.json`·`wandb_observation.json`·`drive_observation.json`에 읽기 전용 재확인을 보존했다.
최종 비교 JSON SHA256은 `8565202a15a2c2a2797eab3aa116178e3be7f05c4948f75957711bba4d09ed25`,
공통 평가 원본 SUMMARY SHA256은 `2f3c84141522bdce47da6f2c0e1da9318c5633dfdf8da43ced5e65466b4bfc2a`다.
평가 계약은 `2a176b2069acacfab42c1f7d900212521cd8b25b2a8119ccc08267df14e0489c`이며,
Vast 원본·학습 설정·production·다른 PC의 수집 프로세스는 변경하지 않았다.

## 기존 링크 호환

<a id="base5-user-oneclick-training-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-user-oneclick-training-261011).

<a id="base5-fine-compiled-prepared-vast-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-fine-compiled-prepared-vast-261011).

<a id="base5-fixed-image-publication-plan-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-fixed-image-publication-plan-261011).

<a id="base5-vastai-4090-학습과-추론-시험-계획"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-vastai-4090-학습과-추론-시험-계획).

<a id="사용자-실행용-딸깍-학습"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#사용자-실행용-딸깍-학습).

<a id="최신-운영--2026-10-12-원거리-평가-확장"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-운영--2026-10-12-원거리-평가-확장).

<a id="최신-실행-대상--2026-10-11-호환-nuscenes-초기화-e1-채택"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-실행-대상--2026-10-11-호환-nuscenes-초기화-e1-채택).

<a id="이전-3클래스-준비-이력--2026-10-11-animal--pedestrian-통합-시점"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이전-3클래스-준비-이력--2026-10-11-animal--pedestrian-통합-시점).

<a id="이전-4클래스-준비-이력--2026-10-11-2020-kst-e2-제한-검증-완료"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이전-4클래스-준비-이력--2026-10-11-2020-kst-e2-제한-검증-완료).

<a id="최초-준비-이력--2026-10-11-1746-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최초-준비-이력--2026-10-11-1746-kst).

<a id="시작과-관측"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#시작과-관측).

<a id="설정과-검증-범위"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#설정과-검증-범위).

<a id="최신-fine-학습-준비--2026-10-11-1549-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-fine-학습-준비--2026-10-11-1549-kst).

<a id="다음-임대용-고정-이미지-게시-계획--2026-10-11-0646-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#다음-임대용-고정-이미지-게시-계획--2026-10-11-0646-kst).

<a id="현재-방식과-변경할-방식"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-방식과-변경할-방식).

<a id="준비검증-순서"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#준비검증-순서).

<a id="2026-10-10-원격-검증-진행--본학습-시작-전"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-원격-검증-진행--본학습-시작-전).

<a id="재개-후-원격-jpeg-전환-완료--2026-10-11-0108-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#재개-후-원격-jpeg-전환-완료--2026-10-11-0108-kst).

<a id="현재-카메라-rgb-계약과-vast-전환--2026-10-11"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-카메라-rgb-계약과-vast-전환--2026-10-11).

<a id="q90-100와-사전-생성-캐시--2026-10-10-2355-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#q90-100와-사전-생성-캐시--2026-10-10-2355-kst).

<a id="이미지-증강-선택--2026-10-10-2305-kst-로컬-검토"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이미지-증강-선택--2026-10-10-2305-kst-로컬-검토).

<a id="좌우-범위-선택-확정--2026-10-10-2243-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#좌우-범위-선택-확정--2026-10-10-2243-kst).

<a id="확대-검증-완료와-네트워크-gate--2026-10-10-21시"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#확대-검증-완료와-네트워크-gate--2026-10-10-21시).

<a id="drive-직접-다운로드-검사--2026-10-10-1718-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#drive-직접-다운로드-검사--2026-10-10-1718-kst).

<a id="전송-방법-추가-비교--2026-10-10-1724-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전송-방법-추가-비교--2026-10-10-1724-kst).

<a id="호출-제한-완화-재검사--2026-10-10-17301732-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#호출-제한-완화-재검사--2026-10-10-17301732-kst).

<a id="전용-drive-앱-인증과-속도-재검사--2026-10-10-1757-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전용-drive-앱-인증과-속도-재검사--2026-10-10-1757-kst).

<a id="전용-연결-8병렬-검사--2026-10-10-1804-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전용-연결-8병렬-검사--2026-10-10-1804-kst).

<a id="두-번째-호스트의-제한-검사--2026-10-10-1817-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#두-번째-호스트의-제한-검사--2026-10-10-1817-kst).

<a id="수동-전송-중단-원인과-부족-조각-복구--2026-10-10-19131932-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#수동-전송-중단-원인과-부족-조각-복구--2026-10-10-19131932-kst).

<a id="부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst).

<a id="현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의).

<a id="초기-준비의-역할-분담--아직-수행하지-않았다"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#초기-준비의-역할-분담--아직-수행하지-않았다).

<a id="2026-10-10-vast-템플릿-안내와-블로그용-증거-기록"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-vast-템플릿-안내와-블로그용-증거-기록).

<a id="화면에서-관측한-내용과-선택-이유"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#화면에서-관측한-내용과-선택-이유).

<a id="안내한-입력값--적용-확인-전"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#안내한-입력값--적용-확인-전).

<a id="학습-gpu와-배치-선택-논의--실측-전"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#학습-gpu와-배치-선택-논의--실측-전).

<a id="캡처-원본-보관"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#캡처-원본-보관).

<a id="gpu-후보-추천--2026-10-10-161856-kst-캡처-기준"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#gpu-후보-추천--2026-10-10-161856-kst-캡처-기준).

<a id="2026-10-10-sunny13-5k-drive-원본-백업"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-sunny13-5k-drive-원본-백업).

<a id="vast-전송-방식-선택"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#vast-전송-방식-선택).

<a id="2026-10-10-로컬-준비-결과"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-로컬-준비-결과).

<a id="임대한-환경에서의-순서"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#임대한-환경에서의-순서).

<a id="로컬-정리-결과와-보존-원칙"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#로컬-정리-결과와-보존-원칙).

<a id="임대-사양-추천"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#임대-사양-추천).

<a id="2026년10월9일-가격-관측"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026년10월9일-가격-관측).

<a id="저장공간과-업로드-범위"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#저장공간과-업로드-범위).

<a id="ssh와-실행-환경"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#ssh와-실행-환경).

<a id="로컬에서-원격-학습까지"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#로컬에서-원격-학습까지).

<a id="rosbag-dev-val과-최종-평가"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#rosbag-dev-val과-최종-평가).

<a id="중단-복구와-종료-기준"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#중단-복구와-종료-기준).

<a id="원격-morai는-학습-환경과-별도-검토"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#원격-morai는-학습-환경과-별도-검토).
