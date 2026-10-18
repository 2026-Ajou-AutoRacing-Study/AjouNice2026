# Base5 원격 실행과 보존 안내

문서 상태: RUNBOOK + COMPATIBILITY ROUTER. 확인 기준: 2026-10-18.
Sunny13 24 epoch 학습·E9/E24 외부 백업은 완료했다. 현재 모델·다음 작업은 [CURRENT](CURRENT.md)를 따른다.
아래 기존 명령은 완료된 동결 run의 점검/복구용이며 새로운 학습 준비를 자동으로 의미하지 않는다.

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

이후 사용자가 **8MB/s 이상이면 진행하고, 그 아래로 지속되면 중단**하도록 기준을
명시적으로 변경했다. 다음 실제 전송은 이 기준을 적용하며, 이전 10MB/s 미달 판정과
측정 이력은 고쳐 쓰지 않는다. 근거는 v5 준비 폴더의 `transfer_probe_261018.json`과
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

설치·전송·OAuth·백업·학습 준비·재개 실험의 명령은 아래 해당 anchor의 당시 기록으로 연결한다.
과거 문서 개편 자체는 원격 접속·이미지 게시·학습·과금 상태를 변경하지 않았다.
위에 명시한 10/18 데이터 전달·새 설정 준비 실행과 구분한다.

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
