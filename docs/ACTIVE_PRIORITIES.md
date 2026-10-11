# AjouNice2026 활성 최우선 계획

마지막 갱신: 2026-10-11.

이 문서는 **아직 끝나지 않았고 다음 작업에서 반드시 다시 확인할 항목만** 둔다.
완료된 수집·학습·실험 수치는 각 CURRENT/EVIDENCE 문서에 보존하며 여기로 다시
복사하지 않는다.

## 현재 production 기준

- Detector: `B3-BF0-F40 E25`, threshold Vehicle/Pedestrian/Obstacle
  `0.40/0.15/0.10`.
- Tracker: CTRV, position measurement noise `0.40 m`.
- E25는 7D detector이므로 detector velocity fusion은 기본 `off`.
- Prediction: Lanelet Map-CV, 현재 3초 horizon.
- Localization production: `/Competition_topic`, heading correction `-1.347 deg`,
  sensor TF 중복 발행 금지. Planning/TF는 Global state, controller는 Smooth
  `/localization/control_kinematic_state`를 사용한다. MRP는 `localization_mrp`.

실행 계약은
[`bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`](bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md)와
[`localization/README.md`](localization/README.md)를 우선한다.

## P0. 대회 측위 종·횡 안정화와 제어 채터링 제거

- 현장점검에서 fusion localization의 steering-rate absolute p95가 약 `1.59 rad/s`,
  GT localization은 약 `0.34 rad/s`였다. 시각화 문제가 아니라 제어 안정성 문제다.
- 정상 구간 횡 채터링은 약 `14.8/18.5 cm`로 양자화된 GNSS update를 Global EKF가
  강하게 따라가고 controller가 Global state를 직접 소비하는 구조가 주원인이다.
- 로컬 한-PC full-stack의 velocity 과적분은 production 속도 bias로 취급하지 않는다.
  현장 bag의 이동거리/속도 적분 비율은 정상이며, 로컬 MORAI 실시간율과 receipt-time
  artifact를 분리해야 한다.
- 경량 GT-object 폐루프에서 production Q/R은 localization `50 Hz`, 횡 frame-jump
  p95 약 `1.31 cm`, steering-rate p95 약 `0.54 rad/s`를 유지했다. 이 test bed 확보는
  완료됐고 `sigma=0.05, Qxy=0.05` preset은 기각한다.
- Global/Local state 분리와 Smooth controller 입력은 경량 전체 경로 폐루프 A/B를
  통과해 production에 반영했다. Quantization-aware 후보는 동일 기록 controller
  A/B에서 Smooth보다 우위가 없었고, 폐루프 승격 전이므로 기본 OFF다.
- 공식 START 고정 위치 + 보정된 Competition heading 초기화는 MR 로컬 대조를
  통과했다. 대회 LAN 출발·GPS 음영 복귀·checkpoint 순간이동 검증은 남아 있다.
  공식 START는 그때까지 opt-in이며 개발 기본 초기 위치는 `/Ego_topic`이다.

현재 production 계약과 개선 순서:
[`localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md`](localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md).
누적 문제 근거:
[`localization/260918_competition_localization_critical_issue_status.md`](localization/260918_competition_localization_critical_issue_status.md).
최신 폐루프 근거:
[`localization/260920_lightweight_closed_loop_evidence.md`](localization/260920_lightweight_closed_loop_evidence.md).

## P1. 근접 장애물·원거리 차량의 빠른 인지 전달

- **2026-10-11 14:32 KST 원격 학습 제외 검증:** [EVIDENCE 23절](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-inference-runtime-vast-261011)의
  4090 300장 추론·수치 통제·CPU 스레드·CUDA pooling·증강 geometry 검사를 완료했다.
  compiler는 출력 gate 실패로 OFF이며 정밀 격자의 빈 출력은 정합 통과가 아니다.
  depth 25cm 새 prepare-only는 만들었지만 학습은 실행하지 않았다. 최신 batch 4 학습 비용·
  저장/재개는 학습 허용 후 진행한다. 14:45 합의로 학습 compile OFF/ON의 짧은 gradient·
  비용·재개 비교 후 선택하기로 했다. [검사 계획](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-training-compile-short-gate-261011)과
  [현재 허브](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub)를 따른다.

- **2026-10-11 로컬 추론 비교:** [300장 전체 서비스·수치 통제 검사](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-inference-runtime-local-261011)를
  완료했다. compile 비용 후보와 아직 실패하는 출력 gate를 구분하고 기본값은 유지한다.
  Base5 4-class 시험 경로를 추가했지만 production E25·ROS 배포는 바꾸지 않았다.
  현재 선택·4090 재확인 순서는 [계획 허브](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub)를 따른다.

- **2026-10-11 로컬 학습 가속:** [실제 로더 A/B·기하 정합·runtime 검증](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-training-runtime-local-261011)을
  완료하고 별도 opt-in 후보를 제공한다. 현재 선택·남은 gate는 [계획 허브](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub)를
  따른다. Vast 접속·본학습·기본 config 변경은 하지 않았다. 다음은 사용자 재개 후 4090 재확인이다.

- **2026-10-11 06:46 KST 계획 통합:** [Base5 현재 계획 허브](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub)를
  현재 결정·남은 순서의 단일 기준으로 사용한다. 다음 임대는 고정 팀 이미지를 게시해 재사용한다.
  이미지 빌드·게시·템플릿 변경은 아직 계획이며 현재 설치된 인스턴스는 유지한다. 이번 학습은
  기존 train/val로 진행하고 rosbag-to-val 구현·평가는 후속이다. 다음 작업은 로컬 추가 가속 검토다.
  아래 날짜별 상세는 과거 결정의 근거이며 현재 실행 지시로 재사용하지 않는다.

- **2026-10-11 06:43 KST 최신 합의:** [depth 25cm 후보와 로컬 추가 최적화 계획](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-depth25-local-optimization-plan-261011)을
  우선한다. 512×1056·공간 7.5/30cm를 유지하고 depth 25cm를 우선 학습 후보로 둔다.
  학습 비용 약 +20%를 감수하는 선택이며 성능 개선은 가설이다. 다음은 로컬 병목 분석·
  정합을 유지하는 학습/추론 가속 검토 → 채택 변경의 4090 확인·새 준비 계약 → 승인 후 본학습이다.
  이번에는 문서만 갱신했고 코드·원격 계약·실험은 변경하지 않았다. 기존 50cm 준비본은
  비교 이력이며 25cm 실행에 그대로 사용하지 않는다. 외부 백업 완료 상태는 유지한다.
  아래의 ‘depth 50cm 유지/다음 바로 본학습’은 이 합의 이전의 기록이다.

- **2026-10-11 외부 checkpoint 백업 완료:** [20절의 실제 병렬 업로드·재다운로드·복원 결과](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-checkpoint-parallel-backup-261011)가
  아래의 180초 미완료 이력보다 우선한다. 499MB full checkpoint를 무손실 조각으로
  저장하고 전체 SHA256·실제 복원·metadata·COMPLETE/LATEST·중복 재검사를 통과했다.
  최신 prepare-only는 `/workspace/base5_training_sunny13_512_b4_e24_55272391_backup_v3_261011`이다.
  데이터·모델·학습 파라미터는 유지하며 본학습은 미시작이다. 다음은 사용자 승인 후
  병렬 백업 watcher와 본학습 시작이다. 인스턴스를 자동 중단하지 않는다.

- **2026-10-11 새 4090 이전:** 기존 중단 인스턴스의 GPU가 재할당되어 사용자가
  55272391을 임대했다. Copy 입력의 읽기 오류는 원본을 보존한 별도 검증본으로 복구했고
  173,294개 파일의 SHA256을 확인했다. [최신 이전·depth·Z 상태](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-vast-depth-z-qualification-261011)가
  아래 기존 인스턴스의 `/root` 경로보다 우선한다. depth 25/50/100cm의 추론·batch 4
  비용, 높이 범위, 저장/재개·전체 val·새 준비 계약을 검증했다. 기본 depth 50cm와
  Z 범위는 유지한다. 본학습은 미시작이며 외부 full checkpoint 백업은 새 호스트에서도
  180초로 제한해 미완료다. 본학습 전 보존 조건을 확정하며 인스턴스를 자동 중단하지 않는다.

- **2026-10-11 최신 실제 4090:** 사용자 재개 후 최신 소스·upright 라벨을 반영했다.
  센서 재전송 없이 로컬과 같은 infos를 확인하고 300장 추론 반복·실제 5K 로더의
  batch 4 비용·저장/재개·전체 val 로더를 검증했다.
  [원격 결과·새 준비 계약·남은 gate](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-vast-grid-qualification-261011)가
  아래 ‘Vast 미변경/반영 전’ 이력보다 우선한다. 기본 512×1056·7.5/30cm·depth 50cm를
  유지한다. 24 epoch 운영 예산은 약 4.7~5.3시간이다. 외부 백업 검사는 180초로 제한해
  중단했으며 아직 미완료다. 본학습 미시작, AP 우위·실제 ROS 전달 비용도 미검증이다.

- **2026-10-11 로컬 공간/depth 비교:** 실제 5K의 근거리 voxel 정보·밀집 GT와
  300장 추론 비용, 짧은 실제 학습 비용을 비교·검증했다.
  [최신 결과와 후속 선택](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-grid-depth-local-261011)이
  아래 ‘다음은 격자 비교’ 기록보다 우선한다. 현재 7.5/30cm·depth 50cm를 기본으로
  유지하며 정밀 5/20cm는 4090 비용 확인 후 선택한다. 비교용 공통 padding을 기본
  ROI로 승격하지 않고, 수집 원본·E25·Vast는 변경하지 않는다. 본학습 AP와 실제
  ROS 전달 비용, 최신 소스의 원격 비용·외부 백업 완료는 아직이다.

- **2026-10-11 최신 파생 GT:** native roll/pitch를 버리던 변환을 고정 yaw의 upright
  enclosure로 보완했다. 기존 센서·native GT·클래스·개수·분할은 보존한다. 현재 학습 입력은
  `artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release`이며 아래 heatmap 수정도 유지한다.
  [라벨 정책·전수·실제 포인트·GPU 재개 검증](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-upright-native-rpy-261011)을
  따른다. 다음 격자/depth 비교는 새 라벨로 진행한다. Vast와 Drive는 이번 작업에서 변경하지 않았다.
  재개 승인 후 원격의 기존 native RPY로 라벨만 재변환하고 새 학습 계약을 만든다.

- **2026-10-11 최신 학습 정합:** Base5 전용 heatmap의 X/Y 교환을 수정했다. 고유 5K의
  11,052 GT와 증강을 포함한 29,714회 정답 peak, 실제 전체 모델 4-iteration 역전파·저장·재개를
  로컬에서 검증했다. 이전 feature 축 검사는 이 target 경로까지 보장하지 못했다.
  raw·GT·split·E25는 유지하며 재수집하지 않는다.
  [최신 근거와 실험 순서](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-heatmap-target-alignment-261011)를
  따른다. 다음은 로컬 공간 격자 5/20·7.5/30·10/40cm 비교 → 선택 격자의 depth 25/50/100cm
  비교 → Vast 최신 소스/새 준비/외부 백업 확인이다. 본학습과 rosbag 성능 검증은 아직이다.

- **2026-10-11 로컬 입력 보완:** Base5에서만 exact XYZ-zero/nonfinite 포인트를 증강 전에
  제거하고 JPEG decode/crop/pack을 uint8으로 유지한다. 실제 GT·픽셀·정규화·투영 정합을
  확인했으며 기존 BEV 증강 강도와 수집 원본·production E25는 유지한다.
  [검사·비용·재개 조건](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-valid-returns-uint8-261011)을
  따른다. Vast는 사용자 중단 상태이며 변경 전 원격 prepare-only 계약을 재사용하지 않는다.
  재개 후 최신 소스 전달·새 준비·짧은 검사·외부 백업 완료가 필요하다. 본학습은 미시작이다.

- **2026-10-11 현재 JPEG 입력 계약:** 사용자 승인으로 카메라 RGB를 train·val·test·분석
  모두 native Q90 JPEG로 통일했다. PNG는 로컬 원본·변환용으로 보존하고, mask·depth·native
  GT·LiDAR·보정은 무손실로 유지한다. 새 config는 `bevfusion_base5_dense_512_native_jpeg90.py`다.
  [전수 정합·Drive 교체·Vast 전환](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#all-jpeg-policy-261011)을
  따른다. 이전 train JPEG / val PNG 및 별도 train 캐시는 비교 이력이다. 이미 준비한 PNG run의
  hash를 덮어쓰거나 그대로 resume하지 않는다. 사용자 재개 후 원격 JPEG 생성·전수 검증과
  새 prepare-only 계약을 완료했다. [원격 입력 전환과 남은 백업 조건](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#vast-jpeg-migration-261011)을
  먼저 따른다. 외부 checkpoint 백업 완료·본학습은 아직이다.

- **이전 선택 — 2026-10-10 23:55 KST:** 사용자 합의로 native JPEG Q90을 네 카메라
  모두 100% 적용한다. blur·노출·noise는 OFF다. 별도 train JPEG 캐시를 추천하고 매번 PNG에서
  변환하는 경로를 대안으로 유지한다. `bevfusion_base5_dense_512_jpeg90_train.py`를 사용하며
  원본 PNG·GT·split·val/test·production은 유지한다.
  [로컬 동등성·로더 비교와 한계](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#jpeg-cache-decision-261010)를
  따른다. CPU 준비 개선을 전체 학습 가속으로 환산하지 않는다. AP와 실제 JPEG 적용 후 학습
  시간은 미측정이다. Vast 재개 요청 후 원격 캐시 생성·전수 검증·새 hash 계약과 외부 checkpoint
  백업 gate를 확인한다. 당시 '전체 train 캐시 아직'은 이전 기록이며 위 10-11 계약이 우선한다.

- **2026-10-10 22:43 KST 범위 결정:** Base5는 전후방 ±100m·좌우 ±30m를 유지한다.
  ±40m 비용 시험은 보존하되 이번 학습에서는 기각한다. 고주로 합류·로터리 대기의
  핵심 객체에 확대 필요성이 입증되지 않았으며 근접·전후방·밀집 사례를 우선한다.
  큰 yaw·곡선의 조기 관측 한계와 [재검토 조건](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#lateral-range-decision-261010)을 남겼다.
  사용자는 Vast를 중단했다고 보고했으며 재개·본학습은 별도 요청 전 실행하지 않는다.

- **2026-10-10 21시 학습 전 확대 검증:** 서로 다른 300장 비용, 64장 전체 모델 동등성,
  실제 5K train batch 1/2/4/8, 전체 val 745장과 원격 저장·재개를 통과했다.
  512×1056·batch 4를 추천한다. 일반 표본 서비스 평균/p95 69.96/70.33ms,
  제안 5K 24 epoch train+val 약 4.8시간이며 드문 지연·ROS 풀스택은 별도다.
  **외부 checkpoint 백업은 원격 DNS/외부 연결 timeout으로 미완료이며 본학습은 미시작이다.**
  네트워크 복구·전체 파일 SHA 재읽기·watcher 검증이 시작 전 남은 gate다.
  [현재 검증과 남은 조건](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#6-4090-확대-표본학습-전-검증--2026-10-10-2023-이후)를 먼저 따른다.

- **2026-10-10 19:49 KST Vast 5K 전달·전수 검증 완료:** 빠진 두 조각만 약 17분
  35초에 복구했다. 19개 조각·결합 압축본·해제 파일 173,290개의 SHA256을 통과했다.
  원격 `/root/base5_sunny13_portable_261010`에 5,000장, train 4,255 / val 745가 있다.
  448/512 모두 token·scene group 분할 겹침 0개다. 본학습·Cloud Sync는 미시작이며
  다음은 최종 입력 선택, 실제 5K loader·학습 재개·체크포인트 외부 백업 검증이다.
  [복구 완료 근거](bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst)가 아래 이전 시점 기록보다 우선한다.

- **2026-10-10 현재 전송 유지 합의:** 기존 Vast 55165763의 사용자 rclone 다운로드를
  먼저 마무리한다. 두 번째 인스턴스는 사용자가 destroy했다고 보고했다. 전송 완료와
  SHA256·해제 완료는 미확인이다. Cloud Sync는 다음 대용량 전송 때 필요하면 검토하는
  선택지이며 초기 OAuth/API 키 설정과 소량 검증이 필요하다. 현재 전송을 중단하거나
  Cloud Sync를 설치·실행하지 않는다.
  [역할 분담·보안·후속 계획](bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의)을 따른다.

- **2026-10-10 18:17 KST 두 번째 Vast 호스트 검사:** 새 인스턴스 55178510의 전용
  Drive curl 4병렬은 데이터 구간 평균 0.748MB/s로 중단했다. rclone 도구 전달도 제한
  시간에 끝나지 않아 기존 8병렬과 동일 조건의 비교는 미완료다. 기존 인스턴스의 사용자
  다운로드를 보존했고 새 호스트의 전체 전송·학습은 시작하지 않았다.
  [두 번째 호스트 근거](bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#두-번째-호스트의-제한-검사--2026-10-10-1817-kst)를 따른다.

- **2026-10-10 18:04 KST 전용 Drive 8병렬 재검사:** API 오류 없이 데이터 구간 평균
  7.720MB/s를 관측했지만 10MB/s 기준 미달로 중단했다. 원격 검사 파일은 제거했고
  전체 5K 전송·본학습은 미시작이다. 지속 속도를 보장하거나 임의로 기준을 완화하지 않는다.
  [8병렬 결과](bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전용-연결-8병렬-검사--2026-10-10-1804-kst)를 따른다.

- **2026-10-10 17:57 KST 전용 Drive 앱 연결 완료:** 기존 `ajou_drive` 보존,
  별도 `ajou_drive_private` 인증·백업 폴더 접근이 성공했다. 원격 rclone 4병렬 재검사에서
  403은 없었지만 데이터 구간 평균 2.073MB/s로 10MB/s 기준에 미달해 중단했다.
  전체 5K 전송·본학습은 여전히 미시작이다. 느린 전송 자동 재개나 임대 변경은 하지 않는다.
  [최신 전송 상태](bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전용-drive-앱-인증과-속도-재검사--2026-10-10-1757-kst)가 이전 공용 앱 오류 기록보다 우선한다.

- **2026-10-10 17:34 KST 원격 비용 검증 완료:** [실제 Vast 4090 측정](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#5-vast-4090-실제-검증--2026-10-10-17311734-kst).
  설치·CPU/CUDA·3개 shape의 동등성·짧은 batch 1/2 역전파를 통과했다. 분리 B의 448/512
  모델 평균은 48.84/51.50ms, 서비스 평균은 65.79/69.71ms다. ROS/full-stack·정확도 인증은 아니다.
  5K 전체 전송은 원격 Drive API 제한과 최신 curl 4병렬 6.088MB/s로 보류했다.
  전송 재시도는 사용자 10MB/s 기준을 지킨다. 최종 shape·원격 저장/재개·외부 백업·본학습은 아직이다.

- **2026-10-10 원격 학습 전 남은 gate:** [로컬 전달·resume/백업 준비](bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-로컬-준비-결과).
  상대경로 5K·직접 실행·epoch 재개·로컬 백업은 준비됐다. 다음은 실제 Vast 이미지/SSH/CUDA 확인 → 4090 비용 측정 →
  최종 crop/batch/초기화 선택 → 외부 백업 검증 → 본학습이다. camera-only pretrained는 검증된 fallback이며
  full BEV pretrained의 축 변환은 아직 검증하지 않았다. **Sunny13 원본 5K Drive 백업은 10-10에 검증 완료**했지만,
  checkpoint 외부 백업 검증·Vast 전송·장기 학습은 아직이다. Drive 공용 client_id의 요청량 한도 오류 403을
  실제로 관측했으므로 장기 자동 백업 전에 본인 client_id 설정과 재검증이 필요하다.

- **2026-10-10 로컬 입력/최적화 검증 완료:** [최신 근거](bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md).
  전수 5K crop/448·512 derived infos/실제 loader-online 정합, 확대 sparse coordinate 축 수정,
  cache A/분리 B의 full-model 동등성 및 3-step 역전파를 검증했다. 아래 10-09 미구현 표기는 과거 기록이다.
  다음은 임대한 4090에서 새 번들 비용/VRAM 확인 → 최종 입력 → 초기화/resume/백업 → 본학습이다.
  과거 4090 확대 비용은 축 수정 전이므로 재측정한다. E25·수집 원본·다른 5개 환경 수집기는 불변이다.
  확대 shape ROS prepared 경로와 4-class 후처리·mission bag 승격은 아직이다.

- **2026-10-10 다음 실행:** Sunny13 최종 선택은 보존하고, 준비된 다른 5개 환경의 새 배치를
  UI 환경 확인 후 수집한다. [새 단일 수집·보충·대시보드 런북](../dataset/morai_od/docs/261010_base5_fresh_environment_runbook.md),
  [현재 수집기 근거](../dataset/morai_od/docs/261010_base5_postcollection_collector_work.md)를 우선한다.
  수집기 검증은 완료됐지만 나머지 실제 25K와 본학습은 미시작이다. GPU/crop은 위 최신 근거를 따른다.
  GPU 작업은 MR 종료 후 입력/crop·투영 → cache/분리 pooling → 동일 조건 비용 → 초기화·학습으로 이어간다.
  아래 2026-10-09 계획의 ‘수집 중/미구현’ 표기를 현재 상태로 재사용하지 않는다.

- **2026-10-09 해상도와 최적화 후속 합의:** [13Hz 예산과 입력 선택](bevfusion/261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md#수집-종료-후-해상도-선택-기준).
  Sunny13 종료 후 crop/투영 → cache/분리 pooling → 동일 조건 4090 비용 → 세로×가로 학습입력 확정이다.
  전처리 포함 평균 50~60ms는 탐색 목표이며 p95·밀집 장면·입력부터 객체 발행까지의 지연을 확인한다.
  448×896/512×1056/576×1152시간은 미측정 최적화 가정이다. TensorRT 성공에 기대어 입력을 키우지 않는다.
  부분 TensorRT는 후속 선택이며 전체전환/원격 MORAI 때문에 본학습을 미루지 않는다. 현재 수집과 runtime은 유지한다.

- **2026-10-09 최신 데이터 수집 합의:** [Sunny13 완료 후 Ego 다양화와 나머지25K](../dataset/morai_od/docs/261009_base5_remaining_environments_spatial_diversity_plan.md).
  현재 Sunny13 5K는 끝까지 완료·보존한다. 종료/Park 뒤 구성표·primary 할당을 유지한 연속 Ego
  샘플링과 환경별 새 배치 생성 경로를 구현·검증한다. 나머지 5 환경은 새 5K씩 수집하며
  총 30K 후 Sunny13 재수집 여부를 판단한다. 아래 동일 geometry6 환경은 과거계획이다.
  기존 다른 환경 복사 manifest는 새검증 전 실행하지 않는다. 현재 수집·production은 변경하지 않는다.

- **2026-10-09 최신 실행 순서:** [Sunny13 종료 후 묶음 실험 합의](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#sunny13-수집-종료-후-묶음-실험-합의).
  이번 5K 수집과 MORAI 종료를 먼저 확인하고 사용자 실행 요청 후 진행한다. 무결성/split →
  입력/crop → cache/분리 pooling → 비용/짧은 학습 → 초기화/smoke/resume 순서다.
  추천 구성 하나와 fallback 하나를 본학습 직전에 보고한다. 현재 수집은 유지하며 GPU 실험은
  종료 후로 미룬다. TensorRT/v3 성공을 본학습 필수조건으로 만들지 않는다.

- **2026-10-09 원격 학습 준비:** [Vast.ai 단일4090 임대·5K 학습/추론 계획](bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md).
  로컬 검증 후 원격 4090에서 최종 입력의 latency/VRAM·짧은 학습을 다시 확인한다.
  임대 전 전달 용량·고정 디스크·registry image와 direct runner를 준비하고,
  외부 checkpoint 백업/재개를 검증한다. 반복 mission bag은 별도 dev-val로 유지한다.
  아직 임대·전송·원격 실행은 하지 않았으며 장기 학습 전 설정·시간·비용을 보고한다.

- **2026-10-09 CUDA-BEVFusion 검토:** [기존 CUDA 사용·v2/v3·배포 호환 조사](bevfusion/261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md).
  학습 체계는 유지하고 cache/분리 pooling을 우선 이식한다. CUDA-BEVFusion depth-outer는
  추론 후보이며 NVIDIA v3와 혼칭하지 않는다. 부분 TensorRT는 잔여 병목 실측 후,
  전체 저장소 이주는 후순위다. 조사만 완료했으며 수집·production·가중치는 변경하지 않았다.

- **2026-10-09 후속 실행 계획:** [Base5 초기화·연산 최적화](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md).
  다음은 고정 기하/인덱스 cache의 CPU 기준·무효화 검사, 제한된 CUDA 연산 단위 시험이다.
  이후 기존 pooling/cache A/v2 방식 B를 분리 검증한다. 수집·production은 유지하고 전체 모델/학습/
  공식 비용 시험은 별도 4090 또는 종료 후다. 아직 최적화 구현이나 CUDA 시험을 한 것은 아니다.
  Base5-only 데이터와 랜덤 초기화를 혼동하지 않는다. E25 fresh는 nuScenes 호환 초기화였고,
  Base5 후보는 현재load_from=None이다. 호환 pretrained + 새 4-class/depth/grid 정책을 추천하며
  adapter 로딩 감사 후 본학습한다. crop/GT 재선별과 checkpoint resume/외부 업로드 검증도 계획에 포함한다.

- **2026-10-09 다음 작업:** 수집을 유지한 채 [카메라 crop 크기와 위치 조사](bevfusion/261009_BASE5_CAMERA_CROP_INVESTIGATION.md)를 따른다.
  측방 잘림 snapshot과 12개 조사 항목을 기준으로 픽셀 밀도, 원본 시야, 카메라별 crop,
  GT visibility와 투영 정합을 먼저 검토한다. CPU 분석 후 우선 후보는 448×896 아래 정렬,
  비용 대안은 384×896, 원거리 밀도 확대 대안은 512×1056이다. 최종 설정은 미확정이며
  다음은 학습·Docker·ROS 전처리 정합과 새 입력의 visibility 재구성 검증이다.
  이후 [4090 비용 시험 계획](bevfusion/261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md)에 후보를 반영해
  팀원용 딸깍 번들을 준비한다. 새 번들은 미구현, 풀스택 측정은 보류이며 본학습 여러 회는 전제하지 않는다.

- **2026-10-09 합의:** [학습split·가시성·temporal 경계](../dataset/morai_od/docs/261009_base5_training_split_visibility_and_temporal_decisions.md).
  6개 환경 30K 수집 계획을 유지하고 완료 후 필요한 조건만 보강한다. 합성 train 약 85/val 약 15이며 동일 geometry의 환경 변형은 split을 고정한다.
  기존 반복 bag은 dev-val, 새 독립 주행은 test다. BabyStroller3의 camera 대응 모호성을 실제 비가시로 단정하지 않는다.
  현재 single-frame Base5·OD velocity OFF이며 temporal 코드는 연구용 보존이지 자동 실행을 뜻하지 않는다.

- 수집 후 입력을 확정할 때 실제 GT visibility와 희귀 자산 coverage를 다시 검사한다.
  crop infos는 임시본이며 새 학습·독립 mission rosbag 검증이 남는다.
  [CURRENT](../dataset/morai_od/docs/BASE5_V2_CURRENT.md)가 과거 v123/v129/v132 운영 명령보다 우선한다.

- 최신[MORAI 답변](../dataset/morai_od/docs/260920_questions_for_morai.md#최신-추가-답변--객체-범위g80차량-속도-기준)에서
  F2 전체 객체가 미션 후보이며 정적 자산의 이동 예외는 없다고 확인했다.
  열린 G80은 공식 출제 제외이며 코드에도 반영했다. 부유 재현 ZIP 전달은 완료됐고
  MORAI 후속 답변 및 나무/불/도로시설 등 기존 background/ignore coverage 검토가 남는다.
  Animal/복합 GT 후보는 유지한다.
  UDP차량 velocity 후륜 기준은 확정됐으나 기존 R01의변환잔차 검증/새평가 계약 선택은 남는다.

- 구 output으로새 runtime 강제 resume하지 않는다. 통신/Park halt는 안전확인후별도 output으로분리한다.
  기존 Sunny13을재수집할지는 전체 30K와 실제 coverage를보고판단한다. 5K 관측을 6회복사하지 않는다.
  새 5 환경의배치규칙은 유지한독립 geometry이며인접시야/같은 지도배경의중복은남는다.
  Ego Park·독립지형·CollisionData·실제 Z/가시성검사는 유지하며 raw GT는사후수정하지 않는다.
  [5060Ti 비용 근거](bevfusion/261006_BASE5_BATCH1_CAPACITY_EVIDENCE.md)는 그대로다.
  4090/확대후보 ROS/full-stack은 아직 별도 검증이며 수집 pilot으로 인증하지 않는다.

- Base5 E2 학습 및 [E1 보행자·장애물9-bag A/B](bevfusion/261005_BASE5_E1_MISSION_ROSBAG_AB.md)는 완료됐다. E1장애물 소폭 개선에도 0–2m 장애물·20–30m 보행자 공백은 미해결이다. 다음은 해당 거리·부분 시야 표적 보강 + replay다. Production E25 유지, Far ROI와 고해상도는 별도 A/B로 유지한다.
- 전방은 빠른 존재 전달을, 후방/로터리는 위치·속도·궤적 안정성을 함께 평가한다.
- 현재 E25의 근접 장애물 누락은 tracker만으로 해결하지 않는다. 부분 시야·얇은
  방향·통과 중 raw/GT를 보강하고 동일 held-out bag에서 재학습 후보를 비교한다.
- 원거리 검증은 40/60/80/100m 가시성부터 시작한다. 현 ±30m config와 향후 모델의
  능력을 구분하며, ROI/학습/평가 범위를 함께 설계한다.
- 고해상도 320×896은 5060 Ti batch 1 약 10–11FPS로측정했다. 정확도·4090 속도는 아직 측정하지 않았다.
- 전후방 ±60m/좌우 ±30m ROI 후보는 축소 입력 forward와 checkpoint 기하 복원 검증을
  통과했다. 확장 GT 재변환·full-input 학습·출력/평가 필터 연계는 아직 미완료다.
- confirmed UNKNOWN→차량 빠른 승격은 opt-in 후보이며 기본 OFF다. 신규 track
  확인을 무조건 생략하거나 검출 없는 객체를 차량으로 만드는 기능은 아니다.
- Production E25 교체는 Detection/Tracking/Map-CV 동일 bag 회귀 검증 후 결정한다.

최신 근거·실행 경계:
[`bevfusion/261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md`](bevfusion/261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md).

## 후속 연구. Base4/Q1 velocity·temporal

- 목적은 100 km/h 고주로 추월에 필요한 Vehicle velocity와 1/2/3초 미래경로를
  개선하는 것이다.
- 과거 개발 bag의 Position-only/GT oracle 숫자를 현재 production 성능으로 재사용하지
  않는다. 최신 팀원 bag의 보정 GT 비교는 BEVFusion index의 2026-10-04 EVIDENCE를 따른다.
- 이미 수행한 `B4T-HWY-P0` 및 Q1 이력을 참고하되, 새 근거 없이 초기 pilot을 자동 재개하지 않는다.
- 새 데이터는 실제 LiDAR cadence의 연속 sequence로 수집한다. LiDAR timestamp에
  Ego/Object pose를 보간하고 학습 GT는
  `velocity_target_ego_mps=[vx,vy]` 하나만 노출한다.
- 과거 sweep은 Ego motion만 현재 frame에 정합한다. 추론에 존재하지 않는 GT Object
  motion compensation은 input에 사용하지 않는다.
- single-frame, 3-sweep, dense temporal BEV, query memory는 이미 실험 이력이 있다.
  다음 architecture는 Q1 결과와 새 bag의 실패 분해를 보고 선택한다.
- Pilot dataset은 sequence 단위 train/val만 사용한다. 기존 고주로 합류·고주로 1
  bag은 반복 사용된 dev 평가이며 최종 test가 아니다. 최종 후보는 새 held-out 고주로
  bag 최소 2개에서 한 번 평가한다.
- 각 모델은 tracker velocity `off/on`을 모두 평가한다. 첫 velocity covariance는
  `0.5 m/s`이며 geometry, association, 2D AVE와 Map-CV FDE를 함께 본다.

실험 이력:
[`bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md`](bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md).

## P2. Localization과 velocity pilot 이후 Prediction 개선

현재 구현값과 미구현 경계는
[`../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md`](../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를
따른다.

- 기본 prediction horizon을 5초로 확장하되 로터리에서는 3--4초로 줄이는
  map-zone-aware 정책을 검토한다.
- 고주로 추월을 방해하는 다중 후보 문제는 단순 `top_k=1` 강제가 아니라 tracker
  history, Lanelet 연결성, 횡위치와 차선 변경 가능성을 결합한 temporal lane
  association으로 Top-1 정확도를 먼저 높인다.
- 같은 차량의 인접 차선 점유 판단이 일정 시간 안정적으로 유지될 때만 Planning이
  추월 gap으로 사용하도록 track age와 prediction confidence 계약을 정한다.
- Localization과 detector velocity 변경을 동시에 섞지 않는다. P0와 P1의 동일 bag
  회귀 검증이 각각 끝난 뒤 별도 브랜치에서 수행한다.

## 완료되어 자동 재개하지 않는 항목

- Base3-Static 10K 수집, validator 소급 감사와 5-point GT 정책.
- Base3 LRASPP, PointPainting PP0/PP1과 FastBEV 연구.
- Base3 BEVFusion E40 학습과 production E25 선택.
- Base3-Online 고주로 334-scene 수집과 E25 fine-tuning.
- Tracker motion-model/covariance sweep과 2D AVE evaluator 수정.

과거 결과가 필요하면 [`bevfusion/README.md`](bevfusion/README.md),
[`../dataset/morai_od/docs/README.md`](../dataset/morai_od/docs/README.md) 및 각
`legacy/README.md`에서 찾는다. 새 근거 없이 완료 항목을 최우선 계획으로 되돌리지 않는다.
