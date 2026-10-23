# BEVFusion 현재 모델과 실행 기준

문서 상태: CURRENT. 최종 근거 확인: 2026-10-23.
2026-10-23 사용자 요청으로 production 기본값은 **전체 30,750장 E12 + camera TRT + head SDPA·좌표 cache**다.
팀 공유 planning 설치기·validator 기본도 E12다. E24/E25는 명시적 복귀 후보이며 아래 과거 실험의 ‘E24 유지’는 당시 결정이다.
구 Sunny13 모델의 native-val best E9와 production E24의 선택 기준을 혼동하지 않는다.
상세 결과는 [작업별 근거 인덱스](README.md)에서 필요한 절만 읽는다.

<a id="model"></a>

## Base5 모델과 학습 상태

| 항목 | 현재 선택과 상태 |
|---|---|
| 학습 완료 | 구 Sunny13 5K는 24 epoch 완료: native val best E9, production E24. 새 Sunny15 5K 일반 코사인과 warm-up restart는 모두 8 epoch 정상 완료하며 각각 마지막 E8이 자체 val best |
| 전체 데이터 학습 | **2026-10-23 10:40:01 KST, 30,750장 학습을 12 epoch·78,408 iteration·exit 0으로 완료했다.** train 26,136 / val 4,614·batch 4·일반 코사인·compile OFF·val batch 1/FP32와 [지표 계약](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-val-logging-261021)은 유지했다. native 전체 best는 **E12 mAP 94.07%(V 97.27 / P 96.53 / O 88.41%)**다. W&B finished와 원격 E12 full/best의 전체 SHA·ZIP CRC·CPU load·계약·상태 보존을 확인했다. **10:48 KST 최종 두 파일의 Drive 백업은 중복 metadata·COMPLETE 부재로 미완료**이며 인스턴스는 자동 Stop/Destroy하지 않았다. production E24는 유지하고 E10의 기존 bag 결과를 E12의 검증으로 전용하지 않는다. [최종 결과·저장·백업 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-e12-completion-261023), [E7 호스트 복귀 이력](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-e7-return-original-host-261023)을 따른다 |
| 전체 데이터 후기 bag 비교 | [E12 최종 비교](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-e12-bags-far-front-261023)는 기존 **14장면·5,986입력/5,822평가**에 E12를 추가했다. 기존/추가 9월 묶음 mAP는 E24→E12 **53.65→60.18 / 55.89→59.58%**이며 E10과는 일부 엇갈린다. 동일 native 양성+음성 512장 전방 80~100m AP는 **34.41→62.06%**, score 0.4·1m 적중은 **38→62/134**다. 이 subset을 전체 val AP와 혼용하지 않는다. [0점·거리별 score 후속](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-zero-lidar-threshold-research-261023)에서 전방 0점 GT는 80~90/90~100m **16/37관측**이며 낮은 score의 근접 후보가 상당수 남았다. 그러나 실제 bag 0점 214관측의 score 0.4→0.1·2m 적중은 **1→4**에 그쳤고 전방 초장거리 catalog 미대응 출력은 **7→73**으로 늘었다. [1점 이상 GT 보조 평가](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-positive-lidar-evaluation-261023)는 E12 전체 val 4,614장의 mAP **94.07→96.09%**를 확인했다. GT만 줄이고 모든 예측·프레임은 보존했으며 평가 불가 class를 0%로 쓰지 않는다. 이는 모델 개선이나 실제 bag 원거리 문제의 해소가 아니다. production E24·threshold·tracker·센서는 유지한다 |
| 완료된 데이터 대조 학습 | 새 Sunny13 **v158a 원형 5K**는 기존 E24의 동결 source/init·seed·24 epoch·batch 4·AdamW·cosine·compile ON 조건으로 10/19 04:11:54~08:48:18 KST, 약 **4시간 36분 24초·exit 0**에 완료했다. full E24·자체 val best E20의 Drive 전체 SHA/readback과 W&B 완료를 확인했다. 같은 val의 E24 교차 평가 mAP는 기존 745장에서 **83.47→87.63%**, 신규 750장에서 **89.93→91.00%**다. 근거리 장애물은 두 val에서 개선됐지만 **30~50m 장애물은 미해결**, 원거리 차량도 일관된 개선이 아니다. E20은 교차 평가하지 않았다. Foggy13의 149장 거리 보강 학습 결과가 아니며 일반 새 학습의 compile OFF와 **기존 production E24는 유지**한다. [완료·백업·네 칸 비교와 한계](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny13-matched-e24-result-261019) |
| 클래스 | vehicle / pedestrian / obstacle. Animal은 학습 라벨만 Pedestrian에 통합하고 원래 의미·asset·BBox는 보존 |
| 시간 모델 | single-frame 7D. temporal modeling·OD velocity supervision은 OFF |
| 이미지 | Q90 JPEG, 모델 입력 512×1056, resize 0.825. [512/576 진단](261016_BASE5_512_576_CROP_HEIGHT_DIAGNOSTIC.md) 후 현재 크기 유지. native 크기·homography 보존 |
| 공간 | 의미 ROI X±100/Y±30 m, LiDAR XY voxel 5 cm / Camera BEV 20 cm. 내부 padding과 의미 ROI를 구분 |
| 깊이 | 1–100 m, bin 간격 0.25 m. 간격이 실제 depth 정확도를 보증하지 않음 |
| 초기화 | 축·물리 단위를 변환한 호환 nuScenes camera/sparse·BEV·fusion·head 재사용. E25/COST-only 가중치 초기화가 아님 |
| 학습 실행 계약 | 완료 E24의 compile ON은 보존한다. 새 Sunny15 두 후보는 compile OFF·nearest·batch 4이며 각각 약 1시간 47분에 완료했다. 두 run identity 차이는 scheduler뿐이며 restart는 cosine E8 resume가 아니다. 동일 val의 center mAP는 cosine 88.49% / restart 88.05%다. 단일 seed의 결과로 일반적인 우열을 단정하지 않으며 짧은 학습 예산의 우선 후보는 일반 코사인으로 유지한다. [대조 결과](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-restarts-e8-result-backup-261019)를 따른다 |
| 검증과 보존 | 구 Sunny13 train 4,255 / val 745, 새 Sunny15 train 4,250 / val 750. E9/E24 Drive 백업·readback은 완료했다. 새 Sunny15 cosine은 10/19 00:10 KST, restart는 03:31 KST에 각각 best E8·full E8 수동 Drive 백업·전체 SHA readback·metadata·COMPLETE·full용 LATEST 검증을 완료했다. full은 학습 상태를 포함하며 best는 weights-only다. 원격 원본은 보존한다. 자동 전송의 속도·1,200초 제한 문제를 해결했다는 뜻은 아니다. [cosine 백업 근거](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-e8-manual-drive-backup-261019)와 아래 restart 백업 근거를 따른다. rosbag val은 train에 혼합하지 않음 |

호스트 복귀 시 W&B 숫자 지표와 새 E8 텍스트 Logs는 **기존 학습 run**에서 확인했다. E7 호스트 복귀도
SDK 콘솔 OFF와 실제 훈련 PID에 묶인 files-only writer를 사용한다. 텍스트는 API 검산 때문에 몇 분 늦을 수 있다.
기존 [콘솔 보조 run](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/base5_full30750_e12_console_261022_v1/logs)은 과거 이력이며 AP 비교 대상이 아니다.
SDK 0.22.3 자체는 교체하지 않았다. [이전 버그 진단](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#wandb-console-recovery-261022)과 위 E5 복구 근거를 구분한다.

데이터 원본·파생 infos·환경 준비는 [데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)에서 관리한다. [본수집 전 입력 검사](../../dataset/morai_od/docs/261016_base5_precollection_final_review.md)의 CPU 전처리 통과는 새 모델 AP 검증이 아니다.
새 Sunny15 5K의 Q90/upright·3클래스 전달본 전수 검증과 Drive 저장을 완료했다.
E24의 실제 입력 계약과 대조했고 같은 JPEG·GT·보정 규칙을 사용한다. Vast 첫 수신의 8MB/s
중단 뒤 사용자 승인으로 정상 범위를 재사용해 전체 archive SHA까지 통과했다. 복원·입력
전수 검증과 두 후보의 데이터 연결·설정 준비·GPU/W&B/Drive 온라인 검사도 모두 통과했다.
일반 코사인 최종 E8 center mAP는 **88.49%(V 94.60 / P 91.17 / O 79.69%)**다.
구 Sunny13 E24 자체 val의 83.47%와 검증셋이 달라 순수 개선량으로 해석하지 않는다.
원거리 차량 수치는 유망하지만 구 E24와 비교하면 10~30m 장애물은 낮은 구간이 남았다.
구 실험의 첫 8 epoch에서 전체 mAP가 가장 높았던 E8끼리 비교하면 장애물 20~30m는
72.90→71.06%이며, 해당 구간 자체 최고끼리는 구 E5 74.59% / 새 E6 71.53%다.
검증셋·스케줄이 다르므로 모델 퇴보나 추가 학습의 회복을 단정하지 않는다.
실제 train의 해당 GT는 610→677개로 늘었다. 새 native 716개 중 센서 지원이 없는 39개를
제외한 수이며, 주된 보강은 0~20m에 집중됐다. [첫 8 epoch·GT 대조](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-first8-distance-review-261019)를 따른다.
공통 val/bag의 새 E8 대 E24 교차 평가는 아직 하지 않았고 production은 E24를 유지한다.
완료 재시작형은 4 epoch×2주기·주기별 500 iteration warm-up, 동일 초기화·seed·학습 예산이다.
restart E8은 vehicle 95.12 / pedestrian 90.09 / obstacle 78.93%다. 차량 일부 지표는 높지만
장애물 20~30m는 cosine 71.06 / restart 69.99%로 이번 scheduler 변경으로 해결되지 않았다.
restart의 best E8(weights-only)과 full E8(optimizer·scheduler·AMP·RNG 포함)도
10/19 03:31 KST 수동 Drive 백업을 완료했다. 두 payload와 metadata·COMPLETE,
full용 LATEST를 전체 재읽기·SHA로 검증했으며 원격 원본은 보존했다.
[restart 백업 근거](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-restarts-e8-result-backup-261019)를 따른다.
이는 기존 자동 백업 오류를 고치거나 그 상태를 성공으로 덮어쓴 것이 아니다.
학습 정상 종료·W&B 수신·실제 Drive 백업 완료는 구분한다.
[변환·전송 근거](../../dataset/morai_od/docs/261018_base5_unified_generation_capture.md#sunny15-training-package)를 따른다.
과거 4클래스 E2나 다른 초기화/shape의 optimizer를 현재 계약에 강제로 resume하지 않는다.
새 학습은 사용자 요청과 새로운 source/data/work 계약을 확인한 뒤 시작한다.

<a id="checkpoints"></a>

## 평가 checkpoint

| 후보 | 로컬 파일 | SHA256 |
|---|---|---|
| E9 native-val best | `artifacts/evaluation/bevfusion/base5_rosbag_quick_261012/checkpoints/E9.pth` | `067205c3f5f5b0abc1bb775116c2b8bca2c9a7f31a0727b959db0ab1ce70b271` |
| E24 마지막 epoch | 같은 디렉터리의 `E24.pth` | `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746` |
| 전체 30,750장 E10 비교 후보 | `artifacts/evaluation/bevfusion/full30750_latest_compare_261023_v1/E10_best.pth` | `6e489870f59ad42b940b20ec7be2e0b0712406c78588f24cb1abf7af28eecae7` |
| 전체 30,750장 E12 native best·비교 후보 | `artifacts/evaluation/bevfusion/e24_e12_bags_261023_v1/checkpoint/E12_best.pth` | `f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8` |

Best는 native val 선정 기준이며 모든 bag·미션에서 우월하다는 뜻은 아니다.
기본 추론 config는 [nearest runtime config](../../perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_nearest_depth_runtime.py)다.

<a id="runtime"></a>

## Production과 시험 실행

| 실행 역할 | 현재 계약 | 실행 기준 |
|---|---|---|
| Production E12 | 전체 30,750장 E12, camera TRT·head SDPA·좌표 cache 요청 ON, AMP·병렬화·nearest, compile OFF, gravity-center, V/P/O 0.40/0.10/0.10, NMS 0.50. PC별 engine 준비 후 자동 선택하며 미준비 시 같은 E12 eager | [배포 handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md), `bevfusion`; `--safe`는 같은 E12 원형 |
| E24 명시적 복귀 | 기존 E24·Base5 입력·threshold 보존, 추가 head/TRT 기본 OFF | `bevfusion --model e24` |
| E25 명시적 복귀 | V/P/O 0.40/0.15/0.10, class-aware rotated BEV NMS 0.50 | [E25 handoff](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md), `bevfusion --model e25` |
| Base5 baseline trial | 별도 namespace·서버. production을 자동 교체하지 않음 | [ROS 패키지](../../base_ws/src/perception/bevfusion_ros/README.md), `base5_trial.launch` |
| E24 선정 trial | AMP·camera 병렬화 ON, 추론 compile OFF, V/P/O 0.40/0.10/0.10 | `base5_e24_calibrated_trial.launch`, [threshold 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-local-threshold-calibration-261012) |
| E9 선정 비교 trial | AMP·camera 병렬화 ON, 추론 compile OFF, V/P/O 0.30/0.20/0.12 | `base5_e9_calibrated_trial.launch`, [E9/E24 최종 선택](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-e9-e24-selection-261013) |
| E9/E25 완주 화면 비교 | 동일 3,369장·공통 중심 XY±30 m·두 모델 모두 0.40/0.15/0.10 | [완주 비교 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-stable-whole-route-view-261012) |

Base5 ROS 입력은 원본 JPEG를 재압축하지 않고 원본 XYZI 0–255를 전달한다.
서버의 `--base5-runtime --point-intensity-source ros255`가 강도를 정확히 한 번 ÷255 한다.
E25의 원래 강도 계약과 혼용하지 않는다. Launch만 실행해서 GPU 서버·checkpoint가 자동 준비된다고 가정하지 않는다.
[후단 CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)의
topic·coasting·motion policy는 detector 선택과 별도다.

평가 pose는 [GT 좌표 계약](GT_EVALUATION_COORDINATE_CONTRACT.md)을 따른다. 운영 localization/TF와 독립 raw Ego GT를 구분하고 MR −1.347°를 중복 적용하지 않는다.
2026-10-15 PR #38·#39·#40으로 LiDAR 50을 포함한 전체 코드를 planning·develop·main에 반영했다. E24 ZIP `261015_lidar50`도 현재 센서 launch 해시에 맞춰 재동결했다. 가중치·추론 설정은 유지하며 [E24 handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)의 설치·검증 기준을 따른다.
Validator 분석은 [현재 런북](../planning/SCENARIO_VALIDATOR_RUNBOOK.md#5-결과-확인공유)을 따른다. [최초 공유 검토](261014_VALIDATOR_ANALYSIS_SHARE_HANDOFF.md)는 이력이며 실제 팀 PC 실행·공유 검증은 별도다.

<a id="optimization"></a>

## 추론 최적화의 현재 결론

완료 E24의 학습 compile ON은 보존하되 다음 학습은 compile OFF로 분리했다. 이는 OFF의 정확도 우위를 인증한 결과가 아니다.
병렬화는 검증된 추론 경로에서 사용하며, 추론 compile은 출력·연속 tracker 예외 때문에 기본 OFF를 유지한다.
[학습된 4090 비용/메모리](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-learned-runtime-vast-261012)와
[실제 ROS tracker 비교](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-compile-actual-ros-tracker-261012)를 구분한다.
48GB 카드의 20GiB 제한 시험은 실제 대회 24GB 카드의 ROS/LAN·풀스택 인증이 아니다.
비용용 빈 출력·query 고정 통제를 완성 모델의 정확도/배포 근거로 사용하지 않는다.

BEVPool/CUDA 경로 개선은 적용된 구현을 따른다. CUDA-BEVFusion은 참고 구현이며 새 프레임워크로 전체 이주한 상태가 아니다.
[부분 TensorRT 실험](261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md#e12-partial-trt-implementation-261023)을 E12·4090에서 구현했다. [60절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-partial-trt-cuda-experiment-261023)의 전체 val 4,614장 동일 AMP 비교 mAP는 **94.0545→94.0661%**, V/P/O 변화는 −0.0017/−0.0014/+0.0378%p다. [61절 후속](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#continuous-trt-competition-startup-261023)의 원본 연속 480장은 서비스 평균 **75.25→65.68ms**, 전달 903박스 추가/누락·실제 EKF 코어의 교차 실행 ID 대응 교체 0이다. 큰 raw yaw 예외와 코어 범위 한계도 기록했다. [62절의 head 추가 검증](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#head-native-ap-runtime-261023)은 camera TRT 고정·SDPA+고정 좌표 cache만 ON으로 전체 4,614장의 mAP **94.0661→94.0656%**, V/P/O 변화 **+0.00004/−0.00017/−0.00128%p**를 확인했다. 앞선 별도 480장의 서비스 평균 **65.97→60.34ms**와 함께 실용적인 선택 후보로 판단해 runtime 옵션을 연결했다. 2026-10-23 사용자 승인으로 **E12 production에서 camera TRT·head SDPA/cache 요청을 기본 ON**으로 승격했다. PC별 setup receipt가 없거나 선택 가속기 준비가 실패하면 같은 E12 eager를 사용하고, `--safe`는 추가 가속을 끈다. topk와 CPU 스레드 증가는 이득/CPU 비용상 채택하지 않았다. **production은 E12이며 compile은 계속 OFF**다. E24/E25는 명시적 복귀로 보존하고 E12 결과를 E24 전체 AP 인증으로 전용하지 않는다. 실제 24GB 풀스택은 아직 미검증이다. [실대회 시작 감사·안전 실행](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md#competition-startup)에서 선택 가속기의 시작 복귀와 `bevfusion --safe`를 구현했고, 네트워크 차단 E24 GPU warmup·실제 ready PONG 및 원본 3입력 RPC를 확인했다. E12·4090 실제 서버 생성과 TRT+head 조합의 3입력 출력도 검증했다. 필수 weights/sensor·중복 publisher 오류는 우회하지 않으며 옛 ZIP 코드 snapshot만으로 동일 weights 설치를 막지 않는다. 팀 PC 배포·실제 ROS/LAN 주행은 별도다. 공개 Orin FPS 비교·Swin 유지 근거는 [배포 추가 검토](261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md#e12-orin-scope-safe-startup-261023)를 따른다. 구조 축소·INT8·새 sparse backend·재학습은 이번 완료 범위가 아니다.

<a id="next"></a>

## 완료 결과와 남은 작업

완료: 학습·백업, 원본/파생 GT 보완, ROS 입력/TCP 정합, 연속 dev bag, threshold 비교,
원거리 GT 평가·기억 A/B, 안정적인 이전 수동 완주 E9/E25 화면 비교다.
E9 별도 threshold 선정과 동일 조건 E9/E24 최종 비교도 완료했다.
원본 Base3 val 1,635장 교차 평가도 완료했다. 해당 분포의 Obstacle AP는 E25 97.95%,
E9 87.55%, E24 87.97%로, Base5 val의 난이도 차이만으로 격차를 설명할 수 없다.
구 Base3 입력의 자차 근처 LiDAR 회신점에 E24가 차량으로 반응했다. 32장 대조에서
해당 점만 제외하자 자차 위치 차량 관측은 30→0건이 됐지만 장애물 AP는 개선되지 않았다.
현재 Base5 val·개발 bag에는 그 회신점이 없었다. 일반적인 자차 제거 필터는 적용하지 않았다.
기본 실행은 E12 0.40/0.10/0.10이며 E24/E25는 명시적 복귀, 구 E9는 비교 후보로 보존한다.
production 승격은 사용자 선택이며 실제 4090 24GB 풀스택 검증 완료를 뜻하지 않는다.
구 입력에 동일 threshold가 그대로 통한다는 뜻은 아니다. [교차 평가 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-on-base3-val-261013)를 따른다.
각 결과의 표본·정밀도·제약은 [근거 인덱스](README.md#evaluation)를 따른다.

후속의 기준은 [원거리 차량과 근거리 객체 개선 계획](261013_BASE5_RANGE_AND_TRACKING_IMPROVEMENT_PLAN.md)과
[오프라인 센서·중복·노드 검토](261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md)다.
팀장님 검증과 로컬 분석을 분리하며 원거리 라벨 삭제·Vehicle 강제 변환·긴 기억은 적용하지 않는다.
2026-10-16 [80~100m 검토와 v153 후보](../../dataset/morai_od/docs/261016_base5_far_vehicle_reinforcement.md)에서 실제 native 지원과 최신 저장 E24 출력을 분리했다. 0점 camera-only도 개선 목표이며 threshold만 낮추지 않는다. 새 배치 검증은 재학습 성능 개선의 인증이 아니고 현재 모델·운영값은 유지한다.

1. 최신 팀장님 E24 완주·무충돌 bag 분석은 완료했다. [활성 우선순위](../ACTIVE_PRIORITIES.md#perception-latency)에 따라 새 계측 bag 도착 시 단계별 지연·실제 consumed path·YOLO 동시 부하를 우선 분석한다. 생성기 보완은 기다리지 않는다.
2. 동일 저장 검출의 Vehicle score 0.40/0.35/0.30/0.20/0.10 CPU 비교를 완료했다.
   [47절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-range-tracking-pivot-261013)의 baseline count는 기존 근거와 같으며 production 값은 유지한다.
3. 낮은 score 갱신의 독립 실제 EKF 원형과 12프로필 비교를 완료했다. 이동·클래스 안정·XY 전용 후보만 시험용으로 남겼다.
   [48절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-low-score-core-probe-261013)의 효과는 주로 멀어지는 차량이며 실제 ROS·새 초기 접근 검증 전에는 production에 적용하지 않는다.
   중복 birth guard와 빠른 차량 승격 OFF를 유지한다. 지도 보조는 미구현 후속이다.
   [49절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-path-consumers-261014)의 소비 코드·기존 bag·현재 지도 재생을 확인했다. LC는 최고점 동점 전체, Cruise는 현재 설정 1개, 로터리는 0.05 이상 후보를 검사한다.
   [50절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-cv-candidates-261014)의 관측 적합도·횡이동 후보도 비교했다. 일부 평균 이득보다 소수 큰 순위 악화가 남아 예측 정책은 유지한다.
   출력 동일 geometry cache·독립 메시지 복사·DFS 최적화는 소스와 이 PC의 설치본에 반영했다. 팀장님 환경·실제 주행은 미검증이다. [CPU 근거](261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#cpu)를 따른다.
   중복 pixel 쓰기의 depth hint 변동을 분리하고 E24 추론에 nearest를 반영했다.
   [Live 검토](261014_LIVE_PERCEPTION_CANDIDATE_REVIEW.md)의 새 3,647장·실제 서버 반복은 차이 0이며 이전 native/dev 945장 근거와 함께 검증했다. 학습·가중치는 미변경이다.
   nearest의 연속 1,570장 A/A는 정확히 같지만 compile association/잔류 예외가 남아 기본 OFF를 유지한다.
   E24 현재 ROS 박스는 tagged bottom→gravity adapter를 사용한다. 실제 ROS 170프레임·512박스와 fresh tracker를 확인했다.
   [Z 계약·정지/reset 경계](261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#box-origin)를 따라 fresh stack으로 시작한다.
   E9/E25 비교·복귀의 legacy 표현, Map-CV future-Z·실제 planner 확인은 별도다. 기존 GT·가중치는 보존한다.
   [연결·이동 후속](261014_PEDESTRIAN_ASSOCIATION_AND_LIDAR_MOTION_FOLLOWUP.md)의 보행자 Gaussian 순위는 운영 pipeline에 ON이다. 허용 조건·수명·차량·장애물 정책은 유지하고 OFF 복귀를 제공한다.
   복합체는 별도 관측한다. [차량 검증](261014_VEHICLE_DIRECTION_REACQUISITION_REVIEW.md)의 입력 경합·TF 실패 pending 보존은 수정·로컬 설치했다.
   [독립 차량·패킷 검증](261014_INDEPENDENT_VEHICLE_AND_LIDAR_50_51_REVIEW.md)의 새 고주로/회전 6회는 도착·무접촉이다. 방향 가드는 원형이고 재취득 공분산 후보도 복구 예외 악화로 운영 미적용이다.
   실제 50은 단일 source batch·수신 age 감소를 재현했고 2026-10-14 사용자 승인으로 운영 기본값을 50에 전환했다. 900 RPM·첫 패킷 stamp는 유지하며 51은 명시적 복귀 값이다. LAN 실전 인증은 별도이고 E24 head peak 변경은 계속 기각한다.
4. [v154 본수집 전 검토](../../dataset/morai_od/docs/261016_base5_precollection_final_review.md)에서 기존 근접 보강·새 원거리 거리 분산·음영 지면 exact 재사용·native 및 입력 변환을 확인했다. 기존 5K·E24·GT 정책은 보존하며 다른 다섯 환경은 별도 최신 배치·검증이 필요하다.
5. 다음 학습의 compile OFF·nearest 설정과 CPU 사전검사는 준비했다. GPU 자격 검증·짧은 학습 비교 뒤 최종 학습하며 Base3 val·개발 bag을 train에 재사용하지 않는다.
   고정 이미지 registry·새 호스트 pull과 독립 rosbag-to-val/test는 별도 후속이다.

원격 명령·재개·백업은 [원격 실행 안내](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md)를 따른다.
인스턴스 실행 여부를 문서 날짜로 추정하지 않으며 사용자 요청 없이 임대·Stop·Destroy하지 않는다.
현재 결정은 이 문서의 기존 항목만 갱신한다. 과거 계획은 [보존된 원문](../perception/legacy/context_261012/README.md)에 있다.
