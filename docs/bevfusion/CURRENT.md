# BEVFusion 현재 모델과 실행 기준

상태: CURRENT. 문서·실행 계약 대조: 2026-10-24.
사용자 승인 production은 **전체 30,750장 E12 + camera TRT + head SDPA·좌표 cache**다.
현재 결정만 이 문서에서 관리한다. 날짜별 실측은 [근거 인덱스](README.md#evaluation)의 관련 절만 읽는다.

<a id="runtime"></a>

## Production과 명시적 복귀

| 선택 | 현재 계약 |
|---|---|
| `bevfusion` | E12·AMP·camera 병렬화·nearest·gravity center, V/P/O 0.40/0.10/0.10, NMS 0.50, compile OFF |
| 선택 가속기 | PC별 camera TRT·head SDPA/cache 요청 ON. setup receipt/engine 준비 실패 시 **같은 E12 eager**로 복귀 |
| `bevfusion --safe` | 같은 E12 weights·입력·threshold를 유지하고 추가 가속만 OFF |
| `bevfusion --model e24` | 기존 E24·Base5 입력·threshold 보존, 추가 head/TRT 기본 OFF |
| `bevfusion --model e25` | 기존 E25 입력·V/P/O 0.40/0.15/0.10·class-aware rotated BEV NMS 0.50 |
| 시험 | `base5_trial.launch` 및 calibrated trial은 별도 서버·namespace이며 production을 자동 교체하지 않음 |

[E12/E24 production handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md#competition-startup),
[E25 복귀 계약](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md),
[ROS 패키지](../../base_ws/src/perception/bevfusion_ros/README.md)에서 설치·실행·중복 publisher 검사를 확인한다.
가속기 실패 복귀는 필수 weights/sensor 오류나 중복 canonical publisher를 우회하지 않는다.
실제 대회 24GB GPU·팀 PC·ROS/LAN 풀스택 검증 완료를 뜻하지 않는다.

## 입력과 후단 경계

- Base5는 원본 Q90 JPEG·4카메라·512×1056/resize 0.825와 보정을 사용한다. native 크기·homography를 보존한다.
- 호스트는 원본 JPEG를 재압축하지 않고 XYZI 0–255를 전달한다. 서버의 `--base5-runtime --point-intensity-source ros255`가 정확히 한 번 ÷255 한다. E25의 원래 강도 계약과 혼용하지 않는다.
- single-frame 7D, vehicle / pedestrian / obstacle이다. Animal은 학습 라벨만 Pedestrian에 통합하며 native 의미·asset·BBox는 보존한다. temporal·OD velocity supervision은 OFF다.
- 의미 ROI X±100/Y±30m, LiDAR XY voxel 5cm·camera BEV 20cm, depth 1–100m/bin 0.25m다. padding·depth bin과 실제 정확도를 구분한다.
- 현재 E12/E24 detection pose는 tagged bottom→geometric/gravity center다. 원점을 hot-switch하지 않고 detector·tracker·planner를 정지한 뒤 fresh stack으로 시작한다.
- 후단은 [Tracking CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를 따른다. **로터리/일반 3초·지도 경로가 충분한 고주로 Map-CV 8초**, moving coast 0.5초다. 지도 부족 시 기존 Map-CV/CTRV 3초를 유지하고 CTRV 8초는 쓰지 않는다. native6초 전역 적용·장기 coasting·최근 연결 후보는 미승격이다.
- 평가 raw Ego GT와 운영 localization/TF는 별도다. [GT 좌표 계약](GT_EVALUATION_COORDINATE_CONTRACT.md)을 따르며 MR −1.347°를 중복 적용하지 않는다.

<a id="model"></a>

## 학습 완료와 보존

| 항목 | 현재 상태와 근거 |
|---|---|
| 전체 30,750장 | 2026-10-23 10:40:01 KST, 12 epoch·78,408 iteration·exit 0 완료. train/val 26,136/4,614, batch 4, 일반 cosine, compile OFF, val batch 1/FP32 |
| native val best | E12 mAP **94.07%** (V/P/O 97.27/96.53/88.41). 전체 SHA·ZIP CRC·CPU load·상태 보존 및 W&B finished 확인 |
| E12 백업 주의 | 완료 당시 최종 두 파일의 Drive 백업은 중복 metadata·COMPLETE 부재로 **미완료**였다. 원격 원본을 보존하고 문서 날짜로 인스턴스 상태를 추정하거나 자동 Stop/Destroy하지 않는다 |
| 완료된 구 run | 구 Sunny13 5K E24(학습 compile ON), native best E9; 새 Sunny15 cosine/restart 각 E8; matched v158a Sunny13 E24/best E20. 동결 source·resume·원격 원본·백업 계약을 보존 |
| 새 학습 | 사용자 요청과 새 source/data/work 계약 확인 뒤 시작. compile OFF·nearest를 기본 후보로 삼되 완료 E24의 compile ON을 소급 변경하지 않음 |

[전체 완료·저장·백업](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-e12-completion-261023),
[원격 학습·재개 인덱스](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md),
[데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)를 따른다.
검증셋·scheduler·seed가 다른 native AP를 순수 개선량으로 해석하거나 bag val을 train에 섞지 않는다.
학습 정상 종료·W&B 수신·실제 Drive 백업 완료는 서로 다른 판정이다.

<a id="checkpoints"></a>

## 평가 checkpoint

| 후보 | 로컬 파일 | SHA256 |
|---|---|---|
| E12 production/native best | `artifacts/evaluation/bevfusion/e24_e12_bags_261023_v1/checkpoint/E12_best.pth` | `f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8` |
| E24 복귀 | `artifacts/evaluation/bevfusion/base5_rosbag_quick_261012/checkpoints/E24.pth` | `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746` |
| E9 구 native best | 같은 디렉터리 `E9.pth` | `067205c3f5f5b0abc1bb775116c2b8bca2c9a7f31a0727b959db0ab1ce70b271` |
| E10 비교 전용 | `artifacts/evaluation/bevfusion/full30750_latest_compare_261023_v1/E10_best.pth` | `6e489870f59ad42b940b20ec7be2e0b0712406c78588f24cb1abf7af28eecae7` |

런타임 모델 config는 [nearest config](../../perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_nearest_depth_runtime.py)다.
‘native best’는 모든 bag·미션에서 우월하다는 뜻이 아니다.

<a id="optimization"></a>

## 최적화·평가의 현재 결론

- E12 camera TRT의 native val 4,614장 동일 AMP mAP는 94.0545→94.0661%, head SDPA/cache 추가는 94.0661→94.0656%였다. 원본 연속 입력·실제 EKF 코어 비교를 별도 수행했다. [60~63절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-partial-trt-cuda-experiment-261023)의 표본·raw yaw 예외·코어 범위를 따른다.
- compile은 연속 tracker 예외 때문에 추론 기본 OFF다. topk/CPU 스레드 증가는 채택하지 않았다. 구조 축소·INT8·새 sparse backend·재학습은 완료 범위가 아니다.
- E12/E24 bag 비교는 14장면·5,986입력/5,822평가다. 동일 native 512장 전방 80~100m AP는 34.41→62.06%였으나 실제 bag 0점 차량은 threshold만 낮춰 해결하지 못했다. 전체 val·subset·실제 bag을 혼용하지 않는다. [57~59절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-e12-bags-far-front-261023)을 따른다.
- [64~65절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-tracking-map-horizon-261023)은 cached eager E12 코어의 coasting·속도·3/5/6초·CV/CTRV 당시 비교다. 새 TRT ROS 30Hz·LAN·planner 폐루프 인증이 아니다. 현재 길이는 위 runtime의 3/8초 계약을 따른다.
- 비용용 빈 출력·query 고정·48GB GPU의 20GiB 제한 시험을 완성 모델의 정확도나 실제 대회 24GB 풀스택 근거로 사용하지 않는다.

<a id="next"></a>

## 남은 작업

1. [활성 우선순위](../ACTIVE_PRIORITIES.md#perception-latency)에 따라 새 계측 bag이 오면 단계별 지연·실제 consumed path·YOLO 동시 부하를 먼저 분석한다. 추가 live 시험은 자동 재개하지 않는다.
2. E12 최종 backup의 COMPLETE·metadata·전체 readback 상태와 팀 PC 가속기 준비·실제 LAN 실행은 각각 확인할 후속이다.
3. 초기/재취득 차량 종속도·로터리 Top-K 분기와 원거리 최초 취득 실패를 구분해 검토한다. threshold 하향·긴 기억·라벨 삭제·강제 Vehicle 변환을 자동 적용하지 않는다.
4. 낮은 score EKF 갱신·lane history·shape/class 후보·mixed/native 6초는 시험용으로 보존한다. 판단 정책 안정화와 명시적 승격 전에는 운영값을 바꾸지 않는다.

[개선 계획](evidence/261013_BASE5_RANGE_AND_TRACKING_IMPROVEMENT_PLAN.md)은 연구 순서이며 현재값을 덮어쓰지 않는다.
정리 전 세부 이력과 당시 ‘E24 유지’ 결정은 [2026-10-24 원문](legacy/context_261024/CURRENT.md)에 보존했다.
