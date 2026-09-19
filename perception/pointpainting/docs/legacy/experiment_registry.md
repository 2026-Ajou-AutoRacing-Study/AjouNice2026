# PointPainting 정식 실험 계보.

## 문서 목적.

- 사람이 PointPainting 정식 학습의 계보, 변경 변수, 대표 checkpoint와 판정을
  빠르게 확인하기 위한 문서임.
- dataloader 확인, 1 epoch, 3 epoch, W&B 연결 확인 등의 smoke run은 기록하지 않음.
- 충분한 epoch를 학습하고 offline validation과 online rosbag 중 하나 이상으로
  비교 판정을 내린 실험만 등록함.
- 세부 구현 과정과 전체 수치는
  `base2_temporal3_velocity_d4_readiness.md`에 보존함.

## 이름 규칙.

- `Tn`은 temporal/sweep 입력 또는 detector velocity regression을 주 가설로 하는
  정식 실험임.
- `Sn`은 single-frame detector의 구조, 데이터, 증강을 주 가설로 하는 정식
  실험임.
- `S1b`처럼 영문 suffix가 붙으면 바로 앞 실험에서 한 변수만 바꾼 통제
  ablation임.
- `E##`는 실험 ID가 아니라 해당 실험에서 최종 비교에 사용한 checkpoint
  epoch임.
  - 예: `S1 E38`은 S1 실험의 epoch 38 checkpoint임.
- `E65`는 이 규칙을 만들기 전부터 사용한 legacy production 별칭임.
  - 정식 표기는 `Legacy E65`로 적음.
- 학습 시작 전에 새 실험 ID를 이 문서의 예정 실험에 등록하고, 평가가 끝나면
  결과와 판정을 갱신함.

## 정식 실험 요약.

| ID | 입력·주요 변경 | 학습 데이터 | 정식 학습 | 대표 checkpoint | 판정 |
|---|---|---|---:|---|---|
| Legacy E65 | Single-frame 9-feature PointPainting. 기존 정적 1,500-frame 중심 학습. | Sunny 13시 static 1,500 | 80 epoch | E65 | 이전 production baseline. |
| T0 | Ego-aligned 3-sweep 10-feature, time lag, 9D box와 Vehicle velocity regression. 기하 증강 없음. | Base2 temporal Train 5,509 / Val 1,181 | 40 epoch | E35 | Offline velocity 학습은 성립했지만 online geometry와 velocity가 모두 나빠 production 기각. |
| S0 | 현재 sweep만 사용하는 single-frame 9-feature 7D box. T0와 같은 target frame·split, 기하 증강 없음. | Base2 single-frame Train 5,509 / Val 1,181 | 40 epoch | E28 | Temporal stacking 원인 분리용 통제군. Online에서 E65를 안정적으로 넘지 못해 production 기각. |
| S1 | S0 + train-only x/y flip, ±45도 rotation, 0.95~1.05 scaling. | S0와 동일 | 40 epoch | E38 | Offline·online Vehicle 성능과 hard-negative 일반화가 개선됨. 현재 Vehicle production 비교 기준. |
| S1b | S1에서 left-right flip만 유지하고 front-back flip 제거. Rotation·scaling은 유지. | S0/S1과 동일 | 40 epoch | E30 | Offline은 S1과 동률이나 두 online rosbag과 downstream이 하락해 기각. |
| S1g | S1 + class별 GT database sampling 10개. 9-feature 7D single-frame. | S0/S1과 동일 | 40 epoch | E37 | Val-derived score 0.40에서 S1의 고주로 성능을 유지하고 로터리·Map-CV를 개선한 새 production 후보. |

## S1g 통제 조건.

### S1g: S1 + GT database sampling.

- 가설은 다음과 같음.
  - Base2의 empty/hard-negative 비중 때문에 실제 객체 supervision 밀도가 낮음.
  - 팀원 legacy 설정의 class별 GT database sampling을 추가하면 S1의 기하 일반화는
    유지하면서 Vehicle·Pedestrian·Obstacle 학습 밀도를 높일 수 있음.
- 첫 통제 실험은 S1과 동일한 split, 9-feature, 7D head, batch size 4, 40 epoch,
  OneCycle LR을 유지함.
- 유일한 변경 변수는 다음 GT database sampling임.
  - Pedestrian 최대 10개.
  - Vehicle 최대 10개.
  - Obstacle 최대 10개.
- Offline Val만으로 채택하지 않고 동일 신로터리·고주로 rosbag의 Full-360 및
  interaction-critical OD, Tracking, Map-CV를 함께 비교함.
- S1g가 S1보다 좋아질 때만 80 epoch schedule 연장 실험을 별도로 수행한다는
  조건이었으며, E37에서 후보 조건을 충족함.

## 주요 결과.

### Legacy E65.

- 기존 production 비교 기준임.
- 동일 frame coverage의 online Vehicle 결과는 다음과 같음.
  - 로터리 center mAP `0.6566`, F1@1m `0.8744`.
  - 고주로 center mAP `0.6571`, F1@1m `0.8010`.

### T0.

- Base2 Val에서 center mAP best는 E25의 `0.9283`임.
- Vehicle mAVE best는 E33의 `0.7713 m/s`임.
- detection과 velocity의 대표 절충 checkpoint로 E35를 사용함.
  - Center mAP `0.9161`.
  - Vehicle mATE `0.1646 m`.
  - Vehicle mAVE `0.7754 m/s`.
- E35 online Vehicle center mAP는 로터리 `0.428`, 고주로 `0.431`임.
- Detector velocity를 tracker에 융합해도 Map-CV가 악화되어 production에서
  제외함.

### S0.

- T0와 동일한 target frame을 single-frame으로 학습해 temporal stacking의 영향을
  분리한 실험임.
- 최종 비교 checkpoint는 E28임.
- 동일 controlled online 평가에서 Vehicle center mAP는 로터리 `0.5275`, 고주로
  `0.7123`임.
- 로터리의 좌측·고회전 Vehicle domain gap이 남아 S1 기하 증강 실험으로
  이어짐.

### S1.

- 최종 비교 checkpoint는 E38임.
- Vehicle production 비교 기준으로 고정한 release는 다음과 같음.
  - 경로: `artifacts/detection3d/pointpainting/releases/260825_base2_s1_singleframe9_geom_aug_e38`.
  - Checkpoint SHA256: `939076e933ac537beaec7183f002b10f10af7e20e92a634da00424ae6cee669c`.
  - 7D detector이므로 detector velocity 발행은 비활성화함.
- Base2 Val 결과는 다음과 같음.
  - 전체 center mAP `0.9417`.
  - Vehicle center AP `0.9767`, mATE `0.0962 m`.
- 동일 controlled online 평가 결과는 다음과 같음.
  - 로터리 center mAP `0.6878`, F1@1m `0.8836`.
  - 고주로 center mAP `0.7717`, F1@1m `0.9403`.
- Legacy E65 대비 downstream 결과는 다음과 같음.
  - 로터리 Map-CV 3초 minFDE-K mean `2.958 -> 1.802 m`.
  - 고주로 Tracking F1 `0.862 -> 0.953`.
  - 고주로 Map-CV 3초 minFDE-K p50 `1.300 -> 1.056 m`.
- 남은 문제는 다음과 같음.
  - S0 대비 180도 yaw 반전이 증가함.
  - 로터리에서는 track 위치·속도 정확도가 좋아졌지만 Tracking recall이
    `89.8 -> 86.4%`로 낮아짐.
  - S1b에서 front-back flip을 제거했지만 전체 online 성능이 하락했으므로,
    단순 flip 제거만으로 해결할 수 있는 문제는 아님.

### S1b.

- 최종 비교 checkpoint는 E30임.
- Base2 Val center mAP는 `0.9415`로 S1 E38의 `0.9417`과 사실상 동률임.
- 동일 controlled online 평가 결과는 다음과 같음.
  - 로터리 center mAP `0.6069`, F1@1m `0.7886`.
  - 고주로 center mAP `0.7100`, F1@1m `0.8382`.
- 고주로 mAOE는 `0.063 rad`로 개선됐지만 로터리 mAOE는 `1.018 rad`로
  악화됐고, 두 scene의 recall·center 정확도·downstream 성능이 모두 S1보다
  낮음.
- FRONT_BACK flip 제거가 전체 production 성능을 높인다는 가설은 기각함.
- 전체 rosbag 비교표는 `rosbag_vehicle_leaderboard.md`에서 관리함.

### S1g.

- 최종 비교 checkpoint는 E37이며 Base2 Val center mAP는 `0.9618`임.
  - S1 E38의 `0.9417`보다 `+0.0201` 높음.
- 고정 production score `0.10`에서는 다음 문제가 발생함.
  - 로터리 Vehicle center mAP/F1은 `0.7197/0.8878`로 개선함.
  - 고주로 center mAP는 `0.8031`로 개선하지만 F1은 `0.8822`로 하락함.
  - 고주로 Tracking F1도 `0.8618`로 하락해 그대로는 production 불가임.
- Base2 Val에서 Vehicle·Obstacle best score가 `0.407/0.409`였으므로 장면별
  tuning 없이 전역 score `0.40`을 calibration 후보로 고정함.
- score `0.40`의 동일 controlled online 결과는 다음과 같음.
  - 로터리 Tracking F1/MOTA `0.9596/0.9223`, center p50 `0.464 m`.
  - 고주로 Tracking F1/MOTA `0.9526/0.9049`, center p50 `0.441 m`.
  - 로터리 3초 minFDE-K p50/p90 `1.096/2.875 m`.
  - 고주로 3초 minFDE-K p50/p90 `0.956/2.410 m`.
- S1 대비 interaction-critical 결과는 다음과 같음.
  - 로터리 OD Recall@1m `89.2% -> 90.5%`.
  - 고주로 OD Recall@1m `97.7% -> 97.3%`로 사실상 동률임.
  - 고주로 conflict-critical tracking coverage는 둘 다 `100%`이며 3초 CV
    minFDE-K p50은 `1.345 -> 0.757 m`로 감소함.
- class별 production 검증 결과는
  `s1g_class_production_validation.md`에 기록함.
  - Vehicle에는 score `0.40`이 적합함.
  - Pedestrian은 낮은 score에서 큰 recall gain이 있지만 Vehicle scene class
    confusion이 남음.
  - Obstacle은 AP가 약 2배 상승했지만 scene별 최적 score와 FP가 불안정함.
- 따라서 전체 3-class production 승격은 보류함.
  - 현재 default는 S1을 유지함.
  - S1g E37은 class-calibration 실험의 기준 checkpoint로 유지함.
  - score `0.10` 결과와 calibrated 결과를 같은 행으로 섞지 않음.

### S1h.

- S1g의 class calibration 원인을 분리하는 후속 controlled ablation임.
- S1g와 동일하게 유지하는 조건은 다음과 같음.
  - Base2 single-frame 9-feature.
  - left-right/front-back flip, rotation, scaling.
  - 모델·anchor·optimizer·40 epoch·batch size 4·seed 0.
  - Pedestrian/Vehicle GT sampling 각각 10개.
- 유일한 변경은 Obstacle GT sampling을 `10 -> 3`으로 줄이는 것임.
- 가설은 다음과 같음.
  - S1g의 Vehicle 좌측·planning-relevant gain과 Pedestrian gain은 유지함.
  - Obstacle 저신뢰 후보·Vehicle scene FP와 scene별 threshold 편차를 줄임.
- 채택 조건은 `s1g_class_production_validation.md`의 S1h 기준을 사용함.
- 최종 비교 checkpoint는 E32이며 Base2 Val center mAP는 `0.9533`임.
  - S1g E37의 `0.9618`보다 `-0.0085` 낮음.
- online 결과는 다음과 같음.
  - 고주로 score 0.40 Obstacle FP는 `79 -> 21`로 감소함.
  - Obstacle3 mAP/best F1은 `0.157/0.336 -> 0.169/0.414`로 개선함.
  - Pedestrian crossing mAP는 `0.442 -> 0.381`로 하락함.
  - Pedestrian distance mAP는 `0.642 -> 0.444`로 하락함.
  - 로터리 Tracking F1은 `0.960 -> 0.894`로 하락함.
  - 고주로 Tracking F1은 `0.953 -> 0.962`로 소폭 개선함.
- 가설 판정은 부분 채택임.
  - Obstacle FP 감소 가설은 확인함.
  - Vehicle·Pedestrian gain 유지 가설은 기각함.
- 전체 production 후보로는 기각하며 S1g E37을 후속 calibration 기준으로 유지함.

## 완료된 정식 실험 설계.

### S1b: left-right-only geometric augmentation.

- 가설은 다음과 같음.
  - S1의 rotation·scaling과 측방 일반화 효과는 유지함.
  - left-right mirror는 유지함.
  - front-back mirror를 제거해 앞뒤 yaw ambiguity를 줄임.
- OpenPCDet 좌표계에서 `x`는 전방, `y`는 좌측임.
  - `random_flip_along_x`는 x축을 기준으로 y 부호를 바꾸므로 left-right
    mirror임.
  - `random_flip_along_y`는 y축을 기준으로 x 부호를 바꾸므로 front-back
    mirror임.
  - 따라서 S1b의 `ALONG_AXIS_LIST`는 `[x]`만 사용함.
- S1과 동일하게 유지할 조건은 다음과 같음.
  - Base2 single-frame9 Train 5,509 / Val 1,181 split.
  - 9-feature, 7D box, anchor, optimizer, batch size, LR schedule, 40 epoch.
  - rotation `-45~+45 deg`, scaling `0.95~1.05`.
- 주 판단 지표는 다음과 같음.
  - Offline Vehicle center AP·mATE·mASE·raw mAOE.
  - Online 로터리·고주로 center mAP와 F1@1m.
  - raw yaw 90도 초과 반전율과 pi-symmetric yaw error.
  - 로터리 Tracking recall·F1·ID switch.
  - CV 안정 구간 Map-CV minFDE-K 1·2·3초 mean, p50, p90, MR@2m, coverage.
- 채택 기준은 S1의 online center mAP와 Map-CV 개선을 실질적으로 유지하면서,
  yaw 반전 또는 로터리 Tracking recall 중 하나 이상을 개선하는 것임.
- S1b가 yaw를 개선하지 못하면 flip보다 anchor/direction head 또는 detector 출력
  orientation 정규화가 병목이라고 판단함.

## 완료된 Base3 후속 계열.

### B3S-PP1-GTDB E34.

- B3S-PP0 이후의 Base3 single-frame class-centric 보강 실험임.
- 자연 Base3 train split은 유지하고 GT database sampling만 추가함.
- 목표 개수는 Vehicle/Pedestrian/Obstacle `20/10/4`임.
- Pedestrian GT DB 후보 풀은 moving/standing `60/40`으로 구성함.
  - moving은 Batch 008의 walking/running/activation-transition single-frame crop임.
- 최소 point는 Vehicle/Pedestrian/Obstacle `5/5/8`이고
  `LIMIT_WHOLE_SCENE=true`를 사용함.
- B3S-PP0와 모델, anchor, loss, geometric augmentation, batch size 4,
  OneCycle LR `0.003`, 40 epoch와 seed를 동일하게 유지함.
- 정식 이름은 `B3S-PP1-GTDB-V20P10O4`임.
- 주 판단은 Base3 Val/Test뿐 아니라 신로터리·고주로·횡단 Pedestrian·허용
  Obstacle rosbag의 class별 AP, recall, box quality와 downstream tracking임.
- E34 Val/Test center mAP는 `0.9786/0.9809`임.
- PP0 대비 7개 rosbag class mAP는 로터리, 고주로1, Vehicle 30 m, 횡단
  Pedestrian, 거리별 Pedestrian에서 개선됐고 고주로 합류와 CargoBox에서 하락함.
- 로터리 Tracking AVE mean은 `1.086 -> 0.809 m/s`, CV-stable 3초 minFDE-K
  p50/p90은 `1.660/4.564 -> 1.243/3.402 m`로 개선됨.
- 고주로1 CV-stable prediction은 소폭 하락함.
- Base3 계열 보존 checkpoint로 채택하되 전체 production은 S1g score `0.40`을 유지함.

## 보류 중인 후속 계열.

- `T1`은 아직 번호를 확정하지 않음.
  - S1 계열로 online geometry baseline을 먼저 확정한 뒤 temporal fusion 구조를
    재설계할 때 등록함.
  - 단순 Ego-only sweep stacking과 기존 velocity weight만 반복하지 않음.
- Vehicle velocity loss weight·moving-balanced supervision 실험도 S1b 이후로
  보류함.
  - single-frame production geometry를 확정하기 전에는 정식 T-series 번호를
    소비하지 않음.
