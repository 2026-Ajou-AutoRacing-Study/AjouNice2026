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
| S1 | S0 + train-only x/y flip, ±45도 rotation, 0.95~1.05 scaling. | S0와 동일 | 40 epoch | E38 | Offline·online Vehicle 성능과 hard-negative 일반화가 개선됨. 현재 최상위 후보지만 front-back flip의 yaw ambiguity 때문에 조건부 채택. |
| S1b | S1에서 left-right flip만 유지하고 front-back flip 제거. Rotation·scaling은 유지. | S0/S1과 동일 | 예정 | 예정 | 다음 정식 통제 실험. |

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
  - x/y 양축 flip 중 front-back flip이 direction supervision을 흔든 가능성을
    S1b로 분리해야 함.

## 다음 정식 실험.

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

## 보류 중인 후속 계열.

- `T1`은 아직 번호를 확정하지 않음.
  - S1 계열로 online geometry baseline을 먼저 확정한 뒤 temporal fusion 구조를
    재설계할 때 등록함.
  - 단순 Ego-only sweep stacking과 기존 velocity weight만 반복하지 않음.
- Vehicle velocity loss weight·moving-balanced supervision 실험도 S1b 이후로
  보류함.
  - single-frame production geometry를 확정하기 전에는 정식 T-series 번호를
    소비하지 않음.
