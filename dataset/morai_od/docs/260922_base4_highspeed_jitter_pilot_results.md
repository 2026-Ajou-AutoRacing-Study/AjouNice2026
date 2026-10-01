# Base4 고속 연속속도 파일럿 결과

> **문서 상태: CURRENT EVIDENCE.** 고속주행도로에서 Ego `80--120 km/h`, NPC
> `25--80 km/h` 연속 속도 분포와 전방/측방/추월 후 후방 전이를 검증한 48-scene
> 파일럿이다. 아직 production corpus나 독립 test set은 아니다.

## 결론

- 자동 수집은 가능하다. 최종적으로 `48/48 scene`, `768/768 trigger frame`을 확보했다.
- 실제 Ego 속도는 `77.96--120.45 km/h`, NPC 속도는 `25.61--79.23 km/h`였다.
- raw와 Ego scan-end deskew 파생 dataset을 같은 scene split로 만들었고 두 dataset 모두
  info validator와 temporal quality audit를 통과했다.
- 다음 실험은 이 작은 dataset으로 V1/T1 학습 파이프라인과 고속 velocity 일반화만
  확인한다. 같은 생성기 val 수치를 production 성능으로 해석하지 않는다.

## 생성·주행 계약

`generate_base4_temporal_highway.py --profile high-speed-pilot`이 다음을 적용한다.

- post-merge 직선 고주로만 사용하고 합류 장면은 포함하지 않는다.
- Ego 목표속도는 `80--120 km/h`, NPC 목표속도는 `25--80 km/h` stratified continuous
  schedule이다. 두 schedule은 독립적으로 섞는다.
- 역할 비율은 `front_overtake : side_passby : rear_receding = 2:1:1`이다.
- scene당 16 frame, nominal LiDAR `12 Hz`, 실제 trigger window 약 `1.7--2.3 s`다.
- 학습 유효 gate는 종방향 `[-30,30] m`, radial `<=32 m`, 횡방향 `<=8 m`다.
- 인접 차선 중심 간격을 고려해 최소 중심거리는 `3.0 m`다.
- 최소 6개 연속 frame에서 actor gate를 만족해야 한다. 이후 Ego가 NPC를 추월해 target이
  30 m 밖으로 사라지는 것은 temporal disappearance로 보존한다.
- 고속 순항 scene은 저장 초기속도를 목표속도로 둔다. 외부제어 연결 전 drift gate는 이
  직선 profile에서만 끄며 합류/곡선 profile의 `0.75 m` 안전 gate는 유지한다.
- NPC spawn은 예상 capture gate 시각뿐 아니라 실측 `2.1 s` clip의 중앙에서 원하는
  상대 위치가 되도록 상대속도로 위상을 보정한다.

## 실제 수집

최종 생성 tag는 `hs48r5`, seed는 `260922301`이다. `hs48r4` 1회차에서 성공한 34개와
`hs48r5` 재시도 14개를 합쳤다. 재시도는 동일 seed/index라 속도·차선·역할 표본을
대체하며 quota를 추가하지 않는다.

```text
generator source/runtime:
  /home/luv/morai_cap/experiments/260922_base4_highspeed_jitter_pilot/

raw bags:
  rosbags/datasets/base4_temporal/260922_base4_highspeed_jitter_hs48r4/
  rosbags/datasets/base4_temporal/260922_base4_highspeed_jitter_hs48r5_retry/
  rosbags/datasets/base4_temporal/260922_base4_highspeed_jitter_hs48r5_retry2/

raw audit:
  /home/luv/morai_cap/experiments/260922_base4_highspeed_jitter_pilot/audit_highspeed48.json
```

1회차 실패는 `14`개였고 재시도 `11+3`개로 모두 채웠다. 주된 원인은 rosbridge control
service가 간헐적으로 5초 늦게 복구돼 초기속도 차량이 직선 구간을 지나친 것이었다.
데이터 계약 실패가 아니라 ScenarioLoad 재연결 지연이며 재시도 시 재현되지 않았다.

원본 bag sensor rate 범위/중앙값은 다음과 같다.

| topic | min | median | max |
|---|---:|---:|---:|
| LiDAR | `10.86` | `11.87` | `13.26 Hz` |
| front camera | `28.84` | `29.82` | `30.33 Hz` |
| Ego | `44.14` | `46.94` | `48.82 Hz` |
| Object | `43.44` | `46.94` | `48.66 Hz` |

raw audit에서는 bag edge LiDAR 후보 `1446`개 중 camera별 `2`개(`0.138%`)가 nearest
age `100 ms`를 약간 넘겨 strict PASS는 아니었다. 최대값은 `105.7--108.6 ms`다.
Converter는 이 후보를 선택하지 않았고 최종 dataset의 camera age max는 raw
`34.01 ms`, deskew `36.80 ms`다. `/Object_topic`의 약 `1 ms` 역행 4건은 timestamp
sort 후 보간했다.

## 파생 dataset

공통 계약은 MR heading correction `-1.347 deg`, LiDAR 시각 Ego/Object interpolation,
geometric BBox center velocity, `3` causal sweep, `minimum_gt_points=5`, Vehicle positive,
`+-30 m` box range, scene 단위 `38 train / 10 val / 0 test`다.

| dataset | train/val frame | Vehicle GT | 비고 |
|---|---:|---:|---|
| `morai_base4_highspeed_jitter_raw_v1` | `471/106` | `577` | scan 내부 deskew off |
| `morai_base4_highspeed_jitter_ego_deskew_v1` | `463/103` | `566` | GT Ego pose, scan-end SE(2) deskew |

두 temporal audit 모두 valid이며 split leakage, unknown asset, 과속 velocity label,
비정상 sweep transform이 없다. Raw BBox-center velocity consistency는 p50/p95
`0.051/0.310 m/s`, deskew는 `0.054/0.272 m/s`다. Deskew CPU 비용은 mean/p95
`1.57/1.84 ms`, point displacement p95의 scene 평균은 `2.26 m`다.

Raw와 deskew는 별도 입력 계약이다. Deskew checkpoint를 online에 올릴 때만
`lidar_deskew_mode=ego_pose_2d`, `lidar_deskew_reference=end`를 사용한다. 어느 쪽도
학습 결과와 고속 held-out bag 비교 전 production 기본값으로 승격하지 않는다.

## 다음 단계

1. 동일 초기 checkpoint와 optimizer로 raw V1/T1, deskew V1/T1의 짧은 mechanism A/B를
   수행한다.
2. 주 지표는 Vehicle velocity MAE, tracker 2D AVE, Map-CV FDE다. 생성기 내부 val AP만으로
   모델을 선택하지 않는다.
3. 결과가 유효하면 같은 생성기를 확대하되 MiddleSize 중심 차종, 합류 `<=59 km/h`,
   실제 controller 주행 held-out bag은 별도 campaign으로 둔다.
4. rosbridge control service 지연은 데이터 오염이 아니라 자동 재시도 대상으로 처리한다.
