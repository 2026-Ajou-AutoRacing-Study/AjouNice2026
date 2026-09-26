# Competition Localization 현재 상태

상태: **CURRENT**
마지막 갱신: 2026-09-26

## 한 줄 결론

현재 production은 GNSS 양자화에 맞춘 Q/R과 Local EKF 기반 Smooth controller
state를 함께 사용한다. Planning과 TF는 Global state를 유지하고 controller만
translation/yaw tau `0.5/0.5 s`의 연속적인 state를 사용한다. GPS 음영 중에도 마지막
유효 z를 유지하며 planar state를 계속 발행한다.

## Production 계약

| 항목 | MR·실대회 | MRP |
|---|---|---|
| alias | `localization` | `localization_mrp` |
| motion source | `/Competition_topic` | `/Ego_topic` |
| heading correction | `-1.347 deg` | `0 deg` (현재 설정; MRP 시나리오·지도 기준 독립 검증은 아직 없음) |
| initial pose | 개발 기본값: 첫 `/Ego_topic` **위치** + `/Competition_topic` **heading**. 대회 후보: 공식 START 위치 + `/Competition_topic` heading | 첫 `/Ego_topic` 위치·heading, 보정 `0 deg` |
| GNSS horizontal sigma | `0.10 m` | `0.10 m` |
| Global EKF Qxy | `0.0005` | `0.0005` |
| planning/TF state | `/localization/kinematic_state` | 동일 |
| controller-facing state | `/localization/control_kinematic_state` | 동일 |
| control translation/yaw tau | `0.5 / 0.5 s` | 동일 |

공통 계약:

- local/global EKF `two_d_mode=true`.
- `predict_to_current_time=true`.
- `publish_sensor_tf=false`.
- UDP는 ROS host receipt-time 정책을 유지한다.
- localization provider는 한 번에 하나만 실행한다.
- 대회에서 `/Ego_topic`과 `/Object_topic`을 기대하지 않는다.
- MR 개발용 `localization`은 초기 위치에만 `/Ego_topic`을 쓴다. 초기 yaw와
  연속 GT yaw는 모두 `/Competition_topic.heading - 1.347 deg`다.
- 대회용 고정 위치 후보는 `localization initial_position_source:=official_start`로
  명시한다. 위치는 v85 공식 규칙 파일의 `start_finish.xyz`를 `morai_map -> map`
  변환하고 yaw는 동일한 Competition heading을 쓴다. 공식 `start_finish.yaw_deg`는
  입력 yaw가 아니라 시작 정합 검증 기준이다. 보정된 yaw가 기준과 `1 deg` 초과로
  다르면 초기화를 거부한다. GNSS map 위치가 공식 START에서 `3 m` 이내이고
  Competition heading과 GNSS stamp 차이가 `0.5 s` 이내인 것도 확인한다.
  MR 로컬 시작점 대조는 통과했으나 대회 LAN 검증 전까지 기본값이 아니다.
- 공식 고정 모드에서는 `/Ego_topic`을 구독하거나 기다리지 않는다. 사고 후
  체크포인트 복귀를 START 초기값으로 반복 처리하지 않는다.

## 공식 START MR 로컬 검증 (2026-09-26)

`06023573` 코드 기준으로 MR에 공식 START 위치·yaw를 지정한 로컬 전용
`0_official_start_localization_probe.json`을 로드하고
`localization initial_position_source:=official_start`를 실행했다. 시나리오 파일은
MORAI SaveFile에만 있으며 Git 전달물이 아니다. 공식 기준은 v85 규칙 파일의
`start_finish`다.

| 항목 | 관측값 |
|---|---:|
| 공식 START morai_map 좌표 | `(-131.689798, -428.331023, 28.543960) m` |
| 공식 START yaw | `61.298126 deg` |
| `/Competition_topic` raw heading | `62.627533 deg` |
| raw heading `- 1.347 deg` | `61.280533 deg` |
| EKF 초기 / 현재 yaw | `61.280762 / 61.280532 deg` |
| EKF 초기 map XY / 공식 START 변환 map XY | `(-0.130667, 117.580) / (-0.130667, 117.580) m` |

초기화는 `/Ego_topic` 없이 완료됐고, 보정된 yaw와 공식 기준 차이는
`0.017593 deg`다. 이는 **MR 로컬 출발 정합 통과**이지 LAN-to-LAN, 전체 주행,
checkpoint 복귀 검증은 아니다. 이 한 표본만으로 `-1.347 deg`를 재조정하지 않는다.
공식 시나리오 로드 직후 z 변화에 따른 차량 자세 안정화가 차이에 기여했을 가능성은
있지만 측정된 원인으로 단정하지 않는다. 시작 직후 Smooth가
`translation_discontinuity` reset을 한 번 기록했으며 반복 여부·주행 영향은 이
시험에서 평가하지 않았다.

MRP의 `0 deg`는 현재 alias 설정과 기존 MRP replay 비교 계약이다. 공식
시나리오 yaw와 지도 heading을 독립 대조한 결과가 아니므로 MR의
`-1.347 deg`와 우열·정합성을 이 결과에서 추론하지 않는다.

## 현재 데이터 흐름

```text
/Competition_topic heading/vx ----> GT motion adapter ----+
/sensor/imu yaw-rate/ax ----------> covariance adapter ---+--> Local EKF
                                                          +--> Global EKF
/sensor/gps quantized x/y --------> GNSS odometry --------+

Local EKF  : odom -> base_link, GNSS가 직접 들어가지 않는 smooth motion
Global EKF : map pose, GNSS x/y correction 포함
Publisher  : Global pose + 최신/마지막 유효 GNSS z -> /localization/kinematic_state
Smooth     : 저역통과한 map -> odom 정합 x 최신 Local pose
             -> /localization/control_kinematic_state
Planning/TF: /localization/kinematic_state
Controller : /localization/control_kinematic_state
```

## 해결된 범위

1. MORAI GNSS는 무잡음이어도 동서 약 `14.8 cm`, 남북 약 `18.5 cm`로 양자화됨을
   확인했다.
2. 기존 sigma `0.05 m`, Qxy `0.05`가 cell transition을 과도하게 따라가는 주원인임을
   확인했다.
3. sigma `0.10 m`, Qxy `0.0005`는 동일 raw bag replay에서 횡 frame-jump p95를
   `5.432 -> 1.444 cm`, 종 frame-jump p95를 `7.234 -> 2.416 cm`로 줄였다.
4. 경량 폐루프에서 production Q/R은 localization `50 Hz`, 횡 frame-jump p95
   `1.31 cm`, steering-rate p95 `0.54 rad/s` 수준을 재현했다.
5. 로컬 full-stack의 velocity 적분 불일치는 production `velocity.x` bias가 아니라
   MORAI 비실시간율과 receipt wall-time 혼합 artifact로 판정했다.
6. GPS 음영 `5.64 s`, `11.01 s` replay에서도 마지막 유효 z를 유지하면서
   `/localization/kinematic_state`를 각각 `50.00 Hz`, `50.01 Hz`로 연속 발행했다.
7. 현재 `/Ego_topic` 임시 초기값 계약으로 수행한 Competition 전체 경로 replay에서
   `49.71 Hz` state와 기존 정확도 수준을 유지했다.
8. 동일 actor-free 전체 경로 폐루프 A/B에서 Smooth state는 완주 시간 손실 없이
   steering-rate 평균을 `15.4%`, 조향 반전 빈도를 `23.8%` 줄였다.
9. 같은 A/B에서 controller-facing XY p50은 `2.44 -> 1.66 cm`, 횡오차 p50은
   `1.77 -> 0.88 cm`로 감소했다.
10. 약 `7.25 s` GPS 음영 재획득 직후 첫 1초의 조향 변화율 p95는
    `0.698 -> 0.221 rad/s`로 `68.3%` 감소했다.

## 남은 위험과 우선순위

### 해결. GPS 음영 중 최종 state 연속성

`morai_gps_to_local_odometry.py`는 `(0,0)` GNSS를 거부한다. 최초 유효 z를 받은 뒤
GNSS가 stale해지면 publisher는 마지막 유효 z만 유지하고 Global EKF의 x/y/yaw/twist를
계속 발행한다. 유효 GNSS가 돌아오면 z hold를 해제한다. 최초 z조차 없는 startup은
기존처럼 준비되지 않은 상태로 판단한다.

### P0-A. GPS 재획득과 checkpoint 순간이동 구분

현재는 음영 종료 첫 GNSS를 일반 update로 즉시 반영한다. 다음 기능은 없다.

- reacquisition 상태기계.
- 복귀 직후 covariance inflation.
- correction step/rate limit.
- 정상 복귀와 checkpoint teleport 구분.

현재 low-Q replay에서는 재획득 직후 폭발적 correction이 관찰되지 않아 임의의
covariance inflation은 넣지 않았다. GPS 음영 미션 폐루프와 checkpoint teleport를
분리해 추가 검증한다.

### 해결. Controller의 Global correction 직접 소비

Local EKF의 연속성을 controller에 전달하는 전용 map-frame state를 사용한다. Global/Local
pose에서 `map -> odom` 정합을 구해 그 correction만 저역통과한 뒤 현재 Local pose와
합성한다. planning과 TF는 기존 Global state를 유지한다.

offline/open-loop 검증 뒤 동일 전체 경로 폐루프 A/B를 수행했다. `tau=0.5/0.5 s`
후보는 gate-to-gate 완주 시간 차이 `0.28%` 이내에서 steering-rate 평균을 `15.4%`,
sign flip을 `23.8%` 줄였다. GPS 재획득 직후 조향 변화율 p95도 `68.3%` 감소했다.
따라서 production alias는 `publish_control_state=true`, controller 기본 입력은
`/localization/control_kinematic_state`로 승격했다. 근거는
`260925_SMOOTH_CONTROL_STATE_CLOSED_LOOP_EVIDENCE.md`에 있다.

후속 quantization-aware 후보는 GNSS를 점이 아닌 cell interval로 처리하고, 그 결과의
횡 displacement만 controller state에 사용하면서 종 위치와 yaw는 Global EKF를
보존한다. 52초/242초 1× replay에서 횡 jump p95가 각각 `1.263 -> 0.213 cm`,
`1.307 -> 0.259 cm`로 줄고 종·yaw는 유지됐다. 기존 Global-LPF보다 더 강한 후보지만
아직 폐루프가 아니므로 `use_quantized_gnss_control_state=false`가 기본이다. 근거는
`260925_QUANTIZED_GNSS_CONTROL_STATE_EVIDENCE.md`에 있다.

세 raw bag에서 위·경도 격자 `1/600000 deg`와 UTM cell 축 `-1.348 deg`를 독립적으로
재확인해 후보의 기존 axis-aligned cell을 바로잡았다. 변경 후 1× replay도 49.99 Hz와
종 위치 보존을 유지했다. 이는 후보 내부 좌표계 정합 수정이며 production 승격은 아니다.

두 개의 합성 8초 GNSS 음영 1× filter replay도 수행했다. stale GNSS에서는 Global로
fallback하고 재획득 후 1초 grace를 둔 결과, 음영과 복귀 첫 1초의 후보가 Global보다
나빠지지 않았으며 복귀 1~4초 횡 p95는 각각 `10.78 -> 4.74 cm`,
`8.87 -> 2.28 cm`로 감소했다. 이는 replay gate 통과이며 실제 폐루프 승격은 아니다.

Planning/TF는 Global state를 유지하고 두 controller node만
`controller_kinematic_state_topic`으로 함께 전환한다. 불연속 판정은
alignment parameter가 아니라 현재 차량의 map pose 차이로 수행한다. `0.5 m / 20 deg`
기준은 정상 replay에서 reset 0회였고, 인위적 8초 음영 두 경우의 `0.53 / 1.38 m`
재획득 correction에서 각각 1회 reset돼 planning과 controller state의 장시간 frame
불일치를 막았다. checkpoint 실측이 없어 아직 production 계약은 아니다.

승격 전 동일 기록 trajectory로 controller를 다시 실행한 open-loop A/B에서 `tau=0.5 s`
후보는 52초/242초 bag 모두 steering-rate p95를 약 4%, sign flip을 약 15~17%
줄였다. 장주행 controller lateral error p50/p95도 `3.95/27.65 -> 2.68/24.14 cm`로
개선됐고, 이후 전체 경로 폐루프도 통과했다.

Quantization-aware 후보도 같은 controller A/B에서 Global보다 개선됐지만, 장주행
sign flip `0.2925/s`, lateral error `2.84/25.08 cm`로 Smooth의 `0.2830/s`,
`2.68/24.14 cm`보다 근소하게 나빴다. 따라서 더 낮은 pose jump만으로 순위를
뒤집지 않고 Smooth 0.5초를 폐루프 1순위로 유지한다.

후속 Ego oracle 감사에서 frame별 종방향 pose 증분이 자체 velocity/heading 예측과
p95 약 `10~12 cm` 불일치함을 확인했다. 전체 거리 적분은 `0.01%` 이내로 맞으므로
속도 bias는 아니며 pose 갱신 시간 불균일이다. 이에 따라 종 frame jump는 진단값으로만
남기고, continuity 승격 판단에는 횡 frame jump와 controller 지표를 사용한다.

`/localization/odometry/local`만 controller에 단순 remap하면 map trajectory와 frame이
달라지므로 금지한다.

### P0-C. Local/Global TF co-time 정합

기존 final-state publisher는 Global state와 약 `3~17 ms` 오래된 최신 Local TF를
합성해 고속에서 TF closure가 p95 `4.5~28.9 cm` 어긋났다. 현재 코드는 Local pose를
Global timestamp까지 외삽해 `map -> odom`을 만들며, production 짧은 bag 1× replay의
closure p95는 `17.40 -> 0.071 cm`로 감소했다. EKF state 자체는 변경하지 않는다.
새 bag에서 50 Hz와 `tf_contract`를 재확인한다.

### P1. 공식 start pose 교체와 checkpoint reset

Competition Status에는 global position이 없다. MR 개발 기본값의 첫 `/Ego_topic`
**위치**는 임의 시나리오를 지원하기 위한 초기값 전용이다. 공식 START 좌표가
확인되어 이를 선택하는 launch 인자를 마련했다. MR 로컬 공식 START 위치·yaw 대조는
통과했지만 현장 LAN 시작 검증 전에는 대회 기본값으로 승격하지 않는다. MRP는 별도
Ego 계약을 유지한다.
사고 후 checkpoint 순간이동 후보를 감지하는 가드를 추가했다. 정상 bag 두 개에서
5 m 초과 innovation은 0건이었고 합성 20 m 이동은 3 sample 뒤 감지했지만, 실제
checkpoint bag과 폐루프가 없어 기본 OFF·monitor-only다. active 후보는 Global EKF만
reset하고 Local odom 연속성을 유지한다. 자동 reset production 계약은 아직 없다.
Quantization-aware control state와 active guard를 결합한 합성 20 m 전체 replay에서는
guard reset이 1회였고 reset 순간 Global–control 차이가 0.0 m였다. 따라서 두 후보의
기계적 결합은 통과했지만, 실제 checkpoint 폐루프 gate는 여전히 남아 있다.
GNSS gap `0.5 s` 초과 또는 timestamp 역행 시 relocation 연속 count를 초기화해 음영
전후 표본이 하나의 3-sample 후보로 잘못 이어지는 상태관리 결함은 제거했다.

### 해결. Competition yaw-rate A/B

현재는 Competition heading/vx와 IMU yaw-rate/ax를 사용한다. Competition
UDP 패킷에는 `angular_velocity.z`가 있지만 기존 `EgoVehicleStatus` ROS type에는 해당
필드가 없어 receiver가 값을 버리고 있었다. 기존 message MD5와 bag 호환성을 유지하기
위해 `/Competition_topic`은 바꾸지 않고 `/competition/raw_twist`에 전체 linear/angular
twist를 추가 발행한다.

260925 전체 경로 bag의 새 토픽으로 확인한 결과 native angular.z는 clockwise
degrees/s이며 ROS yaw-rate에는 `-pi/180` 변환이 필요했다. 변환 후 IMU와의 상관계수는
`0.99995` 이상이었다. IMU를 대체한 1× 전체 EKF replay에서도 횡·종·yaw 성능은
사실상 동등해 production 입력 변경을 기각했다. 두 신호는 같은 simulator state에서
파생되므로 동시에 독립 측정처럼 fuse하지 않는다.

기존 입력 ablation에서는 IMU yaw-rate 제거가 yaw p95를 약 3.6~3.9배 악화했고,
ax 제거는 장주행 종오차를 조금 악화하면서 유의미한 횡 이득이 없었다. Competition
yaw-rate 대체도 의미 있는 우위가 없어 현재 IMU yaw-rate와 ax를 유지한다.

추가 입력 계약 감사에서 최신 receipt-time bag의 IMU yaw-rate와 Competition heading
미분은 상관계수 `0.99925`, p95 오차 `0.0140 rad/s`로 정합했다. IMU TF의 translation에
따른 lever-arm 가속도는 `robot_localization`이 이미 target frame 원점으로 보정하므로
adapter에서 중복 보정하지 않는다. 자세한 근거는
`260925_FUSION_INPUT_CONTRACT_AUDIT.md`와
`260925_COMPETITION_YAW_RATE_AND_YAW_SMOOTHING_EVIDENCE.md`에 있다.

## Production 변경 gate

```text
동일 raw bag offline A/B
-> od_gt 경량 폐루프
-> GPS 음영 진입·유지·복귀
-> LAN-to-LAN 동일 controller
-> production alias/config 변경
```

Offline 위치 오차 하나만 좋아졌다는 이유로 기본값을 바꾸지 않는다. 횡·종 오차,
frame jump, yaw, state rate, steering-rate와 sign flip을 함께 본다.

## 변경하지 않을 항목

- heading correction `-1.347 deg`.
- production Q/R을 추가 sweep의 출발점 없이 임의 수정하는 것.
- UDP receipt-time 정책.
- `velocity.x` 임의 scale.
- local state만 controller에 바로 remap하는 것.
- perception P8/P9 결과를 localization 보정 근거로 사용하는 것.
