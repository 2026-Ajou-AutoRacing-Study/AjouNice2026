# Base4 LiDAR deskew 및 고주로 제어 안전성 근거

- 상태: **CURRENT EVIDENCE**
- 날짜: 2026-09-20
- 적용 범위: Base4-Temporal 고주로 수집기

## 결론

1. MORAI PointCloud2의 point `time`은 단순 장식 field가 아니다. 정상 scan에서는
   point-time Ego deskew가 정적 구조물의 frame 간 정합을 개선했다.
2. 첫 scan 일부는 point-time span이 `1 s`를 넘는 비정상 값이다. `0.15 s`를 넘는
   scan은 deskew 입력에서 제외해야 한다.
3. scan 내부 deskew는 아직 production 기본값으로 켜지 않는다. raw를 보존하고,
   Base4 converter에서 optional A/B로 구현한 뒤 학습과 online inference에 같은
   계약을 적용한다.
4. 최초 고주로 production `p12v3`는 제어권 인수 전에 움직이는 Ego 때문에 ramp 차선을
   이탈했으므로 중단·격리했다. 해당 데이터는 학습에 사용하지 않는다.
5. Ego 초기속도 `0`, pre-control drift gate, 실제 속도/안정화 capture gate, NPC 만남
   시점 역산을 적용한 `r9/r10` smoke는 merge와 70/100 km/h cruise를 통과했다.

## Deskew P0

재현 도구:

```bash
python3 dataset/morai_od/scripts/analyze_morai_lidar_deskew.py --help
```

결과 파일:

```text
artifacts/datasets/base4_temporal_deskew_p0/
  highway_merge_deskew_ab_stratified40.json
```

40 bag, 1,911 cloud, 1,631 consecutive pair를 사용했다. 차량 actor point와 ground를
제외하고, Ego pose를 보간해 정적 world alignment를 비교했다.

| 항목 | 결과 |
|---|---:|
| point-time span p50 / p95 | `0.08149 / 0.09903 s` |
| 최대 span | `1.39206 s` |
| `0.15 s` 초과 scan | `42 / 1,911` |
| zero-time median alignment p50 | `0.22819 m` |
| point-time deskew median alignment p50 | `0.20640 m` |
| reverse-time median alignment p50 | `0.25280 m` |

정방향 point-time 보정은 zero-time 대비 median p50을 약 `9.5%` 개선했고, 반대 방향
보정은 악화시켰다. 따라서 MORAI scan은 완전한 동시 snapshot으로 간주하면 안 된다.
다만 평균 개선은 `0.36660 -> 0.35950 m`로 작고 outlier scan이 있으므로, 이 결과만으로
production deskew를 강제하지 않는다.

## 차선 이탈 원인과 수정

격리한 `p12v3` 4,656 frame에서 조향 절댓값은 p50 `10.328°`, p95/max `12.605°`였고
`45.2%`가 제한에 포화됐다. 원인은 ramp route가 아니라 다음 순서였다.

1. 시나리오가 저장된 45/50/54 km/h로 즉시 출발한다.
2. ScenarioLoad 후 ROS network와 external-control service가 복구되는 동안 steering command가
   없다.
3. Ego가 곡선 진입부를 직진으로 통과한다.
4. 뒤늦게 controller가 붙어 최대 조향으로 route를 복구하려 한다.

수정 계약:

- Ego load speed `0 km/h`.
- external control 전에 기대 위치에서 `0.75 m` 넘게 이동하면 ScenarioLoad 재시도.
- capture 전에 motion arm, 최소 Ego speed, stable-speed window를 모두 확인.
- post-merge 속도는 우선 `70/85/100 km/h`; `115/120`은 기준선에서 제외.
- NPC가 load 직후부터 이동하는 거리를 Ego 가속시간으로 역산해 spawn 위치를 보정.

## R9 실제 smoke

경로: `/home/luv/morai_cap/260920_b4_r9_smoke`

| 구간 | 완료 | 상대차 거리 | 조향 abs p95 / max | 실패 |
|---|---:|---:|---:|---:|
| merge 50 km/h | 16/16 | `17.6--20.8 m` | `0.282 / 0.284°` | 0 |
| cruise 70 km/h | 16/16 | `29.6--42.3 m` | `0.093 / 0.100°` | 0 |
| cruise 100 km/h | 16/16 | `11.6--39.0 m` | `0.230 / 0.240°` | 0 |

초기 직진 이탈, 조향 포화, collision, actor-gate timeout은 재현되지 않았다.
100 km/h는 stable-speed gate가 약 176 m에서 열리므로 16-frame window를 보존하도록
maximum route motion을 `260 m`로 둔다.

## 후방 접근 차선과 lane change

인접 차선 후방 차량의 velocity 관측은 필요하지만, 이를 곧바로 Ego cut-in과 묶지 않는다.

- 1차 Base4 본분포: Ego는 현재 차선을 유지하고 옆 차선 후방/사선 차량을 관측한다.
- 후속 addon: 실제 lane change를 소량만 추가한다.
- addon 안전 gate: 예상 TTC `>= 4--5 s`, 후방 초기 간격 `>= 30 m`, restricted lane 제외.
- NPC 목표속도 `40/50/60 km/h`를 유지하면, 고속 Ego보다 뒤차가 접근하는 장면은 Ego가
  느린 선행차 속도에 맞춘 추월 준비 단계에서 만든다.

실제 cut-in을 첫 수집에 섞으면 controller 실패와 velocity supervision 실패를 구분하기
어려우므로 별도 campaign으로 유지한다.

## 구현 계약과 무재학습 E25 A/B

공통 벡터 연산은
`base_ws/src/perception_object_pipeline/src/perception_object_pipeline/rolling_lidar.py`에
둔다. offline 변환기와 BEVFusion ROS bridge가 같은 point-time gate, pose 보간,
LiDAR extrinsic, scan reference 수식을 호출한다.

- 유효 scan: 모든 point time이 finite이고 `0 < max(time)-min(time) <= 0.15 s`.
- 기준시각: 기본 `header + max(point time)`, 즉 scan end.
- 보정: point별 Ego pose만 보간한다. GT object motion은 online에서 알 수 없으므로
  학습 데이터에도 적용하지 않는다.
- invalid time 또는 pose coverage 실패: raw와 섞지 않고 frame을 drop한다.
- E25 production 기본값: `lidar_deskew_mode=off` 유지.

학습 데이터는 원본 bag을 바꾸거나 dataloader에서 매 epoch 계산하지 않는다. 변환 시 한
번 deskew한 `lidar.bin`을 별도 derived dataset에 저장하며, PKL sample과
`conversion_audit.json`에 source header stamp, reference offset, span, displacement,
reject count를 기록한다. 실제 1-scene/3-frame 변환 smoke에서 3/3 frame이 생성됐고,
벡터화 deskew CPU 비용은 mean `1.90 ms`, p95 `2.13 ms`였다.

기존 E25 weight를 재학습하지 않고 3개 고주로 clip에서 raw/scan-end deskew를 비교했다.
class threshold는 모두 `0.40/0.15/0.10`이다.

| clip | raw AP / F1@1m / mATE | deskew AP / F1@1m / mATE |
|---|---:|---:|
| merge A | `0.6805 / 0.5397 / 0.796 m` | `0.8223 / 0.9552 / 0.439 m` |
| merge B | `0.4632 / 0.0000 / 1.588 m` | `0.7054 / 0.7742 / 0.655 m` |
| cruise | `0.8223 / 0.9851 / 0.520 m` | `0.6888 / 0.9118 / 0.735 m` |
| equal-clip macro | `0.6553 / 0.5083 / -` | `0.7389 / 0.8804 / -` |

runtime coverage macro는 raw `0.8802`, deskew `0.8828`로 유사했다. Deskew 자체의
online CPU 비용은 p50 약 `1.94--2.00 ms`, p95 `2.24--2.41 ms`였으며 end-to-end
latency를 증가시키지 않았다.

결론은 **기하학적으로 유효하지만 무조건적인 production 개선은 아니다**. Ego deskew는
정적 배경과 merge 두 clip을 크게 개선했으나, 움직이는 NPC의 scan 내부 motion은 남기
때문에 cruise 한 clip을 악화시켰다. GT object motion을 offline에서만 보정하면 online
입력과 계약이 달라지므로 금지한다. Base4에서는 raw와 ego-deskew derived dataset을 같은
split로 만들어 학습 A/B하고, 동적 객체의 최종 해결은 causal temporal fusion/tracker
memory에서 검증한다.

평가 산출물:

```text
artifacts/evaluation/bevfusion/deskew_e25_ab_260920/
```

## 팀 전달·학습·추론 계약

현재 이미 공유한 Base3-Static, Base3-Online package와 아직 원본 형태인 Base4 bag에는
deskew가 자동 적용되지 않는다. converter 기본값도 `--lidar-deskew off`다. 현재 생성된
deskew dataset은 3-frame smoke뿐이며 팀 학습용 전체 package는 아직 만들지 않았다.

팀 전달 시에는 파일명과 PKL metainfo에 입력 계약을 명시한 별도 package로 만든다.

```text
base4_<campaign>_raw/
base4_<campaign>_ego_deskew_scan_end/
```

`ego_deskew_scan_end` package는 변환 때 보정된 `lidar.bin`을 이미 저장하므로 팀원은
별도 deskew 코드 없이 평소처럼 학습하면 된다. 그러나 그 checkpoint의 online 추론은
반드시 같은 scan-end Ego deskew를 사용해야 한다. 기존 bridge를 그대로 기본 인자로
실행하면 `off`이므로 계약이 맞지 않는다.

```bash
roslaunch bevfusion_ros base3_online.launch \
  lidar_deskew_mode:=ego_pose_2d \
  lidar_deskew_reference:=end \
  lidar_deskew_pose_topic:=/localization/odometry/local \
  lidar_deskew_pose_type:=odometry
```

`/localization/odometry/local`은 GNSS x/y를 직접 fuse하지 않고 Competition yaw/vx와
IMU로 scan 구간의 smooth relative motion을 제공한다. Global map pose인
`/localization/kinematic_state`는 planning/perception 좌표 기준으로 유지하되, GNSS
quantization jump가 scan 내부 보정에 들어가지 않도록 deskew 입력에서는 분리한다.

반대로 raw dataset checkpoint는 `lidar_deskew_mode:=off`로 추론한다. Base4 A/B가 끝나기
전에는 raw/deskew 중 하나를 이름 없는 공통 package로 배포하거나 production 기본값을
변경하지 않는다.

## 과거에는 왜 exact deskew가 불가능했나

Base2/Base3의 SaveSensorData `*_Intensity.bin`은 point당 `x,y,z,intensity` 네 값만
저장했다. scan 내부 point acquisition time과 scan 구간 Ego pose trajectory가 없어
다음 둘은 구분해야 했다.

- 과거 sweep 전체를 현재 frame으로 옮기는 sweep-to-sweep Ego 보정: frame별 Ego pose가
  있어 가능했다.
- 한 sweep 내부의 point별 Ego deskew: exact point time이 없어 불가능했다.

당시 실험은 `1,200 azimuth x 16 channel`, 고정 LiDAR period, Ego physical velocity와
simulator/wall-time 비율을 이용해 point time을 **근사 역산**했다. 좌표 변환 자체는
구현했지만 exact timing 계약이 아니었고, raw로 학습된 PointPainting에 추론 때만 적용해
성능도 악화됐으므로 production에서 껐다.

현재 MR/MRP hybrid rosbag의 PointCloud2에는 point별 `time`과 `ring`이 있으며,
`/Ego_topic` pose가 약 40--50 Hz로 함께 기록된다. 따라서 방위각 역산 없이 point time에
Ego pose를 직접 보간할 수 있다. 또한 정방향/reverse/zero-time 정적 구조물 A/B로 MORAI
time field가 단순 장식이 아니라는 것까지 확인했다.

## Deskew에서 틀릴 수 있는 값

임의의 보정 gain `10`을 넣고 실제 정답이 `11`인 형태는 아니다. 각 point 좌표는 다음
측정값에서 결정되는 강체변환으로 계산한다.

1. point `time`의 단위·부호와 LiDAR header가 가리키는 시각.
2. 같은 clock에 놓인 Ego pose timestamp와 보간 결과.
3. LiDAR-to-Ego extrinsic translation/yaw.
4. start/mid/end 중 선택한 공통 reference time.
5. pose coverage와 허용 extrapolation.

따라서 튜닝 gain보다 timestamp bias, clock 불일치, extrinsic 오차, 잘못된 time 방향이
실제 위험이다. `0.15 s`는 보정량을 키우는 계수가 아니라 비정상 scan을 버리는 validity
gate다. 현재 구현은 K-City 평면 주행에 맞춘 SE(2) pose 보정이라 roll/pitch가 큰 노면은
별도의 3D deskew 대상이다.

학습 변환은 `/Ego_topic` GT pose를 사용한다. 대회 추론은
`/localization/odometry/local`을 우선 후보로 사용한다. 수식은 같지만 online local
odometry의 timestamp, velocity/yaw 오차와 delay가 크면 deskew도 흔들리므로 최종 승격 전
동일 checkpoint로 GT-offline/local-odom-online end-to-end A/B를 통과해야 한다.
