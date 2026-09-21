# Base4-Temporal 고주로 Velocity Pilot 계획

> **문서 상태: CONTROL-SAFETY REVALIDATED / LIMITED CAMPAIGN READY.** 전체 Base4 데이터셋을 확정하는 문서가 아니라, 고주로에서
> BEVFusion velocity observation의 실효성을 가장 작게 검증하기 위한 통제 실험이다.

## 한 줄 결론

Production E25의 공간 인식 능력을 초기값으로 사용하고, 새 고주로 연속 sequence에서
Vehicle `vx, vy`를 학습한다. 기존 고주로 두 rosbag은 반복 사용된 **development
evaluation**으로만 쓰고, 최종 성능 주장은 새 held-out 고주로 rosbag으로 확인한다.

## P0 선행 gate: MORAI LiDAR scan 생성 방식 판별

Base4 temporal 입력을 최종 확정하기 전에 scan 내부 motion distortion의 존재 여부를
반드시 실험으로 판별한다. 이 항목은 선택적인 성능 개선이 아니라 잘못된 deskew로 전체
데이터를 훼손하지 않기 위한 **최우선 선행 gate**다.

고속 직진과 회전 중 정적 기둥·벽을 같은 구간에서 다음 세 입력으로 비교한다.

1. raw cloud.
2. point `time` 기반 Ego-motion deskew cloud.
3. 모든 point time을 `0`으로 가정한 cloud.

정적 구조물의 두께, 직선성, 이중상·끌림, frame 간 center/edge 분산을 정량화한다. raw가
가장 곧고 deskew 후 휘면 MORAI snapshot/synthetic-time 가능성이 높으므로 scan deskew를
계속 끈다. deskew 후 구조물이 유의하게 선명하고 안정되면 point-time 계약과 Ego
velocity/yaw-rate frame을 재검증한 뒤에만 켠다. 판정 전 기본값은 다음과 같다.

- 과거 **sweep 간** Ego-motion compensation: `on`.
- 한 scan **내부** per-point deskew: `off`.
- 학습용 raw point와 point `time`: 손실 없이 보존.

이 gate는 2026-09-20에 완료됐다. 40 bag / 1,911 cloud에서 point-time deskew가 정적
alignment p50을 `0.22819 -> 0.20640 m`로 약 `9.5%` 개선했다. 반대 방향 보정은
`0.25280 m`로 악화됐다. 따라서 point time은 유효하다고 판단한다. 단, 42개 scan에서
span이 `0.15 s`를 넘고 최대 `1.392 s`였으므로 production 기본값은 계속 off다. raw와
point time을 보존하고 outlier rejection을 포함한 converter/online 동일 A/B 후 켠다.
상세 근거는
[`260920_base4_deskew_and_control_safety_evidence.md`](../../dataset/morai_od/docs/260920_base4_deskew_and_control_safety_evidence.md)에 있다.

이후 공통 벡터화 deskew를 offline converter와 online BEVFusion bridge에 옵션으로
구현했다. 연산비는 약 `2 ms/scan`으로 충분히 작았지만, E25 무재학습 3-clip A/B에서
merge는 개선되고 cruise는 악화됐다. 따라서 production 기본값은 `off`를 유지하며,
Base4 raw/ego-deskew 학습 A/B에서만 최종 채택한다. Offline에서만 가능한 GT object
motion 보정은 online 계약과 달라지므로 사용하지 않는다.

## 왜 지금 이 실험인가

- Production E25 + EKF position-only tracking의 고주로 2D AVE는 약 `0.96 m/s`, 전체
  3초 Top-1 FDE는 약 `4.66 m`다.
- Tracker motion model과 covariance sweep으로 얻은 개선은 유효하지만 작았다.
- GT velocity oracle을 매 update에 보수적으로 융합했을 때 2D AVE는
  `0.852 -> 0.259 m/s`, stable 3초 FDE는 `1.617 -> 0.594 m`로 낮아졌다.
- 따라서 다음 큰 폭의 motion 개선 후보는 tracker 복잡도 증가가 아니라 detector가
  실제 velocity observation을 제공하는 것이다.

Oracle 수치는 production 예상치가 아니라 **velocity 정보가 충분히 정확할 때의 잠재
상한**이다. 실제 head는 반드시 velocity 미사용 tracker와 같은 조건에서 비교한다.

## 범위와 명칭

- Dataset/실험명: `Base4-Temporal-Highway-Pilot`, 축약 `B4T-HWY-P0`.
- 대상 class: 첫 pilot은 `Vehicle`만.
- 대상 구간: 고주로 ramp, merge, mainline, taper, exit.
- 목적: 전체 대회 분포 학습이 아니라 velocity head와 temporal input의 효과 분리.
- 이 결과만으로 전역 production을 교체하지 않는다.

## 수집 계약

### 필요한 motion family

- Ego/NPC 동일 속도 순항.
- Ego가 선행차를 따라잡거나 멀어지는 상대속도 변화.
- NPC 가속·감속과 spawn 직후 안정화 구간.
- 같은 차선 선행/후행 차량.
- 양쪽 인접 차선의 병렬·추월 차량.
- 합류·차선 변경으로 횡속도가 생기는 차량.
- 주 평가 거리 `10--30 m`; 가까운 차량과 30 m 경계 표본도 유지.

### 첫 고주로 campaign의 구간·속도 배분

조기 ramp 가속 상태는 어렵지만 planning이 아직 즉시 반응할 필요가 작다. 따라서 첫
pilot은 합류 이후의 실제 추월·상대속도 분포에 집중한다.

| 구간 | 목표 비율 | Ego 속도 | NPC 상태 |
|---|---:|---|---|
| ramp 초기 접근 | 0% | - | live pilot에서 제어권 전환 전 actor 이탈로 제외 |
| 실제 merge/conflict | 25% | 50/54/58 km/h | pilot은 40/50/60 anchor, production은 더 넓은 연속 범위 |
| post-merge 순항·추월 | 75% | 70/85/100 km/h | pilot은 40/50/60 anchor, production은 더 넓은 연속 범위 |

합치면 실제 합류 25%, 합류 이후 75%다. `115/120 km/h`는 제어 안전성 기준선에서
제외하고, 100 km/h campaign이 안정된 뒤 별도 stress addon에서만 재검토한다.
고속 Ego는 capture 시작 전에 충분히 상류에서 출발해 목표속도에 도달시킨다. 같은 차선의
큰 상대속도는 충돌 위험 때문에 짧은 window로 제한하고, 큰 closing/opening speed는 주로
인접 차선에서 만든다. 첫 pilot에는 실제 lane-change controller를 넣지 않으며, 이후
lateral-motion addon으로 분리한다. 큰 상대속도는 인접 차선에만 배정하고
restricted solid-separated lane은 계속 제외한다.

2026-09-21 MORAI 공식 답변에서 본선 고주로·로터리 NPC 속도가 샘플과 동일하지 않다고
확인됐다. 따라서 위 `40/50/60 km/h`는 현재 temporal pipeline A/B를 통제하기 위한
anchor일 뿐 최종 본선 prior가 아니다. 다음 production manifest는 안전하게 관측 가능한
범위 안에서 NPC 속도를 연속 구간으로 jitter하고, closing/opening/parallel 상대속도 bin이
고르게 채워지도록 구성한다. 또한 본선 고주로 RandomMode 사용 시 `MiddleSize`만 쓸
예정이므로 main vehicle quota는 MiddleSize 중심으로 바꾸고 다른 차종은 robustness
coverage로만 제한한다. 로터리는 고정 차종이지만 비공개이므로 K5 단일 overfit은 하지
않는다.

초기 live pilot 자체는 merge conflict `2/2`, post-merge `9/9`를 완료했지만 이후 본
campaign에서 제어권 인수 전 차선 이탈이 발견되어 그 결과만을 production gate로 쓰지
않는다. zero-speed load와 drift/speed/stability gate를 적용한 `r9`에서 merge/cruise
각 16/16 frame을 통과했고, 추가 100 km/h smoke까지 합쳐 `48/48 frame`, 실패 0,
조향 abs max `0.284°`를 재확인했다.

### sequence 단위

- 독립 snapshot이 아니라 실제 LiDAR cadence의 연속 frame을 저장한다.
- 첫 목표는 약 `250--350 sequence`, sequence당 최소 `12`개 유효 LiDAR frame이다.
  최종 수량은 raw scene 수가 아니라 필터 후 유효 Vehicle track과 motion-family
  coverage로 결정한다.
- `/dataset/scenario_marker` 밖의 load/teleport 구간은 제거한다.
- scene/sequence를 split 단위로 사용하고 인접 frame을 train/val에 나누지 않는다.

### Gated raw 기록과 저장 용량

- 센서는 끊지 않고 계속 수신하되 ROS snapshot ring buffer를 사용한다.
- 정상 `capture_end`에서만 scene bag을 저장하며 scene-load/reconnect 대기는 버린다.
- 2026-09-20 MR 실측에서 32 MiB는 start marker가 잘려 기각했고, 48 MiB는
  `2.71--2.76 s` window를 완전히 보존했다.
- 현재 48 MiB 설정의 실제 파일은 scene당 약 `50.5 MB`다.
- 1,200 scene 약 `61 GB`, 1,500 scene 약 `76 GB`를 예상한다.
- 첫 20--30 scene에서 marker, sensor rate, camera age와 scene-to-bag sidecar를 audit한
  뒤 unattended campaign으로 넘어간다.
- continuous recorder는 진단 fallback이며, raw bag을 녹화 중 삭제하면서 converter를
  병렬 실행하는 방식은 crash 시 source/metadata가 어긋나므로 기본 경로로 쓰지 않는다.

### timestamp와 좌표

- LiDAR header timestamp를 sample 기준시각으로 사용한다.
- Camera는 causal tolerance 안의 가장 가까운 frame을 사용하고 age를 기록한다.
- Ego/Object pose는 LiDAR 시각 앞뒤 상태로 보간한다.
- 학습 label은 LiDAR/Ego 축의 절대 planar vector
  `velocity_target_ego_mps=[vx, vy]` 하나만 노출한다.
- 이 velocity는 catalog 보정 후 geometric BBox center 기준이다. `/Object_topic`의 asset
  pivot velocity에 전후 heading으로 계산한 yaw rate와 pivot-to-center offset의
  `omega x r` 항을 더한다. 짧고 정상적인 bracket가 없으면 pivot velocity로 fallback하고
  instance metadata에 그 이유를 남긴다.
- 위치 미분값은 필요할 때 `diagnostics` namespace에서 GT sanity check에만 사용하며,
  학습 target 이름으로 노출하지 않는다.
- runtime `DetectedObject`에서는 detector/LiDAR `vx, vy`를 box yaw로 회전해 object-body
  twist로 변환하고, tracker가 이를 다시 map `vx, vy` observation으로 사용한다.

### temporal 입력의 motion compensation

- 과거 sweep은 Ego pose로 현재 LiDAR frame에 정합한다.
- GT Object pose로 동적 객체 point를 현재 위치에 강제 정합하지 않는다. 실제 추론에는
  object GT가 없고, 이를 적용하면 velocity 단서를 제거하거나 label leakage가 된다.
- 단순 point concatenation의 object trail이 geometry를 해칠 수 있으므로 single-frame
  velocity head를 먼저 검증한 뒤 temporal feature fusion으로 넘어간다.
- Object pose/velocity는 input 보정이 아니라 label 생성과 감사에 사용한다.

### 센서 시간 정합과 deskew 계약

Raw baseline의 canonical timestamp는 LiDAR header 시각 `t0`다. Deskew A/B의
canonical timestamp는 `t_ref = t0 + max(point time)`, 즉 scan end다. 해당 조건에서는
LiDAR point, camera 선택, Ego/Object GT와 detector output stamp를 모두 `t_ref`에 맞춘다.
서로 다른 세 가지 보정을 혼동하지 않는다.

| 구분 | 의미 | Base4 첫 계약 |
|---|---|---|
| scan 내부 deskew | 한 PointCloud2 안에서 point별 취득시각 차이 보정 | 기본 `off`; A/B는 scan end |
| camera--LiDAR 정합 | nearest camera 시각과 canonical time 사이의 정합 | raw=`t0`, deskew=`t_ref`; fixed extrinsic |
| sweep 간 정합 | 과거 LiDAR frame을 현재 canonical Ego frame으로 이동 | `on` |

현재 production bridge는 LiDAR callback마다 camera buffer에서 가장 가까운 frame을 고르고
고정 extrinsic을 사용한다. 허용차 기본값은 `0.10 s`이며 Ego pose를 이용한 camera 시각
보정은 아직 없다. Base4 raw/index에는 실제 camera timestamp와 `camera_age`, LiDAR
timestamp, Ego pose history를 보존해 나중에 정합 방식을 바꿔도 재취득하지 않도록 한다.

MORAI cloud에는 `ring`과 point `time` field가 있지만, 이것만으로 simulator가 실제로
ray를 시간 순서대로 렌더링했다고 결론 내릴 수 없다. 특히 `12 Hz` message rate의 역수
`83 ms`를 곧바로 physical scan duration으로 간주하거나, `100 km/h`에서 `2.3 m`의 scan
왜곡이 있다고 단정하지 않는다. 순간 snapshot에 Velodyne 형식의 synthetic point time이
붙는 경우라면 deskew가 오히려 가짜 왜곡을 만든다. 과거 production에서 deskew를 끈
이유도 distortion이 없다고 증명했기 때문이 아니라, moving high-speed static-structure
A/B와 velocity/timestamp frame 계약이 충분히 검증되지 않았기 때문이다.

따라서 deskew는 고속 직진·회전 중 정적 벽/기둥을 대상으로 다음을 비교한 뒤에만
채택한다.

1. raw cloud.
2. point `time`을 사용한 Ego-motion deskew.
3. 모든 point가 동시 취득됐다고 가정한 cloud.

deskew 전후 정적 구조물의 두께·직선성·frame 간 안정성을 비교하며, 단순 RViz 인상만으로
결정하지 않는다. 이 A/B가 끝나기 전 B4H-V1/T1의 기본 입력은 raw cloud다.

기존 Base2/Base3 SaveSensorData XYZI에는 exact point time이 없었다. 당시 azimuth, 고정
scan period와 Ego velocity로 시간을 근사했던 것은 현재 rosbag의 실제 PointCloud2
`time` 기반 방식과 다른 계약이다. 현재 shared Base3 package도 자동으로 deskew되지
않으며, 향후 팀 전달은 raw와 `ego_deskew_scan_end` package를 명시적으로 분리한다.

Camera 시간 보정도 독립된 실험 축이다. nearest frame에서 Ego가 이동한 거리는 그대로
3D center 오차와 같지 않고, 실제 효과는 camera age, 회전, 물체 거리·방향, localization
jitter에 달려 있다. 네 camera가 서로 다른 timestamp를 가질 수 있으므로 LiDAR를 각
camera 시각으로 각각 옮기지 않는다. 필요할 경우 camera별 pose 차이를 반영한 동적
`camera2lidar`를 만들어 모두 현재 LiDAR `t0` frame으로 올린다.

### Object motion을 보정하지 않는 이유

센서 입력에서는 Ego motion만 보정한다. Object pose는 GT box를 LiDAR `t0`에 맞추는
label interpolation에는 사용하지만, 과거 object point를 현재 위치로 이동시키는 데에는
사용하지 않는다.

- online inference에는 GT instance association과 object velocity가 없다.
- 학습 때만 GT object compensation을 쓰면 train--runtime mismatch와 label leakage가
  생긴다.
- 현재 위치에 과거 object point를 완전히 중첩하면 velocity를 학습할 displacement 단서가
  사라진다.
- Ego 정합 후 남는 moving-object trail은 point `dt`와 함께 temporal cue로 사용한다.

T1에서 trail이 box geometry를 훼손하면 raw point를 GT로 보정하는 대신 T2의 fused-BEV
memory 또는 이후 Q1의 object query memory로 이동한다. Q1에서는 detector가 예측한
velocity로 query reference point를 propagate할 수 있지만, 이것은 GT object motion
compensation과 다른 online-reproducible 연산이다.

## 학습·평가 split

Pilot dataset은 다음 두 split만 만든다.

- `train`: 약 80%, sequence 단위.
- `val`: 약 20%, 다른 scenario seed·route segment·NPC motion 조합.

별도 dataset `test` split은 만들지 않는다. 작은 pilot에서 표본을 다시 떼기보다 val로
checkpoint를 고르고 rosbag에서 downstream을 검증한다.

단, 현재 `vehicle_highway_merge`와 `vehicle_highway1`은 이미 반복적으로 확인하고
파라미터 선택에 사용했으므로 엄밀한 test가 아니다.

- 기존 두 bag: development rosbag, 빠른 A/B와 epoch 선택 보조.
- 새 고주로 bag 최소 2개: 마지막에 한 번 여는 held-out rosbag.
- held-out은 다른 NPC seed, 속도, spawn timing과 주행을 사용한다.

새 held-out 없이 기존 두 bag만으로 얻은 결과는 `pilot/dev result`라고 보고하며 최종
일반화 성능이라고 표현하지 않는다.

## 비교할 모델

| ID | 초기값 | 입력 | 출력 | 목적 |
|---|---|---|---|---|
| E25-7D | Production E25 | single frame | 7D box | 고정 기준선 |
| B4H-V1 | E25 spatial weights | single frame | 7D + `vx,vy` | head/supervision 효과 |
| B4H-T1 | B4H-V1 또는 E25 | causal 3-sweep | 7D + `vx,vy` | 실제 temporal 이득 |

처음부터 3-sweep 하나만 학습하지 않는다. B4H-V1이 있어야 velocity GT 자체의 품질과
temporal 입력의 추가 이득을 분리할 수 있다.

## Temporal architecture 판단과 확장 순서

### 결론

현재 코드에서 **최소 수정으로 최대 실험 효율**을 노리는 첫 temporal 방식은
StreamPETR query memory가 아니라 원본 BEVFusion의 causal multi-sweep 입력을 복구하는
것이다. StreamPETR는 유력한 후속 후보지만, B4H-T1보다 먼저 구현하지 않는다.

권장 확장 순서는 다음과 같다.

| 단계 | ID | 방식 | 역할 |
|---|---|---|---|
| 0 | E25-7D | 현재 single-frame 7D | 고정 production 기준선 |
| 1 | B4H-V1 | single-frame 9D velocity head | velocity label·head 자체의 효과 분리 |
| 2 | B4H-T1 | Ego 정합 3-sweep + point `dt` | 가장 작은 temporal baseline |
| 3 | B4H-T2 | 직전 fused-BEV ego warp + gated fusion | raw point ghosting을 피하는 feature temporal |
| 4 | B4H-Q1 | StreamPETR-inspired sparse query memory | 긴 객체 이력과 object-centric motion |

### 관련 연구에서 직접 가져올 원칙과 가져오지 않을 주장

다음 연구들은 temporal 정보가 velocity와 detection에 유효하다는 근거지만 센서 modality와
head가 서로 다르므로 논문 수치를 MORAI 성능 예상치로 사용하지 않는다.

| 연구 | 실제 핵심 | Base4에 적용하는 범위 |
|---|---|---|
| nuScenes multi-sweep | 과거 cloud를 keyframe Ego 좌표로 이동하고 point에 time lag 추가 | T1의 Ego 정합+`dt`; scan 내부 deskew 근거로는 사용하지 않음 |
| BEVFusion/CenterPoint | multi-sweep 입력과 velocity regression | V1/T1의 가장 가까운 baseline |
| BEVDet4D | 이전 camera BEV를 현재 frame과 결합해 motion cue 사용 | T2의 저비용 recurrent fusion 참고 |
| BEVFormer | recurrent history BEV와 temporal deformable attention | T2-D의 원리 참고; camera-only 전체 구조는 이식하지 않음 |
| BEVFusion4D | multimodal BEV의 temporal deformable alignment | T2 이후 geometry 퇴행 시 참고 |
| StreamPETR | object-query memory와 motion-aware propagation | Q1에서 Vehicle Top-K memory만 단계적으로 참고 |
| SOLOFusion | short/long-term camera temporal stereo | 현재 LiDAR-camera velocity pilot에는 직접 적용하지 않음 |

nuScenes의 sweep accumulation은 **서로 다른 sweep 전체를 keyframe으로 옮기는 것**이며,
MORAI 한 scan 내부의 per-point ray acquisition을 증명하지 않는다. BEVFormer,
BEVDet4D와 StreamPETR는 주로 camera-only 결과이므로 temporal modeling의 유효성 근거로는
쓰되 LiDAR-camera BEVFusion의 구현 정답으로 취급하지 않는다. 현재 문제와 가장 가까운
multimodal 확장은 BEVFusion4D지만, deformable alignment부터 시작하면 label, dataset,
velocity head 문제와 architecture 효과를 분리할 수 없으므로 V1/T1/T2 뒤에 둔다.

- nuScenes: <https://openaccess.thecvf.com/content_CVPR_2020/papers/Caesar_nuScenes_A_Multimodal_Dataset_for_Autonomous_Driving_CVPR_2020_paper.pdf>
- BEVFusion converter: <https://github.com/mit-han-lab/bevfusion/blob/main/tools/data_converter/nuscenes_converter.py>
- CenterPoint: <https://arxiv.org/abs/2006.11275>
- BEVDet4D: <https://arxiv.org/abs/2203.17054>
- BEVFormer: <https://arxiv.org/abs/2203.17270>
- BEVFusion4D: <https://arxiv.org/abs/2303.17099>
- StreamPETR: <https://arxiv.org/abs/2303.11926>

### 과거 PointPainting 3-sweep 결과의 해석

과거 PointPainting temporal 모델의 velocity가 충분히 좋지 않았다는 사실은 temporal
입력이 production 개선을 보장하지 않는다는 중요한 경고다. 하지만 다음 요인들이 함께
바뀌었기 때문에 `3-sweep이 무효`라는 결론으로 사용하지 않는다.

- Base2에는 고속 Ego, 큰 상대속도, 합류·인접차선 추월 motion family가 부족했다.
- `derived_velocity_mps`와 확정 target의 역할이 전달 과정에서 혼동됐다.
- PointPainting의 raw point concatenation에서는 moving-object trail이 painted geometry를
  흐릴 수 있다.
- velocity head/supervision 효과와 temporal input 효과를 독립적으로 측정하지 않았다.
- SaveSensorData 학습 domain과 실제 asynchronous ROS/UDP rosbag domain 차이가 있었다.

이번에는 V1로 label/head만 먼저 검증하고 T1로 multi-sweep의 추가 이득을 측정한다. T1의
speed MAE/2D AVE가 좋아져도 center/AP와 downstream Map-CV FDE가 나빠지면 채택하지
않으며, 그런 경우에만 T2로 point trail을 feature memory로 대체한다.

### 독립 실험 축

한 run에서 deskew, camera time alignment와 temporal architecture를 동시에 바꾸지 않는다.

| 축 | 기준선 | 후보 |
|---|---|---|
| scan | raw/deskew-off | verified per-point Ego deskew |
| camera sync | nearest + fixed extrinsic | nearest + camera별 Ego-time dynamic transform |
| temporal | V1 single-frame 9D | T1 multi-sweep, 이후 T2 BEV memory |

분석 slice는 Ego speed만으로 제한하지 않는다. Ego speed, object speed, 종·횡 상대속도,
same/adjacent lane, merge 여부, object distance, camera age와 track age를 함께 본다. 최종
판정은 OD center AP/mATE, speed MAE와 실제 2D AVE, tracking AVE, Map-CV FDE@1/2/3s,
latency·VRAM·frame drop을 같은 checkpoint에서 묶어 수행한다.

앞 단계가 채택 gate를 넘지 못했다고 무조건 다음 단계를 구현하지 않는다. 특히 V1만으로
tracker AVE와 Map-CV FDE가 충분히 개선되면 temporal 구조의 production 복잡도를 추가할
이유가 없다.

### B4H-T1이 첫 temporal baseline인 이유

현재 MORAI Base3 config는 single-frame 실험을 위해 원본 MMDetection3D BEVFusion에서
다음 기능을 의도적으로 제거한 상태다.

- `LoadPointsFromMultiSweeps`와 여러 과거 sweep.
- 다섯 번째 point feature인 time lag.
- TransFusion의 `vel=[2,2]` head와 10D box coder.

반대로 upstream BEVFusion config는 이미 9개 과거 sweep, 5-feature LiDAR와 velocity
head를 지원한다. 따라서 B4H-T1은 새 detector를 이식하는 작업이 아니라 기존 경로를
MORAI의 실제 연속 sequence와 3-sweep 계약에 맞게 복구하는 작업에 가깝다. 현재
`adapt_nuscenes_checkpoint.py`에서 제거하는 fifth-channel sparse-conv weight와 velocity
head도 이 단계에서 다시 사용할 수 있다.

과거 sweep은 Ego motion만 현재 LiDAR frame으로 정합하고 point마다 실제 `dt`를 둔다.
GT Object pose를 이용한 object compensation은 online 추론에서 재현할 수 없고 velocity
단서를 지우므로 계속 금지한다.

### B4H-T2: lightweight temporal BEV

B4H-T1에서 동적 객체 trail 때문에 center/AP가 나빠지거나 velocity 개선이 작으면 다음
후보는 직전 fused-BEV 한 장을 쓰는 경량 recurrent fusion이다.

1. 직전 fused-BEV와 그 시각의 Ego pose를 cache한다.
2. 현재 Ego frame으로 과거 BEV를 warp한다.
3. current/previous feature를 concatenate한다.
4. 작은 `1x1 convolution` 또는 learned gate로 다시 기존 head channel에 투영한다.

온라인에서는 직전 BEV를 재사용하므로 매 frame 과거 encoder를 다시 돌리지 않는다.
다만 학습 loader는 순서가 보장된 clip, 동일 clip augmentation, scene reset과 frame-drop
정책을 제공해야 한다. 이 방식도 단순 config 변경은 아니지만 현재 BEVFusion encoder와
TransFusion head를 유지하므로 query-memory 이식보다 변경 범위가 작다.

### StreamPETR를 그대로 첫 선택으로 삼지 않는 이유

StreamPETR의 핵심은 이전 BEV feature의 `grid_sample` cache가 아니라 **object-query
memory**다. 과거 frame의 Top-K query feature, reference point, confidence, timestamp와
Ego pose를 FIFO memory에 보관하고, 현재 query가 propagation transformer로 과거 query와
상호작용한다. 따라서 단순히 `extract_feat` 뒤에 BEV warp 하나를 붙이는 방식과 다르다.

우리 TransFusion head도 query를 사용하므로 개념적으로는 결합 가능하지만 현재 query는
매 frame dense heatmap에서 새로 생성된다. B4H-Q1에는 최소한 다음 변경이 필요하다.

- 최종 Top-K query feature·center·score·velocity의 외부 노출과 저장.
- timestamp/Ego pose 기반 reference point 변환.
- current query와 memory query 사이 temporal attention.
- batch sample별 sequence ID, memory validity와 scene/teleport reset.
- ordered streaming 또는 clip 학습, augmentation 일관성, memory detach/BPTT 정책.
- 초기 miss와 오검출 query가 memory에 누적되는 경우의 age/confidence gate.
- runtime frame drop과 비정상 `dt` 처리.

따라서 sparse memory의 online 계산량은 매력적이지만 구현·학습 위험은 B4H-T1/T2보다
높다. T1 또는 T2로 temporal 정보의 실효성이 확인되고 dense feature memory의 지연이나
velocity 한계가 남을 때 `B4H-Q1`로 통제 실험한다. StreamPETR 전체를 포팅하지 않고
Vehicle Top-K query만 보존하는 작은 memory bank부터 시작한다.

공식 StreamPETR는 camera-only PETR 계열이며 논문 결과는 8-frame sliding-window 학습을
사용했다. 공식 저장소의 streaming-video 학습은 메모리·시간을 줄이지만 더 늦게
수렴한다고 명시한다. 또한 공식 환경은 PyTorch 1.9와 MMDetection3D 1.0.0rc6 계열이라
현재 팀 Docker/MMDetection3D에 파일을 그대로 복사하지 않는다.

- Paper: <https://arxiv.org/abs/2303.11926>
- Official repository: <https://github.com/exiawsh/StreamPETR>
- Official training notes: <https://github.com/exiawsh/StreamPETR/blob/main/docs/training_inference.md>

### SOLOFusion과의 관계

SOLOFusion은 camera-only BEV 계열에서 긴 image history의 coarse cost volume과 짧은
history의 fine-grained stereo를 결합한다. 긴 history와 2-stage training/data sampler가
필요하여 현재 LiDAR-camera BEVFusion velocity pilot의 최소 변경 목표와 맞지 않는다.
아이디어 비교 대상으로는 유지하지만 B4T-HWY-P0 구현 후보에서는 제외한다.

- Paper/repository: <https://github.com/Divadi/SOLOFusion>

### T1 이후 선택 gate

- T1이 V1 대비 velocity와 downstream FDE를 개선하고 geometry gate도 통과하면 T1을
  우선 채택하고 구조 확장을 멈춘다.
- T1 velocity는 좋아지지만 center/AP가 ghosting으로 퇴행하면 T2를 수행한다.
- T2가 정확하지만 latency/VRAM이 production 범위를 넘거나 장기 객체 motion이 여전히
  부족하면 Q1을 수행한다.
- T1과 T2 모두 V1보다 유의미하지 않으면 temporal architecture보다 velocity label,
  motion-family coverage와 loss/covariance 계약을 먼저 재감사한다.

### 팀원 독립 3-sweep 구현 통합 판정 (2026-09-21)

`perception/base4-temporal-3sweep`의 `46d4e175`는 현재 브랜치와 독립적으로
offline 3-sweep builder, dataset/config, validator와 quality audit를 구현했다. 다만
공통 조상 이후 현재 Base4 브랜치가 65개 커밋 앞서 있고, 독립 구현은 다음 현재 계약과
호환되지 않는다.

- 과거 point time을 음수 lag로 저장하지만 현재 runtime은 양수 age를 사용한다.
- 세 sweep을 Nx5 파일로 미리 합치지만 현재 pipeline은 raw XYZI와 causal reference를
  보존하여 loader/runtime가 동일 변환을 재현한다.
- `ObjectStatus.velocity`를 km/h에서 m/s로 바꾸지 않는 경로가 있어 velocity target이
  3.6배가 될 수 있다.
- geometric-BBox-center 회전 속도 보정, scan 내부 deskew 옵션, online sweep buffer,
  ROS velocity 발행과 rosbag downstream 평가는 없다.

따라서 두 dataset/config/runtime를 병렬 유지하거나 전체 commit을 merge하지 않는다.
팀원은 현재 Base4 브랜치를 기준으로 합류한다. 독립 구현에서 가치가 확인된 scene-level
split leakage, sweep SE(3), timestamp gap, 동기 age, BBox-center trajectory/velocity
일관성 검사는 현재 schema용
`tools/audit_base4_temporal_quality.py`로 선별 이식한다. 원격 팀원 브랜치는 구현 근거로
보존한다.

Quality audit의 현재 고정 계약은 다음과 같다.

- separate raw XYZI sweep reference
- past `time_lag_sec > 0`
- velocity unit `m/s`
- velocity reference `geometric_bbox_center`
- scene 단위 split isolation

```bash
python3 perception/detection3d/bevfusion/tools/audit_base4_temporal_quality.py \
  --info-dir artifacts/datasets/morai_base4_temporal_highway_hwy3h1_mrhc_v2
```

MR heading calibration 발견 전 `hwy3h1_raw`로 학습한 V1/T1 E4는 architecture와
runtime pipeline EVIDENCE로만 유지한다. 다음 동일조건 A/B는 MR `-1.347 deg`가 적용된
13,749-frame `hwy3h1_mrhc_v2`와 다음 tracked recipe를 사용한다.

- `base4_hwy3h1_mrhc_v2_v1_e4.yaml`
- `base4_hwy3h1_mrhc_v2_t1_3sweep_e4.yaml`

보정 전후 공통 13,232 frame의 GT center shift는 p50 `0.283 m`, p95 `0.634 m`이므로
예전 checkpoint와 새 dataset 성능을 동일 GT 실험으로 취급하지 않는다.

## Tracker A/B

각 velocity 모델을 동일 detection threshold에서 두 번 평가한다.

1. Detector velocity를 tracker에 넣지 않음.
2. 매 measurement update에 detector velocity를 넣음.
   - 첫 covariance: `0.5 m/s`.
   - 비정상값, `30 m/s` 초과와 비-Vehicle velocity는 거부.

Fusion을 켰을 때 OD geometry, track association 또는 center가 악화되면 velocity
covariance를 키우거나 head를 기각한다. 모델이 velocity를 출력한다는 이유만으로
production fusion을 켜지 않는다.

## 지표와 선택 기준

### OD

- Center-distance AP/precision/recall/F1.
- Center p50/p90, mATE, mASE, mAOE.
- Vehicle speed MAE, 실제 2D AVE, GT-heading 종/횡 velocity MAE.

### Tracking

- F1/MOTA와 track continuity.
- Center p50/p90.
- Speed MAE와 실제 2D AVE.
- Track 생성 후 `0--0.5 s`, 안정 track을 분리해 velocity 수렴시간 확인.

### Prediction/Planning proxy

- Map-CV Top-1 FDE와 minFDE-K `@1/2/3 s`.
- CV-stable과 전체 motion slice를 모두 보고한다.
- 고주로 추월 관점에서 인접 차선 occupancy, track age, 경로 coverage를 함께 본다.

첫 채택 gate는 다음과 같다.

- 기존 E25 대비 Vehicle F1 `-0.01` 이내, center p50 `+0.05 m` 이내.
- Velocity fusion이 velocity 미사용 대비 tracking 2D AVE 또는 stable FDE@3s를
  상대 `10%` 이상 개선.
- 두 development bag 모두 같은 방향으로 개선하며 한 bag 평균으로 다른 bag 퇴행을
  숨기지 않음.
- 전역 production 후보가 되려면 새 held-out 고주로와 기존 Vehicle/Pedestrian/
  Obstacle regression suite를 추가 통과.

## 2026-09-21 구현 상태

실행 순서 1--6의 파이프라인 gate를 24-scene 제한 파일럿에서 통과했다.

- 24 attempted, 23 completed, raw scene bag 23개.
- scene split train 19 / val 4 / test 없음.
- raw 485 frame, Ego-deskew A/B 467 frame.
- LiDAR 시각 Ego/Object interpolation과 9D `[... yaw,vx,vy]` GT 구현.
- current + past 2 Ego-aligned sweep과 positive `dt` loader 구현.
- V1 single-frame velocity head와 T1 3-sweep velocity head가 각각 1-epoch Docker
  smoke 및 validation을 완료.
- matched Vehicle mAVE와 speed MAE를 기존 center metric에 추가하되 공식 NDS로
  오인하지 않도록 분리.

이 PASS는 정확도 결론이 아니다. 구현 결과와 실행법은
[`260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md`](260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md)에
고정한다. 이후 단계는 제한 4-epoch V1/T1 비교와 기존 dev/현장 regression이며,
production 판단 전 새 80--120 km/h held-out controller bag이 필요하다.

## 실행 순서

1. P0 LiDAR raw/deskew/zero-time 고속 정적 구조물 A/B로 scan 내부 deskew 계약을
   확정한다. 단, 결과와 무관하게 원본 point/time은 보존한다.
2. 현재 Base3-Online raw bag/index가 true consecutive temporal sample을 보존하는지
   감사하고, single-frame velocity-head smoke에 재사용 가능한 범위를 확정한다.
3. 새 B4T-HWY-P0 연속 수집기의 marker/timestamp/velocity schema를 2-scene pilot로
   검증한다.
4. 20--30 sequence pilot을 수집해 track continuity, velocity target, camera age,
   point count를 검사한다.
5. B4H-V1 single-frame 9D smoke와 짧은 학습을 먼저 수행한다.
6. B4H-T1 causal 3-sweep을 같은 split과 loss 계약으로 학습한다.
7. 기존 고주로 두 dev bag에서 OD -> tracker off/on -> Map-CV를 순서대로 평가한다.
8. 명확한 후보만 새 held-out bag에서 한 번 평가한다.

## 명시적 비목표

- Base2-Temporal을 최종 production dataset으로 복구하지 않는다.
- Base3-Static 독립 frame을 가짜 연속 sequence로 묶지 않는다.
- GT Object motion compensation을 online input에 사용하지 않는다.
- 기존 고주로 두 bag의 반복 점수를 최종 test 성능으로 부르지 않는다.
- pilot 결과만으로 Pedestrian/Obstacle 또는 전체 route production을 교체하지 않는다.

## 관련 근거

- Tracker·oracle: [`260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`](260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)
- Base3-Online 결과: [`260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md`](260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md)
- Dataset 현재 상태: [`../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`](../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md)
- 기존 Base2 이력: [`../../dataset/morai_od/docs/legacy/base2_10k_d3_temporal_label_index_audit.md`](../../dataset/morai_od/docs/legacy/base2_10k_d3_temporal_label_index_audit.md)
