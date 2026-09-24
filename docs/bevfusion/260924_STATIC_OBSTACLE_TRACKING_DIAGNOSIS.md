# 2026-09-24 static-obstacle full-stack rosbag diagnosis

> **문서 상태: EVIDENCE.** Concrete_Barricade01을 position 4와 position 5에
> 배치해 BEVFusion E25, tracker, prediction, planning, control을 함께 실행한 두
> rosbag의 사후 분석이다. 두 run 모두 실제 충돌이 발생했으므로 성공 회피 성능을
> 주장하는 자료가 아니라 실패 원인을 분리하는 자료로 사용한다.

## 입력과 판정 계약

| 구분 | rosbag | 길이 |
|---|---|---:|
| position 5 | `rosbags/eval/perception/static_obstacle_tracking/260924_222929_bevfusion_static_obstacle_barricade_pos5_01.bag` | 19.78 s |
| position 4 | `rosbags/eval/perception/static_obstacle_tracking/260924_223312_bevfusion_static_obstacle_barricade_pos4_01.bag` | 62.71 s |

- 장애물: `Concrete_Barricade01`, scenario scale 1.
- `/Object_topic`의 이름은 두 run 모두 빈 문자열이었다. GT 비교에는 scenario asset과
  asset catalog의 LiDAR-instance 형상 `0.442 x 2.172 x 0.920 m`를 사용했다.
- detector와 tracker는 메시지 header stamp 기준으로 GT와 동기화했다.
- 이번 경량 bag은 raw camera/LiDAR를 제외했지만 detection, tracking, prediction,
  planning trajectory, static-obstacle debug, control, collision 및 localization 토픽을
  보존했다.

## 결과 요약

| 항목 | position 5 | position 4 |
|---|---:|---:|
| detection GT center error p50 / p95 | 0.905 / 1.097 m | 0.062 / 0.337 m |
| matched detection frame | 41 | 376 |
| tracker ID 수 / switch 수 | 4 / 5 | 2 / 1 |
| 가장 오래 유지된 ID | 약 1.60 s | 약 45.19 s |
| tracker box `y` p05--p95 | 1.098--2.198 m | 2.165--2.185 m |
| tracker GT center error p50 / p95 | 0.856 / 1.507 m | 0.062 / 0.491 m |
| `/CollisionData` 최초 충돌 | 9.502 s | 54.562 s |
| static avoidance candidate가 있었던 frame | 0 | 0 |

## Asset catalog 기준 box size

`/Object_topic.size`는 비교 기준으로 사용하지 않았다. Scenario file에서 asset을
식별하고 `morai_asset_bbox_offsets_base3_v1.json`의 LiDAR-instance geometric BBox를
GT로 사용했다.

| Asset | Catalog GT `(L,W,H)` |
|---|---:|
| Concrete_Barricade01 | `0.442 x 2.172 x 0.920 m` |
| 2016_Hyundai_Sonata | `4.860 x 2.123 x 1.503 m` |

아래 size IoU는 center와 yaw 오차를 제외하고 크기만 같은 중심·방향에 놓아 비교한
진단값이다.

| Bag / output | median `(L,W,H)` | GT 대비 중앙값 오차 | size-only 3D IoU |
|---|---:|---:|---:|
| Barricade position 4 detection | `0.460 x 2.176 x 0.919` | `+4.1%, +0.2%, -0.0%` | `0.958` |
| Barricade position 4 tracking | `0.460 x 2.175 x 0.920` | `+4.2%, +0.1%, -0.0%` | `0.959` |
| Barricade position 5 detection | `0.570 x 2.055 x 0.919` | `+29.0%, -5.4%, -0.1%` | `0.742` |
| Barricade position 5 tracking | `0.637 x 1.914 x 0.915` | `+44.2%, -11.9%, -0.6%` | `0.632` |
| Sonata position 1 detection | `5.004 x 2.084 x 1.498` | `+3.0%, -1.8%, -0.4%` | `0.951` |
| Sonata position 1 tracking | `5.005 x 2.084 x 1.498` | `+3.0%, -1.8%, -0.4%` | `0.951` |
| Sonata manual circle detection | `4.836 x 2.072 x 1.558` | `-0.5%, -2.4%, +3.6%` | `0.939` |
| Sonata manual circle tracking | `4.851 x 2.072 x 1.559` | `-0.2%, -2.4%, +3.7%` | `0.940` |

중앙값만 보면 Sonata manual run도 양호하지만 ID switch 직전 약 4개 detection에서
`Vehicle`이 작은 `UNKNOWN` box, 약 `0.7 x 1.5 x 1.0 m`로 붕괴했다. 기존 track도
마지막 3 frame에서 `1.57 x 1.62 x 1.14 m`까지 작아진 뒤 새 Vehicle track으로
재생성됐다. 따라서 이 run은 전체 중앙값보다 짧은 연속 outlier가 association에 더
중요하다.

Barricade position 5도 같은 양상이다. track별 median과 catalog size-only 3D IoU는
`ID22: 0.871`, `ID23: 0.635`, `ID24: 0.579`, `ID25: 0.297`로, 접근할수록 형상이
단계적으로 무너졌다. 반면 position 4와 Sonata position 1은 detector와 tracker가
catalog 크기를 거의 그대로 유지했다.

### Sonata manual avoidance 02

`260924_231631_bevfusion_static_vehicle_manual_02.bag`은 약 18.9초 동안 Sonata를
자연스럽게 근접 회피한 run이다.

- 실제 Sonata track: 단일 ID로 `0.238--18.835 s`, 559 output frame 연속 유지.
- Vehicle detection 176개: median `4.875 x 2.046 x 1.546 m`, catalog 대비
  `+0.3%, -3.6%, +2.9%`, size-only 3D IoU `0.935`.
- Vehicle tracking 559개: median `4.959 x 2.049 x 1.538 m`, catalog 대비
  `+2.0%, -3.5%, +2.3%`, size-only 3D IoU `0.926`.
- 별도 UNKNOWN track 6 frame은 map `(약 52.8, 428.5)`에 있었고 Sonata
  `(약 70.9, 393.6)`와 무관한 짧은 오검출이다. 차량 ID switch로 세지 않는다.
- `/CollisionData`는 전 구간 비어 있었다.

두 차량의 catalog geometric BBox 중심 간 최소 거리는 약 `2.11 m`였다. Capture
annotation BBox를 2D polygon으로 놓으면 약 1.62초, 42 Ego frame에서 작은 overlap이
생겼고 최대 교차 면적은 약 `0.201 m^2`였지만 물리 collision은 발생하지 않았다.
이는 LiDAR-instance BBox에 labeling margin이 포함되고 물리 collider와 정확히 같지
않다는 MORAI 답변과 일치한다. 따라서 RViz에서 catalog GT box가 순간적으로 닿거나
조금 겹치는 것만으로 실제 충돌이라고 판정하면 안 되고 `/CollisionData`를 함께 봐야
한다.

### Barricade position 04 manual avoidance 01

`260924_232156_bevfusion_static_barricade_pos_04_01.bag`은 약 19.2초 동안
Concrete_Barricade01을 자연스럽게 근접 회피한 run이다. `/CollisionData`는 비어 있고,
Ego IONIQ5와 barricade의 catalog BBox polygon 사이 최소 여유는 약 `0.293 m`였다.

크기 자체는 이전 position 5보다 양호했다.

- UNKNOWN detection 97개: median `0.469 x 2.148 x 0.917 m`, catalog 대비
  `+6.1%, -1.1%, -0.3%`, size-only 3D IoU `0.930`.
- UNKNOWN tracking 303개: median `0.479 x 2.132 x 0.926 m`, catalog 대비
  `+8.5%, -1.8%, +0.7%`, size-only 3D IoU `0.900`.

그러나 동일 장애물 주변에 6개 track ID가 생성됐고, 20 output frame에서는 두 ID가
동시에 존재했다. 관측 구간의 ID와 Ego 거리 범위는 다음과 같다.

| ID suffix | 시간 | Ego 거리 변화 |
|---|---:|---:|
| `07` | `9.419--9.616 s` | `28.3 -> 27.4 m` |
| `06` | `9.529--10.016 s` | `27.8 -> 25.1 m` |
| `08` | `10.047--13.478 s` | `25.1 -> 5.1 m` |
| `09` | `13.087--14.245 s` | `7.2 -> 2.6 m` |
| `0a` | `14.685--16.445 s` | `4.5 -> 17.4 m` |
| `0c` | `16.386--19.215 s` | `16.9 -> 18.6 m` |

Detection은 `9.19--13.93 s`와 `14.47--19.15 s` 두 구간으로 나뉘었다. 가장 가까이
통과할 때 약 `0.54 s` 관측 공백이 생겨 production tracking age `0.5 s`를 약간
넘었다. 하지만 그 전후에도 새 ID가 기존 ID와 겹쳐 생성됐으므로 age만 늘려서는 전체
fragmentation을 해결할 수 없다. Track별 catalog center error p50은 약
`0.10--0.84 m`였고 일부 track의 최대 오차는 `1.08 m`였다.

### Position 04 duplicate detection 재분석

ID가 겹쳐 생성된 구간을 detection header stamp 기준으로 다시 확인하면, 동일 장애물에
두 box가 동시에 나온 frame이 `8.953`, `9.589`, `12.633`, `13.150`, `15.906 s`에
있었다. 같은 frame의 두 center 간 거리는 `0.052--0.130 m`, rotated BEV IoU는
`0.566--0.859`였다. 즉 수 m 떨어진 별도 장애물이나 큰 center jump가 아니라 사실상
같은 barricade를 두 번 검출한 것이다.

이 경우 tracker의 one-to-one association에서는 한 detection만 기존 track에 연결되고
남은 detection은 새 tentative track을 만들 수 있다. 따라서 이 run의 다중 ID를 단순히
association 거리 threshold가 작아서 발생한 문제로 설명할 수 없다. 가까운 duplicate에
대해 matching distance를 늘리는 것도 근본 해결이 아니다.

현재 Base3 BEVFusion 설정은 upstream TransFusion의 `nms_kernel_size=3`과
`test_cfg.nms_type=None`을 상속한다. 동시에 MORAI 3-class를 위해 `dataset='Waymo'`
분기를 재사용하는데, 해당 head는 class index 1과 2의 local-max kernel을 1로 바꾼다.
MORAI class 순서는 Vehicle=0, Pedestrian=1, Obstacle=2이므로 Obstacle heatmap에는 사실상
local suppression이 없고, final rotated box NMS도 꺼져 있다. 이 설정은 위 duplicate를
만들 수 있는 구체적인 코드상 조건이다.

관측 box pair에는 rotated IoU `0.5` 기준을 적용하면 모두 같은-object duplicate로
제거할 수 있었다. 다만 production 변경 전에는 class-aware rotated NMS를 offline 저장
detection에 먼저 적용해 Obstacle recall, 인접한 복수 장애물 분리와 tracker ID 연속성을
함께 검증해야 한다. 정적 장애물이 실제로 약 `0.54 s` 보이지 않은 구간은 NMS와 별개로
남으므로 track lifecycle 보강도 두 번째 단계에서 따로 평가한다.

따라서 이 성공 회피 run은 size가 정상이어도 viewpoint에 따른 center 이동과 근접
관측 공백 때문에 association이 깨질 수 있음을 보여준다. 이전 position 5의 심한
size 붕괴와는 별개의 tracker 강건성 문제이며 planning에는 중복 객체와 바뀐 ID가
동시에 전달될 수 있다.

## Position 5: perception/tracking failure가 실제로 존재한다

장애물이 가까워지는 동안 tracker ID가 `22 -> 23 -> 24 -> 23 -> 24 -> 25`로
바뀌었다. 최대 관측 공백은 약 0.43 s였으며, 폭과 길이도 각각
`0.460--0.842 m`, `1.076--2.225 m` 범위로 크게 변했다. UNKNOWN stationary 정책
때문에 발행 속도는 0이었지만 frame 간 center 이동률 p95는 약 `6.02 m/s`였다.

따라서 position 5는 detector의 위치·형상 변동과 관측 공백이 association 실패로
이어진 사례다. 다만 planning 측에서도 회피 candidate를 한 번도 만들지 않았으므로,
ID fragmentation만으로 전체 실패를 설명할 수는 없다.

## Position 4: tracking이 안정적이어도 회피에 실패한다

초기 confirm 과정의 ID 변경 한 번 뒤에는 동일 ID가 약 45초 유지됐다. box 크기와
중심 위치도 안정적이었다. 그럼에도 차량은 obstacle center가 base_link에서 약
3.53 m 앞에 있을 때 약 `8.17 m/s`로 충돌했다. IONIQ의 base_link-to-front 거리가
약 3.845 m인 점과도 일치한다.

충돌 직전 최종 `/planning/trajectory`는 장애물 중심에서 불과 약 `0.50 m` 떨어진
점을 포함했다. 차량과 barricade 형상을 고려하면 명백한 충돌 경로다. 동시에
`/planning/path_candidate/static_obstacle_avoidance`는 전 구간 비어 있었고,
static-obstacle debug marker는 해당 객체를 `unavoidable_target`으로 표시했다.

따라서 position 4 실패는 perception ID 또는 box jitter 탓으로 볼 수 없다. 이번
설정에서는 planner가 객체를 인식하고도 회피 가능한 candidate를 만들지 못해 직진
경로를 유지한 것이 직접 원인이다. planning margin, drivable area, target selection,
`unavoidable` 판정 조건을 우선 점검해야 한다.

## RViz의 ego 뒤 주황색 짧은 선

이 선은 object prediction이나 Map-CV의 유령 경로가 아니다. 풀스택 RViz의 planning
trajectory visualizer가 `/planning/trajectory` 전체를 속도별 색상으로 그리는데, 매
trajectory에 ego 뒤쪽 약 5 m, 대략 50 point가 남아 있다. 이 중 저속 point가
주황색으로 렌더링된다.

- position 5: 뒤쪽 구간 195/195 frame, 주황 저속 구간 126 frame.
- position 4: 뒤쪽 구간 612/612 frame, 주황 저속 구간 511 frame.
- obstacle prediction은 두 run 모두 stationary 단일 path였고 secondary path는 없었다.

marker 자체는 경량 bag에 없지만 원본 `/planning/trajectory`가 있으므로 동일하게
재구성할 수 있다. 이는 visualization artifact이며 planner나 controller에 다시
입력되는 경로는 아니다. 시각화 수정 시에는 ego에 가장 가까운 trajectory index보다
앞쪽의 과거 segment를 그리지 않도록 clip하면 된다.

## 다음 분리 실험

1. position 4를 planning/control 기준 사례로 사용한다. stable track인데도
   `unavoidable`이 되는 조건과 candidate가 비는 이유를 조사한다.
2. position 5는 perception/tracker 강건성 사례로 사용한다. association gate만 늘리기
   전에 detector center/shape jitter와 0.43 s 공백을 함께 다룬다.
3. 같은 position에서 `od_gt`와 BEVFusion을 녹화할 때 candidate path, planning factor,
   final trajectory 및 CollisionData를 그대로 보존해 planner 입력 차이부터 비교한다.
4. 주황색 선 clip은 진단 표시만 바꾸는 별도 수정으로 취급하고 회피 성능 수정과
   섞지 않는다.
5. Position 04 저장 detection에 Obstacle class rotated NMS를 먼저 적용해 duplicate와
   다중 ID가 사라지는지 검증한다. NMS와 track miss tolerance를 한 번에 바꾸지 않는다.

## 2026-09-25 팀장님 planviz bag 3개 추가 분석

다음 세 bag은 모두
`/Object_topic`, localization, detection, tracking, prediction과 원본 카메라/LiDAR를
포함해 perception/tracker 원인 분석에 사용할 수 있다. 다만 첫 번째만 정적 차량
`2016_Hyundai_Sonata`이고 나머지 두 개는 `Concrete_Barricade01`이다.

- `rosbags/eval/perception/static_obstacle_tracking/260924_201621_teamlead_planviz_static_vehicle_sonata_id_case_01.bag`
- `rosbags/eval/perception/static_obstacle_tracking/260924_235953_teamlead_planviz_concrete_barricade_id_case_01.bag`
- `rosbags/eval/perception/static_obstacle_tracking/260925_000105_teamlead_planviz_concrete_barricade_id_case_02.bag`

### 정적 Sonata (`201621`)

- 약 32.3 m에서 크기 `0.68 x 1.43 x 0.98 m`인 작은 UNKNOWN track이 3회, 0.067초
  나타났다.
- 1.57초 뒤 29.5 m에서 정상 CAR track이 새 ID로 생성됐다.
- 정상 CAR ID는 6.9 m까지 12.67초, 353개 tracking message 동안 한 번도 바뀌지
  않았다.
- 정상 CAR box 중앙값은 약 `4.97 x 2.08 x 1.55 m`로 catalog Sonata
  `4.86 x 2.12 x 1.50 m`와 가깝다.
- detector에도 rotated IoU 0.5 이상의 same-class duplicate는 없었다.

따라서 두 ID가 존재하기는 하지만 이것은 회피 중 정상 차량 track의 ID switching이
아니다. 먼 거리의 작은 UNKNOWN 오검출/초기 획득 뒤 정상 CAR track이 별도로 시작된
형태다. 팀장님이 설명한 side-by-side 초근접 재획득 문제를 입증하려면 차량이 더 가까운
거리에서 사라졌다 다시 잡히는 구간을 포함한 bag이 추가로 필요하다.

### Barricade (`235953`, `000105`)

| bag | matched track ID | 직접 ID 교체 | same-class duplicate pair | 최대 detection 공백 |
| --- | ---: | ---: | ---: | ---: |
| `235953` | 7 | 6 | 6 | 0.164 s |
| `000105` | 5 | 5 | 6 | 0.999 s |

두 bag 모두 center error가 association gate 5 m를 넘어서 바뀐 것이 아니다. 새 ID가
생긴 순간의 corrected-GT center error는 대부분 약 `0.18--0.98 m`였다. 대신 같은
UNKNOWN 장애물에 대해 center가 `0.03--0.29 m`밖에 떨어지지 않고 rotated IoU가
`0.67--0.93`인 box가 동시에 여러 개 발행됐다. `000105`에서는 3.1 m 초근접 frame에
세 box가 동시에 겹쳤고, tracker ID가 두 개 사이에서 연속으로 왕복했다.

이 추가 자료도 detector duplicate가 unmatched detection을 새 track으로 만드는 것이
Barricade ID fragmentation의 주원인이라는 결론을 지지한다. 따라서 해결 순서는
class-aware rotated BEV NMS를 먼저 적용하고, 그 뒤에 남는 실제 detection 공백만
track lifecycle로 보강하는 것이 맞다.

정량 결과 원본은
`artifacts/evaluation/bevfusion/teamlead_vehicle_id_switch_260925/`에 생성했다.

## 2026-09-25 A: detector duplicate suppression 적용 결과

Tracker lifecycle 변경과 분리해 첫 단계 A로 BEVFusion ROS bridge 출력에
class-aware rotated BEV NMS를 적용했다.

- 위치: class별 confidence threshold 적용 직후, canonical detection topic 발행 전.
- 기준: 같은 class이고 rotated BEV IoU가 `0.50`보다 큰 box 중 confidence가 가장 높은
  하나만 유지한다.
- 기본값: `base3_online.launch`에서 활성화. 기존 E25 checkpoint, Docker inference,
  ROS topic/message 계약은 바꾸지 않는다.
- 진단: `/perception/bevfusion/status`의 `nms_suppressed_total`로 누적 제거 수를 확인한다.

13개 정적 장애물·차량 저장 detection을 검사했을 때 정상 Sonata 5개 run에서는 제거가
한 건도 없었다. 반면 문제 barricade run에서는 실제 겹친 frame만 제거됐다.

| bag | 제거 box / duplicate frame |
|---|---:|
| manual position 04 | `5 / 5` |
| teamlead barricade case 01 | `7 / 7` |
| teamlead barricade case 02 | `5 / 4` |

기존 E25 8-scenario 저장 detection 회귀검사에서도 Vehicle 네 bag은 제거 0건이었다.
Obstacle bag 두 개에는 각각 `43`, `21`개의 겹친 box가 있었으며 NMS 후 현재 evaluator의
center AP/F1은 감소하지 않았다.

| E25 obstacle bag | center AP raw -> NMS | F1@1m raw -> NMS |
|---|---:|---:|
| 왕복 | `60.68 -> 61.14%` | `0.855 -> 0.883` |
| 다중 asset | `68.76 -> 68.84%` | `0.829 -> 0.843` |

이 수치는 양쪽 모두 같은 legacy calibrated GT로 다시 계산한 NMS A/B이며, 과거 보고서의
다른 GT 계약 수치와 직접 섞지 않는다. 목적은 recall 손실 여부를 같은 조건에서 확인하는
것이다.

동일 EKF tracker를 2배속으로 raw detection과 NMS detection에 각각 재실행한 결과는 다음과
같다. UUID 값 자체가 매 실행마다 달라지므로 unique ID 수, 동시 track frame과 track
길이를 비교했다.

아래 표는 다른 class의 짧은 오검출을 섞지 않고, 문제 장애물과 같은 `UNKNOWN` track만
집계한 결과다.

| bag | 입력 | UNKNOWN unique ID | 동시 복수 UNKNOWN frame | 최장 UNKNOWN track frame |
|---|---|---:|---:|---:|
| manual position 04 | raw | 6 | 6 | 32 |
|  | NMS | **2** | **0** | **45** |
| teamlead case 01 | raw | 8 | 12 | 23 |
|  | NMS | **3** | **2** | **39** |
| teamlead case 02 | raw | 5 | 5 | 35 |
|  | NMS | **2** | **0** | **55** |

따라서 A는 duplicate detection이 unmatched detection으로 남아 새 ID를 만드는 경로를
실제로 차단한다. 다만 NMS 적용 후에도 unique ID가 2--5개 남는다. 이는 서로 겹친
동시 box 문제가 아니라 detection 공백, 근접 형상 붕괴와 재획득 문제이므로 두 번째
단계 B의 track lifecycle/association 대상으로 남긴다. A 결과만으로 B까지 해결됐다고
판정하지 않는다.

host Python microbenchmark에서 평균 추가 시간은 detection 5/20/100개일 때 각각
약 `0.005/0.068/1.83 ms`였다. 실제 production 검출 수는 대개 이보다 훨씬 적으며,
12 Hz inference budget에 유의미한 지연을 추가하지 않는다.

재현 수치는
`artifacts/evaluation/bevfusion/static_obstacle_rotated_nms_260925/`에 저장했다.

## 2026-09-25 B: confirmed-track lifecycle 및 duplicate birth 보강

A를 통과한 동일 NMS detection을 고정 입력으로 두고 tracker만 B off/on으로
재실행했다. B는 두 문제를 분리해 해결한다.

1. confirmed track 삭제를 최근 frame 개수가 아니라 message stamp 기준 bounded
   coasting으로 바꿨다. moving track은 `0.50 s`, UNKNOWN 및 2 m/s 이하 CAR/TRUCK은
   `1.00 s`까지 유지한다. Tentative track의 strict 삭제 정책은 그대로이며 pedestrian은
   1초 정적 lifetime 대상이 아니다.
2. A의 rotated NMS를 통과한 low-IoU shape outlier가 있을 때, 같은 frame에 이미
   association된 mature large-object track 중심 `0.75 m` 안의 유사 크기 unmatched
   detection은 새 ID를 만들지 않는다. Association 전체를 먼저 갱신하고 birth 후보를
   나중에 처리해 detection 배열 순서에도 의존하지 않게 했다.

이는 전역 association distance `5 m`를 늘리는 변경이 아니다. 서로 다른 인접 객체를
합칠 위험을 제한하기 위해 mature/associated 조건, pedestrian class 경계, 최소 최대변
`1.0 m`, 최대변 비율 `2.0`을 함께 사용한다.

| bag | A only UNKNOWN ID | A+B UNKNOWN ID | A only 복수 frame | A+B 복수 frame |
|---|---:|---:|---:|---:|
| manual position 04 | 2 | **1** | 0 | **0** |
| teamlead case 01 | 3 | **1** | 2 | **0** |
| teamlead case 02 | 2 | **1** | 0 | **0** |

- manual position 04의 `0.554 s` detector gap에서 같은 ID가 재사용됐다.
- teamlead case 02의 `0.988 s` gap도 UNKNOWN stationary 한도 안에서 같은 ID를
  유지했다.
- teamlead case 01의 약 `0.25--0.31 m` center 차이 low-IoU shape outlier는 새 track을
  만들지 않았다.

정상 정적 Sonata 회귀도 함께 수행했다.

| regression bag | B off | B on |
|---|---:|---:|
| teamlead Sonata 정상 CAR | CAR ID `1`, 복수 frame `0` | CAR ID `1`, 복수 frame `0` |
| static vehicle position 01 | CAR ID `1`, 복수 frame `0` | CAR ID `1`, 복수 frame `0` |

첫 Sonata bag의 원거리 작은 UNKNOWN은 B에서 bounded coasting되어 보이는 시간이
늘었지만 정상 CAR와 동시에 중복되거나 합쳐지지 않았고, 정상 CAR의 ID/길이/최대 속도는
off와 동일했다. 이는 정적 UNKNOWN miss tolerance의 의도된 trade-off다. 향후 현장에서는
1초 이상 지속되는 false UNKNOWN 비율도 함께 감시한다.

단위 테스트는 mature stationary gap 재획득, 1초 초과 만료, input-order-independent
duplicate birth 억제, 인접한 별개/작은 객체 보존, legacy frame-count mode를 포함하며
package 전체 `40 tests`가 통과했다. 재현 출력과 요약은
`artifacts/evaluation/bevfusion/static_obstacle_tracker_lifecycle_b_260925/`에 저장했다.
