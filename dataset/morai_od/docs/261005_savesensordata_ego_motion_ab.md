# MR SaveSensorData 정지/이동 Ego 소량 A/B

상태: EVIDENCE / 2026-10-05. Base5 정지 single-frame 수집의 제한적 지지 근거.
실시간 UDP scan의 deskew 판정과 분리한다. 10K 본 수집은 하지 않았다.

## 결론과 적용 범위

현재 MR의 SaveSensorData에서 정지2장, 실제 약27.7km/h 직진2장,
약48.7km/h 직진2장을 캡처했다. 이동 조건에서도 정적 CargoBox와
Concrete_Barricade01의 점군이 동기 annotation 주변에 유지됐으며,
속도 증가에 따라 커다란 scan 내부 형상 왜곡이 생겼다는 근거는 찾지 못했다.
순간 snapshot 가설과 일치하고 Base5 정지 배치 수집을 계속할 근거는 된다.

다만 renderer의 정확한 ray acquisition 방식은 증명하지 않았다. 같은 자세/시점의
완벽한 정지·이동 pair가 아니며 yaw-only 표면 지표, 시점·가시성 변화가 남는다.
실제60/100km/h, 급회전, 동적 객체, Camera motion blur는 이번에 검증하지 않았다.
정지·이동 입력이 완전히 동일하거나 실시간 UDP와 분포가 같다는 결론은 아니다.

## 세팅과 실행

- ROS master + rosbridge, `ego_network_mr_base5_savesensordata_local.json`.
- ROS Ego/Object/ScenarioLoad/ctrl_cmd/EventCmd/SaveSensorData만 사용.
- Capture Mode와 Camera4/LiDAR Instance annotation 저장 활성화.
- 센서 UDP receiver, BEVFusion, planning/controller 전체 스택은 실행하지 않았다.
- 검증된 기존 MR template의 직선 위치를 기준으로, 경로 좌우4m에 scale1
  CargoBox3개와 Concrete_Barricade01 3개를 고정 배치했다. 동적 spawn은 제거했다.
- Ego 초기속도0, 제어권 확보 후 velocity command로 직진했다. 정지 prime으로
  writer/annotation을 먼저 확인하고 측정 요청에서는 제동하지 않았다.
- 요청속도30/60km/h를 그대로 달성 속도로 보고하지 않는다. 짧은 접근 거리에서
  실제 요청 시점 속도는 아래와 같았다. 처음의60km/h 목표 달성 gate 실패도 보존했다.
- 마지막에 brake 명령을 발행하고 실제 Ego 속도가 거의0인 것을 확인했다.
  임시 MR Scenario/EgoNetwork sidecar를 제거했으며 사용자 원본은 변경하지 않았다.

## 실측

각 측정은31개 파일, Camera4/LiDAR/Instance annotation 저장이 완결됐다.
요청 전후 Ego 표본과 timestamp를 저장했다. SaveSensorData 중 telemetry gap이
약0.58–0.61초 발생했다. wall 저장시간 약1.03–1.08초 전체를 simulation 진행시간으로
가정하지 않는다. gap 원인이 렌더링 blocking/일시 정지/발행 지연 중 무엇인지는 미확정이다.

| 조건 | 요청 시점 실제속도 | 저장 전후 전진량 | baseline과 캡처 시점 위치 차이 |
|---|---:|---:|---:|
| 정지1 | 약0km/h | 0m | 기준 |
| 정지2 | 약0km/h | 약0.00002m | 약0m |
| 이동30-1 | 27.764km/h | 4.175m | 0.216m |
| 이동60-1 | 48.731km/h | 7.818m | 0.183m |
| 이동30-2 | 27.744km/h | 4.339m | 0.101m |
| 이동60-2 | 48.667km/h | 6.841m | 0.793m |

속도 단위를 혼동하지 않는다. 이 시험의 Ego velocity.x는 m/s, CtrlCmd velocity는 km/h다.

## 기하학 평가

저장된 Intensity/Instance cloud의 XYZ 일치를 확인하고 Instance nonzero mask로
도로·배경을 제외했다. mask 값1을 UID로 해석하지 않는다. 6개 annotation center의
대응으로3D 강체 정렬을 구했으므로 ROS 수신 pose를 정확한 캡처 pose로 가정하지 않는다.
annotation rigid-fit residual은 평균 약0.0000003–0.0000024m였지만, 이는 동일 정적
annotation의 강체 일관성이지 pointcloud 정확도 자체가 아니다.

측정6장의 라벨 점은 각319–372개였다. 모두6GT의 yaw-only box+20cm 안에 포함됐다.
20cm mask로 outlier를 숨기지 않았는지 자르지 않은 전체 라벨 cloud로도 확인했다.

대표 CargoBox UID9602의 nearest-GT-face distance:

| 조건 | point 수 | p50 | p95 |
|---|---:|---:|---:|
| 정지1 | 64 | 5.4mm | 24.4mm |
| 정지2 | 64 | 5.4mm | 24.4mm |
| 27.764km/h | 60 | 9.3mm | 24.5mm |
| 48.731km/h | 61 | 5.2mm | 25.7mm |
| 27.744km/h | 58 | 6.2mm | 24.5mm |
| 48.667km/h | 65 | 12.5mm | 23.8mm |

Concrete_Barricade01에는 asset 형상·annotation margin 때문일 수 있는 고정적인 face 차이가
있어 zero residual을 기대하지 않는다. 좌우 관측 면·ray sampling·pitch/roll 영향도 남는다.
특히 마지막 고속 frame은 위치차0.793m와 가시성 변화가 있어, 한 박스가 보이지 않게 되고
다른 박스가 나타났다. 전체 점 NN p95 약4.97m를 속도 왜곡이나 검증 성공으로 단정하지 않는다.
다른 이동 frame의 전체 점 NN p95는 약0.029–0.134m지만 시점·sampling 차이를 포함한다.

## 검증 종료 결정

현재 범위에서 Ego 정지/직진 이동 A/B 검증을 마무리하고, Base5의 Ego 정지 배치 기본 설계를 유지한다. 같은 시험의 추가 반복을 Base5 생성기 작업의 필수 선행 조건으로 두지 않는다. 다만 동적 객체의 실제 동작 중 캡처와 생성기 smoke test는 별도 검증 대상이며, 이번 결과가 본 수집 전체의 승인이나 고속·회전 조건의 검증 완료를 의미하지는 않는다.

## Raw / 재현 / 남은 gate

Git 제외 근거 경로:
`planner_logs/analysis/savesensor_motion_ab_261005/`

- `probe.py`: 최종 probe. 초기 gate 실패는 failure JSON과 warmup에 남는다.
- `captures/`: 측정6frame, 각각 telemetry와 sensor artifact.
- `warmup/`: writer prime용. 학습용 측정 frame 수에 포함하지 않는다.
- `summary.json`, `geometry_report.json`, `cargo_box_local_xy.png`.
- `analyze.py`: annotation 기준 재집계. scatter의 내부 점에는 윗면 return도 포함되므로
  XY 투영만 보고 박스가 휘었다고 판정하지 않는다.

재집계: `/usr/bin/python3 planner_logs/analysis/savesensor_motion_ab_261005/analyze.py`.
probe는 실제 연결에서 scenario/control을 변경하므로 연결·안전 확인 없이 재실행하지 않는다.

Base5는 Ego 정지를 유지하고 동적 actor의 실제 movement/animation 동기 캡처와
component-preserving converter를 다음에 검증한다. Camera blur나 고속 회전을 검증할
필요가 생기면 별도 gate로 추가한다. 이번 결과만으로 UDP deskew의 ON/OFF를 바꾸지 않는다.
