# MR VLP16 출력 거리 제한 — EVIDENCE

검증일: 2026-10-05. 현재 대회용 MR 로컬 런처, VLP16, UDP 센서 출력 기준.

## 결론

**현재 MR/VLP16 환경에서 실제 LiDAR 출력에 약100m 거리 제한이 관측됐다.**
Sensor JSON의 `fireDistance`를150으로 변경하고 MR에서 저장해도 실제 return 범위는
늘어나지 않았다. 향후 데이터 수집·인지 설계는 이 환경의 LiDAR 상한을100m로 취급한다.

이는 실측 출력에 관한 결론이다. MORAI 내부 구현 원인, 모든 런처/모델의 공통 제한,
또는 대회 규정상의100m 제한을 확인했다는 뜻은 아니다. 원인 추측을 더 진행하지 않는다.

## 설정과 관측

- 원본 local/LAN Sensor JSON: `fireDistance=100.0`.
- 별도 시험본: `configs/sensors/Sensor_competition_local_lidar150_test.json`.
  원본 대비 변경은 LiDAR `fireDistance=150.0` 한 항목뿐이다.
- MR에 시험본을 로드하고 저장한
  `SaveFile/Sensor/25.S4.MolitComp03/SensorInfo_2023_Hyundai_Ioniq5.json`에서도150 유지.
  사용자 저장 이름150은 `SaveFile/Scenario/R_KR_PR_K-city_2025/150.json`에 확인됐다.
- 실제 ROS 변환기의 `/lidar/lidar_transform/max_range=130.0` 확인.
  100m 초과 return 유무는 PointCloud2뿐 아니라 **변환 전 Velodyne UDP packet**의
  거리값도 읽어 확인했다. ROS pointcloud range filter가100m에서 잘라낸 결과가 아니다.

| 측정 | 결과 |
|---|---|
| 150 설정 후 최초 pointcloud60프레임 | 약12.26Hz, 최대100.014m, 100.5m 초과0점 |
| 최초 원본 UDP 단일 scan | 최대99.89m, 100.5m 초과0 return |
| 원거리 차량7대 배치 후10초 | 원본103 scan, 약10.32Hz, 최대99.446m, 100.5m 초과0 return |
| 원거리 차량의 실제 Ego 기준 XY 거리 | 109.884–113.541m; 사용자 배치 예상120m와 구분 |
| 가까운 차량 추가 후5초 | cloud/raw 각각50 scan, 원본 최대98.702m, 100.5m 초과0 return |
| 추가 차량 | `2016_Kia_Soul(EV)`, UID17, Ego 기준55.578m |

추가 차량의 예상 앞면 위치 부근에7개 point가 모였다.
LiDAR 좌표에서 대략 x54.00–54.28m, y−0.74–0.97m, z−0.95m로 관측됐다.
이것은 broad vehicle-region 대조이며 정식 geometric-BBox 정합/검출 Recall 평가가 아니다.
109.9–113.5m 차량7대의 예상 영역에는 point가 없었다.

반경100.5m를 초과 여부 기준으로 사용해100m 경계의 수치/캘리브레이션 오차를
실제 거리 확장으로 오해하지 않았다. 약100.014m라는 값은150m 확장의 증거가 아니다.

## 수집·모델 계약에 미치는 영향

- Base5 차량20–60m 수집 계획은 유지한다. 80–100m는 별도 sparse/가시성 audit 대상이다.
- 100m가 모든 크기/방향 객체를 안정적으로 검출할 수 있다는 보장은 아니다.
- 100m 밖 인지는 카메라 등의 별도 입력/설계가 필요하며 현재 BEVFusion의 기능으로
  이미 달성했다고 주장하지 않는다. 120/150m를 LiDAR 학습 목표로 자동 확대하지 않는다.
- E25의±30m 학습/출력 ROI와 센서100m 제한은 서로 다른 계약이다.
- production Sensor JSON, UDP alias, 모델 및 threshold는 이번 시험에서 변경하지 않았다.
- 시험본을 쓰고 있다면 다음 production 실행 전 원래100m Sensor JSON을 다시 로드한다.
  이 문서 작성 시 원격으로 MR 설정을 바꾸거나 자동 복원하지 않았다.

## 증거 보존 범위

이 수치는 live read-only 진단의 console 결과를 정리한 것이다. 별도 rosbag/원본 패킷
파일을 녹화하지 않았으며 존재하지 않는 재생용 bag 경로를 제공하지 않는다.
