# Motion-focused 100-frame deskew pilot (`2026-08-21`).

## 목적.

- SaveSensorData와 lightweight timing sidecar의 실제 수집 계약을 검증함.
- 동일한 painted LiDAR 원본으로 raw 3-sweep과 방위각 기반 Ego deskew 3-sweep을
  생성함.
- sweep 간 Ego 정합과 scan 내부 근사 deskew의 효과를 분리해 평가함.
- 9D `vx,vy` target과 OpenPCDet forward/backward 계약까지 smoke 검증함.

## 최종 100-frame 구성.

| Family | Clips | Frames |
| --- | ---: | ---: |
| Highway cruise/overtake | 4 | 40 |
| Roundabout circulation | 3 | 30 |
| Highway merge cruise | 2 | 20 |
| Crosswalk pedestrian | 1 | 10 |
| 합계 | 10 | 100 |

- Primary run은 10개 clip 중 9개, 90 frame이 성공함.
- `dp100_dhc001`은 NPC collision로 실패함.
- 독립 seed의 reserve highway clip 1개로 10 frame을 대체함.
- ROS 연결 또는 SaveSensorData artifact 누락 실패는 없었음.
- primary/replacement validator는 모두 errors 0, status pass임.
- model-neutral split은 clip 단위 `60/20/20` frame임.

Capture root는 다음과 같음.

- `/home/luv/morai_cap/260821_100712_motion_deskew_pilot100`.
- `/home/luv/morai_cap/260821_101153_motion_deskew_pilot100_replacement`.

## Timing metadata 확인.

- frame마다 Camera 4대와 LiDAR artifact가 완전하게 저장됨.
- Ego/Object 연속 이력은 실제 capture profile에서 약 20 Hz로 저장됨.
- capture trigger와 nearest Ego/Object sample 차이는 smoke에서 약 5~8 ms였음.
- Capture Mode에서는 `/clock`, `/sensor/lidar1`, camera compressed ROS stream이
  발행되지 않음.
- 따라서 exact point time은 확보하지 못했고, scan 내부 deskew는 기존처럼
  `1,200 azimuth x 16 channel` 순서와 66.7 ms scan period에 기반한 근사임.

## Dataset 변환.

- PointPainting 9-feature OpenCV dataset: 100 frame.
- temporal target은 각 10-frame clip의 첫 두 frame을 제외한 80 frame임.
- temporal split은 train/val/test `48/16/16`임.
- 9D target은 Vehicle 120개와 Pedestrian 10개이며 누락된 velocity box는 0개임.
- Obstacle은 이번 motion-focused pilot에 포함하지 않았으므로 production 학습 성능을
  판단하는 데이터셋이 아님.

## 기하 A/B 결과.

정적 semantic-background point의 current/past sweep symmetric nearest-neighbour
distance를 80 target 전체에서 비교함.

| 지표 | 정합 전 | Raw sweep 정합 | Azimuth Ego deskew |
| --- | ---: | ---: | ---: |
| mean | 0.266596 m | **0.159364 m** | 0.160197 m |
| p50 | 0.160549 m | **0.057222 m** | 0.057735 m |
| p90 | 0.485631 m | **0.233624 m** | 0.236431 m |
| p95 | 0.611872 m | **0.480141 m** | 0.485212 m |

- sweep 간 Ego 정합은 p50을 약 64.4%, p90을 약 51.9% 줄여 명확히 유효함.
- scan 내부 방위각 근사 deskew는 raw 정합보다 mean/p50/p90/p95가 모두 소폭 나쁨.
- frame p50 개선 비율도 46.25%로 절반 미만임.
- 8 m/s 이상 구간은 deskew p50이 약 0.21 mm 좋아졌지만 p90은 약 11.9 mm,
  p95는 약 2.0 mm 나빠져 tail 개선 근거가 없음.
- point 좌표 변화 자체는 p50 0.126 m, p95 0.697 m로 작지 않음.

### 일반적인 deskew인데도 이번 근사 방식이 좋아지지 않은 이유.

Scan 내부 Ego-motion deskew는 point별 측정 시각과 그 시각의 Ego pose를 알고 있을
때 LiDAR odometry, SLAM, 고속 로보틱스에서 일반적으로 사용하는 처리다. 그러나
모든 3D object detector에 임의의 근사 deskew를 적용하면 AP가 자동으로 향상된다는
뜻은 아니다.

이번 pilot의 `Azimuth Ego deskew`는 exact deskew가 아니다.

- SaveSensorData XYZI에는 point별 timestamp가 없다.
- `1,200 azimuth x 16 channel`, 66.7 ms 회전 주기, clockwise 저장 순서를 가정해
  point time을 역산했다.
- 실제 scan 시작 방위각, 저장 순서, header가 scan 시작·중앙·끝 중 어느 reference를
  뜻하는지는 센서 인터페이스에서 보장되지 않는다.
- scan 동안 Ego가 일정 속도와 일정 yaw rate로 움직인다고 근사한다.
- Ego motion만 보정하므로 움직이는 NPC와 보행자 자체의 scan 내부 motion distortion은
  남는다.

3-sweep의 frame 간 Ego 이동은 약 0.15/0.29초 동안 누적되므로 크고, 저장된 frame별
Ego pose로 직접 정합할 수 있다. 반면 한 scan 내부 구간은 약 0.067초로 짧아 개선
가능한 잔여 오차가 더 작다. 이 작은 오차에 point-time 근사 오차가 섞이면서 raw
sweep 정합보다 순이익이 나오지 않았다.

따라서 이번 결과는 deskew 일반을 기각한 것이 아니다.

- `정합 전 -> Raw sweep 정합`의 큰 개선은 A raw 3-sweep에 이미 포함한다.
- `Raw sweep 정합 -> Azimuth Ego deskew`가 악화된 결과만 현재 B 근사 방식을
  기각하는 근거다.
- exact point time, scan reference timestamp와 구간 Ego pose가 제공되면 exact
  deskew는 다시 검토할 수 있다.
- 현재 확인한 SaveSensorData와 MORAI ROS PointCloud2에는 point별 time field가 없어,
  현 대회 입력 계약 그대로는 exact point-level deskew가 불가능하다. MORAI/driver가
  raw packet firing offset 또는 point time을 추가 제공할 때만 이 제한이 해소된다.

## 학습 계약 smoke.

Raw와 deskew 각각에서 다음을 통과함.

- OpenPCDet train dataset 48 target load.
- 3-sweep 10-feature point 입력.
- 9D box와 class ID를 합친 GT shape `[...,10]`.
- non-zero moving velocity target.
- finite loss와 velocity output channel non-zero gradient.
- inference/serializer의 `N x 9` prediction 유지.

이는 학습 가능성을 검증한 것이며, 48 train target의 AP 차이를 production 근거로
해석하지 않음.

## 결론.

- 3-sweep을 사용할 경우 sweep 간 Ego pose 정합은 유지할 가치가 큼.
- 현재 SaveSensorData만으로 가능한 scan 내부 방위각 근사 deskew는 채택하지 않음.
- 100-frame full A/B 학습은 geometry gate가 실패했고 표본도 너무 작아 수행 이득이
  낮음. forward/backward smoke까지만 보존함.
- production의 `lidar_deskew_mode=off`를 유지함.
- exact sensor point time을 제공하는 capture profile을 확보하거나, 더 신뢰할 수 있는
  scan reference 연결이 가능해질 때만 scan 내부 deskew를 다시 검토함.
