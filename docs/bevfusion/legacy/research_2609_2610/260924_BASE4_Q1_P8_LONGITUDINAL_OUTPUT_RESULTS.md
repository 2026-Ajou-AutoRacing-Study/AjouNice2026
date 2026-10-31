# Base4 Q1-P8 Velocity Frame 및 종방향 Output 결과

> 날짜: 2026-09-24.
> 상태: Frame 계약 수정, 종방향 결합 가능성 확인, production gate는 미통과.

## 목적

P7은 detector 자체의 speed MAE를 낮췄지만 detector `vx, vy`를 초기 1초 동안 EKF에
직접 넣으면 lateral error와 stable Map-CV가 악화했다. P8은 다음 두 원인을 분리했다.

1. detector velocity 좌표계가 tracker map state와 일치하는가.
2. detector의 유효한 초기 종방향 정보만 위치 tracker를 오염시키지 않고 쓸 수 있는가.

## 발견한 frame 계약 오류

BEVFusion target은 절대 객체 velocity를 LiDAR/Ego 축으로 표현한다. 표준
`DetectedObjects` 메시지는 이를 object body twist로 담고, tracker callback이 다시 LiDAR
축 `vx, vy`로 복원한다. 이후 tracker는 position과 yaw를 LiDAR에서 map으로 변환했지만
velocity vector는 회전하지 않았다.

두 개발 bag의 Ego heading 중앙값은 약 `-88 deg`다. 따라서 LiDAR 전방 `+x` 속도를 map
`vx`로 그대로 쓰면 실제 map 종방향과 거의 직교한다. P7 mode 3에서 lateral error가
커진 원인 중 하나다.

P8은 `TransformMeasLiDAR2Global`에서 velocity도 Ego/LiDAR yaw로 map frame에 회전한다.
Base4 target은 상대속도가 아닌 절대속도이므로 Ego velocity를 더하지 않는다. Production
mode 0은 detector velocity를 사용하지 않으므로 동작이 바뀌지 않는다.

## Mode 4: 초기 종방향 output-only 결합

새 mode 4는 detector velocity로 EKF state와 covariance를 수정하지 않는다.

- Vehicle/Truck만 대상이다.
- detector map velocity를 track heading에 투영한다.
- tracker의 횡방향 속도는 그대로 두고 종방향 출력만 detector 쪽으로 보정한다.
- track age `0--1 s`, detector observation age `<=0.25 s`에서만 작동한다.
- longitudinal innovation `<=5 m/s`만 허용한다.
- 1초 경계의 불연속을 막기 위해 blend를 track age에 따라 `1→0`으로 선형 감쇠한다.
- EKF association, position, internal velocity와 다음 prediction state는 position-only다.

기능 기본값은 mode 0이며 mode 4는 명시적으로 켜야 한다.

## 단위·빌드 검증

- tracker build PASS.
- tracker test `28 tests / 0 errors / 0 failures`.
- mode 4에서 detector observation은 저장되지만 EKF `VX/VY`가 직접 초기화되지 않는
  단위시험을 추가했다.
- launch XML과 evaluation shell syntax 검증을 통과했다.

## 개발 rosbag 결과

P7 detection, threshold `0.40/0.15/0.10`과 같은 position-only baseline을 사용했다.

### Tracking

| Bag | 설정 | F1 | center p50 | speed MAE | 2D AVE | long / lateral MAE |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| highway1 | position-only | **`0.9804`** | `0.280` | `0.451` | `0.595` | `0.449 / 0.272` |
| highway1 | P8 tapered | `0.9797` | **`0.280`** | **`0.444`** | **`0.589`** | **`0.442 / 0.270`** |
| highway merge | position-only | **`0.8855`** | `0.590` | `0.891` | `1.047` | `0.896 / 0.325` |
| highway merge | P8 tapered | `0.8852` | **`0.587`** | **`0.857`** | **`1.009`** | **`0.861 / 0.317`** |

P8은 두 bag 모두 speed, 2D AVE와 center를 개선하거나 유지했다. 개선 폭은 highway1
AVE 약 `1.0%`, 합류 약 `3.6%`로 작다.

### Map-CV

| Bag | 설정 | all FDE 1/2/3s | CV-stable minFDE-K 1/2/3s |
| --- | --- | ---: | ---: |
| highway1 | position-only | `0.700 / 1.883 / 3.627` | `0.374 / 0.588 / 0.815` |
| highway1 | P8 tapered | `0.704 / 1.902 / 3.654` | `0.375 / 0.592 / 0.794` |
| highway merge | position-only | `1.621 / 3.317 / 5.620` | `1.246 / 1.885 / 2.600` |
| highway merge | P8 tapered | **`1.593 / 3.182 / 5.368`** | `1.269 / 1.890 / 2.584` |

합류 all-FDE와 양쪽 stable 3초는 좋아졌지만 highway1 all-FDE와 stable 1/2초는 소폭
악화했다. 두 bag의 모든 gate가 같은 방향으로 개선돼야 한다는 production 기준에는
미달한다.

## 판정

1. LiDAR→map detector velocity 회전 수정은 올바른 frame 계약으로 채택한다.
2. P8 mode 4는 detector와 position tracker의 상보성을 실제 ROS pipeline에서 확인한
   연구 옵션으로 보존한다.
3. Production은 계속 mode 0 position-only다.
4. 기존 두 development bag에서 blend/window/innovation을 더 튜닝하지 않는다. 작은
   차이를 반복 최적화하면 bag 편향에 맞출 위험이 크다.
5. 다음 P9는 새 가감속 held-out bag을 확보한 뒤 다음 중 하나만 검증한다.
   - detector longitudinal observation의 uncertainty를 이용한 continuous covariance 결합.
   - tracker 내부 상태가 아니라 Map-CV 입력 speed에만 confidence와 함께 전달.
