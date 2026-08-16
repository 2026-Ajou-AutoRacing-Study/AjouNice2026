# E2 PointPainting Map-CV failure analysis

## 목적과 범위

이 문서는 다음 E2 파이프라인에서 CV-stable 차량의 예측 오차가 커지는
원인을 정리한다.

```text
PointPainting DetectedObjects
  -> EKF TrackedObjects
  -> Map-CV PredictedObjects
  -> MORAI /Object_topic GT 평가
```

정량 분석에는 아래 입력을 사용했다.

- Roundabout 1:
  `sensor_cam_raw_roundabout1_roundabout_only.bag`
- Highway 1: `sensor_cam_raw_highway1.bag`
- 지도: `lanelet2_map_v60.osm`
- 주 평가 구간: 예측 시작부터 각 horizon 끝까지 MORAI GT의 종방향 가속도
  절댓값이 계속 `0.5 m/s^2` 이하인 CV-stable 표본
- 예측 모델: 현재 top-1 Map P1, 종방향 이동 거리는 CV

CV-stable은 미래 GT를 이용하는 **오프라인 평가용 필터**다. 런타임
predictor가 사용할 수 있는 정보가 아니며, tracking 평가는 전체 구간을 유지한다.

## 핵심 결론

현재 큰 오차는 하나의 원인이 아니라 세 문제가 이어져 발생한다.

1. Detection/tracking state에 약 `0.11 s`에 해당하는 종방향 위치 편향이 있다.
2. 로터리에서 매 프레임 독립적으로 Lanelet을 고르므로 작은 위치·yaw 오차가
   잘못된 출구 선택으로 증폭된다.
3. 차량 yaw가 약 `pi` 반전되면 tracker의 body-frame 종방향 속도가 음수가 되고,
   Map predictor가 정합을 포기하여 CTRV fallback이 곡선 도로를 직선으로
   이탈한다.

따라서 Map-CV 자체만 교체해서는 해결되지 않는다. 센서 시간 정렬, tracker의
`pi` 방향 모호성 해소, track-level Lanelet 이력 유지가 각각 필요하다.

## 1. 약 0.11초 시간 정렬 가설

사용자가 언급한 `0.11 m`와 구분해야 한다. 현재 관측된 것은 **0.11초의
시간 차이에 상당하는 위치 편향 가설**이며, 위치 오차가 0.11 m라는 뜻이 아니다.

### 관측 결과

동일한 Roundabout CV-stable 3초 matched 표본 627개에서 prediction header와
같은 시각의 GT를 비교하면 center mean은 `0.747 m`였다. GT를 미래로
`+0.105 s` 이동해 비교하면 `0.484 m`, `+0.11 s`에서는 `0.485 m`가 됐다.

| 비교 | center mean | p50 | p95 |
|---|---:|---:|---:|
| 동일 header timestamp | 0.747 m | 0.742 m | 1.214 m |
| GT `+0.105 s` | **0.484 m** | 0.477 m | 0.885 m |
| GT `+0.11 s` | **0.485 m** | 0.475 m | 0.892 m |

Highway도 동일 header 비교 시 center mean이 약 `1.56 m`였지만, GT를
`+0.11 s` 이동하면 약 `0.55 m`로 감소했다. 종방향 signed bias를 속도로
환산한 값도 Roundabout 약 `0.108 s`, Highway 약 `0.118 s`로 유사했다.

서로 다른 장면에서 비슷한 값이 반복되므로 LiDAR scan 기준 시각, message
header, localization 보간 사이의 시간 의미가 어긋났을 가능성이 높다. 다만
현재 결과만으로 원인이 확정된 것은 아니다.

### 관련 현 설정

EKF tracker 설정은 다음과 같다.

```yaml
cal_detection_individual_time: false
lidar_rotation_period: 0.1
lidar_sync_scan_start: false
```

즉 10 Hz 회전 주기를 알고 있지만 detection별 LiDAR motion compensation은
비활성화되어 있다. 약 `0.1 s`와 관측 편향이 비슷하다는 사실은 가설을
강화하지만, 그대로 `0.11 s`를 하드코딩할 근거는 아니다.

### 해결 방향

- 임시 진단: 평가기에서만 여러 time shift를 sweep하여 장면별 최적값과
  일관성을 확인한다.
- 임시 보정: scan timestamp 의미가 검증된 뒤에만 state를 올바른 기준 시각으로
  외삽한다. 고정 `+0.11 s` 보정은 진단 전에는 production에 넣지 않는다.
- 근본 해결: LiDAR stamp가 scan start/end 중 무엇인지 확정하고, Ego pose를
  scan/object 기준 시각으로 보간하며, 필요하면 deskew 또는 detection individual
  time compensation을 적용한다.

## 2. 로터리의 잘못된 Lanelet·갈림길 선택

Roundabout CV-stable 3초 matched 표본 627개를 runtime 분기별로 나누면 다음과
같다.

| 분류 | 표본 | 비율 | 3초 FDE mean | p50 | p95 |
|---|---:|---:|---:|---:|---:|
| 올바른 mapped route | 258 | 41.1% | 1.571 m | 1.351 m | 3.563 m |
| 잘못된 mapped route | 152 | 24.2% | 10.044 m | 11.904 m | 20.042 m |
| yaw-flip fallback | 214 | 34.1% | 9.242 m | 9.354 m | 11.671 m |
| 기타 fallback | 3 | 0.5% | 10.557 m | - | - |

여기서 `152/627 = 24.2%`는 전체 표본 중 잘못된 route를 선택한 비율이다.
3초 p90 tail은 63개이며, 그중 62개가 잘못된 route였다. 즉
`62/63 = 98.4%`는 **극단 tail 내부의 구성비**다. 두 수치는 분모가 다르므로
모순이 아니다. 나머지 wrong-route 90개는 오차가 컸지만 p90 threshold
아래였다.

현재 P1은 각 프레임의 위치와 yaw로 Lanelet 후보를 새로 점수화한다. 이전
프레임에서 어떤 Lanelet을 따라왔는지, routing graph상 자연스럽게 이어지는지,
lane change가 실제로 관측됐는지를 track 단위로 유지하지 않는다. wrong-route
그룹의 평균 시작 위치 오차는 약 `0.714 m`, yaw 오차는 약 `10.6 deg`였다.
로터리의 인접 차선과 출구가 가까우므로 이 정도 오차만으로도 다른 branch가
선택될 수 있다.

Oracle GT state로 같은 Map-CV를 실행한 3초 결과는 mean 약 `0.435 m`, p95 약
`0.758 m`였다. 따라서 v60 centerline과 CV 경로 전개 자체는 유효하며, 현재
tail은 주로 입력 state와 프레임 독립적인 Lanelet 선택에서 발생한다.

### 해결 방향

- 임시: 현재 위치·yaw 점수에 직전 Lanelet 및 routing continuity 가산점을
  주고, 충분한 횡이동 증거가 없으면 인접 branch로 즉시 전환하지 않는다.
- 임시: 후보 점수가 비슷하면 top-1 하나를 성급히 확정하지 않고 다중 경로와
  confidence를 planning에 전달한다.
- 근본: track별 Lanelet belief를 유지하는 HMM/Bayesian map matching을 적용한다.
  측정 likelihood, 이전 lane belief, routing transition, lane-change 가능성을
  함께 갱신하고 경로별 확률을 calibration한다.

## 3. yaw 반전, 음수 속도, 직선 도로 이탈

### 확인된 현상

Roundabout mapped path 410개는 경로 중 어느 점도 vehicle centerline에서
3 m 이상 벗어나지 않았다. 반면 yaw-flip fallback 214개 중 143개(`66.8%`)는
경로 중 한 점 이상이 centerline에서 3 m 이상 벗어났고, 28개(`13.1%`)는
3초 endpoint도 3 m 이상 벗어났다. 이 그룹의 line-deviation p50은 거의 0으로,
RViz에서 보이는 경로가 실제로 거의 직선이었다.

따라서 두 시각적 실패는 구분해야 한다.

- 잘못된 갈림길: 지도 위에 남지만 다른 차선 또는 출구를 따라간다.
- 도로 밖 직진: 대부분 yaw-flip 때문에 map matching이 거부된 뒤 fallback한
  경로다.

### 왜 반전 yaw인데 물체는 전진하는가

tracker가 실제 이동 방향을 `theta`, 속력을 `v`라고 할 때 다음과 같은 동치
표현이 가능하다.

```text
(yaw, longitudinal speed) = (theta, v)
                          = (theta + pi, -v)
```

두 번째 표현의 world velocity는 다음처럼 첫 번째와 같다.

```text
(-v) * [cos(theta + pi), sin(theta + pi)]
  = v * [cos(theta), sin(theta)]
```

그래서 bbox 화살표는 반대를 향하고 body-frame `linear.x`는 음수여도 track
center는 실제 이동 방향으로 갈 수 있다. 그러나 현재 Map predictor는
`speed < 0.5 m/s`이거나 Lanelet 방향과 yaw 차이가 45도 이상이면 map matching을
포기한다. 이후 CTRV fallback은 낮은 yaw rate로 직선을 그리므로 로터리
centerline을 이탈한다.

### yaw는 어디서 반전되는가

Raw PointPainting detection을 GT와 3 m 이내로 대응시킨 별도 진단에서:

| 장면 | matched detection | yaw 오차 `>90 deg` | 비율 | yaw p50 | yaw p95 |
|---|---:|---:|---:|---:|---:|
| Roundabout | 708 | 233 | **32.9%** | 5.64 deg | 179.24 deg |
| Highway | 349 | - | **16.9%** | 3.02 deg | 178.70 deg |

주 E2 tracking 분석의 `214/627 = 34.1%`와 raw detection의
`233/708 = 32.9%`는 서로 다른 표본이다.

- `32.9%`: raw detection 전체, 3 m matching 기준
- `34.1%`: 30 m ROI, 2 m tracking matching, CV-stable 3초 future가 존재하고
  predicted path까지 나온 627개 기준

따라서 둘 중 하나가 틀린 것이 아니다. 현재 증거로는 `pi` 모호성이 detector
출력에 이미 존재하며 tracker가 안정적으로 해소하지 못한다고 결론낼 수 있다.
단, 같은 detection을 tracker 출력까지 1:1 추적한 paired 전후율은 아직
계산하지 않았으므로 tracker가 몇 건을 교정하거나 추가로 악화했는지는 미확정이다.

PointPainting head는 `USE_DIRECTION_CLASSIFIER: true`, `NUM_DIR_BINS: 2`로
front/back 방향 bin을 예측한다. 3D bbox 자체는 `pi` 회전해도 같은 점유 형상을
만드므로, 16채널 LiDAR의 희소한 점, 측후면 관측, 곡선 주행, 학습-동적 bag 간
domain 차이에서는 direction bin이 모호할 수 있다. ROS 변환은 box yaw를 그대로
quaternion으로 옮기며 별도의 반전은 추가하지 않는다.

EKF tracker도 새 track을 measurement yaw로 초기화하고, 이후 direction score를
이전 track yaw와의 연속성 위주로 갱신한다. world-frame `vx, vy` 또는 Lanelet
방향으로 `pi` 모호성을 명시적으로 풀지 않는다. 최종 body longitudinal speed는
track yaw에 world velocity를 투영해 생성하므로 yaw가 반전되면 음수가 된다.

### 임시 해결

1. **차량 state canonicalization**: 충분히 움직이는 차량은
   `atan2(vy, vx)`를 실제 이동 방향으로 사용한다. `yaw`와 `yaw + pi` 중 이동
   방향에 가까운 것을 택하고 longitudinal speed가 양수가 되도록 표현한다.
   저속에서는 velocity가 불안정하므로 이전 안정 yaw와 history를 유지한다.
2. **Predictor 방어 로직**: `(yaw, speed)`와 `(yaw + pi, -speed)` 두 표현을 모두
   Lanelet 방향과 비교하여 더 일관된 표현을 선택한다. 단순 `abs(speed)`는 yaw와
   velocity의 의미를 깨뜨리므로 사용하지 않는다.
3. **Fallback 개선**: signed body speed 대신 tracker의 world velocity vector로
   경로를 전개한다. 이렇게 하면 반전 yaw 때문에 진행 방향을 잃는 문제를 줄일
   수 있다.

가장 먼저 넣을 임시 수정 위치는 predictor보다 tracker 출력단이 적절하다.
`TrackedObjects`가 Autoware 의미론에 맞게 차량 정면 yaw와 양의 전진 속도를
표현하면 downstream의 모든 모듈이 같은 state를 사용한다.

### 근본 해결

1. Detector의 direction-bin 오류를 동적 로터리·고주로 test에서 별도 측정하고,
   동적/측후면/곡선 표본으로 direction head를 보강하거나 calibration한다.
2. Tracker가 `(theta, v)`와 `(theta + pi, -v)`를 명시적인 두 방향 hypothesis로
   유지하고, velocity direction·detector confidence·yaw history·Lanelet 방향을
   누적해 선택하도록 한다.
3. 저속·정지 구간에서는 direction을 확정하지 않고 uncertainty를 유지한 뒤,
   충분한 displacement가 관측될 때 확정한다.
4. Map matching도 단일 프레임 yaw를 절대 조건으로 쓰지 않고 track-level lane
   belief와 방향 hypothesis를 함께 평가한다.

## 4. 장면별 우선순위

Highway CV-stable 3초 matched 128개에서는 올바른 mapped route가 118개
(`92.2%`), wrong route 7개(`5.5%`), yaw-flip fallback 3개(`2.3%`)였다.
Roundabout보다 branch/yaw 문제가 훨씬 작고, 동일 header 기준 약 `1.56 m`의
초기 center bias가 주된 문제였다. 따라서 우선순위는 다음과 같다.

1. 공통: `0.11 s` 시간 정렬 가설을 계측으로 검증하고 center bias를 제거한다.
2. Roundabout: yaw canonicalization과 track-level Lanelet continuity를 구현한다.
3. Highway: 시간 정렬 후 남는 coverage 부족과 7개 wrong-route 표본을 분리해
   분석한다.
4. 모든 변경은 all-motion과 CV-stable을 함께 평가하되, production 모델 선택은
   CV-stable을 주 지표로 사용한다.

## 권장 구현 순서

1. evaluator에 detection -> tracking paired yaw 진단과 time-shift sweep을 추가한다.
2. 움직이는 차량에 한해 tracker output yaw/speed canonicalization을 구현하고
   기존 E2 bag과 동일 조건으로 회귀 평가한다.
3. predictor fallback을 world velocity 기반으로 바꿔 방어한다.
4. track-level Lanelet hysteresis를 넣고 Roundabout wrong-route 152건 및 p90 tail
   62건이 얼마나 감소하는지 확인한다.
5. 이후 Autoware map-based prediction의 다중 경로 확률·lane history 로직을
   단계적으로 포팅한다.

각 단계는 center/FDE 평균만 보지 말고 coverage, p50, p90, p95, wrong-route,
fallback, yaw-flip 비율을 함께 비교해야 한다. 평균 개선이 일부 path 누락으로
만들어진 것은 아닌지 반드시 확인한다.
