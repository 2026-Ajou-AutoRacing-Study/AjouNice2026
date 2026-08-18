ㅁㅁㅁㅁ# E2 PointPainting Map-CV failure analysis

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

1. Detection/tracking의 bbox center와 MORAI `ObjectStatus.position`이 같은 물리적
   기준점이 아니다. 이를 시간 이동으로 맞추면 약 `0.11 s`처럼 보이는 착시가
   생긴다.
2. 로터리에서 매 프레임 독립적으로 Lanelet을 고르므로 작은 위치·yaw 오차가
   잘못된 출구 선택으로 증폭된다.
3. 차량 yaw가 약 `pi` 반전되면 tracker의 body-frame 종방향 속도가 음수가 되고,
   Map predictor가 정합을 포기하여 CTRV fallback이 곡선 도로를 직선으로
   이탈한다.

따라서 Map-CV 자체만 교체해서는 해결되지 않는다. evaluator의 위치 기준점
분리, tracker의 `pi` 방향 모호성 해소, 갈림길의 다중 hypothesis 활용이 각각
필요하다.

## 1. 약 0.11초 시간 정렬 가설 재검증: 기각

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

이 결과만 보면 시간 편향처럼 보이지만, 움직이는 GT를 미래로 옮기면 **고정된
종방향 공간 offset도** 더 잘 맞게 된다. 따라서 shift sweep 단독으로 시간과
기준점 차이를 구분할 수 없다.

### 2026-08-18 엄밀 재검증

새 evaluator는 다음을 지킨다.

- shift와 관계없이 동일한 estimate/GT association 표본만 사용한다.
- Detection은 sensor frame에 둔 채 GT와 Ego를 같은 후보 시각으로 이동한다.
- Detection, tracking, prediction header 전달과 bag arrival 지연을 별도로 잰다.
- 종방향 오차를 `time slope * speed + spatial intercept`로 분해하고 actor별로
  다시 적합한다.

그 결과 LiDAR→Detection→Tracking→Prediction header 차이는 모두 정확히
`0.000 s`였다. bag 기록 시각에서 header를 뺀 평균은 Roundabout 기준 LiDAR
`0.9 ms`, Detection `34.5 ms`, Tracking `44.3 ms`, Prediction `59.2 ms`였다.
이는 runtime latency이며 state 기준 시각을 `+0.11 s` 이동할 근거가 아니다.

더 결정적인 반례는 Highway의 동일 asset `A2256W001105`다.

| 동일 asset의 관측 | GT 속도 | 동일시각 종방향 offset | offset을 시간으로 오해한 최적 shift |
|---|---:|---:|---:|
| actor A | 16.50 m/s | 1.376 m | 0.085 s |
| actor B | 8.47 m/s | 1.426 m | 0.140 s |

진짜 고정 시간 지연이라면 속도가 약 두 배일 때 공간 오차도 약 두 배여야 한다.
실제로는 속도와 무관하게 약 `1.4 m`로 유지되고, 이를 GT 이동으로 없애려 해서
최적 shift만 속도에 반비례한다. Highway tracking 전체 회귀도 time slope
`-0.005 s`, spatial intercept `1.467 m`, `R^2=0.003`이었다. 즉 시간 성분은
설명력이 없고 고정 공간 성분이 지배적이다. Roundabout은 관측 차량 속도가 모두
약 `5.7 m/s`로 좁아 독립 식별이 어렵지만, 약 `0.6 m / 5.7 m/s = 0.105 s`가
되어 같은 착시를 만든다.

LiDAR cloud에는 point별 `time` field도 있으며, 두 bag의 32-cloud 표본은 header
대비 대략 `-0.171~+0.001 s` 범위였다. header는 scan end에 가깝고, 현재 PointPainting은
이 field를 버리고 full scan을 처리한다. 이는 향후 deskew 과제이지만 관측된
**양의 0.11초 state shift의 증거는 아니다.**

### 관련 현 설정

EKF tracker 설정은 다음과 같다.

```yaml
cal_detection_individual_time: false
lidar_rotation_period: 0.1
lidar_sync_scan_start: false
```

이 값은 upstream tracker의 기존 기본값이며 현재 Sensor_6의 `15 Hz` 설정과도
일치하지 않는다. 다만 `cal_detection_individual_time: false`이므로 현
PointPainting/EKF 경로의 state timestamp에는 사용되지 않는다. 숫자가 관측된
가상 shift와 비슷하다는 것만으로 시간 가설의 증거가 되지 않는다.

### 적용 결론

- production message timestamp와 tracker state에는 `+0.11 s` 보정을 넣지 않는다.
- 절대 center error는 기존대로 남겨 현재 상태의 위치 차이를 표시한다.
- prediction motion 품질에는 GT future 전체를 현재 estimate-GT offset만큼 평행
  이동한 `origin_aligned` ADE/FDE를 함께 기록한다. 이는 초기 box 기준점 차이를
  제거하지만 속도·곡률·갈림길 예측 오차는 그대로 남긴다. 동시에 현재 프레임의
  순수 tracking translation error도 제거하므로 end-to-end 안전성 지표를 대체하지
  않고 motion-model 조건부 진단으로만 사용한다.
- 진짜 bbox-center tracking 정확도가 필요하면 MORAI LiDAR Instance GT 또는
  asset별 검증된 geometric-center 변환을 별도로 확보한다. 모델 출력으로 GT를
  사후 calibration해 tracking 성능을 부풀리지 않는다.
- point-time deskew는 별도 ablation으로 다루며 timestamp 보정과 혼합하지 않는다.

### 2026-08-18 BBox 기준점 교차 검증

고정 `+0.11 s` 보정을 적용하지 않고 시나리오 actor 좌표와 SaveSensorData
`Instance.txt`를 독립적으로 비교했다.

- 190개 장면, 차량 GT box 3,299개를 사용했다.
- 길이 5.5 m 미만 승용차급 2,813개에서 bbox 중심은 scenario actor
  기준점보다 차량 진행 방향으로 중앙값 `+1.340 m` 앞에 있었다.
- 같은 표본의 횡방향 중앙값은 `-0.009 m`, p95는 `0.122 m`였다.
- 길이 7.5 m 이상 버스·대형차의 중앙값은 약 `3.220 m`로 달라서 모든
  asset에 하나의 offset을 적용할 수는 없다.
- PointPainting 온라인 차량 box 크기는 ObjectStatus `size`와 대체로
  일치했다. 고주로에서 1.340 m 기준점 보정 후 detection center p50은
  `1.549 m -> 0.508 m`, BEV IoU p50은 `0.416 -> 0.643`으로 개선됐다.
- 로터리에서는 center p50이 `0.809 -> 0.746 m`로만 개선됐다. 회전식
  LiDAR scan 내 운동 왜곡과 detector translation error가 남아 있으므로,
  이 오차까지 매 프레임 제거하는 `origin_aligned`는 낙관적인 진단값이다.

따라서 지표의 역할은 다음과 같이 구분한다.

1. `ObjectStatus.position` 절대 오차는 actor-reference 호환 지표다.
2. `origin_aligned`는 prediction 경로 형상과 이동거리 모델의 조건부 상한이다.
3. production 비교는 캡처 `Instance_8Points`로 독립 calibration된
   `bbox_center_absolute_distance_m`를 사용한다. 미검증 asset은 억지로
   보정하지 않고 주 지표에서 제외한다.
4. 이번 결과는 고정 time shift가 아니라 공간 기준점 차이를 지지한다.

### Production evaluator 적용

`evaluate_pipeline_prediction.py`의 기본 GT reference를
`bbox-center-calibrated-v1`로 변경했다.

- 모델의 detection/tracking/prediction 좌표는 수정하지 않는다.
- 승용차급 GT만 각 시점 heading 방향으로 `1.340 m` 이동한다.
- 고주로 bag에서 확인된 `6.557 x 2.517 x 2.505 m` profile은 캡처 GT의
  `DataID 30400002` 중앙값 `1.383 m`를 사용한다.
- 다른 미검증 대형차는 주 지표에서 제외하고 개수를 보고한다.
- `ObjectStatus.name`은 같은 문자열이 서로 다른 크기에 재사용되어 안정적인 asset
  key가 아니므로, 명시적 profile은 `length/width/height`로 식별한다.
- 기존 ObjectStatus 기준 결과는 같은 summary의
  `object_status_reference_evaluation`에 함께 저장한다.
- `origin_aligned`는 보정된 BBox GT에서도 현재 estimate 위치에 다시
  정렬한 motion-only 진단으로 유지한다.

PointPainting epoch 65의 CV-stable production 결과는 다음과 같다.

| 장면 | Tracking BBox center p50 | 1 s Top-1/minK p50 | 2 s Top-1/minK p50 | 3 s Top-1/minK p50 |
|---|---:|---:|---:|---:|
| Roundabout 1 | 0.772 m | 0.974 / 0.751 m | 1.640 / 1.129 m | 2.285 / 1.510 m |
| Highway 1 | **0.500 m** | **0.611 / 0.593 m** | **0.925 / 0.869 m** | **1.306 / 1.224 m** |

두 bag 모두 unsupported segment는 `0`이다. Highway는 승용차 17개와 명시적
중형 vehicle profile 2개를 사용한다. Roundabout의 잔여 current error는 기준점
문제만으로 사라지지 않으므로 LiDAR scan motion distortion과 detector/tracker
translation error를 다음 단계에서 분리한다.

CV-stable origin-aligned p50은 현재 tracking translation error를 추가로 제거했을
때 motion/path 모델이 도달하는 조건부 성능을 보여준다.

| 장면 | horizon | Top-1 FDE p50 | origin-aligned | minFDE-K p50 | origin-aligned |
|---|---:|---:|---:|---:|---:|
| Roundabout | 1 s | 0.974 m | **0.688 m** | 0.751 m | **0.554 m** |
| Roundabout | 3 s | **2.285 m** | 2.311 m | **1.510 m** | 1.590 m |
| Highway | 1 s | 0.611 m | **0.514 m** | 0.593 m | **0.514 m** |
| Highway | 3 s | 1.306 m | **1.266 m** | 1.224 m | **1.194 m** |

Roundabout 3초 오차가 거의 그대로인 이유는 고정 위치 offset이 아니라 sibling
branch Top-1 오선택이 지배하기 때문이다. 반면 Highway tracking p50은 legacy
ObjectStatus 기준 `1.479 m`에서 BBox 기준 `0.500 m`로 내려가 기존 약 `1.4 m`
floor의 상당 부분이 기준점 차이였음을 확인했다.

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

1. 공통: evaluator에서 ObjectStatus reference point와 detector bbox center를
   분리하고 absolute/origin-aligned 지표를 함께 유지한다.
2. Roundabout: yaw canonicalization 후 Top-K route의 confidence 순위 오류를
   분해한다. track-level continuity는 비연결 Lanelet 점프가 실제로 확인될 때
   적용한다.
3. Highway: origin alignment 후 남는 motion error, coverage 부족과 7개
   wrong-route 표본을 분리해 분석한다.
4. 모든 변경은 all-motion과 CV-stable을 함께 평가하되, production 모델 선택은
   CV-stable을 주 지표로 사용한다.

## 권장 구현 순서

1. evaluator에 detection -> tracking paired yaw, 엄밀한 time-shift sweep,
   origin-aligned prediction metric을 유지한다.
2. 움직이는 차량에 한해 tracker output yaw/speed canonicalization을 구현하고
   기존 E2 bag과 동일 조건으로 회귀 평가한다.
3. predictor fallback을 world velocity 기반으로 바꿔 방어한다.
4. Roundabout p90 tail에서 비연결 Lanelet 점프와 합법적인 sibling successor의
   Top-1 순위 오류를 분리한다.
5. 전자가 유의미하면 Autoware의 lane history/continuity를 포팅하고, 후자가
   지배적이면 기존 다중 경로의 confidence와 Planning 소비 정책을 먼저 개선한다.

각 단계는 center/FDE 평균만 보지 말고 coverage, p50, p90, p95, wrong-route,
fallback, yaw-flip 비율을 함께 비교해야 한다. 평균 개선이 일부 path 누락으로
만들어진 것은 아닌지 반드시 확인한다.

## 2026-08-17 vehicle orientation canonicalization 결과

첫 번째 완화책을 EKF 내부 association이 아니라 표준 `TrackedObjects` 출력단에
구현했다. CAR/TRUCK이 1.0 m/s 이상으로 움직일 때 EKF world velocity와 box yaw의
차이를 계산하고, 100도 이상이면 `yaw + pi` 표현으로 진입하며 80도 이하가 될
때만 원래 표현으로 복귀한다. 저속에서는 직전 결정을 유지한다. 위치, world
velocity, association 및 detector 출력은 바꾸지 않는다.

동일한 epoch-65 PointPainting detection bag을 재생해 tracker와 Map-CV만 다시
실행했다. 따라서 전후 차이는 detector 추론 변동이 아니라 이 정규화와 그에
따른 map matching 변화다.

### Detection -> tracking paired yaw

| 장면 | 지표 | 기존 | 정규화 후 |
|---|---|---:|---:|
| Roundabout | paired detection yaw flip | 34.8% | 34.8% |
| Roundabout | paired tracking yaw flip | 32.3% | **0.0%** |
| Roundabout | 전체 tracking 음수 종방향 속도 | 33.2% | **0.0%** |
| Roundabout | tracking yaw mean error | 62.7 deg | **8.0 deg** |
| Roundabout | tracking speed mean error | 4.06 m/s | **0.61 m/s** |
| Highway | paired detection yaw flip | 17.6% | 17.6% |
| Highway | paired tracking yaw flip | 13.2% | **1.7%** |
| Highway | 전체 tracking 음수 종방향 속도 | 11.9% | **1.3%** |
| Highway | tracking yaw mean error | 24.0 deg | **5.1 deg** |
| Highway | tracking speed mean error | 2.92 m/s | **0.65 m/s** |

Roundabout paired 641건에서 detector flip 223건을 모두 올바른 방향으로
정규화했으며 correct detection 418건을 새로 뒤집지 않았다. Highway paired
295건에서는 detector flip 52건 중 51건을 교정했다. 남은 5개 tracking flip은
대부분 저속 hysteresis 또는 noisy velocity 경계 표본으로 후속 진단 대상이다.

### CV-stable Map-CV FDE

| 장면 | horizon | 지표 | 기존 | 정규화 후 |
|---|---:|---|---:|---:|
| Roundabout | 1 s | mean / p50 | 1.37 / 1.29 m | **1.27 / 1.04 m** |
| Roundabout | 2 s | mean / p50 | 3.31 / 3.21 m | **2.99 / 1.78 m** |
| Roundabout | 3 s | mean / p50 | 6.29 / 4.94 m | **5.68 / 2.45 m** |
| Roundabout | 3 s | p90 / p95 | **12.92 / 15.29 m** | 15.37 / 17.00 m |
| Highway | 3 s | mean / p50 | 1.79 / 1.48 m | **1.75 / 1.48 m** |
| Highway | 3 s | p90 / p95 | 3.66 / 4.15 m | **3.52 / 3.94 m** |

모든 horizon의 coverage는 전후 동일했다. 따라서 누락을 늘려 평균을 낮춘 결과가
아니다. Roundabout의 대표 오차와 평균은 크게 개선됐지만 p90/p95는 악화됐다.
기존 yaw-flip 객체가 straight fallback하는 문제는 제거됐으나, 정규화 후 지도
정합에 들어온 일부 객체가 잘못된 Top-1 route를 따라 더 큰 tail을 만들었다.
아래의 route-topology 재분석 결과, 이를 곧바로 비연결 Lanelet 점프로 간주해
track-level continuity를 최우선으로 구현하는 것은 부정확하다.

Highway는 방향 반전이 원래 적어 FDE 변화는 작았다. 남은 주요 오차는
ObjectStatus와 bbox center의 기준점 차이, coverage, 실제 motion error이므로 방향
threshold를 더 공격적으로 조정할 우선순위는 낮다.

## 2026-08-17 Roundabout p90 tail route-topology 재분석

yaw canonicalization 결과의 CV-stable 3초 표본 624건을 대상으로 Top-1 FDE
p90인 `15.291 m` 이상 63건을 다시 분류했다. 각 표본에서 현재 Top-1뿐 아니라
Map-CV가 이미 생성한 모든 graph-valid path를 재구성하고, GT 3초 위치에 가장
가까운 best-of-K 경로와 Lanelet topology를 비교했다.

| p90 tail 유형 | 건수 | 비율 |
|---|---:|---:|
| 동일 predecessor의 합법적인 sibling successor 중 Top-1 오선택 | 61 | **96.8%** |
| 현재 Top-K로 해결 불가 | 2 | 3.2% |
| 이전 후보 집합 밖의 비연결 시작 Lanelet 점프 | 0 | **0.0%** |

61건의 세부 topology는 다음 두 쌍으로 완전히 설명된다.

| Top-1 시작 | GT에 가까운 후보 시작 | 공통 predecessor | 건수 |
|---:|---:|---|---:|
| `201108` | `201094` | `201144`, `201146` | 39 |
| `201098` | `201097` | `201094`, `201095` | 22 |

즉 `201094`와 `201108`, `201097`과 `201098`은 각각 graph상 연결되지 않은
엉뚱한 Lanelet 쌍이 아니다. 동일 predecessor에서 정상적으로 갈라지는 합법적
미래 후보다. 화면에서는 잘못된 출구를 선택한 것처럼 보이며 실제로 Top-1은
틀렸지만, 원인은 continuity 단절보다 branch confidence 순위 오류다.

Top-1과 oracle best-of-K의 3초 FDE를 동일 624건에서 비교하면 다음과 같다.

| 경로 선택 | mean | p50 | p90 | p95 |
|---|---:|---:|---:|---:|
| 현재 confidence Top-1 | 5.656 m | 2.440 m | 15.291 m | 17.010 m |
| GT에 가장 가까운 oracle best-of-K | **2.069 m** | **1.712 m** | **4.193 m** | **5.299 m** |

oracle best-of-K는 runtime에서 선택할 수 없는 진단 상한이지만, 미래 경로 생성
coverage는 이미 충분하고 병목이 confidence ranking과 downstream 소비라는 것을
보여준다. 단순히 후보 수를 더 늘리는 실험의 우선순위는 낮다.

61개 오선택 중 GT에 가장 가까운 후보는 59개가 confidence 2위, 2개가 3위였다.
Top-1 confidence 중앙값은 `0.520`, GT에 가까운 후보는 `0.477`이며 둘의 차이
중앙값은 불과 `0.043`이었다. 즉 모델이 압도적으로 잘못된 출구를 확신한 것이
아니라, 이미 생성된 두 합법 후보의 근소한 기하학 점수 차이가 Top-1 순서를
결정한 경우가 대부분이다. Planning에서 합리적인 confidence threshold로 Top-N을
사용하거나 관측된 maneuver 단서로 이 작은 margin을 재정렬할 여지가 크다.

단순 confidence EMA나 이전 Top-1 고정도 직접 해결책은 아니다. 이전 표본이 있는
tail 62건 중 90.3%는 직전 프레임도 이미 오선택이었고, 85.5%는 동일한 잘못된
sibling 쌍을 연속 선택했다. tail과 겹치는 오선택 run의 길이는 중앙값 4 frame
(`0.32 s`), p90 11 frame(`1.10 s`), 최대 12 frame(`1.29 s`)이었다. 따라서 과거
Top-1을 그대로 누적하면 잘못된 선택을 오히려 강화하거나 수정 시점만 늦출 수
있다. 이력은 Top-1 smoothing이 아니라 lateral motion, velocity direction처럼
새로운 maneuver 관측 단서를 추출하는 데 사용해야 한다.

### 수정된 개선 순서

1. evaluator에 top-1/min-K, sibling-successor 오선택률, confidence margin을
   정식 지표로 추가한다.
2. bbox center와 ObjectStatus 기준점 차이를 분리하고 tracking 위치·yaw 오차가
   sibling 후보 순위를 체계적으로 왜곡하는지 확인한다. 고정 `+0.11 s` 보정은
   적용하지 않는다.
3. 현재 branch prior와 곡률 기반 transition score가 두 sibling 후보에 어떤
   confidence를 부여하는지 계측한다.
4. Autoware의 lateral-kinematics history와 maneuver continuity를 offline causal
   ablation으로 먼저 검증하고, 실제
   출구 진입 단서가 관측될 때 confidence를 재정렬한다. 단, 아무 단서가 없는
   branch에서는 하나를 억지로 확정하지 않는다.
5. Planning의 safety-critical 모듈이 confidence threshold 이상의 복수 경로를
   활용하도록 모듈별 정책을 검토한다. 현재 교차로 일부는 복수 경로를 쓰지만
   동적 회피/cut-in 일부는 Top-1만 사용한다.
6. lane history gating은 일반적인 map matching 안정장치로 남기되, 이번 로터리
   p90 tail의 직접 해결책으로 과대평가하지 않는다.

따라서 다음 production 작업은 track-level Lanelet continuity 단독 구현이 아니라
`confidence 진단 -> Autoware식 maneuver/lateral history -> Planning의 Top-N 소비`
순서로 수정한다.

### 중복 route hypothesis 정리의 현재 한계

시작 Lanelet 후보가 달라도 동일한 downstream Lanelet에 도달하고 실제 3초 미래
궤적이 `0.75 m` 이내로 겹치면 하나의 물리적 hypothesis로 묶고, 그룹 내 최고
raw score만 대표 confidence 계산에 사용한다. downstream Lanelet 조건은 갈림길
진입 초기에 아직 기하 거리가 가까운 정상 sibling 경로를 합치는 오류를 막는다.
따라서 동일한 순환 경로가 여러 번 열거되어 실제 출구 후보를 Top-K 밖으로
밀어내는 문제를 방지한다. `maximum_paths=6`은 서로 다른 미래 움직임의 안전
상한이며, 두 갈래에서는 보통 두 경로만 남는 것이 목표다.

다만 현재 score는 정식 확률이 아닌 거리·yaw·분기·전환 곡률 기반 heuristic이다.
따라서 중복 score의 단순 합산은 보류한다. 향후에는 시작 Lanelet posterior와
조건부 branch probability를 각각 정규화한 뒤 동일 hypothesis로 확률 질량을
marginalization하고, Brier score와 calibration curve로 검증해야 한다.
