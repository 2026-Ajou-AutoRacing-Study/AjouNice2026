# MORAI 대회 환경 heading 기준 및 자오선 수렴각 적용 여부 문의

안녕하세요, 아주대학교 AjouNice팀 주상우입니다.

이전에 문의드렸던 대회용 MORAI의 약 `1.347 deg` heading 편차에 대해 추가 분석한 결과,
해당 값이 차량 Spawn 오차나 IMU bias가 아니라 **WGS84 true north와 UTM Zone 52 grid
north 사이의 자오선 수렴각(meridian convergence)** 에서 발생하는 것으로 판단되어 다시
문의드립니다.

## 1. 기존 문의 및 추가 분석 배경

기존 문의에서는 대회 제공 sample scenario를 load했을 때 다음 차이를 확인했습니다.

- Scenario JSON의 Ego 초기 yaw: `62.515 deg`
- 같은 위치에서 `/Ego_topic.heading`: `63.8620452881 deg`
- 두 값의 차이: `+1.3470452881 deg`

이에 대해 차량 Spawn 과정에서 scenario 값과 실제 자세가 다르게 적용되었을 가능성 및 IMU
heading bias가 누적되었을 가능성을 안내받았습니다.

그러나 추가 분석 결과, 해당 편차는 초기 Spawn 위치에서만 발생하지 않았으며 IMU 적분과도
무관하게 전체 주행 구간에서 거의 일정하게 관측되었습니다.

## 2. Spawn 오차 및 IMU bias와 무관한 검증

대회 경로 약 2.2 km를 주행한 rosbag에서 IMU heading을 사용하지 않고,
`/Ego_topic.position`의 연속된 XY 위치 변화만으로 차량의 실제 map 진행 방향을
계산했습니다.

위치 변화량이 `0.1 m` 이상인 5,766개 구간을 비교한 결과는 다음과 같습니다.

| 비교 항목 | wrapped angle error 중앙값 |
|---|---:|
| map XY 이동 방향 - raw `/Ego_topic.heading` | `-1.346265 deg` |
| map XY 이동 방향 - (`heading - 1.347 deg`) | `+0.000735 deg` |

따라서 약 `1.347 deg`의 편차는 다음 특성을 보입니다.

1. 주행 시작점뿐 아니라 전체 2.2 km 경로에서 지속됩니다.
2. 진행 방향이 바뀌어도 거의 일정합니다.
3. IMU를 사용하지 않고 `/Ego_topic`의 position과 heading만 비교해도 재현됩니다.
4. 시간에 따라 점차 증가하는 누적 bias 형태가 아닙니다.

이 결과만으로도 차량의 z Spawn 여유값이나 일반적인 IMU bias 누적으로는 해당 현상을
설명하기 어려운 것으로 판단했습니다.

## 3. 자오선 수렴각과의 수치 비교

Sample scenario 시작 위치에서 기록된 GPS 좌표는 다음과 같습니다.

- Latitude: `37.239066666666666 deg`
- Longitude: `126.77305166666665 deg`

해당 위치를 WGS84 UTM Zone 52N으로 투영했을 때 PROJ가 계산한 자오선 수렴각은 다음과
같습니다.

```text
Convergence: -1d20'53.001" = -1.34805590 deg
```

이를 `/Ego_topic.heading`에 적용하면 다음 결과가 나옵니다.

```text
63.86204529 deg + (-1.34805590 deg)
= 62.51398939 deg
```

이는 Scenario JSON의 `62.515 deg`와 약 `0.00101 deg` 차이로 일치합니다.

| 항목 | 값 |
|---|---:|
| Scenario yaw | `62.51500000 deg` |
| `/Ego_topic.heading` | `63.86204529 deg` |
| UTM Zone 52N convergence | `-1.34805590 deg` |
| convergence 적용 결과 | `62.51398939 deg` |
| Scenario yaw와의 잔차 | 약 `0.00101 deg` |

따라서 현재까지의 결과는 다음 좌표계 구성을 강하게 지지합니다.

```text
Scenario·MGeo·Lanelet map yaw
  : UTM grid north 기준

/Ego_topic·Competition·IMU heading
  : geographic true north 기준
```

## 4. 해당 차이가 localization에 미치는 영향

True-north heading을 별도 변환 없이 UTM map frame의 yaw로 사용하면 차량 전방 거리별
횡방향 오차가 발생합니다.

| 전방 거리 | 약 1.347 deg에 의한 횡방향 오차 |
|---:|---:|
| 5 m | 0.12 m |
| 10 m | 0.24 m |
| 20 m | 0.47 m |
| 30 m | 0.71 m |

이는 GNSS 위치로 localization pose가 반복 보정되는 시스템에서는 겉으로 드러나지 않을 수
있지만, 다음 항목에는 지속적으로 영향을 줄 수 있습니다.

- LiDAR와 map 정합
- 차량 기준 sensor point의 map 좌표 변환
- Lanelet 선택 및 centerline 정합
- 객체 위치와 predicted path 변환
- IMU dead reckoning 및 GNSS 보정 사이의 innovation

다른 참가팀에서도 같은 좌표계 차이가 존재하더라도 GNSS 보정, LiDAR localization 또는 map
matching 과정에서 암묵적으로 흡수되어 아직 명확히 인지되지 않았을 가능성이 있다고
생각합니다. 다만 다른 팀의 구현과 인지 여부는 확인할 수 없으므로 이는 가능성으로만
제시드립니다.

또한 최초 답변에서는 Spawn 및 IMU bias 가능성이 제시되었으나 true north와 grid north의
차이는 언급되지 않았습니다. 따라서 최초 답변 단계에서는 이번 현상의 직접 원인이 자오선
수렴각임이 아직 식별되지 않았을 가능성이 있다고 판단했습니다. MORAI 내부 구현에서 이미
의도된 좌표계 구분이라면 아래 항목의 공식 확인을 부탁드립니다.

## 5. 공식 확인 요청 사항

1. Scenario JSON의 `initPosition.rot.yaw`는 UTM grid north 기준입니까?
2. `/Ego_topic.heading`은 geographic true north 기준입니까?
3. `/Competition_topic`의 heading과 IMU orientation으로 복원한 yaw는 각각 어느 north
   reference를 사용합니까?
4. 대회 MGeo·Lanelet map의 기준 투영법은 WGS84 UTM Zone 52N이 맞습니까?
5. ROS localization pose를 map frame에 배치할 때 참가팀이 자오선 수렴각을 적용해야
   합니까?
6. 참가팀이 적용해야 한다면 권장 변환 수식, 부호, datum 및 projection 정보를 공식
   문서로 제공해주실 수 있습니까?
7. 모든 참가팀과 대회용 launcher에서 동일한 heading reference 계약이 적용됩니까?

## 6. 현재 AjouNice팀의 임시 처리

현재 K-City 대회 환경에서는 다음 고정 보정을 적용했을 때 map XY 진행 방향과 가장 잘
일치했습니다.

```text
map_yaw_deg = received_heading_deg - 1.347 deg
```

다만 이 값은 K-City의 특정 영역에서 얻은 근사값이므로, 공식 좌표계 계약이 확인되면
고정값 대신 현재 위·경도와 map projection을 이용해 자오선 수렴각을 계산하는 방식으로
변경할 예정입니다.

본 문의는 MORAI에서 발행하는 수치가 무조건 잘못되었다고 단정하기 위한 것이 아닙니다.
True north와 grid north가 서로 다른 것은 정상적인 지리 좌표계 현상이지만, Scenario·map과
ROS sensor topic의 기준이 서로 다르다면 localization 구현에 직접적인 영향을 주기 때문에
각 인터페이스의 heading reference 및 변환 책임을 명확히 확인하고자 합니다.

바쁘실 와중에 긴 문의를 검토해주셔서 감사드립니다. 필요하다면 분석에 사용한 rosbag,
계산 코드, 전체 결과표 및 재현 영상을 함께 제공드리겠습니다.
