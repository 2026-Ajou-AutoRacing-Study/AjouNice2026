# MORAI scenario yaw와 ROS heading 기준 확인 요청

## 문의 요약

2026 대회 제공 sample scenario의 Ego 초기 `rot.yaw`와 같은 위치에서 발행되는
`/Ego_topic.heading` 사이에 약 `1.347 deg`의 고정 차이를 확인했다.

분석 결과 이 값은 K-City 시작 위치의 WGS84 true north와 UTM Zone 52 grid north
사이 자오선 수렴각과 거의 정확히 일치한다. 따라서 simulator 수치 오류로 단정하기보다
각 인터페이스가 사용하는 heading 기준을 공식적으로 확인하고 싶다.

## 재현 환경

- Scenario: `2026_molit_comp_sample_scene.json`
- Ego DataID: `20200012`
- Scenario 초기 위치:
  - x: `-131.48599243164063`
  - y: `-427.96099853515625`
  - z: `28.882999420166016`
- Scenario 초기 yaw: `62.515 deg`
- 같은 위치의 시작 GPS:
  - latitude: `37.239066666666666 deg`
  - longitude: `126.77305166666665 deg`
- ROS topic: `/Ego_topic`
- Message field: `morai_msgs/EgoVehicleStatus.heading`

## 관측 결과

정지 상태이며 scenario 초기 위치와 정확히 같은 `/Ego_topic` sample 155개에서 다음을
확인했다.

| 항목 | 값 |
|---|---:|
| Scenario `rot.yaw` | `62.515 deg` |
| `/Ego_topic.heading` | `63.8620452881 deg` |
| `/Ego_topic.heading - scenario yaw` | `+1.3470452881 deg` |
| 정지 sample heading 표준편차 | `0.0 deg` |

시작 GPS 좌표를 WGS84 UTM Zone 52N으로 투영했을 때 PROJ가 계산한 자오선 수렴각은
다음과 같다.

```text
Convergence: -1d20'53.001" = -1.34805590 deg
```

따라서 다음 변환은 약 `0.00101 deg` 오차로 일치한다.

```text
scenario/grid yaw
  ~= Ego true-north heading + meridian convergence
  = 63.86204529 + (-1.34805590)
  = 62.51398939 deg
```

전체 주행에서 위치 변화량이 0.1 m 이상인 5,766개 구간도 확인했다.

| 비교 | wrapped angle error 중앙값 |
|---|---:|
| map XY 이동 방향 - raw `/Ego_topic.heading` | `-1.346265 deg` |
| map XY 이동 방향 - (`heading - 1.347 deg`) | `+0.000735 deg` |

이는 scenario/map XY yaw가 UTM grid north 기준이고 `/Ego_topic.heading`이 true north
기준일 가능성을 강하게 지지한다.

## 공식 확인 요청 사항

1. Scenario JSON `initPosition.rot.yaw`는 UTM grid north 기준인가?
2. `/Ego_topic.heading`은 geographic/true north 기준인가?
3. 대회 제공 `/Competition_topic` 또는 INS heading은 어느 north 기준인가?
4. Lanelet/MGeo map pose와 ROS localization yaw를 정합할 때 참가팀이 자오선 수렴각을
   적용해야 하는가?
5. 적용해야 한다면 권장 수식, 부호, UTM zone과 map datum을 공식 문서로 제공할 수
   있는가?

## 영향

`1.347 deg`를 보정하지 않고 true-north heading으로 UTM map TF를 만들면 차량 전방
거리별 횡방향 오차는 대략 다음과 같다.

| 전방 거리 | 횡방향 오차 |
|---:|---:|
| 5 m | 0.12 m |
| 10 m | 0.24 m |
| 20 m | 0.47 m |
| 30 m | 0.71 m |

LiDAR-map 정합, Lanelet 선택, 객체 좌표 변환과 predicted path에 영향을 줄 수 있으므로
대회 전 heading reference 계약 확인이 필요하다.

## 현재 참가팀 임시 처리

K-City sample 위치에 한해 다음 고정 보정을 사용하고 있다.

```text
map_yaw_deg = Ego_topic.heading_deg - 1.347 deg
```

다만 공식 계약 확인 전에는 simulator bug fix로 간주하지 않으며, 대회 환경의 datum이나
map projection이 달라질 경우 고정값 대신 현재 위치의 meridian convergence를 계산해야
한다고 판단한다.
