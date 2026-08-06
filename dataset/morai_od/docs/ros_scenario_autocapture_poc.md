# MORAI ROS 시나리오 자동 로드·캡처 PoC

## 목적

대량 manifest와 정교한 랜덤 배치 규칙을 만들기 전에 다음 최소 파이프라인이
실제 개인용 MORAI에서 끝까지 동작하는지 검증한다.

```text
짧은 Internal Scenario JSON 3개 생성
→ MORAI 현재 맵의 Scenario 폴더에 설치
→ ROS /ScenarioLoad
→ Ego 위치와 /Object_topic 객체 수 검증
→ ROS /SaveSensorData
→ 실제 파일 생성 확인
→ 저속으로 3 m 이동하며 6회 반복 캡처
```

자동 캡처 자체가 되지 않으면 대량 시나리오 생성은 의미가 없으므로, 먼저
정지 상태의 단일 프레임으로 로드와 저장만 검증한다. 그다음에만 Ego 이동을
켜고 마지막에 3개 시나리오 배치를 실행한다.

## 구현 파일

| 파일 | 역할 |
|---|---|
| `scripts/generate_morai_od_prototype.py` | 공식 waypoint와 MGeo로 시나리오 한 개 생성 |
| `scripts/generate_ros_capture_poc_set.py` | `od_poc_01`~`03`과 manifest 생성·설치 |
| `scripts/ros_scenario_autocapture_poc.py` | ROS 로드, 검증, 이동, 캡처, 파일 검증 |
| `manifests/ros_capture_poc.json` | 3개 장면과 기대 객체 수, 캡처 정책 |

시나리오 stem은 MORAI legacy transport의 30-byte 제한을 피하도록
`od_poc_01`처럼 짧게 유지한다. `/SaveSensorData.file_dir`도 같은 이유로
59 byte 이하만 허용한다.

## 현재 PoC 장면

| 장면 | waypoint anchor | NPC | 보행자 | 장애물 |
|---|---:|---:|---:|---:|
| `p01` / `od_poc_01` | 500 | 4 | 3 | 2 |
| `p02` / `od_poc_02` | 2516 | 5 | 4 | 2 |
| `p03` / `od_poc_03` | 3500 | 4 | 5 | 3 |

모두 sunny, 13:00 메타데이터로 두었다. 현재 Internal Scenario JSON에서
날씨·시간 필드가 검증되지 않았으므로 MORAI UI에서 한 번 수동 설정한다.
이번 PoC의 목표는 환경 다양성이 아니라 자동 로드와 자동 캡처 성공 여부다.

## 한 시나리오당 몇 장을 저장할 것인가

PoC와 초기 대량 수집의 권장 기본값은 **한 시나리오당 6장**이다.

- Ego 목표 속도: 3.6 km/h = 1 m/s
- 프레임 간 이동거리: 0.6 m
- 총 이동거리: 약 3 m
- 장면당 캡처: 최초 위치 1장 + 이동 중 5장

정적 객체를 두고 20 Hz로 10~20장을 연속 저장하면 거의 같은 장면이 반복되어
학습 데이터 수만 부풀 가능성이 크다. 시나리오를 많이 만들 수 있다면
**많은 장면 × 5~8장**이 **적은 장면 × 20장**보다 낫다. 이후 가림,
근거리 소형 객체, 곡선처럼 변화가 큰 장면만 8~12장으로 늘린다.

예를 들어 5,000개 장면에서 6장씩 저장하면 30,000 프레임이다. 실제 split은
프레임이 아니라 시나리오 단위로 나누어 같은 정적 배치가 train과 val에
동시에 들어가지 않게 해야 한다.

## Ego 이동 방식

팀 제어 스택은 이 PoC에 사용하지 않는다. 자동화 스크립트가 공식
`2026_molit_comp_global_path.txt`를 따라 다음과 같이 제한된 이동만 수행한다.

1. `/Ego_topic`에서 현재 위치와 heading을 읽는다.
2. 가장 가까운 공식 waypoint와 3 m 전방 target을 찾는다.
3. 작은 pure-pursuit 조향과 `CtrlCmd.longlCmdType=2` 속도 명령을 20 Hz로 낸다.
4. 약 0.6 m 이동할 때마다 `/SaveSensorData`를 한 번 보낸다.
5. 4 m 이동, 2.5 m 경로 이탈, 40° heading 오차, 충돌 또는 timeout이면
   즉시 brake 명령을 보낸 뒤 실패로 종료한다.

이 방식은 “도로 전체를 안정적으로 주행하는 새 제어기”가 아니다. 데이터
관측점을 3 m 정도 바꾸기 위한 제한형 micro-motion이다. MORAI built-in
controller는 목적지와 내부 route 설정 의존성이 있어 대량 JSON마다 관리가
늘어나므로 첫 PoC에는 사용하지 않는다.

시나리오 로드 후 저장된 keyboard 제어 모드로 돌아갈 수 있으므로
`/Service_MoraiEventCmd`를 호출해 매 장면 AV-ExternalCtrl로 전환한다.
해당 service가 없는 MORAI 버전에서는 UI에서 AV-ExternalCtrl을 선택하고
`--skip-control-mode-service`를 사용하되, 다중 장면 자동화 전에 매번 제어
모드가 유지되는지 확인해야 한다.

## 1. 시나리오 생성 및 MORAI 폴더 설치

개인용 MORAI에서 현재 사용하는 K-City 2025 시나리오 폴더를 지정한다.

```bash
# PoC 시나리오 3개 생성 및 개인용 MORAI에 설치
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_ros_capture_poc_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```

설치 확인:

```bash
# MORAI에 설치된 짧은 시나리오 파일 확인
find \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  -maxdepth 1 -type f -name 'od_poc_*.json' -print
```

## 2. MORAI UI 준비

1. 개인용 MORAI에서 `R_KR_PR_K-city_2025` 맵을 연다.
2. sunny, 13:00로 설정한다.
3. 팀의 현재 Camera 4대 + 3D LiDAR 센서 구성을 적용한다.
4. Camera와 LiDAR에서 필요한 GT(Intensity, Semantic, Instance 등)를 켠다.
5. 모든 GT를 한 번에 저장하려면 각 센서의 **Capture Mode**를 켠다.
6. ROS1 network에서 다음 인터페이스를 켠다.

```text
Subscriber: /ScenarioLoad, /SaveSensorData, /ctrl_cmd
Publisher:  /Ego_topic, /Object_topic, /CollisionData, /clock
Service:    /Service_MoraiEventCmd
```

Capture Mode가 켜진 Camera/LiDAR는 평상시 Sensor List에서 비활성 또는
빨간 상태로 보일 수 있고 `/SaveSensorData` 순간에만 활성화된다. 이는 문서상
정상 동작이다.

## 3. ROS master

터미널 1:

```bash
# ROS master 실행
source /opt/ros/noetic/setup.bash
roscore
```

## 4. 연결 사전 점검

터미널 2:

```bash
# MORAI ROS topic과 제어 모드 service 연결 확인
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosparam set /use_sim_time false

rostopic info /ScenarioLoad
rostopic info /SaveSensorData
rostopic info /ctrl_cmd
rostopic echo -n 1 /Ego_topic
rostopic echo -n 1 /Object_topic
rosservice list | grep -F /Service_MoraiEventCmd
```

`/ScenarioLoad`, `/SaveSensorData`, `/ctrl_cmd`에는 MORAI subscriber가 각각
하나 이상 보여야 한다.

## 5. 저장 경로와 dry-run

저장 경로는 MORAI가 실행되는 같은 PC의 절대 경로이며 legacy 제한 때문에
짧게 둔다.

```bash
# 짧은 외부 데이터 경로 생성 및 자동화 설정 정적 검사
mkdir -p /home/luv/morai_cap/poc

cd ~/AjouNice2026

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc \
  --dry-run
```

## 6. Phase A: 자동 로드 + 정지 1장

이 단계가 가장 중요하다. 이동을 전혀 하지 않고 `p01` 로드, Ego 위치,
객체 수, 실제 센서 파일 생성을 확인한다.

```bash
# p01 자동 로드 후 정지 상태 센서 캡처 1회 검증
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc \
  --scenario p01 \
  --motion-mode stationary \
  --captures-per-scene 1
```

성공 기준:

```text
Scenario verified: od_poc_01
Capture ...: files=N, verified=True
auto-capture complete
```

실제 산출물:

```bash
# MORAI가 생성한 센서별 파일과 실행 보고서 확인
find /home/luv/morai_cap/poc -maxdepth 3 -type f | sort | head -n 80
cat /home/luv/morai_cap/poc/ros_capture_poc_report.json
```

이 단계가 실패하면 대량 실행으로 넘어가지 않는다.

## 7. Phase B: 한 장면 저속 이동 + 6장

MORAI 주변에 사람이 없고 차량이 움직여도 안전한 상태에서 실행한다.
팀 trajectory follower나 다른 `/ctrl_cmd` publisher는 모두 종료한다.

```bash
# p01에서 공식 waypoint를 따라 약 3 m 이동하며 6장 캡처
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc \
  --scenario p01 \
  --motion-mode route-follow
```

다른 노드가 동시에 제어하지 않는지 확인:

```bash
# /ctrl_cmd publisher가 자동화 노드 하나뿐인지 확인
rostopic info /ctrl_cmd
```

## 8. Phase C: 3개 시나리오 자동 배치

Phase A와 B가 모두 통과한 뒤에만 실행한다.

```bash
# 세 장면을 순차 로드하고 각 장면에서 6장씩 자동 캡처
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc \
  --motion-mode route-follow
```

총 기대 캡처 수는 18개다. 하나의 capture 명령은 Camera/LiDAR 각각의 여러
GT 파일을 만들므로 실제 파일 수는 18보다 훨씬 많다.

## 실패 해석

| 증상 | 의미와 조치 |
|---|---|
| ScenarioLoad subscriber 없음 | MORAI ROS Subscriber 설정 확인 |
| Ego 위치가 기대값과 다름 | 현재 맵 폴더, file stem, ScenarioLoad ego 옵션 확인 |
| 객체 수 불일치 | 시나리오 로드 실패 또는 actor active 상태 확인 |
| 새 파일 0개 | Capture Mode, GT 설정, 경로 권한, `file_dir` 지원 확인 |
| Event service 없음 | service를 켜거나 UI 수동 전환 + skip 옵션 |
| 경로/heading safety abort | 이동을 중단한 정상 보호 동작; anchor·heading 검토 |
| collision abort | 해당 장면 폐기 또는 객체 clearance 수정 |

`--allow-unverified-capture`는 MORAI가 `file_dir`를 무시하는지 진단할 때만
사용한다. 대량 수집에서는 파일 생성이 검증되지 않은 캡처를 성공으로
간주하면 안 된다.

## PoC 이후

이 PoC가 성공하면 다음 순서로 확장한다.

1. 객체 유형 catalog와 차선·보도·횡단보도 제약 정교화
2. 장면별 seed와 rejection reason을 manifest에 기록
3. 날씨 2종 × 시간 3종 batch 분리
4. 시나리오당 5~8장 기본, 어려운 장면만 증량
5. 센서 파일 completeness, timestamp, GT 개수 자동 검사
6. 시나리오 단위 train/val/test split
