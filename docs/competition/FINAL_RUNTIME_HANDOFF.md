# 대회 UDP 수신 차단과 인계

상태: 로컬 구현·설치·격리 시험, MORAI 무GT 수신·공식 START 초기화,
정차 전체 스택과 짧은 출발·궤적 단절 제동·P 복귀 통과.
풀코스·팀장님 LAN·본선 판정프로그램 중계 시험은 대기.
기준일 2026-10-29. [본선 통신 계약](FINAL_NETWORK_CURRENT.md)이 운영측 기준이다.

## 변경 범위와 복구 경로

`udp_competition` / `udp_lan_competition`만 `scripts/runtime/run_competition_udp.sh`를 거쳐
`enable_ego_vehicle_status:=false enable_object_info:=false`로 실행한다.
금지 항목은 JSON에 남아 있어도 소켓·ROS publisher를 만들지 않는다.
Competition·Collision 수신과 제어 송신, 센서 드라이버 설정은 유지한다.
ScenarioLoad 등 다른 항목은 이 수신기가 원래 사용하지 않는다.

기존 **`udp` / `udp_lan`은 개발·최종 튜닝용 Ego·Object 기본 ON을 유지한다.**
`udp_dev` / `udp_lan_dev`도 같은 개발 동작으로 호환 유지한다.
`origin udp`는 개발용, `origin udp_competition`은 무GT 대회용이며
`origin` 자체는 로그 표시만 바꾼다.
기존 직접 `roslaunch sensor_interface morai_udp_interface.launch`와 validator도
Ego·Object 기본 ON을 유지한다. 본선 실행에 validator를 쓰지 않는다.
동시에 두 UDP 실행을 켜지 않는다.

## 환경별 실행

아래는 각각 별도 터미널에서 실행하며 `ajou`를 먼저 수행한다.

| 환경 | UDP 명령 | Sensor TF |
|---|---|---|
| luv 로컬 무GT 시험 | `udp_competition` | `sensing` |
| 팀장님 LAN 무GT 시험 | `udp_lan_competition_teamlead` | `sensing_lan` |
| 본선 LAN | `udp_lan_competition` | `sensing_lan` |

팀장님 주소는 `10.42.0.252 → 10.42.0.1`, 본선은 `192.168.0.1 → 192.168.0.10`이다.
팀장님 프로필은 기존 `*_lan_jihun.json` Network·Sensor를 읽는다.
이전 `udp_lan_competition profile:=teamlead` 명령도 같은 동작으로 호환 유지한다.
두 LAN 프로필의 차이는 IP뿐이며 포트·센서 배치는 동일함을 대조했다.
TF 생성은 센서 IP를 쓰지 않으므로 기존 `sensing_lan`을 유지한다.
기존 명시적 `network_config:=... sensor_config:=...` 경로 인자도 그대로 전달한다.
Ego·Object를 true로 덮어쓰는 인자는 대회 진입점에서 거부하고 개발용 명령을 안내한다.

**측위는 반드시 `localization initial_position_source:=official_start`로 실행한다.**
같은 인자를 고정한 `localization_competition`도 제공한다.
bare `localization`의 개발 기본은 첫 Ego 위치를 기다리므로 무GT 입력에서는 사용하지 않는다.
checkpoint 복귀 시 START 초기화를 반복 적용하지 않는다.
기존 인지·판단·제어 실행 순서와 파라미터는 이번 변경 대상이 아니다.

대회 화면은 `planviz_competition` 또는 `planviz show_morai_gt:=false`다.
속도 HUD는 개발/대회 모두 Competition을 읽는다. GT OFF이면 박스 표시뿐 아니라
RViz 녹화 대상에서도 Ego/Object를 제외하고 Competition은 항상 기록한다.
기본 `planviz`의 개발용 GT 표시는 유지한다. `od_gt`, `localization_mrp`,
개발용 `judge`, validator·GNSS 지연 측정은 GT 의존성이 있어 본선 실행에서 제외한다.

## 실행 제한 점검: 소스 변경, 실주행 미검증

- 특정 v101 버전 문자열·절대 경로만 허용하던 launch gate를 제거했다.
  기본 지도는 v101 그대로이며 실제 파일·좌표계·명시한 SHA 일치 검사는 유지한다.
  다른 지도를 명시할 때는 planning·tracking·traffic의 지도와 경로 정의도 함께 맞춘다.
- 요약 로그 세션/원문 파일 생성 실패는 경고하고 알고리즘을 시작한다.
  잘못된 표시 주기·종료 대기시간은 기본값으로 복귀하며 주행 인자를 바꾸지 않는다.
  기록 실패 시 원문 증거가 없고, 자식이 종료된 뒤 wrapper가 기록 실패 코드2를 반환할 수 있다.
- 공식 START의 yaw 차이1도는 초기화를 막지 않고 경고만 한다. 실제 보정된
  Competition yaw를 사용하며 공식 yaw로 덮어쓰지 않는다. 다른 위치를 START로
  강제 초기화하지 않도록 GNSS 거리3m·freshness·TF 보호는 유지한다.
- 필수 모델/센서/지도 파일 누락·잘못된 형식·중복 제어/검출·금지 GT 활성화 방지는 유지한다.
  본선 실행 제한을 푼다는 이유로 잘못된 입력을 정상 데이터처럼 사용하지 않는다.

위 변경은 MORAI·실행 노드·설치본을 건드리지 않고 오프라인 검사한다.
HUD/launch/초기화기 설치본 반영은 별도 패키지 설치 후 새 실행부터 적용된다.

## 빌드와 전달

`sensor_interface` C++와 launch가 바뀌므로 팀장님 PC에서는 pull만으로 적용되지 않는다.
기존 AW149 build cache가 있는 PC에서는 스택을 정상 종료한 후 해당 패키지만 설치한다.

```bash
ajou
cmake --build base_ws/build_aw149/sensor_interface --target install -- -j2
source scripts/workspace/source_aw149.sh
/usr/bin/python3 scripts/runtime/test_competition_udp.py
```

cache가 없거나 다른 checkout을 가리키면 위 명령을 억지로 실행하지 않고 해당 PC의
기존 AW149 빌드 절차로 패키지를 구성한다. 소스·설치 launch의 `enable_object_info`를
확인한다. 새 PR·머지 전에는 팀장님에게 적용 완료로 안내하지 않는다.

후속 HUD·초기화·launch 제한 완화까지 받는 PC는 `sensor_interface`만 설치해서는
부족하다. 스택을 종료한 뒤 아래 패키지의 설치본도 갱신한다(동일 checkout의 기존
AW149 build cache 기준). 설치 후 각 터미널에서 `source ~/AjouNice2026/docs/common_alias.txt`를
실행한다. 저장소 경로가 다르면 그 checkout 경로를 사용한다.

```bash
for pkg in bringup localization_manager traffic_light_detection perception_object_pipeline ajounice_planning_integration; do
  cmake --build "base_ws/build_aw149/$pkg" --target install -- -j2 || break
done
```

모든 패키지 설치가 성공한 뒤 실행한다. 이 변경은 tracker 서브모듈·지도·가중치나
planning/control 알고리즘을 변경하지 않으며, 다른 작업 중인 로컬 설치본에는 자동 적용하지 않는다.

## 검증 범위

- mock·PTY·실행 인자·프로필 생성 검사와 기존 로그 회귀: 85개 통과.
- 실제 설치 수신기와 임시 loopback UDP·격리 ROS master로 Ego/Object 4조합 통과.
  금지 포트를 별도 소켓이 먼저 점유한 상태에서 정상 시작하고 금지 publisher가 없는지 확인했다.
- Competition은 36km/h→10m/s 변환과 호스트 receipt stamp, Collision은 발행을 확인했다.
- 필수 Collision 항목 누락 시 nonzero 종료, 제어는 무입력 자동 송신 없음과
  55byte·mode2/type1·기존 조향 정규화·송수신 endpoint를 확인했다.
- 기존 센서·상태 전체 수신 회귀에서 remote timestamp ±120초에도 receipt-time 유지.
- 승인 센서 파일·기존 canonical Network JSON은 수정하지 않았다.
  source와 설치 launch 일치 및 실제 설치 수신기 시험을 확인했다.

## MORAI 다음 시험

`prepare_competition_network.py --profile local --output-dir <새 폴더>`는 기존 설정을
복사해 허용 상태·제어 외 채널을 비활성화한 `ego_network_no_gt.json`과 SHA receipt를 만든다.
원본·기존 출력은 덮어쓰지 않고 MORAI에 자동 Load하지 않는다.
로컬 후보는 `artifacts/competition_udp_261029/local/ego_network_no_gt.json`에 준비했다.
차량 P·정지에서 후보를 Ego Network에 Load·Connect하고 기존 로컬 Sensor는 유지한다.
이는 로컬 직접 송신 시험용이며 **본선 MORAI에 Load하지 않는다.**
실제 MORAI에서 Ego·Object 소켓/publisher 없이 Competition·Collision·센서 수신을 확인했다.
공식 START 초기화는 `/Ego_topic` 구독 없이 성공했고 초기 yaw61.272°였다.
정차5초 동안 kinematic state250개(약50Hz), P 유지·제어 송신 OFF로 검사했다.
근거는 `artifacts/competition_udp_261029/local/live_inputs_02.json`과
`live_initialization_02.json`이다. 앞선 잘못된 시작 위치27.36m에서는 초기화 거부도 확인했다.
후속 정차 전체 스택에서는 검출6.54Hz·tracking/prediction30Hz·궤적9.95Hz·제어50Hz,
궤적age p95 225ms를 관측했다. RViz·영상·대용량 bag은 끄고 CPU 압력도 기록했다.
그때 전체 스택의 Ego/Object publisher·subscriber는 모두 없었다.
출발시험은 기존 planner·제어기·UDP를 사용하고, 궤적 단절 시 기존 제동을 정차에서
먼저 확인한 뒤 1.5초 이내 송신 시작 구간을 관측했다. 구간 내 속도는1.28m/s까지
관측됐고 최종 정지·P 복귀를 확인했다. 이 값은 전체 제동 구간 최고속도 측정이 아니다.
이 시험은 정상 목표 정차·제동거리·풀코스 또는 CPU가 모든 오차의 원인임을 인증하지 않는다.
근거는 `artifacts/competition_udp_261029/local/parked_stack_01.json`과
`planner_logs/analysis/261029_competition_network_smoke/short_drive_03/result.json`이다.
앞선 두 시험은 임시속도제한 STOP의 시험조건 미충족으로 출발 전에 중단됐다.
시험용 제한은 해제했고 production 코드·제어 파라미터는 변경하지 않았다.
다음 공식 START 초기화 전에 시나리오를 다시 Load한다. 시험으로 차량이 조금 이동했다.
본선 판정프로그램의 Competition/Collision 중계 경로는 팀장님 2PC에서 최종 확인한다.
