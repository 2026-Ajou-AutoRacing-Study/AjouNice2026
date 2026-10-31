# 풀스택 터미널 입력표

이 페이지를 운영 문서 맨 앞에 둔다. [Network·Sensor와 IP·포트](OPERATOR_NETWORK.md),
[출력 설명과 오류 대응](OPERATOR_TERMINALS.md)은 뒤에서 확인한다.
이 표는 프로그램 실행용이며 업데이트·설치·재빌드 명령은 포함하지 않는다.

## 모든 터미널에서 먼저

```bash
source ~/AjouNice2026/docs/common_alias.txt
```

저장소 경로가 다른 PC는 실제 checkout 경로로 바꾼다. 아래 명령은 **각각 별도 터미널**에서
1번부터 순서대로 실행한다. 표 전체를 한 터미널에 붙여넣지 않는다.

## 대회 현장

MORAI `192.168.0.1` → ROS `192.168.0.10`.
MORAI Network는 운영측 시나리오에서 자동 설정한다. Sensor는 제출·승인된 설정을 사용한다.

| 터미널 | 입력 |
|---|---|
| 1 | `rc` |
| 2 | `udp_lan_competition` |
| 3 | `sensing_lan` |
| 4 | `localization_competition` |
| 5 | `bevfusion` |
| 6 | `tracking` |
| 7 | `traffic` |
| 8 | `planning` |
| 9 | `planviz_competition` |
| 10 · 마지막 | `traj_fol_a` |

## 팀장님 대회 조건 시험

MORAI `10.42.0.252` → ROS `10.42.0.1`.
MORAI는 기존 팀장님 LAN Network·Sensor를 Load→Connect한다.
Ego·Object를 받지 않는 조건은 대회와 같지만 운영측 판정프로그램 중계를 검증하는 것은 아니다.

| 터미널 | 입력 |
|---|---|
| 1 | `rc` |
| 2 | `udp_lan_competition_teamlead` |
| 3 | `sensing_lan` |
| 4 | `localization_competition` |
| 5 | `bevfusion` |
| 6 | `tracking` |
| 7 | `traffic` |
| 8 | `planning` |
| 9 | `planviz_competition` |
| 10 · 마지막 | `traj_fol_a` |

## 팀장님 개발과 GT 비교

같은 `10.42.0.252 → 10.42.0.1` 개발망에서 Ego·Object 수신과 GT 표시를 유지한다.
공식 START에서 시작하는 표이며 측위는 계속 Competition·GPS·IMU를 사용한다.

| 터미널 | 입력 |
|---|---|
| 1 | `rc` |
| 2 | 아래 개발용 UDP 명령 |
| 3 | `sensing_lan` |
| 4 | `localization_competition` |
| 5 | `bevfusion` |
| 6 | `tracking` |
| 7 | `traffic` |
| 8 | `planning` |
| 9 | `planviz` |
| 10 · 마지막 | `traj_fol_a` |

터미널 2에 붙여넣는다.

```bash
udp_lan \
  network_config:="$AJOUNICE_ROOT/configs/network/ego_network_competition_lan_jihun.json" \
  sensor_config:="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_lan_jihun.json"
```

이 명령을 짧은 `udp_lan`만으로 대체하면 본선 IP 프로필을 읽으므로 팀장님 개발망과 달라진다.
`od_gt`로 탐지만 바꾸는 실험은 5번의 `bevfusion` **대신** `od_gt` 하나만 실행한다.
GT 객체를 표시하는 것과 GT 객체를 판단 입력으로 사용하는 것은 다르다.

## 제어기 실행 직전 확인

- 차량은 **공식 START·P·정지 상태**로 준비한다. 시나리오 START yaw를 임의 선보정하지 않는다.
- Competition·GPS·IMU가 들어오고 측위 초기화가 완료됐는지 확인한다.
- 검출·추적 입력과 모델 준비가 완료됐는지 확인한다.
- 초기 경로는 자동 요청된다. RViz에서 차량 위치·지도·차량 근처 연결된 궤적을 확인한다.
  Route Status는 표시용이며 Route A 요청·Reset 버튼으로 취급하지 않는다.
- 준비가 확인되면 **10번 제어기를 마지막에 실행한다. 실행하면 출발할 수 있다.**

이미 켜진 스택에 동일 명령을 중복 실행하지 않는다. 리스폰은 운영측 승인과 실제 정지 확인 뒤
수행하며 복귀 후 측위·차량 근처 새 궤적을 확인한다. **Ctrl+C는 차량 정지 명령이 아니다.**
