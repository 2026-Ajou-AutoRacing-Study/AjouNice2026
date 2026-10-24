# 팀장님 LAN validator — 하나씩 확인하기

기존 실행 환경이 준비된 PC 기준입니다. 위에서부터 하나씩 실행하고 막힌 단계에서 멈춥니다.
재설치·ZIP 다운로드는 필요 없습니다.
공유 브랜치 `planning/autoware-universe-to-ros1`에서 진행합니다.

## 1. 업데이트

GUI와 기존 수동 스택을 정상 종료한 뒤:

```bash
cd ~/AjouNice2026
git pull --ff-only
source scripts/workspace/source_aw149.sh
```

이번 validator Python 수정은 루트 실행기로 사용하므로 별도 전체 빌드가 필요 없습니다.
pull이 실패하면 중단하고 출력부터 확인합니다. reset이나 강제 덮어쓰기는 하지 않습니다.

## 2. 중복 실행 확인

```bash
rosnode list
```

`/rosout`만 있거나 master가 아직 없으면 다음으로 갑니다.
udp·측위·인지·planning·제어 노드가 있으면 해당 수동 터미널을 정상 종료합니다.
**MORAI GUI의 Network Connect는 유지**합니다. validator가 ROS 스택을 직접 띄웁니다.

## 3. LAN 파일 준비 — 최초 한 번

전달받은 로그에서는 Ego Network만 LAN 주소로 바뀌었고 Sensor는 127.0.0.1이었습니다.
**MORAI PC `10.42.0.252`, ROS PC `10.42.0.1` 기준**으로 두 JSON과 개인 YAML 사본을 같이 만듭니다.
PC 주소가 달라졌다면 명령의 두 IP만 바꿉니다. 기존 개인 YAML 파일명이 다르면 첫 줄만 수정합니다.

```bash
YAML=base_ws/src/planning_addons/planning_scenario_validator/config/missions/full_course/workspace/full_course_starter.yaml
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/prepare_lan_profile.py \
  --yaml "$YAML" --morai-ip 10.42.0.252 --ros-ip 10.42.0.1
```

준비가 실패하면 여기서 멈춥니다. 원본·공용 파일은 그대로 두고, Git 제외 `workspace/`에 다음이 생깁니다.

- 새 YAML: `full_course_starter_lan.yaml`
- Sensor: `full_course_starter_lan_network/sensor_lan.json`
- Ego Network: `full_course_starter_lan_network/ego_network_lan.json`

출력된 **JSON 두 개를 MORAI PC로 옮겨** Sensor Network와 Ego Network에 각각 Load → Connect합니다.
ROS PC에 파일을 만들었다고 원격 MORAI 설정이 자동으로 바뀌지는 않습니다.
`hostIP`는 MORAI, `destinationIP`는 ROS 주소이며, 새 YAML의 두 파일 경로와 ScenarioLoad 주소도 함께 맞춰집니다.
센서 위치·주기·포트·모델·판단 설정은 바꾸지 않습니다. 같은 명령 재실행은 동일 파일이면 유지하고,
다른 내용의 기존 파일은 덮어쓰지 않습니다. 주소를 다시 바꿀 때는 `--output`으로 다른 개인 YAML 이름을 지정합니다.

새 YAML의 `morai.scenario_install_dir`는 **MORAI가 실제 읽는 Scenario 폴더의 공유/마운트 경로**여야 합니다.
기존 경로는 자동 변경하지 않습니다. ROS PC에 같은 이름의 로컬 폴더만 만드는 것으로는 원격 로드가 안 됩니다.

```bash
YAML="${YAML%.*}_lan.yaml"
./planning_validator "$YAML" --check
```

이 검사는 실제 NIC·방화벽·원격 파일 접근이나 주행 성공을 인증하지 않습니다.

## 4. 신호등 검사 — 시스템 Python 기준

```bash
/usr/bin/python3 scripts/runtime/check_traffic_light_env.py --device 0 --check-ros
echo "실제 종료코드: $?"
```

**PASS 출력과 종료코드 0 둘 다** 확인합니다. 실패하면 재설치부터 하지 말고 이 출력을 확인합니다.

## 5. 새 실험 시작

```bash
./planning_validator_gui
```

**불러오기/새로고침 → `full_course_starter_lan.yaml` 선택 → 새 실험 시작**을 누릅니다.
원본 파일명이 달랐다면 출력된 `*_lan.yaml`을 선택합니다.
과거 실행 재개는 이전 IP의 고정 사본을 사용하므로 이번 확인에는 쓰지 않습니다.
또 실패하면 결과 폴더의 `startup_error.json`과 `stack_logs/runner_console.log`를 보면 됩니다.
`TRAFFIC_LIGHT_CHECK` 실패는 `stack_logs/traffic_light_preflight.json`의 종료코드도 확인합니다.

Sensor/ScenarioLoad 주소 불일치는 로그로 확인된 문제입니다. 다만 과거 종료코드 2의 직접 원인은
그 로그만으로 확정되지 않아, 주소 수정 후에도 위 단계별 검사와 새 오류 기록을 확인해야 합니다.
