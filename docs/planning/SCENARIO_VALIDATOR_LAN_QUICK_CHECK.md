# 팀장님 PC 임시 LAN-to-LAN 개발 — 하나씩 확인하기

**대회용 네트워크 설정 안내가 아닙니다.** 팀장님 PC의 `10.42.0.*` 개발망에서 validator를
확인하기 위한 임시 절차입니다. **대회에서는 현장점검에서 검증한 기존 실행 절차와 설정을 유지합니다.**
아래 개발 전용 파일이나 ScenarioLoad 자동화를 대회용 설정으로 바꿔 쓰지 않습니다.

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

## 3. Git에 포함된 팀장님 개발망 설정 사용

전달받은 로그에서는 Ego Network만 LAN 주소로 바뀌었고 Sensor는 127.0.0.1이었습니다.
**MORAI PC `10.42.0.252` → ROS PC `10.42.0.1`**로 맞춘 JSON 두 개를 Git에 넣었습니다.
이 주소는 이번 PC 개발망의 IP이지 대회/현장 IP가 아닙니다. **별도 생성 명령은 필요 없습니다.**

파일 위치를 확인합니다.

```bash
NET=base_ws/src/planning_addons/planning_scenario_validator/config/internal/network_profiles/teamlead_lan_dev
realpath "$NET/sensor_lan_dev.json" "$NET/ego_network_lan_dev.json"
```

이 **JSON 두 개를 MORAI PC로 옮겨** Sensor Network와 Ego Network에 각각 Load → Connect합니다.
pull만으로 원격 MORAI에 자동 Load되지는 않습니다.

VSCode에서 기존 개인 YAML의 아래 **세 값만** 맞춥니다. **`morai` 블록 전체를 교체하지 말고**
`scenario_install_dir` 등 기존 항목은 유지합니다. 개인 YAML·결과는 계속 Git 제외입니다.

```yaml
network:
  sensor_json: base_ws/src/planning_addons/planning_scenario_validator/config/internal/network_profiles/teamlead_lan_dev/sensor_lan_dev.json
  ego_network_json: base_ws/src/planning_addons/planning_scenario_validator/config/internal/network_profiles/teamlead_lan_dev/ego_network_lan_dev.json
morai:
  scenario_load_host: 10.42.0.252
```

활성 UDP 주소만 맞췄으며 센서 위치·주기·포트는 원본과 같습니다. 모델·판단 설정도 바꾸지 않습니다.
`morai.scenario_install_dir`는 **MORAI가 실제 읽는 Scenario 폴더의 공유/마운트 경로**여야 합니다.
ROS PC에 같은 이름의 로컬 폴더만 만드는 것으로는 원격 로드가 안 됩니다.

```bash
YAML=base_ws/src/planning_addons/planning_scenario_validator/config/missions/full_course/workspace/full_course_starter.yaml
./planning_validator "$YAML" --check
```

개인 YAML 파일명이 다르면 첫 줄만 수정합니다. 과거 실행 폴더의 고정 사본은 수정하지 않습니다.
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

**불러오기/새로고침 → 3번에서 수정한 개인 YAML 선택 → 새 실험 시작**을 누릅니다.
과거 실행 재개는 이전 IP의 고정 사본을 사용하므로 이번 확인에는 쓰지 않습니다.
또 실패하면 결과 폴더의 `startup_error.json`과 `stack_logs/runner_console.log`를 보면 됩니다.
`TRAFFIC_LIGHT_CHECK` 실패는 `stack_logs/traffic_light_preflight.json`의 종료코드도 확인합니다.

Sensor/ScenarioLoad 주소 불일치는 로그로 확인된 문제입니다. 다만 과거 종료코드 2의 직접 원인은
그 로그만으로 확정되지 않아, 주소 수정 후에도 위 단계별 검사와 새 오류 기록을 확인해야 합니다.

이전 안내로 만든 `*_lan.yaml`/`*_lan_dev.yaml`과 개인 JSON은 자동 수정하지 않습니다.
사용 중인 개인 YAML의 세 값만 위 Git 파일로 연결하면 됩니다. IP가 다른 개발망에서만
[개인 사본 생성 도구](../../base_ws/src/planning_addons/planning_scenario_validator/scripts/prepare_lan_profile.py)의
`--help`를 참고합니다. 어느 방식도 대회 설정을 바꾸는 절차는 아닙니다.
