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

## 3. 사용하는 YAML 검사

```bash
YAML=base_ws/src/planning_addons/planning_scenario_validator/config/missions/full_course/workspace/full_course_starter.yaml
./planning_validator "$YAML" --check
```

파일명이 다르면 첫 줄만 수정합니다. 오류가 나오면 그 문구부터 해결합니다.
YAML이 가리키는 실제 JSON과 MORAI GUI에 Load한 JSON을 함께 확인합니다.

- Sensor와 Ego Network: `hostIP` = MORAI PC, `destinationIP` = ROS PC.
- YAML: `morai.scenario_load_host` = MORAI PC. 포트는 기존 값 유지.
- `scenario_install_dir`: **MORAI가 실제 읽는 Scenario 폴더**. 두 PC면 공유/마운트된 경로여야 합니다.

파일명에 local/lan이 붙었는지보다 실제 주소가 중요합니다. 기존 실행 사본은 고치지 않습니다.

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

같은 개인 YAML을 다시 불러오고 **새 실험 시작**을 누릅니다. 과거 실행 재개는 하지 않습니다.
또 실패하면 결과 폴더의 `startup_error.json`과 `stack_logs/runner_console.log`를 보면 됩니다.
`TRAFFIC_LIGHT_CHECK` 실패는 `stack_logs/traffic_light_preflight.json`의 종료코드도 확인합니다.
