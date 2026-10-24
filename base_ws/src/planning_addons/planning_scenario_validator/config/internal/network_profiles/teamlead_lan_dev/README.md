# 팀장님 PC 임시 LAN-to-LAN 개발 설정

두 JSON은 Git에 포함되어 있어 pull 후 그대로 MORAI에 Load할 수 있습니다. 생성 명령은 필요 없습니다.

- MORAI PC: `10.42.0.252`
- ROS PC: `10.42.0.1`
- Sensor Network: [sensor_lan_dev.json](sensor_lan_dev.json)
- Ego Network: [ego_network_lan_dev.json](ego_network_lan_dev.json)

**대회/현장 설정이 아닙니다.** 대회에서는 현장점검에서 검증한 기존 절차와 설정을 사용합니다.
공용 local/LAN 기본값이나 현장 원본을 이 파일로 덮어쓰지 않습니다.

원본은 `configs/sensors/Sensor_competition_local.json`과 validator의
`config/internal/generated/ego_network_validator_local.json`입니다. 활성 UDP의 `hostIP`와
`destinationIP`만 위 주소로 바꿨고, 센서 보정·주기·포트와 ScenarioLoad `9095 → 9096`은 유지했습니다.
비활성 ROS/UDP 설정에 남은 loopback까지 모두 바꾸지 않습니다.

두 파일을 MORAI PC로 옮겨 각 Network 창에서 Load → Connect한 뒤, 개인 YAML도 같은 파일을
참조하도록 맞춥니다. 개인 YAML·수집 결과는 계속 Git 제외입니다.
[짧은 실행 안내](../../../../../../../../docs/planning/SCENARIO_VALIDATOR_LAN_QUICK_CHECK.md)의 3번을 따릅니다.

이 파일을 제공해도 NIC·방화벽·UDP 수신이나 Scenario 폴더 공유가 자동 설정되지는 않습니다.
IP가 바뀐 다른 개발망에는 그대로 쓰지 말고 개인 사본을 만드세요.
