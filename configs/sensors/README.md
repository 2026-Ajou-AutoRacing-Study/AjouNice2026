# 현재 Sensor profile

Production용 canonical JSON은 두 개다. 번호별 원본·mirror·과거 시험용은
모두 [legacy](legacy/README.md)에 보관하며 최신 번호를 골라 쓰는 방식은 폐기한다.

최신 planning이 배포한 `Sensor_competition_lan_jihun.json`은 팀장님 개발망 전용 예외다.
대회 기본값이 아니며 [개발망 안내](../../docs/planning/SCENARIO_VALIDATOR_LAN_QUICK_CHECK.md)를 따른다.

| 역할 | MORAI / 알고리즘에서 선택 |
| --- | --- |
| 로컬 MORAI + ROS | [Sensor_competition_local.json](Sensor_competition_local.json), `udp` + `sensing` |
| LAN 개발 | [Sensor_competition_lan.json](Sensor_competition_lan.json), `udp_lan` + `sensing_lan` |
| 대회 | 제출·검수한 센서 설정과 알고리즘 LAN 수신·TF 설정을 대조. 9월 원본은 재현 참고 |

본선 네트워크는 [최신 통신 계약](../../docs/competition/FINAL_NETWORK_CURRENT.md)을 따른다.
고정 카메라3대는 센서 파일에서 삭제하지 않는다. 미사용 네트워크 Disconnect는 가능하다.
시나리오 네트워크 자동 설정과 센서 파일 Load·Connect는 구분한다.

E12/E24 production은 LAN canonical을 사용한다. 로컬 실행·E25 복귀·공통 bringup의
옛 `Sensor.json`·`Sensor_6.json` 참조는 바이트가 동일한 local canonical로 통일했다.
Sensor4/5는 현행과 주기·LiDAR 설정이 달라 canonical로 바꿔치기하지 않았다.
과거 trial·bag 평가·경량 측위 시험은 필요한 경우 legacy 원본 경로를 명시한다.
Docker 내부 `/config/Sensor.json`은 mount 이름일 뿐 저장소의 중복 파일이 아니다.

현재 camera4대30Hz·VLP16 15Hz·GPS30Hz·IMU50Hz, extrinsic·field 원본·JSON 값은 불변이다.
LAN 보관 IP는 MORAI `192.168.0.1` → 알고리즘 `192.168.0.10`이다.
실제 NIC가 다른 LAN은 양쪽 유효 설정을 맞추며 [validator runbook](../../docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md)을 따른다.

Network는 [같은 역할표](../network/README.md)를 따른다. 정적 검사는 실제 통신·주행 인증이 아니다.

```bash
python3 scripts/runtime/validate_morai_udp_profiles.py
```
