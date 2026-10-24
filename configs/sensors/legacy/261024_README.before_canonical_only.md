# Sensor profile 선택

먼저 실행 역할을 고른다. 번호가 크거나 파일명이 최신이라는 이유로 profile을 바꾸지 않는다.

| 역할 | 선택할 Sensor | 알고리즘 실행·Network |
|---|---|---|
| 로컬 동시 개발: 한 PC의 MORAI + ROS | [Sensor_competition_local.json](Sensor_competition_local.json) | `udp` + `sensing`; [local Network](../network/ego_network_competition_local.json), loopback |
| LAN 개발: MORAI PC와 알고리즘 PC 분리 | [Sensor_competition_lan.json](Sensor_competition_lan.json) | `udp_lan` + `sensing_lan`; [LAN adapter Network](../network/ego_network_competition_lan.json) |
| 실대회 | MORAI PC: [현장 Sensor 원본](../morai_field_reference/AjouNice_sensor.json); 알고리즘 PC: [LAN Sensor](Sensor_competition_lan.json) | [현장 원본과 adapter의 구분](../morai_field_reference/README.md), [Network 역할표](../network/README.md) |

LAN 개발과 대회는 같은 wire 계약을 사용하지만 같은 검증 상태를 뜻하지 않는다.
보관된 canonical LAN endpoint는 MORAI `192.168.0.1` → 알고리즘 `192.168.0.10`이다.
실제 NIC IP가 다르면 같은 LAN 계열이라는 이유만으로 그대로 연결된다고 판단하지 않는다.
다른 주소의 validator 개발 LAN은 [담당 runbook](../../docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md)의
별도 설정·receipt를 따르며 원본을 덮어쓰지 않는다.
센서·UDP·TF·추론에 같은 실행의 calibration을 전달한다. 동일 UDP endpoint에 receiver를
중복 실행하거나 수집용 ROS/SaveSensorData profile을 주행용 UDP와 섞지 않는다.

## 내부 호환 파일: 사용자 선택의 기본값이 아님

| 남긴 파일 | 원경로 유지 이유 |
|---|---|
| [Sensor.json](Sensor.json), [Sensor_6.json](Sensor_6.json) | `validate_morai_udp_profiles.py` 필수 local mirror. localization/sensing/Docker·동결 dataset도 실제 참조 |
| [Sensor_4.json](Sensor_4.json) | Base5 trial/calibrated trial launch와 평가 도구의 실제 입력. Camera20Hz 계약을 자동 교체하지 않음 |
| [Sensor_5.json](Sensor_5.json) | 보호된 `control/trajectory_batch_runner/config/runtime/default.yaml`의 실제 Sensor 입력 |
| [Sensor_localization_gt_lightweight.json](Sensor_localization_gt_lightweight.json) | 명시적 `udp_localization_light`/`sensing_localization_light` 및 GPS-shadow validator. GPS/IMU만 유지 |

`Sensor_competition_local.json`도 동결 Base5/portable 수집·calibration 입력이다.
기존 release의 SHA를 현재 workspace profile에 억지로 맞추거나 구 output을 강제 resume하지 않는다.

## 현재 계약과 archive

canonical local/LAN은 IP 외 JSON 값이 같고, LAN은 현장 Sensor 원본과 JSON 값이 같다.
Camera4대30Hz, VLP-16 1대15Hz, GPS30Hz, IMU50Hz다. camera period의 현장 float32 표현
`0.03333333507180214`, LiDAR `frequencyIndex=2`와 `isVizPointCloud=false`를 유지한다.
`Sensor.json`·`Sensor_6.json` mirror도 canonical local과 같아야 한다.

[legacy catalog](legacy/README.md)는 active source/test/launch와 동결 runtime·release·portable
참조가 없는 옛 numbered/설치별/probe/layout 파일만 보관한다. JSON/YAML 원문은 변경하지 않았다.

저장소 루트에서 정적 계약을 확인한다. 이 검사는 live/LAN 주행 검증이 아니다.

```bash
python3 scripts/runtime/validate_morai_udp_profiles.py
```
