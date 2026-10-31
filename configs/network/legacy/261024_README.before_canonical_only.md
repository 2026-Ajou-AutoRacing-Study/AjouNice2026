# Network profile 선택

| 역할 | MORAI PC에 로드할 Network | 알고리즘 PC 입력 |
|---|---|---|
| 로컬 동시 개발: 한 PC의 MORAI + ROS | [ego_network_competition_local.json](ego_network_competition_local.json) | 같은 파일, `udp`; loopback `127.0.0.1` |
| LAN 개발: 두 PC 분리 | 승인된 LAN wire 설정; 기존 계약은 [현장 원본](../morai_field_reference/2026_molit_comp_network.json) | [ego_network_competition_lan.json](ego_network_competition_lan.json), `udp_lan` |
| 실대회 | [현장 MORAI Network 원본](../morai_field_reference/2026_molit_comp_network.json) | [ego_network_competition_lan.json](ego_network_competition_lan.json), `udp_lan`; 실대회 제공 topic 계약 별도 확인 |

Sensor도 [같은 역할의 profile](../sensors/README.md)을 선택한다.
LAN 개발과 대회는 같은 adapter를 사용할 수 있지만 검증 상태·제공 topic이 같다는 뜻은 아니다.
canonical local/LAN은 UDP IP 외 값이 같으며, 보관 기본 LAN endpoint는 MORAI
`192.168.0.1` → 알고리즘 `192.168.0.10`이다. 실제 NIC IP가 다르면 이 JSON을 그대로
사용한 연결이 인증된 것이 아니다. IP·port·commType을 문서 정리로 바꾸지 않는다.

현장 원본은 MORAI PC용이다. 비활성 `rosConfig.Topic`에 `/dafault_topic`이 남아 있어
알고리즘 bridge에 그대로 넘기지 않는다. LAN adapter는 같은 필수 UDP endpoint에
`/CollisionData`, `/Competition_topic`, `/Ego_topic`, `/Object_topic` 등의 ROS 출력 이름을 준다.
profile에 topic이 있다는 사실은 실대회 제공을 보장하지 않는다.

## 내부 호환·수집용 파일: 주행 profile과 별도

| 남긴 파일 | 원경로 유지 이유 |
|---|---|
| [ego_network_1.json](ego_network_1.json) | canonical local의 필수 mirror; UDP profile 정적 검증과 보호된 control 설정도 참조 |
| [ego_network_local.json](ego_network_local.json) | validator `generate_network_profile.py`가 **local ScenarioLoad source**로 실제 읽음. 옛 전체 설정을 주행 기본값으로 선택하라는 뜻은 아님 |
| [ego_network_1_lan.json](ego_network_1_lan.json) | 같은 generator의 **LAN ScenarioLoad source**. 위 local 파일과 함께 유지 |
| [ego_network_mr_base5_savesensordata_collision_local.json](ego_network_mr_base5_savesensordata_collision_local.json) | 동결 Base5와 portable 수집의 ROS/SaveSensorData/Collision 계약; 주행용 UDP adapter가 아님 |
| [ego_network_mr_minimal_hybrid.json](ego_network_mr_minimal_hybrid.json) | 동결 dataset Base4 생성·녹화·profile generator와 runtime snapshot의 실제 경로 |

validator의 local/LAN 산출물은 위 source와 canonical Network에서 생성한
`planning_scenario_validator/config/internal/generated/ego_network_validator_{local,lan}.json`이다.
fullstack/미션 YAML이 고른 경로를 따른다. 주소가 다른 개발 LAN의 가변 host/destination
설정과 receipt는 [validator 담당 runbook](../../docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md)을
따르며 canonical JSON을 덮어쓰지 않는다.
이번 정리는 generator·생성물·default·동결 source를 변경하지 않았다.

[legacy catalog](legacy/README.md)는 현재 source/test/launch와 동결 runtime·portable에서
읽지 않는 옛 hybrid/port 변형만 원문 보관한다. 이름의 MR/MRP로 현재 선택을 추측하지 않는다.

```bash
python3 scripts/runtime/validate_morai_udp_profiles.py
```

이 정적 검증은 mirror, local/LAN IP-only 차이, 현장 Sensor 및 필수 UDP 채널을 확인한다.
live receipt-time·LAN 통신·주행 검증을 대신하지 않는다.
