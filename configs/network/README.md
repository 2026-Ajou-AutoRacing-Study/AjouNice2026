# 현재 Network profile

현재 위치에는 production용 canonical JSON 두 개만 둔다. mirror·옛 포트·ScenarioLoad
source·과거 수집용 profile은 [legacy](legacy/README.md)로 분리했다.

| 역할 | MORAI PC | 알고리즘 PC |
| --- | --- | --- |
| 로컬 동시 개발 | [competition_local](ego_network_competition_local.json) | 같은 파일, `udp` |
| LAN 개발 | 양쪽 IP를 맞춘 LAN wire 설정 | [competition_lan](ego_network_competition_lan.json), `udp_lan` |
| 대회 | [현장 MORAI 원본](../morai_field_reference/2026_molit_comp_network.json) | 위 LAN adapter |

Sensor는 [센서 역할표](../sensors/README.md)를 따른다. LAN 보관 IP는
`192.168.0.1 → 192.168.0.10`이며 실제 NIC와 다르면 그대로 연결된다고 판단하지 않는다.
현장 Network 원본의 비활성 `rosConfig.Topic`은 bridge 출력 이름과 다르므로
알고리즘에 직접 넘기지 않는다. adapter의 UDP port·ROS topic·JSON 값은 바꾸지 않았다.

Validator의 ScenarioLoad 생성기는 필요한 두 source를 legacy에서 **명시적으로** 읽는다.
이동 전후 local/LAN 생성물은 바이트가 같다. 생성·실행은
[validator runbook](../../docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md)을 따른다.

완료된 수집·재개는 기존 동결 bundle을 사용한다. 과거 dataset/portable 소스·release·SHA는
수정하지 않았다. 옛 수집기의 root 경로 계약을 재현하려면 정리 전 `78d4f929` tree를
별도 worktree에서 사용한다. legacy 파일을 현재 production 폴더로 다시 복사하지 않는다.

```bash
python3 scripts/runtime/validate_morai_udp_profiles.py
```

이 검사는 canonical inventory·local/LAN IP-only 차이·field wire 계약을 확인하며
실제 LAN 통신·receipt-time·주행 품질을 대신하지 않는다.
