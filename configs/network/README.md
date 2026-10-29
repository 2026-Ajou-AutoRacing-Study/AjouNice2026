# 현재 Network profile

Local/LAN 개발·기존 production용 canonical JSON은 두 개다. mirror·옛 포트·ScenarioLoad
source·과거 수집용 profile은 [legacy](legacy/README.md)로 분리했다.

최신 planning이 배포한 `ego_network_competition_lan_jihun.json`은 팀장님 개발망 전용 예외다.
대회 기본값이 아니며 [개발망 안내](../../docs/planning/SCENARIO_VALIDATOR_LAN_QUICK_CHECK.md)를 따른다.

| 역할 | MORAI PC | 알고리즘 PC |
| --- | --- | --- |
| 로컬 동시 개발 | [competition_local](ego_network_competition_local.json) | 같은 파일, `udp` |
| LAN 개발 | 양쪽 IP를 맞춘 LAN wire 설정 | [competition_lan](ego_network_competition_lan.json), `udp_lan` |
| 본선 | 운영측 시나리오 Load 시 네트워크 자동 설정 | 기존 LAN adapter의 본선 중계 경로 적합성 점검 필요 |

본선은 [10월26일 통신 메뉴얼·운영측 답변](../../docs/competition/FINAL_NETWORK_CURRENT.md)을 우선한다.
Collision/Competition은 Client 판정프로그램이 중계한다. 9월 현장점검 원본으로
본선 시나리오 네트워크를 덮어쓰지 않는다. 현장점검 원본은 과거 재현 자료다.
[로컬 수신 차단과 환경별 실행](../../docs/competition/FINAL_RUNTIME_HANDOFF.md):
`udp`/`udp_lan`은 Ego·Object OFF, 개발용 원래 수신은 `udp_dev`/`udp_lan_dev`다.
JSON 값은 유지하며 runtime에서 선택한다. 실제 MORAI 무GT 수신·공식 START 초기화는 통과했고,
전체 주행·본선 판정프로그램 중계 시험은 대기다.

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
