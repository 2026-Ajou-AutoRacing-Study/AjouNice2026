# 본선 통신환경과 시나리오 네트워크

상태: CURRENT — 운영측 전달 계약. 실행 JSON·설치본 적합성 검증과는 구분한다.
2026-10-29 사용자가 전달한 YeongHu Park 작성 「본선 통신환경 세팅 메뉴얼」
(페이지 표시일 2026-10-26)과 Discord 질답을 기준으로 한다.
[공식 문서 링크](https://morai.atlassian.net/wiki/external/Y2Y1YmJhN2EwNDBjNGYwZDgyNmU5OTMyNTQyZTI2MTM)의
내용은 사용자가 붙여준 본문을 기준으로 정리했다. 외부 페이지 별도 대조는 아니다.

## 본선 네트워크의 소유권

- 운영측 답변: 본선 시나리오에 네트워크가 저장돼 있어 시나리오 Load 시 자동 설정된다.
- 사용 불가 채널도 포함돼 있어 네트워크 파일 자체는 공유하지 않는다.
  운영측은 위 메뉴얼대로 설정돼 있다고 답변했다.
- 9월 현장점검 Network JSON으로 본선 시나리오 네트워크를 덮어쓰지 않는다.
- 센서 파일은 규정의 운영 절차에 따라 별도 Load·Connect한다.
  시나리오 네트워크 자동 설정이 센서 파일 자동 적용까지 뜻하지는 않는다.

## PC 연결

Client Windows PC와 참가팀 Ubuntu PC를 이더넷으로 직접 연결한다.

| 항목 | Client PC | 참가팀 PC |
|---|---|---|
| IPv4 | `192.168.0.1` | `192.168.0.10` |
| 마스크 | `255.255.255.0` | `255.255.255.0` |
| 게이트웨이 | 생략 | 생략 |

Windows: Defender 방화벽 고급 설정의 인바운드·아웃바운드에서
`ICMPv4-In / ICMPv4-Out` 규칙을 사용한다. 메뉴얼은 이더넷 속성 공유 탭의
체크항목 두 개도 선택하도록 안내하지만 항목명은 전달 본문에 없다.
방화벽 전체 해제나 다른 공유 설정을 임의로 추가하지 않는다.
ICMP 허용은 ping용이며 UDP 정상까지 보장하지 않는다.

Ubuntu: 설정 → 네트워크 → 유선 톱니바퀴 → IPv4 수동에서 위 값을 입력하고
연결을 OFF→ON한다. Ubuntu에서 `ping 192.168.0.1`, Windows에서
`ping 192.168.0.10`으로 상대 PC에 도달하는지 확인한다.

## 제어와 상태 패킷의 전달 경로

```text
참가팀 제어 명령 → Client MORAI
Client MORAI 상태 → Client 판정프로그램 → 참가팀 Collision / Competition 수신
Client MORAI 센서 → 참가팀 센서 수신
```

| 메뉴얼 항목 | Host IP | Destination IP |
|---|---|---|
| MORAI Cmd Control | `192.168.0.1` | `192.168.0.10` |
| MORAI Publisher, Subscriber, Service | `127.0.0.1` | `127.0.0.1` |
| 센서 | `192.168.0.1` | `192.168.0.10` |

**Collision Data·Competition Vehicle Status는 판정프로그램이 참가팀에 전달한다.**
Client 내부 `127.0.0.1`을 참가팀 Ubuntu의 bind 주소로 복사하지 않는다.
운영측의 판정 전용 채널은 참가팀 수신 대상으로 추가하지 않는다.

| 네트워크명 | 방향 | 메뉴얼 Host Port | 메뉴얼 Destination Port |
|---|---|---:|---:|
| Ego Ctrl Cmd | Sim ← User | 9093 | 9094 |
| Collision Data | Sim → User | 9091 | 9092 |
| Competition Vehicle Status | Sim → User | 9088 | 9099 |

Host/Destination은 **MORAI 메뉴얼의 표기**다. 제어와 상태의 방향이 다르므로
참가팀 송수신 bind 포트로 일괄 해석하지 않는다. 현재 LAN adapter에는 같은 포트
숫자가 있지만 bridge의 bind/send 해석과 실제 판정프로그램 중계 수신은 별도 점검한다.
중계 패킷의 송신 IP·source port·주기·시각 필드가 MORAI 직접 송신과 동일한지는
전달 문서에 없다. 개발 직접 송신 시험을 본선 중계 경로 검증으로 간주하지 않는다.

센서의 Host/Destination Port는 자유이며 위 네트워크 포트 및 다른 수신기와
충돌하지 않게 지정한다. 새 센서별 고정 포트나 extrinsic 변경은 안내되지 않았다.

## 실행 준비에 반영할 사항

로컬 구현·설치·격리 패킷 시험 결과와 팀장님 전달 절차는
[UDP 수신 차단 인계](FINAL_RUNTIME_HANDOFF.md)를 따른다. 실제 MORAI·본선 중계 검증은 별도다.

- Ego Vehicle Status·Object Info·GT 신호·신호 제어·ScenarioLoad를 본선 실행에서 제외한다.
  `enable_ego_vehicle_status:=false` 하나가 다른 개발 채널까지 제거한다는 뜻은 아니다.
- 공식 START는 `localization initial_position_source:=official_start`로 초기화한다.
  Competition 중계 시작 전에는 입력 대기할 수 있으므로 실제 운영 순서에서 확인한다.
- 본선용 ROS adapter·센서 프로필·alias 확정은 별도 작업이다. 기존 canonical과
  9월 현장점검 원본은 자동으로 본선 인증된 파일이 아니다.
- 로컬에서는 같은 허용 입력과 실행 인자를 검증한다. 팀장님 2PC에서는 판정프로그램
  중계·포트 점유·상태 수신·제어·센서 수신을 최종 검증한다.
  수신 timestamp·GNSS 지연 보상값은 새 경로에서 측정 없이 변경하지 않는다.

## 함께 전달된 규정 본문 갱신

2026-10-29 재전달 전문도 v1.2로 표시되지만 10월15일 보관본과 다음 내용이 다르다.
PC 제출 `08:00–09:00`, 오전조 `09:00–12:00`, 점심 `12:00–13:00`,
개회식 `13:30–14:00`, 오후조 `13:30–17:30`(개회식 동시 진행)이다.
실행 후 조작 금지에는 체크포인트 Respawn 예외가 추가됐고, 고정 카메라3대는
센서 파일에서 삭제 불가(미사용 네트워크 Disconnect 가능), 날씨·시간 적용 위치는
전팀 동일이라고 명시됐다. Respawn 예외를 코드·파라미터 수정 허가로 확대하지 않는다.
허용 UDP7종·금지 GT 입력·30초 신호 패널티는 기존과 같다.
