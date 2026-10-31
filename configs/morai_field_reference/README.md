# 2026 MOLIT field MORAI reference files

이 디렉터리는 2026-09-15 현장점검에서 실제 MORAI 주행 PC에 로드하여
풀코스 주행에 성공한 원본 JSON을 보존한다.

## 파일 역할

- [AjouNice_sensor.json](AjouNice_sensor.json): 현장 MORAI PC의 Sensor 설정 원본
- [2026_molit_comp_network.json](2026_molit_comp_network.json): 현장 MORAI PC의 Network 설정 원본

## 합의한 대회 실행 기준

**2026-10-29 본선 계약으로 대체:** [최신 통신 메뉴얼과 운영측 답변](../../docs/competition/FINAL_NETWORK_CURRENT.md)에
따라 본선 네트워크는 운영측 시나리오 Load 시 자동 설정된다. 위 9월 원본으로
본선 네트워크를 덮어쓰지 않는다. 이하 Load 절차는 현장점검 재현 기준이며
본선 중계 경로 검증 완료를 뜻하지 않는다. Ego 미사용·공식 START 초기화 원칙은 유지한다.

현장점검을 재현할 때 MORAI PC에는 위 원본을 로드한다. 이 PC의 보관 경로는 각각
`~/AjouNice2026/configs/morai_field_reference/AjouNice_sensor.json`과
`~/AjouNice2026/configs/morai_field_reference/2026_molit_comp_network.json`이다.
알고리즘 PC는 아래 명령을 **각각 별도 터미널**에서 사용한다. 기존 alias를 준비한 뒤
공식 START에서 정지 상태로 출발하는 기준이며 기본 alias 자체를 바꾼 것은 아니다.

```bash
udp_lan enable_ego_vehicle_status:=false
sensing_lan
localization initial_position_source:=official_start
```

Ego 수신·Ego TF·Ego 초기 위치를 사용하지 않고, 주행 측위는 GPS·IMU·Competition을 쓴다.
`udp_lan`은 현장 원본 Network를 직접 읽지 않고 아래 LAN adapter를 사용한다.
보관 endpoint는 MORAI `192.168.0.1` → 알고리즘 `192.168.0.10`이다. 실제 두 NIC와
다르면 원본을 덮어쓰지 말고 양쪽 실행 설정을 맞춘다.

공식 START 초기화는 fresh GNSS·Competition heading·TF와 위치/방향 검사를 요구한다.
중간 위치·checkpoint 복귀에 START를 강제로 적용하지 않는다. 자세한 조건과
초기화 발행 확인은 [Localization CURRENT](../../docs/localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md#초기화checkpoint)를 따른다.
이 문서는 실행방법의 합의이며 새 대회 LAN 주행 검증 완료 기록이 아니다.

## 원본과 알고리즘 adapter 구분

두 파일은 **MORAI PC에 로드하는 기준 자료**다. 알고리즘 PC에서 `udp_lan`이
직접 읽는 파일은 다음과 같다.

Git 보관본은 리뷰와 `git diff --check`를 위해 CRLF를 LF로만 정규화했다.
다운로드받은 원본의 SHA-256은 다음과 같으며 JSON 값은 보관본과 동일하다.

- `AjouNice_sensor.json`: `e63979c54e551f4d3fbf627ebbd0596f4b4786aea9f5a08609a22dcc973d856d`
- `2026_molit_comp_network.json`: `220eff23795f9520a8b91cf5838821a030fa840aba5659f584c15167ab75cfff`

- `configs/sensors/Sensor_competition_lan.json`
- `configs/network/ego_network_competition_lan.json`

개인 PC 단독 실행에서는 아래 loopback profile을 MORAI와 알고리즘 양쪽의
기준으로 사용한다.

- `configs/sensors/Sensor_competition_local.json`
- `configs/network/ego_network_competition_local.json`

현장 Network 원본은 활성 채널에 UDP(`commType=1`)를 사용하지만, 비활성
`rosConfig.Topic` 필드에는 `/dafault_topic`이 남아 있다. 우리 UDP-to-ROS
브리지는 이 필드를 출력 ROS 토픽 이름으로도 사용하므로 현장 원본 Network
JSON을 `udp_lan` 입력으로 직접 넘기지 않는다.

반대로 알고리즘용 LAN 파일은 현장 원본의 핵심 UDP wire contract를 유지하면서
`/Ego_topic`, `/Object_topic`, `/Competition_topic`, `/CollisionData` 등 ROS 출력
토픽을 명시한 adapter profile이다.

## 검증

```bash
cd ~/AjouNice2026
python3 scripts/runtime/validate_morai_udp_profiles.py
```

검증기는 다음을 별도로 확인한다.

1. 알고리즘용 local profile과 LAN profile이 IP 외에는 같은지
2. 현장 MORAI 원본과 알고리즘용 LAN profile의 센서 및 핵심 UDP 포트 계약이
   일치하는지

현장 원본을 갱신할 때는 파일을 그대로 교체한 뒤 이 검증을 통과시켜야 한다.
