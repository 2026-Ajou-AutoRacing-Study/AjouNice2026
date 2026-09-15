# 2026 MOLIT field MORAI reference files

이 디렉터리는 2026-09-15 현장점검에서 실제 MORAI 주행 PC에 로드하여
풀코스 주행에 성공한 원본 JSON을 보존한다.

## 파일 역할

- `AjouNice_sensor.json`: 현장 MORAI PC의 Sensor 설정 원본
- `2026_molit_comp_network.json`: 현장 MORAI PC의 Network 설정 원본

두 파일은 **MORAI PC에 로드하는 기준 자료**다. 알고리즘 PC에서 `udp_lan`이
직접 읽는 파일은 다음과 같다.

Git 보관본은 리뷰와 `git diff --check`를 위해 CRLF를 LF로만 정규화했다.
다운로드받은 원본의 SHA-256은 다음과 같으며 JSON 값은 보관본과 동일하다.

- `AjouNice_sensor.json`: `e63979c54e551f4d3fbf627ebbd0596f4b4786aea9f5a08609a22dcc973d856d`
- `2026_molit_comp_network.json`: `220eff23795f9520a8b91cf5838821a030fa840aba5659f584c15167ab75cfff`

- `configs/sensors/Sensor_competition_lan.json`
- `configs/network/ego_network_competition_lan.json`

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
