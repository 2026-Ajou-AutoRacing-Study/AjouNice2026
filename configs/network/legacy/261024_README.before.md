# configs/network

- `ego_network_competition_local.json`: 개인 PC 단독 실행용 정식 profile. 대회
  UDP 포트를 유지하고 host/destination IP만 `127.0.0.1`로 설정함.
- `ego_network_competition_lan.json`: 대회 알고리즘 PC의 `udp_lan`용 adapter
  profile. 현장 UDP endpoint와 ROS 출력 topic을 함께 정의함.
- `ego_network_1.json`: 기존 launch와 문서 호환을 위해 남겨 둔 local mirror.
  `ego_network_competition_local.json`과 항상 동일해야 함.
- `ego_network_local.json`: 260916 포트 통일 이전 설정의 보존본으로, 현재 실행에
  사용하지 않음.

정적 계약 검증은 다음 명령으로 수행한다.

```bash
python3 scripts/runtime/validate_morai_udp_profiles.py
```
