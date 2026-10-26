# 신호등 움찔 보완·명시적 v94 팀 적용

기준: 2026-10-26. 기존 ROS1 workspace·NVIDIA 환경이 있는 팀 PC용이다.
일반 `planning`·`traffic`·`tracking`의 지도 기본은 팀 최신 **v88**을 유지한다.
v94는 v88 도로/속도/Route A/B를 보존하고 검증한 정지선8개 node만 옮긴 선택 profile이다.

## 정차하고 스택 종료 후 업데이트

```bash
cd ~/AjouNice2026
git pull --ff-only origin planning/autoware-universe-to-ros1
bash scripts/runtime/update_competition_runtime.sh --map-version 94
```

업데이트 스크립트는 노드·주행을 시작하지 않고, C++ 의존 모듈과 base 설치본·신호등 모델/환경을 맞춘다.
LC의 공개 구조체를 소비하는 sibling 모듈도 함께 빌드해 일부만 새 library가 되는 경우를 피한다.
중간 빌드/설치 실패 시 일부만 갱신될 수 있으므로, 성공 문구가 나오도록 재실행한 뒤 스택을 켠다.
v94가 이미 정확한 SHA로 있으면 지도 다운로드·Drive 인증 없이 재사용한다. 다른 내용의 기존 파일은
덮어쓰지 않는다. 없으면 전용 rclone remote로 이 지도 한 파일만 받고 SHA를 검사한다.
다른 PC의 명시적 remote 이름은 `--map-remote NAME`, 오프라인 지도는 먼저
`python3 scripts/map/prepare_kcity_v94.py --source /path/to/lanelet2_map_v94.osm`으로 준비한다.
luv PC의 기본은 `ajou_drive_private`이며 인증 실패 시 reconnect한다. 공용 OAuth로 자동 우회하지 않는다.
이미 검증된 별도 Python을 쓰면 업데이트 명령 뒤 `--existing-python /absolute/bin/python`을 추가한다.

## 기존 풀스택의 각 담당 터미널에서

기존 `rc`·UDP/LAN·측위·센서·BEVFusion·제어 절차는 그대로 사용한다. 아래는 **각각 별도 터미널**이다.
기존 canonical 노드를 동시에 두 벌 띄우지 않는다. 최신 공용 alias가 준비된 셸 기준:

```bash
planning map_version:=94
traffic map_version:=94
tracking map_version:=94
```

`planning` alias가 없는 경우에는 source 후
`roslaunch ajounice_planning_integration competition_planning_v94.launch`가 같은 지도 profile을 선택한다.
통합 `morai_planning_stack.launch` 사용자는 `map_version:=94` 한 번으로 planning·신호등·Map-CV에
같은 지도를 전달한다. 이 변경이 controller 시작 기본값이나 `/Ego_topic` 사용 여부를 바꾸지는 않는다.
Validator는 기존 v88 경로 그대로이며 이 명령으로 과거 실행 snapshot이나 개인 YAML을 수정하지 않는다.

## 무엇을 막았나

- map 경로만 v94·해시 v88인 혼합: projector가 거부한다. 검사를 끄지 않았다.
- map94·Route A/B88 혼합: requester가 출발 요청을 거부한다. v94 profile은 두 값을 함께 선택한다.
- planning94·신호등/Map-CV88 혼합: 같은 version 인자/공유 map_file을 사용한다.
- `git pull`만 하고 설치본이 옛날인 상태: updater가 관련 C++·Python·launch/config를 설치한다.

## 신호등 정책과 검증 경계

같은 head의 짧은 지도 uncertainty 누락에는 **현재 YOLO crop을 CNN으로 다시 읽는**0.3초 보완을 적용한다.
오래된 초록불을 무한 유지하는 정책이 아니다. 실제 R/Y·화각 이탈·TF/미션 오류는 그 보호와 구별한다.
황색 통과 기본OFF·허가3개 확인·미수신/timeout1초 competition fail-open은 유지한다.
원래 팀장님 bag의 저속 재제동8개를 전부 해결했다는 보증은 아니다. 재생·실주행 근거 및 제한은
[움찔 보완 기록](../perception/traffic_light/261026_SIGNAL_JITTER_RUNTIME_REMEDIATION.md)을 본다.
팀 PC 오류 로그를 아직 받지 못했으므로, 이번 profile 정리는 그 오류의 단일 원인을 확정한 것이 아니다.

2026-10-27 점검: updater 실제 전체 실행·설치본 검사·Python 회귀532개·C++ 신호 판단107개·지도/source 문서 검사가 통과했다.
최신 planning 통합 뒤 로컬 A는 완주·최종 정지/P 및 자연 허가로 원본선 통과를 확인했고,
최종 GREEN 출발→원본선의 재제동은0회였다. **적색 대기 중 처리 지연·미수신에 따른 기존
1초 fail-open GO와 재제동은 별도로 관찰됐으므로 전체 무움찔 인증은 아니다.**
격리 합성 Route A→B 검사에서는 A 검증까지만 완료했고 B 전환은 미검증이다.
새 팀장님 v94 bag의 B STOP5회와 영상 회복 대조도 당시 실제 신호 topic이 없어 인과 확정과 구별한다.
이 업데이트로 timeout/fail-open·차선변경·Cruise 감속 정책을 새로 바꾸지는 않았다.
