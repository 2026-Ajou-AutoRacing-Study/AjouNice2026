# Sibling planning_ws 통합 검증 기록

아래는 통합 구현 시점의 로컬 빌드/기동 기록이다. 이후 통합 변경을 `3616bfd`로 푸시했다. 후속 Git blob 검사에서 provider BAT 2개가 manifest와 다른 바이트로 저장된 것이 발견됐으므로, 새 clone의 재현성에는 [상세 인계서 B0](BASE-WS-INTEGRATION-GUIDE.md)의 수정이 필요하다. 로컬 소스 해시 PASS를 Git checkout 해시 PASS로 해석하지 않는다.

## 판정

**물리적 소스 통합 및 새 경로 빌드 PASS. 실제 base 입력과 폐루프 주행은 미승인.**

| 검사 | 결과 | 증거 |
|---|---|---|
| 지정 branch | `planning/autoware-universe-to-ros1`, 시작 HEAD `db16770` 유지 | Git 작업 트리, 커밋/푸시 없음 |
| 전체 planning 재빌드·설치 | 154/154 = 경계 149 + 지원 5 | `planning_ws/logs/full-build.log` 마지막 `[154 of 154]` |
| addon 빌드·설치 | 2/2 | `planning_ws/logs/addon-build.log` |
| 원본 소스 해시 | 3,997/3,997 동일. 원본 checkpoint의 나머지 4개 pycache 제외 | `planning_ws/logs/source-audit.json` |
| provider 소스 해시 | 6,589/6,589 동일. metadata/cache 143개 제외 | 같은 source audit |
| 중복 package 이름 | 0, planning 154 + 남은 base 23 = 총 177 unique | 같은 source audit. 새 integration addon 1개 추가 반영 |
| 설치 ELF 의존성 | 230개 검사, missing library 0, 이전 root ELF/CMake/environment 참조 0 | `planning_ws/logs/install-audit.json` |
| requester 정책 검사 | 8개 PASS | `test/test_policy.py`: 5 cases, invalid config, age, deadline, hash/geometry, UUID/goal/sequence, moving retry suffix |
| requester ROS 통신 검사 | 10개 PASS, private master | `planning_ws/logs/requester-transport-test.log` |
| 새 설치 full graph + 실제 v61 map | startup PASS, binary map 21,207,824 bytes, publisher 1개 | `planning_ws/logs/install-runtime/result.json`, `planning_ws/logs/install-runtime/planning.log` |

ROS 통신 검사에는 A 요청·새 출력, B trigger pose 요청, fallback lanelet 요청, fallback serialization, 잘못된 case 거부, 서비스 거부, timeout/동시 요청 거부, 잘못된 UUID, stale trajectory, validator invalid가 포함된다. RouteSelector는 mock이며 실제 이동 중 Mission 재경로/안전 판단을 검사한 것이 아니다.

## 발견·수정한 통합 문제

1. `catkin_python_setup()`는 `generate_messages()`보다 먼저 호출해야 하므로 새 requester CMake 순서를 수정했다.
2. 새 map projector publisher가 YAML `LocalCartesianUTM` 문자열을 그대로 발행하면 동결 메시지의 생성 상수와 다르게 전달되어 full map loader가 종료됐다. GDB throw stack은 `autoware::geography_utils::get_lanelet2_projector`를 가리켰다. 생성 상수 `MapProjectorInfo.LOCAL_CARTESIAN_UTM`과 `WGS84`를 사용하도록 addon을 고친 후 재빌드·재기동하여 실제 binary map 발행과 graph 생존을 확인했다. 원본 map/message/algorithm은 변경하지 않았다.
3. 전역 `lib/` ignore 때문에 Lanelet2 C++ 원본 22개가 publication에서 누락될 수 있어 경로를 명시적으로 포함시켰다. vendor 자체 ignore가 숨기는 998개 원본은 exact force-add 목록을 남겼다. 실제 Git stage/push는 하지 않았다.

## runtime PASS의 한계

full graph startup 검사에는 route, ego, TF, 객체, acceleration, occupancy, mode 등 주행 입력을 만들어 넣지 않았다. 따라서 TF 부재 오류 및 입력 대기 메시지는 존재한다. 이 검사는 로딩/링크/지도 transport/서비스 생성 검사이며 **runtime error 전무 또는 trajectory 생성 성공이라는 판정이 아니다**.

사용 vehicle YAML은 설치본의 startup test fixture이며 AjouNice 실차 치수 승인이 아니다. requester와 follower도 이 graph 검사에서는 비활성이다. 종료 후 해당 private ROS master와 launch는 정리했다.

다음 단계와 개발자별 변경 요청은 [INTEGRATION-HANDOFF.md](INTEGRATION-HANDOFF.md)를 따른다. 특히 base serialization, 실제 입력 공급, 이동 중 A→B, 전체 시간대 trajectory 품질/latency 및 controller 폐루프가 남아 있다.
