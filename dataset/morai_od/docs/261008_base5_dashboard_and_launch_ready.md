# Base5 Sunny13 5K 본수집 준비 및 대시보드 보강

2026-10-08. 사용자 확인: 이번 Sunny13 5K를 준비하며, 이후 기존 6환경 30K를 유지한다.
이 작업은 준비 및 읽기 전용 모니터링이다. 본수집·학습·새 모델 추론은 실행하지 않았다.
배치 v123 / 캡처 runtime v125 / 부족분 보충 v124 계약을 유지한다.
대시보드 revision을 캡처 runtime 버전으로 혼동하지 않는다.

## 본수집 입력·보존 확인

- Sunny13 manifest: `~/morai_cap/base5_v2_261008/regions_5k_v123_release/environment_suite/sunny13/manifest.json`.
  SHA `8eb9cd73e0f94e311dbe3cbecbefd5caea5d603782e5b15a81299dd619c23509` 불변이다.
- 현재 코드로 5,000개 장면과 고유 payload를 전수 검사해 `OFFLINE_PASS`, 오류 0을 확인했다. train 4,255 / val 745다.
  오프라인 검사이며, 5,000장의 실제 물리·가시성 인증은 아니다.
- sunny/foggy × 11/13/15의 6개 manifest hash, 5,000개 group/split/scenario hash 동일성과 서로 다른 output을 확인했다.
  여섯 본수집 output은 모두 미생성이다. 실제 날씨·시간·센서는 MR UI에서 확인해야 한다.
- v125의 실제 MR 정지 1 + 달리기 1, CPU 2/2 GT, 완료 resume 시 raw 68개 불변은
  [직전 근거](261008_base5_motion_metadata_v125.md)다. 이번에는 새 capture를 추가하지 않았다.
- 런북의 resume 및 완료 후 부족분 보충 명령은 그대로다. 구 runtime을 강제 resume하지 않는다.
- 읽기 전용 3초 통신 검사: Object 50.20Hz, Ego 49.78Hz, CollisionData 49.79Hz.
  Ego raw 속도 벡터 norm은 약 4.27e-6이며, 수신한 CollisionData의 상대 객체 ID는 없었다.
  이는 당시 연결·정지 상태 확인이지, 본수집 전체의 안전 보장은 아니다.

산출물 root: `/home/luv/morai_cap/base5_v2_261008/prebulk_dashboard_v126/`.
`offline_audit.json`, `input_backup_and_suite_check.json`, `live_readonly_check.json`,
`dashboard_not_started.png` 및 다음 백업을 보존한다.

| 백업 | 내용 | 압축크기 |
|---|---|---:|
| inputs_v123_snapshot.tar.zst | 전체release/6환경/scenario·metadata, MGeo, 원래3자산목록, optionList |166500576B |
| external_references_snapshot.tar.zst | manifest/config 및 5,000개 metadata에 명시된 외부 파일 51개 |22008B |
| runtime_256d3c68.tar.zst | commit256d3c68의dataset scripts/configs/tests 및sensor/network/scenario/global_path |1338329B |

모두 zstd integrity/list를 확인하고 SHA256을 JSON 보고서에 남겼다.
외부 절대 path를 재귀 검사한 결과, 존재하지 않는 reference는 0개다. 외부 height_lookup_live 및
로터리 참조 JSON도 포함한다. 문자열 추출 검사이므로 코드가 동적으로 계산하는 모든 의존성의 완전성 증명은 아니다.
백업은 전체 Git/submodule/Docker/런처를 대체하는 portable 실행본이 아니다. 원래 repository/revision과 함께 복구한다.
기존 원본·실패 파일을 삭제하지 않았고, 구글드라이브 업로드도 수행하지 않았다.

## 대시보드

현재 http://127.0.0.1:8075 에서 Sunny13 bulk root를 본다.
구 8075 프로세스만 교체했고 master/guarded bridge/구 8081은 그대로다. 8076 진단 서버는 회수했다.

- 승인누적및검사완료추이,최근10분승인rate. 실제관측없는구간에가짜선은그리지않는다.
- 후보승인/검토부족/미검사/활성누적시간/수집기ETA/디스크/현재child/recovery.
- manifest계획장수대비패밀리승인,거리bin/방향분포. scene bin이며실제GT객체수/OD성능이아니다.
- quota checkpoint별부족사유. 같은장면내같은사유중복은제거하지만사유가여러개면사유별집계한다.
- 전체actor기준정적4822/이동178의원래검사순서진행. scene의primary motion만세면4823/177로
  틀리므로collection_order계약을사용한다. 보충중에는이순서진행을계산하지않는다.
- 6환경별승인/부족/상태와전체30K승인. 환경파일을자동적용하거나동일group을독립데이터로주장하지않는다.
- 부모의확정승인에child진행중승인을더해중복계산하지않는다.
  부모quota/ledger checkpoint보다현재작은status가앞설수있음을표시한다.
- HTTP2초refresh/서버15초관측. 브라우저탭을닫아도서버가살아있으면관측한다.
  journal은 `~/.cache/ajounice/base5/dashboard_<root-hash>.jsonl`이며capture밖이다.
  최대20000차트점/16MiB이후compact,HTTP표시약600점이다. 새root는별도journal이며
  counter회귀/tornlastline/재시작을처리한다. 실행이전의정밀이력을합성복원하지않는다.
- 큰선택ledger/센서파일의주기적scan은없다. plan은시작시캐시,quota는mtime변경시에만읽는다.
  실패모음이큰경우해당checkpoint읽기비용은존재하지만collector에추가촬영/대기는없다.
- 상태JSON오류/HTTP실패/갱신지연을구분한다. 중단/부족종료를완료로표시하지않는다.
  화면은확인용이며ROS통신/충돌/Park안전을대신판정하거나제어하지않는다.

## 검증과한계

실제 Chrome에서 수집 전 5K / 정적 4,822·이동 178 / 6환경 30K를 확인하고 스크린샷을 남겼다.
격리된 UI fixture에서 두 SVG 그래프·세 polyline 및 진단 문자열 HTML escape를 headless browser로 검사했다.
UI fixture를 실제 수집 상태나 journal에 넣지 않았다.
대시보드 테스트 15개는 정상 진행, pause 중 rate, 재시작, 누적 counter 회귀, 원본 무변경,
사유 중복 제거, secondary mover, 보충, 계획 분모, suite 목표, 깨진 JSON, 브라우저를 확인한다.
전체 Base5 회귀는 **386개 / 77.399초 / PASS**다. py_compile 및 git diff 검사도 통과했다.

예산은 기존 **5K 10–13시간 / 6환경 60–78시간**을 유지한다. v125 기록 증가분은 두 단일 actor 표본에서
약 48/95KiB이며, 전체 5K 추가 storage의 확정치는 아니다. 현재 여유 약 247GiB, 이전 raw/QA 외삽은
5K 약 26.5GiB / 30K 약 159GiB다. metadata·캐시·압축 복제·재시도 여유가 필요하다.
입력 백업은 약 160MiB다. 본수집이 시작되면 첫 100–200셀 실측으로 시간·공간 예산을 갱신한다.
현재 런북 터미널 3으로 시작할 수 있다. 추가 개발 때문에 본수집 대기를 계속 늘리지 않는다.
