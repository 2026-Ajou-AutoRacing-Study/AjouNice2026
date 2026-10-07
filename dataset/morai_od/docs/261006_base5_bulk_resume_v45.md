# Base5 v45 — 6초 10K 준비 및 본수집 실행 경로

2026-10-07: **HISTORICAL RUNBOOK.** 현재는 Sunny11시 직접5K이며
[새 단일 운영 런북](261007_base5_5k_capture_runbook.md)을 사용한다.
아래 모든 v45/v86·10K 명령은 현재 본수집 명령이 아니다.

**v45 운영은 중단됐다.** 실제 본수집은 정적 자전거 충돌로 halted이며, 아래 명령을
그대로 재실행하지 않는다. 최신 정책은 [BASE5_V2_CURRENT.md](BASE5_V2_CURRENT.md)와
이 문서 마지막의 2026년10월7일 운영 변경 절을 따른다. v45 소량검증 결과와 명령은 이력이다.

## 6초 수집 묶음

원본 v35 9초 deck은 보존한다. 새 수집 후보는 다음 경로다.

```text
/home/luv/morai_cap/base5_v2_261006/scenarios_10k_budget6_v45/manifest.json
```

10,000 unique payload / 전수 OFFLINE_PASS / train8607·val1393·test0.
패밀리 highway3000/roundabout700/other_vehicle300/obstacle3000/pedestrian_related2500/
background500 유지. source spec/retry·Ego·목표geometry·appearance speed·split을 비교해
보존했고 자연 이동 출발 여유만6초로 바꿨다. 동작/지면/Z/충돌/가시성의 전수 live 인증은 아니다.
GT는 원본 Instance annotation, lidar1_camera0, ROI[-100,-30,-3,100,30,2]를 유지한다.
Production E25/threshold/tracker/planning/localization 변경 없음.

재생성은 **오프라인 준비 승인과 실촬영 승인이 분리**된다.

```bash
ajou
python3 dataset/morai_od/scripts/prepare_base5_setup_budget_pilot.py \
  --manifest "$HOME/morai_cap/base5_v2_261006/scenarios_10k_v35/manifest.json" \
  --all-scenes --include-static --approve-bulk-prepare --budget-sec 6 \
  --output "$HOME/morai_cap/base5_v2_261006/scenarios_10k_budget6_v45"
```

기존 output은 덮어쓰지 않는다. 새로 생성할 때는 새 output을 지정한다.
10K 파일은 MR GUI 폴더에 넣지 않는다. runner가 소량 임시 파일만 설치/회수한다.

## 실행·정지·재개 계약

- 기본은 최대64셀 pilot. `--approve-bulk-capture`를 명시할 때만 최대10K 실행을 허용한다.
  승인 옵션을 붙여도 기본 limit32는 그대로다. 본수집은 사용자 별도 지시 후에만 실행한다.
- 공유 ROS child는 기본4셀, 최대16셀. 모든 scene network reload 및 boundary/final Park 유지.
- 초기6초 목표 도착 누락만 안전 증거와 hash/Park를 검증한 뒤9초를 한 번 재시도한다.
  통신·충돌·Park·보완 후 목표 실패를 정상 quality review로 덮지 않는다.
- review는 기존 지면/Z/empty-ground/동일quota replacement 경로를 사용한다.
  소진하면 quota/deficit manifest에 남긴다. review/failed/calibration으로 quota를 채우지 않는다.
- 완료/complete_with_deficit resume은 증거 검증 전용이다. 손실된 증거를 새 촬영으로 메우지 않는다.
- paused/stopped의 안전한 child는 검사 후 재개한다. `halted`는 자동 resume 거부:
  수동 안전 복구·원인 확인과 새 output이 필요하다. 코드/원본 hash 계약을 강제로 바꾸지 않는다.
- resume elapsed는 누적 active wall을 보존한다. 사용자가 쉬는 pause 시간은 포함하지 않는다.
- bulk 전체 선택 목록/deficit는64장 추가 진행 단위와 pause/stop/halt/final에 atomic 저장한다.
  raw session ledger는 각 child에 보존되므로 resume은 이를 검증해 재구성한다.
  상태 화면의 finished와 checkpoint_finished가 잠시 다른 것은 손실이 아니다.
- heartbeat는 전체 GT 목록을 재작성하지 않는다. dashboard는 accepted_bins 집계를 읽는다.
  본수집 변환은 complete 상태의 최종 selected_ledger만 사용한다. 진행중 checkpoint를
  최종10K 전달본으로 간주하지 않는다.

## 터미널별 운영 순서 — 처음 실행부터 재개까지

각 터미널에서 먼저 `ajou`를 실행한다. 기존 master/bridge가 실행 중이면 아래1·2를
중복 실행하지 않고 현재 연결을 유지한다. `ajou`는 저장소 이동과 팀 공통 ROS 환경 설정이다.

### 터미널1 — ROS master

```bash
ajou
rc
```

`rc`는 roscore alias다.

### 터미널2 — guarded ROS bridge

```bash
ajou
python3 dataset/morai_od/scripts/run_base5_rosbridge.py
```

일반 `roslaunch rosbridge_server rosbridge_websocket.launch`가 아니라 위 guarded bridge를
사용한다.9090에 기존 bridge가 있으면 중복으로 띄우지 않는다.

MR 설정:

- Capture Mode 사용, 기존 검증된 센서 배치/설정 유지.
- Ego Network에서 아래7-interface profile을 Load → Connect.
- profile 경로:`$HOME/AjouNice2026/configs/network/ego_network_mr_base5_savesensordata_collision_local.json`.
- /Ego_topic, /Object_topic, /CollisionData와 ScenarioLoad/SaveSensorData/제어 서비스 연결 유지.
- 이번 동기 캡처에는 UDP receiver·sensing·localization·BEVFusion·planning·control을
  별도로 실행하지 않는다. rosbag 실주행 녹화용 명령과 혼동하지 않는다.
- 10K scenario 파일을 MR GUI 폴더에 복사하지 않는다. runner가 소량만 임시 설치한다.

### 터미널3 — 본수집 시작 / 재개 공통 명령

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$HOME/morai_cap/base5_v2_261006/scenarios_10k_budget6_v45/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261006/collection_10k_v45" \
  --limit 10000 \
  --chunk-size 4 \
  --approve-bulk-capture \
  --resume
```

`--resume`은 첫 실행에도 사용 가능하다. output이 없으면 처음 시작하고,
기존 안전 기록이 있으면 검증 후 이어서 진행한다. 동일 output에 수집기를 둘 켜지 않는다.
수집 중에는 collector 코드·원본 scenario·network/sensor config를 수정하지 않는다.
코드/원본 hash 변경 후 구 output resume은 거부한다. 계약파일을 강제로 고치지 않는다.
PC 절전/자동 종료를 꺼 둔다. pause 뒤 재개는 아래PAUSE 파일을 먼저 제거한 후 같은 명령이다.

### 인자 뜻

| 인자 | 의미 |
| --- | --- |
| python3 -u | 로그를 버퍼에 쌓지 않고 즉시 출력 |
| --manifest | 6초 준비 설정 Base5 10K의 scenario/metadata 목록 |
| --output | 캡처 raw·감사·진행·재개 기록을 저장하는 campaign 폴더 |
| --limit 10000 | 목록에서 최대10,000 원래 quota 장면을 처리 |
| --chunk-size 4 | 한 공유ROS child에서4장면 처리. 안전PAUSE는 묶음 및 보완 완료 후 적용 |
| --approve-bulk-capture | 기본64장 제한을 넘는 대량 수집의 명시적 실행 허용 |
| --resume | 기존 안전 기록 검증 후 이어서 진행. 첫 실행도 가능 |

**4는 장면당 촬영 수가 아니다.** 기본은 독립 장면당1프레임이며4장면을 한 묶음으로 처리한다.
지면/Z/가시성 보완 등이 필요하면 추가 촬영할 수 있다.
**10,000 quota 처리와10,000 accepted 학습 장면 보장은 다르다.** 보완 소진은 deficit로 남고,
실패/review raw를 성공 장면으로 섞지 않는다. 부족분은 quota.json/deficit_manifest.json으로 확인한다.

### 터미널4 — 대시보드

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/collection_10k_v45" \
  --port 8081
```

브라우저 http://127.0.0.1:8081 . 수집 전에는 status.json 대기 화면일 수 있다.
이 대시보드는 관측 전용이며 닫아도 수집기는 멈추지 않는다.8080은 과거 소량검증 화면이다.
finished는 실제 완료 quota 수, checkpoint_finished는 전체목록 저장 지점이다.
bulk에서는64장 단위로 전체목록을 저장하므로 두 수가 잠시 다른 것이 정상이다.

### 별도 터미널 — 안전 일시정지 / 재개

```bash
ajou
# 현재4셀 및 필요한 품질보완·Park를 끝내고 다음chunk 전에 일시정지
touch "$HOME/morai_cap/base5_v2_261006/collection_10k_v45/PAUSE"
```

대시보드/status.json에서 **paused를 확인할 때까지 기다린다.** 즉시 master/bridge/MR을 끄지 않는다.
재개 전 아래를 실행하고 터미널3의 공통 명령을 그대로 다시 실행한다.

```bash
ajou
rm -f "$HOME/morai_cap/base5_v2_261006/collection_10k_v45/PAUSE"
```

PAUSE는 즉시 멈춤이 아니며, 느린 보완이 진행되면 기다릴 수 있다.
긴급 중단은 기존 STOP/Ctrl-C 경로다. child를 종료시키므로 halted가 될 수 있고,
정상 Park가 한 번 보였다고 unsafe 이전 경계를 자동 승인하지 않는다.
`halted`라면 PAUSE만 지워서 강제 재개하지 말고 실패 로그/안전 증거를 확인한다.
완료를 다시 resume하면 정상적으로 증거 검증만 하고 종료하며 처음부터 다시10K를 찍지 않는다.

## 실제 소량 검증과 비용 제한

같은10K 후보에서12셀만 선택, bulk 실행 경로로4셀 안전pause→남은8셀 resume을 했다.
12/12 최종통과, 실제14촬영, 누적108.464초, CPU train11/val1/test0·GT18.
Dog2 가시성 review1셀은 동일quota replacement2번 후 통과했다.
Event34회, 짧은 handler 응답 오류3회 retry회복, timeout/미종료worker0, 모든 Park 확인.
완료 resume는 추가 촬영 없이 exit0. 후속 PAUSE marker 보완으로 driver hash가 달라지므로
이12셀 output은 현재 코드로 강제 resume하지 않고 읽기전용 근거로 보존한다.

143tests PASS:68셀 mock bulk/heartbeat 목록불변/누적시간/완료무중복/중단resume거부/
PAUSE 경계/deficit 완료증거손실시재촬영금지 포함. Mock는 장시간 MR 인증이 아니다.
디스크 점검 시 가용264GiB였으며 이후 저장량에 따라 달라진다.
현재10K 시간 추정은 기존 대표표본18.71시간과 재시도가정19–20시간대를 유지한다.
이번 I/O 보완의 추가 시간 절감은10K 실제 측정하지 않았으므로 수치로 약속하지 않는다.

공통 root /home/luv/morai_cap/base5_v2_261006/ 아래:

- scenarios_10k_budget6_v45: 새10K OFFLINE_PASS. 본촬영 없음.
- bulk_path_smoke12_v45 / bulk_path_smoke12_cost_v45.json / bulk_path_smoke12_infos_v45.
- bulk_pause8_v45: 최종 PAUSE marker 코드의 별도8셀 live 근거.

최종 marker live 결과:4/8/20.932초에서 PAUSE 경계정지, marker 제거 후8/8완료,
누적51.624초/실제8촬영/Event18회/오류0/timeout0/미종료worker0/모든 Park 확인.
CPU train8/val0/test0·GT10(obstacle2/vehicle5/pedestrian3), review/failed 편입0.
완료 resume exit0, ledger SHA/elapsed/Event18회 불변, 추가 서비스요청·촬영0.
소량28GT 변환(12셀GT18+8셀GT10)은 모델 학습/성능평가를 의미하지 않는다.
# 2026년 10월 7일 충돌 중단 후 운영 변경

기존 `collection_10k_v45`는 `b5v2_00033`의 정적 Bicycle_city 충돌로 halted이다.
완료 체크포인트32장, 원본 약184MiB를 실패 근거로 보존한다. 위 출력으로 resume하거나
과거10K 명령을 그대로 재실행하지 않는다. 아래 본문은 v45 당시 명령의 이력이다.

v46은 unknown geometry의1x1m 임시 박스를3x3m 보수적 배치 proxy로 바꾸고,
측방 회피 gap은 measured geometry 최소30cm/unknown 최소1m로 제한한다.
이 값은 학습 GT 또는 collider 크기가 아니며 actual annotation·충돌 검사가 최종 판단이다.
동적 장면은 빈 지면 및 정적 자산 reference를 먼저 캡처하고0.5m 간격으로 waypoint
높이를 산정한다. 지원되지 않는 지면은 quota 부족으로 남긴다. 기존18.71시간 외삽은
새 calibration-first 비용을 포함하지 않으므로 새 전체10K 시간으로 사용하지 않는다.

정적 우선 deck은 움직이는 장면을 standing으로 바꾸지 않고 별도 보류한다.
10K 전체 class/동작 quota가 채워졌다고 해석하지 않는다. 실행 승인과 최신 검증 결과는
`BASE5_V2_CURRENT.md`를 따른다. 런타임 hash 변경으로 기존 캡처 세션은 재개하지 않는다.

## 새 정적 우선 실행 후보 (2026-10-07)

아래는 새 코드 검증 후 사용자가 본수집을 재개하기 위한 명령이다. 에이전트는 이 본수집을
자동 시작하지 않았다. 기존 raw는 삭제하지 않았으며 새 output을 사용한다.
통신은 앞의 rc → guarded bridge → MR 7-interface/Capture Mode 순서를 유지한다.
별도 UDP/센서 ROS 수신기를 추가할 필요가 없다.

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$HOME/morai_cap/base5_v2_261007/static_first_all_v46/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261007/collection_static_first_v59" \
  --limit 8354 \
  --chunk-size 4 \
  --approve-bulk-capture \
  --resume
```

동적 1,646셀은 이 목록에 들어 있지 않다. 학습 전 원래 quota 보충 여부를 별도 확인한다.
`--resume`는 **동일 manifest·동일 runtime hash의 안전한 중단**에만 사용한다.
구 halted output을 새 코드로 강제 승인하지 않는다.
PAUSE marker는 새 output 아래 만들고, 해제 후 위 명령을 다시 실행한다.

대시보드는 별도 터미널에서 같은 새 output을 지정한다. 현재 검증 화면 8082는 정적32셀의
완료 화면이므로 본수집 화면이라고 혼동하지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261007/collection_static_first_v59" \
  --port 8083
```

근거·한계는 `BASE5_V2_CURRENT.md`의 최신 v59 및 v48 절에 기록했다. 정적32/32와 실제 이동 혼합3/3을
확인했지만 모든 자산·장시간의 live 검증을 대신하지 않는다.

### 10월 7일 후속: Park 유지 및 충돌 처리

정적/동적 모두 Ego는 Park 유지다. 동적 진행을 위해 Drive로 전환하지 않는다.
확인된 CollisionData peer 접촉은 실패 증거를 보존하고 부족분으로 분리한다. 종료·경계 Park가
확인되면 다음 수집 장면을 직접 로드하고 새 상태 검사를 수행한다. 전체 운용의 확인된 충돌
때문에 무조건 halted로 종료하던 기존 설명은 최신 코드에서 대체됐다.
통신/로드/정지/Park 확인 실패는 여전히 halt이며 단순 실패 무시 옵션은 없다.
`deficit_manifest.json` 부족분은 완료 resume 시 재촬영되지 않는다. 별도 보충 캠페인 대상으로 사용한다.
`inject_base5_collision_test.py`는 승인된 진단용 입력 시험에만 사용하는 도구이며 본수집에서는 실행 금지다.
# 2026-10-07 v66 후속 주의

아래 v45/v59 수집 명령은 실행 이력/운영 방법이다. 정적 전체 ground gate와 초기 높이
후보를 보강한 뒤, 원거리 cross-view 지면 검사에서 오판 가능성을 발견했다. 현재 본10K
readiness가 아니므로 구8354셀 deck 또는 중단 output을 그대로 본수집/resume하지 않는다.
`BASE5_V2_CURRENT.md`의 v66과 `261007_base5_static_height_evidence.md`를 먼저 읽는다.

## 2026-10-07 최신 v86 — 새 manifest / 본수집 승인 대기

이 절이 위 v45/v59 실행 경로를 대체한다. **10K 실캡처는 아직 시작하지 않았다.**
최신 근거는 [CURRENT](BASE5_V2_CURRENT.md)와
[본수집 전 통합 검증](261007_base5_prebulk_recovery_evidence.md)이다.
network/master/guarded bridge/Capture Mode 설정은 위 터미널1·2와 동일하다.
수집 중 코드·config·manifest를 변경하지 않는다.

새10K는 모든 quota를 유지하고 정적/정지8,354셀→동적1,646셀 순서로 묶었다.
동적 객체를 정지 객체로 바꾸지 않았다. 각 payload/spec/primary/split/hash는 순서 변경 전과 동일하다.
계속 raw Instance GT/lidar1_camera0/4클래스/ROI[-100,-30,-3,100,30,2]/Ego Park다.
ground 부족은 실측 near witness로 보완하며 가시성/충돌/지형을 면제하지 않는다.
동적 review의 보충 장면은 반드시 새 motion terrain/static 준비를 거친다.

아래는 **사용자가 본수집을 별도로 지시한 뒤 실행할 명령**이다. 준비 검증 요청만으로
이 명령을 실행하지 않는다. 구 collection_10k_v45 또는 halted output에는 resume하지 않는다.

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$HOME/morai_cap/base5_v2_261007/scenarios_10k_ordered_v86/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261007/collection_10k_v86" \
  --limit 10000 --chunk-size 4 --approve-bulk-capture --resume
```

대시보드는 같은 새 collection output을 지정한다. 이전 pilot의 완료 화면을 본수집 화면으로
혼동하지 않는다. 아래 포트가 사용 중이면 기존 화면의 output을 확인하거나 빈 포트를 선택한다.

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261007/collection_10k_v86" --port 8086
```

PAUSE/STOP/안전한 resume/완료 검증 전용 resume/별도 deficit 보충 계약은 위 설명을 따른다.
새 manifest의10K OFFLINE_PASS는 전수 live 인증이 아니다.
지금은 구18.7h 예상 시간을 사용하지 않는다. 강화된 QA를 포함한32셀 표본의
parent wall/승인 수 단순외삽은 약43.6h이며, 정적 우선 순서의 실제10K 시간을 측정한 값도
완료 보장값도 아니다. 시작 후 active wall/승인 수/부족분/저장공간을 다시 계측해야 한다.
# 2026-10-07 최신 목표 변경: 직접5K / 아직 본수집 전

[직접5K최신계획](261007_base5_direct_5k_current_plan.md)을먼저읽는다.
구v86/v45의10Kmanifest·limit10000명령을이번수집에사용하지않는다.
새5K최종manifest·초근접보완은아직준비중이며기존output에새config를섞어resume하지않는다.
