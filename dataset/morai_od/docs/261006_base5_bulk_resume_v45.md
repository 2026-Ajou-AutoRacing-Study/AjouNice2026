# Base5 v45 — 6초 10K 준비 및 본수집 실행 경로

2026-10-06 / CURRENT PREPARED + EVIDENCE. **본10K 캡처·학습 미시작.**

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

### 본수집 명령 — 준비된 예시, 아직 실행하지 않음

기존 roscore/guarded bridge/7-interface/CaptureMode 연결을 유지한다. 중복 bridge를 켜지 않는다.

```bash
ajou
MANIFEST="$HOME/morai_cap/base5_v2_261006/scenarios_10k_budget6_v45/manifest.json"
OUTPUT="$HOME/morai_cap/base5_v2_261006/collection_10k_v45"
python3 dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$MANIFEST" --output "$OUTPUT" \
  --limit 10000 --chunk-size 4 --approve-bulk-capture
```

같은 계약으로 재개할 때 마지막 줄에 `--resume`을 추가한다.
새로 시작하는 명령은 기존 output을 거부한다. 원본/코드 변경 후 구 output resume도 거부한다.

### 안전 일시정지와 즉시 중단은 다름

```bash
# 현재4셀 및 필요한 품질보완·Park를 끝내고 다음chunk 전에 일시정지
touch "$HOME/morai_cap/base5_v2_261006/collection_10k_v45/PAUSE"
```

status.json에서 paused 확인 후 PAUSE 파일을 제거하고 같은 명령에 --resume을 붙인다.
PAUSE는 즉시 멈춤이 아니며, 느린 보완이 진행되면 기다릴 수 있다.
긴급 중단은 기존 STOP/Ctrl-C 경로다. child를 종료시키므로 halted가 될 수 있고,
정상 Park가 한 번 보였다고 unsafe 이전 경계를 자동 승인하지 않는다.

### 모니터링

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/collection_10k_v45" --port 8081
```

브라우저 http://127.0.0.1:8081 . 현재 소량 증거 화면8080은 bulk_path_smoke12_v45다.

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
