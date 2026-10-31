# Base5 v39 — 안전 batch·quota 보완·중단/재개 검증

2026-10-06. EVIDENCE. 본10K 캡처 또는 학습 승인 문서가 아니다.

## 재연결과 실패 기록

사용자 재연결 후 Ego/Object/CollisionData publisher와 EventCmd 서비스,
정지 및 사전 Park 응답을 확인했다. v38의 network 재사용 실패 세션은 보존한다.
`shared_resume_v39` 첫 장면은 품질 검사를 통과했지만 장면 사이 Park가 두 번
timeout하여 다음 장면을 로드하지 않았다. 종료 finally Park와 별도 사후 probe는
정상 응답했다. 네트워크가 유지된 상태에서도 간헐 EventCmd 지연은 남아 있으며
원인은 확정하지 않았다. 종료 Park 성공으로 실패한 boundary를 승인하지 않는다.

새 `shared_resume_v39_repeat`는 동일6장으로 첫1장 pause → 나머지5장 resume →
완료 읽기전용 resume를 통과했다. 모든 boundary/final Park를 확인했다.
실패 세션의 ledger/계약/안전 파일은 수정하지 않았으며 그 세션은 resume 금지다.

## 구현

`scripts/run_base5_batch_campaign.py`는 전체 최대64셀, chunk 최대16셀의 **소량 pilot**이다.
기본 chunk4셀. ROS pub/sub만 공유하며 모든 ScenarioLoad에서 network를 다시 로드한다.
신선한 필수topic·post-load UID·정지·기존 품질 검사·장면 사이 및 최종 Park를 유지한다.

- accepted first pass는 원래 quota cell에 한 번만 선택한다.
- review가 나오면 다음 장면 전에 batch를 종료한다. 모든 boundary/final Park를 확인한
  안전 review만 기존 single-cell campaign으로 보완한다.
- 보완은 기존 지면 probe/Z 재촬영/동일 asset·패밀리·거리·split 재배치 루프를 사용한다.
  첫 review raw를 수동 이식하거나 계약을 위조하지 않고 새 initial을 촬영한다.
  따라서 이 pilot은 review마다 추가 initial 촬영 비용이 있으며 모든 시도를 보존한다.
- 보완한 capture만 quota를 채우고 calibration/검토 raw는 전달 ledger에 포함하지 않는다.
- 통신 오류·실제 물리 충돌·Park 미확인 시 자동 continuation 금지다.
- parent STOP은 batch 및 단일 셀 보완으로 전달한다. timeout/강제중단된 세션은
  상태 확인 없이 자동 resume하지 않는다. 새 캠페인 및 수동 안전 복구가 필요하다.
- 코드/manifest/선택 순서/chunk 크기 계약이 달라지면 resume 거부한다.
- 완료 resume는 기존 provenance·센서 파일·Park를 검사하고 재촬영하지 않는다.

첫12셀 `batch_quota_v39`는 chunk4 중단/재개 후12/12 통과했다.
first pass12회 + cell8의 단일 보완 initial/replacement2회 = 총14회 촬영이다.
cell8 첫 review와 보완 initial은 원본에 남고 최종 replacement만 선택한다.
완료 resume 전후 모든 captures 파일의 정렬 SHA-256 목록 aggregate가 정확히 같았다.
CPU 변환: train10/val2/test0, GT17개(vehicle9/pedestrian4/obstacle4), 제외 review/failed0.
성능 평가·학습 결과는 아니다.

## 남은20셀 실행 — 최종20/32 quota, 안전 중단

`batch_quota_remaining_v39`는20셀 중8셀의 quota를 채운 뒤 중단했다.
cell47은 ground plane 미확인으로 first pass review → 별도 initial review →
empty terrain probe → 다음 quota round initial 승인 경로를 통과했다.
review/terrain은 quota에 포함하지 않았다.

그다음 chunk의75/76/87/99는 각각 품질과 boundary Park를 통과했지만,
chunk 종료 finally의 Park 요청2회가 모두 timeout했다. 최종 안전 조건이 미확인이므로
해당4raw도 전달 quota에 넣지 않았고 남은 두 chunk는 로드하지 않았다.
종료 후 읽기전용 점검에서 Ego publisher/서비스는 남아 있었다. client 소실 사례와
다르며, 직전 boundary Park 성공 직후의 반복 요청 지연도 조사해야 한다.
지연 원인/안전 완전해결을 주장하지 않는다. 요청 횟수·timeout을 늘려 덮지 않았다.
수집 중단 뒤 별도 정지 Park probe는 약17.9ms로정상응답했다.
`park_after_final_failure_v39.json`에보존하며, 이사후성공으로실패수집chunk를소급승인하지않는다.

두 pilot 합계 **최종 선택20/32셀**이다. 24개 원래 셀의 first pass가 촬영됐고
보완5회(terrain1회 포함)로 총29회촬영, 전달 selection20장이다.
미시작8셀과 최종Park 미확인 chunk4셀은 pending12로 남는다.
실패 세션 resume 금지, 원본보존, 수동 안전 복구 후 새output에서 남은 셀만 검증한다.
따라서32/32 PASS 또는 장시간 무인 수집 준비완료로 해석하지 않는다.

live 당시 단위 검증은 Base5 tests109개 PASS다. 새driver6개 테스트는 안전중단,
review 보완/한quota, pause/resume/완료시 재촬영 없음, 계약 변경/대량 제한을 검사한다.

### 후속 중단 처리 보강

live 검증 구현은 commit `84ff5887`로 보존한다. 그 뒤 parent Ctrl-C/예외에도
child STOP→bounded 대기→필요시 독립 process group 종료를 보강했다.
단일 셀 보완도 감독 subprocess로 분리하여 parent 종료 시 capture가 남지 않게 했다.
이 후속은 별도 단위검증이며 MR live의 강제중단/보완 재검증은 아직 하지 않았다.
driver code hash가 바뀌었으므로 위v39 세션도 현driver로 resume하지 않는다.
원본읽기/CPU변환은 가능하며 이후검증은새output을사용한다. 기존contract를수정하지않는다.
선택20장 전체CPU변환은train17/val3/test0·GT28개(vehicle14/pedestrian6/obstacle8),
review/failed제외0으로통과했다. Animal 및남은Z보완셀을이번PASS에포함하지않는다.
후속단위검증112개PASS이며Parent STOP/Ctrl-C 및강제종료process group 테스트를추가했다.

## 경로

공통 root: `/home/luv/morai_cap/base5_v2_261006/`

- canonical10K: `scenarios_10k_v35/manifest.json` (본 캡처 미시작)
- 입력 대표32셀: `capacity_pilot_v36/manifest.json`
- 실패 보존: `shared_resume_v39/`
- 6장 중단/재개: `shared_resume_v39_repeat/`
- 첫12셀: `batch_quota_v39/selected_ledger.jsonl`
- 남은20셀: `batch_quota_remaining_v39/selected_ledger.jsonl`
- 첫12셀 CPU 변환: repo의 ignored `artifacts/datasets/base5_batch_quota_v39/`

## 재현과 모니터링

아래는 새 output의 소량 검증 명령이다. 기존 결과를 덮어쓰지 않는다.
현재 MR Capture Mode와7-interface network/CollisionData 연결을 유지한다.

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$ROOT/capacity_pilot_v36/manifest.json" \
  --output "$ROOT/my_batch_pilot" --limit 12 --chunk-size 4
```

동일 명령에 `--resume`을 붙여 동일 코드/계약의 안전 세션만 재개한다.
STOP 파일이 있다면 상태를 검토한 뒤 제거해야 하며 미확인 Park를 문서 편집으로
통과시키지 않는다. `--stop-after-chunks 1`은 완료 chunk 경계의 pause 검증용이다.

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/batch_quota_remaining_v39" \
  --port 8077
```

http://127.0.0.1:8077 에서 최종 quota 완료와 진행 중 batch 진행률을 따로 표시한다.
진행 중 batch를 이미 완료한 quota에 더하지 않는다. 잔여시간 미계측은0분 대신
계측 중으로 표시한다. 읽기전용 localhost 서비스이며 제어 기능은 없다.

## 제한과 다음

간헐 Park timeout 및 장시간 안정성은 미해결이다. v38의13.85% 개선은 당시 동일6셀
비교 결과이며, 새로운 per-scene Park·quality retry가 들어간 pilot에 그대로 적용할 수 없다.
본10K 비용은 작은 표본의 외삽으로만 다루며 완료시간을 보장하지 않는다.
Production E25/threshold/tracker/planning/localization 계약은 변경하지 않았다.
