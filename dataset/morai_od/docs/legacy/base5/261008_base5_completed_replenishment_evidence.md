# Base5 완료 후 부족분 통합 보충 — v124 EVIDENCE

2026-10-08. v123 배치 release 위의 운영 기능이다. 본5K/30K 수집·학습·모델 성능 실험은 아니다.
Production detector/threshold/tracker/planning/control/localization은 변경하지 않았다.

## 목적과 구현

기존 batch의 completed resume은 검증만 하므로 부족분을 채우지 못했다. 소량 visibility-only 도구는
있었지만 5K 완료 결과의 여러 실패 종류/통합 ledger/지속적인 보충 이력과 연결되지 않았다.
기존 도구/API/테스트는 그대로 보존하고 `base5_campaign_replenishment.py`를 새로 추가했다.

- `run_base5_batch_campaign.py --resume --replenish`: 최초 완료 근거를 읽기 전용으로 검증한다.
  검증되지 않은 성공 표시·calibration·안전 미확인·변경된 manifest/runtime은 보충 근거로 쓰지 않는다.
- 부족 원래 셀만 새 geometry로 준비한다. metadata spec 전체, primary data_id, split,
  scene_group_id 및 환경 config를 일치 검사한다. 거리/방향/미션 단계/걷기·달리기 요청을 완화하지 않는다.
- 기본 셀당 누적2회차, 명시적으로1–3회차를 선택한다. 명령 재실행이 상한을 초기화하지 않는다.
  이전 실패 candidate의 다음 retry부터 생성한다. child의 최대2 quota round/기존 bounded QA도 유지한다.
  2회차는 촬영2장 상한이 아니다. 후보 생성 상한512도 무제한으로 늘리지 않는다.
- 생성 후보가 없는 경우는 `same_cell_placement_unavailable` 부족분이다. 센서·GT를 합성하지 않는다.
  검증된 접촉 실패는 새 배치로 시도하지만 통신/Park/정지 불확실은 전체 halt다.
- `replenishment/state.json`에 최초 승인/부족분과 회차별 manifest/결과를 commit한다.
  기존 성공 row+추가 승인 row를 원래 source_cell당1개로 canonical ledger에 합친다.
  quota/deficit manifest/status도 갱신한다. 원래 raw·실패·source manifest는 삭제하거나 덮어쓰지 않는다.
- cell commit 후 ledger publication 전 끊겼으면 journal로 복원한다. 진행 중 child의 완료 근거가 있으면
  다시 촬영하지 않고 검증한다. 없는 안전 근거를 임의로 채우는 crash 복구는 하지 않는다.
- 안전한 PAUSE→동일 보충 명령 재개를 지원한다. 보충 뒤 ordinary resume은 비촬영 검증이며,
  이전 base만 재구성해 추가 성공을 잃는 문제가 없도록 분기했다.
- nested quality replacement에서도 원래 scene_group_id/환경 split 계약을 유지한다.
  empty background의 재배치도 primary 없는 경우를 올바르게 처리한다.
- 대시보드는 최초 승인/추가 보충/남은 부족/회차/시도를 보여준다. 보충 중 bar는 승인 수이며
  finished=target이어도 부족분이 있으면 완료 승인이 아닌 것으로 표시한다.
- heartbeat는 작은 status만 갱신한다. 수십MiB가 될 수 있는5K ledger/baseline을0.5초마다
  다시 serialize/fsync하지 않고 셀 경계에서만 commit한다. 기본 수집에 새 sensor 촬영은 추가하지 않는다.

## 실제 MR 통합 확인

최종 runtime의 진단 폴더:
`/home/luv/morai_cap/base5_v2_261008/replenishment_v124_verified/`.
`manifest.json`, `synthetic_contact_input.json`, `initial_snapshot.json`, `report.json`, `collection/`, `infos/`를 보존한다.

**접촉 입력은 명시적인 테스트용 ROS fault injection이다. 실제 물리 충돌 재현이라고 해석하지 않는다.**
두 정지 로터리 scene 중 첫 scene에만 `BASE5_SYNTHETIC_CONTACT`/UID987654 메시지를 넣고
첫 실패 기록 이후 publisher를 해제했다. 실제 MR은 정상 시나리오 로드/Park/정지/센서 캡처를 수행했다.
진단 raw/infos는 본 학습에 편입하지 않는다. 본수집 입력/기존 pilot/raw는 그대로다.

| 항목 | 최종 결과 |
|---|---|
| 최초 완료 | 승인1/2, 테스트 접촉 부족분1 |
| 보충 후 | 승인2/2, 추가 보충1, 남은 부족0 |
| 기존 성공분 | row 내용 및 sensor6파일 SHA/inode/mtime/size 불변 |
| 보충+두 검증 wall |11.6479초 |
| ordinary resume | 추가 캡처0, 승인2 유지 |
| 같은 보충 명령 재실행 | 추가 캡처0, ledger/누적elapsed 불변 |
| 제어 event 로그 | 완료 뒤 두 검증에서 내용 불변 |
| CPU 변환 | train2/Vehicle2GT, review/failed 편입0 |
| 환경 token/group | sunny13:b5v2_00163, sunny13:b5v2_00264 / 원래group 유지 |
| localhost dashboard HTTP | status 최초1/보충1/부족0 및 HTML 보충 표시 확인 |

기존6파일은4camera Intensity PNG+LiDAR BIN+LiDAR Instance TXT다. 실패 scene은 pre-capture
접촉으로 SaveSensorData 요청 전에 실패했고, 정상 scene과 새 replacement만 실제 캡처됐다.
첫 실패 시나리오 SHA `36455afbc47f2bcc6e4beda7988f087529a1af4c1db5962f630ca6c567036262`와
실제 새 replacement SHA는 각 manifest/report에 따로 남는다. GT 사후 수정으로 성공시킨 결과가 아니다.

진단 재현 명령은 fresh output에만 사용한다. 본5K 명령이 아니다.

```bash
ajou
python3 -u dataset/morai_od/scripts/probe_base5_replenishment.py \
  --manifest "$HOME/morai_cap/base5_v2_261008/regions_5k_v123_release/pilot_manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261008/replenishment_v124_new_probe" \
  --allow-test-contact-injection
```

MR capture/guarded bridge/ROS 환경을 먼저 준비한다. 다른 full stack/collector와 동시에 실행하지 않는다.
시행 중 네트워크 코드/manifest/runtime 파일을 수정하지 않는다.
이전 `replenishment_v124_probe`, `replenishment_v124_final`, `replenishment_v124_release`는 개발 중 검증 이력으로 보존한다.
최종 이름 분리/empty 처리/누락 manifest 읽기 전용 거부 이후의 근거는 위 `replenishment_v124_verified`다.
실제 batch CLI로도 ordinary resume/보충 재실행을 확인했으며 exit0/추가촬영0이다.

## 자동 검증

`python3 -m unittest discover -s dataset/morai_od/tests -p 'test_base5*.py'`
**369개 PASS / 75.521초**. 기존355개와 신규14개를 함께 검증했다.

신규 보충12개는 부족분-only 통합, PAUSE/resume, pending 완료 child 재사용,
publication 중단 후 복원, 누적 상한·명시적 확장·다른 retry geometry,
STOP/PAUSE 사전 중단, child 안전 실패 후 다음 로드 금지/강제resume 거부,
생성 불가능 부족분, 자산/spec/split/group/환경/hash 불일치, 결과 변조/원본 성공 보호,
최초 미완료·구runtime 계약 거부, 완료 manifest 누락을 임의 재생성하지 않는 검증을 확인한다.
나머지2개는 nested empty/group 보존과 dashboard 승인·finished 구분이다.
기존 소량 visibility-only 보충4개 테스트도 제거하지 않고 유지했다.

## 변경하지 않은 것과 한계

- 배치/계획 manifest/5K·30K 자산 노출 표는 v123 그대로다. 2026-10-08 11:05:24 KST 동결 표를
  실제 가시GT/보충 후 총 노출로 바꿔 읽지 않는다. 실제 노출은 통합 raw/ledger로 따로 집계한다.
- 2-cell MR 확인은 운영 wiring 검증이다. 전패밀리·178 moving·122종·5K의 성공률/시간 보장이 아니다.
  moving 보충에는 기존 독립 지면 준비/두점 경로/실제 이동·Z gate가 계속 적용된다.
- 초반5K ETA10–13h/6환경60–78h는 v123 표본 기반 운영 추정이다. 이번 보충 비용은
  발생한 부족 수/이동 준비에 따라 추가된다. 11.65초를 모든 부족분 비용으로 외삽하지 않는다.
- 원래 spec/group/split은 유지하되 실패 geometry는 바뀐다. 환경별 보충이 다른 배치를 선택하면
  해당 group을 완전히 동일 geometry의 weather A/B라고 주장하지 않는다. leakage 방지는 유지한다.
- 2회차 후 부족분이 남을 수 있다. 기존 기준을 낮추지 않고3회차 명시적 확장 또는 별도 원인 검토를 한다.
  halted의 안전 복구는 자동화하지 않는다.
- runtime v124의 contract에는 새 batch/helper SHA가 포함된다. 구 v123 pilot output은 원래 revision에서
  재현한다. 진행 중 코드를 바꿔 강제resume하지 않는다. 본5K는 현재 코드/fresh output으로 시작한다.

다음은 사용자 승인 후 [런북](261007_base5_5k_capture_runbook.md)의 Sunny13 본수집이다.
완료 뒤 같은 명령에 `--replenish`를 추가하고 통합 ledger의 실제 coverage/CPU 변환을 확인한다.
