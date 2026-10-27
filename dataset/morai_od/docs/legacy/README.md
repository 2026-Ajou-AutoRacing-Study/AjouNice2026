# MORAI 데이터셋 legacy 계보

문서 상태: ARCHIVE. 정리: 2026-10-24.
이곳의 CURRENT·PREPARED·당시 기본값은 역사적 제목이며 현재 승인이 아니다.
현재 데이터와 실행은 [Base5 CURRENT](../BASE5_V2_CURRENT.md), [문서 라우터](../README.md)를 따른다.
요청한 세대의 재현에만 열고 기본 검색·agent context에서는 `legacy/`를 제외한다.

| 계보 | 보존 범위·선택할 진입점 |
|---|---|
| 초기 Base1/2·OD180·Sunny13 | 기존 이 디렉터리의 temporal/dynamic/Scenario Runner·geometry·정적 연구 원문 |
| Base3 Static | [10K 감사](base3_static_10k_final_capture_audit.md), [class-centric 설계](base3_static_classcentric_10k_design.md) |
| Base3 Online | [당시 CURRENT](base3/BASE3_ONLINE_CURRENT_STATE.md), [rosbag 런북](base3/base3_online_rosbag_collection_runbook.md) |
| Base4 temporal/velocity | [당시 CURRENT](base4/BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md), [MR/MRP hybrid 런북](base4/morai_mr_mrp_hybrid_scenario_capture_runbook.md) |
| 초기 Base5·구 E24 | [독립 10K 설계](base5/BASE5_INDEPENDENT_10K_DESIGN.md), [v100 이전 이력](base5/BASE5_V2_HISTORY_THROUGH_V100.md), [v140 완료 계보](base5/261010_base5_flexible_deficit_replacement.md) |
| Base5 v142~v158 | [공간·수집기 개선](base5/261010_base5_postcollection_collector_work.md), [확대 중단](base5/261017_base5_scale_split_qualification.md), [원형 release](base5/261017_base5_unit_release_evidence.md) |
| MORAI 자산·질의·동기 진단 | [공식 답변 원문](morai_reference/260920_questions_for_morai.md), [heading/동기 진단](morai_reference/260921_mr_heading_and_sync_annotation_calibration.md) |
| 정리 전 현재 context | [2026-10-24 원문 snapshots](context_261024/README.md), [기존 10/12 context](../../../../../docs/perception/legacy/context_261012/README.md) |

이동 전후 경로·원문 SHA는 [이동 manifest](MANIFEST.md)에서 확인한다.
과거 본문은 상대 링크까지 byte 그대로 보존했다. 당시 경로의 링크는 manifest에서 새 위치를 찾는다.
완료 원본·native GT·실패 근거·selected ledger·동결 source/runtime 해시는 변경하지 않았다.
옛 scripts는 전수 inventory와 import/`with_name()` closure 계약 때문에 원래 경로에 보존했다.
옛 config도 직접 코드 참조가 있으면 이동하지 않았다. 무참조 이력만 [config archive](../../configs/legacy/README.md)로 분리했다.
기존 수집에 변경된 source·manifest를 강제 resume하거나 부모 승인분을 중복 합산하지 않는다.

## 2026-10-28 승인된 10/8 RGB 보관 형식 변경

사용자 승인으로 `/home/luv/morai_cap/base5_v2_261008`의 RGB `_Intensity.png`
14,608개(3,652장면·4카메라)를 원래 해상도 Q90 JPEG로 변환했다. JPEG 전수 디코드와
Drive 압축본 MD5 확인, 원본/JPEG SHA 대조 뒤 해당 RGB PNG만 삭제했다.
Instance·Semantic·Depth·LiDAR·native GT·과거 메타/결과 442,572개 파일은 보존했다.
Q90은 손실 변환이며 무손실 PNG 백업이 아니다. 기존 수집 inventory/SHA·학습 infos·split은
고쳐 쓰지 않았고 완료 collector를 다시 resume하지 않는다. 새 분석은 외부 `frames.jsonl`을 쓴다.

- 도구: [범위 고정·업로드 검증 후 삭제](../../../../scripts/maintenance/base5_legacy_rgb_backup.py).
- 대응표·보존 목록·압축본 해시·완료 기록: `artifacts/maintenance/base5_legacy_q90_261028_v1/`.
- Drive: `dataset/Base5/legacy_base5_v2_261008_rgb_q90_261028/547b3c79a50182b6/`.
  이 압축본은 실험 전체 RGB와 분석 대응표용이며 native GT 본체는 로컬에 그대로 남는다.
  기존 최종 Sunny13 5K 학습 백업을 교체하거나 현재 30,750장 입력으로 추가한 것이 아니다.
