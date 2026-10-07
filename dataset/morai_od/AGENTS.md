# MORAI perception dataset agent context

이 디렉터리의 수집·변환 작업 전 다음을 읽는다.

1. `docs/README.md`
2. `docs/BASE3_ONLINE_CURRENT_STATE.md`
   - MR 신규 수집은 `docs/MR_ASSET_COLLECTION_CURRENT.md`의 catalog/profile이 우선한다. 과거 MRP 교집합·quarantine을 MR에 적용하지 않는다.
3. Base3-Online rosbag 수집이면 `docs/base3_online_rosbag_collection_runbook.md`
4. Base4 temporal 작업이면 `docs/BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md`와
   `docs/260920_base4_deskew_and_control_safety_evidence.md`
5. Base5 독립 single-frame 수집이면 `docs/BASE5_V2_CURRENT.md` →
   `docs/261007_base5_direct_5k_current_plan.md` →
   `docs/261007_base5_5k_capture_runbook.md` →
   `docs/261007_base5_final_gates_v105_evidence.md`를 읽는다.
   Sunny13시 직접5K가 현재 목표다. v45/10K 명령·과거 ETA를 재사용하지 않는다.
   `docs/261007_base5_sunny13_environment_correction.md`의 정정이 11시 오표기보다 우선한다.
   진행 중인 수집의 manifest/계약은 덮어쓰지 않는다. 기존 sunny11 config는 13시 내용의 호환 alias다.
   v111부터 정적 미션에도122종 전체를 정지 배치한다. family와 GT class를 혼동하지 않는다.
   v112 동적 미션250셀 중12개는 차량 정지 외관 관측이다. 새manifest 정적4171/이동829,
   구4160limit/구output을 재사용하지 않는다. 차량 양보만8–20m이며 희귀 quota는 유지한다.
   생성기 코드 변경 후 구 output resume 금지. 원래 revision 또는 새 manifest/output을 사용한다.
   본수집은 사용자 승인 후이며 raw GT·실패 근거를 보존한다.

CURRENT/EVIDENCE/PLAN/HISTORICAL 상태를 구분한다. 현재 학습 전달본은
`base3_online_highway_motion_v2`이며 nearest-state V1을 자동 선택하지 않는다.
