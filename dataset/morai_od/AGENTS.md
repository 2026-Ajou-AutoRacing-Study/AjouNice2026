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
   Sunny11시 직접5K가 현재 목표다. v45/10K 명령·과거 ETA를 재사용하지 않는다.
   본수집은 사용자 승인 후이며 raw GT·실패 근거를 보존한다.

CURRENT/EVIDENCE/PLAN/HISTORICAL 상태를 구분한다. 현재 학습 전달본은
`base3_online_highway_motion_v2`이며 nearest-state V1을 자동 선택하지 않는다.
