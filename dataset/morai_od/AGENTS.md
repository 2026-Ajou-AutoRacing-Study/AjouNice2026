# MORAI perception dataset agent context

이 디렉터리의 수집·변환 작업 전 다음을 읽는다.

1. `docs/README.md`
2. `docs/BASE3_ONLINE_CURRENT_STATE.md`
   - MR 신규 수집은 `docs/MR_ASSET_COLLECTION_CURRENT.md`의 catalog/profile이 우선한다. 과거 MRP 교집합·quarantine을 MR에 적용하지 않는다.
3. 작업이 수집이면 `docs/base3_online_rosbag_collection_runbook.md`
4. Base4 temporal 작업이면 `docs/BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md`와
   `docs/260920_base4_deskew_and_control_safety_evidence.md`

CURRENT/EVIDENCE/PLAN/HISTORICAL 상태를 구분한다. 현재 학습 전달본은
`base3_online_highway_motion_v2`이며 nearest-state V1을 자동 선택하지 않는다.
