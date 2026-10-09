# MORAI OD dataset 문서 index

마지막 정리: 2026-10-09.

- **최신 / 2026-10-09 환경별 새 공간 배치 합의:** [Sunny13 완료 후 Ego 다양화·나머지25K 계획](261009_base5_remaining_environments_spatial_diversity_plan.md).
  현재Sunny13 5K는 완료·보존한다. 종료/Park 뒤 구성표·주요자산 할당을 유지한 연속Ego 샘플러를
  구현·검증하고, 나머지5환경은 각각 새배치5K로 생성한다. 총30K 후 Sunny13 재수집 필요성을 판단한다.
  아래 같은geometry6환경 계획은 기존입력 재현 이력이며, 새생성 경로는 아직 미구현이다.
  기존 다른환경 복사manifest를 바로 수집하지 않는다. 이번 갱신은 문서만 변경한다.

- **2026-10-09 Base5 학습 준비:** [초기화·BEV 연산 최적화 계획](../../../docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md).
  crop 확정 후 visibility/info 재변환, 호환 nuScenes 초기화와4-class/depth/grid 로딩 감사,
  cache A/v2 B 검증 순서다. 작은 CUDA 연산 시험만 제한적으로 검토하며 현재 수집은 유지한다.
  Base5-only 데이터는 완전 랜덤 초기화 의무가 아니다. 실제 config/collector/production은 이번 정리로 변경하지 않는다.

- **최신 운영 / runtime v133 정지 실패 복구:** [중단 원인·성공분 import·검증](261009_base5_stationary_failure_recovery_v133.md).
  입력v132 유지, 기존1887승인 참조+실제다음1장승인. 새root paused/1888승인/24부족/3088미처리.
  정적 순간 흔들림 최대2초 재확인·검증된 정지 실패만 셀 부족분, 통신/Park 불명확은 중단 유지.
  [CURRENT](BASE5_V2_CURRENT.md)와 [재개 런북](261007_base5_5k_capture_runbook.md)이 아래 수집 중 표기보다 우선한다.

- **2026-10-09 학습·가시성·temporal 결정:** [현재 합의와1771장 가시성 근거](261009_base5_training_split_visibility_and_temporal_decisions.md).
  5K train4255/val745, 같은6환경 split고정·rosbag dev-val/독립test 분리다. 30K 수집 유지,
  완료후 필요한조건만보강한다. BabyStroller3의3/21은7parent×3component이며camera대응모호성도포함한다.
  현재E25/Base5 temporal·OD velocity는미사용, tracker/NMS 개선은유지한다. 본학습은아직미시작이다.

- **현재 본수집 / 2026-10-09 Sunny13 v132 수집 중:** 이번 배치를 유지하고 종료 후 반복 준비 비용을 개선한다.
  [시간 개선 이력·라벨 계약·다음 A/B 실행 계획](261009_base5_collection_cost_and_next_experiments.md)을 읽는다.
  현재 runtime/input 변경 금지, 다음 후보는 상주 worker/준비 대기 중첩이며20% 전체 단축을 검증한다.
  음영 이동24장 독립 준비는 유지한다. 아래 '미시작/승인 대기'는 각 준비 시점의 이력이다.

- **현재 / 2026-10-09 v132 최종 정지 외관·배치 형상:**
  [12카테고리·새 yaw·실험·노출 스냅샷·한계](261009_base5_final_appearance_v132.md)가 아래v129보다 우선한다.
  root `final_appearance_5k_v132_release`, 정지 비차량360°·OBJ/일반 정적 차량 prior,
  static parent 형상41종·음영walk/run 접근/이탈 균형을 적용한다. native GT/5K/122종/6환경30K 유지.
  구763승인/raw와v129/v130을보존하며본5K는현재Sunny13수집중이다.
  [CURRENT](BASE5_V2_CURRENT.md), [현재 실행 런북](261007_base5_5k_capture_runbook.md)을 따른다.

- **현재 / 2026-10-08 v129 정지 외관 yaw 재설계:** 아래v123/v125의 '현재/미시작' 표기는 당시 이력이다.
  사용자가 중단한 구Sunny13 수집763승인은 원본/압축백업 보존, 새입력은 `yaw_5k_v129_release`다.
  정지ped계열70%네방향±15°/30%360°·retry 재표본화, 일반사람근접33셀 제한적gap보완,
  전수5K/6환경·실제MR35/35·CPU35/91GT를 검증했다. 새본5K는 재시작하지 않았다.
  [변경·대조 실험·보존·한계](261008_base5_static_yaw_and_randomization_review.md),
  [CURRENT](BASE5_V2_CURRENT.md), [새 실행 명령](261007_base5_5k_capture_runbook.md)을 따른다.

- **2026-10-08 대시보드 보강/본수집 준비:** 사용자 확인은 Sunny13 5K 뒤 기존6환경30K다.
  v125 runtime/v123 입력 유지, 전수5000/6환경/입력백업 확인, 8075에 그래프/분포/부족사유/환경 진행을 추가했다.
  [준비 결과](261008_base5_dashboard_and_launch_ready.md), [현재 명령](261007_base5_5k_capture_runbook.md)을 따른다.
  본수집·학습은 미시작이다. 대시보드만 바뀌어 캡처resume hash는 v125와 동일하다.

- **2026-10-08 13:05 KST 진단 메타데이터 — runtime v125:** 기존 bounded Object 위치 trace에
  원문 velocity/heading/메시지 시각을 추가했다. 동기 속도 GT가 아니며 학습·평가에는 사용하지 않는다.
  추가 wait/topic/촬영 없이 새 MR 정지1+달리기1 승인, CPU2/2GT·완료resume 추가capture0·375tests PASS다.
  [기록 계약·비용·검증](261008_base5_motion_metadata_v125.md), [CURRENT](BASE5_V2_CURRENT.md),
  [런북](261007_base5_5k_capture_runbook.md)이 아래 v124 준비 기록보다 우선한다. 배치 입력은 v123 그대로다.

- **2026-10-08 12:14 KST 본수집 전 마무리:** v124 runtime/v123 입력 전수5000/6환경 확인,
  새MR33/33·CPU33/72GT·완료resume 추가capture0·370tests PASS다.
  [최종 확인](261008_base5_prebulk_v124_final.md), [CURRENT](BASE5_V2_CURRENT.md),
  [런북](261007_base5_5k_capture_runbook.md)을 따른다. 다음은 승인 후 Sunny13 본5K이며 아직 미시작이다.

- **2026-10-08 v124 운영:** v123의5K 배치는 유지하고 완료 부족분만 `--resume --replenish`로 보충한다.
  원래 셀별 누적 상한·성공/raw 보존·통합 ledger·pause/resume·dashboard를 지원한다.
  [보충 EVIDENCE](261008_base5_completed_replenishment_evidence.md), [런북](261007_base5_5k_capture_runbook.md),
  [CURRENT](BASE5_V2_CURRENT.md)를 따른다. 본5K는 미시작이며 구output에 새runtime을 혼합하지 않는다.

- **2026-10-08 v123 우선:** 고주로740/600·로터리0/350·기타차량80/50·장애물770/600·
  사람계열830/250·독립음영500·배경230으로5000셀을 준비한다. 고주로/로터리는 모든actor 차량만,
  비차량 범용은 다른 구역에 배치한다. 122종 primary 최소5, 장애물 노출2973개다.
  같은geometry6환경30K의group/split을 고정하며 날씨는UI확인이다. 정적4822우선/이동178후순.
  `regions_5k_v123_release`가 현재release다. [CURRENT](BASE5_V2_CURRENT.md),
  [v123 EVIDENCE](261008_base5_region_release_evidence.md), [런북](261007_base5_5k_capture_runbook.md)을 따른다.
  아래 v122 이하의 당시 경로·수량·결과는 이력이다. 본수집은 승인 후다.

- **2026-10-08 v122 이력:** 음영500셀을 바깥 접근150/진입90/내부190/출구70으로 보강한다.
  바깥145positive가 실제 도로상 Ego와 내부 전방 객체를 함께 만족하도록 배치한다.
  오른쪽80/왼쪽20·좌우 엇갈림93셀,476정적 변경/4524entry 보존·5K/122종 최소4/178이동 유지.
  current `shadow_approach_5k_v122_b`; 전수offline/대표live20/20·CPU/resume 검증, 본수집 승인 대기다.
  [CURRENT](BASE5_V2_CURRENT.md), [사전 접근 EVIDENCE](261008_base5_shadow_approach_evidence.md),
  [런북](261007_base5_5k_capture_runbook.md)이 아래 v121 기록보다 우선한다.

- **2026-10-08 v121 우선:** 5K의 범용500셀을 음영 입구120/내부260/출구120으로 재배정한다.
  전체122종을 포함하고 기존1750미션/초근접/로터리 범용을 보존한다. 정적4822/이동178이며
  음영 mover24셀은 사거리 높이 lookup 대신 독립 terrain 준비를 사용한다.
  나머지4500 entry/source는 불변이다. 본수집은 미시작/사용자 승인 대기다.
  현재 경로 `shadow_release_5k_v121`; [CURRENT](BASE5_V2_CURRENT.md),
  [음영 근거·한계](261008_base5_shadow_observation_evidence.md), [런북](261007_base5_5k_capture_runbook.md)을 따른다.
  아래 각 revision의 당시 수량/경로는 이력이며 구 output resume에 새runtime을 섞지 않는다.

- **2026-10-08 v119 유지:** 로터리 미션250셀을 샘플 NPC 경로/실제 Ego 통과 관측으로 교체한다.
  접근20/대기30/진입60/원형 주행90/탈출50, 원형 주행은 앞뒤 차량을 함께 관측한다.
  나머지4750셀 geometry·정적4846/이동154·122종 최소4는 보존한다. 정적 NPC는 초기 Park로 고정한다.
  나머지 중2314셀은 기어/검사 metadata만 갱신했다. 본수집은 자동 시작하지 않는다.
  현재 release는 `roundabout_release_5k_v119`이다. [CURRENT](BASE5_V2_CURRENT.md),
  [로터리 근거](261008_base5_roundabout_route_mission_evidence.md), [런북](261007_base5_5k_capture_runbook.md)을 따른다.
  구 output에 새runtime으로 resume하지 않는다.

- **2026-10-08 v117 이동 계약 유지:** 샘플처럼 출발→도착 두 waypoint의 선형 이동으로 변경했다.
  28종×두 횡단보도×양방향112개 승인용 캡처 PASS. 전체127시도 중 LiDAR GT 누락1건도 보존한다.
  일반 사람의 걷기/달리기 반대 방향을 각각 균형 배정하고 재시도에서도 방향을 고정한다.
  당시5K는 `endpoint_linear_final_5k_v117`이며 최신 v119에서 이동 계약/높이 lookup을 유지한다.
  [CURRENT](BASE5_V2_CURRENT.md), [근거](261008_base5_intersection_ground_evidence.md),
  [런북](261007_base5_5k_capture_runbook.md)을 우선한다. 본수집은 승인 대기다.

- **2026-10-08 v116 이력:** 사용자 보정10개/49waypoint를 원본·sidecar와 백업하고 actual-XY 기반
  높이 보간을 구현했다.28종×두 횡단보도56profile/132회 live PASS, 작은 높이/근거 부족은 warning이다.
  새5K offline 생성4846정적/154이동·122종 최소4회. 사거리 밖은 정지 캡처이며 본수집은 승인 대기다.
  [현재 결과·제약](261008_base5_intersection_ground_evidence.md), [CURRENT](BASE5_V2_CURRENT.md),
  [런북](261007_base5_5k_capture_runbook.md)을 우선한다. 아래 v115 이전 수량/ETA는 이력이다.

- **v115 현재 정적 QA:** 추가 지면 확인용 이동/촬영 없이 원래 LiDAR만 검사한다.
  증거 부족은 warning, 명확한40cm 초과 높이 오차는 제외한다. Ego 정지/충돌/GT/가시성은 유지하며
  동적·calibration 경로는 완화하지 않는다. 실제40셀39승인·271.3초, 정적4171장 약8.1h 조건부 외삽이다.
  [CURRENT](BASE5_V2_CURRENT.md), [실측 근거](261007_base5_static_height_evidence.md#정적-qa-경량화와-40장면-실측--v115)를 우선한다.
  구 manifest/output resume 금지, 새5K manifest 준비가 다음이다.

- **v114 Ego 드리프트 유지:** 도로 밖 가상 회피 Ego 배치를 재현/A-B로 분리했다.
  최종 footprint 도로 지지·회피 쌍 공동 이동·Park 뒤 실제 정지 witness가 현재 계약이다.
  구 v113 output resume 금지, 새 5K manifest 준비가 다음이다.
  [CURRENT](BASE5_V2_CURRENT.md), [Ego 근거](261007_base5_static_height_evidence.md#ego-드리프트-원인-분리와-보완--v114)를 먼저 읽는다.

## Base5 근접·원거리 보강 — CURRENT PILOT

- **v113 최종 XY 초기 높이:** XY 이동 뒤 이전 Z를 쓰던 정적 배치를 도로 선분/연결 경계
  보간으로 보완했다. 가상 lane-change link 제외, 실제 Dog2 A/B 및 정상 근접 검증,
  원본 재촬영 없는 ground witness 보강과 정적 QA 비용 제한을 적용했다.
  새5K/남은 roadside·reference mismatch 및 실패 근거는
  [CURRENT](BASE5_V2_CURRENT.md)와 [높이 EVIDENCE](261007_base5_static_height_evidence.md#최종-xy-기준-보완--v113)를 따른다.

- **v112 동적 미션 차량 소량:** 250슬롯 중12개(4.8%)를 차량 진입/횡단 관측으로 보강했다.
  일반 사람 슬롯만 교체하고 희귀 복합/동물 quota를 유지한다. 차량은 도로 link 위 정지 캡처다.
  새5K와정적4171/이동829순서·대표검증·새output은 [CURRENT](BASE5_V2_CURRENT.md)를 따른다.

- **v111 정적 미션 전체 자산:** 650개 정적 회피 슬롯에 122종 모두 정지 배치한다.
  일반 차량43종·사람·동물·복합도 포함하며 GT 클래스는 원본대로 유지한다.
  나머지4350장면 불변, 새5K OFFLINE_PASS/전체 planned-primary 최소4장이다.
  새 manifest/output 및 대표 live 범위는 [CURRENT](BASE5_V2_CURRENT.md)를 따른다.
  구 output의 resume에 새 runtime을 섞지 않는다.

- **현재 진입점:** [Base5 CURRENT](BASE5_V2_CURRENT.md) →
  [Sunny13시 직접5K 구성](261007_base5_direct_5k_current_plan.md) →
  [5K 운영 런북](261007_base5_5k_capture_runbook.md).
  [v105 최종 gate 근거](261007_base5_final_gates_v105_evidence.md)에 실패·보완·실제 결과를 모은다.
  [Sunny13시 환경 정정](261007_base5_sunny13_environment_correction.md)이
  최근 11시 오표기보다 우선한다. 이미 진행 중인 수집의 manifest/계약을 덮어쓰지 않는다.
  아래 v45/v86 등 "최신" 표현은 해당 시점의 역사이며 현재 실행 명령으로 사용하지 않는다.

- **manifest v105 / runtime v108:** 새5K/122종 전수오프라인 PASS. 차량 미션 근접133셀, 범용 근접175셀,
  크기별 혼합 예산 및 component-preserving GT 유지. 범용 동시 이동 최대2개,
  주 객체/미션 이동 외관은 유지한다. 최종 live/resume/비용 상태는 CURRENT를 확인한다.

- **v98 범용 근접:** [구현·실패·보완·한계](261007_base5_generic_close_v98.md).
  근거리175셀 경계 간격 기반 alongside, Ego 도로 anchor 유지. 좌우 대표2/2,
  GT6/작은 낮은 객체 가시성 실패는 미해결. 차량 근접은 [후속 v100](261007_base5_vehicle_close_v100.md)에서 구현했다.

- **v95 동적 미션 지도 기준 배치:** [구현·실제 검증·한계](261007_base5_dynamic_map_observation_v95.md).
  전방 접근/횡단 관측/양보90%, 통과 직후10%. 250/250 생성 가능,
  대표 live5/5·CPU5장/GT6·212tests PASS. 최종5K/전체미션 인증은 아니다.

- **최신 목표: Sunny13시 직접5K.** [5K구성·설정·미완료gate](261007_base5_direct_5k_current_plan.md).
  구10K산출물은보존하며새기본config로5K를설계한다. 아래10K실측은과거이력이다.

- **범용/미션 객체 수 v89 구현:** [크기 예산 배치·검증](261007_base5_population_budget_v89.md).
  현재5K의 범용positive3K는단일30/일반혼합50/밀집20%,미션은상황별개수로분리했다.
  512장OFFLINE_PASS,실제가시parent/component분리. 최종v93 live4/4는소량근거이며전체gate완료가아니다.

- **본수집 전 최신 PLAN:** [최종 학습 후보5개 보완](261007_base5_final_candidate_five_gates_plan.md).
  구10K/A·B 선별계획보다 [최신5K 작업 순서](261007_base5_direct_5k_current_plan.md#다음-작업-순서--현재-실행-기준)가 우선한다.
  동적미션 위치 기준·초근접·밀집·GT·희귀자산coverage를 보완한 뒤 새5K를 고정한다.
  v86 검증 완료와 현재계획 구현 완료를 혼동하지 않는다.

- **역사 v86: 당시 recovery·통합검증 완료.** 32셀31승인/1부족분 후
  Ped_Pet1 near terrain 보완1/1,최종8/8·동적2/2·ordered4/4·198tests PASS.
  새10K 전수OFFLINE_PASS/실캡처 미시작. 구18.7h ETA 폐기,시간 비용 증가 명시.
  [현재](BASE5_V2_CURRENT.md),
  [근거·제한](261007_base5_prebulk_recovery_evidence.md).

- **역사 v77: static secondary 지면 검증/복합·동적 확인.** 초기 혼합2/4→최종2/2,
  근접 복합2/2·동적2/2(준비 포함47.14초), 구 readiness 이동 계약 수정.
  완료resume 추가촬영0/183tests PASS. 먼 복합/혼합Dog2 보류, 본10K 미재개.
  [현재](BASE5_V2_CURRENT.md), [근거·한계](261007_base5_mixed_motion_ground_evidence.md).

- **역사 v72: 원거리 rigid 정적 ground 검증·원본 재검사.** 동일 배치/실제 pose/hash와
  전체 상대 R/T를 확인해 sparse ground를 보완. 거리44/97/후방88/대형53m4조건 통과,
  같은4조건78.62→53.60초/재촬영4회 생략. 완료resume 추가촬영0/179tests PASS.
  본10K·18.7h 확정 ETA 아님. [현재](BASE5_V2_CURRENT.md),
  [근거·적용 제한](261007_base5_static_rigid_ground_evidence.md).

- **역사 v66: 정적 자산별 높이 후보/전체actor ground gate.** 실제4조건3통과/1보류,
  근접Kona 정상으로 원거리 부유 확정 해석 철회. yaw-only 장거리 지면 보정 금지,
  원거리 좌표계/평면 오판 분리가 남음. 본10K/v59 명령 자동 실행 금지.
  [현재 계약](BASE5_V2_CURRENT.md), [검증 근거](261007_base5_static_height_evidence.md).

- **본수집 터미널 명령의 단일 기준:** [직접5K 운영 런북](261007_base5_5k_capture_runbook.md).
  구 [v45 운영 런북](261006_base5_bulk_resume_v45.md)은 역사이며 당시 본수집은 충돌 중단 상태다.
  rc → guarded bridge → MR7-interface/Capture Mode → 수집 → 대시보드,
  각 인자 뜻·PAUSE/재개·halted 주의를 한곳에 정리했다. rosbag/UDP 런북과 구분한다.

- **역사 v59: Ego Park·확인된 충돌 부족분으로 분리 후 다음 장면 진행.**
  실제 Park 동적3/3, 시험용 CollisionData 입력→실패 제외→다음 정상 실제 캡처 통과.
  완료 resume 재촬영0/CPU 정상1장만 편입. 입력 주입 통합시험과 실제 물리 충돌 복구는 구분한다.
  통신·미확인 Park·새 장면 상태 확인 실패는 계속 halt. [현재 계약·한계](BASE5_V2_CURRENT.md).

- **v48: 충돌 원인 수정·정적 우선 및 지형 기반 동적 검증.**
  정적32/32·동적/혼합3/3, Base5 155tests PASS. 정적8,354셀 전수OFFLINE_PASS,
  동적1,646셀은 지형 준비를 위해 별도 보류. 본수집 자동 재시작/본 학습 없음.
  [현재 상태·실제 근거·한계](BASE5_V2_CURRENT.md). 아래 기록은 각 버전 당시 상태다.

- **과거 v45: 6초10K 전수OFFLINE_PASS·bulk 실행/PAUSE/resume 연결.**
  실제12/12+marker8/8·CPU GT28·143tests PASS, 완료resume 추가촬영0.
  명시적bulk승인·4셀 child·64셀 목록checkpoint·누적시간 보존.
  v35 baseline보존/본10K·학습미시작. [실행·정지·재개 런북](261006_base5_bulk_resume_v45.md).

- **v44: 후방·복합8/8 + 새자산56/56.** 추가64장/66촬영,138tests PASS.
  10K 자연 이동1,646셀의9→6초절약약1.37시간. 대표외삽18.71시간 유지,
  목표누락재시도 가정5–10%/추가30초면19.40–20.09시간. 기본9초/본수집미시작.
  [실측·시간 계산](261006_base5_diverse_coverage_eta_v44.md).

- **v43: 자연 이동16/16·혼합32/32·216.00초.** 6초 준비 후보의
  안전 실패→9초 재시도를 실제 지연 진단으로 확인, 진단 데이터 학습 제외.
  10K 예상18.71시간은 소량 표본 외삽. 기본9초 유지/본수집 미시작.
  [근거](261006_base5_budget6_fallback_v43.md).

- **v42: 한campaign32/32·233.02초/약26% 단축.** chunk8 pause/resume16/16,
  동적출발여유9→6초후보4/4·약25% 단축. 완료resume snapshot 경합수정·132tests PASS.
  canonical10K 기본9초/production은미변경,본10K/학습미시작.
  [근거·최종32장변환경로](261006_base5_batch_capacity_budget_v42.md).

- **v41: guard live·대표32/32·중복 촬영 제거.** 같은4셀53.93→41.95초,
  실제6→5촬영.123tests/완료resume/CPU변환 통과. 전환 오류2회는retry회복,
  timeout0이나장시간인증은 아니다. [근거](261006_base5_guard_seed_recovery_v41.md).
  본10K/학습 미시작.

- **v40 당시: rosbridge 서비스 세대 간 응답 혼선 재현·opt-in 보완.** 추가8quota통과로28/32.
  남은4셀은3raw보존/1미로드다. 기존bridge재시작·MR재연결후guard live검증이필요하다.
  [원인·코드·재연결 명령](261006_base5_eventcmd_rosbridge_v40.md). 본10K/학습미시작.

- **역사 v39: 재연결·안전 ROS batch·quota 보완 연결.** 새6장 pause/resume6/6,
  첫12셀 최종12/12·실제14촬영·CPU17GT 변환 통과. 간헐Park timeout은 미해결이며
  실패boundary는보존/재개금지. 후속20셀은8quota후최종Park미확인으로중단, 합계20/32다.
  109tests PASS. 남은12셀·종료서비스지연 및한계는 [최신근거](261006_base5_safe_batch_quota_v39.md).
  본10K캡처/학습은미시작이다.

- **최신 v38: ROS공유6/6 통과·13.85% 단축 / 네트워크 재사용 실패로 live 중단.**
  network flag=false 두 번째 load에서client0·종료Park미확인. 옵션금지, 모든scene network reload유지.
  사용자 재연결·사전Park 후 새 안전batch/stop-resume검증이 남음. 본10K미시작.
  [최신근거·복구순서](261006_base5_ros_batch_network_evidence_v38.md).

- **최신 v37: 비용 분리·안전 조건부 대기 단축.** 동일6셀56.6→50.7초(10.3%),
  양쪽6/6통과·Park 확인.32장 image cache 감사출력 정확일치, tests100PASS.
  본10K 미시작, 구세션 hash변경 resume 금지. [최신근거](261006_base5_capture_stage_optimization_v37.md).

- **최신 v36: 대표32셀 실제 캡처32/32 통과.** 36회촬영/314.657초, 보완3셀.
  현 방식10K 예상27.53시간·원본54.71GiB(작은 coverage 표본의 외삽).
  본10K 캡처/학습 미시작. [비용·재시도 근거](261006_base5_capture_capacity_v36.md).

- **최신 v35: 10K 시나리오 생성·전수 OFFLINE PASS.** 자연속도/정적자산 정책, 열린G80 제외,
  122종 primary coverage. moving center 경계1건을 수정·재생성했고 tests94개PASS.
  본10K 캡처/학습은 미시작. [canonical 경로·분포·제한](261006_base5_final_10k_v35.md).

- **v33 공식 정책 코드 반영 / live 완료:** 열린 G80 공식 제외, rigid/차량 정지 캡처 적용.
  새12장 offline 및live12/12 PASS, 부족0. pause/resume·Park/충돌·동적Z·CPU30GT 변환 확인.
  원거리 약96m 차량은 camera 근거 중요,20m밖 ground 인증/모델 정확도 검증은 아니다.
  [근거·다음 실행](261006_base5_official_reply_smoke.md).

- **공식 답변 갱신:** [객체 출제 범위·G80·UDP 차량 velocity](260920_questions_for_morai.md#최신-추가-답변--객체-범위g80차량-속도-기준).
  F2 전체 객체가 후보이며 기존 정적 자산의 이동 예외는 없다. 열린 G80은 공식 출제 제외,
  차량 velocity는 후륜 차축 기준으로 확인됐다. 타 자산 부유는 자료 요청 상태다.
  현재 코드/평가기 기본값/학습은 이 문서 갱신으로 변경하지 않았다.

- **최신 v32:** MR OFF에서batch1 및E25 ROS포화 속도를 측정했다.
  Base5320×896/±100m 후보는5060Ti에서약10.58FPS 비용probe다.4090/후보정확도/후보ROS는미측정.
  [CURRENT](BASE5_V2_CURRENT.md), [capacity EVIDENCE](../../../docs/bevfusion/261006_BASE5_BATCH1_CAPACITY_EVIDENCE.md).

- **최신 v31:** 남은 근접·혼합3셀 통과. quality loop를 bounded campaign의 재시도·quota 보충·resume에 연결했다.
  소량 live stop/resume 및89 tests 통과. 본10K·학습·GPU속도 미실행.
  [CURRENT](BASE5_V2_CURRENT.md), [v31 EVIDENCE](261006_base5_campaign_recovery_evidence.md) 우선.

- **최신 v30:** Z 여유0/2/5/10cm 및 실측 지면 보정·아래쪽 대조60시도/57raw.
  작은 상향 여유는 부유를 해결하지 않았다. 지면 기준 입력/waypoint 보정 후24/24 통과.
  일괄Z상향·전역asset offset은 적용하지 않는다.3세션 종료Park 확인,간헐첫timeout은남음.
  [CURRENT](BASE5_V2_CURRENT.md), [v30 EVIDENCE](261006_base5_z_margin_probe_evidence.md) 우선.

- **v29 이력:** zero XYZ 무효점에 의한 근접 지면 오판을 수정했다. 과거 근접·혼합13장
  재검사는9통과/4review다. 독립 empty-ground 촬영/보정·동일cell 재배치 자동loop 구현.
  안전 종료4장/GT8개 학습 변환 CPU smoke PASS. 자동loop 두 번째 종료Park2회timeout으로
  live 중단, 나머지2cell 미실행. 재연결 후 소량 마무리 필요, 본10K/학습/GPU속도미측정.
  [CURRENT](BASE5_V2_CURRENT.md), [v29 EVIDENCE](261006_base5_ground_quality_loop_evidence.md) 우선.

- **v28 이력:** 자산별 정지/동적 외관 정책·자연 속도 뒤쪽 출발을 일반 생성기 기본에 통합했다.
  근접·혼합13장 중 최종 지면 검사 전체 통과2장, 나머지는review다. 별도 입력 Z 수정 후
  재촬영한 개4조건은 공중 gap25–29cm→약−3cm로 개선됐다. 안정된Z만으로 지면을 인증하지 않는다.
  다음은 근접 지형 높이/sparse-ground 보강이며 본10K는 미시작이다.
  [CURRENT](BASE5_V2_CURRENT.md), [지면 EVIDENCE](261006_base5_appearance_ground_integration_evidence.md) 우선.

- **v27:**대표13자산×정지/이동×2반복의52조건/80component GT 비교 완료.
  관절 변화가 있는 사람·동물은 동적 phase 유지, 현재 fixed-pose 복합 객체/rigid 객체는
  정지 우선 권장. object 이동 높이 차이는12조건 matched-height 재촬영으로 제거했다.
  당시 외관 비교이며 지면에 붙음을 인증한 결과가 아니다. 후속 통합은v28 참조.
  [CURRENT](BASE5_V2_CURRENT.md), [외관 EVIDENCE](261006_base5_asset_motion_appearance_evidence.md) 우선.

- **v26:**같은Woman2 속도0/0.35/1.1/2.2m/s×3반복의 뒤쪽출발/실제도착 캡처 검증.
  최초10성공/2차단,재시도 후12유효 캡처. 이동 시 실제 자세/GT 형상 변화 확인.
  자연속도 유지 접근은 가능하며 일반generator 기본 전환·전체동적 자산 검증은 아직이다.
  준비시간 보통3.7초/기록최대8.5초,Park 간헐timeout 미해결.
  [CURRENT](BASE5_V2_CURRENT.md)와 동기캡처 런북 우선.

- **v25:**남은2조건 재캡처 후 yaw 유효42/42조건 완료.
  0°6/6,±3°12/12,±10°12/12,±15°10/12 포함. CollisionData 전부 비충돌.
  Park 사전probe 성공,최종 종료 첫timeout/재시도 성공;간헐지연 원인은 미해결.
  다음은MR 종료 후100m 후보GPU latency/VRAM 비교. 본 수집 승인 아님.
  [CURRENT](BASE5_V2_CURRENT.md) 우선.

- **v24 이력:**실제 yaw0/±3/±10/±15° paired 검사 유효40/42조건.
  ±15° 곡선2조건 횡30m 밖, 직선/인접 차선 검증27조건 모두 포함.
  NPC runtime 이동 표본 제외·재촬영 gate 추가. 종료Park timeout으로 live 중단;
  2조건 대기, 본 수집 승인 아님. [CURRENT](BASE5_V2_CURRENT.md) 우선.

- **v23:**빠른 동작 profile·all-actor 캡처 motion/endpoint gate·48조건 분산검증.
  최초46후보/1검토/1실패,2조건 재배치 및 최종gate 반복 후48조건 canonical 선택.
  GT75개,CPU raw/PKL 검사 통과. lidar1_camera0 기본 결정,실제camera-only5개 보존.
  본10K 생성·캡처·학습은 미실행. [CURRENT](BASE5_V2_CURRENT.md)와 런북 우선.

- **v21 이력:**왕복 대신 heading-aligned 저속 일방향2-waypoint 구현.
  실제6조건+4조건 속도반복10캡처 중9후보/1Dog2 가시성검토, 종료Park·resume 통과.
  새 생성default는 일방향/6초준비여유. 기존 왕복10K deck은 구버전이고 새10K는 생성하지 않았다.
  빠른 동작외관·전체pool 검증은 남음. [CURRENT](BASE5_V2_CURRENT.md) 참조.

- **v20 이력:**종료 Park을 gear-only 요청으로 최소 수정. 기존 동시 요청2회는
  첫 요청 timeout/재시도 성공, gear-only3회는 첫 요청4–11ms 성공. 장시간 검증은 아님.
  [CURRENT](BASE5_V2_CURRENT.md), [런북](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md) 참조.
- **v19 이력:**동일cell 가시성 보충 구현, Dog2 재배치1장 capture 통과(2point).
  종료Park2회 timeout으로 live 중단; 후속로드/10K/학습 없음. GT정책3종 비교와
  info변환/안전중단·resume 테스트35개 통과.
  [CURRENT](BASE5_V2_CURRENT.md), [런북](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md) 참조.

- **v18 이력:**16장 캡처15후보/1가시성검토/0실행실패, 근접 회피8장 통과.
  동적 거리 경로 수정, 추가 근접 유모차/탑승자 및 static AI 존재 확인 검증 완료.
  CollisionData 이력·강제 중단 후 raw 보존·완료 세션 재실행 검증 완료.
  [CURRENT](BASE5_V2_CURRENT.md)와 [실행/재개 런북](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md)을 먼저 읽는다.
  최신10K 후보는v18d offline PASS. 본 수집은 미실행이며 sparse 가시성 보충·최종GT 정책은 남아 있다.

- [독립 Base5 v2 최신 구현·검증·다음 실행](BASE5_V2_CURRENT.md): 최신 v29은 위 항목.
  이전10K 전수 offline PASS, 100m/4-class 후보 GPU loss·역전파 및48장 live41후보/7검토는 EVIDENCE다.
  v15는 미션 차량·장애물 회피 관측1,300장과10–50cm 측방 gap 후보를 추가했다.
  8장+대조4장 실제 캡처, 약13–34cm actual gap 확보. 지면/collider 증거가 없어 review 유지.

- [SaveSensorData Ego 정지/이동 A/B](261005_savesensordata_ego_motion_ab.md): 정지2장·실제28/49km/h 직진 각2장. 현재 동기 캡처에서 큰 속도 의존 형상 왜곡은 미관측, 정지 배치 유지의 제한적 근거. UDP deskew·Camera blur·고속 회전과 분리한다.

- [독립 Base5 10K 상세 설계 — PLAN](BASE5_INDEPENDENT_10K_DESIGN.md): Base3/E25와 별도 corpus, 범용60/미션35/배경5, 고주로·로터리·회피·동작 외관 세부 quota. 기존 v1 구현과 구분한다.

- [G80 배치·복합객체 GT 후속 확인](261005_g80_and_composite_gt_followup.md): G80 수동actor/다른Ego·위치·배치필드9-case 재로드 및 사람/개/자전거/유모차/아기 component 확인.

- [새 MR 카탈로그 연결·12-scene 실제 검증](261005_base5_mr_asset_review_smoke.md): static Vehicle 연결, G80 지면 실패/보류, 복합 객체 동일UID 다중GT 발견. 본수집 전 남은 gate.

- [Base5 MR 동기 캡처 Network](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md): 목적별6-interface JSON 및 MR 설치 경로. 정적 검증과12-scene live 캡처 통과.

- [MR 새 자산 목록 검수·포함/제외 합의](MR_ASSET_COLLECTION_CURRENT.md): 새43/33/70종과 Base5 pool 연결 완료. 동물/아기/복합 객체의3-class 학습 정책은 합의 완료·변환 구현 대기. 표지판은 도로변 전용.

- [Base5-Static 생성기·10k 설계](BASE5_STATIC_CURRENT.md): SaveSensorData single-frame용 bounded generator. 자산 review live 완료, 전체 배치/본 수집은 미실행. 최대64 scene/batch, 대량 사전 생성 금지.

- [MR 복구 객체 rosbag 보강·학습 결과](261004_base5_restored_replay_results.md): 20 scene/60 frame, WoodBox6장 camera review, 727/124 replay E2 완료. Offline best E1, production 미변경.
- [Base3 분포 공백·완료 학습 분석·다음 실행](../../../docs/bevfusion/261005_BASE3_GAPS_AND_BASE5_PILOT_RESULTS.md): 초근접·원거리 허점과 후보 rosbag A/B 계획.
- [복구 객체 GT 보정 결과·재연결](261004_mr_restored_asset_gt_calibration_runbook.md): MR20 scene/40 capture 완료, 복구4종 reference 확보, CargoBox 역사적 size 차이와 UDP 재연결 필요사항.
- [MR 전용 asset 수집 기준](MR_ASSET_COLLECTION_CURRENT.md): WoodBox·MRP 제외 자산 복구, MR 원본 template/default, GT calibration 미완료 항목.
- [`261004_base5_diversity_and_replay_pilot.md`](261004_base5_diversity_and_replay_pilot.md)
  - 후속 43/48 scene, 거리 margin 검증, 명시적 camera-sparse review 정책, 688/115 replay 학습 준비.
- [`261004_base5_reinforcement_smoke_runbook.md`](261004_base5_reinforcement_smoke_runbook.md)
  - MR 24개 정지 + 2개 저속 통과 수집, 84-frame 변환, sparse/거리 경계 품질 gate와 실행·모니터 명령.
  - 본 학습 dataset이나 production 교체 결과는 아니다.

## Base4 rosbag capture — CURRENT

2026-09-30 live 복구 검증 및 본 수집 보류 사유:
[`260930_base4_competition_v2_live_validation.md`](260930_base4_competition_v2_live_validation.md).

1. [`BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md`](BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md)
   - MR/MRP minimal hybrid 계약, 녹화 topic, 실행·audit·임시 변환, 다음 temporal gate.
2. [`morai_mr_mrp_hybrid_scenario_capture_runbook.md`](morai_mr_mrp_hybrid_scenario_capture_runbook.md)
   - launcher별 전체 터미널 순서와 live 검증 근거.
3. [`../../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](../../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md)
   - 고주로 Vehicle velocity head/causal 3-sweep 수집·학습·평가 PLAN.
4. [`260921_mr_heading_and_sync_annotation_calibration.md`](260921_mr_heading_and_sync_annotation_calibration.md)
   - MR `/Ego_topic.heading`의 `-1.347 deg` 보정과 동기 annotation 교차검증 EVIDENCE.
5. [`260922_base4_velocity_diagnostic_pilot_runbook.md`](260922_base4_velocity_diagnostic_pilot_runbook.md)
   - 속도 head 원인 분리를 위한 28-scene MR 진단 수집의 준비 경로와 터미널별 실행 명령.
6. [`260922_base4_velocity_hardcase_v2_results.md`](260922_base4_velocity_hardcase_v2_results.md)
   - 속도 진단 + 병목 + 일반 고속 횡 offset을 합친 51-scene V1/T1 학습 준비 결과.
7. [`260922_base4_highway_first_experiment_dataset.md`](260922_base4_highway_first_experiment_dataset.md)
   - 159-scene raw와 157-scene Ego-deskew twin으로 완성한 고주로 1차 실험 dataset.

## Base3-Online — CURRENT BASELINE / EVIDENCE

다음 순서로 읽는다.

1. [`BASE3_ONLINE_CURRENT_STATE.md`](BASE3_ONLINE_CURRENT_STATE.md)
   - 현재 경로, 데이터 규모, 최종 변환 계약, 전달물, 다음 실험을 한 페이지에 정리.
2. [`base3_online_rosbag_collection_runbook.md`](base3_online_rosbag_collection_runbook.md)
   - 실제 MORAI/ROS 연결, 녹화, audit, 변환 명령.
3. [`260919_base3_online_highway_vehicle_1k_results.md`](260919_base3_online_highway_vehicle_1k_results.md)
   - 수집·정합·visibility·Docker 검증 수치가 있는 EVIDENCE.
4. [`260920_base3_online_highway_static_v2_collection_plan.md`](260920_base3_online_highway_static_v2_collection_plan.md)
   - Base4 전 controlled experiment용 1,000-scene 정적 Vehicle 추가 수집 PREPARED.
5. [`260920_morai_spawn_asset_and_multimodal_gt_plan.md`](260920_morai_spawn_asset_and_multimodal_gt_plan.md)
   - MR/MRP asset 호환성, 고주로 Random Spawn 분포 가설, WoodBox와 BEVFusion
     camera-visible sparse GT 후속 실험 EVIDENCE/PLAN.
6. [`260920_questions_for_morai.md`](260920_questions_for_morai.md)
   - MORAI 담당자에게 전달할 핵심 질문 다섯 가지와 multi-asset GT 정합 첨부 근거.

### 현재 사용 경로

| 용도 | 경로/이름 | 상태 |
|---|---|---|
| 원본 bag | `rosbags/datasets/base3_online/highway_1k/` | CURRENT source |
| runtime metadata | `/home/luv/morai_cap/260919_base3_online_highway_vehicle_1k/` | CURRENT source |
| nearest-state dataset | `artifacts/datasets/morai_base3_online_highway_v1/` | EVIDENCE only |
| motion-aligned dataset | `artifacts/datasets/morai_base3_online_highway_motion_v2/` | CURRENT local |
| 팀원 해제 경로 | `$HOME/ajounice_bevfusion_artifacts/datasets/base3_online_highway_motion_v2/` | CURRENT portable |
| PKL 내부 root | `/datasets/base3_online_highway_motion_v2` | CURRENT container |

## Base3-Static — LEGACY/EVIDENCE

- [`legacy/base3_static_10k_final_capture_audit.md`](legacy/base3_static_10k_final_capture_audit.md)
- [`legacy/base3_10k_route_wide_static_design.md`](legacy/base3_10k_route_wide_static_design.md)
- [`base3_static_asset_catalog.md`](base3_static_asset_catalog.md)
- [`legacy/base3_dynamic_ped_addon_v1_capture_audit.md`](legacy/base3_dynamic_ped_addon_v1_capture_audit.md)

Base3-Static은 폐기된 데이터가 아니다. Production E25를 만든 general-domain baseline이다.
완료된 설계·학습·감사 문서는 `legacy/`에 보존하며 Base3-Online 또는 Base4 수집
명령을 찾을 때 먼저 읽지 않는다. Asset catalog 두 문서는 시나리오 자산 조회를 위해
상위에 유지한다.

## HISTORICAL

다음 문서는 `legacy/`로 이동한 연구 이력 또는 이전 dataset 세대다.

- `legacy/base2_*`, `legacy/dataset_v2_*`, `legacy/temporal_sequence_*`: Base2/temporal 연구.
- `legacy/sunny13_*`, `legacy/od180_*`: 이전 정적 생성기 세대.
- `legacy/rosbag_replay_dataset_migration_plan.md`: ReplayInfo 중심의 이전 migration 계획.
- `legacy/production_online_rosbag_dataset_plan.md`: 완료된 Base3-Online 설계·pilot 계획.
- `legacy/geometry_trial_20.md`, `legacy/scenario_runner_*`: 초기 geometry/runner 검증.

이 문서들의 명령·경로·class 정책을 Base3-Online에 자동 적용하지 않는다.
