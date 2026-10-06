# MORAI OD dataset 문서 index

마지막 정리: 2026-10-06.

## Base5 근접·원거리 보강 — CURRENT PILOT

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
