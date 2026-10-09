# AjouNice2026 활성 최우선 계획

마지막 갱신: 2026-10-09.

이 문서는 **아직 끝나지 않았고 다음 작업에서 반드시 다시 확인할 항목만** 둔다.
완료된 수집·학습·실험 수치는 각 CURRENT/EVIDENCE 문서에 보존하며 여기로 다시
복사하지 않는다.

## 현재 production 기준

- Detector: `B3-BF0-F40 E25`, threshold Vehicle/Pedestrian/Obstacle
  `0.40/0.15/0.10`.
- Tracker: CTRV, position measurement noise `0.40 m`.
- E25는 7D detector이므로 detector velocity fusion은 기본 `off`.
- Prediction: Lanelet Map-CV, 현재 3초 horizon.
- Localization production: `/Competition_topic`, heading correction `-1.347 deg`,
  sensor TF 중복 발행 금지. Planning/TF는 Global state, controller는 Smooth
  `/localization/control_kinematic_state`를 사용한다. MRP는 `localization_mrp`.

실행 계약은
[`bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`](bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md)와
[`localization/README.md`](localization/README.md)를 우선한다.

## P0. 대회 측위 종·횡 안정화와 제어 채터링 제거

- 현장점검에서 fusion localization의 steering-rate absolute p95가 약 `1.59 rad/s`,
  GT localization은 약 `0.34 rad/s`였다. 시각화 문제가 아니라 제어 안정성 문제다.
- 정상 구간 횡 채터링은 약 `14.8/18.5 cm`로 양자화된 GNSS update를 Global EKF가
  강하게 따라가고 controller가 Global state를 직접 소비하는 구조가 주원인이다.
- 로컬 한-PC full-stack의 velocity 과적분은 production 속도 bias로 취급하지 않는다.
  현장 bag의 이동거리/속도 적분 비율은 정상이며, 로컬 MORAI 실시간율과 receipt-time
  artifact를 분리해야 한다.
- 경량 GT-object 폐루프에서 production Q/R은 localization `50 Hz`, 횡 frame-jump
  p95 약 `1.31 cm`, steering-rate p95 약 `0.54 rad/s`를 유지했다. 이 test bed 확보는
  완료됐고 `sigma=0.05, Qxy=0.05` preset은 기각한다.
- Global/Local state 분리와 Smooth controller 입력은 경량 전체 경로 폐루프 A/B를
  통과해 production에 반영했다. Quantization-aware 후보는 동일 기록 controller
  A/B에서 Smooth보다 우위가 없었고, 폐루프 승격 전이므로 기본 OFF다.
- 공식 START 고정 위치 + 보정된 Competition heading 초기화는 MR 로컬 대조를
  통과했다. 대회 LAN 출발·GPS 음영 복귀·checkpoint 순간이동 검증은 남아 있다.
  공식 START는 그때까지 opt-in이며 개발 기본 초기 위치는 `/Ego_topic`이다.

현재 production 계약과 개선 순서:
[`localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md`](localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md).
누적 문제 근거:
[`localization/260918_competition_localization_critical_issue_status.md`](localization/260918_competition_localization_critical_issue_status.md).
최신 폐루프 근거:
[`localization/260920_lightweight_closed_loop_evidence.md`](localization/260920_lightweight_closed_loop_evidence.md).

## P1. 근접 장애물·원거리 차량의 빠른 인지 전달

- **2026-10-09 해상도와 최적화 후속 합의:** [13Hz 예산과 입력 선택](bevfusion/261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md#수집-종료-후-해상도-선택-기준).
  Sunny13 종료 후 crop/투영 → cache/분리 pooling → 동일조건4090 비용 → 세로×가로 학습입력 확정이다.
  전처리 포함 평균50~60ms는 탐색 목표이며 p95·밀집 장면·입력부터 객체 발행까지의 지연을 확인한다.
  448×896/512×1056/576×1152 시간은 미측정 최적화 가정이다. TensorRT 성공에 기대어 입력을 키우지 않는다.
  부분TensorRT는 후속 선택이며 전체전환/원격MORAI 때문에 본학습을 미루지 않는다. 현재수집과runtime은 유지한다.

- **2026-10-09 최신 데이터 수집 합의:** [Sunny13 완료 후 Ego 다양화와 나머지25K](../dataset/morai_od/docs/261009_base5_remaining_environments_spatial_diversity_plan.md).
  현재Sunny13 5K는 끝까지 완료·보존한다. 종료/Park 뒤 구성표·primary 할당을 유지한 연속Ego
  샘플링과 환경별 새배치 생성 경로를 구현·검증한다. 나머지5환경은 새5K씩 수집하며
  총30K 후 Sunny13 재수집 여부를 판단한다. 아래 동일geometry6환경은 과거계획이다.
  기존 다른환경 복사manifest는 새검증 전 실행하지 않는다. 현재수집·production은 변경하지 않는다.

- **2026-10-09 최신 실행 순서:** [Sunny13 종료 후 묶음 실험 합의](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#sunny13-수집-종료-후-묶음-실험-합의).
  이번5K 수집과 MORAI 종료를 먼저 확인하고 사용자 실행 요청 후 진행한다. 무결성/split →
  입력/crop → cache/분리 pooling → 비용/짧은 학습 → 초기화/smoke/resume 순서다.
  추천 구성 하나와 fallback 하나를 본학습 직전에 보고한다. 현재 수집은 유지하며 GPU 실험은
  종료 후로 미룬다. TensorRT/v3 성공을 본학습 필수조건으로 만들지 않는다.

- **2026-10-09 원격 학습 준비:** [Vast.ai 단일4090 임대·5K 학습/추론 계획](bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md).
  로컬 검증 후 원격4090에서 최종 입력의 latency/VRAM·짧은 학습을 다시 확인한다.
  임대 전 전달 용량·고정 디스크·registry image와 direct runner를 준비하고,
  외부 checkpoint 백업/재개를 검증한다. 반복 mission bag은 별도dev-val로 유지한다.
  아직 임대·전송·원격 실행은 하지 않았으며 장기 학습 전 설정·시간·비용을 보고한다.

- **2026-10-09 CUDA-BEVFusion 검토:** [기존 CUDA 사용·v2/v3·배포 호환 조사](bevfusion/261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md).
  학습 체계는 유지하고 cache/분리 pooling을 우선 이식한다. CUDA-BEVFusion depth-outer는
  추론 후보이며 NVIDIA v3와 혼칭하지 않는다. 부분 TensorRT는 잔여 병목 실측 후,
  전체 저장소 이주는 후순위다. 조사만 완료했으며 수집·production·가중치는 변경하지 않았다.

- **2026-10-09 후속 실행 계획:** [Base5 초기화·연산 최적화](bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md).
  다음은 고정 기하/인덱스 cache의 CPU 기준·무효화 검사, 제한된 CUDA 연산 단위 시험이다.
  이후 기존 pooling/cache A/v2 방식 B를 분리 검증한다. 수집·production은 유지하고 전체 모델/학습/
  공식 비용 시험은 별도4090 또는 종료 후다. 아직 최적화 구현이나 CUDA 시험을 한 것은 아니다.
  Base5-only 데이터와 랜덤 초기화를 혼동하지 않는다. E25 fresh는 nuScenes 호환 초기화였고,
  Base5 후보는 현재load_from=None이다. 호환 pretrained + 새4-class/depth/grid 정책을 추천하며
  adapter 로딩 감사 후 본학습한다. crop/GT 재선별과 checkpoint resume/외부 업로드 검증도 계획에 포함한다.

- **2026-10-09 다음 작업:** 수집을 유지한 채 [카메라 crop 크기와 위치 조사](bevfusion/261009_BASE5_CAMERA_CROP_INVESTIGATION.md)를 따른다.
  측방 잘림 snapshot과 12개 조사 항목을 기준으로 픽셀 밀도, 원본 시야, 카메라별 crop,
  GT visibility와 투영 정합을 먼저 검토한다. CPU 분석 후 우선 후보는 448×896 아래 정렬,
  비용 대안은 384×896, 원거리 밀도 확대 대안은 512×1056이다. 최종 설정은 미확정이며
  다음은 학습·Docker·ROS 전처리 정합과 새 입력의 visibility 재구성 검증이다.
  이후 [4090 비용 시험 계획](bevfusion/261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md)에 후보를 반영해
  팀원용 딸깍 번들을 준비한다. 새 번들은 미구현, 풀스택 측정은 보류이며 본학습 여러 회는 전제하지 않는다.

- **2026-10-09 합의:** [학습split·가시성·temporal 경계](../dataset/morai_od/docs/261009_base5_training_split_visibility_and_temporal_decisions.md).
  현재6환경30K수집을유지하고완료후필요조건만보강한다. 합성train약85/val약15·동일geometry환경split고정,
  기존반복bag은dev-val/새독립주행은test다. BabyStroller3 camera대응모호성을실제비가시로단정하지않는다.
  현재single-frame Base5·OD velocity OFF이며temporal코드는연구용보존이지자동실행이아니다.

- **2026-10-09 Sunny13 v132 본5K 수집 중:** 이번 배치는 유지하며 runtime·input·options를 바꾸지 않는다.
  끝난 뒤 반복 준비 비용을 [상주 worker/대기 중첩 A/B](../dataset/morai_od/docs/261009_base5_collection_cost_and_next_experiments.md)로
  검증한다. 전체20% 이상 단축과 GT/안전/resume 유지가 승격 조건이며 아직 미구현이다.
  음영 동적24장 독립 지면 준비는 유지한다. 아래 미시작/승인 대기는 당시 준비 이력이다.

- **Base5 현재는 v132**이다. [최종 외관·배치·12카테고리/검증](../dataset/morai_od/docs/261009_base5_final_appearance_v132.md),
  [CURRENT](../dataset/morai_od/docs/BASE5_V2_CURRENT.md), [실행 런북](../dataset/morai_od/docs/261007_base5_5k_capture_runbook.md)을 따른다.
  root `final_appearance_5k_v132_release`, Sunny13 5K 수집 중→같은6환경30K.
  자유yaw·static parent 형상·음영 접근/이탈 균형만 보완했고 production은 그대로다.
  구수집763승인과v129/v130 raw를보존했다. 아래v123/v125/v129 경로/숫자는당시이력이다.
  수집 후 실제 가시 자산 coverage·새학습/held-out mission rosbag 검증이 남는다.

- 최신[MORAI 답변](../dataset/morai_od/docs/260920_questions_for_morai.md#최신-추가-답변--객체-범위g80차량-속도-기준)에서
  F2 전체 객체가 미션 후보이며 정적 자산의 이동 예외는 없다고 확인했다.
  열린G80은 공식 출제 제외이며 코드에도 반영했다. 부유 재현 ZIP 전달은 완료됐고
  MORAI 후속 답변 및 나무/불/도로시설 등 기존background/ignore coverage 검토가 남는다.
  Animal/복합 GT 후보는 유지한다.
  UDP차량velocity 후륜 기준은 확정됐으나 기존R01의변환잔차 검증/새평가 계약 선택은 남는다.

- 독립 Base5의 현재 목표는 **Sunny13시 직접5K**다. 구10K/v45 명령과 시간 추정은
  현재 실행 기준이 아니다. [CURRENT](../dataset/morai_od/docs/BASE5_V2_CURRENT.md) →
  [5K 계획](../dataset/morai_od/docs/261007_base5_direct_5k_current_plan.md) →
  [단일 운영 런북](../dataset/morai_od/docs/261007_base5_5k_capture_runbook.md) 순으로 읽는다.
  범용 크기 예산·초근접·고주로/로터리 밀집·실제 GT visibility·희귀 자산 coverage를
  [최종 gate 근거](../dataset/morai_od/docs/261007_base5_final_gates_v105_evidence.md)에 묶었다.
  본수집은 최종 보고 후 사용자 승인까지 대기한다. 이전 v23–v100 시행착오·실측은
  dataset evidence 문서에 보존하며 ACTIVE에 중복 나열하지 않는다.
  현재 생성본은 `regions_5k_v123_release`다. 고주로740/600·로터리0/350·기타차량80/50·장애물770/600·
  사람830/250·독립음영500·배경230이며 고주로/로터리의 모든 actor는 일반차량만 넣는다.
  장애물 보조 노출을 다른 허용 혼합 구역에서 보충하고 거리/근접/희귀 quota를 유지한다.
  sunny/foggy×11/13/15 같은5K의30K관측으로 준비하며 환경별group/split누수를 방지한다.
  [구역별 구현·검증](../dataset/morai_od/docs/261008_base5_region_release_evidence.md)이 최신이다.
  음영500geometry/178이동(사거리154+음영24), Park와두점 높이 계약은 이전 검증을 유지하고
  구 output에 새 runtime으로 resume하지 않는다. 통신 halt는 안전 확인 후 새 output으로 분리한다.
  Ego Park·독립 지형·CollisionData·실제 Z/가시성 검사는 유지하며 raw GT는 사후 수정하지 않는다.
  [5060Ti 비용 근거](bevfusion/261006_BASE5_BATCH1_CAPACITY_EVIDENCE.md)는 그대로다.
  4090/확대후보ROS/full-stack은 아직 별도 검증이며 수집 pilot으로 인증하지 않는다.

- Base5 E2 학습 및 [E1 보행자·장애물9-bag A/B](bevfusion/261005_BASE5_E1_MISSION_ROSBAG_AB.md)는 완료됐다. E1 장애물 소폭 개선에도 0–2m 장애물·20–30m 보행자 공백은 미해결이다. 다음은 해당 거리·부분 시야 표적 보강 + replay다. Production E25 유지, Far ROI와 고해상도는 별도 A/B로 유지한다.
- 전방은 빠른 존재 전달을, 후방/로터리는 위치·속도·궤적 안정성을 함께 평가한다.
- 현재 E25의 근접 장애물 누락은 tracker만으로 해결하지 않는다. 부분 시야·얇은
  방향·통과 중 raw/GT를 보강하고 동일 held-out bag에서 재학습 후보를 비교한다.
- 원거리 검증은 40/60/80/100m 가시성부터 시작한다. 현 ±30m config와 향후 모델의
  능력을 구분하며, ROI/학습/평가 범위를 함께 설계한다.
- 고해상도 320×896은 5060 Ti batch1 약10–11FPS로측정했다. 정확도·4090 속도는 미측정이다.
- 전후방 ±60m/좌우 ±30m ROI 후보는 축소 입력 forward와 checkpoint 기하 복원 검증을
  통과했다. 확장 GT 재변환·full-input 학습·출력/평가 필터 연계는 아직 미완료다.
- confirmed UNKNOWN→차량 빠른 승격은 opt-in 후보이며 기본 OFF다. 신규 track
  확인을 무조건 생략하거나 검출 없는 객체를 차량으로 만드는 기능은 아니다.
- Production E25 교체는 Detection/Tracking/Map-CV 동일 bag 회귀 검증 후 결정한다.

최신 근거·실행 경계:
[`bevfusion/261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md`](bevfusion/261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md).

## 후속 연구. Base4/Q1 velocity·temporal

- 목적은 100 km/h 고주로 추월에 필요한 Vehicle velocity와 1/2/3초 미래경로를
  개선하는 것이다.
- 과거 개발 bag의 Position-only/GT oracle 숫자를 현재 production 성능으로 재사용하지
  않는다. 최신 팀원 bag의 보정 GT 비교는 BEVFusion index의 2026-10-04 EVIDENCE를 따른다.
- 이미 수행한 `B4T-HWY-P0` 및 Q1 이력을 참고하되, 새 근거 없이 초기 pilot을 자동 재개하지 않는다.
- 새 데이터는 실제 LiDAR cadence의 연속 sequence로 수집한다. LiDAR timestamp에
  Ego/Object pose를 보간하고 학습 GT는
  `velocity_target_ego_mps=[vx,vy]` 하나만 노출한다.
- 과거 sweep은 Ego motion만 현재 frame에 정합한다. 추론에 존재하지 않는 GT Object
  motion compensation은 input에 사용하지 않는다.
- single-frame, 3-sweep, dense temporal BEV, query memory는 이미 실험 이력이 있다.
  다음 architecture는 Q1 결과와 새 bag의 실패 분해를 보고 선택한다.
- Pilot dataset은 sequence 단위 train/val만 사용한다. 기존 고주로 합류·고주로1
  bag은 반복 사용된 dev 평가이며 최종 test가 아니다. 최종 후보는 새 held-out 고주로
  bag 최소 2개에서 한 번 평가한다.
- 각 모델은 tracker velocity `off/on`을 모두 평가한다. 첫 velocity covariance는
  `0.5 m/s`이며 geometry, association, 2D AVE와 Map-CV FDE를 함께 본다.

실험 이력:
[`bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md`](bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md).

## P2. Localization과 velocity pilot 이후 Prediction 개선

현재 구현값과 미구현 경계는
[`../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md`](../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를
따른다.

- 기본 prediction horizon을 5초로 확장하되 로터리에서는 3--4초로 줄이는
  map-zone-aware 정책을 검토한다.
- 고주로 추월을 방해하는 다중 후보 문제는 단순 `top_k=1` 강제가 아니라 tracker
  history, Lanelet 연결성, 횡위치와 차선 변경 가능성을 결합한 temporal lane
  association으로 Top-1 정확도를 먼저 높인다.
- 같은 차량의 인접 차선 점유 판단이 일정 시간 안정적으로 유지될 때만 Planning이
  추월 gap으로 사용하도록 track age와 prediction confidence 계약을 정한다.
- Localization과 detector velocity 변경을 동시에 섞지 않는다. P0와 P1의 동일 bag
  회귀 검증이 각각 끝난 뒤 별도 브랜치에서 수행한다.

## 완료되어 자동 재개하지 않는 항목

- Base3-Static 10K 수집, validator 소급 감사와 5-point GT 정책.
- Base3 LRASPP, PointPainting PP0/PP1과 FastBEV 연구.
- Base3 BEVFusion E40 학습과 production E25 선택.
- Base3-Online 고주로 334-scene 수집과 E25 fine-tuning.
- Tracker motion-model/covariance sweep과 2D AVE evaluator 수정.

과거 결과가 필요하면 [`bevfusion/README.md`](bevfusion/README.md),
[`../dataset/morai_od/docs/README.md`](../dataset/morai_od/docs/README.md) 및 각
`legacy/README.md`에서 찾는다. 새 근거 없이 완료 항목을 최우선 계획으로 되돌리지 않는다.
