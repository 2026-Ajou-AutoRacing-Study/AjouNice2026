# AjouNice2026 활성 최우선 계획

마지막 갱신: 2026-10-06.

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

- 최신[MORAI 답변](../dataset/morai_od/docs/260920_questions_for_morai.md#최신-추가-답변--객체-범위g80차량-속도-기준)에서
  F2 전체 객체가 미션 후보이며 정적 자산의 이동 예외는 없다고 확인했다.
  열린G80은 공식 출제 제외이며 코드에도 반영했다. 부유 재현 ZIP 전달은 완료됐고
  MORAI 후속 답변 및 나무/불/도로시설 등 기존background/ignore coverage 검토가 남는다.
  Animal/복합 GT 후보는 유지한다.
  UDP차량velocity 후륜 기준은 확정됐으나 기존R01의변환잔차 검증/새평가 계약 선택은 남는다.

- 독립 [Base5 v2](../dataset/morai_od/docs/BASE5_V2_CURRENT.md)의10K 오프라인 생성과
  100m 후보 GPU wiring은 이전 근거다. 최신v23은48조건 분산검증·2조건 대체·빠른 동작/
  all-actor motion gate·lidar1_camera0 정책·CPU 변환 검사 완료다.
  v25 yaw 실측은 유효42/42조건: ±15° 곡선2조건이 횡30m 밖이다.
  정지설정 NPC runtime 이동 표본은 제외/재촬영 gate로 차단했다.
  Park 재확인 후 남은2조건 완료,최종Park 재시도 성공. 간헐 timeout 원인은 미해결이다.
  v32에서 E25 및Base5후보의5060Ti batch1 비용과E25 ROS포화처리량을측정했다.
  근거는[capacity EVIDENCE](bevfusion/261006_BASE5_BATCH1_CAPACITY_EVIDENCE.md)다.
  4090/확대후보ROS/full-stack은미측정이다.
  v26은 속도를 낮추지 않고뒤쪽출발+실제목표도착 capture를Woman2 12조건에서 검증했다.
  이동자세/GT 형상이달라짐을 확인.
  v27은 대표13자산의52조건/80component GT 비교 완료. 일부 object의 이동높이 차이는
  12조건 matched-height 재촬영으로 제거했다. 관절 변화 사람/동물은 동적 phase,
  현재 fixed-pose 복합/rigid 자산은 정지 우선 권장. 미검증 자산에는 자동 일반화하지 않는다.
  v28에서 자산별 외관 정책·자연속도 방식을 일반generator 기본에 통합했다.
  v29에서zero XYZ점의지면오판을수정했고근접·혼합13장재검사는9통과/4review다.
  독립지면촬영/입력Z수정/동일cell재배치자동loop와안전4장변환smoke를구현했다.
  당시종료Park2회timeout으로중단했으나v30사전확인/3세션종료Park통과후소량검증을재개했다.
  v30의60시도/57raw에서작은Z상향여유는부유를해결하지않았다.
  지면기준입력/waypoint보정후24/24통과했으며일괄상향·전역offset은적용하지않는다.
  v31에서 남은 근접·혼합3셀을 마무리했고 quality loop의 bounded campaign/quota 보충/resume을 연결했다.
  소량 stop/resume 및89 tests 통과. 선택ledger CPU변환·MR OFF 속도비교까지완료했다.
  안정된Z도 떠 있을 수 있으며 입력Z 수정 후 개4조건의공중gap은약−3cm로 개선됐다.
  raw GT 사후 보정은 하지 않는다. 자동보완은universal지형높이solver인증이아니다.
  v33 공식정책 반영 후12/12 live·부족0, pause/resume·CPU30GT 변환 통과.
  약96m 전/후방 차량에서camera 근거를 확인했으며20m밖ground/모델정확도 인증은 아니다.
  v35 새 기본10K 전수 생성/audit PASS 및122종 primary 배치 coverage를 완료했다.
  v36 대표32/32실제캡처36회/314.7초,10K 비용외삽27.5시간·원본54.7GiB다.
  [v37 비용분리·최적화](../dataset/morai_od/docs/261006_base5_capture_stage_optimization_v37.md)는
  동일6셀56.6→50.7초/양쪽6통과,32장 감사출력 정확일치와100tests를 확인했다.
  조건부 정지확인 preflight만 단축하고 안전gate는 유지한다. 현코드로 구세션resume 금지.
  [v38 ROS/network 비교](../dataset/morai_od/docs/261006_base5_ros_batch_network_evidence_v38.md):
  ROS공유6/6통과·13.85%단축. network 재사용은2번째로드에서client0/종료Park미확인으로중단했다.
  옵션금지·scene별network reload유지. 당시재연결/사전Park 전 추가로드를중단했다.
  [v39 안전batch/quota](../dataset/morai_od/docs/261006_base5_safe_batch_quota_v39.md)는
  재연결후 새6장resume·첫12셀보완/CPU17GT 변환을통과했다.
  후속합계20/32quota에서최종Park2회timeout으로중단했다. 실패출력resume금지,
  남은12셀/반복Park서비스지연/장시간안정성 검증이남는다. max64셀pilot이며 본10K미시작이다.
  후속[v40 EventCmd/rosbridge](../dataset/morai_od/docs/261006_base5_eventcmd_rosbridge_v40.md)에서
  서비스재등록ID/공유response혼선을설치코드로재현했다. opt-in guard offline통과/live미검증.
  추가8quota로28/32통과,남은4셀은3raw보존/1미로드다. 기존bridge재시작/MR재연결이먼저다.
  후속[v41 guard live/중복촬영 제거](../dataset/morai_od/docs/261006_base5_guard_seed_recovery_v41.md):
  재연결후 남은4셀보완PASS로 대표32/32. 동일4셀53.93→41.95초/실제6→5촬영,
  123tests·완료resume·CPU4장7GT PASS. 전환오류2회retry회복·timeout0;
  장시간 인증이나전체10K22%절감은 아니다. 최신진입점은BASE5_V2_CURRENT를 따른다.
  실제 전체pool/장시간 캡처 검증과4-class runtime 결정은 남는다.
  구왕복10K는 새 기본 수집에 사용하지 않는다. 본 수집/본 학습은 미실행이다.

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
