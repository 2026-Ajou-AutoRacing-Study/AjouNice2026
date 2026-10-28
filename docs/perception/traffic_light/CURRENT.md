# 신호등 인지 CURRENT

상태: 2026-10-28. **v5 지도+CNN·준수 가드·직접 관측·route 보호 ON, 허가 뒤 순수 UNKNOWN 유예0.3초, 황색 판단선 ON·투표·디버그 화면 OFF**다.
환경 실행은 [패키지 README](../../../base_ws/src/perception/traffic_light_detection/README.md)를 따른다. 과거 문서의 개선 제안이나 연구 가중치를 운영 기본값으로 읽지 않는다.

## 운영 기준

- `traffic_light_node.launch`: front JPEG 1280×720, YOLO housing 1클래스,
  confidence 0.25, 최대 처리 10Hz. 운영은 직접 관측 전달 + 판단의 신선한 허가 관측 1개 확인이다.
- 기본은 YOLO26n960 + 지도 연결 + CNN v5, `~/.local/share/ajounice/traffic_light_gpu/bin/python`이다.
  고정 모델 SHA·경로는 [배포 계약](../../../configs/perception/traffic_light_v5_candidate_release.json)을 따른다.
- 운영은 CNN 4-bit 관측 보존이며, 색·화살표를 추가하는 보정은 명시적 legacy에만 남는다.
  `debug_observed_label`은 보정 전, `debug_raw_label`은 이름과 달리 보정 후다.
- 운영 A/B/C/D의 미완충 판독 불가·모순 조합은 UNKNOWN이다. 같은 정상 지도 head의 저확신/무점등은 `no_lamps_hold_sec=0.2`의 공통 기한으로 짧게 완충한다(정차 중 R·황색 포함·새 정상 색 즉시 전달·원본 age0.3초 상한·황색 잔여시간 연장 없음). [no_lamps 비교·실제 소등 유지 위험](261027_NO_LAMPS_BOUNDED_HOLD.md)을 본다. D-1은 passthrough이고
  판단 모듈에서 제외된다. 별도 planning competition fail-open과 혼동하지 않는다.
- native 신호 조회·제어 및 `/Ego_topic`은 **개발 평가용**이다. 운영 후보의 지도 연결 입력은 localization/TF이며 native API·Ego를 사용하지 않는다.
  개발 수집·평가 도구를 대회 알고리즘의 의존성으로 연결하지 않았다.

## 현재 v5 기본과 명시적 복귀

- `selection_mode:=map_association`: route regulatory ID에 속한 실제 housing을 지도 투영·YOLO 박스로 연결한다. 옛 D-1은 미션에서 제외한다.
- 지도 검색은 **yaw4°·수평120px·세로200px**다. 이전3°·90px보다 같은 callback509개에서 현재 램프6개를 회복했고 기존 출력 악화는 없었다. 상단 ROI·최고 confidence만으로 타 신호를 미션으로 지정하지 않는다. [허용 범위 대조·검증 한계](261027_SELECTION_TOLERANCE_COMPARISON.md)를 본다.
- `classifier_mode:=cnn`: RGB letterbox224×128의 ResNet18 4비트 분류기를 사용한다.
  현재 기본이다. 잘못된/미설치 CNN을 조용히 HSV로 바꾸지 않는다.
- 최종 후보는 YOLO26n 960 + 지도 선택 + ResNet18 CNN v5다. YOLO 자료는
  6환경 816장/1,374박스, CNN은 기존792개를 보존하고 작은 D 신호60개를 보강했다.
- 고정 자연 현시의 같은939장에서는 기존 HSV856개 → 새 조합939개가 일치했고(후속 이동 A의 작은·어두운 head에는 별도 [황색 과신](261028_YELLOW_REPEAT_ENVIRONMENT_VALIDATION.md)이 있어 일반화100%가 아니다),
  좌화살표는41/116 →116/116이었다. native 상태 조건부 일치이며 주행100% 인증이 아니다.
- v4의 작은 D 좌화살표 누락은 임계값을 낮추지 않고 자료를 보강해 v5로 개선했다.
  새 pose·6환경60개는60/60 일치, 실제 ROS 저확신 UNKNOWN0·지도 연결346/349였다.
  나머지3개는 측위 불확실성 초과다. callback p95 37.65ms이며 LAN·planner 승격은 별도다.
- 후보의 학습·score 문제·실ROS 입력·연속 평가·제약은
  [통합 검증 기록](261025_YOLO_CNN_MAP_RUNTIME_EVALUATION.md)을 본다.
  모델 파일을 바꾼 것과 운영 기본값을 바꾼 것은 구별한다.
- 이전 yellow/R+Y/unknown 계약은 [정책 통합 대조](261025_SIGNAL_POLICY_INTEGRATION_AUDIT.md)의 당시 기록이다.
  선 전 정차의 Y/R+Y 새 출발과 완충되지 않은 최신 UNKNOWN은 정지 판단이다. 사용자·판단팀 요청으로 기존
  `competition_fail_open=true`의 미수신·timeout GO를 복구했다. 신호등 미실행과 노드 다운은 이 정책에서 구분되지 않는다.
  [황색 스위치·샘플 시간·GUI](261025_DEPARTURE_AND_SIGNAL_LOSS_PROPOSAL.md#yellow-controls-261026)의 당시 OFF 기본·이전 ON 시험은 역사 기록이며 현재는 사용자 요청으로 ON이다.
- 팀 설치·GUI 선택·복귀는 [v5 시험 배포](V5_CANDIDATE_HANDOFF.md). 모델 압축본 한 파일만 공개 읽기이며 데이터·부모 백업은 비공개다.
- 황색 master는 사용자 요청으로 기본 ON이다. ON+guard ON은 [판단선1회 확정v2](261027_LANELET_YELLOW_BRAKING.md): b6·J10·응답지연0의jerk 제동거리, 방향허가 상태로 `d<=D`에 진입하면 GO확정, 후속Y/R에도 유지하되 실제정차시 재판단한다(적색미위반 보장아님). STOP이력은 가상선 전/후 정차·UNKNOWN으로 풀리지 않고 신선한 방향허가로 해제한다. OFF도STOP오버슈트 보호는 유지하며, 미수신/1초timeout GO는 기존예외다. C는201290 하나·원본선 통과후 종료, 추가 콘솔 INFO는 원복했고 영상 하단만 한글 요약한다(영상용 읽기전용 판단 토픽). Cloudy13 실제run02 완주·P 확인, D목표12cm초과STOP→좌회전허가 재출발 확인. 예전v1/시간가정과 [기존 반복검증](261028_YELLOW_REPEAT_ENVIRONMENT_VALIDATION.md)은v2검증과 구분한다. [추가 실주행·제동 한계](261028_DECISION_LINE_PR_VALIDATION.md): 황색 STOP 계산19.55m 대비 실제적분23.25m로8/16/0의 정지 보장은 미승인이다. 후속 통합후 실제36/56km/h 시험은 계산11.18/24.80m 대비 적분10.86/25.16m, 목표−0.16/+0.43m 정차였다. 계산 기본만6/10으로 통일했고 제어 제한은 유지한다. 이는 시험한 race profile용이며 normal2.8/5·LAN 부하는 미검증이다([보정 근거·중간 속도 준비 실패](261028_DECISION_LINE_PR_VALIDATION.md#calibration-261028)). 최신 planning600125b1 통합·v98 자산 검증 후 PR #60으로 배포한다. 명시적 OFF와 동결 snapshot은 보존한다.
- 정차 위치는 Lanelet/`traffic_light.stop_margin: 0.0`이다. **v93 좌표(A+0.75/B+2.6/C+0.35/D+2.6m)**를 팀장님v88-2 도로에 합친 [v94](../../../configs/map/kcity_v94_merged_asset.yaml) 이후 지도 변경은 [지도 CURRENT](../../../kcity_map/README.md)를 따른다. 현재 소스는 v99이고 팀의 새 v99 LAN 기록에서 A/B/C 제동을 확인했다([10/28 검토](../../bevfusion/evidence/261028_TEAM_LAN_AND_DUPLICATE_TRACK_REVIEW.md)). 과거 v97 override 영상과 v99 기록을 구분한다. 계산은6/10/응답지연0을 유지하며 `system_delay=0.5`를 황색 계산의 지연으로 섞지 않는다. 이번 검토에서 지도·제동계수·거리 게이트·제어기는 변경하지 않았다.
- 단발 detector 누락은 원본 age 0.2초, 정상 지도 연결의 CNN 공백은 위 공유0.2초가 우선이며, 그 밖의 기존 이동 중 soft 저확신 경로만 0.3초/원본 age0.5초 한도를 유지한다. 짧은 지도 불확실성 증가에는 최근 동일 head의 **현재 YOLO/CNN 재판독**을 최대0.3초 허용한다(`visual_continuation_sec=0` 복귀). 로컬 소스·설치본에는 이전 박스 IoU가 낮아도 현재 영상 시각의 크기 비례 지도 투영과 현재 검출 박스가 일치하면 같은 head로 재판독하는 보완을 추가·검사했다. 로컬 B 근접·풀경로 폐루프는 PASS했으나 **새 투영 분기는 두 주행에서 발동하지 않아 팀 PC 설치본·LAN 효과 검증은 별도**다. 위 no_lamps 완충을 벗어난 무점등과 실제 화각 이탈은 UNKNOWN으로 남기며, 다음 lanelet ID만으로 다른 YOLO 박스를 강제 연결하는 규칙은 추가하지 않는다. [B bag 원인·후보·실주행 검증](261027_B_ROI_PROJECTION_CONTINUATION.md)을 본다.
- Planning 기본 `defer_unseen_until_observed:=true`: 이번 접근에서 색 근거를 한 번도 읽지 못한 신선한 UNKNOWN에는 정지점을 넣지 않는다. **가까워져도 거리 경계는 없다.** 알려진 색이 하나라도 보이지만 조합·형상이 모호하면 STOP하고 이후 누락에도 이력을 유지한다. 첫 적색·황색은 즉시 STOP, 허가 신호는 현재 기본인 신선한 1개 관측 후 GO다. 첫 색 근거를 읽은 뒤의 UNKNOWN은 기존 완충 한도를 벗어나면 STOP이며, timeout·미수신은 별도 `competition_fail_open` 계약을 따른다. 이는 신호를 녹색으로 재분류하는 기능이 아니다. 대회 시뮬레이션의 DNF 위험을 줄이기 위한 선택이며, 첫 판독 자체가 끝내 없으면 적색을 통과할 수 있는 위험이 남는다. 보수적 복귀는 `defer_unseen_until_observed:=false`, 기존 거리 경계 후보는 `defer_unseen_until_observed:=false defer_far_unseen_unknown:=true`로 선택한다. A/B/C/D 자연 풀주행 2회에서 첫 신호가 일찍 읽혀 거리 경계 효과는 분리 검증되지 않았고, 공식 LAN 판정도 별도다.
- D 근접 누락은 수직 상한200px·조건부640 재검출로 보완했고 Sunny13·Foggy13 무객체 실제 폐루프는 PASS했다.
  [근접 보완·검증 범위](261025_NEAR_SIGNAL_RECOVERY.md) 참고. `traffic_legacy`/GUI legacy는 명시적 복귀다.

## 구간 계약과 자연 현시

아래는 팀원 최신 v1.1 관찰 기록과 현재 설정을 대조한 계약이다.
실측 수치·옛 제안과의 차이는 [대조 기록](261025_TEAM_NOTES_AND_ROBUSTNESS_AUDIT.md)을 본다.

| 구간 / regulatory ID | 대회 경로 | 관찰된 자연 점등 조합 |
|---|---|---|
| A / 201351 | 직진 | R / Y / G |
| B / 201365 | 직진 | R / Y / G |
| C / 201290 | 직진 | R / Y / G+좌화살표 |
| D-2 / 201326 | 좌회전 | G / Y / R+좌화살표 / R+Y / R |
| D-1 / 201397 | 무정차 우회전 | 판단 대상에서 제외, 관측은 별도 |

특히 D-2에서 G와 Y 단독은 좌회전 신호가 아니다.
R+좌화살표 및 R+Y와 구분해야 한다. 이전 phase만으로 Y의 의미를 복원하지 않는다.
위 표는 관측된 자연 프로그램이며 CNN 입력 GT가 아니다. D의 Y도 R로 바꾸지 않고 비허가로 판단한다.
허가 없는 Y/R+Y 새 출발은 금지다. 황색 ON의6m/s²·10m/s³는 의사결정용 근사일 뿐 실측 제동 성능·적색 전 통과·무패널티를 보증하지 않는다.
사용자가 재확인한 미션 명칭은 A·B·C(두 physical head)·D(옛 D-2)다.
동적 교차로 뒤에 보이는 옛 D-1은 검출 라벨로 존재해도 미션 신호 허용 대상으로 쓰지 않는다.

## 확인된 문제와 후보
- 2026-10-28 사용자 선택: 지도+직접 관측의 판단 시작 기본은 **A/B/C40m·D30m**(`observation_start_depth_m`/`observation_start_depth_d_m`), 카메라→신호 깊이다. 시작 후 유지·추가 완충 없음, 전체 복귀는 앞 인자0. legacy는 비활성. [기존 로그 감사](261028_EXISTING_LOG_WITHIN40_AUDIT.md)와 [모두40m였던 단일 주행](261028_DISTANCE40_DELAY0_TRIAL.md)을 구분하며 기본40/30은run02와후속 전체주행에서 확인했고, 별도 B제동시험은 ABC60m override를 썼다, 같은 미션 teleport 초기화도 미지원이다.
1. C016/C018은 같은 신호 프로그램이다. 단일 등 강제 제어의 불일치를 자연 주행 오류로 세지 않는다.
2. 지도 crop은 근사치다. native 상태가 맞아도 선택 연속성을 보장하지 않는다. [과거 연속 제동 대조](261026_TIMED_YELLOW_CANDIDATE.md#continuous-braking-comparison)에서 v90 A/B/D의 허가 판독 중단·짧은 재정지를 확인했다. CPU·화각 하나로 단정하지 않는다.
3. [3/2 영상](261027_TEMPORAL_DRIVING_VIDEO_COMPARISON.md)·[7/4 비교](261027_SEVEN_FOUR_DRIVING_COMPARISON.md)·[옛 전체 경로 감사](261027_LEGACY_ROBUSTNESS_AUDIT.md) 후 [순수 UNKNOWN 유예](261027_UNKNOWN_GRACE_DRIVING_COMPARISON.md)를 **운영0.3초로 승격**했다. 실제 허가 원본/처리 age가 모두 기한 내일 때만 이어주며 새 R/Y·색이 있는 모순은 즉시 기존 판단에 전달한다. 당시 marker ON·허가1회·투표0·황색OFF를 묶었고 현재 황색만 별도로 ON으로 전환했다. traffic OFF/legacy/옛 snapshot은 유예0을 보존한다. [배포·복귀](261027_UNKNOWN_GRACE_RELEASE.md)를 따른다. 과거 profile·D1.5초 hold·UNKNOWN/Y 무조건 GO는 복원하지 않았다.
4. `route_progress_guard`는 기본 ON이다. 지난 미션 재선택을 억제하고 route/큰 재배치/시간 역전 시 초기화한다. timeout은 원본 stamp 기준 1.0초다. 명시적 황색 OFF의 GO_OUT은 지도선 기준 유지, 황색 ON의 제동 가능성은 Lanelet 목표 기준·통과 완료는 원본 MGeo선/앞차축3m 기준으로 분리한다.

- [움찔 실주행 수정](261026_SIGNAL_JITTER_RUNTIME_REMEDIATION.md): 신호등 입력 covariance stress의 실제 재제동1→0 확인 후 **launch 기본0.3초로 적용**. 새 v94 bag B의 STOP5회 근방에서 같은 영상의 현재 녹색 판독 회복을 확인했지만 인과·완치 인증은 아니다. 당시에는 timeout1초·황색OFF·허가3회·ID0 경계를 유지했으며, 허가 확인만 2026-10-27에 별도 1회로 변경했다. 인식 모델 성능은 그대로다. 자연 풀주행 2회의 A/B/C/D 관측열 오프라인 대조에서 안정 허가 구간의 첫 관측→세 번째 관측 간격은 0.209~0.260초였다. 총190개 허가 프레임 중 전후0.25초의 동일 native 상태로 확인 가능한184개는 모두 방향 허가였고, 6개는 그 조건으로 미확인이다. B에는 한 프레임만 허가로 나온 구간 3개가 있었고 주변 native 조회는 녹색이었다. 1회 설정에서는 이런 구간에 궤적이 잠깐 열렸다 닫힐 수 있다. **이190프레임 대조 자체는1회 설정의 폐루프 평가가 아니며**, 오출발률이나 실대회 패널티 변화를 인증하지 않는다. 이후 로컬1회 설정 주행은 위 [허용 범위 대조](261027_SELECTION_TOLERANCE_COMPARISON.md)에 기록했다. 기존 실행 snapshot에서 허가3회를 명시했다면 그대로 유지된다.

## 평가·학습 경계

- Sunny/Foggy × 11/13/15시를 구분하고 환경 API Set 성공 및 전후 Get 값을 기록한다.
  공식 날씨 GUI의 설정 직후 조회는 아직 이전 상태일 수 있어 단일 표시만 믿지 않는다.
- 각 JPEG 앞뒤의 exact native ID·상태를 확인한다. 요청값만으로 GT를 만들지 않는다.
  실제 렌더링·가시성·올바른 대상·crop 포함 여부는 별도 확인한다.
- 자연 현시와 강제로 만든 비자연 현시 stress를 분리한다.
  예를 들어 C의 G 단독 시험은 자연 주행 성능 통계에 섞지 않는다.
- C의 두 동기 신호 중 하나만 강제 변경한 사진은 그 housing 분류 시험용이다.
  전체 화면의 신호 일치·자연 주행 end-to-end 정답으로 쓰지 않는다.
- 동일 pose의 환경·phase·재시도·연속 프레임은 같은 split에 둔다.
  이미 관찰한 test 결과는 이후 튜닝에서 개발 자료로 취급하고 새 시험 시점을 분리한다.
- R/Y/G/좌화살표는 동시점등을 허용하는 multi-label이다.
  가려짐·판독 불가를 소등 또는 음성 라벨로 강제하지 않는다.
- 개발용 투영은 raw Ego heading에서 −1.347°를 한 번 적용한 MGeo 좌표 기준이다.
  이미 보정된 운영 localization/TF에 이 보정을 다시 적용하면 안 된다.
- 새 팀 LAN 기록은 A/B/C/D 정상 색 관측 구간 내부 누락0·A/B/C의6/10 제동 근거를 추가했지만 이동 중 G→Y나 공식 신호 준수100% 인증은 아니다.
  첫 색 이전 UNKNOWN에 순간 음수 속도로 새 STOP 이력이 생기던 결함을 수정했다([조건·182 native/445 Python](../../bevfusion/evidence/261028_TEAM_LAN_AND_DUPLICATE_TRACK_REVIEW.md)). 기존 STOP/R/Y 보호는 유지한다. 수정본은 커밋434083b4와 로컬 정상 증분 설치에 반영됐고, 실제 plugin SHA는 `fe849829e3e23794f689c8667274cd2cb2c06c31222c3ebcdc1e4ca3c4999833`이다. 이후 로컬 E12·traffic ON 전체경로 2회는 정지/P까지 완료했고 관측된 허가 후 재STOP은0, 원본선 통과8회는 native 허가와 일치했다. A의 먼 황색 오판은 거리gate에서 차단됐으며 이동 중 허가G→Y·새 팀LAN은 미검증이다. 로컬 속도×wall시간 적분과 native 위치 이동의 불일치가 있어 이 적분으로 제동계수를 튜닝하지 않는다. 상세는 로컬 `planner_logs/analysis/261028_final_e12_traffic/`에 보존하며 현재6/10·응답지연0은 유지한다.
- 신호를 처음 본 시점은 점등 시작 시점이 아니다. 고정 2.5초 카운트로 황색 잔여 시간을
  보장하지 않으며, 팀 대화의 대회 주기 단축 예상도 운영 상수로 쓰지 않는다.

## 자료 위치
- 팀원 원문·영상·SHA: `artifacts/reference/traffic_light_team_notes_261025/`(원본 ZIP 보존).
- 신규 원본 JPEG·native/환경 증거·실패·QC·후보 결과: `artifacts/evaluation/traffic_light_robustness_261025/`.
- 도구: `scripts/evaluation/*traffic_light*.py`. capture는 개발용 정지/P 시험이다.
