# 신호등 인지 CURRENT

상태: 2026-10-26. **v5 지도+CNN·준수 가드·직접 관측·route 보호 ON, 황색 통과·디버그 화면 기본 OFF**다.
환경 실행은 [패키지 README](../../../base_ws/src/perception/traffic_light_detection/README.md)를 따른다.
과거 문서의 개선 제안이나 연구 가중치를 운영 기본값으로 읽지 않는다.

## 운영 기준

- `traffic_light_node.launch`: front JPEG 1280×720, YOLO housing 1클래스,
  confidence 0.25, 최대 처리 10Hz. 운영은 직접 관측 전달 + 판단의 서로 다른 허가 관측 3개 확인이다.
- 기본은 YOLO26n960 + 지도 연결 + CNN v5, `~/.local/share/ajounice/traffic_light_gpu/bin/python`이다.
  고정 모델 SHA·경로는 [배포 계약](../../../configs/perception/traffic_light_v5_candidate_release.json)을 따른다.
- 운영은 CNN 4-bit 관측 보존이며, 색·화살표를 추가하는 보정은 명시적 legacy에만 남는다.
  `debug_observed_label`은 보정 전, `debug_raw_label`은 이름과 달리 보정 후다.
- 운영 A/B/C/D의 판독 불가·모순 조합은 UNKNOWN이며 가짜 적색/녹색을 만들지 않는다. D-1은 passthrough이고
  판단 모듈에서 제외된다. 별도 planning competition fail-open과 혼동하지 않는다.
- native 신호 조회·제어 및 `/Ego_topic`은 **개발 평가용**이다.
  운영 후보의 지도 연결 입력은 localization/TF이며 native API·Ego를 사용하지 않는다.
  개발 수집·평가 도구를 대회 알고리즘의 의존성으로 연결하지 않았다.

## 현재 v5 기본과 명시적 복귀

- `selection_mode:=map_association`: route regulatory ID에 속한 실제 housing만
  지도 투영과 YOLO 박스로 연결하며 현재 기본이다. 옛 D-1은 미션에서 제외한다.
- `classifier_mode:=cnn`: RGB letterbox224×128의 ResNet18 4비트 분류기를 사용한다.
  현재 기본이다. 잘못된/미설치 CNN을 조용히 HSV로 바꾸지 않는다.
- 최종 후보는 YOLO26n 960 + 지도 선택 + ResNet18 CNN v5다. YOLO 자료는
  6환경 816장/1,374박스, CNN은 기존792개를 보존하고 작은 D 신호60개를 보강했다.
- 고정 자연 현시의 같은939장에서는 기존 HSV856개 → 새 조합939개가 일치했고,
  좌화살표는41/116 →116/116이었다. native 상태 조건부 일치이며 주행100% 인증이 아니다.
- v4의 작은 D 좌화살표 누락은 임계값을 낮추지 않고 자료를 보강해 v5로 개선했다.
  새 pose·6환경60개는60/60 일치, 실제 ROS 저확신 UNKNOWN0·지도 연결346/349였다.
  나머지3개는 측위 불확실성 초과다. callback p95 37.65ms이며 LAN·planner 승격은 별도다.
- 후보의 학습·score 문제·실ROS 입력·연속 평가·제약은
  [통합 검증 기록](261025_YOLO_CNN_MAP_RUNTIME_EVALUATION.md)을 본다.
  모델 파일을 바꾼 것과 운영 기본값을 바꾼 것은 구별한다.
- 이전 yellow/R+Y/unknown 계약은 [정책 통합 대조](261025_SIGNAL_POLICY_INTEGRATION_AUDIT.md)의 당시 기록이다.
  현재는 선 전 정차의 Y/R+Y 새 출발을 막고, missing·stale·UNKNOWN을 GO로 풀지 않는다.
  [황색 스위치·샘플 시간·GUI](261025_DEPARTURE_AND_SIGNAL_LOSS_PROPOSAL.md#yellow-controls-261026)에서 현재 OFF 기본과 이전 ON 시험을 구분한다.
- 팀 설치·GUI 선택·복귀는 [v5 시험 배포](V5_CANDIDATE_HANDOFF.md). 모델 압축본 한 파일만 공개 읽기이며 데이터·부모 백업은 비공개다.
- [시간제한 황색 후보](261026_TIMED_YELLOW_CANDIDATE.md)는 별도 flag 기본 OFF다. 2초 가정·0.2초 여유·허가 2회는 명시적 시험값이며 운영 기본 3회는 유지한다.
- 정차 위치는 Lanelet 지도/`traffic_light.stop_margin: 0.0`으로 관리한다. **v91 후보: v87 대비 A/C+0.75m·B+3m·D+2.75m**다. 선정 시험과 [추가A/B/C/D 재시험](261026_TIMED_YELLOW_CANDIDATE.md#v91-latency-retest)은 PASS, B/D 고유 지연은 미재현이나 부하 인과는 미확정이다. 기본v87·PR55 draft 유지. [시야·중단 이력](261026_TIMED_YELLOW_CANDIDATE.md#map-stopline-visibility-v91)을 본다.
- 단발 detector 누락만 동일 head·유효 지도에서 원본 관측 age 0.2초까지 보존한다. CNN UNKNOWN·모순·지도/TF 실패·기한 만료는 UNKNOWN이다.
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
허가 없는 Y/R+Y 새 출발은 금지다. 물리 통과선·시야 계약 전까지 황색 override는 보류하며 무패널티를 보증하지 않는다.
사용자가 재확인한 미션 명칭은 A·B·C(두 physical head)·D(옛 D-2)다.
동적 교차로 뒤에 보이는 옛 D-1은 검출 라벨로 존재해도 미션 신호 허용 대상으로 쓰지 않는다.

## 확인된 문제와 후보

1. C016/C018은 같은 신호 프로그램이다. 단일 등 강제 제어의 불일치를 자연 주행 오류로 세지 않는다.
2. 지도 crop은 근사치다. native 상태가 맞아도 픽셀 정답·crop 포함이 자동 보증되는 것은 아니다.
3. 초기 crop은 [이전 근거](261025_CNN_CROP_CANDIDATE_EVALUATION.md), 최종 모델은 위 통합 기록을 본다.
4. `route_progress_guard`는 기본 ON이다. 지난 미션 재선택을 억제하고 route/큰 재배치/시간 역전 시 초기화한다.
   0.5초 freshness는 원본 stamp 기준이다. 기본 GO_OUT은 아직 지도 정차선 기준이며 공식 도색선 통과 인증이 아니다.

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
- 이 승격은 로컬 개발 검증 범위다. 공식 LAN·팀 4090·BEVFusion 동시 부하에서의
  전환 지연·실제 센서 단절·공식 신호 준수는 후속 확인이며 주행 100% 인증이 아니다.
- 신호를 처음 본 시점은 점등 시작 시점이 아니다. 고정 2.5초 카운트로 황색 잔여 시간을
  보장하지 않으며, 팀 대화의 대회 주기 단축 예상도 운영 상수로 쓰지 않는다.

## 자료 위치
- 팀원 원문·영상·SHA: `artifacts/reference/traffic_light_team_notes_261025/`(원본 ZIP 보존).
- 신규 원본 JPEG·native/환경 증거·실패·QC·후보 결과:
  `artifacts/evaluation/traffic_light_robustness_261025/`.
- 도구: `scripts/evaluation/*traffic_light*.py`. capture는 개발용 정지/P 시험이다.
