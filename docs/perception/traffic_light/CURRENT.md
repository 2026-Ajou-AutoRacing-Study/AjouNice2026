# 신호등 인지 CURRENT

상태: 2026-10-25 사용자 요청으로 **v5 지도+CNN을 traffic·Validator 기본으로 전환**. 신호 정책은 불변.
환경 실행은 [패키지 README](../../../base_ws/src/perception/traffic_light_detection/README.md)를 따른다.
과거 문서의 개선 제안이나 연구 가중치를 운영 기본값으로 읽지 않는다.

## 운영 기준

- `traffic_light_node.launch`: front JPEG 1280×720, YOLO housing 1클래스,
  confidence 0.25, 최대 처리 10Hz, smoother 7/4.
- 기본은 YOLO26n960 + 지도 연결 + CNN v5, `~/.local/share/ajounice/traffic_light_gpu/bin/python`이다.
  고정 모델 SHA·경로는 [배포 계약](../../../configs/perception/traffic_light_v5_candidate_release.json)을 따른다.
- 관측은 CNN 4-bit 분류, route별 보정은 기존 `traffic_signal_profiles.yaml`이다.
  `debug_observed_label`은 보정 전, `debug_raw_label`은 이름과 달리 보정 후다.
- A/B/C/D-2의 fallback은 적색이다. D-1은 우회전 passthrough이고
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
- yellow/R+Y/unknown을 인지와 BVP가 다르게 처리하는 현재 계약은
  [정책 통합 대조](261025_SIGNAL_POLICY_INTEGRATION_AUDIT.md)를 본다.
  [새 출발·신호 상실 제안](261025_DEPARTURE_AND_SIGNAL_LOSS_PROPOSAL.md)은 아직 운영 적용 전이다.
- 팀 설치·GUI 선택·복귀는 [v5 시험 배포](V5_CANDIDATE_HANDOFF.md)를 따른다.
  승인한 모델 압축본 한 파일은 로그인 없이 받는다. 데이터/부모 백업은 비공개, 주행 검증은 별도다.
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
위 표는 영상 관측을 평가하는 기준이며 CNN 입력에 구간 GT나 phase 정답을 넣는 기능이 아니다.
현행 D-2의 R+Y 통과 설정은 코드 사실이지, 황색을 처음 보고 새로 출발해도 된다는
규정·팀 정책의 확정이 아니다. 공식 정지선 통과와 이미 진행 중인 경우를 구분한 문의가 남아 있다.
사용자가 재확인한 미션 명칭은 A·B·C(두 physical head)·D(옛 D-2)다.
동적 교차로 뒤에 보이는 옛 D-1은 검출 라벨로 존재해도 미션 신호 허용 대상으로 쓰지 않는다.

## 확인된 문제와 후보

1. C016/C018은 같은 신호 프로그램이다. 단일 등 강제 제어의 불일치를 자연 주행 오류로 세지 않는다.
2. 지도 crop은 근사치다. native 상태가 맞아도 픽셀 정답·crop 포함이 자동 보증되는 것은 아니다.
3. 초기 crop은 [이전 근거](261025_CNN_CROP_CANDIDATE_EVALUATION.md), 최종 모델은 위 통합 기록을 본다.
4. 실제 route bag 재생에서 통과한 D의 재선택이 나왔다. 당시 지도와 현행 v87의 누락 lane ID도 있어
   전체 미션 선택 인증은 보류한다. `audit_traffic_light_route_replay.py`로 원래 지도·새 주행을 대조한다.

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
- 운영 승격은 별도다. 원시 관측, 정책 보정 후 출력, 전환 지연, 위험 오통과,
  장기 정차, 실제 ROS 입력·출력을 각각 검증해야 한다.
- 신호를 처음 본 시점은 점등 시작 시점이 아니다. 고정 2.5초 카운트로 황색 잔여 시간을
  보장하지 않으며, 팀 대화의 대회 주기 단축 예상도 운영 상수로 쓰지 않는다.

## 자료 위치

- 팀원 원문·영상·SHA: `artifacts/reference/traffic_light_team_notes_261025/`(원본 ZIP 보존).
- 신규 원본 JPEG·native/환경 증거·실패·QC·후보 결과:
  `artifacts/evaluation/traffic_light_robustness_261025/`.
- 도구: `scripts/evaluation/*traffic_light*.py`. capture는 개발용 정지/P 시험이다.
