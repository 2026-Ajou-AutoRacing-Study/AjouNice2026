# 신호등 인지 CURRENT

상태: 2026-10-25 강건성 분석·CNN 연구 후보 1차 시험 완료. **운영 모델·정책 변경 없음.**
환경 실행은 [패키지 README](../../../base_ws/src/perception/traffic_light_detection/README.md)를 따른다.
과거 문서의 개선 제안이나 연구 가중치를 운영 기본값으로 읽지 않는다.

## 운영 기준

- `traffic_light_node.launch`: front JPEG 1280×720, YOLO housing 1클래스,
  confidence 0.25, 최대 처리 10Hz, smoother 7/4.
- 현재 `best.pt` SHA256:
  `2bb73720568a975c94d79a54cc097a0a4b70ec7c8de23552b4d0e71112160818`.
- 관측은 HSV/형태 분류, route별 보정은 `traffic_signal_profiles.yaml`이다.
  `debug_observed_label`은 보정 전, `debug_raw_label`은 이름과 달리 보정 후다.
- A/B/C/D-2의 fallback은 적색이다. D-1은 우회전 passthrough이고
  판단 모듈에서 제외된다. 별도 planning competition fail-open과 혼동하지 않는다.
- native 신호 조회·제어 및 `/Ego_topic`은 **개발 평가용**이다.
  새 진단 도구는 운영 launch/대회 알고리즘에 연결하지 않았다.

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

## 확인된 문제와 후보

1. C에서 가까운 housing을 놓치고 더 작은 뒤쪽 housing만 검출하는 사례를 재현했다.
   두 등 C016/C018은 **같은 신호 프로그램**이다. 단일 등 강제 제어로 생긴 불일치를
   자연 주행의 다른 신호 선택 오류로 해석하지 않는다. 검출·선택·화질은 분리 평가한다.
2. 올바른 housing crop에서도 좌화살표를 원으로 분류하는 사례를 6환경에서 재현했다.
   HSV 보완 및 multi-label CNN을 같은 crop으로 비교한다.
3. 지도 투영 crop은 근사치다. 초기 시험에서 상단 잘림과 판독 불가 표본을 발견했다.
   native 상태가 맞아도 픽셀 정답·crop 품질이 자동 보증되는 것은 아니다.
4. frozen 특징+linear head의 예비 시험 이후 crop 교정·시각 QC를 거쳐
   ResNet18 본체 fine-tune을 완료했다. 새 pose의 판독 가능한 신호 crop에서는
   CNN 120/120, 기존 HSV 93/120 조합 일치였다. [표본·한계·재현](261025_CNN_CROP_CANDIDATE_EVALUATION.md)을 본다.
   이 결과는 YOLO 미검출·화각 이탈·판독 불가·ROS 주행을 포함한 정확도가 아니다.

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

- 팀원 원문·영상·SHA: `artifacts/reference/traffic_light_team_notes_261025/`.
  원본 ZIP은 변경하지 않았으며 큰 영상·원문은 Git 일반 context에 넣지 않는다.
- 신규 원본 JPEG·native/환경 증거·실패·QC·후보 결과:
  `artifacts/evaluation/traffic_light_robustness_261025/`.
- 도구: 저장소 `scripts/evaluation/*traffic_light*.py`.
  capture 도구는 개발용 정지/P 실험이며 production bringup이 아니다.
