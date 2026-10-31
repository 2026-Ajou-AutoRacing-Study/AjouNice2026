# Base5 MR 새 카탈로그 연결·동기 캡처 검증

2026-10-05. EVIDENCE / 소량 검증 완료, 본 수집·학습 미실행.

후속: [G809-case 및 복합GT4-capture](261005_g80_and_composite_gt_followup.md).
수동G80 actor까지 보존해도 ROS 재로드에서 floating 재현. 복합GT는 실제 구성요소로 확인.

## 구현

- `build_base5_mr_asset_catalog.py` → `configs/morai_asset_catalog_mr_base5_261005.json`.
  새 목록43/33/70의146 DataID를 source SHA와 역할/제외 이유로 보존했다.
- historical/production catalog와 일반 dynamic generator 기본값은 바꾸지 않았다.
  `generate_base5_static.py`만 새 versioned pool을 읽는다.
- Positive 후보 pool: Vehicle59(일반43+정적OBJ16), Pedestrian28, Obstacle19.
  OBJ_G80_Open_Door는 원래 Vehicle 후보17종 중 live 지면 검증 실패로 review 보류한다.
  Animal/복합/신규 유인 가능 자산은 review, 표지판/가로등은 roadside_context,
  tree/fire/speed bump/맨홀은 이번 positive 대상에서 제외한다.
- runtime metadata의 `class`는 MORAI transport container, `training_class`는 모델
  semantic class다. 정적OBJ는 objectList와 transport obstacle에 보존하면서
  training_class=vehicle로 기록한다. 기존검증기가 두 의미를 혼동한 실패를 수정했다.
- 리뷰 대상은 단순 UID→단일박스 변환이나 자동 positive 편입을 하지 않는다.

## 통신·수집

- 새6-interface Network에서 SaveSensorData/ScenarioLoad subscriber 각1,
  Ego/Object 수신 통과. ScenarioLoad로 sidecar를 다시 연결해도 recovery 통과.
- CargoBox 단일 control:25 files, Camera4/LiDAR/Instance annotation 저장 통과.
- 12 scene ×1 capture,12/12 완료,failure0. Ego 정지 상태 raw speed0.021–0.047.
  이 작은 residual을 실제 주행 속도/실시간 센서hz 검증으로 해석하지 않는다.
- G80 높이 재검증2 scene ×1 capture도 저장 통과. 동기 요청은 약1.74–1.77초.
  전체수집 예상시간이나10k throughput을 이 smoke만으로 확정하지 않는다.
- 캡처는 sensor files/Instance completeness와 static actor stability gate를 통과했다.
  Camera partial-view/초근접/전체 map road support/충돌에 대한 포괄검증은 아니다.

Raw와 manifest:

```text
/home/luv/morai_cap/261005_base5_asset_review/
  control/                  # 별도 CargoBox 1회
  captures/                 # 초기 transport-class 실패 근거: 2회 저장 뒤 중단
  scenarios_v2/             # 12-scene semantic/transport 분리 manifest
  captures_v2/              # 12회 canonical review capture
  audit_v2.json             # UID/GT/point 검증
  scenarios_v3/             # G80 요청 z -0.55m 실험 manifest (historical)
  g80_ground_recheck/       # G80 2회 재검증
  g80_ground_recheck_audit.json
```

대량scenario 생성 없음. MR Scenario directory에는 b5a01–12와 해당 sidecar만 설치했다.
현재 마지막 scene은 G80 측면 검증 장면이다. 기존 사용자 asset 목록은 수정하지 않았다.

## 실제 관측 및 판정

| 자산 | UID annotation | yaw-only GT 내부 point | 판정 |
|---|---|---:|---|
| CargoBox | Obstacle 1개 |42| control 저장/정합 통과 |
| WoodBox | Obstacle 1개 |5| sparse 경계, mesh 정합 해결됐다는 뜻 아님 |
| OBJ_Hyundai_Ioniq | Vehicle 1개, 방향2개 |324/871| objectList라도 sensor GT는 Vehicle |
| OBJ_G80_Open_Door | Vehicle 1개, 방향2개 |291/1163| 폭2.780m, 공중배치로 학습 보류 |
| Dog1 | Obstacle 1개 |16| 정지 capture만 검증, UNKNOWN 운동정책 해결 아님 |
| Deer1 | Obstacle 1개 |102| 동일 |
| Ped_Pet1 | 동일 UID: Pedestrian+Obstacle |단일 집계 안 함| 사람과 반려동물 분리 GT |
| Bicycle_Female1 | 동일 UID: Pedestrian+Vehicle |단일 집계 안 함| 탑승자와 자전거 분리 GT |
| BabyStroller1 | 동일 UID: Pedestrian+Obstacle+Pedestrian |단일 집계 안 함| 성인/유모차/아기 복합 GT |
| Bike1 | 동일 UID: Vehicle+Pedestrian |단일 집계 안 함| 실제 camera에서 탑승자 확인 |

단일 GT 내부 point 수는 yaw-only box 검사이고 대상 mesh의 완전성/충돌 형상 인증이
아니다. 다중 component를 단일 UID dictionary로 저장하면 덮어쓰기가 발생한다.
`audit_base5_asset_review.py`는 이를 review_multiple_components_same_uid로 보존한다.
기존 보정 parser를 전역 수정하지 않았으며 복합 객체를 그 parser에 넣지 않는다.

### G80 실패와 임의보정 회수

G80 bbox 하단은 LiDAR z 약-1.167/-1.179m, CargoBox road control은-1.714m였다.
Camera에서도 바퀴와 지면 사이 간격이 확인됐다. 시나리오 요청 z를-0.55m 조정한
재캡처에도 하단은 약-1.166/-1.179m로 유지되고 이미지도 떠 있었다.
따라서 요청좌표 변경이 실제 해당 자산 높이에 반영된다고 가정하지 않는다. 원인이
asset 자체인지 load/ground placement 규칙인지 아직 미확정이다. 임의z 보정은 코드에서
회수하고 해당 asset은 versioned pool에서 review로 보류했다. 영구블랙리스트는 아니다.

## 검증과 다음 gate

- Base5 관련 단위/integration12개 통과. 64-scene 임시 생성은 테스트 후 삭제되며
  MR 본수집이 아니다. 실제 캡처12개 모두 목표UID annotation 확인.
- 다음: 일반OBJ 차량의 near/side partial-view와 road-support 검증, 복합GT의
  component identity/class 정책, 전체 pool의 annotation coverage 및 converter smoke.
- 현 raw_class Vehicle 관측은 Ioniq/G80 두 자산의 근거이며 정적OBJ17종 전체의
  실제 GT class/지면 정합 완료로 확대 해석하지 않는다.
- 새 카탈로그 연결은 학습 PKL 완성 또는 production 교체가 아니다.

재검증:

```bash
python3 -m unittest discover -s dataset/morai_od/tests -p 'test_base5*.py'
python3 dataset/morai_od/scripts/audit_base5_asset_review.py \
  --capture-root "$HOME/morai_cap/261005_base5_asset_review/captures_v2" \
  --manifest "$HOME/morai_cap/261005_base5_asset_review/scenarios_v2/manifest.json" \
  --output "$HOME/morai_cap/261005_base5_asset_review/audit_v2.json"
```
