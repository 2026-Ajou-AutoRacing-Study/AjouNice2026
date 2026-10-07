# MR 전용 객체 카탈로그·수집 계약

상태: CURRENT. 최신 공식 답변 반영 2026-10-06. 향후 수집은 MR만 사용한다.

2026-10-07 후속 답변: MORAI는 제출한 진단 시나리오의 객체/waypoint 공통 Z와
정적 장애물 이동 설정을 지적했다. 자산별 pivot 높이 및 경로 지면을 구분한다.
다른 자산 전체의 런처 부유 버그가 확정됐다는 해석은 철회하며, 실제 캡처 품질 검사를
유지한다. 현재 rigid/차량 정지 정책은 동일하다. 동적 캡처는 독립 지면 경로 준비 전
실행을 금지하고, 검증되지 않은 자산의 초근접 배치에는 보수적인 geometry 여유를 둔다.

## 최신 공식 출제 범위 — 2026-10-06 답변 정리

[MORAI 추가 답변](260920_questions_for_morai.md#최신-추가-답변--객체-범위g80차량-속도-기준)이
아래 과거 추정/보류 문구보다 우선한다.

- GPS 음영 및 정적/동적 장애물 구간의 후보는 대회용 F2에 보이는 전체 객체 에셋이다.
  Vehicle/Pedestrian/Animal/복합 객체가 출제될 수 있다. 전체 균등 출제라는 뜻은 아니다.
- 기존 정적 객체를 움직이는 예외는 없다고 확인했다. CargoBox 같은 rigid static
  자산의 비정상 이동 설정을 대회 동적 미션 분포에 넣지 않는다.
- `OBJ_G80_Open_Door`(40100034)는 **공식 출제 제외**이며 옵션 목록 삭제 조치를
  예고했다. 기존 review 보류보다 강한 제외 사유다. 실제 런처 패치/목록 삭제는
  미확인이고 문 닫힌 G80은 이 제외 대상이 아니다. 후속 v33에서 catalog도
  competition_excluded로 반영했으며 pool loader에 ID 방어 규칙을 추가했다.
- 타 자산의 동적 부유는 MORAI가 재현 시나리오를 요청한 상태다. 다른 자산의
  동일 버그 인정/수정/출제 제외까지 확정된 것은 아니다. 기존 ground/Z quality gate 유지.
- 동물/복합 객체 coverage와 component GT는 유지한다. 나무/불/도로시설 등의
  기존 background/ignore 선택은 우리 학습 정책이지 공식 출제 금지가 아니다.
  공식 전체 범위를 반영한 잔여 coverage/ontology 검토가 필요하며, 모든 자산을
  Obstacle positive로 바꾸거나 무조건 차로 중앙에 배치하지 않는다.

아래 2026-10-05 합의/구현 대기 절은 이력이며, 최신 구현은 Base5 v2 CURRENT를 따른다.

2026-10-06 구현 후속: [Base5 v2 CURRENT](BASE5_V2_CURRENT.md)가 아래의 구현 대기
문구를 대체한다. Component-preserving 변환과 움직이는 Animal의 별도4-class 연구 후보가
연결됐다. 기존3-class production은 유지하며 최종 ontology/runtime은 미확정이다.
새 v2 live는 연결 실패 후 복구하여48장 검사했다. 전체pool 검증을 완료했다는 뜻은 아니다.

최신 독립 수집 목표는 [Sunny13시 직접5K](261007_base5_direct_5k_current_plan.md)이며
quota/자세/동작·실행 경로는 [Base5 CURRENT](BASE5_V2_CURRENT.md)를 따른다.
구 [10K 설계](BASE5_INDEPENDENT_10K_DESIGN.md)는 최초 설계 이력이다.
이 문서의 asset eligibility와 review 보류는 유지하며 새 설계가 자동 pool 편입을 뜻하지 않는다.

## 최신 구현 상태 — 2026-10-05 후속

[새 카탈로그 연결 및12-scene MR 검증](261005_base5_mr_asset_review_smoke.md)을 우선한다.
Base5 전용 versioned catalog를 연결했고 pool은 Vehicle59/Pedestrian28/Obstacle19다.
당시 G80 open door는 지면 실패로 review 보류였고 현재는 위 공식 출제 제외가 우선이다.
Animal/유인 복합 자산도 당시 review였으며 후속 구현 상태는 Base5 v2 CURRENT를 따른다.
아래의 "미구현" 문구는 합의 당시 이력이며 Base5 연결 상태는 이 절과 evidence가
대체한다. 기존 general generator/production/과거 catalog는 변경하지 않았다.

## 동물·아기·복합 객체 학습 정책 — 2026-10-05 합의 / 구현 대기

[G80 및 복합 GT 확인](261005_g80_and_composite_gt_followup.md)에서 실제 구성요소와
annotation ID 구조를 확인했다. 아래는 다음 학습용 정책이며 현재 자동 positive pool에
이미 편입됐다는 뜻이 아니다. Animal/복합 자산의 review 상태는 필요한 변환 검증 전까지 유지한다.

- 모델은 우선 Vehicle/Pedestrian/Obstacle 3-class를 유지한다. 별도 Animal head는 추가하지 않는다.
- 원본·카탈로그에는 animal과 세부 종류를 보존하고, 학습용 동물은 우선 Obstacle로 매핑한다.
  움직이는 동물까지 UNKNOWN 속도0 정책으로 처리하는 것은 한계다. 동적 대응이 필요하면
  별도 class와 tracker/planning 매핑을 함께 검토하며 현재 정책이 해결했다고 주장하지 않는다.
- 아기는 Pedestrian이다. 유모차 내부 아기의 독립 GT도 원본에 보존한다. 실제 가시성과
  크기 기준으로 supervision을 검토하고 이름만으로 일괄 제외/일괄 positive 처리하지 않는다.
  현재 point gate를 임의로 없애거나 camera-visible 기준이 이미 구현됐다고 해석하지 않는다.
- 복합 객체는 소량부터 학습 포함을 지향한다. 사람+개, 성인+유모차+아기 등을 하나의 큰
  박스로 합치지 않고 구성요소별 GT 및 class를 보존한다. 사람/아기=Pedestrian,
  개/유모차=Obstacle을 기본 후보로 관리한다.
- 자전거는 센서 annotation에서 Vehicle이지만 현재 Vehicle→CAR 및 CAR Map-CV 정책과
  의미가 다를 수 있다. 사람은 Pedestrian으로 보존하되 자전거 구성요소의 학습/runtime
  매핑은 검토 대기다. 자동차로 즉시 편입하거나 무조건 Obstacle로 바꾸지 않는다.
- LiDAR Instance의 같은 부모 UID 아래 여러 행을 보존하는 parser가 선행 조건이다.
  `uid -> box` 덮어쓰기를 금지하고 `uid -> components` 또는 독립 instance key를 사용한다.
  순서에만 의존한 시간축 component ID 대응도 검증 없이 확정하지 않는다.
- 일반 사람·차량의 비중을 보존하고 희귀 복합 자산을 별도 관리한다. 소량 학습 이후
  동일 mission rosbag에서 기존 class 성능과 근접 객체 재현 성능을 함께 확인한 뒤 확대한다.

문 닫힌 `2021_Hyundai_G80`(30100010, vehicleList)은 현재 Vehicle positive다.
문 열린 `OBJ_G80_Open_Door`(40100034, objectList)는 별도 asset이며 현재 공식 출제 제외다.
이번 9-case 지면 검증은 문 열린 asset만 대상으로 했으며 일반 G80도 실패한다고 확대하지 않는다.

## 2026-10-05 새 목록 검수 및 후속 정책

아래는 **목록 검수 완료 / 생성기 연결·분류 변경 미구현** 상태다. 아래 합의가 기존
catalog/eligibility/runtime에 이미 적용됐다고 해석하지 않는다.

MR Scenario directory의 `new_vehicle_list.json`, `new_pedestrian_list.json`,
`new_obstacle_list.json`을 DataID 기준으로 기존 catalog와 비교했다.

| 목록 | 기존 고유 종류 | 새 고유 종류 | 검수 결과 |
|---|---:|---:|---|
| Vehicle | 43 | 43 | 기존 누락 없음 |
| Pedestrian 및 관련 객체 | 33 | 33 | 기존 누락 없음 |
| Obstacle 및 관련 객체 | 33 | 70 | 기존 누락 없음, 신규37종 |

Obstacle 목록은81개 배치/70종이다. Tree03(`40200004`)와 EFFECT_Fire01(`40300001`)
모두 포함한다. `_2`를 원본 이름으로 승격하고 EgoNetwork sidecar도 함께 교체했으며
두 파일의 일치를 확인했다. 중복 배치는 삭제하지 않았다. 생성기에서는 DataID로
중복 제거하여 배치 횟수가 asset sampling 가중치로 변하지 않게 해야 한다.
새 목록은 로컬 MR SaveFile에 있으며 이번 문서 변경으로 Git catalog에 편입되지는 않는다.

### 당시 합의 및 검토 경계 (후속 정책은 위 최신 절 우선)

- MRP 미지원·교집합·quarantine을 신규 MR 수집의 제외 이유로 쓰지 않는다.
- 신규 정적 `OBJ_` 차량17종은 수집·Vehicle positive 편입 우선 후보다.
  정지 여부와 semantic class를 분리한다. 이는 현재 미구현인 추천 방향이다.
  `OBJ_G80_Open_Door`는 열린 문을 포함하는 geometric GT를 확인한다.
  일반 차량과 같은 이름이라는 이유로 기존 pivot offset/size를 복사하지 않는다.
- 현재 runtime은 Vehicle→CAR, Pedestrian→PEDESTRIAN, Obstacle→UNKNOWN이다.
  UNKNOWN motion policy는 속도를0으로 만든다. 동물/탑승자처럼 움직일 수 있는
  객체를 그대로 Obstacle로 편입하는 것은 안전한 호환 방안이 아니다.
- Animal(Dog1/Dog2/Deer1), Ped_Pet1, Bicycle_Female1, 사람이 포함될 수 있는
  BabyStroller/Bike/Scooter 계열은 후속 소량 annotation/형상 확인 대상이다.
  이름만으로 탑승자 유무를 확정하지 않는다. 신규 학습에서 동물을 라벨 없는
  hard negative로 섞는 것은 피한다. Animal 클래스 추가나 Pedestrian 편입은
  아직 결정/구현하지 않았다. 기존 catalog의 hard_negative_only/quarantine은 남아 있다.
- 무인 자전거/스쿠터 등 실체 정적 객체는 낮은 비중의 Obstacle 편입 후보다.
- WoodBox/CargoBox 등 실체 장애물은 포함한다. scale1 유지, 센서 캡처의
  geometric GT 사용, rosbag 변환 시 reference coverage 검증을 유지한다.
- **Tree01–09, EFFECT_Fire01, Speed_Bump, 맨홀은 이번 positive 보강 및 재검토
  대상에서 제외한다.** 전체 목록에는 보존하며 기존 도로 배경을 제거하지 않는다.
- **일반 표지판·가로등은 도로변 배경으로만 둔다. 차로 중앙에 랜덤 배치하지
  않으며 이번 집중 수집/검토 대상이 아니다.** 차로를 막는 바리케이드류는 별도
  실체 장애물이다. 기존 정책의 도로시설 context와 Obstacle positive를 혼동하지 않는다.
- 70종 전체 균등 sampling이나 obstacleList 전체를 Obstacle class로 변환하지 않는다.
  정상 차량/보행자 비중을 보존하고 희귀 복합 자산은 별도 관리한다.

### 다음 순서

1. 새 목록 기반의 versioned catalog/template/eligibility를 만들고 역할과 제외 이유를
   명시한다. 기존 production 및 과거 dataset catalog는 재현용으로 보존한다.
2. 정적 차량의 Vehicle 편입과 무인/유인 복합 자산 분리를 loader까지 연결한다.
3. 소량 MR 동기 capture에서 Instance annotation, bbox, 부분 시야, 지면/충돌 상태를
   확인한다. 새 정적 차량의 rosbag calibration은 별도 확보한다.
4. 통과한 pool로 Base5 소량 smoke 후 본 수집 runner를 연결한다. 현재±30m ROI와
   이후 원거리 확장 학습 계약을 구분하며 대량10k 수집부터 시작하지 않는다.

## 현재 코드 기준 — 새 목록 연결 전

- 원본: `configs/morai_asset_catalog_260807.json` (대회 MR에서 저장한 목록).
- Eligibility: `configs/morai_asset_mr_collection_261004.json`.
- Template directory: `$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025`.
- `generate_dynamic_object_capture_set.py`의 catalog/compatibility/runtime directory 기본값을 모두 MR 기준으로 변경했다. MR의 vehicle_list.json, pedestrian_list.json, obstacle_list_without_vehicle_and_ped_like_objects.json을 읽는다.
- MRP 교집합·direct-load quarantine을 MR 수집에 적용하지 않는다. Grandeur, police_f_1, Boy2, Scooter_01 포함. 이전 MR minimal-hybrid에서 네 종 모두 두 위치 2/2 load 성공했다. 이번 변경은 offline pool 연결 검증이며 live 재수집은 아니다.
- WoodBox 포함. 정합 우려는 audit 대상으로 남기되 asset blacklist로 사용하지 않는다. Raw point를 임의 이동시키지 않는다.
- MR `aiPedstrianList`도 읽는다. NCAP dummy를 MRP 문제 때문에 제외하던 subtype gate는 MR에 적용하지 않는다. 신규 MR dummy의 실제 capture 품질은 이후 검증한다.

## 개수와 유지하는 필터

MR 원본은 Vehicle43/Pedestrian container33/Obstacle container33, 총109 entry이다. Speed_Bump와 Tree scope 제외 후 eligible ID는 43/33/31, 총107이다.

Runtime loader는 ontology/generation_policy도 적용한다. 실제 MR template에서 Vehicle43/Pedestrian28/Obstacle 계열31개를 읽었다. Obstacle 계열에는 조건부 배치·배경 역할도 있어 최종 positive-class 분포와 같지는 않다. Animal/Cyclist/복합 객체를 임의로 Pedestrian으로 재분류하지 않는다.

- Speed_Bump·Tree 및 기존 사용자 ontology/scope 제외 유지.
- scale1, 시간 정합·ego drift·가시성/GT point gate 유지. Asset 복구와 quality gate 완화는 별개이다.
- MRP catalog/compatibility는 과거 재현용으로 보존한다. 명시적으로 선택하면 기존 personal profile을 사용할 수 있다.

## 카탈로그 포함 ≠ GT 보정 완료

WoodBox, police_f_1, Boy2, Scooter_01은 MR 20-scene/40-capture 검증으로 measured reference를 확보했다. 별도 `configs/morai_asset_bbox_offsets_mr_restored_261004.json`에 출처와 잔차를 기록했고 MR catalog에도 reference를 연결했다. 기존75-asset production catalog는 변경하지 않았다. CargoBox control은 이번 MR과 역사적 reference의 size가 달라 두 버전을 분리했다. 상세 결과는 [보정 런북](261004_mr_restored_asset_gt_calibration_runbook.md)을 따른다.

**복구 네 종은 reference를 확보했지만 학습 데이터 편입은 아직 미실행이다.** 새 rosbag 수집·변환에서 명시적으로 reference를 적용하고 검증해야 한다. Converter unknown_assets를 확인하고 미식별 객체를 unlabeled background로 둔 채 학습하지 않는다. 신규 dummy/조건부 자산도 reference coverage를 확인한다.

이를 코드로 보장하기 위해 info converter의 `--unknown-asset-policy` 기본값은 `error`다. Missing calibration을 발견하면 PKL 확정 전에 중단한다. 과거의 누락 재현은 `--unknown-asset-policy drop`을 명시해야 하며 production 학습용으로 쓰지 않는다.

## Base5와 과거 결과

새 smoke generator는 WoodBox 포함27 scene, diversity56 scene이다. Pass smoke는 CargoBox/Barricade 두 종을 이름으로 선택한다. 기존 manifest overwrite를 거부하므로 새 출력 directory를 사용한다.

기존24-scene smoke,48-scene diversity,688/115 replay pilot은 변경 전 결과이며 새 객체가 자동 추가되지 않는다. 원본 bag/manifest/PKL와 production E25를 변경하지 않았다.

## 검증

- MR profile unit tests2개 PASS: MR-only fixture, AI pedestrian list, 복구 대상, scope 제외, 명시적 historical MRP profile.
- 기존 dynamic capture tests52개 PASS.
- Base5 tests6개 PASS: WoodBox 생성, 초기속도0, scale1, highway55m margin.
- 후속 MR live 보정20/20 scene, 40/40 capture PASS. 학습은 미실행이다. 동기 보정 이후 rosbag 수집에는 UDP sensor network 재연결이 필요하다.
