# G80 배치 및 복합 자산 GT 후속 확인

2026-10-05. EVIDENCE. G809개+복합자산4개 동기 캡처 완료, 통신/annotation 저장 실패0.
Production·학습·일반 GT parser는 변경하지 않았다.

후속 학습 정책은 [MR CURRENT](MR_ASSET_COLLECTION_CURRENT.md)의
"동물·아기·복합 객체 학습 정책"을 따른다. 3-class 유지와 component별 소량 학습에
합의했지만 현재 parser/pool 편입이 완료된 것은 아니다.

## G80: 생성기 필드 차이가 단독 원인은 아님

`generate_base5_g80_placement_probe.py`로 두 위치에서 수동 저장본의 필드와 자동
배치 필드를 factorized 비교했다. 원래 Ego 위치/새 Ego 시점, isWaypointMode true/false,
initPos 원본0/좌표변환 값, 원래 UNIQUEID10 유지까지 검사했다. 각scene에4m 옆의
CargoBox를 ground control로 배치했다. G80 자체는 scale1.

| 조건 | G80 GT 하단 LiDAR z | CargoBox GT 하단 LiDAR z |
|---|---:|---:|
| 수동 저장 위치·원래 Ego | -1.207 | -1.762 |
| 같은 객체 위치·다른 Ego 시점 | -1.155 | -1.724 |
| waypoint만 false | -1.156 | -1.725 |
| initPos만 변환 | -1.156 | -1.725 |
| 이전 smoke 위치·수동 필드 | -1.166 | -1.730 |
| 이전 smoke 위치·자동 필드 | -1.166 | -1.729 |
| 이전 smoke 위치·waypoint만 false | -1.167 | -1.730 |
| 이전 smoke 위치·initPos만 변환 | -1.167 | -1.730 |

카메라에서 G80 바퀴 아래 간격이 보이고 CargoBox는 지면에 붙는다. 약55–57cm 차이는
두 위치의 ground-control 비교 근거이며 전체맵의 정확한 ground height 추정이 아니다.

9번째검사는 원래 G80 actor dictionary를 UNIQUEID10까지 그대로 보존했다.
요청pos.z=28.486000061, 실제ObjectStatus pos.z=28.985580444로 약+0.500m였다.
원래 위치에서 modified UID를 사용한 경우도 동일28.985580444였다.
Smoke 위치에서는 actual pos.z=28.899719239, CargoBox=28.399749756이었다.

**판정:** 우리가 바꾼 waypoint/initPos/UID 또는 특정Ego 시점만으로 생긴 현상은 아니다.
수동 저장본에서 actor를 그대로 추출해 ROS ScenarioLoad로 재로드해도 재현된다.
사용자가 GUI 수동배치 당시 정상인 것을 확인했다는 사실과 합치면, 현재 유력한 경계는
해당asset의 로드/지면정렬과 GUI 직접배치 경로의 차이다. 내부 mesh/collider 자체 버그인지
ScenarioLoad/ground-snap 동작인지는 아직 확정하지 않는다. 단순히 "asset은 원래 항상
떠 있다"고 결론내리지 않는다. 다음 구분 실험은 GUI에서 동일저장본 로드 및 신규 직접배치
직후 실제pos.z를 ROS 재로드와 대조하는 것이다.

원본사용자목록은 수정하지 않았고 G80 review 보류를 유지한다. 이전-0.55m 요청z 실험은
실제높이를 바꾸지 못했으며 임의GT/point 이동으로 숨기지 않는다.

일반 차량 목록에는 문 닫힌 `2021_Hyundai_G80`(30100010, vehicleList)이 별도로 있다.
현재 Vehicle positive이며 이번 실험 대상이 아니었다. 문 열린 objectList asset의
실패를 일반 G80에 그대로 적용하지 않는다.

## 복합 자산: 여러 GT는 실제 구성요소의 별도 박스

LiDAR Instance 행, CAMERA_1 BBox2D의 class/하위ID, RGB와 Instance 이미지를 대조했다.
유모차는 정면/측면, 반려동물은측면, Bicycle_Female1은반대방향으로 추가캡처하여 확인했다.

| 자산 | 구성요소별 GT | 원래캡처의 높이 |
|---|---|---|
| Ped_Pet1 | 성인 Pedestrian + 개 Obstacle | 1.783m / 0.604m |
| Bicycle_Female1 | 여성 Pedestrian + 자전거 Vehicle | 1.733m / 1.122m |
| BabyStroller1 | 성인 Pedestrian + 유모차 Obstacle + 아기 Pedestrian | 1.715m / 1.010m / 0.584m |
| Bike1 | 자전거 Vehicle + 탑승자 Pedestrian | 1.122m / 1.596m |

즉 전체합성mesh를 둘러싼 하나의박스나 중복오류가 아니다. 추가4개캡처에서도 같은
구성/class가 유지됐다. 유모차 정면RGB에 유모차 내부 아기가 실제로 보인다.

### ID 규칙과 변환 주의

- CAMERA BBox2D는 부모9508/9509/9510/9511과 음수 하위ID를 별도로 기록한다.
  예: BabyStroller1 성인9510, 유모차-951001, 아기-951002.
- LiDAR Instance는 위세 구성요소 모두 부모9510을 반복하여 기록한다.
- 따라서 `records[unique_id] = box` 방식은 두/세개 중 마지막하나만 남긴다.
  `uid -> list[component]` 또는 독립annotation instance key가 필요하다.
- 이름또는DataID에 클래스하나만 지정해 모든구성요소에 적용하면 사람을 차량/장애물로
  오학습할 수 있다. component별 capture raw_class와 geometry를 보존해야 한다.
- 자전거를 MORAI가 Vehicle로 annotation하는 사실은 production의 CAR tracker/prediction
  정책이 사람+자전거 상황에 적절하다는 검증이 아니다. 동물Obstacle 또한 움직임이 있을 때
  UNKNOWN 강제정지정책과 충돌한다. 따라서 현 review상태를 유지하며 자동편입하지 않는다.
- 이 발견은 기존parser의 복합객체 처리 한계다. 과거엔 이자산들을 제외했으므로 기존
  E25 전체학습이 이문제로 잘못됐다고 확대해석하지 않는다.

## 근거 경로

```text
/home/luv/morai_cap/261005_base5_g80_followup/
  scenarios_v3/manifest.json     # 첫8개 factorized probe
  captures/                     # 8회
  scenarios_v4/manifest.json     # 원래UID 보존 추가
  original_uid/                 # 1회
/home/luv/morai_cap/261005_base5_components_followup/
  scenarios/manifest.json
  captures/                     # 4회, RGB/Instance/GT 포함
```

초기 probe 생성 중필수ego field검증 오류가 있었고 simulator 호출 전에 중단됐다.
완성manifest만설치/로드했다. 지면판정은 RGB 확인과 rawGT 비교이며 수정된 GT로
검증하지 않았다. 본수집/학습은 하지 않았다.
