# MORAI 문의 사항

- 작성일: 2026-09-20
- 용도: MORAI 담당자 전달용
- 상태: WoodBox 내부 재현 완료, MORAI 공식 답변 대기

## 전달용 본문

안녕하세요. K-City 자율주행 경진대회 시나리오 및 객체 구성과 관련하여 아래 사항을
확인 부탁드립니다.

1. **NPC 속도 설정**
   - 본 대회의 로터리·고주로 NPC 차량 초기 속도와 목표 속도 범위가 배포된 샘플
     시나리오와 동일한가요?
   - 다르다면 구체적인 값 대신 동일 여부와 대략적인 범위만이라도 확인 가능할까요?

2. **고주로 RandomMode 차량 분포**
   - 샘플 시나리오의 고주로 `RandomMode`가 사용하는 vehicle model pool은 어떤
     범위인가요?
   - 각 모델을 균등하게 선택하는지, 승용차·SUV·트럭·버스 등 차종별 가중치가
     적용되는지도 확인 부탁드립니다.

3. **로터리 NPC 차량 기종**
   - 배포된 샘플 시나리오의 로터리 spawn은 `spawnVehicleTypeID=30100004`, 즉
     `2015_Kia_K5` 한 기종으로 고정되어 있습니다.
   - 본 대회에서도 로터리 NPC 차량 기종이 고정인가요? 고정이라면 샘플과 동일한
     `2015_Kia_K5`인지 확인 부탁드립니다.

4. **WoodBox 3D GT 정합**
   - 개인용 MRP에서 `WoodBox(40100002)`를 거리와 yaw를 바꾸어 배치했을 때,
     CargoBox보다 LiDAR mesh return과 size-derived 3D GT 경계의 차이가 크게
     관측됐습니다. 대회 계열 MR에서도 정상 sensor 주기로 좌·우 위치와 yaw
     `0/30/60/90°`의 CargoBox/WoodBox 동일-pose pair를 반복했으며, WoodBox의
     pose-dependent GT 경계 밖 return이 재현됐습니다. 대표적으로 yaw 90° WoodBox는
     raw GT 내부 point가 0개였지만 box를 5 cm 확장하면 24개가 포함됐습니다.
   - 두 런처의 WoodBox asset/annotation 버전 차이가 있는지, 대회 MR에서 제공되는
     공식 3D GT 크기를 그대로 사용해도 되는지 확인 부탁드립니다.
   - `WoodBox`가 실제 대회 장애물 후보로 출제될 가능성이 있는지도 확인 가능할까요?

감사합니다.

## 선택적으로 추가할 질문

개인용 MRP `R1.260701.H3`에서 `2017_Hyundai_Grandeur(30100001)`를 static actor로
직접 배치한 scenario는 known-good control과 같은 위치에서도 반복 load에 실패했습니다.
반면 같은 런처의 샘플 `RandomMode`에서는 Grandeur가 정상 spawn되고 `/Object_topic`에도
정상 발행됐습니다. direct static ScenarioLoad template/직렬화 경로의 알려진 제한인지
확인 부탁드립니다.

이 항목은 대회 분포에 관한 핵심 세 질문과 별개인 개인용 런처 호환성 문의이므로,
담당자에게 질문이 너무 많아 보이면 후속 문의로 분리한다.

## WoodBox 첨부 근거

대회 MR 정상 주기 최신 근거를 우선 첨부한다.

1. `wbmrc001_bev.png`: CargoBox, `+14/+3 m`, yaw 0°.
2. `wbmrc002_bev.png`: WoodBox, 같은 pose.
3. `wbmrc007_bev.png`: CargoBox, `+18/-3.5 m`, yaw 90°.
4. `wbmrc008_bev.png`: WoodBox, 같은 pose.

로컬 경로:

`artifacts/datasets/woodbox_alignment_260921/mr_normalhz_oblique_grid/`

전달 archive:

`artifacts/handoff/woodbox_mr_normalhz_260921/woodbox_mr_normalhz_evidence_260921.tar.zst`

- 크기: 약 `236 MiB`.
- SHA-256: `edb3fc5dd85177aef0e3b7e2d9bab66e1d50d386ee9a79429a17f73c6cd927e9`.

아래 MRP 그림은 launcher 간 비교가 필요할 때 추가한다.

우선 첨부할 그림:

1. `01_cargobox_10m_yaw30.png`
2. `02_woodbox_10m_yaw30.png`
3. `03_cargobox_10m_yaw45.png`
4. `04_woodbox_10m_yaw45.png`

로컬 경로:

`artifacts/datasets/woodbox_alignment_260920/morai_question_evidence/`

필요할 때 추가할 그림:

- `05_cargobox_15m_yaw45.png`
- `06_woodbox_15m_yaw45.png`

정량 근거는 같은 폴더의 `woodbox_alignment_audit.json`에 있다. 첫 문의에서는 숫자를
과도하게 강조하기보다 동일 pose의 CargoBox/WoodBox 그림을 전달하고, MORAI가 원본
데이터를 요청할 때 rosbag과 전체 40-scene 결과를 추가 제공한다.

## 내부 메모: 문의 순서

1. 처음에는 전달용 본문의 네 질문만 보낸다.
2. WoodBox 그림은 10 m / yaw 30도와 45도 두 쌍만 첨부한다.
3. Grandeur는 선택 질문으로 분리한다.
4. RandomMode 실측 482 birth 결과와 MORAI 답변의 pool/분포를 비교한다.
