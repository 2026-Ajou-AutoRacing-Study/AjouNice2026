# MORAI 문의 사항

- 작성일: 2026-09-21
- 용도: MORAI 담당자 전달용
- 상태: **질문 1·2 MORAI 공식 답변 수신 및 내부 정책 반영 완료**

## 공식 답변 요약과 확정된 내부 계약

2026-09-21 MORAI 담당자 답변으로 다음을 확정했다.

### 본선 시나리오 분포

1. 본선 고속주행도로 NPC의 초기·목표 속도는 샘플 시나리오와 동일하지 않다.
2. 본선 고속주행도로의 RandomMode 사용 여부는 아직 미정이다. RandomMode를 사용하면
   `MiddleSize` 차량만 사용할 예정이다.
3. 본선 회전 교차로 NPC의 초기·목표 속도도 샘플 시나리오와 동일하지 않다.
4. 본선 회전 교차로 NPC는 고정 차종이며, 구체적인 차종은 비공개다.

따라서 샘플의 `40/50/60 km/h`, 고주로 전체 asset 균등 분포,
`2015_Kia_K5` 고정 분포는 재현 근거이지 본선 분포 정답이 아니다. Base4 본 수집은
연속적인 NPC 속도 범위와 상대속도 조합을 넓히고, 본선 주력 분포는 `MiddleSize`
중심으로 구성하되 비공개·미정 사항에 대비한 소량의 차종 다양성 coverage를 남긴다.

### GT와 충돌 형상

1. `/Object_topic.size`는 시뮬레이터 asset library의 기본 제원이다. Scenario scale은
   그래픽과 물리 형상에는 적용되지만 `/Object_topic.size`에는 반영되지 않는다.
2. 차량의 `/Object_topic.position`은 기하학적 BBox 중심이 아니라 후륜 차축 중심이다.
3. LiDAR Instance annotation은 센서 좌표계에서 실제 3D mesh의 최외곽을 포함하는
   기하학적 OBB의 중심·크기이며 point labeling margin이 포함될 수 있다.
4. MORAI는 3D detection 학습과 자체 평가 모두 센서 캡처 annotation GT 사용을
   권장한다.
5. `/CollisionData`는 물리 엔진의 3D collider 간 충돌을 기준으로 하며,
   `/Object_topic` 기본 제원 box보다 센서 캡처의 기하학적 BBox에 가깝다.
6. Planning 충돌 검사는 LiDAR 기반 BBox에 안전 margin을 더하는 방식을 권장한다.

이에 따라 현재 내부 계약은 다음과 같다.

- SaveSensorData/Space Bar 기반 dataset은 LiDAR Instance annotation을 그대로 정답 GT로
  사용한다.
- rosbag 기반 Base4는 raw `/Object_topic` box를 정답으로 직접 사용하지 않는다.
  scale 1 actor에 대해 sensor-capture로 측정한 asset별 rear-axle/pivot→geometric-center
  offset과 capture size를 적용한 GT를 사용한다.
- 회전하는 차량의 geometric center velocity는 rear-axle velocity에 회전 offset 항을
  반영해 계산한다.
- Scenario scale은 당분간 1로 고정한다. scale actor를 도입하려면 rosbag 외의 scenario
  sidecar에서 scale을 명시적으로 전달하고 capture GT와 다시 검증해야 한다.
- rosbag detector/tracker 평가는 학습과 같은 corrected geometric GT를 사용한다.
  raw `/Object_topic` 기준 과거 수치는 GT 정의가 다른 historical evidence로 취급한다.
- Planning은 detector가 예측한 geometric box에 안전 margin을 적용한다.

`WoodBox`에서 관측한 LiDAR return과 annotation 경계 문제는 위 GT 정의 답변과 별개의
정합 문제다. 센서 캡처 annotation이 권장 GT라는 답변만으로 WoodBox 문제가 해결됐다고
간주하지 않으며, 대회 출제 여부와 point/mesh 정합은 후속 확인 대상으로 유지한다.

## 질문 1

안녕하세요, 아주대학교 AjouNice 팀 주상우입니다. 대회 실주행 시나리오에 대해 문의드립니다.

**1. 고속주행도로 NPC 속도 설정**

샘플 시나리오의 고속주행도로 NPC 차량들은 차선에 따라 목표 속도가 다르게 설정된 것을 확인했습니다.

대회 실주행에서도 고속주행도로 NPC 차량의 초기 속도와 목표 속도 범위가 배포된 샘플 시나리오와 동일한가요? 구체적인 값이 아니더라도 동일 여부와 대략적인 범위만 확인해 주셔도 감사하겠습니다.

**2. 고속주행도로 NPC RandomMode 차량 분포**

샘플 시나리오의 고속주행도로 차량은 RandomMode로 spawn되는 것을 확인했습니다. 대회 실주행에서도 같은 방식으로 spawn될 예정인가요?

가능하다면 RandomMode가 사용하는 차량 model pool의 범위와, 차량 모델을 균등하게 선택하는지 또는 승용차, SUV, 트럭, 버스 등 차종별 가중치가 적용되는지도 확인 부탁드립니다.

**3. 회전 교차로 NPC 속도 설정**

대회 실주행에서도 회전 교차로 NPC 차량의 초기 속도와 목표 속도 범위가 배포된 샘플 시나리오와 동일한가요?

**4. 회전 교차로 NPC 차량 기종**

샘플 시나리오의 회전 교차로 차량은 2015_Kia_K5 한 기종으로 고정되어 있습니다.

대회 실주행에서도 회전 교차로 NPC 차량 기종이 고정인가요? 아니면 고속주행도로처럼 RandomMode로 spawn될 예정인가요?

### MORAI 공식 답변

> 전 문의사항에 공통된 답변은 샘플 시나리오와 본선 시나리오 세부 구성이 완전히
> 동일하진 않을 예정입니다.
>
> 1. 샘플 시나리오와 동일하지 않습니다.
> 2. 해당 사항은 아직 미정입니다. RandomMode 사용 시 MiddleSize만 사용할 예정입니다.
> 3. 샘플 시나리오와 동일하지 않습니다.
> 4. 고정 차종으로 배치할 예정이며, 차종은 비공개입니다.

## 질문 2

안녕하세요. 아주대학교 AjouNice 팀 주상우입니다.

대회용 MR에서 /Object_topic과 Space Bar 센서 캡처의 LiDAR Instance annotation을 동일 객체에 대해 비교하던 중, 중심 위치와 크기 정의가 서로 다른 것으로 보이는 사례를 확인했습니다.

정확한 학습 GT와 충돌 회피 기준을 정하기 위해 아래 내용을 문의드립니다.

**1. Scenario scale과 /Object_topic.size**

배포된 샘플 시나리오의 CargoBox에는 scale.x = 2, scale.y = 3, scale.z = 2가 적용되어 있습니다.

그런데 /Object_topic에서는 scale 적용 전 기본 asset 크기로 보이는 0.750 × 0.750 × 0.430 m가 출력됩니다.

**2. /Object_topic.position과 Space Bar 캡처 GT 중심의 차이**

동일 객체에 대해 /Object_topic.position과 MR에서 Space Bar로 저장한 LiDAR Instance annotation의 box center가 일치하지 않는 경우가 확인됐습니다.

예를 들어 2016_Hyundai_Ioniq은 차량 로컬 종방향 기준 약 1.249 m의 차이가 관측됐습니다. 여러 차량에서도 객체별로 일정한 차이가 나타나, 단순 측정 오차보다는 asset origin 또는 pivot과 geometric BBox center가 서로 다른 것일 가능성을 생각하고 있습니다.

Space Bar 캡처 GT는 캡처 결과의 LiDAR Instance annotation에 기록된 unique ID, box center, rotation, size를 읽어 구성했습니다. 같은 unique ID의 /Object_topic 객체를 매칭한 뒤 두 위치를 같은 LiDAR 좌표계로 변환하여 center 차이를 계산했고, 동일 asset을 여러 번 관측한 결과의 중앙값을 사용했습니다.

**3. 두 인터페이스의 BBox 크기 차이**

Scenario scale이 1인 객체에서도 /Object_topic.size와 LiDAR Instance annotation의 size가 서로 달랐습니다.

- CargoBox: /Object_topic 0.750 × 0.750 × 0.430 m / Space Bar 캡처 GT 0.847 × 0.847 × 0.492 m
- PE_Drum: /Object_topic 0.490 × 0.490 × 0.800 m / Space Bar 캡처 GT 0.506 × 0.506 × 0.818 m
- TrafficBarrel: /Object_topic 0.810 × 0.810 × 0.940 m / Space Bar 캡처 GT 0.880 × 0.880 × 1.035 m
- Concrete_Barricade01: /Object_topic 0.390 × 1.920 × 0.810 m / Space Bar 캡처 GT 0.442 × 2.172 × 0.920 m
- PE_Firewall_Orange: /Object_topic 0.600 × 1.350 × 0.900 m / Space Bar 캡처 GT 0.698 × 1.579 × 1.046 m

참고로 현재 저희는 /Object_topic을 다음과 같이 해석하여 BBox를 구성했습니다.

center.x = position.x
center.y = position.y
center.z = position.z + size.z / 2
size = Object_topic.size
yaw = radians(Object_topic.heading)

**4. 학습 및 rosbag 평가에서 사용할 공식 GT**

LiDAR Instance annotation으로 학습하고 /Object_topic으로 rosbag 평가 GT를 구성하면, 두 GT의 중심과 크기 정의가 다를 경우 동일 모델에도 체계적인 오차가 발생할 수 있습니다.

이전에 다른 참가자분께서 동기화 방식의 센서 캡처로 데이터를 수집해 학습한다고 질문하신 내용도 확인하여, 다른 참가팀에도 공통으로 적용될 수 있는 문제라고 생각했습니다.

**5. /CollisionData의 충돌 판정 형상**

대회 규정에는 객체 충돌을 /CollisionData로 판정한다고 명시되어 있습니다. BBox 크기는 충돌 회피 planning의 안전거리와 직접 연결되므로 이 부분은 특히 중요하게 확인하고 싶습니다.

위 현상과 관련하여 다음 사항을 확인 부탁드립니다.

(1) /Object_topic.position은 asset origin, pivot, rear axle, 지면 기준점, geometric BBox center 중 어느 위치를 의미하나요? 위에 적은 저희의 BBox 구성 방식과 size 축 순서가 올바른지도 확인 부탁드립니다.

(2) LiDAR Instance annotation의 center와 size는 각각 어떤 기준의 BBox인가요? 특히 /Object_topic과 값이 다른 것이 의도된 동작인지 궁금합니다.

(3) Scenario scale은 렌더링 mesh, LiDAR Instance annotation, physics collider에 각각 어떻게 적용되나요? /Object_topic.size에는 scale이 적용되지 않는 것이 정상인지, 아니면 수정 가능한 문제인지 확인 부탁드립니다.

(4) 3D Object Detection 학습과 rosbag 평가에서 공식 GT로 사용해야 하는 중심과 크기는 어느 쪽인가요? /Object_topic을 사용해야 한다면 LiDAR Instance annotation과 같은 기준으로 변환하는 공식 방법이나 객체별 center offset 및 size 정보를 제공받을 수 있을까요?

(5) /CollisionData는 /Object_topic BBox와 LiDAR Instance annotation 중 어느 형상을 기준으로 판정하나요? 둘 다 아니라면 별도의 physics collider를 사용하는지, 그 중심과 크기는 어느 GT에 더 가까운지, Scenario scale도 동일하게 적용되는지 확인 부탁드립니다.

(6) 참가팀이 planning 충돌 검사를 구현할 때 기준으로 사용하도록 권장되는 객체 중심과 크기는 어느 쪽인가요?

두 GT의 차이가 의도된 설계라면 각각의 용도와 올바른 변환 방법을 알려주시면 감사하겠습니다. 필요하다면 대회용 MR에서 만든 재현 시나리오, rosbag, Space Bar 센서 캡처 원본과 비교 결과를 전달드리겠습니다.

학습에 사용한 GT와 실제 대회의 충돌 판정 기준이 다르면 학습과 실주행 사이에 체계적인 차이가 발생할 수 있어 긴급히 문의드립니다.

질문에 모호한 부분이 있다면 말씀해 주시는 대로 바로 보충하겠습니다. 긴 질문 읽어주셔서 감사합니다.

### MORAI 공식 답변

> `/Object_topic`의 size 정보는 시뮬레이터 에셋 라이브러리에 등록된 기본 에셋
> 제원입니다. Scale 변환이 시뮬레이터 그래픽/물리에는 반영되지만 ROS topic 상에는
> 제원 정보만 출력됩니다.
>
> `/Object_topic`의 position 정보는 후륜 차축 중심입니다. 차량 동역학 및 ROS
> TF(`base_link`)의 표준 기준축입니다.
>
> LiDAR 센서 GT는 센서 좌표계를 기준으로 객체의 2·3차원 형상을 감싸는 기하학적
> BBox의 정중앙을 계산하여 제공합니다. 센서 캡처 BBox는 3D mesh의 실제 정점의
> 최외곽 돌출부를 모두 포함하는 OBB를 동적으로 생성하며, point cloud labeling을 위한
> margin이 포함되어 asset 규격보다 조금 더 크게 산출됩니다.
>
> 센서 캡처 방식으로 취득한 GT를 학습 데이터로 사용하고 동일한 기준으로 자체 평가할
> 것을 권장합니다. 기하학적 형상에 부합하는 GT도 동기 캡처 방식의 데이터입니다.
>
> `/CollisionData`는 시뮬레이터 물리 엔진에 배치된 3D physics collider 간 충돌을
> 기반으로 판정하므로 기하학적으로 LiDAR 캡처 데이터에 가깝습니다. Planning 충돌
> 검사는 LiDAR BBox에 안전 margin을 더하는 방식을 권장합니다.

## 선택적으로 추가할 질문

`2017_Hyundai_Grandeur(30100001)`를 static actor로 직접 배치한 scenario는 known-good
control과 같은 위치에서도 반복 load에 실패했습니다. 반면 샘플 시나리오의
`RandomMode`에서는 Grandeur가 정상 spawn되고 `/Object_topic`에도 정상 발행됐습니다.
direct static ScenarioLoad template/직렬화 경로의 알려진 제한인지 확인 부탁드립니다.

이 항목은 대회 분포 및 GT 정의 문의와 별개이므로, 담당자에게 질문이 너무 많아 보이면
후속 문의로 분리한다.

## WoodBox 첨부 근거

대회 MR 정상 주기 최신 근거를 우선 첨부한다.

담당자에게 먼저 보낼 최소 rosbag pair:

`/home/luv/woodbox_cargobox_rviz_pair_yaw0_260921_015742.zip`

- 같은 위치와 yaw 0°의 `CargoBox_yaw0.bag`, `WoodBox_yaw0.bag`만 포함한다.
- 크기: 약 `120 MiB`.
- SHA-256: `22083f3adeeb12576644a30fe1ee9d2630357558e2f3efa3185c9805ee19cf5c`.

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

LiDAR point–mesh/BBox 경계 문제는 위 GT 정의에 대한 답변을 받은 뒤 별도 문의한다.
첫 문의에는 이 절의 WoodBox 영상과 수치를 섞지 않는다. MORAI가 GT 정의 확인을 위해
센서 근거를 요청할 때만 대회용 MR에서 얻은 동일-pose CargoBox/WoodBox 자료를
추가한다.

기존 Base3 obstacle benchmark bag을 5~20 m에서 교차검사한 결과, 지면 가까운 5 cm를
제외한 `raw GT -> +10 cm GT` 경계 민감도는 TrafficBarrel `3.1%`,
Concrete_Barricade01 `11.3%`, CargoBox `24.3%`, PE_Firewall_Orange `24.0%`,
PE_Drum `51.3%`였다. 별도 동일-pose pair에서는 CargoBox `22.2%`, WoodBox
`58.7%`였다. 이는 object point instance ID가 없는 raw cloud의 진단 지표이므로 실제
mesh point 오차율로 단정하지 않고, asset별 재현 우선순위를 정하는 내부 근거로만 쓴다.
`concrete-barrier`는 `/Object_topic` size가 `(0,0,0)`이라 동일 지표를 계산할 수 없다.

## 내부 메모: 문의 순서

1. 시나리오 분포 질문과 객체 GT 정의 질문을 한 번에 보내기 부담스러우면 두 문의로
   나눈다.
2. 객체 문의에서는 scale, pivot/center, size, 학습/평가 GT, CollisionData를 함께
   묻는다.
3. LiDAR point–mesh/BBox 정합 문제는 GT 정의 답변 뒤 별도 문의한다.
4. 외부 전달문에는 개인용 런처나 내부 수집 API 이름을 적지 않는다.
5. Grandeur는 선택 질문으로 분리한다.
6. RandomMode 실측 482 birth 결과와 MORAI 답변의 pool/분포를 비교한다.

## 내부 메모: 수치 출처와 외부 전달 전 재현

- 현재 75-asset center-offset catalog와 표의 size 수치는 기존 Base3 센서 캡처의 LiDAR
  `Instance.txt`에서 산출했다. 카메라 `BBox3D.txt`의 position을 사용한 값이 아니다.
- `/Object_topic` 비교값은 캡처 trigger 주변 callback을 기록한 metadata와 unique ID로
  결합했으므로 완전히 동일한 sensor timestamp라고 과장하지 않는다. 정적 actor 또는
  짧은 callback age 표본을 우선 근거로 쓴다.
- 외부 전달 전에는 MR에서 representative Vehicle과 표의 obstacle을 scale 1로 다시
  Space Bar 캡처하고, LiDAR Instance 출력과 `/Object_topic`을 독립적으로 대조한다.
- 최근 MR 수동 캡처 중 일부는 `Intensity.bin`만 남아 있으므로, 재현 때 LiDAR Instance
  annotation 저장이 활성화됐는지 먼저 확인한다.
