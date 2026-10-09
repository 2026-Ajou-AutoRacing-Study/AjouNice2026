# Base5 학습 분리와 가시성 해석 및 temporal 사용 결정

2026-10-09 KST / CURRENT 결정과 EVIDENCE 스냅샷을 구분한다.
Sunny13 v132 본5K를 유지하고 같은 배치를 6환경으로 반복해30K를 취득한다.
일부 자산의 관측 사례가 적다는 이유만으로 중단하거나 전체를 재수집하지 않는다.
완료 후 실제 train coverage와 rosbag 성능으로 필요한 자산·조건만 보강한다.
이번 정리는 문서만 변경하며 캡처 runtime·입력·GT 필터·production 설정을 변경하지 않는다.

## 현재 수집과 보존 원칙

- Base5-only 독립5K single-frame OD, 같은 geometry의 sunny/foggy ×11/13/15 반복이다.
  30K는 관측 수이며30K 독립 배치가 아니다. 현재 Sunny13 이외 환경의 완료를 뜻하지 않는다.
- 기존 raw·native Instance annotation·scenario·자산/parent/component·진단 trace를 보존한다.
  객체 비율은 sampling, 클래스/가시성은 가능한 범위의 재변환, 실패 자산은 제외·교체,
  새로운 hard case는 소량 추가 수집으로 대응한다. 새 수집이 기존 성공분을 덮어쓰지 않는다.
- 전면 재수집은 광범위한 센서 손상이나 필요한 정보가 원본에도 없는 경우에만 별도로 판단한다.
  crop/클래스/필터 변경이 모두 무조건 재수집 없이 가능하다는 보장은 아니다.
- 수집 중 동일5060Ti에서 GPU 학습·추론·latency benchmark를 병행하지 않는다.
  별도GPU/PC에서 학습은 가능하다. CPU 분석도 대량I/O로 캡처를 방해하지 않는다.
- 모델 정확도 상승 폭은 아직 모른다. 과거 소량E25-init pilot이나 offline AP를
  새v132 학습·실주행 성능 보장으로 사용하지 않는다. E25는 fallback으로 보존한다.

운영 경로는 [런북](261007_base5_5k_capture_runbook.md),
종료 후 시간 최적화의 별도 gate는 [반복 비용 계획](261009_base5_collection_cost_and_next_experiments.md)을 따른다.
현재manifest SHA256:
`89b053e7332e20f284c2de28d2f2365a3a9cf157d017c628481d9715ec5a1a77`.

## Train val과 rosbag test 분리

split은 수집 후 프레임을 랜덤하게 나누는 것이 아니라 생성 metadata에 이미 정해져 있다.
변환/환경 merge에서 그 split을 유지한다. 같은 scene의 환경 변형은 반드시 같은 split이다.

| 계획 관측 수 | Train | Val | 합성 Test |
|---|---:|---:|---:|
| Sunny13 5K |4255|745|0|
| 같은5K ×6환경 30K |25530|4470|0|

약85/15이며 실패·review·최종 변환에 따라 실제 학습 장수는 감소할 수 있다.
현재57개 `split_group` 중 train/val 혼합group은0이다. 분리 anchor는 생성 기준도로link지만
최종 Ego 배치link는5개가 train/val 양쪽에 등장하고 인접link의100m 시야도 겹칠 수 있다.
따라서 완전한 지리적 held-out이나 합성val만의 실주행 일반화 인증으로 부르지 않는다.

- Base5 val: epoch별 학습 상태와 후보 checkpoint를 좁히는 용도다.
- rosbag val: 이미 반복 확인한 기존/팀원bag으로 Detection·Tracking·Map-CV를 함께 평가하고
  모델·threshold·tracker 후보를 선택한다. 기본적으로 SGD 학습데이터에 섞지 않는다.
- rosbag test: 아직 튜닝에 쓰지 않은 독립 주행을 미션별로 확보해 설정 확정 후 최종 확인한다.
  같은 주행을 잘라 val/test로 나누거나 이미 반복 튜닝한bag을 최종test로 이름만 바꾸지 않는다.
- 새bag 부족 때문에 본학습을 무조건 미루지는 않는다. 기존bag으로 첫 비교를 하고
  고주로 전후방/추월, 근접 좌우 회피, 동적 횡단, 음영/로터리의 새 주행 확보를 병행할 계획이다.

## LiDAR와 GT가 학습에 들어가는 방식

모델 입력은 현재4카메라와 LiDAR XYZI다. voxel/sparse encoder의 기하 특징과 camera BEV
특징을 결합하며, 점에 객체ID나 정답클래스를 입력하지 않는다. GT는 native annotation의
전체 geometric BBox와 클래스다. 일부점만 보인다고 그점들을 감싼 작은박스로 GT를 줄이지 않는다.

현재 `lidar1_camera0`는 GT OBB 안의 LiDAR≥1점 또는 모델crop의 camera Instance≥16픽셀/
최소extent4픽셀을 positive로 사용한다. 같은parent/raw-class의 여러3D component는 camera
대응이 모호해 camera 조건을 인정하지 않는다. ROI 등 다른 변환조건도 함께 적용한다.
LiDAR 점 수는 OBB 내부점 수이며 해당객체에서 반사된 점만이라는 instance 보장은 없다.

3점·camera 조건미충족GT도 나머지조건을 만족하면 학습에 포함된다. 포함과 학습가능성은 다르며
희박한점만으로 클래스/전체크기를 결정하기는 어렵다. 선명한사례에서 배운형태와 주변맥락이 필요하다.
camera 조건이 false라고 이미지를 지우거나 해당영역을 masking하지 않는다.

안 보이는 secondary component가 제외된 성공장면은 나머지GT와 함께 학습에 유지한다.
그 장면을 실패quota로 다시 찍는 것은 아니다. primary/필수가시개수 실패는 기존 bounded
재시도·완료후 보충 대상이다. camera-visible/LiDAR0 component의 모호성으로 라벨이 제외되면
학습상 잠재적인 누락positive가 될 수 있으므로 별도점검한다. 문제없다고 전부 인증하지 않는다.

## 1771장 승인 스냅샷의 관측 단서

2026-10-09 약14:03 KST read-only 분석. 현재collection의 `selected_ledger.jsonl` 앞1771행을 고정해
분석했으며 live누적/완료5K·30K 수치가 아니다. 이후 같은prefix로 BabyStroller3를 재확인했다.
prefix1771 원본행 SHA256:
`7165dbd39c9d0d76385aa9557064982a9ed8acf6f667830bbdaa71da70b5d54a`.

승인1771장 중 component없는배경99장, raw GT4053개다. 현재positive 미충족139개를 제외하면
ROI내 학습후보3914개다. **139/4053=3.43%는 GT 비율이며 실패장면 비율이 아니다.**

분석용 ‘비교적 풍부한 단서’는 LiDAR≥20점 **또는** camera 대응이 모호하지 않고
modelcrop Instance≥64픽셀/가로세로각8픽셀 이상이다. 학습필터/공식선명도기준이 아니다.

| GT 클래스 | Positive GT | LiDAR≥20점 | 분석용 풍부한 단서 | 비율 |
|---|---:|---:|---:|---:|
| Vehicle |1672|1115|1583|94.7%|
| Obstacle |1126|346|768|68.2%|
| Pedestrian 구성요소 |1011|311|567|56.1%|
| Animal |105|12|60|57.1%|
| 전체 |3914|1784|2978|76.1%|

LiDAR≥5점3044개, camera positive조건3036개, LiDAR0/camera positive202개다.
LiDAR1–4점이며camera 조건false는229개/3914=5.85%다. **229개를 ‘실제 camera에 안 보인
객체229개’라고 부르지 않는다.** component 모호성/작은화면/crop밖도 포함한다.
pixel면적은 형상식별가능성/비가림 보장도 아니다. 이분석에 따른GT 제거·재수집은 하지 않았다.

선택장면 train1518/val253이다. Train GT3331개 중 풍부한단서2574개(77.3%)다.
전체122종 모두 위기준 사례가 있고, train에 등장한120종도 각각 위기준 사례가 있다.
train에서 그사례5개미만은18종이다. ‘희귀’는 공식출제확률이 아니라 현재 관측coverage 의미다.
자산별최소몇장은 학습충분성 보장이 아니며 완료후 train/val 각각 실제횟수를 재확인한다.

## BabyStroller3의 3개와 21개 해석 정정

위 snapshot의train에서 BabyStroller3는 **6개프레임의7개parent 배치**다.
`b5v2_00249`에2개parent가 있으며, 각parent에 성인Pedestrian/유모차Obstacle/아기Pedestrian이
하나씩 나와 **7×3=21개 component GT**다. 유모차21장면 중3장면만잘보인다는뜻이 아니다.

| 구성요소 | Positive GT | 분석용 풍부한단서 |
|---|---:|---:|
| 성인 |7|0|
| 유모차 본체 |7|3|
| 아기 |7|0|

- 21개모두 component별 LiDAR20점미만이다. 현재≥1점조건을 만족해21개모두 학습후보에 남는다.
- 성인/아기는 같은parent ID/Pedestrian class를 공유해 camera ambiguity=true다.
  실제 camera픽셀이 있어도 3D component별 대응을 인증하지 않아 풍부한단서 집계에서 빠진다.
- `b5v2_00441` 약12m에는 Pedestrian camera annotation 두개의model픽셀이786/263개,
  유모차562개다. 786/263을 성인/아기에 각각 확정매칭한 결과로 쓰지 않는다.
  두Ped GT의LiDAR는14/9점, 유모차17점이므로 집계에는 유모차만통과했다.
- 유모차본체의camera model픽셀64/562/246인3배치가 풍부한단서기준을 만족했다.
- 나머지3배치는해당camera modelcrop 픽셀이0이며, 약43m의1배치는본체33픽셀/LiDAR1점이다.
  실제약한관측도있지만 전부camera에안보였거나수집실패였다는해석은잘못이다.

복합GT와 camera대응의보수성 때문에 앞의Pedestrian/Animal 비율도 실제시각적선명도를
과소평가할수있다. 반대로 낮은점수사례를전부유용하다고단정하지않는다.
camera-only 모호성 처리개선은 **raw보존 후재감사/변환 후보**이며 현재수집필터 변경은 아니다.

## Temporal 코드와 현재 사용 여부

‘OD velocity 회귀를 쓰지 않음’과 ‘temporal로 탐지를 개선하지 않음’은 다른 결정이다.
이번대회 우선경로는 single-frame Base5 본학습이다. Temporal 탐지 연구를영구금지하지않지만
독립사진이나6환경반복을가짜연속sequence로학습하지않는다.

| 기능 | Production E25 | Base5 현재 학습config |
|---|---|---|
| 현재4카메라와현재LiDAR |사용|사용|
| T1 과거2LiDAR sweep 추가 |미사용|미사용|
| T2 계열 BEV 시간결합/cache |미사용|미사용|
| Q1 query memory |미사용|미사용|
| OD velocity head/loss |없음|없음|
| detector velocity tracker fusion |mode0/OFF|후단도OFF 유지계획|
| tracker 시간관측 기반 속도추정 |사용|새모델검증에도 유지계획|
| class-aware rotated NMS와tracker lifecycle |사용|배포시 유지할대상|
| 직사각형camera/LiDAR 축과head grid보정 |정사각형E25미변경|single-frame 별도보정사용|

production launch는 `temporal_sweeps=0`, `temporal_feature_cache=false`,
`query_memory_context=false`, `publish_detector_velocity=false`다.
Base5config는 `MoraiRectangularTransFusionHead`이며 query-memory head가 아니다.
plugin등록/import는 기능실행이아니다. T1/T2/Q1 코드·체크포인트·문서는삭제/롤백하지않고연구옵션으로보존한다.
temporal학습가중치/P7직접속도추정이E25/Base5에자동이식되지않는다.
좌표계/평가기·공용실행호환수정은남지만선택되지않은연구모델분기는실행하지않는다.
scan내deskew도temporal과별도이며현재production 기본OFF다.

T2R/P7 detector를켜고tracker만velocity OFF로평가한이력은있다.
T1은recall/latency, T2계열은정확도/비용gate, P7은새bag에서확실한교체근거부족으로승격하지않았다.
최신차량판정은 [GT복구후E25/P7재평가](../../../docs/bevfusion/261004_VEHICLE_STATUS_SIZE_GT_RECOVERY.md),
이력은 [Q1로드맵](../../../docs/bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md)을따른다.
과거개발bag지표를현재production보다무조건우수한성능으로인용하지않는다.

## 수집 후 진행 순서

1. 환경별종료·Park·완료후보충을확인하고 성공raw/실패근거를보존한다.
2. 자산별train coverage·거리/방향·실제가시parent/component·camera-only 모호성을감사한다.
   같은배치6환경은관측수는늘지만기하학적각도/거리/가림부족을해결하지않는다.
3. 고정split로Base5 single-frame 학습본을변환한다. 이번Base5 본학습은아직미시작이다.
4. offline val로후보를좁힌뒤 rosbag val에서 Detection/Tracking/Map-CV·거리별누락·첫전달거리·ID연속성을비교한다.
   새모델threshold는현재0.40/0.15/0.10을최적이라고고정하지않고별도선택한다.
5. 부족이확인된자산/근접/원거리조건만추가수집하고필요하면파인튜닝한다. 전체재수집을기본값으로하지않는다.
6. 설정확정후미사용독립rosbag test로최종확인한다. 최적화/모델교체는각각별도검증이며현재수집계약을덮어쓰지않는다.
