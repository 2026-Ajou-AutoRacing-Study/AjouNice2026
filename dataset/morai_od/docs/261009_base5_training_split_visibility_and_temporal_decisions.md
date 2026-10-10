# Base5 학습 분리와 가시성 해석 및 temporal 사용 결정

2026-10-09 KST / CURRENT 결정과 EVIDENCE 스냅샷을 구분한다.
이 문서의 미구현·미시작 표현은 아래 각 관측 시점의 기록이다. 후속 구현·완료 결과는
[수집기 후속 근거](261010_base5_postcollection_collector_work.md)와
[입력·pooling 최적화 근거](../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md)를 우선한다.
Sunny13 v132 본 5K를 유지한다. 종료 후 구성표·primary 할당을 유지한 Ego 위치 다양화를
구현·검증하고, 나머지 5 환경은 새 배치 5K씩 생성해 총 30K를 취득할 계획이다.
최신 순서와 공간/split 검증은 [종료 후 공간 다양성 계획](261009_base5_remaining_environments_spatial_diversity_plan.md)을 따른다.
일부 자산의 관측 사례가 적다는 이유만으로 중단하거나 전체를 재수집하지 않는다.
완료 후 실제 train coverage와 rosbag 성능으로 필요한 자산·조건만 보강한다.
이번 정리는 문서만 변경하며 캡처 runtime·입력·GT 필터·production 설정을 변경하지 않는다.

## 현재 수집과 보존 원칙

- Base5-only single-frame OD, sunny/foggy ×11/13/15의 6 환경 총 30K 목표다.
  같은 geometry 반복은 기존 구현 이력이며, 다음 5 환경의 새 geometry 생성은 아직 미구현이다.
  새 배치도 같은 지도/미션을 공유하므로 완전히 독립적인 배경 30K라고 부르지 않는다.
  현재 Sunny13 이외 환경의 완료를 뜻하지 않는다.
- 기존 raw·native Instance annotation·scenario·자산/parent/component·진단 trace를 보존한다.
  객체 비율은 sampling, 클래스/가시성은 가능한 범위의 재변환, 실패 자산은 제외·교체,
  새로운 hard case는 소량 추가 수집으로 대응한다. 새 수집이 기존 성공분을 덮어쓰지 않는다.
- 전면 재수집은 광범위한 센서 손상이나 필요한 정보가 원본에도 없는 경우에만 별도로 판단한다.
  crop/클래스/필터 변경이 모두 무조건 재수집 없이 가능하다는 보장은 아니다.
- 수집 중 동일 5060Ti에서 GPU 학습·추론·latency benchmark를 병행하지 않는다.
  별도 GPU/PC에서 학습은 가능하다. CPU 분석도 대량 I/O로 캡처를 방해하지 않는다.
  2026-10-09 후속 합의의 예외는 작은 연산 단위 CUDA correctness 시험뿐이다.
  전체 모델/학습 허용은 아니며 [실행 경계](../../../docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md)를 따른다.
- 모델 정확도 상승 폭은 아직 모른다. 과거 소량 E25-init pilot이나 offline AP를
  새 v132 학습·실주행 성능 보장으로 사용하지 않는다. E25는 fallback으로 보존한다.

운영 경로는 [런북](261007_base5_5k_capture_runbook.md),
종료 후 시간 최적화의 별도 gate는 [반복 비용 계획](261009_base5_collection_cost_and_next_experiments.md)을 따른다.
현재 manifest SHA256:
`89b053e7332e20f284c2de28d2f2365a3a9cf157d017c628481d9715ec5a1a77`.

## Train val과 rosbag test 분리

split은 수집 후 프레임을 랜덤하게 나누는 것이 아니라 생성 metadata에 이미 정해져 있다.
변환/환경 merge에서 그 split을 유지한다. 같은 scene의 환경 변형은 반드시 같은 split이다.
새 geometry는 논리적 quota slot과 실제 scene/group을 구분하고 도로/공간별 split을 환경 전체에서
일관되게 적용한다. 최종 XY/link와 인접시야의 중복은 별도 감사하며 slot번호만으로 공간누수를 인증하지 않는다.

| 계획 관측 수 | Train | Val | 합성 Test |
|---|---:|---:|---:|
| Sunny13 5K |4255|745|0|
| 이전 동일 geometry ×6 환경 계획값 |25530|4470|0|

약 85/15이며 실패·review·최종 변환에 따라 실제 학습 장수는 감소할 수 있다.
새 geometry25K를 합친 30K의 split 수는 새 manifest와 실제변환 결과로 다시 계산한다.
위 25530/4470은 최신계획의 확정장수가 아니다. 현재 Sunny13의 split은 보존한다.
현재 57개 `split_group` 중 train/val 혼합 group은 0이다. 분리 anchor는 생성 기준도로 link지만
최종 Ego 배치 link는 5개가 train/val 양쪽에 등장하고 인접 link의 100m 시야도 겹칠 수 있다.
따라서 완전한 지리적 held-out이나 합성 val만의 실주행 일반화 인증으로 부르지 않는다.

- Base5 val: epoch별 학습 상태와 후보 checkpoint를 좁히는 용도다.
- rosbag val: 이미 반복 확인한 기존/팀원 bag으로 Detection·Tracking·Map-CV를 함께 평가하고
  모델·threshold·tracker 후보를 선택한다. 기본적으로 SGD 학습데이터에 섞지 않는다.
- rosbag test: 아직 튜닝에 쓰지 않은 독립 주행을 미션별로 확보해 설정 확정 후 최종 확인한다.
  같은 주행을 잘라 val/test로 나누거나 이미 반복 튜닝한 bag을 최종 test로 이름만 바꾸지 않는다.
- 새 bag 부족 때문에 본학습을 무조건 미루지는 않는다. 기존 bag으로 첫 비교를 하고
  고주로 전후방/추월, 근접 좌우 회피, 동적 횡단, 음영/로터리의 새 주행 확보를 병행할 계획이다.

## LiDAR와 GT가 학습에 들어가는 방식

모델 입력은 현재 4카메라와 LiDAR XYZI다. voxel/sparse encoder의 기하 특징과 camera BEV
특징을 결합하며, 점에 객체 ID나 정답 클래스를 입력하지 않는다. GT는 native annotation의
전체 geometric BBox와 클래스다. 일부 점만 보인다고 그 점들을 감싼 작은 박스로 GT를 줄이지 않는다.

현재 `lidar1_camera0`는 GT OBB 안의 LiDAR≥1점 또는 모델 crop의 camera Instance≥16픽셀/
최소 extent 4픽셀을 positive로 사용한다. 같은 parent/raw-class의 여러 3D component는 camera
대응이 모호해 camera 조건을 인정하지 않는다. ROI 등 다른 변환 조건도 함께 적용한다.
LiDAR 점 수는 OBB 내부 점 수이며 해당 객체에서 반사된 점만이라는 instance 보장은 없다.

3점·camera 조건 미충족 GT도 나머지 조건을 만족하면 학습에 포함된다. 포함과 학습 가능성은 다르며
희박한 점만으로 클래스/전체 크기를 결정하기는 어렵다. 선명한 사례에서 배운 형태와 주변 맥락이 필요하다.
camera 조건이 false라고 이미지를 지우거나 해당 영역을 masking하지 않는다.

안 보이는 secondary component가 제외된 성공 장면은 나머지 GT와 함께 학습에 유지한다.
그 장면을 실패 quota로 다시 찍는 것은 아니다. primary/필수 가시 개수 실패는 기존 bounded
재시도·완료 후 보충 대상이다. camera-visible/LiDAR 0 component의 모호성으로 라벨이 제외되면
학습상 잠재적인 누락 positive가 될 수 있으므로 별도 점검한다. 문제없다고 전부 인증하지 않는다.

## 1771장 승인 스냅샷의 관측 단서

2026-10-09 약 14:03 KST read-only 분석. 현재 collection의 `selected_ledger.jsonl` 앞 1771행을 고정해
분석했으며 실시간 누적/완료 5K·30K 수치가 아니다. 이후 같은 prefix로 BabyStroller3를 재확인했다.
prefix 1771 원본 행 SHA256:
`7165dbd39c9d0d76385aa9557064982a9ed8acf6f667830bbdaa71da70b5d54a`.

승인 1771장 중 component가 없는 배경 99장, raw GT 4053개다. 현재 positive 미충족 139개를 제외하면
ROI 내 학습 후보 3914개다. **139/4053=3.43%는 GT 비율이며 실패 장면 비율이 아니다.**

분석용 ‘비교적 풍부한 단서’는 LiDAR≥20점 **또는** camera 대응이 모호하지 않고
model crop Instance≥64픽셀/가로·세로 각 8픽셀 이상이다. 학습 필터/공식 선명도 기준이 아니다.

| GT 클래스 | Positive GT | LiDAR≥20점 | 분석용 풍부한 단서 | 비율 |
|---|---:|---:|---:|---:|
| Vehicle |1672|1115|1583|94.7%|
| Obstacle |1126|346|768|68.2%|
| Pedestrian 구성요소 |1011|311|567|56.1%|
| Animal |105|12|60|57.1%|
| 전체 |3914|1784|2978|76.1%|

LiDAR≥5점 3044개, camera positive 조건 3036개, LiDAR 0/camera positive 202개다.
LiDAR 1–4점이며 camera 조건 false는 229개/3914=5.85%다. **229개를 ‘실제 camera에 안 보인
객체 229개’라고 부르지 않는다.** component 모호성/작은 화면/crop 밖도 포함한다.
픽셀 면적은 형상 식별 가능성/비가림 보장도 아니다. 이 분석에 따른 GT 제거·재수집은 하지 않았다.

선택 장면 train 1518/val 253이다. Train GT 3331개 중 풍부한 단서 2574개(77.3%)다.
전체 122종 모두 위 기준 사례가 있고, train에 등장한 120종도 각각 위 기준 사례가 있다.
train에서 그 사례 5개 미만은 18종이다. ‘희귀’는 공식 출제 확률이 아니라 현재 관측 coverage 의미다.
자산별 최소 몇 장은 학습 충분성 보장이 아니며 완료 후 train/val 각각 실제 횟수를 재확인한다.

## BabyStroller3의 3개와 21개 해석 정정

위 snapshot의 train에서 BabyStroller3는 **6개 프레임의 7개 parent 배치**다.
`b5v2_00249`에 2개 parent가 있으며, 각 parent에 성인 Pedestrian/유모차 Obstacle/아기 Pedestrian이
하나씩 나와 **7×3=21개 component GT**다. 유모차 21장면 중 3장면만 잘 보인다는 뜻이 아니다.

| 구성요소 | Positive GT | 분석용 풍부한 단서 |
|---|---:|---:|
| 성인 |7|0|
| 유모차 본체 |7|3|
| 아기 |7|0|

- 21개 모두 component별 LiDAR 20점 미만이다. 현재 ≥1점 조건을 만족해 21개 모두 학습후보에 남는다.
- 성인/아기는 같은 parent ID/Pedestrian class를 공유해 camera ambiguity=true다.
  실제 camera 픽셀이 있어도 3D component별 대응을 인증하지 않아 풍부한 단서 집계에서 빠진다.
- `b5v2_00441` 약 12m에는 Pedestrian camera annotation 두개의 model 픽셀이 786/263개,
  유모차 562개다. 786/263을 성인/아기에 각각 확정 매칭한 결과로 쓰지 않는다.
  두 Ped GT의 LiDAR는 14/9점, 유모차 17점이므로 집계에는 유모차만 통과했다.
- 유모차 본체의 camera model 픽셀 64/562/246인 3배치가 풍부한 단서 기준을 만족했다.
- 나머지 3배치는 해당 camera model crop 픽셀이 0이며, 약 43m의 1배치는 본체 33픽셀/LiDAR 1점이다.
  실제로 약하게 관측된 사례도 있지만 전부 camera에 안 보였거나 수집 실패였다는 해석은 잘못이다.

복합 GT와 camera 대응의 보수성 때문에 앞의 Pedestrian/Animal 비율도 실제 시각적 선명도를
과소평가할 수 있다. 반대로 낮은 점수 사례를 전부 유용하다고 단정하지 않는다.
camera-only 모호성 처리 개선은 **raw 보존 후 재감사/변환 후보**이며 현재 수집 필터 변경은 아니다.

## Temporal 코드와 현재 사용 여부

‘OD velocity 회귀를 쓰지 않음’과 ‘temporal로 탐지를 개선하지 않음’은 다른 결정이다.
이번 대회의 우선 경로는 single-frame Base5 본학습이다. Temporal 탐지 연구를 영구히 금지하는 것은 아니지만,
독립 사진이나 6개 환경 반복을 가짜 연속 sequence로 학습하지 않는다.

| 기능 | Production E25 | Base5 현재 학습 config |
|---|---|---|
| 현재 4카메라와 현재 LiDAR |사용|사용|
| T1 과거 2 LiDAR sweep 추가 |미사용|미사용|
| T2 계열 BEV 시간 결합/cache |미사용|미사용|
| Q1 query memory |미사용|미사용|
| OD velocity head/loss |없음|없음|
| detector velocity tracker fusion |mode0/OFF|후단도 OFF 유지 계획|
| tracker 시간관측 기반 속도 추정 |사용|새 모델 검증에도 유지 계획|
| class-aware rotated NMS와 tracker lifecycle |사용|배포 시 유지할 대상|
| 직사각형 camera/LiDAR 축과 head grid 보정 |정사각형 E25 미변경|single-frame 별도 보정 사용|

production launch는 `temporal_sweeps=0`, `temporal_feature_cache=false`,
`query_memory_context=false`, `publish_detector_velocity=false`다.
Base5 config는 `MoraiRectangularTransFusionHead`이며 query-memory head가 아니다.
plugin 등록/import는 기능 실행을 뜻하지 않는다. T1/T2/Q1 코드·체크포인트·문서는 삭제하거나 롤백하지 않고 연구 옵션으로 보존한다.
temporal 학습 가중치나 P7 직접 속도 추정이 E25/Base5에 자동으로 이식되지는 않는다.
좌표계·평가기·공용 실행 호환 수정은 남지만 선택되지 않은 연구 모델 분기는 실행하지 않는다.
scan 내 deskew도 temporal과 별개이며 현재 production의 기본 설정은 OFF다.

T2R/P7 detector를 켜고 tracker만 velocity OFF로 평가한 이력은 있다.
T1은 recall/latency, T2 계열은 정확도·비용 기준, P7은 새 bag에서의 확실한 교체 근거 부족으로 승격하지 않았다.
최신 차량 판단은 [GT 복구 후 E25/P7 재평가](../../../docs/bevfusion/261004_VEHICLE_STATUS_SIZE_GT_RECOVERY.md),
이력은 [Q1 로드맵](../../../docs/bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md)을 따른다.
과거 개발 bag 지표를 현재 production보다 무조건 우수한 성능으로 인용하지 않는다.

## 수집 후 점수제에 따른 실제 학습 분포 점검

2026-10-09 KST 사용자 합의 / 점검은 PLAN이며 생성기 수정은 결정하지 않았다.
현재 점수제와 Sunny13 성공 raw는 유지한다. 작은 객체 비중이 높다는 이유만으로 중단·폐기·
전면 재수집하지 않는다. 점수제는 장면의 배치 밀도를 조절하는 도구이지 클래스 균형이나
학습 난도를 보장하는 장치가 아니다.

범용 regular/dense는 작은 자산 여러 개로 같은 예산을 채울 수 있고, 남은 예산 때문에
큰 자산이 후보에서 제외될 수 있다. 단, 자산 pool 가중치와 primary 할당도 분포에 영향을
주므로 크기별 비율 차이를 점수제 하나의 인과 효과로 해석하지 않는다.

Sunny13 v135 승인 4,945장 선택 ledger의 읽기 전용 관측 스냅샷에서 범용 혼합은
regular 1,193장/dense 478장이다. 2점 자산은 primary 795/1,671=47.6%,
secondary 2,329/4,550=51.2%다. 이 값은 metadata의 배치 parent 기준이며 최종 유효 GT
비율이 아니다. 2점은 footprint 면적/최장변 기준이고 camera에서 작게 보인다는 뜻이 아니다.
이후 보충/변환/환경 변경 시 현재 비율로 재사용하지 않고 ledger/정책 버전과 분모를 다시 기록한다.

수집 종료 후 보존된 raw를 대상으로 다음을 점검한다.

- [ ] 최종 crop·ROI·가시성 정책으로 변환한 실제 유효 GT를 자산/클래스/크기별로 집계한다.
      배치 parent, 가시 parent, component GT를 구분하며 복합 자산의 여러 GT를 중복 배치로 세지 않는다.
- [ ] train/val과 범용/미션을 분리하고 primary/secondary, 거리·방향·가시성별 교차 분포를 확인한다.
      배치 크기 점수와 camera pixel/거리/부분 가림 등 실제 관측 난도를 별도로 본다.
- [ ] 일부 작은 자산의 반복 과다, 큰 자산의 부족, 희귀 자산 및 근접/원거리 조건의 누락을 확인한다.
      단순히 모든 자산의 개수를 균등하게 맞추거나 작은 객체를 어려운 사례로 간주하지 않는다.
- [ ] 실제 편중/coverage 부족이 확인될 때만 다음 5 환경 생성 전 보조 자산 선택 가중치·상한 등
      최소 조정을 검토한다. primary/family/거리/미션 quota는 기본 보존하고 변경 시 근거와 새 계약을 기록한다.
      현재 점수제 폐지·특정 객체 제외·추가 수집을 자동 실행하지 않는다.

## 수집 후 진행 순서

1. 환경별 종료·Park·완료 후보충을 확인하고 성공 raw/실패 근거를 보존한다.
2. 자산별 train coverage·거리/방향·실제 가시 parent/component·camera-only 모호성을 감사한다.
   위 점수제 분포 점검을 포함한다. 최종 crop 전 집계는 잠정값이며 확정 crop 변환 후 다시 확인한다.
   Sunny13 종료 후 Ego 위치 다양화를 구현·검증하고 나머지 5 환경은 새 배치로 생성한다.
   자산 할당/미션 quota는 유지하며 실제 30K의 위치·각도·거리·가림 coverage를 다시 확인한다.
3. 고정 split로 Base5 single-frame 학습본을 변환한다. 이번 Base5 본학습은 아직 미시작이다.
   입력 crop 확정 후 visibility/info를 재계산한다. [초기화 계획](../../../docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md)에 따라
   호환 nuScenes 가중치 + 새 4-class 부분을 추천하며 Base3 데이터를 기본 학습본에 섞지 않는다.
   현재 Base5 config의 load_from=None은 아직 이 추천으로 변경되지 않았다.
4. offline val로 후보를 좁힌 뒤 rosbag val에서 Detection/Tracking/Map-CV·거리별 누락·첫 전달 거리·ID 연속성을 비교한다.
   새 모델 threshold는 현재의 0.40/0.15/0.10이 최적이라고 고정하지 않고 별도로 선택한다.
5. 부족이 확인된 자산·근접·원거리 조건만 추가 수집하고 필요하면 파인튜닝한다. 전체 재수집을 기본값으로 삼지 않는다.
6. 설정 확정 후 미사용 독립 rosbag test로 최종 확인한다. 최적화·모델 교체는 각각 별도로 검증하며 현재 수집 계약을 덮어쓰지 않는다.
