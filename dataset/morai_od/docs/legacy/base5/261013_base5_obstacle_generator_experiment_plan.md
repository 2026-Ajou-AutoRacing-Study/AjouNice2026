# Base5 장애물 생성기 실험과 보강 계획

문서 상태: 실험 설계의 기준. 구현과 현재 결과는 [보강 실험 근거](261013_base5_obstacle_support_experiments.md)를 따른다.
기준일: 2026-10-13.
후속 계획 주석(2026-10-16): 현재 본수집 전 작업은 [정적 미션 범위와 자산 크기 다양화 계획](261016_base5_mission_extent_and_asset_scale_plan.md)을 따른다. 아래 이전 실험·판단은 보존하며 새 자산·배율의 승인으로 사용하지 않는다.

Sunny13 완료 5K는 보존한다. 다음 다섯 환경의 새 25K를 수집하기 전에 MORAI 배치 실험과
실제 학습 GT 분석을 반복해 장애물 생성 정책을 개선한다. 객체 개수·거리 비율·자산 배분을
지금 확정하지 않는다. 앞서 제안한 근거리 1.5~2배와 약 96장 pilot도 시작 후보이지 합격 조건이 아니다.

사용자가 MORAI 통제권을 맡겨 생성기 변경·소량 캡처·native 대조·복구 검증을 수행했다.
새 25K는 offline 배치만 생성했고 본수집은 시작하지 않았다. 긴 정지 NPC의 선택적 보완과
Sunny13 정적 64/64·이동 8/8 검증을 거쳐 v150 release를 준비했다. [현재 후보 런북](261013_base5_obstacle_support_runbook.md)을 따른다.
다른 agent의 planning_validator·주행·수집 세션과 동시에 시나리오를 로드하지 않는다.

원거리 차량 전달과 다음 거리 분포의 후속 방향은 [공통 개선 계획](../../../docs/bevfusion/261013_BASE5_RANGE_AND_TRACKING_IMPROVEMENT_PLAN.md)을 따른다.
비차량의 근중거리 집중은 다음 후보이며 기존 원거리 GT 삭제나 v150의 무검증 변경을 뜻하지 않는다.

2026-10-14 MORAI OFF 분석의 후속 근거는 [센서·모델·중복 검토](../../../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#model-data)에 있다.
완료 5K의 native component→infos 누락은 없었다. 장애물 yaw는 이미 균등하고,
0~10m의 후방 495/전방 285개 노출은 전방 보충의 근거다. 20~30m는 610 GT 중
LiDAR 20점 이상 4.43%지만 camera mask-rich까지 합치면 75.25%로, 전부 비가시인
것이 아니다. v150의 짝 표본에서 해당 구간 LiDAR-rich 증가가 확인되지 않아 개수
증가와 sensor support 개선을 구분한다. 현 E24의 query peak만 바꾸는 실험은 중복을
없앴지만 장애물 TP를 479→369로 줄여 기각했다. 학습 query 구성·depth hint 안정화도
모델 측 별도 요인으로 검토하고, 생성기 비율이나 기존 GT를 이 결과만으로 바꾸지 않는다.

## 현재 근거와 해석

모델 비교는 [Base3 val 교차 평가 44절](../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-on-base3-val-261013)을 따른다.
아래 AP는 같은 원본 Base3 val의 center AP이며 학습 노출 수와 다른 분모다.

| 장애물 중심 거리 | E25 AP | E9 AP | E24 AP |
|---|---:|---:|---:|
| 0~10m | 97.27% | 88.91% | 88.87% |
| 10~20m | 97.57% | 88.27% | 88.38% |
| 20~30m | 93.66% | 67.87% | 67.59% |

원래 train infos의 Obstacle GT를 중심 XY 거리로 나눈 관측 수는 다음과 같다.
Base3 7,632프레임과 Base5 4,255프레임을 각각 분모로 사용한다. 같은 배치의 반복 관측은
독립 객체로 보지 않으며, BEV 증강·range filtering 후 매 epoch의 정확한 gradient 노출량도 아니다.

| 중심 거리 | Base3 GT | Base5 GT | Base3 1,000프레임당 | Base5 1,000프레임당 | Base5 중 LiDAR 5점 이상 |
|---|---:|---:|---:|---:|---:|
| 0~10m | 3,902 | 780 | 511.3 | 183.3 | 605 |
| 10~20m | 5,746 | 837 | 752.9 | 196.7 | 708 |
| 20~30m | 1,043 | 610 | 136.7 | 143.4 | 286 |
| 30~50m | 5 | 324 | 0.7 | 76.1 | 52 |
| 50~80m | 0 | 55 | 0 | 12.9 | 5 |
| 80~110m | 0 | 9 | 0 | 2.1 | 0 |

80~110m는 현재 직사각형 ROI 안의 대각선 위치를 포함하는 집계 구간이며 인지 범위를 확장한 것이 아니다.
LiDAR 5점 이상은 센서 지원량의 한 구간이지 선명함·가시성·정확한 형상의 보증이 아니다.

0~20m 노출은 Base3가 약 3.3배 많았다. 그러나 20~30m의 등장 횟수는 비슷한데 AP 격차가
더 크므로 전체 개수 증가만으로 해결할 수 있다고 가정하지 않는다. 자산별 분산, point 희소성,
가림·crop, GT geometry, 학습 구조·초기화 차이를 분리한다. Base3 공통 평가가 30m 미만이므로
이 결과로 30~100m 장애물의 필요성을 기각하지 않는다. 절대적인 데이터 부족이나 보강 후 AP 향상도 아직 확정하지 않는다.

GT 집계 입력은 다음 두 파일이다. 라벨·split·원본 데이터는 변경하지 않는다.

- `artifacts/datasets/morai_base3_bevfusion_portable_v1/morai_base3_infos_train.pkl`
- `artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release/infos/512x1056/morai_base5_infos_train.pkl`

## 보존할 계약과 실험 가능한 항목

보존할 기본 계약은 [데이터 CURRENT](BASE5_V2_CURRENT.md)와 [자산 계약](MR_ASSET_COLLECTION_CURRENT.md)을 따른다.

- 완료 Sunny13 raw·native annotation·selected ledger·학습 source·E9/E24 checkpoint를 유지한다.
- 총 6환경 30K 계획, 환경별 독립 geometry, 클래스 의미·복합 component, Q90·upright GT 정책을 유지한다.
- 고주로·로터리 차량 전용, 공식 제외 자산, 원거리 차량·보행자와 주요 미션 사례를 보호한다.
- 지면·Ego 정지·CollisionData·가시성 QA와 유한 retry를 우회하지 않는다. 시험을 성공시키려고 GT를 추가하거나 낮은 point GT를 일괄 삭제하지 않는다.
- Base3 val/test를 학습에 넣지 않는다. 배치 개선이 detector의 depth·초기화 문제까지 해결했다고 주장하지 않는다.

기존 패밀리 수·primary floor·방향·mission phase·split은 시작 기준이다. 보조 객체 구성,
크기 budget, 자산별 가중치·최소 노출, 거리 배분은 실측에 따라 조정할 수 있다. 필요하면 패밀리 비율도
검토하되 기존 미션·희귀 자산을 잃는 변화는 명시한다. 논리 slot과 물리 link/group split의 계보를
보존하고 이전 manifest와 달라진 계약을 새 policy·delta로 기록한다. frozen guard를 조용히 완화하지 않는다.

## 실험 루프

각 반복은 다음 순서로 수행한다. 완료 작업을 작은 횟수로 끊기보다 필요한 검증까지 이어 수행하되
안전 중단·권한·세션 충돌은 예외다. 중간 사용자 응답이 없어도 허용 범위의 계획 수정은 계속할 수 있다.

1. 기존 5K의 자산·거리·방향·지원량·복합 GT·혼합 budget 분포와 누락·중복 사례를 대조한다.
2. 한두 요인을 바꾼 후보를 새 경로에 생성한다. 가능한 비교에서는 Ego·primary·seed·센서를 맞추고 변경 요인과 예상 효과를 적는다.
3. CPU에서 높이 초기값·BBox 경계 간격·허용 구역·split·quota·배치 실패 사유를 검사한다.
4. 통제권을 맡긴 MORAI에서 소량 캡처한다. 원본 LiDAR·카메라·native annotation·Object trace·QA·시간을 보존한다.
5. 실제 학습 GT와 영상/point 사례를 확인하고 후보를 유지·수정·기각한다. 판단과 실패 근거를 남기고 다음 반복을 설계한다.
6. 문제가 남으면 해당 조건을 더 시험한다. 다양성·지원량·실패·시간의 균형이 설명될 때 새 정책을 동결한다.

현재 구현에서 후속 환경 geometry는 `prepare_base5_fresh_environment.py`와
`base5_spatial_diversity.py`, 객체 선택 budget은 `base5_population_budget.py`, 최종 배치는
`generate_base5_independent.py`를 거친다. 보강은 이 경로와 replenishment의 계약까지 연결한다.
소량 후보만 좋아지고 본수집 재시도에서 옛 거리·보조 객체 정책으로 돌아가는 상태로 끝내지 않는다.

## 우선 시험할 후보

| 후보 | 배치 실험 | 판단할 내용 |
|---|---|---|
| 근거리 노출 | 허용된 범용 혼합의 보조 장애물을 0~20m에 배치하고 필요하면 budget 조정 | GT 증가가 실제 지원량 증가인지, 다른 객체 노출 감소·가림·과밀이 생기는지 |
| 자산별 분산 | 기존 열 장애물과 새 자산의 거리별 floor·선택 가중치를 비교 | CargoBox만 늘어나지 않는지, 희귀 자산 coverage가 유지되는지 |
| 20~30m 표현 | 같은 자산·비슷한 거리에서 yaw·측면/정면·가림·배치 높이를 대조 | 개수가 아니라 지원량·크기/중심/yaw·복합 라벨 문제인지 |
| 원거리 지원 | 30~50/50~80/80~100m 및 ROI 모서리의 장애물 사례를 확인 | 실제 인지 가능한 정보가 있는지, 큰 장애물과 작은 장애물의 필요성이 다른지 |
| 회피와 음영 | 단일 회피·좌우 통과·음영 접근/내부에서 대표 자산을 배치 | 미션 현실성, BBox 경계 간격, 초기 접지·가시성·동작 정책이 유지되는지 |

0~20m의 1.5~2배는 첫 후보의 탐색 범위다. 20~30m는 단순히 현상 유지하는 구간이 아니라
별도 보강 대상이다. 30m 이상은 제외·무기한 보류하지 않으며 원거리 센서 지원을 조사한 뒤
개수·거리·자산 크기 배분을 판단한다. 중심 거리와 BBox 경계 clearance를 혼용하지 않는다.

## 매 반복의 보고와 종료 조건

보고서는 frame·parent·component GT를 구분하고 원래 semantic class와 학습 class를 함께 남긴다.
거리별 집계는 전후좌우 및 크기·asset·yaw·family로 다시 나눈다. 정적 배치 수만으로 성공을 판정하지 않는다.

- LiDAR 0/1~4/5~19/20점 이상과 카메라 유효 영역·crop·가시성 근거를 분리한다. 투영된 BBox만으로 실제 비가림을 확정하지 않는다.
- retained/제외 GT, parent/component 대응, 중심·크기·yaw, 같은 객체의 중복 출력과 누락을 확인한다.
- 고정 E9/E24 출력은 진단용으로 사용할 수 있으나 아직 학습하지 않은 후보 데이터의 출력을 보강 효과의 증명으로 삼지 않는다.
- GPU 추론 때문에 live 캡처 시간·안정성 비교가 오염되지 않도록 별도 분석 구간을 둔다. 재학습·임대 GPU 조작은 이번 문서화 요청에 포함하지 않는다.
- native 실패·충돌·심각한 부유·재시도·캡처 시간·저장량을 기록한다. 약 96장 pilot은 필요에 따라 확대하거나 줄이되 asset/조건별 표본 부족을 표시한다.

비교 pilot은 조건을 의도적으로 나눈 진단 표본이다. 전체 5K의 실제 quota로 가중한 추정과
별도 보고하며 이를 독립 실전 test나 향상된 AP로 부르지 않는다. 맑은 한 환경의 성공만으로
Foggy·역광 조건을 검증했다고 하지 않는다. 대표 sunny/foggy 조건은 각각 qualification을 수행한다.

후보의 유효 장애물 coverage·센서 지원·미션 현실성이 개선되고, 기존 차량/사람/희귀 자산의
중요 사례가 유지되며, 실패·시간 비용이 수용 가능한 수준이면 동결한다. 목표 증가율에 못 미쳐도
근거가 납득되면 더 낮은 보강량으로 결론낼 수 있다. 신뢰할 만한 개선이 없으면 기존 v142 정책을
유지한다. 배치 분석만으로 학습 성능의 인과 개선을 인증하지 않는다.

## 다음 수집으로 연결

[저장 bag과 tracker 정책 검토](../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-tracker-planning-policy-review-261013)에서
중복 birth guard를 꺼도 근거리 recall 이득은 없었고 미대응은 늘었다. 생성기 pilot은
가림 전후·회피 후 측방/측후방 부분 관측·가까운 distinct actor를 포함해 detector box
중복과 실제 여러 객체를 구분한다. 정지 Ego의 20~30m 반복 출력만으로 초근접 통과를
검증했다고 하지 않는다. Tracker 필터 제거로 데이터 보강을 대체하지 않으며 배치 비율은 여전히 실험으로 결정한다.

실험 후보·raw·분석·결정·source/code/manifest hash는 새 root에 보존한다. 기존
`fresh_suite_final_v142`와 `collector_release_v142`는 baseline으로 유지하고 덮어쓰지 않는다.
새 정책은 five-environment 전수 생성·CPU 감사, live QA, 실패 보충·완료 resume를 통과한 뒤
새 manifest·release와 실행 명령으로 전달한다. 과거 8~11시간/35GiB 예산은 보강 후 다시 산정한다.

최종 보고에는 채택/기각한 변화, asset별·거리별 예상 유효 GT, 남은 한계, 5K 시간·디스크 예상,
수집·resume·대시보드 명령을 포함한다. 완료 Sunny13는 새 pilot과 섞지 않으며 재취득을 전제로 하지 않는다.
