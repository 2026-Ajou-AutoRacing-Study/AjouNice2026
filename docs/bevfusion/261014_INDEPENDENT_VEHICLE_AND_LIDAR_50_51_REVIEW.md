# 독립 차량 장면과 LiDAR 패킷 묶음 검증

문서 상태: EVIDENCE. 검증일: 2026-10-14.
운영 선택은 [CURRENT](CURRENT.md)와 [후단 CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를 따른다.
이 시험은 새 교통 장면의 방향 보호·재취득 복구 후보와 실제 50/51 패킷 입력을 구분하여 검증한다.
E24 가중치·threshold·추론 compile OFF·기존 5K는 유지한다.
같은 날 후속 사용자 승인으로 LiDAR 기본값을 50에 전환했다. 아래 P/51 종료 기록은 전환 전 실측이다.

## 독립 입력과 안전한 녹화

같은 대회 맵과 검증된 Ego 시작점·NPC 지면·MGeo 경로·차선 변경 비활성 설정을 유지한다.
차종·초기/목표 속도·생성 주기는 기존 실패 녹화와 다르게 지정했다.
지도와 경로까지 독립인 시험이나 실제 역주행 NPC 시험은 아니다.
정상 후진·정지 후 후진은 별도의 합성 안전 검사로 구분한다.

| 장면 | NPC | 초기→목표 속도 | 생성 주기 | 실제 주행 거리 |
|---|---|---|---|---:|
| 고주로 출발 | Ioniq / Niro HEV / 2020 K5 | 0→36 / 48 / 62 km/h | 13초 | 약 220m |
| 고주로 흐름 | G80 / 2016 Sonata / Solati H350 | 24→50 / 34→62 / 42→74 km/h | 15초 | 약 220m |
| 로터리 회전 | Ioniq | 7→24 km/h | 7.5초 | 약 100m |

각 장면을 실제 드라이버 51/50으로 각각 녹화했다. GT pose·GT detection을 판단/제어 보조로
사용했으며 여섯 번 모두 목적지 도착·접촉 0이다. 학습된 E24의 폐루프 완주 인증은 아니다.
원본·시나리오·설정·events·로그는
`rosbags/eval/perception/261014_independent_vehicle_lidar/traffic_variants_v2/`에 보존한다.
원본 SHA256은 `progress.json`의 각 완료 행에 있다.

첫 시험 `highspeed_grouping_v1`은 로드 후 제어 준비 중 Ego가 움직여 목적지를 지나쳤고
TIMEOUT으로 종료됐다. 시험용 wrapper가 준비 동안 유일한 P·제동 송신기를 유지하고,
실제 P·정지를 확인한 뒤 경로 준비 완료 시 D 제어기로 소유권을 넘기도록 보완했다.
validator 공유 소스·일반 제어 정책은 변경하지 않았다. 첫 실패를 정상 완주에 합산하지 않는다.

여섯 원본 마감 뒤 최초 종료 수신 확인도 실패했다. 부모의 ROS 초기화와 실제 P/51 복귀를
명시한 별도 종료 경로로 재검증했다. 녹화를 반복하거나 P 전환을 위해 장면을 재로드하지 않았다.
최종 gear=1, 속도 약 0.000001m/s 미만, 실제 scan 51개·cut_angle −0.01을 확인했다.
시험 소유 제어기·수신기는 모두 종료하고 임시 npackets override를 제거했다.

## 시나리오 경계와 평가 좌표

녹화는 ScenarioLoad 이전부터 시작한다. 이전 장면의 다른 차종과 Ego 순간 이동을 새 장면의
검출 오류로 세지 않는다. `SCENARIO_READY`를 새 장면 경계로 사용하며 원본 이전 구간도 보존한다.
E24 입력 3,341개 중 새 장면은 3,325개, 이전 장면은 16개다.
각 새 장면의 초기 Park 이력을 포함하여 fresh tracker로 시작한다. 중간 입력을 임의로 빼지 않는다.

| 장면·패킷 | 새 장면 입력 | 초기 Park | 실제 제어 | 종료 후 |
|---|---:|---:|---:|---:|
| 출발 51 | 536 | 282 | 233 | 21 |
| 출발 50 | 590 | 324 | 244 | 22 |
| 흐름 51 | 526 | 285 | 221 | 20 |
| 흐름 50 | 551 | 297 | 231 | 23 |
| 회전 51 | 554 | 310 | 220 | 24 |
| 회전 50 | 568 | 322 | 223 | 23 |

센서는 원본 ROS XYZI 0–255와 Q90 JPEG를 유지한다. 카메라는 해당 cloud 수신 전에 이미 받은
메시지 중 header 오차 0.10초 이내만 사용한다. 각 새 장면의 선택 구간은 중간 누락 없이 연결된다.
센서 원본의 모든 패킷/프레임 무누락 인증과는 구분한다.

GT는 [좌표 계약](GT_EVALUATION_COORDINATE_CONTRACT.md)에 따라 raw Ego heading −1.347°와
자산별 중심 보정을 사용하고, 실제 기록된 map→morai_map 고정 TF로 map 좌표에 정렬한다.
운영 TF와 물리 GT를 별도 보존한다. raw UID 재사용 generation과 reconstructed actor key를 구분한다.
ROI 밖 미래 GT도 보존하여 현재 ROI를 벗어났다는 이유로 3초 미래 평가를 잘라내지 않는다.
카탈로그 대응은 visibility-qualified annotation/AP가 아니다.

## 실제 50과 51 입력의 차이

| 장면 | 51 header→cloud 수신 평균 / p95 | 50 평균 / p95 |
|---|---:|---:|
| 고주로 출발 | 89.06 / 104.67ms | 42.52 / 52.77ms |
| 고주로 흐름 | 86.97 / 103.52ms | 42.06 / 51.71ms |
| 로터리 회전 | 78.75 / 93.45ms | 38.25 / 45.12ms |

고주로 ≥40km/h에서도 출발 88.90→42.60ms, 흐름 84.17→41.57ms다.
로터리 ≥5km/h·|yaw rate|≥3°/s 구간은 75.55→38.43ms다.
이는 첫 패킷 header→bag 수신 age이며 순수 GPU/드라이버 연산시간이나 정확한 ray 취득 지연이 아니다.
서로 다른 실제 주행이므로 시뮬레이터 부하·주행 차이도 포함한다.

51의 원본 1,629개는 모두 367.2°·두 source footer, 50의 1,722개는 모두 360°·단일 footer였다.
원본 cloud 3,351개 모두 raw in-range point count와 일치했다. 전체 XYZ decoder 정합은 별도다.
이번 실제 50 세 실행은 7.5° 시작 위상이 유지됐다.

기존 고속 raw51에서 같은 패킷을 중간 위상부터 고정 50개씩 다시 묶으면 두 footer가 계속 섞였다.
가상 시작 위상 50개 중 source 시작에 맞는 한 위상만 단일 footer였다.
이 위상 실험을 실제 실패 확률 98%로 해석하지 않는다. 각도 360°와 단일 source 취득 시점은 다른 조건이다.
source 경계 정렬 50과 cut0은 단일 footer였지만 일반 고정 50이 그 경계를 자동 탐색하는 것은 아니다.

기존 고속 ≥40km/h raw51의 두 footer 사이 Ego 이동은 평균 1.31m·p95 1.63m였다.
반복 방향 sector 약 2%와 서로 다른 시점의 반환점 혼합은 별개다.
50의 첫 패킷 host header도 source footer보다 평균 약 37~41ms 이르다.
실험 footer 참조를 진짜 LAN 센서 시계나 고정 운영 시간 보정값으로 사용하지 않는다.

## 새 E24 검출과 초기 방향

E24 nearest·AMP·카메라 병렬화 ON·compile OFF로 3,341개 원본을 전체 forward했다.
이후 새 장면 3,325개를 각 fresh native tracker에 넣었다. 가드 OFF와 독립 원본 코어는
정확히 같고, 가드 ON도 여섯 장면 모두 중심·ID·속도를 바꾸지 않았다.

| 장면·패킷 | 2m GT track 대응 | 평균 XY 오차 | 속도 벡터 오차 평균 / p95 | ID 교체 후보 |
|---|---:|---:|---:|---:|
| 출발 51 | 1,166 / 1,391 | 0.306m | 0.789 / 2.299m/s | 3 |
| 출발 50 | 1,280 / 1,471 | 0.230m | 0.676 / 1.684m/s | 1 |
| 흐름 51 | 620 / 819 | 0.416m | 0.925 / 2.544m/s | 0 |
| 흐름 50 | 687 / 878 | 0.341m | 0.788 / 2.295m/s | 0 |
| 회전 51 | 894 / 1,027 | 0.280m | 0.635 / 1.739m/s | 1 |
| 회전 50 | 912 / 1,056 | 0.266m | 0.576 / 1.553m/s | 1 |

분모는 각 header의 카탈로그·현재 ROI 대응이다. AP가 아니며 실제 두 주행의 객체 위상과
관측 수가 다르므로 이 표만으로 50이 모든 상황에서 우월하다고 결론내리지 않는다.
각 방식의 2m GT 근처 다중 ID는 모두 0이었다. 새 ID가 만들어질 때 주변 stale 차량이
있던 후보는 출발 51에서 1개·흐름 51에서 2개였으며 별도 재취득 원형에 사용했다.

GT 속도 ≥1m/s·예측 속도 ≥0.5m/s의 5,397 대응 중 반대 방향은 회전 50의 1개였다.
생성 후 2초 이내 대응은 1,076개다. 해당 track은 태어난 지 0.152초, 세 번째 관측에서
확정됐으며 0.075초 동안 raw 중심이 실제 이동과 반대로 약 0.41m 움직였다.
이는 약 40.26m의 새 Ioniq이며, 당시 박스 LiDAR support는 15점이었다.
50패킷 단일 source batch·반복 sector 0·Ego 정지 상태였으므로 source 혼합이나
Ego 회전을 이 사례의 직접 원인으로 확정하지 않는다.
world 속도는 직전 [−1.593, −2.160]에서 [+0.654, +0.097]m/s로 반전했고,
다음 0.079초 관측에서 [−1.177, −1.389]로 복구됐다. 중심 오차는 0.352m였다.

실제 association history는 3개·0.152초였고 trusted_stamp=0이었다.
최소 4개·0.30초를 요구하는 가드에는 신뢰할 과거 방향이 없었다.
3점 fit도 RMS 0.171m로 기준 0.10m보다 크고, fit 속도 0.732m/s로 기준 1m/s보다 작았다.
관측 수만 3개로 줄여도 신뢰할 방향을 얻는 사례가 아니었다.
yaw에 π를 더하거나 이전 yaw를 사용해도 잘못된 world 속도는 남았다.
XY만 native prior로 대체한 진단에서는 속도 반전이 사라져 중심 관측의 영향을 분리했다.
새로운 초기 사례를 억지로 수정하려고 이 한 장면에 맞춰 history 조건을 낮추지 않는다.
가드의 정상 사례 비개입과 극초기 오류 완전 해결은 다른 결론이다.

## 반복성과 시간 기준 민감도

움직이는 입력을 장면당 20개, 총 120개 선택하여 다시 두 번 추론했다.
같은 새 프로세스의 A/A raw·운영 출력은 모두 정확히 같다.
전체 추론과 다른 새 프로세스의 비교는 bitwise 동일하지 않았다.
같은 233개 운영 박스의 XY 차이는 평균 0.091mm·p95 0.203mm·최대 0.590mm,
score 차이 최대 0.00148이었다. 박스 추가·제거는 0개다.
yaw 축 차이는 p95 0.028°지만 최대 2.154°인 고주로 약 57m 차량 예외가 있다.
AMP·cuDNN benchmark ON 조건에서 관측한 차이이며 특정 커널이 원인이라고 확정하지 않는다.
가드·복구 A/B는 한 번 고정한 검출을 함께 사용하여 이 변동과 정책 효과를 섞지 않는다.
다른 프로세스까지 완전 결정적이라고 인증하지 않는다.

동일한 실제 50의 1,709프레임에서 센서·예측·모델 stamp는 유지하고 GT만 source footer의
실험 시각으로 바꾸었다. 같은 객체·같은 예측 2,679쌍의 현재 XY 평균은
0.24469→0.21140m, 중앙값 0.20855→0.18133m, p90 0.46082→0.39317m,
p95 0.56217→0.47306m였다. 개별 악화 p95는 +0.23772m, 최대 +0.75893m다.
2m 대응은 2,681/3,406→2,686/3,405이고 GT birth/ROI 경계의 8개 제외·7개 추가를 구분했다.
전체 오차가 시간축만으로 설명된다는 뜻이 아니다. 이 GT-only 자료를 tracker 속도/FDE의
시간축이나 runtime timestamp 수정에 사용하지 않는다.

## 재취득 복구 공분산 후보

[이전 차량 검증](261014_VEHICLE_DIRECTION_REACQUISITION_REVIEW.md)의 P+R 연결은 구 ID 중복을
줄였지만 누적 위치 lag가 위치–속도 교차 공분산을 통해 순간 속도 변화로 반영되어 과대 추정이 남았다.
단발 R 확대 대신, 기존 P-only에서 거부되고 차량 P+R에서만 새로 허용되는 관측에 한해
XY와 동적 상태의 교차 공분산을 분리하는 두 연구 원형을 비교했다. 5m 제한·GT 미사용은 유지한다.

| 원형 | 기존 고주로 동일 GT 실제 Map-CV 3초 평균 | 개별 최대 악화 | 결정 |
|---|---:|---:|---|
| 예측 위치 유지·교차 공분산 분리 | 3.71285→3.72235m, 975쌍 | 5.156m | 운영 미적용 |
| 관측 위치/R 재설정·동적 상태 유지 | 3.71006→3.71644m, 973쌍 | 2.267m | 운영 미적용 |

교정된 위치 reset은 기존 Sonata의 약 −14m/s 과대 속도를 약 −9.795m/s로 줄였다.
해당 GT는 약 −9.96m/s다. 그러나 높은 score의 단발 중심 outlier를 실제 재취득으로 오인하면
합성 3초 Cartesian-CV 오차 악화 최대가 분리 2.984m·reset 8.467m였다.
한 사건의 복구 속도 개선만으로 일반 정책을 채택하지 않는다.
5개 개발 bag·기존 22개 noise/합성·새 8개 반례를 사용한 35사례와 실제 Map-CV를 구분한다.

초기 `reset_v1`은 XY를 관측 R로 초기화한 뒤 같은 관측을 다시 업데이트하여 과신하는 결함이 있었다.
그 결과는 폐기했다. 최종 `reset_v2b`는 XY 관측을 한 번만 사용하고 yaw 업데이트는 유지한다.
두 원형 모두 설치된 tracker와 production launch에는 들어가지 않는다.

새 여섯 장면을 더한 41사례에서도 보류 결론은 유지한다. 출발 51의 실제 Map-CV 공통
1,066쌍에서 3초 평균은 기존 5.86122m·분리 5.85224m·reset 5.89026m다.
분리는 평균이 약 9mm 좋아졌지만 개별 악화 최대 3.820m, 1m 초과 개선 6·악화 3개였다.
reset은 개별 악화 최대 9.422m, 1m 초과 개선 0·악화 7개였다.
ID 교체 후보 3→2의 이득으로 이 악화를 상쇄했다고 보지 않는다.

| 같은 1,066표본의 3초 오차 | 평균 | 중앙값 | p90 | p95 |
|---|---:|---:|---:|---:|
| 기존 | 5.861m | 5.983m | 10.916m | 11.785m |
| 교차 공분산 분리 | 5.852m | 5.921m | 10.916m | 11.785m |
| 위치 reset | 5.890m | 6.062m | 10.935m | 11.859m |

후보에만 추가된 미래 표본 2개를 포함한 1,068개 전체 평균은 분리 5.880m·reset 5.914m다.
서로 다른 분모의 평균을 위 같은 표본 A/B와 섞지 않는다.
흐름 51은 현재 대응이 늘었지만 공통 미래 표본은 같아, 새로 추가된 복구 표본을 별도로 본다.
다른 네 장면의 코어는 원본과 정확히 같았다. 보류 원형을 production에 설치하지 않았다.

높은 score의 중심 outlier가 이미 속도를 오염시킨 경우, 다음 정상 중심을 복구하면서
그 오염 속도를 보존할 수 있다. 반대로 박스 크기가 크게 달라졌어도 실제로 같은 Solati인
새 사례가 있어 단순 크기 비율 제한으로 정상 복구를 막을 위험도 확인했다.
한 번의 위치 innovation이나 크기만으로 실제 재취득·중심 outlier·다른 차량을 확정하지 않는다.

## 현재 결정과 다음 순서

초기 방향 가드는 신뢰할 과거 이력이 있는 튐을 보호하는 원형으로 유지한다.
첫 확정 순간의 불확실한 속도를 억지로 전진 방향으로 만들지 않는다.
재취득 P+R·교차 공분산 분리·위치 reset은 독립 예외 악화 때문에 운영 미적용이다.
E24·bounded coasting·Map-CV 경로 순위·planner 제동은 유지한다.

50은 이번 로컬 실제 입력에서 source 혼합과 수신 age를 줄였고 E24 검출·연결도 확인했다.
초기 보고의 51 임시 유지는 에이전트의 보수적 판단이었으며 사용자의 요청이 아니었다.
후속 사용자 승인으로 운영 기본값을 50에 전환했다. LAN·실제 팀 PC의 timestamp·카메라 정합·연속
검출/추적·planner 영향 확인은 별도로 남는다.
새 5K 생성기 거리·가시성·자산별 분포는 [생성기 실험 계획](../../dataset/morai_od/docs/261013_base5_obstacle_generator_experiment_plan.md)으로 이어간다.
현재 센서가 희소한 원거리 사례와 가까운 큰/작은 박스의 중심 튐을 구분하여 보강하고,
원거리 라벨 삭제나 tracker 정책을 바꿔 학습 부족을 숨기지 않는다. 본수집은 시작하지 않았다.

## 근거와 재현 도구

### 같은 날 LiDAR 50 운영 전환

사용자의 명시적 전환 승인으로 MORAI 전용 두 launch의 기본 packet count를 50으로 변경한다.
일반 Velodyne 드라이버의 `ceil(754/15)` 계산은 수정하지 않는다. RPM 900·첫 패킷 stamp·
cut_angle −0.01·기존 calibration과 decoder는 유지한다. 51 비교 실험에는 명시적으로 51을 전달하고,
새 실험 종료 복귀는 P/50을 확인한다. 과거 scan·bag·GT·가중치는 수정하지 않는다.

원시 입력 녹화기의 기본 감사 계약도 50으로 전환한다. 과거 51 재현은 driver에 명시적 51을
전달하고 녹화기에 `--expected-packets 51`을 지정한다. 비교 도구가 다른 정책을 같은 이름으로
실행하거나 과거 51 bag을 자동으로 50 계약에 합격시키지 않도록 구분한다.

적용 범위는 `dataset/base5`와 이 PC 설치본이며 planning 브랜치 통합·팀 PC 설치는 별도다.
재시작·복귀 방법은 [production handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md#lidar-50-적용과-51-복귀)를 따른다.

두 launch의 기본값·51 복귀·잘못된 인자 거부, 합성 bag의 50/51 계약과 독립 캠페인 준비 등
회귀 검사 34개를 통과했다. `sensor_interface` 증분 빌드·설치도 완료했다.
설치된 게이트웨이를 packet count override 없이 실행하고, 제어 송신 없이 실제 입력을 확인했다.
30개 scan·30개 cloud 모두 50패킷·360°·단일 source footer이며 header는 첫 패킷 stamp와 같았다.
cloud callback 수신 age는 평균 38.29ms·p95 43.72ms였다. 별도 Python 구독기의 수신 시각이므로
과거 bag receipt나 E24 결과 지연과 직접 동일한 지표로 비교하지 않는다.
원본 확인 결과는 `planner_logs/analysis/261014_lidar50_default_promotion/live_smoke.json`에 보존한다.
시나리오 로드·운전·모델 추론은 하지 않았으며 시험 소유 수신기만 종료하고 사용자 master를 유지했다.

상세 산출물의 공통 루트는 `planner_logs/analysis/261014_independent_vehicle_lidar_followup/`다.
`inputs_v1`은 시나리오 경계가 없어 폐기했으며 `inputs_v2/inputs.json`·`gt_pack.json.gz`를 사용한다.
패킷 원본 비교는 `lidar/existing_source_final_v2`, 새 실제 비교는 `lidar/new_source_final`이다.
재취득 원형은 `recovery/decision.json`과 `decouple_v1`·`reset_v2b`에 보존한다.
새 입력의 최종 방향은 `direction_v2`, 실제 association history는 `new_direction_event_final`,
추론은 `inference_v1`·`repeat_inference_v1`, 반복 대조는 `repeat_report.json`이다.
시간 민감도는 `source_footer_gt.json`·`clock_analysis_v1`이며 원본 GT와 구분한다.
새 재취득 원형은 `recovery/decouple_independent_v4`·`reset_independent_v4`·`mapcv_independent_v4`다.

| 단계 | 도구 |
|---|---|
| 독립 교통 파라미터·원본 형상 보존 검사 | `scripts/rosbag/record/build_independent_vehicle_probe_scenes.py` |
| GT 제어·P 준비·실제 51/50 녹화·소유 프로세스 종료 | `scripts/rosbag/record/run_independent_vehicle_lidar_campaign.py` |
| source footer·위상·수신 age·고속/회전 입력 분석 | `scripts/evaluation/lidar_followup_packet_audit.py` |
| 움직이는 Ego·시나리오 경계·원본 입력·전체 GT 고정 | `scripts/evaluation/prepare_independent_vehicle_inputs.py` |
| 같은 새 검출의 방향 가드·독립 native 코어 대조 | `scripts/evaluation/audit_independent_vehicle_direction.py` |
| 별도 공분산 복구 원형·noise/후진 반례 | `scripts/evaluation/qualify_vehicle_covariance_recovery.py` |
| 실제 association history·중심/yaw 단일 변수 원인 분리 | `scripts/evaluation/diagnose_independent_direction_event.py` |
| ID 재사용을 구분한 고정 actor·현재 gate 밖 복구 상태 | `scripts/evaluation/audit_vehicle_recovery_actor.py` |
| 모델 입력 유지·source footer GT-only 현재 중심 민감도 | `scripts/evaluation/prepare_source_footer_gt_sensitivity.py`, `analyze_independent_clock_sensitivity.py` |

라이브는 사용자 ROS master와 MORAI를 사용하지만 다른 캠페인·UDP·제어기와 동시에 실행하지 않는다.
오프라인 학습을 시작하지 않으며 E24 full forward만 별도 Docker에서 실행한다.
기본값 승격·LAN 풀스택·새 5K 본수집은 이 입력 검증만으로 자동 승인하지 않는다.
