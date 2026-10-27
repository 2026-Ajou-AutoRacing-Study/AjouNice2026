# 2026-10-27 미션 신호 선택과 지도 허용 범위 비교

상태: 소스 기본을 **yaw 불확실성 4°·수평 검색 상한120px**로 완화했다. 이전 기본은3°·90px다. 세로200px, 지도상의 다른 등기구 경쟁 검사, 박스 크기/모양, 현재 CNN 판독과 신호 정책은 유지한다. 설치·주행 결과는 아래 검증 절을 따른다.

## 선택의 역할과 상단 ROI 비교

YOLO는 화면의 여러 신호등 박스를 찾는다. 그중 주행 경로의 A/B/C/D에 해당하는 박스를 골라야 한다. YOLO confidence는 등기구 검출 확신이지 미션 ID나 색 판독의 확신이 아니다.

팀 v88 bag 69.729초에서 B는 confidence0.868, 오른쪽 다른 교차로 신호는0.922였다. 둘 다 상단 영역에 있으므로 상단 ROI에서 최고 confidence를 고르면 다른 신호를 B로 읽게 된다. [원본과 두 박스 비교 도표](../../../artifacts/evaluation/traffic_light_robustness_261025/roi_confidence_comparison_261027_v1/B_mission_vs_highest_confidence.png)의 좌표와 원본 SHA는 같은 폴더의 `source_and_boxes.json`에 있다. 그림은 원본 픽셀과 박스 표시만 사용했다.

고정 pose의 자연 신호969장에서는 중앙·최대 confidence·최대 크기 선택 모두 검수된 named housing의 미션 그룹과 일치했다. 따라서 기존 ROI 방식이 항상 나쁜 것은 아니다. 다만 실제 v88 B 주행에서 지도와 동일 B를 선택한960 검출38개를 비교하면 최고 confidence는8개, 최대 크기는4개에서 다른 박스를 골랐다. 중앙 선택은 이38개에서 동일 B를 골랐다. 반대로 **지도 미연결인데 다른 YOLO 박스가 있는10프레임 중8개**는 다른 교차로 등기구·뒷면이었다. 이들은 원본 대표 프레임을 직접 확인하고 연속 장면과 geometry를 대조한 개발 분석이며 전량 독립 사람 GT는 아니다. 무조건 중앙/상단 박스를 미션 신호로 지정하는 fallback은 추가하지 않았다.

C의 두 동기 등기구에서는 큰 crop을 고르는 이점도 있다. 당시 v4 CNN 평가에서 중앙 선택은 작은 뒤쪽 head를242회 선택했고 원시 램프 일치891/939, 지도 선택은939/939였다. 이는 v4 당시 자료의 crop 차이이며 현재 v5의 정확도 수치가 아니다.

## 허용 범위 실험과 결정

현재 소스·고정 v5 모델·팀 v88 B와 v94 A/B/C/D의 같은 원본 JPEG/측위를 사용했다. yaw 상한만3→4/5/10°로 높이면 수평90px 제한에 다시 걸려 B primary 연결31개가 늘지 않았다. yaw4°와 수평120px를 함께 적용하면39개가 됐다. 더 넓은10°·240px 및 크기비 완화는 이 자료에서 추가 이득이 없었다.

최종 비교는 단순 박스 연결뿐 아니라 실제 inference callback의 근접 재검출·검출 누락/저확신 완충·visual continuation을 포함했다. 저장된 기존 출력의 tuple/head/box509개를 먼저509/509 재현한 뒤 후보를 적용했다.

| 실제 활성 미션 | 이전3°·90px | 후보4°·120px |
|---|---:|---:|
| 팀 v88 B | 38/91 | 38/91 |
| 팀 v94 A | 56/61 | 56/61 |
| 팀 v94 B | 75/90 | 77/90 |
| 팀 v94 C | 37/74 | 39/74 |
| 팀 v94 D | 31/81 | 33/81 |

분자는 최종 nonempty 램프 관측 수다. 분모에는 멀거나 이미 화각 밖인 프레임도 있어 정확도/가시 프레임 재현율로 읽지 않는다. 기존 정상 출력 악화0개, 변경6개는 모두 UNKNOWN에서 현재 crop의 램프 관측으로 회복됐다. B 적색·녹색 각1개, C 녹색+좌화살표2개, D 적색+좌화살표2개다. 원본 영상과 crop을 육안 확인했다. 독립 native GT가 없는 이6개를 전체 주행 정확도라고 부르지 않는다. 팀 v88 B는 기존 visual continuation이 이미 같은 관측을 복구해 최종 수가 같다.

전체 callback 보고서·재현 명령은 로컬 `planner_logs/analysis/261027_projection_tolerance_sweep_REPORT.md`, 원본 결과는 `261027_projection_tolerance_sweep_callback.json`, 회복6개 도표는 `261027_projection_tolerance_sweep_callback_review.jpg`다. 원본 입력·모델은 변경하지 않았다.

별도로 검수 housing과 저장 검출을 사용한 순수 선택 stress를 수행했다. 개발348장(과거640 검출)은 선택339개·named 미션 일치339개, 자연969장(최종960 검출)은 선택969개·named 미션 일치969개로 완화 전후 동일했다. 실제 다른 등기구 박스18개만 넣은 challenge의 미션 오연결은0개였다. yaw sigma3.5°와 평균 yaw±3°를 **인위적으로 주입**한 경우 새 범위는 해당 선택을 유지했다. 실제 공분산 분포·오연결률의 인증은 아니며 특히 다른 등기구 음성 표본18개는 작다. 재현 도구는 `scripts/evaluation/audit_traffic_light_selection_tolerance.py`, 결과는 `artifacts/evaluation/traffic_light_robustness_261025/selection_tolerance_stress_261027_v2.json`이다.

## 실행·검증 경계

기본 변경은 `traffic_light_map_selector.py`의 공유 설정과 `traffic_light_node.launch`에 같이 반영했다. v5 호환 launch도 두 인자를 전달하므로 과거 기준은 다음으로 선택할 수 있다.

```bash
roslaunch traffic_light_detection traffic_light_v5_candidate.launch \
  map_max_yaw_sigma_degrees:=3 map_max_padding_px:=90
```

과거 replay·ROI cap 분석의3°·90px 대조군은 명시적으로 고정해 새 기본값을 상속하지 않는다. 새 replay 추출은 `--yaw-cap 4 --horizontal-cap 120`을 명시한다. 기존 실행 snapshot과 원본 자료는 수정하지 않는다.

관련 selector/helper/실제 callback/visual continuation 시험91개와 route·replay·기존 ROI audit 시험46개가 통과했다. 넓어진 검색에서도 더 높은 confidence의 다른 지도 신호가 선택되지 않는 회귀와,3.5°에서 현재 목표 박스를 처음부터 읽고4.1°는 거부하는 경계를 포함했다.

로컬 설치기는 GPU 모델 추론·설치된 helper/launch/C++ 플러그인의 현 소스 일치를 확인했다. 새 기본값으로 자연 신호·v88·로컬 `od_gt`·실제 ROS1 planner/controller 풀경로를 실행해212.3초에 목적지 도착·충돌0·P 종료로 PASS했다. 실행 snapshot의 yaw4°·수평120px·세로200px를 확인했다. 근거는 `planner_logs/validation/traffic_light_compliance_full_regression_261027_141706/success/`다. 자연 현시가 이전 주행과 달라 소요시간이나 판독 수의 차이를 이 변경의 인과 효과로 해석하지 않는다. 팀 PC LAN·BEVFusion 동시 부하의 효과는 별도다.

새 bag의 선택 config1118개도 모두4°·120px였고 미션별 다른 head 연결은 없었다. A/B/C/D의 per-head padding 초과 거부는0개였으며 B의 첫 GREEN부터29프레임은 중간 UNKNOWN 없이 이어졌다. 신호등을 지나며 화각을 벗어나는 마지막10프레임 UNKNOWN은 남았다. B 정지점 재삽입은 관측되지 않았고16→8m/s 감속은 후기 UNKNOWN보다2.265초 먼저 시작해 이전과 유사한 위치·속도 프로파일이었다. B matched수 자체는33/72로 이전39/74보다 작아 이 서로 다른 자연 주행을 recall 상승 근거로 쓰지 않는다. visual continuation 성공0회여서 보통 지도 연결만으로 주행했다.

`no_lamps` 완충과 실제 화각 밖의 과거 색 유지 여부는 이 변경의 대상이 아니다. 이번 완화는 이미 화면에 있는 등기구를 공분산 제한만으로 놓치는 경우를 줄인다.
