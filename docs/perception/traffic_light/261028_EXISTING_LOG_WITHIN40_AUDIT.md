# 기존 6환경 주행 로그의 40m 이내 재집계

상태: EVIDENCE / 2026-10-28. 새 주행·추론·ROS 변경 없이 기존 JSONL만 읽었다.
이는 독립 test set의 검출 AP나 전체 pipeline 정확도가 아니라 기록된 crop의 native 일치 감사다.

## 대상과 계산

- `artifacts/evaluation/traffic_yellow_repeated_261028/`의
  `run_01_sunny13_natural`, `run_03_foggy13_no_SET`, `run_04_sunny15_no_SET`,
  `run_05_sunny11_no_SET`, `run_06_foggy11_no_SET`, `run_07_foggy15_no_SET`.
- 각 `observer/observations.jsonl`의 debug_selection/classifier/runtime를
  **정확한 source_stamp_ns + regulatory group**으로 결합하고 중복 시각은 한 번만 센다.
- 깊이는 선택 로그의 `roi.depth_m`: 카메라 전방축→신호등 housing 깊이이며 정지선 거리가 아니다.
- 현재 matched 로그 중 깊이가 없는 짧은 구간은 동일 group의 양쪽 기록이 각각0.5초 이내일 때만
  깊이를 선형 보간했다. 아래0~40m 표본에 해당하는 보간은5개(그중1개는 selection missing)다.
  이 보간은 색 GT를 보간하는 것이 아니며 독립 기하/가시성 검증도 아니다.
- 원본 이미지 시각은 observer receipt monotonic에서 receipt ROS−source ROS age를 빼 연결했다.
  기존 `native_bracket` 함수로 exact physical head의 전후 request/response가0.25초 안이고
  동일 색이며 사이 실패가 없을 때만 native 상태를 검증했다. 최근 값 extrapolation은 하지 않았다.
- CNN 원시 R/Y/G/좌화살표4비트 전체가 native bitmask와 같은지 비교했다.
  no_lamps는 판독 성공으로 세지 않는다. 기존 hold 뒤 최종 색과 원시 CNN을 혼동하지 않는다.

## 결과

10~40m 범위에서 선택/현재 crop이 연결되고 native 검증까지 가능한 표본:

| 미션 | native 검증 표본 | 전체 비트 일치 | 판독 공백 | 다른 색 판독 |
|---|---:|---:|---:|---:|
| A | 1,303 | 1,303 | 0 | 0 |
| B | 103 | 103 | 0 | 0 |
| C | 651 | 651 | 0 | 0 |
| D | 860 | 858 | 2 | 0 |
| 합계 | 2,917 | 2,915 (99.93%) | 2 | 0 |

- Foggy13 D:31.56m、source1793118151340963527、native G(16)、CNN tuples[]。
- Foggy15 D:31.92m、source1793119952670245625、native G(16)、CNN tuples[]。
  두 경우 최종도 UNKNOWN이었다. D의 G 단독은 좌회전 허가가 아니므로 이 표본은 잘못된 GO가 아니다.
- 별도 0~10m의 matched/native 검증 표본51개는51개 일치(A20/B6/D25; C0).
  합치면 **0~40m 조건부2,966/2,968=99.93%**다.
- 10~40m 선택 로그2,973개 중 matched2,972개, 보간으로 범위를 배정한 D missing1개가 있었다.
  matched55개는 native 시각/전환 대조가 불충분해 정확도 분모에서 제외했다.
- 0~10m에는 matched53개, 그중2개는 native 검증 불충분이었다.

## 이 수치로 말할 수 없는 것

모든 카메라 프레임을 분모로 한 YOLO recall·오매칭률·가시 신호 전체 인식률은 아니다.
기존 로그의 missing 구간에는 roi.depth_m가 없고, 양쪽 근거리 기록으로 보간할 수 없는
선택 로그2,729개가 남았다(먼 거리·화각 이탈 등을 포함하며 모두 실패라는 뜻도 아니다).
따라서 그 구간을 성공으로 간주하거나 삭제해서 전체 pipeline100%라고 주장하면 안 된다.
특히 사용자가 제외하려는 화면 상단 이탈은 독립 이미지 가시성 판정 없이 깔끔하게 분리되지 않는다.
지도 identity 선택 자체가 틀리지 않았다는 독립 수작업 box GT도 이 계산에는 없다.

표본 대부분은 같은 접근/정차 중 연속 프레임이다. 이미 튜닝에 살펴본 개발 자료이며
새로운 독립 시험 이미지 수로 표현하지 않는다. Sunny13은 옛 planner plugin,
Foggy13은 D 이후 owner 종료가 있었으므로 전체 goal PASS를 주장하지 않는다.
응답지연0·40m gate를 적용했을 때 실제 행동이 어떻게 달라지는지의 폐루프 재현도 아니다.

결론: **기존 로그에서40m 이내로 자를 수 있고, 잘 연결된 crop의 신호 판독은 매우 안정적이었다.**
다만40m 기준이 전체 YOLO/지도/CNN pipeline100%를 보장한다는 결론은 아니다.

## 후속: 미매칭도 분모에 넣은 기하 기반 감사

사용자가 미매칭/보완 뒤의 실제 누락을 요청하여 위 matched 조건부 집계를 확장했다.
추가 주행 없이 기록된 raw Ego(이미지 stamp에서80ms 이내 최근접), −1.347°보정,
기존 카메라 extrinsic과 housing geometry로 깊이/화각을 별도 계산했다.
같은 미션 housing 중 하나 이상이 전방40m 이내·정면45° 이내·3px 이상이며
전체 투영 박스가 화면 안인 처리 callback을 분모에 넣었다. matched 여부로 분모를 정하지 않았다.
보간 분모와 달라 이전 수치에 단순 합산할 수 없다.

| 미션 | 대상 callback | 원시 판독 가능 | 인지 완충 뒤 판독 가능 | 최종 UNKNOWN |
|---|---:|---:|---:|---:|
| A | 1,350 | 1,350 | 1,350 | 0 |
| B | 111 | 110 | 110 | 1 |
| C | 660 | 660 | 660 | 0 |
| D | 906 | 900 | 900 | 6 |
| 합계 | 3,027 | 3,020 (99.77%) | 3,020 (99.77%) | 7 |

이는 색 판독 **가용률**이다. native 검증2,970개에서는 비트 일치2,963개(99.76%),
UNKNOWN7개, 다른 색 출력0개였다. 나머지57개는 native 전환 bracket38개,
시간 범위 초과15개, 사이 조회 실패4개이며, 이57개의 인지 출력은 모두 있었다.
즉 native 대조 불가57개를 인지 실패57개로 세면 안 된다.

### 남은 7개

1. D 첫 관측 진입 약31~32m:4개(2개 no_lamps,2개 detector0).
   모두 직전 관측도 UNKNOWN이고 약0.10~0.13초 뒤 G를 처음 읽었다.
   기억할 이전 색이 없으므로 hold가 작동하지 않은 것이 맞다. 당시 G단독은 좌회전 불허다.
2. Foggy11 D 약11.46m:1개. TF latest보다 이미지 시각이1.56ms 빨라 lookup 실패.
   R+좌화살표→UNKNOWN→R+좌화살표, 공백0.123초이며 인지 최종에도 남았다.
   영상162.2초 허가 후162.3~162.4초 공백,162.5초 회복.
   실제속도 약0→0.30→1.11km/h, brake0.2→0.100→0으로 재제동은 확인되지 않았다.
   planner 인접 실제 로그는 허가 require_stop0이므로, **planner가 공백을 안 소비했을 수 있고
   UNKNOWN 유예가 실제 발동했다고 단정하지 않는다.**
3. Sunny13 B 약8.30m 및 Foggy15 D 약8.31m:2개. detector0이며 runtime projection은
   no_visible_route_head였다. 같은 exact-source 디버그 영상111.0/155.5초를 확인했을 때
   실제 housing/램프는 아직 화면 상단에 보였다. 따라서 단순히 화면 밖이라 정상이라고 제외하지 않는다.
   이후 상단 화각 이탈로 UNKNOWN이 계속되므로 모든 후속 공백을1프레임 실패로 표현하지 않는다.
   이 표는 독립 기하상 전체 housing이 보이는 경계의 실패 callback만 센다.

7개 모두 인지 최종 출력에는 남았으므로 완충 뒤100%라는 결론은 아니다.
완충 유무의 같은 입력 반사실/재주행 비교나 actuator 인과 인증도 아니다.
현재 영상에서 신호등이 정상 가시인 모든 순간에 대한 독립 pixel identity GT는 여전히 없다.
raw Ego 근사 깊이와 실제 matched projection 깊이 차이는 run별중앙값0.025~0.077m,
최대0.694m였다. 특히 상단 경계는 실제 pitch/roll·측위 오차의 영향을 받을 수 있다.
또한 녹화되지 않은 incoming camera 프레임, group0 callback을 포함한 detector recall이 아니다.

재현 도구/전체 실패 stamp/분포:
`artifacts/evaluation/traffic_yellow_repeated_261028/audit_within40_pipeline.py`,
`within40_pipeline_geometry_audit_v1.json`(같은 디렉터리).

## 5분 내 A/D 재학습 필요성 검토

기존 exact-source 영상과 A 원본 crop sheet를 다시 읽고, D no_lamps2개의
원본 JPEG 및 다음 정상 프레임을 bag에서 새 파생 폴더로 추출했다(원본 bag 불변).

- **A:** Sunny15/Sunny11의 황색 오판 crop은 어둡고 작아 사람도 황색 점등을
  지지할 수 없는 영상이다. p(Y)는 약0.63~0.88로 실제 과신이 있었다.
  문제 깊이는 약65~76m이고, 이번 기존6환경의40m 가시성 proxy 감사에서는
  A1350개 모두 판독 출력, native 검증1322개 모두 일치했다.
  현재40m 판단 후보를 선택한다면 A 때문에 즉시 재학습할 근거는 부족하다.
  원거리 판단까지 재개하려면 판독불가/작은 어두운 crop을 포함한 별도 보강을 검토한다.
  native R이라는 이유만으로 사람이 읽지 못하는 crop에 R을 강제 라벨링하지 않는다.
- **D:** 처음 들어오는4개 영상 모두 우회전 중 패널이 화면 오른쪽 끝에 걸린 순간이었다.
  Foggy15 no_lamps 원본에서는 녹색 램프가 화면에 아직 들어오지 않았고,
  Foggy13에서는 오른쪽 끝에 극히 일부만 걸렸다. 다음0.101/0.125초 프레임에서
  램프가 안쪽으로 들어오며 실제 녹색으로 판독됐다.
  따라서 앞서 native G 대비 no_lamps2개를 **일반적인 가시 신호 CNN 실패나
  즉시 학습보강이 필요한2개로 해석한 것은 과했다.** 기하상 housing 가시성과
  실제 램프 가시성이 달랐던 경계 사례이며, 출력 가용률 표 자체는 그대로 유지한다.
- D의 TF1개·근접 상단1개는 재학습으로 해결한다고 볼 수 없다.
- 결론: **현재 시간 예산에서는 YOLO/CNN 재학습을 시작하지 않는다.**
  40m 후보와 별개인 TF/투영 경계 문제를 작은 범위로 다루는 편이 근거에 맞다.
  운영 기본값 승격·PR·새 주행·추가 학습은 이 감사에서 수행하지 않았다.

D 원본 쌍: 각 `run_03_foggy13_no_SET`, `run_07_foggy15_no_SET` 하위
`quick_D_edge_original_review/images/frame_000000.jpg`(공백),
`frame_000001.jpg`(다음 정상 판독). A는 각 run의
`agent_exact_raw_review_v1/derived_exact_crop_10x_NN.png`를 참조한다.
