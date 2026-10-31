# 팀원 수동주행 rosbag 원본 감사

- 상태: EVIDENCE — 모델 추론 전 raw-only 감사.
- 날짜: 2026-10-04.
- 작업 브랜치: `dataset/base5`.
- 원본: `rosbags/eval/perception/260923_teammate_manual/`.
- 재현 스크립트: `scripts/rosbag/inspect/audit_teammate_eval_bags.py`.
- 상세 산출물: `artifacts/evaluation/perception/teammate_manual_260923/raw_bag_audit.json`.

## 판정

16개 bag 약 9.03 GiB가 정상적으로 열리며 기존 audit의 파일 크기와 topic별 message
count가 일치한다. 고주로 7개, 로터리/보행자/장애물 각 3개다. 총 녹화 길이는
약 11.29분이다. 기존 metadata-only 사본은 `~/Music/rosbag0923`으로 되돌렸고
다운로드 ZIP은 `~/Music/new_rosbag`에 보존했다.

새 실주행 평가 자료로 사용 가능하다. 다만 아래 GT와 시간 구간 조건을 먼저 맞춰야 한다.
이번 감사는 영상/포인트의 품질, 차선 준수, 실제 추월 성공, detector 성능을 판정하지 않는다.

## 고주로 실제 분포

속도는 Ego/Object의 forward `velocity.x`를 km/h로 변환한 값이다. NPC 범위와 수는
Ego와 raw topic pivot의 2D 거리가 30m 이내였던 관측만 집계했다. 카탈로그 보정 BBox
중심 거리 또는 production의 ±30m 사각 범위와 동일하지 않다.

| 시작 시각 / run | 길이 s | 주행거리 m | Ego 최고 km/h | 근처 NPC 속도 km/h | 근처 NPC ID 수 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 16:13:15 H01 development | 64.8 | 858.8 | 88.7 | 43.8–75.0 | 5 |
| 16:16:16 H02 evaluation_locked | 63.3 | 859.6 | 73.6 | 33.2–65.0 | 4 |
| 16:18:31 H02 evaluation_locked | 47.2 | 623.6 | 76.9 | 32.9–50.0 | 5 |
| 16:21:08 H02 evaluation_locked | 68.9 | 852.2 | 76.3 | 34.1–50.1 | 3 |
| 16:23:01 H03 evaluation_locked | 55.1 | 853.9 | 101.7 | 38.9–80.0 | 5 |
| 16:26:16 H04 evaluation_locked | 60.4 | 851.9 | 92.5 | 51.7–85.0 | 4 |
| 16:28:34 H05 training_candidate | 54.0 | 868.1 | 95.8 | 43.7–74.9 | 4 |

모든 고주로 bag에 Ego 앞/뒤 NPC의 30m 이내 관측이 모두 있다. 고주로에는 충돌 message의
non-empty payload와 연속 Ego 관측 간 3m 초과 위치 step이 없다. 다만 이 검사는
차선 이탈/접촉 미신고/실제 추월 수행을 보장하지 않는다.

16:18:31 H02는 다른 run보다 짧은 경로를 주행하므로 전체 구간 완주로 집계하지 않는다.
100km/h 이상은 H03의 최고 속도만 확인됐으며 고속 구간의 충분한 분량은 별도 검증해야 한다.
고주로 이동거리 / receipt-time 속도적분 비율은 0.968–1.000이다. 이전 로컬 full-stack의
약 22% 속도 과적분과 같은 규모의 전역 불일치는 이 bag들에서 관측되지 않는다.

## 시간축

- LiDAR 전체 녹화 길이 기준 평균 약 11.2–12.5Hz, camera 약 27.7–29.9Hz.
- 읽은 Ego/Object/sensor topic의 header 반복 및 역행은 0개.
- LiDAR header는 record time보다 중앙값 약 74–94ms 앞선다. camera/Ego/Object는
  record time과 거의 일치한다. scan 조립/출판 지연일 수 있으므로 이 차이만으로
  센서 동기 버그나 잘못된 timestamp라고 판정하지 않는다.
- 양방향 nearest camera의 LiDAR-header 대비 최대 차이는 모든 camera/bag에서
  100ms 미만이다. 미래 camera도 허용하는 offline nearest 검사이며 causal live
  입력의 실제 시점 정합을 보장하지 않는다.
- H01 34.21–34.72s에 공통 토픽 공백 약 0.52s.
- 16:21:08 H02에는 Ego 300ms 초과 공백 4회; 최대 약 0.68s.
- H03 52.77–54.85s에 Ego 공백 2.08s; LiDAR/camera 최대 간격 약 2.15s.
  그 사이 Ego 위치 변화는 약 0.27m다. 시뮬레이터 pause/전반 지연 가능성은 있으나
  bag만으로 원인을 확정하지 않는다.

원본을 유지한다. Tracking/temporal/future-path 평가 시 긴 공백을 넘어 trajectory를
보간하지 않고 연속 구간을 나누며, 공백 포함/제외 계약을 모든 모델에 똑같이 적용한다.

## 다른 미션과 GT 주의점

- 장애물 3개에는 근처 obstacle ID가 각각 6개 있다.
  16:38:03 O01에는 collision payload가 16.49–16.95s와 18.94–26.72s에 기록됐다.
  마지막 구간의 반복 message를 수백 건의 독립 충돌로 세지 않는다.
  16:38:59 O01 및 16:40:12 O02에는 collision payload가 없다.
  첫 O01은 실패/충돌 사례로 보존하고 clean avoidance 기준선과 분리한다.
- 보행자 3개에서 같은 ID의 위치 이동은 약 13.83m이며 x/y/z velocity는 모두 0이다.
  즉 실제 정적 객체가 아니다. 보행자 AVE를 raw velocity=0으로 평가하지 않는다.
  연속 position의 적절한 시간 구간으로 운동 GT를 복원하고 별도 검증해야 한다.
  16:35:43 P02의 19.7초 녹화는 파일 정상이며 최소 20초 audit만 미달했다.
- 로터리 근처 차량은 bag별 3개 ID이며 관측 forward speed는 약 8.3–22.6km/h다.
  collision payload는 없다.
- Vehicle `name`은 asset 이름 대신 MGeo link ID이고 Pedestrian/Obstacle `name`은
  빈 문자열이다. Vehicle은 기존 size-based catalog resolver의 성공률을 확인한다.
  Pedestrian/Obstacle은 evaluator가 size만으로 추측하지 않으므로 scenario의
  UNIQUEID/DataID 매핑을 이용한 asset 식별이 필요하다. 이 연결 확인 전 raw
  position/size를 정확한 캡처 GT로 취급하지 않는다.
- 같은 NPC ID가 오랜 미관측 뒤 다시 나타나는 구간이 있다. 예를 들어 H01의 ID5는
  1.38초 미관측 후 이전 위치와 약 478m 차이로 등장한다. spawn/ID 재사용 등의
  가능성을 분리하기 위해 GT trajectory도 gap/discontinuity에서 분할해야 한다.
- UDP Ego heading은 MR 계약에 따라 -1.347°를 한 번 적용한다. 이미 보정한
  localization heading에 다시 적용하지 않는다. 감사의 앞/뒤 분류는 이 계약을 사용했다.

## 시나리오와 다음 평가 계약

모든 metadata의 scenario SHA256가
`origin/dataset/human-driven-eval-capture`의 `configs/scenario/human_driving_evaluation/`
아래 9개 JSON과 일치했다. Scenario 내용을 새로 추측할 필요 없이 해당 원본으로
asset/scale/배치/속도 profile을 대조할 수 있다. 현재 브랜치에는 이 handoff 디렉터리가
없으며 이 감사는 그 브랜치를 merge하거나 production 코드를 변경하지 않았다.

기존 `development`, `evaluation_locked`, `training_candidate` 역할을 유지한다.
H05를 학습에 사용하면 독립 test로 집계하지 않는다. 반복 H02도 서로 다른 실제
주행 녹화이지만 같은 시나리오라는 점을 명시한다.

모델 추론 전 GT resolver/보행자 velocity/시간 공백 처리를 확인한 뒤, E25와 Q1-P7의
detection-only 비교 및 동일 tracker/prediction 비교를 같은 bag·range·threshold로 한다.
이 문서의 수치는 raw 감사이며 모델 성능 수치가 아니다.

```bash
cd ~/AjouNice2026
/usr/bin/python3 scripts/rosbag/inspect/audit_teammate_eval_bags.py \
  rosbags/eval/perception/260923_teammate_manual \
  --output artifacts/evaluation/perception/teammate_manual_260923/raw_bag_audit.json
```
