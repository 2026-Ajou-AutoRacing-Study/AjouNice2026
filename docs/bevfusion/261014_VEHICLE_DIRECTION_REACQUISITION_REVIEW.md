# 차량 초기 운동 방향과 재취득 검증

문서 상태: EVIDENCE. 검증일: 2026-10-14.
현재 운영 선택은 [BEVFusion CURRENT](CURRENT.md)와
[후단 CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를 따른다.

새 차량의 반대 방향 속도는 높은 score의 검출 중심 이동으로 재현됐다.
짧은 방향 보호 원형은 일부 미래 위치 오차를 줄였지만 완전한 해결은 아니다.
차량 재취득의 P+R 연결 후보는 ID 유지에도 복구 속도와 미래 위치의 악화가 남아 보류한다.
별도로 발견한 입력 완료 경합과 TF 실패의 정상 대기 입력 손상은 정책을 바꾸지 않는 수정으로 분리했다.
가중치·학습·생성기·planner 정책·LiDAR 기본 51·추론 compile OFF는 유지한다.

## 입력과 비교 범위

최신 팀장님 원본은
`rosbags/eval/perception/261014_teammate_fullstack/full_baseline_00_sample_20261014_012302_01/`의
`planbag_full_baseline_00_sample_P_20261014_012302.bag`다. SHA256은
`15698fa7c6ec264775c3b0e004980ad46a70cdf057f15e344416a7fd06dfb4d8`이다.
기록 당시 commit은 `3d85a22`이며 현재 nearest hint와 ROS 원점 보완 전의 E24 입력이다.
이번 후보는 같은 저장 검출을 사용한다. 현재 GPU 전체 forward의 정확도를 다시 인증한 시험이 아니다.

| 비교 단계 | 입력과 시간 계약 | 검증하는 항목 |
|---|---|---|
| 기록 분석 | 원본 검출·tracking·prediction과 원래 수신 시각 | 실제 관측된 문제와 사건 구분 |
| 독립 실제 EKF 코어 | 최신 1,825개 유효 TF 입력과 개발 4 bag의 고정 E24 예측 | gate·연결·속도·중심·ID·후속 Map-CV |
| 격리 실제 ROS | 동일 1,825개 입력의 header·수신 availability, native 30Hz 출력 | message adapter·출력 시계·정책 효과 |
| 합성 반례 | 정상 후진·정지·출발·급회전·두 차량·noise·누락 | 실제 움직임이나 다른 차량을 잘못 수정하는지 |

최신 검출 캐시 1,859개 중 TF를 만들 수 없는 34개는 기존 제외를 유지했다.
개발 H03/R01/P01/O02는 각각 512/326/269/463개 고정 예측을 재사용한다.
개발 예측에 없는 앞부분이나 프레임을 임의로 복원하지 않는다.
실제 ROS 비교는 원본 수신 availability와 공통 clock·callback·처리 완료 barrier를 사용한다.
이 fixture는 논리 동등성 검사이며 비동기 실전 CPU 부하·TF·GPU·planner·폐루프 인증은 아니다.

GT는 평가에만 사용한다. 원본 Ego heading −1.347°와 자산별 BBox 중심 보정은
[GT 좌표 계약](GT_EVALUATION_COORDINATE_CONTRACT.md)을 따른다. 운영 TF의 보정을 중복 적용하지 않는다.
현재 GT 2m 대응은 가시성 검증된 AP가 아니며, 서로 다른 단계의 프레임 수를 합산하지 않는다.

## 초기 속도 반전의 원인

Kona tracking UUID 끝 `0030`은 약 11~14m 거리에서 12개 기록 출력 동안 반대 세계 속도를 냈다.
아래 시각은 bag 시작 대비 입력 header 시각이다. 출력 수신 시각과 같지 않다.

| 입력 시각 | 검출 중심 Y | score | 업데이트 후 EKF 세계 Y 속도 |
|---|---:|---:|---:|
| 124.070초 | 809.516m | 0.627 | −0.904m/s |
| 124.170초 | 810.363m | 0.961 | +0.647m/s |
| 124.269초 | 810.769m | 0.664 | +1.991m/s |

첫 이상 중심은 100ms 동안 약 85cm 반대쪽으로 이동했다.
그 관측의 위치 innovation은 0.886m, Mahalanobis 값은 2.15로 기존 gate 5를 통과했다.
score가 0.5 이상이라 기본 위치 측정 노이즈를 사용했다.
같은 ID의 XY 업데이트가 속도를 반전시켰으며 yaw는 −84~−86°로 연속이었다.
출력 yaw의 약 180° 변화는 world velocity를 보존하는 박스 표현의 canonicalization이었다.
`world velocity → canonical yaw → body twist` 변환 자체를 잘못된 운동 방향의 원인으로 보지 않는다.

검출 중심 오차도 동반한다. 방향 보호가 검출 중심·박스 크기·AP까지 해결한 것은 아니다.
높은 detection score를 정확한 위치나 속도의 보증으로 해석하지 않는다.

## 초기 방향 보호 원형

원형은 태어난 지 2초 이내의 차량에만 적용한다. 실제 association된 과거 검출 중심의
0.45초 창에서 최소 4점·0.30초 span·fit RMS 0.10m 이내의 이동 방향을 참조한다.
현재 창이 불안정하고 EKF 속도가 최근 신뢰 방향과 반대일 때만 속도 벡터 부호를 바꾼다.
유효한 정지나 반대 방향 이동이 관측되면 기존 방향을 고집하지 않는다.
차량 heading의 π 동치, 큰 축 회전, 오래된 track·입력, ID 슬롯 재사용을 별도로 검사한다.
지도 진행 방향으로 강제하거나 모든 body 속도를 양수로 만드는 정책이 아니다.

| 시간 정책 | 실제 ROS Kona 반대 방향 출력¹ | 중심·ID·출력 stamp 변경 |
|---|---:|---|
| 기존 | 12개 | 기준 |
| 출력 시각 기준 0.4초 보호 | 10개 | 없음 |
| 측정 시각 판정과 출력 freshness 분리 | 3개 | 없음 |

¹ 동일 고정 Kona 이력에서 world Y 속도 >0.5m/s인 출력이다.
GT와 world 속도의 내적을 쓰는 별도 반대 방향 정의의 수와 혼용하지 않는다.

측정 시간 정책의 0.4초는 마지막 신뢰 관측과 최신 관측 사이의 시간이다.
출력 copy는 최신 입력보다 최대 0.5초 이내라는 별도 freshness 조건을 따른다.
원본 입력 age 평균 236ms를 빼거나 stamp를 새로 찍지 않는다.
따라서 마지막 신뢰 관측부터 출력까지의 전체 age가 0.4초 이하라는 뜻은 아니다.
coasting·30Hz 재출력이 새 관측을 추가하거나 보호 기간을 갱신하지 않는다.

격리 ROS V3는 네 모드 각각 1,825개 subscriber 입력 ACK와 5,486개 출력을 확인했다.
guard-only 대응 5,439객체의 중심·ID 집합·stamp는 정확히 같고 9개 객체 출력의 속도만 바뀌었다.
정상 개발 4 bag과 정방향·태생부터 후진·정지 후 후진·작은 U-turn 합성에는 가드 출력 변화가 없었다.

| 동일 GT 비교 | 표본 | 3초 위치 오차 평균 | 중앙값 | p90 | p95 |
|---|---:|---:|---:|---:|---:|
| 실제 ROS 고주로 현재 2m 대응 | 2,842 | 3.927→3.923m | 2.635→2.635m | 9.607→9.607m | 11.593→11.593m |
| 보호된 Kona 고정 이력 | 9 | 22.445→13.166m | 23.914→12.947m | 25.258→14.523m | 25.387→14.636m |

9개 보호 표본은 모두 개선됐고 누락·악화는 0이다. 1초 오차 평균도 6.875→3.782m였다.
그중 6개는 현재 중심 오차가 2m보다 커 일반 조건부 FDE에서 빠진다.
따라서 별도로 미리 지정한 같은 Kona 이력을 사용하여 9개 전체를 확인했다.
일반 성능의 분모나 위치 gate를 느슨하게 만든 결과가 아니다.
보호 종료 후 반대 방향 출력 3개와 큰 미래 오차는 남는다.
NPC가 가속하므로 완벽한 현재 GT를 사용해도 3초 등속 미래 오차가 약 9m인 해당 표본이 있다.
단일 사건을 위해 보호 시간을 계속 늘리거나 전체 속도를 과거 중심 차분으로 덮지 않는다.
이 가드는 격리 실행기에만 있으며 설치된 production node에는 추가하지 않았다.

## 재취득과 구 ID 잔류

Sonata의 기존 UUID 끝 `002c`와 새 `0036`은 원본 기록에서 7개 출력·0.20초 동안 함께 남았다.
기존 박스가 실제 차량보다 뒤에 있어 두 중심은 약 5.8~7.0m 떨어졌다.
근접 IoU 중복이나 한 GT 2m 안의 다중 박스 검사만으로는 찾지 못하는 유형이다.

독립 코어에서 새 ID가 생긴 관측은 기존 예측과 2.537m 떨어져 있었다.
5m Cartesian 제한 안이지만 예측 공분산 P만 사용한 Mahalanobis 값 5.445가 gate 5를 넘었다.
기존 속도가 실제 접근 속도를 따라가지 못한 위치 lag와 연결 조건을 구분했다.
P+R 후보는 EKF 업데이트와 같은 측정 노이즈 R을 포함하되 5m 물리적 제한을 유지했다.
낮은 score의 R 10배 규칙도 같고, 다른 클래스의 연결은 변경하지 않는다.

| 후보 | 구·신 ID 동시 출력 | 후속 비용과 결정 |
|---|---|---|
| 기존 | 코어 2회, 격리 ROS 6회 | 기준. 원본 기록 7회와 시간 경계가 다름 |
| 차량 P+R 연결 | 코어·격리 ROS 0회 | 기존 ID 복구 속도가 약 14m/s로 과대 보정되어 보류 |
| 복구 관측 XY-R 2·4·10배 | 코어에서 연결 유지 | 다음 정상 업데이트에서 과대 속도가 다시 발생해 보류 |

현재 GT 대응은 1,436→1,437/1,788, 미대응 출력은 40→37로 개선됐다.
그러나 코어 고주로 동일 GT 975표본의 3초 평균 오차는 3.713→3.730m로 악화됐다.
해당 Sonata의 코어 3초 오차는 약 4.7→9.7~10.3m로 커졌다.
단발 XY-R 2배와 10배도 개별 악화 최대 5.84/6.66m가 남았고,
실제 ROS P+R의 개별 악화도 최대 6.26m였다.
전체 평균·중앙값·p95 변화가 작다는 이유로 이 복구 예외를 숨기지 않는다.

5개 bag과 22개 합성·noise 사례를 4개 후보로 비교했다.
noise 스트레스에서는 switch와 중복 감소가 있었지만, 다른 차량을 합치지 않는 제한과
속도·미래 위치의 잔여 악화를 함께 판단했다. ID 유지 자체를 예측 개선으로 보지 않는다.
5~7m 전역 병합·강제 old ID 삭제·긴 coasting·GT 기반 운영 연결은 적용하지 않았다.

## 별도 입력 보존 수정

기존 worker는 mutex 안에서 입력 A를 복사한 뒤, 처리 끝에 새 입력 플래그를 mutex 없이 지웠다.
그 사이 callback이 B를 넣으면 A의 완료가 B의 pending 표시까지 지울 수 있었다.
이를 입력 generation별 완료 확인으로 바꿨다. A 처리 중 B·C가 들어오면 최신 C는 남는다.
기존 latest-value 방식은 유지하며 과거 입력을 FIFO로 무한 적재하지 않는다.

Callback은 TF 변환을 포함한 완성 패킷을 지역 버퍼에서 만든 후 공유 상태에 넣는다.
새 패킷의 TF 변환 실패가 대기 중인 정상 패킷을 빈 객체로 덮지 않는다.
늦게 완료된 오래된 패킷·중복 stamp·비정상 stamp도 정상 최신 패킷을 덮지 않는다.
이 수정은 초기 Kona 속도 반전의 원인이나 기록 bag의 누락 원인으로 확정한 것이 아니다.
정책 후보와 분리하여 실제 입력 경합을 다룬다. 가중치·코어 필터·launch·gate·수명 값은 바꾸지 않는다.

실제 ROS에서 clock을 멈춘 채 정상 A를 대기시킨 뒤 TF 실패 B를 넣자, 구 노드는
새 header의 빈 객체를 처리했고 수정 노드는 A를 보존했다. 늦은 stamp·동일 stamp의 B도
구 노드는 A를 덮었지만 수정 노드는 거부했다. 각 시험의 실패 메시지와 처리 경계를 확인했다.
정상 입력 1,825개를 재생한 네 모드의 5,486개 출력은 수정 전 V3와 직렬화 바이트가
모두 같았다. 중심·속도·ID뿐 아니라 공분산·flags·score·Header.seq를 포함한다.
이 PC의 AW149 tracker 설치본을 갱신했다. 팀 planning 브랜치에 이번 후속을 자동 머지하지 않았다.
독립 검토에서 Ego mesh의 callback-written frame 문자열도 immutable world frame으로 정리했다.
전체 wire 대조는 그 직전 입력 복구 snapshot을 사용했다. 마지막 mesh 정리는 기본 mesh OFF의
tracking 연산을 바꾸지 않으며 최종 source의 Release 빌드·실제 TF pending 시험을 별도로 확인했다.
입력 복구 snapshot과 최종 frame 정리판의 정상 A 보존 출력도 실제 bag의 직렬화 바이트가 같았다.
시계가 뒤로 리셋되는 rosbag 반복 재생은 여전히 fresh node가 필요하다. clock reset 지원을 추가한 것은 아니다.

## 재현 도구와 보존

| 역할 | 실행기 |
|---|---|
| native prior·gate·birth 근거 | `scripts/evaluation/audit_vehicle_reacquisition_core.py` |
| P+R·복구 XY-R·noise 반례 | `scripts/evaluation/qualify_vehicle_innovation_gate.py` |
| 초기 방향 격리 코어 | `scripts/evaluation/build_vehicle_direction_probe.py`, `audit_vehicle_direction_probe.py` |
| 실제 Map-CV의 고정 코어 비교 | `scripts/evaluation/audit_vehicle_map_prediction.py` |
| private ROS 연구 wrapper·barrier 비교 | `scripts/evaluation/build_vehicle_ros_candidates.py`, `replay_vehicle_ros_candidates.py` |
| 골든 통과 후 ROS→Map-CV 평가 | `scripts/evaluation/audit_vehicle_ros_prediction.py` |
| 실제 TF 실패·늦은/중복 stamp의 pending 보존 | `scripts/evaluation/probe_vehicle_detection_receipt_ros.py` |

각 `--help`의 source·parameter·GT 경로와 새 output 경로를 명시한다.
재생은 사용자 ROS master 11311과 분리된 localhost 포트 20000~65535만 허용한다.
실행기가 만든 프로세스만 종료하며 MORAI·기존 master·controller를 조작하지 않는다.
가드의 stamp·ID·중심 동등성 또는 입력 ACK가 깨지면 후속 효과 평가를 거부한다.

산출물은 git 제외 경로
`planner_logs/analysis/261014_vehicle_motion_reacquisition_candidates_v1/`에 보존한다.
`baseline_v3`, `direction_v2`, `direction_measurement_clock_v1`, `reacquisition`,
`mapcv/ros_v3_highway_fixed_actor`, `mapcv/ros_v3_fullcourse`, `ros_actual_v3`를 구분한다.
입력 수정의 정상 출력 대조는 `ros_receipt_nominal_v1`, 실패 재현은
`receipt_tf_pending_v1`·`receipt_late_pending_v1`·`receipt_duplicate_pending_v1`에 보존한다.
최종 source의 짧은 실제 대조는 `receipt_tf_pending_final_recorded`와 `receipt_final_revision_wire_aa`다.
`ros_actual_v1`은 callback 경계가 다른 6개 ID/위치 출력이 있어 효과 근거에서 제외했다.
`ros_actual_v2b`부터 공통 input·처리 barrier와 정확한 clock tick을 검증했다.
원본이 같고 출력 stamp가 같아도 소비한 입력이 같은지는 별도로 확인해야 한다.

## 검사 결과

native tracker C++ 39개, 후보 평가 Python 37개, Map-CV core·node 22개,
기존 continuity 37개, GT 원점·TF 7개, context 8개 검사를 통과했다.
입력 receipt의 6개 C++ 검사는 worker 처리 중 새 callback·동일/늦은 stamp·최신값 방식·
실패 패킷과 첫 stamp 0의 경계를 포함한다. AW149 Release 빌드·설치와 문서 경로 검사도 통과했다.
별도 실제 ROS의 3개 실패 재현 및 정상 wire-exact 대조는 단위검사와 구분한다.

## 다음 판단 기준

방향 가드의 완전 해결 여부는 새 입력의 검출 중심 품질과 남은 보호 종료 구간까지 확인한다.
P+R 복구는 현재 위치·ID뿐 아니라 복구 속도·공분산·미래 오차를 함께 보완해야 한다.
현재 Map-CV의 경로 순위나 Top-K를 바꾸어 잘못된 world velocity를 숨기지 않는다.
차량 fallback은 signed 속도를 보존하는 CTRV이고 비차량 fallback은 Cartesian CV다.
완벽한 현재 속도도 미래 NPC 가속을 미리 아는 것은 아니다.

후속 새 완주 bag에서 detector 입력 처리 개수와 마지막 관측 age를 확인한다.
현재 완주 bag의 MPT 제약 충돌·안전 제동은 별도 planning 과제이며 이번 후보로 해결했다고 하지 않는다.
LiDAR 50/cut0 고속·LAN 검증과 장애물 생성기 분포 확정은 이 기록의 후보 승격과 별도로 진행한다.
