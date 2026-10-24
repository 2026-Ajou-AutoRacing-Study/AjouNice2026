# 현재 Localization 근거와 제한

상태: CURRENT 근거 요약. 2026-10-24 문서 정리이며 새 runtime 검증은 없다.
아래는 현재 기본값과 연결되는 결과만 선별했다. 원본·수치·부정 결과·실행명은
[원본 bag catalog](legacy/evidence/260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md)와
[누적 실험 기록](legacy/evidence/261015_GPS_SHADOW_CLOSED_LOOP_IMPROVEMENT.md)에 보존한다.
archive의 이전 HOLD/기본값을 현재 기본값으로 읽지 않는다.

## 현재 채택 근거

| 계약 | 보존된 근거 | 해석 한계 |
|---|---|---|
| sigma `0.10m`, Qxy `0.0005` | 동일 raw bag의 횡/종 frame-jump p95 `5.432→1.444cm` / `7.234→2.416cm`; 경량 폐루프 약50Hz | 시간축·부하가 다른 full-stack 간 차이를 Q/R 단독 효과로 해석하지 않음 |
| GPS 음영 state 연속성 | invalid GPS에서도 마지막 z 유지, 5.64/11.01초 replay 약50Hz | 연속 발행은 위치 정확도 보장이 아님 |
| co-time TF | Global stamp까지 Local 외삽 후 closure p95 `17.40→0.071cm` | 짧은 1× replay 근거; 새 bag에서도 closure/rate 확인 필요 |
| Global soft vy prior | 최신 pair 동일 raw input OFF/ON replay의 음영 Global 횡p95 `56.44→6.59cm` | 같은 Ego trajectory 재사용이며 OFF 차량 재주행이 아님 |
| vehicle-pivot Smooth | 5개 같은 입력의 Smooth 비교 후 실제 MR 무객체 전체 코스2회·음영 pair2회 PASS, 충돌0·약50Hz | pair는 두 장애물을 같은 쪽으로 회피; 양방향 회피/LAN/slip 증명 아님. 일부 p95·motion residual 악화도 존재 |
| 긴 음영 정차 | 실제 음영50.25초 중 Park hold30.07초 뒤 같은 PP/PI로 재출발·목적지/P 확인 | 로컬 MR이며 LAN/충돌/slip 검증 아님; 추가 ZUPT 근거로 확대하지 않음 |
| 공식 START opt-in | Ego 없이 초기화, 보정 yaw와 공식 기준 차이 `0.017593deg` | MR 로컬 출발 정합만 통과; LAN/주행/checkpoint 복귀는 별도 |

직접 원본 근거:

- [Q/R 채택](legacy/evidence/260919_quantized_gnss_qr_tuning_evidence.md),
  [경량 폐루프](legacy/evidence/260920_lightweight_closed_loop_evidence.md)
- [음영 발행 계약](legacy/evidence/260924_GNSS_OUTAGE_CONTINUITY_AND_CONTRACT_AUDIT.md),
  [TF co-time](legacy/evidence/260925_TF_COTIME_CONTRACT_EVIDENCE.md)
- [최초 Smooth 실제 폐루프](legacy/evidence/260925_SMOOTH_CONTROL_STATE_CLOSED_LOOP_EVIDENCE.md):
  당시 tau0.5/0.5와 현재 vehicle-pivot0.5/0.1/0.5를 구분한다.
- soft/개선 Smooth·정차·checkpoint·최신 센서 부하 대조는 위 누적 기록,
  공식 START 상세는 [보존 CURRENT snapshot](legacy/snapshots/261015_COMPETITION_LOCALIZATION_CURRENT_STATE.md).

## 기본값으로 채택하지 않은 결과

- GNSS 종방향20ms 보상: 최신 입력 Smooth 종 mean/p95 `27.21/57.92→2.99/7.64cm`지만
  9/19 heldout은 `2.46/6.59→21.86/36.54cm`로 악화했다. 전체 코스 후보는 path 실패로
  중단돼 승격 gate가 미완료다. `gnss_motion_compensation_sec=0.0`을 유지한다.
- 실제 무카메라·무LiDAR 실행 `loc_fullcourse_true_lightweight_261015_112304`는 보상0초로
  PASS153.71초·충돌0·P, 음영7.276초·약50Hz였다. raw 상대lag는 검색격자상0ms,
  새 전체 이동 Smooth XY mean/p50/p95 `2.18/1.75/4.87cm`였다. 센서부하 가설을
  지지하는 별도 실행이지 동일조건 Sensor ON 인과 A/B나 LAN 완료가 아니다.
- 9/15 LAN 현장bag 역산은 수ms 수준의 header 등가lag와 추가 보상0초를 지지했다.
  raw GPS 부재로 물리 GPS 지연 직접 측정이나 현재 production LAN 재검증이 아니다.
- [quantization-aware state](legacy/evidence/260925_QUANTIZED_GNSS_CONTROL_STATE_EVIDENCE.md),
  [격자 input ablation](legacy/evidence/260925_GNSS_QUANTIZATION_LATTICE_AUDIT.md),
  [합성 outage/relocation](legacy/evidence/260925_CHECKPOINT_RELOCATION_GUARD_EVIDENCE.md)는
  연구·기계적 재현이다. quantized state/active guard는 기본 OFF다.
- [IMU 입력 ablation](legacy/evidence/260925_MOTION_INPUT_ABLATION_EVIDENCE.md)과
  [Competition yaw-rate 대체](legacy/evidence/260925_COMPETITION_YAW_RATE_AND_YAW_SMOOTHING_EVIDENCE.md)는
  현재 IMU yaw-rate/ax 유지 근거다. native Competition angular.z는 clockwise deg/s이며
  같은 simulator 유래 신호를 IMU와 독립 측정처럼 동시 fusion하지 않는다.
- [Ego oracle timing](legacy/evidence/260925_EGO_ORACLE_TIMING_AUDIT.md)은 종 frame-jump
  해석의 한계다. [입력 계약](legacy/evidence/260925_FUSION_INPUT_CONTRACT_AUDIT.md)의
  lever-arm 가속도는 robot_localization에서 보정하므로 adapter에서 중복 보정하지 않는다.

## Checkpoint·운영 대기

변경 전 제공 툴0–13은 유효 GNSS로 자연 복구했으나 Smooth 정합까지 약2.78–7.03초가
걸렸다. 툴14(현행CP14)의 raw NMEA zero는 로컬 변경 전 관측이며 운영측 음영 범위 조정
후에도 같다고 단정하지 않는다. 툴12(현행CP12) 재출발 PASS는 무객체·로컬 개발 경로다.
공식 remaining-checkpoint/배우 history 복구나 즉시 제어 재개의 보장은 아니다.

알려진 위치 prior를 준 개발 성공과 위치정보 없는 GPSzero 실패는 별도 조건이다.
운영측 변경 이후 GPS 수신·정합·fresh route·기존 제어 재개 확인은 여전히 남아 있다.
